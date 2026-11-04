"""Runs inside Blender: build/mesh.json -> objects, materials, AC empties -> FBX for ksEditor (+ .blend to inspect).

blender -b --factory-startup -P blender_build.py -- build/mesh.json build/fbx/track.fbx [ui/preview.png]
(the optional third path renders the AC menu preview after the export)
"""
import json
import os
import sys

import bpy
from mathutils import Matrix, Vector

src, out = sys.argv[sys.argv.index("--") + 1:][:2]
with open(src) as f:
    data = json.load(f)

# Road, concrete, kerbs and lines get procedural textures + normal maps (textures.py). What is left here is flat:
COLORS = {"grass": (0.18, 0.40, 0.12), "ground": (0.22, 0.36, 0.15)}
INVISIBLE = {"collider"}  # fully transparent (alpha 0): collision-only objects, shader ksPerPixelAT in ksEditor
ALPHA_TEXTURES = {"tree": lambda: textures.tree_atlas(), "fence": lambda: textures.fence_mesh()}  # cut out (alpha test)
PHOTOS = ("ortho", "ortho_near")  # orthophotos: + mask and detail textures (ksMultilayer in ksEditor)
MATTE = set(PHOTOS) | {"grass", "ground"}
# asphalt on the sharp photo's UVs (trackgen drapes 1ROAD / 2ROAD when there is one): base = the photo's asphalt
# brightness map (road_macro), detail = the 8 m asphalt tile
ROAD_MACRO = bool(data.get("photo_near"))
MACRO_GREY = 0.5  # road_macro stores 0.5 x the brightness ratio
ROAD_GAIN = 0.75  # magicMult = ROAD_GAIN / MACRO_GREY: macro x road.png (0.41) x 1.5 = 0.31, as on the Kunos tracks
RED, YELLOW = (0.72, 0.07, 0.06), (0.93, 0.72, 0.08)
STRIPES = {}  # stripes per tile, axis (0 = u / image x, 1 = v / image y)

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import textures  # noqa: E402  (numpy, bundled with Blender)
from trackgen import ROAD_UV_M, SMOOTH_MATERIALS  # noqa: E402

bpy.ops.wm.read_factory_settings(use_empty=True)
scene = bpy.context.scene
texdir = os.path.join(os.path.dirname(os.path.abspath(out)), "texture")
os.makedirs(texdir, exist_ok=True)
for stale in os.listdir(texdir):  # only this build's textures should sit next to the FBX
    os.remove(os.path.join(texdir, stale))
materials = {}
road_tile = []  # (road.png, road_nm.png) when the asphalt uses road_macro


def save_image(name, rgb, colorspace="sRGB"):
    """(h, w, 3) float array, row 0 = bottom -> texture/<name>.png"""
    import numpy as np
    h, w, _ = rgb.shape
    img = bpy.data.images.new(name, w, h)
    img.colorspace_settings.name = colorspace
    img.pixels.foreach_set(np.dstack((rgb, np.ones((h, w)))).astype(np.float32).ravel())
    img.filepath_raw = os.path.join(texdir, name + ".png")
    img.file_format = "PNG"
    img.save()
    return img


_photos = {}


def load_photo(key):
    """The GUGiK photo data[key] ("photo" / "photo_near") as a 4096^2 image, graded (textures.grade_photo: the aerial
    haze washes the grass out); loaded once (the road map reuses it)."""
    if key not in _photos:
        import numpy as np
        img = bpy.data.images.load(data[key])
        img.scale(4096, 4096)
        w, h = img.size
        px = np.empty(w * h * 4, np.float32)
        img.pixels.foreach_get(px)
        px = px.reshape(h, w, 4)
        px[..., :3] = textures.grade_photo(px[..., :3])
        img.pixels.foreach_set(px.ravel())
        _photos[key] = img
    return _photos[key]


def photo_mask(name, img):
    """<name>_mask.png next to the photo (R = grass, G = bare soil; half the photo's resolution is plenty)."""
    import numpy as np
    w, h = img.size
    px = np.empty(w * h * 4, np.float32)
    img.pixels.foreach_get(px)
    rgb = px.reshape(h, w, 4)[..., :3]
    rgb = rgb.reshape(h // 2, 2, w // 2, 2, 3).mean(axis=(1, 3))
    textures.write_png(os.path.join(texdir, name + "_mask.png"), textures.ortho_mask(rgb))


def road_macro():
    """road_macro.png: the asphalt brightness map from the sharp photo (grey, 0.5 = the median asphalt), plus the
    asphalt tile road.png / road_nm.png as its detail and a solid-red mask (one detail layer only)."""
    import numpy as np
    img = load_photo("photo_near")
    w, h = img.size
    px = np.empty(w * h * 4, np.float32)
    img.pixels.foreach_get(px)
    uvs = [uv for o in data["objects"] if o["name"] in ("1ROAD", "2ROAD") for uv in o["uvs"]]
    ref = tuple(np.clip(np.array([[int(v * h) for _, v in uvs], [int(u * w) for u, _ in uvs]]), 0, h - 1))
    ratio = textures.asphalt_macro(px.reshape(h, w, 4)[..., :3], ref_px=ref)  # 1 = the track's median asphalt
    macro = save_image("road_macro", np.repeat((MACRO_GREY * ratio)[..., None], 3, axis=2))
    make, strength = textures.SURFACES["road"]
    albedo, height = make()
    tile = (save_image("road", albedo), save_image("road_nm", textures.normal_map(height, strength), "Non-Color"))
    textures.write_png(os.path.join(texdir, "mask_r.png"), np.tile([1.0, 0, 0, 0], (4, 4, 1)))
    return macro, tile


def detail_textures():
    """grass_detail / soil_detail (+ _nm) into texture/; returns {name: (image, normal image, tile_m)}."""
    out = {}
    for dname, (make, strength, tile) in textures.DETAILS.items():
        albedo, height = make()
        out[dname] = (save_image(dname, albedo), save_image(dname + "_nm", textures.normal_map(height, strength),
                                                             "Non-Color"), tile)
    return out


# specular (ksSpecular, ksSpecularEXP) per material for ksEditor; the rest stays matte (grass, soil, photo terrain)
SHINE = {"kerb": (0.25, 30), "kerb_apron": (0.15, 20), "kerb_concrete": (0.1, 15),
         "concrete": (0.05, 10), "paint_white": (0.25, 30), "paint_mint": (0.25, 30), "chequer": (0.25, 30),
         "tyre_red": (0.15, 15), "tyre_yellow": (0.15, 15), "tyre_black": (0.15, 15), "bench": (0.1, 15),
         "fence": (0.3, 20)}
FLAT_GROUND = ("ROAD", "GRASS", "LINES_", "KERB", "APRON")  # objects lying on the ground: no shadow casting


def ks_persistence(details):
    """build/fbx/<model>.fbx.ini: every material's ksEditor shader, values and textures (File -> Load Persistence).
    The photo materials and the asphalt use ksMultilayer_fresnel_nm (CSP's recreated shader, acc-shaders
    recreated/ksMultilayer_fresnel_nm_ps.fx): txDiffuse x (detail R..A weighted by txMask) x magicMult, the details
    sampled at world position x multR..multA (repeats per metre), the one detail normal map at mesh UV x
    detailNMMult; ksAlphaRef -193 turns on CSP's tile variation of the details. Lighting as on the Kunos tracks
    (magione, drift: asphalt and grass 0.4 / 0.4), and so is diffuse x detail x magicMult: grass ~0.3, asphalt ~0.31."""
    import datetime
    import persistence

    def uv_per_m(key):  # UV units per metre on the photo (the 4096^2 photo is ~10 % anisotropic)
        with open(os.path.splitext(data[key])[0] + ".json") as f:
            uv = json.load(f)["uv"]
        return (abs(uv[0][0]) + abs(uv[1][1])) / 2

    multilayer = {"ksAmbient": 0.4, "ksDiffuse": 0.4, "ksAlphaRef": -193}
    mats = {}
    for name in sorted(materials):
        spec, exp = SHINE.get(name, (0.0, 10))
        vals = {"ksSpecular": spec, "ksSpecularEXP": exp}
        if name in PHOTOS and details:
            per_m = uv_per_m("photo" if name == "ortho" else "photo_near")
            grass, soil = (round(1 / details[d][2], 3) for d in ("grass_detail", "soil_detail"))
            mats[name] = ("ksMultilayer_fresnel_nm",
                          {**vals, **multilayer, "multR": grass, "multG": soil, "multB": grass, "multA": grass,
                           "magicMult": round(1 / textures.DETAIL_MEAN, 2),
                           "detailNMMult": round(1 / (per_m * details["grass_detail"][2]), 1)},
                          {"txDiffuse": f"{name}.png", "txMask": f"{name}_mask.png", "txDetailR": "grass_detail.png",
                           "txDetailG": "soil_detail.png", "txDetailB": "grass_detail.png",
                           "txDetailA": "grass_detail.png", "txDetailNM": "grass_detail_nm.png"})
        elif name == "road" and ROAD_MACRO:
            tile = round(1 / ROAD_UV_M, 3)
            mats[name] = ("ksMultilayer_fresnel_nm",
                          {**vals, **multilayer, "ksSpecularEXP": 8,
                           "multR": tile, "multG": tile, "multB": tile, "multA": tile,
                           "magicMult": round(ROAD_GAIN / MACRO_GREY, 2),
                           "detailNMMult": round(1 / (uv_per_m("photo_near") * ROAD_UV_M), 1),
                           # sheen at grazing angles, much weaker than Kunos' 2 / 1.8: those have a reflection mask
                           # in the diffuse alpha, road_macro's alpha is 1 everywhere
                           "tarmacSpecularMultiplier": 1, "fresnelC": 0.0, "fresnelEXP": 4, "fresnelMaxLevel": 0.5},
                          {"txDiffuse": "road_macro.png", "txMask": "mask_r.png", "txDetailR": "road.png",
                           "txDetailG": "road.png", "txDetailB": "road.png", "txDetailA": "road.png",
                           "txDetailNM": "road_nm.png"})
        elif name in INVISIBLE or name in ALPHA_TEXTURES:  # alpha-tested: collision (alpha 0), trees, fence mesh
            mats[name] = ("ksPerPixelAT", {**vals, "ksAlphaRef": 0.5}, {"txDiffuse": f"{name}.png"})
        elif os.path.exists(os.path.join(texdir, name + "_nm.png")):
            mats[name] = ("ksPerPixelNM", vals, {"txDiffuse": f"{name}.png", "txNormal": f"{name}_nm.png"})
        else:
            mats[name] = ("ksPerPixel", vals, {"txDiffuse": f"{name}.png"})
    meshes = [o["name"] for o in data["objects"]]
    no_shadow = [m for m in meshes if any(k in m for k in FLAT_GROUND) or m == "1WALL"]
    text = persistence.fbx_ini(
        os.path.basename(out), mats, meshes, [e["name"] for e in data["empties"]],
        date=datetime.datetime.now().strftime("%m/%d/%Y %I:%M:%S %p"), no_shadow=no_shadow)
    with open(os.path.abspath(out) + ".ini", "w", newline="") as f:
        f.write(text)
    # the same settings for kn5.py, which writes the KN5 itself (no ksEditor needed)
    with open(os.path.splitext(os.path.abspath(out))[0] + ".materials.json", "w", encoding="utf-8") as f:
        json.dump({"materials": mats, "no_shadow": no_shadow}, f, indent=1)
    print(f"ksEditor persistence -> {out}.ini ({len(mats)} materials)")


def preview_details(details):
    """.blend / preview only (after the FBX export, which wants the photo wired straight into Base Color): photo x
    detail picked by the mask, detail normals - what ksMultilayer should show in AC."""
    import numpy as np
    for name in PHOTOS:
        mat = materials.get(name)
        if not mat:
            continue
        nodes, links = mat.node_tree.nodes, mat.node_tree.links
        bsdf = next(n for n in nodes if n.type == "BSDF_PRINCIPLED")
        photo = next(n for n in nodes if n.type == "TEX_IMAGE")
        mask = nodes.new("ShaderNodeTexImage")
        mask.image = bpy.data.images.load(os.path.join(texdir, name + "_mask.png"))
        mask.image.colorspace_settings.name = "Non-Color"
        sep = nodes.new("ShaderNodeSeparateColor")
        links.new(mask.outputs["Color"], sep.inputs["Color"])
        pos = nodes.new("ShaderNodeNewGeometry")
        picked = {}
        for dname, (img, nimg, tile) in details.items():
            mp = nodes.new("ShaderNodeMapping")
            mp.inputs["Scale"].default_value = (1 / tile, 1 / tile, 1 / tile)
            links.new(pos.outputs["Position"], mp.inputs["Vector"])
            t, nt = nodes.new("ShaderNodeTexImage"), nodes.new("ShaderNodeTexImage")
            t.image, nt.image = img, nimg
            links.new(mp.outputs["Vector"], t.inputs["Vector"])
            links.new(mp.outputs["Vector"], nt.inputs["Vector"])
            picked[dname] = (t, nt)
        col, nrm = nodes.new("ShaderNodeMix"), nodes.new("ShaderNodeMix")
        for m in (col, nrm):
            m.data_type = "RGBA"
            links.new(sep.outputs["Green"], m.inputs["Factor"])
        links.new(picked["grass_detail"][0].outputs["Color"], col.inputs[6])
        links.new(picked["soil_detail"][0].outputs["Color"], col.inputs[7])
        links.new(picked["grass_detail"][1].outputs["Color"], nrm.inputs[6])
        links.new(picked["soil_detail"][1].outputs["Color"], nrm.inputs[7])
        mul = nodes.new("ShaderNodeMix")
        mul.data_type, mul.blend_type = "RGBA", "MULTIPLY"
        mul.inputs["Factor"].default_value = 1.0
        links.new(photo.outputs["Color"], mul.inputs[6])
        links.new(col.outputs[2], mul.inputs[7])
        gain = nodes.new("ShaderNodeVectorMath")  # back to the photo's brightness (magicMult in AC)
        gain.operation = "SCALE"
        lin = ((textures.DETAIL_MEAN + 0.055) / 1.055) ** 2.4  # the detail's average, linear
        gain.inputs["Scale"].default_value = float(1 / lin)
        links.new(mul.outputs[2], gain.inputs[0])
        links.new(gain.outputs["Vector"], bsdf.inputs["Base Color"])
        nmap = nodes.new("ShaderNodeNormalMap")
        nmap.inputs["Strength"].default_value = 0.6
        links.new(nrm.outputs[2], nmap.inputs["Color"])
        links.new(nmap.outputs["Normal"], bsdf.inputs["Normal"])
        nodes.active = photo  # Workbench (menu preview) draws the active image: the photo


def preview_road():
    """.blend / preview only: road_macro (UV) x the asphalt tile (8 m, world position) + the tile's normals."""
    mat = materials.get("road")
    if not (ROAD_MACRO and mat and road_tile):
        return
    nodes, links = mat.node_tree.nodes, mat.node_tree.links
    bsdf = next(n for n in nodes if n.type == "BSDF_PRINCIPLED")
    macro = next(n for n in nodes if n.type == "TEX_IMAGE")
    pos, mp = nodes.new("ShaderNodeNewGeometry"), nodes.new("ShaderNodeMapping")
    mp.inputs["Scale"].default_value = (1 / ROAD_UV_M,) * 3
    links.new(pos.outputs["Position"], mp.inputs["Vector"])
    t, nt = nodes.new("ShaderNodeTexImage"), nodes.new("ShaderNodeTexImage")
    t.image, nt.image = road_tile
    for node in (t, nt):
        links.new(mp.outputs["Vector"], node.inputs["Vector"])
    mul = nodes.new("ShaderNodeMix")
    mul.data_type, mul.blend_type = "RGBA", "MULTIPLY"
    mul.inputs["Factor"].default_value = 1.0
    links.new(macro.outputs["Color"], mul.inputs[6])
    links.new(t.outputs["Color"], mul.inputs[7])
    gain = nodes.new("ShaderNodeVectorMath")
    gain.operation = "SCALE"
    gain.inputs["Scale"].default_value = float((ROAD_GAIN / MACRO_GREY) ** 2.2)  # magicMult, in linear
    links.new(mul.outputs[2], gain.inputs[0])
    links.new(gain.outputs["Vector"], bsdf.inputs["Base Color"])
    nmap = nodes.new("ShaderNodeNormalMap")
    links.new(nt.outputs["Color"], nmap.inputs["Color"])
    links.new(nmap.outputs["Normal"], bsdf.inputs["Normal"])
    nodes.active = t  # Workbench (menu preview): the asphalt tile reads better than the grey map


def material(name):
    if name not in materials:
        normal = None
        if name in ("ortho", "ortho_near"):  # GUGiK orthophotos, resampled to a power-of-two texture
            img = load_photo("photo" if name == "ortho" else "photo_near")
            img.filepath_raw = os.path.join(texdir, name + ".png")
            img.file_format = "PNG"
            img.save()
            photo_mask(name, img)
        elif name == "road" and ROAD_MACRO:  # FBX: the brightness map; the tile is its detail (ksMultilayer)
            img, road_tile[:] = road_macro()
        elif name in ALPHA_TEXTURES:  # RGBA (ksPerPixelAT): written as is, loaded back for the preview
            path = os.path.join(texdir, name + ".png")
            textures.write_png(path, ALPHA_TEXTURES[name]())
            img = bpy.data.images.load(path)
        elif name in textures.SURFACES:  # procedural texture + normal map (shader ksPerPixelNM in ksEditor)
            make, strength = textures.SURFACES[name]
            albedo, height = make()
            img = save_image(name, albedo)
            normal = save_image(name + "_nm", textures.normal_map(height, strength), "Non-Color")
        else:  # flat colours / stripes still waiting for a real texture
            img = bpy.data.images.new(name, 16, 16, alpha=name in INVISIBLE)
            if name in INVISIBLE:
                img.pixels = [0.0, 0.0, 0.0, 0.0] * (16 * 16)
            elif name in STRIPES:
                count, axis = STRIPES[name]
                img.pixels = [c for y in range(16) for x in range(16)
                              for c in (*(RED if ((x, y)[axis] * count // 16) % 2 == 0 else YELLOW), 1.0)]
            else:
                img.pixels = [*COLORS[name], 1.0] * (16 * 16)
            img.filepath_raw = os.path.join(texdir, name + ".png")
            img.file_format = "PNG"
            img.save()
        mat = bpy.data.materials.new(name)
        if not mat.node_tree:
            mat.use_nodes = True
        nodes, links = mat.node_tree.nodes, mat.node_tree.links
        bsdf = next(n for n in nodes if n.type == "BSDF_PRINCIPLED")
        if name in MATTE:  # grass and soil don't shine: no grey-white sheen at grazing angles
            bsdf.inputs["Roughness"].default_value = 1.0
            for spec in ("Specular IOR Level", "Specular"):  # Blender 4+ / 3.x
                if spec in bsdf.inputs:
                    bsdf.inputs[spec].default_value = 0.0
        tex = nodes.new("ShaderNodeTexImage")
        tex.image = img
        links.new(tex.outputs["Color"], bsdf.inputs["Base Color"])
        if name in INVISIBLE or name in ALPHA_TEXTURES:  # cut out in the .blend preview too
            links.new(tex.outputs["Alpha"], bsdf.inputs["Alpha"])
            if hasattr(mat, "surface_render_method"):
                mat.surface_render_method = "DITHERED"
            else:
                mat.blend_method = "CLIP"
        if normal:  # so the .blend preview shows the relief too
            ntex, nmap = nodes.new("ShaderNodeTexImage"), nodes.new("ShaderNodeNormalMap")
            ntex.image = normal
            links.new(ntex.outputs["Color"], nmap.inputs["Color"])
            links.new(nmap.outputs["Normal"], bsdf.inputs["Normal"])
        materials[name] = mat
    return materials[name]


for o in data["objects"]:
    me = bpy.data.meshes.new(o["name"])
    me.from_pydata([tuple(v) for v in o["verts"]], [], o["faces"])
    uv = me.uv_layers.new(name="UVMap")
    loop_verts = [0] * len(me.loops)
    me.loops.foreach_get("vertex_index", loop_verts)
    uv.data.foreach_set("uv", [c for vi in loop_verts for c in o["uvs"][vi]])
    if o["material"] in SMOOTH_MATERIALS:  # one normal per vertex: ksEditor doesn't split it (vertex limit)
        me.polygons.foreach_set("use_smooth", [True] * len(me.polygons))
    if me.validate():
        print(f"WARNING: Blender fixed invalid geometry in {o['name']}")
    me.update()
    me.materials.append(material(o["material"]))
    ob = bpy.data.objects.new(o["name"], me)
    scene.collection.objects.link(ob)

for e in data["empties"]:
    # AC dummies: local Z = driving direction, local Y = up (after the FBX Y-up conversion below)
    (x, y, z), (fx, fy) = e["pos"], e["fwd"]
    ob = bpy.data.objects.new(e["name"], None)
    ob.empty_display_type = "ARROWS"
    ob.matrix_world = Matrix(((-fy, 0, fx, x), (fx, 0, fy, y), (0, 1, 0, z), (0, 0, 0, 1)))
    scene.collection.objects.link(ob)

# open the .blend on the track: the default view looks at the origin from a few metres, but the track lies ~90 m up
road = [v for o in data["objects"] if o["name"] == "1ROAD" for v in o["verts"]]
if road:
    from mathutils import Euler
    centre = Vector([(min(v[k] for v in road) + max(v[k] for v in road)) / 2 for k in range(3)])
    span = max(max(v[k] for v in road) - min(v[k] for v in road) for k in (0, 1))
    for screen in bpy.data.screens:
        for area in screen.areas:
            if area.type == "VIEW_3D":
                space = area.spaces[0]
                space.clip_start, space.clip_end = 0.05, 5000.0
                space.shading.type = "MATERIAL"
                r3d = space.region_3d
                r3d.view_perspective = "PERSP"
                r3d.view_location, r3d.view_distance = centre, span
                r3d.view_rotation = Euler((0.75, 0.0, 0.35)).to_quaternion()
details = detail_textures() if any(m in materials for m in PHOTOS) else {}
ks_persistence(details)
bpy.ops.export_scene.fbx(
    filepath=os.path.abspath(out),
    object_types={"MESH", "EMPTY"},
    apply_scale_options="FBX_SCALE_ALL",  # default (All Local) shows up 100x too big in ksEditor
    axis_forward="-Z", axis_up="Y",
    use_mesh_modifiers=False,
    add_leaf_bones=False,
    path_mode="RELATIVE",
    embed_textures=False,
)
print(f"exported {len(data['objects'])} meshes, {len(data['empties'])} empties -> {out}")
if details:  # the .blend (and the menu preview) show the photo with its detail, as ksMultilayer should in AC
    preview_details(details)
preview_road()
bpy.ops.wm.save_as_mainfile(filepath=os.path.splitext(os.path.abspath(out))[0] + ".blend")

preview = sys.argv[sys.argv.index("--") + 1:][2:3]
if preview:  # ui/preview.png for the AC menu: textured aerial view, camera fitted to the asphalt
    import math
    from mathutils import Euler
    scene.render.engine = "BLENDER_WORKBENCH"
    scene.display.shading.light, scene.display.shading.color_type = "STUDIO", "TEXTURE"
    scene.render.resolution_x, scene.render.resolution_y, scene.render.resolution_percentage = 1024, 576, 100
    scene.render.image_settings.file_format = "PNG"
    scene.world = bpy.data.worlds.new("sky")
    scene.world.color = (0.55, 0.65, 0.8)
    cam = bpy.data.objects.new("preview_cam", bpy.data.cameras.new("preview_cam"))
    cam.data.lens, cam.data.clip_end = 35, 5000
    scene.collection.objects.link(cam)
    scene.camera = cam
    cam.rotation_euler = Euler((math.radians(40), 0, math.radians(20)))  # oblique, from the south-south-west
    road = [v for o in data["objects"] if o["name"] == "1ROAD" for v in o["verts"]]
    lo = [min(v[k] for v in road) for k in range(3)]
    hi = [max(v[k] for v in road) for k in range(3)]
    corners = [Vector((x, y, z)) for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])]
    loc, _ = cam.camera_fit_coords(bpy.context.evaluated_depsgraph_get(), [c for p in corners for c in p])
    cam.location = loc - (cam.matrix_world.to_3x3() @ Vector((0, 0, -1))) * 12.0  # a little margin around the track
    scene.render.filepath = os.path.abspath(preview[0])
    os.makedirs(os.path.dirname(scene.render.filepath), exist_ok=True)
    bpy.ops.render.render(write_still=True)
    print(f"preview -> {preview[0]}")
