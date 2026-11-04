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
RED, YELLOW = (0.72, 0.07, 0.06), (0.93, 0.72, 0.08)
STRIPES = {}  # stripes per tile, axis (0 = u / image x, 1 = v / image y)

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import textures  # noqa: E402  (numpy, bundled with Blender)

bpy.ops.wm.read_factory_settings(use_empty=True)
scene = bpy.context.scene
texdir = os.path.join(os.path.dirname(os.path.abspath(out)), "texture")
os.makedirs(texdir, exist_ok=True)
for stale in os.listdir(texdir):  # only this build's textures should sit next to the FBX
    os.remove(os.path.join(texdir, stale))
materials = {}


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


def material(name):
    if name not in materials:
        normal = None
        if name in ("ortho", "ortho_near"):  # GUGiK orthophotos, resampled to a power-of-two texture
            img = bpy.data.images.load(data["photo" if name == "ortho" else "photo_near"])
            img.scale(4096, 4096)
            img.filepath_raw = os.path.join(texdir, name + ".png")
            img.file_format = "PNG"
            img.save()
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
        tex = nodes.new("ShaderNodeTexImage")
        tex.image = img
        links.new(tex.outputs["Color"], bsdf.inputs["Base Color"])
        if name in INVISIBLE:  # invisible in the .blend preview too
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
    if o["material"] == "ground":
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
bpy.ops.wm.save_as_mainfile(filepath=os.path.splitext(os.path.abspath(out))[0] + ".blend")
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
