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

COLORS = {  # ponytail: flat-colour textures for the MVP; swap the PNGs in build/fbx/texture for real ones
    "road": (0.40, 0.41, 0.41), "paint_white": (0.92, 0.92, 0.90), "grass": (0.18, 0.40, 0.12), "concrete": (0.62, 0.61, 0.58), "ground": (0.22, 0.36, 0.15),
}
RED, YELLOW = (0.72, 0.07, 0.06), (0.93, 0.72, 0.08)
# Two-colour stripes, from onboard footage: kerbs painted red/yellow in 0.5 m stripes (UV v = distance / 4 m),
# tyre walls stacked in red and yellow blocks of ~2 m (UV u = length / 4 m). Texture tile = 4 m.
STRIPES = {"kerb": (8, 1), "wall": (2, 0)}  # stripes per tile, axis (0 = u / image x, 1 = v / image y)

bpy.ops.wm.read_factory_settings(use_empty=True)
scene = bpy.context.scene
texdir = os.path.join(os.path.dirname(os.path.abspath(out)), "texture")
os.makedirs(texdir, exist_ok=True)
for stale in os.listdir(texdir):  # only this build's textures should sit next to the FBX
    os.remove(os.path.join(texdir, stale))
materials = {}


def material(name):
    if name not in materials:
        if name == "ortho":  # GUGiK orthophoto, resampled to a power-of-two texture
            img = bpy.data.images.load(data["photo"])
            img.scale(4096, 4096)
        else:
            img = bpy.data.images.new(name, 16, 16)
        if name == "ortho":
            pass
        elif name == "kerb_apron":  # u across the kerb (0 = asphalt side): red band, then the white outer border
            img.pixels = [c for y in range(16) for x in range(16) for c in (*(RED if x < 10 else COLORS["paint_white"]), 1.0)]
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
        nodes = mat.node_tree.nodes
        bsdf = next(n for n in nodes if n.type == "BSDF_PRINCIPLED")
        tex = nodes.new("ShaderNodeTexImage")
        tex.image = img
        mat.node_tree.links.new(tex.outputs["Color"], bsdf.inputs["Base Color"])
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
