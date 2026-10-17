"""Runs inside Blender: build/mesh.json -> objects, materials, AC empties -> FBX for ksEditor (+ .blend to inspect).

blender -b --factory-startup -P blender_build.py -- build/mesh.json build/fbx/track.fbx
"""
import json
import os
import sys

import bpy
from mathutils import Matrix

src, out = sys.argv[sys.argv.index("--") + 1:][:2]
with open(src) as f:
    data = json.load(f)

COLORS = {  # ponytail: flat-colour textures for the MVP; swap the PNGs in build/fbx/texture for real ones
    "road": (0.10, 0.10, 0.11), "kerb": (0.75, 0.08, 0.08), "grass": (0.18, 0.40, 0.12),
    "wall": (0.85, 0.85, 0.85), "ground": (0.22, 0.36, 0.15),
}

bpy.ops.wm.read_factory_settings(use_empty=True)
scene = bpy.context.scene
texdir = os.path.join(os.path.dirname(os.path.abspath(out)), "texture")
os.makedirs(texdir, exist_ok=True)
materials = {}


def material(name):
    if name not in materials:
        img = bpy.data.images.new(name, 16, 16)
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
