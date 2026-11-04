"""KN5 writer: build/mesh.json + the ksEditor material settings + texture/ -> the track's .kn5, without ksEditor.

Layout as ksEditor 1.x writes it (checked byte for byte by parsing one of its files to EOF): "sc6969", version 6
and an extra 0; textures (active, name, size, raw PNG bytes); materials (name, shader, alpha blend, alpha tested,
depth mode, properties as name + 10 floats = value1, value2[2], value3[3], value4[4], then texture mappings as
name, slot, file); then the node tree. Node: class (1 = dummy with a 4 x 4 matrix, 2 = mesh), name, child count,
active. Mesh: cast shadows, visible, transparent, vertices (position, normal, uv, tangent: 11 floats), uint16
indices, material index, layer, LOD in / out, bounding sphere (centre, radius), renderable.

Conventions (also from that file): AC is y-up, position = (x, z, -y) of ours (the FBX export's -Z forward / Y up),
uv = (u, -v), triangle winding unchanged (the axis swap is a rotation), tangent = normalise(dP/dv x N) with that
flipped v, made orthogonal to N. Dummies (AC_START_0 ...): rows X, Y (up), Z (driving direction), translation.
Flat-shaded objects get a vertex per (vertex, face normal), so coplanar neighbours share their corners; smooth
ones (trackgen.SMOOTH_MATERIALS) one per vertex with an area-weighted normal.

Pure stdlib (struct, math): runs in plain Python, no Blender needed.
"""
import json
import math
import os
import struct

import persistence
from trackgen import SMOOTH_MATERIALS

MAX_VERTS = 65535  # uint16 indices


def _ac(p):
    return (p[0], p[2], -p[1])


def _sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def _dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _norm(a, fallback=(0.0, 1.0, 0.0)):
    n = math.sqrt(_dot(a, a))
    return (a[0] / n, a[1] / n, a[2] / n) if n > 1e-12 else fallback


def _add_into(acc, k, v):
    a = acc.get(k)
    acc[k] = v if a is None else (a[0] + v[0], a[1] + v[1], a[2] + v[2])


def mesh_buffers(o):
    """Our object -> (vertices as 11-float tuples, uint16 triangle indices), in AC's axes."""
    P = [_ac(v) for v in o["verts"]]
    UV = [(u, -v) for u, v in o["uvs"]]
    smooth = o["material"] in SMOOTH_MATERIALS
    tris = [(f[0], f[i], f[i + 1]) for f in o["faces"] for i in range(1, len(f) - 1)]
    # per triangle: area-weighted normal (cross product) and dP/dv
    keyed, normals, bitans = [], {}, {}
    for a, b, c in tris:
        e1, e2 = _sub(P[b], P[a]), _sub(P[c], P[a])
        n = _cross(e1, e2)
        du1, dv1 = UV[b][0] - UV[a][0], UV[b][1] - UV[a][1]
        du2, dv2 = UV[c][0] - UV[a][0], UV[c][1] - UV[a][1]
        det = du1 * dv2 - du2 * dv1
        dpdv = ((e2[0] * du1 - e1[0] * du2) / det, (e2[1] * du1 - e1[1] * du2) / det,
                (e2[2] * du1 - e1[2] * du2) / det) if abs(det) > 1e-12 else (0.0, 0.0, 0.0)
        if smooth:
            keys = (a, b, c)
        else:  # one vertex per (vertex, face normal): a flat face's corners, shared by coplanar neighbours
            fn = _norm(n)
            nk = (round(fn[0], 4), round(fn[1], 4), round(fn[2], 4))
            keys = tuple((v, nk) for v in (a, b, c))
        for k in keys:
            _add_into(normals, k, n)
            _add_into(bitans, k, _norm(dpdv, (0.0, 0.0, 0.0)))
        keyed.append(keys)
    index, verts = {}, []
    for k in normals:
        vi = k if smooth else k[0]
        n = _norm(normals[k])
        t = _cross(bitans[k], n)
        t = _norm(_sub(t, tuple(n[i] * _dot(n, t) for i in range(3))),
                  _norm(_cross(n, (0.0, 0.0, 1.0)), (1.0, 0.0, 0.0)))
        index[k] = len(verts)
        verts.append((*P[vi], *n, *UV[vi], *t))
    if len(verts) > MAX_VERTS:
        raise ValueError(f"{o['name']}: {len(verts)} vertices, KN5 limit is {MAX_VERTS} - split it")
    idx = [index[k] for keys in keyed for k in keys]
    return verts, idx


def _str(s):
    b = s.encode("utf-8")
    return struct.pack("<i", len(b)) + b


def _dummy(name, rows, children):
    return (struct.pack("<i", 1) + _str(name) + struct.pack("<iB", len(children), 1)
            + struct.pack("<16f", *[x for r in rows for x in r]) + b"".join(children))


def _mesh(name, verts, idx, material, cast_shadows):
    xs, ys, zs = ([v[k] for v in verts] for k in range(3))
    centre = ((min(xs) + max(xs)) / 2, (min(ys) + max(ys)) / 2, (min(zs) + max(zs)) / 2)
    radius = max(math.dist(centre, v[:3]) for v in verts)
    return b"".join([
        struct.pack("<i", 2), _str(name), struct.pack("<iB", 0, 1),
        struct.pack("<BBB", int(cast_shadows), 1, 0),
        struct.pack("<i", len(verts)), b"".join(struct.pack("<11f", *v) for v in verts),
        struct.pack("<i", len(idx)), struct.pack(f"<{len(idx)}H", *idx),
        struct.pack("<iiff", material, 0, 0.0, 0.0), struct.pack("<4f", *centre, radius), struct.pack("<B", 1)])


def empty_rows(e):
    """AC dummy matrix rows from our empty (pos, fwd = driving direction in plan)."""
    (x, y, z), (fx, fy) = e["pos"], e["fwd"]
    return ((-fy, 0.0, -fx, 0.0), (0.0, 1.0, 0.0, 0.0), (fx, 0.0, -fy, 0.0), (x, z, -y, 1.0))


def write(path, mesh, materials, texdir, root="track", no_shadow=()):
    """mesh: build/mesh.json data; materials: {name: (shader, values, textures)} as for persistence.fbx_ini."""
    names = list(materials)
    used = [o for o in mesh["objects"] if o["faces"]]
    missing = {o["material"] for o in used} - set(names)
    assert not missing, f"no material settings for {sorted(missing)}"
    textures = []
    for _, _, tex in materials.values():
        textures += [t for t in tex.values() if t not in textures]
    out = [b"sc6969", struct.pack("<ii", 6, 0), struct.pack("<i", len(textures))]
    for t in textures:
        with open(os.path.join(texdir, t), "rb") as f:
            data = f.read()
        out += [struct.pack("<i", 1), _str(t), struct.pack("<i", len(data)), data]
    out.append(struct.pack("<i", len(names)))
    for name in names:
        shader, values, tex = materials[name]
        varnames, slots, alpha_test = persistence.SHADERS[shader]
        values = {**persistence.LIGHT, **values}
        out += [_str(name), _str(shader), struct.pack("<BBi", 0, int(alpha_test), 0), struct.pack("<i", len(varnames))]
        for v in varnames:
            out += [_str(v), struct.pack("<10f", float(values.get(v, 0.0)), *[0.0] * 9)]
        maps = [(t, slot) for t, slot in slots if t in tex]
        out.append(struct.pack("<i", len(maps)))
        for t, slot in maps:
            out += [_str(t), struct.pack("<i", slot), _str(tex[t])]
    children = []
    for o in used:
        verts, idx = mesh_buffers(o)
        children.append(_mesh(o["name"], verts, idx, names.index(o["material"]), o["name"] not in no_shadow))
    children += [_dummy(e["name"], empty_rows(e), []) for e in mesh["empties"]]
    out.append(_dummy(root, ((1.0, 0, 0, 0), (0, 1.0, 0, 0), (0, 0, 1.0, 0), (0, 0, 0, 1.0)), children))
    with open(path, "wb") as f:
        f.write(b"".join(out))
    return len(children)


def write_from_build(build_dir, folder, out_path):
    """build/mesh.json + build/fbx/<folder>.materials.json (written by blender_build) -> out_path."""
    with open(os.path.join(build_dir, "mesh.json")) as f:
        mesh = json.load(f)
    with open(os.path.join(build_dir, "fbx", folder + ".materials.json"), encoding="utf-8") as f:
        mat = json.load(f)
    materials = {k: tuple(v) for k, v in mat["materials"].items()}
    return write(out_path, mesh, materials, os.path.join(build_dir, "fbx", "texture"), folder, set(mat["no_shadow"]))
