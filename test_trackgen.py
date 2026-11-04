"""Self-check for trackgen: python3 test_trackgen.py"""
import json
import math
import os

import fit
import trackgen

R = 50.0
cfg = json.load(open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "data", "track.json"), encoding="utf-8"))
cfg.update(length_m=2 * math.pi * R, runoff_m=80.0, placeholder=[], reverse=False, start_xy=[R, 10.0])  # runoff > R: inside must be capped
pts = [(R * math.cos(a), R * math.sin(a), 0.0, 8.0) for a in (i * math.tau / 16 for i in range(16))]

geo = trackgen.build(cfg, pts)
errors, warnings = trackgen.validate(cfg, pts, geo)
assert not errors, errors
assert abs(geo["length"] - 2 * math.pi * R) / (2 * math.pi * R) < 0.005, geo["length"]
assert all(s[3] == 8.0 for s in geo["samples"])

# CCW circle: left = inside. Without traced barriers, the inner wall stays between the road and the centre
# (run-off capped by the free space), the outer wall beyond the road.
assert all(4.0 < R - math.hypot(*p[:2]) < R for p in geo["barriers"]["auto_l"])
assert all(math.hypot(*p[:2]) > R + 4.0 for p in geo["barriers"]["auto_r"])

# traced features: kerb only where a kerb strip is drawn, timing gates on the painted line, barrier on the road = error
feats = {"start_finish": [[R - 5, 0.0], [R + 5, 0.0]],
         "kerbs": [{"name": "k", "points": [[0.0, R - 4.5], [-10.0, R - 5.5]]}],
         "barriers": [{"name": "b", "points": [[-R - 10, 0.0], [-R - 15, 0.0]]}],
         "extra_asphalt": [{"name": "a", "points": [[R + 6, -2.0], [R + 9, -2.0], [R + 9, 2.0], [R + 6, 2.0]]}]}
g2 = trackgen.build(cfg, pts, None, feats)
assert not trackgen.validate(cfg, pts, g2)[0]
kerb = next(o for o in g2["objects"] if o["name"] == "1KERB")
strips = len(trackgen.KERB_PROFILE["stripes"]) - 1  # the humped cross-section
assert 0 < len(kerb["faces"]) < 20 * strips and all(v[1] > R - 10 for v in kerb["verts"]), len(kerb["faces"])
lift = trackgen.KERB_STYLES["stripes"][2]
top = max(v[2] for v in kerb["verts"]) - min(v[2] for v in kerb["verts"])
assert lift * 0.9 < top < lift + 0.05, ("striped kerb peaks ~lift above the asphalt", top)
gates = {e["name"]: e["pos"] for e in g2["empties"] if e["name"].startswith("AC_TIME")}
assert abs(gates["AC_TIME_0_L"][0] - (R - 5.5)) < 0.01 and abs(gates["AC_TIME_0_R"][0] - (R + 5.5)) < 0.01, gates
assert any(o["name"] == "2ROAD" for o in g2["objects"])
# the plate follows the ground on a ~1 m grid and the run-off grass makes way for it
plate = next(o for o in g2["objects"] if o["name"] == "2ROAD")
assert len(plate["faces"]) > 20 and all(math.dist(plate["verts"][f[0]][:2], plate["verts"][f[1]][:2]) <= 1.01
                                        for f in plate["faces"]), len(plate["faces"])
grass = next(o for o in g2["objects"] if o["name"] == "1GRASS")
on_plate = [f for f in grass["faces"] if all(R + 6.5 < grass["verts"][v][0] < R + 8.5 and abs(grass["verts"][v][1]) < 1.5 for v in f)]
assert not on_plate, len(on_plate)
assert trackgen._in_poly((1, 1), [(0, 0), (2, 0), (2, 2), (0, 2)]) and not trackgen._in_poly((3, 1), [(0, 0), (2, 0), (2, 2), (0, 2)])
# concave plate (U shape): ear clipping covers exactly its area, all faces up, nothing in the notch
u_shape = [(0, 0), (6, 0), (6, 4), (4, 4), (4, 1), (2, 1), (2, 4), (0, 4)]
tris = trackgen._triangulate(u_shape)
area = lambda a, b, c: ((b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])) / 2
assert all(area(*t) > 0 for t in tris) and abs(sum(area(*t) for t in tris) - 18.0) < 1e-9, tris
pv, pf, _ = trackgen._plate(u_shape, lambda x, y: 0.0)
assert not any(trackgen._in_poly((sum(pv[i][0] for i in f) / 3, sum(pv[i][1] for i in f) / 3), [(2, 1), (4, 1), (4, 4), (2, 4)])
               for f in pf)

# kerb styles: each lands in its own object (physics), kerb UVs run 0 -> 1 across the kerb, unknown style = error
for style, name in (("apron", "1APRON"), ("concrete", "2APRON"), ("stripes", "1KERB")):
    feats["kerbs"][0]["style"] = style
    g3 = trackgen.build(cfg, pts, None, feats)
    objs = {o["name"]: o for o in g3["objects"]}
    assert name in objs and not ({"1KERB", "1APRON", "2APRON"} - {name}) & set(objs), sorted(objs)
    us = sorted({round(u, 6) for u, _ in objs[name]["uvs"]})
    assert us[0] == 0.0 and us[-1] == 1.0, us
    road_z = max(v[2] for v in objs["1ROAD"]["verts"])  # flat test track: road at z = 0
    top = max(v[2] for v in objs[name]["verts"]) - road_z
    # every kerb climbs from the asphalt (u = 0, road level) to the verge grass (u = 1, GRASS_LIFT_M above it);
    # the pours are that ramp, the striped kerbs hump 5 cm on the way
    assert abs(top - (0.05 if style == "stripes" else trackgen.GRASS_LIFT_M)) < 1e-9, (style, top)
    for (x, y, z), (u, _) in zip(objs[name]["verts"], objs[name]["uvs"]):
        assert u > 0 or abs(z - road_z) < 1e-9, ("inner edge on the asphalt", z)
        assert u < 1 or abs(z - road_z - trackgen.GRASS_LIFT_M) < 1e-9, ("outer edge on the grass", z)
    grass_z = [v[2] for v in objs["1GRASS"]["verts"] if abs(math.hypot(*v[:2]) - R) < 6]
    assert abs(max(grass_z) - road_z - trackgen.GRASS_LIFT_M) < 1e-9, "verge grass a little above the asphalt"
# kerbs narrow to a point at their ends over their taper (start, end); none where a kerb meets another kerb
f = trackgen._kerb_taper([None, 0, 0, 0, 0, 0, 1, 1, None, None], [[2.0, 4.0], None], 1.0, 1.0)
assert f[1] == 0.0 and abs(f[2] - 0.5) < 1e-9 and f[3] == 1.0, f  # kerb 0 starts with a 2 m taper
assert f[5] == 1.0 and f[6] == 1.0  # kerb 0 -> kerb 1: no neck where they meet
assert f[8] == 0.0 and abs(f[7] - 1.0) < 1e-9, f  # kerb 1 ends with the 1 m default taper
kerb_v = next(o for o in g2["objects"] if o["name"] == "1KERB")["verts"]
widths = sorted(abs(math.hypot(*v[:2]) - R) for v in kerb_v)  # this test kerb is on the outside: |r - R| - 4 = width
assert widths[0] < 4.2 and widths[-1] > 4.5, (widths[0], widths[-1])  # pointed ends, full width in the middle

# raised kerbs ramp up from 0 at both ends of every run (also across the seam), full lift after ramp_m
up = trackgen._end_taper([False, True, True, True, True, False, False, False], 0.03, 1.0, 1.5)
assert up[1] == 0.0 and up[5] == 0.0 and abs(up[2] - 0.02) < 1e-9 and abs(up[3] - 0.03) < 1e-9 and up[7] == 0.0, up
seam = trackgen._end_taper([True, True, False, False, False, False, True, True], 0.03, 1.0, 0.5)
assert seam[0] == 0.03 and seam[2] == 0.0 and seam[6] == 0.0 and seam[7] == 0.03, seam

# terrain: explicit triangles, and its height function lies exactly on them (run-off edge, plates, walls use it)
mesh, hz = trackgen._terrain_grid(lambda x, y: 0.1 * x + 0.02 * x * y, (0.0, 0.0, 8.0, 8.0), 4.0)
tv, tf, _ = mesh()
assert all(len(f) == 3 for f in tf) and len(tf) == 2 * 2 * 2, len(tf)
mesh9, _ = trackgen._terrain_grid(lambda x, y: 0.0, (0.0, 0.0, 9.0, 6.0), 4.0)  # ends exactly on the extent
assert {(min(v[k] for v in mesh9()[0]), max(v[k] for v in mesh9()[0])) for k in (0, 1)} == {(0.0, 9.0), (0.0, 6.0)}
for f in tf:
    (ax, ay, az), (bx, by, bz), (cx, cy, cz) = (tv[i] for i in f)
    px, py = (ax + bx + cx) / 3, (ay + by + cy) / 3
    assert abs(hz(px, py) - (az + bz + cz) / 3) < 1e-9

feats["kerbs"][0]["style"] = "zebra"
try:
    trackgen.build(cfg, pts, None, feats)
    raise AssertionError("unknown kerb style accepted")
except ValueError as e:
    assert "zebra" in str(e)
del feats["kerbs"][0]["style"]
feats["barriers"].append({"name": "on_road", "points": [[R - 10, 1.0], [R + 10, 1.0]]})
assert any("crosses on_road" in e for e in trackgen.validate(cfg, pts, trackgen.build(cfg, pts, None, feats))[0])

# spawns sit on the centreline ring and point along it
for e in geo["empties"]:
    if e["name"].startswith("AC_START"):
        x, y, _ = e["pos"]
        assert abs(math.hypot(x, y) - R) < 2.5 and abs(e["fwd"][0] * x + e["fwd"][1] * y) < 0.1 * R, e

# painted grid pattern and lane lines, relative to the start line: slots on the pattern (row k // 2, column k % 2),
# lane lines parallel to the centreline (constant offset, common start and end), boxes + chequered line drawn
gfeats = dict(feats, barriers=[b for b in feats["barriers"] if b["name"] != "on_road"],
              grid={"first_m": 1.0, "row_m": 7.0, "stagger_m": 2.0, "lateral_m": [-1.2, 1.2], "painted": 4},
              painted_lines=[{"name": f"lane{k}", "offset": o, "from_m": -4.0, "to_m": 30.0} for k, o in enumerate((-2.0, 2.0))])
g4 = trackgen.build(cfg, pts, None, gfeats)
st = [e for e in g4["empties"] if e["name"].startswith("AC_START")]
assert len(st) == cfg["grid_slots"]
def lateral(p):  # signed distance to the centreline, + = left (inside of this CCW circle)
    s = min(g4["samples"], key=lambda q: math.dist(q[:2], p[:2]))
    return math.hypot(*s[:2]) - math.hypot(*p[:2])
for k, e in enumerate(st):
    assert abs(lateral(e["pos"]) - gfeats["grid"]["lateral_m"][k % 2]) < 0.05, (k, lateral(e["pos"]))
gap = lambda a, b: math.dist(st[a]["pos"][:2], st[b]["pos"][:2])
row_at = lambda k: 7.0 * (R - gfeats["grid"]["lateral_m"][k % 2]) / R  # rows measured along the centreline
assert all(abs(gap(k, k - 2) - row_at(k)) < 0.05 for k in range(2, len(st))), [round(gap(k, k - 2), 2) for k in range(2, len(st))]
lanes = next(o for o in g4["objects"] if o["name"] == "LINES_LANES")
lat = [lateral(v) for v in lanes["verts"]]  # each lane is 0.1 m wide: its two edges at offset +-0.05
for o in (-2.0, 2.0):
    mine = [d for d in lat if abs(d - o) < 0.5]
    assert mine and max(mine) - min(mine) < 0.12, ("lane not parallel to the centreline", o, min(mine), max(mine))
names = {o["name"] for o in g4["objects"]}
assert {"LINES_LANES", "LINES_GRID", "LINES_START"} <= names, names
assert len(next(o for o in g4["objects"] if o["name"] == "LINES_GRID")["faces"]) == 4 * 3  # painted boxes only
road_z4 = max(v[2] for o in g4["objects"] if o["name"] == "1ROAD" for v in o["verts"])  # flat test track
for o in g4["objects"]:  # markings lie on the asphalt (a few mm up against z-fighting), they don't float
    if o["name"].startswith("LINES_"):
        assert all(0 < v[2] - road_z4 <= 0.011 for v in o["verts"]), (o["name"], max(v[2] for v in o["verts"]) - road_z4)
assert trackgen.validate(cfg, pts, g4)[0] == [], trackgen.validate(cfg, pts, g4)[0]

# two photos: grass/terrain faces wholly inside the sharper track-area photo use it, the rest the terrain photo;
# no face lost, UVs within 0..1 of their own photo
far_photo = {"x0": -200.0, "y0": -200.0, "x1": 200.0, "y1": 200.0}  # (this test's run-off reaches ~135 m out)
near_photo = {"x0": -100.0, "y0": -100.0, "x1": 100.0, "y1": 100.0}
g5, g6 = trackgen.build(cfg, pts, None, feats, far_photo), trackgen.build(cfg, pts, None, feats, far_photo, near_photo)
count = lambda g, names: sum(len(o["faces"]) for o in g["objects"] if o["name"] in names)
assert count(g5, {"1GRASS", "2GRASS"}) == count(g6, {"1GRASS", "2GRASS", "3GRASS", "4GRASS"})
mats = {o["name"]: o["material"] for o in g6["objects"]}
assert mats["1GRASS"] == mats["2GRASS"] == "ortho_near" and mats["3GRASS"] == "ortho", mats
assert all(-1e-9 <= c <= 1 + 1e-9 for o in g6["objects"] if o["material"].startswith("ortho") for uv in o["uvs"] for c in uv)

# a broken input (two points far apart) must be flagged
broken = pts[:8] + [(p[0] + 200, p[1], 0.0, 8.0) for p in pts[8:]]
assert trackgen.validate(cfg, broken, trackgen.build(cfg, broken))[0]
# edge fitting: 8 m asphalt, white edge line, then a shaded greenish verge (unsaturated, like asphalt) -> edge on the line
def photo(x, y):
    d = abs(y)
    return (120, 125, 124) if d < 3.9 else (230, 230, 228) if d < 4.0 else (88, 101, 90) if d < 5.0 else (110, 135, 100)
road = [[float(x), 0.0, 0.0, 8.0] for x in range(5)]
e = fit.fit_edges(road, [(0.0, 1.0)] * 5, lambda x, y: fit.classify(photo(x, y)))
assert all(abs(a - 4.0) < 0.15 and abs(b - 4.0) < 0.15 for a, b in e), e

assert abs(fit.ray_hit((0, 0), (0, 1), [[(-1, 3), (1, 3)]], 5) - 3) < 1e-9 and fit.ray_hit((0, 0), (0, 1), [[(-1, 3), (1, 3)]], 2) is None

# procedural textures (numpy: bundled with Blender, optional here): right size, 0..1, repeatable, seamless tiles
try:
    import numpy as np
    import textures
except ImportError:
    print("numpy not installed - texture checks skipped (Blender has its own)")
else:
    for tname, (make, strength) in textures.SURFACES.items():
        albedo, height = make()
        assert albedo.ndim == 3 and albedo.shape[2] == 3 and albedo.shape[:2] == height.shape, tname
        assert albedo.min() >= 0 and albedo.max() <= 1, tname
        assert np.array_equal(albedo, make()[0]), f"{tname}: not deterministic"
        nm = textures.normal_map(height, strength)
        assert nm.min() >= 0 and nm.max() <= 1 and nm[..., 2].min() > 0.5, tname
        # across the tile seam the height changes no more than between interior neighbours (the kerb stripes'
        # hard edges along v are excluded: they repeat every stripe, also across the seam)
        for axis in (0, 1) if tname != "kerb" else (1,):
            seam = np.abs(np.take(height, 0, axis) - np.take(height, -1, axis)).mean()
            inner = np.abs(np.diff(height, axis=axis)).mean()
            assert seam < 3 * inner, (tname, axis, seam, inner)
    # orthophoto detail textures: tileable multipliers around DETAIL_MEAN; the mask splits grass from bare ground
    for tname, (make, strength, tile) in textures.DETAILS.items():
        albedo, height = make()
        assert tile > 0 and abs(albedo.mean() - textures.DETAIL_MEAN) < 0.05, (tname, albedo.mean())
        for axis in (0, 1):
            seam = np.abs(np.take(height, 0, axis) - np.take(height, -1, axis)).mean()
            assert seam < 3 * np.abs(np.diff(height, axis=axis)).mean(), (tname, axis)
    photo_px = np.zeros((40, 40, 3))
    photo_px[:, :20] = (0.25, 0.40, 0.18)  # grass
    photo_px[:, 20:] = (0.55, 0.48, 0.36)  # dry bare soil
    m = textures.ortho_mask(photo_px)
    assert m.shape == (40, 40, 4) and np.allclose(m[..., 0] + m[..., 1], 1) and not m[..., 2:].any()
    assert m[:, :10, 0].min() > 0.99 and m[:, 30:, 1].min() > 0.99, "mask: grass in R, bare soil in G"
    # asphalt brightness map: 1 at the reference asphalt, paint / tyres ignored (filled from around), clipped
    road_px = np.full((40, 40, 3), 0.40)
    road_px[:, 25:] = 0.44  # a lighter, less rubbered band
    road_px[18:21, :] = 0.95  # a white line across
    road_px[5:8, 5:8] = (0.6, 0.1, 0.1)  # a red tyre
    r = textures.asphalt_macro(road_px, ref_px=(np.arange(40).repeat(10), np.tile(np.arange(10), 40)))
    assert abs(r[10, 5] - 1) < 0.01 and abs(r[30, 35] - 1.1) < 0.01, (r[10, 5], r[30, 35])
    assert abs(r[19, 5] - 1) < 0.02 and abs(r[6, 6] - 1) < 0.02, "paint and tyres filled from the asphalt around"
    assert r.min() >= 0.9 and r.max() <= 1.2

# ground height: exact track height on the asphalt even with a higher straight 8 m away (markings sit on it), smooth
# in between (plates across an old shortcut don't step where the nearest straight switches)
two = [[float(x), 0.0, 0.0, 4.0] for x in range(0, 40)] + [[float(x), 8.0, 0.6, 4.0] for x in range(39, -1, -1)]
zf = trackgen._ground_fn(two, [10.0] * len(two), None)
assert abs(zf(20.3, 1.5) + trackgen.GROUND_SINK_M) < 1e-9, zf(20.3, 1.5)  # on the low straight, 1.5 m off its axis
assert abs(zf(20.3, 6.5) - 0.6 + trackgen.GROUND_SINK_M) < 1e-9  # on the high one
mid = [zf(20.3, 2.0 + 0.1 * k) for k in range(41)]  # edge to edge across the gap
assert all(b >= a - 1e-9 for a, b in zip(mid, mid[1:])) and max(b - a for a, b in zip(mid, mid[1:])) < 0.05, mid

# ksEditor vertex count (KN5 limit): smooth = vertices, flat = face corners (upper bound)
quad = {"verts": [(0, 0, 0), (1, 0, 0), (1, 1, 0), (0, 1, 0), (2, 0, 0), (2, 1, 0)], "faces": [[0, 1, 2, 3], [1, 4, 5, 2]]}
assert trackgen.ac_vertex_count({**quad, "material": "road"}) == 6
assert trackgen.ac_vertex_count({**quad, "material": "kerb"}) == 8
assert all(trackgen.ac_vertex_count(o) < 65535 for o in geo["objects"]), "an object over the KN5 vertex limit"

# ksEditor persistence: every shader variable listed (unset = 0), textures in their slots (unset = NULL), CRLF
import persistence  # noqa: E402
ini = persistence.fbx_ini("t.fbx", {"road": ("ksMultilayer_fresnel_nm", {"multR": 24.8}, {"txDiffuse": "a.png"})},
                          ["1ROAD"], ["AC_START_0"], no_shadow=["1ROAD"]).split("\r\n")
assert "VARCOUNT=16" in ini and "VAR_6_FLOAT1=24.8" in ini and "VAR_0_FLOAT1=0.35" in ini, "values and lighting"
assert "RES_0_TEXTURE=a.png" in ini and "RES_6_SLOT=21" in ini and "RES_6_TEXTURE=NULL" in ini
assert "[model_FBX: t.fbx_1ROAD_1ROAD]" in ini and "CAST_SHADOWS=0" in ini
assert "[model_FBX: t.fbx_AC_START_0]" in ini and "[model_FBX: t.fbx_AC_START_0_AC_START_0]" not in ini
try:
    persistence.fbx_ini("t.fbx", {"x": ("ksPerPixel", {}, {"txNormal": "n.png"})}, [], [])
    raise AssertionError("a slot the shader doesn't have must fail")
except AssertionError as e:
    assert "has no" in str(e), e

# trees: three crossed vertical planes each, double-sided, standing on the ground; the sprite atlas has alpha
tv, tf, tu = trackgen._trees([[0.0, 0.0, 2.0, 8.0], [10.0, 5.0, 3.0, 12.0]], lambda x, y: 1.0)
assert len(tf) == 2 * 3 * 2 and len(tv) == 2 * 3 * 4, (len(tf), len(tv))
assert min(v[2] for v in tv) == 0.9 and max(v[2] for v in tv[:12]) == 8.9, "0.1 m into the ground, h tall"
assert all(abs(trackgen._normal(tv, f)[2]) < 1e-9 for f in tf), "vertical planes"
atlas = textures.tree_atlas(w=64, h=128)
assert atlas.shape == (128, 64 * trackgen.TREE_VARIANTS, 4) and set(np.unique(atlas[..., 3])) <= {0.0, 1.0}
assert 0.15 < atlas[..., 3].mean() < 0.6, atlas[..., 3].mean()

# stands: benches along the line, the fence `away` behind them (+left normal here: y > 0), 1.8 m tall, posts
sb, sf, sline = trackgen._stands([[0.0, 0.0], [10.0, 0.0]], lambda x, y: 0.0, 1)
assert len(sb) >= 3 * 3 and all(abs(p[1] - trackgen.FENCE["offset"]) < 1e-9 for p in sline), sline
assert max(v[2] for v, _, _ in sf for v in v) == trackgen.FENCE["height"] - 0.05
seat_tops = {round(max(v[2] for v in part[0]), 3) for part in sb}
assert trackgen.BENCH["seat"] in seat_tops, seat_tops

# KN5 writer: a one-quad track + a start dummy round-trips through the layout ksEditor writes; AC axes (x, z, -y),
# uv (u, -v), winding kept (face normal along the vertex normal), tangent orthogonal to the normal
import struct  # noqa: E402
import tempfile  # noqa: E402
import kn5  # noqa: E402
quad = {"objects": [{"name": "1ROAD", "material": "road", "verts": [(0, 0, 1), (4, 0, 1), (4, 4, 1), (0, 4, 1)],
                     "faces": [[0, 1, 2, 3]], "uvs": [(0, 0), (1, 0), (1, 1), (0, 1)]}],
        "empties": [{"name": "AC_START_0", "pos": [1, 2, 1], "fwd": [0.0, 1.0]}]}
with tempfile.TemporaryDirectory() as tmp:
    open(os.path.join(tmp, "a.png"), "wb").write(b"\x89PNGtest")
    kn5.write(os.path.join(tmp, "t.kn5"), quad, {"road": ("ksPerPixel", {}, {"txDiffuse": "a.png"})}, tmp, "t")
    blob = open(os.path.join(tmp, "t.kn5"), "rb").read()
assert blob[:6] == b"sc6969" and struct.unpack_from("<iii", blob, 6) == (6, 0, 1)
at = blob.index(b"1ROAD") + 5 + 4 + 1 + 3  # name, child count, active, cast / visible / transparent
nv = struct.unpack_from("<i", blob, at)[0]
vs = [struct.unpack_from("<11f", blob, at + 4 + 44 * k) for k in range(nv)]
ni = struct.unpack_from("<i", blob, at + 4 + 44 * nv)[0]
tri = struct.unpack_from("<3H", blob, at + 8 + 44 * nv)
assert nv == 4 and ni == 6, (nv, ni)
assert vs[2][:3] == (4.0, 1.0, -4.0) and vs[2][6:8] == (1.0, -1.0), vs[2]  # (x, z, -y), (u, -v)
e1, e2 = [[vs[tri[k]][i] - vs[tri[0]][i] for i in range(3)] for k in (1, 2)]
fn = (e1[1] * e2[2] - e1[2] * e2[1], e1[2] * e2[0] - e1[0] * e2[2], e1[0] * e2[1] - e1[1] * e2[0])
assert all(abs(v[4] - 1) < 1e-6 for v in vs) and fn[1] > 0, ("normal up, winding kept", fn)
assert all(abs(sum(v[3 + i] * v[8 + i] for i in range(3))) < 1e-6 for v in vs), "tangent _|_ normal"
start = blob.index(b"AC_START_0") + 10 + 4 + 1
assert struct.unpack_from("<16f", blob, start)[8:15] == (0.0, 0.0, -1.0, 0.0, 1.0, 1.0, -2.0), "Z row = driving direction"

print("ok", f"{geo['length']:.2f} m,", sum(len(o['faces']) for o in geo['objects']), "faces")
