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
assert 0 < len(kerb["faces"]) < 20 and all(v[1] > R - 10 for v in kerb["verts"]), len(kerb["faces"])
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
    if style != "stripes":  # concrete pours: raised 3 cm, ramps at both sides, edges stay at road level
        assert abs(top - 0.03) < 1e-9 and min(v[2] for v in objs[name]["verts"]) == road_z, top
    else:
        assert abs(top) < 1e-9, top
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

print("ok", f"{geo['length']:.2f} m,", sum(len(o['faces']) for o in geo['objects']), "faces")
