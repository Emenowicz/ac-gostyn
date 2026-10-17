"""Self-check for trackgen: python3 test_trackgen.py"""
import json
import math
import os

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
print("ok", f"{geo['length']:.2f} m,", sum(len(o['faces']) for o in geo['objects']), "faces")
