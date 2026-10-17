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

# CCW circle: left = inside. Inner wall must stay between the road and the centre, outer wall beyond the road.
assert all(4.0 < R - math.hypot(*p[:2]) < R for p in geo["polylines"]["wall_l"])
assert all(math.hypot(*p[:2]) > R + 4.0 for p in geo["polylines"]["wall_r"])

# walls face the track: inner wall normals point outward (away from centre), outer wall normals inward
wall = next(o for o in geo["objects"] if o["name"] == "1WALL")
for f in wall["faces"]:
    nx, ny, _ = trackgen._normal(wall["verts"], f)
    cx, cy = (sum(wall["verts"][i][k] for i in f) / 4 for k in (0, 1))
    toward_centre = -(nx * cx + ny * cy)
    assert (toward_centre < 0) == (math.hypot(cx, cy) < R), f

# spawns sit on the centreline ring and point along it
for e in geo["empties"]:
    if e["name"].startswith("AC_START"):
        x, y, _ = e["pos"]
        assert abs(math.hypot(x, y) - R) < 2.5 and abs(e["fwd"][0] * x + e["fwd"][1] * y) < 0.1 * R, e

# a broken input (two points far apart) must be flagged
broken = pts[:8] + [(p[0] + 200, p[1], 0.0, 8.0) for p in pts[8:]]
assert trackgen.validate(cfg, broken, trackgen.build(cfg, broken))[0]
print("ok", f"{geo['length']:.2f} m,", sum(len(o['faces']) for o in geo['objects']), "faces")
