"""Track geometry from data: centreline + widths + elevation -> meshes, AC empties, validation.

Pure stdlib, so it runs (and is testable) without Blender. Coordinates: x east, y north, z up, metres.
"""
import csv
import json
import math
import os
from collections import defaultdict
from itertools import combinations

UV_M = 4.0  # metres per texture tile
GROUND_SINK_M = 0.05  # terrain sits below the drivable surfaces to avoid z-fighting


def load(track_path, csv_path):
    with open(track_path, encoding="utf-8") as f:
        cfg = json.load(f)
    pts = []
    with open(csv_path, newline="", encoding="utf-8") as f:
        for r in csv.DictReader(line for line in f if not line.startswith("#")):
            pts.append((float(r["x"]), float(r["y"]),
                        float(r.get("z") or 0), float(r.get("width") or cfg["default_width_m"])))
    if cfg.get("reverse"):
        pts.reverse()
    # drop duplicates (incl. a repeated closing point) - they break the centripetal parametrisation
    pts = [p for i, p in enumerate(pts) if math.dist(p[:2], pts[i - 1][:2]) > 0.01]
    return cfg, pts


# --- curve -------------------------------------------------------------------------------------

def _catmull_rom(p0, p1, p2, p3, u):
    """Centripetal Catmull-Rom (Barry-Goldman) on (x, y, z, width), parametrised by xy distance."""
    t1 = math.dist(p0[:2], p1[:2]) ** 0.5
    t2 = t1 + math.dist(p1[:2], p2[:2]) ** 0.5
    t3 = t2 + math.dist(p2[:2], p3[:2]) ** 0.5
    t = t1 + (t2 - t1) * u

    def lerp(a, b, ta, tb):
        return [((tb - t) * x + (t - ta) * y) / (tb - ta) for x, y in zip(a, b)]

    a1, a2, a3 = lerp(p0, p1, 0, t1), lerp(p1, p2, t1, t2), lerp(p2, p3, t2, t3)
    return lerp(lerp(a1, a2, 0, t2), lerp(a2, a3, t1, t3), t1, t2)


def resample(pts, step):
    """Closed spline through pts, resampled to equal arc steps. Returns (samples [x,y,z,w], length)."""
    n = len(pts)
    dense = []
    for i in range(n):
        p0, p1, p2, p3 = (pts[(i + j) % n] for j in (-1, 0, 1, 2))
        m = max(2, int(4 * math.dist(p1[:2], p2[:2]) / step) + 1)
        dense += [_catmull_rom(p0, p1, p2, p3, k / m) for k in range(m)]
    dense.append(dense[0])
    cum = [0.0]
    for a, b in zip(dense, dense[1:]):
        cum.append(cum[-1] + math.dist(a[:3], b[:3]))
    length = cum[-1]
    count = max(3, round(length / step))
    out, j = [], 0
    for k in range(count):
        s = k * length / count
        while cum[j + 1] < s:
            j += 1
        f = (s - cum[j]) / ((cum[j + 1] - cum[j]) or 1)
        out.append([a + (b - a) * f for a, b in zip(dense[j], dense[j + 1])])
    return out, length


def _box(v, r):
    n = len(v)
    return [sum(v[(i + j) % n] for j in range(-r, r + 1)) / (2 * r + 1) for i in range(n)]


def _min_filter(v, r):
    n = len(v)
    return [min(v[(i + j) % n] for j in range(-r, r + 1)) for i in range(n)]


def frames(samples, ds):
    """Unit tangents, left normals and smoothed signed curvature (+ = left turn) per sample."""
    n = len(samples)
    tang, heading = [], []
    for i in range(n):
        a, b = samples[i - 1], samples[(i + 1) % n]
        dx, dy = b[0] - a[0], b[1] - a[1]
        d = math.hypot(dx, dy)
        tang.append((dx / d, dy / d))
        c = samples[(i + 1) % n]
        heading.append(math.atan2(c[1] - samples[i][1], c[0] - samples[i][0]))
    k = [math.remainder(heading[i] - heading[i - 1], math.tau) / ds for i in range(n)]
    left = [(-ty, tx) for tx, ty in tang]
    return tang, left, _box(k, max(1, round(3 / ds)))


def clearance(samples, left):
    """Per sample and side: radius of the largest empty disc touching the centreline there.

    Offsets kept below it can't fold on tight inside corners or overlap a neighbouring section of
    track - one rule for both. For each other point j the disc centre along the normal at distance
    t is equidistant from i and j when t = |d|^2 / (2 * d.n).
    ponytail: O(n^2), ~1 s for 1 km at 1 m steps; use a grid/KD-tree if tracks get much longer.
    """
    n = len(samples)
    caps = {1: [math.inf] * n, -1: [math.inf] * n}
    for i in range(n):
        xi, yi = samples[i][0], samples[i][1]
        nx, ny = left[i]
        for j in range(n):
            if j == i:
                continue
            dx, dy = samples[j][0] - xi, samples[j][1] - yi
            lat = dx * nx + dy * ny
            if abs(lat) > 1e-6:
                side = 1 if lat > 0 else -1
                t = (dx * dx + dy * dy) / (2 * abs(lat))
                if t < caps[side][i]:
                    caps[side][i] = t
    return caps


# --- meshes ------------------------------------------------------------------------------------

def _strip(rows_a, rows_b, mask, flip):
    """Quad strip between two rows of (pos, uv) (len n+1). mask[i] enables the quad i..i+1."""
    verts, uvs, faces, idx = [], [], [], {}

    def vid(k, rows):
        key = (k, rows is rows_b)
        if key not in idx:
            idx[key] = len(verts)
            verts.append(rows[k][0])
            uvs.append(rows[k][1])
        return idx[key]

    for i, on in enumerate(mask):
        if on:
            q = [vid(i, rows_a), vid(i + 1, rows_a), vid(i + 1, rows_b), vid(i, rows_b)]
            faces.append(q[::-1] if flip else q)
    return verts, faces, uvs


def _merge(name, material, parts):
    verts, faces, uvs = [], [], []
    for v, f, uv in parts:
        base = len(verts)
        verts += v
        uvs += uv
        faces += [[base + i for i in face] for face in f]
    return {"name": name, "material": material, "verts": verts, "faces": faces, "uvs": uvs}


def load_ground(path):
    """Regular x,y,h grid (data/elevation.csv) -> bilinear height(x, y), or None if there is no file."""
    if not os.path.exists(path):
        return None
    with open(path, newline="", encoding="utf-8") as f:
        rows = [(float(r["x"]), float(r["y"]), float(r["h"])) for r in csv.DictReader(line for line in f if not line.startswith("#"))]
    xs, ys = sorted({r[0] for r in rows}), sorted({r[1] for r in rows})
    cell = xs[1] - xs[0]
    grid = {(round((x - xs[0]) / cell), round((y - ys[0]) / cell)): h for x, y, h in rows}

    def height(x, y):
        fx = min(max((x - xs[0]) / cell, 0), len(xs) - 1.0001)
        fy = min(max((y - ys[0]) / cell, 0), len(ys) - 1.0001)
        i, j, tx, ty = int(fx), int(fy), fx % 1, fy % 1
        return ((grid[i, j] * (1 - tx) + grid[i + 1, j] * tx) * (1 - ty)
                + (grid[i, j + 1] * (1 - tx) + grid[i + 1, j + 1] * tx) * ty)
    return height


def _terrain(samples, reach, extent, cell, ground):
    """Grid at the track's height inside `reach` of the centreline, blending into the NMT ground beyond it.

    ponytail: nearest-sample search is O(vertices x samples), ~2 s here; use a grid index if it grows.
    """
    x0, y0, x1, y1 = extent
    nx, ny = int((x1 - x0) / cell) + 1, int((y1 - y0) / cell) + 1
    verts, uvs = [], []
    for j in range(ny + 1):
        for i in range(nx + 1):
            x, y = x0 + i * cell, y0 + j * cell
            d2, k = min(((x - s[0]) ** 2 + (y - s[1]) ** 2, k) for k, s in enumerate(samples))
            z = samples[k][2] - GROUND_SINK_M
            if ground:
                t = min(max((math.sqrt(d2) - reach[k]) / 6.0, 0.0), 1.0)
                z += t * (ground(x, y) - GROUND_SINK_M - z)
            verts.append((x, y, z))
            uvs.append((x / UV_M, y / UV_M))
    faces = [[j * (nx + 1) + i, j * (nx + 1) + i + 1, (j + 1) * (nx + 1) + i + 1, (j + 1) * (nx + 1) + i]
             for j in range(ny) for i in range(nx)]
    return verts, faces, uvs


def build(cfg, pts, ground=None):
    samples, length = resample(pts, cfg["sample_step_m"])
    n = len(samples)
    ds = length / n
    r = round(cfg.get("smooth_m", 0) / ds)
    if r:  # tracing noise (GPS/OSM, +-1-2 m) makes the spline wiggle; the asphalt edge would fold on it
        samples = [list(p) for p in zip(*(_box(_box([s[c] for s in samples], r), r) for c in range(4)))]
        length = sum(math.dist(a[:3], b[:3]) for a, b in zip(samples, samples[1:] + samples[:1]))
        ds = length / n
    r = round(cfg.get("z_smooth_m", 0) / 2 / ds)
    if r:  # NMT is noisy by ~10 cm; the asphalt shouldn't be
        for sm, z in zip(samples, _box(_box([sm[2] for sm in samples], r), r)):
            sm[2] = z
    tang, left, curv = frames(samples, ds)
    caps = clearance(samples, left)
    r = max(1, round(5 / ds))
    caps = {s: _box(_min_filter(c, r), r) for s, c in caps.items()}  # smoothed, never above the raw cap

    half = [s[3] / 2 for s in samples]
    kerb_out, wall_off = {}, {}
    for side in (1, -1):
        kerb_out[side] = [max(h + 0.1, min(h + cfg["kerb_width_m"], c - 0.2)) for h, c in zip(half, caps[side])]
        wall_off[side] = [max(ko + 0.1, min(h + cfg["kerb_width_m"] + cfg["runoff_m"], 0.95 * c))
                          for h, ko, c in zip(half, kerb_out[side], caps[side])]
    kerb = {side: [side * k > cfg["kerb_min_curvature"] for k in curv] for side in (1, -1)}  # inside of corners

    def rows(offs, dz=0.0, wall=False):
        out = []
        for k in range(n + 1):  # row n duplicates row 0 at s=length, so UVs don't wrap at the seam
            i = k % n
            x, y, z = samples[i][0] + offs[i] * left[i][0], samples[i][1] + offs[i] * left[i][1], samples[i][2] + dz
            s = k * ds
            out.append(((x, y, z), (s / UV_M, dz / UV_M) if wall else (offs[i] / UV_M, s / UV_M)))
        return out

    every = [True] * n
    road_l, road_r = rows(half), rows([-h for h in half])
    objects = [_merge("1ROAD", "road", [_strip(road_r, road_l, every, False)])]
    kerbs, grass, walls = [], [], []
    for side in (1, -1):
        edge = road_l if side == 1 else road_r
        ko = rows([side * o for o in kerb_out[side]])
        wo = [side * o for o in wall_off[side]]
        flip = side < 0
        kerbs.append(_strip(edge, ko, kerb[side], flip))
        grass.append(_strip(edge, ko, [not m for m in kerb[side]], flip))
        grass.append(_strip(ko, rows(wo), every, flip))
        walls.append(_strip(rows(wo, wall=True), rows(wo, cfg["wall_height_m"], wall=True), every, flip))
    objects += [_merge("1KERB", "kerb", kerbs), _merge("1GRASS", "grass", grass), _merge("1WALL", "wall", walls)]

    wall_lines = [[p for p, _ in rows([side * o for o in wall_off[side]])] for side in (1, -1)]
    xs = [p[0] for line in wall_lines for p in line]
    ys = [p[1] for line in wall_lines for p in line]
    m = cfg["terrain_margin_m"]
    reach = [max(wall_off[1][i], wall_off[-1][i]) + 1.0 for i in range(n)]
    objects.append(_merge("2GRASS", "ground", [_terrain(samples, reach, (min(xs) - m, min(ys) - m, max(xs) + m, max(ys) + m),
                                                        cfg["terrain_cell_m"], ground)]))
    objects = [o for o in objects if o["faces"]]

    return {
        "samples": samples, "length": length, "ds": ds, "curvature": curv,
        "objects": objects, "empties": _empties(cfg, samples, tang, left, length, ds),
        "polylines": {"road_l": [p for p, _ in road_l], "road_r": [p for p, _ in road_r],
                      "wall_l": wall_lines[0], "wall_r": wall_lines[1]},
        "corners": sum(1 for i in range(n) if (abs(curv[i]) > cfg["kerb_min_curvature"])
                       and not abs(curv[i - 1]) > cfg["kerb_min_curvature"]),
    }


def _empties(cfg, samples, tang, left, length, ds):
    n = len(samples)
    flip = -1 if cfg.get("spawn_flip") else 1

    # start line = centreline sample nearest to start_xy, so editing the centreline doesn't move it
    i0 = min(range(n), key=lambda i: math.dist(samples[i][:2], cfg["start_xy"]))

    def at(back, lateral, name):
        i = (i0 - round(back / ds)) % n
        x, y, z = samples[i][:3]
        return {"name": name, "pos": (x + lateral * left[i][0], y + lateral * left[i][1], z),
                "fwd": (flip * tang[i][0], flip * tang[i][1])}

    half = samples[i0][3] / 2
    sp = cfg["grid_spacing_m"]
    out = [at(0, half + 0.5, "AC_TIME_0_L"), at(0, -half - 0.5, "AC_TIME_0_R"),
           at(cfg["hotlap_back_m"], 0, "AC_HOTLAP_START_0")]
    out += [at(3 + (k // 2) * sp + (k % 2) * sp / 2, (half / 2) * (1 if k % 2 == 0 else -1), f"AC_START_{k}")
            for k in range(cfg["grid_slots"])]
    grid_end = 3 + ((cfg["grid_slots"] + 1) // 2 + 1) * sp
    out += [at(grid_end + k * sp, 0, f"AC_PIT_{k}") for k in range(cfg["pit_slots"])]
    return out


# --- validation --------------------------------------------------------------------------------

def _crossings(polylines, cell=5.0):
    """Pairs of non-adjacent segments that properly intersect (xy). Grid-bucketed."""
    segs = [(a, b, pi, i) for pi, line in enumerate(polylines) for i, (a, b) in enumerate(zip(line, line[1:]))]
    grid = defaultdict(list)
    for k, (a, b, _, _) in enumerate(segs):
        for cx in range(math.floor(min(a[0], b[0]) / cell), math.floor(max(a[0], b[0]) / cell) + 1):
            for cy in range(math.floor(min(a[1], b[1]) / cell), math.floor(max(a[1], b[1]) / cell) + 1):
                grid[cx, cy].append(k)

    def orient(p, q, r):
        return (q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0])

    hits = set()
    for bucket in grid.values():
        for u, v in combinations(bucket, 2):
            a, b, pa, ia = segs[u]
            c, d, pb, ib = segs[v]
            last = len(polylines[pa]) - 2
            if pa == pb and (abs(ia - ib) <= 1 or {ia, ib} == {0, last}):
                continue
            if orient(a, b, c) * orient(a, b, d) < 0 and orient(c, d, a) * orient(c, d, b) < 0:
                hits.add((min(u, v), max(u, v)))
    return [(segs[u], segs[v]) for u, v in sorted(hits)]


def _normal(verts, face):
    nx = ny = nz = 0.0
    for a, b in zip(face, face[1:] + face[:1]):  # Newell's method
        (x1, y1, z1), (x2, y2, z2) = verts[a], verts[b]
        nx += (y1 - y2) * (z1 + z2)
        ny += (z1 - z2) * (x1 + x2)
        nz += (x1 - x2) * (y1 + y2)
    return nx, ny, nz


def validate(cfg, pts, geo):
    """Returns (errors, warnings). Errors make the track unusable; warnings need a human look."""
    errors, warnings = [], []
    L, target, tol = geo["length"], cfg["length_m"], cfg["length_tolerance"]
    dev = (L - target) / target
    msg = f"length {L:.1f} m vs official {target} m ({dev:+.1%}, tolerance {tol:.0%})"
    (errors if abs(dev) > tol else warnings if abs(dev) > 0.01 else []).append(msg)

    widths = [s[3] for s in geo["samples"]]
    lo, hi = cfg["width_sanity_m"]
    bad = [i for i, w in enumerate(widths) if not lo <= w <= hi]
    if bad:
        errors.append(f"width outside sanity range {lo}-{hi} m at {len(bad)} samples, "
                      f"first at s={bad[0] * geo['ds']:.0f} m ({widths[bad[0]]:.2f} m)")
    off = [w for w in widths if not cfg["width_min_m"] <= w <= cfg["width_max_m"]]
    if off:  # measured widths may legitimately differ a bit from the website's round numbers
        warnings.append(f"width outside official {cfg['width_min_m']}-{cfg['width_max_m']} m on {len(off) * geo['ds']:.0f} m "
                        f"of track (range {min(widths):.1f}-{max(widths):.1f} m)")

    s = 0.0
    for a, b in zip(pts, pts[1:] + pts[:1]):
        gap = math.dist(a[:2], b[:2])
        if gap > cfg["max_input_gap_m"]:
            warnings.append(f"input gap {gap:.0f} m between centreline points at s~{s:.0f} m - verify this section")
        s += gap

    sm = geo["samples"]
    k = max(1, round(5 / geo["ds"]))
    grade = max(abs(sm[(i + k) % len(sm)][2] - sm[i][2]) / (k * geo["ds"]) for i in range(len(sm)))
    if grade > cfg.get("max_grade", 0.08):
        warnings.append(f"steepest 5 m of track: {grade:.1%} grade - check elevation data")
    worst = max(math.dist(a[:3], b[:3]) for a, b in zip(sm, sm[1:] + sm[:1]))
    if worst > 1.5 * geo["ds"]:
        errors.append(f"surface discontinuity: sample spacing {worst:.2f} m > 1.5 x {geo['ds']:.2f} m")

    hits = _crossings(list(geo["polylines"].values()))
    names = list(geo["polylines"])
    for (a, _, pa, _), (_, _, pb, _) in hits[:5]:
        errors.append(f"{names[pa]} crosses {names[pb]} near ({a[0]:.0f}, {a[1]:.0f})")
    if len(hits) > 5:
        errors.append(f"... {len(hits) - 5} more edge crossings")

    for o in geo["objects"]:
        if len(o["verts"]) >= 65535:
            errors.append(f"{o['name']}: {len(o['verts'])} vertices, KN5 limit is 65534 per object - split it")
        if o["name"] == "1ROAD" and len(o["faces"]) != len(sm):
            errors.append(f"1ROAD has {len(o['faces'])} quads for {len(sm)} samples - asphalt has holes")
        wall = o["material"] == "wall"
        for f in o["faces"]:
            nx, ny, nz = _normal(o["verts"], f)
            area = math.sqrt(nx * nx + ny * ny + nz * nz)
            if area < 1e-9 or (abs(nz) > 0.01 * area if wall else nz <= 0):
                errors.append(f"{o['name']}: degenerate or flipped face {f}")
                break

    warnings.append(f"corners detected (|k| > {cfg['kerb_min_curvature']}): {geo['corners']} "
                    f"vs official {cfg['turns']} - rough shape check only")
    if cfg.get("placeholder"):
        warnings.append("PLACEHOLDER values in use: " + ", ".join(cfg["placeholder"]))
    return errors, warnings
