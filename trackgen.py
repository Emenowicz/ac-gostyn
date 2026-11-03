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
# kerb "style" in features.json -> (AC object name = physics, material, raised by [m]). Painted kerbs drive as KERB;
# the concrete pours (red or grey) are slightly raised like a kerb but rumble much less: own APRON surface
# (project author). One object per material, so the two pours are 1APRON and 2APRON.
KERB_STYLES = {"stripes": ("1KERB", "kerb", 0.0),  # red/yellow stripes, 0.5 m
               "apron": ("1APRON", "kerb_apron", 0.03),  # red-dyed concrete band with a light outer border
               "concrete": ("2APRON", "kerb_concrete", 0.03)}  # plain grey concrete
KERB_OBJECTS = {name for name, _, _ in KERB_STYLES.values()}
KERB_RAMP_M = 0.15  # a raised kerb climbs and drops over this width at both sides (no vertical step to catch a wheel)
KERB_END_RAMP_M = 0.6  # ... and over this length where it starts and ends along the track


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


def _end_taper(mask, lift, ds, ramp_m):
    """Per-sample lift of a raised kerb along its quads `mask` (circular): 0 at both ends of every run, full `lift`
    after `ramp_m`, so the kerb doesn't start with a step (and an open end you can see under)."""
    n = len(mask)
    on = [mask[r - 1] or mask[r] for r in range(n)]  # row r joins quads r-1 and r
    end = [mask[r - 1] != mask[r] for r in range(n)]
    if not any(end):
        return [lift if on[0] else 0.0] * n
    d = [0 if end[r] or not on[r] else n for r in range(n)]  # rows from the nearest end of the run
    for _ in range(2):  # twice round, for runs across the seam
        for r in range(n):
            d[r] = min(d[r], d[r - 1] + 1)
        for r in reversed(range(n)):
            d[r] = min(d[r], d[(r + 1) % n] + 1)
    return [lift * min(1.0, d[r] * ds / ramp_m) if on[r] else 0.0 for r in range(n)]


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


def _ground_fn(samples, reach, ground):
    """Height of the ground at (x, y): track height inside `reach` of the centreline, blending into NMT beyond it.

    ponytail: nearest-sample search is O(samples) per call, ~2 s for the whole terrain; use a grid index if it grows.
    """
    def z_at(x, y):
        d2, k = min(((x - s[0]) ** 2 + (y - s[1]) ** 2, k) for k, s in enumerate(samples))
        z = samples[k][2] - GROUND_SINK_M
        if ground:
            t = min(max((math.sqrt(d2) - reach[k]) / 6.0, 0.0), 1.0)
            z += t * (ground(x, y) - GROUND_SINK_M - z)
        return z
    return z_at


def _terrain_grid(z_at, extent, cell):
    """Terrain mesh on a `cell` grid and its exact height function.

    Each cell is split into two triangles along the same diagonal ((i, j)-(i+1, j+1)), written out as triangles so
    no exporter or ksEditor picks the diagonal: the run-off edge, plates and barrier feet can then sit exactly on it.
    """
    x0, y0, x1, y1 = extent
    # cells of `cell` m, the last column/row narrower so the mesh ends exactly on the extent (the photo's: UVs 0..1)
    nx, ny = max(1, math.ceil((x1 - x0) / cell - 1e-9)), max(1, math.ceil((y1 - y0) / cell - 1e-9))
    gx = [min(x0 + i * cell, x1) for i in range(nx + 1)]
    gy = [min(y0 + j * cell, y1) for j in range(ny + 1)]
    zs = {}

    def zv(i, j):
        if (i, j) not in zs:
            zs[i, j] = z_at(gx[i], gy[j])
        return zs[i, j]

    def height(x, y):
        i = min(max(math.floor((x - x0) / cell), 0), nx - 1)
        j = min(max(math.floor((y - y0) / cell), 0), ny - 1)
        tx, ty = (x - gx[i]) / (gx[i + 1] - gx[i]), (y - gy[j]) / (gy[j + 1] - gy[j])
        z00, z10, z01, z11 = zv(i, j), zv(i + 1, j), zv(i, j + 1), zv(i + 1, j + 1)
        if tx >= ty:  # triangle (00, 10, 11)
            return z00 + tx * (z10 - z00) + ty * (z11 - z10)
        return z00 + tx * (z11 - z01) + ty * (z01 - z00)  # triangle (00, 11, 01)

    def mesh():
        verts = [(gx[i], gy[j], zv(i, j)) for j in range(ny + 1) for i in range(nx + 1)]
        v = lambda i, j: j * (nx + 1) + i
        faces = [f for j in range(ny) for i in range(nx)
                 for f in ([v(i, j), v(i + 1, j), v(i + 1, j + 1)], [v(i, j), v(i + 1, j + 1), v(i, j + 1)])]
        return verts, faces, [(p[0] / UV_M, p[1] / UV_M) for p in verts]
    return mesh, height


def _barrier(points, z_at, height, thick):
    """Tyre wall along a polyline: two vertical sides + top, `thick` wide, standing on the ground."""
    m = len(points)
    closed = m > 2 and math.dist(points[0], points[-1]) < 1e-6
    rows = {}
    for side in (1, -1):
        for top in (False, True):
            rows[side, top] = []
    s = 0.0
    for i, (x, y) in enumerate(points):
        a = points[i - 1] if i > 0 else (points[-2] if closed else points[0])
        b = points[i + 1] if i < m - 1 else (points[1] if closed else points[-1])
        dx, dy = b[0] - a[0], b[1] - a[1]
        d = math.hypot(dx, dy) or 1.0
        nx, ny = -dy / d, dx / d
        if i:
            s += math.dist(points[i - 1], (x, y))
        zb = z_at(x, y)
        for side in (1, -1):
            px, py = x + side * thick / 2 * nx, y + side * thick / 2 * ny
            rows[side, False].append(((px, py, zb - 0.1), (s / UV_M, 0.0)))
            rows[side, True].append(((px, py, zb + height), (s / UV_M, height / UV_M)))
    every = [True] * (m - 1)
    return [_strip(rows[1, False], rows[1, True], every, True), _strip(rows[-1, False], rows[-1, True], every, False),
            _strip(rows[-1, True], rows[1, True], every, False)]


def _in_poly(p, poly):
    """Point-in-polygon (even-odd ray crossing)."""
    x, y, inside = p[0], p[1], False
    for (ax, ay), (bx, by) in zip(poly, poly[1:] + poly[:1]):
        if (ay > y) != (by > y) and x < ax + (y - ay) * (bx - ax) / (by - ay):
            inside = not inside
    return inside


def _triangulate(poly):
    """Ear clipping of a simple polygon (convex or not) -> counter-clockwise triangles."""
    cross = lambda o, a, b: (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
    pts = list(poly)
    if sum(a[0] * b[1] - b[0] * a[1] for a, b in zip(pts, pts[1:] + pts[:1])) < 0:
        pts.reverse()  # counter-clockwise -> faces point up
    tris = []
    while len(pts) > 3:
        for i in range(len(pts)):
            a, b, c = pts[i - 1], pts[i], pts[(i + 1) % len(pts)]
            if cross(a, b, c) <= 1e-12:
                continue  # reflex (or collinear) corner: not an ear
            if any(cross(a, b, p) >= 0 and cross(b, c, p) >= 0 and cross(c, a, p) >= 0
                   for p in pts if p not in (a, b, c)):
                continue  # another corner inside: cutting here would overlap
            tris.append((a, b, c))
            del pts[i]
            break
        else:
            raise ValueError(f"extra_asphalt polygon is not simple (self-intersecting?): {poly}")
    return tris + [tuple(pts)]


def _plate(poly, z_at, cell=1.0):
    """Simple polygon, ear-clipped, each triangle split into a k x k triangle grid (~`cell` m) so the plate follows the
    ground instead of spanning it flat (the terrain between the corners would poke through). One k for the whole plate,
    so an edge shared by two triangles is split at the same points on both sides."""
    tris = _triangulate(poly)
    k = max(1, math.ceil(max(math.dist(a, b) for t in tris for a, b in ((t[0], t[1]), (t[1], t[2]), (t[2], t[0]))) / cell))
    verts, faces = [], []
    for a, b, c in tris:
        idx = {}
        for i in range(k + 1):
            for j in range(k + 1 - i):
                x = a[0] + i / k * (b[0] - a[0]) + j / k * (c[0] - a[0])
                y = a[1] + i / k * (b[1] - a[1]) + j / k * (c[1] - a[1])
                idx[i, j] = len(verts)
                verts.append((x, y, z_at(x, y) + GROUND_SINK_M - 0.02))  # just under the main asphalt where they overlap
        for i in range(k):
            for j in range(k - i):
                faces.append([idx[i, j], idx[i + 1, j], idx[i, j + 1]])
                if i + j < k - 1:
                    faces.append([idx[i + 1, j], idx[i + 1, j + 1], idx[i, j + 1]])
    return verts, faces, [(x / UV_M, y / UV_M) for x, y, _ in verts]


TYRE_R = 0.3  # tyre barriers: car tyres lying flat, 0.6 m across (= wall_thickness_m), ~0.2 m each
TYRE_BLOCK = 3  # stacks per colour block (red / yellow alternate, as painted)


def _tyre_stacks(points, base_fn, height, seg=8):
    """Visual tyre stacks along a barrier polyline: one stack every 2 * TYRE_R, round(height / 0.2) tyres high.

    Each stack is a ring profile (waisted at every joint between tyres, a flat top annulus around the hole) with
    u around the stack and v up it. Returns {colour: (verts, faces, uvs)}; the collision stays the plain 1WALL box.
    """
    n_t = max(1, round(height / 0.2))
    th, top = height / n_t, height + 0.01  # 1 cm above the collision box, so its top doesn't flicker through
    prof = [(k * th / 2, TYRE_R * (0.88 if k % 2 == 0 else 1.0)) for k in range(2 * n_t + 1)]
    prof[-1] = (top, prof[-1][1])
    prof.append((top, TYRE_R * 0.55))  # inner edge of the top annulus; the dark box shows through as the hole
    total = sum(math.dist(a, b) for a, b in zip(points, points[1:]))
    count = max(1, round(total / (2 * TYRE_R)))
    out = {"tyre_red": ([], [], []), "tyre_yellow": ([], [], [])}
    seg_i, done = 0, 0.0
    for j in range(count + (0 if points[0] == points[-1] else 1)):
        s = j * total / count if count else 0.0
        while seg_i < len(points) - 2 and done + math.dist(points[seg_i], points[seg_i + 1]) < s:
            done += math.dist(points[seg_i], points[seg_i + 1])
            seg_i += 1
        a, b = points[seg_i], points[seg_i + 1]
        t = min(1.0, (s - done) / (math.dist(a, b) or 1.0))
        cx, cy = a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])
        z0 = base_fn(cx, cy)
        verts, faces, uvs = out["tyre_red" if (j // TYRE_BLOCK) % 2 == 0 else "tyre_yellow"]
        base = len(verts)
        for z, r in prof:
            for k in range(seg + 1):  # seg + 1 columns: the texture seam gets its own vertices
                ang = 2 * math.pi * k / seg
                verts.append((cx + r * math.cos(ang), cy + r * math.sin(ang), z0 + z))
                uvs.append((k / seg, min(z, height) / height))
        for ring in range(len(prof) - 1):
            for k in range(seg):
                p = base + ring * (seg + 1) + k
                faces.append([p, p + 1, p + seg + 2, p + seg + 1])  # outward walls, then the top annulus (up)
    return out


def _ribbon(points, width, zfn):
    """Flat strip `width` wide centred on a polyline, faces up; u 0 (right of the polyline's direction) -> 1 (left),
    v = distance / UV_M along. For painted markings (no physics)."""
    verts, uvs, faces, s = [], [], [], 0.0
    m = len(points)
    for i, (x, y) in enumerate(points):
        a, b = points[max(i - 1, 0)], points[min(i + 1, m - 1)]
        d = math.hypot(b[0] - a[0], b[1] - a[1]) or 1.0
        lx, ly = -(b[1] - a[1]) / d, (b[0] - a[0]) / d
        if i:
            s += math.dist(points[i - 1], (x, y))
        for side, u in ((-1, 0.0), (1, 1.0)):  # right, left
            px, py = x + side * width / 2 * lx, y + side * width / 2 * ly
            verts.append((px, py, zfn(px, py)))
            uvs.append((u, s / UV_M))
    for i in range(m - 1):
        faces.append([2 * i, 2 * i + 2, 2 * i + 3, 2 * i + 1])  # right, right next, left next, left: up
    return verts, faces, uvs


GRID_BOX = (1.2, 1.5, 0.12)  # painted grid box (U open at the back): length along, width across, line width [m]
START_LINE_M = 0.8  # chequered start/finish band, across the track (orthophoto)


def _grid_box(cx, cy, fx, fy, zfn):
    """U-shaped grid box centred on a start slot, closed at the front (driving direction fx, fy)."""
    ln, wd, lw = GRID_BOX
    lx, ly = -fy, fx
    p = lambda a, l: (cx + a * fx + l * lx, cy + a * fy + l * ly)
    parts = [_ribbon([p(ln / 2 - lw / 2, -wd / 2), p(ln / 2 - lw / 2, wd / 2)], lw, zfn)]  # front bar
    for side in (-1, 1):
        parts.append(_ribbon([p(-ln / 2, side * (wd / 2 - lw / 2)), p(ln / 2 - lw, side * (wd / 2 - lw / 2))], lw, zfn))
    return parts


def _dist_to_polyline(p, line):
    best = math.inf
    for a, b in zip(line, line[1:]):
        dx, dy = b[0] - a[0], b[1] - a[1]
        t = max(0.0, min(1.0, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / (dx * dx + dy * dy or 1.0)))
        best = min(best, math.hypot(a[0] + t * dx - p[0], a[1] + t * dy - p[1]))
    return best


def load_features(path):
    """data/features.json (traced barriers, kerbs, start line, extra asphalt), or None if there is no file."""
    if not os.path.exists(path):
        return None
    with open(path, encoding="utf-8") as f:
        return json.load(f)


def _split(o, keep):
    """Object -> (faces where keep(face), the other faces), each with only the vertices it uses."""
    parts = []
    for want in (True, False):
        remap, verts, uvs, faces = {}, [], [], []
        for f in o["faces"]:
            if bool(keep(f)) != want:
                continue
            for i in f:
                if i not in remap:
                    remap[i] = len(verts)
                    verts.append(o["verts"][i])
                    uvs.append(o["uvs"][i])
            faces.append([remap[i] for i in f])
        parts.append(dict(o, verts=verts, uvs=uvs, faces=faces))
    return parts


def build(cfg, pts, ground=None, features=None, photo=None, photo_near=None):
    """photo / photo_near: ortho.json / ortho_near.json extents (local metres) - the terrain photo and an optional
    sharper one of the track area, draped on the grass and terrain faces wholly inside it."""
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
    raw_caps = clearance(samples, left)
    r = max(1, round(cfg.get("offset_smooth_m", 5) / ds))
    caps = {s: _box(_min_filter(c, r), r) for s, c in raw_caps.items()}  # smoothed, never above the raw cap

    half = [s[3] / 2 for s in samples]
    kw = cfg["kerb_width_m"]
    if features:  # kerbs where the asphalt edge runs along a traced kerb strip; each strip may carry its measured width
        klines = features.get("kerbs", [])

        def nearest_kerb(p):
            d, j = min(((_dist_to_polyline(p, k["points"]), j) for j, k in enumerate(klines)), default=(math.inf, None))
            return j if d < cfg["kerb_snap_m"] else None
        kerb_id = {side: [nearest_kerb((sm[0] + side * h * l[0], sm[1] + side * h * l[1])) for sm, h, l in zip(samples, half, left)]
                   for side in (1, -1)}
        kerb = {side: [j is not None for j in ids] for side, ids in kerb_id.items()}
        kwidth = {side: [klines[j].get("width", kw) if j is not None else kw for j in ids] for side, ids in kerb_id.items()}
    else:  # fallback: inside of corners
        kerb_id = {side: [None] * n for side in (1, -1)}
        kerb = {side: [side * k > cfg["kerb_min_curvature"] for k in curv] for side in (1, -1)}
        kwidth = {side: [kw] * n for side in (1, -1)}
    kerb_out, wall_off = {}, {}
    for side in (1, -1):
        # kerbs keep a constant width along the asphalt edge (as painted) unless the free space really ends;
        # the raw cap is used because the smoothed one also narrows the kerb before and after a tight apex
        kerb_out[side] = [max(h + 0.1, min(h + w, 0.9 * c)) for h, w, c in zip(half, kwidth[side], raw_caps[side])]
        # run-off just stops where space runs out (the terrain covers the rest) instead of pinching into a notch
        wall_off[side] = [ko + 0.1 if 0.85 * c - ko < 0.5 else min(max(h + kw + cfg["runoff_m"], ko + 0.5), 0.85 * c)
                          for h, ko, c in zip(half, kerb_out[side], caps[side])]

    def rows(offs, dz=0.0, u=None, zfn=None):
        """Row of (pos, uv) at lateral offsets `offs`. dz: scalar or per-sample list; u: fixed texture u for the whole
        row (kerbs: 0 = asphalt side, 1 = outer side); zfn(x, y): height from a function instead of the track's."""
        out = []
        for k in range(n + 1):  # row n duplicates row 0 at s=length, so UVs don't wrap at the seam
            i = k % n
            x, y = samples[i][0] + offs[i] * left[i][0], samples[i][1] + offs[i] * left[i][1]
            z = (zfn(x, y) if zfn else samples[i][2]) + (dz[i] if isinstance(dz, list) else dz)
            out.append(((x, y, z), (offs[i] / UV_M if u is None else u, k * ds / UV_M)))
        return out

    every = [True] * n
    road_l, road_r = rows(half), rows([-h for h in half])
    objects = [_merge("1ROAD", "road", [_strip(road_r, road_l, every, False)])]
    # asphalt texture in plan (like the plates): along-track UVs squeeze it at the inside of a hairpin
    objects[0]["uvs"] = [(v[0] / UV_M, v[1] / UV_M) for v in objects[0]["verts"]]
    # painted lines: visual only (no digit prefix = no physics in AC), lifted 5 mm so they don't z-fight the asphalt
    lift, lw = 0.005, cfg["line_width_m"]
    edge_lines = [_strip(rows([side * (h - lw - 0.05) for h in half], lift), rows([side * (h - 0.05) for h in half], lift),
                         every, side < 0) for side in (1, -1)]
    objects.append(_merge("LINES_WHITE", "paint_white", edge_lines))
    plates = [p["points"] for p in (features or {}).get("extra_asphalt", [])]

    # the terrain comes first: the run-off's outer edge, the plates and the barrier feet sit exactly on it
    reach = [max(wall_off[1][i], wall_off[-1][i]) + 1.0 for i in range(n)]
    z_at = _ground_fn(samples, reach, ground)
    outer_xy = {side: [p[:2] for p, _ in rows([side * o for o in wall_off[side]])] for side in (1, -1)}
    if features:
        barriers = {b["name"]: [tuple(p) for p in b["points"]] for b in features.get("barriers", [])}
    else:  # no traced barriers: a wall along the outer edge of the run-off on both sides
        barriers = {f"auto_{'l' if side == 1 else 'r'}": outer_xy[side] for side in (1, -1)}
    pts_all = [p[:2] for line in list(outer_xy.values()) + list(barriers.values()) for p in line]
    m = cfg["terrain_margin_m"]
    extent = ((photo["x0"], photo["y0"], photo["x1"], photo["y1"]) if photo else
              (min(p[0] for p in pts_all) - m, min(p[1] for p in pts_all) - m,
               max(p[0] for p in pts_all) + m, max(p[1] for p in pts_all) + m))
    terrain_mesh, ground_z = _terrain_grid(z_at, extent, cfg["terrain_cell_m"])

    def off_plate(a, b):  # run-off quads i..i+1 between rows a and b, dropped only where they lie wholly on a plate
        return [not any(all(_in_poly(p[0], pl) for p in (a[i], a[i + 1], b[i], b[i + 1])) for pl in plates)
                for i in range(n)]  # (a quad only partly on a plate would leave a hole down to the terrain)

    klines = (features or {}).get("kerbs", [])
    for k in klines:
        if k.get("style", "stripes") not in KERB_STYLES:
            raise ValueError(f"kerb {k['name']}: unknown style {k['style']!r} (one of {', '.join(KERB_STYLES)})")
    style = {side: [klines[j].get("style", "stripes") if j is not None else "stripes" for j in ids]
             for side, ids in kerb_id.items()}
    kerbs, grass, outer = {s: [] for s in KERB_STYLES}, [], {}
    for side in (1, -1):
        edge = road_l if side == 1 else road_r
        ko = rows([side * o for o in kerb_out[side]])
        outer[side] = rows([side * o for o in wall_off[side]], zfn=ground_z)  # run-off slopes down onto the terrain
        flip = side < 0
        for s, (_, _, lift) in KERB_STYLES.items():
            mask = [m and st == s for m, st in zip(kerb[side], style[side])]
            if lift:  # cross-section: edge -> ramp up -> flat top -> ramp down -> outer edge (u 0 -> 1 across)
                ramp = [min(KERB_RAMP_M, (o - h) / 3) for h, o in zip(half, kerb_out[side])]
                up = _end_taper(mask, lift, ds, KERB_END_RAMP_M)  # and along the track: no step where it starts/ends
                prof = [rows([side * h for h in half], u=0.0),
                        rows([side * (h + r) for h, r in zip(half, ramp)], up, u=0.1),
                        rows([side * (o - r) for o, r in zip(kerb_out[side], ramp)], up, u=0.9),
                        rows([side * o for o in kerb_out[side]], u=1.0)]
                kerbs[s] += [_strip(a, b, mask, flip) for a, b in zip(prof, prof[1:])]
            else:
                kerbs[s].append(_strip(rows([side * h for h in half], u=0.0),
                                       rows([side * o for o in kerb_out[side]], u=1.0), mask, flip))
        grass.append(_strip(edge, ko, [not m and f for m, f in zip(kerb[side], off_plate(edge, ko))], flip))
        grass.append(_strip(ko, outer[side], off_plate(ko, outer[side]), flip))
    objects += [_merge(name, material, kerbs[s]) for s, (name, material, _) in KERB_STYLES.items()]
    objects.append(_merge("1GRASS", "grass", grass))

    edge_lines = {side: [p for p, _ in outer[side]] for side in (1, -1)}
    heights = {b["name"]: b.get("height", cfg["wall_height_m"]) for b in (features or {}).get("barriers", [])}
    walls = [part for name, line in barriers.items()
             for part in _barrier(line, ground_z, heights.get(name, cfg["wall_height_m"]), cfg["wall_thickness_m"])]
    x0, y0, x1, y1 = (v + d for v, d in zip(extent, (1, 1, -1, -1)))  # perimeter, so nobody drives off the world
    # collision box around the tyre stacks; "collider" is a fully transparent alpha-test material: AC doesn't draw it
    objects.append(_merge("1WALL", "collider", walls))
    on_kerb = {b["name"] for b in (features or {}).get("barriers", []) if b.get("on_kerb")}
    stacks = {"tyre_red": [], "tyre_yellow": []}
    for name, line in barriers.items():
        if name.startswith("auto_"):
            continue
        # tyres on a kerb stand on it (track level); the rest on the terrain, sunk 2 cm so they don't float
        base = (lambda x, y: max(ground_z(x, y), z_at(x, y) + GROUND_SINK_M) - 0.02) if name in on_kerb else \
               (lambda x, y: ground_z(x, y) - 0.02)
        for colour, part in _tyre_stacks(line, base, heights.get(name, cfg["wall_height_m"])).items():
            stacks[colour].append(part)
    objects += [_merge(f"TYRES_{c.split('_')[1].upper()}", c, parts) for c, parts in stacks.items()]  # visual only

    # painted markings (visual only, lifted like the edge lines; the chequered line a little higher where they cross)
    on_road = lambda dz: (lambda x, y: z_at(x, y) + GROUND_SINK_M + dz)
    along = _along(samples, tang, left, ds, _start_index(cfg, samples, features)[0])
    lanes = []
    for pl in (features or {}).get("painted_lines", []):  # parallel to the centreline, from_m..to_m before the line
        a, b = pl["from_m"], pl["to_m"]
        steps = max(1, math.ceil(abs(b - a) / ds))
        lanes.append(_ribbon([along(a + (b - a) * k / steps, pl["offset"])[:2] for k in range(steps + 1)],
                             pl.get("width", lw), on_road(lift)))
    for cl in (features or {}).get("painted_cross", []):  # dashed line across the asphalt, at_m before the line
        x, y, _, (tx, ty) = along(cl["at_m"], 0.0)
        i = min(range(n), key=lambda k: math.dist(samples[k][:2], (x, y)))
        reach, dash, gap = samples[i][3] / 2 - 0.15, cl.get("dash_m", 0.3), cl.get("gap_m", 0.2)
        t = -reach
        while t < reach:  # each dash a short ribbon along the left normal (-ty, tx)
            e = min(t + dash, reach)
            lanes.append(_ribbon([(x - ty * t, y + tx * t), (x - ty * e, y + tx * e)], cl.get("width", 0.12),
                                 on_road(1.5 * lift)))
            t = e + gap
    objects.append(_merge("LINES_LANES", "paint_white", lanes))
    empties = _empties(cfg, samples, tang, left, ds, features)
    painted = ((features or {}).get("grid") or {}).get("painted", 0)
    boxes = [part for e in empties[3:3 + painted]
             for part in _grid_box(e["pos"][0], e["pos"][1], e["fwd"][0], e["fwd"][1], on_road(lift))]
    objects.append(_merge("LINES_GRID", "paint_mint", boxes))
    if features and features.get("start_finish"):
        objects.append(_merge("LINES_START", "chequer",
                              [_ribbon([tuple(p) for p in features["start_finish"]], START_LINE_M, on_road(2 * lift))]))
    objects.append(_merge("2WALL", "concrete", _barrier([(x0, y0), (x1, y0), (x1, y1), (x0, y1), (x0, y0)], ground_z, 2.0, 0.5)))
    if plates:  # on the terrain mesh, or on z_at where that is higher (under the track asphalt they overlap)
        objects.append(_merge("2ROAD", "road", [_plate(pl, lambda x, y: max(z_at(x, y), ground_z(x, y))) for pl in plates]))
    objects.append(_merge("2GRASS", "ground", [terrain_mesh()]))
    objects = [o for o in objects if o["faces"]]
    if photo:  # orthophoto draped over the terrain and the run-off grass: planar UVs over the photo's extent
        def photo_uv(ph):  # local metres -> UV of that photo: its affine (EPSG:2180, turned) or its axis-aligned extent
            if "uv" in ph:
                (au, bu, cu), (av, bv, cv) = ph["uv"]
                return lambda x, y: (au * x + bu * y + cu, av * x + bv * y + cv)
            return lambda x, y: ((x - ph["x0"]) / (ph["x1"] - ph["x0"]), (y - ph["y0"]) / (ph["y1"] - ph["y0"]))
        drape = lambda o, mat, ph: dict(o, material=mat, uvs=[photo_uv(ph)(v[0], v[1]) for v in o["verts"]])
        out = []
        for o in objects:
            if o["name"] not in ("1GRASS", "2GRASS"):
                out.append(o)
            elif photo_near:  # faces wholly inside the sharper track-area photo use it; the rest the terrain photo
                ne = (photo_near["x0"], photo_near["y0"], photo_near["x1"], photo_near["y1"])
                inside = lambda f: all(ne[0] <= o["verts"][i][0] <= ne[2] and ne[1] <= o["verts"][i][1] <= ne[3] for i in f)
                near, far = _split(o, inside)
                far["name"] = {"1GRASS": "4GRASS", "2GRASS": "3GRASS"}[o["name"]]  # same GRASS physics
                out += [drape(near, "ortho_near", photo_near), drape(far, "ortho", photo)]
            else:
                out.append(drape(o, "ortho", photo))
        objects = [o for o in out if o["faces"]]

    return {
        "samples": samples, "length": length, "ds": ds, "curvature": curv,
        "objects": objects, "empties": empties,
        "polylines": {"road_l": [p for p, _ in road_l], "road_r": [p for p, _ in road_r],
                      "edge_l": edge_lines[1], "edge_r": edge_lines[-1]},
        "barriers": barriers, "kerb_id": kerb_id, "left": left,
        "on_kerb": {b["name"] for b in (features or {}).get("barriers", []) if b.get("on_kerb")},
        "corners": sum(1 for i in range(n) if (abs(curv[i]) > cfg["kerb_min_curvature"])
                       and not abs(curv[i - 1]) > cfg["kerb_min_curvature"]),
    }


def _start_index(cfg, samples, features):
    """Centreline sample nearest the start/finish line (or start_xy), so editing the centreline doesn't move it."""
    line = (features or {}).get("start_finish")
    sx, sy = ((line[0][0] + line[1][0]) / 2, (line[0][1] + line[1][1]) / 2) if line else cfg["start_xy"]
    return min(range(len(samples)), key=lambda i: math.dist(samples[i][:2], (sx, sy))), (sx, sy)


def _along(samples, tang, left, ds, i0):
    """(back, lateral) -> (x, y, z, tangent): `back` metres before the start line against the driving direction
    (negative = after it), `lateral` metres to the left; interpolated between samples."""
    n = len(samples)

    def at(back, lateral):
        f = i0 - back / ds
        i, t = math.floor(f), f - math.floor(f)
        a, b = i % n, (i + 1) % n
        lerp = lambda p, q: tuple(p[k] + t * (q[k] - p[k]) for k in range(len(p)))
        (x, y, z), (lx, ly), (tx, ty) = lerp(samples[a][:3], samples[b][:3]), lerp(left[a], left[b]), lerp(tang[a], tang[b])
        dl, dt = math.hypot(lx, ly) or 1.0, math.hypot(tx, ty) or 1.0
        return x + lateral * lx / dl, y + lateral * ly / dl, z, (tx / dt, ty / dt)
    return at


def _empties(cfg, samples, tang, left, ds, features):
    flip = -1 if cfg.get("spawn_flip") else 1
    line = features.get("start_finish") if features else None
    i0, (sx, sy) = _start_index(cfg, samples, features)
    along = _along(samples, tang, left, ds, i0)

    def at(back, lateral, name):
        x, y, z, (tx, ty) = along(back, lateral)
        return {"name": name, "pos": (x, y, z), "fwd": (flip * tx, flip * ty)}

    half = samples[i0][3] / 2
    sp = cfg["grid_spacing_m"]
    out = [at(0, half + 0.5, "AC_TIME_0_L"), at(0, -half - 0.5, "AC_TIME_0_R"),
           at(cfg["hotlap_back_m"], 0, "AC_HOTLAP_START_0")]
    if line:  # timing gates on the painted line, 0.5 m beyond each end
        z, (lx, ly) = samples[i0][2], left[i0]
        for e in out[:2]:
            p = max(line, key=lambda q: (q[0] - sx) * lx * (1 if e["name"].endswith("L") else -1)
                    + (q[1] - sy) * ly * (1 if e["name"].endswith("L") else -1))
            d = math.dist(p, (sx, sy)) or 1.0
            e["pos"] = (p[0] + (p[0] - sx) / d * 0.5, p[1] + (p[1] - sy) / d * 0.5, z)
    grid = (features or {}).get("grid")
    if grid:  # the painted grid pattern (orthophoto): slot k in row k // 2, column k % 2 (column 0 nearest the line)
        out += [at(grid["first_m"] + (k // 2) * grid["row_m"] + (k % 2) * grid["stagger_m"],
                   grid["lateral_m"][k % 2], f"AC_START_{k}") for k in range(cfg["grid_slots"])]
    else:  # two staggered columns, sp apart
        out += [at(3 + (k // 2) * sp + (k % 2) * sp / 2, (half / 2) * (1 if k % 2 == 0 else -1), f"AC_START_{k}")
                for k in range(cfg["grid_slots"])]
    # pit boxes share the grid slots: the paddock isn't modelled, and AC uses pits (practice) and grid (race) separately
    out += [dict(e, name=f"AC_PIT_{k}") for k, e in enumerate(out[3:3 + cfg["pit_slots"]])]
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

    tang = frames(sm, geo["ds"])[0]
    for key in ("edge_l", "edge_r"):  # run-off edge must move forward with the track, never fold back
        line = geo["polylines"][key]
        back = [i for i in range(len(sm)) if (line[i + 1][0] - line[i][0]) * tang[i][0] + (line[i + 1][1] - line[i][1]) * tang[i][1] < 0]
        if back:
            errors.append(f"{key} folds back at {len(back)} samples, first near ({line[back[0]][0]:.0f}, {line[back[0]][1]:.0f})")
    for key in ("road_l", "road_r"):  # a very tight inner edge makes kerbs/run-off pinch into a notch
        line = geo["polylines"][key]
        for i in range(3, len(line) - 3):
            a, b, c = line[i - 3], line[i], line[i + 3]
            area2 = abs((b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]))
            if area2 > 1e-9 and math.dist(a[:2], b[:2]) * math.dist(b[:2], c[:2]) * math.dist(c[:2], a[:2]) / (2 * area2) < 2.5:
                warnings.append(f"{key}: asphalt edge radius < 2.5 m near ({b[0]:.0f}, {b[1]:.0f}) - raise smooth_m or check the width")
                break
    for group in (geo["polylines"], {"road_l": geo["polylines"]["road_l"], "road_r": geo["polylines"]["road_r"],
                                     **geo["barriers"]}):
        names = list(group)
        hits = _crossings(list(group.values()))
        for (a, _, pa, _), (_, _, pb, _) in hits[:5]:
            errors.append(f"{names[pa]} crosses {names[pb]} near ({a[0]:.0f}, {a[1]:.0f})")
        if len(hits) > 5:
            errors.append(f"... {len(hits) - 5} more crossings")

    # barrier footprint (centre line +- half thickness) vs asphalt and kerbs; tyres on a kerb can be real (apex markers)
    quads = {o["name"]: [[o["verts"][i] for i in f] for f in o["faces"]] for o in geo["objects"]
             if o["name"] == "1ROAD" or o["name"] in KERB_OBJECTS}
    ht = cfg["wall_thickness_m"] / 2

    def inside(p, poly):
        c = False
        for (x1, y1, _), (x2, y2, _) in zip(poly, poly[1:] + poly[:1]):
            if (y1 > p[1]) != (y2 > p[1]) and p[0] < (x2 - x1) * (p[1] - y1) / (y2 - y1) + x1:
                c = not c
        return c

    for name, line in geo["barriers"].items():
        if name.startswith("auto_"):
            continue
        on = set()
        for a, b in zip(line, line[1:]):
            steps = max(1, int(math.dist(a, b) / 0.25))
            for k in range(steps + 1):
                p = (a[0] + (b[0] - a[0]) * k / steps, a[1] + (b[1] - a[1]) * k / steps)
                for surface, qs in quads.items():
                    if surface not in on and any(
                            min(v[0] for v in q) - ht <= p[0] <= max(v[0] for v in q) + ht
                            and min(v[1] for v in q) - ht <= p[1] <= max(v[1] for v in q) + ht
                            and any(inside((p[0] + dx, p[1] + dy), q) for dx, dy in ((0, 0), (ht, 0), (-ht, 0), (0, ht), (0, -ht)))
                            for q in qs):
                        on.add(surface)
        if "1ROAD" in on:
            errors.append(f"barrier {name} stands on the asphalt - move it in data/features.json")
        elif on & KERB_OBJECTS and name not in geo.get("on_kerb", ()):
            warnings.append(f"barrier {name} stands on a kerb - fine if the tyres really lie there (check the overlay)")
    for o in geo["objects"]:
        if len(o["verts"]) >= 65535:
            errors.append(f"{o['name']}: {len(o['verts'])} vertices, KN5 limit is 65534 per object - split it")
        if o["name"] == "1ROAD" and len(o["faces"]) != len(sm):  # one quad per sample = no holes
            errors.append(f"1ROAD has {len(o['faces'])} quads for {len(sm)} samples - asphalt has holes")
        wall = o["name"].endswith("WALL")  # vertical sides are fine, upside-down not
        round_ = o["name"].startswith("TYRES_")  # tyre bulges face down below their widest point: only degenerate counts
        for f in o["faces"]:
            nx, ny, nz = _normal(o["verts"], f)
            area = math.sqrt(nx * nx + ny * ny + nz * nz)
            if area < 1e-9 or (not round_ and (nz < -0.01 * area if wall else nz <= 0)):
                errors.append(f"{o['name']}: degenerate or flipped face {f}")
                break

    warnings.append(f"corners detected (|k| > {cfg['kerb_min_curvature']}): {geo['corners']} "
                    f"vs official {cfg['turns']} - rough shape check only")
    if cfg.get("placeholder"):
        warnings.append("PLACEHOLDER values in use: " + ", ".join(cfg["placeholder"]))
    return errors, warnings
