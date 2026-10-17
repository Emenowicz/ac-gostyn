"""Kartodrom Gostyń -> Assetto Corsa track generator.

  python3 build.py import-osm [--force]   fetch centreline from OpenStreetMap -> data/centerline.csv
  python3 build.py fetch-ortho            download GUGiK orthophoto of the area -> build/cache/ortho.bmp
  python3 build.py fit --force            snap centreline + per-point width to the asphalt on the orthophoto
  python3 build.py fetch-elevation        GUGiK NMT heights -> z in data/centerline.csv + data/elevation.csv
  python3 build.py validate               build geometry, validate, write build/mesh.json + build/preview.svg
  python3 build.py mod                    write mod folder (models.ini, surfaces.ini, ui_track.json)
  python3 build.py fbx                    validate + Blender headless -> build/fbx/<folder>.fbx
  python3 build.py all                    validate + fbx + mod

Blender: $BLENDER, else `blender` on PATH, else the macOS app bundle.
"""
import argparse
import json
import math
import os
import shutil
import statistics
import subprocess
import sys
import urllib.parse
import urllib.request
from datetime import date

import fit
import trackgen

ROOT = os.path.dirname(os.path.abspath(__file__))
TRACK_JSON = os.path.join(ROOT, "data", "track.json")
CENTERLINE = os.path.join(ROOT, "data", "centerline.csv")
BUILD = os.path.join(ROOT, "build")
CACHE = os.path.join(BUILD, "cache")
ORTHO_WMS = "https://mapy.geoportal.gov.pl/wss/service/PZGIK/ORTO/WMS/HighResolution"
NMT = "https://services.gugik.gov.pl/nmt/?request=GetHByPointList&list="
ELEVATION = os.path.join(ROOT, "data", "elevation.csv")
FEATURES = os.path.join(ROOT, "data", "features.json")
EARTH_R = 6371008.8

# OSM ways A, B. A[0]->A[1] is a 27 m chord duplicating B's last 38 m arc, so the loop is A[1:] + B[1:]
# (B ends on A[1]). The two short raceway ways nearby (558923497, 558923506) look like shortcuts - not used.
OSM_WAYS = (378990687, 558923489)

SURFACES = [  # ponytail: PLACEHOLDER physics, tune after the first drive
    dict(KEY="ROAD", FRICTION=0.99, DAMPING=0, DIRT_ADDITIVE=0, IS_VALID_TRACK=1, SIN_HEIGHT=0, SIN_LENGTH=0,
         VIBRATION_GAIN=0, VIBRATION_LENGTH=0),
    dict(KEY="KERB", FRICTION=0.92, DAMPING=0, DIRT_ADDITIVE=0, IS_VALID_TRACK=1, SIN_HEIGHT=0, SIN_LENGTH=0,
         VIBRATION_GAIN=1.0, VIBRATION_LENGTH=0.5),
    dict(KEY="GRASS", FRICTION=0.6, DAMPING=0.1, DIRT_ADDITIVE=1, IS_VALID_TRACK=0, SIN_HEIGHT=0.03, SIN_LENGTH=0.5,
         VIBRATION_GAIN=0.5, VIBRATION_LENGTH=0.3),
    dict(KEY="WALL", FRICTION=0.3, DAMPING=0, DIRT_ADDITIVE=0, IS_VALID_TRACK=0, SIN_HEIGHT=0, SIN_LENGTH=0,
         VIBRATION_GAIN=0, VIBRATION_LENGTH=0),
]


def import_osm(cfg, force):
    if os.path.exists(CENTERLINE) and not force:
        sys.exit(f"{CENTERLINE} exists (may contain manual fixes) - use --force to overwrite")
    nodes, ways = {}, []
    for way_id in OSM_WAYS:
        elements = json.loads(fetch(f"https://api.openstreetmap.org/api/0.6/way/{way_id}/full.json"))["elements"]
        nodes.update({e["id"]: (e["lat"], e["lon"]) for e in elements if e["type"] == "node"})
        ways.append(next(e["nodes"] for e in elements if e["type"] == "way"))
    a, b = ways
    if b[-1] != a[1] or b[0] != a[-1]:
        sys.exit("OSM topology changed - re-check the way assembly in build.py")
    loop = a[1:] + b[1:-1]
    with open(CENTERLINE, "w", encoding="utf-8", newline="") as f:
        f.write(f"# Centreline: OpenStreetMap ways {' + '.join(map(str, OSM_WAYS))}, (c) OpenStreetMap contributors, ODbL\n"
                f"# x east / y north [m], equirectangular around origin_lat_lon from track.json.\n"
                f"# z and width empty = flat / default_width_m (PLACEHOLDER). Edit freely, re-run build.\n"
                "x,y,z,width\n")
        for nid in loop:
            x, y = to_local(cfg, *nodes[nid])
            f.write(f"{x:.2f},{y:.2f},,\n")
    print(f"wrote {CENTERLINE}: {len(loop)} points")


def to_local(cfg, lat, lon):
    """WGS84 -> local metres (equirectangular around origin_lat_lon; exact enough for a few hundred metres)."""
    lat0, lon0 = cfg["origin_lat_lon"]
    return (math.radians(lon - lon0) * EARTH_R * math.cos(math.radians(lat0)), math.radians(lat - lat0) * EARTH_R)


def to_latlon(cfg, x, y):
    lat0, lon0 = cfg["origin_lat_lon"]
    return lat0 + math.degrees(y / EARTH_R), lon0 + math.degrees(x / (EARTH_R * math.cos(math.radians(lat0))))


def to_pl1992(lat, lon):
    """WGS84 -> EPSG:2180 (PUWG 1992) as (x northing, y easting), Krueger series (mm accuracy)."""
    f = 1 / 298.257222101  # GRS80
    n = f / (2 - f)
    A = 6378137.0 / (1 + n) * (1 + n ** 2 / 4 + n ** 4 / 64)
    alpha = (n / 2 - 2 * n ** 2 / 3 + 5 * n ** 3 / 16, 13 * n ** 2 / 48 - 3 * n ** 3 / 5, 61 * n ** 3 / 240)
    e = 2 * math.sqrt(n) / (1 + n)
    phi, lam = math.radians(lat), math.radians(lon - 19.0)
    t = math.sinh(math.atanh(math.sin(phi)) - e * math.atanh(e * math.sin(phi)))
    xi, eta = math.atan2(t, math.cos(lam)), math.atanh(math.sin(lam) / math.sqrt(1 + t * t))
    xs = xi + sum(a * math.sin(2 * j * xi) * math.cosh(2 * j * eta) for j, a in enumerate(alpha, 1))
    es = eta + sum(a * math.cos(2 * j * xi) * math.sinh(2 * j * eta) for j, a in enumerate(alpha, 1))
    return 0.9993 * A * xs - 5300000.0, 0.9993 * A * es + 500000.0


def nmt_heights(cfg, xy, batch=150):
    out = []
    for i in range(0, len(xy), batch):
        pl = [to_pl1992(*to_latlon(cfg, x, y)) for x, y in xy[i:i + batch]]
        body = fetch(NMT + urllib.parse.quote(",".join(f"{a:.2f} {b:.2f}" for a, b in pl))).decode().strip()
        heights = [float(p.split()[2]) for p in body.split(",")]
        if len(heights) != len(pl) or not all(0 < h < 1700 for h in heights):
            sys.exit(f"unexpected NMT reply: {body[:300]}")
        out += heights
        print(f"  NMT {min(i + batch, len(xy))}/{len(xy)}", end="\r")
    print()
    return out


def fetch_elevation(cfg):
    with open(CENTERLINE, encoding="utf-8") as f:
        lines = f.read().splitlines()
    rows = [i for i, line in enumerate(lines) if line and not line.startswith("#") and not line.startswith("x,")]
    xy = [tuple(map(float, lines[i].split(",")[:2])) for i in rows]
    for i, h in zip(rows, nmt_heights(cfg, xy)):
        x, y, _, w = lines[i].split(",")
        lines[i] = f"{x},{y},{h:.2f},{w}"
    lines.insert(next(i for i, line in enumerate(lines) if line.startswith("x,")),
                 "# z = terrain height from GUGiK NMT (m a.s.l., PL-EVRF2007-NH), fetched by `build.py fetch-elevation`.")
    with open(CENTERLINE, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")

    with open(os.path.join(CACHE, "ortho.json")) as f:
        m = json.load(f)  # terrain grid covers the orthophoto extent
    c = cfg["terrain_cell_m"]
    grid = [(m["x0"] + i * c, m["y0"] + j * c) for j in range(int((m["y1"] - m["y0"]) / c) + 1)
            for i in range(int((m["x1"] - m["x0"]) / c) + 1)]
    with open(ELEVATION, "w", encoding="utf-8") as f:
        f.write(f"# Terrain heights, GUGiK NMT (m a.s.l., PL-EVRF2007-NH), {c} m grid, fetched {date.today()}.\nx,y,h\n")
        f.writelines(f"{x:.1f},{y:.1f},{h:.2f}\n" for (x, y), h in zip(grid, nmt_heights(cfg, grid)))
    print(f"z written to {len(rows)} centreline points, {len(grid)} terrain points -> {ELEVATION}")


def fetch(url, timeout=120):
    req = urllib.request.Request(url, headers={"User-Agent": "ac-gostyn-track-generator"})
    with urllib.request.urlopen(req, timeout=timeout) as r:
        return r.read()


def fetch_ortho(cfg):
    _, pts = trackgen.load(TRACK_JSON, CENTERLINE)
    feats = trackgen.load_features(FEATURES) or {}
    xy = [p[:2] for p in pts] + [q for k in ("barriers", "extra_asphalt") for b in feats.get(k, []) for q in b["points"]]
    m, ppm = cfg["ortho_margin_m"], cfg["ortho_px_per_m"]  # the photo is also the terrain's extent and texture
    x0, x1 = math.floor(min(p[0] for p in xy) - m), math.ceil(max(p[0] for p in xy) + m)
    y0, y1 = math.floor(min(p[1] for p in xy) - m), math.ceil(max(p[1] for p in xy) + m)
    w, h = round((x1 - x0) * ppm), round((y1 - y0) * ppm)
    if max(w, h) > 4096:
        sys.exit(f"{w}x{h} px exceeds the WMS limit of 4096 - lower ortho_px_per_m or ortho_margin_m")
    (la0, lo0), (la1, lo1) = to_latlon(cfg, x0, y0), to_latlon(cfg, x1, y1)
    params = dict(SERVICE="WMS", REQUEST="GetMap", VERSION="1.3.0", LAYERS="Raster", STYLES="", CRS="EPSG:4326",
                  BBOX=f"{la0:.8f},{lo0:.8f},{la1:.8f},{lo1:.8f}", WIDTH=w, HEIGHT=h, FORMAT="image/bmp")
    data = fetch(ORTHO_WMS + "?" + urllib.parse.urlencode(params))
    if not data.startswith(b"BM"):
        sys.exit("WMS did not return a BMP: " + data[:300].decode("utf-8", "replace"))
    os.makedirs(CACHE, exist_ok=True)
    with open(os.path.join(CACHE, "ortho.bmp"), "wb") as f:
        f.write(data)
    with open(os.path.join(CACHE, "ortho.json"), "w") as f:
        json.dump({"x0": x0, "y0": y0, "x1": x1, "y1": y1, "px_per_m": ppm, "fetched": date.today().isoformat(),
                   "source": "Geoportal.gov.pl / GUGiK, ortofotomapa (WMS HighResolution)"}, f, indent=1)
    print(f"ortho {w}x{h} px, {x1 - x0}x{y1 - y0} m -> build/cache/ortho.bmp")


def _circular_fill(values):
    """Replace None by linear interpolation between the nearest valid neighbours (closed loop)."""
    n, good = len(values), [i for i, v in enumerate(values) if v is not None]
    out = list(values)
    for k, i in enumerate(good):
        j = good[(k + 1) % len(good)]
        gap = (j - i) % n or n
        for step in range(1, gap):
            out[(i + step) % n] = values[i] + (values[j] - values[i]) * step / gap
    return out


def fit_centreline(cfg, force):
    if not force:
        sys.exit("fit rewrites data/centerline.csv - commit it first, then run with --force")
    ortho = fit.Ortho(os.path.join(CACHE, "ortho.bmp"), os.path.join(CACHE, "ortho.json"))
    _, pts = trackgen.load(TRACK_JSON, CENTERLINE)
    samples, length0 = trackgen.resample(pts, cfg["fit_step_m"])
    n, r = len(samples), 3
    for _ in range(2):  # re-centre, then measure again from the new centre
        ds = sum(math.dist(a[:2], b[:2]) for a, b in zip(samples, samples[1:] + samples[:1])) / n
        _, left, _ = trackgen.frames(samples, ds)
        edges = fit.fit_edges(samples, left, ortho.classify)
        bad = [e is None or not cfg["width_sanity_m"][0] <= sum(e) <= cfg["width_sanity_m"][1] for e in edges]
        if sum(bad) > n / 3:
            sys.exit(f"{sum(bad)}/{n} samples without a clear asphalt edge - is the orthophoto aligned?")
        dl, dr = (_circular_fill([None if b else e[k] for e, b in zip(edges, bad)]) for k in (0, 1))
        dl, dr = ([statistics.median(v[(i + j) % n] for j in range(-r, r + 1)) for i in range(n)] for v in (dl, dr))
        samples = [[s[0] + (a - b) / 2 * nx, s[1] + (a - b) / 2 * ny, s[2], a + b]
                   for s, (nx, ny), a, b in zip(samples, left, dl, dr)]
    widths = [s[3] for s in samples]
    length = sum(math.dist(a[:2], b[:2]) for a, b in zip(samples, samples[1:] + samples[:1]))
    print(f"length {length0:.1f} -> {length:.1f} m; width min {min(widths):.1f} / median {statistics.median(widths):.1f}"
          f" / max {max(widths):.1f} m; {sum(bad)}/{n} samples interpolated (no clear edge)")
    runs, start = [], None
    for i, b in enumerate(bad + [False]):
        if b and start is None:
            start = i
        elif not b and start is not None:
            runs.append((start, i))
            start = None
    for a, b in runs:
        x, y = samples[(a + b) // 2][:2]
        print(f"  interpolated {(b - a) * ds:.0f} m around ({x:.0f}, {y:.0f})")
    if cfg.get("reverse"):
        samples.reverse()  # the CSV keeps its original order; track.json's reverse flag still applies
    with open(CENTERLINE, "w", encoding="utf-8", newline="") as f:
        f.write(f"# Centreline + width fitted to the GUGiK orthophoto ({ortho.meta['fetched']}) by `build.py fit`,\n"
                f"# starting from OpenStreetMap ways {' + '.join(map(str, OSM_WAYS))}, (c) OpenStreetMap contributors, ODbL.\n"
                "# x east / y north [m] around origin_lat_lon from track.json; width = asphalt edge to edge [m].\n"
                "x,y,z,width\n")
        for x, y, z, w in samples:
            f.write(f"{x:.2f},{y:.2f},{'' if not z else f'{z:.2f}'},{w:.2f}\n")
    print(f"wrote {CENTERLINE}: {n} points")


def generate(cfg):
    _, pts = trackgen.load(TRACK_JSON, CENTERLINE)
    photo = None
    if os.path.exists(os.path.join(CACHE, "ortho.json")):
        with open(os.path.join(CACHE, "ortho.json")) as f:
            photo = json.load(f)
    geo = trackgen.build(cfg, pts, trackgen.load_ground(ELEVATION), trackgen.load_features(FEATURES), photo)
    errors, warnings = trackgen.validate(cfg, pts, geo)
    for w in warnings:
        print("WARN ", w)
    for e in errors:
        print("ERROR", e)
    os.makedirs(BUILD, exist_ok=True)
    with open(os.path.join(BUILD, "mesh.json"), "w") as f:
        json.dump({"objects": geo["objects"], "empties": geo["empties"],
                   "photo": os.path.join(CACHE, "ortho.bmp") if photo else None}, f)
    write_preview(geo, os.path.join(BUILD, "preview.svg"))
    if os.path.exists(os.path.join(CACHE, "ortho.json")):
        write_ortho_overlay(geo, pts, os.path.join(BUILD, "ortho_overlay.svg"))
    zs = [p[2] for p in geo["samples"]]
    print(f"elevation {min(zs):.2f}-{max(zs):.2f} m a.s.l., length {geo['length']:.1f} m, {len(geo['samples'])} samples, "
          f"{sum(len(o['faces']) for o in geo['objects'])} faces -> build/mesh.json, build/preview.svg")
    if errors:
        sys.exit(f"{len(errors)} validation error(s)")
    return geo


def write_preview(geo, path):
    colors = {"road": "#333", "kerb": "#d22", "grass": "#5a3", "wall": "#eee"}
    polys = []
    for o in geo["objects"]:
        if o["material"] in colors:
            for f in o["faces"]:
                pts = " ".join(f"{o['verts'][i][0]:.1f},{-o['verts'][i][1]:.1f}" for i in f)
                polys.append(f'<polygon points="{pts}" fill="{colors[o["material"]]}"/>')
    for b in geo["barriers"].values():
        line = " ".join(f"{p[0]:.1f},{-p[1]:.1f}" for p in b)
        polys.append(f'<polyline points="{line}" fill="none" stroke="#000" stroke-width="0.6"/>')
    for e in geo["empties"]:
        x, y, _ = e["pos"]
        color = "#00f" if e["name"].startswith("AC_TIME") else "#fc0"
        polys.append(f'<circle cx="{x:.1f}" cy="{-y:.1f}" r="0.8" fill="{color}"><title>{e["name"]}</title></circle>')
        polys.append(f'<line x1="{x:.1f}" y1="{-y:.1f}" x2="{x + 3 * e["fwd"][0]:.1f}" y2="{-y - 3 * e["fwd"][1]:.1f}" '
                     f'stroke="{color}" stroke-width="0.4"/>')
    xs = [p[0] for p in geo["polylines"]["edge_l"] + geo["polylines"]["edge_r"]]
    ys = [-p[1] for p in geo["polylines"]["edge_l"] + geo["polylines"]["edge_r"]]
    w, h = max(xs) - min(xs) + 10, max(ys) - min(ys) + 10
    with open(path, "w") as f:
        f.write(f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="{min(xs) - 5:.0f} {min(ys) - 5:.0f} {w:.0f} {h:.0f}" '
                f'width="{1000 * w / max(w, h):.0f}" height="{1000 * h / max(w, h):.0f}" '
                f'style="background:#7a5">{"".join(polys)}</svg>')


def write_ortho_overlay(geo, pts, path):
    """Orthophoto + generated geometry, for checking the model against reality in a browser."""
    with open(os.path.join(CACHE, "ortho.json")) as f:
        m = json.load(f)
    k = m["px_per_m"]

    def line(points, color, width=2.0):
        p = " ".join(f"{(x - m['x0']) * k:.0f},{(m['y1'] - y) * k:.0f}" for x, y, *_ in points)
        return f'<polyline points="{p}" fill="none" stroke="{color}" stroke-width="{width}"/>'

    el = [line(geo["polylines"][key] , "#ff0") for key in ("road_l", "road_r")]
    el += [line(geo["polylines"][key], "#f0f", 1.5) for key in ("edge_l", "edge_r")]
    el += [line(b, "#0ff", 3) for b in geo["barriers"].values()]
    for o in geo["objects"]:
        if o["name"] in ("1KERB", "2ROAD"):
            color = "#f80" if o["name"] == "1KERB" else "#ff0"
            el += [line([o["verts"][i] for i in f + f[:1]], color, 1.5) for f in o["faces"]]
    el += [f'<circle cx="{(x - m["x0"]) * k:.0f}" cy="{(m["y1"] - y) * k:.0f}" r="3" fill="#0ff"/>' for x, y, *_ in pts]
    for e in geo["empties"]:
        x, y, _ = e["pos"]
        el.append(f'<circle cx="{(x - m["x0"]) * k:.0f}" cy="{(m["y1"] - y) * k:.0f}" r="6" '
                  f'fill="{"#00f" if e["name"].startswith("AC_TIME") else "#fc0"}"><title>{e["name"]}</title></circle>')
    w, h = round((m["x1"] - m["x0"]) * k), round((m["y1"] - m["y0"]) * k)
    with open(path, "w") as f:
        f.write(f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {w} {h}" width="{w}" height="{h}">'
                f'<image href="cache/ortho.bmp" width="{w}" height="{h}"/>{"".join(el)}</svg>')


def write_mod(cfg, geo):
    track = os.path.join(BUILD, "content", "tracks", cfg["folder"])
    os.makedirs(os.path.join(track, "data"), exist_ok=True)
    os.makedirs(os.path.join(track, "ui"), exist_ok=True)
    with open(os.path.join(track, "models.ini"), "w") as f:
        f.write(f"[MODEL_0]\nFILE={cfg['folder']}.kn5\nPOSITION=0,0,0\nROTATION=0,0,0\n")
    with open(os.path.join(track, "data", "surfaces.ini"), "w") as f:
        f.write("; PLACEHOLDER physics values - tune after test drives\n")
        for i, s in enumerate(SURFACES):
            s = dict(s, WAV="", WAV_PITCH=0, FF_EFFECT="NULL", BLACK_FLAG_TIME=0, IS_PITLANE=0)
            f.write(f"\n[SURFACE_{i}]\n" + "".join(f"{k}={v}\n" for k, v in s.items()))
    sm = geo["samples"]
    area = sum(a[0] * b[1] - b[0] * a[1] for a, b in zip(sm, sm[1:] + sm[:1]))  # >0 = counter-clockwise
    lat, lon = cfg["origin_lat_lon"]
    ui = {
        "name": cfg["name"],
        "description": f"Kartodrom Gostyń (built {cfg['year']}). Work in progress - geometry from OpenStreetMap, "
                       "placeholder widths/elevation. Map data (c) OpenStreetMap contributors, ODbL.",
        "tags": ["karting", "circuit", "poland", "wip"],
        "geotags": [f"{lat:.5f}", f"{lon:.5f}"],
        "country": cfg["country"], "city": cfg["city"],
        "length": f"{geo['length']:.0f}", "width": f"{cfg['width_min_m']:.0f}-{cfg['width_max_m']:.0f}",
        "pitboxes": str(cfg["pit_slots"]),
        "run": "counter-clockwise" if area > 0 else "clockwise",
        "url": cfg["url"], "year": cfg["year"], "version": "0.1",
    }
    with open(os.path.join(track, "ui", "ui_track.json"), "w", encoding="utf-8") as f:
        json.dump(ui, f, ensure_ascii=False, indent=2)
    print(f"mod -> {os.path.relpath(track, ROOT)}  (add {cfg['folder']}.kn5 from ksEditor)")


def blender_exe():
    for exe in (os.environ.get("BLENDER"), shutil.which("blender"), "/Applications/Blender.app/Contents/MacOS/Blender"):
        if exe and os.path.exists(exe):
            return exe
    sys.exit("Blender not found - set BLENDER=/path/to/blender")


def export_fbx(cfg):
    out = os.path.join(BUILD, "fbx", cfg["folder"] + ".fbx")
    os.makedirs(os.path.dirname(out), exist_ok=True)
    cmd = [blender_exe(), "-b", "--factory-startup", "-P", os.path.join(ROOT, "blender_build.py"), "--",
           os.path.join(BUILD, "mesh.json"), out]
    subprocess.run(cmd, check=True)
    print(f"fbx -> {os.path.relpath(out, ROOT)}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("command", choices=["import-osm", "fetch-ortho", "fit", "fetch-elevation", "validate", "mod", "fbx", "all"])
    ap.add_argument("--force", action="store_true", help="import-osm / fit: overwrite data/centerline.csv")
    args = ap.parse_args()
    with open(TRACK_JSON, encoding="utf-8") as f:
        cfg = json.load(f)
    if args.command == "import-osm":
        return import_osm(cfg, args.force)
    if args.command == "fetch-ortho":
        return fetch_ortho(cfg)
    if args.command == "fit":
        return fit_centreline(cfg, args.force)
    if args.command == "fetch-elevation":
        return fetch_elevation(cfg)
    geo = generate(cfg)
    if args.command in ("fbx", "all"):
        export_fbx(cfg)
    if args.command in ("mod", "all"):
        write_mod(cfg, geo)


if __name__ == "__main__":
    main()
