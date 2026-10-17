"""Kartodrom Gostyń -> Assetto Corsa track generator.

  python3 build.py import-osm [--force]   fetch centreline from OpenStreetMap -> data/centerline.csv
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
import subprocess
import sys
import urllib.request

import trackgen

ROOT = os.path.dirname(os.path.abspath(__file__))
TRACK_JSON = os.path.join(ROOT, "data", "track.json")
CENTERLINE = os.path.join(ROOT, "data", "centerline.csv")
BUILD = os.path.join(ROOT, "build")

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
        req = urllib.request.Request(f"https://api.openstreetmap.org/api/0.6/way/{way_id}/full.json",
                                     headers={"User-Agent": "ac-gostyn-track-generator"})
        with urllib.request.urlopen(req, timeout=60) as r:
            elements = json.load(r)["elements"]
        nodes.update({e["id"]: (e["lat"], e["lon"]) for e in elements if e["type"] == "node"})
        ways.append(next(e["nodes"] for e in elements if e["type"] == "way"))
    a, b = ways
    if b[-1] != a[1] or b[0] != a[-1]:
        sys.exit("OSM topology changed - re-check the way assembly in build.py")
    loop = a[1:] + b[1:-1]
    lat0, lon0 = cfg["origin_lat_lon"]
    R = 6371008.8
    with open(CENTERLINE, "w", encoding="utf-8", newline="") as f:
        f.write(f"# Centreline: OpenStreetMap ways {' + '.join(map(str, OSM_WAYS))}, (c) OpenStreetMap contributors, ODbL\n"
                f"# x east / y north [m], equirectangular around origin_lat_lon from track.json.\n"
                f"# z and width empty = flat / default_width_m (PLACEHOLDER). Edit freely, re-run build.\n"
                "x,y,z,width\n")
        for nid in loop:
            lat, lon = nodes[nid]
            x = math.radians(lon - lon0) * R * math.cos(math.radians(lat0))
            y = math.radians(lat - lat0) * R
            f.write(f"{x:.2f},{y:.2f},,\n")
    print(f"wrote {CENTERLINE}: {len(loop)} points")


def generate(cfg):
    _, pts = trackgen.load(TRACK_JSON, CENTERLINE)
    geo = trackgen.build(cfg, pts)
    errors, warnings = trackgen.validate(cfg, pts, geo)
    for w in warnings:
        print("WARN ", w)
    for e in errors:
        print("ERROR", e)
    os.makedirs(BUILD, exist_ok=True)
    with open(os.path.join(BUILD, "mesh.json"), "w") as f:
        json.dump({"objects": geo["objects"], "empties": geo["empties"]}, f)
    write_preview(geo, os.path.join(BUILD, "preview.svg"))
    print(f"length {geo['length']:.1f} m, {len(geo['samples'])} samples, "
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
    for p in ("wall_l", "wall_r"):
        line = " ".join(f"{x:.1f},{-y:.1f}" for x, y, _ in geo["polylines"][p])
        polys.append(f'<polyline points="{line}" fill="none" stroke="#000" stroke-width="0.4"/>')
    for e in geo["empties"]:
        x, y, _ = e["pos"]
        color = "#00f" if e["name"].startswith("AC_TIME") else "#fc0"
        polys.append(f'<circle cx="{x:.1f}" cy="{-y:.1f}" r="0.8" fill="{color}"><title>{e["name"]}</title></circle>')
        polys.append(f'<line x1="{x:.1f}" y1="{-y:.1f}" x2="{x + 3 * e["fwd"][0]:.1f}" y2="{-y - 3 * e["fwd"][1]:.1f}" '
                     f'stroke="{color}" stroke-width="0.4"/>')
    xs = [p[0] for p in geo["polylines"]["wall_l"] + geo["polylines"]["wall_r"]]
    ys = [-p[1] for p in geo["polylines"]["wall_l"] + geo["polylines"]["wall_r"]]
    w, h = max(xs) - min(xs) + 10, max(ys) - min(ys) + 10
    with open(path, "w") as f:
        f.write(f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="{min(xs) - 5:.0f} {min(ys) - 5:.0f} {w:.0f} {h:.0f}" '
                f'width="{1000 * w / max(w, h):.0f}" height="{1000 * h / max(w, h):.0f}" '
                f'style="background:#7a5">{"".join(polys)}</svg>')


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
    ap.add_argument("command", choices=["import-osm", "validate", "mod", "fbx", "all"])
    ap.add_argument("--force", action="store_true", help="import-osm: overwrite data/centerline.csv")
    args = ap.parse_args()
    with open(TRACK_JSON, encoding="utf-8") as f:
        cfg = json.load(f)
    if args.command == "import-osm":
        return import_osm(cfg, args.force)
    geo = generate(cfg)
    if args.command in ("fbx", "all"):
        export_fbx(cfg)
    if args.command in ("mod", "all"):
        write_mod(cfg, geo)


if __name__ == "__main__":
    main()
