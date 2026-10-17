"""Orthophoto reference: 24-bit BMP reader in local metres, pixel classes, asphalt edge fitting. Pure stdlib."""
import json
import struct

ASPHALT, GRASS, OTHER = "asphalt", "grass", "other"


class Ortho:
    """BMP + bbox in local metres (written by `build.py fetch-ortho`)."""

    def __init__(self, bmp_path, meta_path):
        with open(meta_path) as f:
            self.meta = json.load(f)
        with open(bmp_path, "rb") as f:
            self.data = f.read()
        if self.data[:2] != b"BM":
            raise ValueError(f"{bmp_path} is not a BMP")
        self.offset = struct.unpack_from("<I", self.data, 10)[0]
        self.w, h, _, bpp = struct.unpack_from("<iiHH", self.data, 18)
        if bpp not in (24, 32):
            raise ValueError(f"unsupported BMP depth {bpp}")
        self.bottom_up, self.h = h > 0, abs(h)
        self.bpp = bpp // 8
        self.stride = (self.w * self.bpp + 3) & ~3

    def rgb(self, x, y):
        m = self.meta
        px = int((x - m["x0"]) * self.w / (m["x1"] - m["x0"]))
        py = int((m["y1"] - y) * self.h / (m["y1"] - m["y0"]))
        if not (0 <= px < self.w and 0 <= py < self.h):
            return None
        row = self.h - 1 - py if self.bottom_up else py
        b, g, r = self.data[self.offset + row * self.stride + px * self.bpp:][:3]
        return r, g, b

    def classify(self, x, y):
        return classify(self.rgb(x, y))


def classify(rgb):
    """ponytail: fixed RGB thresholds tuned on the summer GUGiK orthophoto; retune if the imagery changes."""
    if rgb is None:
        return OTHER
    r, g, b = rgb
    sat = max(rgb) - min(rgb)
    if sat < 18 and max(rgb) > 45:  # grey asphalt (slightly green-tinted here) and white paint on it
        return ASPHALT
    if g >= max(r, b) + 5:
        return GRASS
    return OTHER


def fit_edges(samples, left, classify_at, max_m=8.0, step=0.1, run_m=0.5):
    """Per sample: (left_dist, right_dist) to the asphalt edge along the normal, or None if not found.

    Edge = start of the first run of >= run_m metres of non-asphalt. None when the ray stays on asphalt up to
    max_m (junctions, paddock access) or the centre itself isn't on asphalt.
    """
    out = []
    for s, (nx, ny) in zip(samples, left):
        if classify_at(s[0], s[1]) != ASPHALT:
            out.append(None)
            continue
        dists = []
        for side in (1, -1):
            run, t, found = 0, 0.0, None
            while t <= max_m:
                if classify_at(s[0] + side * t * nx, s[1] + side * t * ny) == ASPHALT:
                    run = 0
                else:
                    run += 1
                    if run * step >= run_m:
                        found = t - run * step + step
                        break
                t += step
            dists.append(found)
        out.append(None if None in dists else tuple(dists))
    return out
