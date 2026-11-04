"""Procedural, tileable surface textures with normal maps (numpy only - runs in Blender's bundled Python).

Every texture is built from white noise filtered in the frequency domain, so it wraps at its edges and tiles
without seams; fixed seeds make every build write the same images. Arrays are row 0 = bottom (Blender's pixel
order), x = texture u, y = texture v. Each maker returns (albedo (h, w, 3), height (h, w)); normal_map() turns the
height into a tangent-space normal map (OpenGL convention: green = +v).

Texel sizes match the UVs written by trackgen (UV_M = 4 m per tile, ROAD_UV_M = 8 m for asphalt): asphalt 2048 px
per 8 m and concrete 1024 px per 4 m (4 mm/px), kerbs 256 px across the kerb (u 0..1) x 1024 px per 4 m along it.
"""
import numpy as np

RED, YELLOW, WHITE = (0.72, 0.07, 0.06), (0.93, 0.72, 0.08), (0.92, 0.92, 0.90)
ASPHALT, CONCRETE = (0.40, 0.41, 0.41), (0.62, 0.61, 0.58)
RUBBER, DIRT = (0.08, 0.08, 0.08), (0.36, 0.31, 0.22)


def noise(shape, sigma_px, seed):
    """Tileable Gaussian-filtered noise, zero mean, unit variance; sigma_px = feature size in pixels."""
    white = np.random.default_rng(seed).standard_normal(shape)
    fy, fx = np.fft.fftfreq(shape[0])[:, None], np.fft.fftfreq(shape[1])[None, :]
    out = np.real(np.fft.ifft2(np.fft.fft2(white) * np.exp(-2 * (np.pi * sigma_px) ** 2 * (fx ** 2 + fy ** 2))))
    return (out - out.mean()) / (out.std() + 1e-12)


def smoothstep(e0, e1, x):
    t = np.clip((x - e0) / (e1 - e0), 0.0, 1.0)
    return t * t * (3 - 2 * t)


def mix(a, b, t):
    """Blend colour arrays/tuples a -> b by t (h, w)."""
    return np.asarray(a) * (1 - t[..., None]) + np.asarray(b) * t[..., None]


def normal_map(height, strength):
    """Tangent-space normal map (0..1 RGB) from a height field; wraps at the edges like the textures."""
    dx = (np.roll(height, -1, 1) - np.roll(height, 1, 1)) / 2
    dy = (np.roll(height, -1, 0) - np.roll(height, 1, 0)) / 2
    n = np.dstack((-dx * strength, -dy * strength, np.ones_like(height)))
    return n / np.linalg.norm(n, axis=2, keepdims=True) * 0.5 + 0.5


def asphalt(size=2048, seed=10):
    """Asphalt concrete, 8 m tile (ROAD_UV_M) at 4 mm/px: fine grain, light and dark aggregate, a faint mottle.
    Nothing bigger than ~15 cm and nothing that stands out: across 800 m of track a 0.5 m blotch or a repair patch
    in the tile shows up as a regular grid; uniform fine texture doesn't."""
    s = (size, size)
    grain, stones, mottle = (noise(s, sig, seed + k) for k, sig in enumerate((0.8, 1.6, 35)))
    light, dark = smoothstep(1.3, 2.2, stones), smoothstep(1.5, 2.5, -stones)
    v = 0.035 * grain + 0.10 * light - 0.08 * dark + 0.008 * mottle
    albedo = np.clip(np.asarray(ASPHALT) + v[..., None], 0, 1)
    return albedo, 0.5 * grain + 1.5 * light - 0.8 * dark


def concrete(size=(1024, 1024), base=CONCRETE, seed=20):
    """Cast concrete: fine grain, small dark pores, broad tonal variation and faint stains. size = (h, w)."""
    grain, pores, mottle, stain = (noise(size, sig, seed + k) for k, sig in enumerate((0.8, 1.0, 50, 180)))
    pore = smoothstep(2.2, 3.0, -pores)
    v = 0.03 * grain - 0.12 * pore + 0.035 * mottle
    albedo = mix(np.clip(np.asarray(base) + v[..., None], 0, 1), DIRT, 0.25 * smoothstep(0.8, 1.6, stain))
    return albedo, 0.4 * grain - 1.2 * pore


def _kerb_wear(albedo, h, w, seed, rubber=0.55):
    """Shared kerb ageing: worn paint, tyre rubber on the asphalt side, dirt thrown up on the outer edge."""
    u = (np.arange(w)[None, :] + 0.5) / w * np.ones((h, 1))
    wear = smoothstep(0.9, 1.9, noise((h, w), 5, seed))
    smudge = smoothstep(-0.3, 1.2, noise((h, w), 22, seed + 1)) * smoothstep(0.55, 0.05, u)
    dirt = smoothstep(0.78, 1.0, u) * smoothstep(-0.8, 0.8, noise((h, w), 10, seed + 2))
    albedo = mix(albedo, (0.55, 0.53, 0.50), 0.55 * wear)  # paint worn down to the concrete
    albedo = mix(albedo, RUBBER, rubber * smudge)
    return mix(albedo, DIRT, 0.6 * dirt), wear


def kerb_stripes(w=256, h=1024, stripes=8, seed=30, lip=0.84):
    """Red/yellow kerb, `stripes` stripes per 4 m tile along v (0.5 m), worn, rubbered, dirty; the raised outer
    edge (u > lip, the peak of the profile by the grass) is one continuous red band (photos of the track)."""
    y = np.arange(h)[:, None] * np.ones((1, w))
    u = (np.arange(w)[None, :] + 0.5) / w * np.ones((h, 1))
    stripe = ((y * stripes // h) % 2).astype(float) * (u < lip)
    grain = noise((h, w), 1.0, seed + 5)
    albedo = mix(np.full((h, w, 3), RED), YELLOW, stripe) * (1 + 0.05 * grain[..., None])
    albedo, wear = _kerb_wear(np.clip(albedo, 0, 1), h, w, seed)
    return np.clip(albedo, 0, 1), 0.5 * grain - 1.0 * wear


APRON_RED, APRON_BLUE = (0.74, 0.27, 0.24), (0.38, 0.48, 0.74)  # faded paint (orthophoto, ground photos 2025)


def kerb_apron(w=256, h=1024, red_end=0.46, blue_end=0.90, seed=40):
    """Painted concrete pour, across from the asphalt: a red band, a blue band, a thin white border by the grass
    (orthophoto 2026-04 and ground photos 2025-10 of all three red pours); concrete texture, rubber and dirt."""
    base, height = concrete((h, w), CONCRETE, seed)
    u = (np.arange(w)[None, :] + 0.5) / w * np.ones((h, 1))
    tone = base / np.asarray(CONCRETE)  # keep the concrete's light/dark variation, recoloured
    paint = np.where((u < red_end)[..., None], APRON_RED, np.where((u < blue_end)[..., None], APRON_BLUE, WHITE))
    albedo = np.clip(tone * paint, 0, 1)
    albedo, _ = _kerb_wear(albedo, h, w, seed + 3, rubber=0.4)
    return np.clip(albedo, 0, 1), height


def kerb_concrete(w=256, h=1024, seed=50):
    """Grey concrete pour beside the asphalt, same texel density as the kerbs."""
    albedo, height = concrete((h, w), CONCRETE, seed)
    albedo, _ = _kerb_wear(albedo, h, w, seed + 3, rubber=0.35)
    return np.clip(albedo, 0, 1), height


def paint_white(size=256, seed=60, colour=WHITE):
    """Painted line: off-white (or `colour`) with fine grain and a little grime."""
    grain, grime = noise((size, size), 1.0, seed), smoothstep(0.8, 1.8, noise((size, size), 12, seed + 1))
    albedo = mix(np.clip(np.asarray(colour) + 0.03 * grain[..., None], 0, 1), (0.55, 0.55, 0.52), 0.35 * grime)
    return albedo, 0.3 * grain


def chequer(w=128, h=512, rows=3, squares=16, seed=90):
    """Chequered start/finish band: `rows` squares across the band (u), `squares` per 4 m tile along the line (v)."""
    u = (np.arange(w)[None, :] + 0.5) / w * np.ones((h, 1))
    v = (np.arange(h)[:, None] + 0.5) / h * np.ones((1, w))
    black = ((np.floor(u * rows) + np.floor(v * squares)) % 2).astype(float)
    grain, worn = noise((h, w), 1.0, seed), smoothstep(1.0, 2.0, noise((h, w), 6, seed + 1))
    albedo = mix(np.full((h, w, 3), WHITE), (0.07, 0.07, 0.07), black * (1 - 0.6 * worn))
    albedo = mix(albedo, ASPHALT, 0.5 * worn * (1 - black))  # worn white shows the asphalt
    return np.clip(albedo * (1 + 0.04 * grain[..., None]), 0, 1), 0.3 * grain


def tyre(colour, size=256, seed=70):
    """Painted tyre stack (u around, v up): paint worn through to black rubber, mud near the ground, fine grain."""
    s = (size, size)
    v = (np.arange(size)[:, None] + 0.5) / size * np.ones((1, size))
    grain, worn, mud = noise(s, 1.2, seed), noise(s, 6, seed + 1), noise(s, 14, seed + 2)
    albedo = np.clip(np.asarray(colour) * (0.9 + 0.08 * grain[..., None]), 0, 1)
    albedo = mix(albedo, RUBBER, 0.85 * smoothstep(1.0, 1.8, worn))
    albedo = mix(albedo, DIRT, 0.7 * smoothstep(0.35, 0.0, v) * smoothstep(-1.0, 0.6, mud))
    return np.clip(albedo, 0, 1), 0.6 * grain - 0.8 * smoothstep(1.0, 1.8, worn)


def noise_dir(shape, sig_along, sig_across, angle, seed):
    """Tileable noise stretched along `angle` (radians from +x): streaks sig_along long, sig_across wide (pixels)."""
    white = np.random.default_rng(seed).standard_normal(shape)
    fy, fx = np.fft.fftfreq(shape[0])[:, None], np.fft.fftfreq(shape[1])[None, :]
    fa, fc = fx * np.cos(angle) + fy * np.sin(angle), -fx * np.sin(angle) + fy * np.cos(angle)
    out = np.real(np.fft.ifft2(np.fft.fft2(white) * np.exp(-2 * np.pi ** 2 * ((sig_along * fa) ** 2 + (sig_across * fc) ** 2))))
    return (out - out.mean()) / (out.std() + 1e-12)


# Detail textures for the orthophoto (ksMultilayer in AC): grey-ish multipliers around DETAIL_MEAN, tiled every
# tile_m metres over the photo, chosen per pixel by the photo's mask (R = grass, G = bare soil). The photo keeps the
# colour; the detail only adds what 5 cm pixels can't show close up (blades, clods, pebbles).
DETAIL_MEAN = 0.8


def grass_detail(size=512, seed=100):
    """Short grass seen from a kart: blades in all directions, darker gaps, light dry tips (1.5 m tile, 3 mm/px)."""
    s = (size, size)
    blades = sum(noise_dir(s, 9, 0.9, a, seed + k) for k, a in enumerate((0.3, 1.2, 2.0, 2.7))) / 2
    clumps, gaps = noise(s, 18, seed + 10), noise(s, 3, seed + 11)
    tips = smoothstep(1.2, 2.2, blades)
    v = 0.18 * blades + 0.06 * clumps - 0.12 * smoothstep(1.0, 2.0, -gaps)
    tint = mix(np.full(s + (3,), (0.97, 1.03, 0.96)), (1.08, 1.04, 0.86), 0.6 * tips)  # dry tips a bit yellow
    albedo = np.clip(DETAIL_MEAN * (1 + v)[..., None] * tint, 0, 1)
    return albedo, blades + 0.5 * clumps - 1.0 * smoothstep(1.0, 2.0, -gaps)


def soil_detail(size=512, seed=110):
    """Bare dry soil / worn verge: clods, small pebbles, fine sand, shallow ruts (2 m tile, 4 mm/px)."""
    s = (size, size)
    grain, clods, pebbles, ruts = noise(s, 0.9, seed), noise(s, 7, seed + 1), noise(s, 1.6, seed + 2), \
        noise_dir(s, 90, 6, 0.4, seed + 3)
    stone = smoothstep(1.8, 2.6, pebbles)
    v = 0.06 * grain + 0.10 * clods + 0.22 * stone - 0.08 * smoothstep(0.8, 1.8, -ruts)
    tint = mix(np.full(s + (3,), (1.02, 1.0, 0.97)), (0.96, 0.97, 1.03), stone)  # pebbles a little greyer
    albedo = np.clip(DETAIL_MEAN * (1 + v)[..., None] * tint, 0, 1)
    return albedo, 0.4 * grain + 0.8 * clods + 1.5 * stone - 0.6 * smoothstep(0.8, 1.8, -ruts)


# name -> (maker, normal-map strength, tile in metres)
DETAILS = {"grass_detail": (grass_detail, 3.0, 1.5), "soil_detail": (soil_detail, 3.0, 2.0)}


def _box_blur(a, r):
    """Box blur of radius r pixels along both axes (cumulative sums, edges clamped)."""
    for axis in (0, 1):
        pad = [(0, 0)] * a.ndim
        pad[axis] = (r + 1, r)
        c = np.cumsum(np.pad(a, pad, mode="edge"), axis=axis)
        a = (np.take(c, range(2 * r + 1, c.shape[axis]), axis) - np.take(c, range(0, c.shape[axis] - 2 * r - 1), axis)) / (2 * r + 1)
    return a


def ortho_mask(rgb, blur_px=2):
    """Detail mask for an orthophoto (h, w, 3, 0..1): R = grass (clearly green), G = no grass (bare soil, sand, dry
    worn verge, gravel - also asphalt, hidden under the track anyway), B = A = 0 (unused). Soft edges."""
    greenness = rgb[..., 1] - np.maximum(rgb[..., 0], rgb[..., 2])
    grass = smoothstep(0.004, 0.035, greenness)
    for _ in range(2):  # two box passes ~ a Gaussian
        grass = _box_blur(grass, blur_px)
    z = np.zeros_like(grass)
    return np.dstack((grass, 1 - grass, z, z))


def asphalt_macro(rgb, ref_px=None, radius_px=4, clip=(0.9, 1.2)):
    """Brightness map of the asphalt on an orthophoto (h, w, 3, 0..1) -> (h, w) multiplier around 1: the rubbered
    racing line, darker and lighter sections (clipped at -10 %: deeper is tree shadow, which AC draws itself).
    Only plain asphalt counts (grey, neither white paint - nor the 2 px of blur around
    it - nor dark tyres or shadows, no grass or kerb colour); the gaps are filled from the asphalt
    around (normalised box blur, which also smooths away the pixel noise). 1 = the median of the pixels ref_px
    ((rows, cols): the track's own asphalt; default all of it), clipped."""
    lum = rgb.mean(axis=2)
    sat = rgb.max(axis=2) - rgb.min(axis=2)
    near_paint = _box_blur((lum > 0.62).astype(float), 2) > 0
    valid = ((sat < 0.08) & (lum > 0.12) & (lum < 0.62) & ~near_paint).astype(float)
    num, den = lum * valid, valid
    for _ in range(2):
        num, den = _box_blur(num, radius_px), _box_blur(den, radius_px)
    ratio = np.where(den > 0.05, num / np.maximum(den, 1e-6), np.nan)
    ref = ratio[ref_px] if ref_px is not None else ratio
    ref = np.nanmedian(ref) if np.isfinite(ref).any() else 1.0
    return np.clip(np.nan_to_num(ratio / ref, nan=1.0), *clip)


TREE_KINDS = [  # (leaf colour, crown centre height, crown half-width, half-height) as fractions of the sprite
    ((0.20, 0.31, 0.10), 0.62, 0.40, 0.36),  # broadleaf, round
    ((0.25, 0.34, 0.12), 0.66, 0.36, 0.32),  # lighter, higher crown
    ((0.17, 0.27, 0.11), 0.58, 0.44, 0.40),  # dark, wide
    ((0.11, 0.21, 0.12), None, 0.36, None),  # conifer: a cone from 12 % up to the top
]


def tree_atlas(w=512, h=1024, seed=120):
    """Tree sprites side by side (trackgen.TREE_VARIANTS = len(TREE_KINDS)), RGBA, row 0 = bottom, alpha 0 / 1 for
    the alpha test: a trunk, and a crown made of leaf clusters (random discs inside the crown's outline) with a ragged
    edge, lit from above and darker inside. Transparent pixels carry the crown colour, so mipmaps don't fringe dark."""
    rng = np.random.default_rng(seed)
    out = []
    for kind, (leaf, cy, rx, ry) in enumerate(TREE_KINDS):
        y, x = np.mgrid[0:h, 0:w]
        u, v = (x + 0.5) / w, (y + 0.5) / h * (h / w)  # v in sprite widths, so discs stay round
        top = h / w
        if cy is None:  # cone
            frac = np.clip((v / top - 0.12) / 0.88, 0, 1)
            outline = (np.abs(u - 0.5) < rx * (1 - frac) + 0.03) & (v / top > 0.12)
        else:
            outline = ((u - 0.5) / rx) ** 2 + ((v / top - cy) * top / (ry * top)) ** 2 < 1
        crown = np.zeros((h, w), bool)
        shade = np.zeros((h, w))
        conifer = cy is None
        # cluster centres well inside the outline (eroded by the cluster size), so the silhouette stays compact
        count, (rmin, rmax) = (420, (0.025, 0.05)) if conifer else (320, (0.035, 0.07))
        inner = outline.copy()
        k = int(rmax * w * 0.7)
        for di, dj in ((k, 0), (-k, 0), (0, k), (0, -k)):
            inner &= np.roll(np.roll(outline, di, 0), dj, 1)
        cand = np.argwhere(inner if inner.any() else outline)
        edge_noise = noise((h, w), 1.2, seed + kind)
        for _ in range(count):
            i, j = cand[rng.integers(len(cand))]
            r = rng.uniform(rmin, rmax) * w
            d = np.hypot(x - j, y - i) / r
            # solid core, leafy (ragged) rim: holes only in the outer 25 % of each cluster
            disc = (d < 0.75) | ((d < 1) & (edge_noise > (d - 0.75) * 8 - 1.0))
            crown |= disc
            shade = np.maximum(shade, np.where(disc, 1 - 0.5 * d + 0.25 * (y - i) / r, 0))  # each cluster lit from above
        grain = noise((h, w), 1.5, seed + 10 + kind)
        height_light = 0.70 + 0.35 * np.clip(v / top, 0, 1)
        # darker deep inside the crown (self-shadow), lighter on the clusters' sunny tops, a little hue variation
        depth = np.clip(1 - np.hypot((u - 0.5) / rx, (v / top - (cy if cy else 0.5)) / 0.4), 0, 1)
        hue = 1 + 0.08 * noise((h, w), 25, seed + 30 + kind)[..., None] * np.array([1.0, 0.4, -0.6])
        col = np.asarray(leaf) * hue * (height_light * (0.70 + 0.40 * np.clip(shade, 0, 1)) * (1 - 0.25 * depth)
                                        * (1 + 0.12 * grain))[..., None]
        trunk = (np.abs(u - 0.5) < 0.035 - 0.015 * v / top) & (v / top < (0.5 if cy is None else cy))
        bark = np.asarray((0.30, 0.23, 0.16)) * (0.8 + 0.2 * noise((h, w), 1.0, seed + 20 + kind))[..., None]
        rgb = np.where(crown[..., None], col, np.where(trunk[..., None], bark, np.asarray(leaf)))
        alpha = (crown | trunk).astype(float)
        out.append(np.dstack((np.clip(rgb, 0, 1), alpha)))
    return np.concatenate(out, axis=1)


def bench_atlas(w=512, h=256, seed=140):
    """Spectator benches: u 0..0.25 red-painted planks, u 0.25..0.5 yellow ones (the pairs alternate, project author /
    drone photos), both with grain, joints between the three planks and wear to bare wood; u 0.5..1 the dark metal of
    the frames."""
    y, x = np.mgrid[0:h, 0:w]
    wood, yellow = x < w // 2, x >= w // 4
    grain = noise((h, w), 0.8, seed) + 0.6 * noise((h, w), 3.0, seed + 1)
    joint = (y % (h // 3)) < 3
    paint = np.where(yellow[..., None], np.asarray((0.78, 0.62, 0.16)), np.asarray((0.52, 0.14, 0.09)))
    paint = paint * (1 + 0.10 * grain[..., None])
    paint = mix(paint, (0.42, 0.32, 0.26), 0.25 * smoothstep(1.0, 2.2, noise((h, w), 6, seed + 2)))  # worn
    metal = np.asarray((0.11, 0.11, 0.12)) * (1 + 0.15 * noise((h, w), 1.0, seed + 3)[..., None])
    albedo = np.where(wood[..., None], np.where(joint[..., None], np.asarray((0.12, 0.08, 0.05)), paint), metal)
    height = np.where(wood, 0.6 * grain - 2.0 * joint, 0.3 * noise((h, w), 1.0, seed + 4))
    return np.clip(albedo, 0, 1), height


def fence_wire(w=256, h=180, seed=155):
    """A plain wire fence, one 2 m x 1.4 m panel: five horizontal wires (alpha test), rusty grey."""
    img = np.zeros((h, w, 4))
    img[..., :3] = (0.45, 0.43, 0.40)
    for z in np.linspace(0.12, 0.97, 5):
        r = int(z * h)
        img[r - 1:r + 1, :, 3] = 1.0
    return img


def fence_mesh(w=512, h=384, cell_px=10, seed=150):
    """Wire mesh fence, one 2.5 m x 1.8 m panel (trackgen.FENCE uv_m / height), RGBA, row 0 = bottom: diamond mesh
    ~5 cm, galvanised grey, a tube along the top and the bottom; alpha 1 on the wire only (alpha test)."""
    y, x = np.mgrid[0:h, 0:w]
    a = ((x + y) % cell_px < 2) | ((x - y) % cell_px < 2)
    rails = (y < 6) | (y >= h - 8)
    wire = a | rails
    tone = 0.55 + 0.06 * noise((h, w), 2.0, seed)
    rgb = np.dstack((tone, tone + 0.01, tone + 0.02)) * np.where(rails, 0.85, 1.0)[..., None]
    return np.dstack((np.clip(rgb, 0, 1), wire.astype(float)))


GLYPHS = {  # 5 x 7 block letters for the club sign (rows top to bottom)
    "K": ["10001", "10010", "10100", "11000", "10100", "10010", "10001"],
    "A": ["01110", "10001", "10001", "11111", "10001", "10001", "10001"],
    "R": ["11110", "10001", "10001", "11110", "10100", "10010", "10001"],
    "T": ["11111", "00100", "00100", "00100", "00100", "00100", "00100"],
    "O": ["01110", "10001", "10001", "10001", "10001", "10001", "01110"],
    "D": ["11110", "10001", "10001", "10001", "10001", "10001", "11110"],
    "M": ["10001", "11011", "10101", "10101", "10001", "10001", "10001"],
    "G": ["01110", "10001", "10000", "10111", "10001", "10001", "01111"],
    "S": ["01111", "10000", "10000", "01110", "00001", "00001", "11110"],
    "Y": ["10001", "10001", "01010", "00100", "00100", "00100", "00100"],
    "Ń": ["00010", "10101", "11001", "10101", "10011", "10001", "10001"],
    "a": ["00000", "00000", "01110", "00001", "01111", "10001", "01111"],
    "r": ["00000", "00000", "10110", "11001", "10000", "10000", "10000"],
    "t": ["01000", "01000", "11100", "01000", "01000", "01001", "00110"],
    "o": ["00000", "00000", "01110", "10001", "10001", "10001", "01110"],
    "d": ["00001", "00001", "01101", "10011", "10001", "10001", "01111"],
    "m": ["00000", "00000", "11010", "10101", "10101", "10101", "10101"],
    "s": ["00000", "00000", "01111", "10000", "01110", "00001", "11110"],
    "y": ["00000", "10001", "10001", "10001", "01111", "00001", "01110"],
    "ń": ["00010", "00100", "10110", "11001", "10001", "10001", "10001"],
    " ": ["00000"] * 7,
}


def _text(img, text, x0, y0, cell, colour):
    """Block letters into img (row 0 = bottom): top-left of the first letter at pixel (x0, y0 from the top)."""
    h = img.shape[0]
    for k, ch in enumerate(text):
        for r, row in enumerate(GLYPHS[ch]):
            for c, bit in enumerate(row):
                if bit == "1":
                    x, y = x0 + (k * 6 + c) * cell, y0 + r * cell
                    img[h - y - cell:h - y, x:x + cell] = colour


def _facade_base(w, h, colour, seed, grain=0.03):
    tone = noise((h, w), 1.5, seed) * grain + noise((h, w), 20, seed + 1) * grain * 0.8
    return np.clip(np.asarray(colour) * (1 + tone[..., None]), 0, 1), 0.3 * noise((h, w), 1.0, seed + 2)


def _rect(img, height, ppm, x0, x1, z0, z1, colour, depth=-1.0):
    """A wall feature in metres (x along the tile, z up) into albedo img and height map (row 0 = bottom)."""
    j0, j1, i0, i1 = int(x0 * ppm), int(x1 * ppm), int(z0 * ppm), int(z1 * ppm)
    img[max(0, i0):i1, max(0, j0):j1] = colour
    height[max(0, i0):i1, max(0, j0):j1] = depth


def _windows(img, height, ppm, xs, zs, size, glass=(0.10, 0.12, 0.15), frame=(0.92, 0.92, 0.90)):
    for x in xs:
        for z in zs:
            _rect(img, height, ppm, x - 0.06, x + size[0] + 0.06, z - 0.06, z + size[1] + 0.06, frame, 0.3)
            _rect(img, height, ppm, x, x + size[0], z, z + size[1], glass, -1.0)


def _chequer(img, height, ppm, x0, x1, z0, z1, sq, green=(0.08, 0.42, 0.20)):
    h = img.shape[0]
    for i in range(int((z1 - z0) / sq + 0.5)):
        for j in range(int((x1 - x0) / sq + 0.5)):
            if (i + j) % 2 == 0:
                _rect(img, height, ppm, x0 + j * sq, x0 + (j + 1) * sq, z0 + i * sq, z0 + (i + 1) * sq, green, 0.0)


def club_facade(seed=200, ppm=48):
    """The main club building, one 6 m x 7.5 m tile (trackgen FACADE_TILE_M): plain white render, a pair of narrow
    vertical windows high up, a light grey plinth (photo of the building from the paddock, 2026)."""
    w, h = int(6.0 * ppm), int(7.5 * ppm)
    img, ht = _facade_base(w, h, WALL_YELLOW, seed, grain=0.02)
    _windows(img, ht, ppm, (2.2, 3.2), (4.7,), (0.45, 1.3))
    _rect(img, ht, ppm, 0, 6.0, 0, 0.3, (0.62, 0.62, 0.60), 0.2)
    return img, ht


# club, annex and booth repainted in the track's colours (project author, 2026): yellow walls, red accents where
# the green squares / bands / panels were
WALL_YELLOW, ACCENT_RED = (0.93, 0.76, 0.20), (0.74, 0.10, 0.07)
ANNEX_PINK, ANNEX_GREEN = (0.62, 0.12, 0.09), ACCENT_RED  # plinth, accents
FASCIA = 0.35  # the black edge under the annex roof


def _annex_base(w, h, ppm, length, seed):
    img, ht = _facade_base(w, h, WALL_YELLOW, seed, grain=0.02)
    _rect(img, ht, ppm, 0, length, 3.6 - FASCIA, 3.6, (0.09, 0.09, 0.10), 0.4)  # black fascia under the roof
    _rect(img, ht, ppm, 0, length, 0, 0.32, ANNEX_PINK, 0.2)  # pink plinth
    return img, ht


def annex_facade(seed=205, ppm=64):
    """The single-storey annex in front of the club building, one 4 m x 3.6 m tile: white, black fascia, pink plinth."""
    w, h = int(4.0 * ppm), int(3.6 * ppm)
    return _annex_base(w, h, ppm, 4.0, seed)


def _stair_squares(img, ht, ppm, x0, z0, sq, steps, direction):
    """Green squares stepping diagonally (the sign's pattern): one square up and `direction` across per step, a
    second square beside it every other step."""
    for k in range(steps):
        x, z = x0 + direction * k * sq, z0 + k * sq
        _rect(img, ht, ppm, x, x + sq, z, z + sq, ANNEX_GREEN, 0.0)
        if k % 2 == 0:
            x2 = x + direction * sq
            _rect(img, ht, ppm, x2, x2 + sq, z, z + sq, ANNEX_GREEN, 0.0)


def annex_sign(length=11.3, seed=215, ppm=80):
    """The annex's front to the paddock, mapped once across the wall (`length` m x 3.6 m), as on the photo from the
    paddock: dark green "Kartodrom Gostyń" top left, green stepped squares at both sides and a green panel on the
    right, two brown roller shutters, posters, a light grey door, black fascia, pink plinth."""
    w, h = int(length * ppm), int(3.6 * ppm)
    img, ht = _annex_base(w, h, ppm, length, seed)
    sq = 0.28
    _stair_squares(img, ht, ppm, 0.2, 0.55, sq, 6, 1)
    _stair_squares(img, ht, ppm, length - 0.75, 0.55, sq, 6, -1)
    _rect(img, ht, ppm, length * 0.62, length * 0.80, 0.32, 2.6, ANNEX_GREEN, 0.0)  # green panel, right
    rows = np.arange(h)
    slats = (rows % int(0.08 * ppm)) < 2
    for x0 in (length * 0.18, length * 0.48):  # roller shutters
        _rect(img, ht, ppm, x0, x0 + 1.35, 1.35, 2.6, (0.36, 0.25, 0.18), -0.5)
        i0, i1 = int(1.35 * ppm), int(2.6 * ppm)
        band = img[i0:i1, int(x0 * ppm):int((x0 + 1.35) * ppm)]
        band[slats[i0:i1]] *= 0.8
    for x0, z0, ww, hh, col in ((length * 0.35, 1.9, 1.0, 0.8, (0.80, 0.80, 0.76)),  # posters
                                (length * 0.20, 0.75, 1.2, 0.5, (0.55, 0.62, 0.72)),
                                (length * 0.48, 0.75, 1.2, 0.5, (0.55, 0.62, 0.72))):
        _rect(img, ht, ppm, x0, x0 + ww, z0, z0 + hh, col, 0.1)
    _rect(img, ht, ppm, length * 0.83, length * 0.83 + 0.95, 0.32, 2.3, (0.78, 0.79, 0.80), -0.4)  # door
    cell = max(1, int(0.32 / 7 * ppm))  # 0.32 m tall lettering just under the fascia
    _text(img, "Kartodrom Gostyń", int(0.45 * ppm), int((FASCIA + 0.3) * ppm), cell, (0.50, 0.05, 0.04))
    return img, ht


def red_roof(size=256, seed=245, colour=(0.62, 0.20, 0.15)):
    """Roof sheet of the dark shed north of the annex, red, 8 m tile."""
    g = noise((size, size), 1.0, seed) * 0.05
    rib = (((np.arange(size) + 8) % 16) < 3)[None, :] * np.ones((size, 1))  # ribs clear of the tile edge
    img = np.asarray(colour) * ((1 + g) * np.where(rib > 0, 0.85, 1.0))[..., None]
    return np.clip(img, 0, 1), rib + 0.2 * g


BOOTH_H, BOOTH_W = 5.0, 3.2  # the booth: two storeys, square (drone close-up)


def booth_facade(seed=220, ppm=80):
    """Race control booth side wall, one 3.2 m x 5 m tile (drone close-up): white panels, green stepped squares
    across the upper storey, a dark fascia under the roof."""
    w, h = int(BOOTH_W * ppm), int(BOOTH_H * ppm)
    img, ht = _facade_base(w, h, WALL_YELLOW, seed, grain=0.02)
    _rect(img, ht, ppm, 0, BOOTH_W, 0, 0.25, (0.62, 0.62, 0.60), 0.2)  # plinth
    _rect(img, ht, ppm, 0, BOOTH_W, 2.45, 2.55, (0.75, 0.76, 0.75), 0.3)  # floor line between the storeys
    _stair_squares(img, ht, ppm, 0.15, 2.75, 0.32, 6, 1)
    _rect(img, ht, ppm, 1.6, BOOTH_W - 0.15, 3.0, 4.4, ANNEX_GREEN, 0.0)  # the big green panel
    _rect(img, ht, ppm, 0, BOOTH_W, BOOTH_H - 0.25, BOOTH_H, (0.30, 0.32, 0.33), 0.4)  # fascia
    return img, ht


def booth_sign(seed=225, ppm=100):
    """The booth's front to the track, mapped once across the wall (3.2 m x 5 m): the light blue double door below,
    a green band with stepped squares, the window band of the cabin above (under the awning), a dark fascia."""
    w, h = int(BOOTH_W * ppm), int(BOOTH_H * ppm)
    img, ht = _facade_base(w, h, WALL_YELLOW, seed, grain=0.02)
    _rect(img, ht, ppm, 0, BOOTH_W, 0, 0.25, (0.62, 0.62, 0.60), 0.2)
    _rect(img, ht, ppm, 1.0, 2.3, 0.25, 2.25, (0.55, 0.68, 0.76), -0.5)  # door
    _rect(img, ht, ppm, 1.64, 1.66, 0.25, 2.25, (0.35, 0.45, 0.52), -0.6)  # its two leaves
    _rect(img, ht, ppm, 0, BOOTH_W, 2.45, 2.85, ANNEX_GREEN, 0.0)  # green band under the windows
    _stair_squares(img, ht, ppm, 0.1, 2.0, 0.22, 2, 1)
    for k in range(3):  # three window panes with white frames
        x0 = 0.12 + k * (BOOTH_W - 0.24) / 3
        _rect(img, ht, ppm, x0, x0 + (BOOTH_W - 0.24) / 3, 2.95, 4.35, (0.92, 0.92, 0.90), 0.3)
        _rect(img, ht, ppm, x0 + 0.06, x0 + (BOOTH_W - 0.24) / 3 - 0.06, 3.01, 4.29, (0.10, 0.12, 0.15), -1.0)
    _rect(img, ht, ppm, 0, BOOTH_W, BOOTH_H - 0.25, BOOTH_H, (0.30, 0.32, 0.33), 0.4)
    return img, ht


def shed_facade(seed=230, ppm=80):
    """Kart garages by the paddock, one 3 m x 3 m tile: dark grey trapezoidal sheet with a roller door."""
    w, h = int(3.0 * ppm), int(3.0 * ppm)
    img, ht = _facade_base(w, h, (0.30, 0.32, 0.34), seed)
    x = np.arange(w)
    ribs = (((x + int(0.1 * ppm)) % int(0.2 * ppm)) < int(0.05 * ppm))  # not on the tile edge
    img[:, ribs] *= 1.25
    ht[:, ribs] = 1.0
    _rect(img, ht, ppm, 0.3, 2.7, 0.0, 2.4, (0.55, 0.56, 0.56), -0.4)
    rows = (np.arange(h) % int(0.1 * ppm)) < 2  # roller door slats (rows from the bottom)
    sub = img[:int(2.4 * ppm), int(0.3 * ppm):int(2.7 * ppm)]
    sub[rows[:int(2.4 * ppm)]] *= 0.8
    return np.clip(img, 0, 1), ht


def roof(size=512, seed=240):
    """Flat roof membrane / gravel, 8 m tile (the plates' uv): mid grey, mottled."""
    g = noise((size, size), 1.0, seed) * 0.04 + noise((size, size), 30, seed + 1) * 0.05
    return np.clip(np.asarray((0.46, 0.46, 0.45)) * (1 + g[..., None]), 0, 1), 0.4 * noise((size, size), 1.0, seed + 2)


def timing_board(w=512, h=256, seed=250):
    """The timing screen: black, a blue header and rows of white / yellow / green characters (the Facebook photo)."""
    img = np.full((h, w, 3), 0.03)
    img[h - 34:h - 6, 8:w - 8] = (0.10, 0.25, 0.65)
    rng = np.random.default_rng(seed)
    for r in range(7):
        y = h - 50 - r * 28
        for c, (x0, x1, col) in enumerate(((14, 40, (0.95, 0.85, 0.2)), (52, 300, (0.92, 0.92, 0.92)),
                                            (320, 420, (0.3, 0.9, 0.35)), (430, 498, (0.95, 0.85, 0.2)))):
            x = x0
            while x < x1:
                cw = int(rng.integers(8, 13))
                if rng.random() > 0.12:
                    img[y - 16:y, x:x + cw - 3] = col
                x += cw
    return img, np.zeros((h, w))


def lamp_atlas(w=64, h=64):
    """Lamp posts and the board frame: u 0..0.5 dark grey metal, u 0.5..1 the white luminaire."""
    img = np.empty((h, w, 3))
    img[:, :w // 2] = (0.20, 0.21, 0.22)
    img[:, w // 2:] = (0.72, 0.74, 0.75)  # galvanised steel: fence posts, gates; luminaires
    return img, np.zeros((h, w))


def railing(w=256, h=16):
    """Steel railing tube, red and white stripes (one of each per trackgen RAIL_STRIPE_M along u)."""
    img = np.empty((h, w, 3))
    img[:, :w // 2] = (0.75, 0.08, 0.06)
    img[:, w // 2:] = (0.92, 0.92, 0.90)
    shade = 0.75 + 0.25 * np.sin(np.linspace(0, np.pi, h))[:, None, None]  # round tube
    return np.clip(img * shade, 0, 1), np.zeros((h, w))


PHOTO_SATURATION, PHOTO_GAIN = 1.8, 0.92


def grade_photo(rgb, sat=PHOTO_SATURATION, gain=PHOTO_GAIN):
    """Aerial photo -> texture colours: the haze leaves the grass pale and bluish (0.31 0.36 0.28 on the 5 cm photo,
    vs 0.30 0.33 0.18 for diffuse x detail on the Kunos tracks and 0.33 0.45 0.22 on a ground photo of the track).
    Saturation up around the luminance, a little darker; greys (asphalt) keep their tone."""
    lum = (rgb * np.array([0.2126, 0.7152, 0.0722])).sum(-1, keepdims=True)
    return np.clip((lum + (rgb - lum) * sat) * gain, 0, 1)


def write_png(path, rgba):
    """(h, w, 4) 0..1, row 0 = bottom -> 8-bit RGBA PNG (straight alpha, written as is - no premultiplying)."""
    import struct
    import zlib
    h, w, _ = rgba.shape
    px = np.round(np.clip(rgba[::-1], 0, 1) * 255).astype(np.uint8)
    raw = np.concatenate((np.zeros((h, 1), np.uint8), px.reshape(h, w * 4)), axis=1).tobytes()
    chunk = lambda t, d: struct.pack(">I", len(d)) + t + d + struct.pack(">I", zlib.crc32(t + d) & 0xFFFFFFFF)
    with open(path, "wb") as f:
        f.write(b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 6, 0, 0, 0))
                + chunk(b"IDAT", zlib.compress(raw, 6)) + chunk(b"IEND", b""))


# name -> (maker, normal-map strength); trackgen's material names
SURFACES = {"road": (asphalt, 6.0), "concrete": (concrete, 4.0), "kerb": (kerb_stripes, 3.0),
            "kerb_apron": (kerb_apron, 4.0), "kerb_concrete": (kerb_concrete, 4.0), "paint_white": (paint_white, 2.0),
            "paint_mint": (lambda: paint_white(seed=65, colour=(0.66, 0.78, 0.72)), 2.0),  # grid boxes (photo)
            "chequer": (chequer, 2.0),
            "tyre_red": (lambda: tyre((0.62, 0.08, 0.06), seed=70), 3.0),
            "tyre_yellow": (lambda: tyre((0.88, 0.70, 0.12), seed=80), 3.0),
            "tyre_black": (lambda: tyre((0.09, 0.09, 0.09), seed=90), 3.0),  # unpainted rows (embankment)
            "bench": (bench_atlas, 2.0),
            "paint_blue": (lambda: paint_white(seed=66, colour=(0.45, 0.62, 0.82)), 2.0),  # paddock pit strip
            "club": (club_facade, 2.0), "annex": (annex_facade, 2.0), "annex_sign": (annex_sign, 2.0),
            "roof_red": (red_roof, 2.0), "awning": (lambda: red_roof(seed=246, colour=(0.33, 0.20, 0.13)), 2.0),
            "booth": (booth_facade, 2.0), "booth_sign": (booth_sign, 2.0),
            "shed": (shed_facade, 2.0), "roof": (roof, 2.0), "timing_board": (timing_board, 0.5),
            "lamp": (lamp_atlas, 0.5), "railing": (railing, 0.5)}
# tile seams that must be invisible (0 = along v, 1 = along u): facades repeat along the walls only, the sign and
# the screen are mapped once, the kerb stripes' hard edges repeat every stripe
SEAM_AXES = {"kerb": (1,), "club": (1,), "annex": (1,), "booth": (1,), "shed": (1,), "annex_sign": (),
             "booth_sign": (),
             "timing_board": (),
             "lamp": (), "railing": ()}  # flat colours, no relief
