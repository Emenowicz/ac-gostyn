"""Procedural, tileable surface textures with normal maps (numpy only - runs in Blender's bundled Python).

Every texture is built from white noise filtered in the frequency domain, so it wraps at its edges and tiles
without seams; fixed seeds make every build write the same images. Arrays are row 0 = bottom (Blender's pixel
order), x = texture u, y = texture v. Each maker returns (albedo (h, w, 3), height (h, w)); normal_map() turns the
height into a tangent-space normal map (OpenGL convention: green = +v).

Texel sizes match the UVs written by trackgen (UV_M = 4 m per tile): asphalt and concrete 1024 px per 4 m
(4 mm/px), kerbs 256 px across the kerb (u 0..1) x 1024 px per 4 m along it.
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


def asphalt(size=1024, seed=10):
    """Asphalt concrete: fine grain, light and dark aggregate, soft blotches and smoother darker repair patches."""
    s = (size, size)
    grain, stones, mottle, patches = (noise(s, sig, seed + k) for k, sig in enumerate((0.8, 1.6, 35, 120)))
    light, dark = smoothstep(1.3, 2.2, stones), smoothstep(1.5, 2.5, -stones)
    patch = smoothstep(0.9, 1.4, patches)  # sealed repairs: darker, smoother
    rough = 1 - 0.6 * patch
    v = (0.035 * grain + 0.10 * light - 0.08 * dark) * rough + 0.012 * mottle - 0.035 * patch
    albedo = np.clip(np.asarray(ASPHALT) + v[..., None], 0, 1)
    return albedo, (0.5 * grain + 1.5 * light - 0.8 * dark) * rough


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


def kerb_stripes(w=256, h=1024, stripes=8, seed=30):
    """Red/yellow kerb, `stripes` stripes per 4 m tile along v (0.5 m), worn, rubbered, dirty."""
    y = np.arange(h)[:, None] * np.ones((1, w))
    stripe = ((y * stripes // h) % 2).astype(float)
    grain = noise((h, w), 1.0, seed + 5)
    albedo = mix(np.full((h, w, 3), RED), YELLOW, stripe) * (1 + 0.05 * grain[..., None])
    albedo, wear = _kerb_wear(np.clip(albedo, 0, 1), h, w, seed)
    return np.clip(albedo, 0, 1), 0.5 * grain - 1.0 * wear


def kerb_apron(w=256, h=1024, red_part=0.625, seed=40):
    """Red-dyed concrete band (asphalt side) with a light outer border, concrete texture, rubber and dirt."""
    base, height = concrete((h, w), CONCRETE, seed)
    u = (np.arange(w)[None, :] + 0.5) / w * np.ones((h, 1))
    red = (u < red_part).astype(float)
    tone = base / np.asarray(CONCRETE)  # keep the concrete's light/dark variation, recoloured
    albedo = mix(np.clip(tone * WHITE, 0, 1), np.clip(tone * RED, 0, 1), red)
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


# name -> (maker, normal-map strength); trackgen's material names
SURFACES = {"road": (asphalt, 6.0), "concrete": (concrete, 4.0), "kerb": (kerb_stripes, 3.0),
            "kerb_apron": (kerb_apron, 4.0), "kerb_concrete": (kerb_concrete, 4.0), "paint_white": (paint_white, 2.0),
            "paint_mint": (lambda: paint_white(seed=65, colour=(0.66, 0.78, 0.72)), 2.0),  # grid boxes (photo)
            "chequer": (chequer, 2.0),
            "tyre_red": (lambda: tyre((0.62, 0.08, 0.06), seed=70), 3.0),
            "tyre_yellow": (lambda: tyre((0.88, 0.70, 0.12), seed=80), 3.0)}
