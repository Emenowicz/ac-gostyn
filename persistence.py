"""ksEditor persistence file (<model>.fbx.ini): every material's shader, values and textures, plus per-object flags.

ksEditor: import the FBX, select the MODEL node in the scene tree, File -> Load Persistence -> this file. The format
is what ksEditor's own File -> Save Persistence writes (VERSION=4): materials listed with all of their shader's
variables (each as FLOAT1..FLOAT4) and texture slots, then one section per FBX node.
"""

# shader -> (variables in ksEditor's order, (texture, slot) pairs, alpha tested); copied from a ksEditor save
_BASE = ["ksAmbient", "ksDiffuse", "ksSpecular", "ksSpecularEXP", "ksEmissive", "ksAlphaRef"]
SHADERS = {
    "ksPerPixel": (_BASE, [("txDiffuse", 0)], False),
    "ksPerPixelNM": (_BASE + ["fresnelC", "fresnelEXP", "nmObjectSpace", "isAdditive", "fresnelMaxLevel", "boh"],
                     [("txDiffuse", 0), ("txNormal", 1)], False),
    "ksMultilayer_fresnel_nm": (_BASE + ["multR", "multG", "multB", "multA", "magicMult", "fresnelC", "fresnelEXP",
                                         "fresnelMaxLevel", "tarmacSpecularMultiplier", "detailNMMult"],
                                [("txDiffuse", 0), ("txMask", 1), ("txDetailR", 2), ("txDetailG", 3), ("txDetailB", 4),
                                 ("txDetailA", 5), ("txDetailNM", 21)], False),
    "ksPerPixelAT": (_BASE, [("txDiffuse", 0)], True),
}
# lighting response shared by all materials (as on the Kunos tracks, e.g. magione: kerbs 0.3-0.4 / 0.35-0.4); ksEditor's
# defaults are ambient 0 (black in shade) and diffuse 0.8 (washed out in the sun)
LIGHT = {"ksAmbient": 0.35, "ksDiffuse": 0.35, "ksSpecular": 0.0, "ksSpecularEXP": 10.0}


def _num(x):
    return f"{x:g}"


def material_block(i, name, shader, values, textures):
    """[MATERIAL_i] section; values / textures not given default to 0 / NULL (as ksEditor writes unset ones)."""
    names, slots, alpha_test = SHADERS[shader]
    unknown = (set(values) - set(names)) | (set(textures) - {t for t, _ in slots})
    assert not unknown, f"{name}: {shader} has no {sorted(unknown)}"
    out = [f"[MATERIAL_{i}]", f"NAME={name}", f"SHADER={shader}", "ALPHABLEND=0", f"ALPHATEST={int(alpha_test)}",
           "DEPTHMODE=0", f"VARCOUNT={len(names)}"]
    for k, var in enumerate(names):
        out += [f"VAR_{k}_NAME={var}", f"VAR_{k}_FLOAT1={_num(values.get(var, 0.0))}", f"VAR_{k}_FLOAT2=0,0",
                f"VAR_{k}_FLOAT3=0,0,0", f"VAR_{k}_FLOAT4=0,0,0,0"]
    out.append(f"RESCOUNT={len(slots)}")
    for k, (tex, slot) in enumerate(slots):
        out += [f"RES_{k}_NAME={tex}", f"RES_{k}_SLOT={slot}", f"RES_{k}_TEXTURE={textures.get(tex, 'NULL')}"]
    return out


def fbx_ini(fbx_name, materials, meshes, empties, date="", no_shadow=(), hidden=()):
    """The persistence file's text (CRLF, like ksEditor's).

    materials: {name: (shader, {variable: value}, {texture slot: file name})}
    meshes / empties: FBX object names; no_shadow: meshes that don't cast shadows; hidden: meshes not rendered
    (still in the KN5, so physics still sees them)."""
    out = ["[HEADER]", "VERSION=4", "DLC_KEY=0", "USERNAME=", f"DATE={date}", "MD5_HASH=", "",
           "[MATERIAL_LIST]", f"COUNT={len(materials)}", ""]
    for i, (name, (shader, values, textures)) in enumerate(materials.items()):
        out += material_block(i, name, shader, {**LIGHT, **values}, textures) + [""]
    root = f"model_FBX: {fbx_name}"
    out += [f"[{root}]", "ACTIVE=1", "PRIORITY=0", ""]
    for name in sorted(list(meshes) + list(empties)):
        out += [f"[{root}_{name}]", "ACTIVE=1", "PRIORITY=0", ""]
        if name in meshes:  # the mesh node under the object node carries the render flags
            out += [f"[{root}_{name}_{name}]", "ACTIVE=1", "PRIORITY=0", "VISIBLE=1", "TRANSPARENT=0",
                    f"CAST_SHADOWS={int(name not in no_shadow)}", "LOD_IN=0", "LOD_OUT=0",
                    f"RENDERABLE={int(name not in hidden)}", ""]
    return "\r\n".join(out)
