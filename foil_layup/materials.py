"""Built-in material library and stack presets.

Cured ply thickness is estimated from areal weight, fibre density and fibre
volume fraction:  t[mm] = gsm / (rho_f[g/cm3] * Vf * 1000).
Edit the numbers to match your own process (wet layup ~0.40 Vf, vacuum bag
~0.50, infusion ~0.55, prepreg ~0.60). User materials override built-ins by id.
"""

RESIN_DENSITY = 1.15  # g/cm3, epoxy

FIBRES = {
    "carbon": 1.78,
    "glass": 2.55,
    "basalt": 2.67,
    "aramid": 1.44,
    "flax": 1.45,
}

# Colours are only for the 3D bodies and previews.
COLOURS = {
    "carbon": "#2b2f36",
    "glass": "#cfe3e8",
    "basalt": "#7a5a48",
    "aramid": "#d8b437",
    "flax": "#b9a37a",
    "process": "#e5884f",
    "core": "#e9d9a6",
    "mat": "#a4c27a",
}


def fabric(mid, name, fibre, form, gsm, vf, weave=""):
    rho = FIBRES[fibre]
    t = gsm / (rho * vf * 1000.0)
    dens = (vf * rho + (1 - vf) * RESIN_DENSITY) * 1000.0
    return {
        "id": mid,
        "name": name,
        "category": "fabric",
        "fibre": fibre,
        "form": form,
        "weave": weave,
        "gsm": gsm,
        "vf": vf,
        "thickness": round(t, 3),
        "density": round(dens),
        "color": COLOURS[fibre],
        "builtin": True,
    }


def other(mid, name, category, thickness, density, color, form="", gsm=0):
    return {
        "id": mid,
        "name": name,
        "category": category,
        "fibre": "",
        "form": form,
        "weave": "",
        "gsm": gsm,
        "vf": 0,
        "thickness": thickness,
        "density": density,
        "color": color,
        "builtin": True,
    }


BUILTIN_MATERIALS = [
    # ---- carbon woven
    fabric("c160p", "Carbon 160g plain 3K", "carbon", "woven", 160, 0.50, "plain"),
    fabric("c200t", "Carbon 200g 2x2 twill 3K", "carbon", "woven", 200, 0.50, "2x2 twill"),
    fabric("c245t", "Carbon 245g 2x2 twill 3K", "carbon", "woven", 245, 0.50, "2x2 twill"),
    fabric("c300t", "Carbon 300g 2x2 twill 6K", "carbon", "woven", 300, 0.50, "2x2 twill"),
    fabric("c400t", "Carbon 400g 2x2 twill 12K", "carbon", "woven", 400, 0.50, "2x2 twill"),
    # ---- carbon spread tow
    fabric("cst80", "Carbon spread tow 80g 0/90", "carbon", "spread tow", 80, 0.55, "spread tow"),
    fabric("cst120", "Carbon spread tow 120g 0/90", "carbon", "spread tow", 120, 0.55, "spread tow"),
    fabric("cst160", "Carbon spread tow 160g 0/90", "carbon", "spread tow", 160, 0.55, "spread tow"),
    fabric("custp75", "Carbon UD spread tow tape 75g", "carbon", "UD", 75, 0.58, "spread tow UD"),
    # ---- carbon non-crimp
    fabric("cud150", "Carbon UD 150g", "carbon", "UD", 150, 0.55),
    fabric("cud200", "Carbon UD 200g", "carbon", "UD", 200, 0.55),
    fabric("cud300", "Carbon UD 300g", "carbon", "UD", 300, 0.55),
    fabric("cbx300", "Carbon biax +/-45 300g", "carbon", "biax", 300, 0.52, "+/-45 stitched"),
    fabric("cbx400", "Carbon biax +/-45 400g", "carbon", "biax", 400, 0.52, "+/-45 stitched"),
    # ---- basalt
    fabric("b220t", "Basalt 220g 2x2 twill", "basalt", "woven", 220, 0.50, "2x2 twill"),
    fabric("b300p", "Basalt 300g plain", "basalt", "woven", 300, 0.50, "plain"),
    fabric("bud300", "Basalt UD 300g", "basalt", "UD", 300, 0.55),
    fabric("bbx300", "Basalt biax +/-45 300g", "basalt", "biax", 300, 0.52, "+/-45 stitched"),
    fabric("bbx600", "Basalt biax +/-45 600g", "basalt", "biax", 600, 0.52, "+/-45 stitched"),
    # ---- glass
    fabric("g80p", "E-glass 80g plain (surface)", "glass", "woven", 80, 0.45, "plain"),
    fabric("g163p", "E-glass 163g plain", "glass", "woven", 163, 0.50, "plain"),
    fabric("g200t", "E-glass 200g 2x2 twill", "glass", "woven", 200, 0.50, "2x2 twill"),
    fabric("gbx300", "E-glass biax +/-45 300g", "glass", "biax", 300, 0.52, "+/-45 stitched"),
    fabric("gbx600", "E-glass biax +/-45 600g", "glass", "biax", 600, 0.52, "+/-45 stitched"),
    fabric("gud300", "E-glass UD 300g", "glass", "UD", 300, 0.55),
    # ---- other fibres
    fabric("a170p", "Aramid 170g plain", "aramid", "woven", 170, 0.50, "plain"),
    fabric("f200t", "Flax 200g 2x2 twill", "flax", "woven", 200, 0.45, "2x2 twill"),
    # ---- mats / bulkers
    other("csm300", "Glass chopped strand mat 300g", "mat", 0.65, 1450, COLOURS["mat"], "mat", 300),
    other("cmat2", "Core mat 2 mm (Soric/Lantor type)", "mat", 2.0, 600, COLOURS["mat"], "mat"),
    other("cmat3", "Core mat 3 mm (Soric/Lantor type)", "mat", 3.0, 600, COLOURS["mat"], "mat"),
    # ---- process consumables (normally used as external layers)
    other("pp85", "Peel ply nylon 85g", "process", 0.12, 1140, COLOURS["process"], "peel ply", 85),
    other("rf", "Perforated release film", "process", 0.03, 950, "#9fc3e6", "release film"),
    other("bre150", "Breather 150g", "process", 1.5, 100, "#d9d9d9", "breather", 150),
    other("fmesh", "Infusion flow mesh", "process", 0.8, 300, "#80b0d0", "flow media"),
    # ---- cores
    other("pvc80", "PVC foam 80 kg/m3", "core", 0, 80, COLOURS["core"], "foam"),
    other("pvc130", "PVC foam 130 kg/m3", "core", 0, 130, COLOURS["core"], "foam"),
    other("pvc200", "PVC foam 200 kg/m3", "core", 0, 200, "#e3c98a", "foam"),
    other("pet100", "PET foam 100 kg/m3", "core", 0, 100, "#e8e0b8", "foam"),
    other("paul", "Paulownia wood", "core", 0, 280, "#d2b48c", "wood"),
    other("balsa", "Balsa 150 kg/m3", "core", 0, 150, "#e8d3a2", "wood"),
    other("solid", "Solid laminate / filler", "core", 0, 1500, "#5a5a5a", "solid"),
]


BUILTIN_PRESETS = [
    {
        "id": "carbon-skin-2x",
        "name": "Carbon skin: spread tow + 2x 200g twill",
        "layers": [
            {"name": "Outer spread tow", "material": "cst120", "count": 1, "angles": "0", "side": "wrap"},
            {"name": "Twill 45", "material": "c200t", "count": 1, "angles": "45", "side": "wrap"},
            {"name": "Twill 0", "material": "c200t", "count": 1, "angles": "0", "side": "wrap"},
        ],
        "external": [{"name": "Peel ply", "material": "pp85", "count": 1, "angles": "0", "side": "wrap"}],
    },
    {
        "id": "basalt-skin",
        "name": "Basalt/carbon budget skin",
        "layers": [
            {"name": "Carbon twill face", "material": "c200t", "count": 1, "angles": "0", "side": "wrap"},
            {"name": "Basalt biax", "material": "bbx300", "count": 2, "angles": "0", "side": "wrap"},
        ],
        "external": [],
    },
    {
        "id": "uni-caps",
        "name": "UD spar caps top/bottom (needs a bound sketch)",
        "layers": [
            {"name": "Top cap", "material": "cud300", "count": 3, "angles": "0", "side": "top",
             "bound": {"sketch": "", "mode": "inside"}, "outsideMaterial": "core"},
            {"name": "Bottom cap", "material": "cud300", "count": 3, "angles": "0", "side": "bottom",
             "bound": {"sketch": "", "mode": "inside"}, "outsideMaterial": "core"},
        ],
        "external": [],
    },
]


def in_mould(material, vf):
    """Copy of a fabric with thickness and laminate density at fibre volume
    fraction vf (the consolidated, in-mould state). UD packs slightly tighter.
    Non-fabrics, unknown fibres and vf <= 0 are returned unchanged."""
    rho = FIBRES.get(material.get("fibre"))
    if vf <= 0 or material.get("category") != "fabric" or not rho or not material.get("gsm"):
        return material
    v = min(0.68, vf + (0.03 if material.get("form") == "UD" else 0.0))
    m = dict(material)
    m["vf"] = round(v, 3)
    m["thickness"] = round(material["gsm"] / (rho * v * 1000.0), 4)
    m["density"] = round((v * rho + (1 - v) * RESIN_DENSITY) * 1000.0)
    return m


def material_index(user_materials=None):
    idx = {m["id"]: dict(m) for m in BUILTIN_MATERIALS}
    for m in user_materials or []:
        mm = dict(m)
        mm["builtin"] = False
        idx[mm["id"]] = mm
    return idx
