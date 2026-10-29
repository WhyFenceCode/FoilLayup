"""Layer table <-> CSV, so stacks can be edited in a spreadsheet.

One row per stack line. 'group' is 'layer' (inside the body, listed outside-in)
or 'external' (outside the body, listed from the surface outward). Lengths mm,
chord limits as fractions 0-1, splits separated by ';'.
"""
import csv
import io

COLUMNS = ["group", "id", "name", "material", "count", "angles", "thickness", "side",
           "span_from", "span_to", "chord_from", "chord_to", "bound_sketch", "bound_mode",
           "outside_material", "seam", "splits"]


def _rng(v, k):
    return "" if not v or v[k] in (None, "") else v[k]


def stack_to_csv(stack):
    buf = io.StringIO()
    w = csv.DictWriter(buf, fieldnames=COLUMNS)
    w.writeheader()
    for group, key in (("external", "external"), ("layer", "layers")):
        for r in stack.get(key) or []:
            b = r.get("bound") or {}
            w.writerow({
                "group": group, "id": r.get("id", ""), "name": r.get("name", ""),
                "material": r.get("material", ""), "count": r.get("count", 1),
                "angles": r.get("angles", "0"), "thickness": r.get("thickness") or "",
                "side": r.get("side", "wrap"),
                "span_from": _rng(r.get("span"), 0), "span_to": _rng(r.get("span"), 1),
                "chord_from": _rng(r.get("chord"), 0), "chord_to": _rng(r.get("chord"), 1),
                "bound_sketch": b.get("sketch", ""), "bound_mode": b.get("mode", "inside") if b.get("sketch") else "",
                "outside_material": r.get("outsideMaterial") or "",
                "seam": r.get("seam", "auto"),
                "splits": ";".join(str(v) for v in r.get("splits") or []),
            })
    return buf.getvalue()


def _num(v):
    v = (v or "").strip()
    return None if v == "" else float(v)


def csv_to_stack(text):
    """Returns {'layers': [...], 'external': [...]} to merge into a stack."""
    out = {"layers": [], "external": []}
    for row in csv.DictReader(io.StringIO(text)):
        row = {k.strip().lower(): (v or "").strip() for k, v in row.items() if k}
        r = {
            "id": row.get("id") or None,
            "name": row.get("name", ""),
            "material": row.get("material", ""),
            "count": int(float(row.get("count") or 1)),
            "angles": row.get("angles") or "0",
            "side": row.get("side") or "wrap",
            "seam": row.get("seam") or "auto",
        }
        if row.get("thickness"):
            r["thickness"] = float(row["thickness"])
        span = (_num(row.get("span_from")), _num(row.get("span_to")))
        if any(v is not None for v in span):
            r["span"] = list(span)
        chord = (_num(row.get("chord_from")), _num(row.get("chord_to")))
        if any(v is not None for v in chord):
            r["chord"] = list(chord)
        if row.get("bound_sketch"):
            r["bound"] = {"sketch": row["bound_sketch"], "mode": row.get("bound_mode") or "inside"}
        if row.get("outside_material"):
            r["outsideMaterial"] = row["outside_material"]
        if row.get("splits"):
            r["splits"] = [float(v) for v in row["splits"].split(";") if v.strip()]
        group = (row.get("group") or "layer").lower()
        out["external" if group.startswith("ext") else "layers"].append(r)
    return out


# --------------------------------------------------------------------------- reports
def _rows_to_csv(sections):
    """sections: [(title, header list, rows list of lists)] -> one CSV text with a
    blank line between sections, so it opens as readable blocks in a spreadsheet."""
    buf = io.StringIO()
    w = csv.writer(buf)
    for k, (title, header, rows) in enumerate(sections):
        if k:
            w.writerow([])
        w.writerow([title])
        if header:
            w.writerow(header)
        for r in rows:
            w.writerow(r)
    return buf.getvalue()


def build_report_csv(res):
    """CSV of a Generate result: bodies, mass by material/fibre, totals, seams."""
    bodies = res.get("bodies") or []
    lam = [b for b in bodies if b.get("role") in ("ply", "fill")]
    core = [b for b in bodies if b.get("role") in ("core", "corefill", "gap")]
    ext = [b for b in bodies if b.get("role") == "external"]
    dry = sum(b.get("dry_g") or 0 for b in lam)
    wet = sum(b.get("mass_g") or 0 for b in lam)
    core_g = sum(b.get("mass_g") or 0 for b in core)
    sections = [
        ("Foil Layup build report", None, [["generated", res.get("when", "")], ["design", res.get("design", "")]]),
        ("Bodies", ["body", "role", "material", "volume_cm3", "dry_fibre_g", "with_resin_g"],
         [[b.get("name"), b.get("role"), b.get("material"), b.get("volume_cm3"),
           "" if b.get("dry_g") is None else b.get("dry_g"), b.get("mass_g")] for b in bodies]),
    ]
    bd = res.get("breakdown") or {}
    sections.append(("Mass by material", ["material", "plies", "dry_fibre_g", "with_resin_g"],
                     [[m["material"], m["plies"], m["dry_g"], m["wet_g"]] for m in bd.get("byMaterial", [])]
                     + [["all " + f["fibre"], "", f["dry_g"], f["wet_g"]] for f in bd.get("byFibre", [])]))
    sections.append(("Totals", ["item", "grams"], [
        ["dry fibre (all plies)", round(dry, 1)],
        ["resin in the laminate", round(wet - dry, 1)],
        ["laminate with resin", round(wet, 1)],
        ["core (incl. drape gaps)", round(core_g, 1)],
        ["finished part", round(wet + core_g, 1)],
        ["external layers, dry (removed after cure)", round(sum(b.get("dry_g") or 0 for b in ext), 1)],
    ]))
    if res.get("seams"):
        sections.append(("Seams", ["ply", "seam", "ok"],
                         [[s["piece"] + " " + s.get("label", ""), s["msg"], "yes" if s.get("ok") else "check"]
                          for s in res["seams"]]))
    if res.get("skipped"):
        sections.append(("Not built", None, [[s] for s in res["skipped"]]))
    return _rows_to_csv(sections)


def flatten_report_csv(res):
    """CSV of a Flatten result: the pattern table plus mass by material."""
    pats = res.get("patterns") or []
    cols = ["label", "material", "angle_deg", "seam", "area_m2", "dry_fabric_g", "with_resin_g",
            "length_mm", "width_mm", "rms_strain_pct", "p95_shear_deg", "holes"]
    sections = [
        ("Foil Layup flat patterns", None, [["generated", res.get("when", "")], ["folder", res.get("folder", "")],
                                            ["roll length mm", round(res["rollLength"], 1) if isinstance(res.get("rollLength"), (int, float)) else ""]]),
        ("Patterns", cols, [[p.get(c, "") for c in cols] for p in pats]),
    ]
    by = {}
    for p in pats:
        if p.get("kind") == "ext":
            continue
        m = by.setdefault(p.get("material"), [0, 0.0, 0.0])
        m[0] += 1
        m[1] += p.get("dry_fabric_g") or 0
        m[2] += p.get("with_resin_g") or 0
    sections.append(("Mass by material", ["material", "pieces", "dry_fibre_g", "with_resin_g"],
                     [[k, v[0], round(v[1], 1), round(v[2], 1)] for k, v in by.items()]))
    return _rows_to_csv(sections)
