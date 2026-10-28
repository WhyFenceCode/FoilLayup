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
