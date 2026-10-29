"""Flatten command: flat patterns as DXF/SVG/CSV and as sketches in Fusion."""
import json
import os

import adsk.core
import adsk.fusion

from . import engine, export, materials, model
from .fusion_build import ATTR
from .fusion_geo import CM, FusionError, design, find_by_token, keepout_bodies, keepout_field_fn, \
    load_bounds, preferred_up, sample_sections, target_context, detect_frame


def _generated_keepout_tools(comp):
    """Grown keep-out bodies left by Generate (real bodies, clearance included)."""
    out = []
    for b in comp.bRepBodies:
        a = b.attributes.itemByName(ATTR, "role")
        if a:
            try:
                if json.loads(a.value).get("role") == "keepout":
                    out.append((b, 0.0, b.name))
            except ValueError:
                pass
    return out


def flatten(stack_data, user_materials, folder, make_sketches, log, progress):
    des = design()
    target = find_by_token((stack_data.get("target") or {}).get("token"))
    if target is None:
        raise FusionError("Pick the foil body first (Setup tab).")
    target = adsk.fusion.BRepBody.cast(target)
    mats = materials.material_index(user_materials)
    stack = model.Stack(stack_data, mats)
    st = stack.settings
    native, comp, _inv = target_context(target)
    frame = detect_frame(target, stack_data.get("frame") or {})

    raw = sample_sections(native, frame, int(st["flatStations"]), float(st["tipInset"]),
                          model.span_limit_stations(stack, frame),
                          lambda k, n: progress("Sampling sections", k, n))
    sections = engine.build_sections(raw, int(st["flatPoints"]), float(st["chordRefSmoothing"]))
    if len(sections) < 3:
        raise FusionError("Could not slice the body; check the span axis.")
    bounds = load_bounds(stack.bound_sketches, frame, log)

    kos = _generated_keepout_tools(comp)
    if not kos and stack_data.get("keepouts"):
        kos = keepout_bodies(stack_data, target)
        log("Keep-out holes use the keep-out bodies directly (run Generate first for exact clearances)")
    ko_fn = keepout_field_fn(kos, frame)

    def prog(n, total, label):
        progress("Flattening %s" % label, n, total)
        return True

    results, pieces = engine.flatten_all(stack, sections, frame, bounds, ko_fn, prog, log,
                                         stack_data.get("seamPlan"))
    if not results:
        raise FusionError("No flat patterns were produced.")

    written, length, too_wide, svg = export.write_outputs(folder, results, float(st["rollWidth"]))
    for n in too_wide:
        log("Pattern %s is wider than the roll (%.0f mm)" % (results[n]["label"], st["rollWidth"]))

    if make_sketches:
        progress("Drawing flat patterns", 0, 1)
        _draw(des, results, float(st["rollWidth"]), log)

    table = []
    for r in results:
        row = dict(r["book"])
        row["label"] = r["label"]
        table.append(row)
    return {"folder": folder, "files": [os.path.basename(p) for p in written], "patterns": table,
            "rollLength": round(length, 1), "svg": svg}


def _draw(des, results, roll_width, log):
    """Sketch every pattern (nested) on the ground plane of a new component."""
    root = des.rootComponent
    occ = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    comp = occ.component
    comp.name = "Flat patterns"
    plane = comp.xYConstructionPlane if preferred_up() == "Z" else comp.xZConstructionPlane
    offsets, _length, _tw = export.nest(results, roll_width)
    for r, off in zip(results, offsets):
        if off is None:
            continue
        sk = comp.sketches.add(plane)
        sk.name = export.safe_name(r["label"])[:60]
        sk.isComputeDeferred = True
        dx, dy = off
        _poly(sk, [(x + dx, y + dy) for x, y in r["outline"]])
        for h in r["holes"]:
            _poly(sk, [(x + dx, y + dy) for x, y in h])
        lines, texts = export.pattern_annotations(r, r["label"])
        for a, b in lines:
            ln = sk.sketchCurves.sketchLines.addByTwoPoints(
                adsk.core.Point3D.create((a[0] + dx) * CM, (a[1] + dy) * CM, 0),
                adsk.core.Point3D.create((b[0] + dx) * CM, (b[1] + dy) * CM, 0))
            ln.isConstruction = True
        try:
            x, y, h, s = texts[0]
            ti = sk.sketchTexts.createInput3(s, adsk.core.ValueInput.createByReal(h * CM))
            ti.setAsMultiLine(adsk.core.Point3D.create((x + dx) * CM, (y + dy) * CM, 0),
                              adsk.core.Point3D.create((x + dx + h * len(s) * 0.7) * CM, (y + dy + h * 1.6) * CM, 0),
                              adsk.core.HorizontalAlignments.LeftHorizontalAlignment,
                              adsk.core.VerticalAlignments.BottomVerticalAlignment, 0)
            sk.sketchTexts.add(ti)
        except Exception as e:
            log("Label skipped for %s (%s)" % (r["label"], e))
        sk.isComputeDeferred = False


def _poly(sk, pts):
    lines = sk.sketchCurves.sketchLines
    first = prev = None
    n = len(pts)
    for k in range(n):
        a = prev.endSketchPoint if prev else adsk.core.Point3D.create(pts[0][0] * CM, pts[0][1] * CM, 0)
        if k == n - 1:
            b = first.startSketchPoint
        else:
            b = adsk.core.Point3D.create(pts[k + 1][0] * CM, pts[k + 1][1] * CM, 0)
        ln = lines.addByTwoPoints(a, b)
        if first is None:
            first = ln
        prev = ln
