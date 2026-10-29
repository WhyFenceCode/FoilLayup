"""Pure-Python orchestration shared by the Fusion commands and the tests.

The Fusion adapter supplies raw section loops; everything that decides layer
geometry, seams and flat patterns happens here.
"""
from . import flatten, geom, model, seams


def build_sections(raw, m, chord_smoothing=0.05):
    """raw: list of (s, loop2d). Returns Section objects (skips degenerate),
    with the smoothed chord reference used by chord-% limits."""
    out = []
    for s, loop in raw:
        if loop is None or len(loop) < 8:
            continue
        try:
            out.append(model.Section(s, loop, m))
        except (ValueError, ZeroDivisionError):
            continue
    model.smooth_chord_reference(out, chord_smoothing)
    return out


def tip_schedule(stack, sections, frame):
    """Spanwise ply drops towards the tips.

    Walking out from the root, the thicker side's stack plus half the minimum
    core (tipCoreMin) must fit in the foil's half thickness. When it no longer
    does, plies end in this order: limited plies (spar caps, bands, one-sided)
    innermost first, then the other inner plies innermost first. The outermost
    ply never drops; it runs to the tip. Sets unit.tip_end (span_ref) and
    returns [(unit, end)] in drop order.
    """
    st = stack.settings
    units = stack.units
    for u in units:
        u.tip_end = None
    if not st.get("tipSchedule", True) or len(units) < 2 or len(sections) < 2:
        return []
    core_half = float(st.get("tipCoreMin", 1.0)) / 2.0
    pts = sorted((model.span_ref(sec.s, frame), sec.thickness / 2.0) for sec in sections)
    merged = []
    for sr, hv in pts:
        if merged and sr - merged[-1][0] < 0.5:
            merged[-1] = (merged[-1][0], min(merged[-1][1], hv))  # both halves must fit
        else:
            merged.append((sr, hv))
    limited = [u for u in units[1:] if u.has_user_region or u.side != "wrap"]
    others = [u for u in units[1:] if u not in limited]
    order = list(reversed(limited)) + list(reversed(others))
    present = list(units)

    def need():
        top = sum(u.t for u in present if u.side in ("wrap", "top"))
        bot = sum(u.t for u in present if u.side in ("wrap", "bottom"))
        return max(top, bot) + core_half

    ends = []
    room = float("inf")
    for (s0, h0), (s1, h1) in zip(merged, merged[1:]):
        n = max(1, int((s1 - s0) / 1.0))  # 1 mm steps
        for k in range(n + 1):
            t = k / n
            sr = s0 + (s1 - s0) * t
            room = min(room, h0 + (h1 - h0) * t)  # thickness only ever shrinks outward
            while order and need() > room:
                u = order.pop(0)
                present.remove(u)
                u.tip_end = sr
                ends.append((u, sr))
    return ends


def log_tip_schedule(ends, frame, log):
    if not ends or not log:
        return
    where = "from centre" if frame.get("symmetric", True) else "from root"
    log("Tip ply drops (|s| %s): " % where + ", ".join("%s @ %.0f mm" % (u.name, e) for u, e in ends))


def depth_tables(stack, sections, frame, bounds):
    ramp = float(stack.settings["dropRatio"])
    deps = [model.section_depths(sec, stack.units, frame, bounds, ramp) for sec in sections]
    ext = [model.external_depths(sec, stack.ext_units) for sec in sections]
    return deps, ext


def boundary_loop(sec, depth, min_gap, trim=None):
    """Offset loop for one surface at one section, ready for lofting.

    trim[j] is the depth of the medial axis clearance this surface needs at
    vertex j (default depth + min_gap); the loop ends smoothly where the
    section gets thinner than that. Trimmed loops get their tip rounded and are
    resampled to the section's point structure (TE first, LE in the middle) so
    neighbouring sections loft cleanly. Loops that are tiny or cross themselves
    are dropped (None) and the loft skips that station.
    """
    if trim is None:
        trim = [d + min_gap for d in depth]
    pts, trimmed = geom.offset_trimmed(sec.pts, sec.n, sec.h, depth, trim)
    if pts is None:
        return None
    if trimmed:
        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]
        if max(xs) - min(xs) < 8 * min_gap or max(ys) - min(ys) < 4 * min_gap:
            return None
        # keep the natural points plus the rounded cap (resampling would crush a
        # sub-millimetre cap into a spike); one light rounding pass, then start
        # at the tip so neighbouring sections line up in the loft
        pts = geom.chaikin(pts, 1)
        tip = max(range(len(pts)), key=lambda j: pts[j][0])
        pts = pts[tip:] + pts[:tip]
    if geom.signed_area(pts) <= 0 or geom.self_intersects(pts):
        return None
    return pts


def loft_plan(stack, sections, deps, ext, min_gap):
    """Section loops for every surface that has to be lofted.

    Returns dict name -> list of (s, loop2d or None) where names are
    'after:<row>', 'full:<row>' and 'ext:<row>'.

    The inner surface of row r is the outer surface of row r+1, so it is
    trimmed where row r+1's plies no longer fit (model.end_gap): row r+1 then
    ends with a blunt end and the outer layers carry on to the edge.
    """
    plan = {}
    prev = [sec.pts for sec in sections]       # surface each new one must stay inside
    prev_trim = [None] * len(sections)         # trim depth of the surface above
    for ri, row in enumerate(stack.units):
        nxt = stack.units[ri + 1] if ri + 1 < len(stack.units) else None
        gap = model.end_gap(nxt.ply_t, stack.settings, nxt.end_gap) if nxt else min_gap
        after, trims = [], []
        for i, sec in enumerate(sections):
            loop, t = _nested_loop(sec, deps[i][ri]["after"], gap, prev_trim[i], prev[i])
            after.append(loop)
            trims.append(t)
        plan["after:%s" % row.id] = [(sec.s, after[i]) for i, sec in enumerate(sections)]
        if not row.is_full:
            plan["full:%s" % row.id] = [
                (sec.s, _nested_loop(sec, deps[i][ri]["full"], gap, prev_trim[i], prev[i])[0])
                for i, sec in enumerate(sections)]
        prev = after
        prev_trim = trims
    for ri, row in enumerate(stack.ext_units):
        plan["ext:%s" % row.id] = [(sec.s, boundary_loop(sec, ext[i][ri]["after"], min_gap))
                                   for i, sec in enumerate(sections)]
    return plan


NEST_MARGIN = 0.01  # mm


TIP_STEP = 0.05  # mm of thickness between the trimmed tips of nested surfaces


def _nested_loop(sec, depth, gap, prev_trim, outer):
    """Boundary loop that sits strictly inside `outer`. If its rounded tip would
    poke past the outer surface's tip, the tip is stepped back a little more
    instead of dropping the station. Returns (loop or None, trim used)."""
    base = _trim_after(depth, gap, prev_trim)
    for extra in (0.0, 0.05, 0.15, 0.3):
        trim = [t + extra for t in base]
        loop = _nested(boundary_loop(sec, depth, gap, trim), outer)
        if loop is not None:
            return loop, trim
    return None, base


def _trim_after(depth, gap, prev_trim):
    """Trim depth for a surface: its own depth + end gap, but always at least
    TIP_STEP beyond the surface above, so the tips of nested surfaces never
    meet (near-touching tips make Fusion's booleans fail)."""
    t = [d + gap for d in depth]
    if prev_trim is None:
        return t
    return [max(a, b + TIP_STEP) for a, b in zip(t, prev_trim)]


def _nested(loop, outer):
    """Keep an inner surface's section strictly inside the one above it (the
    rounded/resampled trailing-edge tips can otherwise poke out by a hair)."""
    if loop is None:
        return None
    if outer is None:
        return None  # nothing above it at this station, so nothing to be inside
    loop = geom.clamp_inside(loop, outer, NEST_MARGIN)
    if geom.signed_area(loop) <= 0 or geom.self_intersects(loop):
        return None
    return loop


def longest_run(entries, max_gap=2):
    """Longest run of stations that have a loop. Up to max_gap missing stations
    inside a run are skipped (a dropped bad section), not treated as an end."""
    best, cur, gap = [], [], 0
    for s, loop in entries:
        if loop is not None:
            cur.append((s, loop))
            gap = 0
        else:
            gap += 1
            if gap > max_gap:
                if len(cur) > len(best):
                    best = cur
                cur = []
    if len(cur) > len(best):
        best = cur
    return best


def side_mask_quads(sec, side, extend):
    """Quad in the section plane covering the region above (top) or below
    (bottom) the chord line, extended past LE and TE."""
    le, te = sec.le_pt, sec.te_pt
    d = sec.chord_dir
    up = sec.up_dir if side == "top" else (-sec.up_dir[0], -sec.up_dir[1])
    a = (le[0] - d[0] * extend, le[1] - d[1] * extend)
    b = (te[0] + d[0] * extend, te[1] + d[1] * extend)
    c = (b[0] + up[0] * extend, b[1] + up[1] * extend)
    e = (a[0] + up[0] * extend, a[1] + up[1] * extend)
    return [a, b, c, e]


def region_planform_polys(row, sections, frame, extend, include_tip=True):
    """Planform (s, c) polygons for the row's chord/span limits (not bounds).
    Returns None if the row has neither."""
    span = row.eff_span if include_tip else row.span
    if not (row.chord or span):
        return None
    def inside_span(s):
        if not span:
            return True
        sr = model.span_ref(s, frame)
        s0, s1 = span
        return (s0 is None or sr >= s0 - 1e-6) and (s1 is None or sr <= s1 + 1e-6)

    runs, cur = [], []
    for sec in sections:
        if inside_span(sec.s):
            cur.append(sec)
        elif cur:
            runs.append(cur)
            cur = []
    if cur:
        runs.append(cur)
    polys = []
    c0, c1 = row.chord if row.chord else (None, None)
    for run in runs:
        if len(run) < 2:
            continue
        fwd, aft = [], []
        for sec in run:
            cle, cte = sec.ref_le, sec.ref_te
            cf = cle + (c0 if c0 not in (None,) and c0 > 0 else 0) * (cte - cle)
            ca = cle + (c1 if c1 not in (None,) and c1 < 1 else 1) * (cte - cle)
            if c0 is None or c0 <= 0:
                cf = min(p[0] for p in sec.pts) - extend
            if c1 is None or c1 >= 1:
                ca = max(p[0] for p in sec.pts) + extend
            fwd.append((sec.s, cf))
            aft.append((sec.s, ca))
        # extend run ends to the body ends when the run touches them
        if run[0] is sections[0]:
            fwd.insert(0, (run[0].s - extend, fwd[0][1]))
            aft.insert(0, (run[0].s - extend, aft[0][1]))
        if run[-1] is sections[-1]:
            fwd.append((run[-1].s + extend, fwd[-1][1]))
            aft.append((run[-1].s + extend, aft[-1][1]))
        polys.append(fwd + list(reversed(aft)))
    return polys


# --------------------------------------------------------------------------- previews
def section_preview(stack, sec, frame, bounds):
    """Loops for drawing one section in the palette."""
    deps = model.section_depths(sec, stack.units, frame, bounds, float(stack.settings["dropRatio"]))
    ext = model.external_depths(sec, stack.ext_units)
    mg = float(stack.settings["minGap"])
    rows = stack.units
    out = {"s": sec.s, "outer": sec.pts, "chord": sec.chord, "thickness": sec.thickness, "rows": [], "ext": []}
    for rd in deps:
        row = rd["row"]
        out["rows"].append({
            "id": row.id,
            "name": row.name,
            "color": row.material["color"],
            "after": boundary_loop(sec, rd["after"], _next_gap(stack, rows, row, mg)),
            "presence": rd["presence"],
        })
    for rd in ext:
        out["ext"].append({"id": rd["row"].id, "name": rd["row"].name, "color": rd["row"].material["color"],
                           "after": boundary_loop(sec, rd["after"], mg)})
    return out


def _next_gap(stack, rows, row, min_gap):
    k = rows.index(row)
    return model.end_gap(rows[k + 1].ply_t, stack.settings, rows[k + 1].end_gap) if k + 1 < len(rows) else min_gap


def stack_summary(stack):
    """Thickness totals and areal mass per side (nominal, no drops)."""
    top = bottom = 0.0
    rows = []
    for r in stack.rows:
        if r.side in ("wrap", "top"):
            top += r.t
        if r.side in ("wrap", "bottom"):
            bottom += r.t
        rows.append({"id": r.id, "name": r.name, "t": r.t, "side": r.side})
    ext = sum(r.t for r in stack.external)
    return {"top": top, "bottom": bottom, "external": ext, "rows": rows}


# --------------------------------------------------------------------------- flatten pipeline
def plan_pieces(stack, sections, frame, bounds, log=None, seam_plan=None):
    """Pieces with seams. seam_plan (piece id -> 'bottom:0.62') locks seams that
    were planned earlier (e.g. at 3D generation) for plies left on 'auto'."""
    deps, ext = depth_tables(stack, sections, frame, bounds)
    pieces = flatten.build_pieces(stack, sections, deps, ext, frame, stack.settings)
    for pc in pieces:
        if seam_plan and pc.seam_spec == "auto" and pc.id in seam_plan:
            pc.seam_spec = seam_plan[pc.id]
    seams.plan_seams(pieces, sections, stack.settings, log)
    return pieces, deps, ext


def seam_plan_of(pieces):
    out = {}
    for pc in pieces:
        if pc.seam:
            side = pc.seam["side"]
            out[pc.id] = side if side in ("te", "le") else "%s:%.4f" % (side, pc.seam["x"])
    return out


def flatten_all(stack, sections, frame, bounds, keepout_fn=None, progress=None, log=None, seam_plan=None):
    pieces, deps, ext = plan_pieces(stack, sections, frame, bounds, log, seam_plan)
    settings = dict(stack.settings)
    settings["_hand"] = frame.get("hand", 1)
    results = []
    for n, pc in enumerate(pieces):
        if progress and not progress(n, len(pieces), pc.label):
            break
        parts = flatten.flatten_piece(pc, sections, settings, keepout_fn)
        for k, part in enumerate(parts):
            label = "%s %s" % (pc.id, pc.label) + ("" if len(parts) == 1 else " part %d" % (k + 1))
            mat = pc.material
            gsm = mat.get("gsm") or 0
            area_m2 = part["area"] / 1e6
            part.update({
                "piece": pc,
                "label": label,
                "file": "%s_%s" % (pc.id, mat["id"]) + ("" if len(parts) == 1 else "_p%d" % (k + 1)),
                "color": mat["color"],
                "book": {
                    "piece": pc.id + ("" if len(parts) == 1 else "-p%d" % (k + 1)),
                    "layer": pc.row.name if pc.row else "",
                    "kind": pc.kind,
                    "material": mat["name"],
                    "angle_deg": pc.angle,
                    "seam": pc.seam["msg"] if pc.seam else "none",
                    "area_m2": round(area_m2, 5),
                    "dry_fabric_g": round(area_m2 * gsm, 1),
                    # cured laminate (fibre + resin at the material's Vf): area x thickness x density
                    "with_resin_g": round(area_m2 * (pc.ply.t if pc.ply else pc.row.t) * mat["density"], 1),
                    "length_mm": round(part["bbox"][2] - part["bbox"][0], 1),
                    "width_mm": round(part["bbox"][3] - part["bbox"][1], 1),
                    "p95_strain_pct": round(part["stats"]["maxStrain"] * 100, 2),
                    "rms_strain_pct": round(part["stats"]["rmsStrain"] * 100, 3),
                    "p95_shear_deg": round(part["stats"].get("maxShear", 0.0), 1),
                    "mean_shear_deg": round(part["stats"].get("meanShear", 0.0), 2),
                    "holes": len(part["holes"]),
                },
            })
            results.append(part)
    return results, pieces
