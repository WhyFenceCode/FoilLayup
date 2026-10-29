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


def tip_schedule(stack, sections, frame, bounds=None, zone=(0.1, 0.75)):
    """Spanwise ply drops towards the tips.

    Walking out from the root, at every point across the structural part of
    the chord (zone, as chord fractions) the stack actually present there
    (each ply's band, side and bound respected) plus half the minimum core
    (tipCoreMin) must fit in the local half thickness. Where it does not, the
    first ply in the drop order that covers that point ends: limited plies
    (spar caps, bands, one-sided) innermost first, then the other inner plies
    innermost first. The outermost ply never drops. The end is interpolated
    between stations from the remaining clearance. Sets unit.tip_end
    (span_ref) and returns [(unit, end)] in drop order.
    """
    st = stack.settings
    units = stack.units
    for u in units:
        u.tip_end = None
    if not st.get("tipSchedule", True) or len(units) < 2 or len(sections) < 2:
        return []
    bounds = bounds or {}
    core_half = float(st.get("tipCoreMin", 1.0)) / 2.0
    limited = [u for u in units[1:] if u.has_user_region or u.side != "wrap"]
    others = [u for u in units[1:] if u not in limited]
    order = list(reversed(limited)) + list(reversed(others))

    # per station: zone vertices, their half thickness and which plies cover them
    # Checked points: wherever a limited ply (cap, band, one-sided) lies, and
    # the thickest point of each surface. Thin regions towards the LE/TE are
    # left to the chordwise ply stop rule, they must not end plies spanwise.
    stations = []
    for sec in sections:
        zone_js = [j for j in range(sec.m) if zone[0] <= sec.x[j] <= zone[1]]
        if not zone_js:
            continue
        pres = {}
        for u in units:
            _fs, _fr, presence = model.row_fields(u, sec, frame, bounds)
            pres[u.id] = presence
        js = set()
        for u in limited:
            p_ = pres[u.id]
            js.update(j for j in zone_js if p_ is None or p_[j] >= 0)
        top = [j for j in zone_js if j < sec.le]
        bot = [j for j in zone_js if j > sec.le]
        for side_js in (top, bot):
            if side_js:
                js.add(max(side_js, key=lambda j: sec.h[j]))
        js = sorted(js)
        cover = {u.id: [pres[u.id] is None or pres[u.id][j] >= 0 for j in js] for u in units}
        stations.append((model.span_ref(sec.s, frame), [sec.h[j] for j in js], cover))
    stations.sort(key=lambda x: x[0])

    def margins(station, present):
        """Clearance per zone vertex (half thickness - stack - half core)."""
        _sr, hs, cover = station
        out = []
        for k, hk in enumerate(hs):
            depth = sum(u.t for u in present if cover[u.id][k])
            out.append(hk - depth - core_half)
        return out

    present = list(units)
    ends = []
    prev = None
    for station in stations:
        while order:
            m1 = margins(station, present)
            worst = min(range(len(m1)), key=lambda k: m1[k])
            if m1[worst] >= 0:
                break
            cover = station[2]
            u = next((v for v in order if cover[v.id][worst]), None)
            if u is None:
                break  # only the outermost ply is left at that point
            order.remove(u)
            end = station[0]
            if prev is not None:
                m0 = min(margins(prev, present))
                if m0 > 0:  # interpolate where the clearance ran out
                    end = prev[0] + (station[0] - prev[0]) * m0 / (m0 - m1[worst])
            present.remove(u)
            u.tip_end = end
            ends.append((u, end))
        prev = station
    # stagger: no two plies end at the same place; an earlier drop moves inward
    gap = float(st.get("tipDropSpacing", 10.0))
    if gap > 0:
        for k in range(len(ends) - 2, -1, -1):
            u, e = ends[k]
            nxt = ends[k + 1][1]
            if e > nxt - gap:
                e = nxt - gap
                u.tip_end = e
                ends[k] = (u, e)
    return ends


def drop_stations(stack, frame):
    """Span positions to add as sections so lofts and patterns follow every
    ply drop tightly: each ply end and the end of its drape ramp (otherwise
    the inner layers drape over a whole station gap, leaving a long void)."""
    sym = frame.get("symmetric", True)
    ratio = float(stack.settings["dropRatio"])
    out = []

    def add(sr):
        if sr is None or sr < 0:
            return
        out.extend([sr, -sr] if sym else [sr + frame["sMin"]])

    for u in stack.units:
        ramp = max(ratio * u.t, 1.5)
        if u.tip_end is not None:
            add(u.tip_end)
            add(u.tip_end + ramp)
        if u.span:
            s0, s1 = u.span
            if s0 is not None:
                add(s0 - ramp)
            if s1 is not None:
                add(s1 + ramp)
    return out


def mass_breakdown(rows):
    """Laminate mass by material and by fibre, dry and with resin.

    rows: dicts with material, fibre, dry_g, mass_g (plies and fills only).
    byFibre only lists a fibre that has more than one material (e.g. carbon
    twill + carbon UD); a single material already is its fibre total.
    """
    mats, order = {}, []
    for r in rows:
        key = r["material"]
        if key not in mats:
            mats[key] = {"material": key, "fibre": r.get("fibre") or "", "dry_g": 0.0, "wet_g": 0.0, "plies": 0}
            order.append(key)
        m = mats[key]
        m["dry_g"] += r.get("dry_g") or 0.0
        m["wet_g"] += r.get("mass_g") or 0.0
        m["plies"] += 1
    by_mat = [dict(mats[k], dry_g=round(mats[k]["dry_g"], 1), wet_g=round(mats[k]["wet_g"], 1)) for k in order]
    fib = {}
    for m in by_mat:
        if not m["fibre"]:
            continue
        f = fib.setdefault(m["fibre"], {"fibre": m["fibre"], "dry_g": 0.0, "wet_g": 0.0, "materials": 0})
        f["dry_g"] += m["dry_g"]
        f["wet_g"] += m["wet_g"]
        f["materials"] += 1
    by_fib = [dict(f, dry_g=round(f["dry_g"], 1), wet_g=round(f["wet_g"], 1))
              for f in fib.values() if f["materials"] > 1]
    return {"byMaterial": by_mat, "byFibre": by_fib}


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
    """Section of one layer surface at one station, ready for lofting.

    trim[j] is the clearance this surface needs at vertex j (default depth +
    min_gap); the surface stops where the section gets thinner than that.
    Returns points from the upper stop point round the LE to the lower stop
    point (2n-1 points, the LE in the middle); the section is closed by the
    straight line from the last point back to the first. None if nothing fits.
    """
    if trim is None:
        trim = [d + min_gap for d in depth]
    chains = geom.offset_chains(sec.pts, sec.n, sec.h, depth, trim, sec.le)
    if chains is None:
        return None
    upper, lower = chains
    # Same make-up at every station, like the foil's own sections: each chain
    # resampled to the same number of points with the same spacing (dense at
    # the LE and at the stopped end), so lofts match point to point.
    n = max(8, sec.m // 2)
    up = geom.resample_open(upper, n)
    lo = geom.resample_open(lower, n)
    pts = list(reversed(up)) + lo[1:]   # upper stop -> LE -> lower stop
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    if max(xs) - min(xs) < 8 * min_gap or max(ys) - min(ys) < 2 * min_gap:
        return None
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
            rd = deps[i][ri]
            if ri > 0 and max(rd["a"]) <= 0.0 and prev[i] is not None:
                # the ply is absent at this station: its inner surface is the
                # surface above, point for point
                after.append(prev[i])
                trims.append(prev_trim[i])
                continue
            loop, t = _nested_loop(sec, rd["after"], gap, prev_trim[i], prev[i])
            after.append(loop)
            trims.append(t)
        plan["after:%s" % row.id] = [(sec.s, after[i]) for i, sec in enumerate(sections)]
        if not row.uses_inner_surface:
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


def simplify_run(run, tol=0.05):
    """Drop stations a surface does not need. A loft is a smooth curve through
    its sections, so a section is redundant when a cubic through the nearest
    kept sections on each side (two per side where available) reproduces it
    to within tol (mm) at every point. Sections of one surface share their
    point layout, so this is an exact point-by-point check. Ends are kept.
    run: [(s, loop)] -> subset."""
    n = len(run)
    if n <= 4:
        return run
    kept = list(range(n))
    ss = [r[0] for r in run]
    loops = [r[1] for r in run]

    def predicted_dev(k, nbrs):
        if any(len(loops[j]) != len(loops[k]) for j in nbrs):
            return float("inf")
        x = ss[k]
        w = []
        for a in nbrs:  # Lagrange weights at x
            wa = 1.0
            for b in nbrs:
                if b != a:
                    wa *= (x - ss[b]) / (ss[a] - ss[b])
            w.append(wa)
        # distance from each predicted point to the section's actual curve (its
        # nearby segments), so points sliding along the curve as a ply stop
        # moves do not count as a shape change
        lk = loops[k]
        m = len(lk)
        worst = 0.0
        for i in range(m):
            qx = qy = 0.0
            for wa, a in zip(w, nbrs):
                qx += wa * loops[a][i][0]
                qy += wa * loops[a][i][1]
            seg = lk[max(0, i - 6):min(m, i + 7)]
            d = geom.min_dist_polyline(qx, qy, seg)
            if d > worst:
                worst = d
                if worst > tol:
                    break
        return worst

    changed = True
    while changed and len(kept) > 4:
        changed = False
        pos = 1
        while pos < len(kept) - 1:
            left = kept[max(0, pos - 2):pos]
            right = kept[pos + 1:pos + 3]
            if predicted_dev(kept[pos], left + right) <= tol:
                del kept[pos]
                changed = True
                pos += 1  # leave the next one for the next pass (no two in a row)
            else:
                pos += 1
    return [run[k] for k in kept]


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
    """Loops for drawing one section in the palette, built exactly like the
    loft plan (same trimming, nesting and tip ends). Call tip_schedule on the
    span sections first so plies that have ended here are shown as ended."""
    deps = [model.section_depths(sec, stack.units, frame, bounds, float(stack.settings["dropRatio"]))]
    ext = [model.external_depths(sec, stack.ext_units)]
    mg = float(stack.settings["minGap"])
    plan = loft_plan(stack, [sec], deps, ext, mg)
    sr = model.span_ref(sec.s, frame)
    out = {"s": sec.s, "outer": sec.pts, "chord": sec.chord, "thickness": sec.thickness, "rows": [], "ext": []}
    for rd in deps[0]:
        row = rd["row"]
        ended = row.tip_end is not None and sr > row.tip_end
        out["rows"].append({
            "id": row.id,
            "name": row.name + (" (ended at %.0f)" % row.tip_end if ended else ""),
            "color": row.material["color"],
            "after": plan["after:%s" % row.id][0][1],
            "presence": rd["presence"],
            "ended": ended,
        })
    for row in stack.ext_units:
        out["ext"].append({"id": row.id, "name": row.name, "color": row.material["color"],
                           "after": plan["ext:%s" % row.id][0][1]})
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
