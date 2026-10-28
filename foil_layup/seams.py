"""Seam (unwrap cut) planning for wrapped plies.

A wrapped ply covers the whole loop of at least one section and so needs a
spanwise cut before it can be flattened. Seams are spanwise lines at a constant
chord fraction on one side ("bottom:0.6"), or at the trailing/leading edge.

Rules applied by the automatic planner:
  * keep seamEdgeMargin (mm, along the surface) from the LE, the TE and every
    ply edge in the stack (ply drops, spar cap edges, one-sided ply ends);
  * keep seamStagger (mm) from the seams of the seamWindow plies either side;
  * never put two seams within 3 mm of each other anywhere in the stack;
  * prefer the seamPreferSide, then the flattest, most open position.
"""


def frac_index_for(sec, side, x):
    """Fractional vertex index of chord fraction x on a side ('top'/'bottom')."""
    m, le = sec.m, sec.le
    if side == "te":
        return 0.0
    if side == "le":
        return float(le)
    x = min(max(x, 0.0), 1.0)
    if side == "top":
        rng = range(0, le)
    else:
        rng = range(le, m)
    best, best_err = None, None
    for j in rng:
        x0 = sec.x[j]
        x1 = sec.x[(j + 1) % m]
        lo, hi = min(x0, x1), max(x0, x1)
        if lo <= x <= hi and hi > lo:
            return j + (x - x0) / (x1 - x0)
        err = abs(x0 - x)
        if best_err is None or err < best_err:
            best, best_err = float(j), err
    return best


def arc_at(sec, fj):
    j = int(fj) % sec.m
    t = fj - int(fj)
    a0 = sec.cum[j]
    a1 = sec.cum[j + 1]
    return a0 + (a1 - a0) * t


def cyc(sec, a, b):
    d = abs(a - b) % sec.length
    return min(d, sec.length - d)


def zero_crossings(sec, field):
    """Arc positions where a per-vertex field changes sign along the loop."""
    out = []
    m = sec.m
    for j in range(m):
        f0, f1 = field[j], field[(j + 1) % m]
        if (f0 >= 0) != (f1 >= 0):
            t = f0 / (f0 - f1) if f0 != f1 else 0.5
            out.append(sec.cum[j] + (sec.cum[j + 1] - sec.cum[j]) * t)
    return out


def parse_seam(spec):
    spec = (spec or "auto").strip().lower()
    if spec in ("auto", "te", "le"):
        return spec, None
    if ":" in spec:
        side, x = spec.split(":", 1)
        side = side.strip()
        if side in ("top", "bottom"):
            try:
                xv = float(x)
                if xv > 1.0:  # accept percent
                    xv /= 100.0
                return side, xv
            except ValueError:
                pass
    raise ValueError("Seam '%s' not understood (use auto, te, le, top:0.4, bottom:60)" % spec)


def plan_seams(pieces, sections, settings, log=None):
    """Assign piece.seam = {'side','x','fj':[...per section], 'ok', 'msg'} for
    every piece that wraps fully around at least one section."""
    stagger = float(settings["seamStagger"])
    margin = float(settings["seamEdgeMargin"])
    window = int(settings["seamWindow"])
    prefer = settings.get("seamPreferSide", "bottom")

    # edges every seam must avoid, per section
    edges = []
    for i, sec in enumerate(sections):
        e = [0.0, sec.cum[sec.le]]
        for p in pieces:
            f = p.fields[i]
            if f is not None:
                e += zero_crossings(sec, f)
        edges.append(e)

    wrapped = [p for p in pieces if p.needs_seam]
    placed = []  # (order, piece, arcs)
    candidates = []
    for side in ("bottom", "top"):
        for k in range(3, 98):
            candidates.append((side, k / 100.0))
    # span weight of each station (for "share of span in conflict")
    n = len(sections)
    wts = []
    for i in range(n):
        a = sections[max(i - 1, 0)].s
        b = sections[min(i + 1, n - 1)].s
        wts.append(max(b - a, 1e-6))
    wsum = sum(wts)

    def exists(p):
        return [f is None or max(f) > 0.5 for f in p.fields]

    order = 0
    conflicts = []
    for p in wrapped:
        here = exists(p)
        structural = p.kind != "ext"
        try:
            mode, xval = parse_seam(p.seam_spec)
        except ValueError as e:
            mode, xval = "auto", None
            if log:
                log(str(e))
        if mode in ("te", "le"):
            fj = [frac_index_for(sec, mode, 0) for sec in sections]
            p.seam = {"side": mode, "x": 1.0 if mode == "te" else 0.0, "fj": fj, "ok": True,
                      "msg": "%s seam (explicit)" % mode.upper()}
            if structural:
                placed.append((order, p, [arc_at(sec, f) for sec, f in zip(sections, fj)], here))
                order += 1
            continue
        cand_list = [(mode, xval)] if mode in ("top", "bottom") else candidates
        # only seams that share span with this piece can clash
        others = [(o, a, ex) for o, q, a, ex in placed] if structural else []

        best = None
        for side, x in cand_list:
            fj = [frac_index_for(sec, side, x) for sec in sections]
            arcs = [arc_at(sec, f) for sec, f in zip(sections, fj)]
            bad = 0.0
            near_sum = edge_sum = 0.0
            for i, sec in enumerate(sections):
                a = arcs[i]
                e_d = min(cyc(sec, a, e) for e in edges[i])
                near_d = any_d = 1e9
                for o, qa, ex in others:
                    if not (ex[i] and here[i]):
                        continue
                    d = cyc(sec, a, qa[i])
                    any_d = min(any_d, d)
                    if order - o <= window:
                        near_d = min(near_d, d)
                if e_d < margin or near_d < stagger or any_d < 3.0:
                    bad += wts[i]
                near_sum += wts[i] * min(near_d, 3 * stagger)
                edge_sum += wts[i] * min(e_d, 2 * margin)
            score = (near_sum + edge_sum) / wsum
            if side == prefer:
                score += 0.5 * stagger
            score -= abs(x - 0.55) * stagger * 0.5  # mild preference for mid-chord
            # a fully valid position always wins; otherwise trade the share of
            # span in conflict against spacing so seams still spread out evenly
            key = (bad == 0, score - 2.0 * stagger * bad / wsum)
            if best is None or key > best[0]:
                best = (key, side, x, fj, arcs, bad / wsum)
        _key, side, x, fj, arcs, bad_frac = best
        ok = bad_frac < 0.10  # the narrow tips can never hold every seam
        msg = "%s at %.0f%% chord" % (side, x * 100)
        if bad_frac > 0:
            msg += " (rules broken over %.0f%% of the span)" % (bad_frac * 100)
        if not ok:
            conflicts.append(p.label)
        p.seam = {"side": side, "x": x, "fj": fj, "ok": ok, "msg": msg}
        if structural:
            placed.append((order, p, arcs, here))
            order += 1
    if conflicts and log:
        log("Seams: %d of %d wrapped plies cannot meet %.0f mm stagger (+/-%d plies) and %.0f mm edge "
            "margin everywhere; they were spread as evenly as possible. Lower the stagger or window, "
            "use TE seams, or add span splits. Affected: %s"
            % (len(conflicts), len(wrapped), stagger, window, margin, ", ".join(conflicts)))
    return wrapped
