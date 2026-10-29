"""Flat pattern generation (pure Python).

Each ply's mid-surface is sampled as a structured grid (span stations x loop
vertices), cut open at its seam, flattened by length-preserving relaxation
(edges plus both quad diagonals) and clipped to the ply outline with marching
squares on a signed field (side / region / span segment / seam / keep-outs).
"""
import math

from . import geom, model


class Piece:
    """One flat pattern source: a ply, a fill or an external layer, optionally
    limited to a span segment."""

    def __init__(self, kind, row, ply, seg, label, material, angle, depth_index):
        self.kind = kind
        self.row = row
        self.ply = ply
        self.seg = seg
        self.label = label
        self.material = material
        self.angle = angle
        self.depth_index = depth_index  # row index into the depth list
        self.fields = []   # per section: signed field (or None = everywhere)
        self.depths = []   # per section: mid-surface depth per vertex
        self.needs_seam = False
        self.seam = None
        self.seam_spec = row.seam if row else "auto"
        self.id = ""


def _segments(row, symmetric):
    if not row.splits:
        return [None]
    cuts = set()
    for v in row.splits:
        cuts.add(v)
        if symmetric:
            cuts.add(-v)
    cuts = sorted(cuts)
    bounds = [-math.inf] + cuts + [math.inf]
    return [(bounds[k], bounds[k + 1]) for k in range(len(bounds) - 1)]


def _seg_field(seg, s, overlap):
    if seg is None:
        return math.inf
    lo, hi = seg
    return min(s - lo + overlap / 2.0, hi - s + overlap / 2.0)


def build_pieces(stack, sections, depths, ext_depths, frame, settings):
    """Create pieces with per-section fields and mid-surface depths."""
    overlap = float(settings.get("seamOverlap", 0.0))
    min_gap = float(settings.get("minGap", 0.15))
    pieces = []
    first = True  # the outermost body ply runs to the edge; inner plies may end before it
    for ri, row in enumerate(stack.units):
        segs = _segments(row, frame.get("symmetric", True))
        units = [("ply", p, p.angle, row.material, p.name, p.index) for p in row.plies]
        if row.outside and row.outside != "core":
            mat = stack.materials[row.outside]
            if mat["category"] != "core":
                units.append(("fill", None, 0.0, mat, "%s fill (%s)" % (row.name, mat["name"]), None))
        for kind, ply, ang, mat, label, k in units:
            for si, seg in enumerate(segs):
                pc = Piece(kind, row, ply, seg, label + ("" if seg is None else " seg %d" % (si + 1)),
                           mat, ang, ri)
                pc.id = (ply.id if ply else row.id + ".fill") + ("" if seg is None else "-s%d" % (si + 1))
                for i, sec in enumerate(sections):
                    rd = depths[i][ri]
                    fs, fr = rd["side"], rd["region"]
                    if kind == "fill":
                        parts = [x for x in (fs, [-v for v in fr] if fr else None) if x is not None]
                    else:
                        parts = [x for x in (fs, fr) if x is not None]
                    if kind == "fill":
                        outer = rd["before"]
                        gap = model.end_gap(row.t, settings, row.end_gap)
                    else:
                        outer = [d + row.ply_t * k for d in rd["before"]]
                        gap = model.end_gap(row.ply_t, settings, row.end_gap)
                    if not (first and kind == "ply" and k == 0):
                        # stop where this ply no longer fits inside the layers above it
                        parts.append(_room_field(sec, [sec.h[j] - outer[j] - gap for j in range(sec.m)]))
                    f = None
                    if parts:
                        f = [min(vals) for vals in zip(*parts)]
                    sf = _seg_field(seg, sec.s, overlap)
                    if sf != math.inf:
                        f = [min(v, sf) for v in f] if f else [sf] * sec.m
                    pc.fields.append(f)
                    if kind == "fill":
                        d = [rd["before"][j] + row.t / 2.0 for j in range(sec.m)]
                    else:
                        d = [rd["before"][j] + row.ply_t * (k + 0.5) for j in range(sec.m)]
                    pc.depths.append([max(0.0, min(d[j], sec.h[j] - min_gap)) for j in range(sec.m)])
                pieces.append(pc)
        first = False
    for ri, row in enumerate(stack.ext_units):
        for p in row.plies:
            pc = Piece("ext", row, p, None, p.name, row.material, p.angle, ri)
            pc.id = p.id
            for i, sec in enumerate(sections):
                rd = ext_depths[i][ri]
                pc.fields.append(None)
                pc.depths.append([rd["before"][j] - row.ply_t * (p.index + 0.5) for j in range(sec.m)])
            pieces.append(pc)
    for pc in pieces:
        pc.needs_seam = any(f is None or min(f) >= 0 for f in pc.fields)
    return pieces


def _room_field(sec, f):
    """Ply-end field that matches the 3D sections (geom.offset_chains): the
    ply wraps the nose even where the small nose radius fails the room test,
    and on each side it runs out from the nose until the first point without
    room, then stops for good. Positive = ply present."""
    m, le = sec.m, sec.le
    g = [-abs(v) - 1e-3 for v in f]  # absent unless reached from the nose
    for step in (-1, 1):
        j = le
        # the nose zone (geom.NOSE_X of chord) is covered: a small nose radius
        # failing the room test there never ends a ply
        while 0 < j < m and sec.x[j] < geom.NOSE_X:
            g[j] = max(f[j], 1e-3)
            j += step
        while 0 < j < m:
            if f[j] < 0:
                break        # first point without room: the ply stops here
            g[j] = f[j]
            j += step
    g[le] = max(g[le], 1e-3)
    return g


# --------------------------------------------------------------------------- strip grid
def _mid_point(sec, j, d):
    p, n = sec.pts[j], sec.n[j]
    return (sec.s, p[0] + n[0] * d, p[1] + n[1] * d)


def build_strip(piece, sections, overlap):
    """Return (X, G, info) where X[i][k] are 3D nodes and G[i][k] the clip field."""
    m = sections[0].m
    # choose where the strip is cut open
    if piece.needs_seam and piece.seam:
        fj = piece.seam["fj"]
        j0 = int(math.floor(min(fj))) - 1
        # wrap-around guard: if seams straddle index 0 use the unwrapped values
        if max(fj) - min(fj) > m / 2:
            fj = [f + m if f < m / 2 else f for f in fj]
            j0 = int(math.floor(min(fj))) - 1
        extra_arc = overlap
    else:
        # cut at the column that is least active over all sections
        best, j0 = None, 0
        for j in range(m):
            v = max((f[j] if f is not None else 1e9) for f in piece.fields)
            if best is None or v < best:
                best, j0 = v, j
        fj = None
        extra_arc = 0.0
    ncol = m + 1
    if fj is not None:
        span_needed = max(fj) - min(fj) + 3
        ncol = m + int(math.ceil(span_needed))
        if extra_arc > 0:
            # enough columns to cover the overlap on the shortest section
            minstep = min(min(sec.cum[j + 1] - sec.cum[j] for j in range(m)) for sec in sections)
            ncol += int(math.ceil(extra_arc / max(minstep, 1e-3))) + 1
            ncol = min(ncol, 2 * m)
    X, G, A = [], [], []
    for i, sec in enumerate(sections):
        row_x, row_g, row_a = [], [], []
        f = piece.fields[i]
        d = piece.depths[i]
        acc = 0.0
        prev = None
        for k in range(ncol):
            j = (j0 + k) % m
            p = _mid_point(sec, j, d[j])
            if prev is not None:
                acc += geom.dist(p, prev)
            prev = p
            row_x.append(p)
            row_a.append(acc)
            row_g.append(f[j] if f is not None else math.inf)
        if fj is not None:
            # arc of the seam inside this strip row
            col = fj[i] - j0
            k0 = int(col)
            t = col - k0
            k0 = max(0, min(ncol - 2, k0))
            a_seam = row_a[k0] + (row_a[k0 + 1] - row_a[k0]) * t
            L = _loop_len_mid(sec, d)
            for k in range(ncol):
                rel = row_a[k] - a_seam
                row_g[k] = min(row_g[k], rel, L + extra_arc - rel)
        X.append(row_x)
        G.append(row_g)
        A.append(row_a)
    return X, G, {"j0": j0, "ncol": ncol}


def _loop_len_mid(sec, d):
    pts = [_mid_point(sec, j, d[j]) for j in range(sec.m)]
    return geom.loop_length(pts, closed=True)


# --------------------------------------------------------------------------- flatten
OMEGA = 1.7  # successive over-relaxation factor
W_SHEAR = 0.05  # weak shear stiffness (fabric trellises, but is not a free mechanism)


def fibre_model_for(material):
    """'woven' (two orthogonal fibre sets), 'ud' or 'paper' (isotropic)."""
    if material.get("category") in ("process", "mat"):
        return "paper"
    if material.get("form") == "UD":
        return "ud"
    return "woven"


def flatten_grid(X, G, iterations=250, tol=1e-4, angle_deg=0.0, fibre_model="woven", hand=1):
    """Flatten the relevant part of a 3D grid like a fabric: edges along the
    fibre directions are stiff, edges across them shear freely (pin-jointed
    net behaviour); 'paper' keeps every edge length.

    Returns (U, active, stats) with fibre strain and shear angle statistics."""
    ni, nk = len(X), len(X[0])
    # cell size estimate for the relevance margin
    cell = 0.0
    cnt = 0
    for i in range(0, ni - 1, max(1, ni // 8)):
        for k in range(0, nk - 1, max(1, nk // 16)):
            cell = max(cell, geom.dist(X[i][k], X[i + 1][k]), geom.dist(X[i][k], X[i][k + 1]))
            cnt += 1
    margin = 1.5 * cell
    cells = []
    for i in range(ni - 1):
        for k in range(nk - 1):
            g = max(G[i][k], G[i][k + 1], G[i + 1][k], G[i + 1][k + 1])
            if g > -margin:
                cells.append((i, k))
    if not cells:
        return None, None, None
    active = set()
    for i, k in cells:
        active.update(((i, k), (i, k + 1), (i + 1, k), (i + 1, k + 1)))

    # initial layout: unroll each row by arc length about a reference column
    cols = sorted({k for _, k in active})
    kref = cols[len(cols) // 2]
    U = {}
    y = 0.0
    for i in range(ni):
        if i > 0:
            y += geom.dist(X[i][kref], X[i - 1][kref])
        x = 0.0
        row = {kref: 0.0}
        for k in range(kref + 1, nk):
            x += geom.dist(X[i][k], X[i][k - 1])
            row[k] = x
        x = 0.0
        for k in range(kref - 1, -1, -1):
            x -= geom.dist(X[i][k], X[i][k + 1])
            row[k] = x
        for k in range(nk):
            if (i, k) in active:
                U[(i, k)] = [row[k], y]

    # Constraints are linear combinations d = sum(g_m * x_m) of three nodes whose
    # length must equal L0. Plain edges use g = (-1, 1, 0).
    idx = {key: n for n, key in enumerate(sorted(active))}
    keys = sorted(active)
    ux = [U[key][0] for key in keys]
    uy = [U[key][1] for key in keys]
    cons = []
    seen = set()
    th = math.radians(angle_deg)
    cth, sth = math.cos(th), math.sin(th)

    def P(n):
        return X[n[0]][n[1]]

    def cell_frame(a, b, d):
        """Tangent frame (span dir, in-plane perpendicular, outward normal)."""
        t_s = geom.unit(geom.sub(P(d), P(a)))
        t_c = geom.unit(geom.sub(P(b), P(a)))
        n = geom.unit(geom.cross3(t_s, t_c))
        hint = (0.0, t_c[2], -t_c[1])  # outward for CCW section columns
        if geom.dot(n, hint) * hand < 0:
            n = geom.scale(n, -1.0)
        t_p = geom.cross3(n, t_s)
        return t_s, t_p, n

    def add_edge(a, b, rest=None):
        if a in idx and b in idx:
            e = (idx[a], idx[b])
            if e not in seen:
                seen.add(e)
                L0 = rest if rest is not None else geom.dist(P(a), P(b))
                cons.append((e[0], e[1], e[1], -1.0, 1.0, 0.0, L0, 1.0))

    def add_chords(tri, frame):
        """Warp/weft (and weak shear) chords through a planar triangle."""
        if not all(t in idx for t in tri):
            return
        p3 = [P(t) for t in tri]
        nt = geom.unit(geom.cross3(geom.sub(p3[1], p3[0]), geom.sub(p3[2], p3[0])))
        if geom.norm(nt) < 0.5:
            return
        if geom.dot(nt, frame[2]) < 0:
            nt = geom.scale(nt, -1.0)
        ts = geom.sub(frame[0], geom.scale(nt, geom.dot(frame[0], nt)))
        ts = geom.unit(ts)
        tp = geom.cross3(nt, ts)
        q = [(geom.dot(geom.sub(p, p3[0]), ts), geom.dot(geom.sub(p, p3[0]), tp)) for p in p3]
        gx = sum(v[0] for v in q) / 3.0
        gy = sum(v[1] for v in q) / 3.0
        dirs = [(0.0, 1.0)]
        if fibre_model == "woven":
            dirs.append((90.0, 1.0))
        else:
            dirs.append((90.0, W_SHEAR))
        dirs += [(45.0, W_SHEAR), (-45.0, W_SHEAR)]
        for rot, w in dirs:
            ang = th + math.radians(rot)
            ux_, uy_ = math.cos(ang), math.sin(ang)
            hits = []
            for e in range(3):
                i0, i1 = e, (e + 1) % 3
                ex, ey = q[i1][0] - q[i0][0], q[i1][1] - q[i0][1]
                den = ux_ * (-ey) - uy_ * (-ex)
                if abs(den) < 1e-12:
                    continue
                rx, ry = q[i0][0] - gx, q[i0][1] - gy
                t = (rx * (-ey) - ry * (-ex)) / den
                s = (ux_ * ry - uy_ * rx) / den
                if -1e-9 <= s <= 1 + 1e-9:
                    bc = [0.0, 0.0, 0.0]
                    bc[i0] += 1 - s
                    bc[i1] += s
                    hits.append((t, bc))
            if len(hits) < 2:
                continue
            hits.sort(key=lambda h: h[0])
            (t0, b0), (t1, b1) = hits[0], hits[-1]
            L0 = t1 - t0
            if L0 < 1e-6:
                continue
            g = [b1[m] - b0[m] for m in range(3)]
            n_ = [idx[t] for t in tri]
            cons.append((n_[0], n_[1], n_[2], g[0], g[1], g[2], L0, w))

    frames = {}
    for i, k in cells:
        a, b, c, d = (i, k), (i, k + 1), (i + 1, k + 1), (i + 1, k)
        fr = cell_frame(a, b, d)
        frames[(i, k)] = fr
        if fibre_model == "paper":
            add_edge(a, b)
            add_edge(d, c)
            add_edge(a, d)
            add_edge(b, c)
            add_edge(a, c)
            # The other diagonal gets its length with triangles (a,b,c) and (a,c,d)
            # hinged flat about a-c, so folds (sharp TE, tight LE) along grid lines
            # unroll isometrically instead of being treated as stretch.
            add_edge(b, d, _hinged_diagonal(P(a), P(b), P(c), P(d)))
        else:
            # triangles never straddle a fold because folds run along grid lines
            add_chords((a, b, c), fr)
            add_chords((a, c, d), fr)

    it_done = 0
    rev = list(reversed(cons))
    for it in range(iterations):
        worst = 0.0
        # symmetric sweeps with over-relaxation converge much faster on long strips
        om = OMEGA if it < iterations - 10 else 1.0
        for n0, n1, n2, g0, g1, g2, L0, w in (cons if it % 2 == 0 else rev):
            dx = g0 * ux[n0] + g1 * ux[n1] + g2 * ux[n2]
            dy = g0 * uy[n0] + g1 * uy[n1] + g2 * uy[n2]
            L = math.sqrt(dx * dx + dy * dy)
            if L < 1e-12:
                continue
            err = L - L0
            if w >= 1.0 and abs(err) > worst:
                worst = abs(err)
            if n1 == n2:
                den = g0 * g0 + g1 * g1
            else:
                den = g0 * g0 + g1 * g1 + g2 * g2
            s = om * w * err / (L * den)
            sx, sy = dx * s, dy * s
            ux[n0] -= g0 * sx
            uy[n0] -= g0 * sy
            ux[n1] -= g1 * sx
            uy[n1] -= g1 * sy
            if n2 != n1:
                ux[n2] -= g2 * sx
                uy[n2] -= g2 * sy
        it_done = it + 1
        if worst < tol:
            break

    # fibre strain and shear per cell from the deformation gradient,
    # area weighted over cells that lie inside the ply outline
    fs_sq, w_sum = 0.0, 0.0
    shear_sum = 0.0
    strain_samples, shear_samples = [], []  # (value, area) for percentiles
    for (i, k), (t_s, t_p, _n) in frames.items():
        a, b, d = (i, k), (i, k + 1), (i + 1, k)
        if min(G[i][k], G[i][k + 1], G[i + 1][k], G[i + 1][k + 1]) < 0:
            continue
        if not (a in idx and b in idx and d in idx):
            continue
        e1 = geom.sub(P(b), P(a))
        e2 = geom.sub(P(d), P(a))
        A = ((geom.dot(e1, t_s), geom.dot(e2, t_s)), (geom.dot(e1, t_p), geom.dot(e2, t_p)))
        det = A[0][0] * A[1][1] - A[0][1] * A[1][0]
        area = abs(det)
        if area < 1e-6:
            continue
        inv = ((A[1][1] / det, -A[0][1] / det), (-A[1][0] / det, A[0][0] / det))
        ia, ib, id_ = idx[a], idx[b], idx[d]
        B = ((ux[ib] - ux[ia], ux[id_] - ux[ia]), (uy[ib] - uy[ia], uy[id_] - uy[ia]))
        F = [[B[r][0] * inv[0][c] + B[r][1] * inv[1][c] for c in range(2)] for r in range(2)]
        wv = (F[0][0] * cth + F[0][1] * sth, F[1][0] * cth + F[1][1] * sth)
        fv = (-F[0][0] * sth + F[0][1] * cth, -F[1][0] * sth + F[1][1] * cth)
        lw, lf = math.hypot(*wv), math.hypot(*fv)
        if lw < 1e-12 or lf < 1e-12:
            continue
        strains = [lw - 1.0] + ([] if fibre_model == "ud" else [lf - 1.0])
        for s_ in strains:
            fs_sq += area * s_ * s_
            strain_samples.append((abs(s_), area))
        w_sum += area * len(strains)
        cosang = max(-1.0, min(1.0, (wv[0] * fv[0] + wv[1] * fv[1]) / (lw * lf)))
        sh = abs(90.0 - math.degrees(math.acos(cosang)))
        shear_samples.append((sh, area))
        shear_sum += area * sh
    n_area = w_sum / (1 if fibre_model == "ud" else 2) if w_sum else 0.0
    stats = {
        "maxStrain": _percentile(strain_samples, 0.95),
        "rmsStrain": math.sqrt(fs_sq / w_sum) if w_sum else 0.0,
        "maxShear": _percentile(shear_samples, 0.95),
        "meanShear": shear_sum / n_area if n_area else 0.0,
        "iterations": it_done,
        "model": fibre_model,
    }
    out = {key: (ux[n], uy[n]) for key, n in idx.items()}
    return out, active, stats


def _percentile(samples, q):
    """Area-weighted percentile; ignores the odd sliver cell at a fold or tip."""
    if not samples:
        return 0.0
    samples = sorted(samples)
    total = sum(w for _, w in samples)
    acc = 0.0
    for v, w in samples:
        acc += w
        if acc >= q * total:
            return v
    return samples[-1][0]


def _hinged_diagonal(a, b, c, d):
    """Distance b-d after unfolding triangles (a,b,c) and (a,c,d) into a plane."""
    L = geom.dist(a, c)
    if L < 1e-12:
        return geom.dist(b, d)

    def place(p, side):
        r0 = geom.dist(a, p)
        r1 = geom.dist(c, p)
        x = (r0 * r0 - r1 * r1 + L * L) / (2 * L)
        y = math.sqrt(max(r0 * r0 - x * x, 0.0))
        return x, side * y

    bx, by = place(b, -1)
    dx, dy = place(d, 1)
    return math.hypot(bx - dx, by - dy)


# --------------------------------------------------------------------------- contour
def contour(U, G, active):
    """Marching squares of G >= 0 over the flattened grid. Returns closed loops.
    The grid is padded with a negative ring so loops always close."""
    if not active:
        return []
    ii = [i for i, _ in active]
    kk = [k for _, k in active]
    i0, i1, k0, k1 = min(ii), max(ii), min(kk), max(kk)

    def val(i, k):
        if (i, k) not in active or i < i0 or i > i1 or k < k0 or k > k1:
            return -1.0
        g = G[i][k]
        return -1e-9 if g == 0 else g

    def pos(i, k):
        ic = min(max(i, i0), i1)
        kc = min(max(k, k0), k1)
        if (ic, kc) in U:
            return U[(ic, kc)]
        # padding nodes outside the active set: nearest active neighbour
        for di in (0, -1, 1):
            for dk in (0, -1, 1):
                if (ic + di, kc + dk) in U:
                    return U[(ic + di, kc + dk)]
        return (0.0, 0.0)

    def cross(a, b):
        va, vb = val(*a), val(*b)
        t = va / (va - vb) if va != vb else 0.5
        t = min(max(t, 0.0), 1.0)
        pa, pb = pos(*a), pos(*b)
        return (pa[0] + (pb[0] - pa[0]) * t, pa[1] + (pb[1] - pa[1]) * t)

    segs = []
    pts = {}

    def ekey(a, b):
        return (a, b) if a < b else (b, a)

    for i in range(i0 - 1, i1 + 1):
        for k in range(k0 - 1, k1 + 1):
            c = [(i, k), (i, k + 1), (i + 1, k + 1), (i + 1, k)]
            v = [val(*p) >= 0 for p in c]
            code = sum(1 << n for n in range(4) if v[n])
            if code in (0, 15):
                continue
            e = [ekey(c[n], c[(n + 1) % 4]) for n in range(4)]
            for n in range(4):
                if v[n] != v[(n + 1) % 4] and e[n] not in pts:
                    pts[e[n]] = cross(c[n], c[(n + 1) % 4])
            crossing = [e[n] for n in range(4) if v[n] != v[(n + 1) % 4]]
            if len(crossing) == 2:
                segs.append((crossing[0], crossing[1]))
            elif len(crossing) == 4:
                centre = sum(val(*p) for p in c) / 4.0
                # pair edges so that the region stays connected through the centre if centre >= 0
                if (centre >= 0) == v[0]:
                    segs.append((e[0], e[1]))
                    segs.append((e[2], e[3]))
                else:
                    segs.append((e[3], e[0]))
                    segs.append((e[1], e[2]))
    # chain
    adj = {}
    for a, b in segs:
        adj.setdefault(a, []).append(b)
        adj.setdefault(b, []).append(a)
    loops = []
    used = set()
    for a, b in segs:
        if (a, b) in used or (b, a) in used:
            continue
        loop = [a]
        prev, cur = a, b
        used.add((a, b))
        while cur != a:
            loop.append(cur)
            nxt = [n for n in adj[cur] if n != prev and (cur, n) not in used and (n, cur) not in used]
            if not nxt:
                break
            used.add((cur, nxt[0]))
            prev, cur = cur, nxt[0]
        if len(loop) >= 3:
            poly = [pts[key] for key in loop]
            poly = _dedupe(poly)
            if len(poly) >= 3 and abs(geom.signed_area(poly)) > 1e-3:
                loops.append(poly)
    return loops


def _dedupe(poly, tol=1e-6):
    out = []
    for p in poly:
        if not out or geom.dist(p, out[-1]) > tol:
            out.append(p)
    if len(out) > 1 and geom.dist(out[0], out[-1]) <= tol:
        out.pop()
    return out


def simplify(poly, tol=0.05):
    """Douglas-Peucker on a closed polygon."""
    if len(poly) < 5:
        return poly
    # split at the farthest pair to treat the closed loop as two open chains
    a = 0
    b = max(range(len(poly)), key=lambda i: geom.dist(poly[0], poly[i]))
    c1 = _dp(poly[a:b + 1], tol)
    c2 = _dp(poly[b:] + [poly[0]], tol)
    return c1[:-1] + c2[:-1]


def _dp(pts, tol):
    if len(pts) < 3:
        return pts
    a, b = pts[0], pts[-1]
    imax, dmax = 0, -1.0
    for i in range(1, len(pts) - 1):
        d = geom.dist_point_segment(pts[i], a, b)
        if d > dmax:
            imax, dmax = i, d
    if dmax <= tol:
        return [a, b]
    left = _dp(pts[: imax + 1], tol)
    right = _dp(pts[imax:], tol)
    return left[:-1] + right


# --------------------------------------------------------------------------- orientation
def orient_pattern(U, X, active, angle_deg, hand=1):
    """Return (rotate, mirror) so the pattern is viewed from the outside of the
    part (mould side) and the fabric warp (ply angle) lies along +X.

    hand is +1 if the local (s, c, u) frame is right-handed in the model, -1 if not.
    """
    agree = 0
    for (i, k) in active:
        if (i + 1, k) in U and (i, k + 1) in U:
            a, b, c = U[(i, k)], U[(i + 1, k)], U[(i, k + 1)]
            e_s = (b[0] - a[0], b[1] - a[1])
            e_c = (c[0] - a[0], c[1] - a[1])
            s2 = e_s[0] * e_c[1] - e_s[1] * e_c[0]
            X0, Xs, Xc = X[i][k], X[i + 1][k], X[i][k + 1]
            v_s = geom.sub(Xs, X0)
            v_c = geom.sub(Xc, X0)
            # columns run CCW in the (c, u) section, so outward = (dc, du) turned right
            n_out = (0.0, v_c[2], -v_c[1])
            s3 = geom.dot(geom.cross3(v_s, v_c), n_out) * hand
            if s2 != 0 and s3 != 0:
                agree += 1 if (s2 > 0) == (s3 > 0) else -1
    mirror = agree < 0
    sx = sy = 0.0
    for (i, k) in active:
        if (i + 1, k) in U:
            a, b = U[(i, k)], U[(i + 1, k)]
            dx, dy = b[0] - a[0], b[1] - a[1]
            if mirror:
                dx = -dx
            L = math.hypot(dx, dy)
            if L > 0:
                sx += dx / L
                sy += dy / L
    phi = math.atan2(sy, sx)
    rotate = -(phi + math.radians(angle_deg))
    return rotate, mirror


def transform_loops(loops, rotate, mirror):
    cr, sr = math.cos(rotate), math.sin(rotate)
    out = []
    for poly in loops:
        q = []
        for x, y in poly:
            if mirror:
                x = -x
            q.append((x * cr - y * sr, x * sr + y * cr))
        out.append(q)
    return out


def flatten_piece(piece, sections, settings, keepout_fn=None):
    """Full pipeline for one piece. Returns a dict describing the pattern."""
    overlap = float(settings.get("seamOverlap", 0.0))
    X, G, info = build_strip(piece, sections, overlap)
    if keepout_fn is not None:
        flat_nodes = [p for row in X for p in row]
        H = keepout_fn(flat_nodes)
        if H is not None:
            nk = len(X[0])
            for i in range(len(X)):
                for k in range(nk):
                    h = H[i * nk + k]
                    if h < G[i][k]:
                        G[i][k] = h
    # split into runs of stations where the piece exists
    exists = [any(g >= 0 for g in row) for row in G]
    runs, cur = [], []
    for i, e in enumerate(exists):
        if e:
            cur.append(i)
        elif cur:
            runs.append(cur)
            cur = []
    if cur:
        runs.append(cur)
    parts = []
    for run in runs:
        lo = max(0, run[0] - 1)
        hi = min(len(X) - 1, run[-1] + 1)
        Xr, Gr = X[lo:hi + 1], G[lo:hi + 1]
        U, active, stats = flatten_grid(Xr, Gr, int(settings.get("flatIterations", 250)),
                                        angle_deg=piece.angle, fibre_model=fibre_model_for(piece.material),
                                        hand=int(settings.get("_hand", 1)))
        if U is None:
            continue
        loops = contour(U, Gr, active)
        if not loops:
            continue
        rotate, mirror = orient_pattern(U, Xr, active, piece.angle, int(settings.get("_hand", 1)))
        loops = transform_loops(loops, rotate, mirror)
        loops = [simplify(lp, 0.02) for lp in loops]
        loops.sort(key=lambda lp: -abs(geom.signed_area(lp)))
        # drop slivers left where a ply ends at a tip (< 1% of the main part or < 5 cm2)
        big = abs(geom.signed_area(loops[0]))
        loops = [lp for lp in loops if abs(geom.signed_area(lp)) >= max(500.0, 0.01 * big) or lp is loops[0]]
        outers, holes = [], []
        for lp in loops:
            c = lp[0]
            if any(geom.point_in_polygon(c, o) for o in outers):
                holes.append(lp)
            else:
                outers.append(lp)
        for o in outers:
            hs = [h for h in holes if geom.point_in_polygon(h[0], o)]
            area = abs(geom.signed_area(o)) - sum(abs(geom.signed_area(h)) for h in hs)
            xs = [p[0] for p in o]
            ys = [p[1] for p in o]
            parts.append({
                "outline": o,
                "holes": hs,
                "area": area,
                "bbox": (min(xs), min(ys), max(xs), max(ys)),
                "spanAngle": -piece.angle,  # span direction relative to +X (warp)
                "stats": stats,
            })
    return parts
