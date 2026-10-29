"""Pure-Python geometry helpers used by the layup engine.

No Fusion dependency, so everything here can be unit tested outside Fusion.
All lengths are millimetres. 2D points are (x, y) tuples, 3D points (x, y, z).
"""
import math

EPS = 1e-12


# --------------------------------------------------------------------------- vectors
def sub(a, b):
    return tuple(x - y for x, y in zip(a, b))


def add(a, b):
    return tuple(x + y for x, y in zip(a, b))


def scale(a, k):
    return tuple(x * k for x in a)


def dot(a, b):
    return sum(x * y for x, y in zip(a, b))


def norm(a):
    return math.sqrt(dot(a, a))


def dist(a, b):
    return norm(sub(a, b))


def unit(a):
    n = norm(a)
    return scale(a, 1.0 / n) if n > EPS else tuple(0.0 for _ in a)


def lerp(a, b, t):
    return tuple(x + (y - x) * t for x, y in zip(a, b))


def cross3(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


# --------------------------------------------------------------------------- polylines
def chain_polylines(polylines, tol):
    """Join open polylines whose end points touch into closed loops.

    Returns a list of loops (closing point not repeated). Pieces that never close
    are returned too, so the caller can decide what to do with them.
    """
    pieces = [list(p) for p in polylines if len(p) >= 2]
    loops = []
    while pieces:
        cur = pieces.pop(0)
        grown = True
        while grown and dist(cur[0], cur[-1]) > tol:
            grown = False
            for i, p in enumerate(pieces):
                if dist(cur[-1], p[0]) <= tol:
                    cur.extend(p[1:])
                elif dist(cur[-1], p[-1]) <= tol:
                    cur.extend(reversed(p[:-1]))
                elif dist(cur[0], p[-1]) <= tol:
                    cur = p[:-1] + cur
                elif dist(cur[0], p[0]) <= tol:
                    cur = list(reversed(p[1:])) + cur
                else:
                    continue
                pieces.pop(i)
                grown = True
                break
        if dist(cur[0], cur[-1]) <= tol and len(cur) > 2:
            cur = cur[:-1]
        cleaned = [cur[0]]
        for p in cur[1:]:
            if dist(p, cleaned[-1]) > tol * 0.01:
                cleaned.append(p)
        loops.append(cleaned)
    return loops


def signed_area(pts):
    a = 0.0
    n = len(pts)
    for i in range(n):
        x1, y1 = pts[i][0], pts[i][1]
        x2, y2 = pts[(i + 1) % n][0], pts[(i + 1) % n][1]
        a += x1 * y2 - x2 * y1
    return 0.5 * a


def loop_length(pts, closed=True):
    n = len(pts)
    total = sum(dist(pts[i], pts[i + 1]) for i in range(n - 1))
    if closed and n > 1:
        total += dist(pts[-1], pts[0])
    return total


def cumulative_lengths(pts):
    out = [0.0]
    for i in range(1, len(pts)):
        out.append(out[-1] + dist(pts[i - 1], pts[i]))
    return out


def sample_path(pts, cum, s):
    """Point at arc length s along an open polyline with cumulative lengths cum."""
    if s <= 0:
        return pts[0]
    if s >= cum[-1]:
        return pts[-1]
    lo, hi = 0, len(cum) - 1
    while hi - lo > 1:
        mid = (lo + hi) // 2
        if cum[mid] <= s:
            lo = mid
        else:
            hi = mid
    seg = cum[hi] - cum[lo]
    t = (s - cum[lo]) / seg if seg > EPS else 0.0
    return lerp(pts[lo], pts[hi], t)


def clustered_params(n, beta=0.7):
    """n+1 parameters in [0, 1], clustered at both ends (beta=0 uniform, 1 cosine)."""
    out = []
    for i in range(n + 1):
        u = i / n
        out.append((1 - beta) * u + beta * 0.5 * (1 - math.cos(math.pi * u)))
    return out


def resample_open(pts, n, beta=0.7):
    cum = cumulative_lengths(pts)
    return [sample_path(pts, cum, t * cum[-1]) for t in clustered_params(n, beta)]


def resample_uniform_closed(pts, n):
    closed = list(pts) + [pts[0]]
    cum = cumulative_lengths(closed)
    return [sample_path(closed, cum, cum[-1] * i / n) for i in range(n)]


# --------------------------------------------------------------------------- airfoil sections
def orient_ccw(pts):
    return list(pts) if signed_area(pts) > 0 else list(reversed(pts))


def find_te_index(pts, chord_hint=None):
    """Trailing edge = the max-x point; for a blunt base the middle of the base."""
    xmax = max(p[0] for p in pts)
    xmin = min(p[0] for p in pts)
    tol = 0.002 * (xmax - xmin)
    cand = [i for i, p in enumerate(pts) if p[0] >= xmax - tol]
    if len(cand) == 1:
        return cand[0]
    # candidates form one contiguous run around the base (possibly wrapping)
    n = len(pts)
    cset = set(cand)
    start = next(i for i in cand if (i - 1) % n not in cset)
    run = [start]
    while (run[-1] + 1) % n in cset and len(run) < n:
        run.append((run[-1] + 1) % n)
    return run[len(run) // 2]


def rotate_to(pts, i):
    return pts[i:] + pts[:i]


def airfoil_resample(loop2d, m):
    """Resample a closed section loop to m points (m even).

    Output is CCW in (c, u), index 0 at the trailing edge, index m//2 at the
    leading edge, indices 1..m//2-1 on the upper surface and m//2+1..m-1 on the
    lower surface. Points cluster towards both edges.
    """
    pts = orient_ccw(loop2d)
    te = find_te_index(pts)
    pts = rotate_to(pts, te)
    te_pt = pts[0]
    le = max(range(len(pts)), key=lambda i: dist(pts[i], te_pt))
    upper = pts[: le + 1]
    lower = pts[le:] + [pts[0]]
    half = m // 2
    up = resample_open(upper, half)
    lo = resample_open(lower, half)
    return up[:-1] + lo[:-1]


def vertex_normals(pts):
    """Inward unit normals of a CCW loop (left of the tangent)."""
    n = len(pts)
    out = []
    for i in range(n):
        a = unit(sub(pts[i], pts[i - 1]))
        b = unit(sub(pts[(i + 1) % n], pts[i]))
        t = unit(add(a, b))
        if norm(t) < 0.5:  # cusp: fall back to the chord between neighbours
            t = unit(sub(pts[(i + 1) % n], pts[i - 1]))
        out.append((-t[1], t[0]))
    return out


def medial_radius(pts, normals, cap=1e6):
    """Shrinking-ball radius at each vertex: the largest inscribed circle tangent
    there. Inward offsets deeper than this cross the medial axis."""
    n = len(pts)
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    out = []
    for i in range(n):
        px, py = xs[i], ys[i]
        nx, ny = normals[i]
        best = cap
        for j in range(n):
            if j == i:
                continue
            dx = xs[j] - px
            dy = ys[j] - py
            dn = dx * nx + dy * ny
            if dn > 1e-9:
                r = (dx * dx + dy * dy) / (2.0 * dn)
                if r < best:
                    best = r
        out.append(best)
    return out


def ray_half_thickness(pts, normals, cap=1e6):
    """Half the distance from each vertex along its inward normal to the
    opposite side of the loop. On cambered or tilted thin regions the
    shrinking-ball radius overestimates the room; this catches it."""
    n = len(pts)
    segs = []
    for i in range(n):
        a, b = pts[i], pts[(i + 1) % n]
        segs.append((min(a[0], b[0]), max(a[0], b[0]), min(a[1], b[1]), max(a[1], b[1]), a, b, i))
    out = []
    for j in range(n):
        px, py = pts[j]
        nx, ny = normals[j]
        best = cap
        for x0, x1, y0, y1, a, b, i in segs:
            if i == j or i == (j - 1) % n:
                continue
            ex, ey = b[0] - a[0], b[1] - a[1]
            den = nx * ey - ny * ex
            if abs(den) < 1e-12:
                continue
            rx, ry = a[0] - px, a[1] - py
            t = (rx * ey - ry * ex) / den
            if t <= 1e-9 or t >= best:
                continue
            u = (rx * ny - ry * nx) / den
            if -1e-9 <= u <= 1 + 1e-9:
                best = t
        out.append(0.5 * best)
    return out


def offset_trimmed(pts, normals, h, depth, trim):
    """Offset a CCW loop inward by per-vertex depth, trimmed continuously.

    A vertex is usable while h >= trim (trim = depth + end gap, i.e. the layer
    still has room). Where usability changes between two vertices the exact
    crossing point is interpolated, so the trimmed tip moves smoothly from one
    station to the next instead of jumping by whole vertices.
    Returns (points, trimmed?) or (None, True) when nothing is left.
    """
    n = len(pts)

    def P(j):
        return (pts[j][0] + normals[j][0] * depth[j], pts[j][1] + normals[j][1] * depth[j])

    f = [(h[j] - trim[j]) if depth[j] > 0 else 1.0 for j in range(n)]
    if all(v >= 0 for v in f):
        return [P(j) for j in range(n)], False
    if all(v < 0 for v in f):
        return None, True
    # Walk from the start of a usable run. Each gap (TE, tips, a thin LE) is
    # closed with a smooth rounded cap between its two exact cut points, so the
    # tip has no hooks whatever the edge shape (sharp, blunt, rounded).
    start = next(j for j in range(n) if f[j] >= 0 and f[j - 1] < 0)
    out = []
    gap_from = None
    gap_pts = []  # section points that were cut away in the current gap
    skip_next = False
    for step in range(n):
        j = (start + step) % n
        k = (j + 1) % n
        if f[j] < 0:
            gap_pts.append(pts[j])
        if f[j] >= 0:
            if skip_next and out:
                skip_next = False  # too close to the cut point just placed
            else:
                out.append(P(j))
        if (f[j] >= 0) != (f[k] >= 0):
            t = f[j] / (f[j] - f[k])
            cut = lerp(P(j), P(k), t)
            close = 0.3 * dist(P(j), P(k))
            if f[j] >= 0:  # leaving a usable run
                if out and dist(out[-1], cut) < close:
                    out.pop()
                out.append(cut)
                gap_from = cut
                gap_pts = []
            else:          # entering a usable run: cap from gap_from to cut
                if gap_from is not None:
                    toward = None
                    if gap_pts:
                        cx = sum(p[0] for p in gap_pts) / len(gap_pts)
                        cy = sum(p[1] for p in gap_pts) / len(gap_pts)
                        toward = (cx, cy)
                    out.extend(_round_cap(gap_from, cut, toward=toward))
                out.append(cut)
                skip_next = dist(cut, P(k)) < close
    if len(out) < 8 or signed_area(out) <= 0:
        return None, True
    return out, True


def _round_cap(a, b, segments=6, bulge=0.5, toward=None):
    """Points strictly between a and b on a half-ellipse bulging outward
    (towards `toward`, the middle of what was cut away, else to the right of
    a->b, which is outward for a CCW loop); depth = bulge x half the gap."""
    m = lerp(a, b, 0.5)
    d = sub(b, a)
    L = norm(d)
    if L < 1e-9:
        return []
    out_n = (d[1] / L, -d[0] / L)
    if toward is not None:
        v = unit(sub(toward, m))
        if dot(v, out_n) > 0.3:  # only if it really points out of the loop
            out_n = v
    r = 0.5 * L * bulge
    pts = []
    for i in range(1, segments):
        th = math.pi * i / segments
        pts.append((m[0] + (a[0] - m[0]) * math.cos(th) + out_n[0] * r * math.sin(th),
                    m[1] + (a[1] - m[1]) * math.cos(th) + out_n[1] * r * math.sin(th)))
    return pts


# --------------------------------------------------------------------------- planform polygons
def point_in_polygon(p, poly):
    x, y = p
    inside = False
    n = len(poly)
    j = n - 1
    for i in range(n):
        xi, yi = poly[i]
        xj, yj = poly[j]
        if (yi > y) != (yj > y):
            xc = xi + (y - yi) * (xj - xi) / (yj - yi)
            if x < xc:
                inside = not inside
        j = i
    return inside


def dist_point_segment(p, a, b):
    ab = sub(b, a)
    L2 = dot(ab, ab)
    if L2 < EPS:
        return dist(p, a)
    t = max(0.0, min(1.0, dot(sub(p, a), ab) / L2))
    return dist(p, add(a, scale(ab, t)))


def signed_distance_polys(p, polys):
    """Signed distance to a set of closed polygons, even-odd fill, + inside."""
    if not polys:
        return float("inf")
    inside = False
    dmin = float("inf")
    for poly in polys:
        if point_in_polygon(p, poly):
            inside = not inside
        n = len(poly)
        for i in range(n):
            d = dist_point_segment(p, poly[i], poly[(i + 1) % n])
            if d < dmin:
                dmin = d
    return dmin if inside else -dmin


def polys_bbox(polys):
    xs = [p[0] for poly in polys for p in poly]
    ys = [p[1] for poly in polys for p in poly]
    return min(xs), min(ys), max(xs), max(ys)


def nearest_on_polygon(p, poly):
    best, bq = None, None
    n = len(poly)
    for i in range(n):
        a, b = poly[i], poly[(i + 1) % n]
        ab = sub(b, a)
        L2 = dot(ab, ab)
        t = 0.0 if L2 < EPS else max(0.0, min(1.0, dot(sub(p, a), ab) / L2))
        q = add(a, scale(ab, t))
        d = dist(p, q)
        if best is None or d < best:
            best, bq = d, q
    return bq


def clamp_inside(loop, outer, margin):
    """Pull any vertex of loop that lies outside outer back onto outer, margin
    inward. Vertices on or inside outer (e.g. where a one-sided ply is absent
    and the two surfaces coincide) are left exactly as they are."""
    out = []
    for p in loop:
        # resampled polylines of coincident curves differ by sag noise (~1e-3 mm)
        if signed_distance_polys(p, [outer]) >= -0.02:
            out.append(p)
            continue
        q = nearest_on_polygon(p, outer)
        d = sub(q, p) if point_in_polygon(p, outer) else sub(p, q)
        n = norm(d)
        inward = scale(d, -1.0 / n) if n > EPS else (0.0, 0.0)
        # inward points from the boundary into the polygon
        if not point_in_polygon(add(q, scale(inward, margin)), outer):
            inward = scale(inward, -1.0)
        out.append(add(q, scale(inward, margin)))
    return out


def chaikin(pts, passes=2):
    """Corner-cutting smoothing of a closed loop (rounds trimmed tips)."""
    for _ in range(passes):
        out = []
        n = len(pts)
        for i in range(n):
            a, b = pts[i], pts[(i + 1) % n]
            out.append((0.75 * a[0] + 0.25 * b[0], 0.75 * a[1] + 0.25 * b[1]))
            out.append((0.25 * a[0] + 0.75 * b[0], 0.25 * a[1] + 0.75 * b[1]))
        pts = out
    return pts


def _seg_cross(p1, p2, p3, p4):
    d1 = (p4[0] - p3[0]) * (p1[1] - p3[1]) - (p4[1] - p3[1]) * (p1[0] - p3[0])
    d2 = (p4[0] - p3[0]) * (p2[1] - p3[1]) - (p4[1] - p3[1]) * (p2[0] - p3[0])
    d3 = (p2[0] - p1[0]) * (p3[1] - p1[1]) - (p2[1] - p1[1]) * (p3[0] - p1[0])
    d4 = (p2[0] - p1[0]) * (p4[1] - p1[1]) - (p2[1] - p1[1]) * (p4[0] - p1[0])
    return (d1 > 0) != (d2 > 0) and (d3 > 0) != (d4 > 0)


def self_intersects(pts):
    """True if a closed polygon crosses itself (bounding-box pruned O(n^2))."""
    n = len(pts)
    segs = []
    for i in range(n):
        a, b = pts[i], pts[(i + 1) % n]
        segs.append((min(a[0], b[0]), max(a[0], b[0]), min(a[1], b[1]), max(a[1], b[1]), a, b))
    for i in range(n):
        x0, x1, y0, y1, a, b = segs[i]
        for j in range(i + 2, n):
            if i == 0 and j == n - 1:
                continue
            u0, u1, v0, v1, c, d = segs[j]
            if u0 > x1 or u1 < x0 or v0 > y1 or v1 < y0:
                continue
            if _seg_cross(a, b, c, d):
                return True
    return False
