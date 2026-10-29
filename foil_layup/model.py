"""Stack definition, section analysis and per-section depth fields (pure Python).

Coordinates: every section lives in a plane normal to the span axis. Inside a
section, c runs along the chord axis (leading edge -> trailing edge) and u
along the up axis. Planform coordinates are (s, c). All lengths in mm.

A *row* is one line of the stack editor (one material, n plies). Rows are the
unit for 3D bodies; plies are the unit for flat patterns.
"""
import math

from . import geom

DEFAULT_SETTINGS = {
    "construction": "offset",  # 'offset': exact offsets of the body; 'loft': lofted sections
    "loftStations": 40,     # sections used to loft the 3D layer surfaces
    "loftPoints": 120,      # points per loft section
    "flatStations": 60,     # sections used for flat patterns
    "flatPoints": 160,      # points per flat-pattern section
    "analysisPoints": 200,  # points used for thickness / medial analysis
    "dropRatio": 20.0,      # ply drop ramp length = ratio x row thickness
    "minGap": 0.15,         # mm: minimum half-thickness kept inside an inner surface
    "chordRefSmoothing": 0.05,  # chord limits use LE/TE smoothed over this share of
                                # the span, so a centre hub does not drag bands sideways
    "tipSchedule": True,    # drop plies spanwise towards the tips so the stack always fits
    "tipCoreMin": 1.0,      # mm: core thickness at which inner plies start to end
    "plyEndRatio": 1.0,     # inner plies stop where the space inside the layers above
                            # them is less than ratio x ply thickness (no feather edges)
    "tipInset": 0.4,        # mm: first/last station inset from the body ends
    "seamStagger": 25.0,    # mm: min distance between seams of nearby plies
    "seamWindow": 3,        # plies either side that must respect seamStagger
    "seamEdgeMargin": 15.0, # mm: min distance from LE/TE and bound edges
    "seamOverlap": 0.0,     # mm: extra material past the seam (lap joint)
    "seamPreferSide": "bottom",
    "flatIterations": 400,
    "rollWidth": 1270.0,    # mm, for nesting the cut sheet
    "organize": True,       # move generated bodies into a new component
    "hideOriginal": True,
}

SIDES = ("wrap", "top", "bottom")


class StackError(Exception):
    pass


# --------------------------------------------------------------------------- stack
def parse_angles(text, count):
    vals = []
    for tok in str(text if text is not None else "0").replace(";", ",").split(","):
        tok = tok.strip()
        if tok:
            try:
                vals.append(float(tok))
            except ValueError:
                raise StackError("Bad ply angle '%s'" % tok)
    if not vals:
        vals = [0.0]
    return [vals[i % len(vals)] for i in range(count)]


class Ply:
    def __init__(self, row, index, angle):
        self.row = row
        self.index = index
        self.angle = angle
        self.t = row.ply_t
        self.id = "%s.%d" % (row.id, index + 1)
        self.name = "%s %d/%d" % (row.name, index + 1, row.count)
        self.seam = None  # filled in by seams.plan_seams


class Row:
    def __init__(self, data, materials, number, external=False):
        self.data = data
        self.external = external
        self.id = str(data.get("id") or ("E%d" % number if external else "L%d" % number))
        self.number = number
        mid = data.get("material")
        if mid not in materials:
            raise StackError("Layer %d uses unknown material '%s'" % (number, mid))
        self.material = materials[mid]
        self.count = max(1, int(data.get("count", 1)))
        self.ply_t = float(data.get("thickness") or self.material["thickness"])
        if self.ply_t <= 0:
            raise StackError("Layer %d (%s) has zero thickness; core materials cannot be layers"
                             % (number, self.material["name"]))
        self.t = self.ply_t * self.count
        self.name = data.get("name") or self.material["name"]
        self.side = data.get("side", "wrap")
        if self.side not in SIDES:
            raise StackError("Layer %d: side must be wrap/top/bottom" % number)
        self.span = _range(data.get("span"))
        self.chord = _range(data.get("chord"))
        b = data.get("bound") or {}
        self.bound_sketch = (b.get("sketch") or "").strip()
        self.bound_mode = b.get("mode", "inside")
        self.outside = data.get("outsideMaterial") or None
        if self.outside and self.outside != "core" and self.outside not in materials:
            raise StackError("Layer %d: unknown outside material '%s'" % (number, self.outside))
        self.seam = (data.get("seam") or "auto").strip().lower()
        eg = data.get("endGap")
        self.end_gap = float(eg) if eg not in (None, "") else None  # mm, overrides plyEndRatio
        self.splits = [float(v) for v in (data.get("splits") or [])]
        self.tip_end = None  # span_ref where the tip drop schedule ends this ply
        self.plies = [Ply(self, i, a) for i, a in enumerate(parse_angles(data.get("angles", "0"), self.count))]
        if external and (self.side != "wrap" or self.has_user_region):
            # External consumables are modelled as full wraps.
            self.side, self.span, self.chord, self.bound_sketch = "wrap", None, None, ""

    @property
    def has_user_region(self):
        """Limits set in the stack editor (span, chord, bound sketch)."""
        return bool(self.span or self.chord or self.bound_sketch)

    @property
    def has_region(self):
        return self.has_user_region or self.tip_end is not None

    @property
    def eff_span(self):
        """Span limits including the automatic tip end."""
        s0, s1 = self.span if self.span else (None, None)
        if self.tip_end is not None:
            s1 = self.tip_end if s1 is None else min(s1, self.tip_end)
        return None if s0 is None and s1 is None else (s0, s1)

    def split(self, materials):
        """One single-ply unit per ply, so every ply becomes its own body and ends
        at its own depth. Units keep the row's limits and the ply's id/name."""
        out = []
        for p in self.plies:
            d = dict(self.data)
            d.update({"count": 1, "angles": str(p.angle), "id": p.id})
            u = Row(d, materials, self.number, self.external)
            u.name = p.name
            u.plies[0].id = p.id
            u.plies[0].name = p.name
            u.parent = self
            out.append(u)
        return out

    @property
    def is_full(self):
        """True when the row has the same thickness everywhere (no collapse)."""
        if self.side != "wrap":
            return False
        return not self.has_region or bool(self.outside)


def _range(v):
    if not v:
        return None
    a, b = v
    a = None if a in ("", None) else float(a)
    b = None if b in ("", None) else float(b)
    if a is None and b is None:
        return None
    return (a, b)


class Stack:
    def __init__(self, data, materials):
        self.data = data
        self.materials = materials
        self.settings = dict(DEFAULT_SETTINGS)
        self.settings.update(data.get("settings") or {})
        self.rows = [Row(r, materials, i + 1) for i, r in enumerate(data.get("layers") or [])]
        self.external = [Row(r, materials, i + 1, True) for i, r in enumerate(data.get("external") or [])]
        core = data.get("core") or {}
        self.core_material = materials.get(core.get("material") or "pvc130")
        self.core_name = core.get("name") or "Core"
        frame = data.get("frame") or {}
        self.symmetric = bool(frame.get("symmetric", True))
        ids = [r.id for r in self.rows + self.external]
        if len(ids) != len(set(ids)):
            raise StackError("Layer ids must be unique")
        # geometry is built per ply: each unit is one ply of an editor row
        self.units = [u for r in self.rows for u in r.split(materials)]
        self.ext_units = [u for r in self.external for u in r.split(materials)]

    @property
    def plies(self):
        return [p for r in self.rows for p in r.plies]

    @property
    def bound_sketches(self):
        return sorted({r.bound_sketch for r in self.rows if r.bound_sketch})


# --------------------------------------------------------------------------- sections
class Section:
    """One chordwise slice, resampled so index 0 = TE and index m//2 = LE."""

    def __init__(self, s, loop2d, m):
        self.s = s
        self.m = m
        self.le = m // 2
        self.pts = geom.airfoil_resample(loop2d, m)
        self.n = geom.vertex_normals(self.pts)
        self.h = geom.medial_radius(self.pts, self.n)
        ray = geom.ray_half_thickness(self.pts, self.n)
        self.h = [min(a_, b_) for a_, b_ in zip(self.h, ray)]
        # The sampled shrinking ball cannot see a sharp corner (it is limited by
        # the neighbour spacing), but no layer fits at a sharp TE: a corner has
        # zero clearance. Otherwise trimming flips between "reaches the TE" and
        # "stops short" from one station to the next and lofts fold over.
        # A corner is a turning spike next to a nearly straight neighbour; a tight but
        # smooth leading edge turns steadily over several points and is not one.
        turn = []
        for j in range(self.m):
            a_ = geom.unit(geom.sub(self.pts[j], self.pts[j - 1]))
            b_ = geom.unit(geom.sub(self.pts[(j + 1) % self.m], self.pts[j]))
            turn.append(math.acos(max(-1.0, min(1.0, geom.dot(a_, b_)))))
        # (a blunt base has two corners side by side, so compare with either neighbour)
        for j in range(self.m):
            nb = min(turn[j - 1], turn[(j + 1) % self.m])
            if turn[j] > math.radians(25) and turn[j] > 3.0 * nb:
                self.h[j] = 0.0
        te, le = self.pts[0], self.pts[self.le]
        self.te_pt, self.le_pt = te, le
        self.chord = geom.dist(te, le)
        self.chord_dir = geom.unit(geom.sub(te, le))
        # normal to the chord line pointing to the upper surface
        self.up_dir = (-self.chord_dir[1], self.chord_dir[0])
        self.x = [geom.dot(geom.sub(p, le), self.chord_dir) / self.chord for p in self.pts]
        closed = self.pts + [self.pts[0]]
        self.cum = geom.cumulative_lengths(closed)  # cum[j] = arc from TE over the top
        self.length = self.cum[-1]
        # largest inscribed circle ~ max thickness (LE excluded: its ball is the LE radius)
        # reference LE/TE (planform c) for chord limits; see smooth_chord_reference
        self.ref_le, self.ref_te = le[0], te[0]
        self.thickness = 2.0 * max(self.h[j] for j in range(self.m) if 0.05 < self.x[j] < 0.95)

    def arc(self, j):
        return self.cum[j]

    def side_field(self, side):
        """Signed arc distance (mm) to the upper (side='top') or lower region."""
        L, a_le = self.length, self.cum[self.le]
        out = []
        for j in range(self.m):
            a = self.cum[j]
            if side == "top":
                inside = 0 <= a <= a_le
                f = min(a, a_le - a) if inside else -min(a - a_le, L - a)
            else:
                inside = a >= a_le
                f = min(a - a_le, L - a) if inside else -min(a, a_le - a)
            out.append(f)
        return out

    def world(self, frame, j_or_pt, depth=0.0):
        """Local (s, c, u) for a vertex index, optionally offset by depth."""
        if isinstance(j_or_pt, int):
            p, n = self.pts[j_or_pt], self.n[j_or_pt]
            p = (p[0] + n[0] * depth, p[1] + n[1] * depth)
        else:
            p = j_or_pt
        return (self.s, p[0], p[1])


def span_ref(s, frame):
    """Span coordinate used for span limits (distance from centre if symmetric)."""
    if frame.get("symmetric", True):
        return abs(s)
    return s - frame["sMin"]


# --------------------------------------------------------------------------- fields
def region_field(row, sec, frame, bounds, chord_at=None):
    """Signed distance (mm, + inside) of each vertex to the row's region limits
    (span range, chord range, bound sketch). None if the row has no region."""
    if not row.has_region:
        return None
    sr = span_ref(sec.s, frame)
    polys = bounds.get(row.bound_sketch) if row.bound_sketch else None
    if row.bound_sketch and not polys:
        raise StackError("Bound sketch '%s' (layer %d) not found or has no closed loops"
                         % (row.bound_sketch, row.number))
    out = []
    for j in range(sec.m):
        f = float("inf")
        if row.eff_span:
            s0, s1 = row.eff_span
            if s0 is not None:
                f = min(f, sr - s0)
            if s1 is not None:
                f = min(f, s1 - sr)
        if row.chord:
            c0, c1 = row.chord
            x = (sec.pts[j][0] - sec.ref_le) / (sec.ref_te - sec.ref_le)
            if c0 is not None and c0 > 0:
                f = min(f, (x - c0) * sec.chord)
            if c1 is not None and c1 < 1:
                f = min(f, (c1 - x) * sec.chord)
        if polys:
            d = geom.signed_distance_polys((sec.s, sec.pts[j][0]), polys)
            f = min(f, d if row.bound_mode != "outside" else -d)
        out.append(f)
    return out


def row_fields(row, sec, frame, bounds):
    fs = sec.side_field(row.side) if row.side != "wrap" else None
    fr = region_field(row, sec, frame, bounds)
    if row.outside:
        presence = fs
    elif fs is None:
        presence = fr
    elif fr is None:
        presence = fs
    else:
        presence = [min(a, b) for a, b in zip(fs, fr)]
    return fs, fr, presence


def _smoothstep(x):
    """S-shaped 0..1 ramp: ply drops taper without creases at either end."""
    x = min(1.0, max(0.0, x))
    return x * x * (3.0 - 2.0 * x)


def smooth_chord_reference(sections, fraction):
    """Set sec.ref_le / sec.ref_te: the LE/TE planform positions smoothed along
    the span (Gaussian, sigma = fraction x span, weighted by station spacing so
    densely sampled hubs do not dominate), never wider than the real section.
    Chord-% limits then follow the main wing through a hub or pod instead of
    jumping sideways with the local fairing."""
    n = len(sections)
    if n < 3 or fraction <= 0:
        return
    span = sections[-1].s - sections[0].s
    sigma = max(fraction * span, 1e-6)
    wts = []
    for i in range(n):
        a = sections[max(i - 1, 0)].s
        b = sections[min(i + 1, n - 1)].s
        wts.append(max(b - a, 1e-6) / 2.0)
    le = [sec.le_pt[0] for sec in sections]
    te = [sec.te_pt[0] for sec in sections]
    for i, sec in enumerate(sections):
        sw = sl = st = 0.0
        for k in range(n):
            w = wts[k] * math.exp(-0.5 * ((sections[k].s - sec.s) / sigma) ** 2)
            sw += w
            sl += w * le[k]
            st += w * te[k]
        rl, rt = sl / sw, st / sw
        lo, hi = min(le[i], te[i]), max(le[i], te[i])
        rl = min(max(rl, lo), hi)
        rt = min(max(rt, lo), hi)
        if abs(rt - rl) < 0.2 * abs(te[i] - le[i]):  # degenerate: keep the local chord
            rl, rt = le[i], te[i]
        sec.ref_le, sec.ref_te = rl, rt


def section_depths(sec, rows, frame, bounds, ramp_ratio):
    """Depth fields for each row at one section.

    Returns a list of dicts (one per row) with per-vertex arrays:
      before: depth of the row's outer surface
      after:  depth of the row's inner surface (ramped where the row is absent)
      full:   before + full row thickness (used to cut the row slot)
      side, region, presence: signed fields (or None)
    """
    D = [0.0] * sec.m
    out = []
    for row in rows:
        fs, fr, presence = row_fields(row, sec, frame, bounds)
        ramp = max(ramp_ratio * row.t, 1e-6)
        if presence is None:
            a = [1.0] * sec.m
        else:
            a = [1.0 if f >= 0 else _smoothstep(1.0 + f / ramp) for f in presence]
        after = [D[j] + row.t * a[j] for j in range(sec.m)]
        out.append({
            "row": row,
            "before": D,
            "after": after,
            "full": [D[j] + row.t for j in range(sec.m)],
            "a": a,
            "side": fs,
            "region": fr,
            "presence": presence,
        })
        D = after
    return out


def external_depths(sec, rows):
    D = [0.0] * sec.m
    out = []
    for row in rows:
        after = [d - row.t for d in D]
        out.append({"row": row, "before": D, "after": after})
        D = after
    return out


def end_gap(ply_t, settings, override=None):
    """Half-gap (mm) below which a ply of thickness ply_t no longer continues:
    the layer's own 'stop where space <' value, else plyEndRatio x ply thickness."""
    mg = float(settings.get("minGap", 0.15))
    if override is not None:
        return max(mg, override)
    return max(mg, float(settings.get("plyEndRatio", 1.0)) * ply_t)


def ply_mid_depth(ply, rd, j):
    """Depth of a ply's mid-surface at vertex j of a section (for flattening)."""
    before = rd["before"][j]
    k = ply.index
    return before + ply.t * (k + 0.5)


# --------------------------------------------------------------------------- stations
def station_positions(s_min, s_max, n, inset, profile=None, extra=()):
    """Span stations clustered towards both tips and, if a profile function
    g(s) -> tuple of shape metrics is given, towards rapid shape changes."""
    a, b = s_min + inset, s_max - inset
    if profile is None:
        base = [a + (b - a) * t for t in geom.clustered_params(n - 1, 0.6)]
    else:
        fine = [a + (b - a) * t for t in geom.clustered_params(4 * n, 0.6)]
        vals = [profile(s) for s in fine]
        scale = [max(abs(v[k]) for v in vals) or 1.0 for k in range(len(vals[0]))]
        w = [0.0]
        for i in range(1, len(fine)):
            dv = math.sqrt(sum(((vals[i][k] - vals[i - 1][k]) / scale[k]) ** 2 for k in range(len(scale))))
            ds = (fine[i] - fine[i - 1]) / (b - a)
            w.append(w[-1] + ds + 2.0 * dv)
        base = []
        for k in range(n):
            target = w[-1] * k / (n - 1)
            i = next(i for i in range(len(w)) if w[i] >= target - 1e-12)
            if i == 0:
                base.append(fine[0])
            else:
                t = (target - w[i - 1]) / max(w[i] - w[i - 1], 1e-12)
                base.append(fine[i - 1] + t * (fine[i] - fine[i - 1]))
    # Rapid shape changes (a centre pod) need dense stations, so only
    # near-duplicates are merged. Span-limit stations take priority.
    min_gap = max(1.0, 0.04 * (b - a) / max(n - 1, 1))
    must = sorted({round(e, 6) for e in extra if a <= e <= b})
    pts = sorted(set([round(v, 6) for v in base]))
    keep = list(must)
    for v in pts:
        if all(abs(v - k) >= min_gap for k in keep):
            keep.append(v)
    keep.sort()
    for end in (a, b):  # always keep the two end stations
        if all(abs(end - k) > 1e-9 for k in keep):
            keep = [k for k in keep if abs(k - end) >= min_gap] + [end]
    keep.sort()
    # stations dropped for crowding are given back where the gaps are widest
    while len(keep) < n + len(must):
        i = max(range(len(keep) - 1), key=lambda k: keep[k + 1] - keep[k])
        if keep[i + 1] - keep[i] < 2 * min_gap:
            break
        keep.insert(i + 1, 0.5 * (keep[i] + keep[i + 1]))
    return keep


def span_limit_stations(stack, frame):
    """Extra stations at the span limits so masks and ply drops are resolved."""
    out = []
    for r in stack.rows:
        for lim in (r.span or ()):
            if lim is None:
                continue
            if frame.get("symmetric", True):
                out += [lim, -lim]
            else:
                out.append(lim + frame["sMin"])
        for sp in r.splits:
            out += [sp, -sp] if frame.get("symmetric", True) else [sp + frame["sMin"]]
    return out
