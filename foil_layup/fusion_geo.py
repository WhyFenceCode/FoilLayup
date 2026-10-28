"""Fusion-side geometry access: frame detection, sections, bounds, keep-outs.

Fusion works in cm internally; everything handed to the pure-Python engine is mm.
"""
import math

import adsk.core
import adsk.fusion

from . import geom, model

CM = 0.1   # mm -> cm
MM = 10.0  # cm -> mm

AXES = {"X": (1.0, 0.0, 0.0), "Y": (0.0, 1.0, 0.0), "Z": (0.0, 0.0, 1.0)}


class FusionError(Exception):
    pass


def app():
    return adsk.core.Application.get()


def design():
    d = adsk.fusion.Design.cast(app().activeProduct)
    if not d:
        raise FusionError("Switch to the Design workspace first.")
    return d


def preferred_up():
    pref = app().preferences.generalPreferences.defaultModelingOrientation
    return "Z" if pref == adsk.core.DefaultModelingOrientations.ZUpModelingOrientation else "Y"


def length_units():
    try:
        return design().unitsManager.defaultLengthUnits
    except Exception:
        return "mm"


def mm_per_unit(units=None):
    units = units or length_units()
    return {"mm": 1.0, "cm": 10.0, "m": 1000.0, "in": 25.4, "ft": 304.8}.get(units, 1.0)


# --------------------------------------------------------------------------- entities
def find_by_token(token):
    if not token:
        return None
    try:
        ents = design().findEntityByToken(token)
    except Exception:
        return None
    return ents[0] if ents else None


def body_path(body):
    occ = body.assemblyContext
    if occ:
        return "%s / %s" % (occ.fullPathName, body.name)
    return body.name


def list_bodies():
    """All solid bodies in the design as proxies (world coordinates)."""
    d = design()
    root = d.rootComponent
    out = []
    for b in root.bRepBodies:
        if b.isSolid:
            out.append(b)
    for occ in root.allOccurrences:
        for b in occ.component.bRepBodies:
            if b.isSolid:
                out.append(b.createForAssemblyContext(occ))
    return out


def list_sketches():
    d = design()
    out = []
    for comp in d.allComponents:
        for sk in comp.sketches:
            out.append((comp, sk))
    return out


def target_context(body):
    """(native body, component, world->component matrix or None)."""
    native = body.nativeObject if body.assemblyContext else body
    comp = native.parentComponent
    inv = None
    if body.assemblyContext:
        inv = body.assemblyContext.transform2.copy()
        inv.invert()
    return native, comp, inv


# --------------------------------------------------------------------------- frame
def detect_frame(body, frame_cfg):
    """Span/chord/up axes for the body in its own component space.

    frame_cfg: {spanAxis: auto|X|Y|Z, upAxis: auto|X|Y|Z, flipUp, symmetric,
                spanCentre (mm offset of the symmetry plane from bbox centre)}
    """
    native = body.nativeObject if body.assemblyContext else body
    bb = native.boundingBox
    lo = (bb.minPoint.x * MM, bb.minPoint.y * MM, bb.minPoint.z * MM)
    hi = (bb.maxPoint.x * MM, bb.maxPoint.y * MM, bb.maxPoint.z * MM)
    ext = {"X": hi[0] - lo[0], "Y": hi[1] - lo[1], "Z": hi[2] - lo[2]}
    span = frame_cfg.get("spanAxis") or "auto"
    if span == "auto":
        span = max(ext, key=ext.get)
    up = frame_cfg.get("upAxis") or "auto"
    others = [a for a in "XYZ" if a != span]
    if up == "auto":
        # Respect the Fusion up preference when it is the thin direction of the
        # foil; otherwise (foil modelled on its side) use the thinnest axis.
        pref = preferred_up()
        thinnest = min(others, key=lambda a: ext[a])
        up = pref if pref == thinnest else thinnest
    if up == span:
        raise FusionError("Up axis cannot be the span axis")
    chord_axis = [a for a in "XYZ" if a not in (span, up)][0]
    S = AXES[span]
    U = AXES[up]
    if frame_cfg.get("flipUp"):
        U = geom.scale(U, -1.0)
    C = geom.cross3(U, S)  # (S, C, U) right handed
    centre = tuple((a + b) / 2.0 for a, b in zip(lo, hi))
    off = float(frame_cfg.get("spanCentre") or 0.0)
    origin = geom.add(centre, geom.scale(S, off))
    half = ext[span] / 2.0
    frame = {
        "spanAxis": span, "upAxis": up, "chordAxis": chord_axis,
        "S": S, "C": C, "U": U, "origin": origin,
        "sMin": -half - off, "sMax": half - off,
        "symmetric": bool(frame_cfg.get("symmetric", True)),
        "hand": 1,
        "extents": ext,
        "preferredUp": preferred_up(),
    }
    # make c run LE -> TE: the trailing edge end is the thinner one
    votes = 0
    for f in (-0.3, -0.15, 0.15, 0.3):
        s = (frame["sMin"] + frame["sMax"]) / 2.0 + f * (frame["sMax"] - frame["sMin"])
        loop = raw_section(native, frame, s)
        if loop:
            votes += _te_vote(loop)
    if votes < 0:
        frame["C"] = geom.scale(C, -1.0)
        frame["hand"] = -1
    return frame


def _te_vote(loop):
    """+1 if the max-c end looks like the trailing edge, -1 if the min-c end."""
    cs = [p[0] for p in loop]
    c0, c1 = min(cs), max(cs)
    ch = c1 - c0

    def thick_at(c):
        us = []
        for i in range(len(loop)):
            a, b = loop[i], loop[(i + 1) % len(loop)]
            if (a[0] - c) * (b[0] - c) <= 0 and a[0] != b[0]:
                t = (c - a[0]) / (b[0] - a[0])
                us.append(a[1] + t * (b[1] - a[1]))
        return (max(us) - min(us)) if len(us) >= 2 else 0.0

    t_lo = thick_at(c0 + 0.04 * ch)
    t_hi = thick_at(c1 - 0.04 * ch)
    if abs(t_lo - t_hi) < 1e-6:
        return 0
    return 1 if t_hi < t_lo else -1


def to_model(frame, s, c, u):
    """Local (s, c, u) mm -> component-space Point3D (cm)."""
    o, S, C, U = frame["origin"], frame["S"], frame["C"], frame["U"]
    p = [o[k] + S[k] * s + C[k] * c + U[k] * u for k in range(3)]
    return adsk.core.Point3D.create(p[0] * CM, p[1] * CM, p[2] * CM)


def to_local(frame, pt):
    p = (pt.x * MM, pt.y * MM, pt.z * MM)
    d = geom.sub(p, frame["origin"])
    return (geom.dot(d, frame["S"]), geom.dot(d, frame["C"]), geom.dot(d, frame["U"]))


def vec(v):
    return adsk.core.Vector3D.create(v[0], v[1], v[2])


# --------------------------------------------------------------------------- sections
def raw_section(native_body, frame, s, tol_mm=0.01):
    """Largest closed loop of the body cut at span station s, in (c, u) mm."""
    tbm = adsk.fusion.TemporaryBRepManager.get()
    plane = adsk.core.Plane.create(to_model(frame, s, 0, 0), vec(frame["S"]))
    try:
        wire = tbm.planeIntersection(native_body, plane)
    except Exception:
        return None
    if not wire:
        return None
    polylines = []
    for edge in wire.edges:
        ev = edge.evaluator
        ok, p0, p1 = ev.getParameterExtents()
        if not ok:
            continue
        ok, pts = ev.getStrokes(p0, p1, tol_mm * CM)
        if not ok or len(pts) < 2:
            continue
        pl = []
        for p in pts:
            _, c, u = to_local(frame, p)
            pl.append((c, u))
        polylines.append(pl)
    loops = geom.chain_polylines(polylines, 0.01)
    closed = [lp for lp in loops if len(lp) >= 8]
    if not closed:
        return None
    return max(closed, key=lambda lp: abs(geom.signed_area(lp)))


def section_profile(loop):
    cs = [p[0] for p in loop]
    us = [p[1] for p in loop]
    return (min(cs), max(cs), max(us) - min(us))


def sample_sections(native_body, frame, n, inset, extra=(), progress=None):
    """Stations clustered at the tips and where the shape changes fast."""
    cache = {}

    def get(s):
        key = round(s, 6)
        if key not in cache:
            cache[key] = raw_section(native_body, frame, s)
        return cache[key]

    def profile(s):
        lp = get(s)
        return section_profile(lp) if lp else (0.0, 0.0, 0.0)

    positions = model.station_positions(frame["sMin"], frame["sMax"], n, inset, profile, extra)
    raw = []
    for k, s in enumerate(positions):
        if progress:
            progress(k, len(positions))
        raw.append((s, get(s)))
    return raw


# --------------------------------------------------------------------------- bounds
def load_bounds(names, frame, log=None):
    """Planform polygons (s, c) for each named sketch (closed loops only)."""
    out = {}
    if not names:
        return out
    sketches = list_sketches()
    for name in names:
        sk = next((s for c, s in sketches if s.name == name), None)
        if sk is None:
            if log:
                log("Bound sketch '%s' not found" % name)
            continue
        polylines = []
        for curve in sk.sketchCurves:
            if curve.isConstruction:
                continue
            g = curve.worldGeometry
            ev = g.evaluator
            ok, p0, p1 = ev.getParameterExtents()
            if not ok:
                continue
            ok, pts = ev.getStrokes(p0, p1, 0.005)
            if not ok:
                continue
            pl = []
            for p in pts:
                s, c, _u = to_local(frame, p)
                pl.append((s, c))
            polylines.append(pl)
        loops = [lp for lp in geom.chain_polylines(polylines, 0.05) if len(lp) >= 3]
        loops = [lp for lp in loops if abs(geom.signed_area(lp)) > 1.0]
        if not loops and log:
            log("Bound sketch '%s' has no closed loops" % name)
        try:
            n = sk.xDirection.crossProduct(sk.yDirection)
            if abs(n.dotProduct(vec(frame["U"]))) < 0.95 and log:
                log("Bound sketch '%s' is not drawn in a top view plane; it is projected along the up axis" % name)
        except Exception:
            pass
        out[name] = loops
    return out


# --------------------------------------------------------------------------- keep-outs
def keepout_bodies(stack_data, target_body):
    """Resolve keep-outs to (temporary body in target component space, clearance mm, name)."""
    tbm = adsk.fusion.TemporaryBRepManager.get()
    _native, _comp, inv = target_context(target_body)
    out = []
    for ko in stack_data.get("keepouts") or []:
        b = find_by_token(ko.get("token"))
        if b is None:
            raise FusionError("Keep-out body '%s' no longer exists" % ko.get("name"))
        b = adsk.fusion.BRepBody.cast(b)
        tmp = tbm.copy(b)
        if inv is not None:
            tbm.transform(tmp, inv)
        out.append((tmp, float(ko.get("clearance") or 0.0), ko.get("name") or b.name))
    return out


def keepout_field_fn(kos, frame):
    """Signed distance (mm, + outside) of local nodes to the keep-outs minus clearance."""
    if not kos:
        return None
    mm_ = app().measureManager
    boxes = []
    for body, clr, _ in kos:
        bb = body.boundingBox
        lo = (bb.minPoint.x * MM - clr - 5, bb.minPoint.y * MM - clr - 5, bb.minPoint.z * MM - clr - 5)
        hi = (bb.maxPoint.x * MM + clr + 5, bb.maxPoint.y * MM + clr + 5, bb.maxPoint.z * MM + clr + 5)
        boxes.append((lo, hi))

    def fn(nodes):
        out = []
        for (s, c, u) in nodes:
            p = to_model(frame, s, c, u)
            pm = (p.x * MM, p.y * MM, p.z * MM)
            best = math.inf
            for (body, clr, _), (lo, hi) in zip(kos, boxes):
                if not all(lo[k] <= pm[k] <= hi[k] for k in range(3)):
                    continue
                try:
                    inside = body.pointContainment(p) == adsk.fusion.PointContainment.PointInsidePointContainment
                except Exception:
                    inside = False
                if inside:
                    v = -max(clr, 0.01)
                else:
                    try:
                        d = mm_.measureMinimumDistance(p, body).value * MM
                    except Exception:
                        d = math.inf
                    v = d - clr
                best = min(best, v)
            out.append(best)
        return out

    return fn
