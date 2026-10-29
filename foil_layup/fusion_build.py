"""Generate the layer bodies and core in Fusion.

Everything is built inside one Base Feature (parametric designs) so the
timeline gets a single, fast, undoable item; direct designs build in place.

Body logic, from the outside in (S_k = lofted inner surface after row k):
  full row:     ply  = S_{k-1} - S_k
  partial row:  slot = S_{k-1} - F_k        (F_k = full-thickness surface)
                ply  = slot n side mask n region mask
                (outside material: fill = slot n side mask - region mask)
  core:         S_last
  external:     ext_k = E_k - E_{k-1}         (E_0 = the original body)
"""
import json
import os

import adsk.core
import adsk.fusion

from . import engine, flatten, geom, materials, model
from .fusion_geo import CM, MM, FusionError, design, find_by_token, keepout_bodies, load_bounds, \
    sample_sections, target_context, detect_frame, to_model, vec, app

ATTR = "FoilLayup"
BIG_EXTEND = 40.0  # mm added around masks


class _BadSpan(Exception):
    def __init__(self, i0, i1, msg):
        super().__init__(msg)
        self.i0, self.i1, self.msg = i0, i1, msg


class Builder:
    def __init__(self, comp, base, frame, log, progress):
        self.comp = comp
        self.base = base
        self.frame = frame
        self.log = log
        self.progress = progress
        self.planes = []
        self.sketches = []
        self.tbm = adsk.fusion.TemporaryBRepManager.get()

    # ------------------------------------------------------------------ helpers
    def _plane(self, s):
        inp = self.comp.constructionPlanes.createInput()
        inp.setByPlane(adsk.core.Plane.create(to_model(self.frame, s, 0, 0), vec(self.frame["S"])))
        pl = self.comp.constructionPlanes.add(inp)
        pl.isLightBulbOn = False
        self.planes.append(pl)
        return pl

    def _up_plane(self):
        inp = self.comp.constructionPlanes.createInput()
        inp.setByPlane(adsk.core.Plane.create(to_model(self.frame, 0, 0, 0), vec(self.frame["U"])))
        pl = self.comp.constructionPlanes.add(inp)
        pl.isLightBulbOn = False
        self.planes.append(pl)
        return pl

    def _sketch(self, plane, name):
        sk = self.comp.sketches.add(plane)
        sk.name = name
        sk.isComputeDeferred = True
        self.sketches.append(sk)
        return sk

    def _spline_profile(self, s, loop, name):
        """Section profile built like a foil section: fitted splines from the
        upper stop point to the nose and from the nose to the lower stop point,
        closed by a straight line (the ply's blunt end / the trailing edge)."""
        sk = self._sketch(self._plane(s), "%s @%.1f" % (name, s))
        # upper spline (stop -> nose) and lower spline (nose -> stop) meet at
        # the nose, which may be pointed deep in the stack; a straight line
        # closes the stopped end. Same three edges at every station.
        nose = (len(loop) - 1) // 2
        model_pts = [sk.modelToSketchSpace(to_model(self.frame, s, c, u)) for c, u in loop]
        splines = []
        for part in (model_pts[:nose + 1], model_pts[nose:]):
            coll = adsk.core.ObjectCollection.create()
            for p_ in part:
                coll.add(p_)
            splines.append(sk.sketchCurves.sketchFittedSplines.add(coll))
        up, lo = splines
        sk.sketchCurves.sketchLines.addByTwoPoints(lo.endSketchPoint, up.startSketchPoint)
        sk.isComputeDeferred = False
        sk.isVisible = False
        if sk.profiles.count < 1:
            return None
        return max(list(sk.profiles), key=lambda p: p.areaProperties().area)

    def _poly_profile(self, s, poly, name):
        sk = self._sketch(self._plane(s), "%s @%.1f" % (name, s))
        self._polyline(sk, [to_model(self.frame, s, c, u) for c, u in poly])
        sk.isComputeDeferred = False
        sk.isVisible = False
        return sk.profiles.item(0) if sk.profiles.count else None

    # Features created inside a base feature edit return nothing, so new
    # bodies are found by comparing the component's bodies before and after.
    def _all_bodies(self):
        out = list(self.comp.bRepBodies)
        if self.base:
            try:
                out += list(self.base.sourceBodies)
            except Exception:
                pass
        return out

    def _tokens(self):
        return {b.entityToken for b in self._all_bodies()}

    def _new_bodies(self, before, feat=None):
        """Bodies made by a feature: from the feature when it returns one,
        otherwise by comparing the body lists before and after."""
        if feat is not None:
            try:
                if feat.bodies.count:
                    return [feat.bodies.item(i) for i in range(feat.bodies.count)]
            except Exception:
                pass
        seen, out = set(), []
        for b in self._all_bodies():
            t = b.entityToken
            if t not in before and t not in seen:
                seen.add(t)
                out.append(b)
        return out

    def _to_temp(self, reals):
        """Transient copies of real bodies; the real ones are removed."""
        temps = [self.tbm.copy(r) for r in reals]
        for r in reals:
            try:
                r.deleteMe()
            except Exception:
                self.log("Could not remove helper body %s" % r.name)
        return temps

    def _loft_once(self, profs):
        loft_in = self.comp.features.loftFeatures.createInput(
            adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        for p in profs:
            loft_in.loftSections.add(p)
        loft_in.isSolid = True
        if self.base:
            loft_in.targetBaseFeature = self.base
        before = self._tokens()
        feat = self.comp.features.loftFeatures.add(loft_in)
        temps = self._to_temp(self._new_bodies(before, feat))
        if not temps:
            raise RuntimeError("loft produced no body")
        return temps

    def _loft_range(self, stations, profs, i0, i1, name):
        """Loft profs[i0..i1]; on failure split in two halves sharing the middle
        section. Returns [(i0, i1, body)]. Raises _BadSpan(i0, i1) when two
        neighbouring sections cannot be lofted."""
        try:
            return [(i0, i1, b) for b in self._loft_once(profs[i0:i1 + 1])]
        except RuntimeError as e:
            if i1 - i0 <= 1:
                raise _BadSpan(i0, i1, str(e).splitlines()[0] if str(e) else "loft failed")
            mid = (i0 + i1) // 2
            return (self._loft_range(stations, profs, i0, mid, name)
                    + self._loft_range(stations, profs, mid, i1, name))

    def _robust_loft(self, run, make_profile, name, inset=0.0, tips=(True, True)):
        """One smooth loft through run [(s, section)].

        Loft pieces are never joined (joined pieces make unreliable cutting
        tools). When the full loft fails, lofting in halves only locates the
        bad span; then a span near a tip makes the surface end before it (inner
        plies stop short of the tip, as at the trailing edge), a span mid-way
        drops one section, and if no single span fails the most crowded
        section is dropped. Then the full loft is tried again.
        """
        run = list(run)
        cache = {}
        end_shift = max(inset, 0.01)  # keep new ends off shared station planes

        def prof(entry):
            key = (round(entry[0], 6), id(entry[1]))
            if key not in cache:
                cache[key] = make_profile(entry[0], entry[1], name)
            return cache[key]

        crowded_drops = 0
        decimated = False
        for attempt in range(15):
            run = [e for e in run if prof(e) is not None]
            if len(run) < 2:
                return None
            stations = [e[0] for e in run]
            profs = [prof(e) for e in run]
            try:
                return self._loft_once(profs)[0]
            except RuntimeError as err:
                first = str(err).splitlines()[0] if str(err) else "loft failed"
            n = len(run)
            try:
                self._loft_range(stations, profs, 0, n - 1, name)  # diagnosis only
            except _BadSpan as e:
                edge = max(1, n // 4)
                # only a real wing tip may be cut short; a zone end at the centre must
                # reach past the cut plane, so a failure there drops a section instead
                if e.i1 <= edge and tips[0]:
                    self.log("%s: cannot loft s=%.1f..%.1f (%s); surface starts at s=%.1f instead"
                             % (name, stations[e.i0], stations[e.i1], e.msg, stations[e.i1]))
                    run = run[e.i1:]
                    run[0] = (run[0][0] + end_shift, run[0][1])
                elif e.i0 >= n - 1 - edge and tips[1]:
                    self.log("%s: cannot loft s=%.1f..%.1f (%s); surface ends at s=%.1f instead"
                             % (name, stations[e.i0], stations[e.i1], e.msg, stations[e.i0]))
                    run = run[:e.i0 + 1]
                    run[-1] = (run[-1][0] - end_shift, run[-1][1])
                else:
                    drop = e.i1 if e.i1 < n - 1 else e.i0
                    self.log("%s: cannot loft s=%.1f..%.1f (%s); dropping section s=%.1f"
                             % (name, stations[e.i0], stations[e.i1], e.msg, stations[drop]))
                    del run[drop]
                continue
            if n <= 2:
                break
            if crowded_drops >= 3:
                if decimated or n < 8:
                    break
                # every piece lofts but the whole does not: try every other station
                decimated = True
                self.log("%s: full loft still failing; trying every other section (%d -> %d)"
                         % (name, n, (n + 1) // 2 + (0 if n % 2 else 1)))
                run = [e for i, e in enumerate(run) if i % 2 == 0 or i == n - 1]
                continue
            crowded_drops += 1
            # every piece lofts on its own: thin out the most crowded section
            k = min(range(1, n - 1), key=lambda i: stations[i + 1] - stations[i - 1])
            self.log("%s: full loft failed (%s); dropping crowded section s=%.1f" % (name, first, stations[k]))
            del run[k]
        raise FusionError("Could not loft '%s' (around s=%.1f..%.1f). Try more loft points per "
                          "section, a different number of loft stations, or a larger min inner gap"
                          % (name, run[0][0], run[-1][0]) if run else "Could not loft '%s'" % name)

    def loft(self, entries, name, inset=0.0, tips=(True, True)):
        """Transient solid lofted through closed fitted splines at (s, loop2d).

        inset (mm) moves the two end sections inward along the span, so nested
        surfaces never end in the same plane as the body they are cut from
        (coplanar end caps make the booleans fail)."""
        run = engine.longest_run(entries)
        if len(run) < 2:
            return None
        return self._robust_loft(_shift_ends(run, inset, *tips), self._spline_profile, name, inset, tips)

    def loft_polys(self, entries, name, outset=1.0):
        """Transient solid lofted through straight-sided polygons (side masks),
        reaching outset mm past the end sections."""
        return self._robust_loft(_shift_ends(list(entries), -outset), self._poly_profile, name)

    def _polyline(self, sk, model_pts):
        lines = sk.sketchCurves.sketchLines
        sp = [sk.modelToSketchSpace(p) for p in model_pts]
        first = prev = None
        for k in range(len(sp)):
            a = prev.endSketchPoint if prev else sp[k]
            b = sp[(k + 1) % len(sp)] if k + 1 < len(sp) else (first.startSketchPoint if first else sp[0])
            ln = lines.addByTwoPoints(a, b)
            if first is None:
                first = ln
            prev = ln

    def extrude_planform(self, polys, name, depth_mm, keep=None):
        """Transient extrusion of planform (s, c) polygons along the up axis.
        keep(s, c) -> bool selects which sketch regions to extrude."""
        sk = self._sketch(self._up_plane(), name)
        for poly in polys:
            simple = flatten.simplify(poly, 0.05) if len(poly) > 8 else poly
            self._polyline(sk, [to_model(self.frame, s, c, 0) for s, c in simple])
        sk.isComputeDeferred = False
        coll = adsk.core.ObjectCollection.create()
        for prof in sk.profiles:
            cen = prof.areaProperties().centroid
            s, c, _u = _local(self.frame, sk.sketchToModelSpace(cen))
            if keep is None or keep(s, c):
                coll.add(prof)
        if coll.count == 0:
            raise FusionError("No regions to extrude for '%s'" % name)
        ext_in = self.comp.features.extrudeFeatures.createInput(
            coll, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        ext_in.setSymmetricExtent(adsk.core.ValueInput.createByReal(depth_mm * CM), True)
        if self.base:
            ext_in.targetBaseFeature = self.base
        before = self._tokens()
        feat = self.comp.features.extrudeFeatures.add(ext_in)
        sk.isVisible = False
        temps = self._to_temp(self._new_bodies(before, feat))
        if not temps:
            raise FusionError("Extrude of '%s' produced no body" % name)
        body = temps[0]
        for other in temps[1:]:
            self.boolean(body, other, "union")
        return body

    def materialize(self, tmp, name):
        """Add a transient body to the design (inside the base feature)."""
        if self.base:
            b = self.comp.bRepBodies.add(tmp, self.base)
        else:
            b = self.comp.bRepBodies.add(tmp)
        b.name = name
        return b

    def copy(self, body):
        return self.tbm.copy(body)

    def boolean(self, target, tool, op, what=""):
        """In-place boolean on a transient target ('cut', 'intersect', 'union')."""
        kind = {
            "cut": adsk.fusion.BooleanTypes.DifferenceBooleanType,
            "intersect": adsk.fusion.BooleanTypes.IntersectionBooleanType,
            "union": adsk.fusion.BooleanTypes.UnionBooleanType,
        }[op]
        try:
            ok = self.tbm.booleanOperation(target, tool, kind)
        except RuntimeError as e:
            msg = str(e).splitlines()[0] if str(e) else "boolean failed"
            raise FusionError("%s (%s): %s" % (what or "boolean", op, msg))
        if not ok:
            raise FusionError("%s (%s) failed" % (what or "boolean", op))
        return target

    def offset(self, tmp, mm, what="", skin_only=True):
        """Transient copy of tmp with its faces offset by mm (+ out, - in),
        using Fusion's own offset (exact, handles hubs and fillets).
        skin_only leaves planar faces that stand along the up axis (stop
        walls, span-end cuts, a blunt trailing-edge base) where they are, so
        the skin offsets meet them instead of the kernel offsetting tiny faces."""
        real = self.materialize(tmp, "FL offset")
        faces = [real.faces.item(i) for i in range(real.faces.count)]
        if skin_only:
            up = vec(self.frame["U"])
            skin = []
            for f in faces:
                pl = adsk.core.Plane.cast(f.geometry)
                if pl is not None and abs(pl.normal.dotProduct(up)) < 0.1:
                    continue
                skin.append(f)
            if skin:
                faces = skin
        inp = self.comp.features.offsetFacesFeatures.createInput(
            faces, adsk.core.ValueInput.createByReal(mm * CM))
        if self.base:
            inp.targetBaseFeature = self.base
        try:
            self.comp.features.offsetFacesFeatures.add(inp)
        except RuntimeError as e:
            try:
                real.deleteMe()
            except Exception:
                pass
            msg = str(e).splitlines()[0] if str(e) else "offset failed"
            raise FusionError("%s (offset %.3f mm): %s" % (what or "offset", mm, msg))
        return self._to_temp([real])[0]

    def grow(self, tmp, mm):
        """Transient copy of tmp offset outward by mm (via an offset-faces feature)."""
        if mm <= 0:
            return tmp
        real = self.materialize(tmp, "FL grow")
        faces = [real.faces.item(i) for i in range(real.faces.count)]
        inp = self.comp.features.offsetFacesFeatures.createInput(faces, adsk.core.ValueInput.createByReal(mm * CM))
        if self.base:
            inp.targetBaseFeature = self.base
        self.comp.features.offsetFacesFeatures.add(inp)
        return self._to_temp([real])[0]

    def span_box(self, s0, s1):
        """Transient box covering the whole section between span positions s0 and s1."""
        bb = self.frame["bbox_local"]
        ext = BIG_EXTEND * 2
        s0 = max(s0, bb[0][0] - ext)
        s1 = min(s1, bb[1][0] + ext)
        lo = [s0, bb[0][1] - ext, bb[0][2] - ext]
        hi = [s1, bb[1][1] + ext, bb[1][2] + ext]
        centre = to_model(self.frame, (lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2, (lo[2] + hi[2]) / 2)
        obb = adsk.core.OrientedBoundingBox3D.create(
            centre, vec(self.frame["S"]), vec(self.frame["C"]),
            (hi[0] - lo[0]) * CM, (hi[1] - lo[1]) * CM, (hi[2] - lo[2]) * CM)
        return self.tbm.createBox(obb)

    def box(self):
        bb = self.frame["bbox_local"]
        ext = BIG_EXTEND * 2
        lo = [bb[0][k] - ext for k in range(3)]
        hi = [bb[1][k] + ext for k in range(3)]
        centre = to_model(self.frame, (lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2, (lo[2] + hi[2]) / 2)
        obb = adsk.core.OrientedBoundingBox3D.create(
            centre, vec(self.frame["S"]), vec(self.frame["C"]),
            (hi[0] - lo[0]) * CM, (hi[1] - lo[1]) * CM, (hi[2] - lo[2]) * CM)
        return self.tbm.createBox(obb)

    def cleanup(self):
        for sk in self.sketches:
            try:
                sk.deleteMe()
            except Exception:
                pass
        for pl in self.planes:
            try:
                pl.deleteMe()
            except Exception:
                pass


def _local(frame, pt):
    p = (pt.x * MM, pt.y * MM, pt.z * MM)
    d = geom.sub(p, frame["origin"])
    return (geom.dot(d, frame["S"]), geom.dot(d, frame["C"]), geom.dot(d, frame["U"]))


# --------------------------------------------------------------------------- appearance
_APPEARANCE_CACHE = {}


def appearance_for(des, color_hex, name):
    key = color_hex
    if key in _APPEARANCE_CACHE:
        try:
            if _APPEARANCE_CACHE[key].isValid:
                return _APPEARANCE_CACHE[key]
        except Exception:
            pass
    aname = "FoilLayup %s" % color_hex
    ap = des.appearances.itemByName(aname)
    if ap is None:
        lib = None
        for i in range(app().materialLibraries.count):
            l = app().materialLibraries.item(i)
            if "Appearance" in l.name:
                lib = l
                break
        if lib is None:
            return None
        src = None
        for nm in ("Plastic - Matte (Yellow)", "Paint - Enamel Glossy (Yellow)", "Plastic - Glossy (Yellow)"):
            src = lib.appearances.itemByName(nm)
            if src:
                break
        if src is None:
            for i in range(lib.appearances.count):
                a = lib.appearances.item(i)
                if a.appearanceProperties.itemByName("Color"):
                    src = a
                    break
        if src is None:
            return None
        ap = des.appearances.addByCopy(src, aname)
    prop = adsk.core.ColorProperty.cast(ap.appearanceProperties.itemByName("Color"))
    if prop:
        h = color_hex.lstrip("#")
        prop.value = adsk.core.Color.create(int(h[0:2], 16), int(h[2:4], 16), int(h[4:6], 16), 0)
    _APPEARANCE_CACHE[key] = ap
    return ap


def tag(body, role, data):
    try:
        d = dict(data)
        d["role"] = role
        body.attributes.add(ATTR, "role", json.dumps(d))
    except Exception:
        pass


# --------------------------------------------------------------------------- main entry
def remove_previous(des, log):
    """Delete the timeline group (or tagged bodies) from a previous run."""
    removed = False
    if des.designType == adsk.fusion.DesignTypes.ParametricDesignType:
        groups = des.timeline.timelineGroups
        for i in range(groups.count - 1, -1, -1):
            g = groups.item(i)
            if g.name.startswith("Foil Layup"):
                g.deleteMe(True)
                removed = True
        if not removed:
            for i in range(des.timeline.count - 1, -1, -1):
                ent = des.timeline.item(i).entity
                try:
                    if ent and ent.name.startswith("Foil Layup"):
                        des.timeline.item(i).entity.deleteMe()
                        removed = True
                except Exception:
                    pass
    else:
        for comp in des.allComponents:
            for b in list(comp.bRepBodies):
                if b.attributes.itemByName(ATTR, "role"):
                    b.deleteMe()
                    removed = True
    if removed:
        log("Removed previous layup")


def generate(stack_data, user_materials, log, progress):
    des = design()
    target = find_by_token((stack_data.get("target") or {}).get("token"))
    if target is None:
        raise FusionError("Pick the foil body first (Setup tab).")
    target = adsk.fusion.BRepBody.cast(target)
    mats = materials.material_index(user_materials)
    stack = model.Stack(stack_data, mats)
    st = stack.settings

    remove_previous(des, log)
    target = adsk.fusion.BRepBody.cast(find_by_token(stack_data["target"]["token"]))
    native, comp, _inv = target_context(target)
    frame = detect_frame(target, stack_data.get("frame") or {})
    log("Frame: span %s, up %s%s, chord %s (Fusion up preference: %s)" % (
        frame["spanAxis"], "-" if (stack_data.get("frame") or {}).get("flipUp") else "+",
        frame["upAxis"], frame["chordAxis"], frame["preferredUp"]))

    progress("Sampling sections", 0, 1)
    raw = sample_sections(native, frame, int(st["loftStations"]), float(st["tipInset"]),
                          model.span_limit_stations(stack, frame) + ([0.0] if frame.get("symmetric", True) else []),
                          lambda k, n: progress("Sampling sections", k, n))
    sections = engine.build_sections(raw, int(st["loftPoints"]), float(st["chordRefSmoothing"]))
    if len(sections) < 3:
        raise FusionError("Could not slice the body; check the span axis.")
    lo = [min(sec.s for sec in sections), min(p[0] for sec in sections for p in sec.pts),
          min(p[1] for sec in sections for p in sec.pts)]
    hi = [max(sec.s for sec in sections), max(p[0] for sec in sections for p in sec.pts),
          max(p[1] for sec in sections for p in sec.pts)]
    lo[0], hi[0] = frame["sMin"], frame["sMax"]
    frame["bbox_local"] = (lo, hi)
    bounds = load_bounds(stack.bound_sketches, frame, log)
    _dump_debug(stack_data, user_materials, frame, raw, bounds, log)
    engine.log_tip_schedule(engine.tip_schedule(stack, sections, frame, bounds), frame, log)
    deps, ext = engine.depth_tables(stack, sections, frame, bounds)
    plan = engine.loft_plan(stack, sections, deps, ext, float(st["minGap"]))
    pieces, _d, _e = engine.plan_pieces(stack, sections, frame, bounds, log, stack_data.get("seamPlan"))
    seam_plan = engine.seam_plan_of(pieces)

    tl_start = des.timeline.count if des.designType == adsk.fusion.DesignTypes.ParametricDesignType else None
    base = None
    if tl_start is not None:
        base = comp.features.baseFeatures.add()
        base.name = "Foil Layup"
        base.startEdit()
    b = Builder(comp, base, frame, log, progress)
    pending = []  # (transient body, name, role, info, material)
    made = []     # (real body, role, info, material)
    skipped = []  # plies that could not be built
    try:
        n_rows = len(stack.units)
        step = _end_step(sections, max(n_rows, len(stack.ext_units)))
        thickness_span = 2 * max(abs(hi[2]), abs(lo[2])) + 2 * BIG_EXTEND
        ctx = dict(b=b, stack=stack, native=native, plan=plan, sections=sections, frame=frame,
                   bounds=bounds, step=step, thickness_span=thickness_span, log=log,
                   progress=progress, skipped=skipped)
        if st.get("construction", "loft") == "offset":
            pending = _build_offsets(ctx)
        elif st.get("splitAtCentre") and frame.get("symmetric", True) and sections[0].s < 0 < sections[-1].s:
            # One smooth loft through a whole symmetric wing (hub included) is
            # fragile; each half is built on its own and joined at the centre plane.
            halves = []
            for zone in ((-1e9, 0.0), (0.0, 1e9)):
                progress("Building %s half" % ("left" if zone[0] < 0 else "right"), 0, 1)
                halves.append(_build_zone(ctx, zone))
            pending = _join_halves(b, halves, log)
        else:
            pending = _build_zone(ctx, None)
        # keep-outs: cut every result, keep the grown tool (hidden) for Flatten
        tools = []
        for tmp, clr, name in keepout_bodies(stack_data, target, log):
            progress("Keep-out: %s" % name, n_rows + 1, n_rows + 2)
            tool = b.grow(tmp, clr)
            for p in pending:
                try:
                    b.boolean(p[0], tool, "cut", "keep-out %s" % name)
                except FusionError as e:
                    log("Keep-out %s did not cut %s (%s)" % (name, p[1], e))
            tools.append((tool, name, clr))
        progress("Adding bodies", n_rows + 2, n_rows + 2)
        for tmp, name, role, info, mat in pending:
            if _has_volume(tmp):
                made.append((b.materialize(tmp, name), role, info, mat))
        for tool, name, clr in tools:
            real = b.materialize(tool, "FL keep-out %s" % name)
            real.isLightBulbOn = False
            tag(real, "keepout", {"name": name, "clearance": clr})
    finally:
        b.cleanup()
        if base:
            base.finishEdit()

    # appearance, tags
    for body, role, info, mat in made:
        try:
            ap = appearance_for(des, mat["color"], mat["name"])
            if ap:
                body.appearance = ap
        except Exception:
            pass
        tag(body, role, info)

    if st.get("hideOriginal", True):
        try:
            target.isLightBulbOn = False
        except Exception:
            pass

    _seam_sketch(comp, frame, pieces, sections, log)

    if tl_start is not None and des.timeline.count - tl_start > 1:
        try:
            g = des.timeline.timelineGroups.add(tl_start, des.timeline.count - 1)
            g.name = "Foil Layup"
        except Exception:
            pass

    report = []
    for body, role, info, mat in made:
        try:
            vol = body.physicalProperties.volume  # cm3
        except Exception:
            vol = 0.0
        lam = vol * mat["density"] / 1000.0
        dry = None
        t = info.get("t") if isinstance(info, dict) else None
        if role in ("ply", "external", "fill") and t and mat.get("gsm"):
            area_m2 = vol / (t / 10.0) * 1e-4  # volume / thickness
            dry = area_m2 * mat["gsm"]
        report.append({
            "name": body.name, "role": role, "material": mat["name"],
            "volume_cm3": round(vol, 3), "mass_g": round(lam, 2),
            "dry_g": round(dry, 2) if dry is not None else None,
        })
    plies = [r for r in report if r["role"] in ("ply", "fill")]
    lam_plies = sum(r["mass_g"] for r in plies)
    dry_plies = sum(r["dry_g"] or 0 for r in plies)
    core = sum(r["mass_g"] for r in report if r["role"] in ("core", "corefill", "gap"))
    part = lam_plies + core
    ext = sum(r["mass_g"] for r in report if r["role"] == "external")
    log("Mass: dry fibre %.1f g, resin %.1f g, laminate %.1f g, core %.1f g, finished part %.1f g%s" % (
        dry_plies, lam_plies - dry_plies, lam_plies, core, part,
        (", external layers %.1f g (removed after cure)" % ext) if ext else ""))
    return {"bodies": report, "seamPlan": seam_plan, "skipped": skipped,
            "seams": [{"piece": pc.id, "label": pc.label, "msg": pc.seam["msg"], "ok": pc.seam["ok"]}
                      for pc in pieces if pc.seam],
            "frame": {k: frame[k] for k in ("spanAxis", "upAxis", "chordAxis", "sMin", "sMax", "hand")}}


def _dump_debug(stack_data, user_materials, frame, raw, bounds, log):
    """Save what the engine works from, so a failing run can be reproduced
    outside Fusion (APPDATA/FoilLayup/last_generate.json)."""
    try:
        folder = os.path.join(os.environ.get("APPDATA", os.path.expanduser("~")), "FoilLayup")
        os.makedirs(folder, exist_ok=True)
        data = {
            "stack": stack_data,
            "userMaterials": user_materials or [],
            "frame": {k: v for k, v in frame.items() if k != "bbox_local"},
            "raw": [[s, loop] for s, loop in raw],
            "bounds": bounds,
        }
        with open(os.path.join(folder, "last_generate.json"), "w", encoding="utf8") as f:
            json.dump(data, f)
    except Exception as e:
        log("Debug dump skipped (%s)" % e)


def _stop_prism(b, entries, depth, name, sections=None, te_margin=0.0, drop_ends=0):
    """Vertical prism over the planform where a surface exists: from far ahead
    of the LE back to its trimmed tip at each station, spanning the stations
    where it exists. Cutting the surface above with it before offsetting
    removes the regions too thin for the next ply, so the offset never fails
    there and the ply ends bluntly at this line."""
    te_of = {round(sec.s, 6): sec.te_pt[0] for sec in sections or []}
    present = [(s_, loop) for s_, loop in entries if loop is not None]
    if drop_ends:
        present = present[drop_ends:-drop_ends]
    if len(present) < 2:
        return None
    front = min(min(p[0] for p in loop) for _s, loop in present) - BIG_EXTEND
    aft = []
    for s_, loop in present:
        c = max(p[0] for p in loop)
        te = te_of.get(round(s_, 6))
        if te is not None:
            c = min(c, te - te_margin)  # keep the stop wall clear of the trailing edge
        aft.append((s_, c))
    poly = aft + [(present[-1][0], front), (present[0][0], front)]
    return b.extrude_planform([poly], name, depth)


def _build_offsets(ctx):
    """Plies, fills, core and external layers built from exact offsets of the
    body (no lofted surfaces). Returns pending (transient, name, role, info, material)."""
    b, stack, native, plan = ctx["b"], ctx["stack"], ctx["native"], ctx["plan"]
    sections, frame, bounds = ctx["sections"], ctx["frame"], ctx["bounds"]
    thickness_span = ctx["thickness_span"]
    log, progress, skipped = ctx["log"], ctx["progress"], ctx["skipped"]
    pending = []
    n_rows = len(stack.units)
    cur = b.copy(native)
    for ri, row in enumerate(stack.units):
        progress("Layer %d/%d: %s" % (ri + 1, n_rows, row.name), ri, n_rows + 2)
        label = "FL%02d %s" % (ri + 1, row.name)
        info = {"row": row.id, "name": row.name, "material": row.material["id"],
                "angles": [p.angle for p in row.plies], "t": row.t}
        if cur is None:
            log("Layer '%s' has no room left; skipped" % row.name)
            continue
        key = "full:%s" % row.id if ("full:%s" % row.id) in plan else "after:%s" % row.id
        try:
            inner, no_room = None, False
            last = None
            # stop line clear of the trailing edge; on failure move it further
            # forward and pull the ends back from the wingtips, then retry
            for te_margin, drop_ends in ((max(1.0, 4 * row.t), 0), (2.5, 1), (5.0, 3)):
                prism = _stop_prism(b, plan[key], thickness_span, "FL stop %d" % (ri + 1),
                                    sections, te_margin, drop_ends)
                if prism is None:
                    no_room = True
                    break
                base_ = b.copy(cur)
                try:
                    b.boolean(base_, prism, "intersect", "%s: trim where the next ply no longer fits" % label)
                    inner = b.offset(base_, -row.t, "%s: inner surface" % label)
                    if te_margin > max(1.0, 4 * row.t) or drop_ends:
                        log("%s: stop line moved %.1f mm off the trailing edge%s to make the offset work"
                            % (label, te_margin, " and %d stations back from the tips" % drop_ends if drop_ends else ""))
                    break
                except FusionError as e:
                    last = e
            if inner is None and not no_room:
                raise last
            mask = None
            if row.side != "wrap":
                quads = [(sec.s, engine.side_mask_quads(sec, row.side, sec.chord + BIG_EXTEND))
                         for sec in sections]
                mask = b.loft_polys(quads, "FL side mask %d" % (ri + 1))
            region = (_region_tool(b, row, sections, frame, bounds, thickness_span, ri, include_tip=False)
                      if row.has_user_region else None)
            tip = None
            if row.tip_end is not None:  # spanwise end from the tip drop schedule
                if frame.get("symmetric", True):
                    tip = b.span_box(-row.tip_end, row.tip_end)
                else:
                    tip = b.span_box(-1e9, frame["sMin"] + row.tip_end)
        except FusionError as e:
            log("WARNING: %s could not be built and was skipped (%s)" % (label, e))
            skipped.append(label)
            continue
        slot = fill = None
        errors = []
        for order in ("limits first", "cut first"):
            try:
                slot, fill = _ply_body(b, cur, inner, mask, region, bool(row.outside), order, label, row.side)
                break
            except FusionError as e:
                errors.append("%s: %s" % (order, e))
                slot = fill = None
        if slot is None:
            log("WARNING: %s could not be built and was skipped (%s)" % (label, "; ".join(errors)))
            skipped.append(label)
            continue
        if tip is not None:
            for part in (slot, fill):
                if part is not None and _has_volume(part):
                    b.boolean(part, tip, "intersect", "%s: tip end" % label)
        if _has_volume(slot):
            pending.append((slot, label, "ply", info, row.material))
        else:
            log("Layer '%s' came out empty" % row.name)
        if fill is not None and _has_volume(fill):
            if row.outside == "core":
                pending.append((fill, "%s fill (core)" % label, "corefill", info, stack.core_material))
            else:
                m = stack.materials[row.outside]
                pending.append((fill, "%s fill %s" % (label, m["name"]), "fill",
                                dict(info, material=m["id"]), m))
        # The next surface is always the clean full-depth offset (offsetting a
        # pocket one ply deep would collapse its walls). Where a limited ply is
        # absent, the strip it leaves is a "drape gap": in the part the inner
        # layers drape outward and the core grows by that volume, so it is
        # counted with the core material and the part mass stays right.
        if inner is None:
            cur = None
        else:
            nxt = inner
            if tip is not None:
                # past the ply's end the layers inside move outward into its place
                try:
                    nxt = b.copy(inner)
                    b.boolean(nxt, tip, "intersect", "%s: surface inside the ply end" % label)
                    beyond = b.copy(cur)
                    b.boolean(beyond, tip, "cut", "%s: surface past the ply end" % label)
                    if _has_volume(beyond):
                        b.boolean(nxt, beyond, "union", "%s: step at the ply end" % label)
                except FusionError as e:
                    log("WARNING: %s: layers inside cannot step out past its end (%s)" % (label, e))
                    nxt = inner
            if row.side != "wrap" or (row.has_user_region and not row.outside):
                try:
                    gap = b.copy(cur)
                    b.boolean(gap, nxt, "cut", "%s: drape gap" % label)
                    for part in (slot, fill):
                        if part is not None and _has_volume(part):
                            b.boolean(gap, part, "cut", "%s: drape gap" % label)
                    if _has_volume(gap):
                        pending.append((gap, "%s drape gap" % label, "gap", info, stack.core_material))
                except FusionError as e:
                    log("%s: drape gap not built (%s)" % (label, e))
            cur = nxt
    if cur is not None and _has_volume(cur):
        for p in [p for p in pending if p[2] == "corefill"]:
            try:
                b.boolean(cur, p[0], "union", "core + %s" % p[1])
                pending.remove(p)
            except FusionError as e:
                log("Could not join %s into the core (%s); kept separate" % (p[1], e))
        pending.append((cur, stack.core_name, "core", {"material": stack.core_material["id"]},
                        stack.core_material))
    # external consumables: lofted outward offsets (an outward offset of the
    # whole body fails at the singular wingtip point)
    prev = b.copy(native)
    for ri, row in enumerate(stack.ext_units):
        progress("External %d/%d: %s" % (ri + 1, len(stack.ext_units), row.name), n_rows, n_rows + 2)
        name = "FX%02d %s" % (ri + 1, row.name)
        try:
            outer = b.loft(plan["ext:%s" % row.id], "FL E%d" % (ri + 1), ctx["step"] * (ri + 1))
            if outer is None:
                continue
            keep_outer = b.copy(outer)
            b.boolean(outer, prev, "cut", name)
        except FusionError as e:
            log("WARNING: %s could not be built and was skipped (%s)" % (name, e))
            skipped.append(name)
            continue
        pending.append((outer, name, "external",
                        {"row": row.id, "name": row.name, "material": row.material["id"], "t": row.t},
                        row.material))
        prev = keep_outer
    return pending


def _zone_entries(entries, zone):
    """Stations of a loft inside a span zone, plus the first one beyond each
    interior boundary, so the loft runs through the centre and is cut there."""
    if zone is None:
        return entries
    z0, z1 = zone
    inside = [i for i, (s, _l) in enumerate(entries) if z0 <= s <= z1]
    if not inside:
        return []
    i0, i1 = inside[0], inside[-1]
    if z0 > -1e8 and i0 > 0:
        i0 -= 1
    if z1 < 1e8 and i1 < len(entries) - 1:
        i1 += 1
    return entries[i0:i1 + 1]


def _build_zone(ctx, zone):
    """Plies, fills, core and external layers for one span zone (None = all).
    Returns pending (transient body, name, role, info, material), trimmed to the zone."""
    b, stack, native, plan = ctx["b"], ctx["stack"], ctx["native"], ctx["plan"]
    sections, frame, bounds = ctx["sections"], ctx["frame"], ctx["bounds"]
    step, thickness_span = ctx["step"], ctx["thickness_span"]
    log, progress, skipped = ctx["log"], ctx["progress"], ctx["skipped"]
    side = "" if zone is None else (" (left)" if zone[0] < 0 else " (right)")

    def zp(key):
        return _zone_entries(plan[key], zone)

    zsecs = [sec for _s, sec in _zone_entries([(sec.s, sec) for sec in sections], zone)]
    tips = (True, True) if zone is None else (zone[0] < -1e8, zone[1] > 1e8)
    pending = []
    n_rows = len(stack.units)
    cur = b.copy(native)
    for ri, row in enumerate(stack.units):
        progress("Layer %d/%d: %s" % (ri + 1, n_rows, row.name), ri, n_rows + 2)
        label = "FL%02d %s" % (ri + 1, row.name)
        info = {"row": row.id, "name": row.name, "material": row.material["id"],
                "angles": [p.angle for p in row.plies], "t": row.t}
        if cur is None:
            log("Layer '%s' has no room left; skipped" % row.name)
            continue
        inset = step * (ri + 1)  # each inner surface ends a little inside the one above
        after = b.loft(zp("after:%s" % row.id), "FL S%d" % (ri + 1), inset, tips)
        # tools: the inner surface to cut out, and the masks limiting this ply
        slot = fill = None
        errors = []
        try:
            if row.is_full:
                inner = after
            else:
                inner = b.loft(zp("full:%s" % row.id), "FL F%d" % (ri + 1), inset, tips)
            mask = None
            if row.side != "wrap":
                quads = [(sec.s, engine.side_mask_quads(sec, row.side, sec.chord + BIG_EXTEND))
                         for sec in zsecs]
                mask = b.loft_polys(quads, "FL side mask %d" % (ri + 1))
            region = (_region_tool(b, row, sections, frame, bounds, thickness_span, ri)
                      if row.has_region else None)
        except FusionError as e:
            log("WARNING: %s%s could not be built and was skipped (tools: %s)" % (label, side, e))
            skipped.append(label + side)
            cur = after
            continue
        # Limits first on a fresh copy of the surface above, then the cut, is the
        # gentlest order for the kernel; the other order is the fallback.
        for order in ("limits first", "cut first"):
            try:
                slot, fill = _ply_body(b, cur, inner, mask, region, bool(row.outside), order, label, row.side)
                break
            except FusionError as e:
                errors.append("%s: %s" % (order, e))
                slot = fill = None
        if slot is None:
            log("WARNING: %s%s could not be built and was skipped (%s)" % (label, side, "; ".join(errors)))
            skipped.append(label + side)
            cur = after
            continue
        if _has_volume(slot):
            pending.append((slot, label, "ply", info, row.material))
        else:
            log("Layer '%s' came out empty" % row.name)
        if fill is not None and _has_volume(fill):
            if row.outside == "core":
                pending.append((fill, "%s fill (core)" % label, "corefill", info, stack.core_material))
            else:
                m = stack.materials[row.outside]
                pending.append((fill, "%s fill %s" % (label, m["name"]), "fill",
                                dict(info, material=m["id"]), m))
        cur = after
    # core, with any 'core' fills joined in
    if cur is not None and _has_volume(cur):
        for p in [p for p in pending if p[2] == "corefill"]:
            try:
                b.boolean(cur, p[0], "union", "core + %s" % p[1])
                pending.remove(p)
            except FusionError as e:
                log("Could not join %s into the core (%s); kept separate" % (p[1], e))
        pending.append((cur, stack.core_name, "core", {"material": stack.core_material["id"]},
                        stack.core_material))
    # external layers: ext_k = E_k - E_{k-1}, E_0 = the original body
    prev = b.copy(native)
    for ri, row in enumerate(stack.ext_units):
        progress("External %d/%d: %s" % (ri + 1, len(stack.ext_units), row.name), n_rows, n_rows + 2)
        # outer layers end slightly inside the layer below so it cuts cleanly through
        outer = b.loft(zp("ext:%s" % row.id), "FL E%d" % (ri + 1), step * (ri + 1), tips)
        if outer is None:
            continue
        keep_outer = b.copy(outer)
        b.boolean(outer, prev, "cut", "FX%02d %s" % (ri + 1, row.name))
        pending.append((outer, "FX%02d %s" % (ri + 1, row.name), "external",
                        {"row": row.id, "name": row.name, "material": row.material["id"]}, row.material))
        prev = keep_outer
    if zone is not None:
        box = b.span_box(zone[0], zone[1])
        kept = []
        for item in pending:
            try:
                b.boolean(item[0], box, "intersect", "%s: trim to%s" % (item[1], side))
                if _has_volume(item[0]):
                    kept.append(item)
            except FusionError as e:
                log("WARNING: %s%s could not be trimmed at the centre (%s)" % (item[1], side, e))
        pending = kept
    return pending


def _join_halves(b, halves, log):
    """Join left/right bodies of the same name; keep both if a join fails."""
    left, right = halves
    rmap = {}
    for item in right:
        rmap.setdefault(item[1], []).append(item)
    out = []
    for item in left:
        mates = rmap.get(item[1])
        if mates:
            other = mates.pop(0)
            try:
                b.boolean(item[0], other[0], "union", "%s: join halves" % item[1])
                out.append(item)
                continue
            except FusionError as e:
                log("%s: halves kept as separate bodies (%s)" % (item[1], e))
                out.append((item[0], item[1] + " L") + tuple(item[2:]))
                out.append((other[0], other[1] + " R") + tuple(other[2:]))
                continue
        out.append(item)
    for items in rmap.values():
        out.extend(items)
    return out


def _ply_body(b, outer, inner, mask, region, with_fill, order, label, side):
    """Ply body = (outer surface - inner surface) limited by the side mask and
    region; fill = the same slot outside the region. Works on copies so a
    failed attempt leaves 'outer' untouched for the next one."""
    def limit(body):
        fill = None
        if mask is not None:
            b.boolean(body, mask, "intersect", "%s: %s-side mask" % (label, side))
        if region is not None:
            if with_fill:
                fill = b.copy(body)
                b.boolean(fill, region, "cut", "%s: fill outside limits" % label)
            b.boolean(body, region, "intersect", "%s: span/chord/bound limits" % label)
        return fill

    body = b.copy(outer)
    if order == "limits first":
        fill = limit(body)
        if inner is not None:
            b.boolean(body, inner, "cut", "%s: inner surface" % label)
            if fill is not None:
                b.boolean(fill, inner, "cut", "%s: fill inner surface" % label)
    else:
        if inner is not None:
            b.boolean(body, inner, "cut", "%s: inner surface" % label)
        fill = limit(body)
    return body, fill


def _shift_ends(run, inset, lo=True, hi=True):
    """Move the first and last (s, loop) stations inward by inset mm (outward if negative)."""
    if not inset or len(run) < 2:
        return run
    run = list(run)
    if lo:
        run[0] = (run[0][0] + inset, run[0][1])
    if hi:
        run[-1] = (run[-1][0] - inset, run[-1][1])
    return run


def _end_step(sections, n_surfaces):
    """Per-surface end inset: small, but always well inside the first station gap."""
    gaps = [sections[1].s - sections[0].s, sections[-1].s - sections[-2].s]
    return max(0.002, min(0.05, 0.4 * min(gaps) / max(n_surfaces + 1, 1)))


def _has_volume(tmp):
    try:
        return tmp is not None and tmp.lumps.count > 0 and tmp.faces.count > 0
    except Exception:
        return False


def _region_tool(b, row, sections, frame, bounds, depth, ri, include_tip=True):
    """Transient solid covering the row's chord/span limits and bound sketch."""
    polys = engine.region_planform_polys(row, sections, frame, BIG_EXTEND, include_tip)
    tool = b.extrude_planform(polys, "FL region %d" % (ri + 1), depth) if polys else None
    if row.bound_sketch:
        bp = bounds.get(row.bound_sketch) or []
        if not bp:
            raise FusionError("Bound sketch '%s' has no closed loops" % row.bound_sketch)
        btool = b.extrude_planform(bp, "FL bound %d" % (ri + 1), depth,
                                   keep=lambda s, c: geom.signed_distance_polys((s, c), bp) > 0)
        if row.bound_mode == "outside":
            tool = tool if tool is not None else b.box()
            b.boolean(tool, btool, "cut")
        elif tool is None:
            tool = btool
        else:
            b.boolean(tool, btool, "intersect")
    elif tool is None:
        tool = b.box()
    return tool


def _seam_sketch(comp, frame, pieces, sections, log):
    """3D seam lines for each wrapped ply, on its mid-surface."""
    wrapped = [pc for pc in pieces if pc.seam]
    if not wrapped:
        return
    try:
        sk = comp.sketches.add(comp.xYConstructionPlane)
        sk.name = "Foil Layup seams"
        sk.isComputeDeferred = True
        for pc in wrapped:
            pts = adsk.core.ObjectCollection.create()
            for i, sec in enumerate(sections):
                fj = pc.seam["fj"][i] % sec.m
                j0 = int(fj)
                t = fj - j0
                j1 = (j0 + 1) % sec.m
                d0, d1 = pc.depths[i][j0], pc.depths[i][j1]
                p0 = sec.pts[j0]
                p1 = sec.pts[j1]
                n0, n1 = sec.n[j0], sec.n[j1]
                c = (p0[0] + n0[0] * d0) * (1 - t) + (p1[0] + n1[0] * d1) * t
                u = (p0[1] + n0[1] * d0) * (1 - t) + (p1[1] + n1[1] * d1) * t
                pts.add(sk.modelToSketchSpace(to_model(frame, sec.s, c, u)))
            if pts.count >= 2:
                sk.sketchCurves.sketchFittedSplines.add(pts)
        sk.isComputeDeferred = False
    except Exception as e:
        log("Seam sketch skipped (%s)" % e)
