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
        sk = self._sketch(self._plane(s), "%s @%.1f" % (name, s))
        pts = adsk.core.ObjectCollection.create()
        for c, u in loop:
            pts.add(sk.modelToSketchSpace(to_model(self.frame, s, c, u)))
        spl = sk.sketchCurves.sketchFittedSplines.add(pts)
        spl.isClosed = True
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

    def _robust_loft(self, run, make_profile, name, inset=0.0):
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
                if e.i1 <= edge:
                    self.log("%s: cannot loft s=%.1f..%.1f (%s); surface starts at s=%.1f instead"
                             % (name, stations[e.i0], stations[e.i1], e.msg, stations[e.i1]))
                    run = run[e.i1:]
                    run[0] = (run[0][0] + end_shift, run[0][1])
                elif e.i0 >= n - 1 - edge:
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
            if n <= 2 or crowded_drops >= 3:
                break
            crowded_drops += 1
            # every piece lofts on its own: thin out the most crowded section
            k = min(range(1, n - 1), key=lambda i: stations[i + 1] - stations[i - 1])
            self.log("%s: full loft failed (%s); dropping crowded section s=%.1f" % (name, first, stations[k]))
            del run[k]
        raise FusionError("Could not loft '%s' (around s=%.1f..%.1f). Try more loft points per "
                          "section, a different number of loft stations, or a larger min inner gap"
                          % (name, run[0][0], run[-1][0]) if run else "Could not loft '%s'" % name)

    def loft(self, entries, name, inset=0.0):
        """Transient solid lofted through closed fitted splines at (s, loop2d).

        inset (mm) moves the two end sections inward along the span, so nested
        surfaces never end in the same plane as the body they are cut from
        (coplanar end caps make the booleans fail)."""
        run = engine.longest_run(entries)
        if len(run) < 2:
            return None
        return self._robust_loft(_shift_ends(run, inset), self._spline_profile, name, inset)

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
                          model.span_limit_stations(stack, frame),
                          lambda k, n: progress("Sampling sections", k, n))
    sections = engine.build_sections(raw, int(st["loftPoints"]))
    if len(sections) < 3:
        raise FusionError("Could not slice the body; check the span axis.")
    lo = [min(sec.s for sec in sections), min(p[0] for sec in sections for p in sec.pts),
          min(p[1] for sec in sections for p in sec.pts)]
    hi = [max(sec.s for sec in sections), max(p[0] for sec in sections for p in sec.pts),
          max(p[1] for sec in sections for p in sec.pts)]
    lo[0], hi[0] = frame["sMin"], frame["sMax"]
    frame["bbox_local"] = (lo, hi)
    bounds = load_bounds(stack.bound_sketches, frame, log)
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
    try:
        cur = b.copy(native)
        n_rows = len(stack.units)
        step = _end_step(sections, max(n_rows, len(stack.ext_units)))
        thickness_span = 2 * max(abs(hi[2]), abs(lo[2])) + 2 * BIG_EXTEND
        for ri, row in enumerate(stack.units):
            progress("Layer %d/%d: %s" % (ri + 1, n_rows, row.name), ri, n_rows + 2)
            label = "FL%02d %s" % (ri + 1, row.name)
            info = {"row": row.id, "name": row.name, "material": row.material["id"],
                    "angles": [p.angle for p in row.plies], "t": row.t}
            if cur is None:
                log("Layer '%s' has no room left; skipped" % row.name)
                continue
            inset = step * (ri + 1)  # each inner surface ends a little inside the one above
            after = b.loft(plan["after:%s" % row.id], "FL S%d" % (ri + 1), inset)
            slot = cur  # everything outside the next surface belongs to this row
            if row.is_full:
                if after is not None:
                    b.boolean(slot, after, "cut", "%s: inner surface" % label)
            else:
                full = b.loft(plan["full:%s" % row.id], "FL F%d" % (ri + 1), inset)
                if full is not None:
                    b.boolean(slot, full, "cut", "%s: full-thickness surface" % label)
                if row.side != "wrap":
                    quads = [(sec.s, engine.side_mask_quads(sec, row.side, sec.chord + BIG_EXTEND))
                             for sec in sections]
                    b.boolean(slot, b.loft_polys(quads, "FL side mask %d" % (ri + 1)), "intersect",
                              "%s: %s-side mask" % (label, row.side))
            fill = None
            if row.has_region:
                region = _region_tool(b, row, sections, frame, bounds, thickness_span, ri)
                if row.outside:
                    fill = b.copy(slot)
                    b.boolean(fill, region, "cut", "%s: fill outside limits" % label)
                b.boolean(slot, region, "intersect", "%s: span/chord/bound limits" % label)
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
            outer = b.loft(plan["ext:%s" % row.id], "FL E%d" % (ri + 1), step * (ri + 1))
            if outer is None:
                continue
            keep_outer = b.copy(outer)
            b.boolean(outer, prev, "cut", "FX%02d %s" % (ri + 1, row.name))
            pending.append((outer, "FX%02d %s" % (ri + 1, row.name), "external",
                            {"row": row.id, "name": row.name, "material": row.material["id"]}, row.material))
            prev = keep_outer
        # keep-outs: cut every result, keep the grown tool (hidden) for Flatten
        tools = []
        for tmp, clr, name in keepout_bodies(stack_data, target):
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
        report.append({
            "name": body.name, "role": role, "material": mat["name"],
            "volume_cm3": round(vol, 3), "mass_g": round(vol * mat["density"] / 1000.0, 2),
        })
    part = sum(r["mass_g"] for r in report if r["role"] in ("ply", "fill", "core", "corefill"))
    ext = sum(r["mass_g"] for r in report if r["role"] == "external")
    log("Mass: plies %.1f g, core %.1f g, finished part %.1f g%s" % (
        sum(r["mass_g"] for r in report if r["role"] == "ply"),
        sum(r["mass_g"] for r in report if r["role"] in ("core", "corefill")), part,
        (", external layers %.1f g" % ext) if ext else ""))
    return {"bodies": report, "seamPlan": seam_plan,
            "seams": [{"piece": pc.id, "label": pc.label, "msg": pc.seam["msg"], "ok": pc.seam["ok"]}
                      for pc in pieces if pc.seam],
            "frame": {k: frame[k] for k in ("spanAxis", "upAxis", "chordAxis", "sMin", "sMax", "hand")}}


def _shift_ends(run, inset):
    """Move the first and last (s, loop) stations inward by inset mm (outward if negative)."""
    if not inset or len(run) < 2:
        return run
    run = list(run)
    run[0] = (run[0][0] + inset, run[0][1])
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


def _region_tool(b, row, sections, frame, bounds, depth, ri):
    """Transient solid covering the row's chord/span limits and bound sketch."""
    polys = engine.region_planform_polys(row, sections, frame, BIG_EXTEND)
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
