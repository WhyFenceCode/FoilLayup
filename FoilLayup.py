"""Foil Layup - composite layer design, splitting and flattening for Fusion.

Adds a "Foil Layup" panel to Design > Utilities with one button that opens the
layup palette. Heavy work (Generate, Flatten) runs inside Fusion commands so
each run is a single undo step.
"""
import json
import os
import sys
import traceback

import adsk.core
import adsk.fusion

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)

from foil_layup import engine, materials, model  # noqa: E402
from foil_layup import fusion_build, fusion_flat, fusion_geo  # noqa: E402

APP_ID = "FoilLayup"
PALETTE_ID = "FoilLayupPalette"
CMD_OPEN = "FoilLayupOpen"
CMD_RUN = "FoilLayupRun"
PANEL_ID = "SolidScriptsAddinsPanel"
DATA_DIR = os.path.join(os.environ.get("APPDATA", HERE), "FoilLayup")
LIBRARY_FILE = os.path.join(DATA_DIR, "library.json")
LOG_FILE = os.path.join(DATA_DIR, "log.txt")

_handlers = []
_SPAN_CACHE = {}  # span sections for previews, keyed by body + frame + resolution
_pending = {}  # job queued for the run command


def _app():
    return adsk.core.Application.get()


def _ui():
    return _app().userInterface


# --------------------------------------------------------------------------- logging
class Log:
    def __init__(self):
        self.lines = []

    def __call__(self, msg):
        self.lines.append(str(msg))
        try:
            os.makedirs(DATA_DIR, exist_ok=True)
            with open(LOG_FILE, "a", encoding="utf8") as f:
                f.write(str(msg) + "\n")
        except OSError:
            pass


def _send(action, payload):
    pal = _ui().palettes.itemById(PALETTE_ID)
    if pal:
        pal.sendInfoToHTML(action, json.dumps(payload))


# --------------------------------------------------------------------------- persistence
def _load_library():
    try:
        with open(LIBRARY_FILE, encoding="utf8") as f:
            return json.load(f)
    except (OSError, ValueError):
        return {"materials": [], "presets": []}


def _save_library(lib):
    os.makedirs(DATA_DIR, exist_ok=True)
    with open(LIBRARY_FILE, "w", encoding="utf8") as f:
        json.dump(lib, f, indent=1)


def _load_stack():
    try:
        des = fusion_geo.design()
        a = des.attributes.itemByName(APP_ID, "stack")
        return json.loads(a.value) if a else None
    except Exception:
        return None


def _save_stack(stack):
    des = fusion_geo.design()
    des.attributes.add(APP_ID, "stack", json.dumps(stack))


# --------------------------------------------------------------------------- palette requests
def _lists():
    bodies = []
    for b in fusion_geo.list_bodies():
        try:
            bodies.append({"token": b.entityToken, "name": fusion_geo.body_path(b)})
        except Exception:
            pass
    sketches = sorted({sk.name for _c, sk in fusion_geo.list_sketches()})
    return {"bodies": bodies, "sketches": sketches}


def _init_payload():
    lib = _load_library()
    payload = {
        "builtinMaterials": materials.BUILTIN_MATERIALS,
        "builtinPresets": materials.BUILTIN_PRESETS,
        "library": lib,
        "stack": _load_stack(),
        "defaults": model.DEFAULT_SETTINGS,
        "units": fusion_geo.length_units(),
        "mmPerUnit": fusion_geo.mm_per_unit(),
        "preferredUp": fusion_geo.preferred_up(),
        "libraryFile": LIBRARY_FILE,
    }
    payload.update(_lists())
    return payload


def _pick_selected():
    sel = _ui().activeSelections
    for i in range(sel.count):
        ent = sel.item(i).entity
        body = adsk.fusion.BRepBody.cast(ent)
        if body is None:
            face = adsk.fusion.BRepFace.cast(ent)
            body = face.body if face else None
        if body:
            return {"token": body.entityToken, "name": fusion_geo.body_path(body)}
    return {"error": "Select a body (or one of its faces) in the canvas first."}


def _analyze(stack_data):
    """Frame, station metrics, planform outline and seam preview."""
    log = Log()
    target = fusion_geo.find_by_token((stack_data.get("target") or {}).get("token"))
    if target is None:
        return {"error": "Pick the foil body first."}
    target = adsk.fusion.BRepBody.cast(target)
    mats = materials.material_index(_load_library().get("materials"))
    stack = model.Stack(stack_data, mats)
    native, _comp, _inv = fusion_geo.target_context(target)
    frame = fusion_geo.detect_frame(target, stack_data.get("frame") or {})
    n = max(12, min(40, int(stack.settings["loftStations"])))
    raw = fusion_geo.sample_sections(native, frame, n, float(stack.settings["tipInset"]),
                                     model.span_limit_stations(stack, frame))
    sections = engine.build_sections(raw, 120, float(stack.settings["chordRefSmoothing"]))
    bounds = fusion_geo.load_bounds(stack.bound_sketches, frame, log)
    engine.log_tip_schedule(engine.tip_schedule(stack, sections, frame, bounds), frame, log)
    stations = [{"s": sec.s, "chord": sec.chord, "thickness": sec.thickness,
                 "le": sec.le_pt[0], "te": sec.te_pt[0]} for sec in sections]
    seams_out = []
    regions = {}
    try:
        pieces, _d, _e = engine.plan_pieces(stack, sections, frame, bounds, log, stack_data.get("seamPlan"))
        for pc in pieces:
            if pc.seam:
                line = []
                for i, sec in enumerate(sections):
                    fj = pc.seam["fj"][i] % sec.m
                    j = int(round(fj)) % sec.m
                    line.append((sec.s, sec.pts[j][0], 1 if j <= sec.le else -1))
                seams_out.append({"piece": pc.id, "label": pc.label, "msg": pc.seam["msg"],
                                  "ok": pc.seam["ok"], "line": line})
        for row in stack.rows:
            polys = engine.region_planform_polys(row, sections, frame, 0.0)
            if polys:
                regions[row.id] = polys
    except model.StackError as e:
        log(str(e))
    thin = []
    summary = engine.stack_summary(stack)
    for sec in sections:
        if sec.thickness < summary["top"] + summary["bottom"]:
            thin.append(sec.s)
    if thin:
        log("Stack (%.2f mm top + %.2f mm bottom) is thicker than the section at %d stations "
            "(|s| >= %.0f mm); inner layers end there and the outer layers fill the tip"
            % (summary["top"], summary["bottom"], len(thin), min(abs(s) for s in thin)))
    return {"frame": {k: frame[k] for k in ("spanAxis", "upAxis", "chordAxis", "sMin", "sMax", "hand",
                                            "preferredUp", "extents")},
            "stations": stations, "bounds": bounds, "regions": regions, "seams": seams_out,
            "summary": summary, "log": log.lines}


def _preview(stack_data, s):
    target = fusion_geo.find_by_token((stack_data.get("target") or {}).get("token"))
    if target is None:
        return {"error": "Pick the foil body first."}
    target = adsk.fusion.BRepBody.cast(target)
    mats = materials.material_index(_load_library().get("materials"))
    stack = model.Stack(stack_data, mats)
    native, _comp, _inv = fusion_geo.target_context(target)
    frame = fusion_geo.detect_frame(target, stack_data.get("frame") or {})
    s = max(frame["sMin"] + 0.5, min(frame["sMax"] - 0.5, float(s)))
    loop = fusion_geo.raw_section(native, frame, s)
    if not loop:
        return {"error": "No section at s=%.1f" % s}
    st = stack.settings
    m = int(st["loftPoints"])
    sec = model.Section(s, loop, m)
    # the whole-span sections are needed for the tip drop schedule and the
    # smoothed chord reference; they only change with the body or the frame
    key = (stack_data["target"]["token"], json.dumps(stack_data.get("frame") or {}, sort_keys=True),
           int(st["loftStations"]), float(st["tipInset"]), m)
    if _SPAN_CACHE.get("key") != key:
        raw = fusion_geo.sample_sections(native, frame, int(st["loftStations"]), float(st["tipInset"]))
        _SPAN_CACHE.update(key=key, sections=engine.build_sections(raw, m, 0.0))
    span = [x for x in _SPAN_CACHE["sections"] if abs(x.s - s) > 0.05] + [sec]
    span.sort(key=lambda x: x.s)
    model.smooth_chord_reference(span, float(st["chordRefSmoothing"]))
    bounds = fusion_geo.load_bounds(stack.bound_sketches, frame)
    engine.tip_schedule(stack, span, frame, bounds)
    out = engine.section_preview(stack, sec, frame, bounds)
    out.update(sMin=frame["sMin"], sMax=frame["sMax"])
    return out


def _choose_folder(title):
    dlg = _ui().createFolderDialog()
    dlg.title = title
    if dlg.showDialog() != adsk.core.DialogResults.DialogOK:
        return None
    return dlg.folder


def _file_dialog(save, title, filt, initial=""):
    dlg = _ui().createFileDialog()
    dlg.title = title
    dlg.filter = filt
    if initial:
        dlg.initialFilename = initial
    res = dlg.showSave() if save else dlg.showOpen()
    if res != adsk.core.DialogResults.DialogOK:
        return None
    return dlg.filename


def _export_stack(data):
    path = _file_dialog(True, "Export layup", "Layup JSON (*.json);;Layer CSV (*.csv)", "layup.json")
    if not path:
        return {"cancelled": True}
    # a saved layup is a recipe, not tied to this design's body or its seams
    data = {k: v for k, v in data.items() if k not in ("target", "seamPlan")}
    if path.lower().endswith(".csv"):
        from foil_layup import csvio
        with open(path, "w", newline="", encoding="utf8") as f:
            f.write(csvio.stack_to_csv(data))
    else:
        with open(path, "w", encoding="utf8") as f:
            json.dump(data, f, indent=1)
    return {"path": path}


def _import_stack():
    path = _file_dialog(False, "Import layup", "Layup JSON or CSV (*.json;*.csv)")
    if not path:
        return {"cancelled": True}
    with open(path, encoding="utf8") as f:
        text = f.read()
    if path.lower().endswith(".csv"):
        from foil_layup import csvio
        return {"path": path, "partial": csvio.csv_to_stack(text)}
    return {"path": path, "stack": json.loads(text)}


def _handle(action, data):
    if action == "init":
        return _init_payload()
    if action == "lists":
        return _lists()
    if action == "pickSelected":
        return _pick_selected()
    if action == "saveStack":
        _save_stack(data)
        return {"ok": True}
    if action == "saveLibrary":
        _save_library(data)
        return {"ok": True}
    if action == "analyze":
        return _analyze(data)
    if action == "preview":
        return _preview(data["stack"], data["s"])
    if action == "exportStack":
        return _export_stack(data)
    if action == "importStack":
        return _import_stack()
    if action == "openFolder":
        os.startfile(data["folder"])
        return {"ok": True}
    if action in ("generate", "flatten"):
        if action == "flatten":
            folder = data.get("folder") or _choose_folder("Folder for flat patterns (DXF/SVG/CSV)")
            if not folder:
                return {"cancelled": True}
            data["folder"] = folder
        _save_stack(data["stack"])
        _pending.clear()
        _pending.update({"action": action, "data": data})
        _ui().commandDefinitions.itemById(CMD_RUN).execute()
        return {"started": True}
    return {"error": "Unknown action %s" % action}


class PaletteHandler(adsk.core.HTMLEventHandler):
    def notify(self, args):
        try:
            data = json.loads(args.data) if args.data else {}
            out = _handle(args.action, data)
        except (model.StackError, fusion_geo.FusionError) as e:
            out = {"error": str(e)}
        except Exception:
            out = {"error": traceback.format_exc()}
            Log()(out["error"])
        args.returnData = json.dumps(out)


class PaletteClosed(adsk.core.UserInterfaceGeneralEventHandler):
    def notify(self, args):
        pass


# --------------------------------------------------------------------------- commands
class OpenCreated(adsk.core.CommandCreatedEventHandler):
    def notify(self, args):
        try:
            pal = _ui().palettes.itemById(PALETTE_ID)
            if pal is None:
                url = os.path.join(HERE, "palette", "index.html").replace("\\", "/")
                pal = _ui().palettes.add(PALETTE_ID, "Foil Layup", url, True, True, True, 1100, 800, True)
                pal.dockingState = adsk.core.PaletteDockingStates.PaletteDockStateFloating
                h = PaletteHandler()
                pal.incomingFromHTML.add(h)
                _handlers.append(h)
                c = PaletteClosed()
                pal.closed.add(c)
                _handlers.append(c)
            pal.isVisible = True
        except Exception:
            _ui().messageBox("Foil Layup failed to open:\n" + traceback.format_exc())


class RunCreated(adsk.core.CommandCreatedEventHandler):
    def notify(self, args):
        cmd = args.command
        cmd.isRepeatable = False
        h = RunExecute()
        cmd.execute.add(h)
        _handlers.append(h)


class RunExecute(adsk.core.CommandEventHandler):
    def notify(self, args):
        job = dict(_pending)
        _pending.clear()
        if not job:
            return
        log = Log()
        log("---- %s" % job["action"])
        prog = _ui().createProgressDialog()
        prog.isCancelButtonShown = False
        prog.show("Foil Layup", "Starting", 0, 100, 0)

        def progress(label, k, n):
            try:
                prog.message = label
                prog.progressValue = int(100 * k / max(n, 1))
                adsk.doEvents()
            except Exception:
                pass

        data = job["data"]
        lib = _load_library()
        try:
            if job["action"] == "generate":
                res = fusion_build.generate(data["stack"], lib.get("materials"), log, progress)
                stack = data["stack"]
                stack["seamPlan"] = res["seamPlan"]
                _save_stack(stack)
            else:
                res = fusion_flat.flatten(data["stack"], lib.get("materials"), data["folder"],
                                          bool(data.get("sketches", True)), log, progress)
            res["log"] = log.lines
            prog.hide()
            _send(job["action"] + "Done", res)
        except Exception as e:
            prog.hide()
            msg = str(e) if isinstance(e, (model.StackError, fusion_geo.FusionError)) else traceback.format_exc()
            log(msg)
            args.executeFailed = True
            args.executeFailedMessage = str(e)[:500]
            _send(job["action"] + "Done", {"error": msg, "log": log.lines})


def run(context):
    try:
        ui = _ui()
        for cid, name, tip, res, created in (
            (CMD_OPEN, "Foil Layup", "Design composite layers for a foil, split it and make flat patterns",
             "resources/open", OpenCreated),
            (CMD_RUN, "Foil Layup run", "Internal: runs Generate / Flatten", "", RunCreated),
        ):
            cd = ui.commandDefinitions.itemById(cid)
            if cd:
                cd.deleteMe()
            resdir = os.path.join(HERE, res) if res and os.path.isdir(os.path.join(HERE, res)) else ""
            cd = ui.commandDefinitions.addButtonDefinition(cid, name, tip, resdir)
            h = created()
            cd.commandCreated.add(h)
            _handlers.append(h)
        ws = ui.workspaces.itemById("FusionSolidEnvironment")
        panel = ws.toolbarPanels.itemById(PANEL_ID)
        if panel and not panel.controls.itemById(CMD_OPEN):
            ctl = panel.controls.addCommand(ui.commandDefinitions.itemById(CMD_OPEN))
            ctl.isPromotedByDefault = True
            ctl.isPromoted = True
    except Exception:
        _ui().messageBox("Foil Layup failed to start:\n" + traceback.format_exc())


def stop(context):
    try:
        ui = _ui()
        pal = ui.palettes.itemById(PALETTE_ID)
        if pal:
            pal.deleteMe()
        ws = ui.workspaces.itemById("FusionSolidEnvironment")
        panel = ws.toolbarPanels.itemById(PANEL_ID)
        if panel:
            ctl = panel.controls.itemById(CMD_OPEN)
            if ctl:
                ctl.deleteMe()
        for cid in (CMD_OPEN, CMD_RUN):
            cd = ui.commandDefinitions.itemById(cid)
            if cd:
                cd.deleteMe()
    except Exception:
        pass
    _handlers.clear()
