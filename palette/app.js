/* Foil Layup palette. Talks to FoilLayup.py through adsk.fusionSendData. */
"use strict";

const S = {
  init: null,
  stack: null,
  mats: {},        // id -> material (built-in + user)
  library: { materials: [], presets: [] },
  units: "mm",
  k: 1,            // mm per display unit
  analysis: null,
  preview: null,
  open: {},        // expanded detail rows by id
};

const $ = (sel, el = document) => el.querySelector(sel);
const $$ = (sel, el = document) => [...el.querySelectorAll(sel)];
const esc = (s) => String(s ?? "").replace(/[&<>"]/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;" }[c]));
const fmt = (v, d = 2) => (v === null || v === undefined || v === "" || isNaN(v) ? "" : (+v).toFixed(d));
const toU = (mm) => (mm === null || mm === undefined || mm === "" ? "" : +(mm / S.k).toPrecision(6));
const fromU = (v) => (v === "" || v === null || v === undefined || isNaN(+v) ? null : +v * S.k);

// ------------------------------------------------------------------ bridge
function bridgeReady() {
  return typeof window.adsk !== "undefined" && window.adsk.fusionSendData;
}
async function call(action, data = {}) {
  let raw;
  if (bridgeReady()) {
    raw = await window.adsk.fusionSendData(action, JSON.stringify(data));
  } else if (window.FoilLayupMock) {
    raw = await window.FoilLayupMock(action, data);
  } else {
    throw new Error("Fusion bridge not available");
  }
  const out = raw ? JSON.parse(raw) : {};
  if (out && out.error) throw new Error(out.error);
  return out;
}
window.fusionJavaScriptHandler = {
  handle(action, data) {
    try {
      const payload = JSON.parse(data || "{}");
      if (action === "generateDone") onGenerateDone(payload);
      else if (action === "flattenDone") onFlattenDone(payload);
    } catch (e) {
      logLine("handler error: " + e);
    }
    return "OK";
  },
};

function logLine(msg, cls) {
  const el = $("#log");
  const line = document.createElement("div");
  if (cls) line.className = cls;
  line.textContent = msg;
  el.appendChild(line);
  el.scrollTop = el.scrollHeight;
}
function fail(e) {
  logLine(String(e.message || e), "err");
  $("#busy").textContent = "";
  alert(String(e.message || e).slice(0, 800));
}

// ------------------------------------------------------------------ model helpers
function defaultStack() {
  return {
    version: 1,
    target: null,
    frame: { spanAxis: "auto", upAxis: "auto", flipUp: false, symmetric: true, spanCentre: 0 },
    settings: { ...(S.init?.defaults || {}) },
    external: [],
    layers: [],
    keepouts: [],
    core: { material: "pvc130", name: "Core" },
    seamPlan: {},
  };
}
function rebuildMaterials() {
  S.mats = {};
  for (const m of S.init.builtinMaterials) S.mats[m.id] = { ...m };
  for (const m of S.library.materials || []) S.mats[m.id] = { ...m, builtin: false };
}
function matList(filter) {
  return Object.values(S.mats).filter(filter || (() => true));
}
function layerMaterials() {
  return matList((m) => m.category !== "core");
}
/* Cured ply thickness in the mould: fabrics at the in-mould fibre fraction
   (Setup), same rule as the Python side (materials.in_mould). */
function plyT(r) {
  if (+r.thickness) return +r.thickness;
  const m = S.mats[r.material];
  if (!m) return 0;
  const vf = +((S.stack && S.stack.settings && S.stack.settings.mouldVf) || 0);
  const rho = FIBRES[m.fibre];
  if (vf > 0 && m.category === "fabric" && rho && m.gsm) {
    const v = Math.min(0.68, vf + (m.form === "UD" ? 0.03 : 0));
    return m.gsm / (rho * v * 1000);
  }
  return +m.thickness;
}
function rowT(r) {
  return plyT(r) * Math.max(1, +r.count || 1);
}
function newId(prefix) {
  const used = new Set([...S.stack.layers, ...S.stack.external].map((r) => r.id));
  let n = 1;
  while (used.has(prefix + n)) n++;
  return prefix + n;
}
function newRow(group) {
  const ext = group === "external";
  return {
    id: newId(ext ? "E" : "L"),
    name: "",
    material: ext ? "pp85" : "c200t",
    count: 1,
    angles: "0",
    side: "wrap",
    seam: "auto",
  };
}
function saveStack() {
  clearTimeout(saveStack.t);
  saveStack.t = setTimeout(() => call("saveStack", S.stack).catch(() => {}), 400);
  renderTotals();
}

// ------------------------------------------------------------------ tabs
function initTabs() {
  $$("#tabs button").forEach((b) =>
    b.addEventListener("click", () => {
      $$("#tabs button").forEach((x) => x.classList.toggle("on", x === b));
      $$(".tab").forEach((t) => t.classList.toggle("on", t.id === "tab-" + b.dataset.tab));
      if (b.dataset.tab === "stack" && !S.preview && S.stack.target) loadPreview();
    })
  );
}

// ------------------------------------------------------------------ setup tab
function renderSetup() {
  const st = S.stack;
  const sel = $("#target");
  sel.innerHTML = '<option value="">— choose body —</option>' +
    S.init.bodies.map((b) => `<option value="${esc(b.token)}">${esc(b.name)}</option>`).join("");
  if (st.target) {
    if (!S.init.bodies.some((b) => b.token === st.target.token)) {
      sel.insertAdjacentHTML("beforeend", `<option value="${esc(st.target.token)}">${esc(st.target.name)} (saved)</option>`);
    }
    sel.value = st.target.token;
  }
  $("#spanAxis").value = st.frame.spanAxis || "auto";
  $("#upAxis").value = st.frame.upAxis || "auto";
  $("#flipUp").checked = !!st.frame.flipUp;
  $("#symmetric").checked = st.frame.symmetric !== false;
  $("#spanCentre").value = toU(st.frame.spanCentre || 0);
  $("#upHint").textContent = `Fusion up preference: ${S.init.preferredUp}-up. "auto" uses it when it is the thin direction of the foil.`;
  $$("[data-set]").forEach((el) => {
    const key = el.dataset.set;
    let v = st.settings[key];
    if (el.type === "checkbox") el.checked = !!v;
    else el.value = el.hasAttribute("data-len") ? toU(v) : v;
  });
  $$(".u").forEach((el) => (el.textContent = `(${S.units})`));
}
function bindSetup() {
  $("#target").addEventListener("change", (e) => {
    const b = S.init.bodies.find((x) => x.token === e.target.value);
    S.stack.target = b ? { token: b.token, name: b.name } : null;
    S.analysis = null;
    S.preview = null;
    saveStack();
  });
  $("#useSel").addEventListener("click", async () => {
    try {
      const r = await call("pickSelected");
      S.stack.target = { token: r.token, name: r.name };
      if (!S.init.bodies.some((b) => b.token === r.token)) S.init.bodies.push(r);
      renderSetup();
      saveStack();
    } catch (e) { fail(e); }
  });
  $("#refreshLists").addEventListener("click", refreshLists);
  for (const id of ["spanAxis", "upAxis"]) $("#" + id).addEventListener("change", (e) => { S.stack.frame[id] = e.target.value; saveStack(); });
  $("#flipUp").addEventListener("change", (e) => { S.stack.frame.flipUp = e.target.checked; saveStack(); });
  $("#symmetric").addEventListener("change", (e) => { S.stack.frame.symmetric = e.target.checked; saveStack(); });
  $("#spanCentre").addEventListener("change", (e) => { S.stack.frame.spanCentre = fromU(e.target.value) || 0; saveStack(); });
  $$("[data-set]").forEach((el) =>
    el.addEventListener("change", () => {
      const key = el.dataset.set;
      let v;
      if (el.type === "checkbox") v = el.checked;
      else if (el.tagName === "SELECT") v = el.value;
      else v = el.hasAttribute("data-len") ? fromU(el.value) : +el.value;
      S.stack.settings[key] = v;
      saveStack();
      if (key === "mouldVf") { renderStack(); schedulePreview(); }
    })
  );
  $("#analyze").addEventListener("click", analyze);
}
async function refreshLists() {
  try {
    const r = await call("lists");
    S.init.bodies = r.bodies;
    S.init.sketches = r.sketches;
    renderSetup();
    renderStack();
    renderKeepouts();
  } catch (e) { fail(e); }
}
async function analyze() {
  if (!S.stack.target) return fail(new Error("Pick the foil body first."));
  $("#analysisInfo").textContent = "Analyzing...";
  try {
    const a = await call("analyze", S.stack);
    S.analysis = a;
    const f = a.frame;
    const root = a.stations.reduce((m, s) => (Math.abs(s.s) < Math.abs(m.s) ? s : m), a.stations[0]);
    const tmax = Math.max(...a.stations.map((s) => s.thickness));
    const lines = [
      `Span ${f.spanAxis}, up ${f.upAxis}, chord ${f.chordAxis}  (span ${fmt((f.sMax - f.sMin) / S.k, 1)} ${S.units})`,
      `Centre chord ${fmt(root.chord / S.k, 1)} ${S.units}, max thickness ${fmt(tmax, 2)} mm, ${a.stations.length} stations`,
      `Stack top ${fmt(a.summary.top)} mm, bottom ${fmt(a.summary.bottom)} mm, external ${fmt(a.summary.external)} mm`,
      ...a.seams.map((s) => `Seam ${s.piece}: ${s.msg}`),
      ...a.log,
    ];
    $("#analysisInfo").textContent = lines.join("\n");
    drawPlanform();
  } catch (e) {
    $("#analysisInfo").textContent = "";
    fail(e);
  }
}

// ------------------------------------------------------------------ planform plot
function drawPlanform() {
  const a = S.analysis;
  const el = $("#planform");
  if (!a) { el.innerHTML = ""; return; }
  const st = a.stations;
  const W = 1000;
  const sMin = Math.min(...st.map((s) => s.s)), sMax = Math.max(...st.map((s) => s.s));
  const cMin = Math.min(...st.map((s) => s.le)), cMax = Math.max(...st.map((s) => s.te));
  const pad = 20;
  const sc = (W - 2 * pad) / (sMax - sMin);
  const H = Math.max(120, (cMax - cMin) * sc + 2 * pad);
  const X = (s) => pad + (s - sMin) * sc;
  const Y = (c) => pad + (c - cMin) * sc;
  const path = (pts) => pts.map((p, i) => (i ? "L" : "M") + X(p[0]).toFixed(1) + " " + Y(p[1]).toFixed(1)).join(" ");
  let svg = `<svg viewBox="0 0 ${W} ${H.toFixed(0)}" xmlns="http://www.w3.org/2000/svg">`;
  const outline = st.map((s) => [s.s, s.le]).concat(st.slice().reverse().map((s) => [s.s, s.te]));
  svg += `<path d="${path(outline)} Z" fill="#8a96a3" fill-opacity="0.18" stroke="#56606b" stroke-width="1"/>`;
  for (const [rid, polys] of Object.entries(a.regions || {})) {
    for (const p of polys) svg += `<path d="${path(p)} Z" fill="#0b6bcb" fill-opacity="0.08" stroke="#0b6bcb" stroke-dasharray="4 3" stroke-width="0.8"><title>${esc(rid)} limits</title></path>`;
  }
  for (const [name, polys] of Object.entries(a.bounds || {})) {
    for (const p of polys) svg += `<path d="${path(p)} Z" fill="#b25b00" fill-opacity="0.12" stroke="#b25b00" stroke-width="1"><title>${esc(name)}</title></path>`;
  }
  const colours = ["#c0392b", "#16a085", "#8e44ad", "#d35400", "#2c3e50", "#27ae60", "#2980b9"];
  a.seams.forEach((sm, i) => {
    const pts = sm.line.map((p) => [p[0], p[1]]);
    const dash = sm.line[Math.floor(sm.line.length / 2)][2] < 0 ? ' stroke-dasharray="6 3"' : "";
    svg += `<path d="${path(pts)}" fill="none" stroke="${colours[i % colours.length]}" stroke-width="1.4"${dash}><title>${esc(sm.piece + " " + sm.msg)}</title></path>`;
  });
  const sNow = S.previewS ?? 0;
  svg += `<line x1="${X(sNow)}" x2="${X(sNow)}" y1="0" y2="${H}" stroke="#0b6bcb" stroke-width="1"/>`;
  svg += `<text x="${pad}" y="${H - 4}" font-size="11" fill="#6a7480">LE at top, TE at bottom. Seams: solid = top side, dashed = bottom side. Blue dashed = span/chord limits, orange = bound sketches.</text>`;
  svg += "</svg>";
  el.innerHTML = svg;
  el.querySelector("svg").addEventListener("click", (ev) => {
    const r = ev.currentTarget.getBoundingClientRect();
    const x = ((ev.clientX - r.left) / r.width) * W;
    S.previewS = sMin + (x - pad) / sc;
    $("#pvS").value = fmt(toU(S.previewS), 1);
    drawPlanform();
    loadPreview();
  });
}

// ------------------------------------------------------------------ materials tab
const FIBRES = { carbon: 1.78, glass: 2.55, basalt: 2.67, aramid: 1.44, flax: 1.45 };
const FORMS = ["woven", "spread tow", "UD", "biax", "mat", "peel ply", "release film", "breather", "flow media", "foam", "wood", "solid"];
const CATS = ["fabric", "mat", "process", "core"];
function calcThickness(m) {
  const rho = FIBRES[m.fibre];
  if (!rho || !m.gsm || !m.vf) return m.thickness;
  return +(m.gsm / (rho * m.vf * 1000)).toFixed(3);
}
function calcDensity(m) {
  const rho = FIBRES[m.fibre];
  if (!rho || !m.vf) return m.density;
  return Math.round((m.vf * rho + (1 - m.vf) * 1.15) * 1000);
}
function renderMaterials() {
  const q = ($("#matFilter").value || "").toLowerCase();
  const rows = matList((m) => !q || (m.name + " " + m.fibre + " " + m.form + " " + m.id).toLowerCase().includes(q));
  rows.sort((a, b) => (a.builtin === b.builtin ? 0 : a.builtin ? 1 : -1) || CATS.indexOf(a.category) - CATS.indexOf(b.category));
  const head = "<tr><th></th><th>Id</th><th>Name</th><th>Category</th><th>Fibre</th><th>Form</th><th>g/m²</th><th>Vf</th><th title='Library value. The layup uses the in-mould Vf from Setup unless that is 0.'>Ply t library (mm)</th><th>Density (kg/m³)</th><th></th></tr>";
  const body = rows.map((m) => {
    if (m.builtin) {
      return `<tr><td><span class="swatch" style="background:${esc(m.color)}"></span></td><td>${esc(m.id)}</td><td>${esc(m.name)}</td><td>${esc(m.category)}</td>
        <td>${esc(m.fibre)}</td><td>${esc(m.form)}</td><td class="num">${m.gsm || ""}</td><td class="num">${m.vf || ""}</td>
        <td class="num">${fmt(m.thickness, 3)}</td><td class="num">${m.density}</td>
        <td class="act"><button class="icon" data-dup="${esc(m.id)}" title="Duplicate as custom">⧉</button></td></tr>`;
    }
    const opt = (list, v) => list.map((x) => `<option ${x === v ? "selected" : ""}>${x}</option>`).join("");
    return `<tr data-mid="${esc(m.id)}"><td><input type="color" data-f="color" value="${esc(m.color)}"></td><td>${esc(m.id)}</td>
      <td><input data-f="name" value="${esc(m.name)}"></td>
      <td><select data-f="category">${opt(CATS, m.category)}</select></td>
      <td><select data-f="fibre"><option></option>${opt(Object.keys(FIBRES), m.fibre)}</select></td>
      <td><select data-f="form">${opt(FORMS, m.form)}</select></td>
      <td><input type="number" data-f="gsm" value="${m.gsm || ""}"></td>
      <td><input type="number" step="0.01" data-f="vf" value="${m.vf || ""}"></td>
      <td><input type="number" step="0.001" data-f="thickness" value="${m.thickness}"></td>
      <td><input type="number" data-f="density" value="${m.density}"></td>
      <td class="act"><button class="icon" data-calc="${esc(m.id)}" title="Thickness and density from g/m², fibre and Vf">ƒ</button>
        <button class="icon" data-dup="${esc(m.id)}" title="Duplicate">⧉</button>
        <button class="icon" data-del="${esc(m.id)}" title="Delete">✕</button></td></tr>`;
  }).join("");
  $("#matTable").innerHTML = head + body;
  $("#libPath").textContent = "Library file: " + (S.init.libraryFile || "");
}
function userMat(id) {
  return S.library.materials.find((m) => m.id === id);
}
function bindMaterials() {
  $("#matFilter").addEventListener("input", renderMaterials);
  $("#matNew").addEventListener("click", () => {
    let n = 1;
    while (S.mats["u" + n]) n++;
    S.library.materials.push({ id: "u" + n, name: "New material", category: "fabric", fibre: "carbon", form: "woven",
      weave: "", gsm: 200, vf: 0.5, thickness: 0.225, density: 1465, color: "#444444" });
    rebuildMaterials();
    renderMaterials();
  });
  $("#libSave").addEventListener("click", async () => {
    try { await call("saveLibrary", S.library); logLine("Library saved"); } catch (e) { fail(e); }
  });
  $("#matTable").addEventListener("click", (ev) => {
    const t = ev.target;
    if (t.dataset.dup) {
      const src = S.mats[t.dataset.dup];
      let n = 1;
      while (S.mats[src.id + "_" + n]) n++;
      S.library.materials.push({ ...src, id: src.id + "_" + n, name: src.name + " (copy)", builtin: false });
    } else if (t.dataset.del) {
      S.library.materials = S.library.materials.filter((m) => m.id !== t.dataset.del);
    } else if (t.dataset.calc) {
      const m = userMat(t.dataset.calc);
      m.thickness = calcThickness(m);
      m.density = calcDensity(m);
    } else return;
    rebuildMaterials();
    renderMaterials();
    renderStack();
  });
  $("#matTable").addEventListener("change", (ev) => {
    const tr = ev.target.closest("tr[data-mid]");
    if (!tr) return;
    const m = userMat(tr.dataset.mid);
    const f = ev.target.dataset.f;
    m[f] = ev.target.type === "number" ? +ev.target.value : ev.target.value;
    rebuildMaterials();
    renderStack();
  });
}

// ------------------------------------------------------------------ stack tab
function matOptions(sel, filter) {
  const groups = {};
  for (const m of matList(filter)) (groups[m.category] = groups[m.category] || []).push(m);
  return Object.entries(groups).map(([cat, ms]) =>
    `<optgroup label="${esc(cat)}">` + ms.map((m) => `<option value="${esc(m.id)}" ${m.id === sel ? "selected" : ""}>${esc(m.name)}</option>`).join("") + "</optgroup>"
  ).join("");
}
function sketchOptions(sel) {
  const list = S.init.sketches || [];
  const extra = sel && !list.includes(sel) ? [sel] : [];
  return '<option value="">— none —</option>' + [...list, ...extra].map((n) => `<option ${n === sel ? "selected" : ""}>${esc(n)}</option>`).join("");
}
function rowHtml(r, i, group) {
  const t = rowT(r);
  const m = S.mats[r.material];
  const ext = group === "external";
  const hasRegion = r.span || r.chord || (r.bound && r.bound.sketch) || (r.splits && r.splits.length);
  const sideSel = ext ? "<td>wrap</td>" :
    `<td><select data-f="side">${["wrap", "top", "bottom"].map((s) => `<option ${s === (r.side || "wrap") ? "selected" : ""}>${s}</option>`).join("")}</select></td>`;
  let html = `<tr data-i="${i}" data-g="${group}">
    <td class="num">${i + 1}</td>
    <td><span class="swatch" style="background:${esc(m ? m.color : "#ccc")}"></span></td>
    <td><input class="name" data-f="name" value="${esc(r.name)}" placeholder="${esc(m ? m.name : "")}"></td>
    <td><select class="mat" data-f="material">${matOptions(r.material, (x) => x.category !== "core")}</select></td>
    <td><input type="number" min="1" data-f="count" value="${r.count || 1}" style="width:48px"></td>
    <td><input data-f="angles" value="${esc(r.angles ?? "0")}" style="width:70px" title="Ply angles from span, comma separated; cycles over the plies"></td>
    <td class="num" title="Row thickness">${fmt(t, 3)}</td>
    ${sideSel}
    <td><input data-f="seam" list="seamOpts" value="${esc(r.seam || "auto")}" style="width:84px" title="auto, te, le, top:40, bottom:60 (percent chord)"></td>
    <td class="act">
      ${ext ? "" : `<button class="icon" data-act="more" title="Region, bounds, splits">${S.open[r.id] ? "▾" : "▸"}${hasRegion ? "•" : ""}</button>`}
      <button class="icon" data-act="up" title="Move out">↑</button><button class="icon" data-act="down" title="Move in">↓</button>
      <button class="icon" data-act="dup" title="Duplicate">⧉</button><button class="icon" data-act="del" title="Delete">✕</button></td></tr>`;
  if (!ext && S.open[r.id]) {
    const span = r.span || [null, null], chord = r.chord || [null, null];
    const b = r.bound || {};
    html += `<tr class="detail" data-i="${i}" data-g="${group}"><td></td><td colspan="9"><div class="detail-grid">
      <label>Span limits ${S.stack.frame.symmetric !== false ? "|from centre|" : "from root"} (${S.units})
        <span class="pair"><input type="number" step="any" data-f="span0" value="${toU(span[0])}" placeholder="start">
        <input type="number" step="any" data-f="span1" value="${toU(span[1])}" placeholder="end"></span></label>
      <label>Chord limits (% chord)
        <span class="pair"><input type="number" step="any" data-f="chord0" value="${chord[0] == null ? "" : +(chord[0] * 100).toFixed(2)}" placeholder="0">
        <input type="number" step="any" data-f="chord1" value="${chord[1] == null ? "" : +(chord[1] * 100).toFixed(2)}" placeholder="100"></span></label>
      <label>Bound sketch (planform outline)
        <span class="pair"><select data-f="boundSketch">${sketchOptions(b.sketch)}</select>
        <select data-f="boundMode"><option value="inside" ${b.mode !== "outside" ? "selected" : ""}>inside</option><option value="outside" ${b.mode === "outside" ? "selected" : ""}>outside</option></select></span></label>
      <label>Outside the limits use
        <select data-f="outsideMaterial"><option value="">nothing (layer ends; inner layers move out)</option>
          <option value="core" ${r.outsideMaterial === "core" ? "selected" : ""}>core material</option>
          ${matOptions(r.outsideMaterial, () => true)}</select></label>
      <label>Ply thickness override (mm)<input type="number" step="any" data-f="thickness" value="${r.thickness || ""}" placeholder="${m ? fmt(plyT({ material: r.material }), 3) : ""}"></label>
      <label title="Each ply of this layer stops where the space left inside the layers above it is thinner than this (half-thickness, mm). Empty = Setup's 'Inner ply end' x ply thickness.">Stop where space &lt; (mm, each ply)<input type="number" step="any" min="0" data-f="endGap" value="${r.endGap ?? ""}" placeholder="${fmt((S.stack.settings.plyEndRatio ?? 1) * plyT(r), 3)}"></label>
      <label>Pattern splits at span (${S.units}; ";" separated)<input data-f="splits" value="${(r.splits || []).map((v) => toU(v)).join("; ")}" placeholder="e.g. 0; 250"></label>
      <label>Id<input data-f="id" value="${esc(r.id)}"></label>
    </div><p class="hint">Top/bottom layers sit on one side only and end at the LE/TE. Limits make a layer end early (ply drops ramp at ${S.stack.settings.dropRatio}:1). With "nothing", inner layers move outward to fill the gap; with a material or the core, the slot is filled so the inner layers stay put (e.g. spar caps with foam beside them).</p></td></tr>`;
  }
  return html;
}
function renderStack() {
  if (!S.stack) return;
  const head = `<tr><th>#</th><th></th><th>Name</th><th>Material</th><th>Plies</th><th>Angles°</th><th>t mm</th><th>Side</th><th>Seam</th><th></th></tr>`;
  $("#extTable").innerHTML = head + S.stack.external.map((r, i) => rowHtml(r, i, "external")).join("");
  $("#layerTable").innerHTML = head + S.stack.layers.map((r, i) => rowHtml(r, i, "layers")).join("");
  $("#coreMat").innerHTML = matOptions(S.stack.core.material, (m) => m.category === "core");
  $("#coreName").value = S.stack.core.name || "Core";
  const ps = $("#presetSel");
  const all = [...S.init.builtinPresets.map((p) => ({ ...p, builtin: true })), ...(S.library.presets || [])];
  ps.innerHTML = all.map((p, i) => `<option value="${i}">${esc(p.name)}${p.builtin ? "" : " (mine)"}</option>`).join("");
  renderTotals();
}
function renderTotals() {
  if (!S.stack) return;
  let top = 0, bot = 0, gTop = 0, gBot = 0;
  for (const r of S.stack.layers) {
    const t = rowT(r);
    const m = S.mats[r.material] || {};
    const g = (m.density || 0) * t / 1000; // kg/m2
    if (r.side !== "bottom") { top += t; gTop += g; }
    if (r.side !== "top") { bot += t; gBot += g; }
  }
  $("#totals").textContent = `Top ${fmt(top)} mm (${fmt(gTop * 1000, 0)} g/m²) · Bottom ${fmt(bot)} mm (${fmt(gBot * 1000, 0)} g/m²) · ${S.stack.layers.length} layers`;
}
function bindStack() {
  for (const id of ["extTable", "layerTable"]) {
    const tbl = $("#" + id);
    tbl.addEventListener("change", (ev) => {
      const tr = ev.target.closest("tr[data-i]");
      if (!tr) return;
      const list = S.stack[tr.dataset.g];
      const r = list[+tr.dataset.i];
      const f = ev.target.dataset.f;
      const v = ev.target.value;
      switch (f) {
        case "count": r.count = Math.max(1, Math.round(+v || 1)); break;
        case "span0": case "span1": {
          r.span = r.span || [null, null];
          r.span[f === "span0" ? 0 : 1] = fromU(v);
          if (r.span[0] == null && r.span[1] == null) delete r.span;
          break;
        }
        case "chord0": case "chord1": {
          r.chord = r.chord || [null, null];
          r.chord[f === "chord0" ? 0 : 1] = v === "" ? null : +v / 100;
          if (r.chord[0] == null && r.chord[1] == null) delete r.chord;
          break;
        }
        case "boundSketch": r.bound = { ...(r.bound || { mode: "inside" }), sketch: v }; if (!v) delete r.bound; break;
        case "boundMode": r.bound = { ...(r.bound || { sketch: "" }), mode: v }; break;
        case "outsideMaterial": if (v) r.outsideMaterial = v; else delete r.outsideMaterial; break;
        case "thickness": if (v === "") delete r.thickness; else r.thickness = +v; break;
        case "endGap": if (v === "") delete r.endGap; else r.endGap = Math.max(0, +v); break;
        case "splits": r.splits = v.split(/[;,]/).map((x) => x.trim()).filter((x) => x !== "").map((x) => fromU(x)); if (!r.splits.length) delete r.splits; break;
        case "id": {
          const nv = v.trim();
          if (nv && ![...S.stack.layers, ...S.stack.external].some((x) => x !== r && x.id === nv)) r.id = nv;
          break;
        }
        default: r[f] = v;
      }
      renderStack();
      saveStack();
      schedulePreview();
    });
    tbl.addEventListener("click", (ev) => {
      const btn = ev.target.closest("button[data-act]");
      if (!btn) return;
      const tr = btn.closest("tr[data-i]");
      const list = S.stack[tr.dataset.g];
      const i = +tr.dataset.i;
      const act = btn.dataset.act;
      if (act === "more") S.open[list[i].id] = !S.open[list[i].id];
      if (act === "up" && i > 0) [list[i - 1], list[i]] = [list[i], list[i - 1]];
      if (act === "down" && i < list.length - 1) [list[i + 1], list[i]] = [list[i], list[i + 1]];
      if (act === "dup") { const c = JSON.parse(JSON.stringify(list[i])); c.id = newId(tr.dataset.g === "external" ? "E" : "L"); list.splice(i + 1, 0, c); }
      if (act === "del") list.splice(i, 1);
      renderStack();
      if (act !== "more") { saveStack(); schedulePreview(); }
    });
  }
  $$("button[data-add]").forEach((b) => b.addEventListener("click", () => {
    S.stack[b.dataset.add].push(newRow(b.dataset.add));
    renderStack();
    saveStack();
    schedulePreview();
  }));
  $("#coreMat").addEventListener("change", (e) => { S.stack.core.material = e.target.value; saveStack(); schedulePreview(); });
  $("#coreName").addEventListener("change", (e) => { S.stack.core.name = e.target.value; saveStack(); });
  const presetAt = () => {
    const all = [...S.init.builtinPresets, ...(S.library.presets || [])];
    return all[+$("#presetSel").value];
  };
  const fresh = (rows, prefix) => rows.map((r) => ({ ...JSON.parse(JSON.stringify(r)), id: undefined })).map((r) => { r.id = newId(prefix); S.stack[prefix === "E" ? "external" : "layers"].push(r); return r; });
  $("#presetApply").addEventListener("click", () => {
    const p = presetAt();
    if (!p || !confirm(`Replace the current stack with "${p.name}"?`)) return;
    S.stack.layers = [];
    S.stack.external = [];
    fresh(p.layers || [], "L");
    fresh(p.external || [], "E");
    S.stack.seamPlan = {};
    renderStack(); saveStack(); schedulePreview();
  });
  $("#presetAppend").addEventListener("click", () => {
    const p = presetAt();
    if (!p) return;
    fresh(p.layers || [], "L");
    fresh(p.external || [], "E");
    renderStack(); saveStack(); schedulePreview();
  });
  $("#presetSave").addEventListener("click", async () => {
    const name = $("#presetName").value.trim();
    if (!name) return fail(new Error("Give the preset a name."));
    const strip = (rows) => rows.map((r) => { const c = JSON.parse(JSON.stringify(r)); delete c.id; return c; });
    S.library.presets = (S.library.presets || []).filter((p) => p.name !== name);
    S.library.presets.push({ id: "user-" + Date.now(), name, layers: strip(S.stack.layers), external: strip(S.stack.external) });
    try { await call("saveLibrary", S.library); renderStack(); logLine(`Preset "${name}" saved`); } catch (e) { fail(e); }
  });
  $("#exportStack").addEventListener("click", async () => {
    try { const r = await call("exportStack", S.stack); if (r.path) logLine("Exported " + r.path); } catch (e) { fail(e); }
  });
  $("#importStack").addEventListener("click", async () => {
    try {
      const r = await call("importStack");
      if (r.cancelled) return;
      if (r.stack) {
        // keep this design's body selection; ignore any body or seams in the file
        const keepTarget = S.stack.target;
        S.stack = { ...defaultStack(), ...r.stack, target: keepTarget, seamPlan: {} };
        S.stack.settings = { ...S.init.defaults, ...(r.stack.settings || {}) };
        S.stack.frame = { ...defaultStack().frame, ...(r.stack.frame || {}) };
      } else if (r.partial) {
        S.stack.layers = []; S.stack.external = [];
        for (const g of ["layers", "external"]) for (const row of r.partial[g]) { row.id = row.id || newId(g === "external" ? "E" : "L"); S.stack[g].push(row); }
      }
      const missing = [...S.stack.layers, ...S.stack.external].filter((x) => !S.mats[x.material]).map((x) => x.material);
      if (missing.length) logLine("Unknown materials in import: " + [...new Set(missing)].join(", "), "err");
      renderAll(); saveStack(); logLine("Imported " + r.path);
    } catch (e) { fail(e); }
  });
  $("#pvGo").addEventListener("click", () => { S.previewS = fromU($("#pvS").value) || 0; loadPreview(); });
  $("#pvS").addEventListener("change", () => { S.previewS = fromU($("#pvS").value) || 0; syncSlider(); loadPreview(); });
  $("#pvSlider").addEventListener("input", (ev) => {
    S.previewS = +ev.target.value;
    $("#pvS").value = fmt(toU(S.previewS), 1);
    clearTimeout(loadPreview.t);
    loadPreview.t = setTimeout(loadPreview, 250); // follow the drag without flooding Fusion
  });
  $("#pvExag").addEventListener("input", drawPreview);
  $("#pvZoom").addEventListener("change", drawPreview);
}

// ------------------------------------------------------------------ section preview
function schedulePreview() {
  clearTimeout(schedulePreview.t);
  if (!S.stack.target) return;
  schedulePreview.t = setTimeout(loadPreview, 700);
}
async function loadPreview() {
  if (!S.stack.target) return;
  try {
    S.preview = await call("preview", { stack: S.stack, s: S.previewS || 0 });
    if (S.preview.sMin !== undefined) S.span = [S.preview.sMin, S.preview.sMax];
    syncSlider();
    drawPreview();
  } catch (e) {
    $("#preview").innerHTML = `<p class="err">${esc(e.message)}</p>`;
  }
}
function syncSlider() {
  const sl = $("#pvSlider");
  if (!sl) return;
  if (S.span) {
    sl.min = (S.span[0] + 0.5).toFixed(1);
    sl.max = (S.span[1] - 0.5).toFixed(1);
  }
  sl.value = S.previewS || 0;
}
function drawPreview() {
  const p = S.preview;
  const el = $("#preview");
  if (!p) return;
  const ex = +$("#pvExag").value;
  const zoom = $("#pvZoom").value;
  const all = p.outer.concat(...p.ext.filter((e) => e.after).map((e) => e.after));
  let cMin = Math.min(...all.map((q) => q[0])), cMax = Math.max(...all.map((q) => q[0]));
  const uMid = (Math.min(...all.map((q) => q[1])) + Math.max(...all.map((q) => q[1]))) / 2;
  const chord = cMax - cMin;
  if (zoom === "le") cMax = cMin + chord * 0.15;
  if (zoom === "te") cMin = cMax - chord * 0.15;
  const vis = all.filter((q) => q[0] >= cMin && q[0] <= cMax);
  const uMin = Math.min(...vis.map((q) => q[1])), uMax = Math.max(...vis.map((q) => q[1]));
  const W = 900, pad = 10;
  const sc = (W - 2 * pad) / (cMax - cMin);
  const H = Math.max(120, Math.min(700, (uMax - uMin) * sc * ex + 2 * pad));
  const X = (c) => pad + (c - cMin) * sc;
  const Y = (u) => H / 2 - (u - (uMin + uMax) / 2) * sc * ex;
  const d = (pts) => pts.map((q, i) => (i ? "L" : "M") + X(q[0]).toFixed(2) + " " + Y(q[1]).toFixed(2)).join(" ") + " Z";
  let svg = `<svg viewBox="0 0 ${W} ${H.toFixed(0)}" xmlns="http://www.w3.org/2000/svg"><defs><clipPath id="cp"><rect x="0" y="0" width="${W}" height="${H}"/></clipPath></defs><g clip-path="url(#cp)">`;
  const legend = [];
  for (let i = p.ext.length - 1; i >= 0; i--) {
    const e = p.ext[i];
    if (e.after) svg += `<path d="${d(e.after)}" fill="${esc(e.color)}" stroke="#0005" stroke-width="0.4"/>`;
    legend.push([e.color, "ext: " + e.name]);
  }
  // plies that ended before this station (tip drops) are not drawn
  const live = p.rows.filter((r) => !r.ended);
  const cols = rowColours(live);
  svg += `<path d="${d(p.outer)}" fill="${esc(cols.length ? cols[0] : coreColor())}" stroke="#000" stroke-width="0.6"/>`;
  live.forEach((r, i) => {
    const next = i + 1 < cols.length ? cols[i + 1] : coreColor();
    if (r.after) svg += `<path d="${d(r.after)}" fill="${esc(next)}" stroke="#0006" stroke-width="0.4"/>`;
    legend.push([cols[i], r.name]);
  });
  p.rows.filter((r) => r.ended).forEach((r) => legend.push(["transparent", r.name]));
  legend.push([coreColor(), "core"]);
  svg += "</g></svg>";
  el.innerHTML = svg;
  $("#pvLegend").innerHTML = `<span>s = ${fmt(p.s / S.k, 1)} ${S.units}, chord ${fmt(p.chord / S.k, 1)}, t ${fmt(p.thickness, 2)} mm</span>` +
    legend.map(([c, n]) => `<span><i style="background:${esc(c)}"></i>${esc(n)}</span>`).join("");
}
function shade(hex, f) {
  const h = String(hex || "#888888").replace("#", "");
  const ch = [0, 2, 4].map((i) => parseInt(h.slice(i, i + 2), 16));
  const out = ch.map((c) => Math.max(0, Math.min(255, Math.round(f > 0 ? c + (255 - c) * f : c * (1 + f)))));
  return "#" + out.map((c) => c.toString(16).padStart(2, "0")).join("");
}
/* Same-material neighbours get alternating shades so every layer stays visible. */
function rowColours(rows) {
  const out = [];
  rows.forEach((r, i) => {
    let c = r.color;
    if (i > 0 && rows[i - 1].color === r.color) c = shade(r.color, out[i - 1] === r.color ? 0.35 : 0);
    out.push(c);
  });
  return out;
}
function coreColor() {
  const m = S.mats[S.stack.core.material];
  return m ? m.color : "#e9d9a6";
}

// ------------------------------------------------------------------ keepouts
function renderKeepouts() {
  $("#koBody").innerHTML = S.init.bodies.map((b) => `<option value="${esc(b.token)}">${esc(b.name)}</option>`).join("");
  const ko = S.stack.keepouts || [];
  $("#koTable").innerHTML = "<tr><th>Body</th><th>Clearance (mm)</th><th></th></tr>" + ko.map((k, i) =>
    `<tr data-i="${i}"><td>${esc(k.name)}</td><td><input type="number" step="any" min="0" value="${k.clearance ?? 0.5}"></td>
     <td class="act"><button class="icon" data-del="${i}">✕</button></td></tr>`).join("");
}
function bindKeepouts() {
  const add = (b) => {
    if (!b) return;
    if (S.stack.target && b.token === S.stack.target.token) return fail(new Error("The foil itself cannot be a keep-out."));
    S.stack.keepouts = S.stack.keepouts || [];
    if (S.stack.keepouts.some((k) => k.token === b.token)) return;
    S.stack.keepouts.push({ token: b.token, name: b.name, clearance: 0.5 });
    renderKeepouts();
    saveStack();
  };
  $("#koAdd").addEventListener("click", () => add(S.init.bodies.find((b) => b.token === $("#koBody").value)));
  $("#koSel").addEventListener("click", async () => { try { add(await call("pickSelected")); } catch (e) { fail(e); } });
  $("#koTable").addEventListener("change", (ev) => {
    const tr = ev.target.closest("tr[data-i]");
    S.stack.keepouts[+tr.dataset.i].clearance = +ev.target.value;
    saveStack();
  });
  $("#koTable").addEventListener("click", (ev) => {
    if (ev.target.dataset.del === undefined) return;
    S.stack.keepouts.splice(+ev.target.dataset.del, 1);
    renderKeepouts();
    saveStack();
  });
}

// ------------------------------------------------------------------ build
function bindBuild() {
  $("#replanSeams").addEventListener("click", () => { S.stack.seamPlan = {}; saveStack(); logLine("Locked seams cleared; they will be re-planned."); });
  $("#generate").addEventListener("click", async () => {
    if (!S.stack.target) return fail(new Error("Pick the foil body first (Setup)."));
    if (!S.stack.layers.length && !S.stack.external.length) return fail(new Error("Add some layers first (Stack)."));
    $("#busy").textContent = "Generating... (see the Fusion progress window)";
    try { await call("generate", { stack: S.stack }); } catch (e) { fail(e); }
  });
  $("#flatten").addEventListener("click", async () => {
    if (!S.stack.target) return fail(new Error("Pick the foil body first (Setup)."));
    $("#busy").textContent = "Flattening...";
    try {
      const r = await call("flatten", { stack: S.stack, sketches: $("#flatSketches").checked });
      if (r.cancelled) $("#busy").textContent = "";
    } catch (e) { fail(e); }
  });
}
function onGenerateDone(r) {
  $("#busy").textContent = "";
  (r.log || []).forEach((l) => logLine(l));
  if (r.error) return fail(new Error(r.error));
  if (r.seamPlan) S.stack.seamPlan = r.seamPlan;
  const sum = (pred, key = "mass_g") => r.bodies.filter(pred).reduce((a, b) => a + (b[key] || 0), 0);
  const isLam = (b) => b.role === "ply" || b.role === "fill";
  const lam = sum(isLam), dry = sum(isLam, "dry_g");
  const core = sum((b) => b.role === "core" || b.role === "corefill" || b.role === "gap");
  const ext = sum((b) => b.role === "external"), extDry = sum((b) => b.role === "external", "dry_g");
  const part = lam + core;
  let html = `<p>Generated ${r.bodies.length} bodies (one per ply). Finished part <b>${fmt(part, 0)} g</b>; dry fibre <b>${fmt(dry, 0)} g</b>.</p>`;
  if (r.skipped && r.skipped.length) {
    html += `<p class="err">Not built (see log): ${r.skipped.map(esc).join(", ")}. The masses exclude them.</p>`;
  }
  html += `<table class="tbl"><tr><th>Body</th><th>Role</th><th>Material</th><th>Volume cm³</th><th>Dry fibre g</th><th>With resin g</th></tr>` +
    r.bodies.map((b) => `<tr><td>${esc(b.name)}</td><td>${esc(b.role === "gap" ? "drape gap (core)" : b.role)}</td><td>${esc(b.material)}</td>
      <td class="num">${fmt(b.volume_cm3, 2)}</td><td class="num"><b>${b.dry_g == null ? "" : fmt(b.dry_g, 1)}</b></td>
      <td class="num">${fmt(b.mass_g, 1)}</td></tr>`).join("") + "</table>";
  if (r.seams && r.seams.length) {
    html += `<h3>Seams</h3><table class="tbl"><tr><th>Ply</th><th>Seam</th><th></th></tr>` +
      r.seams.map((s) => `<tr><td>${esc(s.piece)} ${esc(s.label)}</td><td>${esc(s.msg)}</td><td>${s.ok ? '<span class="badge ok">ok</span>' : '<span class="badge warn">check</span>'}</td></tr>`).join("") + "</table>";
  }
  html += `<h3>Mass</h3><table class="tbl mass">
    <tr><td>Dry fibre (all plies)</td><td class="num"><b>${fmt(dry, 1)} g</b></td></tr>
    <tr><td>Resin in the laminate</td><td class="num">${fmt(lam - dry, 1)} g</td></tr>
    <tr><td>Laminate with resin</td><td class="num">${fmt(lam, 1)} g</td></tr>
    <tr><td>Core (incl. drape gaps)</td><td class="num">${fmt(core, 1)} g</td></tr>
    <tr><th>Finished part</th><th class="num">${fmt(part, 1)} g</th></tr>
    ${ext ? `<tr><td>External layers (removed after cure), dry</td><td class="num">${fmt(extDry, 1)} g</td></tr>` : ""}
  </table>`;
  if (r.breakdown) html += breakdownTable(r.breakdown);
  html += `<p class="hint">Dry fibre = ply area × fabric g/m² (area = body volume ÷ ply thickness). With resin uses the laminate density at the in-mould Vf.</p>`;
  $("#results").innerHTML = html;
}
/* Mass by material, then fibre totals only where a fibre has several materials
   (same rule as engine.mass_breakdown). rows: {material, fibre, dry_g, wet_g}. */
function massBreakdown(rows) {
  const mats = new Map();
  for (const r of rows) {
    const m = mats.get(r.material) || { material: r.material, fibre: r.fibre || "", dry_g: 0, wet_g: 0, plies: 0 };
    m.dry_g += r.dry_g || 0;
    m.wet_g += r.wet_g || 0;
    m.plies += 1;
    mats.set(r.material, m);
  }
  const byMaterial = [...mats.values()];
  const fib = new Map();
  for (const m of byMaterial) {
    if (!m.fibre) continue;
    const f = fib.get(m.fibre) || { fibre: m.fibre, dry_g: 0, wet_g: 0, materials: 0 };
    f.dry_g += m.dry_g; f.wet_g += m.wet_g; f.materials += 1;
    fib.set(m.fibre, f);
  }
  return { byMaterial, byFibre: [...fib.values()].filter((f) => f.materials > 1) };
}
function breakdownTable(b) {
  const cap = (s) => s.charAt(0).toUpperCase() + s.slice(1);
  let h = `<h3>Mass by material</h3><table class="tbl mass"><tr><th>Material</th><th>Plies</th><th>Dry fibre g</th><th>With resin g</th></tr>`;
  h += b.byMaterial.map((m) => `<tr><td>${esc(m.material)}</td><td class="num">${m.plies}</td><td class="num"><b>${fmt(m.dry_g, 1)}</b></td><td class="num">${fmt(m.wet_g, 1)}</td></tr>`).join("");
  h += b.byFibre.map((f) => `<tr><th>All ${esc(f.fibre)}</th><td></td><th class="num">${fmt(f.dry_g, 1)}</th><th class="num">${fmt(f.wet_g, 1)}</th></tr>`).join("");
  return h + "</table>";
}
function onFlattenDone(r) {
  $("#busy").textContent = "";
  (r.log || []).forEach((l) => logLine(l));
  if (r.error) return fail(new Error(r.error));
  const badge = (b) => {
    if (b.p95_shear_deg > 45 || b.rms_strain_pct > 3) return '<span class="badge bad">split / dart</span>';
    if (b.p95_shear_deg > 30 || b.rms_strain_pct > 1) return '<span class="badge warn">check drape</span>';
    return '<span class="badge ok">ok</span>';
  };
  let html = `<p>${r.patterns.length} patterns written to <a href="#" id="openFolder">${esc(r.folder)}</a> · roll length ${fmt(r.rollLength / S.k, 0)} ${S.units}</p>`;
  html += `<table class="tbl"><tr><th>Piece</th><th>Material</th><th>Angle</th><th>Seam</th><th>Area m²</th><th>Dry fibre g</th><th>With resin g</th><th>Size (mm)</th><th>Fibre strain rms %</th><th>Shear p95 °</th><th>Drape</th></tr>` +
    r.patterns.map((b) => `<tr><td>${esc(b.label)}</td><td>${esc(b.material)}</td><td class="num">${b.angle_deg}</td><td>${esc(b.seam)}</td>
      <td class="num">${fmt(b.area_m2, 4)}</td><td class="num"><b>${fmt(b.dry_fabric_g, 1)}</b></td><td class="num">${fmt(b.with_resin_g, 1)}</td><td class="num">${fmt(b.length_mm, 0)} × ${fmt(b.width_mm, 0)}</td>
      <td class="num">${fmt(b.rms_strain_pct, 2)}</td><td class="num">${fmt(b.p95_shear_deg, 1)}</td><td>${badge(b)}</td></tr>`).join("") + "</table>";
  const byName = {};
  for (const m of Object.values(S.mats)) byName[m.name] = m;
  html += breakdownTable(massBreakdown(r.patterns.filter((b) => b.kind !== "ext").map((b) => ({
    material: b.material, fibre: (byName[b.material] || {}).fibre || "", dry_g: b.dry_fabric_g, wet_g: b.with_resin_g }))));
  html += `<div class="plot" style="margin-top:8px">${r.svg}</div>`;
  $("#results").innerHTML = html;
  const a = $("#openFolder");
  if (a) a.addEventListener("click", (e) => { e.preventDefault(); call("openFolder", { folder: r.folder }).catch(fail); });
}

// ------------------------------------------------------------------ start
function renderAll() {
  renderSetup();
  renderMaterials();
  renderStack();
  renderKeepouts();
}
async function start() {
  for (let i = 0; i < 50 && !bridgeReady() && !window.FoilLayupMock; i++) await new Promise((r) => setTimeout(r, 100));
  try {
    S.init = await call("init");
  } catch (e) {
    document.body.insertAdjacentHTML("afterbegin", `<p class="err">${esc(e.message)}</p>`);
    return;
  }
  S.units = S.init.units;
  S.k = S.init.mmPerUnit;
  S.library = S.init.library || { materials: [], presets: [] };
  S.library.materials = S.library.materials || [];
  S.library.presets = S.library.presets || [];
  rebuildMaterials();
  S.stack = { ...defaultStack(), ...(S.init.stack || {}) };
  S.stack.settings = { ...S.init.defaults, ...(S.stack.settings || {}) };
  S.stack.frame = { ...defaultStack().frame, ...(S.stack.frame || {}) };
  S.stack.core = { ...defaultStack().core, ...(S.stack.core || {}) };
  // the palette state lives in the design; never let the browser restore old form values
  $$("input, select").forEach((el) => el.setAttribute("autocomplete", "off"));
  initTabs();
  bindSetup();
  bindMaterials();
  bindStack();
  bindKeepouts();
  bindBuild();
  renderAll();
}
document.addEventListener("DOMContentLoaded", start);
