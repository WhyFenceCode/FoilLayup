"""DXF / SVG / CSV writers and simple roll nesting for flat patterns (mm)."""
import csv
import io
import math
import os
import re


def safe_name(text):
    return re.sub(r"[^A-Za-z0-9._-]+", "_", text).strip("_")[:80] or "pattern"


# --------------------------------------------------------------------------- annotation
def arrow(x, y, length, angle_deg, head=None):
    a = math.radians(angle_deg)
    head = head or max(3.0, length * 0.15)
    ex, ey = x + length * math.cos(a), y + length * math.sin(a)
    h1 = (ex - head * math.cos(a - 0.4), ey - head * math.sin(a - 0.4))
    h2 = (ex - head * math.cos(a + 0.4), ey - head * math.sin(a + 0.4))
    return [((x, y), (ex, ey)), ((ex, ey), h1), ((ex, ey), h2)]


def pattern_annotations(part, label):
    """Annotation for one pattern part, in its own coordinates:
    (lines, texts, dashed). Arrows show RIGHT (towards the right wing tip,
    seen from above facing forward) and FRONT (towards the leading edge, one
    per side of a wrapped ply); the dashed line is the leading edge."""
    x0, y0, x1, y1 = part["bbox"]
    cx = (x0 + x1) / 2.0
    cy = (y0 + y1) / 2.0
    w, h = x1 - x0, y1 - y0
    size = max(12.0, min(150.0, 0.15 * w, 0.6 * h if h > 40 else 0.15 * w))  # RIGHT / warp arrows
    th = max(3.0, min(14.0, 0.025 * w, 0.18 * h))                              # text height
    marks = part.get("marks") or {}
    lines, texts, dashed = [], [], []
    texts.append((cx - size / 2, cy + th * 0.8, th, label))
    r = marks.get("right")
    narrow = h < 60.0  # strips: spread the marks along the length, clear of the label
    ax0 = cx - 0.25 * w if narrow else cx
    ry = cy if narrow else cy - th * 1.2
    if r:
        ang = math.degrees(math.atan2(r[1], r[0]))
        sx, sy = ax0 - r[0] * size / 2, ry - r[1] * size / 2
        lines += arrow(sx, sy, size, ang)
        texts.append((sx + r[0] * size + th * 0.3, sy + r[1] * size - th * 0.4, th * 0.8, "RIGHT"))
    warp_along_span = r is not None and abs(r[1]) < 0.05 and abs(part.get("spanAngle", 0.0)) < 0.5
    if warp_along_span:
        texts.append((ax0 - size / 2, ry - th * 1.6, th * 0.65, "warp along span"))
    else:
        wy = ry - th * 2.2
        lines += arrow(cx - size / 2, wy, size * 0.7, 0.0)
        texts.append((cx + size * 0.25, wy - th * 0.3, th * 0.65, "warp"))
    le_lines = marks.get("le") or []
    ln = max(8.0, min(60.0, 0.3 * h))
    for f in marks.get("fronts") or []:
        (dx, dy) = f["dir"]
        if le_lines:
            # point at the LE line from this side, a quarter along the part
            # (a side's average position can fall on the other skin when the
            # seam splits it across both edges of the pattern)
            pl = max(le_lines, key=len)
            tx, ty = min(pl, key=lambda q: abs(q[0] - (cx + 0.22 * w)))
            tx, ty = tx - dx * th * 0.3, ty - dy * th * 0.3
            sx, sy = tx - dx * ln, ty - dy * ln
        else:
            (ax, ay) = f["at"]
            if narrow:
                ax = cx + 0.25 * w
            sx, sy = ax - dx * ln / 2, ay - dy * ln / 2
        lines += arrow(sx, sy, ln, math.degrees(math.atan2(dy, dx)))
        texts.append((sx + th * 0.4, sy - dy * th * 0.2 + (-th * 0.9 if dy > 0 else th * 0.3), th * 0.7,
                      "FRONT (%s)" % f["side"]))
    for pl in le_lines:
        dashed.append(pl)
        mx, my = pl[len(pl) // 2]
        texts.append((mx + th * 0.3, my + th * 0.3, th * 0.6, "LE"))
    return lines, texts, dashed


def _shift(poly, dx, dy):
    return [(x + dx, y + dy) for x, y in poly]


# --------------------------------------------------------------------------- DXF (R12, mm)
class Dxf:
    def __init__(self):
        self.body = io.StringIO()

    def _w(self, code, value):
        self.body.write("%d\n%s\n" % (code, value))

    def polyline(self, pts, layer="OUTLINE", closed=True):
        self._w(0, "POLYLINE")
        self._w(8, layer)
        self._w(66, 1)
        self._w(10, 0.0)
        self._w(20, 0.0)
        self._w(30, 0.0)
        self._w(70, 1 if closed else 0)
        for x, y in pts:
            self._w(0, "VERTEX")
            self._w(8, layer)
            self._w(10, "%.4f" % x)
            self._w(20, "%.4f" % y)
            self._w(30, 0.0)
        self._w(0, "SEQEND")
        self._w(8, layer)

    def line(self, a, b, layer="ANNOT"):
        self._w(0, "LINE")
        self._w(8, layer)
        self._w(10, "%.4f" % a[0])
        self._w(20, "%.4f" % a[1])
        self._w(30, 0.0)
        self._w(11, "%.4f" % b[0])
        self._w(21, "%.4f" % b[1])
        self._w(31, 0.0)

    def text(self, x, y, h, s, layer="ANNOT"):
        self._w(0, "TEXT")
        self._w(8, layer)
        self._w(10, "%.4f" % x)
        self._w(20, "%.4f" % y)
        self._w(30, 0.0)
        self._w(40, "%.3f" % h)
        self._w(1, s.replace("\n", " ")[:250])

    def save(self, path):
        with open(path, "w", newline="\r\n") as f:
            f.write("0\nSECTION\n2\nHEADER\n9\n$ACADVER\n1\nAC1009\n9\n$INSUNITS\n70\n4\n0\nENDSEC\n")
            f.write("0\nSECTION\n2\nTABLES\n0\nTABLE\n2\nLAYER\n70\n3\n")
            for name, col in (("OUTLINE", 7), ("HOLES", 1), ("ANNOT", 3), ("LE", 5)):
                f.write("0\nLAYER\n2\n%s\n70\n0\n62\n%d\n6\nCONTINUOUS\n" % (name, col))
            f.write("0\nENDTAB\n0\nENDSEC\n")
            f.write("0\nSECTION\n2\nENTITIES\n")
            f.write(self.body.getvalue())
            f.write("0\nENDSEC\n0\nEOF\n")


def write_part_dxf(dxf, part, label, dx=0.0, dy=0.0):
    dxf.polyline(_shift(part["outline"], dx, dy), "OUTLINE")
    for h in part["holes"]:
        dxf.polyline(_shift(h, dx, dy), "HOLES")
    lines, texts, dashed = pattern_annotations(part, label)
    for a, b in lines:
        dxf.line((a[0] + dx, a[1] + dy), (b[0] + dx, b[1] + dy))
    for pl in dashed:
        dxf.polyline(_shift(pl, dx, dy), "LE", closed=False)
    for x, y, h, t in texts:
        dxf.text(x + dx, y + dy, h, t)


# --------------------------------------------------------------------------- nesting
def nest(items, roll_width, gap=10.0):
    """Column packing along a roll: X = roll length, Y = across the roll.
    Patterns are never rotated so the warp stays along the roll. Returns
    (offsets, used length, indices of patterns wider than the roll)."""
    order = sorted(range(len(items)), key=lambda n: -(items[n]["bbox"][2] - items[n]["bbox"][0]))
    offsets = [None] * len(items)
    cols = []  # [x, width, y_used]
    too_wide = []
    for n in order:
        x0, y0, x1, y1 = items[n]["bbox"]
        w, h = x1 - x0, y1 - y0
        if h > roll_width:
            too_wide.append(n)
        col = None
        for c in cols:
            if w <= c[1] + 1e-9 and c[2] + h <= roll_width:
                col = c
                break
        if col is None:
            x = cols[-1][0] + cols[-1][1] + gap if cols else 0.0
            col = [x, w, 0.0]
            cols.append(col)
        offsets[n] = (col[0] - x0, col[2] - y0)
        col[2] += h + gap
    length = cols[-1][0] + cols[-1][1] if cols else 0.0
    return offsets, length, too_wide


# --------------------------------------------------------------------------- SVG
def svg_document(items, offsets, width, height, title=""):
    """Overview of the nested roll: patterns filled by material, with their
    RIGHT / FRONT arrows and LE lines, a ruler along the roll, a scale bar and
    a material key underneath."""
    pad = 10.0
    fs = max(12.0, min(40.0, max(width, height) / 110.0))   # footer text size (mm)
    mats = []
    for it in items:
        name = (it.get("book") or {}).get("material") or ""
        if name and name not in [m[0] for m in mats]:
            mats.append((name, it.get("color", "#888")))
    per_row = max(1, int(width // (fs * 16)))
    key_rows = (len(mats) + per_row - 1) // per_row
    foot = fs * (6.5 + 1.6 * (key_rows + 3))
    W, H = width + 2 * pad, height + 2 * pad + foot
    out = ['<svg xmlns="http://www.w3.org/2000/svg" viewBox="%g %g %g %g" width="100%%">' % (-pad, -pad, W, H)]
    out.append('<title>%s</title>' % _esc(title))
    # own background so the sheet reads the same in light and dark viewers
    out.append('<rect x="%g" y="%g" width="%g" height="%g" fill="#ffffff"/>' % (-pad, -pad, W, H))
    # the roll
    out.append('<rect x="0" y="0" width="%.2f" height="%.2f" fill="none" stroke="#bbb" stroke-width="0.8" '
               'stroke-dasharray="6 4"/>' % (width, height))
    out.append('<g transform="scale(1,-1) translate(0,%g)">' % (-height))
    texts = []
    for it, off in zip(items, offsets):
        if off is None:
            continue
        dx, dy = off
        d = _path(_shift(it["outline"], dx, dy))
        for h in it["holes"]:
            d += _path(_shift(h, dx, dy))
        out.append('<path d="%s" fill="%s" fill-opacity="0.35" fill-rule="evenodd" stroke="#333" stroke-width="0.6"/>'
                   % (d, it.get("color", "#888")))
        lines, tx, dashed = pattern_annotations(it, it.get("label", ""))
        for a, b in lines:
            out.append('<line x1="%.2f" y1="%.2f" x2="%.2f" y2="%.2f" stroke="#c0392b" stroke-width="0.8" '
                       'stroke-linecap="round"/>' % (a[0] + dx, a[1] + dy, b[0] + dx, b[1] + dy))
        for pl in dashed:
            out.append('<polyline points="%s" fill="none" stroke="#1f5fbf" stroke-width="0.7" stroke-dasharray="4 3"/>'
                       % " ".join("%.2f,%.2f" % (x + dx, y + dy) for x, y in pl))
        texts += [(x + dx, y + dy, h, t) for x, y, h, t in tx]
    out.append("</g>")
    for x, y, h, t in texts:
        out.append('<text x="%.1f" y="%.1f" font-size="%.1f" font-family="sans-serif" fill="#222">%s</text>'
                   % (x, height - y, h, _esc(t)))
    # ruler along the roll: ticks every 100 mm, numbers every 500 mm
    y = height + fs * 0.4
    k = 0
    while k * 100.0 <= width + 1e-6:
        big = k % 5 == 0
        out.append('<line x1="%.1f" y1="%.1f" x2="%.1f" y2="%.1f" stroke="#555" stroke-width="%.1f"/>'
                   % (k * 100.0, y, k * 100.0, y + fs * (0.9 if big else 0.5), 0.8 if big else 0.5))
        if big:
            out.append('<text x="%.1f" y="%.1f" font-size="%.1f" font-family="sans-serif" text-anchor="middle" '
                       'fill="#555">%d</text>' % (k * 100.0, y + fs * 2.0, fs * 0.8, k * 100))
        k += 1
    out.append('<text x="%.1f" y="%.1f" font-size="%.1f" font-family="sans-serif" text-anchor="end" fill="#555">'
               'roll length (mm), roll width %.0f mm</text>' % (width, y + fs * 3.2, fs * 0.8, height))
    # scale bar sized to the sheet (100 / 200 / 500 mm), in ten steps
    bar = next(v for v in (100.0, 200.0, 500.0, 1000.0) if v >= width / 10.0 or v == 1000.0)
    y = height + fs * 5.0
    for k in range(10):
        out.append('<rect x="%.1f" y="%.1f" width="%.1f" height="%.1f" fill="%s" stroke="#222" stroke-width="0.6"/>'
                   % (k * bar / 10, y, bar / 10, fs * 0.6, "#222" if k % 2 == 0 else "#fff"))
    for v in (0, bar / 2, bar):
        out.append('<text x="%.1f" y="%.1f" font-size="%.1f" font-family="sans-serif" text-anchor="middle">%s</text>'
                   % (v, y + fs * 1.7, fs * 0.8, "%d mm" % v if v == bar else "%d" % v))
    out.append('<text x="%.1f" y="%.1f" font-size="%.1f" font-family="sans-serif">dimensions in mm; '
               'patterns are seen from the outside (mould side)</text>' % (bar + fs * 2, y + fs * 0.6, fs * 0.8))
    # key: materials, then the marks
    y = height + fs * 8.0
    out.append('<text x="0" y="%.1f" font-size="%.1f" font-family="sans-serif" font-weight="bold">Key</text>'
               % (y, fs))
    for n, (name, col) in enumerate(mats):
        x = (n % per_row) * fs * 16
        yy = y + fs * 1.6 * (n // per_row + 1)
        out.append('<rect x="%.1f" y="%.1f" width="%.1f" height="%.1f" fill="%s" fill-opacity="0.35" stroke="#333" '
                   'stroke-width="0.6"/>' % (x, yy - fs * 0.85, fs * 1.6, fs, col))
        out.append('<text x="%.1f" y="%.1f" font-size="%.1f" font-family="sans-serif">%s</text>'
                   % (x + fs * 2.0, yy, fs * 0.85, _esc(name)))
    yy = y + fs * 1.6 * (key_rows + 1)
    marks = [("arrow", "RIGHT: towards the right wing tip (seen from above, facing forward)"),
             ("arrow", "FRONT: towards the leading edge (top / bottom side of the ply)"),
             ("dash", "LE: leading edge line (fold of a wrapped ply)")]
    for n, (kind, text) in enumerate(marks):
        x = (n % 3) * fs * 26
        if kind == "arrow":
            for a, b in arrow(x, yy - fs * 0.35, fs * 1.6, 0.0, fs * 0.45):
                out.append('<line x1="%.1f" y1="%.1f" x2="%.1f" y2="%.1f" stroke="#c0392b" stroke-width="1.2"/>'
                           % (a[0], a[1], b[0], b[1]))
        else:
            out.append('<line x1="%.1f" y1="%.1f" x2="%.1f" y2="%.1f" stroke="#1f5fbf" stroke-width="1.2" '
                       'stroke-dasharray="4 3"/>' % (x, yy - fs * 0.35, x + fs * 1.6, yy - fs * 0.35))
        out.append('<text x="%.1f" y="%.1f" font-size="%.1f" font-family="sans-serif">%s</text>'
                   % (x + fs * 2.0, yy, fs * 0.75, _esc(text)))
    out.append("</svg>")
    return "\n".join(out)


def _path(poly):
    return "M" + " L".join("%.2f %.2f" % p for p in poly) + " Z "


def _esc(s):
    return str(s).replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")


# --------------------------------------------------------------------------- CSV
def ply_book_csv(rows):
    buf = io.StringIO()
    if not rows:
        return ""
    w = csv.DictWriter(buf, fieldnames=list(rows[0].keys()))
    w.writeheader()
    for r in rows:
        w.writerow(r)
    return buf.getvalue()


def write_outputs(folder, items, roll_width, title="Foil layup"):
    """Write one DXF per pattern, a nested sheet DXF, an SVG overview and the
    ply book CSV. items: dicts with outline, holes, bbox, spanAngle, label,
    file, color and book (CSV row)."""
    os.makedirs(folder, exist_ok=True)
    written = []
    for it in items:
        d = Dxf()
        write_part_dxf(d, it, it["label"])
        p = os.path.join(folder, safe_name(it["file"]) + ".dxf")
        d.save(p)
        written.append(p)
    offsets, length, too_wide = nest(items, roll_width)
    sheet = Dxf()
    for it, off in zip(items, offsets):
        if off is not None:
            write_part_dxf(sheet, it, it["label"], off[0], off[1])
    sheet.polyline([(0, 0), (length, 0), (length, roll_width), (0, roll_width)], "ANNOT")
    p = os.path.join(folder, "_nested_sheet.dxf")
    sheet.save(p)
    written.append(p)
    svg = svg_document(items, offsets, max(length, 1.0), roll_width, title)
    p = os.path.join(folder, "_overview.svg")
    with open(p, "w", encoding="utf8") as f:
        f.write(svg)
    written.append(p)
    p = os.path.join(folder, "_ply_book.csv")
    with open(p, "w", newline="", encoding="utf8") as f:
        f.write(ply_book_csv([it["book"] for it in items]))
    written.append(p)
    return written, length, too_wide, svg
