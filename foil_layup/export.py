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
    """Warp arrow (+X), span arrow, and label text for one pattern part."""
    x0, y0, x1, y1 = part["bbox"]
    cx = (x0 + x1) / 2.0
    cy = (y0 + y1) / 2.0
    size = max(10.0, min(x1 - x0, y1 - y0) * 0.25)
    lines = arrow(cx - size / 2, cy, size, 0.0)
    lines += arrow(cx, cy - size * 0.6, size * 0.6, part["spanAngle"] + 0.0)
    th = max(2.5, min(8.0, size * 0.12))
    texts = [(cx - size / 2, cy + th * 0.8, th, label), (cx - size / 2, cy - th * 2.2, th * 0.7, "warp 0 deg ->")]
    return lines, texts


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
            for name, col in (("OUTLINE", 7), ("HOLES", 1), ("ANNOT", 3)):
                f.write("0\nLAYER\n2\n%s\n70\n0\n62\n%d\n6\nCONTINUOUS\n" % (name, col))
            f.write("0\nENDTAB\n0\nENDSEC\n")
            f.write("0\nSECTION\n2\nENTITIES\n")
            f.write(self.body.getvalue())
            f.write("0\nENDSEC\n0\nEOF\n")


def write_part_dxf(dxf, part, label, dx=0.0, dy=0.0):
    dxf.polyline(_shift(part["outline"], dx, dy), "OUTLINE")
    for h in part["holes"]:
        dxf.polyline(_shift(h, dx, dy), "HOLES")
    lines, texts = pattern_annotations(part, label)
    for a, b in lines:
        dxf.line((a[0] + dx, a[1] + dy), (b[0] + dx, b[1] + dy))
    for x, y, h, s in texts:
        dxf.text(x + dx, y + dy, h, s)


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
    pad = 10
    out = ['<svg xmlns="http://www.w3.org/2000/svg" viewBox="%g %g %g %g" width="100%%">'
           % (-pad, -pad, width + 2 * pad, height + 2 * pad)]
    out.append('<title>%s</title>' % _esc(title))
    # own background so the sheet reads the same in light and dark viewers
    out.append('<rect x="%g" y="%g" width="%g" height="%g" fill="#ffffff"/>'
               % (-pad, -pad, width + 2 * pad, height + 2 * pad))
    out.append('<g transform="scale(1,-1) translate(0,%g)">' % (-height))
    for it, off in zip(items, offsets):
        if off is None:
            continue
        dx, dy = off
        part = it
        d = _path(_shift(part["outline"], dx, dy))
        for h in part["holes"]:
            d += _path(_shift(h, dx, dy))
        out.append('<path d="%s" fill="%s" fill-opacity="0.35" fill-rule="evenodd" stroke="#333" stroke-width="0.6"/>'
                   % (d, it.get("color", "#888")))
        lines, _ = pattern_annotations(part, "")
        for a, b in lines:
            out.append('<line x1="%.2f" y1="%.2f" x2="%.2f" y2="%.2f" stroke="#0a7" stroke-width="0.6"/>'
                       % (a[0] + dx, a[1] + dy, b[0] + dx, b[1] + dy))
    out.append("</g>")
    for it, off in zip(items, offsets):
        if off is None:
            continue
        x0, y0, x1, y1 = it["bbox"]
        cx = (x0 + x1) / 2 + off[0]
        cy = height - ((y0 + y1) / 2 + off[1])
        out.append('<text x="%.1f" y="%.1f" font-size="%.1f" font-family="sans-serif" text-anchor="middle">%s</text>'
                   % (cx, cy, max(4.0, min(12.0, (y1 - y0) * 0.12)), _esc(it.get("label", ""))))
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
