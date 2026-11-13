"""Flat outlines in and out of DXF: sheet parts as ``PlanarRegion`` values.

``read_parts`` reads the closed outlines of a drawing (lines, arcs, circles and
polylines with bulges, the entities a flat part is drawn with) and returns one
``PlanarRegion`` per part: an outline with the holes drawn inside it.
``write_regions`` writes regions back as closed polylines, for a remnant or a
nest to be opened elsewhere. This is 2D line/arc geometry only: no splines,
ellipses, blocks or 3D, each of which is refused by name; camlib holds no CAD
kernel.

Nothing about a drawing is assumed. The caller states the drawing unit
(``unit_mm``, millimetres per drawing unit); a file whose header states
another unit (``$INSUNITS``) is refused. The caller states how far apart two
end points may be and still be joined (``join_tolerance_mm``); the largest gap
actually closed becomes the region's ``boundary_error_mm``, which a thermal
cut adds to its allowance. Entities that are not outlines (text, dimensions)
are refused unless the caller lists their types in ``ignore``.

The format is the ASCII DXF group-code format (Autodesk, "DXF Reference",
ENTITIES section: LINE, ARC, CIRCLE, LWPOLYLINE, POLYLINE/VERTEX/SEQEND; a
polyline vertex's group 42 is its bulge, the tangent of a quarter of the arc's
included angle, positive counter-clockwise, which is camlib's bulge too).
"""

from dataclasses import dataclass
import math
from pathlib import Path

from .planar import Boundary, Component, PlanarRegion

# $INSUNITS values (DXF Reference, HEADER section) with their size in mm.
_INSUNITS_MM = {1: 25.4, 2: 304.8, 4: 1.0, 5: 10.0, 6: 1000.0, 8: 25.4e-6, 9: 0.0254, 10: 914.4, 13: 1e-3, 14: 100.0}
_OUTLINES = ("LINE", "ARC", "CIRCLE", "LWPOLYLINE", "POLYLINE")


class DxfError(ValueError):
    """A drawing camlib cannot read exactly; the message names the entity."""


@dataclass(frozen=True)
class _Segment:
    a: tuple
    b: tuple
    bulge: float
    layer: str
    what: str

    def reversed(self):
        return _Segment(self.b, self.a, -self.bulge, self.layer, self.what)


def _pairs(text):
    lines = text.splitlines()
    if len(lines) % 2:
        lines = lines[:-1] if not lines[-1].strip() else lines
    if len(lines) % 2:
        raise DxfError("DXF: the file does not end on a whole group (a code line and a value line)")
    out = []
    for i in range(0, len(lines), 2):
        try:
            code = int(lines[i].strip())
        except ValueError:
            raise DxfError(
                f"DXF: line {i + 1} is not a group code: {lines[i].strip()!r} (binary DXF and DWG are not read)"
            ) from None
        out.append((code, lines[i + 1].strip()))
    return out


def _number(value, what):
    try:
        number = float(value)
    except ValueError:
        raise DxfError(f"DXF: {what} is not a number: {value!r}") from None
    if not math.isfinite(number):
        raise DxfError(f"DXF: {what} is not finite")
    return number


def _sections(pairs):
    """{section name: its pairs}, from 0/SECTION 2/<name> ... 0/ENDSEC."""
    sections, i = {}, 0
    while i < len(pairs):
        if pairs[i] == (0, "SECTION") and i + 1 < len(pairs) and pairs[i + 1][0] == 2:
            name, start = pairs[i + 1][1].upper(), i + 2
            end = start
            while end < len(pairs) and pairs[end] != (0, "ENDSEC"):
                end += 1
            if end == len(pairs):
                raise DxfError(f"DXF: section {name} has no ENDSEC")
            sections[name] = pairs[start:end]
            i = end + 1
        else:
            i += 1
    return sections


def _header_units(header):
    for i, (code, value) in enumerate(header):
        if code == 9 and value.upper() == "$INSUNITS" and i + 1 < len(header):
            return int(_number(header[i + 1][1], "$INSUNITS"))
    return None


def _entities(pairs):
    """[(type, [(code, value), ...])] of a section; a POLYLINE takes its
    VERTEX entities up to SEQEND."""
    out, current = [], None
    for code, value in pairs:
        if code == 0:
            current = (value.upper(), [])
            out.append(current)
        elif current is None:
            raise DxfError("DXF: the ENTITIES section does not start with an entity")
        else:
            current[1].append((code, value))
    merged, i = [], 0
    while i < len(out):
        kind, groups = out[i]
        if kind == "POLYLINE":
            vertices = []
            i += 1
            while i < len(out) and out[i][0] == "VERTEX":
                vertices.append(out[i][1])
                i += 1
            if i == len(out) or out[i][0] != "SEQEND":
                raise DxfError("DXF: a POLYLINE has no SEQEND")
            merged.append((kind, groups, vertices))
        elif kind in ("VERTEX", "SEQEND"):
            raise DxfError(f"DXF: a {kind} outside a POLYLINE")
        else:
            merged.append((kind, groups, None))
        i += 1
    return merged


def _get(groups, code, what, default=None):
    for c, value in groups:
        if c == code:
            return value
    if default is None:
        raise DxfError(f"DXF: {what} has no group {code}")
    return default


def _arc_segments(center, radius, start, sweep, layer, what):
    """An arc from angle `start` over `sweep` radians (counter-clockwise when
    positive) as pieces of at most a half turn (camlib's arcs are at most pi)."""
    pieces = max(1, math.ceil(abs(sweep) / math.pi - 1e-12))
    step = sweep / pieces
    at = lambda angle: (center[0] + radius * math.cos(angle), center[1] + radius * math.sin(angle))  # noqa: E731
    return [
        _Segment(at(start + k * step), at(start + (k + 1) * step), math.tan(step / 4), layer, what)
        for k in range(pieces)
    ]


def _polyline_segments(points, closed, layer, what):
    """[(x, y, bulge)] to segments; a bulge over a half turn is split."""
    out = []
    count = len(points) if closed else len(points) - 1
    for k in range(count):
        (x0, y0, bulge), (x1, y1, _) = points[k], points[(k + 1) % len(points)]
        if (x0, y0) == (x1, y1):
            continue
        if abs(bulge) <= 1:
            out.append(_Segment((x0, y0), (x1, y1), bulge, layer, what))
            continue
        # More than a half turn: the same arc from its centre, in two pieces.
        sweep = 4 * math.atan(bulge)
        chord = math.hypot(x1 - x0, y1 - y0)
        radius = chord / (2 * math.sin(abs(sweep) / 2))
        # The centre lies to the left of the chord for a counter-clockwise arc
        # under a half turn and to its right over one.
        offset = radius * math.cos(sweep / 2) * (1 if sweep > 0 else -1)
        mx, my = (x0 + x1) / 2, (y0 + y1) / 2
        nx, ny = -(y1 - y0) / chord, (x1 - x0) / chord
        center = (mx + nx * offset, my + ny * offset)
        start = math.atan2(y0 - center[1], x0 - center[0])
        pieces = _arc_segments(center, radius, start, sweep, layer, what)
        # End exactly on the drawn points.
        pieces[0] = _Segment((x0, y0), pieces[0].b, pieces[0].bulge, layer, what)
        pieces[-1] = _Segment(pieces[-1].a, (x1, y1), pieces[-1].bulge, layer, what)
        out.extend(pieces)
    return out


def _outline_segments(kind, groups, vertices, index):
    layer = _get(groups, 8, kind, "0")
    handle = _get(groups, 5, kind, "")
    what = f"{kind} {index}" + (f" (handle {handle})" if handle else "") + f" on layer {layer!r}"
    number = lambda code, default=None: _number(  # noqa: E731
        _get(groups, code, what, None if default is None else str(default)), f"group {code} of {what}"
    )
    if any(code in (30, 31, 38) and _number(value, what) != 0 for code, value in groups):
        raise DxfError(f"DXF: {what} is not in the Z = 0 plane; flat parts are drawn in XY")
    if any(code == 230 and _number(value, what) != 1 for code, value in groups):
        raise DxfError(f"DXF: {what} has an extrusion direction other than +Z; redraw it in the XY plane")
    if kind == "LINE":
        a, b = (number(10), number(20)), (number(11), number(21))
        return [] if a == b else [_Segment(a, b, 0.0, layer, what)], False
    if kind in ("ARC", "CIRCLE"):
        center, radius = (number(10), number(20)), number(40)
        if radius <= 0:
            raise DxfError(f"DXF: {what} has a radius that is not positive")
        if kind == "CIRCLE":
            return _arc_segments(center, radius, 0.0, 2 * math.pi, layer, what), True
        start, end = math.radians(number(50)), math.radians(number(51))
        sweep = (end - start) % (2 * math.pi) or 2 * math.pi
        return _arc_segments(center, radius, start, sweep, layer, what), False
    if kind == "LWPOLYLINE":
        closed = int(number(70, 0)) & 1 == 1
        points, current = [], None
        for code, value in groups:
            if code == 10:
                current = [_number(value, what), None, 0.0]
                points.append(current)
            elif code == 20 and current is not None:
                current[1] = _number(value, what)
            elif code == 42 and current is not None:
                current[2] = _number(value, what)
        if any(p[1] is None for p in points) or len(points) < 2:
            raise DxfError(f"DXF: {what} has fewer than two complete vertices")
        return _polyline_segments([tuple(p) for p in points], closed, layer, what), closed
    # POLYLINE with VERTEX entities.
    flags = int(number(70, 0))
    if flags & ~1:
        raise DxfError(
            f"DXF: {what} is a curve-fit, spline-fit, 3D or mesh polyline (flags {flags}); only 2D polylines of lines and arcs are read"
        )
    points = []
    for vertex in vertices:
        get = lambda code, default: _number(_get(vertex, code, "VERTEX", str(default)), f"a vertex of {what}")  # noqa: E731
        if get(30, 0) != 0:
            raise DxfError(f"DXF: {what} has a vertex off the Z = 0 plane")
        points.append((get(10, math.nan), get(20, math.nan), get(42, 0)))
    if len(points) < 2 or any(math.isnan(p[0]) or math.isnan(p[1]) for p in points):
        raise DxfError(f"DXF: {what} has fewer than two complete vertices")
    return _polyline_segments(points, flags & 1 == 1, layer, what), flags & 1 == 1


def _chain(segments, tolerance):
    """Closed loops from segments joined end to end within `tolerance`, and
    the largest gap closed. Every end must meet exactly one other end."""
    cell = max(tolerance, 1e-12) * 2
    grid = {}
    ends = []  # (segment index, 0 start / 1 end, point)
    for i, s in enumerate(segments):
        for which, p in ((0, s.a), (1, s.b)):
            ends.append((i, which, p))
            grid.setdefault((math.floor(p[0] / cell), math.floor(p[1] / cell)), []).append(len(ends) - 1)

    def near(k):
        _, _, p = ends[k]
        cx, cy = math.floor(p[0] / cell), math.floor(p[1] / cell)
        found = []
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                for other in grid.get((cx + dx, cy + dy), ()):
                    if other != k and math.hypot(ends[other][2][0] - p[0], ends[other][2][1] - p[1]) <= tolerance:
                        found.append(other)
        return found

    partner = {}
    for k, (i, _, p) in enumerate(ends):
        found = near(k)
        if len(found) != 1:
            where = f"({p[0]:.6g}, {p[1]:.6g})"
            if not found:
                raise DxfError(
                    f"DXF: {segments[i].what} ends at {where} with nothing joined to it within join_tolerance_mm: "
                    "the outline is open there. Close it in the drawing, or state a larger join_tolerance_mm if the "
                    "gap is a drawing inaccuracy"
                )
            raise DxfError(
                f"DXF: {len(found) + 1} ends meet at {where} ({segments[i].what} among them): an outline branches or is "
                "drawn twice there. Remove the duplicate or the extra entity"
            )
        partner[k] = found[0]
    used, loops, gap = [False] * len(segments), [], 0.0
    for first in range(len(segments)):
        if used[first]:
            continue
        loop, i, leave = [], first, 1
        while not used[i]:
            used[i] = True
            loop.append(segments[i] if leave == 1 else segments[i].reversed())
            k = partner[2 * i + leave]
            gap = max(
                gap, math.hypot(ends[k][2][0] - ends[2 * i + leave][2][0], ends[k][2][1] - ends[2 * i + leave][2][1])
            )
            i, leave = ends[k][0], 1 - ends[k][1]
        if i != first:
            raise DxfError(f"DXF: the outline through {segments[first].what} does not close on itself")
        loops.append(loop)
    return loops, gap


def _flatten(loop, chord=None):
    """Points along a loop (arcs as short chords), for area and containment
    only; the loop itself keeps its arcs."""
    points = []
    for s in loop:
        points.append(s.a)
        if s.bulge:
            sweep = 4 * math.atan(s.bulge)
            steps = max(2, math.ceil(abs(sweep) / (math.pi / 16)))
            length = math.hypot(s.b[0] - s.a[0], s.b[1] - s.a[1])
            radius = length / (2 * math.sin(abs(sweep) / 2))
            offset = radius * math.cos(sweep / 2) * (1 if sweep > 0 else -1)
            nx, ny = -(s.b[1] - s.a[1]) / length, (s.b[0] - s.a[0]) / length
            cx, cy = (s.a[0] + s.b[0]) / 2 + nx * offset, (s.a[1] + s.b[1]) / 2 + ny * offset
            start = math.atan2(s.a[1] - cy, s.a[0] - cx)
            for k in range(1, steps):
                angle = start + sweep * k / steps
                points.append((cx + radius * math.cos(angle), cy + radius * math.sin(angle)))
    return points


def _area(points):
    return sum(a[0] * b[1] - b[0] * a[1] for a, b in zip(points, points[1:] + points[:1])) / 2


def _inside(p, points):
    inside = False
    for a, b in zip(points, points[1:] + points[:1]):
        if (a[1] > p[1]) != (b[1] > p[1]) and p[0] < a[0] + (p[1] - a[1]) * (b[0] - a[0]) / (b[1] - a[1]):
            inside = not inside
    return inside


def _boundary(loop, counter_clockwise, scale):
    if (_area(_flatten(loop)) > 0) != counter_clockwise:
        loop = [s.reversed() for s in reversed(loop)]
    return Boundary(tuple((s.a[0] * scale, s.a[1] * scale, s.bulge) for s in loop))


def read_parts(source, *, unit_mm, join_tolerance_mm, layers=None, ignore=(), name=None):
    """The parts drawn in a DXF file as ``PlanarRegion`` values, one per
    outline with the holes drawn directly inside it (an outline inside a hole
    is a part of its own), in the order of their outlines in the file.

    ``source`` is a path, or the file's text. ``unit_mm`` is the size of one
    drawing unit in millimetres (1 for a drawing in mm, 25.4 for inches);
    there is no default, and a file whose ``$INSUNITS`` says otherwise is
    refused. ``join_tolerance_mm`` is how far apart two ends may be and still
    be one corner; the largest gap closed is each region's
    ``boundary_error_mm``. ``layers`` restricts reading to those layer names.
    ``ignore`` lists entity types that are not outlines and may be skipped
    (``("TEXT", "DIMENSION")``); any other entity that is not a line, arc,
    circle or 2D polyline is refused by name. Whether the loops cross or
    touch is checked when a region is used (camlib prepares it then)."""
    for label, value in (("unit_mm", unit_mm), ("join_tolerance_mm", join_tolerance_mm)):
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
            raise DxfError(f"DXF: {label} must be a positive number")
    is_path = isinstance(source, Path) or (isinstance(source, str) and "\n" not in source)
    text = Path(source).read_text(errors="strict") if is_path else source
    label = name or (Path(source).stem if is_path else "dxf")
    sections = _sections(_pairs(text))
    if "ENTITIES" not in sections:
        raise DxfError("DXF: the file has no ENTITIES section")
    stated = _header_units(sections.get("HEADER", []))
    if stated not in (None, 0):
        size = _INSUNITS_MM.get(stated)
        if size is None or not math.isclose(size, unit_mm, rel_tol=1e-12):
            raise DxfError(
                f"DXF: the file's $INSUNITS is {stated}"
                + (f" ({size:g} mm per unit)" if size else " (a unit camlib does not know)")
                + f" but unit_mm={unit_mm:g} was declared; they must agree"
            )
    ignore = {kind.upper() for kind in ignore}
    if ignore & set(_OUTLINES):
        raise DxfError("DXF: ignore= lists an outline entity type; use layers= to leave outlines out")
    wanted = None if layers is None else set(layers)
    tolerance = join_tolerance_mm / unit_mm
    open_segments, loops = [], []
    for index, (kind, groups, vertices) in enumerate(_entities(sections["ENTITIES"])):
        if wanted is not None and _get(groups, 8, kind, "0") not in wanted:
            continue
        if kind not in _OUTLINES:
            if kind in ignore:
                continue
            handle = _get(groups, 5, kind, "")
            raise DxfError(
                f"DXF: entity {index} is a {kind}"
                + (f" (handle {handle})" if handle else "")
                + f" on layer {_get(groups, 8, kind, '0')!r}: only LINE, ARC, CIRCLE, LWPOLYLINE and 2D POLYLINE "
                "are read. Convert it to lines and arcs in the drawing (explode splines, ellipses and blocks), "
                "or list its type in ignore= if it is not part of an outline"
            )
        segments, closed = _outline_segments(kind, groups, vertices, index)
        if not segments:
            continue
        if closed:
            loops.append(segments)
        else:
            open_segments.extend(segments)
    gap = 0.0
    if open_segments:
        chained, gap = _chain(open_segments, tolerance)
        loops.extend(chained)
    if not loops:
        raise DxfError("DXF: no outline was found" + (f" on layers {sorted(wanted)}" if wanted else ""))
    flat = [_flatten(loop) for loop in loops]
    for loop, points in zip(loops, flat):
        if abs(_area(points)) <= tolerance * tolerance:
            raise DxfError(f"DXF: the outline through {loop[0].what} encloses no area")
    # Depth by containment: outlines at even depth are parts, the loops
    # directly inside them (odd depth) their holes.
    order = sorted(range(len(loops)), key=lambda i: -abs(_area(flat[i])))
    parent = {}
    for rank, i in enumerate(order):
        probe = flat[i][0]
        for j in reversed(order[:rank]):  # the smallest loop that holds it
            if _inside(probe, flat[j]):
                parent[i] = j
                break
    depth = {}
    for i in order:
        depth[i] = depth[parent[i]] + 1 if i in parent else 0
    parts = []
    for i in range(len(loops)):
        if depth[i] % 2:
            continue
        holes = tuple(_boundary(loops[j], False, unit_mm) for j in range(len(loops)) if parent.get(j) == i)
        parts.append(
            PlanarRegion(
                f"{label}-{len(parts) + 1}",
                (Component(_boundary(loops[i], True, unit_mm), holes),),
                gap * unit_mm,
                {"dxf": label, "layer": loops[i][0].layer},
            )
        )
    return tuple(parts)


def _text(regions):
    lines = []
    add = lambda code, value: lines.extend((f"{code:>3}", str(value)))  # noqa: E731
    number = lambda v: repr(float(v))  # noqa: E731
    # Units stated in the header: millimetres, metric.
    for code, value in (
        (0, "SECTION"),
        (2, "HEADER"),
        (9, "$ACADVER"),
        (1, "AC1009"),
        (9, "$INSUNITS"),
        (70, 4),
        (9, "$MEASUREMENT"),
        (70, 1),
        (0, "ENDSEC"),
        (0, "SECTION"),
        (2, "ENTITIES"),
    ):
        add(code, value)
    for region in regions:
        for component in region.components:
            for layer, boundary in [("OUTLINE", component.outer), *(("HOLE", hole) for hole in component.holes)]:
                for code, value in ((0, "POLYLINE"), (8, layer), (66, 1), (70, 1)):
                    add(code, value)
                for x, y, bulge in boundary.vertices:
                    for code, value in ((0, "VERTEX"), (8, layer), (10, number(x)), (20, number(y)), (30, "0.0")):
                        add(code, value)
                    if bulge:
                        add(42, number(bulge))
                add(0, "SEQEND")
    for code, value in ((0, "ENDSEC"), (0, "EOF")):
        add(code, value)
    return "\n".join(lines) + "\n"


def write_regions(regions, path=None):
    """Regions (one ``PlanarRegion`` or several) as an ASCII DXF in
    millimetres: every outline a closed polyline on layer ``OUTLINE``, every
    hole one on layer ``HOLE``, arcs as bulges. Returns the text; writes it
    to ``path`` when given. ``read_parts(path, unit_mm=1, ...)`` reads it back."""
    if isinstance(regions, PlanarRegion):
        regions = (regions,)
    regions = tuple(regions)
    if not regions or any(not isinstance(r, PlanarRegion) for r in regions):
        raise DxfError("DXF: write_regions takes one PlanarRegion or several")
    text = _text(regions)
    if path is not None:
        Path(path).write_text(text)
    return text
