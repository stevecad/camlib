"""Flat parts in and out of DXF (camlib.dxf): what is read is the drawing's
own lines and arcs, units and joins are declared, and anything camlib cannot
read exactly is refused by name."""

import math
import random
import tempfile
import unittest
from pathlib import Path

from camlib.dxf import DxfError, read_parts, write_regions
from camlib.planar import Boundary, Component, PlanarRegion


def dxf(*entities, header=()):
    lines = []
    if header:
        lines += ["0", "SECTION", "2", "HEADER", *[str(v) for v in header], "0", "ENDSEC"]
    lines += ["0", "SECTION", "2", "ENTITIES"]
    for entity in entities:
        lines += [str(v) for v in entity]
    lines += ["0", "ENDSEC", "0", "EOF"]
    return "\n".join(lines) + "\n"


def line(a, b, layer="0"):
    return (0, "LINE", 8, layer, 10, a[0], 20, a[1], 11, b[0], 21, b[1])


def arc(c, r, start, end, layer="0"):
    return (0, "ARC", 8, layer, 10, c[0], 20, c[1], 40, r, 50, start, 51, end)


def circle(c, r, layer="0"):
    return (0, "CIRCLE", 8, layer, 10, c[0], 20, c[1], 40, r)


def lwpolyline(points, closed=True, layer="0"):
    out = [0, "LWPOLYLINE", 8, layer, 90, len(points), 70, 1 if closed else 0]
    for p in points:
        out += [10, p[0], 20, p[1]] + ([42, p[2]] if len(p) > 2 and p[2] else [])
    return tuple(out)


def area(boundary):
    """Signed area of a line/arc boundary, exactly (segments under arcs)."""
    v = boundary.vertices
    total = 0.0
    for (x0, y0, bulge), (x1, y1, _) in zip(v, v[1:] + v[:1]):
        total += (x0 * y1 - x1 * y0) / 2
        if bulge:
            sweep = 4 * math.atan(bulge)
            r = math.hypot(x1 - x0, y1 - y0) / (2 * math.sin(abs(sweep) / 2))
            total += math.copysign(r * r * (abs(sweep) - math.sin(abs(sweep))) / 2, sweep)
    return total


class ReadTest(unittest.TestCase):
    def read(self, text, **options):
        values = dict(unit_mm=1, join_tolerance_mm=1e-6)
        values.update(options)
        return read_parts(text, **values)

    def test_lines_and_arcs_in_any_order_and_direction_become_one_outline_with_its_holes(self):
        # A 60 x 40 plate with a 5 mm fillet at one corner, drawn as loose
        # lines and an arc, with a circle and a slot (a closed polyline with
        # two half-turn bulges) inside it.
        entities = [
            line((0, 0), (60, 0)),
            line((60, 0), (60, 35)),
            arc((55, 35), 5, 0, 90),
            line((55, 40), (0, 40)),
            line((0, 40), (0, 0)),
            circle((15, 20), 6),
            lwpolyline([(30, 15, 0), (45, 15, 1), (45, 25, 0), (30, 25, 1)]),
        ]
        expected = 60 * 40 - (25 - math.pi * 25 / 4)
        for seed in range(6):
            mixed = entities[:]
            random.Random(seed).shuffle(mixed)
            # Reverse some lines: an outline is drawn in no particular direction.
            mixed = [line(e[9:12:2], e[5:8:2]) if e[1] == "LINE" and (seed + k) % 2 else e for k, e in enumerate(mixed)]
            (part,) = self.read(dxf(*mixed))
            (component,) = part.components
            self.assertAlmostEqual(area(component.outer), expected, places=9)  # counter-clockwise
            self.assertEqual(len(component.holes), 2)
            self.assertEqual(
                sorted(round(-area(h), 9) for h in component.holes),  # clockwise
                sorted([round(math.pi * 36, 9), round(15 * 10 + math.pi * 25, 9)]),
            )
            self.assertEqual(part.boundary_error_mm, 0)
            for boundary in (component.outer, *component.holes):
                self.assertTrue(all(abs(v[2]) <= 1 for v in boundary.vertices))

    def test_each_outline_is_a_part_and_an_outline_in_a_hole_is_its_own_part(self):
        ring = [circle((0, 0), 20), circle((0, 0), 12)]
        parts = self.read(dxf(*ring, circle((0, 0), 5), lwpolyline([(40, 0), (50, 0), (50, 10), (40, 10)])))
        self.assertEqual(len(parts), 3)
        by_area = sorted(parts, key=lambda p: area(p.components[0].outer))
        self.assertAlmostEqual(area(by_area[0].components[0].outer), math.pi * 25)
        self.assertEqual(by_area[0].components[0].holes, ())
        self.assertEqual(len(by_area[2].components[0].holes), 1)  # the ring
        self.assertEqual([p.name for p in parts], ["dxf-1", "dxf-2", "dxf-3"])

    def test_units_are_declared_and_must_agree_with_the_file(self):
        square = lwpolyline([(0, 0), (2, 0), (2, 1), (0, 1)])
        (part,) = self.read(dxf(square), unit_mm=25.4, join_tolerance_mm=0.001)
        self.assertAlmostEqual(area(part.components[0].outer), 2 * 25.4 * 25.4)
        inches = dxf(square, header=(9, "$INSUNITS", 70, 1))
        self.read(inches, unit_mm=25.4)
        with self.assertRaisesRegex(DxfError, r"\$INSUNITS is 1 \(25.4 mm per unit\) but unit_mm=1 was declared"):
            self.read(inches)
        with self.assertRaises(TypeError):
            read_parts(dxf(square))  # no default unit or join tolerance
        with self.assertRaisesRegex(DxfError, "unit_mm must be a positive number"):
            self.read(dxf(square), unit_mm=0)

    def test_a_gap_is_closed_only_within_the_declared_tolerance_and_reported(self):
        gapped = [line((0, 0), (10, 0)), line((10, 0.004), (10, 10)), line((10, 10), (0, 10)), line((0, 10), (0, 0))]
        with self.assertRaisesRegex(DxfError, r"LINE 0 on layer '0' ends at \(10, 0\) with nothing joined.*open there"):
            self.read(dxf(*gapped))
        (part,) = self.read(dxf(*gapped), join_tolerance_mm=0.01)
        self.assertAlmostEqual(part.boundary_error_mm, 0.004)
        with self.assertRaisesRegex(DxfError, "3 ends meet at"):
            self.read(dxf(*gapped, line((0, 0), (-5, -5))), join_tolerance_mm=0.01)

    def test_what_is_not_a_line_or_an_arc_is_refused_by_name(self):
        square = lwpolyline([(0, 0), (2, 0), (2, 1), (0, 1)])
        spline = (0, "SPLINE", 5, "2F", 8, "cut", 70, 8)
        with self.assertRaisesRegex(DxfError, r"entity 1 is a SPLINE \(handle 2F\) on layer 'cut'.*explode splines"):
            self.read(dxf(square, spline))
        text = (0, "TEXT", 8, "notes", 10, 0, 20, 0, 40, 3, 1, "PART 7")
        with self.assertRaisesRegex(DxfError, "entity 1 is a TEXT.*ignore="):
            self.read(dxf(square, text))
        self.assertEqual(len(self.read(dxf(square, text), ignore=("text",))), 1)
        self.assertEqual(len(self.read(dxf(square, spline), layers=("0",))), 1)
        with self.assertRaisesRegex(DxfError, "not in the Z = 0 plane"):
            self.read(dxf((0, "LINE", 8, "0", 10, 0, 20, 0, 30, 5, 11, 1, 21, 0, 31, 5)))
        with self.assertRaisesRegex(DxfError, "no outline was found on layers"):
            self.read(dxf(square), layers=("cut",))
        with self.assertRaisesRegex(DxfError, "not a group code.*binary DXF"):
            self.read("AutoCAD Binary DXF\nx\n")

    def test_a_polyline_arc_over_a_half_turn_is_split_and_keeps_its_area(self):
        # A keyhole: a 270 degree arc closed by two lines to its centre.
        r = 10.0
        bulge = math.tan(math.radians(270) / 4)
        (part,) = self.read(dxf(lwpolyline([(r, 0, bulge), (0, -r, 0), (0, 0, 0)])))
        outer = part.components[0].outer
        self.assertTrue(all(abs(v[2]) <= 1 for v in outer.vertices))
        self.assertAlmostEqual(area(outer), 0.75 * math.pi * r * r, places=9)


class WriteTest(unittest.TestCase):
    def test_regions_round_trip_through_a_file_in_millimetres(self):
        plate = PlanarRegion(
            "plate",
            (
                Component(
                    Boundary(((0, 0, 0), (60, 0, 0), (60, 35, math.tan(math.pi / 8)), (55, 40, 0), (0, 40, 0))),
                    (Boundary.circle((15, 20), 6), Boundary(((30, 15, 0), (30, 25, -1), (45, 25, 0), (45, 15, -1)))),
                ),
            ),
        )
        disc = PlanarRegion("disc", (Component(Boundary.circle((100, 20), 8)),))
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "remnant sheet.dxf"
            text = write_regions((plate, disc), path)
            self.assertIn("$INSUNITS\n 70\n4\n", text)
            back = read_parts(path, unit_mm=1, join_tolerance_mm=1e-9)
        self.assertEqual([p.name for p in back], ["remnant sheet-1", "remnant sheet-2"])
        for original, read in zip((plate, disc), back):
            a, b = original.components[0], read.components[0]
            self.assertAlmostEqual(area(a.outer), area(b.outer), places=9)
            self.assertEqual(len(a.holes), len(b.holes))
            self.assertAlmostEqual(sum(abs(area(h)) for h in a.holes), sum(abs(area(h)) for h in b.holes), places=9)
            # The same points, exactly (written with full precision).
            self.assertEqual({(v[0], v[1]) for v in a.outer.vertices}, {(v[0], v[1]) for v in b.outer.vertices})
        with self.assertRaisesRegex(DxfError, "one PlanarRegion or several"):
            write_regions(())


if __name__ == "__main__":
    unittest.main()
