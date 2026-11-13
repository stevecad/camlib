"""Nesting sheet parts for a thermal cut through the Python API: a proven,
repeatable layout that runs as a ThermalCut job; parts that do not fit are
refused or listed."""

from dataclasses import replace
import math
import unittest

import camlib
from camlib.planar import Boundary, Component, PlanarRegion
from camlib.simulation import OperationUpdate, VerificationUpdate


def circle(c, r):
    return Boundary(((c[0] + r, c[1], 1), (c[0] - r, c[1], 1)))


def rect(x0, y0, x1, y1):
    return Boundary.polygon(((x0, y0), (x1, y0), (x1, y1), (x0, y1)))


def one(outer, holes=()):
    return PlanarRegion("part", (Component(outer, tuple(holes)),), 0)


PLATE = one(rect(0, 0, 70, 50), (circle((22, 25), 19), circle((60, 10), 4)))
WASHER = one(circle((0, 0), 9), (circle((0, 0), 3),))
SHEET = one(rect(0, 0, 200, 120))


def cut(parts=None):
    torch = camlib.Torch("torch", kerf_width=1.0)
    return camlib.ThermalCut(
        "plasma",
        torch,
        parts,
        2500,
        1.5,
        lead_in=camlib.Lead(1, 1),
        lead_out=camlib.Lead(0.5, 1),
        pierce_clearance_mm=0.5,
        pierce_delay_s=0.2,
        power_percent=100,
        tool_number=1,
        length_register=1,
    )


def nest(**options):
    values = dict(part_spacing_mm=1, edge_margin_mm=3, resolution_mm=1)
    values.update(options)
    parts = values.pop(
        "parts",
        (camlib.NestPart(PLATE, 2, (0, 90)), camlib.NestPart(WASHER, 5, (0,))),
    )
    return camlib.Nest.for_cut(cut(), values.pop("sheet", SHEET), parts, **values)


class NestingTest(unittest.TestCase):
    def test_for_cut_reads_the_cut_and_the_nest_is_proven_and_repeatable(self):
        n = nest()
        self.assertEqual((n.kerf_mm, n.lead_in, n.pierce_clearance_mm), (1.0, camlib.Lead(1, 1), 0.5))
        result = n.run()
        self.assertEqual(result.unplaced, ())
        self.assertEqual([(p.part, p.copy) for p in result.placements], [(0, 0), (0, 1)] + [(1, k) for k in range(5)])
        self.assertEqual(len(result.region.components), 7)
        # Lead reach: the lead-in's hypot(1, 2) plus the pierce clearance.
        reach = math.hypot(1, 2) + 0.5
        self.assertAlmostEqual(result.lead_reach_mm, reach, places=12)
        check = result.check
        self.assertEqual(check["status"], "proven")
        self.assertAlmostEqual(check["part_distance_mm"], 2 * (1 + 0.005 + reach) + 1, places=12)
        self.assertAlmostEqual(check["sheet_distance_mm"], 1 + 0.005 + reach + 3, places=12)
        self.assertGreaterEqual(check["min_part_distance_mm"], check["part_distance_mm"])
        self.assertGreaterEqual(check["min_sheet_distance_mm"], check["sheet_distance_mm"])
        # Washers went into the plates' large holes.
        self.assertGreaterEqual(check["parts_in_holes"], 1)
        plate_area = 70 * 50 - math.pi * (19**2 + 4**2)
        washer_area = math.pi * (81 - 9)
        self.assertAlmostEqual(result.utilisation, (2 * plate_area + 5 * washer_area) / (200 * 120), places=9)
        again = n.run()
        self.assertEqual(result.placements, again.placements)
        self.assertEqual(result.region, again.region)
        searched = replace(n, search_iterations=6, seed=3)
        self.assertEqual(searched.run().placements, searched.run().placements)
        # The check alone accepts the layout, and refuses it once two parts
        # are moved onto each other.
        self.assertEqual(n.verify(result.placements, result.region), check)
        first, second = result.placements[0], result.placements[1]
        moved = (first, replace(second, translation_mm=first.translation_mm), *result.placements[2:])
        components = list(result.region.components)
        components[1] = components[0]
        with self.assertRaisesRegex(ValueError, "nest check"):
            n.verify(moved, replace(result.region, components=tuple(components)))

    def test_parts_that_do_not_fit_are_refused_or_listed(self):
        many = (camlib.NestPart(PLATE, 9, (0,)),)
        with self.assertRaisesRegex(ValueError, r"do not fit on the sheet.*part 0: \d of 9.*allow_unplaced"):
            nest(parts=many).run()
        result = nest(parts=many, allow_unplaced=True).run()
        ((part, left),) = result.unplaced
        self.assertEqual(part, 0)
        self.assertEqual(len(result.placements) + left, 9)
        self.assertGreater(left, 0)

    def test_the_start_corner_and_fill_axis_choose_where_the_remnant_is_left(self):
        x0, y0, x1, y1 = self.sheet_box()
        default = nest().run()
        self.assertEqual(nest(start_corner="lower_left", fill_axis="y").run().placements, default.placements)
        for corner in ("lower_left", "lower_right", "upper_left", "upper_right"):
            for axis in ("y", "x"):
                result = nest(start_corner=corner, fill_axis=axis).run()
                self.assertEqual(result.check["status"], "proven")
                self.assertEqual(len(result.placements), len(default.placements))
                bx0, by0, bx1, by1 = result.used_bounds_mm
                # The parts hug the two edges of the start corner.
                near_x = bx0 - x0 if "left" in corner else x1 - bx1
                far_x = x1 - bx1 if "left" in corner else bx0 - x0
                near_y = by0 - y0 if "lower" in corner else y1 - by1
                far_y = y1 - by1 if "lower" in corner else by0 - y0
                self.assertLess(near_x, far_x + 1e-9, (corner, axis))
                self.assertLess(near_y, far_y + 1e-9, (corner, axis))
        with self.assertRaisesRegex(ValueError, "start_corner|unknown variant"):
            nest(start_corner="middle").run()
        with self.assertRaisesRegex(ValueError, "fill_axis|unknown variant"):
            nest(fill_axis="z").run()

    def sheet_box(self):
        xs = [v[0] for v in SHEET.components[0].outer.vertices]
        ys = [v[1] for v in SHEET.components[0].outer.vertices]
        return min(xs), min(ys), max(xs), max(ys)

    def test_priority_parts_go_first_and_a_filler_takes_what_is_left(self):
        small = one(rect(0, 0, 20, 12))
        parts = (camlib.NestPart(PLATE, 1, (0,)), camlib.NestPart(small, 2, (0,), priority=1))
        result = nest(parts=parts).run()
        corner = {p.part: math.hypot(*p.translation_mm) for p in result.placements}
        self.assertLess(corner[1], corner[0])
        many = (camlib.NestPart(PLATE, 2, (0,)), camlib.NestPart(small, 400, (0, 90), fill=True))
        result = nest(parts=many).run()
        self.assertEqual(result.unplaced, ())
        ((part, copies),) = result.filled
        self.assertEqual(part, 1)
        self.assertTrue(0 < copies < 400)
        self.assertEqual(sum(p.part == 1 for p in result.placements), copies)
        self.assertEqual(result.check["status"], "proven")

    def test_parts_that_overflow_one_sheet_go_on_the_next(self):
        many = (camlib.NestPart(PLATE, 5, (0, 90)), camlib.NestPart(WASHER, 6, (0,)))
        with self.assertRaisesRegex(ValueError, "do not fit on the sheet"):
            nest(parts=many).run()
        sheets = (SHEET, SHEET, SHEET, SHEET)
        used, left = nest(parts=many).run_sheets(sheets)
        self.assertEqual(left, ())
        self.assertEqual([index for index, _ in used], list(range(len(used))))
        self.assertGreater(len(used), 1)
        counts = [0, 0]
        for _, result in used:
            self.assertEqual(result.check["status"], "proven")
            for p in result.placements:
                counts[p.part] += 1
        self.assertEqual(counts, [5, 6])
        with self.assertRaisesRegex(ValueError, r"left over after 1 sheet\(s\).*part 0"):
            nest(parts=many).run_sheets(sheets[:1])
        used, left = nest(parts=many, allow_unplaced=True).run_sheets(sheets[:1])
        self.assertEqual(len(used), 1)
        self.assertEqual(left[0][0], 0)

    def test_inputs_are_declared_not_assumed(self):
        with self.assertRaises(TypeError):
            camlib.NestPart(PLATE)  # no default quantity or rotations
        with self.assertRaises(TypeError):
            camlib.Nest.for_cut(cut(), SHEET, ())  # no default spacing, margin or resolution
        with self.assertRaisesRegex(ValueError, "rotations_deg"):
            nest(parts=(camlib.NestPart(PLATE, 1, ()),)).run()
        with self.assertRaisesRegex(ValueError, "one outline"):
            both = PlanarRegion("two", PLATE.components + (Component(circle((200, 200), 5)),), 0)
            nest(parts=(camlib.NestPart(both, 1, (0,)),)).run()

    def test_a_nest_runs_as_a_thermal_cut_job_with_every_move_clear(self):
        result = nest().run()
        op = replace(cut(), parts=result.region)
        setup = camlib.Setup(
            "sheet",
            stock=camlib.StockSpec((0, 0, 0), (200, 120, 3), 0.25),
            tools=(op.tool,),
            operations=(camlib.OperationNode(op, sources=()),),
            initial_position_mm=(0, 0, 40),
            verification=camlib.StreamChecks(stock_contact=camlib.StockContactOptions()),
        )
        statuses, report = set(), None
        with camlib.Job("nested", "1", (setup,)).regenerate() as run:
            for u in run:
                v = u.update
                if isinstance(v, VerificationUpdate):
                    statuses.add(v.status)
                elif isinstance(v, OperationUpdate) and v.stage == "end":
                    report = v.report
        self.assertEqual(statuses, {"clear"})
        self.assertEqual(report["parts"], 7)
        # Two loops per washer, three per plate.
        self.assertEqual(report["loops"], 2 * 3 + 5 * 2)


def flat(boundary):
    """A boundary's points with its arcs as short chords (for a point test)."""
    points, v = [], boundary.vertices
    for (x0, y0, bulge), (x1, y1, _) in zip(v, v[1:] + v[:1]):
        points.append((x0, y0))
        if bulge:
            sweep = 4 * math.atan(bulge)
            chord = math.hypot(x1 - x0, y1 - y0)
            radius = chord / (2 * math.sin(abs(sweep) / 2))
            offset = radius * math.cos(sweep / 2) * (1 if sweep > 0 else -1)
            cx = (x0 + x1) / 2 - (y1 - y0) / chord * offset
            cy = (y0 + y1) / 2 + (x1 - x0) / chord * offset
            start = math.atan2(y0 - cy, x0 - cx)
            for k in range(1, 24):
                points.append(
                    (cx + radius * math.cos(start + sweep * k / 24), cy + radius * math.sin(start + sweep * k / 24))
                )
    return points


def within(p, points):
    inside = False
    for a, b in zip(points, points[1:] + points[:1]):
        if (a[1] > p[1]) != (b[1] > p[1]) and p[0] < a[0] + (p[1] - a[1]) * (b[0] - a[0]) / (b[1] - a[1]):
            inside = not inside
    return inside


def in_region(p, region):
    return any(within(p, flat(c.outer)) and not any(within(p, flat(h)) for h in c.holes) for c in region.components)


class RemnantTest(unittest.TestCase):
    def test_the_remnant_is_the_sheet_less_everything_the_cut_can_reach(self):
        planned = nest(start_corner="lower_left")
        result = planned.run()
        with self.assertRaises(TypeError):
            planned.remnant(result)  # no default margin
        with self.assertRaisesRegex(ValueError, "remnant_margin_mm -1"):
            planned.remnant(result, remnant_margin_mm=-1)
        left = planned.remnant(result, remnant_margin_mm=2)
        self.assertEqual(left.check["status"], "checked_at_points")
        self.assertGreaterEqual(left.check["min_part_distance_mm"], left.cut_reach_mm + 2 - 1e-3)
        self.assertAlmostEqual(left.cut_reach_mm, result.lead_reach_mm + planned.kerf_mm + 0.005, places=9)
        # Less than the sheet minus the parts' solid area; more than nothing.
        self.assertLess(left.area_mm2, left.sheet_area_mm2 - result.placed_area_mm2)
        self.assertGreater(left.area_mm2, 0.2 * left.sheet_area_mm2)
        self.assertEqual(len(left.pieces), len(left.region.components))
        self.assertEqual(left.region.source["remnant_of"], SHEET.name)
        # A larger margin leaves less.
        self.assertLess(planned.remnant(result, remnant_margin_mm=6).area_mm2, left.area_mm2)
        # The same from the cut's own parts region.
        self.assertEqual(planned.remnant(result.region, remnant_margin_mm=2).area_mm2, left.area_mm2)
        # Nothing cut: the whole sheet.
        nothing = planned.remnant(PlanarRegion("none", ()), remnant_margin_mm=2)
        self.assertEqual(nothing.area_mm2, nothing.sheet_area_mm2)

    def test_no_cell_the_cut_removed_lies_in_the_remnant(self):
        # Independent of the remnant's construction: cut the nest on the
        # simulated stock, then every cell the torch cut through or into
        # must lie outside the remnant (margin 0: the tightest claim).
        from camlib.simulation import Simulation

        planned = nest()
        result = planned.run()
        left = planned.remnant(result, remnant_margin_mm=0)
        op = replace(cut(), parts=result.region)
        with Simulation(
            stock=camlib.StockSpec((0, 0, 0), (200, 120, 3), 0.25),
            tools=(op.tool,),
            operations=(op,),
            revision="1",
            initial_position_mm=(0, 0, 40),
        ) as sim:
            for _ in sim.run_operations():
                pass
            s = sim.state
            self.assertTrue(s.finished)
            cut_cells = [
                ((i + 0.5) * s.dx, (j + 0.5) * s.dy)
                for j in range(s.ny)
                for i in range(s.nx)
                if s.heights[2 * (j * s.nx + i)] < 3 - 1e-9
            ]
        self.assertGreater(len(cut_cells), 500)
        polygons = [(flat(c.outer), [flat(h) for h in c.holes]) for c in left.region.components]
        inside = [
            p
            for p in cut_cells
            if any(within(p, outer) and not any(within(p, h) for h in holes) for outer, holes in polygons)
        ]
        self.assertEqual(inside, [])

    def test_a_remnant_saved_as_dxf_is_the_sheet_of_the_next_nest(self):
        import tempfile
        from pathlib import Path

        from camlib.dxf import read_parts, write_regions

        planned = nest(start_corner="lower_left", fill_axis="y")
        first = planned.run()
        left = planned.remnant(first, remnant_margin_mm=2)
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "remnant.dxf"
            write_regions(left.pieces[0], path)
            (sheet,) = read_parts(path, unit_mm=1, join_tolerance_mm=1e-6)
        washers = (camlib.NestPart(WASHER, 3, (0,)),)
        second = nest(sheet=sheet, parts=washers, start_corner="upper_right").run()
        self.assertEqual(second.check["status"], "proven")
        self.assertEqual(len(second.placements), 3)
        # The new parts lie in the remnant, clear of what the first cut removed.
        for component in second.region.components:
            for x, y, _ in component.outer.vertices:
                self.assertTrue(in_region((x, y), left.region), (x, y))
                for old in first.region.components:
                    self.assertFalse(within((x, y), flat(old.outer)))


if __name__ == "__main__":
    unittest.main()
