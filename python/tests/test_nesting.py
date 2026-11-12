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


if __name__ == "__main__":
    unittest.main()
