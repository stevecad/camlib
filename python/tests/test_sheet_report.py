"""Part labels placed by the nest, the cut report of a sheet, the crop cut and
lead placement and corner treatment (#296 E2, E3, D3, A4, A3). The placed marks are compared with the placements applied
in the test, the report's lengths and times with the moves of the run itself,
and the crop cut with the moves and the remnant's outline."""

from dataclasses import replace
import math
import unittest

import camlib
from camlib.simulation import OperationUpdate, VerificationUpdate

from test_nesting import PLATE, SHEET, WASHER, cut, in_region, nest

MARK_WIDTH = 0.8
# On the plate: an "L" and a quarter arc, clear of both holes and not
# symmetric, so a wrong turn or a missed mirror moves them.
PLATE_MARKS = (
    camlib.TracePath((45, 40), (camlib.TraceLine((65, 40)), camlib.TraceLine((65, 30)))),
    camlib.TracePath((48, 22), (camlib.TraceArc((52, 22), -90),)),
)
WASHER_MARKS = (camlib.TracePath((5, 0), (camlib.TraceLine((7, 0)),)),)


def samples(path, n=16):
    """Points along a TracePath, n per segment (the test's own walk)."""
    points, p = [], tuple(path.start_mm)
    for segment in path.segments:
        if isinstance(segment, camlib.TraceLine):
            q = tuple(segment.end_mm)
            points += [(p[0] + (q[0] - p[0]) * k / n, p[1] + (q[1] - p[1]) * k / n) for k in range(n + 1)]
        else:
            c, sweep = segment.center_mm, math.radians(segment.sweep_deg)
            r, a = math.dist(p, c), math.atan2(p[1] - c[1], p[0] - c[0])
            points += [
                (c[0] + r * math.cos(a + sweep * k / n), c[1] + r * math.sin(a + sweep * k / n)) for k in range(n + 1)
            ]
            q = points[-1]
        p = q
    return points


def placed(point, placement):
    x, y = point
    if placement.mirrored:
        x = -x
    a = math.radians(placement.rotation_deg)
    return (
        math.cos(a) * x - math.sin(a) * y + placement.translation_mm[0],
        math.sin(a) * x + math.cos(a) * y + placement.translation_mm[1],
    )


def length(curve):
    if curve["kind"] == "line":
        return math.dist(curve["start"], curve["end"])
    return math.dist(curve["start"][:2], curve["center"][:2]) * abs(curve["sweep_rad"])


def middle(curve):
    """The point halfway along a line or an arc (the test's own)."""
    a, b = curve["start"][:2], curve["end"][:2]
    if curve["kind"] == "line":
        return ((a[0] + b[0]) / 2, (a[1] + b[1]) / 2)
    c, half = curve["center"][:2], curve["sweep_rad"] / 2
    x, y = a[0] - c[0], a[1] - c[1]
    return (c[0] + math.cos(half) * x - math.sin(half) * y, c[1] + math.sin(half) * x + math.cos(half) * y)


def labelled_nest():
    parts = (
        camlib.NestPart(PLATE, 2, (90,), marks=PLATE_MARKS),
        camlib.NestPart(WASHER, 3, (0,), marks=WASHER_MARKS),
    )
    return nest(parts=parts), parts


def run(result, **options):
    op = replace(cut(), parts=result.region, **options)
    marking = camlib.ThermalMark(
        "labels",
        op.tool,
        result.region,
        result.marks,
        4000,
        1.5,
        power_percent=15,
        mark_width_mm=MARK_WIDTH,
        tool_number=1,
        length_register=1,
    )
    setup = camlib.Setup(
        "sheet",
        stock=camlib.StockSpec((0, 0, 0), (200, 120, 3), 0.25),
        tools=(op.tool,),
        operations=((marking,) if result.marks else ()) + (camlib.OperationNode(op, sources=()),),
        initial_position_mm=(0, 0, 40),
        verification=camlib.StreamChecks(stock_contact=camlib.StockContactOptions()),
    )
    out = dict(statuses=set(), reports={}, motions=[], events=[])
    with camlib.Job("labelled", "1", (setup,)).regenerate() as r:
        for u in r:
            v = u.update
            if isinstance(v, VerificationUpdate):
                out["statuses"].add(v.status)
            elif isinstance(v, OperationUpdate):
                if v.stage == "events":
                    out["events"] += [e for e in v.events]
                if v.stage == "end":
                    out["reports"][v.operation] = v.report
            else:
                out["motions"].append(v.motion)
    return out


class PartLabelTest(unittest.TestCase):
    def test_marks_are_placed_with_every_copy_of_their_part(self):
        planned, parts = labelled_nest()
        result = planned.run()
        self.assertEqual(len(result.placements), 5)
        self.assertEqual(len(result.marks), 2 * 2 + 3 * 1)
        seen = set()
        for path, (index, mark) in zip(result.marks, result.mark_placements):
            placement = result.placements[index]
            seen.add((index, mark))
            source = parts[placement.part].marks[mark]
            for p, q in zip(samples(source), samples(path)):
                expected = placed(p, placement)
                self.assertLess(math.dist(expected, q), 1e-9)
                self.assertTrue(in_region(q, result.region), q)
        self.assertEqual(len(seen), 7)
        # The plates were turned a quarter: so were their marks.
        self.assertTrue(all(result.placements[i].rotation_deg == 90 for i, _ in result.mark_placements[:4]))
        # A nest without marks returns none.
        self.assertEqual(nest().run().marks, ())

    def test_the_placed_marks_are_marked_and_proven_inside_their_parts(self):
        result = labelled_nest()[0].run()
        out = run(result)
        self.assertEqual(out["statuses"], {"clear"})
        report = out["reports"]["labels"]
        self.assertEqual(report["containment"], "proven")
        self.assertEqual(report["parts"], [index for index, _ in result.mark_placements])
        self.assertEqual(report["paths_completed"], 7)

    def test_a_mark_that_leaves_its_part_is_refused_by_the_marking(self):
        parts = (camlib.NestPart(WASHER, 2, (0,), marks=(camlib.TracePath((5, 0), (camlib.TraceLine((12, 0)),)),)),)
        result = nest(parts=parts).run()
        with self.assertRaisesRegex(Exception, "part"):
            run(result)


class SheetReportTest(unittest.TestCase):
    def test_the_report_is_the_run(self):
        planned = labelled_nest()[0]
        result = planned.run()
        remnant = planned.remnant(result, remnant_margin_mm=2)
        out = run(result)
        nozzle = camlib.Consumable("nozzle", life_pierces=400, life_cut_mm=50_000)
        electrode = camlib.Consumable("electrode", life_pierces=300)
        report = camlib.sheet_report(
            out["reports"].values(), nest=result, remnant=remnant, rapid_mm_min=12_000, consumables=(nozzle, electrode)
        )
        motions = out["motions"]
        fed = [m for m in motions if m["kind"] != "rapid"]
        rapids = [m for m in motions if m["kind"] == "rapid"]
        self.assertAlmostEqual(report.cut_length_mm, sum(length(m["curve"]) for m in motions if m["kind"] == "cut"))
        self.assertAlmostEqual(report.mark_length_mm, sum(length(m["curve"]) for m in motions if m["kind"] == "mark"))
        self.assertAlmostEqual(
            report.rapid_length_mm,
            sum(math.dist(m["curve"]["start"][:2], m["curve"]["end"][:2]) for m in rapids),
        )
        self.assertAlmostEqual(report.feed_seconds, sum(length(m["curve"]) / m["feed_mm_min"] * 60 for m in fed))
        dwells = [e["seconds"] for e in out["events"] if e["kind"] == "dwell"]
        self.assertEqual(report.pierces, len(dwells))
        self.assertEqual(report.pierces, 2 * 3 + 3 * 2)
        self.assertAlmostEqual(report.dwell_seconds, sum(dwells))
        self.assertAlmostEqual(report.rapid_seconds, report.rapid_length_mm / 12_000 * 60)
        self.assertAlmostEqual(report.seconds, report.feed_seconds + report.dwell_seconds + report.rapid_seconds)
        self.assertEqual((report.parts, report.mark_starts), (5, 7))
        self.assertEqual(report.utilisation, result.utilisation)
        self.assertEqual(report.remnant_area_mm2, remnant.area_mm2)
        self.assertLess(report.part_area_mm2 + report.remnant_area_mm2, report.sheet_area_mm2)
        self.assertEqual(
            report.consumables,
            (
                dict(name="nozzle", pierces=12 / 400, cut_length=report.cut_length_mm / 50_000),
                dict(name="electrode", pierces=12 / 300, cut_length=None),
            ),
        )
        # Without a rapid rate there is no rapid or total time; without a
        # nest or a remnant no areas.
        bare = camlib.sheet_report(out["reports"].values())
        self.assertEqual(
            (bare.rapid_seconds, bare.seconds, bare.utilisation, bare.remnant_area_mm2), (None, None, None, None)
        )
        self.assertEqual(bare.cut_length_mm, report.cut_length_mm)

    def test_what_is_not_declared_is_refused(self):
        with self.assertRaisesRegex(ValueError, "at least one"):
            camlib.sheet_report(())
        with self.assertRaisesRegex(ValueError, "ThermalCut and ThermalMark"):
            camlib.sheet_report(({"strategy": "pocket"},))
        with self.assertRaisesRegex(ValueError, "life_pierces"):
            camlib.Consumable("nozzle")
        with self.assertRaisesRegex(ValueError, "positive"):
            camlib.Consumable("nozzle", life_cut_mm=0)


class CropCutTest(unittest.TestCase):
    def test_the_crop_cut_is_last_and_the_offcut_is_a_piece_of_the_remnant(self):
        planned = nest()
        result = planned.run()
        x = result.used_bounds_mm[2] + 12
        self.assertLess(x, 190)
        crop = camlib.CropCut(((x, 0), (x, 120)))
        out = run(result, crop=crop)
        self.assertEqual(out["statuses"], {"clear"})
        report = out["reports"]["plasma"]
        self.assertEqual(report["crop_length_mm"], 120)
        self.assertEqual(report["pierces"], report["loops"] + 1)
        cuts = [m["curve"] for m in out["motions"] if m["kind"] == "cut"]
        self.assertEqual((tuple(cuts[-1]["start"][:2]), tuple(cuts[-1]["end"][:2])), ((x, 0), (x, 120)))
        self.assertTrue(all(c["end"][0] < x - 5 for c in cuts[:-1]))
        # The offcut: the one piece of the remnant beyond the cut, the sheet's
        # full height, starting where the kerf, the pierce clearance and the
        # margin end.
        left = planned.remnant(result, remnant_margin_mm=2, crop=crop)
        beyond = [piece for piece in left.pieces if all(v[0] > x for v in piece.components[0].outer.vertices)]
        self.assertEqual(len(beyond), 1)
        xs = [v[0] for v in beyond[0].components[0].outer.vertices]
        ys = [v[1] for v in beyond[0].components[0].outer.vertices]
        reach = 0.5 + 0.5 + 2
        self.assertTrue(x + reach <= min(xs) <= x + reach + 0.1, (x, min(xs)))
        self.assertEqual((max(xs), min(ys), max(ys)), (200, 0, 120))
        # Without the crop cut the offcut is still joined to the skeleton:
        # one piece fewer.
        self.assertEqual(len(left.pieces), len(planned.remnant(result, remnant_margin_mm=2).pieces) + 1)
        # The report counts the crop cut's pierce and length.
        sheet = camlib.sheet_report(out["reports"].values())
        self.assertAlmostEqual(sheet.cut_length_mm, sum(length(c) for c in cuts))
        self.assertEqual(sheet.pierces, len([e for e in out["events"] if e["kind"] == "dwell"]))

    def test_a_crop_cut_through_a_part_is_refused(self):
        result = nest().run()
        x = (result.used_bounds_mm[0] + result.used_bounds_mm[2]) / 2
        with self.assertRaisesRegex(Exception, "crop cut"):
            run(result, crop=camlib.CropCut(((x, 0), (x, 120))))
        with self.assertRaisesRegex(Exception, "crop needs two points"):
            run(result, crop=camlib.CropCut(((190, 0),)))


class LeadPlacementTest(unittest.TestCase):
    """Leads kept off declared edges of nested parts (#296 A4)."""

    # The plate's bottom edge, in the plate's own coordinates.
    EDGE = camlib.TracePath((0, 0), (camlib.TraceLine((70, 0)),))

    def starts(self, out):
        """Where each loop starts: the first cutting move after a lead."""
        found, led = [], False
        for m in out["motions"]:
            if m["kind"] == "cut" and led:
                found.append(tuple(m["curve"]["start"][:2]))
            led = m["kind"] == "lead"
        return found

    def test_no_lead_edges_travel_with_the_nest_and_the_cut_keeps_off_them(self):
        parts = (camlib.NestPart(PLATE, 2, (0, 90), no_lead=(self.EDGE,)), camlib.NestPart(WASHER, 3, (0,)))
        result = nest(parts=parts).run()
        self.assertEqual(len(result.no_lead), 2)
        # Each placed edge is the plate's edge under its placement.
        plates = [p for p in result.placements if p.part == 0]
        for path, placement in zip(result.no_lead, plates):
            for p, q in zip(samples(self.EDGE), samples(path)):
                self.assertLess(math.dist(placed(p, placement), q), 1e-9)
        kept = [q for path in result.no_lead for q in samples(path, 280)]  # every 0.25 mm
        plain = self.starts(run(result))
        self.assertTrue(any(min(math.dist(p, q) for q in kept) < 3 for p in plain))
        rules = camlib.LeadPlacement(result.no_lead, keep_off_distance_mm=3, prefer="corner")
        out = run(result, lead_placement=rules)
        self.assertEqual(out["statuses"], {"clear"})
        starts = self.starts(out)
        self.assertEqual(len(starts), len(plain))
        # 3 mm along the edge, and the kerf path's own half millimetre out.
        self.assertTrue(all(min(math.dist(p, q) for q in kept) > 3.4 for p in starts), starts)
        self.assertGreaterEqual(out["reports"]["plasma"]["leads_moved"], 1)
        self.assertNotIn("leads_moved", run(result)["reports"]["plasma"])

    def test_a_part_with_no_edge_left_for_a_lead_is_refused(self):
        ring = camlib.TracePath((9, 0), (camlib.TraceArc((0, 0), 360),))
        result = nest(parts=(camlib.NestPart(WASHER, 1, (0,), no_lead=(ring,)),)).run()
        with self.assertRaisesRegex(Exception, "kept free of leads"):
            run(result, lead_placement=camlib.LeadPlacement(result.no_lead, keep_off_distance_mm=1))
        with self.assertRaisesRegex(ValueError, "prefer"):
            camlib.LeadPlacement((), keep_off_distance_mm=1, prefer="left").to_dict()


class CornerTest(unittest.TestCase):
    """Corner loops and corner slow-down on nested parts (#296 A3)."""

    LOOPS = camlib.CornerLoops(3, min_angle_deg=45, max_angle_deg=135)
    SLOW = camlib.CornerSlowdown(4, feed_percent=50, min_angle_deg=60)

    def looped_nest(self):
        return camlib.Nest.for_cut(
            replace(cut(), corner_loops=self.LOOPS),
            SHEET,
            (camlib.NestPart(PLATE, 1, (90,)), camlib.NestPart(WASHER, 2, (0,))),
            part_spacing_mm=1,
            edge_margin_mm=3,
            resolution_mm=1,
        )

    def test_the_nest_keeps_room_for_the_loops_and_the_cut_puts_them_in_the_scrap(self):
        planned = self.looped_nest()
        result = planned.run()
        # The loop of a square corner reaches 3 / tan(22.5 deg) from the
        # point where the kerf path's edges meet, 0.505 * sqrt(2) from the
        # drawn corner; beyond the kerf's far edge (1.01 from the part) and
        # with half a kerf of its own that is the room the nest keeps.
        offset = 0.505
        far = offset * math.sqrt(2) + 3 / math.tan(math.radians(22.5))
        self.assertGreaterEqual(result.lead_reach_mm + 1e-9, far + 0.5 - 2 * offset)
        self.assertGreater(result.lead_reach_mm, nest().run().lead_reach_mm)
        out = run(result, corner_loops=self.LOOPS)
        self.assertEqual(out["statuses"], {"clear"})
        report = out["reports"]["plasma"]
        # The plate's four corners, less one if its outline starts there.
        self.assertIn(report["corner_loops"], (3, 4))
        plain = run(result)["reports"]["plasma"]
        self.assertNotIn("corner_loops", plain)
        each = 2 * (offset + 3) + 1.5 * math.pi * 3 - offset * math.pi / 2
        self.assertAlmostEqual(report["cut_length_mm"] - plain["cut_length_mm"], report["corner_loops"] * each, 6)
        # No cut runs inside a part, every cut stays on the sheet, and the
        # plates' outlines reach just that far from their rectangles.
        plates = [p for p in result.placements if p.part == 0]
        washers = [p.translation_mm for p in result.placements if p.part == 1]

        def off_plate(q, placement):
            a = math.radians(-placement.rotation_deg)
            x, y = q[0] - placement.translation_mm[0], q[1] - placement.translation_mm[1]
            x, y = math.cos(a) * x - math.sin(a) * y, math.sin(a) * x + math.cos(a) * y
            return math.hypot(max(-x, x - 70, 0), max(-y, y - 50, 0))

        reached = 0.0
        for m in out["motions"]:
            if m["kind"] != "cut":
                continue
            for q in (m["curve"]["start"][:2], middle(m["curve"]), m["curve"]["end"][:2]):
                self.assertFalse(in_region(q, result.region), q)
                self.assertTrue(0 < q[0] < 200 and 0 < q[1] < 120, q)
                if all(math.dist(q, w) > 10 for w in washers):
                    reached = max(reached, min(off_plate(q, p) for p in plates))
        self.assertAlmostEqual(reached, far, 6)
        # What the loops cut is gone from the remnant too.
        with_loops = planned.remnant(result, remnant_margin_mm=1).area_mm2
        without = nest().remnant(result, remnant_margin_mm=1).area_mm2
        self.assertLess(with_loops, without)

    def test_the_feed_drops_round_corners_and_the_time_says_so(self):
        result = nest().run()
        plain = run(result, height_control=True)
        out = run(result, height_control=True, corner_slowdown=self.SLOW)
        self.assertEqual(out["statuses"], {"clear"})
        report = out["reports"]["plasma"]
        # Two plates, four corners each; the circles are no corners.
        self.assertEqual(report["corner_slowdowns"], 8)
        slowed = [m for m in out["motions"] if m["kind"] == "cut" and m["feed_mm_min"] == 1250]
        total = sum(length(m["curve"]) for m in slowed)
        # 4 mm either side of each corner's quarter arc; a cut stops short of
        # closing nowhere here (no negative overcut).
        self.assertAlmostEqual(total, 8 * (8 + 0.505 * math.pi / 2), 6)
        # The same path at half the feed over those stretches: the report's
        # time grows by exactly their time at full feed.
        self.assertAlmostEqual(
            report["feed_seconds"] - plain["reports"]["plasma"]["feed_seconds"], total / 2500 * 60, 6
        )
        self.assertAlmostEqual(report["cut_length_mm"], plain["reports"]["plasma"]["cut_length_mm"], 6)
        # Height control is held off over each: more switches than without.
        switches = lambda o: sum(1 for e in o["events"] if e.get("kind") == "height_control")  # noqa: E731
        self.assertGreater(switches(out), switches(plain))
        held = replace(self.SLOW, height_control_off=False)
        self.assertEqual(switches(run(result, height_control=True, corner_slowdown=held)), switches(plain))

    def test_declared_values_only(self):
        with self.assertRaises(TypeError):
            camlib.CornerLoops(3)
        with self.assertRaises(TypeError):
            camlib.CornerSlowdown(4)
        result = nest().run()
        with self.assertRaisesRegex(Exception, "max_angle_deg"):
            run(result, corner_loops=camlib.CornerLoops(3, min_angle_deg=45, max_angle_deg=180))
        with self.assertRaisesRegex(Exception, "feed_percent"):
            run(result, corner_slowdown=replace(self.SLOW, feed_percent=0))


if __name__ == "__main__":
    unittest.main()
