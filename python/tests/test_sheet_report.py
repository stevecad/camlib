"""Part labels placed by the nest, the cut report of a sheet and the crop cut
(#296 E2, E3, D3). The placed marks are compared with the placements applied
in the test, the report's lengths and times with the moves of the run itself,
and the crop cut with the moves and the remnant's outline."""

from dataclasses import replace
import math
import unittest

import camlib
from camlib.simulation import OperationUpdate, VerificationUpdate

from test_nesting import PLATE, WASHER, cut, in_region, nest

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


if __name__ == "__main__":
    unittest.main()
