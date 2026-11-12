"""The contact-angle measure against cases with a closed-form answer:
python -m unittest discover -s bench -p test_*.py."""

import math
import unittest

import numpy as np

import engagement

R = 5.0
# A 100 x 100 mm box of stock, top at Z10, in 0.5 mm cells.
STOCK = (0.0, 0.0, 0.5, 0.5, 200, 200)
HEIGHTS = np.full(200 * 200 * 2, 10.0)


def line(start, end, kind="cut"):
    return dict(id=0, kind=kind, curve=dict(kind="line", start=list(start), end=list(end)))


def arc(start, centre, sweep, kind="cut", end_z=None):
    radius = math.hypot(start[0] - centre[0], start[1] - centre[1])
    angle = math.atan2(start[1] - centre[1], start[0] - centre[0]) + sweep
    end = [centre[0] + radius * math.cos(angle), centre[1] + radius * math.sin(angle), start[2]]
    if end_z is not None:
        end[2] = end_z
    return dict(
        id=0,
        kind=kind,
        curve=dict(
            kind="arc" if end_z is None else "helix",
            plane="xy",
            start=list(start),
            end=end,
            center=[centre[0], centre[1], start[2]],
            sweep_rad=sweep,
        ),
    )


def angles(motions, index=None):
    """Each measured move's contact angles by its index (or one move's)."""
    by_index = {i: values for i, _, values in engagement.contact_angles(motions, R, STOCK, HEIGHTS)}
    return by_index if index is None else by_index[index]


class ContactAngleTest(unittest.TestCase):
    def test_a_slot_and_side_passes_read_their_textbook_angles(self):
        # A full slot is 180 degrees; a pass w beside it, acos(1 - w/r).
        motions = [line((20, 20, 5), (80, 20, 5))]
        y = 20.0
        for width in (0.5, 1.0, 3.0, 5.0):
            motions.append(line((80, y, 5), (80, y + width, 5), "link"))
            motions.append(line((80, y + width, 5), (20, y + width, 5)))
            motions.append(line((20, y + width, 5), (80, y + width, 5), "link"))
            y += width
        by_index = angles(motions)
        self.assertTrue((by_index[0] == 180).all())
        for k, width in enumerate((0.5, 1.0, 3.0, 5.0)):
            exact = math.degrees(math.acos(1 - width / R))
            measured = by_index[2 + 3 * k]
            # Away from the ends (the first cutter radius leaves the slot's
            # round end), every step reads the angle to one sample.
            body = measured[len(measured) // 4 : -len(measured) // 8]
            self.assertLessEqual(np.abs(body - exact).max(), engagement.SAMPLE_DEG, (width, exact, body))
            # A return along the pass just cut touches nothing at all: the
            # rim runs tangent to both walls, which a grid cannot decide.
            self.assertTrue((by_index[3 + 3 * k] == 0).all())

    def test_a_curved_front_reads_its_exact_angle(self):
        # A circular slot of radius 10 about (50, 50), then one lap w further
        # out: the rim meets the uncut stock outside radius 10 + r, an arc of
        # acos(((10 + r)^2 - (10 + w)^2 - r^2) / (2 (10 + w) r)) on its
        # leading half. More than the straight pass of the same width: the
        # front wraps around the cutter.
        for sweep in (math.tau, -math.tau):
            for width in (1.0, 3.0):
                outer = 10 + width
                motions = [
                    arc((60, 50, 5), (50, 50), sweep),
                    line((60, 50, 5), (50 + outer, 50, 5), "link"),
                    arc((50 + outer, 50, 5), (50, 50), sweep),
                ]
                exact = math.degrees(math.acos(((10 + R) ** 2 - outer**2 - R**2) / (2 * outer * R)))
                self.assertGreater(exact, math.degrees(math.acos(1 - width / R)))
                by_index = angles(motions)
                measured = by_index[2]
                body = measured[len(measured) // 4 : -len(measured) // 4]
                self.assertLessEqual(np.abs(body - exact).max(), engagement.SAMPLE_DEG, (width, exact))
                self.assertTrue((by_index[0][: len(by_index[0]) // 2] == 180).all())

    def test_depth_decides_what_an_earlier_move_cleared(self):
        # A slot cut at Z7 leaves the stock below it: a pass along the same
        # line at Z5 is a full slot again, and one at Z7 or above cuts nothing.
        first = line((20, 50, 7), (80, 50, 7))
        for z, expected in ((5, 180), (7, 0), (8, 0)):
            motions = [first, line((80, 50, 7), (80, 50, z), "plunge"), line((80, 50, z), (20, 50, z))]
            self.assertTrue((angles(motions, 2) == expected).all(), z)
        # A helix clears only where its tip is at or below the level.
        motions = [
            arc((60, 50, 10), (50, 50), math.tau, "lead", end_z=5),
            arc((60, 50, 5), (50, 50), math.tau),
        ]
        measured = angles(motions, 1)
        self.assertTrue((measured[: len(measured) // 2] == 180).all())
        # Above the stock, or beside it, there is nothing to touch.
        self.assertTrue((angles([line((20, 50, 10), (80, 50, 10))], 0) == 0).all())
        self.assertTrue((angles([line((-20, -20, 5), (-20, 120, 5))], 0) == 0).all())

    def test_the_summary_bands_add_up(self):
        motions = [
            line((20, 20, 5), (80, 20, 5)),
            line((80, 20, 5), (80, 23, 5), "link"),
            line((80, 23, 5), (20, 23, 5)),
            line((20, 23, 5), (80, 23, 5), "link"),
        ]
        limit = math.degrees(math.acos(1 - 3 / R))
        record = engagement.summary(motions, R, STOCK, HEIGHTS, limit)
        self.assertAlmostEqual(record["cut_mm"], 120)
        self.assertAlmostEqual(record["link_mm"], 63)
        shares = [record[k] for k in ("over_share", "within_10_share", "under_10_20_share", "under_20_share")]
        self.assertAlmostEqual(sum(shares) + record["none_share"], 1, places=3)
        # The slot is over the limit; the side pass sits on it.
        self.assertAlmostEqual(record["over_share"], 0.5, delta=0.05)
        self.assertGreater(record["within_10_share"], 0.4)
        # The 3 mm step sideways cuts; the return touches nothing.
        self.assertAlmostEqual(record["link_none_share"], 60 / 63, places=3)
        self.assertEqual(engagement.summary(motions, R, STOCK, HEIGHTS)["mean_deg"], record["mean_deg"])


class PublishedRecordTest(unittest.TestCase):
    """The record in results, the report and the comparison."""

    RECORD = dict(
        cut_mm=540.0,
        mean_deg=49.2,
        max_deg=75.0,
        none_share=0.03,
        limit_deg=75.5,
        over_share=0.0,
        within_10_share=0.3,
        under_10_20_share=0.3,
        under_20_share=0.37,
        link_mm=464.0,
        link_none_share=0.95,
        entry_feed_mm=16.0,
        entry_feed_mean_deg=103.9,
    )

    def results(self):
        import copy
        import json
        from pathlib import Path

        old = json.loads((Path(__file__).parent / "results" / "baseline.json").read_text(encoding="utf-8"))
        new = copy.deepcopy(old)
        operation = next(w for w in new["workloads"] if w["name"] == "adaptive-pocket")["operations"]["adaptive"]
        operation.update(link_mm=464.0, engagement=dict(self.RECORD))
        new["engagement_method"] = dict(sample_deg=0.5, over_deg=1.0, step_per_radius=0.1)
        return old, new

    def test_the_report_shows_engagement_only_for_results_that_record_it(self):
        import report

        old, new = self.results()
        for w in old["workloads"]:
            for o in w["operations"].values():
                o.pop("engagement", None)
                o.pop("link_mm", None)
        self.assertNotIn("## Engagement of adaptive clearing", report.render(old))
        text = report.render(new)
        self.assertIn("## Engagement of adaptive clearing", text)
        self.assertIn(
            "| 75.5 | 540 | 49.2 | 75.0 | 0.0% | 30.0% | 30.0% | 37.0% | 3.0% | 464 (95.0%) | 16 (103.9) |", text
        )
        self.assertIn("sampled every 0.5°", text)

    def test_the_comparison_skips_fields_a_result_lacks_and_lists_engagement_without_failing(self):
        import copy

        import compare

        old, new = self.results()
        for w in old["workloads"]:
            for o in w["operations"].values():
                o.pop("engagement", None)
                o.pop("link_mm", None)
        # A baseline from before these fields: nothing to compare them with.
        self.assertEqual(compare.toolpath_changes(old, new), [])
        self.assertEqual(compare.engagement_changes(old, new), [])
        # Once recorded, link length is toolpath data; engagement is listed apart.
        other = copy.deepcopy(new)
        operation = next(w for w in other["workloads"] if w["name"] == "adaptive-pocket")["operations"]["adaptive"]
        operation["link_mm"] = 470.0
        operation["engagement"]["mean_deg"] = 49.3
        self.assertEqual(
            compare.toolpath_changes(new, other), [("adaptive-pocket", "adaptive", "link_mm", 464.0, 470.0)]
        )
        self.assertEqual(
            compare.engagement_changes(new, other), [("adaptive-pocket", "adaptive", "mean_deg", 49.2, 49.3)]
        )
        self.assertEqual(compare.toolpath_changes(new, new), [])


if __name__ == "__main__":
    unittest.main()
