"""Holding tabs at given positions on a contour (camlib.Tabs(positions_mm=...)).

Checked from the emitted motions and from the simulated stock: the tool rides
at the tab top over each given point and nowhere else, and material of the tab
height stands there and nowhere else along the cut."""

import math
import unittest

import camlib
from camlib.planar import Boundary, Component, PlanarRegion
from camlib.simulation import OperationUpdate, Simulation, SimulationError, StockSpec

# A 20 mm square part (10..30) with a 10 mm square hole, cut through 10 mm
# stock to Z0 with a 6 mm tool in 2 mm steps. Tabs 3 mm wide and 3 mm high
# stand to Z3: the last two passes (Z2 and Z0) rise over them.
RADIUS, WIDTH, TOP = 3.0, 3.0, 3.0
HALF = WIDTH / 2 + RADIUS  # level travel either side of a tab center
TIGHT = 0.01  # stock height bounds carry the sweep's chord error


def config(tabs, boundaries="all"):
    tool = camlib.ToolAssembly(
        "em6",
        camlib.EndMill("e6", diameter=6, flute_length=20, overall_length=60),
        35,
        60,
        (camlib.HolderSection(35, 60, 8, 8),),
    )
    part = PlanarRegion(
        "part",
        (
            Component(
                Boundary.polygon(((10, 10), (30, 10), (30, 30), (10, 30))),
                (Boundary.polygon(((15, 15), (15, 25), (25, 25), (25, 15))),),
            ),
        ),
        1e-4,
    )
    op = camlib.Contour(
        "walls",
        tool,
        part,
        0,
        2,
        14,
        boundaries=boundaries,
        feed_mm_min=600,
        plunge_mm_min=150,
        spindle_rpm=12000,
        tool_number=1,
        length_register=1,
        coolant="flood",
        tabs=tabs,
    )
    return dict(
        stock=StockSpec((0, 0, 0), (40, 40, 10), 0.2),
        tools=(tool,),
        operations=(op,),
        revision="1",
        initial_position_mm=(0, 0, 14),
    )


def run(tabs, boundaries="all"):
    """(motions, report, {cell center: (lower, upper)}) of the finished run."""
    motions, report = [], None
    with Simulation(**config(tabs, boundaries)) as sim:
        for update in sim.run_operations():
            if isinstance(update, OperationUpdate):
                if update.stage == "end":
                    report = update.report
            else:
                motions.append(update.motion)
        state = sim.state
        cells = {
            (state.min_mm[0] + (x + 0.5) * state.dx, state.min_mm[1] + (y + 0.5) * state.dy): (
                state.heights[2 * (y * state.nx + x)],
                state.heights[2 * (y * state.nx + x) + 1],
            )
            for y in range(state.ny)
            for x in range(state.nx)
        }
    return motions, report, cells


def outside_square(p, lo, hi):
    """Distance from p to the square lo..hi, 0 inside it."""
    return math.hypot(*(max(lo - c, 0, c - hi) for c in p))


class TabsAtPositionsTest(unittest.TestCase):
    def test_tabs_stand_at_the_given_points_and_nowhere_else(self):
        # Two edge points (one 0.005 mm off the edge, inside the 0.01 mm
        # tolerance) and a corner. The tool center runs 3 mm outside the part
        # (plus camlib's 0.005 mm boundary margin): the tab centers on that path.
        points = ((22, 9.995), (30, 21), (10, 30))
        centers = ((22, 7), (33, 21), (10 - 3 / math.sqrt(2), 30 + 3 / math.sqrt(2)))
        tabs = camlib.Tabs(width_mm=WIDTH, height_mm=TOP, shape="rectangle", positions_mm=points)
        self.assertEqual(tabs.to_dict()["positions_mm"], [[22.0, 9.995], [30.0, 21.0], [10.0, 30.0]])
        self.assertEqual(tabs.to_dict()["position_tolerance_mm"], 0.01)
        self.assertNotIn("count", tabs.to_dict())
        motions, report, cells = run(tabs)

        # Motions: level travel at the tab top happens only within HALF of a
        # tab center, and reaches HALF either side of the straight-edge tabs.
        level = [
            m["curve"]
            for m in motions
            if m["kind"] != "rapid" and m["curve"]["start"][2] == TOP == m["curve"]["end"][2]
        ]
        self.assertTrue(level)
        ends = [tuple(c[k][:2]) for c in level for k in ("start", "end")]
        for p in ends:
            self.assertLessEqual(min(math.dist(p, c) for c in centers), HALF + 0.01, p)
        for c in centers[:2]:
            reach = [math.dist(p, c) for p in ends if math.dist(p, c) <= HALF + 0.01]
            # Both ends, on each of the two passes.
            self.assertEqual(sum(abs(r - HALF) < 0.01 for r in reach), 4, (c, reach))
        # Below the tab top the tool never comes over a straight-edge tab
        # (outer loop only: the hole's loop is cut without tabs, see below).
        for m in motions:
            for k in ("start", "end"):
                p = m["curve"][k]
                if p[2] < TOP - 1e-9 and outside_square(p[:2], 10, 30) > 1:
                    self.assertGreaterEqual(min(math.dist(p[:2], c) for c in centers[:2]), HALF - 0.01, p)

        # Stock, along the middle of the outer cut: the full tab height at
        # each tab, the floor everywhere farther than a tab can reach.
        standing = cut = 0
        for p, (lower, upper) in cells.items():
            if abs(outside_square(p, 10, 30) - 3) > 0.3:
                continue
            self.assertLessEqual(upper, TOP + TIGHT, p)
            near = min(math.dist(p, c) for c in centers)
            if near < WIDTH / 2 - 0.4:
                self.assertGreaterEqual(lower, TOP - TIGHT, p)
                standing += 1
            elif near > HALF + RADIUS + 0.4:
                self.assertLessEqual(upper, TIGHT, p)
                cut += 1
        self.assertGreaterEqual(standing, 3 * 20)
        self.assertGreater(cut, 500)

        # No point is on the hole: its loop gets no tab (the slug is cut
        # free), and the report says so.
        said = [d for d in report["diagnostics"] if "no tab position" in d]
        self.assertEqual(len(said), 1, report["diagnostics"])
        self.assertIn("hole 0 of component 0", said[0])
        for p, (_, upper) in cells.items():
            # Everything the 6 mm tool reaches from its loop (18..22) inside the hole.
            if outside_square(p, 18, 22) < 2.5:
                self.assertLessEqual(upper, TIGHT, p)

    def test_positions_that_cannot_be_placed_are_refused(self):
        def refused(points, pattern):
            tabs = camlib.Tabs(width_mm=WIDTH, height_mm=TOP, shape="rectangle", positions_mm=points)
            with self.assertRaisesRegex(SimulationError, pattern):
                run(tabs)

        # 1 mm off the part boundary: named with its distance, never moved.
        refused(((20, 9),), r"\(20, 9\) is not on a selected part boundary.*1\.0000 mm away")
        # Within a larger stated tolerance the same point is accepted.
        motions, _, _ = run(
            camlib.Tabs(
                width_mm=WIDTH, height_mm=TOP, shape="rectangle", positions_mm=((20, 9),), position_tolerance_mm=1.5
            )
        )
        self.assertTrue(any(m["curve"]["start"][2] == TOP == m["curve"]["end"][2] for m in motions))
        # Two tabs 4 mm apart overlap (3 mm wide plus the 6 mm tool).
        refused(((20, 10), (24, 10)), "overlap")

    def test_positions_exclude_count_and_spacing_and_defaults_are_unchanged(self):
        with self.assertRaisesRegex(ValueError, "not both"):
            camlib.Tabs(3, 2, count=4, positions_mm=((20, 10),))
        with self.assertRaisesRegex(ValueError, "not both"):
            camlib.Tabs(3, 2, spacing_mm=10, positions_mm=((20, 10),))
        with self.assertRaisesRegex(ValueError, "non-empty"):
            camlib.Tabs(3, 2, positions_mm=())
        with self.assertRaisesRegex(ValueError, "either count or spacing_mm"):
            camlib.Tabs(3, 2, count=4, spacing_mm=10)
        self.assertEqual(camlib.Tabs(3, 2).count, 4)
        self.assertEqual(camlib.Tabs(3, 2).to_dict(), {"width_mm": 3, "height_mm": 2, "shape": "triangle", "count": 4})
        self.assertEqual(
            camlib.Tabs(3, 2, spacing_mm=10).to_dict(),
            {"width_mm": 3, "height_mm": 2, "shape": "triangle", "spacing_mm": 10},
        )


if __name__ == "__main__":
    unittest.main()
