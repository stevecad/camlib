"""Morph and flow finishing: generated XY passes followed on the surface."""

import unittest

import camlib
from camlib.simulation import OperationUpdate, Simulation, Update, VerificationUpdate
from test_surface_patterns import roof


def run(config, op):
    report, moves, statuses = None, [], set()
    with Simulation(
        **dict(config, operations=(op,), verification=camlib.StreamChecks(stock_contact=camlib.StockContactOptions()))
    ) as sim:
        for update in sim.run_operations():
            if isinstance(update, OperationUpdate) and update.stage == "end":
                report = update.report
            elif isinstance(update, Update):
                moves.append(update.motion)
            elif isinstance(update, VerificationUpdate):
                statuses.add(update.status)
        state = sim.state

        def target(x):
            return 4 + min(x, 24 - x) / 3

        for y in range(state.ny):
            for x in range(state.nx):
                x0, x1 = x * state.dx, (x + 1) * state.dx
                i = y * state.nx + x
                assert state.heights[2 * i + 1] >= min(target(x0), target(x1)), (x, y)
        removed = state.volume()["removed_lower_mm3"]
    return report, moves, statuses, removed


class MorphFlowTest(unittest.TestCase):
    def test_morph_between_two_edges(self):
        config = roof().parallel_job()
        tool, surface = config["tools"][0], config["operations"][0].surface
        a = camlib.TracePath((1, 2), (camlib.TraceLine((23, 2)),))
        b = camlib.TracePath((1, 14), (camlib.TraceLine((23, 10)),))
        op = camlib.Morph(
            "morph",
            tool,
            surface,
            0,
            0.5,
            12,
            a,
            b,
            feed_mm_min=600,
            plunge_mm_min=150,
            spindle_rpm=12000,
            tool_number=1,
            length_register=1,
            coolant="flood",
        )
        report, moves, statuses, removed = run(config, op)
        self.assertEqual(report["strategy"], "morph")
        self.assertNotIn("violation", statuses)
        self.assertGreater(removed, 1)
        # Neighbouring passes at most a stepover apart: the widest gap is 12
        # at x=1, so 24 intervals.
        self.assertEqual(report["rows"], 25)

    def test_flow_paths_follow_a_cad_face(self):
        from camlib.cadquery import flow_paths

        config = roof().parallel_job()
        tool, surface = config["tools"][0], config["operations"][0].surface
        solid = roof().target()
        # The roof's left slope: from (0, y, 4) up to the ridge (12, y, 8).
        face = max(
            solid.faces().vals(), key=lambda f: f.Center().z if f.normalAt().z > 0.5 and f.Center().x < 12 else -1
        )
        paths = flow_paths(face, stepover_mm=0.6)
        self.assertGreater(len(paths), 5)
        op = camlib.Project(
            "flow",
            tool,
            surface,
            0,
            12,
            paths,
            feed_mm_min=600,
            plunge_mm_min=150,
            spindle_rpm=12000,
            tool_number=1,
            length_register=1,
            coolant="flood",
        )
        report, moves, statuses, removed = run(config, op)
        self.assertNotIn("violation", statuses)
        self.assertGreater(removed, 0.5)

    def test_flow_rows_are_not_repeated(self):
        # A torus fillet's u lines: the bisection stops a rounding error short
        # of the last line, which then came again (issue #87).
        import math

        import cadquery as cq
        from camlib.cadquery import flow_paths

        part = cq.Workplane("XY").box(80, 80, 10, centered=False)
        part = part.faces(">Z").workplane().center(40, 40).circle(25).extrude(12)
        solid = part.faces(">Z").edges().fillet(8).val()
        torus = next(f for f in solid.Faces() if f.geomType() == "TORUS")
        paths = flow_paths(torus, stepover_mm=1, along="u")
        starts = [p.start_mm for p in paths]
        self.assertEqual(len(paths), 9)
        self.assertTrue(all(math.dist(a, b) > 0.5 for a, b in zip(starts, starts[1:])), starts)
        self.assertAlmostEqual(starts[-1][0], 57, places=6)

    def test_project_passes_that_meet_but_for_rounding_link(self):
        # Project over two passes whose end and start coincide but for
        # rounding (a 1e-11 mm link, issue #87): planned, the link left out.
        config = roof().parallel_job()
        tool, surface = config["tools"][0], config["operations"][0].surface
        a = camlib.TracePath((1, 6), (camlib.TraceLine((23, 6)),))
        b = camlib.TracePath((23 + 1e-11, 6), (camlib.TraceLine((1, 8)),))
        op = camlib.Project(
            "flow",
            tool,
            surface,
            0,
            12,
            (a, b),
            feed_mm_min=600,
            plunge_mm_min=150,
            spindle_rpm=12000,
            tool_number=1,
            length_register=1,
            coolant="flood",
        )
        report, moves, statuses, removed = run(config, op)
        self.assertNotIn("violation", statuses)
        self.assertEqual(report["rows_completed"], 2)


if __name__ == "__main__":
    unittest.main()
