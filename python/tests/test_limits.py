"""Machining limits (containment and avoid areas) on every kind of toolpath.
Each check here is independent of the planner: every generated move is
sampled and the cutter disk is measured against the areas."""

import math
import unittest

import camlib
from camlib import AvoidArea, Limits
from camlib.planar import Boundary, Component, PlanarRegion
from camlib.simulation import Simulation, SimulationError, Update
from test_surface_patterns import roof

R = 3  # tool radius


def square(x0, y0, x1, y1, name="area"):
    return PlanarRegion(name, (Component(Boundary.polygon(((x0, y0), (x1, y0), (x1, y1), (x0, y1)))),), 1e-4)


def tool():
    holder = (camlib.HolderSection(35, 60, 8, 8),)
    return camlib.ToolAssembly(
        "em6", camlib.EndMill("e6", diameter=2 * R, flute_length=20, overall_length=60), 35, 60, holder
    )


def distance_to_box(p, box):
    x0, y0, x1, y1 = box
    dx = max(x0 - p[0], 0, p[0] - x1)
    dy = max(y0 - p[1], 0, p[1] - y1)
    return math.hypot(dx, dy)


def points(motion, step=0.05):
    c = motion["curve"]
    a, b = c["start"], c["end"]
    if c["kind"] == "line":
        n = max(1, int(math.dist(a, b) / step))
        return [tuple(a[k] + (b[k] - a[k]) * i / n for k in range(3)) for i in range(n + 1)]
    o, sweep = c["center"], c["sweep_rad"]
    r = math.dist(a[:2], o[:2])
    start = math.atan2(a[1] - o[1], a[0] - o[0])
    n = max(2, int(abs(sweep) * r / step))
    return [
        (
            o[0] + r * math.cos(start + sweep * i / n),
            o[1] + r * math.sin(start + sweep * i / n),
            a[2] + (b[2] - a[2]) * i / n,
        )
        for i in range(n + 1)
    ]


def run(config, operations):
    moves = []
    with Simulation(**dict(config, operations=operations)) as sim:
        for update in sim.run_operations():
            if isinstance(update, Update):
                moves.append(update.motion)
        assert sim.state.finished
    return moves


def assert_clear(test, moves, box, top, slack=1e-3):
    for m in moves:
        for p in points(m):
            if p[2] < top:
                test.assertGreaterEqual(distance_to_box(p, box), R - slack, (m["kind"], p))


def base(**values):
    t = tool()
    config = dict(
        stock=camlib.StockSpec((0, 0, 0), (40, 40, 10), 0.25), tools=(t,), revision="1", initial_position_mm=(0, 0, 20)
    )
    config.update(values)
    return t, config


def limited(op, limits):
    return dict(op.to_dict(), limits=limits.to_dict())


CLAMP = (15, 15, 25, 25)
AVOID = Limits(avoid=(AvoidArea(square(*CLAMP, name="clamp"), 12),))


class LimitsTest(unittest.TestCase):
    def test_pocket_and_facing_route_around_a_clamp(self):
        t, config = base()
        pocket = camlib.Pocket(
            "pocket",
            t,
            square(2, 2, 38, 38),
            10,
            8,
            2,
            2.5,
            20,
            feed_mm_min=600,
            plunge_mm_min=150,
            spindle_rpm=12000,
            tool_number=1,
            length_register=1,
            coolant="flood",
        )
        facing = camlib.Facing(
            "face",
            t,
            9,
            1,
            2.5,
            20,
            feed_mm_min=600,
            plunge_mm_min=150,
            spindle_rpm=12000,
            tool_number=1,
            length_register=1,
            coolant="flood",
        )
        for op in (pocket, facing):
            with self.subTest(op.id):
                moves = run(config, (limited(op, AVOID),))
                self.assertTrue(any(m["kind"] == "cut" for m in moves))
                assert_clear(self, moves, CLAMP, 12)

    def test_contour_lifts_over_a_clamp(self):
        t, config = base()
        part = square(10, 10, 30, 30, name="part")
        clamp = (5, 18, 12, 22)  # over the left wall's band
        limits = Limits(avoid=(AvoidArea(square(*clamp, name="clamp"), 12),))
        contour = camlib.Contour(
            "walls",
            t,
            part,
            6,
            2,
            20,
            lead_in_mm=2,
            lead_out_mm=2,
            feed_mm_min=600,
            plunge_mm_min=150,
            spindle_rpm=12000,
            tool_number=1,
            length_register=1,
            coolant="flood",
        )
        moves = run(config, (limited(contour, limits),))
        assert_clear(self, moves, clamp, 12)
        # The rest of the left wall is still cut at the final depth.
        self.assertTrue(
            any(
                m["kind"] == "cut" and m["curve"]["end"][2] == 6 and m["curve"]["start"][0] == m["curve"]["end"][0]
                for m in moves
            )
        )

    def test_parallel_finishing_stays_inside_the_containment(self):
        config = roof().parallel_job()
        op = config["operations"][0]
        box = (6, 4, 18, 12)
        limits = Limits(contain=square(*box, name="window"))
        # The window's rows start inside the stock, so they are entered by a
        # declared ramp (a plunge there would cut into the stock).
        entry = camlib.RampEntry(angle_deg=10, min_length_mm=1).to_dict()
        moves = run(config, (dict(limited(op, limits), entry=entry),))
        assembly = config["tools"][0]
        r = (assembly.to_dict() if hasattr(assembly, "to_dict") else assembly)["cutter"]["diameter_mm"] / 2
        cuts = [m for m in moves if m["kind"] == "cut"]
        self.assertTrue(cuts)
        for m in cuts:
            for p in points(m):
                # The whole cutter disk stays inside the window: its center
                # is at least a radius from every edge.
                self.assertTrue(
                    box[0] + r - 1e-3 <= p[0] <= box[2] - r + 1e-3 and box[1] + r - 1e-3 <= p[1] <= box[3] - r + 1e-3,
                    (p, r),
                )

    def test_waterline_stays_inside_the_containment(self):
        config = roof().parallel_job()
        parallel = config["operations"][0]
        op = camlib.Waterline(
            "walls",
            config["tools"][0],
            parallel.surface,
            0,
            7.5,
            4.5,
            1,
            12,
            feed_mm_min=600,
            plunge_mm_min=150,
            spindle_rpm=12000,
            tool_number=1,
            length_register=1,
            coolant="flood",
        )
        box = (2, 2, 14, 14)
        assembly = config["tools"][0]
        r = (assembly.to_dict() if hasattr(assembly, "to_dict") else assembly)["cutter"]["diameter_mm"] / 2
        moves = run(config, (limited(op, Limits(contain=square(*box, name="window"))),))
        cuts = [m for m in moves if m["kind"] == "cut"]
        self.assertTrue(cuts)
        for m in cuts:
            for p in points(m):
                self.assertTrue(
                    box[0] + r - 1e-3 <= p[0] <= box[2] - r + 1e-3 and box[1] + r - 1e-3 <= p[1] <= box[3] - r + 1e-3,
                    (p, r),
                )

    def test_refusals(self):
        t, config = base()
        drill = camlib.Drilling(
            "drill",
            t,
            ((20, 20),),
            5,
            20,
            feed_mm_min=150,
            retract_feed_mm_min=600,
            spindle_rpm=6000,
            tool_number=1,
            length_register=1,
            coolant="flood",
        )
        with self.assertRaisesRegex(
            SimulationError, "drilling hole 0 at \\(20.0000, 20.0000\\) reaches into an avoid area"
        ):
            run(config, (limited(drill, AVOID),))
        tall = Limits(avoid=(AvoidArea(square(*CLAMP, name="clamp"), 30),))
        pocket = camlib.Pocket(
            "pocket",
            t,
            square(2, 2, 38, 38),
            10,
            8,
            2,
            2.5,
            20,
            feed_mm_min=600,
            plunge_mm_min=150,
            spindle_rpm=12000,
            tool_number=1,
            length_register=1,
            coolant="flood",
        )
        with self.assertRaisesRegex(SimulationError, "passes through an avoid area"):
            run(config, (limited(pocket, tall),))

    def test_setup_avoid_reaches_every_operation(self):
        t, config = base()
        pocket = camlib.Pocket(
            "pocket",
            t,
            square(2, 2, 38, 38),
            10,
            8,
            2,
            2.5,
            20,
            feed_mm_min=600,
            plunge_mm_min=150,
            spindle_rpm=12000,
            tool_number=1,
            length_register=1,
            coolant="flood",
        )
        setup = camlib.Setup(
            "top",
            stock=config["stock"],
            tools=(t,),
            operations=(camlib.OperationNode(pocket),),
            initial_position_mm=(0, 0, 20),
            avoid=AVOID.avoid,
        )
        job = camlib.Job("limits", "1", (setup,))
        definition = job.compile().setup("top").config["operations"][0]
        self.assertEqual(definition["limits"]["avoid"][0]["top_z_mm"], 12)
        moves = []
        with job.regenerate() as runs:
            for u in runs:
                if isinstance(u.update, Update):
                    moves.append(u.update.motion)
        assert_clear(self, moves, CLAMP, 12)


if __name__ == "__main__":
    unittest.main()
