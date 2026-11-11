from dataclasses import replace
import importlib.util
import math
from pathlib import Path
import unittest

import camlib
from camlib.simulation import OperationUpdate, Simulation, SimulationError

ROOT = Path(__file__).resolve().parents[2]


def example():
    spec = importlib.util.spec_from_file_location("standalone_trace", ROOT / "examples/standalone_job.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def trace(tool, **overrides):
    paths = (
        camlib.TracePath(
            (4, 4),
            (
                camlib.TraceLine((14, 4)),
                camlib.TraceLine((14, 12)),
                camlib.TraceLine((4, 12)),
                camlib.TraceLine((4, 4)),
            ),
        ),
        camlib.TracePath((17, 4), (camlib.TraceLine((20, 4)), camlib.TraceArc((20, 7), -180))),
    )
    values = dict(top_z_mm=6, depth_z_mm=5, stepdown_mm=0.5, clearance_z_mm=8, tool_number=7, length_register=9)
    values.update(overrides)
    return camlib.Trace(
        "engrave",
        tool,
        paths,
        **{
            'feed_mm_min': 600,
            'plunge_mm_min': 150,
            'spindle_rpm': 12000,
            'tool_number': 1,
            'length_register': 1,
            'coolant': "flood",
            **values,
        },
    )


class TraceTest(unittest.TestCase):
    def test_trace_simulates_checks_and_posts_native_arcs(self):
        ex = example()
        tool = ex.end_mill_assembly("mill4", 4)
        setup = camlib.Setup(
            "top",
            stock=camlib.StockSpec((0, 0, 0), (24, 16, 6), 0.5),
            tools=(tool,),
            operations=(camlib.OperationNode(trace(tool, retract_z_mm=7, feed_z_mm=6.5), sources=()),),
            initial_position_mm=(0, 0, 8),
            verification=camlib.StreamChecks(stock_contact=camlib.StockContactOptions()),
        )
        job = camlib.Job("trace", "1", (setup,))
        with job.post("top", machine=ex.machine_for(job)) as posted:
            posted.wait()
            player = camlib.Player(posted)
            player.seek(camlib.Cursor(posted.status()["available"]))
            self.assertGreater(player.state.volume()["removed_upper_mm3"], 0)
            nc, start = [], 1
            while page := posted.nc_lines(start, limit=256):
                nc.extend(line["text"] for line in page)
                start = page[-1]["number"] + 1
        text = "\n".join(nc)
        self.assertIn("G2", text)  # the clockwise half circle stays a native arc
        self.assertIn("Z5", text)  # final depth
        self.assertTrue(
            all(
                line.startswith(
                    (
                        "G0",
                        "G1",
                        "G2",
                        "G3",
                        "G17",
                        "G21",
                        "G90",
                        "G91",
                        "G94",
                        "G97",
                        "G40",
                        "G49",
                        "G80",
                        "G92",
                        "G61",
                        "G54",
                        "G43",
                        "T",
                        "M",
                    )
                )
                for line in nc
                if line.strip()
            )
        )

    def test_invalid_trace_is_refused_before_motion(self):
        ex = example()
        tool = ex.end_mill_assembly("mill4", 4)
        config = dict(
            stock=camlib.StockSpec((0, 0, 0), (24, 16, 6), 0.5),
            tools=(tool,),
            operations=(trace(tool, depth_z_mm=6),),
            revision="1",
            initial_position_mm=(0, 0, 8),
        )
        with Simulation(**config) as sim:
            with self.assertRaisesRegex(SimulationError, "invalid trace parameters: depth_z_mm"):
                list(sim.run_operations())
            self.assertEqual(sim.state.completed, 0)


def step_chain():
    """East, a left turn north, a clockwise quarter arc to east, then east."""
    return camlib.TracePath(
        (4, 5),
        (
            camlib.TraceLine((12, 5)),
            camlib.TraceLine((12, 9)),
            camlib.TraceArc((14, 9), -90),
            camlib.TraceLine((20, 11)),
        ),
    )


def chain_distance(path, p):
    """Distance from p to the path, computed here from the path's definition
    (not by the engine)."""
    at, best = tuple(path.start_mm), math.inf
    for segment in path.segments:
        if isinstance(segment, camlib.TraceLine):
            end = tuple(segment.end_mm)
            dx, dy = end[0] - at[0], end[1] - at[1]
            u = max(0.0, min(1.0, ((p[0] - at[0]) * dx + (p[1] - at[1]) * dy) / (dx * dx + dy * dy)))
            best = min(best, math.dist(p, (at[0] + u * dx, at[1] + u * dy)))
        else:
            c, sweep = segment.center_mm, math.radians(segment.sweep_deg)
            radius = math.dist(at, c)
            start = math.atan2(at[1] - c[1], at[0] - c[0])
            end = (c[0] + radius * math.cos(start + sweep), c[1] + radius * math.sin(start + sweep))
            turn = (math.atan2(p[1] - c[1], p[0] - c[0]) - start) * math.copysign(1, sweep) % math.tau
            if turn <= abs(sweep):
                best = min(best, abs(math.dist(p, c) - radius))
            else:
                best = min(best, math.dist(p, at), math.dist(p, end))
        at = end
    return best


def curve_points(curve, step=0.02):
    start, end = curve["start"], curve["end"]
    if curve["kind"] == "line":
        n = int(math.dist(start[:2], end[:2]) / step) + 2
        return [(start[0] + (end[0] - start[0]) * i / n, start[1] + (end[1] - start[1]) * i / n) for i in range(n + 1)]
    c, sweep = curve["center"], curve["sweep_rad"]
    radius = math.dist(start[:2], c[:2])
    a0 = math.atan2(start[1] - c[1], start[0] - c[0])
    n = int(radius * abs(sweep) / step) + 2
    return [
        (c[0] + radius * math.cos(a0 + sweep * i / n), c[1] + radius * math.sin(a0 + sweep * i / n))
        for i in range(n + 1)
    ]


class TraceSideOffsetTest(unittest.TestCase):
    def setup(self, tool, side_offset, **checks):
        op = replace(trace(tool), paths=(step_chain(),), side_offset=side_offset)
        return dict(
            stock=camlib.StockSpec((0, 0, 0), (24, 16, 6), 0.5),
            tools=(tool,),
            operations=(op,),
            initial_position_mm=(0, 0, 8),
            **checks,
        )

    def test_default_serialisation_is_unchanged(self):
        tool = example().end_mill_assembly("mill4", 4)
        self.assertNotIn("side_offset", trace(tool).to_dict())
        self.assertEqual(
            replace(trace(tool), side_offset=camlib.TraceSideOffset("left", 0.5)).to_dict()["side_offset"],
            {"side": "left", "radial_allowance_mm": 0.5},
        )
        with self.assertRaisesRegex(ValueError, "side must be 'left' or 'right'"):
            camlib.TraceSideOffset("inside")

    def test_cutter_stays_beside_the_open_chain_and_cuts_to_it(self):
        tool = example().end_mill_assembly("mill4", 4)
        offset, chain = 2 + 0.5, step_chain()
        cuts, report = [], None
        with Simulation(**self.setup(tool, camlib.TraceSideOffset("left", 0.5)), revision="1") as sim:
            for update in sim.run_operations():
                if isinstance(update, OperationUpdate):
                    if update.stage == "end":
                        report = update.report
                elif update.motion["kind"] == "cut":
                    curve = update.motion["curve"]
                    if curve["start"][2] == curve["end"][2]:
                        cuts.append(curve)
            self.assertGreater(sim.state.volume()["removed_upper_mm3"], 0)
        self.assertEqual((report["side"], report["side_offset_mm"]), ("left", offset))
        self.assertEqual({c["start"][2] for c in cuts}, {5.5, 5.0})
        final = [c for c in cuts if c["start"][2] == 5.0]
        # Starts and ends on the normals through the chain's ends, on the left.
        self.assertAlmostEqual(math.dist(final[0]["start"][:2], (4, 5 + offset)), 0, places=9)
        self.assertAlmostEqual(math.dist(final[-1]["end"][:2], (20, 11 + offset)), 0, places=9)
        self.assertEqual([c["kind"] for c in final], ["line", "line", "arc", "line"])
        points = [p for c in cuts for p in curve_points(c)]
        for p in points:
            # Never closer than the offset (no gouge), never farther (cuts to the edge).
            self.assertAlmostEqual(chain_distance(chain, p), offset, delta=1e-6)
        # Beside each edge, clear of the inside corner at (12, 5) the cutter cannot enter.
        for target in ((8, 5), (12, 8.5), (17, 11)):
            self.assertLessEqual(min(math.dist(p, target) for p in points), offset + 1e-3)

    def test_side_offset_posts_through_the_checks(self):
        ex = example()
        tool = ex.end_mill_assembly("mill4", 4)
        config = self.setup(
            tool,
            camlib.TraceSideOffset("left", 0.5),
            verification=camlib.StreamChecks(stock_contact=camlib.StockContactOptions()),
        )
        operations = tuple(camlib.OperationNode(op, sources=()) for op in config.pop("operations"))
        job = camlib.Job("trace-side", "1", (camlib.Setup("top", operations=operations, **config),))
        with job.post("top", machine=ex.machine_for(job)) as posted:
            posted.wait()
            nc, start = [], 1
            while page := posted.nc_lines(start, limit=256):
                nc.extend(line["text"] for line in page)
                start = page[-1]["number"] + 1
        text = "\n".join(nc)
        self.assertIn("G2", text)  # the offset arc (radius 2 + 2.5) stays a native arc
        self.assertIn("Y7.5", text)  # the first edge, offset 2.5 to the left

    def test_unreachable_side_is_refused_before_motion(self):
        tool = example().end_mill_assembly("mill4", 4)
        # To the right the arc of radius 2 would need a radius of 2 - 2.5.
        with Simulation(**self.setup(tool, camlib.TraceSideOffset("right", 0.5)), revision="1") as sim:
            with self.assertRaisesRegex(SimulationError, "path 0, segment 2: the arc of radius 2.*smaller tool"):
                list(sim.run_operations())
            self.assertEqual(sim.state.completed, 0)


if __name__ == "__main__":
    unittest.main()
