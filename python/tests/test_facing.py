from dataclasses import replace
import importlib.util
from pathlib import Path
import unittest

from camlib import Engine
from camlib.operations import HelicalEntry, TrochoidalSlot
from camlib.playback import Cursor, Player, RecordedRun
from camlib.simulation import OperationUpdate, Simulation, SimulationError
from test_simulation import tool

ROOT = Path(__file__).resolve().parents[2]


def example():
    spec = importlib.util.spec_from_file_location("facing_example", ROOT / "python/tests/support/facing.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class FacingTest(unittest.TestCase):
    def test_face_streams_material_and_complete_typed_program_matches_real_cad_target(self):
        module = example()
        config = module.facing_job()
        events = []
        first = False
        reports = []
        with Simulation(**config) as sim:
            for update in sim.run_operations():
                if isinstance(update, OperationUpdate):
                    reports.append(update.report)
                    events.extend(update.events)
                else:
                    events.append(dict(kind="motion", motion=update.motion))
                    if update.changed_count and not first:
                        first = True
                        self.assertFalse(sim.state.finished)  # streamed: before the run's end
                        self.assertFalse(sim.state.finished)
            self.assertTrue(first and sim.state.finished)
            self.assertEqual(reports[-1]["cut_rows_emitted"], 8)
            self.assertEqual(reports[-1]["clearance_returns"], 2)
            self.assertEqual(reports[-1]["nominal_uncovered_area_mm2"], 0)
            self.assertLess(reports[-1]["center_xy_min_mm"][0], -4)
            self.assertGreater(reports[-1]["center_xy_max_mm"][0], 44)
            self.assertLessEqual(max(sim.state.heights[1::2]), 8.001)
            self.assertGreaterEqual(min(sim.state.heights[::2]), 7.999)
            volume = sim.state.volume()
            count = sim.state.completed
            target = module.cadquery_target().val()
            self.assertTrue(target.isValid())
            ideal = 40 * 24 * 10 - target.Volume()
            self.assertAlmostEqual(ideal, 1920, places=6)
            self.assertLessEqual(volume["removed_lower_mm3"], ideal)
            self.assertGreaterEqual(volume["removed_upper_mm3"], ideal)
            self.assertGreater(volume["removed_lower_mm3"], ideal * 0.99)
        program = dict(
            version=1,
            id="facing-program",
            revision="1",
            units="mm",
            tolerance_mm=1e-5,
            frame=dict(origin_mm=(0, 0, 0), x=(1, 0, 0), y=(0, 1, 0), z=(0, 0, 1)),
            initial_position_mm=(0, 0, 12),
            tools=[t.to_dict() for t in config["tools"]],
            events=events,
        )
        with Engine() as engine:
            self.assertEqual(
                engine._request("validate_program", program=program, capabilities=None)["summary"]["motions"], count
            )

    def test_automatic_axis_matches_the_cheaper_explicit_checked_route(self):
        from camlib import StockSpec

        config = example().facing_job()
        config["stock"] = StockSpec((0, 0, 0), (24, 40, 10), 0.25)
        base = config["operations"][0]

        def run(axis, rapid=None):
            op = replace(base, axis=axis, auto_rapid_mm_min=rapid)
            cuts, reports = [], []
            with Simulation(**dict(config, operations=(op,))) as sim:
                for update in sim.run_operations():
                    if isinstance(update, OperationUpdate):
                        reports.append(update.report)
                    else:
                        cuts.append(update.motion)
                self.assertTrue(sim.state.finished)
                self.assertLessEqual(max(sim.state.heights[1::2]), 8.001)
                self.assertGreaterEqual(min(sim.state.heights[::2]), 7.999)
            return cuts, reports[-1]

        x, x_report = run("x")
        y, y_report = run("y")
        auto, report = run("auto", 6000.0)
        self.assertLess(len(y), len(x))
        self.assertEqual(auto, y)
        self.assertEqual(report["axis"], "y")
        self.assertLess(report["auto_axis_seconds"][1], report["auto_axis_seconds"][0])
        self.assertNotIn("auto_axis_seconds", x_report)
        self.assertEqual(y_report["nominal_uncovered_area_mm2"], 0)

    def test_automatic_axis_reports_unavailable_overtravel_and_keeps_the_y_stream(self):
        from camlib import StockSpec

        config = example().facing_job()
        config["stock"] = StockSpec((999960, 0, 0), (999999, 24, 10), 0.5)
        config["initial_position_mm"] = (999950, 0, 12)

        def run(axis, rapid=None):
            op = replace(config["operations"][0], axis=axis, auto_rapid_mm_min=rapid)
            moves, report = [], None
            with Simulation(**dict(config, operations=(op,))) as sim:
                for update in sim.run_operations():
                    if not isinstance(update, OperationUpdate):
                        moves.append(update.motion)
                    elif update.stage == "end":
                        report = update.report
                self.assertTrue(sim.state.finished)
            return moves, report

        with self.assertRaisesRegex(SimulationError, "coordinate overtravel"):
            run("x")
        explicit, _ = run("y")
        auto, report = run("auto", 6000.0)
        self.assertEqual(auto, explicit)
        self.assertEqual(report["axis"], "y")
        self.assertIsNone(report["auto_axis_seconds"][0])
        self.assertIsNone(report["auto_axis_refusals"][1])
        self.assertIn("coordinate overtravel", report["auto_axis_refusals"][0])

    def test_retained_facing_seek_shows_partial_removal_and_replays_same_stock(self):
        with RecordedRun(**{'rapid_mm_min': 6000, **example().facing_job()}, checkpoint_every=8) as run:
            player = Player(run)
            player.pause()
            run.wait()
            player.seek(Cursor(3, 0.5))
            partial = player.state.heights.tobytes()
            volume = player.state.volume()["removed_lower_mm3"]
            self.assertGreater(volume, 0)
            player.seek(Cursor(run.status()["available"]))
            self.assertTrue(player.state.finished)
            self.assertGreater(player.state.volume()["removed_lower_mm3"], volume)
            player.seek(Cursor(3, 0.5))
            self.assertEqual(player.state.heights.tobytes(), partial)

    def test_facing_and_slot_share_stock_and_ordered_tool_changes(self):
        config = example().facing_job()
        face = replace(config["operations"][0], floor_z_mm=9)
        slot = TrochoidalSlot(
            "slot",
            tool(),
            (8, 12),
            (20, 12),
            6,
            10,
            8,
            1,
            12,
            tool_number=2,
            length_register=2,
            feed_mm_min=600,
            entry_feed_mm_min=150,
            spindle_rpm=12000,
            coolant="flood",
            entry=HelicalEntry(3),
        )
        config.update(tools=(*config["tools"], tool()), operations=(face, slot), revision="face-then-slot")
        controls = []
        face_removed = None
        with Simulation(**config) as sim:
            for update in sim.run_operations():
                if isinstance(update, OperationUpdate):
                    controls.append((update.operation, update.stage))
                    if update.operation == face.id and update.stage == "end":
                        face_removed = sim.state.volume()["removed_lower_mm3"]
            self.assertEqual(controls, [("face", "begin"), ("face", "end"), ("slot", "begin"), ("slot", "end")])
            self.assertGreater(sim.state.volume()["removed_lower_mm3"], face_removed + 50)
            self.assertTrue(sim.state.finished)

    def test_non_center_cutting_facing_tool_is_allowed_and_bad_entry_is_rejected(self):
        config = example().facing_job()
        t = config["tools"][0]
        t = replace(t, cutter=replace(t.cutter, center_cutting=False))
        config.update(tools=(t,))
        config["operations"] = (replace(config["operations"][0], tool=t, axis="y", mode="one_way", reverse=True),)
        with Simulation(**config) as sim:
            list(sim.run_operations())
            self.assertTrue(sim.state.finished)
        config["operations"] = (replace(config["operations"][0], entry_air_gap_mm=0),)
        with Simulation(**config) as sim:
            with self.assertRaisesRegex(SimulationError, "facing parameters"):
                list(sim.run_operations())
            self.assertFalse(sim.state.finished)
            self.assertEqual(sim.state.completed, 0)

    def test_facing_round_clamps_stops_at_them_and_leaves_only_what_they_block(self):
        # Two clamps whose keep-out areas reach the passes along the block's
        # X- and X+ edges (#422): the rows stop at them and are linked at
        # depth, nothing enters them, every rise is straight up, and what
        # stands above the floor afterwards is only what they block.
        from test_limits import distance_to_box, points

        module = example()
        config = module.clamped_facing_job()
        moves, reports = [], []
        with Simulation(**config) as sim:
            for update in sim.run_operations():
                if isinstance(update, OperationUpdate):
                    reports.append(update.report)
                else:
                    moves.append(update.motion)
            self.assertTrue(sim.state.finished)
            heights = sim.state.heights.tolist()
        report = reports[-1]
        self.assertEqual(report["strategy"], "facing_round_limits")
        radius, top, floor = 5, module.CLAMP_TOP_Z, 9
        for m in moves:
            a, b = m["curve"]["start"], m["curve"]["end"]
            if b[2] > a[2]:
                self.assertEqual(a[:2], b[:2], m)
            for p in points(m, 0.02):
                if p[2] < top:
                    for box in module.CLAMPS:
                        self.assertGreaterEqual(distance_to_box(p, box), radius, (m, p))
        # One entry for the rows and one for each clamp's edge (a link from
        # one clamp to the other at depth would cut across the block); the
        # pocket this used to be planned as made thousands of boundary moves.
        self.assertEqual(sum(m["kind"] == "plunge" for m in moves), 3)
        self.assertLess(len(moves), 120)
        around = report["around_limits"]
        self.assertGreater(around["unreachable_area_mm2"], 0)
        self.assertEqual(around["entries"], 3)
        nx, cell = 200, 0.5
        half = cell * 2**0.5 / 2
        for i, upper in enumerate(heights[1::2]):
            if upper > floor + 1e-3:
                c = ((i % nx + 0.5) * cell, (i // nx + 0.5) * cell)
                near = min(distance_to_box(c, box) for box in module.CLAMPS)
                self.assertLessEqual(near, 0.021 + half, (c, upper))
        self.assertGreaterEqual(min(heights[::2]), floor - 1e-3)
