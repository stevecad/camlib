from dataclasses import replace
import importlib.util
import math
from pathlib import Path
import unittest
from camlib import Engine
from camlib.playback import Cursor, Player, RecordedRun
from camlib.simulation import OperationUpdate, Simulation, SimulationError, Update

ROOT = Path(__file__).resolve().parents[2]


def example():
    spec = importlib.util.spec_from_file_location("contour_example", ROOT / "python/tests/support/contour.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class ContourTest(unittest.TestCase):
    def test_real_cad_input_streams_checked_profile_and_preserves_band_scope(self):
        module = example()
        config = module.contour_job()
        op = config["operations"][0]
        self.assertEqual(op.region.source["adapter"], "analytic_horizontal_lines_circles_v1")
        events = []
        first = False
        report = None
        with Simulation(**config) as sim:
            for update in sim.run_operations():
                if isinstance(update, OperationUpdate):
                    events.extend(update.events)
                    if update.stage == "end":
                        report = update.report
                else:
                    events.append(dict(kind="motion", motion=update.motion))
                    if update.changed_count and not first:
                        first = True
                        self.assertFalse(sim.state.finished)  # streamed: before the run's end
                        self.assertFalse(sim.state.finished)
            self.assertTrue(first and sim.state.finished)
            self.assertEqual(report["cut_edges_emitted"], 16)
            self.assertEqual(report["loops_completed"], 8)
            self.assertEqual(report["geometric_residual_status"], "unknown")
            self.assertIn("not_certified", report["wall_completion"])
            self.assertEqual(report["path_safety_margin_mm"], 0.005)
            target = module.cadquery_target().val()
            self.assertTrue(target.isValid())
            ideal = 40 * 40 * 10 - target.Volume()
            self.assertAlmostEqual(ideal, 768 * math.pi, places=6)
            volume = sim.state.volume()
            self.assertLessEqual(volume["removed_lower_mm3"], ideal)
            self.assertGreaterEqual(volume["removed_upper_mm3"], ideal)
            self.assertGreater(volume["removed_lower_mm3"], ideal * 0.8)
            count = sim.state.completed
        program = dict(
            version=1,
            id="contour-program",
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

    def test_contour_retained_arc_seek_replays_exact_partial_stock(self):
        with RecordedRun(**{'rapid_mm_min': 6000, **example().contour_job()}, checkpoint_every=8) as run:
            player = Player(run)
            player.pause()
            run.wait()
            player.seek(Cursor(3, 0.5))
            partial = player.state.heights.tobytes()
            v = player.state.volume()["removed_lower_mm3"]
            self.assertGreater(v, 0)
            player.seek(Cursor(run.status()["available"]))
            self.assertTrue(player.state.finished)
            self.assertGreater(player.state.volume()["removed_lower_mm3"], v)
            player.seek(Cursor(3, 0.5))
            self.assertEqual(player.state.heights.tobytes(), partial)

    def test_retract_and_feed_heights_reach_the_engine_and_shape_links(self):
        config = example().contour_job()
        op = config["operations"][0]
        self.assertNotIn("retract_z_mm", op.to_dict())  # None keeps cache keys unchanged
        config["operations"] = (replace(op, retract_z_mm=11, feed_z_mm=10.5),)
        with Simulation(**config) as sim:
            moves = [u.motion for u in sim.run_operations() if isinstance(u, Update)]
        rapids = [m for m in moves if m["kind"] == "rapid"]

        def straight_up_to_feed_height(m):
            a, b = m["curve"]["start"], m["curve"]["end"]
            return a[:2] == b[:2] and b[2] == 10.5 > a[2]

        # Below the feed height a rapid is only a proven straight-up retract
        # out of the cut, ending at the feed height.
        self.assertTrue(
            all(
                min(m["curve"]["start"][2], m["curve"]["end"][2]) >= 10.5 or straight_up_to_feed_height(m)
                for m in rapids
            )
        )
        # The tool leaves each pass straight up to the feed height: at feed
        # through the stock outside the pass, which stands beside it (#430),
        # and at rapid from its top (#441).
        self.assertTrue(any(straight_up_to_feed_height(m) for m in moves if m["kind"] in ("rapid", "retract")))
        self.assertIn(11, [m["curve"]["end"][2] for m in rapids])
        for m, n in zip(moves, moves[1:]):
            if m["kind"] == "retract" and m["curve"]["end"][2] != 10.5:
                self.assertEqual(n["kind"], "rapid", (m, n))
                self.assertEqual(n["curve"]["start"], m["curve"]["end"], (m, n))
                self.assertTrue(straight_up_to_feed_height(n), (m, n))
        self.assertEqual(moves[-1]["curve"]["end"][2], 12)
        config["operations"] = (replace(op, feed_z_mm=10),)
        with Simulation(**config) as sim:
            with self.assertRaisesRegex(SimulationError, "stock top"):
                list(sim.run_operations())
            self.assertEqual(sim.state.completed, 0)

    def test_invalid_hole_lead_is_rejected_before_first_motion(self):
        config = example().contour_job()
        op = config["operations"][0]
        config["operations"] = (replace(op, boundaries="holes", lead_in_mm=2),)
        with Simulation(**config) as sim:
            with self.assertRaisesRegex(SimulationError, "admission"):
                list(sim.run_operations())
            self.assertEqual(sim.state.completed, 0)
            self.assertFalse(sim.state.finished)

    def test_facing_and_contour_share_stock_and_ordinary_python_tool_setup(self):
        config = example().contour_job()
        profile = replace(config["operations"][0], tool_number=2, length_register=2)
        from camlib.operations import Facing
        from camlib.tools import EndMill, HolderSection, ToolAssembly

        face_tool = ToolAssembly(
            "face8",
            EndMill("em8", diameter=8, flute_length=30, overall_length=50),
            35,
            60,
            (HolderSection(35, 60, 5, 5),),
        )
        face = Facing(
            "face",
            face_tool,
            9,
            1,
            6,
            12,
            feed_mm_min=600,
            plunge_mm_min=150,
            spindle_rpm=12000,
            tool_number=1,
            length_register=1,
            coolant="flood",
        )
        config.update(tools=(face_tool, *config["tools"]), operations=(face, profile), revision="face-profile")
        controls = []
        before = None
        with Simulation(**config) as sim:
            for update in sim.run_operations():
                if isinstance(update, OperationUpdate):
                    controls.append((update.operation, update.stage))
                    if update.operation == face.id and update.stage == "end":
                        before = sim.state.volume()["removed_lower_mm3"]
            self.assertEqual(controls, [("face", "begin"), ("face", "end"), ("profile", "begin"), ("profile", "end")])
            self.assertGreater(sim.state.volume()["removed_lower_mm3"], before + 1000)
            self.assertTrue(sim.state.finished)
