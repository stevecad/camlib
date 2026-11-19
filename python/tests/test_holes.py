from dataclasses import replace
import importlib.util
import itertools
import math
import os
from pathlib import Path
import unittest
from unittest import mock
from camlib import Engine
from camlib.operations import Drilling, Bore
from camlib.playback import Cursor, Player, RecordedRun
from camlib.simulation import OperationUpdate, Simulation, SimulationError

ROOT = Path(__file__).resolve().parents[2]


def example():
    spec = importlib.util.spec_from_file_location("hole_example", ROOT / "python/tests/support/holes.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class HoleOperationsTest(unittest.TestCase):
    def test_drill_point_target_cad_volume_and_depth_reference(self):
        module = example()
        config = module.hole_job()
        pilot = config["operations"][0]
        config.update(operations=(pilot,), tools=(config["tools"][0],))
        report = None
        with Simulation(**config) as sim:
            for update in sim.run_operations():
                if isinstance(update, OperationUpdate) and update.stage == "end":
                    report = update.report
            self.assertTrue(sim.state.finished)
            h = 2 / math.tan(math.radians(59))
            self.assertAlmostEqual(report["point_height_mm"], h)
            self.assertEqual(report["tip_target_z_mm"], 4)
            self.assertAlmostEqual(report["full_diameter_z_mm"], 4 + h)
            self.assertEqual(report["cut_pecks"], 6)
            target = module.cadquery_drill_target().val()
            self.assertTrue(target.isValid())
            expected = 2 * (math.pi * 4 * (6 - h) + math.pi * 4 * h / 3)
            self.assertAlmostEqual(40 * 24 * 10 - target.Volume(), expected, places=6)
            v = sim.state.volume()
            self.assertLessEqual(v["removed_lower_mm3"], expected)
            self.assertGreaterEqual(v["removed_upper_mm3"], expected)
        pilot = replace(pilot, depth_reference="full_diameter", peck_mode="chip_break", depth_z_mm=4)
        config["operations"] = (pilot,)
        with Simulation(**config) as sim:
            end = [u for u in sim.run_operations() if isinstance(u, OperationUpdate)][-1]
            self.assertAlmostEqual(end.report["tip_target_z_mm"], 4 - h)
            self.assertEqual(end.report["full_diameter_z_mm"], 4)

    def test_bore_from_solid_streams_definite_material_and_matches_cad_cylinders(self):
        module = example()
        config = module.hole_job()
        config.update(operations=(config["operations"][1],), tools=(config["tools"][1],))
        first = False
        report = None
        helices = 0
        with Simulation(**config) as sim:
            for update in sim.run_operations():
                if isinstance(update, OperationUpdate):
                    if update.stage == "end":
                        report = update.report
                else:
                    helices += update.motion["curve"]["kind"] == "helix"
                    # Surface-touch uncertainty alone is not definite removal.
                    if not first and sim.state.volume()["removed_lower_mm3"] > 0.001:
                        first = True
                        self.assertFalse(sim.state.finished)  # streamed: before the run's end
                        self.assertFalse(sim.state.finished)
            self.assertTrue(first and sim.state.finished)
            self.assertEqual(helices, 6)
            self.assertEqual(report["rings_per_layer"], 2)
            self.assertTrue(report["core_cleared_nominal"])
            self.assertAlmostEqual(report["nominal_machined_diameter_mm"], 11.99)
            target = module.cadquery_target().val()
            self.assertTrue(target.isValid())
            ideal = 2 * math.pi * 36 * 6
            self.assertAlmostEqual(40 * 24 * 10 - target.Volume(), ideal, places=6)
            v = sim.state.volume()
            self.assertLessEqual(v["removed_lower_mm3"], ideal)
            self.assertGreaterEqual(v["removed_upper_mm3"], ideal)
            self.assertGreater(v["removed_lower_mm3"], ideal * 0.8)

    def test_cad_pattern_pilot_then_bore_has_ordered_tools_and_valid_typed_program(self):
        config = example().hole_job()
        events = []
        controls = []
        pilot_removed = None
        helices = 0
        self.assertEqual(config["operations"][0].centers_mm, ((10.0, 12.0), (28.0, 12.0)))
        with Simulation(**config) as sim:
            for update in sim.run_operations():
                if isinstance(update, OperationUpdate):
                    controls.append((update.operation, update.stage))
                    events.extend(update.events)
                    if update.operation == "pilot" and update.stage == "end":
                        pilot_removed = sim.state.volume()["removed_lower_mm3"]
                else:
                    events.append(dict(kind="motion", motion=update.motion))
                    helices += update.motion["curve"]["kind"] == "helix"
            self.assertEqual(
                controls, [("pilot", "begin"), ("pilot", "end"), ("finish-holes", "begin"), ("finish-holes", "end")]
            )
            self.assertGreater(sim.state.volume()["removed_lower_mm3"], pilot_removed + 800)
            count = sim.state.completed
            self.assertGreater(helices, 0)
            self.assertTrue(sim.state.finished)
        program = dict(
            version=1,
            id="hole-program",
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

    def test_retained_drill_and_helical_bore_seek_replay_exact_stock(self):
        with RecordedRun(**{'rapid_mm_min': 6000, **example().hole_job()}, checkpoint_every=8) as run:
            player = Player(run)
            player.pause()
            run.wait()
            controls = list(run.operation_controls())
            bore_start = controls[2]["completed"]
            for cursor in (Cursor(3, 0.5), Cursor(bore_start + 4, 0.5)):
                player.seek(cursor)
                partial = player.state.heights.tobytes()
                volume = player.state.volume()["removed_lower_mm3"]
                self.assertGreater(volume, 0.001)
                player.seek(Cursor(run.status()["available"]))
                self.assertGreater(player.state.volume()["removed_lower_mm3"], volume)
                player.seek(cursor)
                self.assertEqual(player.state.heights.tobytes(), partial)

    def test_bounded_python_patterns_and_invalid_depth_fail_before_motion(self):
        # No count cap: an endless pattern is refused by the memory budget.
        with mock.patch.dict(os.environ, {"CAMLIB_MEMORY_BUDGET_BYTES": str(64 << 20)}):
            with self.assertRaisesRegex(ValueError, "hole centers needs"):
                Drilling(
                    "d",
                    "tool",
                    itertools.repeat((1, 2)),
                    4,
                    12,
                    feed_mm_min=150,
                    retract_feed_mm_min=600,
                    spindle_rpm=6000,
                    tool_number=1,
                    length_register=1,
                    coolant="flood",
                )
        with self.assertRaisesRegex(ValueError, "two coordinates"):
            Bore(
                "b",
                "tool",
                (itertools.repeat(1),),
                12,
                4,
                2,
                12,
                entry_feed_mm_min=150,
                feed_mm_min=600,
                retract_feed_mm_min=600,
                spindle_rpm=12000,
                tool_number=1,
                length_register=1,
                coolant="flood",
            )
        config = example().hole_job()
        op = config["operations"][0]
        config["operations"] = (replace(op, depth_reference="full_diameter", depth_z_mm=0),)
        with Simulation(**config) as sim:
            with self.assertRaisesRegex(SimulationError, "drilling parameters"):
                list(sim.run_operations())
            self.assertEqual(sim.state.completed, 0)
            self.assertFalse(sim.state.finished)
