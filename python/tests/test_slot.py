import importlib.util
import math
from pathlib import Path
import unittest

from camlib import Engine
from camlib.operations import Pocket, TrochoidalSlot
from camlib.planar import Boundary, Component, PlanarRegion
from camlib.playback import Cursor, Player, RecordedRun
from camlib.simulation import OperationUpdate, Simulation, SimulationError, StockSpec
from test_simulation import tool

ROOT = Path(__file__).resolve().parents[2]


def slot():
    return TrochoidalSlot(
        "slot",
        tool(),
        (8, 12),
        (14, 12),
        6,
        10,
        8,
        1,
        12,
        feed_mm_min=600,
        entry_feed_mm_min=150,
        spindle_rpm=12000,
        tool_number=1,
        length_register=1,
        coolant="flood",
    )


class TrochoidalSlotTest(unittest.TestCase):
    def test_python_native_slot_streams_actual_stock_and_a_valid_complete_program(self):
        op = slot()
        config = dict(
            stock=StockSpec((0, 0, 0), (24, 24, 10), 0.25),
            tools=(tool(),),
            operations=(op,),
            revision="python-slot",
            initial_position_mm=(0, 0, 12),
        )
        events, reports, first_stock = [], [], False
        with Simulation(**config) as sim:
            for update in sim.run_operations():
                if isinstance(update, OperationUpdate):
                    reports.append(update.report)
                    events.extend(update.events)
                else:
                    events.append(dict(kind="motion", motion=update.motion))
                    if update.changed_count and not first_stock:
                        first_stock = True
                        self.assertIsNone(sim._process.poll())
                        self.assertFalse(sim.state.finished)
            self.assertTrue(first_stock and sim.state.finished)
            self.assertEqual(reports[-1]["seed_orbits"], 2)
            self.assertIn("NOT limited", reports[-1]["seed_entry_policy"])
            self.assertLessEqual(reports[-1]["max_bounded_engagement_deg"], 90)
            ideal = (6 * 6 + 9 * math.pi) * 2
            volume = sim.state.volume()
            self.assertLessEqual(volume["removed_lower_mm3"], ideal)
            self.assertGreaterEqual(volume["removed_upper_mm3"], ideal - 0.5)
            self.assertGreater(volume["removed_lower_mm3"], ideal * 0.8)
            count = sim.state.completed
        program = dict(
            version=1,
            id="slot-program",
            revision="1",
            units="mm",
            tolerance_mm=1e-5,
            frame=dict(origin_mm=(0, 0, 0), x=(1, 0, 0), y=(0, 1, 0), z=(0, 0, 1)),
            initial_position_mm=(0, 0, 12),
            tools=[tool().to_dict()],
            events=events,
        )
        with Engine() as engine:
            self.assertEqual(
                engine._request("validate_program", program=program, capabilities=None)["summary"]["motions"], count
            )

    def test_mixed_pocket_and_slot_retained_run_replays_actual_material(self):
        region = PlanarRegion("pocket", (Component(Boundary.polygon(((3, 3), (11, 3), (11, 11), (3, 11)))),))
        rough = Pocket(
            "rough",
            tool(),
            region,
            10,
            9,
            1,
            1.5,
            12,
            feed_mm_min=600,
            plunge_mm_min=150,
            spindle_rpm=12000,
            tool_number=1,
            length_register=1,
            coolant="flood",
        )
        with RecordedRun(
            StockSpec((0, 0, 0), (24, 24, 10), 0.25),
            (tool(),),
            operations=(rough, slot()),
            revision="mixed-slot",
            initial_position_mm=(0, 0, 12),
            checkpoint_every=32,
            rapid_mm_min=6000,
        ) as run:
            player = Player(run)
            player.pause()
            run.wait()
            controls = list(run.operation_controls())
            self.assertEqual(
                [(c["operation"], c["stage"]) for c in controls],
                [("rough", "begin"), ("rough", "end"), ("slot", "begin"), ("slot", "end")],
            )
            first_slot = controls[2]["completed"] + 1
            player.seek(Cursor(first_slot, 0.5))
            partial = player.state.heights.tobytes()
            player.seek(Cursor(run.status()["available"]))
            self.assertTrue(player.state.finished)
            self.assertGreater(player.state.volume()["removed_lower_mm3"], 100)
            player.seek(Cursor(first_slot, 0.5))
            self.assertEqual(player.state.heights.tobytes(), partial)

    def test_matching_cadquery_target_has_independent_capsule_volume(self):
        path = ROOT / "python/tests/support/trochoidal_slot.py"
        spec = importlib.util.spec_from_file_location("slot_example", path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        target = module.cadquery_target().val()
        self.assertTrue(target.isValid())
        self.assertAlmostEqual(target.Volume(), 40 * 24 * 10 - (20 * 6 + 9 * math.pi) * 2, places=6)
        self.assertEqual(module.slot_job()["operations"][0].boundary_tolerance_mm, 0.01)

    def test_invalid_slot_fails_before_any_motion(self):
        from dataclasses import replace

        with Simulation(
            StockSpec((0, 0, 0), (24, 24, 10), 0.25),
            (tool(),),
            operations=(replace(slot(), width_mm=4),),
            revision="bad-slot",
            initial_position_mm=(0, 0, 12),
        ) as sim:
            with self.assertRaisesRegex(SimulationError, "trochoidal slot"):
                list(sim.run_operations())
            self.assertFalse(sim.state.finished)
            self.assertEqual(sim.state.completed, 0)


S_PATH = ((6, 12, 1), (20, 12, -1), (34, 12, 0))


def s_slot(**overrides):
    values = dict(
        feed_mm_min=600,
        entry_feed_mm_min=150,
        spindle_rpm=12000,
        tool_number=1,
        length_register=1,
        coolant="flood",
        path_mm=S_PATH,
    )
    values.update(overrides)
    return TrochoidalSlot("s", tool(), None, None, 6, 10, 9, 1, 12, **values)


class CurvedSlotTest(unittest.TestCase):
    """#9: slots along a centreline of lines and arcs."""

    def test_s_slot_plans_within_the_engagement_limit_and_posts(self):
        import camlib
        from camlib.post import LoweringOptions, Machine, ToolSetting
        from camlib.postprocessor import GCodeDialect

        op = s_slot(max_engagement_deg=75)
        self.assertEqual(op.to_dict()["path_mm"], [list(map(float, v)) for v in S_PATH])
        self.assertNotIn("start_mm", op.to_dict())
        setup = camlib.Setup("top", StockSpec((0, 0, 0), (40, 24, 10), 0.25), (tool(),), (op,), (0, 0, 12))
        job = camlib.Job("s-slot", "1", (setup,))
        report = None
        with job.regenerate() as run:
            for event in run:
                if isinstance(event.update, OperationUpdate) and event.update.stage == "end":
                    report = event.update.report
        self.assertEqual(report["path_pieces"], 2)
        self.assertEqual(report["corner_links"], 0)
        self.assertLessEqual(report["max_bounded_engagement_deg"], 75)
        machine = Machine(
            "mill",
            (ToolSetting.for_tool(tool(), number=1, length_register=1, position_after_mm=(0, 0, 12)),),
            dialect=GCodeDialect.linuxcnc(),
            output_units="mm",
            decimals=3,
            max_feed_mm_min=10000,
            max_spindle_rpm=30000,
            planes=("xy", "xz", "yz"),
            arcs=True,
            helices=True,
            max_sweep_rad=math.tau,
            max_tool_number=99,
            max_length_register=99,
            tool_changes=True,
            flood=True,
            mist=False,
            tool_change_position_mode="preserve_tip",
            rapid_mm_min=6000,
        )
        text = job.post_nc(
            "top", machine=machine, lowering=LoweringOptions(native_arcs=True), accept={"unchecked_machine"}
        ).text
        # Orbits and the links along the arcs are circular blocks.
        self.assertGreater(sum(line.startswith(("G2", "G3")) for line in text.splitlines()), 10)

    def test_path_or_points_not_both(self):
        with self.assertRaisesRegex(ValueError, "either start_mm and end_mm"):
            TrochoidalSlot(
                "s",
                tool(),
                (8, 12),
                (14, 12),
                6,
                10,
                9,
                1,
                12,
                feed_mm_min=600,
                entry_feed_mm_min=150,
                spindle_rpm=12000,
                tool_number=1,
                length_register=1,
                coolant="flood",
                path_mm=S_PATH,
            )
        with self.assertRaisesRegex(ValueError, "x, y, bulge_to_next"):
            s_slot(path_mm=((6, 12), (20, 12)))

    def test_too_tight_bend_is_refused_before_any_motion(self):
        op = s_slot(path_mm=((10, 12, 1), (11.8, 12, 0)))
        config = dict(
            stock=StockSpec((0, 0, 0), (40, 24, 10), 0.25),
            tools=(tool(),),
            operations=(op,),
            revision="tight",
            initial_position_mm=(0, 0, 12),
        )
        with Simulation(**config) as sim:
            with self.assertRaisesRegex(SimulationError, "bends toward the orbit side"):
                list(sim.run_operations())
