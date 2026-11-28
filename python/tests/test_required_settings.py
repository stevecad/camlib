"""Machine-dependent settings have no defaults; controller rules match LinuxCNC.

Cutting data (feeds, spindle speed, coolant) and the T/H numbers depend on the
tool, material and machine: every operation requires them. The default dialect
accepts arc words exactly as LinuxCNC's interp_arc.cc does.
"""

import dataclasses
import importlib.util
import unittest
from dataclasses import replace
from pathlib import Path

import camlib.operations as operations
import camlib.turning as turning
from camlib.postprocessor import GCodeDialect
from camlib.simulation import Update

ROOT = Path(__file__).resolve().parents[2]
MACHINE_DEPENDENT = {
    "feed_mm_min",
    "plunge_mm_min",
    "entry_feed_mm_min",
    "retract_feed_mm_min",
    "spindle_rpm",
    "coolant",
    "tool_number",
    "length_register",
    "pierce_delay_s",
    "power_percent",
    "feed_mm_rev",
    "rpm",
}


class RequiredSettingsTest(unittest.TestCase):
    def test_no_operation_defaults_machine_dependent_settings(self):
        checked = 0
        for module in (operations, turning):
            for name in dir(module):
                cls = getattr(module, name)
                if not (isinstance(cls, type) and dataclasses.is_dataclass(cls) and cls.__module__ == module.__name__):
                    continue
                for f in dataclasses.fields(cls):
                    if f.name in MACHINE_DEPENDENT and f.default is not None:
                        checked += 1
                        self.assertIs(f.default, dataclasses.MISSING, f"{name}.{f.name} has a default")
                        self.assertIs(f.default_factory, dataclasses.MISSING, f"{name}.{f.name}")
        self.assertGreater(checked, 100)

    def test_operation_without_cutting_data_is_refused(self):
        with self.assertRaisesRegex(TypeError, "spindle_rpm"):
            operations.Facing("face", "t1", floor_z_mm=5, stepdown_mm=1, stepover_mm=2, clearance_z_mm=8)
        with self.assertRaisesRegex(TypeError, "rpm"):
            turning.TurnFace("face", "cnmg", z_mm=0, feed_mm_rev=0.15)

    def test_machine_declares_everything_about_itself(self):
        import camlib.post
        from camlib.post import Machine
        from camlib.playback import RecordedRun

        required = {f.name for f in dataclasses.fields(Machine) if f.default is dataclasses.MISSING}
        self.assertLessEqual(
            {
                "dialect",
                "output_units",
                "decimals",
                "max_feed_mm_min",
                "max_spindle_rpm",
                "rapid_mm_min",
                "planes",
                "arcs",
                "helices",
                "max_sweep_rad",
                "max_tool_number",
                "max_length_register",
                "tool_changes",
                "flood",
                "mist",
                "tool_change_position_mode",
            },
            required,
        )
        with self.assertRaisesRegex(TypeError, "dialect"):
            Machine("mill", ())
        # One posting path: no second (legacy) setup type or built-in formatter.
        for name in ("Setup", "Post", "OutputChecker", "LoweredPost", "LoweredOutputChecker", "GrblContext"):
            self.assertFalse(hasattr(camlib.post, name), name)
        with self.assertRaisesRegex(TypeError, "rapid_mm_min"):
            RecordedRun(None, (), (), revision="1", initial_position_mm=(0, 0, 0))
        # G99 is an explicit LinuxCNC capability; generic defaults stay unchanged.
        self.assertEqual(GCodeDialect.linuxcnc(), GCodeDialect().with_commands(cycle_return_r="G99"))

    def test_default_dialect_accepts_arcs_as_linuxcnc(self):
        d = GCodeDialect()
        self.assertEqual((d.radius_tolerance_mm, d.radius_relative_tolerance), (0.00127, 0.001))
        self.assertTrue(d.radius_accepts(2, 2.00127))
        self.assertFalse(d.radius_accepts(2, 2.005))  # 0.25 % and above T
        self.assertTrue(d.radius_accepts(10, 10.005))  # 0.05 %
        self.assertFalse(d.radius_accepts(200, 200.15))  # above 100 T
        exact = replace(d, radius_tolerance_mm=0)
        self.assertTrue(exact.radius_accepts(5, 5))
        self.assertFalse(exact.radius_accepts(5, 5.000001))
        with self.assertRaises(ValueError):
            replace(d, radius_relative_tolerance=0.002)

    def test_rapid_retracts_can_be_disabled(self):
        spec = importlib.util.spec_from_file_location("standalone_required", ROOT / "examples/standalone_job.py")
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        job = module.build_job()

        def kinds(job):
            with job.regenerate() as run:
                return [e.update.motion["kind"] for e in run if isinstance(e.update, Update)]

        on = kinds(job)
        off = kinds(replace(job, setups=(replace(job.setups[0], rapid_retracts=False),)))
        # With them on, the retracts proven clear become rapids; those that
        # start beside stock the cutter just cut feed through it (#430), and
        # each is split there: its rest is a rapid (#441), one move more.
        split = len(on) - len(off)
        self.assertGreater(split, 0)
        self.assertLess(on.count("retract"), off.count("retract"))
        self.assertGreater(off.count("retract"), 0)
        self.assertEqual(on.count("rapid") - off.count("rapid"), off.count("retract") - on.count("retract") + split)


if __name__ == "__main__":
    unittest.main()
