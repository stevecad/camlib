"""Regression tests for #433: bounds with no source outside the motion checks.

Every test here fails on main before #433.
"""

from dataclasses import replace
import math
from pathlib import Path
import unittest

import camlib
from camlib.dynamics import MotionPolicy
from camlib.motion import WorkFrame
from camlib.post import GCodeDialect, GCodeDecoder, Machine, ToolSetting, post_job, post_job_input
from test_drill_cycles import example, job as cycle_job, linuxcnc as cycle_mill, tool_table as cycle_tools
from test_dwell import job as dwell_job
from test_post import fixture as post_fixture
from test_tapping import posted_text
from test_thermal import job as thermal_job, table as thermal_table, wincnc_edge_start_job

ROOT = Path(__file__).resolve().parents[2]


def linuxcnc_mill(j, *, dialect=None, **overrides):
    values = dict(
        tool_change_position_mode="preserve_tip",
        dialect=dialect if dialect is not None else GCodeDialect.linuxcnc(),
        output_units="mm",
        decimals=3,
        max_feed_mm_min=10000,
        max_spindle_rpm=30000,
        planes=("xy", "xz", "yz"),
        arcs=True,
        helices=True,
        max_sweep_rad=math.tau,
        max_tool_number=99999,
        max_length_register=99999,
        tool_changes=True,
        flood=True,
        mist=True,
        rapid_mm_min=6000,
    )
    values.update(overrides)
    return Machine("mill", cycle_tools(j), **values)


class BoundSourcesTest(unittest.TestCase):
    def test_1_dialect_without_dwell_max_refuses_g4_g82_pierce_and_edge_delay_by_name(self):
        # GCodeDialect has no default dwell_max_s.
        self.assertIsNone(GCodeDialect().dwell_max_s)
        self.assertIsNone(GCodeDialect.linuxcnc().dwell_max_s)

        # G4 drilling dwell: refused by name without dwell_max_s; posts with it.
        g4_job = dwell_job(0.5)
        no_dwell = linuxcnc_mill(g4_job, dialect=GCodeDialect.linuxcnc())
        with self.assertRaisesRegex(
            Exception, r"spot: dwell 0\.5 s cannot be written:.*no dwell range.*GCodeDialect\.dwell_max_s"
        ):
            g4_job.post_nc("top", machine=no_dwell, accept={"unchecked_machine"})
        with_dwell = linuxcnc_mill(g4_job, dialect=GCodeDialect.linuxcnc(dwell_max_s=1e5))
        g4_nc = g4_job.post_nc("top", machine=with_dwell, accept={"unchecked_machine"}).text
        self.assertEqual(g4_nc.count("G4 P0.5\n"), 2)
        rust_g4 = post_job(post_job_input(g4_job, "top", machine=with_dwell, accept=("unchecked_machine",)))
        self.assertEqual(rust_g4["program"], g4_nc)

        # G82 canned drilling dwell: refused by name without dwell_max_s; posts with it.
        g82_job = cycle_job(dwell_s=0.5)
        no_dwell_g82 = cycle_mill(g82_job, dialect=GCodeDialect.linuxcnc())
        with self.assertRaisesRegex(
            Exception, r"holes: dwell 0\.5 s cannot be written:.*no dwell range.*GCodeDialect\.dwell_max_s"
        ):
            g82_job.post_nc("top", machine=no_dwell_g82, accept={"unchecked_machine"})
        with_dwell_g82 = cycle_mill(g82_job, dialect=GCodeDialect.linuxcnc(dwell_max_s=1e5))
        g82_nc = g82_job.post_nc("top", machine=with_dwell_g82, accept={"unchecked_machine"}).text
        self.assertIn("G82 X8 Y8 Z4 R11 P0.5 F150\n", g82_nc)
        rust_g82 = post_job(post_job_input(g82_job, "top", machine=with_dwell_g82, accept=("unchecked_machine",)))
        self.assertEqual(rust_g82["program"], g82_nc)

        # Plasma pierce delay: refused by name without dwell_max_s; posts with it.
        plasma_job = thermal_job()
        plasma_dialect_none = GCodeDialect(beam_s_max=1, height_control_word="P2").with_commands(
            height_control_on="M63", height_control_off="M62"
        )
        plasma_machine_none = Machine(
            "plasma",
            thermal_table(plasma_job),
            dialect=plasma_dialect_none,
            tool_change_position_mode="preserve_tip",
            output_units="mm",
            decimals=3,
            max_feed_mm_min=10000,
            max_spindle_rpm=30000,
            planes=("xy", "xz", "yz"),
            arcs=True,
            helices=True,
            max_sweep_rad=math.tau,
            max_tool_number=99999,
            max_length_register=99999,
            tool_changes=True,
            flood=True,
            mist=True,
            rapid_mm_min=6000,
        )
        with self.assertRaisesRegex(
            Exception, r"plasma: dwell 0\.5 s cannot be written:.*no dwell range.*GCodeDialect\.dwell_max_s"
        ):
            plasma_job.post_nc(
                "sheet",
                machine=plasma_machine_none,
                lowering=camlib.LoweringOptions(native_arcs=True),
                accept={"unchecked_machine"},
            )
        plasma_machine_ok = replace(plasma_machine_none, dialect=replace(plasma_dialect_none, dwell_max_s=1e5))
        plasma_nc = plasma_job.post_nc(
            "sheet",
            machine=plasma_machine_ok,
            lowering=camlib.LoweringOptions(native_arcs=True),
            accept={"unchecked_machine"},
        ).text
        self.assertIn("G4 P0.5\n", plasma_nc)

        # WinCNC edge-start delay: refused by name when dwell_max_s is unset.
        w, edge_job, edge_machine = wincnc_edge_start_job(delay_s=0.125)
        edge_no_dwell = replace(edge_machine, dialect=replace(edge_machine.dialect, dwell_max_s=None))
        with self.assertRaisesRegex(
            Exception, r"plasma: dwell 0\.125 s cannot be written:.*no dwell range.*GCodeDialect\.dwell_max_s"
        ):
            edge_job.post_nc(
                "sheet",
                machine=edge_no_dwell,
                post=w.WinCNCPlasmaPost,
                decoder=w.WinCNCPlasmaDecoder,
                lowering=camlib.LoweringOptions(native_arcs=True),
                accept={"unchecked_machine"},
            )

        # Decoder also refuses G4 when dwell_max_s is None.
        header = g4_job.compile().setup("top").program_header()
        dec = GCodeDecoder(no_dwell, header)
        for line in (
            "G21",
            "G90",
            "G91.1",
            "G17",
            "G94",
            "G97",
            "G40",
            "G49",
            "G80",
            "G92.1",
            "G61.1",
            "M5",
            "M9",
            "G54",
        ):
            dec.read_line(line)
        with self.assertRaisesRegex(ValueError, r"GCodeDialect\.dwell_max_s"):
            dec.read_line("G4 P0.5")

    def test_2_declared_dwell_max_below_requested_dwell_refuses_by_name(self):
        j = dwell_job(10.5)
        m = linuxcnc_mill(j, dialect=GCodeDialect.linuxcnc(dwell_max_s=10))
        with self.assertRaisesRegex(Exception, r"spot: dwell 10\.5 s.*10 s.*GCodeDialect\.dwell_max_s"):
            j.post_nc("top", machine=m, accept={"unchecked_machine"})

        g82 = cycle_job(dwell_s=10.5)
        m_g82 = cycle_mill(g82, dialect=GCodeDialect.linuxcnc(dwell_max_s=10))
        with self.assertRaisesRegex(Exception, r"holes: dwell 10\.5 s.*10 s.*GCodeDialect\.dwell_max_s"):
            g82.post_nc("top", machine=m_g82, accept={"unchecked_machine"})

        header = j.compile().setup("top").program_header()
        dec = GCodeDecoder(m, header)
        for line in (
            "G21",
            "G90",
            "G91.1",
            "G17",
            "G94",
            "G97",
            "G40",
            "G49",
            "G80",
            "G92.1",
            "G61.1",
            "M5",
            "M9",
            "G54",
        ):
            dec.read_line(line)
        with self.assertRaisesRegex(ValueError, r"10\.5 s.*10 s.*GCodeDialect\.dwell_max_s"):
            dec.read_line("G4 P10.5")

    def test_3_centroid_327_67_still_applies(self):
        centroid = example("centroid_post")
        ok_job = dwell_job(327.67)
        m = centroid.define_machine(
            "Acorn",
            cycle_tools(ok_job),
            traverse_feed_mm_min=3000,
            work_to_machine=WorkFrame(),
            decimals=3,
            output_units="mm",
            max_feed_mm_min=10000,
            max_spindle_rpm=30000,
            planes=("xy", "xz", "yz"),
            arcs=True,
            helices=True,
            max_sweep_rad=math.tau,
            max_tool_number=99999,
            max_length_register=99999,
            tool_changes=True,
            flood=True,
            mist=True,
            rapid_mm_min=6000,
        )
        nc = posted_text(ok_job, m, centroid.CentroidPost, centroid.CentroidDecoder)
        self.assertIn("G4 P327.67", nc)

        too_long = dwell_job(327.68)
        with self.assertRaisesRegex(Exception, r"spot: dwell 327\.68 s.*327\.67 s.*GCodeDialect\.dwell_max_s"):
            posted_text(too_long, m, centroid.CentroidPost, centroid.CentroidDecoder)

    def test_4_probe_fast_feed_held_to_machine_max_feed(self):
        _, base = post_fixture()
        over = GCodeDialect.linuxcnc(probing="inspection_plus", probe_fast_feed_mm_min=6000)
        with self.assertRaisesRegex(
            ValueError,
            r"6000.*5000.*GCodeDialect\.probe_fast_feed_mm_min.*Machine\.max_feed_mm_min",
        ):
            replace(base, dialect=over, max_feed_mm_min=5000)

        # Equal to max_feed_mm_min is accepted.
        equal = GCodeDialect.linuxcnc(probing="inspection_plus", probe_fast_feed_mm_min=5000)
        m_eq = replace(base, dialect=equal, max_feed_mm_min=5000)
        self.assertEqual(m_eq.dialect.probe_fast_feed_mm_min, 5000)

        # Above the old 1e5 cap, within Machine.max_feed_mm_min, is accepted.
        fast = GCodeDialect.linuxcnc(probing="inspection_plus", probe_fast_feed_mm_min=2e5)
        m_fast = replace(base, dialect=fast, max_feed_mm_min=2.5e5)
        self.assertEqual(m_fast.dialect.probe_fast_feed_mm_min, 2e5)

    def test_5_hole_dwell_and_thermal_delays_drop_invented_caps_and_post_enforces_dwell_max(self):
        # Drilling dwell_s = 2e5 plans and is held to dwell_max_s at post time.
        long_drill = dwell_job(2e5)
        m_short = linuxcnc_mill(long_drill, dialect=GCodeDialect.linuxcnc(dwell_max_s=1e5))
        with self.assertRaisesRegex(Exception, r"spot: dwell 200000\.0 s.*100000\.0 s.*GCodeDialect\.dwell_max_s"):
            long_drill.post_nc("top", machine=m_short, accept={"unchecked_machine"})
        m_ok = linuxcnc_mill(long_drill, dialect=GCodeDialect.linuxcnc(dwell_max_s=3e5))
        nc = long_drill.post_nc("top", machine=m_ok, accept={"unchecked_machine"}).text
        self.assertIn("G4 P200000.\n", nc)

        # Thermal pierce_delay_s = 2000 plans and is held to dwell_max_s at post time.
        long_plasma = thermal_job(pierce_delay_s=2000)
        d_short = GCodeDialect(beam_s_max=1, height_control_word="P2", dwell_max_s=1000).with_commands(
            height_control_on="M63", height_control_off="M62"
        )
        m_p_short = Machine(
            "plasma",
            thermal_table(long_plasma),
            dialect=d_short,
            tool_change_position_mode="preserve_tip",
            output_units="mm",
            decimals=3,
            max_feed_mm_min=10000,
            max_spindle_rpm=30000,
            planes=("xy", "xz", "yz"),
            arcs=True,
            helices=True,
            max_sweep_rad=math.tau,
            max_tool_number=99999,
            max_length_register=99999,
            tool_changes=True,
            flood=True,
            mist=True,
            rapid_mm_min=6000,
        )
        with self.assertRaisesRegex(Exception, r"plasma: dwell 2000\.0 s.*1000 s.*GCodeDialect\.dwell_max_s"):
            long_plasma.post_nc(
                "sheet",
                machine=m_p_short,
                lowering=camlib.LoweringOptions(native_arcs=True),
                accept={"unchecked_machine"},
            )
        m_p_ok = replace(m_p_short, dialect=replace(d_short, dwell_max_s=3000))
        nc_p = long_plasma.post_nc(
            "sheet",
            machine=m_p_ok,
            lowering=camlib.LoweringOptions(native_arcs=True),
            accept={"unchecked_machine"},
        ).text
        self.assertIn("G4 P2000.\n", nc_p)

    def test_6_timing_settings_beyond_old_caps_accepted_and_invalid_values_name_field(self):
        program, machine = post_fixture()
        high = replace(
            machine,
            max_feed_mm_min=2e7,
            max_normal_accel_mm_s2=2e9,
            max_axis_feed_mm_min=(2e7, 2e7, 2e7),
            motion_policy=MotionPolicy(
                tangential_accel_mm_s2=2e9,
                rapid_feed_mm_min=2e7,
                per_block_s=25.0,
            ),
        )
        scheduled = high.schedule(program)
        self.assertGreater(scheduled.motion_seconds, 25.0)

        for bad in (0, -1, float("nan"), float("inf")):
            with self.assertRaisesRegex(ValueError, rf"tangential_accel_mm_s2.*{bad}"):
                MotionPolicy(tangential_accel_mm_s2=bad, rapid_feed_mm_min=6000)
            with self.assertRaisesRegex(ValueError, rf"rapid_feed_mm_min.*{bad}"):
                MotionPolicy(tangential_accel_mm_s2=100, rapid_feed_mm_min=bad)
            with self.assertRaisesRegex(ValueError, rf"max_feed_mm_min.*{bad}"):
                replace(high, max_feed_mm_min=bad).schedule(program)
        for bad in (-1, float("nan"), float("inf")):
            with self.assertRaisesRegex(ValueError, rf"per_block_s.*{bad}"):
                MotionPolicy(tangential_accel_mm_s2=100, rapid_feed_mm_min=6000, per_block_s=bad)

    def test_7_haas_settings_22_and_52_and_owner_declared_dwell_max(self):
        haas = example("haas_ngc_post")
        _, base = post_fixture()
        # Settings 22 and 52 accept values above the old invented 100 mm cap.
        m = haas.define_machine(
            "Haas",
            base.tools,
            traverse_feed_mm_min=3000,
            coordinate_system="fanuc",
            h_t_agreement=False,
            g92_shift_mm=(0, 0, 0),
            rotation_scaling=False,
            work_to_machine=WorkFrame(),
            setting_22_mm=150,
            setting_52_mm=150,
            output_units="mm",
            decimals=3,
            max_feed_mm_min=10000,
            max_spindle_rpm=30000,
            max_sweep_rad=math.tau,
            tool_changes=True,
            flood=True,
            tool_change_position_mode="table",
            rapid_mm_min=6000,
        )
        self.assertEqual(m.setting_22_mm, 150)
        self.assertEqual(m.setting_52_mm, 150)
        # Default Haas define_machine leaves dwell_max_s unset.
        self.assertIsNone(m.dialect.dwell_max_s)

        for field, bad in (
            ("setting_22_mm", -1),
            ("setting_22_mm", 0),
            ("setting_22_mm", float("nan")),
            ("setting_22_mm", float("inf")),
            ("setting_52_mm", -1),
            ("setting_52_mm", float("nan")),
            ("setting_52_mm", float("inf")),
        ):
            with self.assertRaisesRegex(ValueError, field):
                haas.define_machine(
                    "Haas",
                    base.tools,
                    traverse_feed_mm_min=3000,
                    coordinate_system="fanuc",
                    h_t_agreement=False,
                    g92_shift_mm=(0, 0, 0),
                    rotation_scaling=False,
                    work_to_machine=WorkFrame(),
                    output_units="mm",
                    decimals=3,
                    max_feed_mm_min=10000,
                    max_spindle_rpm=30000,
                    max_sweep_rad=math.tau,
                    tool_changes=True,
                    flood=True,
                    tool_change_position_mode="table",
                    rapid_mm_min=6000,
                    **{field: bad},
                )

    def test_8_carvera_identity_check_and_byte_identical_example(self):
        from test_rust_carvera_post import drill_job, machine_for

        carvera = example("carvera_post")
        j = drill_job(dwell_s=0.5)
        m_with_dwell = machine_for(j)
        # Default Carvera define_machine has no dwell_max_s.
        header = j.compile().setup("top").program_header()
        tools = tuple(
            ToolSetting.for_tool(t, number=i, length_register=i, position_after_mm=header.initial_position_mm)
            for i, t in enumerate(header.tools, 1)
        )
        length = tools[0].gauge_length_mm
        frame = WorkFrame(origin_mm=(0, 0, -3 - header.initial_position_mm[2] - length))
        m_default = carvera.define_machine(
            "synthetic Carvera",
            tools,
            initial_gauge_length_mm=length,
            unmodified_coordinates=True,
            automatic_tool_length=True,
            work_to_machine=frame,
            tool_change_position_mode="table",
            output_units="mm",
            decimals=3,
            max_feed_mm_min=10000,
            max_spindle_rpm=30000,
            max_tool_number=6,
            max_length_register=6,
            tool_changes=True,
            flood=False,
            mist=True,
            rapid_mm_min=6000,
        )
        self.assertIsNone(m_default.dialect.dwell_max_s)
        # Without dwell_max_s, Carvera refuses the drilling dwell by name (both Python and Rust).
        with self.assertRaisesRegex(Exception, r"holes: dwell 0\.5 s.*GCodeDialect\.dwell_max_s"):
            j.post_nc(
                "top",
                machine=m_default,
                post=carvera.CarveraPost,
                decoder=carvera.CarveraDecoder,
                accept={"unchecked_machine"},
            )
        # With dwell_max_s=1e5, Carvera posts byte-identically to carvera-drill.nc.
        py_nc = j.post_nc(
            "top",
            machine=m_with_dwell,
            post=carvera.CarveraPost,
            decoder=carvera.CarveraDecoder,
            accept={"unchecked_machine"},
        ).text
        rs = post_job(post_job_input(j, "top", machine=m_with_dwell, accept=("unchecked_machine",)))
        golden = (ROOT / "fixtures/posts/rust/carvera-drill.nc").read_text()
        self.assertEqual(py_nc, golden)
        self.assertEqual(rs["program"], golden)


if __name__ == "__main__":
    unittest.main()
