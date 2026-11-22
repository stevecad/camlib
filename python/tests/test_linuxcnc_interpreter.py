"""Posted LinuxCNC programs, executed by LinuxCNC's own interpreter (#20).

Needs `rs274` (LinuxCNC's standalone interpreter); skipped where it is not
installed. See `support/linuxcnc_interpreter.py`.
"""

import importlib.util
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest
from unittest.mock import patch

from support import linuxcnc_interpreter as linuxcnc

ROOT = Path(__file__).resolve().parents[2]


def example(name):
    spec = importlib.util.spec_from_file_location(name, ROOT / "examples" / f"{name}.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def operations_of(setup):
    """{operation id: the operation} of a setup (through its dependency nodes)."""
    found = {}
    for entry in setup.operations:
        operation = getattr(entry, "operation", entry)
        found[operation.id] = operation
    return found


class InterpreterComparisonTest(unittest.TestCase):
    def compare_line(self, commands, feed=600):
        start, end = (13.0, 12.0, 4.0), (12.0, 12.0, 4.0)
        planned = [
            dict(
                curve=dict(kind="line", start=start, end=end),
                kind="feed",
                tolerance=0,
                where="line after a repeated endpoint",
                state=dict(feed=float(feed)),
            )
        ]
        with patch.object(linuxcnc, "interpret", return_value=commands):
            moves = linuxcnc._interpreted("", (), start)
        return linuxcnc._match(self, planned, moves)

    def test_a_repeated_endpoint_does_not_shift_the_next_feed_comparison(self):
        # The fixed fixture emits G1 X13 Y12 Z4 F150 after its island arc,
        # then G1 X12 Y12 Z4 F600. Its decoded plan omits the no-motion block.
        self.assertEqual(
            self.compare_line(
                [
                    ("SET_FEED_RATE", ["150"]),
                    ("STRAIGHT_FEED", ["13", "12", "4"]),
                    ("SET_FEED_RATE", ["600"]),
                    ("STRAIGHT_FEED", ["12", "12", "4"]),
                ]
            ),
            1,
        )

    def test_the_next_actual_move_still_requires_the_planned_feed(self):
        with self.assertRaisesRegex(AssertionError, "feed at line"):
            self.compare_line(
                [
                    ("SET_FEED_RATE", ["150"]),
                    ("STRAIGHT_FEED", ["13", "12", "4"]),
                    ("STRAIGHT_FEED", ["12", "12", "4"]),
                ]
            )

    def test_a_nonzero_straight_move_is_retained_even_below_print_resolution(self):
        machine = linuxcnc.Machine((0, 0, 0))
        self.assertEqual(len(machine.apply("STRAIGHT_FEED", ["0.00001", "0", "0"])), 1)


class LinuxCNCInterpreterTest(unittest.TestCase):
    def setUp(self):
        linuxcnc.require()

    def run_job(self, job, machine, setup_id="top", **post):
        setup = next(s for s in job.setups if s.id == setup_id)
        with (
            TemporaryDirectory(prefix="camlib-linuxcnc-") as folder,
            job.post(setup_id, machine=machine, **post) as posted,
        ):
            posted.wait()
            path = Path(folder) / "program.ngc"
            # A synthetic machine: the export is for the interpreter only.
            posted.export(path, accept={"unchecked_machine"})
            tools = [n for t in machine.tools for n in (t.number, t.length_register, t.diameter_register) if n]
            return linuxcnc.compare(
                self, posted, path.read_text(), operations_of(setup), tools, setup.initial_position_mm
            )

    def test_the_interpreter_executes_the_standalone_job_as_planned(self):
        ex = example("standalone_job")
        job = ex.build_job()
        self.assertGreater(self.run_job(job, ex.machine_for(job)), 40)

    def test_the_interpreter_executes_the_two_tool_rest_job_as_planned(self):
        ex = example("standalone_job")
        job = ex.build_rest_job()
        self.assertGreater(self.run_job(job, ex.rest_machine_for(job)), 40)

    def test_the_interpreter_executes_the_fixed_milling_validation_job_as_planned(self):
        from camlib.post import LoweringOptions

        ex = example("standalone_job")
        job = ex.build_validation_job()
        self.assertGreater(self.run_job(job, ex.rest_machine_for(job), lowering=LoweringOptions(native_arcs=True)), 40)

    def test_a_pocket_round_an_island_with_arcs(self):
        import test_arc_output_policy as arcs

        job, machine = arcs.island_job()
        # At three decimals some arcs cannot be written with consistent
        # radii; those few are written as checked line segments.
        self.assertGreater(self.run_job(job, machine, lowering=arcs.LoweringOptions(native_arcs=True)), 40)

    def test_drilling_canned_cycles(self):
        import test_drill_cycles as drills

        for options in (dict(), dict(peck_mm=1.5), dict(peck_mm=1.5, peck_mode="chip_break"), dict(canned_cycle=False)):
            with self.subTest(**options):
                job = drills.job(**options)
                self.assertGreaterEqual(self.run_job(job, drills.linuxcnc(job)), 8)

    def test_r_plane_and_decreasing_pecks(self):
        import test_drilling_options as drilling

        for options in (
            {"r_plane_travel": True},
            {"r_plane_travel": True, "peck_mm": 1.5},
            {"r_plane_travel": True, "peck_mm": 1.5, "peck_mode": "chip_break"},
            {"canned_cycle": False, "peck_schedule": drilling.PeckSchedule(3, 1, 1)},
        ):
            source = drilling.fixture(**options)
            self.assertGreaterEqual(self.run_job(source, drilling.linuxcnc(source)), 8)

    def test_contour_with_wear_compensation(self):
        import test_cutter_compensation as comp

        job = comp.job()
        self.assertGreater(self.run_job(job, comp.machine(job)), 8)

    def test_rigid_tapping(self):
        import test_tapping as taps

        job = taps.job(taps.tapping)
        self.assertGreater(self.run_job(job, taps.machine(job)), 8)

    def test_a_stated_retract_before_a_tool_change_in_place(self):
        # #374. The interpreter's tool table holds zero offsets, so the two
        # tools here have one gauge length: the tip is where the program
        # says before and after M6, and the retract, the change and the
        # descent are compared block by block.
        import test_tool_change_pose as change
        from camlib import ToolChangeInPlace

        order = ("short", "long")
        job = change.in_place_job(order, 12, ToolChangeInPlace(retract_gauge_z_mm=150), long=75)
        self.assertGreaterEqual(self.run_job(job, change.in_place_machine(order, long=75)), 16)

    def test_a_stated_end_position(self):
        # #378: the Z rapid and the XY rapid after the last operation.
        import test_drill_cycles as drills
        import test_end_position as ends

        for end in (ends.UP, ends.END):
            with self.subTest(end=end):
                job = ends.job(end)
                self.assertGreaterEqual(self.run_job(job, drills.linuxcnc(job)), 8)

    def run_lathe(self, plan, dialect, change=None, **settings):
        """Post a planned lathe job for a LinuxCNC lathe with `dialect` and
        compare the interpreter's run with the plan's events. `change`
        alters the posted text first (to show a wrong program is found)."""
        from camlib.post import Machine, ToolSetting

        program = plan.program
        tools = tuple(
            ToolSetting.for_tool(t, number=i, length_register=i, position_after_mm=program.initial_position_mm)
            for i, t in enumerate(program.tools, 1)
        )
        decimals = 3
        machine = Machine(
            "lathe",
            tools,
            dialect=dialect,
            planes=("xz",),
            tool_change_position_mode="preserve_tip",
            output_units="mm",
            decimals=decimals,
            max_feed_mm_min=10000,
            max_spindle_rpm=6000,
            arcs=True,
            helices=True,
            max_sweep_rad=6.283185307179586,
            max_tool_number=99999,
            max_length_register=99999,
            tool_changes=True,
            flood=True,
            mist=True,
            rapid_mm_min=6000,
            **settings,
        )
        posted = plan.job.post_nc("lathe", machine=machine, accept={"unchecked_machine"})
        # The program's own tolerance, half a unit of the last decimal on
        # each axis (X is written as a diameter or a radius: a radius is the
        # coarser).
        tolerance = program.tolerance_mm + 0.5 * 10**-decimals * 3**0.5
        return linuxcnc.compare_program(
            self,
            program.to_dict()["events"],
            change(posted.text) if change else posted.text,
            [n for t in tools for n in (t.number, t.length_register)],
            program.initial_position_mm,
            tolerance,
        )

    def test_a_lathe_faces_roughs_and_finishes_in_radius_mode(self):
        import test_turning as turning
        from camlib.post import GCodeDialect

        plan = turning.job().plan()
        self.assertGreater(self.run_lathe(plan, GCodeDialect().with_commands(lathe_radius_mode="G8")), 40)

    def surface_speed_job(self):
        import camlib
        import test_turning as turning
        from camlib.post import GCodeDialect

        css = dict(rpm=3000, surface_speed_m_min=200)
        thread = turning.tool("thread60", camlib.ThreadInsert(60, 3), nose_radius_mm=0.1, width_mm=16, length_mm=60)
        plan = turning.job(
            feed_per_rev=True,
            tools=(camlib.TurnToolSetting(turning.cnmg(), 1, 1), camlib.TurnToolSetting(thread, 2, 2)),
            operations=(
                camlib.TurnFace("face", "cnmg", 0, feed_mm_rev=0.15, **css),
                camlib.TurnRough("rough", "cnmg", feed_mm_rev=0.25, **css),
                camlib.TurnFinish("finish", "cnmg", feed_mm_rev=0.1, **css),
                camlib.TurnThread("thread", "thread60", -20, -38, 15, 2, 1.227, rpm=600),
            ),
        ).plan()
        # G7 (X as diameter), G96 D<limit> S<m/min>, G95 F<mm/rev>, G33 K<lead>.
        dialect = GCodeDialect(x_diameter=True, css_limit_word="D").with_commands(
            lathe_diameter_mode="G7", thread_synchronized="G33", css_on="G96", feed_per_rev="G95"
        )
        return plan, dialect

    def test_a_lathe_in_diameter_mode_with_surface_speed_feed_per_revolution_and_a_thread(self):
        self.assertGreater(self.run_lathe(*self.surface_speed_job()), 60)

    def test_wrong_lathe_words_are_found(self):
        # The same program with one word changed each time: the surface
        # speed, its rpm limit, a feed per revolution, a thread's lead, and
        # a diameter. Each must be reported, by what it changes.
        plan, dialect = self.surface_speed_job()
        for old, new, reported in (
            ("G96 D3000 S200", "G96 D3000 S180", "spindle at motion"),
            ("G96 D3000 S200", "G96 D2500 S200", "css_max at motion"),
            # A feed move's own F word (the G95 block's is repeated on each).
            (" F0.25\nG1 ", " F0.3\nG1 ", "feed at motion"),
            (" K2\n", " K2.5\n", "lead at motion"),
            ("G1 X-1.6 ", "G1 X-1.7 ", "mm off motion"),
        ):
            with self.subTest(old=old, new=new):

                def change(text):
                    self.assertIn(old, text)
                    return text.replace(old, new, 1)

                with self.assertRaisesRegex(AssertionError, reported):
                    self.run_lathe(plan, dialect, change=change)

    def test_a_lathe_grooves_and_parts_off(self):
        import camlib
        import test_turning as turning
        from camlib.post import GCodeDialect

        groove = turning.tool("groove2", camlib.GrooveInsert(2, 22), nose_radius_mm=0.2, width_mm=2, length_mm=60)
        profile = (
            (0, 0, 0),
            (10, 0, 0),
            (10, -20, 0),
            (15, -20, 0),
            (15, -25, 0),
            (12, -25, 0),
            (12, -28, 0),
            (15, -28, 0),
            (15, -40, 0),
        )
        plan = turning.job(
            profile=profile,
            tools=(camlib.TurnToolSetting(turning.cnmg(), 1, 1), camlib.TurnToolSetting(groove, 2, 2)),
            operations=(
                camlib.TurnFace("face", "cnmg", 0, feed_mm_rev=0.15, rpm=1000),
                camlib.TurnRough("rough", "cnmg", feed_mm_rev=0.25, rpm=1000),
                camlib.TurnGroove("groove", "groove2", -25, -28, 12, peck_mm=1, feed_mm_rev=0.05, rpm=800),
                camlib.TurnPartOff("part", "groove2", -40, feed_mm_rev=0.05, rpm=800),
            ),
        ).plan()
        self.assertGreater(self.run_lathe(plan, GCodeDialect().with_commands(lathe_radius_mode="G8")), 40)

    def test_a_lathe_drills_bores_and_threads_inside(self):
        import math

        import camlib
        import test_turning as turning
        from camlib.post import GCodeDialect

        cone = 8 / math.tan(math.radians(59))
        drill = turning.tool("drill16", camlib.DrillInsert(16, 118, 50), nose_radius_mm=0, width_mm=40, length_mm=60)
        bar = turning.tool(
            "bar",
            camlib.TurningInsert.iso(
                included_angle_deg=55, approach_angle_deg=95, edge_length_mm=6, nose_radius_mm=0.4
            ),
            nose_radius_mm=0.4,
            width_mm=10,
            length_mm=80,
            offset_mm=1,
            side="inner",
        )
        thread = turning.tool(
            "thread_in",
            camlib.ThreadInsert(60, 2),
            nose_radius_mm=0.1,
            width_mm=10,
            length_mm=80,
            offset_mm=2,
            side="inner",
        )
        plan = turning.job(
            stock=camlib.TurnStockSpec(-60, 0, 20),
            profile=((0, 0, 0), (18, 0, 0), (18, -40, 0)),
            bore=((11, 0, 0), (11, -25, 0), (8, -25, 0), (0, -25 - cone, 0)),
            tools=(
                camlib.TurnToolSetting(drill, 1, 1),
                camlib.TurnToolSetting(bar, 2, 2),
                camlib.TurnToolSetting(thread, 3, 3),
            ),
            operations=(
                camlib.TurnDrill("drill", "drill16", -25 - cone, peck_mm=10, feed_mm_rev=0.12, rpm=900),
                camlib.TurnBoreRough("bore_rough", "bar", -24, feed_mm_rev=0.15, rpm=1000),
                camlib.TurnBoreFinish("bore_finish", "bar", -24, feed_mm_rev=0.08, rpm=1200),
                camlib.TurnThread("thread", "thread_in", 0, -15, 12, 1.5, 0.92, rpm=600),
            ),
        ).plan()
        dialect = GCodeDialect(x_diameter=True).with_commands(lathe_diameter_mode="G7", thread_synchronized="G33")
        self.assertGreater(self.run_lathe(plan, dialect), 40)

    def run_qtplasmac(self, change=None, job=None, declared=None, **cut):
        """Post a plasma job (by default the test job: a plate with three
        holes and a washer) with the QtPlasmaC example post and compare the
        interpreter's run with the plan. `declared`: more machine
        declarations; `change` alters the posted text first."""
        import camlib
        import test_thermal as thermal
        from camlib.post import ToolSetting

        post = example("qtplasmac_post")
        torch = camlib.Torch("torch", kerf_width=1.5)
        job = job(torch) if job else thermal.job(torch, **cut)
        header = job.compile().setup("sheet").program_header()
        machine = post.define_machine(
            "table",
            ToolSetting.for_tool(torch, number=1, length_register=1, position_after_mm=header.initial_position_mm),
            output_units="mm",
            decimals=3,
            max_feed_mm_min=10000,
            max_spindle_rpm=30000,
            planes=("xy", "xz", "yz"),
            arcs=True,
            helices=True,
            max_sweep_rad=6.283185307179586,
            max_tool_number=99999,
            max_length_register=99999,
            tool_changes=True,
            rapid_mm_min=6000,
            **(declared or {}),
        )
        with (
            TemporaryDirectory(prefix="camlib-linuxcnc-") as folder,
            job.post(
                "sheet",
                machine=machine,
                post=post.QtPlasmaCPost,
                decoder=post.QtPlasmaCDecoder,
                lowering=camlib.LoweringOptions(native_arcs=True),
            ) as posted,
        ):
            posted.wait()
            path = Path(folder) / "program.ngc"
            posted.export(path, accept={"unchecked_machine"})
            text = path.read_text()
            return linuxcnc.compare_thermal(
                self,
                linuxcnc.planned_events(job),
                change(text) if change else text,
                header.initial_position_mm,
                linuxcnc.deviation_bounds(posted),
                # QtPlasmaC: the torch is tool 0, on as `M3 $0 S1`; `M62 P2`
                # turns torch height control off, `M63 P2` on.
                torch_tool=0,
                full_power_s=1.0,
                thc_off_output=2,
            )

    FIXED_HEIGHT = dict(
        height_mode="fixed", pierce_height_mm=None, travel_height_mm=None, plunge_mm_min=None, pierce_delay_s=0
    )

    def test_a_qtplasmac_program_cuts_with_the_torch_and_height_control_as_planned(self):
        self.assertGreater(self.run_qtplasmac(**self.FIXED_HEIGHT), 60)

    def test_a_qtplasmac_program_with_programmed_heights(self):
        # Pierce height, travel height, a plunge to the cut height and a
        # pierce delay written in the program; small holes at reduced feed.
        self.assertGreater(self.run_qtplasmac(), 60)

    def test_a_qtplasmac_program_that_marks_then_cuts(self):
        # Part numbers marked, then the parts cut, with a marking and a
        # cutting material declared. QtPlasmaC changes material with M190,
        # which is a script of its configuration and not a code of the
        # interpreter: the standalone interpreter stops there. So first show
        # exactly that, then run the program without its M190 blocks: the
        # marks' and the cuts' paths, feeds, torch and height control are
        # compared as for any other program, and the wait that follows each
        # material change (M66) is executed. Which material is selected, and
        # that it is restored before the cuts, only camlib's decoder checks.
        import test_thermal_mark as marks

        declared = dict(mark_material=7, cut_material=3, material_wait_s=2)
        with self.assertRaisesRegex(AssertionError, "Unknown m code used: M190"):
            self.run_qtplasmac(job=marks.marked_job, declared=declared)

        def without_material_changes(text):
            lines = text.splitlines()
            self.assertEqual([n for n in lines if n.startswith("M190")], ["M190 P7", "M190 P3"])
            return "\n".join(n for n in lines if not n.startswith("M190")) + "\n"

        moves = self.run_qtplasmac(job=marks.marked_job, declared=declared, change=without_material_changes)
        self.assertGreater(moves, 60)

    def test_wrong_plasma_words_are_found(self):
        # The same program with one block changed each time: a torch-off
        # removed (a rapid with the torch on), height control switched the
        # wrong way, and the feed-hold word left out.
        for old, new, reported in (
            ("M5 $0\n", "", "spindle at motion"),
            ("M63 P2\n", "M62 P2\n", "outputs at motion"),
            ("M52 P1\n", "", "adaptive_feed at motion"),
        ):
            with self.subTest(old=old, new=new):

                def change(text):
                    self.assertIn(old, text)
                    return text.replace(old, new, 1)

                with self.assertRaisesRegex(AssertionError, reported):
                    self.run_qtplasmac(change=change, **self.FIXED_HEIGHT)

    def test_a_block_linuxcnc_rejects_fails_the_comparison(self):
        with self.assertRaisesRegex(AssertionError, "rejected the program"):
            linuxcnc.interpret("G21\nG1 X1 F100\nG999\nM2\n")

    def test_a_move_off_the_plan_is_found(self):
        # The same job with one coordinate of the program changed by 0.01 mm.
        ex = example("standalone_job")
        job = ex.build_job()
        machine = ex.machine_for(job)
        setup = job.setups[0]
        with TemporaryDirectory(prefix="camlib-linuxcnc-") as folder, job.post("top", machine=machine) as posted:
            posted.wait()
            path = Path(folder) / "program.ngc"
            posted.export(path, accept={"unchecked_machine"})
            lines = path.read_text().splitlines()
            at = next(i for i, line in enumerate(lines) if line.startswith("G1 X27 Y3.428"))
            lines[at] = lines[at].replace("Y3.428", "Y3.438")
            with self.assertRaisesRegex(AssertionError, "mm off motion"):
                linuxcnc.compare(
                    self,
                    posted,
                    "\n".join(lines) + "\n",
                    operations_of(setup),
                    [7, 9],
                    setup.initial_position_mm,
                )


if __name__ == "__main__":
    unittest.main()
