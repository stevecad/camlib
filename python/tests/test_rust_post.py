"""The Rust default post (issue #120): for the same job and machine it writes
byte-for-byte the program the Python default post writes, decodes it
independently, replays it with the same checks and applies the same export
gate. The machine and dialect cross as ``Machine.to_dict()`` JSON."""

from dataclasses import replace
import importlib.util
import json
import math
from pathlib import Path
import subprocess
import unittest

from camlib._buffers import native
from camlib.export import ExportRefused
from camlib.motion import (
    Arc,
    Coolant,
    Cycle,
    CutterCompensation,
    Dwell,
    FeedPerRev,
    Line,
    Motion,
    Program,
    Spindle,
    Stop,
    SurfaceSpeed,
    ToolChange,
    WorkOffset,
)
from camlib.post import GCodeDialect, LoweringOptions, Machine, ToolSetting, post_job, post_job_input, post_program

ROOT = Path(__file__).resolve().parents[2]


def example(name):
    spec = importlib.util.spec_from_file_location(name, ROOT / "examples" / f"{name}.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def python_release(job, machine, accept, setup="top", **options):
    """What Job.post_nc releases (or refuses) with the Python default post."""
    try:
        p = job.post_nc(setup, machine=machine, accept=accept, **options)
    except ExportRefused as refused:
        return dict(released=False, statuses=list(refused.statuses), findings=refused.findings)
    return dict(
        released=True,
        text=p.text,
        lines=p.lines,
        motions=p.motions,
        findings=p.findings,
        accepted=list(p.accepted),
        unchecked=list(p.unchecked),
        machine=p.machine,
    )


def plain(value):
    return json.loads(json.dumps(value))


def summary(findings):
    return [(f["completed"], f["status"], len(f["findings"])) for f in findings]


class RustPostTest(unittest.TestCase):
    def assert_same(self, job, machine, accept=("unchecked_machine",), setup="top", lowering=None):
        options = {} if lowering is None else dict(lowering=lowering)
        python = python_release(job, machine, set(accept), setup, **options)
        rust = post_job(post_job_input(job, setup, machine=machine, accept=accept, lowering=lowering))
        self.assertEqual(rust["released"], python["released"], rust.get("refused"))
        if not python["released"]:
            self.assertIsNone(rust["program"])
            self.assertEqual(rust["refused"]["statuses"], python["statuses"])
            self.assertEqual(summary(rust["findings"]), summary(plain(python["findings"])))
            return rust
        # The same bytes reach the machine.
        self.assertEqual(rust["program"], python["text"])
        self.assertEqual((rust["lines"], rust["motions"]), (python["lines"], python["motions"]))
        self.assertEqual(rust["accepted"], python["accepted"])
        self.assertEqual(rust["unchecked"], python["unchecked"])
        self.assertEqual(rust["findings"], plain(python["findings"]))
        self.assertEqual(rust["machine"]["status"], python["machine"]["status"])
        self.assertEqual(summary_machine(rust["machine"]), summary_machine(plain(python["machine"])))
        return rust

    def test_standalone_example(self):
        ex = example("standalone_job")
        job = ex.build_job()
        rust = self.assert_same(job, ex.machine_for(job))
        self.assertIn("G43 H9", rust["program"])
        self.assertEqual(rust["unchecked"], ["scene", "material", "machine"])
        # Without accepting the missing machine model, nothing is released.
        refused = self.assert_same(job, ex.machine_for(job), accept=())
        self.assertEqual(refused["refused"]["statuses"], ["unchecked_machine"])
        self.assertIn("export refused", refused["refused"]["message"])

    def test_standalone_rest_example(self):
        ex = example("standalone_job")
        job = ex.build_rest_job()
        rust = self.assert_same(job, ex.rest_machine_for(job))
        self.assertIn("T8", rust["program"])
        # tests/rust_post.rs checks the Rust post against this program
        # without Python; it must stay the Python post's.
        self.assertEqual(rust["program"], (ROOT / "fixtures/posts/rust/standalone-rest.nc").read_bytes().decode())

    @unittest.skipUnless(importlib.util.find_spec("cadquery"), "needs CadQuery")
    def test_cadquery_example(self):
        ex = example("cadquery_job")
        job = ex.build_job()
        self.assert_same(job, ex.machine_for(job))

    def test_lowered_arcs_inch_and_traverse_feed(self):
        ex = example("standalone_job")
        job = ex.build_job()
        machine = ex.machine_for(job)
        for lowering in (LoweringOptions(native_arcs=True), LoweringOptions()):
            rust = self.assert_same(job, machine, lowering=lowering)
            self.assertIn("arc_output", rust["posting"])
        self.assert_same(job, replace(machine, output_units="inch", decimals=4))
        rust = self.assert_same(job, replace(machine, traverse_feed_mm_min=5000))
        self.assertNotIn("\nG0 ", rust["program"])

    def test_canned_drilling_and_boring_cycles(self):
        from test_drill_cycles import boring_job, job, linuxcnc

        for drilling, code in (
            (dict(), "G81 "),
            (dict(dwell_s=0.5), "G82 "),
            (dict(peck_mm=1.5), "G83 "),
            (dict(peck_mm=1.5, peck_mode="chip_break"), "G73 "),
            (dict(peck_mm=1.5, canned_cycle=False), "G1 X8 Y8 Z8.5 F150"),
        ):
            j = job(**drilling)
            rust = self.assert_same(j, linuxcnc(j))
            self.assertIn(code, rust["program"], drilling)
            rust = self.assert_same(j, linuxcnc(j), lowering=LoweringOptions(native_arcs=True))
            self.assertIn(code, rust["program"], drilling)
        for reaming, code in ((dict(), "G85 "), (dict(dwell_s=0.4), "G89 "), (dict(spindle_stop=True), "G86 ")):
            j = boring_job(**reaming)
            rust = self.assert_same(j, linuxcnc(j))
            self.assertIn(code, rust["program"], reaming)
        # Undeclared cycles are written as explicit moves.
        j = job(peck_mm=1.5)
        rust = self.assert_same(j, linuxcnc(j, dialect=GCodeDialect.linuxcnc().with_commands(peck_cycle=None)))
        self.assertNotIn("G83", rust["program"])

    def test_fine_boring_with_an_oriented_stop(self):
        from test_drill_cycles import linuxcnc
        from test_fine_boring import job

        dialect = GCodeDialect.linuxcnc(oriented_edge_deg=0).with_commands(spindle_orient="M19", fine_bore_cycle="G76")
        for canned in (True, False):
            j = job(canned_cycle=canned)
            rust = self.assert_same(j, linuxcnc(j, dialect=dialect))
            self.assertEqual("G76 " in rust["program"], canned)
            self.assertEqual("M19" in rust["program"], not canned)

    def test_tapping_cycles(self):
        from test_tapping import job, machine, tapping

        j = job(tapping)
        rust = self.assert_same(j, machine(j))
        self.assertIn("G33.1 Z4 K1", rust["program"])
        # A data-only dialect with a canned G84 cycle instead.
        g84 = GCodeDialect.linuxcnc().with_commands(tap_cycle="G84", tap_synchronized=None)
        rust = self.assert_same(j, machine(j, dialect=g84))
        self.assertIn("G84 ", rust["program"])

    def test_cutter_compensation(self):
        from test_cutter_compensation import job, machine

        j = job(compensation="wear")
        rust = self.assert_same(j, machine(j))
        self.assertIn("G41 D7", rust["program"])
        # Turned on and off in the move block (FANUC/Haas style), as data.
        m = machine(j)
        self.assert_same(j, replace(m, dialect=replace(m.dialect, compensation_with_move=True)))

    def test_plasma_beam_and_height_control(self):
        from test_thermal import job, table

        j = job()
        dialect = GCodeDialect(beam_s_max=1, height_control_word="P2").with_commands(
            height_control_on="M63", height_control_off="M62"
        )
        m = Machine(
            "plasma",
            table(j),
            dialect=dialect,
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
        rust = self.assert_same(j, m, setup="sheet", lowering=LoweringOptions(native_arcs=True))
        self.assertEqual(rust["program"].splitlines().count("M63 P2"), 5)

    def test_declared_machine_model(self):
        from test_machine import model

        ex = example("standalone_job")
        job = ex.build_job()
        machine = ex.machine_for(job)
        header = job.compile().setup("top").program_header()
        declared = model(header, machine)
        p = header.initial_position_mm
        declared = replace(declared, initial_axes_mm=(p[0], p[1], p[2] + header.tools[0].gauge_length_mm))
        rust = self.assert_same(job, replace(machine, crash_model=declared), accept=())
        self.assertEqual(rust["machine"]["status"], "clear")
        self.assertNotIn("machine", rust["unchecked"])
        self.assertEqual(rust["machine"]["decoded_motions"], rust["motions"])

    def test_machine_json_is_the_posted_definition(self):
        ex = example("standalone_job")
        job = ex.build_job()
        machine = ex.machine_for(job)
        data = machine.to_dict()
        with job.post("top", machine=machine) as posted:
            posted.wait()
            self.assertEqual(plain(posted.posting_manifest()["definition"]["machine"]), plain(data))
        rust = post_job(post_job_input(job, "top", machine=machine, accept={"unchecked_machine"}))
        self.assertEqual(rust["posting"]["definition"]["machine"], plain(data))
        # Every field is stated: a missing one is refused, as is an invalid one.
        source = post_job_input(job, "top", machine=machine)
        for change, message in (
            (lambda m: m.pop("decimals"), "missing field `decimals`"),
            (lambda m: m["dialect"].pop("tap_pitch_word"), "missing field `tap_pitch_word`"),
            (lambda m: m.update(decimals=1), "invalid machine output units/precision"),
            (lambda m: m.update(tolerance_mm=0.2), "tolerance_mm must be finite and in [1e-9, .1]"),
            (lambda m: m["dialect"].update(axes=["X", "Y", "X"]), "distinct uppercase letters"),
            (lambda m: m.update(unknown=1), "unknown field `unknown`"),
        ):
            data = json.loads(json.dumps(source))
            change(data["machine"])
            with self.assertRaises(ValueError) as error:
                post_job(data)
            self.assertIn(message, str(error.exception))

    def test_refusals_name_the_problem(self):
        ex = example("standalone_job")
        job = ex.build_job()
        machine = ex.machine_for(job)
        data = post_job_input(job, "top", machine=machine)
        for change, message in (
            (dict(accept=["everything"]), "accept takes"),
            (dict(frame=dict(origin_mm=[1, 0, 0], x=[1, 0, 0], y=[0, 1, 0], z=[0, 0, 1])), "frame differ"),
        ):
            with self.assertRaises(ValueError) as error:
                post_job({**data, **change})
            self.assertIn(message, str(error.exception))
        scheduled = dict(data, machine=dict(data["machine"], schedule_post_feeds=True, motion_policy={}))
        with self.assertRaises(ValueError) as error:
            post_job(scheduled)
        self.assertIn("schedule_post_feeds", str(error.exception))
        # A tool-change pose the machine does not produce is refused before planning.
        moved = replace(machine, tools=(replace(machine.tools[0], position_after_mm=(1, 2, 30)),))
        with self.assertRaises(ValueError) as error:
            post_job(post_job_input(job, "top", machine=moved))
        self.assertIn("ToolChangeTable.from_machine", str(error.exception))

    def test_probing_cycles_post_byte_for_byte(self):
        # Probing (#7, #147): every cycle, LinuxCNC G38.x and Inspection Plus
        # macros on the default post, written and decoded by the Rust post
        # exactly as by the Python post.
        import camlib
        import test_probing as probing
        from test_drill_cycles import linuxcnc

        values = {**probing.COMMON, **probing.FEEDS}
        macro = {**probing.COMMON, **probing.FANUC_FEEDS}
        diameter = camlib.ToolSetDiameter(
            "diameter",
            probing.MILL,
            setter_center_mm=(60, 15),
            setter_top_z_mm=2,
            setter_diameter_mm=10,
            depth_mm=3,
            clearance_mm=4,
            clearance_z_mm=20,
            search_mm=1,
            tool_number=1,
            length_register=1,
            measure_feed_mm_min=50,
            return_feed_mm_min=500,
            spindle_rpm=800,
            spindle_direction="ccw",
        )
        g38_cases = (
            (probing.corner(),),
            (probing.bore(),),
            # Two cycles in one program: distinct O-word labels.
            (probing.corner(), probing.bore()),
            (camlib.ProbeSurface("s", probing.PROBE, start_mm=(-8, 15, 5), axis="x", surface_mm=0, **values),),
            (camlib.ProbeSurface("z", probing.PROBE, start_mm=(20, 15, 15), axis="z", surface_mm=10, **values),),
            (
                camlib.ProbeInsideCorner(
                    "i", probing.PROBE, corner_mm=(40, 30), z_mm=5, material=(1, 1), clearance_mm=5, **values
                ),
            ),
            (
                camlib.ProbeBoss(
                    "b", probing.PROBE, center_mm=(20, 15), z_mm=5, diameter_mm=12, clearance_mm=4, **values
                ),
            ),
            (camlib.ProbePocket("p", probing.PROBE, center_mm=(20, 15), z_mm=5, axis="y", width_mm=10, **values),),
            (
                camlib.ProbeWeb(
                    "w", probing.PROBE, center_mm=(20, 15), z_mm=5, axis="x", width_mm=8, clearance_mm=4, **values
                ),
            ),
            (probing.tool_length(),),
            (diameter,),
        )
        # The synthetic stock is a solid block: a stylus inside a bore,
        # pocket or web touches it (stock-contact violations). Accepting them
        # releases every program, so its bytes are compared.
        released = ("unchecked_machine", "violation")
        for operations in g38_cases:
            with self.subTest(g38=[op.id for op in operations]):
                j = probing.job(*operations)
                self.assert_same(j, probing.g38(j), accept=released)
        # Refused alike, listing the same unaccepted findings.
        j = probing.job(probing.bore())
        self.assertFalse(self.assert_same(j, probing.g38(j))["released"])
        z_macro = {**macro, "return_feed_mm_min": 1200}
        macro_cases = (
            (probing.corner(**probing.FANUC_FEEDS), False),
            (probing.bore(**probing.FANUC_FEEDS), True),
            (camlib.ProbeSurface("s", probing.PROBE, start_mm=(-8, 15, 5), axis="x", surface_mm=0, **macro), False),
            (
                camlib.ProbeSurface("z", probing.PROBE, start_mm=(20, 15, 15), axis="z", surface_mm=10, **z_macro),
                False,
            ),
            (
                camlib.ProbeInsideCorner(
                    "i", probing.PROBE, corner_mm=(40, 30), z_mm=5, material=(1, 1), clearance_mm=5, **macro
                ),
                False,
            ),
            (
                camlib.ProbePocket("p", probing.PROBE, center_mm=(20, 15), z_mm=5, axis="y", width_mm=10, **macro),
                True,
            ),
        )
        for op, switching in macro_cases:
            with self.subTest(inspection_plus=op.id, switching=switching):
                j = probing.job(op)
                dialect = GCodeDialect.linuxcnc(
                    probing="inspection_plus", probe_fast_feed_mm_min=2000, probe_switching=switching
                )
                rust = self.assert_same(j, linuxcnc(j, dialect), accept=released)
                self.assertIn("G65 P98", rust["program"])

    def test_probing_refusals_name_the_same_problem(self):
        import camlib
        import test_probing as probing
        from test_drill_cycles import linuxcnc

        macro = GCodeDialect.linuxcnc(probing="inspection_plus", probe_fast_feed_mm_min=2000)
        boss = camlib.ProbeBoss(
            "b",
            probing.PROBE,
            center_mm=(20, 15),
            z_mm=5,
            diameter_mm=12,
            clearance_mm=4,
            **{**probing.COMMON, **probing.FANUC_FEEDS},
        )
        for op, dialect, message in (
            (probing.corner(), None, "declares no probing back end"),
            (boss, macro, "not modelled for the Inspection Plus macros"),
            (probing.corner(), macro, r"0.6 x the fast feed #119 \(1200 mm/min\)"),
        ):
            with self.subTest(message=message):
                j = probing.job(op)
                machine = linuxcnc(j, dialect)
                with self.assertRaisesRegex(Exception, message):
                    j.post_nc("top", machine=machine, accept={"unchecked_machine"})
                with self.assertRaisesRegex(ValueError, message):
                    post_job(post_job_input(j, "top", machine=machine, accept=("unchecked_machine",)))

    @unittest.skipUnless((ROOT / "target/release/cam-post").is_file(), "needs the cam-post build")
    def test_cam_post_command(self):
        ex = example("standalone_job")
        job = ex.build_job()
        data = post_job_input(job, "top", machine=ex.machine_for(job), accept={"unchecked_machine"})
        result = subprocess.run(
            [str(ROOT / "target/release/cam-post")], input=json.dumps(data), capture_output=True, text=True, check=True
        )
        self.assertEqual(json.loads(result.stdout), post_job(data))
        failed = subprocess.run([str(ROOT / "target/release/cam-post")], input="{}", capture_output=True, text=True)
        self.assertEqual(failed.returncode, 1)
        self.assertIn("cam-post: post job input", failed.stderr)


def summary_machine(report):
    return [(f.get("status"), f.get("kind"), tuple(f.get("bodies", ()))) for f in report.get("findings", ())]


def mill(tools, dialect=None, **settings):
    values = dict(
        dialect=dialect or GCodeDialect.linuxcnc(),
        output_units="mm",
        decimals=4,
        max_feed_mm_min=10000,
        max_spindle_rpm=3000,
        planes=("xy", "xz", "yz"),
        arcs=True,
        helices=True,
        max_sweep_rad=math.tau,
        max_tool_number=99,
        max_length_register=99,
        tool_changes=True,
        flood=True,
        mist=True,
        rapid_mm_min=6000,
        tool_change_position_mode="table",
    )
    values.update(settings)
    return Machine("explicit", tools, **values)


def rust_program(program, machine, lowering=None):
    data = dict(program=program.to_dict(), machine=machine.to_dict())
    if lowering is not None:
        data["lowering"] = dict(
            chord_tolerance_mm=lowering.chord_tolerance_mm,
            max_segments_per_source=lowering.max_segments_per_source,
            max_total_segments=lowering.max_total_segments,
            native_arcs=lowering.native_arcs,
        )
    return json.loads(native.post_program(json.dumps(data)))


class RustPostEventsTest(unittest.TestCase):
    """Explicit programs (post_program): hooks a milling job does not reach."""

    def assert_same(self, program, machine, lowering=None):
        python = "".join(chunk.text for chunk in post_program(program, machine, lowering=lowering))
        rust = rust_program(program, machine, lowering)
        self.assertEqual(rust["program"], python)
        return rust["program"]

    def test_controls_stops_coolant_and_dwell(self):
        from test_post import fixture

        p, m = fixture()
        self.assert_same(p, m)
        t = p.tools[0]
        events = (
            WorkOffset(1),
            ToolChange("t1", 2, 7, (5, 0, 3)),
            Spindle(1200, "ccw"),
            Coolant("mist"),
            Coolant("flood"),
            Motion(1, "op", "t1", Line((5, 0, 3), (5, 0, 1)), 300, "plunge"),
            Dwell(0.25),
            Motion(2, "op", "t1", Line((5, 0, 1), (5, 0, 3)), None, "rapid"),
            Stop(False),
            Stop(True),
            Spindle(0),
            Coolant("off"),
        )
        program = replace(p, events=events)
        machine = mill((ToolSetting.for_tool(t, number=2, length_register=7, position_after_mm=(5, 0, 3)),))
        text = self.assert_same(program, machine)
        self.assertIn("G4 P0.25", text)
        self.assertIn("M4 S1200", text)

    def test_canned_cycles_select_g17_after_an_xz_or_yz_arc(self):
        """#183: cycles drill along Z only in G17 (in G18 LinuxCNC drills along Y)."""
        from camlib.post import NativeGCodeDecoder
        from test_post import fixture

        p, m = fixture()
        start = (5, 0, 3)
        for plane, end, centre, sweep in (
            ("xz", (10, 0, 8), (10, 0, 3), math.pi / 2),
            ("yz", (5, 5, 8), (5, 5, 3), -math.pi / 2),
        ):
            for cycle, extra, code in (
                ("drill", {}, "G81"),
                ("dwell", dict(dwell_s=0.5), "G82"),
                ("bore", {}, "G85"),
            ):
                with self.subTest(plane=plane, cycle=code):
                    x, y, z = end
                    arc = Motion(1, "op", "t1", Arc(start, end, centre, sweep, plane), 1200)
                    if cycle == "bore":
                        # Boring cycles start at the R plane (initial level = R).
                        moves = (
                            Motion(2, "op", "t1", Line(end, (x, y, 5)), None, "rapid"),
                            Cycle("bore", (x, y), 1, 5, 5, 100, 2),
                            Motion(3, "drill", "t1", Line((x, y, 5), (x, y, 1)), 100),
                            Motion(4, "drill", "t1", Line((x, y, 1), (x, y, 5)), 100),
                        )
                    else:
                        covers = (
                            Motion(2, "drill", "t1", Line(end, (x, y, 5)), None, "rapid"),
                            Motion(3, "drill", "t1", Line((x, y, 5), (x, y, 1)), 150),
                            *((Dwell(0.5),) if extra else ()),
                            Motion(4, "drill", "t1", Line((x, y, 1), end), None, "rapid"),
                        )
                        moves = (Cycle(cycle, (x, y), 1, 5, 8, 150, len(covers), **extra), *covers)
                    events = (*p.events[:4], arc, *moves, Coolant("off"), Spindle(0))
                    program = replace(p, events=events)
                    text = self.assert_same(program, m).splitlines()
                    i = next(k for k, line in enumerate(text) if line.startswith(code))
                    self.assertEqual(text[i - 2 : i], ["G17", "G98"])
                    broken = text[: i - 2] + text[i - 1 :]
                    decoder = NativeGCodeDecoder(m, program)
                    with self.assertRaisesRegex(ValueError, "G17"):
                        for line in broken:
                            decoder.read_line(line, dict(id=1, operation="file", kind="cut"))

    def test_a_plane_is_selected_once_and_again_only_when_it_changes(self):
        """#204: plane selection is modal (G17/G18/G19, modal group 2), so the
        post writes a plane code only for a change, and the decoder still has
        the plane of every arc from the program alone."""
        from camlib.post import GCodePost, NativeGCodeDecoder
        from test_post import fixture

        p, m = fixture()
        q = math.pi / 2
        curves = (
            Arc((5, 0, 3), (0, 5, 3), (0, 0, 3), q, "xy"),
            Arc((0, 5, 3), (5, 0, 3), (0, 0, 3), -q, "xy"),
            Arc((5, 0, 3), (10, 0, 8), (10, 0, 3), q, "xz"),
            Arc((10, 0, 8), (5, 0, 3), (10, 0, 3), -q, "xz"),
            Arc((5, 0, 3), (5, 5, 8), (5, 5, 3), -q, "yz"),
            Arc((5, 5, 8), (5, 0, 3), (5, 5, 3), q, "yz"),
            Line((5, 0, 3), (6, 0, 3)),
            Arc((6, 0, 3), (0, 6, 3), (0, 0, 3), q, "xy"),
        )
        moves = tuple(Motion(i + 1, "op", "t1", c, 1200) for i, c in enumerate(curves))
        program = replace(p, events=(*p.events[:4], *moves, Coolant("off"), Spindle(0)))
        text = self.assert_same(program, m)
        lines = text.splitlines()
        # The header's G17 covers the first two arcs; then each change, once.
        self.assertEqual([x for x in lines if x in ("G17", "G18", "G19")], ["G17", "G18", "G19", "G17"], text)
        planes = [c.plane for c in curves if isinstance(c, Arc)]
        self.assertEqual(sum(x.startswith(("G2 ", "G3 ")) for x in lines), len(planes))

        # The same blocks through the Python hooks (a subclass is not taken by
        # the native batch formatter).
        class Hooked(GCodePost):
            def on_motion(self, motion):
                super().on_motion(motion)

        self.assertEqual("".join(chunk.text for chunk in post_program(program, m, post=Hooked)), text)

        def decoded(lines):
            decoder = NativeGCodeDecoder(m, program)
            events = [e for x in lines for e in decoder.read_line(x, dict(id=1, operation="file", kind="cut"))]
            return [e.curve.plane for e in events if isinstance(e, Motion) and isinstance(e.curve, Arc)]

        self.assertEqual(decoded(lines), planes)
        # Without a plane code that changes the plane, the arcs after it are
        # not the planned ones: the block is refused or read in the old plane.
        changes = [i for i, x in enumerate(lines) if x in ("G17", "G18", "G19")][1:]
        self.assertEqual(len(changes), 3)
        for i in changes:
            with self.subTest(omitted=lines[i], line=i):
                try:
                    got = decoded(lines[:i] + lines[i + 1 :])
                except ValueError:
                    continue
                self.assertNotEqual(got, planes)

    def test_a_cycle_that_selected_g17_leaves_it_selected(self):
        """#204 with #183: the G17 a canned cycle needed after an XZ arc is the
        plane of the XY arc after it (no second G17); an XZ arc selects G18 again."""
        from test_post import fixture

        p, m = fixture()
        q = math.pi / 2
        covers = (
            Motion(2, "drill", "t1", Line((10, 0, 8), (10, 0, 5)), None, "rapid"),
            Motion(3, "drill", "t1", Line((10, 0, 5), (10, 0, 1)), 150),
            Motion(4, "drill", "t1", Line((10, 0, 1), (10, 0, 8)), None, "rapid"),
        )
        events = (
            *p.events[:4],
            Motion(1, "op", "t1", Arc((5, 0, 3), (10, 0, 8), (10, 0, 3), q, "xz"), 1200),
            Cycle("drill", (10, 0), 1, 5, 8, 150, len(covers)),
            *covers,
            Motion(5, "op", "t1", Arc((10, 0, 8), (0, 10, 8), (0, 0, 8), q, "xy"), 1200),
            Motion(6, "op", "t1", Arc((0, 10, 8), (5, 10, 3), (5, 10, 8), -q, "xz"), 1200),
            Coolant("off"),
            Spindle(0),
        )
        text = self.assert_same(replace(p, events=events), m)
        lines = text.splitlines()
        self.assertEqual([x for x in lines if x in ("G17", "G18", "G19")], ["G17", "G18", "G17", "G18"], text)
        self.assertEqual(sum(x.startswith(("G2 ", "G3 ")) for x in lines), 3)

    def test_lathe_words_as_data(self):
        """Diameter X, T<tt><oo>, constant surface speed, feed per revolution."""
        from test_post import fixture

        p, _ = fixture()
        t = p.tools[0]
        dialect = GCodeDialect.linuxcnc(
            x_diameter=True, tool_change="lathe_t", css_limit_word="D", compensation_register=False
        ).with_commands(css_on="G96", feed_per_rev="G95", lathe_diameter_mode="G7", tool_change=None, length_on=None)
        start = (20, 0, 5)
        face, out = Line((20, 0, 2), (12, 0, 2)), Line((12, 0, 2), (12, 0, 5))

        def per_rev(curve):
            # The feed bound of a feed per revolution move at constant surface speed.
            return 0.15 * native.css_rpm_bound(json.dumps(curve.to_dict()), 150, 2500)

        events = (
            WorkOffset(1),
            ToolChange("t1", 2, 3, start),
            SurfaceSpeed(150, 2500, "cw"),
            Coolant("flood"),
            Motion(1, "face", "t1", Line(start, (20, 0, 2)), 600, "plunge"),
            FeedPerRev(0.15),
            CutterCompensation("left", "xz"),
            Motion(2, "face", "t1", face, per_rev(face), "cut"),
            CutterCompensation("off", "xz"),
            Motion(3, "face", "t1", out, per_rev(out), "cut"),
            FeedPerRev(0),
            Spindle(800, "cw"),
            Motion(4, "face", "t1", Line((12, 0, 5), (20, 0, 5)), None, "rapid"),
            Spindle(0),
            Coolant("off"),
        )
        program = Program(p.id, p.revision, p.tools, start, events, tolerance_mm=p.tolerance_mm)
        machine = mill(
            (ToolSetting.for_tool(t, number=2, length_register=3, position_after_mm=start),),
            dialect=dialect,
            planes=("xz",),
        )
        text = self.assert_same(program, machine)
        self.assertIn("T0203", text)
        self.assertIn("G96 D2500 S150 M3", text)


if __name__ == "__main__":
    unittest.main()
