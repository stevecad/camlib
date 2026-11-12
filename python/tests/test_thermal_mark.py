"""Plasma/laser marking through the Python API (#18): part numbers marked on
nested parts, then cut. The simulated sheet stays whole under the marks, the
order is refused when wrong, and every posted program is decoded back to
marking and cutting in order, with the machine's declared marking words."""

from dataclasses import replace
import math
import unittest

import camlib
from camlib.motion import Beam, BeamMark
from camlib.planar import PlanarRegion
from camlib.post import GCodeDialect, GCodePost, LoweringOptions, Machine, ToolSetting, post_job, post_job_input
from camlib.simulation import OperationUpdate, SimulationError, VerificationUpdate

from test_drill_cycles import example
from test_thermal import THICKNESS, parts

MARK_WIDTH = 0.8
MARK_HEIGHT = 2.5
CELL = 0.25


def line(a, b):
    return camlib.TracePath(a, (camlib.TraceLine(b),))


def numbers():
    """Part numbers as strokes: "7" on the plate (part 0), "1" and an arc on
    the washer nested in its hole (part 1)."""
    return (
        camlib.TracePath((26, 92), (camlib.TraceLine((34, 92)), camlib.TraceLine((29, 80)))),
        line((68, 57), (68, 63)),
        camlib.TracePath((50, 60), (camlib.TraceArc((60, 60), -70),)),
    )


def in_part(p, margin):
    """The part holding p with ``margin`` around it (analytic)."""
    washer = math.dist(p, (60, 60))
    if 5 + margin <= washer <= 15 - margin:
        return 1
    if (
        20 + margin <= p[0] <= 120 - margin
        and 20 + margin <= p[1] <= 100 - margin
        and washer >= 25 + margin
        and math.dist(p, (100, 40)) >= 6 + margin
        and math.dist(p, (100, 80)) >= 3 + margin
    ):
        return 0
    return None


def mark(tool, paths=None, *, power=100, **options):
    return camlib.ThermalMark(
        "numbers",
        tool,
        parts(),
        numbers() if paths is None else paths,
        4000,
        MARK_HEIGHT,
        power_percent=power,
        mark_width_mm=MARK_WIDTH,
        tool_number=1,
        length_register=1,
        **options,
    )


def cut(tool, *, power=100, **options):
    values = dict(lead_in=camlib.Lead(2, 3), lead_out=camlib.Lead(1, 2), overcut_mm=-0.5)
    values.update(options)
    return camlib.ThermalCut(
        "plasma",
        tool,
        parts(),
        2500,
        MARK_HEIGHT,
        pierce_delay_s=0,
        power_percent=power,
        tool_number=1,
        length_register=1,
        **values,
    )


def job(operations, tool, initial=(0, 0, 40)):
    setup = camlib.Setup(
        "sheet",
        stock=camlib.StockSpec((0, 0, 0), (200, 120, THICKNESS), CELL),
        tools=(tool,),
        operations=tuple(operations),
        initial_position_mm=initial,
        verification=camlib.StreamChecks(stock_contact=camlib.StockContactOptions()),
    )
    return camlib.Job("marked", "1", (setup,))


def marked_job(tool=None, *, mark_power=100, cut_power=100, **mark_options):
    tool = tool or camlib.Torch("torch", kerf_width=1.5)
    return job((mark(tool, power=mark_power, **mark_options), cut(tool, power=cut_power)), tool)


def setting(j, tool):
    header = j.compile().setup("sheet").program_header()
    return ToolSetting.for_tool(tool, number=1, length_register=1, position_after_mm=header.initial_position_mm)


DECLARED = dict(
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
    rapid_mm_min=6000,
)


def machine(j, dialect):
    tool = j.setups[0].tools[0]
    return Machine(
        "table",
        (setting(j, tool),),
        dialect=dialect,
        tool_change_position_mode="preserve_tip",
        flood=True,
        mist=True,
        **DECLARED,
    )


def post(j, m, **options):
    return j.post_nc(
        "sheet", machine=m, lowering=LoweringOptions(native_arcs=True), accept={"unchecked_machine"}, **options
    ).text.splitlines()


# The process codes of an imaginary controller, as its owner would declare
# them: camlib knows none.
MODE = GCodeDialect(beam_s_max=100, beam_mark="mode").with_commands(mark_mode="M101", cut_mode="M102")
POWER = GCodeDialect(beam_s_max=1000, beam_mark="power")


def run(j):
    """Stream the job: motions, event kinds, per-operation reports and stock
    volumes, verification statuses, and the final stock."""
    result = dict(motions=[], events=[], reports={}, removed={}, statuses=set())
    with j.regenerate() as r:
        for u in r:
            v = u.update
            if isinstance(v, VerificationUpdate):
                result["statuses"].add(v.status)
            elif isinstance(v, OperationUpdate):
                if v.stage == "events":
                    result["events"] += [(v.operation, e["kind"]) for e in v.events]
                if v.stage == "end":
                    result["reports"][v.operation] = v.report
                    result["removed"][v.operation] = r._simulation.state.volume()["removed_lower_mm3"]
            else:
                result["motions"].append(v.motion)
        result["heights"] = bytes(r.states["sheet"].heights)
    return result


def cell(heights, p):
    import struct

    nx = round(200 / CELL)
    index = int(p[1] / CELL) * nx + int(p[0] / CELL)
    return struct.unpack_from("<dd", heights, 16 * index)


def point(curve, t):
    start, end = curve["start"], curve["end"]
    if curve["kind"] == "line":
        return tuple(a + t * (b - a) for a, b in zip(start, end))
    c = curve["center"]
    r = math.dist(start[:2], c[:2])
    a = math.atan2(start[1] - c[1], start[0] - c[0]) + t * curve["sweep_rad"]
    return (c[0] + r * math.cos(a), c[1] + r * math.sin(a), start[2])


class ThermalMarkTest(unittest.TestCase):
    def test_part_numbers_are_marked_then_cut_and_the_sheet_stays_whole_under_them(self):
        result = run(marked_job(mark_power=15))
        self.assertEqual(result["statuses"], {"clear"})
        report = result["reports"]["numbers"]
        self.assertEqual(report["strategy"], "thermal_mark")
        self.assertEqual(report["parts"], [0, 1, 1])
        self.assertEqual(report["containment"], "proven")
        self.assertEqual(report["paths_completed"], 3)
        self.assertAlmostEqual(report["mark_length_mm"], 8 + 13 + 6 + 10 * math.radians(70))
        # Marking removed no stock; the cut did.
        self.assertEqual(result["removed"]["numbers"], 0)
        self.assertGreater(result["removed"]["plasma"], 1000)
        # One marking beam per path, each turned off, before any cutting beam.
        beams = [(op, kind) for op, kind in result["events"] if kind in ("beam", "beam_mark")]
        self.assertEqual(beams[:6], [("numbers", "beam_mark"), ("numbers", "beam")] * 3)
        self.assertTrue(all(b == ("plasma", "beam") for b in beams[6:]))
        # Every mark precedes every cut, and is a move of kind "mark".
        kinds = [(m["operation"], m["kind"]) for m in result["motions"]]
        marks = [m for m in result["motions"] if m["kind"] == "mark"]
        self.assertEqual(len(marks), 4)
        last_mark = max(i for i, k in enumerate(kinds) if k[1] == "mark")
        first_cut = min(i for i, k in enumerate(kinds) if k[1] in ("cut", "lead", "plunge"))
        self.assertLess(last_mark, first_cut)
        self.assertEqual({k for op, k in kinds if op == "numbers"}, {"rapid", "mark"})
        # Independent of the planner: every mark lies in its part by half its
        # width, at the marking height and feed, and the cut left the sheet
        # whole under it.
        for m in marks:
            self.assertEqual(m["feed_mm_min"], 4000)
            owner = in_part(point(m["curve"], 0.5)[:2], MARK_WIDTH / 2)
            self.assertIsNotNone(owner)
            for k in range(101):
                p = point(m["curve"], k / 100)
                self.assertEqual(p[2], THICKNESS + MARK_HEIGHT)
                self.assertEqual(in_part(p[:2], MARK_WIDTH / 2), owner, p)
                self.assertEqual(cell(result["heights"], p), (THICKNESS, THICKNESS), p)
        # The kerf beside the plate's left edge went through.
        self.assertEqual(cell(result["heights"], (20 - 0.75, 60)), (0, 0))

    def test_marking_after_the_cut_is_refused(self):
        tool = camlib.Torch("torch", kerf_width=1.5)
        with self.assertRaisesRegex(SimulationError, "part 0 is already cut.*before the cut"):
            run(job((cut(tool), mark(tool)), tool))

    def test_a_mark_off_its_part_is_refused_unless_asked(self):
        tool = camlib.Torch("torch", kerf_width=1.5)
        for path, message in (
            (line((98, 40), (102, 40)), "outside every part"),  # in the bolt hole
            (line((25, 60), (15, 60)), "not proven inside part 0"),  # across the edge
            (line((70, 60), (90, 60)), "not proven inside part 1"),  # washer to plate
        ):
            with self.assertRaisesRegex(SimulationError, message + ".*allow_off_part"):
                run(job((mark(tool, (path,)),), tool))
            result = run(job((mark(tool, (path,), allow_off_part=True),), tool))
            self.assertEqual(result["reports"]["numbers"]["containment"], "not_checked")
            self.assertEqual(result["removed"]["numbers"], 0)
        # Off the sheet: the stock model itself refuses the move.
        with self.assertRaisesRegex(SimulationError, "leaves the sheet"):
            run(job((mark(tool, (line((190, 60), (205, 60)),), allow_off_part=True),), tool))
        # A milling cutter cannot mark.
        mill = camlib.ToolAssembly(
            "mill",
            camlib.EndMill("mill", diameter=6, flute_length=20, overall_length=60),
            30,
            45,
            (camlib.HolderSection(30, 45, 12, 12),),
        )
        with self.assertRaisesRegex(SimulationError, "beam tool"):
            run(job((mark(mill),), mill))

    def test_a_declared_marking_process_is_written_and_decoded(self):
        """beam_mark="mode": the controller's own process codes."""
        j = marked_job(mark_power=15)
        nc = post(j, machine(j, MODE))
        nc = nc[nc.index("M101") :]  # after the program's preamble
        beams = [n for n in nc if n in ("M101", "M102", "M5") or n.startswith("M3 ")]
        # The marking process once, three marks at 15 %, then the cutting
        # process before the first cut, and only cuts at 100 % after it.
        self.assertEqual(beams[:7], ["M101", "M3 S15", "M5", "M3 S15", "M5", "M3 S15", "M5"])
        self.assertEqual(beams[7:9], ["M102", "M3 S100"])
        self.assertEqual((nc.count("M101"), nc.count("M102"), nc.count("M3 S15"), nc.count("M3 S100")), (1, 1, 3, 6))
        # A program that only cuts still states the cutting process first:
        # the controller may have been left in its marking process.
        tool = j.setups[0].tools[0]
        only_cut = job((cut(tool),), tool)
        nc = post(only_cut, machine(only_cut, MODE))
        self.assertLess(nc.index("M102"), nc.index("M3 S100"))
        self.assertNotIn("M101", nc)

        # The decoder does not trust the post. A post that fires the marking
        # beam without selecting the marking process, one that selects the
        # cutting process for a mark, and one that leaves the marking process
        # on for the cuts are all refused.
        class NoProcess(GCodePost):
            def on_beam_mark(self, event):
                d = self.machine.dialect
                self.emit(d.code(d.beam_effect), self.word(d.speed_word, event.power_percent))

        class WrongProcess(GCodePost):
            def on_beam_mark(self, event):
                d = self.machine.dialect
                self.emit(d.code("cut_mode"))
                self.emit(d.code(d.beam_effect), self.word(d.speed_word, event.power_percent))

        class NotRestored(GCodePost):
            def on_beam(self, event):
                d = self.machine.dialect
                if event.power_percent == 0:
                    return super().on_beam(event)
                self.emit(d.code(d.beam_effect), self.word(d.speed_word, event.power_percent))

        class FullPower(GCodePost):
            def on_beam_mark(self, event):
                super().on_beam_mark(replace(event, power_percent=100))

        for broken, message in (
            (NoProcess, "before its process"),
            (WrongProcess, "decoded effects differ"),
            (NotRestored, "decoded effects differ"),
            (FullPower, "decoded effects differ"),
        ):
            with self.assertRaisesRegex(Exception, message):
                post(j, machine(j, MODE), post=broken)

    def test_a_laser_marks_with_its_power_word(self):
        """beam_mark="power": the same beam word at the marking power."""
        laser = camlib.LaserHead("laser", kerf_width=0.2)
        j = job(
            (
                mark(laser, power=12),
                cut(laser, power=80, lead_in=camlib.Lead(1, 1), lead_out=camlib.Lead(0.5, 0.5)),
            ),
            laser,
        )
        nc = post(j, machine(j, POWER))
        beams = [n for n in nc if n.startswith("M3 ")]
        self.assertEqual(beams, ["M3 S120"] * 3 + ["M3 S800"] * 6)

        # The power words are decoded and compared: a mark at the cutting
        # power, or a cut at the marking power, is refused.
        class MarkTooStrong(GCodePost):
            def on_beam_mark(self, event):
                super().on_beam_mark(replace(event, power_percent=80))

        class CutTooWeak(GCodePost):
            def on_beam(self, event):
                super().on_beam(replace(event, power_percent=12) if event.power_percent else event)

        for broken in (MarkTooStrong, CutTooWeak):
            with self.assertRaisesRegex(Exception, "decoded effects differ"):
                post(j, machine(j, POWER), post=broken)

    def test_a_machine_that_declares_no_marking_refuses(self):
        j = marked_job()
        with self.assertRaisesRegex(Exception, "declares no beam marking"):
            post(j, machine(j, GCodeDialect(beam_s_max=1)))
        # The declaration itself is checked.
        with self.assertRaisesRegex(ValueError, "mark_mode=..., cut_mode="):
            machine(j, GCodeDialect(beam_s_max=1, beam_mark="mode"))
        with self.assertRaisesRegex(ValueError, "mark_mode=..., cut_mode="):
            machine(j, GCodeDialect(beam_s_max=1, beam_mark="mode").with_commands(mark_mode="M101"))
        with self.assertRaisesRegex(ValueError, "process codes of beam_mark='mode'"):
            GCodeDialect(beam_s_max=1, beam_mark="power").with_commands(mark_mode="M101", cut_mode="M102")
        with self.assertRaisesRegex(ValueError, "beam_mark must be"):
            GCodeDialect(beam_s_max=1, beam_mark="scribe")
        with self.assertRaisesRegex(ValueError, "needs a declared beam"):
            GCodeDialect(beam_mark="power")
        # An undeclared marking setting does not change a machine's record.
        m = machine(j, GCodeDialect(beam_s_max=1))
        self.assertNotIn("beam_mark", m.to_dict()["dialect"])
        self.assertEqual(machine(j, POWER).to_dict()["dialect"]["beam_mark"], "power")

    def test_typed_program_rules_reach_python_posting(self):
        """A cut under the marking beam never reaches a controller."""
        from camlib.motion import Line, Motion, Program, ToolChange, WorkOffset
        from camlib.post import post_program

        tool = camlib.Torch("torch", kerf_width=1.5)
        j = marked_job(tool)
        a, b = (10.0, 10.0, 8.0), (30.0, 10.0, 8.0)

        def program(*events):
            return Program(
                "p",
                "1",
                (tool,),
                (0.0, 0.0, 40.0),
                (
                    WorkOffset(1),
                    ToolChange("torch", 1, 1, (0.0, 0.0, 40.0)),
                    Motion(1, "op", "torch", Line((0.0, 0.0, 40.0), a), None, "rapid"),
                    *events,
                ),
            )

        def text(p):
            return "".join(chunk.text for chunk in post_program(p, machine=m)).splitlines()

        m = machine(j, MODE)
        good = program(BeamMark(20), Motion(2, "op", "torch", Line(a, b), 1000, "mark"), Beam(0))
        nc = text(good)
        self.assertEqual(nc[nc.index("M101") + 1], "M3 S20")
        for events, message in (
            ((BeamMark(20), Motion(2, "op", "torch", Line(a, b), 1000, "cut")), "only marking motions are fed"),
            ((Beam(100), Motion(2, "op", "torch", Line(a, b), 1000, "mark")), "marking motion requires"),
            # (The decoder refuses the process code written with the beam on.)
            ((BeamMark(20), Beam(100)), "the beam off|turn it off before firing the cutting beam"),
        ):
            with self.assertRaisesRegex(Exception, message):
                text(program(*events))

    def test_the_posted_replay_refuses_a_mark_over_cut_sheet(self):
        """The decoded program is replayed on the stock: a mark across a
        kerf fails there, whatever planned it."""
        from tempfile import TemporaryDirectory

        from camlib.motion import Line, Motion, Program, ToolChange, WorkOffset
        from camlib.posted import PostedRun

        tool = camlib.Torch("torch", kerf_width=1.5)
        j = marked_job(tool)
        z = THICKNESS + MARK_HEIGHT
        start = (0.0, 0.0, 40.0)
        a, b = (40.0, 50.0, z), (80.0, 50.0, z)

        def replay(mark_from, mark_to):
            program = Program(
                "p",
                "1",
                (tool,),
                start,
                (
                    WorkOffset(1),
                    ToolChange("torch", 1, 1, start),
                    Motion(1, "op", "torch", Line(start, a), None, "rapid"),
                    Beam(100),
                    Motion(2, "op", "torch", Line(a, b), 1000, "cut"),
                    Beam(0),
                    Motion(3, "op", "torch", Line(b, mark_from), None, "rapid"),
                    BeamMark(20),
                    Motion(4, "op", "torch", Line(mark_from, mark_to), 1000, "mark"),
                    Beam(0),
                ),
            )
            stock = camlib.StockSpec((0, 0, 0), (200, 120, THICKNESS), CELL)
            with TemporaryDirectory() as directory:
                with PostedRun(program, machine=machine(j, MODE), stock=stock, directory=directory) as run:
                    run.wait()
                    return run.posting_status()

        self.assertEqual(replay((40.0, 60.0, z), (80.0, 60.0, z))["checked_motions"], 4)
        with self.assertRaisesRegex(SimulationError, "marking move 4 .* cut through"):
            replay((60.0, 40.0, z), (60.0, 60.0, z))

    def test_rust_post_writes_the_same_marking_programs(self):
        for j, dialect in (
            (marked_job(mark_power=15), MODE),
            (marked_job(mark_power=15), POWER),
        ):
            m = machine(j, dialect)
            python = post(j, m)
            rust = post_job(
                post_job_input(
                    j, "sheet", machine=m, accept=("unchecked_machine",), lowering=LoweringOptions(native_arcs=True)
                )
            )
            self.assertTrue(rust["released"], rust.get("refused"))
            self.assertEqual(rust["program"].splitlines(), python)
        # And refuses the same undeclared machine.
        j = marked_job()
        with self.assertRaisesRegex(Exception, "declares no beam marking"):
            post_job(
                post_job_input(
                    j,
                    "sheet",
                    machine=machine(j, GCodeDialect(beam_s_max=1)),
                    accept=("unchecked_machine",),
                    lowering=LoweringOptions(native_arcs=True),
                )
            )

    def test_grbl_laser_marks_by_power_when_declared(self):
        from camlib.post import post_events
        from camlib.posted import OperationEvents
        from camlib.simulation import Simulation

        grbl = example("grbl_post")
        laser = camlib.LaserHead("laser", kerf_width=0.2)
        j = job(
            (
                mark(laser, power=15),
                cut(laser, power=80, lead_in=camlib.Lead(1, 1), lead_out=camlib.Lead(0.5, 0.5)),
            ),
            laser,
        )
        setup = j.compile().setup("sheet")
        header = setup.program_header()

        def text(**declared):
            m = grbl.define_machine(
                "GRBL laser",
                ToolSetting.for_tool(laser, number=1, length_register=1, position_after_mm=header.initial_position_mm),
                work_offset_machine_mm=(0, 0, 0),
                initial_machine_mm=header.initial_position_mm,
                beam_s_max=1000,
                output_units="mm",
                decimals=6,
                max_feed_mm_min=10000,
                max_spindle_rpm=1000,
                rapid_mm_min=10000,
                flood=False,
                mist=False,
                **declared,
            )
            source = OperationEvents(Simulation(**setup.simulation_options(None)), reuse_active_tool=True)
            try:
                chunks = post_events(
                    header,
                    source,
                    machine=m,
                    post=grbl.GrblPost,
                    decoder=grbl.GrblDecoder,
                    lowering=LoweringOptions(chord_tolerance_mm=0.001),
                )
                return "".join(chunk.text for chunk in chunks).splitlines()
            finally:
                source.close()

        nc = text(beam_mark="power")
        beams = [n for n in nc if n.startswith("M4 ")]
        self.assertEqual(beams, ["M4 S150"] * 3 + ["M4 S800"] * 6)
        with self.assertRaisesRegex(Exception, "declares no beam marking"):
            text()
        with self.assertRaisesRegex(ValueError, "GRBL has no marking process"):
            text(beam_mark="mode")

    def test_qtplasmac_marks_with_declared_materials(self):
        q = example("qtplasmac_post")
        tool = camlib.Torch("torch", kerf_width=1.5)
        j = marked_job(tool)

        def table(**declared):
            return q.define_machine("table", setting(j, tool), **declared, **DECLARED)

        options = dict(post=q.QtPlasmaCPost, decoder=q.QtPlasmaCDecoder)
        m = table(mark_material=7, cut_material=3, material_wait_s=2)
        nc = post(j, m, **options)
        process = [n for n in nc if n.startswith(("M190", "M66", "M3 ")) or n == "M5 $0"]
        # The marking material, in the documented order, before the marks;
        # the cutting material again before the first cut.
        self.assertEqual(process[:3], ["M190 P7", "M66 P3 L3 Q2", "M3 $0 S1"])
        self.assertEqual(process[3:8], ["M5 $0", "M3 $0 S1", "M5 $0", "M3 $0 S1", "M5 $0"])
        self.assertEqual(process[8:11], ["M190 P3", "M66 P3 L3 Q2", "M3 $0 S1"])
        self.assertEqual((nc.count("M190 P7"), nc.count("M190 P3"), nc.count("M3 $0 S1")), (1, 1, 9))
        # The first fed move after each material change states its feed.
        for change, feed in (("M190 P7", "F4000"), ("M190 P3", "F2500")):
            after = nc[nc.index(change) :]
            first = next(n for n in after if n.startswith(("G1", "G2", "G3")))
            self.assertIn(feed, first.split())

        # Read back from the material blocks alone: a post that leaves the
        # marking material selected for the cuts is refused.
        class NotRestored(q.QtPlasmaCPost):
            def on_beam(self, event):
                self.emit("M5 $0" if event.power_percent == 0 else "M3 $0 S1")

        class NoWait(q.QtPlasmaCPost):
            def _select_material(self, number):
                if getattr(self, "_material", None) != number:
                    self.emit(f"M190 P{number}")
                    self._material = number

        for broken, message in ((NotRestored, "decoded effects differ"), (NoWait, "followed by its wait")):
            with self.assertRaisesRegex(Exception, message):
                post(j, m, post=broken, decoder=q.QtPlasmaCDecoder)
        # No declared materials: marking is refused, and a cut-only program
        # is written as before, without material blocks.
        with self.assertRaisesRegex(Exception, "declares no marking"):
            post(j, table(), **options)
        only_cut = job((cut(tool),), tool)
        nc = post(only_cut, q.define_machine("table", setting(only_cut, tool), **DECLARED), **options)
        self.assertFalse([n for n in nc if n.startswith(("M190", "M66"))])
        with self.assertRaisesRegex(ValueError, "together"):
            table(mark_material=7)
        with self.assertRaisesRegex(ValueError, "together"):
            table(mark_material=3, cut_material=3, material_wait_s=2)
        # The torch fires at the material's current: no power word to scale.
        weak = marked_job(tool, mark_power=15)
        with self.assertRaisesRegex(Exception, "power_percent=100"):
            post(weak, m, **options)

    def test_plasma_posts_without_a_marking_method_refuse(self):
        """Centroid and WinCNC: no marking method is declared for them."""
        tool = camlib.Torch("torch", kerf_width=1.5)
        j = job((mark(tool), cut(tool)), tool, initial=(0, 0, THICKNESS + MARK_HEIGHT))
        for name, post_class, decoder in (
            ("centroid_plasma_post", "CentroidPlasmaPost", "CentroidPlasmaDecoder"),
            ("wincnc_plasma_post", "WinCNCPlasmaPost", "WinCNCPlasmaDecoder"),
        ):
            module = example(name)
            declared = {k: v for k, v in DECLARED.items() if k != "planes"}
            if name.startswith("wincnc"):
                declared.pop("max_sweep_rad")
            m = module.define_machine("table", setting(j, tool), **declared)
            with self.assertRaisesRegex(Exception, "declares no marking method"):
                post(j, m, post=getattr(module, post_class), decoder=getattr(module, decoder))


class ThermalMarkDefinitionTest(unittest.TestCase):
    def test_definition_and_recipe_scope(self):
        tool = camlib.Torch("torch", kerf_width=1.5)
        data = mark(tool).to_dict()
        self.assertEqual(data["kind"], "thermal_mark")
        self.assertEqual(data["tool"], "torch")
        self.assertNotIn("travel_height_mm", data)
        self.assertEqual(len(data["paths"]), 3)
        self.assertIsInstance(parts(), PlanarRegion)
        # Nothing about the machine is assumed: the marking values are required.
        with self.assertRaises(TypeError):
            camlib.ThermalMark("m", tool, parts(), numbers(), 4000, 2.5, tool_number=1, length_register=1)


if __name__ == "__main__":
    unittest.main()
