"""Canned drilling cycles: one block per hole where the machine declares the
cycle, decoded back into the controller's moves and checked against the plan;
explicit moves elsewhere."""

from camlib.postprocessor import GCodeDialect

import importlib.util
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest

import camlib
from camlib import RegenerationCache
from camlib.motion import WorkFrame
from camlib.post import Machine, ToolSetting
from camlib.simulation import OperationUpdate
from test_tapping import posted_text, tools

ROOT = Path(__file__).resolve().parents[2]
HOLES = ((8, 8), (20, 8))


def job(**drilling):
    drill, _ = tools()
    values = dict(canned_cycle=True, reentry_clearance_mm=0.254, chip_break_lift_mm=0.254)
    values.update(drilling)
    op = camlib.Drilling(
        "holes",
        drill,
        HOLES,
        4,
        12,
        **{
            'feed_mm_min': 150,
            'retract_feed_mm_min': 600,
            'spindle_rpm': 6000,
            'tool_number': 1,
            'length_register': 1,
            'coolant': "flood",
            **values,
        },
    )
    setup = camlib.Setup(
        "top",
        stock=camlib.StockSpec((0, 0, 0), (28, 16, 10), 0.1),
        tools=(drill,),
        operations=(camlib.OperationNode(op, sources=()),),
        initial_position_mm=(0, 0, 12),
        verification=camlib.StreamChecks(stock_contact=camlib.StockContactOptions()),
    )
    return camlib.Job("cycles", "1", (setup,))


def tool_table(j):
    header = j.compile().setup("top").program_header()
    return tuple(
        ToolSetting.for_tool(t, number=i, length_register=i, position_after_mm=header.initial_position_mm)
        for i, t in enumerate(header.tools, 1)
    )


def linuxcnc(j, dialect=None):
    return Machine(
        "mill",
        tool_table(j),
        tool_change_position_mode="preserve_tip",
        dialect=dialect or GCodeDialect.linuxcnc(),
        output_units='mm',
        decimals=3,
        max_feed_mm_min=10000,
        max_spindle_rpm=30000,
        planes=('xy', 'xz', 'yz'),
        arcs=True,
        helices=True,
        max_sweep_rad=6.283185307179586,
        max_tool_number=99999,
        max_length_register=99999,
        tool_changes=True,
        flood=True,
        mist=True,
        rapid_mm_min=6000,
    )


def example(name):
    spec = importlib.util.spec_from_file_location(name, ROOT / f"examples/{name}.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def cycle_lines(nc):
    lines = [line for line in nc if line.split(" ")[0] in ("G73", "G81", "G82", "G83", "G98", "G80")]
    # The program header cancels cycles (G80) once at the start.
    assert lines[0] == "G80", lines
    return lines[1:]


class DrillCycleTest(unittest.TestCase):
    def test_linuxcnc_writes_one_block_per_hole(self):
        for drilling, block in (
            (dict(), "G81 X{x} Y8 Z4 R11 F150"),
            (dict(dwell_s=0.5), "G82 X{x} Y8 Z4 R11 P0.5 F150"),
            (dict(peck_mm=1.5), "G83 X{x} Y8 Z4 R11 Q1.5 F150"),
            (dict(peck_mm=1.5, peck_mode="chip_break"), "G73 X{x} Y8 Z4 R11 Q1.5 F150"),
        ):
            j = job(**drilling)
            nc = posted_text(j, linuxcnc(j))
            self.assertEqual(cycle_lines(nc), ["G98", block.format(x=8), block.format(x=20), "G80"], drilling)
            # Only the lift, traverse and final retract remain as explicit moves.
            self.assertFalse(any(line.startswith(("G0 ", "G1 ")) for line in nc), nc)

    def test_cycles_post_with_every_lowering_mode(self):
        # A canned cycle's block carries its hole's moves: checked in one
        # batch with them under lowering too (issue #77), decoded back and
        # compared with the plan move by move.
        from camlib.post import LoweringOptions

        modes = ((), (LoweringOptions(),), (LoweringOptions(native_arcs=True),))
        for drilling, code in ((dict(), "G81"), (dict(peck_mm=1.5), "G83")):
            j = job(**drilling)
            expected = None
            for mode in modes:
                options = dict(lowering=mode[0]) if mode else {}
                with j.post("top", machine=linuxcnc(j), **options) as posted:
                    posted.wait()
                    status, posting = posted.status(), posted.posting_status()
                    self.assertIsNone(status["error"], (code, mode))
                    self.assertTrue(posting["finished"], (code, mode))
                    # Every planned move was checked against the posted text.
                    self.assertEqual(posting["checked_motions"], status["available"], (code, mode))
                    for index in range(status["available"]):
                        proof = posted.posted_motion(index)
                        self.assertTrue(proof["stock_committed"])
                    nc, start = [], 1
                    while page := posted.nc_lines(start, limit=256):
                        nc.extend(line["text"] for line in page)
                        start = page[-1]["number"] + 1
                blocks = [line for line in cycle_lines(nc) if line.startswith(code)]
                self.assertEqual(len(blocks), len(HOLES), (code, mode, nc))
                # The same program whatever the lowering (no arcs to lower).
                if expected is None:
                    expected = nc
                self.assertEqual(nc, expected, (code, mode))

    def test_return_amount_must_match_the_controller(self):
        j = job(peck_mm=1.5, reentry_clearance_mm=0.5)
        with self.assertRaisesRegex(Exception, "reentry_clearance_mm 0.5 mm differs .* set reentry_clearance_mm=0.254"):
            posted_text(j, linuxcnc(j))

    def test_undeclared_cycles_are_written_as_explicit_moves(self):
        j = job(peck_mm=1.5)
        no_peck = GCodeDialect.linuxcnc().with_commands(peck_cycle=None)
        nc = posted_text(j, linuxcnc(j, dialect=no_peck))
        self.assertEqual(cycle_lines(nc), [])
        self.assertTrue(any(line.startswith("G1 X8 Y8 Z9.5") for line in nc), nc)

    def haas(self, j, **settings):
        haas = example("haas_ngc_post")
        m = haas.define_machine(
            "Haas",
            tool_table(j),
            traverse_feed_mm_min=5000,
            coordinate_system="fanuc",
            h_t_agreement=True,
            g92_shift_mm=(0, 0, 0),
            rotation_scaling=False,
            work_to_machine=WorkFrame(),
            decimals=3,
            tool_change_position_mode="preserve_tip",
            output_units='mm',
            max_feed_mm_min=10000,
            max_spindle_rpm=30000,
            max_sweep_rad=6.283185307179586,
            tool_changes=True,
            flood=True,
            rapid_mm_min=6000,
            **settings,
        )
        return haas, m

    def test_haas_canned_cycles(self):
        """#13: Haas NGC writes G81/G82/G83/G73 (96-8210 Rev N), decoded back
        into the controller's moves and checked against the plan."""
        for drilling, block in (
            (dict(), "G81 X{x}. Y8. Z4. R11. F150."),
            (dict(dwell_s=0.5), "G82 X{x}. Y8. Z4. R11. P0.5 F150."),
            (dict(peck_mm=1.5), "G83 X{x}. Y8. Z4. R11. Q1.5 F150."),
            (dict(peck_mm=1.5, peck_mode="chip_break"), "G73 X{x}. Y8. Z4. R11. Q1.5 F150."),
        ):
            with self.subTest(drilling):
                j = job(**drilling)
                haas, m = self.haas(j, setting_22_mm=0.254, setting_52_mm=0)
                nc = posted_text(j, m, haas.HaasNGCPost, haas.HaasNGCDecoder)
                self.assertEqual(cycle_lines(nc), ["G98", block.format(x=8), block.format(x=20), "G80"])

    def test_haas_pecking_needs_its_settings(self):
        j = job(peck_mm=1.5)
        haas, m = self.haas(j)
        with self.assertRaisesRegex(Exception, "G83 needs Setting 22 \\(Can Cycle Delta Z\\)"):
            posted_text(j, m, haas.HaasNGCPost, haas.HaasNGCDecoder)
        haas, m = self.haas(j, setting_22_mm=0.254)
        with self.assertRaisesRegex(Exception, "G83 needs Setting 52"):
            posted_text(j, m, haas.HaasNGCPost, haas.HaasNGCDecoder)
        haas, m = self.haas(j, setting_22_mm=0.254, setting_52_mm=2)
        with self.assertRaisesRegex(Exception, "Setting 52 = 2 mm clears chips above R"):
            posted_text(j, m, haas.HaasNGCPost, haas.HaasNGCDecoder)
        chip = job(peck_mm=1.5, peck_mode="chip_break")
        haas, m = self.haas(chip)
        with self.assertRaisesRegex(Exception, "G73 needs Setting 22"):
            posted_text(chip, m, haas.HaasNGCPost, haas.HaasNGCDecoder)
        # The operation's return amount must be the installed Setting 22.
        haas, m = self.haas(j, setting_22_mm=0.5, setting_52_mm=0)
        with self.assertRaisesRegex(Exception, "differs from Setting 22 .* set reentry_clearance_mm=0.5"):
            posted_text(j, m, haas.HaasNGCPost, haas.HaasNGCDecoder)

    def test_haas_decoder_reads_undotted_p_as_milliseconds(self):
        j = job(dwell_s=0.5)
        haas, m = self.haas(j)
        decoder = haas.HaasNGCDecoder(m, j.compile().setup("top").program_header())
        decoder.read_line("%")
        decoder.read_line(f"O{m.program_number}")
        with self.assertRaisesRegex(ValueError, "without a decimal point is milliseconds"):
            decoder.read_line("G82 X8. Y8. Z4. R11. P500 F150.", None)

    def test_centroid_and_fanuc_cycles(self):
        centroid = example("centroid_post")
        j = job(peck_mm=1.5, dwell_s=None)
        tools = tool_table(j)
        m = centroid.define_machine(
            "Acorn",
            tools,
            traverse_feed_mm_min=3000,
            work_to_machine=WorkFrame(),
            decimals=3,
            peck_clearance_mm=0.254,
            output_units='mm',
            max_feed_mm_min=10000,
            max_spindle_rpm=30000,
            planes=('xy', 'xz', 'yz'),
            arcs=True,
            helices=True,
            max_sweep_rad=6.283185307179586,
            max_tool_number=99999,
            max_length_register=99999,
            tool_changes=True,
            flood=True,
            mist=True,
            rapid_mm_min=6000,
        )
        nc = posted_text(j, m, centroid.CentroidPost, centroid.CentroidDecoder)
        self.assertEqual(cycle_lines(nc), ["G98", "G83 X8 Y8 Z4 R11 Q1.5 F150", "G83 X20 Y8 Z4 R11 Q1.5 F150", "G80"])
        # Without Parameter 83 declared, G83 is not written.
        plain = centroid.define_machine(
            "Acorn",
            tools,
            traverse_feed_mm_min=3000,
            work_to_machine=WorkFrame(),
            decimals=3,
            output_units='mm',
            max_feed_mm_min=10000,
            max_spindle_rpm=30000,
            planes=('xy', 'xz', 'yz'),
            arcs=True,
            helices=True,
            max_sweep_rad=6.283185307179586,
            max_tool_number=99999,
            max_length_register=99999,
            tool_changes=True,
            flood=True,
            mist=True,
            rapid_mm_min=6000,
        )
        self.assertEqual(cycle_lines(posted_text(j, plain, centroid.CentroidPost, centroid.CentroidDecoder)), [])
        fanuc = example("fanuc_0imb_post")
        oem = dict(tool_change='M6', spindle_cw='M3', spindle_ccw='M4', spindle_off='M5', coolant_off='M9', flood='M8')
        f = fanuc.define_machine(
            "FANUC",
            tools,
            startup_units="mm",
            startup_standard_modes=True,
            g92_shift_mm=(0, 0, 0),
            m_codes=oem,
            traverse_feed_mm_min=3000,
            work_to_machine=WorkFrame(),
            decimals=3,
            tool_change_position_mode="table",
            output_units='mm',
            max_feed_mm_min=10000,
            max_spindle_rpm=30000,
            max_sweep_rad=6.283185307179586,
            tool_changes=True,
            flood=True,
            rapid_mm_min=6000,
        )
        nc = posted_text(job(dwell_s=0.25), f, fanuc.Fanuc0iMBPost, fanuc.Fanuc0iMBDecoder)
        self.assertEqual(
            cycle_lines(nc), ["G98", "G82 X8. Y8. Z4. R11. P250 F150.", "G82 X20. Y8. Z4. R11. P250 F150.", "G80"]
        )

    def test_post_nc_and_cached_regeneration(self):
        j = job(peck_mm=1.5)
        self.assertEqual(j.post_nc("top", machine=linuxcnc(j), accept={"unchecked_machine"}).text.count("G83 "), 2)
        with TemporaryDirectory() as directory:
            cache = RegenerationCache(directory)
            for hit in (False, True):
                with j.regenerate(cache) as run:
                    updates = [u.update for u in run]
                ops = [u for u in updates if isinstance(u, OperationUpdate)]
                self.assertEqual(ops[-1].report["regeneration"]["cache_hit"], hit)
                cycles = [e for u in ops if u.stage == "events" for e in u.events if e["kind"] == "cycle"]
                self.assertEqual([c["center_mm"] for c in cycles], [[8, 8], [20, 8]])
                # The first cycle is announced before any motion.
                self.assertEqual(ops[1].stage, "events")
                self.assertEqual(ops[1].completed, ops[0].completed)


def boring_job(**reaming):
    drill, _ = tools()
    holder = (camlib.HolderSection(35, 60, 6, 6),)
    reamer = camlib.ToolAssembly(
        "ream52",
        camlib.EndMill("r52", diameter=5.2, flute_length=20, overall_length=60, center_cutting=False),
        35,
        60,
        holder,
    )
    values = dict(retract_z_mm=11, canned_cycle=True, tool_number=2, length_register=2)
    values.update(reaming)
    ops = (
        camlib.Drilling(
            "drill",
            drill,
            HOLES,
            0.5,
            12,
            tool_number=1,
            length_register=1,
            feed_mm_min=150,
            retract_feed_mm_min=600,
            spindle_rpm=6000,
            coolant="flood",
        ),
        camlib.Reaming(
            "bore",
            reamer,
            HOLES,
            5,
            4,
            12,
            **{
                'feed_mm_min': 100,
                'retract_feed_mm_min': 100,
                'spindle_rpm': 1000,
                'tool_number': 1,
                'length_register': 1,
                'coolant': "flood",
                **values,
            },
        ),
    )
    setup = camlib.Setup(
        "top",
        stock=camlib.StockSpec((0, 0, 0), (28, 16, 10), 0.1),
        tools=(drill, reamer),
        operations=tuple(camlib.OperationNode(op, sources=()) for op in ops),
        initial_position_mm=(0, 0, 12),
        verification=camlib.StreamChecks(stock_contact=camlib.StockContactOptions()),
    )
    return camlib.Job("boring", "1", (setup,))


class BoringCycleTest(unittest.TestCase):
    def test_linuxcnc_boring_cycles(self):
        for reaming, block in (
            (dict(), "G85 X{x} Y8 Z4 R11 F100"),
            (dict(dwell_s=0.4), "G89 X{x} Y8 Z4 R11 P0.4 F100"),
            (dict(spindle_stop=True), "G86 X{x} Y8 Z4 R11 F100"),
        ):
            j = boring_job(**reaming)
            nc = posted_text(j, linuxcnc(j))
            bores = [line for line in nc if line.startswith(("G85", "G86", "G89"))]
            self.assertEqual(bores, [block.format(x=8), block.format(x=20)], reaming)
            # The spindle stop/restart of G86 is part of the cycle block.
            first = nc.index(bores[0])
            self.assertFalse(any(line.startswith(("M5", "M3")) for line in nc[first : nc.index("G80", first)]))

    def test_haas_boring_cycles(self):
        """#13: Haas G85/G89 (96-8210 Rev N); G86 is not declared (the manual
        does not say the spindle restarts), so a stop-boring hole is written
        as explicit moves."""
        for reaming, block in (
            (dict(), "G85 X{x}. Y8. Z4. R11. F100."),
            (dict(dwell_s=0.4), "G89 X{x}. Y8. Z4. R11. P0.4 F100."),
        ):
            with self.subTest(reaming):
                j = boring_job(**reaming)
                haas, m = DrillCycleTest.haas(None, j)
                nc = posted_text(j, m, haas.HaasNGCPost, haas.HaasNGCDecoder)
                bores = [line for line in nc if line.startswith(("G85", "G86", "G89"))]
                self.assertEqual(bores, [block.format(x=8), block.format(x=20)])
        j = boring_job(spindle_stop=True)
        haas, m = DrillCycleTest.haas(None, j)
        nc = posted_text(j, m, haas.HaasNGCPost, haas.HaasNGCDecoder)
        self.assertFalse(any(line.startswith("G86") for line in nc))

    def test_every_check_is_clear_and_events_follow_the_controller(self):
        from camlib.simulation import VerificationUpdate

        for reaming in (dict(), dict(spindle_stop=True), dict(dwell_s=0.2)):
            statuses, spindles = set(), []
            with boring_job(**reaming).regenerate() as run:
                for u in run:
                    v = u.update
                    if isinstance(v, VerificationUpdate):
                        statuses.add(v.status)
                    elif isinstance(v, OperationUpdate) and v.operation == "bore" and v.stage == "events":
                        spindles += [e["rpm"] for e in v.events if e["kind"] == "spindle"]
            self.assertEqual(statuses, {"clear"}, reaming)
            self.assertEqual(spindles, [0, 1000, 0, 1000] if reaming.get("spindle_stop") else [], reaming)

    def test_centroid_writes_g85_and_expands_g86(self):
        centroid = example("centroid_post")
        j = boring_job()
        header = j.compile().setup("top").program_header()
        # CNC12 M6 moves Z home: the table states each tool's tip pose after
        # M6. Here: the drill at the start, the reamer above the last hole.
        tools = tuple(
            ToolSetting.for_tool(t, number=i, length_register=i, position_after_mm=pose)
            for i, (t, pose) in enumerate(zip(header.tools, (header.initial_position_mm, (20, 8, 12))), 1)
        )
        m = centroid.define_machine(
            "Acorn",
            tools,
            traverse_feed_mm_min=3000,
            work_to_machine=WorkFrame(),
            decimals=3,
            output_units='mm',
            max_feed_mm_min=10000,
            max_spindle_rpm=30000,
            planes=('xy', 'xz', 'yz'),
            arcs=True,
            helices=True,
            max_sweep_rad=6.283185307179586,
            max_tool_number=99999,
            max_length_register=99999,
            tool_changes=True,
            flood=True,
            mist=True,
            rapid_mm_min=6000,
        )
        nc = posted_text(j, m, centroid.CentroidPost, centroid.CentroidDecoder)
        self.assertEqual(
            [line for line in nc if line.startswith("G85")], ["G85 X8 Y8 Z4 R11 F100", "G85 X20 Y8 Z4 R11 F100"]
        )
        j = boring_job(spindle_stop=True)
        nc = posted_text(j, m, centroid.CentroidPost, centroid.CentroidDecoder)
        self.assertFalse(any(line.startswith("G86") for line in nc))
        self.assertGreaterEqual(nc.count("M5"), 2)

    def test_canned_boring_needs_r_at_the_travel_height(self):
        with self.assertRaisesRegex(Exception, "set feed_z_mm"):
            with boring_job(feed_z_mm=10.5).regenerate() as run:
                list(run)


class ToolChangePoseTest(unittest.TestCase):
    def test_m6_z_home_is_planned_and_posted_without_table_poses(self):
        from dataclasses import replace

        centroid = example("centroid_post")
        j = boring_job()
        # M6 sends the gauge line to machine Z 80; work zero is machine Z 0.
        setup = replace(j.setups[0], tool_change=camlib.ToolChangePosition(gauge_z_mm=80))
        j = camlib.Job("boring", "1", (setup,))
        header = j.compile().setup("top").program_header()
        tools = tuple(
            ToolSetting.for_tool(t, number=i, length_register=i, position_after_mm=(0, 0, 0))
            for i, t in enumerate(header.tools, 1)
        )
        m = centroid.define_machine(
            "Acorn",
            tools,
            traverse_feed_mm_min=3000,
            work_to_machine=WorkFrame(),
            decimals=3,
            tool_change_position_mode="z_home",
            tool_change_home_z_mm=80,
            output_units='mm',
            max_feed_mm_min=10000,
            max_spindle_rpm=30000,
            planes=('xy', 'xz', 'yz'),
            arcs=True,
            helices=True,
            max_sweep_rad=6.283185307179586,
            max_tool_number=99999,
            max_length_register=99999,
            tool_changes=True,
            flood=True,
            mist=True,
            rapid_mm_min=6000,
        )
        nc = posted_text(j, m, centroid.CentroidPost, centroid.CentroidDecoder)
        # Each tool comes down from Z home (80 - gauge 60 = tip Z 20).
        for tool in ("T1", "T2"):
            at = nc.index(tool)
            self.assertTrue(
                any(line.endswith("Z12 F3000") or " Z12 " in line for line in nc[at : at + 6]), nc[at : at + 6]
            )
        from camlib.simulation import OperationUpdate

        with j.regenerate() as run:
            begins = [u.update for u in run if isinstance(u.update, OperationUpdate) and u.update.stage == "begin"]
        poses = [e["position_after_mm"] for b in begins for e in b.events if e["kind"] == "tool_change"]
        self.assertEqual(poses, [[0, 0, 20], [20, 8, 20]])


if __name__ == "__main__":
    unittest.main()
