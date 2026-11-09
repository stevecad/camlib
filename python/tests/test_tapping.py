from camlib.postprocessor import GCodeDialect
import importlib.util
from pathlib import Path
import unittest

import camlib
from camlib.post import Machine, ToolSetting
from camlib.simulation import Simulation, SimulationError, ToolChangePosition
from camlib.tools import HolderSection, Tap, ToolAssembly

ROOT = Path(__file__).resolve().parents[2]
HOLES = ((8, 8), (20, 8))


def tools():
    holder = (HolderSection(35, 60, 6, 6),)
    drill = ToolAssembly("drill5", camlib.Drill("d5", diameter=5, flute_length=30, overall_length=60), 35, 60, holder)
    tap = ToolAssembly(
        "m6", Tap("m6-tap", drill_diameter=5, thread_length=12, overall_length=60, shank_diameter=6), 35, 60, holder
    )
    return drill, tap


def job(tap_op, drill_depth=0.5, tool_change=None):
    drill, tap = tools()
    ops = (
        camlib.Drilling(
            "drill",
            drill,
            HOLES,
            drill_depth,
            12,
            tool_number=1,
            length_register=1,
            feed_mm_min=150,
            retract_feed_mm_min=600,
            spindle_rpm=6000,
            coolant="flood",
        ),
        tap_op(tap),
    )
    setup = camlib.Setup(
        "top",
        stock=camlib.StockSpec((0, 0, 0), (28, 16, 10), 0.1),
        tools=(drill, tap),
        operations=tuple(camlib.OperationNode(op, sources=()) for op in ops),
        initial_position_mm=(0, 0, 12),
        verification=camlib.StreamChecks(stock_contact=camlib.StockContactOptions()),
        tool_change=tool_change,
    )
    return camlib.Job("tap", "1", (setup,))


def tapping(tap, **overrides):
    values = dict(
        pitch_mm=1, depth_z_mm=4, clearance_z_mm=12, spindle_rpm=600, feed_z_mm=11, tool_number=2, length_register=2
    )
    values.update(overrides)
    return camlib.Tapping(
        "tap", tap, HOLES, **{'spindle_rpm': 500, 'tool_number': 1, 'length_register': 1, 'coolant': "flood", **values}
    )


def machine(job, **settings):
    header = job.compile().setup("top").program_header()
    return Machine(
        "Three-axis mill",
        tuple(
            ToolSetting.for_tool(t, number=i, length_register=i, position_after_mm=header.initial_position_mm)
            for i, t in enumerate(header.tools, 1)
        ),
        tool_change_position_mode="preserve_tip",
        **{
            'dialect': GCodeDialect.linuxcnc(),
            'output_units': 'mm',
            'decimals': 3,
            'max_feed_mm_min': 10000,
            'max_spindle_rpm': 30000,
            'planes': ('xy', 'xz', 'yz'),
            'arcs': True,
            'helices': True,
            'max_sweep_rad': 6.283185307179586,
            'max_tool_number': 99999,
            'max_length_register': 99999,
            'tool_changes': True,
            'flood': True,
            'mist': True,
            'rapid_mm_min': 6000,
            **settings,
        },
    )


def posted_text(job, machine, post=None, decoder=None, **options):
    kwargs = dict(options) if post is None else dict(options, post=post, decoder=decoder)
    with job.post("top", machine=machine, **kwargs) as posted:
        posted.wait()
        nc, start = [], 1
        while page := posted.nc_lines(start, limit=256):
            nc.extend(line["text"] for line in page)
            start = page[-1]["number"] + 1
    return nc


class TappingTest(unittest.TestCase):
    def test_linuxcnc_g33_1_per_hole_with_checks(self):
        j = job(tapping)
        nc = posted_text(j, machine(j))
        cycles = [line for line in nc if line.startswith("G33.1")]
        self.assertEqual(cycles, ["G33.1 Z4 K1"] * 2)
        # Each cycle starts at the R plane above its hole.
        for i, line in enumerate(nc):
            if line.startswith("G33.1"):
                self.assertTrue(any("Z11" in nc[k] for k in range(i - 3, i)), nc[i - 3 : i + 1])

    def test_g84_cycle_is_cancelled_after_each_hole(self):
        spec = importlib.util.spec_from_file_location("haas", ROOT / "examples/haas_ngc_post.py")
        haas = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(haas)
        j = job(tapping)
        header = j.compile().setup("top").program_header()
        tools = tuple(
            ToolSetting.for_tool(t, number=i, length_register=i, position_after_mm=header.initial_position_mm)
            for i, t in enumerate(header.tools, 1)
        )
        from camlib.motion import WorkFrame

        m = haas.define_machine(
            "Haas",
            tools,
            traverse_feed_mm_min=5000,
            coordinate_system="fanuc",
            h_t_agreement=True,
            g92_shift_mm=(0, 0, 0),
            rotation_scaling=False,
            work_to_machine=WorkFrame(),
            decimals=3,
            tool_change_position_mode="preserve_tip",
            rigid_tapping=True,
            output_units='mm',
            max_feed_mm_min=10000,
            max_spindle_rpm=30000,
            max_sweep_rad=6.283185307179586,
            tool_changes=True,
            flood=True,
            rapid_mm_min=6000,
        )
        nc = posted_text(j, m, haas.HaasNGCPost, haas.HaasNGCDecoder)
        cycles = [i for i, line in enumerate(nc) if line.startswith("G84")]
        self.assertEqual([nc[i] for i in cycles], ["G84 X8. Y8. Z4. R11. F600.", "G84 X20. Y8. Z4. R11. F600."])
        for i in cycles:
            self.assertEqual(nc[i + 1], "G80")
        without = haas.define_machine(
            "Haas",
            tools,
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
        )
        with self.assertRaisesRegex(Exception, "declares no tapping cycle"):
            posted_text(j, without, haas.HaasNGCPost, haas.HaasNGCDecoder)

    def test_shallow_drill_is_refused_before_the_tap_goes_down(self):
        drill, tap = tools()
        config = dict(
            stock=camlib.StockSpec((0, 0, 0), (28, 16, 10), 0.1),
            tools=(drill, tap),
            operations=(
                camlib.Drilling(
                    "drill",
                    drill,
                    HOLES,
                    5,
                    12,
                    tool_number=1,
                    length_register=1,
                    feed_mm_min=150,
                    retract_feed_mm_min=600,
                    spindle_rpm=6000,
                    coolant="flood",
                ),
                tapping(tap),
            ),
            revision="1",
            initial_position_mm=(0, 0, 12),
        )
        with Simulation(**config) as sim:
            with self.assertRaisesRegex(SimulationError, "tapping hole 0: the stock is not proven open"):
                list(sim.run_operations())


def example(name):
    spec = importlib.util.spec_from_file_location(name, ROOT / f"examples/{name}.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


# Where M6 leaves each tool: gauge line at Z 100 over the work origin; the
# FANUC tool table declares the same tip poses (tool_change_position_mode
# "table").
CHANGE = ToolChangePosition(gauge_z_mm=100, xy_mm=(0, 0))


def tool_table(j):
    header = j.compile().setup("top").program_header()
    return tuple(
        ToolSetting.for_tool(t, number=i, length_register=i, position_after_mm=(0, 0, 100 - t.gauge_length_mm))
        for i, t in enumerate(header.tools, 1)
    )


def fanuc_machine(fanuc, j, **settings):
    oem = dict(tool_change="M6", spindle_cw="M3", spindle_ccw="M4", spindle_off="M5", coolant_off="M9", flood="M8")
    from camlib.motion import WorkFrame

    return fanuc.define_machine(
        "FANUC",
        tool_table(j),
        startup_units="mm",
        startup_standard_modes=True,
        g92_shift_mm=(0, 0, 0),
        m_codes=oem,
        traverse_feed_mm_min=3000,
        work_to_machine=WorkFrame(),
        decimals=3,
        tool_change_position_mode="table",
        output_units="mm",
        max_feed_mm_min=10000,
        max_spindle_rpm=30000,
        max_sweep_rad=6.283185307179586,
        tool_changes=True,
        flood=True,
        rapid_mm_min=6000,
        **settings,
    )


def spindle_before(nc, index):
    """The last spindle block (M3/M4/M5) before line ``index``."""
    return next(line for line in reversed(nc[:index]) if any(m in line.split() for m in ("M3", "M4", "M5")))


class LeftHandTappingTest(unittest.TestCase):
    """#10: Tapping(hand="left") turns the spindle counterclockwise into the
    hole; posts write a left-hand cycle and decoders read the hand back."""

    def test_hand_is_validated_and_serialized_only_when_left(self):
        drill, tap = tools()
        self.assertNotIn("hand", tapping(tap).to_dict())
        self.assertEqual(tapping(tap, hand="left").to_dict()["hand"], "left")
        with self.assertRaisesRegex(ValueError, "hand must be 'right' or 'left'"):
            tapping(tap, hand="LH")

    def test_linuxcnc_g33_1_under_m4(self):
        j = job(lambda tap: tapping(tap, hand="left"))
        nc = posted_text(j, machine(j))
        cycles = [i for i, line in enumerate(nc) if line.startswith("G33.1")]
        self.assertEqual([nc[i] for i in cycles], ["G33.1 Z4 K1"] * 2)
        for i in cycles:
            self.assertIn("M4", spindle_before(nc, i).split())
        right = posted_text(job(tapping), machine(j))
        i = next(i for i, line in enumerate(right) if line.startswith("G33.1"))
        self.assertIn("M3", spindle_before(right, i).split())

    def test_fanuc_rigid_g74_left_and_g84_right(self):
        fanuc = example("fanuc_0imb_post")
        for hand, code, spindle in (("left", "G74", "M4"), ("right", "G84", "M3")):
            with self.subTest(hand):
                j = job(lambda tap: tapping(tap, hand=hand), tool_change=CHANGE)
                m = fanuc_machine(fanuc, j, rigid_tapping=True)
                nc = j.post_nc(
                    "top",
                    machine=m,
                    post=fanuc.Fanuc0iMBPost,
                    decoder=fanuc.Fanuc0iMBDecoder,
                    accept={"unchecked_machine"},
                ).text.splitlines()
                cycles = [i for i, line in enumerate(nc) if line.startswith(("G74", "G84"))]
                # F = lead x S = 1 mm x 600 rpm (G94), R = the current Z.
                self.assertEqual(
                    [nc[i] for i in cycles], [f"{code} X8. Y8. Z4. R11. F600.", f"{code} X20. Y8. Z4. R11. F600."]
                )
                for i in cycles:
                    self.assertEqual(nc[i + 1], "G80")
                    self.assertIn(spindle, spindle_before(nc, i).split())

    def test_haas_g74_reverse_tap(self):
        from camlib.motion import WorkFrame

        haas = example("haas_ngc_post")
        j = job(lambda tap: tapping(tap, hand="left"))
        header = j.compile().setup("top").program_header()
        tools = tuple(
            ToolSetting.for_tool(t, number=i, length_register=i, position_after_mm=header.initial_position_mm)
            for i, t in enumerate(header.tools, 1)
        )
        m = haas.define_machine(
            "Haas",
            tools,
            traverse_feed_mm_min=5000,
            coordinate_system="fanuc",
            h_t_agreement=True,
            g92_shift_mm=(0, 0, 0),
            rotation_scaling=False,
            work_to_machine=WorkFrame(),
            decimals=3,
            tool_change_position_mode="preserve_tip",
            rigid_tapping=True,
            output_units="mm",
            max_feed_mm_min=10000,
            max_spindle_rpm=30000,
            max_sweep_rad=6.283185307179586,
            tool_changes=True,
            flood=True,
            rapid_mm_min=6000,
        )
        nc = posted_text(j, m, haas.HaasNGCPost, haas.HaasNGCDecoder)
        cycles = [i for i, line in enumerate(nc) if line.startswith(("G74", "G84"))]
        self.assertEqual([nc[i] for i in cycles], ["G74 X8. Y8. Z4. R11. F600.", "G74 X20. Y8. Z4. R11. F600."])
        for i in cycles:
            self.assertEqual(nc[i + 1], "G80")
            self.assertIn("M4", spindle_before(nc, i).split())

    def test_dialects_without_left_hand_tapping_refuse(self):
        fanuc = example("fanuc_0imb_post")
        j = job(lambda tap: tapping(tap, hand="left"), tool_change=CHANGE)
        # FANUC without rigid tapping declared: no tapping cycle at all.
        with self.assertRaisesRegex(Exception, "declares no left-hand tapping"):
            j.post_nc(
                "top",
                machine=fanuc_machine(fanuc, j),
                post=fanuc.Fanuc0iMBPost,
                decoder=fanuc.Fanuc0iMBDecoder,
                accept={"unchecked_machine"},
            )
        # A G84-only dialect (right-hand canned cycle) refuses a left-hand tap.
        j = job(lambda tap: tapping(tap, hand="left"))
        g84 = GCodeDialect.linuxcnc().with_commands(tap_synchronized=None, tap_cycle="G84")
        with self.assertRaisesRegex(Exception, "declares no left-hand tapping"):
            posted_text(j, machine(j, dialect=g84))

    def test_decoder_reads_the_hand_a_right_hand_block_cannot_pass_as_left(self):
        from camlib.post import GCodePost, NativeGCodeDecoder

        class WrongCycle(GCodePost):
            """Writes G84 (right hand) whatever the spindle direction."""

            def tap(self, motion):
                saved = self._spindle_direction
                self._spindle_direction = "cw"
                try:
                    super().tap(motion)
                finally:
                    self._spindle_direction = saved

        class WrongSpindle(GCodePost):
            """Writes M3 for a counterclockwise spindle event."""

            def on_spindle(self, event):
                from dataclasses import replace as _replace

                super().on_spindle(_replace(event, direction="cw") if event.rpm else event)
                if event.rpm:
                    self._spindle_direction = event.direction

        j = job(lambda tap: tapping(tap, hand="left"))
        both = GCodeDialect.linuxcnc().with_commands(tap_synchronized=None, tap_cycle="G84", tap_cycle_left="G74")
        # Control: the honest post writes G74 and passes.
        nc = posted_text(j, machine(j, dialect=both))
        self.assertEqual(sum(line.startswith("G74") for line in nc), 2)
        # G84 under the decoded M4: the decoder refuses the block.
        with self.assertRaisesRegex(Exception, "clockwise \\(M3\\) for G84"):
            posted_text(j, machine(j, dialect=both), WrongCycle, NativeGCodeDecoder)
        # M3 where the plan turns counterclockwise: the decoded spindle
        # direction differs from the plan.
        with self.assertRaisesRegex(Exception, "decoded effects differ from source"):
            posted_text(j, machine(j), WrongSpindle, NativeGCodeDecoder)


class ReamingTest(unittest.TestCase):
    def test_reams_drilled_holes_feed_in_feed_out_and_refuses_undrilled(self):
        holder = (HolderSection(35, 60, 6, 6),)
        drill = ToolAssembly(
            "drill5", camlib.Drill("d5", diameter=5, flute_length=30, overall_length=60), 35, 60, holder
        )
        reamer = ToolAssembly(
            "ream52",
            camlib.EndMill("r52", diameter=5.2, flute_length=20, overall_length=60, center_cutting=False),
            35,
            60,
            holder,
        )
        ream = camlib.Reaming(
            "ream",
            reamer,
            HOLES,
            5,
            4,
            12,
            feed_z_mm=11,
            tool_number=2,
            length_register=2,
            feed_mm_min=100,
            retract_feed_mm_min=100,
            spindle_rpm=1000,
            coolant="flood",
        )
        stock = camlib.StockSpec((0, 0, 0), (28, 16, 10), 0.1)
        drilling = camlib.Drilling(
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
        )
        moves = []
        with Simulation(
            stock=stock,
            tools=(drill, reamer),
            operations=(drilling, ream),
            revision="1",
            initial_position_mm=(0, 0, 12),
            verification=camlib.StreamChecks(stock_contact=camlib.StockContactOptions()),
        ) as sim:
            for update in sim.run_operations():
                motion = getattr(update, "motion", None)
                if motion is not None and motion["operation"] == "ream" and motion["kind"] != "rapid":
                    moves.append((motion["kind"], motion["curve"]["start"][2], motion["curve"]["end"][2]))
        self.assertEqual(moves, [("plunge", 11, 4), ("retract", 4, 11)] * 2)
        with Simulation(
            stock=stock, tools=(reamer,), operations=(ream,), revision="1", initial_position_mm=(0, 0, 12)
        ) as sim:
            with self.assertRaisesRegex(SimulationError, "reaming hole 0: the stock is not proven open"):
                list(sim.run_operations())


if __name__ == "__main__":
    unittest.main()
