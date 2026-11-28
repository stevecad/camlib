"""Per-arc output policy at standard controller precision (3 mm decimals).

Regression for a Ø6 pocket around an r=4 island: on a 0.001 mm grid no
exactly radius-consistent G2/G3 words exist for some of its arcs, so the
default post rejects the job. LoweringOptions(native_arcs=True) keeps every
exactly representable arc and lowers only the others to certified lines, and
the posted run reports how many arcs took each form and why.
"""

from camlib.postprocessor import GCodeDialect

from decimal import Decimal
from pathlib import Path
import tempfile
import re
import unittest

from camlib.jobs import Job, Setup
from camlib.operations import Pocket
from camlib.playback import Cursor, Player, RecordedRun
from camlib.planar import Boundary, Component, PlanarRegion
from camlib.post import LoweringOptions, Machine, ToolSetting
from camlib.simulation import SimulationError, StockSpec
from camlib.tools import EndMill, HolderSection, ToolAssembly
from camlib.verification import StockContactOptions, StreamChecks


def island_job():
    mill = ToolAssembly(
        "m6", EndMill("em6", diameter=6, flute_length=20, overall_length=60), 30, 70, (HolderSection(30, 70, 10, 10),)
    )
    region = PlanarRegion(
        "pocket", (Component(Boundary.polygon(((5, 5), (35, 5), (35, 25), (5, 25))), (Boundary.circle((20, 15), 4),)),)
    )
    pocket = Pocket(
        "pocket",
        mill,
        region,
        top_z_mm=10,
        floor_z_mm=6,
        stepdown_mm=1.5,
        stepover_mm=2,
        clearance_z_mm=15,
        feed_mm_min=600,
        plunge_mm_min=150,
        spindle_rpm=12000,
        tool_number=1,
        length_register=1,
        coolant="flood",
    )
    setup = Setup(
        "top",
        StockSpec((0, 0, 0), (40, 30, 10), 0.5),
        (mill,),
        (pocket,),
        (0, 0, 15),
        verification=StreamChecks(stock_contact=StockContactOptions()),
    )
    job = Job("island", "1", (setup,))
    header = job.compile().setup("top").program_header()
    machine = Machine(
        "Three-axis mill",
        (
            ToolSetting.for_tool(
                header.tools[0], number=1, length_register=1, position_after_mm=header.initial_position_mm
            ),
        ),
        dialect=GCodeDialect.linuxcnc(),
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
        tool_change_position_mode='table',
        rapid_mm_min=6000,
    )
    return job, machine


def nc(posted):
    lines, start = [], 1
    while page := posted.nc_lines(start, limit=256):
        lines.extend(line["text"] for line in page)
        start = page[-1]["number"] + 1
    return lines


def adaptive_job(stepover):
    """Issue #445's pocket: a 104 x 64 x 21 mm block (Z0 on top) faced to
    Z-1, then an AdaptivePocket X27..77 Y17..47 to Z-9 with a 12 mm end mill,
    90 degree engagement limit and a helical entry."""
    from camlib.operations import AdaptivePocket, Facing, HelicalEntry

    mill = ToolAssembly(
        "em", EndMill("em", diameter=12, flute_length=26, overall_length=75), 40, 90, (HolderSection(40, 90, 12, 12),)
    )
    common = dict(
        feed_mm_min=1000, plunge_mm_min=300, spindle_rpm=10000, tool_number=1, length_register=1, coolant="flood"
    )
    region = PlanarRegion("pocket", (Component(Boundary(((27, 47, 0), (27, 17, 0), (77, 17, 0), (77, 47, 0)))),))
    operations = (
        Facing("face", mill, -1, 1, 6, 10, **common),
        AdaptivePocket(
            "pocket", mill, region, 0, -9, 12, stepover, 10, max_engagement_deg=90, entry=HelicalEntry(), **common
        ),
    )
    setup = Setup(
        "top",
        StockSpec((0, 0, -21), (104, 64, 0), 0.5),
        (mill,),
        operations,
        (0, 0, 10),
        verification=StreamChecks(stock_contact=StockContactOptions()),
    )
    job = Job("adaptive-arcs", "1", (setup,))
    header = job.compile().setup("top").program_header()
    machine = Machine(
        "Three-axis mill",
        (
            ToolSetting.for_tool(
                header.tools[0], number=1, length_register=1, position_after_mm=header.initial_position_mm
            ),
        ),
        dialect=GCodeDialect.linuxcnc(),
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
        # The tool change keeps the tip where it is (both operations change
        # to the same tool).
        tool_change_position_mode='preserve_tip',
        rapid_mm_min=6000,
    )
    return job, machine


class ArcOutputPolicyTest(unittest.TestCase):
    def test_an_adaptive_pocket_posts_its_arcs_natively(self):
        # #445: the engagement front plans its strokes as arcs (711 of this
        # pocket's 1,128 moves). Lowered to chords they were 11,414 blocks.
        # Posted with native arcs, an arc that has no exactly consistent words
        # from where the native arc before it ended is lowered from that
        # decoded start, which the words moved: its uniform first chord plus
        # that offset exceeded the output tolerance and the whole program was
        # refused ("continuous lowering error"). A shorter lead chord now
        # takes the offset, and every block is still certified. Posted as the
        # benchmark posts a run: its streamed events as a Program, through
        # post_program, which decodes every block and certifies it against its
        # source move.
        from camlib.motion import Program
        from camlib.post import post_program
        from camlib.postprocessor import _typed
        from camlib.simulation import OperationUpdate, Simulation, Update

        job, machine = adaptive_job(0.6)
        setup = job.compile().setup("top")
        events, curves = [], 0
        with Simulation(**setup.simulation_options()) as sim:
            manifest = sim.state.manifest
            for u in sim.run_operations():
                if isinstance(u, OperationUpdate):
                    events.extend(_typed(e) for e in u.events)
                elif isinstance(u, Update):
                    events.append(_typed(dict(kind="motion", motion=u.motion)))
                    curves += u.motion["curve"]["kind"] in ("arc", "helix")
        tools = tuple(ToolAssembly.from_dict(t) for t in manifest["tools"])
        program = Program("adaptive-arcs", "1", tools, tuple(manifest["initial_position_mm"]), tuple(events))
        self.assertEqual(curves, 720)
        text = "".join(c.text for c in post_program(program, machine, lowering=LoweringOptions(native_arcs=True)))
        lines = text.splitlines()
        native = sum(line.startswith(("G2 ", "G3 ")) for line in lines)
        # Lowered to chords these were 11,414 blocks; here most stay arcs.
        self.assertGreaterEqual(native, 700)
        self.assertLess(len(lines), 2000)
        self.check_exact_circles(lines)

    def test_default_precision_rejects_and_native_arcs_policy_is_exact_and_reported(self):
        job, machine = island_job()
        self.assertEqual(machine.decimals, 3)
        with self.assertRaisesRegex(SimulationError, "native_arcs=True"):
            with job.post("top", machine=machine) as posted:
                posted.wait()
        with job.post("top", machine=machine, lowering=LoweringOptions(native_arcs=True)) as posted:
            posted.wait()
            lines = nc(posted)
            player = Player(posted)
            player.seek(Cursor(posted.status()["available"]))
            self.assertEqual(player.verification["progress"]["violation"], 0)
            self.assertEqual(player.verification["progress"]["uncertain"], 0)
            # Reported, not silent: every one of the 12 source arcs is counted.
            self.assertEqual(
                posted.status()["posting"]["arc_output"],
                dict(native=0, lines_policy=0, lines_machine=0, lines_precision=12),
            )
            with tempfile.TemporaryDirectory() as directory:
                posted.save(Path(directory) / "saved")
                with RecordedRun.open(Path(directory) / "saved") as saved:
                    self.assertEqual(saved.status()["posting"]["arc_output"]["lines_precision"], 12)
        # At 3 decimals none of this island's arcs has exact words: all lowered.
        self.assertFalse([line for line in lines if line.startswith(("G2 ", "G3 "))])
        # Where exact words exist (4 decimals here), every source arc stays native.
        from dataclasses import replace

        with job.post(
            "top", machine=replace(machine, decimals=4), lowering=LoweringOptions(native_arcs=True)
        ) as posted:
            posted.wait()
            precise = nc(posted)
            self.assertEqual(
                posted.status()["posting"]["arc_output"],
                dict(native=12, lines_policy=0, lines_machine=0, lines_precision=0),
            )
        self.assertEqual(sum(line.startswith(("G2 ", "G3 ")) for line in precise), 12)
        self.check_exact_circles(precise)

    def check_exact_circles(self, lines):
        # Every emitted circle's start/end radii, recomputed from its decimal
        # words independently of the engine, agree within 5e-6 mm.
        position = None
        for line in lines:
            words = dict(re.findall(r"([XYZIJ])(-?[0-9.]+)", line))
            if line.startswith(("G0 ", "G1 ", "G2 ", "G3 ")) and position is None:
                position = [Decimal(words.get(k, "0")) for k in "XYZ"]
                continue
            if position is None:
                continue
            target = [Decimal(words[k]) if k in words else position[i] for i, k in enumerate("XYZ")]
            if line.startswith(("G2 ", "G3 ")):
                cx, cy = position[0] + Decimal(words["I"]), position[1] + Decimal(words["J"])
                r0 = ((position[0] - cx) ** 2 + (position[1] - cy) ** 2).sqrt()
                r1 = ((target[0] - cx) ** 2 + (target[1] - cy) ** 2).sqrt()
                # Engine consistency tolerance for emitted circles (5e-6 mm).
                self.assertLessEqual(abs(r0 - r1), Decimal("0.000005"), line)
            if line.startswith(("G0 ", "G1 ", "G2 ", "G3 ")):
                position = target


if __name__ == "__main__":
    unittest.main()
