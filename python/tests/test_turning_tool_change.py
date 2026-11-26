"""Lathe tool-change modes (#416): where a tool change leaves the new tool's
controlled point (the tip kept, the carriage kept, the carriage at a machine
position), declared on the setup and on the machine, and that planning,
posting and the decoded program all start each tool from that point."""

import hashlib
import math
import unittest

import camlib
from camlib.motion import Motion, ToolChange
from camlib.simulation import Simulation
from camlib.post import GCodeDialect, Machine, ToolSetting, post_program
from camlib.turning import TurnToolChange

from test_drill_cycles import example
from test_turning import cnmg, job, tool

GROOVED = (
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
# From the carriage reference to each tool's controlled point (radius, z):
# the turning tool, and a blade 3 mm shorter toward the axis and 12 mm longer
# toward the chuck. With the turning tool's tip at the start (30, 20) the
# carriage reference is at (55, 60).
TURNING_TOOL = (-25.0, -40.0)
BLADE = (-22.0, -52.0)
CARRIAGE = (55.0, 60.0)
START = (30.0, 0.0, 20.0)
BLADE_START = (33.0, 0.0, 8.0)


def two_tools(change, blade=BLADE, **changes):
    """Face and rough with the turning tool (T0101), groove and part off
    with a grooving blade (T0202)."""
    groove = tool("groove2", camlib.GrooveInsert(2, 22), nose_radius_mm=0.2, width_mm=2, length_mm=60)
    values = dict(
        profile=GROOVED,
        tools=(
            camlib.TurnToolSetting(cnmg(), 1, 1, carriage_to_tip_mm=TURNING_TOOL),
            camlib.TurnToolSetting(groove, 2, 2, carriage_to_tip_mm=blade),
        ),
        operations=(
            camlib.TurnFace("face", "cnmg", 0, feed_mm_rev=0.15, rpm=1000),
            camlib.TurnRough("rough", "cnmg", feed_mm_rev=0.25, rpm=1000),
            camlib.TurnGroove("groove", "groove2", -25, -28, 12, peck_mm=1, feed_mm_rev=0.05, rpm=800),
            camlib.TurnPartOff("part", "groove2", -40, feed_mm_rev=0.05, rpm=800),
        ),
        tool_change=change,
    )
    values.update(changes)
    return job(**values)


DECLARED = dict(
    max_spindle_rpm=4000,
    output_units="mm",
    decimals=3,
    max_feed_mm_min=10000,
    arcs=True,
    helices=False,
    max_sweep_rad=math.tau,
    max_tool_number=12,
    max_length_register=12,
    tool_changes=True,
    flood=True,
    mist=False,
    rapid_mm_min=8000,
)


def machine(post, plan, mode="table", after=None, offsets=None, **settings):
    """The post's machine for ``plan``'s tools: ``after`` each tool's
    position after its change (default: the start), ``offsets`` each tool's
    carriage_to_tip_mm."""
    tools = tuple(
        ToolSetting.for_tool(
            t,
            number=i,
            length_register=i,
            position_after_mm=(after or {}).get(t.id, plan.program.initial_position_mm),
            carriage_to_tip_mm=(offsets or {}).get(t.id),
        )
        for i, t in enumerate(plan.program.tools, 1)
    )
    return post.define_machine("lathe", tools, tool_change_position_mode=mode, **{**DECLARED, **settings})


def posts():
    c, f = example("centroid_lathe_post"), example("fanuc_lathe_post")
    return ((c, c.CentroidLathePost, c.CentroidLatheDecoder), (f, f.FanucLathePost, f.FanucLatheDecoder))


def nc(plan, m, post, decoder):
    return plan.job.post_nc("lathe", machine=m, post=post, decoder=decoder, accept={"unchecked_machine"}).text


def decoded(plan, m, post, decoder):
    """The program as its posted text reads back: (tool changes, motions)."""
    events = [e for chunk in post_program(plan.program, machine=m, post=post, decoder=decoder) for e in chunk.events]
    return [e for e in events if isinstance(e, ToolChange)], [e for e in events if isinstance(e, Motion)]


def planned_changes(plan):
    return [(e.tool, tuple(e.position_after_mm)) for e in plan.program.events if isinstance(e, ToolChange)]


def first_move(plan, tool_id):
    return next(e for e in plan.program.events if isinstance(e, Motion) and e.tool == tool_id)


OFFSETS = {"cnmg": TURNING_TOOL, "groove2": BLADE}


class LatheToolChangeTest(unittest.TestCase):
    def test_the_tip_kept_is_the_default_and_its_programs_are_unchanged(self):
        # No mode, the tip kept stated, and offsets it does not read: one
        # setup revision apart, the same moves and the same program text.
        groove = tool("groove2", camlib.GrooveInsert(2, 22), nose_radius_mm=0.2, width_mm=2, length_mm=60)
        bare = (camlib.TurnToolSetting(cnmg(), 1, 1), camlib.TurnToolSetting(groove, 2, 2))
        plain = two_tools(None, tools=bare).plan()
        stated = two_tools(TurnToolChange.tip_kept(), tools=bare).plan()
        offsets = two_tools(TurnToolChange.tip_kept()).plan()
        self.assertEqual(TurnToolChange(), TurnToolChange.tip_kept())
        config = plain.job.compile().setup("lathe").config
        self.assertNotIn("tool_change", config["turning"])
        self.assertNotIn("tool_change", config)
        self.assertTrue(all("carriage_to_tip_mm" not in t for t in config["turning"]["tools"]))
        # The stated default compiles to the very same setup (and revision).
        self.assertEqual(stated.job.compile().setup("lathe").config, config)
        self.assertEqual(planned_changes(plain), [("cnmg", START), ("groove2", START)])
        self.assertEqual(first_move(plain, "groove2").curve.start, START)
        for module, post, decoder in posts():
            m = machine(module, plain)
            self.assertNotIn("carriage_to_tip_mm", str(m.to_dict()))
            text = nc(plain, m, post, decoder)
            self.assertEqual(nc(stated, m, post, decoder), text)
            self.assertEqual(nc(offsets, m, post, decoder), text)
            self.assertIn("T0202", text.splitlines())
            self.assertEqual(
                hashlib.sha256(text.encode()).hexdigest(),
                TIP_KEPT_SHA256[module.__name__],
                "the tip-kept program differs from the one main posts",
            )

    def test_the_carriage_kept_starts_the_new_tool_by_the_offset_difference(self):
        plan = two_tools(TurnToolChange.carriage_kept()).plan()
        # The turning tool is put away with its tip at the start, the
        # carriage reference at (55, 60); the blade's tip is at (55, 60) +
        # (-22, -52).
        self.assertEqual(CARRIAGE, (START[0] - TURNING_TOOL[0], START[2] - TURNING_TOOL[1]))
        self.assertEqual(planned_changes(plan), [("cnmg", START), ("groove2", BLADE_START)])
        self.assertEqual(first_move(plan, "groove2").curve.start, BLADE_START)
        for module, post, decoder in posts():
            m = machine(module, plan, mode="preserve_carriage", offsets=OFFSETS)
            self.assertEqual(m.to_dict()["tools"][1]["carriage_to_tip_mm"], BLADE)
            # Released: planned, posted, read back and replayed on the stock,
            # every step starting the blade at the same point.
            text = nc(plan, m, post, decoder).splitlines()
            self.assertIn("T0202", text)
            # The same blocks as the tip-kept program here (the blade's way
            # in is the same absolute moves); what differs is where camlib
            # knows the blade to be while they run.
            kept = two_tools(None).plan()
            self.assertEqual("\n".join(text) + "\n", nc(kept, machine(module, kept), post, decoder))
            changes, motions = decoded(plan, m, post, decoder)
            self.assertEqual([(c.tool, tuple(c.position_after_mm)) for c in changes][-1], ("groove2", BLADE_START))
            first = next(e for e in motions if e.tool == "groove2")
            self.assertEqual(tuple(first.curve.start), BLADE_START)
            # The decoder must know the mode: read as if the tip were kept,
            # the blade would start at (30, 20) and the program is refused.
            assumed = machine(module, plan)
            with self.assertRaises(Exception) as refused:
                decoded(plan, assumed, post, decoder)
            # (Its first rapid then reads as a move of X and Z together,
            # which these controls do not run along a straight line.)
            self.assertIn("moving X and Z together", str(refused.exception))

    def test_a_tool_left_below_the_safe_radius_leaves_outward_first(self):
        # A blade 10 mm longer toward the axis: its tip is left at radius 20,
        # z 8, under the approach's safe radius (the tip at 22: the bar's 20
        # and the clearance). Its approach starts with a rapid straight out,
        # which the tip-kept program (from radius 30) does not have.
        blade = (-35.0, -52.0)
        plan = two_tools(TurnToolChange.carriage_kept(), blade=blade).plan()
        self.assertEqual(planned_changes(plan)[-1], ("groove2", (20.0, 0.0, 8.0)))
        first = first_move(plan, "groove2")
        self.assertEqual((first.kind, first.curve.start, first.curve.end[1:]), ("rapid", (20.0, 0.0, 8.0), (0.0, 8.0)))
        self.assertAlmostEqual(first.curve.end[0], 22, places=12)
        kept = two_tools(None, blade=blade).plan()
        for module, post, decoder in posts():
            m = machine(module, plan, mode="preserve_carriage", offsets={"cnmg": TURNING_TOOL, "groove2": blade})
            text = [line.replace("G00", "G0").replace(".", "") for line in nc(plan, m, post, decoder).splitlines()]
            i = text.index("T0202")
            self.assertIn("G0 X44", text[i + 1 : i + 4])
            before = [
                line.replace("G00", "G0").replace(".", "")
                for line in nc(kept, machine(module, kept), post, decoder).splitlines()
            ]
            self.assertNotIn("G0 X44", before)

    def test_a_start_the_tip_offset_does_not_round_trip_plans_and_posts(self):
        # (32 + 0.8) - 0.8 is not 32 in binary floating point (0.8: this
        # insert's tip from its nose center). The program's point is carried
        # exactly: the first move starts at the stated start and a move
        # along Z keeps its X.
        self.assertNotEqual((32 + 0.8) - 0.8, 32)
        plan = job(initial_position_mm=(32, 20)).plan()
        first = next(e for e in plan.program.events if isinstance(e, Motion))
        self.assertEqual(first.curve.start, (32.0, 0.0, 20.0))
        self.assertEqual(first.curve.end[0], 32.0)
        for module, post, decoder in posts():
            self.assertIn("T0101", nc(plan, machine(module, plan), post, decoder))

    def test_a_machine_position_starts_each_tool_at_the_position_plus_its_offset(self):
        change = TurnToolChange.machine_position(*CARRIAGE)
        lathe = two_tools(change)
        settings = {s.tool.id: s for s in lathe.job.setups[0].tools}
        self.assertEqual(change.tip_mm(settings["cnmg"]), START)
        self.assertEqual(change.tip_mm(settings["groove2"]), BLADE_START)
        plan = lathe.plan()
        self.assertEqual(planned_changes(plan), [("cnmg", START), ("groove2", BLADE_START)])
        self.assertEqual(first_move(plan, "groove2").curve.start, BLADE_START)
        # Each tool is put away, and the program ends, with the carriage at
        # the stated position: the change itself has nothing left to move.
        moves = [e for e in plan.program.events if isinstance(e, Motion)]
        before = moves[moves.index(first_move(plan, "groove2")) - 1]
        self.assertEqual((before.tool, before.curve.end), ("cnmg", START))
        for stated, ended in zip(BLADE_START, moves[-1].curve.end):
            self.assertAlmostEqual(stated, ended, places=12)
        after = {id: change.tip_mm(s) for id, s in settings.items()}
        for module, post, decoder in posts():
            m = machine(module, plan, after=after)
            self.assertIn("T0202", nc(plan, m, post, decoder).splitlines())
            changes, motions = decoded(plan, m, post, decoder)
            self.assertEqual([(c.tool, tuple(c.position_after_mm)) for c in changes][-1], ("groove2", BLADE_START))
            # A table that leaves every tool at the start is another machine.
            with self.assertRaisesRegex(ValueError, r"tool groove2: the setup starts it at \(33.0, 0.0, 8.0\)"):
                nc(plan, machine(module, plan), post, decoder)
            with self.assertRaisesRegex(ValueError, "preserve_carriage.*tool change is machine_position"):
                nc(plan, machine(module, plan, mode="preserve_carriage", offsets=OFFSETS), post, decoder)
        # The setup starts where the first change leaves the first tool.
        with self.assertRaisesRegex(Exception, "initial_position_mm.*first tool change leaves \"cnmg\""):
            two_tools(change, initial_position_mm=(31, 20)).plan()

    def test_a_first_move_that_hits_from_the_true_tip_is_found(self):
        # A blade 18 mm longer toward the axis and 50 mm toward the chuck:
        # its tip is at (55, 60) + (-43, -90) = (12, -30), inside the part
        # (radius 15 there). Judged with the tip kept, the same tools and
        # operations start the blade at (30, 20) and are released.
        deep = (-43.0, -90.0)
        assumed = two_tools(None, blade=deep).plan()
        for module, post, decoder in posts():
            self.assertIn("T0202", nc(assumed, machine(module, assumed), post, decoder))
        for change in (TurnToolChange.carriage_kept(), TurnToolChange.machine_position(*CARRIAGE)):
            with self.assertRaisesRegex(
                Exception,
                'groove: .*into the part.*tool change leaves tool "groove2" before it moves.*radius 12, z -30',
            ):
                two_tools(change, blade=deep).plan()
        # A blade left at radius 15.25, z -30: outside the part (15) but in
        # the roughing allowance the bar still has there (15.3).
        shallow = (-39.75, -90.0)
        two_tools(None, blade=shallow).plan()
        with self.assertRaisesRegex(Exception, "groove: .*touch the stock.*radius 15.25, z -30"):
            two_tools(TurnToolChange.carriage_kept(), blade=shallow).plan()
        # A blade 112 mm long toward the chuck: its tip at (30, -52), clear
        # of the bar but behind the chuck face (z -50).
        long = (-25.0, -112.0)
        two_tools(None, blade=long).plan()
        with self.assertRaisesRegex(Exception, "groove: .*reach the chuck.*radius 30, z -52"):
            two_tools(TurnToolChange.carriage_kept(), blade=long).plan()

    def test_what_is_not_declared_is_refused_by_name(self):
        groove = tool("groove2", camlib.GrooveInsert(2, 22), nose_radius_mm=0.2, width_mm=2, length_mm=60)
        missing = (
            camlib.TurnToolSetting(cnmg(), 1, 1, carriage_to_tip_mm=TURNING_TOOL),
            camlib.TurnToolSetting(groove, 2, 2),
        )
        for change in (TurnToolChange.carriage_kept(), TurnToolChange.machine_position(*CARRIAGE)):
            with self.assertRaisesRegex(ValueError, "tool 'groove2' has no carriage_to_tip_mm.*camlib assumes none"):
                two_tools(change, tools=missing).job.compile()
        with self.assertRaisesRegex(ValueError, "tool 'groove2' has no carriage_to_tip_mm"):
            TurnToolChange.machine_position(*CARRIAGE).tip_mm(missing[1])
        with self.assertRaisesRegex(ValueError, "only a machine_position tool change"):
            TurnToolChange.carriage_kept().tip_mm(missing[0])
        with self.assertRaisesRegex(ValueError, "tip_kept, carriage_kept or machine_position"):
            TurnToolChange("turret")
        with self.assertRaisesRegex(ValueError, "position_mm"):
            TurnToolChange("machine_position")
        with self.assertRaisesRegex(ValueError, "position_mm"):
            TurnToolChange("carriage_kept", (1, 2))
        # A milling tool-change pose is not a lathe's, and the reverse.
        with self.assertRaisesRegex(ValueError, "TurnToolChange"):
            two_tools(camlib.ToolChangeInPlace()).job.compile()
        # The engine refuses the same when it is handed the setup directly.
        config = two_tools(TurnToolChange.carriage_kept()).job.compile().setup("lathe").simulation_options()
        del config["turning"]["tools"][1]["carriage_to_tip_mm"]
        with self.assertRaisesRegex(Exception, 'tool "groove2" has no carriage_to_tip_mm'):
            with Simulation(**config) as sim:
                for _ in sim.run_operations():
                    pass

        plan = two_tools(TurnToolChange.carriage_kept()).plan()
        kept = two_tools(None).plan()
        for module, post, decoder in posts():
            # The machine's own declaration.
            with self.assertRaisesRegex(ValueError, "preserve_carriage.*tool 'groove2' has none"):
                machine(module, plan, mode="preserve_carriage", offsets={"cnmg": TURNING_TOOL})
            with self.assertRaisesRegex(ValueError, "tool 'cnmg': carriage_to_tip_mm must be finite"):
                machine(module, plan, mode="preserve_carriage", offsets={**OFFSETS, "cnmg": (math.nan, 0)})
            with self.assertRaisesRegex(ValueError, "preserve_carriage"):
                machine(module, plan, mode="preserve_tip")
            # Setup and machine must declare the same change.
            with self.assertRaisesRegex(ValueError, 'carriage kept.*tool_change_position_mode is "table"'):
                nc(plan, machine(module, plan), post, decoder)
            carriage = machine(module, plan, mode="preserve_carriage", offsets=OFFSETS)
            with self.assertRaisesRegex(ValueError, "preserve_carriage.*tool change is tip_kept"):
                nc(kept, carriage, post, decoder)
            # And the same offsets.
            other = machine(module, plan, mode="preserve_carriage", offsets={**OFFSETS, "groove2": (-22.0, -51.0)})
            with self.assertRaisesRegex(ValueError, "tool groove2: .*one tool has one offset"):
                nc(plan, other, post, decoder)
        # preserve_carriage is a lathe's T word, not a spindle tool change.
        with self.assertRaisesRegex(ValueError, "preserve_carriage is a lathe tool change"):
            Machine(
                "mill",
                (ToolSetting("t", 1, 1, 50.0, carriage_to_tip_mm=(0, 0)),),
                dialect=GCodeDialect(),
                planes=("xy",),
                tool_change_position_mode="preserve_carriage",
                **DECLARED,
            )

    def test_the_committed_stock_state_places_a_tool_change_by_the_mode(self):
        from camlib.jobs import JobRun
        from camlib.simulation import ProtocolError

        for change, tip in (
            (None, START),
            (TurnToolChange.carriage_kept(), BLADE_START),
            (TurnToolChange.machine_position(*CARRIAGE), BLADE_START),
        ):
            run = JobRun(two_tools(change).job.compile(), None)
            for _ in run:
                pass
            state = run.states["lathe"]
            # Asked of a fresh consumer of the same run: after the turning
            # tool stopped at the start.
            self.assertEqual(state.tool_change_tip("groove2", position=START, previous="cnmg"), tip)
            # The same tool again: no offset changes, in any mode.
            self.assertEqual(state.tool_change_tip("cnmg", position=START, previous="cnmg"), START)
            with self.assertRaises(ProtocolError):
                state.tool_change_tip("other")

    def test_a_tool_change_is_placed_nowhere_but_where_the_mode_leaves_it(self):
        from camlib.simulation import ProtocolError, Update

        def mid_run(change):
            """The committed state after the turning tool's first moves."""
            setup = two_tools(change).job.compile().setup("lathe")
            with Simulation(**setup.simulation_options()) as sim:
                moves = 0
                for update in sim.run_operations():
                    moves += isinstance(update, Update)
                    if moves == 5:
                        return sim.state.clone()

        # The tip kept, as on main: only the start position, in main's words;
        # not where the last move ended, nor anywhere else.
        state = mid_run(None)
        here = tuple(state.position)
        self.assertNotEqual(here, START)
        state.position = START
        for away in (here, BLADE_START, (30.0, 0.0, 21.0)):
            with self.assertRaisesRegex(
                ProtocolError, r"^a lathe tool change starts at the turret \(start\) position$"
            ):
                state.position = away
        self.assertEqual(tuple(state.position), here)
        # The carriage kept: only the last position moved by an offset
        # difference (the same tool again: where it is); not the start.
        state = mid_run(TurnToolChange.carriage_kept())
        here = tuple(state.position)
        blade = state.tool_change_tip("groove2")
        self.assertEqual(blade, (here[0] + 3.0, 0.0, here[2] - 12.0))
        state.position = blade
        state.position = here
        for away in (START, BLADE_START, (here[0], 0.0, here[2] + 1.0)):
            with self.assertRaisesRegex(ProtocolError, "carriage_kept tool change leaves the tool"):
                state.position = away
        # A machine position: only each tool's own point.
        state = mid_run(TurnToolChange.machine_position(*CARRIAGE))
        here = tuple(state.position)
        state.position = START
        state.position = BLADE_START
        for away in (here, (30.0, 0.0, 21.0)):
            with self.assertRaisesRegex(ProtocolError, "machine_position tool change leaves the tool"):
                state.position = away


# The programs this test's tip-kept setup posted on main before #416 (the
# Centroid CNC12 and FANUC lathe posts).
TIP_KEPT_SHA256 = {
    "centroid_lathe_post": "10982777b5c20581751af7a7e52386db0ac3a7f0b185f8f07bc3e3876d686f6b",
    "fanuc_lathe_post": "fa14250eee23c2855837517676be9ab7feca74d1d2853c66a047ac357812e846",
}


if __name__ == "__main__":
    unittest.main()
