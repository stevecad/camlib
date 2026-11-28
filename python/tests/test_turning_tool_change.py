"""Lathe tool-change modes (#416): where a tool change leaves the new tool's
controlled point (the tip kept, the carriage kept, the carriage at a machine
position), declared on the setup and on the machine, and that planning,
posting and the decoded program all start each tool from that point."""

import hashlib
import math
import unittest
from dataclasses import replace

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


def bare_tools():
    """The two tools with no carriage_to_tip_mm: the offsets are the
    control's alone."""
    groove = tool("groove2", camlib.GrooveInsert(2, 22), nose_radius_mm=0.2, width_mm=2, length_mm=60)
    return (camlib.TurnToolSetting(cnmg(), 1, 1), camlib.TurnToolSetting(groove, 2, 2))


# A start far enough out for a tool post whose tips are within 30 mm (radius)
# and 40 mm (z) of each other: the box spans radius 30..90, z 40..120.
BOUND = (30.0, 40.0)
FAR = (60.0, 80.0)
FAR_START = (60.0, 0.0, 80.0)


def within(bound=BOUND, start=FAR, **changes):
    values = dict(tools=bare_tools(), initial_position_mm=start)
    values.update(changes)
    return two_tools(TurnToolChange.carriage_kept(tip_within_mm=bound), **values)


def bounded(module, plan, bound=BOUND, **settings):
    """The post's machine for a tool post that states only its bound."""
    return machine(module, plan, mode="preserve_carriage", tool_change_tip_within_mm=bound, **settings)


def regroup_block(module, start=FAR_START):
    """The rapid of X (a diameter) and Z to the start, as each post writes it."""
    if module.__name__ == "fanuc_lathe_post":
        return f"G0 X{2 * start[0]:g}. Z{start[2]:g}."
    return f"G0 X{2 * start[0]:g} Z{start[2]:g}"


class LatheToolChangeWithinABoundTest(unittest.TestCase):
    """#421: the carriage kept, the tools' offsets known only to the control,
    one stated bound on how far any tool's tip can be from any other's."""

    def test_a_start_far_enough_out_is_released_with_the_regroup_after_the_t_word(self):
        lathe = within()
        config = lathe.job.compile().setup("lathe").config
        self.assertEqual(
            config["turning"]["tool_change"], {"mode": "carriage_kept_within", "tip_within_mm": [30.0, 40.0]}
        )
        self.assertTrue(all("carriage_to_tip_mm" not in t for t in config["turning"]["tools"]))
        plan = lathe.plan()
        # Every tool starts at the start position, exactly.
        self.assertEqual(planned_changes(plan), [("cnmg", FAR_START), ("groove2", FAR_START)])
        self.assertEqual(first_move(plan, "groove2").curve.start, FAR_START)
        # From the start position on, the plan is the tip-kept plan.
        kept = two_tools(None, tools=bare_tools(), initial_position_mm=FAR).plan()
        self.assertEqual([e.to_dict() for e in plan.program.events], [e.to_dict() for e in kept.program.events])
        self.assertEqual(plan.stock, kept.stock)
        for module, post, decoder in posts():
            m = bounded(module, plan)
            self.assertEqual(m.to_dict()["tool_change_tip_within_mm"], BOUND)
            self.assertTrue(all("carriage_to_tip_mm" not in t for t in m.to_dict()["tools"]))
            text = nc(plan, m, post, decoder).splitlines()
            # The T word, then the rapid of both axes to the start position.
            i = text.index("T0202")
            self.assertEqual(text[i + 1], regroup_block(module))
            # The first tool is the one mounted at the start: no regroup.
            j = text.index("T0101")
            self.assertNotEqual(text[j + 1], regroup_block(module))
            # Against the program of a turret that brings every tool to the
            # start: that one block more, and the blade's first move then
            # names X as well (the regroup commanded both axes under the new
            # offset; after the turret's change X stands where the machine
            # put it and gets no word). Every other block is the same, and
            # so are the moves both programs decode to.
            turret_machine = machine(module, kept)
            turret = nc(kept, turret_machine, post, decoder).splitlines()
            k = turret.index("T0202")
            self.assertEqual(text[: i + 1], turret[: k + 1])
            self.assertEqual(text[i + 2 : i + 4], turret[k + 1 : k + 3])
            x = regroup_block(module).split(" ")[1]
            self.assertEqual(text[i + 4].replace(f" {x}", ""), turret[k + 3])
            self.assertNotEqual(text[i + 4], turret[k + 3])
            self.assertEqual(text[i + 5 :], turret[k + 4 :])
            self.assertEqual(
                [e.to_dict() for e in decoded(plan, m, post, decoder)[1]],
                [e.to_dict() for e in decoded(kept, turret_machine, post, decoder)[1]],
            )
            # Read back: the change is complete at the start position, and
            # the blade's first move starts there.
            changes, motions = decoded(plan, m, post, decoder)
            self.assertEqual([(c.tool, tuple(c.position_after_mm)) for c in changes], planned_changes(plan))
            first = next(e for e in motions if e.tool == "groove2")
            self.assertEqual(tuple(first.curve.start), FAR_START)

    def test_a_start_too_close_for_the_bound_is_refused_before_any_program(self):
        # From the usual start (radius 30, z 20) the box spans radius 0..60,
        # z -20..60: the bar is in it. A turret that brings every tool to
        # the start is released from there.
        close = (30, 20)
        kept = two_tools(None, tools=bare_tools(), initial_position_mm=close).plan()
        for module, post, decoder in posts():
            self.assertIn("T0202", nc(kept, machine(module, kept), post, decoder))
        lathe = within(start=close)
        expected = (
            'groove: the tool change from "cnmg" to "groove2" is not clear: .*'
            r"within radius ±30 mm, z ±40 mm \(tip_within_mm\) of where the tool before it stopped "
            r"\(radius 30, z 20\).*rapid from there to the start position \(radius 30, z 20\).*"
            r"Start further from the work \(the setup's initial_position_mm\), or state a smaller tip_within_mm"
        )
        with self.assertRaisesRegex(Exception, expected):
            lathe.plan()
        # Posting plans the setup itself: no program text is returned.
        header = lathe.job.compile().setup("lathe").program_header()

        class Header:
            program = header

        for module, post, decoder in posts():
            m = bounded(module, Header)
            with self.assertRaisesRegex(Exception, "tool change from .cnmg. to .groove2. is not clear"):
                lathe.job.post_nc("lathe", machine=m, post=post, decoder=decoder, accept={"unchecked_machine"})
        # A smaller bound, if it is true of the tool post, is the other fix.
        within(bound=(5, 10), start=close).plan()

    def test_the_chuck_and_the_jaws_inside_the_box_are_refused(self):
        # The start at radius 60, z 30. A bound of 85 mm along z reaches
        # z -55, behind the chuck face (z -50); the radius stays 30..90,
        # clear of the bar.
        within(bound=(30, 20), start=(60, 30)).plan()
        with self.assertRaisesRegex(
            Exception,
            'tool change from "cnmg" to "groove2" is not clear: the tool would reach the chuck '
            r"\(z below -50.000\)\. .*z ±85 mm.*start position \(radius 60, z 30\)",
        ):
            within(bound=(30, 85), start=(60, 30)).plan()
        # With a grip the jaws hold the bar behind a chuck face further
        # forward: a bound that cleared the bare chuck reaches them.
        operations = (
            camlib.TurnFace("face", "cnmg", 0, feed_mm_rev=0.15, rpm=1000),
            camlib.TurnRough("rough", "cnmg", feed_mm_rev=0.25, rpm=1000),
            camlib.TurnGroove("groove", "groove2", -25, -28, 12, peck_mm=1, feed_mm_rev=0.05, rpm=800),
        )
        within(bound=(30, 76), start=(60, 30), operations=operations).plan()
        jaws = dict(chuck_z_mm=-44, grip=camlib.ChuckGrip(16, 20), operations=operations)
        two_tools(None, tools=bare_tools(), initial_position_mm=(60, 30), **jaws).plan()
        with self.assertRaisesRegex(
            Exception,
            r"is not clear: the tool would reach the chuck \(z below -44.000\): its jaws hold the work over the "
            "16 mm behind that face.*tip_within_mm",
        ):
            within(bound=(30, 76), start=(60, 30), **jaws).plan()

    def test_a_program_whose_block_after_the_t_word_is_not_the_regroup_is_refused(self):
        plan = within().plan()

        def edited(post, edit):
            """The post with the blocks of its second tool change edited."""

            class Edited(post):
                def on_tool_change(self, event):
                    super().on_tool_change(event)
                    if len(self._lines) > 1:
                        self._lines = edit(list(self._lines))

            return Edited

        for module, post, decoder in posts():
            m = bounded(module, plan)
            block = regroup_block(module)
            x, z = block.split(" ")[1:]
            rapid = block.split(" ")[0]
            far = block.replace(x, x.replace("120", "100"))
            self.assertNotEqual(far, block)
            cases = {
                # The rapid left out: the next block is the spindle.
                "without": (lambda lines: lines[:1], "the tool change was written without it"),
                # One axis at a time, as every other rapid here.
                "x only": (lambda lines: [lines[0], f"{rapid} {x}", f"{rapid} {z}"], "this block is not that rapid"),
                "z only": (lambda lines: [lines[0], f"{rapid} {z}"], "this block is not that rapid"),
                # A feed move to the same point.
                "fed": (
                    lambda lines: [lines[0], lines[1].replace(rapid, "G1" if rapid == "G0" else "G01") + " F100."],
                    "this block is not that rapid",
                ),
                # A rapid of both axes to somewhere else.
                "elsewhere": (lambda lines: [lines[0], far], "this rapid goes to radius 50, z 80 mm instead"),
                # Another block first.
                "late": (lambda lines: [lines[0], "M8", lines[1]], "this block is not that rapid"),
            }
            for name, (edit, problem) in cases.items():
                with self.subTest(post=module.__name__, case=name):
                    with self.assertRaises(Exception) as refused:
                        nc(plan, m, edited(post, edit), decoder)
                    message = str(refused.exception)
                    self.assertIn("after the tool change to 'groove2' its tip is only known to be within", message)
                    self.assertIn("radius ±30 mm, z ±40 mm", message)
                    self.assertIn("to the start position, radius 60, z 80 mm", message)
                    self.assertIn(problem, message)
            # The post's own blocks, unedited, are read back and released.
            self.assertIn(block, nc(plan, m, edited(post, lambda lines: lines), decoder).splitlines())
            # Read as an exact carriage-kept or a turret machine, the regroup
            # block is a rapid of X and Z together, which those refuse; read
            # by this machine, a program without it is refused (above).
            exact = machine(module, plan, mode="preserve_carriage", offsets={"cnmg": TURNING_TOOL, "groove2": BLADE})
            self.assertIsNone(exact.tool_change_tip_within_mm)
            self.assertNotIn("tool_change_tip_within_mm", exact.to_dict())

    def test_the_replay_of_the_posted_program_checks_the_box_again(self):
        # The plan is released for a 30 x 40 mm bound. Its events, posted
        # for and read back by a setup and machine that state a larger one
        # (60 x 90: radius 0..120, z -10..170 holds the bar's front), are
        # refused when the decoded program is replayed on the stock: the
        # check does not rest on the planner's.
        from camlib.posted import PostedRun

        plan = within().plan()
        wide = within(bound=(60, 90))
        with self.assertRaisesRegex(Exception, 'tool change from "cnmg" to "groove2" is not clear'):
            wide.plan()
        config = wide.job.compile().setup("lathe").config
        for module, post, decoder in posts():
            m = bounded(module, plan, bound=(60.0, 90.0))
            with self.assertRaisesRegex(Exception, 'tool change from "cnmg" to "groove2" is not clear'):
                with PostedRun(
                    replace(plan.program, revision=config["revision"]),
                    machine=m,
                    stock=config["stock"],
                    post=post,
                    decoder=decoder,
                    sweep=config["sweep"],
                    turning=config["turning"],
                ) as run:
                    run.export(None, accept={"unchecked_machine"})
            # The same events for the bound they were planned with replay.
            narrow = within().job.compile().setup("lathe").config
            with PostedRun(
                replace(plan.program, revision=narrow["revision"]),
                machine=bounded(module, plan),
                stock=narrow["stock"],
                post=post,
                decoder=decoder,
                sweep=narrow["sweep"],
                turning=narrow["turning"],
            ) as run:
                self.assertIn("T0202", run.export(None, accept={"unchecked_machine"})["text"])

    def test_the_bound_is_declared_on_both_sides_and_never_assumed(self):
        plan = within().plan()
        exact_plan = two_tools(TurnToolChange.carriage_kept(), initial_position_mm=FAR).plan()
        kept = two_tools(None, tools=bare_tools(), initial_position_mm=FAR).plan()
        # The setup's side.
        change = TurnToolChange.carriage_kept(tip_within_mm=(30, 40))
        self.assertEqual((change.mode, change.tip_within_mm, change.position_mm), ("carriage_kept", BOUND, None))
        self.assertIsNone(TurnToolChange.carriage_kept().tip_within_mm)
        self.assertEqual(TurnToolChange.carriage_kept().to_dict(), {"mode": "carriage_kept"})
        for bad in ((-1, 40), (30, math.nan), (30, math.inf), (30,), (30, 40, 1), 30, ("30", 40), (True, 40)):
            with self.assertRaisesRegex(ValueError, "tip_within_mm is .radius, z.: two finite distances"):
                TurnToolChange.carriage_kept(tip_within_mm=bad)
        for mode in ("tip_kept", "machine_position"):
            with self.assertRaisesRegex(ValueError, "tip_within_mm .* belongs to a carriage_kept tool change"):
                TurnToolChange(mode, (1, 2) if mode == "machine_position" else None, (30, 40))
        # Without a bound and without offsets the carriage kept is refused,
        # as before: no bound is assumed.
        with self.assertRaisesRegex(ValueError, "tool 'cnmg' has no carriage_to_tip_mm.*camlib assumes none"):
            two_tools(TurnToolChange.carriage_kept(), tools=bare_tools()).job.compile()
        # Declared offsets are not read, but two further apart than the
        # bound say the bound is false (the blade is 3 and 12 mm from the
        # turning tool).
        with self.assertRaisesRegex(
            Exception, 'tools "cnmg" and "groove2".*radius 3, z 12 mm apart.*the bound does not hold'
        ):
            two_tools(TurnToolChange.carriage_kept(tip_within_mm=(30, 10)), initial_position_mm=FAR).plan()
        declared = two_tools(TurnToolChange.carriage_kept(tip_within_mm=BOUND), initial_position_mm=FAR).plan()
        self.assertEqual([e.to_dict() for e in declared.program.events], [e.to_dict() for e in plan.program.events])
        for module, post, decoder in posts():
            # The machine's side: the bound belongs to preserve_carriage,
            # and stands in for the offsets there.
            with self.assertRaisesRegex(ValueError, "tool_change_tip_within_mm .* belongs to .*preserve_carriage"):
                machine(module, plan, tool_change_tip_within_mm=BOUND)
            for bad in ((-1, 40), (30, math.nan), (30,), 30, ("30", 40)):
                with self.assertRaisesRegex(ValueError, "tool_change_tip_within_mm is .x radius, z."):
                    bounded(module, plan, bound=bad)
            with self.assertRaisesRegex(ValueError, "tool 'cnmg' has none.*tool_change_tip_within_mm"):
                machine(module, plan, mode="preserve_carriage")
            # Setup and machine state the same thing.
            m = bounded(module, plan)
            with self.assertRaisesRegex(ValueError, "the machine states only a bound for its tool post"):
                nc(exact_plan, m, post, decoder)
            with self.assertRaisesRegex(ValueError, "preserve_carriage.*tool change is tip_kept"):
                nc(kept, m, post, decoder)
            exact = machine(module, plan, mode="preserve_carriage", offsets={"cnmg": TURNING_TOOL, "groove2": BLADE})
            with self.assertRaisesRegex(ValueError, "within a bound .*but the machine states no bound"):
                nc(plan, exact, post, decoder)
            with self.assertRaisesRegex(ValueError, 'carriage kept.*tool_change_position_mode is "table"'):
                nc(plan, machine(module, plan), post, decoder)
            with self.assertRaisesRegex(ValueError, "one tool post has one bound"):
                nc(plan, bounded(module, plan, bound=(30, 41)), post, decoder)

    def test_the_committed_stock_state_starts_each_tool_at_the_start(self):
        from camlib.jobs import JobRun

        run = JobRun(within().job.compile(), None)
        for _ in run:
            pass
        state = run.states["lathe"]
        self.assertEqual(state.tool_change_tip("groove2", position=FAR_START, previous="cnmg"), FAR_START)
        self.assertEqual(state.tool_change_tip("cnmg", position=FAR_START, previous="cnmg"), FAR_START)


class ByteIdentityTest(unittest.TestCase):
    def test_the_other_modes_post_what_they_posted_before_the_bound_existed(self):
        # The compiled setup, the machine definition and the program of the
        # tip-kept, the exact carriage-kept and the machine-position jobs,
        # as the commit before #421 wrote them (657ce97).
        import json

        after = {"cnmg": START, "groove2": BLADE_START}
        jobs = {
            "tip_kept": (None, dict()),
            "carriage_kept": (TurnToolChange.carriage_kept(), dict(mode="preserve_carriage", offsets=OFFSETS)),
            "machine_position": (TurnToolChange.machine_position(*CARRIAGE), dict(after=after)),
        }

        def sha(value):
            raw = (
                value
                if isinstance(value, bytes)
                else json.dumps(value, sort_keys=True, indent=1, default=list).encode()
            )
            return hashlib.sha256(raw).hexdigest()

        for name, (change, settings) in jobs.items():
            lathe = two_tools(change)
            got = {"setup": sha(lathe.job.compile().setup("lathe").config)}
            plan = lathe.plan()
            for module, post, decoder in posts():
                m = machine(module, plan, **settings)
                got[module.__name__ + ".machine"] = sha(m.to_dict())
                got[module.__name__ + ".nc"] = sha(nc(plan, m, post, decoder).encode())
            self.assertEqual(got, BEFORE_421[name], name)


# The programs this test's tip-kept setup posted on main before #416 (the
# Centroid CNC12 and FANUC lathe posts).
TIP_KEPT_SHA256 = {
    "centroid_lathe_post": "10982777b5c20581751af7a7e52386db0ac3a7f0b185f8f07bc3e3876d686f6b",
    "fanuc_lathe_post": "fa14250eee23c2855837517676be9ab7feca74d1d2853c66a047ac357812e846",
}

# What the three modes that existed before #421 compiled, declared and posted
# at the commit before it (657ce97): SHA-256 of the compiled setup, of each
# machine definition and of each program, for this file's two-tool job.
BEFORE_421 = {
    "tip_kept": {
        "setup": "5292485d88d9bb47446b3139321c57bb0c092ade2fdb196e23228966b224d539",
        "centroid_lathe_post.machine": "c36c22928be5ea05f0b6b7a8b96f335f9806b05d8c2d26de464ed83776dec4a1",
        "centroid_lathe_post.nc": "10982777b5c20581751af7a7e52386db0ac3a7f0b185f8f07bc3e3876d686f6b",
        "fanuc_lathe_post.machine": "f58cfaccd754a96cff39968a3631e07179daf1630604058874076a43083263f3",
        "fanuc_lathe_post.nc": "fa14250eee23c2855837517676be9ab7feca74d1d2853c66a047ac357812e846",
    },
    "carriage_kept": {
        "setup": "3ab045d70e0af676672ecaaf4021cd18882f9d493fec8da0a1cff2158d6439d3",
        "centroid_lathe_post.machine": "517dad99fd66f45055fd631480776a940839a52e0ae3df6abbeb5cf2c5bd4673",
        "centroid_lathe_post.nc": "10982777b5c20581751af7a7e52386db0ac3a7f0b185f8f07bc3e3876d686f6b",
        "fanuc_lathe_post.machine": "21ba39d7ab04c3645edfa73c90c3494cdbd223f7f2a47ec7b0fc1f804ab103d6",
        "fanuc_lathe_post.nc": "fa14250eee23c2855837517676be9ab7feca74d1d2853c66a047ac357812e846",
    },
    "machine_position": {
        "setup": "552421501f7ce69405672bca6cc217bf17502a949978127312952ccf00b69fe9",
        "centroid_lathe_post.machine": "23d9d70bb2eda84ec37e54afb60ed36b718f936371d3fd2d4d08c5a738e18d6f",
        "centroid_lathe_post.nc": "61d9b3d2f594387a76f99842bfd30761404e1d4087987c38f6cc5dd15c5f4ad7",
        "fanuc_lathe_post.machine": "321b899734cd1110ae4599607570039bcd0d6e241c4901d0ba0ec8adf08f7c22",
        "fanuc_lathe_post.nc": "3776a3ba9ba8f88b8675d494e42761a10068de77dfbeb8acc621c06d7f49ac88",
    },
}


def written(value, decimals=4, diameter=False):
    """A millimetre value as an inch output at ``decimals`` writes it (X a
    diameter), back in millimetres."""
    factor = 2 if diameter else 1
    return round(value * factor / 25.4, decimals) * 25.4 / factor


class BoundedToolChangeAtTheOutputResolutionTest(unittest.TestCase):
    """#424: a start position in millimetres that the inch output cannot
    write exactly. The regroup after each T word goes where the program
    writes the start; the replay checks the change from there."""

    # Inch output at 4 decimals (0.0001 in: 0.00254 mm in Z, 0.00127 mm in
    # radius for a diameter X), with the setup's and the machine's
    # tolerance above that rounding.
    INCH = dict(output_units="inch", decimals=4, tolerance_mm=0.003)

    def test_a_start_the_output_rounds_is_released_and_reads_back_continuously(self):
        start = (60.0127, 80.0123)
        lathe = within(start=start, tolerance_mm=0.003)
        plan = lathe.plan()
        to = (written(start[0], diameter=True), 0.0, written(start[1]))
        self.assertNotEqual((to[0], to[2]), start)  # it does not round-trip
        for module, post, decoder in posts():
            with self.subTest(post=module.__name__):
                m = bounded(module, plan, **self.INCH)
                text = nc(plan, m, post, decoder).splitlines()
                # The regroup: the rapid of both axes after the T word, to
                # the start as the inch words write it.
                i = text.index("T0202")
                x, z = (f"{2 * start[0] / 25.4:.4f}".rstrip("0"), f"{start[1] / 25.4:.4f}".rstrip("0"))
                self.assertEqual(text[i + 1], f"G0 X{x} Z{z}")
                changes, motions = decoded(plan, m, post, decoder)
                # Read back: every move starts where the one before ended,
                # through each change; the blade's change completes at the
                # written start, where its first move starts and where the
                # turning tool's last move ended.
                for a, b in zip(motions, motions[1:]):
                    self.assertEqual(tuple(b.curve.start), tuple(a.curve.end), (a.id, b.id))
                for got, want in zip(changes[1].position_after_mm, to):
                    self.assertAlmostEqual(got, want, places=9)
                k = next(i for i, e in enumerate(motions) if e.tool == "groove2")
                self.assertEqual(tuple(motions[k].curve.start), tuple(changes[1].position_after_mm))
                self.assertEqual(tuple(motions[k - 1].curve.end), tuple(changes[1].position_after_mm))

    def test_a_start_whose_written_point_takes_the_box_into_the_work_is_refused_naming_it(self):
        # A bound of 40.0005 mm along z from z 39.998: the plan's box ends
        # 0.0025 mm in front of the faced end, within the setup's 0.003 mm.
        # Written in inch the start is z 39.99738, and the box reaches past
        # that tolerance into the work: refused, before any program.
        bound, start = (30.0, 40.0005), (35.0, 39.998)
        lathe = within(bound=bound, start=start, tolerance_mm=0.003)
        plan = lathe.plan()
        expected = (
            r'the tool change from "cnmg" to "groove2" is not clear.* \(the tool change as the posted program was '
            r"read back: the tool before it stopped at radius 34\.99993, z 39\.99738 and the regroup rapid goes to "
            r"radius 34\.99993, z 39\.99738, as written\)\. The setup's start position, radius 35, z 39\.998 mm "
            r"\(initial_position_mm\), is not on the output resolution of 0\.0001 inch \(the machine's decimals=4, "
            r"X as a diameter\).*Give a start position the output writes exactly \(a radius on a 0\.00127 mm step "
            r"and a z on a 0\.00254 mm step\), or start further from the work$"
        )
        for module, post, decoder in posts():
            with self.subTest(post=module.__name__):
                m = bounded(module, plan, bound=bound, **self.INCH)
                with self.assertRaisesRegex(Exception, expected):
                    nc(plan, m, post, decoder)
        # The same start in millimetres at 4 decimals is written exactly and
        # released.
        for module, post, decoder in posts():
            m = bounded(module, plan, bound=bound, output_units="mm", decimals=4, tolerance_mm=0.003)
            self.assertIn("T0202", nc(plan, m, post, decoder))


if __name__ == "__main__":
    unittest.main()
