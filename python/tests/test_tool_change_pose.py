"""Multi-tool jobs and the machine's tool-change pose (#65): the setup plans
each new tool where the machine's tool change leaves it, the posted replay
accepts a tool table's poses, and a setup that disagrees with the machine is
refused before posting with both poses and the fix named."""

import unittest

import camlib
from camlib import GCodeDialect, Machine, ToolChangeInPlace, ToolChangePosition, ToolChangeTable, ToolSetting
from camlib.motion import WorkFrame

HOLES = ((8, 8), (20, 8))


def tools():
    drill = camlib.ToolAssembly(
        "drill5",
        camlib.Drill("drill5", diameter=5, point_angle=118, flute_length=20, overall_length=60),
        40,
        80,
        (camlib.HolderSection(40, 80, 10, 10),),
    )
    spot = camlib.ToolAssembly(
        "drill3",
        camlib.Drill("drill3", diameter=3, point_angle=118, flute_length=12, overall_length=50),
        30,
        75,
        (camlib.HolderSection(30, 75, 10, 10),),
    )
    return drill, spot


def job(tool_change):
    drill, spot = tools()
    ops = tuple(
        camlib.Drilling(
            f"drill{k}",
            tool,
            HOLES,
            depth,
            12,
            tool_number=k,
            length_register=k,
            feed_mm_min=150,
            retract_feed_mm_min=600,
            spindle_rpm=6000,
            coolant="flood",
        )
        for k, (tool, depth) in enumerate(((spot, 8.0), (drill, 4.0)), 1)
    )
    setup = camlib.Setup(
        "top",
        stock=camlib.StockSpec((0, 0, 0), (28, 16, 10), 0.1),
        tools=(drill, spot),
        operations=tuple(camlib.OperationNode(op, sources=()) for op in ops),
        initial_position_mm=(0, 0, 40),
        verification=camlib.StreamChecks(stock_contact=camlib.StockContactOptions()),
        tool_change=tool_change,
    )
    return camlib.Job("tool-change", "1", (setup,))


def machine(table, mode="table", tool_table=None, **settings):
    drill, spot = tools()
    by_id = {"drill5": drill, "drill3": spot}
    numbers = {"drill3": 1, "drill5": 2}
    return Machine(
        "Three-axis mill",
        tool_table
        or tuple(
            ToolSetting.for_tool(by_id[t], number=numbers[t], length_register=numbers[t], position_after_mm=tip)
            for t, tip in table.items()
        ),
        tool_change_position_mode=mode,
        **{
            "dialect": GCodeDialect.linuxcnc(),
            "output_units": "mm",
            "decimals": 3,
            "max_feed_mm_min": 10000,
            "max_spindle_rpm": 30000,
            "planes": ("xy", "xz", "yz"),
            "arcs": True,
            "helices": True,
            "max_sweep_rad": 6.283185307179586,
            "max_tool_number": 99,
            "max_length_register": 99,
            "tool_changes": True,
            "flood": True,
            "mist": False,
            "rapid_mm_min": 6000,
            **settings,
        },
    )


# A tool table whose poses share no gauge line or XY: only a table can plan it.
TABLE = {"drill3": (0.0, 0.0, 45.0), "drill5": (5.0, -3.0, 38.0)}


def posted(j, m):
    with j.post("top", machine=m) as run:
        run.wait()
        nc, start = [], 1
        while page := run.nc_lines(start, limit=256):
            nc.extend(line["text"] for line in page)
            start = page[-1]["number"] + 1
        status = run.status()
        # The posted program was replayed into stock to the end.
        assert status["finished"] and status["error"] is None, status
        return nc


class ToolChangeTableTest(unittest.TestCase):
    def test_table_machine_posts_and_replays_a_multi_tool_job(self):
        m = machine(TABLE)
        j = job(ToolChangeTable.from_machine(m))
        nc = posted(j, m)
        self.assertEqual(sum("M6" in line.split() for line in nc), 2)
        # The same job without the posted replay agrees.
        self.assertEqual(len(j.post_nc("top", machine=m, accept={"unchecked_machine"}).text.splitlines()), len(nc))

    def test_planner_starts_each_new_tool_at_its_table_pose(self):
        from camlib.simulation import OperationUpdate

        m = machine(TABLE)
        starts = {}
        with job(ToolChangeTable.from_machine(m)).regenerate() as run:
            for event in run:
                update = event.update
                if isinstance(update, OperationUpdate) and update.stage == "begin":
                    change = next(e for e in update.events if e["kind"] == "tool_change")
                    starts[change["tool"]] = tuple(change["position_after_mm"])
        self.assertEqual(starts, TABLE)

    def test_gauge_position_matching_the_table_also_posts(self):
        # A table whose tips are a common gauge line over a fixed XY is the
        # same machine as ToolChangePosition: both plans post.
        drill, spot = tools()
        table = {t.id: (0.0, 0.0, 100 - t.gauge_length_mm) for t in (drill, spot)}
        m = machine(table)
        a = posted(job(ToolChangePosition(gauge_z_mm=100, xy_mm=(0, 0))), m)
        b = posted(job(ToolChangeTable.from_machine(m)), m)
        self.assertEqual(a, b)


class PrePostCheckTest(unittest.TestCase):
    def test_table_machine_without_setup_pose_is_refused_naming_the_fix(self):
        with self.assertRaisesRegex(ValueError, r"tool drill3: .*\(0, 0, 40\).*\(0\.0, 0\.0, 45\.0\).*from_machine"):
            job(None).post_nc("top", machine=machine(TABLE), accept={"unchecked_machine"})

    def test_wrong_gauge_line_is_refused_before_posting(self):
        drill, spot = tools()
        m = machine({t.id: (0.0, 0.0, 100 - t.gauge_length_mm) for t in (drill, spot)})
        with self.assertRaisesRegex(
            ValueError, "tool drill3: the setup starts it at .*Setup.tool_change is ToolChangePosition"
        ):
            job(ToolChangePosition(gauge_z_mm=90, xy_mm=(0, 0))).post_nc("top", machine=m, accept={"unchecked_machine"})
        # The replayed post is checked the same way, before anything runs.
        with self.assertRaisesRegex(ValueError, "tool drill3"):
            job(ToolChangePosition(gauge_z_mm=90, xy_mm=(0, 0))).post("top", machine=m)

    def test_preserve_tip_machine_refuses_a_setup_pose(self):
        m = machine(dict.fromkeys(TABLE, (0.0, 0.0, 40.0)), mode="preserve_tip")
        with self.assertRaisesRegex(ValueError, "preserve_tip.*Setup\\(tool_change=None\\)"):
            job(ToolChangeTable(TABLE)).post_nc("top", machine=m, accept={"unchecked_machine"})

    def test_z_home_machine_refuses_a_fixed_xy_and_a_table(self):
        m = machine(
            dict.fromkeys(TABLE, (0.0, 0.0, 40.0)),
            mode="z_home",
            tool_change_home_z_mm=100,
            work_to_machine=WorkFrame(),
        )
        for change in (ToolChangePosition(gauge_z_mm=100, xy_mm=(0, 0)), ToolChangeTable(TABLE)):
            with self.assertRaisesRegex(ValueError, r"Z home.*ToolChangePosition\(gauge_z_mm=100"):
                job(change).post_nc("top", machine=m, accept={"unchecked_machine"})
        with self.assertRaisesRegex(ValueError, "tool drill3: .*Z home: gauge line at work Z 100"):
            job(ToolChangePosition(gauge_z_mm=95)).post_nc("top", machine=m, accept={"unchecked_machine"})


class TableValueTest(unittest.TestCase):
    def test_validation_and_machine_table(self):
        with self.assertRaises(ValueError):
            ToolChangeTable({})
        with self.assertRaises(ValueError):
            ToolChangeTable({"a": (0, 0, float("nan"))})
        table = ToolChangeTable({"b": (1, 2, 3), "a": (4, 5, 6)})
        self.assertEqual(table.tips_mm, (("a", (4, 5, 6)), ("b", (1, 2, 3))))
        self.assertEqual(table.to_dict(), {"tips_mm": {"a": [4.0, 5.0, 6.0], "b": [1.0, 2.0, 3.0]}})
        self.assertEqual(ToolChangeTable.from_machine(machine(TABLE)).tip("drill5"), (5.0, -3.0, 38.0))
        with self.assertRaisesRegex(ValueError, "not a table"):
            ToolChangeTable.from_machine(machine(TABLE, mode="preserve_tip"))
        with self.assertRaisesRegex(ValueError, "no entry for tool"):
            ToolChangeTable.from_machine(machine(TABLE), tools=("drill5", "other"))

    def test_engine_refuses_a_table_for_an_undeclared_tool(self):
        with self.assertRaisesRegex(Exception, "tool change pose needs finite work coordinates, for declared tools"):
            with job(ToolChangeTable({**TABLE, "ghost": (0, 0, 50)})).regenerate() as run:
                list(run)


def gauge_tools():
    """Tools of 75 and 115 mm gauge length (#168)."""
    short = camlib.ToolAssembly(
        "short",
        camlib.Drill("short", diameter=3, point_angle=118, flute_length=12, overall_length=50),
        30,
        75,
        (camlib.HolderSection(30, 75, 10, 10),),
    )
    long = camlib.ToolAssembly(
        "long",
        camlib.Drill("long", diameter=5, point_angle=118, flute_length=20, overall_length=60),
        40,
        115,
        (camlib.HolderSection(40, 115, 10, 10),),
    )
    return short, long


def in_place_job(order, clearance, tool_change=ToolChangeInPlace()):
    """Two drilling operations, one per tool in ``order``; ``clearance`` is
    each operation's clearance height (one value: the same for both)."""
    by_id = {t.id: t for t in gauge_tools()}
    clearances = clearance if isinstance(clearance, tuple) else (clearance, clearance)
    ops = tuple(
        camlib.Drilling(
            f"drill{k}",
            by_id[tool],
            HOLES,
            12.0 - 4.0 * k,
            clearances[k - 1],
            tool_number=k,
            length_register=k,
            feed_mm_min=150,
            retract_feed_mm_min=600,
            spindle_rpm=6000,
            coolant="flood",
        )
        for k, tool in enumerate(order, 1)
    )
    setup = camlib.Setup(
        "top",
        stock=camlib.StockSpec((0, 0, 0), (28, 16, 10), 0.1),
        tools=tuple(by_id.values()),
        operations=tuple(camlib.OperationNode(op, sources=()) for op in ops),
        initial_position_mm=(0, 0, 70),
        verification=camlib.StreamChecks(stock_contact=camlib.StockContactOptions()),
        tool_change=tool_change,
    )
    return camlib.Job("tool-change-in-place", "1", (setup,))


def in_place_machine(order, mode="preserve_spindle", **settings):
    by_id = {t.id: t for t in gauge_tools()}
    table = tuple(
        ToolSetting.for_tool(by_id[t], number=k, length_register=k, position_after_mm=(0, 0, 70))
        for k, t in enumerate(order, 1)
    )
    return machine({}, mode=mode, tool_table=table, **settings)


def planned(j):
    """(tool-change poses, each tool's motions) of a regenerated job."""
    from camlib.simulation import OperationUpdate

    poses, moves = [], {}
    with j.regenerate() as run:
        for event in run:
            update = event.update
            if isinstance(update, OperationUpdate):
                if update.stage == "begin":
                    poses += [
                        (e["tool"], tuple(e["position_after_mm"])) for e in update.events if e["kind"] == "tool_change"
                    ]
            elif hasattr(update, "motion"):
                moves.setdefault(update.motion["tool"], []).append(update.motion)
    return poses, moves


class ToolChangeInPlaceTest(unittest.TestCase):
    """A machine whose tool change does not move the spindle (LinuxCNC M6
    with no configured tool-change move, #168): the gauge line stays, so the
    tip moves by the difference of the gauge lengths."""

    def test_planner_moves_the_tip_by_the_gauge_length_difference(self):
        for order, shift in ((("short", "long"), -40.0), (("long", "short"), 40.0)):
            with self.subTest(order=order):
                poses, moves = planned(in_place_job(order, clearance=(60, 12)))
                first, second = order
                end = tuple(moves[first][-1]["curve"]["end"])
                self.assertEqual(poses[0], (first, (0.0, 0.0, 70.0)))
                # The spindle stays over the last hole; only the tip height differs.
                self.assertEqual(poses[1], (second, (end[0], end[1], end[2] + shift)))
                self.assertEqual(tuple(moves[second][0]["curve"]["start"]), poses[1][1])

    def test_two_tool_linuxcnc_program_posts_decodes_and_replays(self):
        for order in (("short", "long"), ("long", "short")):
            with self.subTest(order=order):
                m = in_place_machine(order)
                self.assertEqual(m.dialect, GCodeDialect.linuxcnc())
                j = in_place_job(order, clearance=(60, 12))
                # The posted program is decoded with the machine's tool-change
                # behaviour and compared with the plan, then replayed on stock.
                nc = posted(j, m)
                self.assertEqual(sum("M6" in line.split() for line in nc), 2)
                self.assertEqual(sum(line.split()[0] == "G43" for line in nc if line.split()), 2)
                released = j.post_nc("top", machine=m, accept={"unchecked_machine"})
                self.assertEqual(released.text.splitlines(), nc)
                self.assertEqual(tuple(released.findings), ())
                # The Rust post (its own decoder and replay) writes the same program.
                from camlib.post import post_job, post_job_input

                rust = post_job(post_job_input(j, "top", machine=m, accept={"unchecked_machine"}))
                self.assertTrue(rust["released"])
                self.assertEqual(rust["program"], released.text)

    def test_playback_seeks_across_the_change_from_every_checkpoint(self):
        # A checkpoint does not retain the tool of its last move: a seek that
        # resumes right at the change still takes the new tool's first move.
        from camlib.playback import Cursor, Player

        for order in (("short", "long"), ("long", "short")):
            with self.subTest(order=order):
                j = in_place_job(order, clearance=(60, 12))
                with j.post("top", machine=in_place_machine(order), checkpoint_every=1) as run:
                    run.wait()
                    count = run.status()["available"]
                    player = Player(run, compact=True)
                    for cursor in (*range(count, -1, -1), *range(count + 1)):
                        player.seek(Cursor(cursor))
                    self.assertEqual(run.status()["error"], None)

    def test_stock_state_takes_only_the_gauge_line_pose_also_after_a_restore(self):
        import tempfile
        from pathlib import Path

        from camlib.motion import Line, Motion
        from camlib.simulation import Simulation, StockState

        def simulation():
            return Simulation(
                camlib.StockSpec((0, 0, 0), (28, 16, 10), 0.5),
                gauge_tools(),
                revision="1",
                initial_position_mm=(0, 0, 70),
                tool_change=ToolChangeInPlace(),
            )

        first = Motion(1, "a", "short", Line((0, 0, 70), (5, 5, 60)), None, "rapid")
        # The 115 mm tool's tip is 40 mm below where the 75 mm tool stopped.
        second = Motion(2, "b", "long", Line((5, 5, 20), (5, 5, 30)), None, "rapid")
        with simulation() as sim, tempfile.TemporaryDirectory() as directory:
            frames = [u.frame for u in sim.motions([first, second])]
            for many in (False, True):
                state = StockState(sim.manifest_frame)
                state.accept(frames[0])
                path = Path(directory) / "after-first.bin"
                state.write_checkpoint(path)
                for restored in (False, True):
                    if restored:
                        # The checkpoint does not say which tool made its last move.
                        state = StockState(sim.manifest_frame)
                        state.restore_checkpoint(path)
                    if many:
                        _, updates = state.accept_many(frames[1].raw, 0)
                        list(updates)
                    else:
                        state.accept(frames[1])
                    self.assertEqual(state.position, (5, 5, 30))
        # A start that is neither the old tip nor the gauge-line pose is refused.
        for z in (21, 100):
            wrong = Motion(2, "b", "long", Line((5, 5, z), (5, 5, 30)), None, "rapid")
            with simulation() as sim:
                with self.assertRaisesRegex(Exception, "continuity"):
                    list(sim.motions([first, wrong]))

    def test_decoder_places_the_tip_from_the_gauge_lengths_alone(self):
        # The decoder knows only the machine: the same blocks read with a
        # tip-preserving machine leave the second tool 40 mm away.
        from camlib.motion import Program
        from camlib.post import GCodeDecoder

        order = ("short", "long")
        tips = {}
        for mode in ("preserve_spindle", "preserve_tip"):
            m = in_place_machine(order, mode=mode)
            program = Program("p", "1", gauge_tools(), (0, 0, 70), ())
            decoder = GCodeDecoder(m, program)
            decoder.position = (20.0, 8.0, 60.0)
            decoder._activate(1)
            first = decoder._length(1).position_after_mm
            decoder._activate(2)
            tips[mode] = (first, decoder._length(2).position_after_mm)
        self.assertEqual(tips["preserve_spindle"], ((20.0, 8.0, 60.0), (20.0, 8.0, 20.0)))
        self.assertEqual(tips["preserve_tip"], ((20.0, 8.0, 60.0), (20.0, 8.0, 60.0)))

    def test_longer_tool_changed_in_into_the_stock_is_found(self):
        # The 75 mm tool stops 2 mm above the stock; the 115 mm tool changed
        # in there reaches 38 mm into it.
        order = ("short", "long")
        found = (
            "start Z -28 is below clearance_z_mm 12.*leaves the spindle where it stands.*"
            "tip is at Z -28, 40 mm below the Z 12 where short stopped"
        )
        with self.assertRaisesRegex(Exception, found):
            in_place_job(order, clearance=12).post_nc(
                "top", machine=in_place_machine(order), accept={"unchecked_machine"}
            )
        with self.assertRaisesRegex(Exception, found):
            with in_place_job(order, clearance=12).regenerate() as run:
                list(run)
        # The other way round the shorter tool arrives 40 mm higher: clean.
        order = ("long", "short")
        released = in_place_job(order, clearance=12).post_nc(
            "top", machine=in_place_machine(order), accept={"unchecked_machine"}
        )
        self.assertEqual(tuple(released.findings), ())

    def test_setup_and_machine_must_both_declare_it(self):
        order = ("short", "long")
        spindle = in_place_machine(order)
        for change in (None, ToolChangePosition(gauge_z_mm=150), ToolChangeTable({"short": (0, 0, 70)})):
            with self.assertRaisesRegex(ValueError, r"preserve_spindle.*ToolChangeInPlace\(\)"):
                in_place_job(order, (60, 12), change).post_nc("top", machine=spindle, accept={"unchecked_machine"})
            with self.assertRaisesRegex(ValueError, "ToolChangeInPlace"):
                in_place_job(order, (60, 12), change).post("top", machine=spindle)
        for mode, extra in (("preserve_tip", {}), ("table", {}), ("z_home", {"tool_change_home_z_mm": 200})):
            with self.assertRaisesRegex(ValueError, f'ToolChangeInPlace.*"{mode}".*preserve_spindle'):
                in_place_job(order, (60, 12)).post_nc(
                    "top", machine=in_place_machine(order, mode=mode, **extra), accept={"unchecked_machine"}
                )

    def test_machine_refuses_the_mode_where_it_is_not_modelled(self):
        with self.assertRaisesRegex(ValueError, "work Z axis along machine Z"):
            in_place_machine(("short", "long"), work_to_machine=WorkFrame(x=(1, 0, 0), y=(0, 0, 1), z=(0, -1, 0)))
        with self.assertRaisesRegex(ValueError, "must be table, preserve_tip, preserve_spindle or z_home"):
            in_place_machine(("short", "long"), mode="in_place")
        self.assertEqual(ToolChangeInPlace().to_dict(), {"in_place": True})


if __name__ == "__main__":
    unittest.main()
