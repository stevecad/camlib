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


def gauge_tools(long=115):
    """Tools of 75 and 115 mm gauge length (#168); ``long``: the longer
    tool's gauge length."""
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
        long,
        (camlib.HolderSection(40, long, 10, 10),),
    )
    return short, long


CONTACT = camlib.StreamChecks(stock_contact=camlib.StockContactOptions())


def in_place_job(order, clearance, tool_change=ToolChangeInPlace(), long=115, verification=CONTACT):
    """Two drilling operations, one per tool in ``order``; ``clearance`` is
    each operation's clearance height (one value: the same for both)."""
    by_id = {t.id: t for t in gauge_tools(long)}
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
        verification=verification,
        tool_change=tool_change,
    )
    return camlib.Job("tool-change-in-place", "1", (setup,))


def in_place_machine(order, mode="preserve_spindle", long=115, **settings):
    by_id = {t.id: t for t in gauge_tools(long)}
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

    def consumer_frames(self, stock, first_tool):
        """(manifest frame, [first move by ``first_tool``, the 115 mm tool's
        first move after its change in place]) of one run; every call gives
        the same run (id, revision, manifest), so its frames and checkpoints
        fit a consumer of any other call."""
        from camlib.motion import Line, Motion
        from camlib.simulation import Simulation

        mid = camlib.ToolAssembly(
            "mid",
            camlib.Drill("mid", diameter=4, point_angle=118, flute_length=16, overall_length=55),
            35,
            95,
            (camlib.HolderSection(35, 95, 10, 10),),
        )
        gauge = {"short": 75, "mid": 95}[first_tool]
        moves = [
            Motion(1, "a", first_tool, Line((0, 0, 70), (5, 5, 60)), None, "rapid"),
            # The spindle stays: the 115 mm tool's tip is lower by the difference.
            Motion(2, "b", "long", Line((5, 5, 60 + gauge - 115), (5, 5, 30)), None, "rapid"),
        ]
        with Simulation(
            stock,
            (*gauge_tools(), mid),
            revision="1",
            run_id="in-place",
            initial_position_mm=(0, 0, 70),
            tool_change=ToolChangeInPlace(),
        ) as sim:
            frames = [u.frame for u in sim.motions(moves)]
            return sim.manifest_frame, frames

    def test_consumers_measure_the_change_from_exactly_the_spindle_tool_also_after_a_restore(self):
        # After the 75 mm tool the 115 mm tool's tip is at Z20; after the
        # 95 mm tool it would be at Z40. A consumer takes only the pose
        # measured from the tool that is in the spindle, live or restored
        # from a checkpoint (which records that tool), on both stock models.
        import tempfile
        from pathlib import Path

        from camlib.dexel_state import DexelStockState
        from camlib.simulation import ProtocolError, StockState, stock_state

        stocks = {
            StockState: camlib.StockSpec((0, 0, 0), (28, 16, 10), 0.5),
            DexelStockState: camlib.DexelStockSpec((0, 0, 0), (28, 16, 10), cell_size_mm=0.5),
        }
        for kind, stock in stocks.items():
            manifest, (short_first, after_short) = self.consumer_frames(stock, "short")
            other, (mid_first, after_mid) = self.consumer_frames(stock, "mid")
            self.assertEqual(manifest.raw, other.raw)
            for first, right, wrong in ((short_first, after_short, after_mid), (mid_first, after_mid, after_short)):
                with self.subTest(kind=kind.__name__, first=first), tempfile.TemporaryDirectory() as directory:
                    live = stock_state(manifest)
                    self.assertIsInstance(live, kind)
                    live.accept(first)
                    path = Path(directory) / "after-first.bin"
                    live.write_checkpoint(path)

                    def consumer(restored):
                        state = stock_state(manifest)
                        if restored:
                            state.restore_checkpoint(path)
                        else:
                            state.accept(first)
                        return state

                    for restored in (False, True):
                        for many in (False, True):
                            if many and kind is DexelStockState:
                                continue
                            state = consumer(restored)
                            with self.assertRaisesRegex(ProtocolError, "continuity"):
                                if many:
                                    list(state.accept_many(wrong.raw, 0)[1])
                                else:
                                    state.accept(wrong)
                            self.assertEqual(state.completed, 1)
                            if many:
                                list(state.accept_many(right.raw, 0)[1])
                            else:
                                state.accept(right)
                            self.assertEqual(tuple(state.position), (5, 5, 30))
                        # The pose a consumer computes for the change, and
                        # the only position it lets the run be given.
                        state = consumer(restored)
                        tool = "short" if first is short_first else "mid"
                        tip = state.tool_change_tip("long", previous=tool)
                        self.assertEqual(tip, (5, 5, 60 + {"short": 75, "mid": 95}[tool] - 115))
                        if kind is DexelStockState:
                            # The tri-dexel state lets the run be given only a
                            # pose some tool has after a change from the tool
                            # in the spindle (its receiver then checks the
                            # move's own tool exactly, as above).
                            gauges = {"short": 75, "mid": 95, "long": 115}
                            valid = {(5, 5, 60 + gauges[tool] - g) for g in gauges.values()}
                            state.position = tip
                            other = {"short": "mid", "mid": "short"}[tool]
                            for wrong_tip in ((5, 5, tip[2] + 1), (5, 5, 60 + gauges[other] - 115)):
                                if wrong_tip in valid:
                                    # The 115 mm tool's pose after the 95 mm
                                    # tool is the 95 mm tool's after the 75 mm one.
                                    self.assertEqual((tool, wrong_tip), ("short", (5, 5, 40)))
                                    continue
                                with self.assertRaisesRegex(ProtocolError, "declared tool-change pose"):
                                    state.position = wrong_tip

    def test_checkpoint_without_the_spindle_tool_restores_and_refuses_a_change_in_place(self):
        # A checkpoint written before the spindle tool was recorded: flag
        # cleared, its four bytes removed, length and checksum recomputed.
        import struct
        import tempfile
        import zlib
        from pathlib import Path

        from camlib.simulation import ProtocolError, StockState

        def older(raw, run_id="in-place", revision="1"):
            flags = 20 + 2 + len(run_id) + 2 + len(revision) + 32 + 8 + 24
            tool = flags + 1 + 8 + 8 + 4
            self.assertEqual(raw[flags], 3)
            old = bytearray(raw[:tool] + raw[tool + 4 :])
            old[flags] = 1
            old[12:16] = struct.pack("<I", len(old) - 20)
            old[16:20] = struct.pack("<I", zlib.crc32(memoryview(old)[20:], zlib.crc32(memoryview(old)[:16])))
            return bytes(old)

        stock = camlib.StockSpec((0, 0, 0), (28, 16, 10), 0.5)
        manifest, (first, second) = self.consumer_frames(stock, "short")
        with tempfile.TemporaryDirectory() as directory:
            state = StockState(manifest)
            state.accept(first)
            path = Path(directory) / "new.bin"
            state.write_checkpoint(path)
            old = Path(directory) / "old.bin"
            old.write_bytes(older(path.read_bytes()))
            for many in (False, True):
                restored = StockState(manifest)
                restored.restore_checkpoint(old)
                self.assertEqual((restored.completed, restored.position), (1, (5, 5, 60)))
                with self.assertRaisesRegex(ProtocolError, "does not record the tool in the spindle.*replay the run"):
                    if many:
                        list(restored.accept_many(second.raw, 0)[1])
                    else:
                        restored.accept(second)
                self.assertEqual(restored.completed, 1)
                # It stays unknown in the checkpoints this state writes.
                again = Path(directory) / "again.bin"
                restored.write_checkpoint(again)
                self.assertEqual(again.read_bytes(), old.read_bytes())

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
        with self.assertRaisesRegex(
            ValueError, "must be table, preserve_tip, preserve_spindle, preserve_carriage or z_home"
        ):
            in_place_machine(("short", "long"), mode="in_place")
        self.assertEqual(ToolChangeInPlace().to_dict(), {"in_place": True})


class PlannedRetractTest(unittest.TestCase):
    """A stated retract before every tool change in place (#374): the old
    tool goes straight up until the spindle gauge line is at the stated work
    Z, as an ordinary checked, posted and decoded rapid. Tools of 75 and
    130 mm gauge length: the second is 55 mm longer."""

    ORDER = ("short", "long")

    def job(self, retract, clearance=12, **options):
        change = ToolChangeInPlace() if retract is None else ToolChangeInPlace(retract_gauge_z_mm=retract)
        return in_place_job(self.ORDER, clearance, change, long=130, **options)

    def machine(self, **settings):
        return in_place_machine(self.ORDER, long=130, **settings)

    def test_value(self):
        self.assertEqual(ToolChangeInPlace().to_dict(), {"in_place": True})
        self.assertEqual(
            ToolChangeInPlace(retract_gauge_z_mm=150).to_dict(), {"in_place": True, "retract_gauge_z_mm": 150.0}
        )
        for bad in (float("nan"), float("inf"), 1e7, "150", True):
            with self.assertRaisesRegex(ValueError, "retract_gauge_z_mm"):
                ToolChangeInPlace(retract_gauge_z_mm=bad)

    def test_the_planner_retracts_the_old_tool_to_the_stated_gauge_line(self):
        poses, moves = planned(self.job(150))
        last = moves["short"][-1]
        before = moves["short"][-2]["curve"]["end"]
        # One rapid straight up from where the 75 mm tool stopped (Z 12),
        # until its gauge line is at Z 150: its tip at Z 75.
        self.assertEqual(last["kind"], "rapid")
        self.assertEqual(last["curve"]["start"], before)
        self.assertEqual(tuple(last["curve"]["end"]), (before[0], before[1], 75.0))
        self.assertEqual(before[2], 12.0)
        # The 130 mm tool is changed in there: its tip 55 mm lower, at Z 20.
        self.assertEqual(poses[1], ("long", (before[0], before[1], 20.0)))
        self.assertEqual(tuple(moves["long"][0]["curve"]["start"]), poses[1][1])
        # Without the value the plan has no such move.
        _, plain = planned(in_place_job(self.ORDER, (70, 12), long=130))
        _, stated = planned(in_place_job(self.ORDER, (70, 12), ToolChangeInPlace(retract_gauge_z_mm=150), long=130))
        self.assertEqual(len(stated["short"]), len(plain["short"]) + 1)
        self.assertEqual(stated["short"][:-1], plain["short"])

    def test_two_tool_linuxcnc_program_posts_one_z_rapid_before_the_change_and_decodes(self):
        m = self.machine()
        self.assertEqual(m.dialect, GCodeDialect.linuxcnc())
        j = self.job(150)
        # Posted, decoded with the machine's tool change (which knows no
        # retract: it only keeps the spindle), compared with the plan and
        # replayed on the stock.
        nc = posted(j, m)
        change = nc.index("T2")
        # One Z rapid from where the 75 mm tool stopped, then the stops and
        # the change; the 130 mm tool's first move comes straight down the
        # same XY from Z 20 to its clearance.
        self.assertEqual(
            nc[change - 5 : change + 6],
            [
                "G1 X20 Y8 Z12 F600",
                "G0 X20 Y8 Z75",
                "M9",
                "M5",
                "G54",
                "T2",
                "M6",
                "G43 H2",
                "M3 S6000",
                "M8",
                "G0 X20 Y8 Z12",
            ],
        )
        self.assertEqual(sum("Z75" in line for line in nc), 1)
        released = j.post_nc("top", machine=m, accept={"unchecked_machine"})
        self.assertEqual(released.text.splitlines(), nc)
        self.assertEqual(tuple(released.findings), ())
        # The Rust post (its own decoder and replay) writes the same program.
        from camlib.post import post_job, post_job_input

        rust = post_job(post_job_input(j, "top", machine=m, accept={"unchecked_machine"}))
        self.assertTrue(rust["released"])
        self.assertEqual(rust["program"], released.text)

    def test_without_the_value_the_job_is_refused_as_a_tool_change(self):
        for run in (
            lambda j: j.post_nc("top", machine=self.machine(), accept={"unchecked_machine"}),
            lambda j: list(j.regenerate()),
        ):
            with self.assertRaises(Exception) as caught:
                run(self.job(None))
            text = str(caught.exception)
            self.assertRegex(
                text,
                r"operation drill2 cannot start after its tool change \(start Z -43 is below clearance_z_mm 12\): "
                r"the tool change from short to long leaves the spindle where it stands.*tip is at Z -43, 55 mm below "
                r"the Z 12 where short stopped.*ToolChangeInPlace\(retract_gauge_z_mm=",
            )
            self.assertNotIn("invalid hole parameters", text)

    def test_stated_and_unstated_programs_differ_only_by_the_retract(self):
        # A job that needs no retract (the first tool's clearance is high
        # enough): stating one adds exactly the retract and the longer
        # descent after the change.
        m = self.machine()
        plain = posted(in_place_job(self.ORDER, (70, 12), long=130), m)
        stated = posted(in_place_job(self.ORDER, (70, 12), ToolChangeInPlace(retract_gauge_z_mm=150), long=130), m)
        self.assertEqual([line for line in stated if line != "G0 X20 Y8 Z75"], plain)
        self.assertEqual(len(stated), len(plain) + 1)

    def test_a_retract_too_low_for_the_longer_tool_is_refused_naming_the_change(self):
        # Gauge Z 140: the 130 mm tool arrives with its tip at Z 10, below
        # its operation's clearance 12.
        with self.assertRaisesRegex(
            Exception,
            r"operation drill2 cannot start after its tool change \(start Z 10 is below clearance_z_mm 12\): the tool "
            r"change from short to long is made in place after the stated retract to gauge Z 140 .*leaves long's tip "
            r"at Z 10: state a retract_gauge_z_mm of at least the operation's clearance_z_mm plus long's gauge "
            r"length 130",
        ):
            self.job(140).post_nc("top", machine=self.machine(), accept={"unchecked_machine"})
        # A retract never goes down: gauge Z 80 is below where the 75 mm
        # tool stopped (tip Z 12, gauge line Z 87).
        with self.assertRaisesRegex(
            Exception,
            r"tool change from short to long in place: the stated retract to gauge Z 80 .*would lower short's tip "
            r"to Z 5, below the Z 12 where operation drill1 left it.*at least 87",
        ):
            list(self.job(80).regenerate())
        # Exactly at the gauge line where the tool stopped: no move at all.
        _, moves = planned(self.job(145, clearance=(70, 12)))
        _, plain = planned(self.job(None, clearance=(70, 12)))
        self.assertEqual(moves, plain)

    def test_a_retract_outside_the_machine_travel_is_refused_by_the_machine_check(self):
        from camlib.export import ExportRefused
        from camlib.verification import MachineLimits, Scene

        def checks(max_gauge_z):
            limits = MachineLimits.gauge_line((-500, -500, 0), (500, 500, max_gauge_z), 10000)
            return camlib.StreamChecks(scene=Scene("mill", "1", machine=limits), stock_contact=CONTACT.stock_contact)

        m = self.machine()
        released = self.job(150, verification=checks(150)).post_nc("top", machine=m, accept={"unchecked_machine"})
        self.assertEqual(tuple(released.findings), ())
        _, moves = planned(self.job(150))
        retract = moves["short"][-1]["id"]
        with self.assertRaises(ExportRefused) as caught:
            self.job(150, verification=checks(149.5)).post_nc("top", machine=m, accept={"unchecked_machine"})
        first = caught.exception.findings[0]
        self.assertEqual((first["completed"], first["status"]), (retract, "violation"))
        self.assertRegex(str(caught.exception), rf"first at move {retract}: machine_")

    def test_a_retract_through_a_fixture_is_refused_naming_the_move(self):
        from camlib.export import ExportRefused
        from camlib.planar import Boundary, Component, PlanarRegion
        from camlib.verification import Prism, Scene

        # A bridge over the last hole, above everything the operations
        # reach (the 75 mm tool's holder top is at Z 87 while it drills):
        # only the retract takes the holder up through it.
        square = Boundary.polygon(((19, 6), (22, 6), (22, 10), (19, 10)))
        over = Prism("bridge", PlanarRegion("bridge", (Component(square),)), 90, 100)
        scene = camlib.StreamChecks(scene=Scene("mill", "1", bodies=(over,)), stock_contact=CONTACT.stock_contact)
        _, moves = planned(self.job(150))
        retract = moves["short"][-1]["id"]
        with self.assertRaises(ExportRefused) as caught:
            self.job(150, verification=scene).post_nc("top", machine=self.machine(), accept={"unchecked_machine"})
        self.assertEqual(caught.exception.findings[0]["completed"], retract)
        self.assertRegex(str(caught.exception), rf"first at move {retract}: .*bridge")

    def test_playback_seeks_across_the_retract_and_the_change_from_every_checkpoint(self):
        from camlib.playback import Cursor, Player

        j = self.job(150)
        with j.post("top", machine=self.machine(), checkpoint_every=1) as run:
            run.wait()
            count = run.status()["available"]
            player = Player(run, compact=True)
            for cursor in (*range(count, -1, -1), *range(count + 1)):
                player.seek(Cursor(cursor))
            self.assertEqual(run.status()["error"], None)
        # The planned run too, partway through the retract itself.
        with j.record("top", rapid_mm_min=6000, checkpoint_every=1) as run:
            run.wait()
            count = run.status()["available"]
            _, moves = planned(j)
            retract = moves["short"][-1]["id"]
            player = Player(run, compact=True)
            for cursor in (*range(count, -1, -1), *range(count + 1)):
                player.seek(Cursor(cursor))
            state = player.seek(Cursor(retract - 1, 0.5))
            self.assertEqual(tuple(state.position), (20.0, 8.0, 43.5))
            self.assertEqual(run.status()["error"], None)

    def test_a_regeneration_cache_replays_the_same_run(self):
        import tempfile

        from camlib.jobs import RegenerationCache

        with tempfile.TemporaryDirectory() as directory:
            cache = RegenerationCache(directory)

            def run():
                hits, moves = [], []
                with self.job(150).regenerate(cache) as updates:
                    for event in updates:
                        update = event.update
                        if hasattr(update, "motion"):
                            moves.append(update.motion)
                        elif getattr(update, "stage", None) == "end":
                            hits.append(update.report["regeneration"]["cache_hit"])
                return hits, moves

            cold, warm = run(), run()
            self.assertEqual((cold[0], warm[0]), ([False, False], [True, True]))
            self.assertEqual(cold[1], warm[1])
            self.assertEqual(sum(tuple(m["curve"]["end"])[2] == 75.0 for m in warm[1]), 1)

    def test_only_a_planned_milling_run_takes_a_retract(self):
        from camlib.motion import Line, Motion
        from camlib.simulation import Simulation, SimulationError

        # A run that is given its moves makes its own retracts.
        with self.assertRaisesRegex((SimulationError, ValueError, RuntimeError), "retract"):
            with Simulation(
                camlib.StockSpec((0, 0, 0), (28, 16, 10), 0.5),
                gauge_tools(),
                revision="1",
                run_id="given",
                initial_position_mm=(0, 0, 70),
                tool_change=ToolChangeInPlace(retract_gauge_z_mm=150),
            ) as sim:
                list(sim.motions([Motion(1, "a", "short", Line((0, 0, 70), (5, 5, 60)), None, "rapid")]))


if __name__ == "__main__":
    unittest.main()
