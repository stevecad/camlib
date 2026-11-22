"""Program end (#378): a setup states where its program ends. After the last
operation the tool goes straight up at rapid until the spindle gauge line is
at the stated work Z, then at rapid to the stated XY, as ordinary moves that
are streamed, checked, posted on every mill post and decoded. Without the
value the program is unchanged."""

import unittest

import camlib
from camlib import EndPosition
from camlib.export import ExportRefused
from camlib.motion import WorkFrame
from camlib.post import post_job, post_job_input
from test_drill_cycles import DrillCycleTest, example, linuxcnc, tool_table
from test_tapping import tools

HOLES = ((8, 8), (20, 8))
CONTACT = camlib.StreamChecks(stock_contact=camlib.StockContactOptions())
# The drill's gauge length is 60 mm: gauge Z 100 is its tip at Z 40.
END = EndPosition(gauge_z_mm=100, xy_mm=(-10, 30))
UP = EndPosition(gauge_z_mm=100)


def job(end_position=None, verification=CONTACT, coolant="flood", initial_z=12, **drilling):
    """Two drilled holes with one tool, ending 12 mm above the part over the
    last hole (X20 Y8) unless an end position is stated."""
    drill, _ = tools()
    op = camlib.Drilling(
        "holes",
        drill,
        HOLES,
        4,
        12,
        feed_mm_min=150,
        retract_feed_mm_min=600,
        spindle_rpm=6000,
        tool_number=1,
        length_register=1,
        coolant=coolant,
        **drilling,
    )
    setup = camlib.Setup(
        "top",
        stock=camlib.StockSpec((0, 0, 0), (28, 16, 10), 0.1),
        tools=(drill,),
        operations=(camlib.OperationNode(op, sources=()),),
        initial_position_mm=(0, 0, initial_z),
        verification=verification,
        end_position=end_position,
    )
    return camlib.Job("end", "1", (setup,))


def planned(j):
    """The motions of a regenerated job and the motion count at its
    operation's end."""
    from camlib.simulation import OperationUpdate

    moves, ends = [], []
    with j.regenerate() as run:
        for event in run:
            update = event.update
            if isinstance(update, OperationUpdate):
                if update.stage == "end":
                    ends.append(len(moves))
            elif hasattr(update, "motion"):
                moves.append(update.motion)
    return moves, ends


def decoded_end(j, machine, post=None, decoder=None):
    """(NC lines, the decoded program's last pose) of the posted job: the
    posted blocks are decoded, compared with the plan and replayed."""
    from camlib.playback import Cursor, Player

    options = {} if post is None else dict(post=post, decoder=decoder)
    with j.post("top", machine=machine, **options) as run:
        run.wait()
        status = run.status()
        assert status["finished"] and status["error"] is None, status
        nc, start = [], 1
        while page := run.nc_lines(start, limit=256):
            nc.extend(line["text"] for line in page)
            start = page[-1]["number"] + 1
        state = Player(run, compact=True).seek(Cursor(status["available"]))
        return nc, tuple(state.position)


def haas(j):
    return DrillCycleTest.haas(None, j)


def centroid(j):
    c = example("centroid_post")
    m = c.define_machine(
        "Acorn",
        tool_table(j),
        traverse_feed_mm_min=3000,
        work_to_machine=WorkFrame(),
        decimals=3,
        output_units="mm",
        max_feed_mm_min=10000,
        max_spindle_rpm=30000,
        planes=("xy", "xz", "yz"),
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
    return m, c.CentroidPost, c.CentroidDecoder


def fanuc(j):
    f = example("fanuc_0imb_post")
    oem = dict(tool_change="M6", spindle_cw="M3", spindle_ccw="M4", spindle_off="M5", coolant_off="M9", flood="M8")
    m = f.define_machine(
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
    )
    return m, f.Fanuc0iMBPost, f.Fanuc0iMBDecoder


def lnc(j):
    import test_lnc_mill_post as t

    return t.lnc(tool_table(j)), t.LNCMillPost, t.LNCMillDecoder


def siemens(j):
    import test_siemens808d_post as t

    m = t.define_machine(
        "Test OEM 808D",
        tool_table(j),
        tool_change="m6",
        coolant_codes=t.OEM,
        work_to_machine=WorkFrame(),
        max_feed_mm_min=10000,
        max_spindle_rpm=30000,
        max_sweep_rad=6.283185307179586,
        tool_changes=True,
        tool_change_position_mode="table",
        rapid_mm_min=6000,
        output_units="mm",
        decimals=3,
    )
    return m, t.Siemens808DPost, t.Siemens808DDecoder


def siemens_iso(j):
    import test_siemens808d_iso_post as t

    m = t.define_machine(
        "ISO milling example",
        tool_table(j),
        startup_units="mm",
        output_units="mm",
        decimals=3,
        extern_toolprog_mode=0,
        cutting_edge_default=0,
        tool_corr_move_mode=0,
        tool_length_const=17,
        tool_change="m6",
        coolant_codes=t.OEM,
        traverse_feed_mm_min=3000,
        work_to_machine=WorkFrame(),
        max_feed_mm_min=10000,
        max_spindle_rpm=30000,
        max_sweep_rad=6.283185307179586,
        max_tool_number=99,
        max_length_register=99,
        tool_changes=True,
        tool_change_position_mode="table",
        rapid_mm_min=6000,
    )
    return m, t.Siemens808DISOPost, t.Siemens808DISODecoder


def carvera(j):
    from test_rust_carvera_post import machine_for

    c = example("carvera_post")
    return machine_for(j), c.CarveraPost, c.CarveraDecoder


def grbl(j):
    import test_grbl_post as t
    from camlib.post import ToolSetting

    header = j.compile().setup("top").program_header()
    tool = ToolSetting.for_tool(
        header.tools[0], number=1, length_register=1, position_after_mm=header.initial_position_mm
    )
    # The preloaded tool's tip at the program start (work X0 Y0 Z12).
    m = t.define_machine(
        "GRBL router",
        tool,
        work_offset_machine_mm=(1000, -2000, 100),
        initial_machine_mm=(1000, -2000, 112),
        **t.ROUTER,
    )
    return m, t.GrblPost, t.GrblDecoder


MILL_POSTS = {
    "linuxcnc": lambda j: (linuxcnc(j), None, None),
    "haas": lambda j: (lambda h, m: (m, h.HaasNGCPost, h.HaasNGCDecoder))(*haas(j)),
    "centroid": centroid,
    "fanuc_0imb": fanuc,
    "lnc": lnc,
    "siemens808d": siemens,
    "siemens808d_iso": siemens_iso,
    "grbl": grbl,
}


class EndPositionValueTest(unittest.TestCase):
    def test_value(self):
        self.assertEqual(UP.to_dict(), {"gauge_z_mm": 100.0})
        self.assertEqual(END.to_dict(), {"gauge_z_mm": 100.0, "xy_mm": [-10.0, 30.0]})
        for bad in (
            dict(gauge_z_mm=float("nan")),
            dict(gauge_z_mm=1e7),
            dict(gauge_z_mm="100"),
            dict(gauge_z_mm=100, xy_mm=(1,)),
            dict(gauge_z_mm=100, xy_mm=(1, float("inf"))),
        ):
            with self.assertRaisesRegex(ValueError, "end position"):
                EndPosition(**bad)

    def test_without_the_value_the_setup_and_its_manifest_are_unchanged(self):
        config = job().compile().setup("top").config
        self.assertNotIn("end_position", config)
        self.assertEqual(job(END).compile().setup("top").config["end_position"], END.to_dict())
        # The value is part of the job: stating it is another revision.
        self.assertNotEqual(job().compile().revision, job(END).compile().revision)
        self.assertNotEqual(job(UP).compile().revision, job(END).compile().revision)


class PlannedEndTest(unittest.TestCase):
    def test_the_planner_ends_with_a_z_rapid_then_an_xy_rapid(self):
        plain, plain_ends = planned(job())
        self.assertEqual(tuple(plain[-1]["curve"]["end"]), (20.0, 8.0, 12.0))
        for end, tail in (
            (UP, [((20, 8, 12), (20, 8, 40))]),
            (END, [((20, 8, 12), (20, 8, 40)), ((20, 8, 40), (-10, 30, 40))]),
            # Already at that gauge height: only the XY rapid.
            (EndPosition(gauge_z_mm=72, xy_mm=(-10, 30)), [((20, 8, 12), (-10, 30, 12))]),
            # Already there: no move.
            (EndPosition(gauge_z_mm=72, xy_mm=(20, 8)), []),
            (EndPosition(gauge_z_mm=72), []),
        ):
            with self.subTest(end=end):
                moves, ends = planned(job(end))
                self.assertEqual(moves[: len(plain)], plain)
                extra = moves[len(plain) :]
                self.assertEqual(
                    [(tuple(m["curve"]["start"]), tuple(m["curve"]["end"])) for m in extra],
                    [(tuple(map(float, a)), tuple(map(float, b))) for a, b in tail],
                )
                self.assertTrue(all(m["kind"] == "rapid" and m["tool"] == "drill5" for m in extra))
                self.assertEqual([m["id"] for m in extra], list(range(len(plain) + 1, len(moves) + 1)))
                # They are the last operation's last motions, before its end.
                self.assertEqual(ends, [len(moves)])

    def test_only_the_last_operation_is_followed_by_the_end_moves(self):
        drill, _ = tools()

        def two(end):
            ops = tuple(
                camlib.Drilling(
                    f"holes{k}",
                    drill,
                    HOLES,
                    6 - 2 * k,
                    12,
                    feed_mm_min=150,
                    retract_feed_mm_min=600,
                    spindle_rpm=6000,
                    tool_number=1,
                    length_register=1,
                    coolant="flood",
                )
                for k in (0, 1)
            )
            setup = camlib.Setup(
                "top",
                stock=camlib.StockSpec((0, 0, 0), (28, 16, 10), 0.1),
                tools=(drill,),
                operations=tuple(camlib.OperationNode(op, sources=()) for op in ops),
                initial_position_mm=(0, 0, 12),
                verification=CONTACT,
                end_position=end,
            )
            return camlib.Job("end", "1", (setup,))

        plain, _ = planned(two(None))
        moves, _ = planned(two(END))
        self.assertEqual(moves[: len(plain)], plain)
        self.assertEqual([m["operation"] for m in moves[len(plain) :]], ["holes1", "holes1"])

    def test_an_end_position_below_where_the_tool_stopped_is_refused(self):
        with self.assertRaisesRegex(
            Exception,
            r"end position: the stated gauge Z 70 \(EndPosition\(gauge_z_mm=70\)\) would lower drill5's tip to Z 10, "
            r"below the Z 12 where the last operation holes left it: the end move only goes up; state a gauge_z_mm "
            r"of at least 72",
        ):
            list(job(EndPosition(gauge_z_mm=70)).regenerate())

    def test_an_end_position_outside_travel_is_refused_naming_the_move(self):
        from camlib.verification import MachineLimits, Scene

        def checks(max_gauge, min_x=-500):
            limits = MachineLimits.gauge_line((min_x, -500, 0), (500, 500, max_gauge), 10000)
            return camlib.StreamChecks(scene=Scene("mill", "1", machine=limits), stock_contact=CONTACT.stock_contact)

        j = job(END)
        m = linuxcnc(j)
        moves, _ = planned(j)
        z_move, xy_move = moves[-2]["id"], moves[-1]["id"]
        released = job(END, verification=checks(100, -10)).post_nc("top", machine=m, accept={"unchecked_machine"})
        self.assertEqual(tuple(released.findings), ())
        for verification, move in ((checks(99.5), z_move), (checks(100, -9.5), xy_move)):
            with self.assertRaises(ExportRefused) as caught:
                job(END, verification=verification).post_nc("top", machine=m, accept={"unchecked_machine"})
            first = caught.exception.findings[0]
            self.assertEqual((first["completed"], first["status"]), (move, "violation"))
            self.assertRegex(str(caught.exception), rf"first at move {move}: machine_")

    def test_an_end_position_through_a_fixture_is_refused_naming_the_move(self):
        from camlib.planar import Boundary, Component, PlanarRegion
        from camlib.verification import Prism, Scene

        # A clamp standing between the last hole and the end XY, up to Z 45.
        square = Boundary.polygon(((2, 18), (8, 18), (8, 24), (2, 24)))
        clamp = Prism("clamp", PlanarRegion("clamp", (Component(square),)), 0, 45)
        scene = camlib.StreamChecks(scene=Scene("mill", "1", bodies=(clamp,)), stock_contact=CONTACT.stock_contact)
        j = job(END, verification=scene)
        moves, _ = planned(job(END))
        with self.assertRaises(ExportRefused) as caught:
            j.post_nc("top", machine=linuxcnc(j), accept={"unchecked_machine"})
        self.assertEqual(caught.exception.findings[0]["completed"], moves[-1]["id"])
        self.assertRegex(str(caught.exception), rf"first at move {moves[-1]['id']}: .*clamp")
        # Above the clamp the same end position is clear.
        high = EndPosition(gauge_z_mm=110, xy_mm=(-10, 30))
        j = job(high, verification=scene)
        self.assertEqual(tuple(j.post_nc("top", machine=linuxcnc(j), accept={"unchecked_machine"}).findings), ())
        # An avoid area on the way (below its top) refuses the plan itself.
        drill, _ = tools()
        avoid = camlib.AvoidArea(PlanarRegion("clamp", (Component(square),)), 45)
        base = job(END).setups[0]
        from dataclasses import replace

        blocked = camlib.Job("end", "1", (replace(base, avoid=(avoid,)),))
        with self.assertRaisesRegex(
            Exception,
            r"the end position's XY move \(a rapid from \[20\.0, 8\.0, 40\.0\] to \[-10\.0, 30\.0, 40\.0\]\): .*avoid area",
        ):
            list(blocked.regenerate())

    def test_the_run_seeks_from_every_checkpoint_and_replays_from_a_cache(self):
        import tempfile

        from camlib.jobs import RegenerationCache
        from camlib.playback import Cursor, Player

        j = job(END)
        with j.post("top", machine=linuxcnc(j), checkpoint_every=1) as run:
            run.wait()
            count = run.status()["available"]
            player = Player(run, compact=True)
            for cursor in (*range(count, -1, -1), *range(count + 1)):
                player.seek(Cursor(cursor))
            state = player.seek(Cursor(count - 1, 0.5))
            self.assertEqual(tuple(state.position), (5.0, 19.0, 40.0))
            self.assertEqual(run.status()["error"], None)
        with tempfile.TemporaryDirectory() as directory:
            cache = RegenerationCache(directory)

            def run_cached(end):
                hits, moves = [], []
                with job(end).regenerate(cache) as updates:
                    for event in updates:
                        update = event.update
                        if hasattr(update, "motion"):
                            moves.append(update.motion)
                        elif getattr(update, "stage", None) == "end":
                            hits.append(update.report["regeneration"]["cache_hit"])
                return hits, moves

            cold, warm = run_cached(END), run_cached(END)
            self.assertEqual((cold[0], warm[0]), ([False], [True]))
            self.assertEqual(cold[1], warm[1])
            # The operation's cache entry is the operation alone: the same
            # entry serves the job without an end position.
            hits, moves = run_cached(None)
            self.assertEqual((hits, moves), ([True], warm[1][:-2]))

    def test_where_an_end_position_does_not_apply_it_is_refused(self):
        from camlib.motion import Line, Motion
        from camlib.simulation import Simulation, SimulationError

        drill, _ = tools()
        # A run that is given its moves makes its own end move.
        with self.assertRaisesRegex((SimulationError, ValueError, RuntimeError), "end position"):
            with Simulation(
                camlib.StockSpec((0, 0, 0), (28, 16, 10), 0.5),
                (drill,),
                revision="1",
                run_id="given",
                initial_position_mm=(0, 0, 12),
                end_position=UP,
            ) as sim:
                list(sim.motions([Motion(1, "a", "drill5", Line((0, 0, 12), (5, 5, 12)), None, "rapid")]))


class PostedEndTest(unittest.TestCase):
    """Each mill post writes the end moves as ordinary blocks after the last
    operation's last move and before the spindle and coolant stop and the
    program end; the decoded program ends at the planned pose."""

    def test_linuxcnc_program_end(self):
        j = job(END)
        m = linuxcnc(j)
        nc, pose = decoded_end(j, m)
        self.assertEqual(nc[-7:], ["G1 X20 Y8 Z12 F600", "G0 X20 Y8 Z40", "G0 X-10 Y30 Z40", "M9", "M5", "G49", "M2"])
        self.assertEqual(pose, (-10.0, 30.0, 40.0))
        plain, plain_pose = decoded_end(job(), m)
        self.assertEqual(plain[-5:], ["G1 X20 Y8 Z12 F600", "M9", "M5", "G49", "M2"])
        self.assertEqual(plain_pose, (20.0, 8.0, 12.0))
        # The two rapids are all that differs.
        self.assertEqual(nc[:-6] + nc[-4:], plain)
        up, up_pose = decoded_end(job(UP), m)
        self.assertEqual(up[:-5] + up[-4:], plain)
        self.assertEqual(up[-5], "G0 X20 Y8 Z40")
        self.assertEqual(up_pose, (20.0, 8.0, 40.0))
        released = j.post_nc("top", machine=m, accept={"unchecked_machine"})
        self.assertEqual(released.text.splitlines(), nc)
        self.assertEqual(tuple(released.findings), ())
        # The Rust post (its own decoder and replay) writes the same program.
        rust = post_job(post_job_input(j, "top", machine=m, accept={"unchecked_machine"}))
        self.assertTrue(rust["released"])
        self.assertEqual(rust["program"], released.text)

    # Each post's blocks from the last operation's last move to the end of
    # the program: (the move, the Z end move, the XY end move, the rest
    # without an end position, the rest with one). A post that declares a
    # traverse feed writes its rapids as G1 at that feed, these too. FANUC
    # and LNC cancel the length offset in a block that states the gauge
    # line's Z: with an end position that is the stated gauge Z.
    ENDS = {
        "linuxcnc": ("G1 X20 Y8 Z12 F600", "G0 X20 Y8 Z40", "G0 X-10 Y30 Z40", ["M9", "M5", "G49", "M2"], None),
        "haas": (
            "G1 X20. Y8. Z12. F600.",
            "G1 X20. Y8. Z40. F5000.",
            "G1 X-10. Y30. Z40. F5000.",
            ["M9", "M5", "G49", "M30", "%"],
            None,
        ),
        "centroid": (
            "G1 X20 Y8 Z12 F600",
            "G1 X20 Y8 Z40 F3000",
            "G1 X-10 Y30 Z40 F3000",
            ["M9", "M5", "G49", "M30"],
            None,
        ),
        "fanuc_0imb": (
            "G1 X20. Y8. Z12. F600.",
            "G1 X20. Y8. Z40. F3000.",
            "G1 X-10. Y30. Z40. F3000.",
            ["M9", "M5", "G1 G49 Z72. F3000.", "M30", "%"],
            ["M9", "M5", "G1 G49 Z100. F3000.", "M30", "%"],
        ),
        "lnc": (
            "G1 X20. Y8. Z12. F600.",
            "G1 X20. Y8. Z40. F3000.",
            "G1 X-10. Y30. Z40. F3000.",
            ["M9", "M5", "G1 G49 Z72. F3000.", "M30"],
            ["M9", "M5", "G1 G49 Z100. F3000.", "M30"],
        ),
        "siemens808d": ("G1 X20 Y8 Z12 F600", "G0 X20 Y8 Z40", "G0 X-10 Y30 Z40", ["M9", "M5", "D0", "M2"], None),
        "grbl": ("G1 Z12 F600", "G0 Z40", "G0 X-10 Y30 Z40", ["M9", "M5", "G49", "M2"], None),
        "siemens808d_iso": (
            "G1 X20. Y8. Z12. F600.",
            "G1 X20. Y8. Z40. F3000.",
            "G1 X-10. Y30. Z40. F3000.",
            ["M9", "M5", "G49", "M30"],
            None,
        ),
    }

    def test_every_mill_post_writes_the_end_moves_before_the_stop_and_decodes_them(self):
        self.assertEqual(set(self.ENDS), set(MILL_POSTS))
        for name, define in MILL_POSTS.items():
            with self.subTest(post=name):
                last, z_move, xy_move, rest, rest_stated = self.ENDS[name]
                rest_stated = rest_stated or rest
                plain_job = job()
                m, post, decoder = define(plain_job)
                plain, plain_pose = decoded_end(plain_job, m, post, decoder)
                self.assertEqual(plain_pose, (20.0, 8.0, 12.0))
                self.assertEqual(plain[-len(rest) - 1 :], [last, *rest])
                body = plain[: -len(rest)]
                for end, pose, moves in (
                    (UP, (20.0, 8.0, 40.0), [z_move]),
                    (END, (-10.0, 30.0, 40.0), [z_move, xy_move]),
                ):
                    nc, decoded = decoded_end(job(end), m, post, decoder)
                    # The decoded program ends at the planned end pose.
                    self.assertEqual(decoded, pose)
                    # The same program up to the last operation's last
                    # move, then the end moves, then the coolant and
                    # spindle stop and the program end.
                    self.assertEqual(nc, [*body, *moves, *rest_stated])

    def test_carvera_takes_its_final_retract_from_the_stated_end_position(self):
        # The Carvera post writes G53 G0 Z-3 before M30 only where the plan
        # already ends at that height, and refuses otherwise ("plan the
        # retract"). The synthetic machine's Z-3 is the gauge height the job
        # starts at: work Z 30 + 60 = 90. The drilling ends 18 mm lower.
        from test_rust_carvera_post import machine_for

        c = example("carvera_post")
        low = job(coolant="mist", initial_z=30)
        m = machine_for(low)
        with self.assertRaisesRegex(Exception, "plan the retract"):
            decoded_end(low, m, c.CarveraPost, c.CarveraDecoder)
        for end, pose, blocks in (
            (EndPosition(gauge_z_mm=90), (20.0, 8.0, 30.0), ["G0 X20 Y8 Z30"]),
            (EndPosition(gauge_z_mm=90, xy_mm=(-10, 30)), (-10.0, 30.0, 30.0), ["G0 X20 Y8 Z30", "G0 X-10 Y30 Z30"]),
        ):
            j = job(end, coolant="mist", initial_z=30)
            nc, decoded = decoded_end(j, m, c.CarveraPost, c.CarveraDecoder)
            self.assertEqual(decoded, pose)
            self.assertEqual(
                nc[-len(blocks) - 6 :], ["G1 X20 Y8 Z12 F600", *blocks, "M400", "M9", "M5", "G53 G0 Z-3", "M30"]
            )
            rust = post_job(post_job_input(j, "top", machine=m, accept={"unchecked_machine"}))
            self.assertTrue(rust["released"])
            self.assertEqual(rust["program"].splitlines(), nc)

    def test_the_rust_haas_post_writes_the_same_program(self):
        j = job(END)
        h, m = haas(j)
        nc, _ = decoded_end(j, m, h.HaasNGCPost, h.HaasNGCDecoder)
        rust = post_job(post_job_input(j, "top", machine=m, accept={"unchecked_machine"}))
        self.assertTrue(rust["released"])
        self.assertEqual(rust["program"].splitlines(), nc)


class OtherMachinesTest(unittest.TestCase):
    """A lathe and a cutting table end in their own terms: an end position
    (a spindle gauge Z, then XY) is refused for them, never reinterpreted."""

    def test_a_lathe_setup_refuses_it_and_already_ends_at_its_turret_index_position(self):
        import test_turning

        with self.assertRaisesRegex(ValueError, "already ends at its turret index position"):
            test_turning.job(end_position=UP).job.compile()
        # Without it: the last move returns the tool to the setup's start
        # (turret index) position, radius 30 at Z 20.
        plan = test_turning.job()
        moves = [u.update.motion for u in plan.job.regenerate() if hasattr(u.update, "motion")]
        self.assertEqual(tuple(moves[-1]["curve"]["end"]), (30.0, 0.0, 20.0))
        self.assertEqual(moves[-1]["kind"], "rapid")

    def test_a_cutting_table_refuses_it(self):
        from dataclasses import replace

        import test_thermal

        sheet = test_thermal.job().setups[0]
        stated = camlib.Job("plasma", "1", (replace(sheet, end_position=EndPosition(gauge_z_mm=100, xy_mm=(0, 0))),))
        with self.assertRaisesRegex(Exception, "a cutting table has no spindle gauge line"):
            list(stated.regenerate())


if __name__ == "__main__":
    unittest.main()
