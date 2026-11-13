"""Lathe setups through the Python API: a Setup with turning stock and a
TurningSetup, run like any setup (each move checked and streamed with its
stock change), posted (LinuxCNC radius mode G8 with G18 arcs; diameter X
words) and read back."""

import unittest
from pathlib import Path
from dataclasses import dataclass, fields, replace

import camlib
from camlib.jobs import JobRun
from camlib.post import GCodeDialect, Machine, ToolSetting, post_program
from camlib.postprocessor import _typed
from camlib.simulation import OperationUpdate

SHAFT = ((0, 0, 0), (10, 0, 0), (10, -20, 0), (15, -20, 0), (15, -40, 0))
TURNING = {f.name for f in fields(camlib.TurningSetup)}


def body(insert, side):
    """These tests' tool bodies off the centre plane (#103), physically
    plausible for their parts: outside, a 3 mm positive insert (7 degrees)
    on a flat seat, the shank 10 mm below the face to 2 mm above; inside a
    bore, a 1.5 mm insert (11 degrees), the bar 1.5 mm below to 1 mm above;
    a drill has no insert body."""
    if isinstance(insert, camlib.DrillInsert):
        return dict(y_mm=(-10, 10)), {}
    inner = side == "inner"
    return dict(y_mm=(-1.5, 1) if inner else (-10, 2)), dict(
        seat=camlib.TurningSeat(0, 0),
        insert_thickness_mm=1.5 if inner else 3,
        insert_clearance_deg=11 if inner else 7,
    )


def tool(id, insert, *, nose_radius_mm, width_mm, length_mm, offset_mm=0, side="outer", hand="right", **declared):
    """A turning tool with camlib's rectangular holder (every value stated);
    ``declared`` overrides the test body (``y_mm``, ``seat``, thickness...)."""
    holder_body, insert_body = body(insert, side)
    holder = camlib.TurningHolder.rectangle(
        insert=insert,
        nose_radius_mm=nose_radius_mm,
        side=side,
        width_mm=width_mm,
        length_mm=length_mm,
        offset_mm=offset_mm,
        y_mm=declared.pop("y_mm", holder_body["y_mm"]),
    )
    return camlib.TurningTool(
        id,
        insert=insert,
        nose_radius_mm=nose_radius_mm,
        side=side,
        hand=hand,
        holder=holder,
        **{**insert_body, **declared},
    )


def cnmg(nose_radius_mm=0.8):
    """The 80 degree insert on a 95 degree holder these tests turn with."""
    insert = camlib.TurningInsert.iso(
        included_angle_deg=80, approach_angle_deg=95, edge_length_mm=12, nose_radius_mm=nose_radius_mm
    )
    return tool("cnmg", insert, nose_radius_mm=nose_radius_mm, width_mm=20, length_mm=100)


@dataclass
class Planned:
    """A lathe setup's run: its operation reports, final stock and program
    header, and the job (for posting)."""

    job: camlib.Job
    reports: list
    stock: dict
    program: object


class LatheJob:
    def __init__(self, job):
        self.job = job

    def plan(self):
        """Run the setup; collect the streamed events into its program."""
        reports, events = [], []
        run = JobRun(self.job.compile(), None)
        for update in run:
            u = update.update
            if isinstance(u, OperationUpdate):
                events.extend(_typed(e) for e in u.events)
                if u.stage == "end":
                    reports.append(u.report)
            else:
                events.append(_typed(dict(kind="motion", motion=u.motion)))
        header = self.job.compile().setup("lathe").program_header()
        program = replace(header, events=(*header.events, *events))
        return Planned(self.job, reports, run.states["lathe"].stock(), program)


def job(**changes):
    """A lathe job (one setup, "lathe"); ``changes`` replace setup or
    turning-setup values."""
    tool = cnmg()
    values = dict(
        stock=camlib.TurnStockSpec(-60, 2, 20),
        profile=SHAFT,
        chuck_z_mm=-50,
        clearance_mm=2,
        tolerance_mm=1e-3,
        tools=(camlib.TurnToolSetting(tool, 1, 1),),
        operations=(
            camlib.TurnFace("face", "cnmg", 0, feed_mm_rev=0.15, rpm=1000),
            camlib.TurnRough("rough", "cnmg", feed_mm_rev=0.25, rpm=1000),
            camlib.TurnFinish("finish", "cnmg", feed_mm_rev=0.1, rpm=1200),
        ),
        initial_position_mm=(30, 20),
    )
    values.update(changes)
    r, z = values.pop("initial_position_mm")
    turning = camlib.TurningSetup(**{k: values.pop(k) for k in list(values) if k in TURNING})
    setup = camlib.Setup("lathe", initial_position_mm=(r, 0, z), turning=turning, **values)
    return LatheJob(camlib.Job("shaft", "1", (setup,)))


def lathe(plan, dialect=None):
    tools = tuple(
        ToolSetting.for_tool(t, number=1, length_register=1, position_after_mm=plan.program.initial_position_mm)
        for t in plan.program.tools
    )
    return Machine(
        "lathe",
        tools,
        dialect=dialect or GCodeDialect().with_commands(lathe_radius_mode="G8"),
        planes=("xz",),
        tool_change_position_mode="preserve_tip",
        output_units='mm',
        decimals=3,
        max_feed_mm_min=10000,
        max_spindle_rpm=30000,
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


def text(plan, machine):
    return plan.job.post_nc("lathe", machine=machine, accept={"unchecked_machine"}).text.splitlines()


class TurningTest(unittest.TestCase):
    def test_plan_reports_and_stock(self):
        plan = job().plan()
        self.assertEqual([r["id"] for r in plan.reports], ["face", "rough", "finish"])
        self.assertEqual(plan.reports[0]["passes"], 2)
        cells = plan.stock["cells"]
        # Finished diameter: r 10 over the front step.
        mid = cells[int((-10 - -60) / 0.25)]
        self.assertAlmostEqual(mid["outer"]["upper_mm"], 10, places=6)
        self.assertGreaterEqual(mid["outer"]["lower_mm"], 10 - 1e-6)

    def test_lathe_run_streams_moves_with_stock_and_cancels(self):
        from camlib.simulation import Simulation, Update

        setup = job().job.compile().setup("lathe")
        # Streamed: each move arrives with the stock it leaves; removal only grows.
        with Simulation(**setup.simulation_options()) as sim:
            removed, moves, ops = [], 0, []
            for update in sim.run_operations():
                if isinstance(update, Update):
                    moves += 1
                    removed.append(sim.state.volume()["removed_lower_mm3"])
                elif update.stage == "begin":
                    ops.append(update.operation)
            self.assertTrue(sim.state.finished)
        self.assertEqual(ops, ["face", "rough", "finish"])
        self.assertGreater(moves, 20)
        self.assertEqual(removed, sorted(removed))
        self.assertGreater(removed[-1], 0)
        # Cancelled mid-run: the run stops; no finished stock.
        with Simulation(**setup.simulation_options()) as sim:
            updates = sim.run_operations()
            for _, update in zip(range(5), updates):
                pass
            sim.cancel()
            with self.assertRaises(Exception):
                for _ in updates:
                    pass
            self.assertFalse(sim.state.finished)

    def test_recorded_lathe_run_plays_back_actual_stock(self):
        import tempfile

        from camlib.playback import Cursor, Player, RecordedRun

        lathe_job = job().job
        live = job().plan().stock["cells"]
        with tempfile.TemporaryDirectory() as directory:
            with lathe_job.record("lathe", rapid_mm_min=6000, checkpoint_every=8, directory=directory) as run:
                run.wait()
                total = run.status()["available"]
                self.assertGreater(total, 20)
                self.assertGreater(run.status()["checkpoints"], 2)
                player = Player(run)
                player.seek(Cursor(total))
                self.assertTrue(player.state.finished)
                self.assertEqual(player.state.stock()["cells"], live)
                end = player.state.volume()
                # Back to the start: nothing removed; forward again from a
                # checkpoint: the same stock as the run.
                player.seek(Cursor())
                self.assertEqual(player.state.volume()["removed_upper_mm3"], 0)
                player.seek(Cursor(total // 2))
                middle = player.state.volume()["removed_lower_mm3"]
                self.assertTrue(0 < middle < end["removed_lower_mm3"])
                self.assertEqual(player.last_seek["checkpoint"] is not None, True)
                # Within a move: the move's first part cut from its start.
                # Find a cutting move; the partial removal lies between the
                # move's start and end stock.
                for k in range(total // 2, total):
                    player.seek(Cursor(k))
                    before = player.state.volume()["removed_lower_mm3"]
                    player.seek(Cursor(k + 1))
                    after = player.state.volume()["removed_lower_mm3"]
                    if after - before > 1:
                        break
                player.seek(Cursor(k, 0.5))
                half = player.state.volume()["removed_lower_mm3"]
                self.assertTrue(before < half < after, (before, half, after))
                self.assertIsNotNone(player.motion)
                player.seek(Cursor(k, 1.0))
                self.assertAlmostEqual(player.state.volume()["removed_lower_mm3"], after, places=6)
                # Playing advances with the clock; pausing holds.
                player.seek(Cursor())
                player.play()
                player.tick(5)
                self.assertGreater(player.cursor.completed, 0)
                player.pause()
                held = player.cursor
                player.tick(5)
                self.assertEqual(player.cursor, held)
                saved = run.save(f"{directory}/saved")
            reopened = RecordedRun.open(saved)
            try:
                player = Player(reopened)
                player.seek(Cursor(total))
                self.assertEqual(player.state.stock()["cells"], live)
            finally:
                reopened.close()

    def test_posted_lathe_program_replays_on_the_stock(self):
        import tempfile

        from camlib.playback import Cursor, Player

        plan = job().plan()
        machine = lathe(plan)
        with plan.job.post("lathe", machine=machine) as run:
            run.wait()
            player = Player(run)
            player.seek(Cursor(run.status()["available"]))
            posted = player.state.stock()
            # The posted program (3 decimals) removes the planned stock to
            # within its output rounding.
            worst = max(
                abs(a["outer"]["upper_mm"] - b["outer"]["upper_mm"])
                for a, b in zip(posted["cells"], plan.stock["cells"])
            )
            self.assertLess(worst, 2e-3)
            self.assertAlmostEqual(posted["remaining_upper_mm3"], plan.stock["remaining_upper_mm3"], delta=5)
            with tempfile.TemporaryDirectory() as directory:
                written = run.export(f"{directory}/shaft.nc", accept={"unchecked_machine"})
                text = Path(written["path"]).read_text()
            self.assertIn("G18", text)
            self.assertEqual(text, plan.job.post_nc("lathe", machine=machine, accept={"unchecked_machine"}).text)

    def test_material_check_compares_the_stock_with_the_part(self):
        plan = job().plan()
        machine = lathe(plan)
        finished = plan.job.post_nc("lathe", machine=machine, accept={"unchecked_machine"})
        self.assertEqual(finished.unchecked, ("machine",))
        report = finished.material
        self.assertEqual(report["gouge"]["assessment"], "within_tolerance")
        self.assertIn("proof", report["gouge"])
        # Finished: material certainly left only in the sharp inside corner
        # at the shoulder (z -20), which the 0.8 mm nose radius cannot reach.
        corner = [f for f in report["findings"] if f["residual"] == "definite_excess"]
        self.assertTrue(corner)
        for f in corner:
            self.assertEqual(f["side"], "outer")
            self.assertTrue(-20 <= f["z_mm"][0] and f["z_mm"][1] <= -20 + 0.8 + 0.25, f)
        # Each operation's end report carries the check after it: roughing
        # leaves its allowance (certain leftover material).
        reports = {r["id"]: r["material"] for r in plan.reports}
        self.assertGreater(reports["rough"]["residual"]["definite_cells"], 0)
        self.assertLess(reports["finish"]["residual"]["definite_cells"], reports["rough"]["residual"]["definite_cells"])
        rough_only = job(operations=(camlib.TurnRough("rough", "cnmg", feed_mm_rev=0.25, rpm=1000),)).plan()
        material = rough_only.job.post_nc("lathe", machine=lathe(rough_only), accept={"unchecked_machine"}).material
        self.assertEqual(material["residual"]["assessment"], "definite_excess")
        sides = {f["side"] for f in material["findings"]}
        self.assertIn("outer", sides)
        # The posted program's replayed stock is checked the same way.
        import tempfile

        with plan.job.post("lathe", machine=machine) as run, tempfile.TemporaryDirectory() as directory:
            written = run.export(f"{directory}/shaft.nc", accept={"unchecked_machine"})
        self.assertEqual(written["unchecked"], ("machine",))
        self.assertEqual(written["material"]["gouge"]["assessment"], "within_tolerance")
        self.assertEqual(written["material"]["residual"]["definite_cells"], len(corner))

    def test_a_negative_insert_on_its_catalogue_seat_faces_roughs_and_finishes(self):
        # The example of docs/operations.md, "Turning": a CNMG insert (no
        # clearance of its own) on a PCLNR holder, whose seat tilts it -6
        # degrees in rake and in inclination, the values a holder catalogue
        # prints. The tilt is what backs both flanks off the work. With the
        # inclination applied the wrong way round the body below the nose
        # reached toward the axis and every cut was refused as a rub
        # ("the insert's body below its cutting face would rub the stock").
        insert = camlib.TurningInsert.iso_code("CNMG120408", holder="PCLNR2525M12")
        holder = camlib.TurningHolder.rectangle(
            insert=insert,
            nose_radius_mm=insert.nose_radius_mm,
            side="outer",
            width_mm=25,
            length_mm=150,
            offset_mm=0,
            y_mm=(-25, 3),
        )

        def plan(seat):
            cutter = camlib.TurningTool("cnmg", insert=insert, side="outer", hand="right", holder=holder, seat=seat)
            return job(
                tools=(camlib.TurnToolSetting(cutter, 1, 1),),
                operations=(
                    camlib.TurnFace("face", "cnmg", 0, feed_mm_rev=0.15, rpm=1000),
                    camlib.TurnRough("rough", "cnmg", depth_of_cut_mm=2, allowance_mm=0.3, feed_mm_rev=0.25, rpm=1000),
                    camlib.TurnFinish("finish", "cnmg", feed_mm_rev=0.1, rpm=1200),
                ),
            ).plan()

        done = plan(camlib.TurningSeat(rake_deg=-6, inclination_deg=-6))
        self.assertEqual([r["id"] for r in done.reports], ["face", "rough", "finish"])
        # The same insert tilted nose-up has no clearance below the nose:
        # that is the rub, and it is still refused.
        with self.assertRaisesRegex(Exception, "body below its cutting face would rub the stock"):
            plan(camlib.TurningSeat(rake_deg=-6, inclination_deg=6))

    def test_a_posted_arc_outside_the_setups_tolerance_is_refused_by_name(self):
        # A chamfered, relieved shaft finished with a 0.4 mm nose, posted
        # for a FANUC lathe at three decimals. The machine's posting
        # tolerance (0.01 mm by default) lets an arc be written a few
        # microns from the planned one; the setup allows 0.001 mm against
        # the part. The read-back program is refused, and the refusal says
        # which two tolerances disagree (it used to say "check the insert's
        # angles against the profile", which is not what is wrong).
        import math

        from test_drill_cycles import example

        post = example("fanuc_lathe_post")
        relief_z = -16 - 1.8 / math.tan(math.radians(30))
        profile = (
            (0, 0, 0),
            (8.5, 0, 0),
            (10, -1.5, 0),
            (10, -16, 0),
            (8.2, relief_z, 0),
            (8.2, -21.5, -math.tan(math.pi / 8)),
            (9.7, -23, 0),
            (15, -23, 0),
            (15, -40, 0),
        )

        def cutter(id, code, holder_code, width, length, y):
            insert = camlib.TurningInsert.iso_code(code, holder=holder_code)
            holder = camlib.TurningHolder.rectangle(
                insert=insert,
                nose_radius_mm=insert.nose_radius_mm,
                side="outer",
                width_mm=width,
                length_mm=length,
                offset_mm=0,
                y_mm=y,
            )
            return camlib.TurningTool(
                id, insert=insert, side="outer", hand="right", holder=holder, seat=camlib.TurningSeat(0, 0)
            )

        def posted(tolerance):
            plan = job(
                profile=profile,
                tolerance_mm=tolerance,
                tools=(
                    camlib.TurnToolSetting(cutter("rougher", "CCMT120408", "SCLCR2525M12", 25, 150, (-25, 3)), 1, 1),
                    camlib.TurnToolSetting(cutter("finisher", "VCMT160404", "SVJCR2020K16", 20, 125, (-20, 3)), 2, 2),
                ),
                operations=(
                    camlib.TurnFace("face", "rougher", 0, stepdown_mm=1, feed_mm_rev=0.15, rpm=1200),
                    camlib.TurnRough(
                        "rough", "rougher", depth_of_cut_mm=2, allowance_mm=0.3, feed_mm_rev=0.25, rpm=1200
                    ),
                    camlib.TurnFinish("finish", "finisher", feed_mm_rev=0.1, rpm=1500),
                ),
                initial_position_mm=(40, 30),
            ).plan()
            tools = tuple(
                ToolSetting.for_tool(t, number=i, length_register=i, position_after_mm=plan.program.initial_position_mm)
                for i, t in enumerate(plan.program.tools, 1)
            )
            machine = post.define_machine(
                "lathe",
                tools,
                max_spindle_rpm=4000,
                program_number=1001,
                output_units="mm",
                decimals=3,
                max_feed_mm_min=10000,
                arcs=True,
                helices=False,
                max_sweep_rad=math.tau,
                max_tool_number=8,
                max_length_register=8,
                tool_changes=True,
                flood=True,
                mist=False,
                rapid_mm_min=8000,
            )
            return plan.job.post_nc(
                "lathe",
                machine=machine,
                post=post.FanucLathePost,
                decoder=post.FanucLatheDecoder,
                accept={"unchecked_machine"},
                lowering=camlib.LoweringOptions(native_arcs=True),
            )

        with self.assertRaises(Exception) as refused:
            posted(1e-3)
        message = str(refused.exception)
        self.assertIn("into the part", message)
        self.assertIn("in the program as it was read back", message)
        self.assertIn("machine's tolerance_mm", message)
        self.assertIn("setup's tolerance_mm (0.001)", message)
        self.assertNotIn("insert's angles", message)
        # With a setup tolerance the posting fits in, the same program is released.
        self.assertTrue(posted(2e-3).text)

    def test_linuxcnc_radius_mode_post(self):
        plan = job().plan()
        nc = text(plan, lathe(plan))
        self.assertIn("G8", nc)
        self.assertIn("G18", nc)
        # A lathe has no Y axis: no block carries a Y word.
        self.assertEqual([line for line in nc if any(w.startswith("Y") for w in line.split())], [])
        # The finished front diameter: X10 (radius) along Z.
        self.assertTrue(any(line.startswith("G1 X10.8 Z-") or " X10.8 " in line for line in nc), nc[:40])

    def test_diameter_x_words(self):
        plan = job().plan()
        dialect = GCodeDialect(x_diameter=True)
        nc = text(plan, lathe(plan, dialect))
        xs = [float(w[1:]) for line in nc if line.startswith(("G0", "G1")) for w in line.split() if w[0] == "X"]
        # Radius 10.8 (nose center on the r 10 diameter) is written X21.6.
        self.assertIn(21.6, xs)
        self.assertNotIn(10.8, xs)

    def test_groove_and_part_off(self):
        groove = tool("groove2", camlib.GrooveInsert(2, 22), nose_radius_mm=0.2, width_mm=2, length_mm=60)
        profile = (
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
        plan = job(
            profile=profile,
            tools=(camlib.TurnToolSetting(cnmg(), 1, 1), camlib.TurnToolSetting(groove, 2, 2)),
            operations=(
                camlib.TurnFace("face", "cnmg", 0, feed_mm_rev=0.15, rpm=1000),
                camlib.TurnRough("rough", "cnmg", feed_mm_rev=0.25, rpm=1000),
                camlib.TurnGroove("groove", "groove2", -25, -28, 12, peck_mm=1, feed_mm_rev=0.05, rpm=800),
                camlib.TurnPartOff("part", "groove2", -40, feed_mm_rev=0.05, rpm=800),
            ),
        ).plan()
        self.assertEqual([r["passes"] for r in plan.reports][2:], [2, 1])
        cells = plan.stock["cells"]
        self.assertAlmostEqual(cells[int((-26.5 - -60) / 0.25)]["outer"]["upper_mm"], 12, places=6)
        self.assertEqual(cells[int((-41 - -60) / 0.25)]["outer"]["upper_mm"], 0)

    def test_threading_posts_synchronized_moves(self):
        thread = tool("thread60", camlib.ThreadInsert(60, 3), nose_radius_mm=0.1, width_mm=16, length_mm=60)
        plan = job(
            tools=(camlib.TurnToolSetting(cnmg(), 1, 1), camlib.TurnToolSetting(thread, 2, 2)),
            operations=(
                camlib.TurnFace("face", "cnmg", 0, feed_mm_rev=0.15, rpm=1000),
                camlib.TurnRough("rough", "cnmg", feed_mm_rev=0.25, rpm=1000),
                camlib.TurnFinish("finish", "cnmg", feed_mm_rev=0.1, rpm=1200),
                camlib.TurnThread("thread", "thread60", -20, -38, 15, 2, 1.227, rpm=600),
            ),
        ).plan()
        self.assertEqual(plan.reports[-1]["passes"], 7)
        tools = tuple(
            ToolSetting.for_tool(t, number=i, length_register=i, position_after_mm=plan.program.initial_position_mm)
            for i, t in enumerate(plan.program.tools, 1)
        )
        dialect = GCodeDialect().with_commands(lathe_radius_mode="G8", thread_synchronized="G33")
        m = Machine(
            "lathe",
            tools,
            dialect=dialect,
            planes=("xz",),
            tool_change_position_mode="preserve_tip",
            output_units='mm',
            decimals=3,
            max_feed_mm_min=10000,
            max_spindle_rpm=30000,
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
        nc = text(plan, m)
        g33 = [line for line in nc if line.startswith("G33")]
        self.assertEqual(len(g33), 7)
        self.assertTrue(all(line.endswith("K2") for line in g33), g33)
        # Without a declared threading code the post refuses.
        with self.assertRaisesRegex(Exception, "thread_synchronized"):
            text(
                plan,
                Machine(
                    "lathe",
                    tools,
                    dialect=GCodeDialect().with_commands(lathe_radius_mode="G8"),
                    planes=("xz",),
                    tool_change_position_mode="preserve_tip",
                    output_units='mm',
                    decimals=3,
                    max_feed_mm_min=10000,
                    max_spindle_rpm=30000,
                    arcs=True,
                    helices=True,
                    max_sweep_rad=6.283185307179586,
                    max_tool_number=99999,
                    max_length_register=99999,
                    tool_changes=True,
                    flood=True,
                    mist=True,
                    rapid_mm_min=6000,
                ),
            )

    def test_lathe_t_words(self):
        plan = job().plan()
        dialect = GCodeDialect(x_diameter=True, tool_change="lathe_t")
        nc = text(plan, lathe(plan, dialect))
        self.assertIn("T0101", nc)
        self.assertNotIn("M6", nc)

    def test_axis_drilling(self):
        import math

        cone = 5 / math.tan(math.radians(59))
        drill = tool("drill10", camlib.DrillInsert(10, 118, 40), nose_radius_mm=0, width_mm=30, length_mm=60)
        plan = job(
            bore=((5, 0, 0), (5, -20, 0), (0, -20 - cone, 0)),
            tools=(camlib.TurnToolSetting(cnmg(), 1, 1), camlib.TurnToolSetting(drill, 2, 2)),
            operations=(
                camlib.TurnFace("face", "cnmg", 0, feed_mm_rev=0.15, rpm=1000),
                camlib.TurnDrill("drill", "drill10", -20 - cone, peck_mm=6, feed_mm_rev=0.12, rpm=900),
            ),
        ).plan()
        cell = plan.stock["cells"][int((-10 - -60) / 0.25)]
        self.assertAlmostEqual(cell["inner"]["lower_mm"], 5)
        self.assertAlmostEqual(cell["inner"]["upper_mm"], 5)

    def test_drill_bore_and_thread_inside(self):
        import math

        cone = 8 / math.tan(math.radians(59))
        drill = tool("drill16", camlib.DrillInsert(16, 118, 50), nose_radius_mm=0, width_mm=40, length_mm=60)
        bar = tool(
            "bar",
            camlib.TurningInsert.iso(
                included_angle_deg=55, approach_angle_deg=95, edge_length_mm=6, nose_radius_mm=0.4
            ),
            nose_radius_mm=0.4,
            width_mm=10,
            length_mm=80,
            offset_mm=1,
            side="inner",
        )
        thread = tool(
            "thread_in",
            camlib.ThreadInsert(60, 2),
            nose_radius_mm=0.1,
            width_mm=10,
            length_mm=80,
            # The bar's top 2 mm under the nose: off the centre plane it clears
            # the 11 mm crests at the last pass too (#103).
            offset_mm=2,
            side="inner",
        )
        plan = job(
            stock=camlib.TurnStockSpec(-60, 0, 20),
            profile=((0, 0, 0), (18, 0, 0), (18, -40, 0)),
            bore=((11, 0, 0), (11, -25, 0), (8, -25, 0), (0, -25 - cone, 0)),
            tools=(
                camlib.TurnToolSetting(drill, 1, 1),
                camlib.TurnToolSetting(bar, 2, 2),
                camlib.TurnToolSetting(thread, 3, 3),
            ),
            operations=(
                camlib.TurnDrill("drill", "drill16", -25 - cone, peck_mm=10, feed_mm_rev=0.12, rpm=900),
                camlib.TurnBoreRough("bore_rough", "bar", -24, feed_mm_rev=0.15, rpm=1000),
                camlib.TurnBoreFinish("bore_finish", "bar", -24, feed_mm_rev=0.08, rpm=1200),
                # M24 x 1.5 internal, 15 mm long.
                camlib.TurnThread("thread", "thread_in", 0, -15, 12, 1.5, 0.92, rpm=600),
            ),
        ).plan()
        cell = plan.stock["cells"][int((-20 - -60) / 0.25)]
        self.assertAlmostEqual(cell["inner"]["lower_mm"], 11)
        self.assertAlmostEqual(cell["inner"]["upper_mm"], 11)
        cell = plan.stock["cells"][int((-10 - -60) / 0.25)]
        self.assertAlmostEqual(cell["inner"]["lower_mm"], 11)  # crests
        self.assertAlmostEqual(cell["inner"]["upper_mm"], 12)  # roots
        tools = tuple(
            ToolSetting.for_tool(t, number=i, length_register=i, position_after_mm=plan.program.initial_position_mm)
            for i, t in enumerate(plan.program.tools, 1)
        )
        dialect = GCodeDialect(x_diameter=True).with_commands(lathe_diameter_mode="G7", thread_synchronized="G33")
        m = Machine(
            "lathe",
            tools,
            dialect=dialect,
            planes=("xz",),
            tool_change_position_mode="preserve_tip",
            output_units='mm',
            decimals=3,
            max_feed_mm_min=10000,
            max_spindle_rpm=30000,
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
        g33 = [line for line in text(plan, m) if line.startswith("G33")]
        self.assertEqual(len(g33), 7)
        # Full depth: the imaginary tip at the major diameter (M24), X words
        # in diameter.
        self.assertIn("X24 ", g33[-1] + " ")

    def test_constant_surface_speed_and_feed_per_rev(self):
        from dataclasses import replace

        css = dict(rpm=3000, surface_speed_m_min=200)
        thread = tool("thread60", camlib.ThreadInsert(60, 3), nose_radius_mm=0.1, width_mm=16, length_mm=60)
        plan_tools = (camlib.TurnToolSetting(cnmg(), 1, 1), camlib.TurnToolSetting(thread, 2, 2))
        plan = job(
            feed_per_rev=True,
            tools=plan_tools,
            operations=(
                camlib.TurnFace("face", "cnmg", 0, **{'feed_mm_rev': 0.15, 'rpm': 1000, **css}),
                camlib.TurnRough("rough", "cnmg", **{'feed_mm_rev': 0.25, 'rpm': 1000, **css}),
                camlib.TurnFinish("finish", "cnmg", **{'feed_mm_rev': 0.1, 'rpm': 1200, **css}),
                camlib.TurnThread("thread", "thread60", -20, -38, 15, 2, 1.227, rpm=600),
            ),
        ).plan()
        tools = tuple(
            ToolSetting.for_tool(t, number=i, length_register=i, position_after_mm=plan.program.initial_position_mm)
            for i, t in enumerate(plan.program.tools, 1)
        )

        def machine(dialect, **settings):
            return Machine(
                "lathe",
                tools,
                dialect=dialect,
                planes=("xz",),
                tool_change_position_mode="preserve_tip",
                max_spindle_rpm=6000,
                **{
                    'output_units': 'mm',
                    'decimals': 3,
                    'max_feed_mm_min': 10000,
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

        # LinuxCNC: G96 D<limit> S<speed>, G95 F; radius X.
        linuxcnc = GCodeDialect(css_limit_word="D").with_commands(
            lathe_radius_mode="G8", thread_synchronized="G33", css_on="G96", feed_per_rev="G95"
        )
        nc = text(plan, machine(linuxcnc))
        self.assertIn("G96 D3000 S200 M3", nc)
        self.assertEqual([line for line in nc if line.startswith("G95")], ["G95 F0.15", "G95 F0.25", "G95 F0.1"])
        # Threading: back to a fixed rpm before it.
        i = nc.index("G97", nc.index("G96 D3000 S200 M3"))
        self.assertEqual(nc[i + 1], "M3 S600")
        # Centroid / FANUC lathes: G50 S<limit> before G96, G99 per rev and
        # G98 per minute, T0101, diameter X.
        centroid = replace(
            GCodeDialect(x_diameter=True, tool_change="lathe_t").with_commands(
                cycle_return_initial=None,
                feed_per_minute="G98",
                css_on="G96",
                css_limit="G50",
                feed_per_rev="G99",
                thread_synchronized="G32",
            ),
            thread_lead_word="F",
        )
        nc = text(plan, machine(centroid))
        i = nc.index("G50 S3000")
        self.assertEqual(nc[i + 1], "G96 S200 M3")
        self.assertIn("G99 F0.15", nc)
        # Inch output: surface speed in ft/min, feed per rev in inches. A
        # metric feed (0.15 mm = F0.0059 in) is off by more than the feed
        # tolerance at full speed: refused; inch feeds are written exactly.
        with self.assertRaisesRegex(Exception, "F0.0059"):
            text(plan, machine(linuxcnc, output_units="inch", decimals=4))
        inch = job(
            feed_per_rev=True,
            tools=plan_tools,
            operations=(
                camlib.TurnFace("face", "cnmg", 0, feed_mm_rev=0.254, **{'rpm': 1000, **css}),
                camlib.TurnRough("rough", "cnmg", feed_mm_rev=0.254, **{'rpm': 1000, **css}),
                camlib.TurnFinish("finish", "cnmg", feed_mm_rev=0.1016, **{'rpm': 1200, **css}),
            ),
        ).plan()
        # Actual posted-stock replay refuses coarse output whose rounded
        # finishing path reaches the part beyond its declared tolerance
        # (0.001 mm). This part's corners are now followed with arcs (#14)
        # whose 4-decimal inch words happen to stay within it (the chords
        # that used to round the corners did not), so the refusal is shown
        # on a diameter that rounds 0.00115 mm into the part at 4 decimals
        # (X0.3940 for 10.00875 mm).
        text(inch, machine(linuxcnc, output_units="inch", decimals=4))
        radius = 3940 * 0.00254 + 0.00115
        coarse = job(
            feed_per_rev=True,
            tools=plan_tools,
            profile=((0, 0, 0), (radius, 0, 0), (radius, -20, 0), (15, -20, 0), (15, -40, 0)),
            operations=(
                camlib.TurnFace("face", "cnmg", 0, feed_mm_rev=0.254, **{'rpm': 1000, **css}),
                camlib.TurnRough("rough", "cnmg", feed_mm_rev=0.254, **{'rpm': 1000, **css}),
                camlib.TurnFinish("finish", "cnmg", feed_mm_rev=0.1016, **{'rpm': 1200, **css}),
            ),
        ).plan()
        with self.assertRaisesRegex(camlib.SimulationError, "insert would cut"):
            text(coarse, machine(linuxcnc, output_units="inch", decimals=4))
        text(coarse, machine(linuxcnc, output_units="inch", decimals=5))
        nc = text(inch, machine(linuxcnc, output_units="inch", decimals=5))
        self.assertIn("G96 D3000 S656.16798 M3", nc)
        self.assertIn("G95 F0.01", nc)
        self.assertIn("G95 F0.004", nc)
        # Without the codes the post refuses.
        with self.assertRaisesRegex(Exception, "constant surface speed"):
            text(plan, machine(GCodeDialect().with_commands(lathe_radius_mode="G8", thread_synchronized="G33")))
        # CSS needs feeds per revolution.
        with self.assertRaisesRegex(Exception, "feed_per_rev"):
            job(operations=(camlib.TurnFace("face", "cnmg", 0, **{'feed_mm_rev': 0.15, 'rpm': 1000, **css}),)).plan()

    def test_centroid_lathe_post(self):
        from test_drill_cycles import example

        c = example("centroid_lathe_post")
        css = dict(rpm=3000, surface_speed_m_min=200)
        thread = tool("thread60", camlib.ThreadInsert(60, 3), nose_radius_mm=0.1, width_mm=16, length_mm=60)
        plan = job(
            feed_per_rev=True,
            tools=(camlib.TurnToolSetting(cnmg(), 1, 1), camlib.TurnToolSetting(thread, 2, 2)),
            operations=(
                camlib.TurnFace("face", "cnmg", 0, **{'feed_mm_rev': 0.15, 'rpm': 1000, **css}),
                camlib.TurnRough("rough", "cnmg", **{'feed_mm_rev': 0.25, 'rpm': 1000, **css}),
                camlib.TurnFinish("finish", "cnmg", **{'feed_mm_rev': 0.1, 'rpm': 1200, **css}),
                camlib.TurnThread("thread", "thread60", -20, -38, 15, 2, 1.227, rpm=600),
            ),
        ).plan()
        tools = tuple(
            ToolSetting.for_tool(t, number=i, length_register=i, position_after_mm=plan.program.initial_position_mm)
            for i, t in enumerate(plan.program.tools, 1)
        )
        m = c.define_machine(
            "lathe",
            tools,
            max_spindle_rpm=4000,
            output_units='mm',
            decimals=3,
            max_feed_mm_min=10000,
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
        nc = "".join(
            x.text
            for x in post_program(plan.program, machine=m, post=c.CentroidLathePost, decoder=c.CentroidLatheDecoder)
        ).splitlines()
        self.assertEqual(nc[:10], ["G21", "G18", "G98", "G97", "G40", "G80", "G52 X0 Z0", "G61", "M5", "M9"])
        for code in ("G90", "G91.1", "G43", "G49", "M6"):
            self.assertNotIn(code, " ".join(nc).split(" "))
        self.assertEqual(nc.count("G18"), 1)
        self.assertFalse(any(w.startswith("Y") for line in nc for w in line.split(" ")))
        i = nc.index("T0101")
        self.assertEqual(nc[i + 1 : i + 5], ["G50 S3000", "G96 S200 M3", "G99 F0.15", "M8"])
        # Coolant and spindle off before the next tool, on again after it.
        i = nc.index("T0202")
        self.assertIn("M9", nc[i - 4 : i])
        self.assertEqual(nc[i + 1 : i + 4], ["G97", "M3 S600", "M8"])
        g32 = [line for line in nc if line.startswith("G32")]
        self.assertEqual(len(g32), 7)
        self.assertTrue(all(line.endswith(" F2") for line in g32), g32)
        self.assertEqual(nc[-3:], ["M9", "M5", "M30"])
        # Tool-nose compensation in the control (wear): G42 T01 (the active
        # offset, manual 9.15) on its own line before the entry, G40 after.
        wear = job(
            operations=(
                camlib.TurnFace("face", "cnmg", 0, feed_mm_rev=0.15, rpm=1000),
                camlib.TurnRough("rough", "cnmg", feed_mm_rev=0.25, rpm=1000),
                camlib.TurnFinish("finish", "cnmg", compensation="wear", feed_mm_rev=0.1, rpm=1200),
            ),
        ).plan()
        nc_wear = "".join(
            x.text
            for x in post_program(wear.program, machine=m, post=c.CentroidLathePost, decoder=c.CentroidLatheDecoder)
        ).splitlines()
        self.assertEqual(nc_wear.count("G42 T01"), 1)
        self.assertEqual(nc_wear.count("G40"), 2)  # the header's and the cancel
        # A diagonal rapid is refused (G0 runs each axis at its own rate).
        from camlib.motion import Motion, Line

        events = list(plan.program.events)
        k = next(n for n, e in enumerate(events) if isinstance(e, Motion) and e.kind == "rapid")
        e = events[k]
        events[k] = Motion(
            e.id,
            e.operation,
            e.tool,
            Line(e.curve.start, (e.curve.start[0] + 1, 0, e.curve.start[2] + 1)),
            None,
            "rapid",
        )
        events.insert(
            k + 1,
            Motion(
                e.id + 1000000,
                e.operation,
                e.tool,
                Line((e.curve.start[0] + 1, 0, e.curve.start[2] + 1), e.curve.start),
                None,
                "rapid",
            ),
        )
        with self.assertRaisesRegex(Exception, "one axis|own rate|X or Z only"):
            "".join(
                x.text
                for x in post_program(
                    replace(plan.program, events=tuple(events[: k + 1])),
                    machine=m,
                    post=c.CentroidLathePost,
                    decoder=c.CentroidLatheDecoder,
                )
            )

    def test_fanuc_lathe_post(self):
        from test_drill_cycles import example

        f = example("fanuc_lathe_post")
        css = dict(rpm=3000, surface_speed_m_min=200)
        thread = tool("thread60", camlib.ThreadInsert(60, 3), nose_radius_mm=0.1, width_mm=16, length_mm=60)
        plan = job(
            feed_per_rev=True,
            tools=(camlib.TurnToolSetting(cnmg(), 1, 1), camlib.TurnToolSetting(thread, 2, 2)),
            operations=(
                camlib.TurnFace("face", "cnmg", 0, **{'feed_mm_rev': 0.15, 'rpm': 1000, **css}),
                camlib.TurnRough("rough", "cnmg", **{'feed_mm_rev': 0.25, 'rpm': 1000, **css}),
                camlib.TurnFinish("finish", "cnmg", **{'feed_mm_rev': 0.1, 'rpm': 1200, **css}),
                camlib.TurnThread("thread", "thread60", -20, -38, 15, 2, 1.227, rpm=600),
            ),
        ).plan()
        tools = tuple(
            ToolSetting.for_tool(t, number=i, length_register=i, position_after_mm=plan.program.initial_position_mm)
            for i, t in enumerate(plan.program.tools, 1)
        )
        m = f.define_machine(
            "lathe",
            tools,
            max_spindle_rpm=4000,
            program_number=1234,
            output_units='mm',
            decimals=3,
            max_feed_mm_min=10000,
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
        nc = "".join(
            x.text for x in post_program(plan.program, machine=m, post=f.FanucLathePost, decoder=f.FanucLatheDecoder)
        ).splitlines()
        self.assertEqual(nc[:10], ["%", "O1234", "G21", "G18", "G98", "G97", "G40", "G80", "M5", "M9"])
        for code in ("G90", "G91", "G43", "G49", "M6", "G52"):
            self.assertNotIn(code, " ".join(nc).split(" "))
        i = nc.index("T0101")
        self.assertEqual(nc[i + 1 : i + 5], ["G50 S3000", "G96 S200 M3", "G99 F0.15", "M8"])
        # Every dimensional word has its decimal point.
        for line in nc:
            for w in line.split(" "):
                if w[:1] in "XZIKF":
                    self.assertIn(".", w, line)
        g32 = [line for line in nc if line.startswith("G32")]
        self.assertEqual(len(g32), 7)
        self.assertTrue(all(line.endswith(" F2.") for line in g32), g32)
        self.assertEqual(nc[-4:], ["M9", "M5", "M30", "%"])
        # Tool-nose compensation in the control (wear): G42 in the entry
        # block, G40 in the retract's, no register word.
        wear = replace(
            plan.program,
            events=job(
                feed_per_rev=True,
                operations=(
                    camlib.TurnFace("face", "cnmg", 0, feed_mm_rev=0.15, rpm=1000),
                    camlib.TurnRough("rough", "cnmg", feed_mm_rev=0.25, rpm=1000),
                    camlib.TurnFinish("finish", "cnmg", compensation="wear", feed_mm_rev=0.1, rpm=1200),
                ),
            )
            .plan()
            .program.events,
        )
        nc = "".join(
            x.text for x in post_program(wear, machine=m, post=f.FanucLathePost, decoder=f.FanucLatheDecoder)
        ).splitlines()
        on = [line for line in nc if " G42 " in line]
        off = [line for line in nc if " G40 " in line]
        self.assertEqual(len(on), 1)
        self.assertTrue(on[0].startswith("G1 G42 X") and " D" not in on[0], on)
        self.assertEqual(len(off), 1)
        self.assertTrue(off[0].startswith("G1 G40 X"), off)

    def test_refusals(self):
        with self.assertRaisesRegex(Exception, "into the part"):
            job(
                profile=((0, 0, 0), (15, 0, 0), (15, -10, 0), (10, -13, 0), (10, -40, 0)),
                operations=(camlib.TurnFinish("finish", "cnmg", feed_mm_rev=0.1, rpm=1200),),
            ).plan()
        # A face deeper than twice the nose radius is cut by the insert's edges.
        job(
            tools=(camlib.TurnToolSetting(cnmg(0.4), 1, 1),),
            operations=(camlib.TurnFace("face", "cnmg", 0, stepdown_mm=2, feed_mm_rev=0.15, rpm=1000),),
        ).plan()
        with self.assertRaisesRegex(Exception, "unknown tool"):
            job(operations=(camlib.TurnFace("face", "nope", 0, feed_mm_rev=0.15, rpm=1000),)).plan()


if __name__ == "__main__":
    unittest.main()
