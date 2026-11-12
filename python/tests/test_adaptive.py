from camlib.postprocessor import GCodeDialect
from dataclasses import replace
from functools import lru_cache
import importlib.util
from pathlib import Path
import unittest
from camlib import Engine
from camlib.playback import Cursor, Player, RecordedRun
from camlib.simulation import OperationUpdate, Simulation, SimulationError


@lru_cache(None)
def example():
    path = Path(__file__).resolve().parents[2] / "python/tests/support/adaptive_pocket.py"
    spec = importlib.util.spec_from_file_location("adaptive_example", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@lru_cache(None)
def config():
    # These tests are of the orbit planner; the default is engagement_front (#190).
    return example().adaptive_job(path_strategy="disk_front")


class AdaptiveTest(unittest.TestCase):
    def test_actual_cad_corner_streams_material_and_complete_typed_program(self):
        c = config()
        shape = example().target().val()
        self.assertTrue(shape.isValid())
        self.assertAlmostEqual(shape.Volume(), 6248.0)
        events = []
        report = None
        levels = set()
        first = False
        with Simulation(**c) as sim:
            for update in sim.run_operations():
                if isinstance(update, OperationUpdate):
                    events.extend(update.events)
                    if update.stage == "end":
                        report = update.report
                else:
                    events.append(dict(kind="motion", motion=update.motion))
                    if update.motion["kind"] == "cut":
                        levels.add(update.motion["curve"]["start"][2])
                    if not first and sim.state.volume()["removed_lower_mm3"] > 0.001:
                        first = True
                        self.assertFalse(sim.state.finished)
                        self.assertIsNone(sim._process.poll())
            self.assertTrue(first and sim.state.finished)
            self.assertEqual(levels, {9.0, 8.0})
            self.assertEqual(report["navigation_components"], 1)
            self.assertEqual(report["seed_orbits"], 2)
            self.assertLessEqual(report["max_bounded_engagement_deg"], 90.0)
            self.assertGreater(report["turn_arcs"], 0)
            self.assertGreater(report["geometric_residual_area_mm2"], 0)
            self.assertGreater(sim.state.volume()["removed_lower_mm3"], 400.0)
            self.assertLessEqual(sim.state.volume()["removed_lower_mm3"], 6760.0 - shape.Volume())
            count = sim.state.completed
        program = dict(
            version=1,
            id="adaptive-program",
            revision="1",
            units="mm",
            tolerance_mm=1e-5,
            frame=dict(origin_mm=(0, 0, 0), x=(1, 0, 0), y=(0, 1, 0), z=(0, 0, 1)),
            initial_position_mm=(0, 0, 12),
            tools=[t.to_dict() for t in c["tools"]],
            events=events,
        )
        with Engine() as engine:
            result = engine._request("validate_program", program=program, capabilities=None)
            self.assertEqual(result["summary"]["motions"], count)

    def test_retained_partial_corner_cut_replay_and_invalid_before_motion(self):
        c = dict(config())
        op = c["operations"][0]
        c["operations"] = (
            replace(
                op,
                floor_z_mm=9,
            ),
        )
        with RecordedRun(**{'rapid_mm_min': 6000, **(c)}, checkpoint_every=32) as run:
            player = Player(run)
            player.pause()
            run.wait(timeout=60)
            count = run.status()["available"]
            self.assertGreater(count, 100)
            partial = None
            for i in range(min(count, 40)):
                player.seek(Cursor(i))
                before = player.state.heights.tobytes()
                player.seek(Cursor(i, 0.5))
                if player.motion["kind"] == "cut" and player.state.heights.tobytes() != before:
                    partial = Cursor(i, 0.5)
                    break
            self.assertIsNotNone(partial)
            saved = player.state.heights.tobytes()
            player.seek(Cursor(count))
            self.assertTrue(player.state.finished)
            self.assertGreater(player.state.volume()["removed_lower_mm3"], 180)
            player.seek(partial)
            self.assertEqual(player.state.heights.tobytes(), saved)
            self.assertLessEqual(run.status()["largest_frame_bytes"], 8 * 1024 * 1024)
        for bad in [
            replace(op, orbit_radius_mm=0),
            replace(op, max_engagement_deg=0),
            replace(op, rest_tolerance_mm=0.01),
        ]:
            c["operations"] = (bad,)
            with Simulation(**c) as sim:
                with self.assertRaises(SimulationError):
                    list(sim.run_operations())
                self.assertEqual(sim.state.completed, 0)


class StockFrontDevelopmentTest(unittest.TestCase):
    def test_connected_front_links_cache_checked_post_and_fractional_replay(self):
        import json
        import os
        from tempfile import TemporaryDirectory
        from camlib.jobs import Job, Setup, RegenerationCache
        from camlib.operations import AdaptivePocket
        from camlib.post import Machine, ToolSetting
        from camlib.simulation import StockSpec, SweepOptions
        from camlib.tools import ToolAssembly
        from camlib.verification import StreamChecks, StockContactOptions

        manifest = json.loads(
            (Path(__file__).resolve().parents[2] / "fixtures/operations/adaptive-connected.json").read_text()
        )
        tool = ToolAssembly.from_dict(manifest["tools"][0])
        definition = dict(manifest["operations"][0])
        definition.pop("kind")
        definition.update(tool=tool, path_strategy="stock_front")
        operation = AdaptivePocket(
            **{
                'feed_mm_min': 600,
                'plunge_mm_min': 150,
                'spindle_rpm': 12000,
                'tool_number': 1,
                'length_register': 1,
                'coolant': "flood",
                **definition,
            }
        )
        stock = StockSpec(**{k: v for k, v in manifest["stock"].items() if k not in ("version", "units")})
        setup = Setup(
            "top",
            stock,
            (tool,),
            (operation,),
            tuple(manifest["initial_position_mm"]),
            sweep=SweepOptions(**manifest["sweep"]),
            verification=StreamChecks(stock_contact=StockContactOptions()),
        )
        job = Job("connected-front", "cold", (setup,))
        executable = os.environ.get("CAMLIB_TEST_ENGINE")
        with TemporaryDirectory() as directory:
            cache = RegenerationCache(directory)
            saved = None
            for revision in ("cold", "warm"):
                report = None
                with replace(job, revision=revision).regenerate(cache, executable=executable) as run:
                    for event in run:
                        update = event.update
                        if isinstance(update, OperationUpdate) and update.stage == "end":
                            report = update.report
                    self.assertTrue(run.finished)
                    state = run.states["top"]
                    self.assertTrue(report["front"]["coverage_complete"])
                    self.assertGreater(report["front"]["cleared_reentry_links"], 0)
                    self.assertLess(state.completed, 3734)
                    self.assertEqual(report["regeneration"]["cache_hit"], revision == "warm")
                    if saved is None:
                        saved = state.heights.tobytes()
                    else:
                        self.assertEqual(state.heights.tobytes(), saved)
                        self.assertEqual(report["regeneration"]["planned_motions"], 0)
                        self.assertEqual(report["regeneration"]["computed_sweeps"], 0)
            machine = Machine(
                "Example mill",
                (ToolSetting.for_tool(tool, number=1, length_register=1, position_after_mm=setup.initial_position_mm),),
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
            with job.post("top", machine=machine, cache=cache, executable=executable, checkpoint_every=32) as run:
                run.wait(timeout=300)
                self.assertTrue(run.posting_status()["finished"])
                player = Player(run)
                last = Cursor(run.status()["available"])
                player.seek(last)
                self.assertTrue(player.state.finished)
                self.assertEqual(player.verification["progress"]["violation"], 0)
                self.assertEqual(player.verification["progress"]["uncertain"], 0)
                for cursor in (Cursor(last.completed // 2, 0.5), Cursor(last.completed - 5, 0.25)):
                    player.seek(cursor)
                    partial = player.state.heights.tobytes()
                    player.seek(last)
                    player.seek(cursor)
                    self.assertEqual(player.state.heights.tobytes(), partial)
                self.assertTrue(run.posted_motion(last.completed - 1)["stock_committed"])

    def test_cached_larger_tool_preserves_efficient_rest_across_revisions(self):
        import json
        from tempfile import TemporaryDirectory
        from camlib.jobs import RegenerationCache

        path = Path(__file__).resolve().parents[2] / "python/tests/support/stock_front_rest.py"
        spec = importlib.util.spec_from_file_location("cached_stock_front_rest_example", path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        cold = None
        with TemporaryDirectory() as directory:
            for index, revision in enumerate(("rest-cold", "rest-warm", "rest-feed")):
                c = module.rest_job()
                c.update(revision=revision, regeneration=RegenerationCache(directory).options({}))
                if index == 2:
                    c["operations"] = (c["operations"][0], replace(c["operations"][1], feed_mm_min=700))
                ends, volumes, counts = {}, {}, {}
                with Simulation(**c) as sim:
                    for update in sim.run_operations():
                        if isinstance(update, OperationUpdate) and update.stage == "end":
                            ends[update.operation] = update.report
                            volumes[update.operation] = sim.state.volume()["removed_lower_mm3"]
                        elif not isinstance(update, OperationUpdate):
                            name = update.motion["operation"]
                            counts[name] = counts.get(name, 0) + 1
                    self.assertTrue(sim.state.finished)
                    summary = {
                        name: dict(motions=counts[name], regeneration=report["regeneration"])
                        for name, report in ends.items()
                    }
                    print(json.dumps(dict(revision=revision, operations=summary)), flush=True)
                    self.assertEqual(set(ends), {"rough", "rest"})
                    self.assertTrue(ends["rest"]["front"]["coverage_complete"])
                    self.assertLessEqual(ends["rest"]["front"]["max_bounded_engagement_deg"], 90.0)
                    self.assertGreater(volumes["rest"], volumes["rough"] + 0.1)
                    if index == 1:
                        for name, report in ends.items():
                            self.assertTrue(report["regeneration"]["cache_hit"], name)
                            self.assertEqual(report["regeneration"]["planned_motions"], 0)
                            self.assertEqual(report["regeneration"]["computed_sweeps"], 0)
                            self.assertGreater(report["regeneration"]["validated_cached_columns"], 0)
                            self.assertEqual(report["regeneration"]["cached_reach_cell_visits"], 0)
                        self.assertEqual(sim.state.heights.tobytes(), cold)
                    elif index == 2:
                        self.assertTrue(ends["rough"]["regeneration"]["cache_hit"])
                        self.assertFalse(ends["rest"]["regeneration"]["cache_hit"])
                        self.assertGreater(ends["rest"]["regeneration"]["planned_motions"], 0)
                        self.assertEqual(sim.state.heights.tobytes(), cold)
                    else:
                        self.assertTrue(all(not r["regeneration"]["cache_hit"] for r in ends.values()))
                        cold = sim.state.heights.tobytes()
                    self.assertLess(ends["rest"]["motions"], 1000)

    def test_cad_larger_then_smaller_tool_reuses_native_columns_through_python_stream(self):
        path = Path(__file__).resolve().parents[2] / "python/tests/support/stock_front_rest.py"
        spec = importlib.util.spec_from_file_location("stock_front_rest_example", path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        shape = module.target().val()
        self.assertTrue(shape.isValid())
        self.assertAlmostEqual(shape.Volume(), 6360.0)
        ends, volumes, tools = {}, {}, set()
        with Simulation(**module.rest_job()) as sim:
            for update in sim.run_operations():
                if isinstance(update, OperationUpdate):
                    if update.stage == "end":
                        ends[update.operation] = update.report
                        volumes[update.operation] = sim.state.volume()["removed_lower_mm3"]
                else:
                    tools.add(update.motion["tool"])
            self.assertTrue(sim.state.finished)
            self.assertEqual(set(ends), {"rough", "rest"})
            self.assertEqual(tools, {"rough8", "rest4"})
            self.assertTrue(ends["rest"]["front"]["coverage_complete"])
            self.assertLessEqual(ends["rest"]["front"]["max_bounded_engagement_deg"], 90.0)
            # The saved fresh D4 case needs2679moves. Bound this end-to-end
            # mostly-cleared case independently of exact trajectory ordering.
            self.assertLess(ends["rest"]["motions"], 1000)
            self.assertGreater(volumes["rest"], volumes["rough"] + 0.1)

    def test_python_selects_native_front_and_streams_committed_removal_before_cancel(self):
        from camlib.operations import HelicalEntry

        for entry, limit in [(None, 16), (HelicalEntry(angle_deg=3), 5)]:
            with self.subTest(entry=entry):
                c = dict(config())
                op = c["operations"][0]
                self.assertEqual(op.to_dict()["path_strategy"], "disk_front")
                op = replace(op, floor_z_mm=9, path_strategy="stock_front", entry=entry)
                self.assertEqual(op.to_dict()["path_strategy"], "stock_front")
                c["operations"] = (op,)
                cuts, startup_removed, helix = 0, 0.0, False
                with Simulation(**c) as sim:
                    for update in sim.run_operations():
                        if isinstance(update, OperationUpdate):
                            self.assertEqual(update.stage, "begin")
                            continue
                        if update.motion["curve"]["kind"] == "helix":
                            helix = True
                        if update.motion["kind"] == "cut":
                            cuts += 1
                            if cuts == 1:
                                startup_removed = sim.state.volume()["removed_lower_mm3"]
                            if cuts >= limit:
                                self.assertGreater(sim.state.volume()["removed_lower_mm3"], startup_removed + 1)
                                self.assertFalse(sim.state.finished)
                                sim.cancel()
                                break
                    self.assertGreaterEqual(cuts, limit)
                    self.assertEqual(helix, entry is not None)
                    self.assertTrue(sim.cancelled)

    def test_complete_front_stream_cache_revision_and_retained_stock_playback(self):
        from tempfile import TemporaryDirectory
        from camlib.jobs import RegenerationCache
        from camlib.operations import HelicalEntry
        from camlib.simulation import StockState, ProtocolError

        c = dict(config())
        c["operations"] = (
            replace(c["operations"][0], floor_z_mm=9, path_strategy="stock_front", entry=HelicalEntry(angle_deg=3)),
        )
        with TemporaryDirectory() as directory:
            c["regeneration"] = RegenerationCache(directory).options({})
            c["revision"] = "front-complete-a"
            with RecordedRun(**{'rapid_mm_min': 6000, **c}, checkpoint_every=32) as run:
                player = Player(run)
                run.wait_available(16, timeout=60)
                self.assertFalse(run.status()["finished"])
                player.pause()
                partial = None
                for i in range(16):
                    player.seek(Cursor(i))
                    before = player.state.heights.tobytes()
                    player.seek(Cursor(i, 0.5))
                    if player.state.heights.tobytes() != before:
                        partial = Cursor(i, 0.5)
                        break
                self.assertIsNotNone(partial)
                saved = player.state.heights.tobytes()
                self.assertGreater(player.state.volume()["removed_lower_mm3"], 0)
                run.wait(timeout=300)
                count = run.status()["available"]
                self.assertGreater(count, 100)
                self.assertLess(count, 10000)
                player.tick(100)
                self.assertEqual(player.cursor, partial)
                self.assertEqual(player.state.heights.tobytes(), saved)
                player.seek(Cursor(count))
                self.assertTrue(player.state.finished)
                self.assertGreater(player.state.volume()["removed_lower_mm3"], 180)
                complete = player.state.heights.tobytes()
                player.seek(partial)
                self.assertEqual(player.state.heights.tobytes(), saved)
                self.assertEqual(player.last_seek["replayed"], partial.completed)
                player.set_speed(2)
                player.play()
                seconds = player.seconds
                player.tick(0.01)
                self.assertAlmostEqual(player.seconds, seconds + 0.02)
                player.pause()
                self.assertLessEqual(run.status()["largest_frame_bytes"], 8 * 1024 * 1024)
                self.assertLess(run.status()["retained_bytes"], run.retained_bytes_limit)
                old_frame = run.read_chunk(0)
                c["revision"] = "front-complete-b"
                report, begun = None, False
                with Simulation(**c) as sim:
                    for update in sim.run_operations():
                        if isinstance(update, OperationUpdate):
                            self.assertTrue(update.report["regeneration"]["cache_hit"])
                            if update.stage == "begin":
                                begun = True
                            else:
                                report = update.report
                    self.assertTrue(begun and sim.state.finished)
                    self.assertEqual(sim.state.completed, count)
                    self.assertEqual(sim.state.heights.tobytes(), complete)
                    self.assertEqual(sim.state.manifest["run"]["revision"], c["revision"])
                    self.assertTrue(report["front"]["coverage_complete"])
                    self.assertGreater(report["front"]["reentries"], 0)
                    self.assertEqual(report["regeneration"]["planned_motions"], 0)
                    self.assertEqual(report["regeneration"]["computed_sweeps"], 0)
                    fresh = StockState(sim.manifest_frame)
                    with self.assertRaises(ProtocolError):
                        fresh.accept(old_frame)
                    self.assertEqual(fresh.completed, 0)
            self.assertFalse(list(Path(directory).rglob("*.tmp")))

    def test_the_default_follows_the_stock_and_proves_its_engagement(self):
        """#190: AdaptivePocket's default plans from the stock (engagement_front): far less motion than the orbit
        planner on the same pocket, one helix per depth level, every cutting stroke proven within
        acos(1 - stepover / r)."""
        import math
        from camlib.simulation import OperationUpdate, Simulation

        def run(c):
            motions, report = 0, None
            with Simulation(**c) as sim:
                for update in sim.run_operations():
                    if isinstance(update, OperationUpdate):
                        if update.stage == "end":
                            report = update.report
                    else:
                        motions += 1
                self.assertTrue(sim.state.finished)
                return motions, report, sim.state.volume()["removed_lower_mm3"]

        default = example().adaptive_job()
        self.assertEqual(default["operations"][0].to_dict()["path_strategy"], "engagement_front")
        moves, report, removed = run(default)
        orbit_moves, _, orbit_removed = run(example().adaptive_job(path_strategy="disk_front"))
        self.assertLess(moves * 3, orbit_moves)
        # The same pocket: both remove its volume (224 x 2 mm3 less what a 4 mm cutter leaves in the corners).
        self.assertGreater(removed, 0.93 * 448)
        self.assertAlmostEqual(removed, orbit_removed, delta=0.05 * 448)
        self.assertEqual(
            (report["strategy"], report["clearing"], report["completed_layers"]),
            ("adaptive_pocket", "engagement_front", 2),
        )
        limit = math.degrees(math.acos(1 - 1.5 / 2))
        self.assertAlmostEqual(report["max_engagement_deg"], limit)
        for layer in report["layers"]:
            front = layer["clearing"]
            self.assertEqual(front["helical_entries"], 1)
            self.assertLessEqual(front["proven_max_engagement_deg"], limit)
        # Orbit options belong to the orbit planner.
        from dataclasses import replace

        bad = dict(default, operations=(replace(default["operations"][0], orbit_radius_mm=1),))
        with self.assertRaisesRegex(Exception, "does not use orbits"):
            run(bad)

    def test_link_and_entry_options_speed_up_links_that_are_clear_of_stock(self):
        """#211: AdaptivePocket takes the link and entry options of Roughing's adaptive clearing. A link at the
        link feed changes no stock cell and has no stock under the cutter (the stream's own heights and its
        stock-contact check, not the planner's report); the path is unchanged; the options are absent from the
        job unless set; the orbit planner refuses them. (Chip thinning on the pocket: tests/adaptive.rs.)"""
        import math
        from dataclasses import replace

        from camlib.simulation import Simulation, Update, VerificationUpdate, curve_length
        from camlib.verification import StockContactOptions, StreamChecks

        base = example().adaptive_job()
        op = base["operations"][0]
        radius, cutting = 2.0, op.feed_mm_min
        for key in ("link_feed_mm_min", "link_chip_thinning", "rapid_descent_clearance_mm", "ramp_clearance_mm"):
            self.assertNotIn(key, op.to_dict())
        full = replace(op, link_feed_mm_min=3000, link_chip_thinning=True, rapid_descent_clearance_mm=0.25)
        self.assertEqual(
            {k: full.to_dict()[k] for k in ("link_feed_mm_min", "link_chip_thinning", "rapid_descent_clearance_mm")},
            dict(link_feed_mm_min=3000, link_chip_thinning=True, rapid_descent_clearance_mm=0.25),
        )

        def run(**options):
            config = dict(base, operations=(replace(op, **options),))
            moves, findings, checked, under = [], [], [], 0
            with Simulation(**config, verification=StreamChecks(stock_contact=StockContactOptions())) as sim:
                state = sim.state
                for u in sim.run_operations():
                    if isinstance(u, VerificationUpdate) and u.status not in (None, "clear"):
                        findings.append((u.completed, u.status))
                    if not isinstance(u, Update):
                        continue
                    m = u.motion
                    moves.append(m)
                    if m["kind"] != "link" or m["feed_mm_min"] != options.get("link_feed_mm_min"):
                        continue
                    # A link at the link feed: no stock cell changed, and every cell wholly under the cutter
                    # (its centre more than the cell's half diagonal inside the rim) is already down at the
                    # link's depth.
                    self.assertEqual(u.changed_count, 0, m)
                    (x0, y0, z), (x1, y1, _) = m["curve"]["start"], m["curve"]["end"]
                    length2 = (x1 - x0) ** 2 + (y1 - y0) ** 2
                    heights, inside = state.heights, radius - math.hypot(state.dx, state.dy) / 2
                    columns = range(
                        max(0, int((min(x0, x1) - radius - state.min_mm[0]) / state.dx)),
                        min(state.nx, int((max(x0, x1) + radius - state.min_mm[0]) / state.dx) + 1),
                    )
                    rows = range(
                        max(0, int((min(y0, y1) - radius - state.min_mm[1]) / state.dy)),
                        min(state.ny, int((max(y0, y1) + radius - state.min_mm[1]) / state.dy) + 1),
                    )
                    for iy in rows:
                        cy = state.min_mm[1] + (iy + 0.5) * state.dy
                        for ix in columns:
                            cx = state.min_mm[0] + (ix + 0.5) * state.dx
                            t = max(0.0, min(1.0, ((cx - x0) * (x1 - x0) + (cy - y0) * (y1 - y0)) / length2))
                            if math.hypot(cx - x0 - t * (x1 - x0), cy - y0 - t * (y1 - y0)) <= inside:
                                self.assertLessEqual(heights[2 * (iy * state.nx + ix) + 1], z + 1e-3, m)
                                under += 1
                    checked.append(m["id"])
                self.assertTrue(sim.state.finished)
                removed = sim.state.volume()["removed_lower_mm3"]
            self.assertEqual(findings, [])
            seconds = sum(curve_length(m["curve"]) / m["feed_mm_min"] * 60 for m in moves if m["feed_mm_min"])
            self.assertTrue(under > 100 or not checked)
            return moves, seconds, removed, len(checked)

        def joined(moves):
            """The path with a link's collinear stretches as one move (a link is emitted one stretch per feed)."""
            out = []
            for m in moves:
                c = m["curve"]
                if out and m["kind"] == "link" and out[-1][0] == "link" and out[-1][1]["end"] == c["start"]:
                    a, b, e = out[-1][1]["start"], c["start"], c["end"]
                    cross = (b[0] - a[0]) * (e[1] - a[1]) - (b[1] - a[1]) * (e[0] - a[0])
                    if abs(cross) < 1e-9 and (b[0] - a[0]) * (e[0] - b[0]) + (b[1] - a[1]) * (e[1] - b[1]) > 0:
                        out[-1] = ("link", dict(out[-1][1], end=e))
                        continue
                out.append((m["kind"], c))
            return out

        plain, plain_seconds, plain_removed, _ = run()
        fast, fast_seconds, fast_removed, checked = run(link_feed_mm_min=3000)
        self.assertGreater(checked, 5)
        link_feeds = {m["feed_mm_min"] for m in fast if m["kind"] == "link"}
        # Links not proven clear keep the cutting feed; nothing else takes the link feed.
        self.assertEqual(link_feeds, {cutting, 3000})
        self.assertTrue(all(m["feed_mm_min"] != 3000 for m in fast if m["kind"] != "link"))
        self.assertTrue(all(m["feed_mm_min"] != 3000 for m in plain))
        self.assertEqual(joined(plain), joined(fast))
        self.assertEqual(plain_removed, fast_removed)
        self.assertLess(fast_seconds, plain_seconds - 1)
        # Rapid descent: the second level is entered by a rapid into the first (the stock-contact check above
        # passed on every rapid). The cutting passes are the same; an entry whose descent is moved back from the
        # material adds a short link at depth.
        quick, _, quick_removed, _ = run(rapid_descent_clearance_mm=0.25, ramp_clearance_mm=0.5)
        lowest = lambda moves: min(m["curve"]["end"][2] for m in moves if m["kind"] == "rapid")  # noqa: E731
        self.assertGreaterEqual(lowest(plain), 10)
        self.assertLess(lowest(quick), 9.5)
        curves = lambda moves, kind: [m["curve"] for m in moves if m["kind"] == kind]  # noqa: E731
        self.assertEqual(curves(quick, "cut"), curves(plain, "cut"))
        added = [c for c in curves(quick, "link") if c not in curves(plain, "link")]
        self.assertTrue(all(c in curves(quick, "link") for c in curves(plain, "link")))
        self.assertTrue(all(math.dist(c["start"], c["end"]) <= radius for c in added), added)
        plunged = lambda moves: sum(c["start"][2] - c["end"][2] for c in curves(moves, "plunge"))  # noqa: E731
        self.assertLess(plunged(quick), plunged(plain) / 2)
        self.assertEqual(quick_removed, plain_removed)
        # Validation and refusals.
        with self.assertRaisesRegex(Exception, "link_chip_thinning needs link_feed_mm_min"):
            run(link_chip_thinning=True)
        with self.assertRaisesRegex(Exception, "link_feed_mm_min 0 must be positive"):
            run(link_feed_mm_min=0)
        for key, value in (
            ("link_feed_mm_min", 3000),
            ("link_chip_thinning", True),
            ("rapid_descent_clearance_mm", 0.5),
            ("ramp_clearance_mm", 0.5),
        ):
            with self.assertRaisesRegex(Exception, f"{key} applies to engagement_front clearing only"):
                run(path_strategy="disk_front", **{key: value})

    def test_roughing_exposes_same_strategy_and_unknown_strategy_is_rejected(self):
        from camlib.operations import AdaptiveClearing

        self.assertEqual(AdaptiveClearing(0.25).to_dict()["path_strategy"], "engagement_front")
        self.assertEqual(AdaptiveClearing(0.25, path_strategy="stock_front").to_dict()["path_strategy"], "stock_front")
        self.assertEqual(
            AdaptiveClearing(0.25, path_strategy="stock_front", cutting_sense="conventional").to_dict()[
                "cutting_sense"
            ],
            "conventional",
        )
        c = dict(config())
        c["operations"] = (replace(c["operations"][0], path_strategy="not-a-strategy"),)
        with self.assertRaisesRegex(SimulationError, "unknown variant"):
            with Simulation(**c) as sim:
                list(sim.run_operations())

    def test_front_sweep_failure_is_not_cached_as_a_complete_operation(self):
        from tempfile import TemporaryDirectory
        from camlib.jobs import RegenerationCache
        from camlib.operations import HelicalEntry
        from camlib.simulation import SweepOptions

        c = dict(config())
        c["operations"] = (
            replace(c["operations"][0], floor_z_mm=9, path_strategy="stock_front", entry=HelicalEntry(angle_deg=3)),
        )
        # Deliberately exhaust native sweep work: a real producer error must
        # propagate and drop its cache writer even though this path can finish
        # when given its normal resource budget.
        c["sweep"] = SweepOptions(max_cell_visits=1)
        prefixes = []
        with TemporaryDirectory() as directory:
            c["regeneration"] = RegenerationCache(directory).options({})
            for revision in ("front-incomplete-a", "front-incomplete-b"):
                c["revision"] = revision
                begun = False
                with Simulation(**c) as sim:
                    with self.assertRaisesRegex(SimulationError, "cell-visit budget exceeded"):
                        for update in sim.run_operations():
                            if isinstance(update, OperationUpdate):
                                self.assertEqual(update.stage, "begin")
                                self.assertFalse(update.report["regeneration"]["cache_hit"])
                                begun = True
                    self.assertTrue(begun)
                    self.assertFalse(sim.state.finished)
                    self.assertGreater(sim.state.completed, 0)
                    self.assertEqual(sim.state.manifest["run"]["revision"], revision)
                    prefixes.append(sim.state.heights.tobytes())
            self.assertEqual(prefixes[0], prefixes[1])
            self.assertFalse(list(Path(directory).rglob("*.tmp")))
