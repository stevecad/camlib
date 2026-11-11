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
    return example().adaptive_job()


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
                self.assertNotIn("path_strategy", op.to_dict())
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
