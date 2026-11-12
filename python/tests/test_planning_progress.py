"""Planning progress (#198): which operation a run is planning and how far along.

A run asked for progress yields ``ProgressUpdate``s between its own updates:
layers done of layers planned for roughing, the area left in each layer for
adaptive clearing, passes done of passes planned for surface finishing, and
no fraction where the planner can count nothing. The fraction never decreases
within an operation and ends at 1. Progress is no part of the run: a run that
is not asked yields none, the motions and stock are the same either way, and a
recorded run stores none.
"""

import gzip
import hashlib
import importlib.util
import json
from pathlib import Path
import time
import unittest

from camlib.simulation import OperationUpdate, ProgressUpdate, ProtocolError, Simulation, Update

ROOT = Path(__file__).resolve().parents[2]


def fixture(name):
    manifest = json.loads((ROOT / "fixtures/operations" / f"{name.removesuffix('-2')}.json").read_text())
    if name == "roughing-adaptive-2":
        # Two depth levels of adaptive clearing with the engagement front,
        # with a 6 mm tool: a few seconds instead of the fixture's minute.
        operation = manifest["operations"][0]
        assert operation["clearing"]["path_strategy"] == "engagement_front"
        operation["floor_z_mm"] = 6
        operation["clearing"]["max_radial_engagement_mm"] = 1.0
        manifest["tools"][0]["cutter"].update(diameter_mm=6, shank_diameter_mm=6)
    return dict(
        stock=manifest["stock"],
        tools=manifest["tools"],
        operations=manifest["operations"],
        revision="1",
        run_id="progress",
        initial_position_mm=manifest["initial_position_mm"],
        sweep=manifest["sweep"],
    )


def run(name, interval):
    """(progress updates, every other update's frame bytes, final stock digest)."""
    progress, frames = [], []
    with Simulation(**fixture(name)) as sim:
        updates = sim.run_operations() if interval is None else sim.run_operations(progress_interval_s=interval)
        for update in updates:
            if isinstance(update, ProgressUpdate):
                progress.append(update)
            else:
                frames.append(update.frame.raw)
        assert sim.state.finished
        stock = hashlib.sha256(sim.state.heights).hexdigest()
    return progress, frames, stock


def job_example():
    path = ROOT / "examples/standalone_job.py"
    spec = importlib.util.spec_from_file_location("standalone_example", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class PlanningProgressTest(unittest.TestCase):
    def check(self, name, operation, basis):
        """The acceptance on a one-operation run; returns its updates."""
        plain_progress, plain, plain_stock = run(name, None)
        self.assertEqual(plain_progress, [])
        progress, frames, stock = run(name, 0)
        # The same run: every motion, control and check frame, and the stock.
        self.assertEqual(frames, plain)
        self.assertEqual(stock, plain_stock)
        self.assertGreaterEqual(len(progress), 3)
        for update in progress:
            self.assertEqual((update.operation, update.index, update.operations), (operation, 0, 1))
        self.assertEqual((progress[0].basis, progress[0].fraction, progress[0].done), (None, None, None))
        fractions = [update.fraction for update in progress[1:]]
        self.assertEqual({update.basis for update in progress[1:]}, {basis})
        self.assertEqual(fractions, sorted(fractions))
        self.assertGreaterEqual(fractions[0], 0)
        self.assertEqual(fractions[-1], 1)
        self.assertEqual(progress[-1].done, progress[-1].total)
        return progress

    def test_roughing_counts_layers(self):
        progress = self.check("roughing", "rough", "layers")
        total = progress[1].total
        self.assertEqual(total, 4)  # 10 mm down to 2 mm in 2 mm steps
        self.assertEqual([u.done for u in progress[1:]], list(range(total + 1)))
        self.assertEqual([u.fraction for u in progress[1:]], [done / total for done in range(total + 1)])
        self.assertTrue(all(u.area_left_mm2 is None for u in progress))

    def test_adaptive_clearing_counts_the_area_left_in_each_layer(self):
        progress = self.check("roughing-adaptive-2", "rough", "area")
        total = progress[1].total
        self.assertEqual(total, 2)
        for layer in range(total):
            areas = [
                (u.area_left_mm2, u.area_start_mm2, u.fraction)
                for u in progress
                if u.done == layer and u.area_left_mm2 is not None
            ]
            self.assertGreaterEqual(len(areas), 2, layer)
            self.assertEqual(areas[0][0], areas[0][1])
            self.assertLess(areas[-1][0], 0.5 * areas[0][1])
            for left, start, fraction in areas:
                self.assertTrue(0 <= left <= start)
                # (layers done + the layer's cleared share) / layers, or more
                # where an earlier, higher value is held.
                self.assertGreaterEqual(fraction, (layer + 1 - left / start) / total - 1e-12)
                self.assertLessEqual(fraction, (layer + 1) / total)
        # Movement inside a layer, not only at its ends.
        self.assertTrue(any(0 < u.fraction * total % 1 < 1 for u in progress[1:]))

    def test_surface_finishing_counts_passes(self):
        progress = self.check("parallel", "finish-roof", "passes")
        total = progress[1].total
        self.assertGreater(total, 4)
        self.assertEqual([u.done for u in progress[1:]], list(range(total + 1)))

    def test_an_operation_with_nothing_to_count_has_no_fraction(self):
        _, plain, _ = run("holes", None)
        progress, frames, _ = run("holes", 0)
        self.assertEqual(frames, plain)
        self.assertEqual(
            [(u.operation, u.index, u.operations) for u in progress], [("pilot", 0, 2), ("finish-holes", 1, 2)]
        )
        for update in progress:
            self.assertEqual((update.basis, update.fraction, update.done, update.total), (None, None, None, None))

    def test_progress_starts_before_the_operation_begins_and_at_an_interval(self):
        seen = []
        with Simulation(**fixture("roughing")) as sim:
            for update in sim.run_operations(progress_interval_s=0.001):
                if isinstance(update, ProgressUpdate):
                    seen.append(update)
                elif not seen:
                    self.fail("an update of the run came before its first progress update")
                elif isinstance(update, OperationUpdate) and update.stage == "begin":
                    begun_after = len(seen)
        self.assertGreaterEqual(begun_after, 1)
        fractions = [u.fraction for u in seen if u.fraction is not None]
        self.assertEqual(fractions, sorted(fractions))
        self.assertEqual(fractions[-1], 1)

    def test_a_lathe_run_reports_each_operation_without_a_fraction(self):
        from test_turning import job as lathe_job

        setup = lathe_job().job.compile().setup("lathe")

        def frames(interval):
            progress, rest = [], []
            with Simulation(**setup.simulation_options(), run_id="progress") as sim:
                for update in sim.run_operations(progress_interval_s=interval):
                    (progress if isinstance(update, ProgressUpdate) else rest).append(update)
            return progress, [update.frame.raw for update in rest]

        _, plain = frames(None)
        progress, rest = frames(0)
        self.assertEqual(rest, plain)
        self.assertEqual(
            [(u.operation, u.index, u.operations) for u in progress],
            [("face", 0, 3), ("rough", 1, 3), ("finish", 2, 3)],
        )
        self.assertTrue(all(u.basis is None and u.fraction is None for u in progress))

    def test_a_run_reporting_progress_still_cancels(self):
        with Simulation(**fixture("roughing")) as sim:
            updates = sim.run_operations(progress_interval_s=0.001)
            for _, update in zip(range(20), updates):
                pass
            sim.cancel()
            with self.assertRaises(Exception):
                for _ in updates:
                    pass
            self.assertFalse(sim.state.finished)

    def gaps(self, name, interval, *, until_first_move=False, **stock):
        """Wall-clock seconds between consecutive progress updates of a run
        (from its start, and up to its end or first move), and how long it
        ran."""
        options = fixture(name)
        options["stock"].update(stock)
        gaps = []
        with Simulation(**options) as sim:
            updates = sim.run_operations(progress_interval_s=interval)
            started = last = time.perf_counter()
            for update in updates:
                now = time.perf_counter()
                if isinstance(update, ProgressUpdate):
                    gaps.append(now - last)
                    last = now
                elif until_first_move and isinstance(update, Update):
                    break
            gaps.append(time.perf_counter() - last)
            if until_first_move:
                sim.cancel()
        return gaps, time.perf_counter() - started

    def test_a_plan_reports_at_its_interval_before_its_first_move(self):
        # The adaptive roughing fixture plans for seconds before it moves.
        # The interval is far below the planning time on any machine, so the
        # test does not depend on how fast this one is.
        interval = 0.002
        gaps, planned = self.gaps("roughing-adaptive", interval, until_first_move=True)
        self.assertGreater(planned, 20 * interval, "planning no longer spans several intervals: use a slower plan")
        self.assertGreaterEqual(len(gaps), planned / interval / 4)
        self.assertLess(max(gaps), 2 * interval + 0.5, gaps)

    def test_a_plan_reports_at_its_interval_between_moves(self):
        # Between two moves the planner chooses its next pass; the timer
        # sends progress through that too. The interval is a few
        # milliseconds, far below any plan's time, so the test does not
        # depend on how fast the planner or the machine is (the scan this
        # once relied on being slow was made fast, #192).
        interval = 0.002
        gaps, planned = self.gaps("roughing-open-block", interval, cell_size_mm=0.15)
        self.assertGreater(planned, 20 * interval, "planning no longer spans several intervals: use a slower plan")
        self.assertGreaterEqual(len(gaps), planned / interval / 4)
        self.assertLess(max(gaps), 2 * interval + 0.5, gaps)

    def test_updates_between_a_move_and_its_check_leave_the_checked_run_unchanged(self):
        # The timer may send an update between any two frames of the run,
        # also between a move and its verification record.
        from camlib.verification import StockContactOptions, StreamChecks

        def frames(interval):
            progress, rest = 0, []
            options = fixture("roughing")
            with Simulation(**options, verification=StreamChecks(stock_contact=StockContactOptions())) as sim:
                for update in sim.run_operations(progress_interval_s=interval):
                    if isinstance(update, ProgressUpdate):
                        progress += 1
                    else:
                        rest.append(update.frame.raw)
                self.assertTrue(sim.state.finished)
            return progress, rest

        _, plain = frames(None)
        self.assertTrue(any(raw[10] == 8 for raw in plain))  # verification records
        for _ in range(3):
            progress, rest = frames(0.001)
            self.assertGreater(progress, 2)
            self.assertEqual(rest, plain)

    def test_the_interval_is_checked(self):
        for interval in (-1, 0.0001, 1.5, float("nan"), True, "1"):
            with Simulation(**fixture("holes")) as sim:
                with self.assertRaises(ValueError):
                    next(sim.run_operations(progress_interval_s=interval))

    def test_a_progress_frame_is_not_a_stored_frame(self):
        from camlib.simulation import Frame
        import struct
        import zlib

        body = b"{}"
        header = struct.pack("<8sHHI", b"CAMLSTRM", 1, 12, len(body))
        raw = header + struct.pack("<I", zlib.crc32(body, zlib.crc32(header))) + body
        with self.assertRaises(ProtocolError):
            Frame.parse(raw)

    def test_a_recorded_run_reports_progress_and_stores_none(self):
        """The journal and the saved run are the same bytes whether the run
        reported progress (as ``job.record`` does unless told not to), on
        every change, or not at all."""
        from tempfile import TemporaryDirectory

        from camlib.playback import Cursor, Player, RecordedRun

        def contents(directory):
            files = {}
            for path in sorted(Path(directory).iterdir()):
                data = path.read_bytes()
                if path.suffix == ".gz":
                    # A gzip member carries a timestamp: compare its content.
                    data = gzip.decompress(data)
                elif path.name == "run.json":
                    # What differs between any two runs of the same job: the
                    # one wall-clock figure a saved run keeps, and the digests
                    # over it and over the gzip files.
                    saved = json.loads(data)
                    self.assertIsInstance(saved["status"].pop("first_stock_seconds"), float)
                    for digest in ("integrity", "metadata_sha256"):
                        self.assertIn(digest, saved)
                        del saved[digest]
                    data = json.dumps(saved, sort_keys=True).encode()
                files[path.name] = hashlib.sha256(data).hexdigest()
            return files

        job = job_example().build_job()
        results = {}
        # "default" passes nothing: RecordedRun's own 0.25 s.
        for case, options in (
            ("off", dict(progress_interval_s=None)),
            ("changes", dict(progress_interval_s=0)),
            ("default", {}),
        ):
            with TemporaryDirectory() as directory:
                with job.record("top", rapid_mm_min=6000, run_id="progress", **options) as recording:
                    self.assertTrue(recording.wait()["finished"])
                    latest = recording.planning_progress()
                    if case == "off":
                        self.assertIsNone(latest)
                    else:
                        # The last operation, complete.
                        self.assertEqual((latest.operation, latest.index, latest.operations), ("pocket", 1, 2))
                        self.assertEqual((latest.basis, latest.fraction), ("layers", 1))
                    self.assertNotIn("progress", recording.status())
                    journal = contents(recording.directory)
                    player = Player(recording)
                    player.seek(Cursor(recording.status()["available"]))
                    stock = hashlib.sha256(player.state.heights).hexdigest()
                    saved = recording.save(Path(directory) / "saved")
                with RecordedRun.open(saved) as reopened:
                    self.assertIsNone(reopened.planning_progress())
                    status = reopened.status()
                    status.pop("first_stock_seconds")
                results[case] = (journal, stock, contents(saved), status)
        self.assertIn("run.json", results["off"][2])
        self.assertEqual(results["changes"], results["off"])
        self.assertEqual(results["default"], results["off"])

    def test_posting_reports_progress_and_posts_the_same_program(self):
        example = job_example()
        job = example.build_job()
        machine = example.machine_for(job)
        plain = job.post_nc("top", machine=machine, accept={"unchecked_machine"})
        seen = []
        posted = job.post_nc(
            "top", machine=machine, accept={"unchecked_machine"}, on_progress=seen.append, progress_interval_s=0
        )
        self.assertEqual(posted.text, plain.text)
        self.assertEqual({u.operation for u in seen}, {"face", "pocket"})
        for operation in ("face", "pocket"):
            fractions = [u.fraction for u in seen if u.operation == operation and u.fraction is not None]
            self.assertEqual(fractions, sorted(fractions))
        self.assertEqual((seen[-1].operation, seen[-1].fraction), ("pocket", 1))

    def test_a_job_run_yields_progress_only_when_asked(self):
        job = job_example().build_job()
        plain = [u.update for u in job.regenerate()]
        self.assertFalse(any(isinstance(u, ProgressUpdate) for u in plain))
        asked = [u.update for u in job.regenerate(progress_interval_s=0)]
        progress = [u for u in asked if isinstance(u, ProgressUpdate)]
        self.assertEqual({u.operation for u in progress}, {"face", "pocket"})
        self.assertEqual(
            [u.motion for u in asked if isinstance(u, Update)],
            [u.motion for u in plain if isinstance(u, Update)],
        )


if __name__ == "__main__":
    unittest.main()
