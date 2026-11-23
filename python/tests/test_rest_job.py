"""Two-cutter CAD job: real remaining stock, regeneration and checked posting."""

from dataclasses import replace
from functools import lru_cache
import importlib.util
import os
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest

from camlib.jobs import RegenerationCache
from camlib.playback import Cursor, Player
from camlib.simulation import OperationUpdate, ProtocolError, StockState, Update, curve_length


@lru_cache(None)
def example(name="cadquery_job"):
    path = Path(__file__).resolve().parents[2] / "examples" / f"{name}.py"
    spec = importlib.util.spec_from_file_location("rest_job_example", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def engine():
    return os.environ.get("CAMLIB_TEST_ENGINE")


def collect(job, cache=None):
    ends, volumes, completed, motions = {}, {}, {}, []
    rest_air_mm = 0.0
    first_frame = None
    with job.regenerate(cache, executable=engine()) as run:
        for event in run:
            update = event.update
            sim = run._simulation
            if isinstance(update, OperationUpdate) and update.stage == "end":
                ends[update.operation] = update.report
                volumes[update.operation] = sim.state.volume()["removed_lower_mm3"]
                completed[update.operation] = sim.state.completed
            elif isinstance(update, Update):
                motions.append(update.motion)
                if (
                    update.motion["operation"] == "rest"
                    and update.motion["kind"] != "rapid"
                    and not update.changed_count
                ):
                    rest_air_mm += curve_length(update.motion["curve"])
                if first_frame is None:
                    first_frame = update.frame
            manifest = sim.manifest_frame
        assert run.finished
        return dict(
            ends=ends,
            volumes=volumes,
            completed=completed,
            rest_air_mm=rest_air_mm,
            motions=motions,
            stats=run.statistics,
            state=run.states["top"],
            manifest=manifest,
            first_frame=first_frame,
        )


class RestJobTest(unittest.TestCase):
    def test_rest_pocket_skips_air_above_committed_cuts(self):
        fixture = example("standalone_job")
        job = fixture.build_rest_job()
        result = collect(job)
        self.assertGreater(result["ends"]["rest"]["rest_history_skipped_strokes"], 0)
        self.assertLess(result["rest_air_mm"], 35.0)
        # Four fractions of the corner cells: 2 mm deep, 1 mm^2 in all. Both
        # operations plan whole stepdowns from the faced top (#381), so the
        # rest cut takes the corners to the pocket's floor bound exactly.
        self.assertAlmostEqual(result["volumes"]["rest"] - result["volumes"]["pocket"], 2.0, places=7)

    def test_cad_two_tools_stock_cache_and_upstream_changes(self):
        with TemporaryDirectory() as directory:
            cache = RegenerationCache(directory)
            job = example().build_rest_job()
            cold = collect(job, cache)
            rest = cold["ends"]["rest"]
            self.assertGreater(rest["rest_skipped_strokes"], 0)
            self.assertGreater(cold["volumes"]["rest"], cold["volumes"]["pocket"])
            self.assertEqual(rest["rest_from_generation"], cold["completed"]["pocket"])
            self.assertEqual(rest["rest_through_generation"], cold["completed"]["rest"])
            self.assertEqual({m["tool"] for m in cold["motions"]}, {"mill4", "mill2"})
            warm = collect(example().build_rest_job(revision=2), cache)
            self.assertTrue(all(s["cache_hit"] for s in warm["stats"].values()))
            self.assertEqual(cold["state"].heights.tobytes(), warm["state"].heights.tobytes())
            receiver = StockState(warm["manifest"])
            with self.assertRaises(ProtocolError):
                receiver.accept(cold["first_frame"])
            self.assertEqual(receiver.completed, 0)
            for changed in (
                example().build_rest_job(revision=3, small_diameter=1.5),
                example().build_rest_job(revision=4, rough_floor=4),
            ):
                result = collect(changed, cache)
                self.assertTrue(result["stats"][("top", "face")]["cache_hit"])
                self.assertFalse(result["stats"][("top", "rest")]["cache_hit"])
                reference = collect(changed)
                self.assertEqual(result["state"].heights.tobytes(), reference["state"].heights.tobytes())

    def test_cancel_during_rest_then_new_revision_retries_to_same_stock(self):
        with TemporaryDirectory() as directory:
            cache = RegenerationCache(directory)
            job = example().build_rest_job()
            with job.regenerate(cache, executable=engine()) as run:
                stream = iter(run)
                for event in stream:
                    update = event.update
                    if isinstance(update, Update) and update.motion["operation"] == "rest":
                        break
                else:
                    self.fail("no rest motion reached the consumer")
                run.cancel()
                stream.close()
                self.assertFalse(run.finished)
                self.assertNotIn("top", run.states)
            revised = replace(job, revision="retry")
            retry = collect(revised, cache)
            self.assertTrue(retry["stats"][("top", "pocket")]["cache_hit"])
            self.assertEqual(retry["state"].heights.tobytes(), collect(revised)["state"].heights.tobytes())

    def test_checked_post_tool_changes_and_seek_keep_actual_rest_stock(self):
        job = example().build_rest_job()
        with job.post("top", machine=example().rest_machine_for(job), executable=engine(), checkpoint_every=8) as run:
            run.wait(timeout=60)
            self.assertTrue(run.posting_status()["finished"])
            lines = []
            start = 1
            while page := run.nc_lines(start, limit=256):
                lines.extend(row["text"] for row in page)
                start = page[-1]["number"] + 1
            self.assertEqual(lines.count("M6"), 2)
            self.assertIn("T7", lines)
            self.assertIn("T8", lines)
            player = Player(run)
            end = Cursor(run.status()["available"])
            player.seek(end)
            final = player.state.heights.tobytes()
            self.assertGreater(player.state.volume()["removed_lower_mm3"], 0)
            player.seek(Cursor())
            player.seek(end)
            self.assertEqual(player.state.heights.tobytes(), final)
            self.assertTrue(run.posted_motion(end.completed - 1)["stock_committed"])
