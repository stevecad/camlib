"""Standalone completed journals preserve actual native and posted stock/proofs."""

from dataclasses import replace
import hashlib
import importlib.util
import json
from pathlib import Path
import shutil
import subprocess
import sys
from tempfile import TemporaryDirectory
import threading
import unittest
from unittest.mock import patch

from camlib.playback import Cursor, Player, RecordedRun
from camlib.posted import PostedRun
from camlib.simulation import ProtocolError, SimulationError
from test_playback import run
from test_simulation import moves
from test_posted_playback import config


def job_example():
    path = Path(__file__).resolve().parents[2] / 'examples/standalone_job.py'
    spec = importlib.util.spec_from_file_location('standalone_example', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def digest(player):
    return hashlib.sha256(player.state.heights).hexdigest()


class PersistenceTest(unittest.TestCase):
    def test_native_stock_fractional_seek_clock_and_process_restart(self):
        with TemporaryDirectory() as directory:
            saved = Path(directory) / 'saved'
            expected = {}
            with run(checkpoint_every=2) as recording:
                recording.wait()
                player = Player(recording, compact=True)
                for cursor in (Cursor(0, 0.3), Cursor(2, 0.6), Cursor(4), Cursor(1, 0.2)):
                    player.seek(cursor)
                    expected[str(cursor)] = digest(player)
                recording.save(saved)
            self.assertFalse(recording.directory.exists())
            with patch('subprocess.Popen', side_effect=AssertionError('opening started a process')):
                reopened = RecordedRun.open(saved)
            with reopened:
                self.assertFalse(reopened.status()['producing'])
                self.assertTrue(reopened.wait_stopped())
                player = Player(reopened, compact=True)
                for cursor in (Cursor(4), Cursor(1, 0.2), Cursor(2, 0.6), Cursor(0, 0.3)):
                    player.seek(cursor)
                    self.assertEqual(digest(player), expected[str(cursor)])
                player.seek(Cursor())
                player.set_speed(2)
                player.play()
                player.tick(0.1)
                self.assertAlmostEqual(player.seconds, 0.2)
                player.pause()
                old = player.cursor
                player.tick(1)
                self.assertEqual(player.cursor, old)
            self.assertTrue(saved.exists())
            code = '''import hashlib,json,sys
from camlib.playback import RecordedRun,Player,Cursor
with RecordedRun.open(sys.argv[1]) as run:
 p=Player(run,compact=True);p.seek(Cursor(4))
 print(json.dumps(dict(stock=hashlib.sha256(p.state.heights).hexdigest(),finished=p.state.finished)))
'''
            output = subprocess.check_output([sys.executable, '-c', code, str(saved)], text=True)
            result = json.loads(output)
            self.assertEqual(result['stock'], expected[str(Cursor(4))])
            self.assertTrue(result['finished'])
            with self.assertRaisesRegex(SimulationError, 'closed'):
                reopened.wait_ready()

    def test_posted_tools_controls_checks_nc_and_compact_stock_survive_close(self):
        example = job_example()
        job = example.build_rest_job()
        with TemporaryDirectory() as directory:
            with job.post('top', machine=example.rest_machine_for(job), checkpoint_every=3) as posted:
                posted.wait()
                count = posted.status()['available']
                player = Player(posted, compact=True)
                player.seek(Cursor(count))
                final = digest(player)
                verification = player.verification
                status = posted.posting_status()
                lines = list(posted.nc_lines(1, limit=256))
                controls = list(posted.operation_controls())
                first = posted.posted_motion(0)
                last = posted.posted_motion(count - 1)
                player.seek(Cursor(count // 2, 0.4))
                partial = digest(player)
                saved = posted.save(Path(directory) / 'posted')
            with PostedRun.open(saved) as reopened:
                self.assertEqual(reopened.posting_status(), status)
                self.assertEqual(list(reopened.nc_lines(1, limit=256)), lines)
                self.assertEqual(list(reopened.operation_controls()), controls)
                self.assertEqual(reopened.posted_motion(0), first)
                self.assertEqual(reopened.posted_motion(count - 1), last)
                player = Player(reopened, compact=True)
                player.seek(Cursor(count // 2, 0.4))
                self.assertEqual(digest(player), partial)
                player.seek(Cursor(count))
                self.assertEqual(digest(player), final)
                self.assertEqual(player.verification, verification)
                self.assertEqual(reopened.status()['posting']['finished'], True)

    def test_native_operation_and_material_reports_retain_exact_identity_and_uncertainty(self):
        from camlib.material import Target
        from camlib.verification import MaterialCheck, StockContactOptions, StreamChecks

        example = job_example()
        job = example.build_job()
        setup = job.setups[0]
        pocket = setup.operations[1]
        checks = StreamChecks(
            stock_contact=StockContactOptions(),
            material=(MaterialCheck(Target("floor", "1", pocket.region, pocket.floor_z_mm)),),
        )
        job = replace(job, setups=(replace(setup, verification=checks),))
        with TemporaryDirectory() as directory:
            with job.record('top', checkpoint_every=3, rapid_mm_min=6000) as recording:
                recording.wait()
                count = recording.status()['available']
                controls = list(recording.operation_controls())
                self.assertEqual({c['operation'] for c in controls}, {'face', 'pocket'})
                verification = recording.verification_status()
                self.assertIsNotNone(verification['material'])
                saved = recording.save(Path(directory) / 'material')
            with RecordedRun.open(saved) as reopened:
                self.assertEqual(list(reopened.operation_controls()), controls)
                self.assertEqual(reopened.verification_status(), verification)
                player = Player(reopened)
                player.seek(Cursor(count // 2))
                self.assertLessEqual(player.verification['at_completed'], count // 2)
                player.seek(Cursor(count))
                self.assertEqual(player.verification, verification)

    def test_corruption_foreign_journal_metadata_and_unsafe_inventory_reject(self):
        with TemporaryDirectory() as directory:
            root = Path(directory)
            with run() as recording:
                recording.wait()
                saved = recording.save(root / 'good')
            with run() as recording:
                recording.wait()
                foreign = recording.save(root / 'foreign')
            for name in (
                'motions.bin',
                'motions.idx',
                'controls.bin',
                'verification.idx',
                'end.bin',
                '0000000000000004.checkpoint.gz',
            ):
                with self.subTest(file=name):
                    copy = root / 'bad'
                    shutil.copytree(saved, copy)
                    p = copy / name
                    p.write_bytes(p.read_bytes() + b'changed')
                    with self.assertRaises(ProtocolError):
                        RecordedRun.open(copy)
                    shutil.rmtree(copy)
            for change in ('foreign', 'traversal', 'count', 'symlink', 'extra'):
                with self.subTest(change=change):
                    copy = root / 'bad'
                    shutil.copytree(saved, copy)
                    meta = copy / 'run.json'
                    data = json.loads(meta.read_text())
                    if change == 'foreign':
                        shutil.copy2(foreign / 'manifest.bin', copy / 'manifest.bin')
                    elif change == 'traversal':
                        data['integrity'][0]['path'] = '../outside'
                        meta.write_text(json.dumps(data))
                    elif change == 'count':
                        data['status']['available'] += 1
                        data.pop('metadata_sha256')
                        data['metadata_sha256'] = hashlib.sha256(
                            json.dumps(data, sort_keys=True, separators=(',', ':')).encode()
                        ).hexdigest()
                        meta.write_text(json.dumps(data))
                    elif change == 'symlink':
                        (copy / 'motions.bin').unlink()
                        try:
                            (copy / 'motions.bin').symlink_to(saved / 'motions.bin')
                        except OSError as error:
                            if getattr(error, 'winerror', None) != 1314:
                                raise
                            shutil.rmtree(copy)
                            self.skipTest('Windows symlink test requires Developer Mode or symlink privilege')
                    else:
                        (copy / 'extra').write_text('unexpected')
                    with self.assertRaises(ProtocolError):
                        RecordedRun.open(copy)
                    shutil.rmtree(copy)
            with self.assertRaisesRegex(ValueError, 'kind'):
                PostedRun.open(saved)
            with self.assertRaisesRegex(SimulationError, 'budget'):
                RecordedRun.open(saved, max_bytes=1024)

    def test_cancel_copy_open_and_existing_destination_leave_original_intact(self):
        with TemporaryDirectory() as directory:
            root = Path(directory)
            with run() as recording:
                recording.wait()
                saved = recording.save(root / 'good')
                before = (saved / 'run.json').read_bytes()
                with self.assertRaises(FileExistsError):
                    recording.save(saved)
                self.assertEqual((saved / 'run.json').read_bytes(), before)
                calls = 0

                def cancel():
                    nonlocal calls
                    calls += 1
                    return calls > 4

                with self.assertRaisesRegex(SimulationError, 'cancelled'):
                    recording.save(root / 'cancelled', cancel=cancel)
                self.assertFalse((root / 'cancelled').exists())
                with self.assertRaisesRegex(SimulationError, 'budget'):
                    recording.save(root / 'small', max_bytes=1024)
                self.assertFalse((root / 'small').exists())
                with self.assertRaisesRegex(ValueError, 'outside'):
                    recording.save(recording.directory / 'recursive')
            with self.assertRaisesRegex(SimulationError, 'cancelled'):
                RecordedRun.open(saved, cancel=lambda: True)
            with RecordedRun.open(saved) as reopened:
                self.assertTrue(reopened.status()['finished'])

    def test_live_cancelled_and_failed_runs_cannot_be_committed(self):
        gate = threading.Event()

        def source():
            yield moves()[0]
            gate.wait(3)

        with TemporaryDirectory() as directory:
            with run(source(), batch_size=1) as recording:
                try:
                    recording.wait_available(1)
                    with self.assertRaisesRegex(SimulationError, 'finished'):
                        recording.save(Path(directory) / 'live')
                    recording.cancel()
                    gate.set()
                    self.assertTrue(recording.wait_stopped())
                    with self.assertRaisesRegex(SimulationError, 'finished'):
                        recording.save(Path(directory) / 'cancelled')
                finally:
                    gate.set()

            def failure():
                yield moves()[0]
                raise ValueError('source failure')

            with run(failure(), batch_size=1) as recording:
                with self.assertRaises(SimulationError):
                    recording.wait()
                with self.assertRaisesRegex(SimulationError, 'finished'):
                    recording.save(Path(directory) / 'failed')
            self.assertEqual(list(Path(directory).iterdir()), [])

    def test_empty_native_and_posted_status_extensions_are_preserved(self):
        with TemporaryDirectory() as directory:
            with run(()) as recording:
                recording.wait()
                saved = recording.save(Path(directory) / 'empty')
            with RecordedRun.open(saved) as reopened:
                player = Player(reopened)
                player.seek(Cursor())
                self.assertTrue(player.state.finished)
            program, options = config(max_normal_accel_mm_s2=80)
            program = replace(program, events=(*program.events[:3], program.events[4]))
            with PostedRun(program, **options) as posted:
                posted.wait()
                saved = posted.save(Path(directory) / 'policy')
                status = posted.posting_status()
                manifest = posted.posting_manifest()
            with PostedRun.open(saved) as reopened:
                self.assertEqual(reopened.posting_status(), status)
                self.assertEqual(reopened.posting_manifest(), manifest)


if __name__ == '__main__':
    unittest.main()
