from camlib.postprocessor import GCodeDialect
from dataclasses import replace
import json
import struct
import threading
import unittest

from camlib.motion import Arc, Coolant, Helix, Line, Motion, Spindle
from camlib.post import Chunk, GCodePost, LoweringOptions, Machine
from camlib.posted import OperationEvents, PostedRun
from camlib.playback import Cursor, NotReady, Player
from camlib.simulation import ProtocolError, Simulation, SimulationError, StockSpec
from camlib.verification import StockContactOptions, StreamChecks
from test_post import fixture


def config(**options):
    p, s = fixture()
    return p, dict(
        machine=Machine(
            'Normal Python posted playback',
            s.tools,
            **{
                'dialect': GCodeDialect.linuxcnc(),
                'output_units': 'mm',
                'decimals': 3,
                'max_feed_mm_min': 10000,
                'max_spindle_rpm': 30000,
                'planes': ('xy', 'xz', 'yz'),
                'arcs': True,
                'helices': True,
                'max_sweep_rad': 6.283185307179586,
                'max_tool_number': 99999,
                'max_length_register': 99999,
                'tool_changes': True,
                'flood': True,
                'mist': True,
                'tool_change_position_mode': 'table',
                'rapid_mm_min': 6000,
                **options,
            },
        ),
        stock=StockSpec((0, -10, 0), (20, 10, 10), 0.5),
    )


class PostedPlaybackTest(unittest.TestCase):
    def test_nominal_curve_policy_survives_journal_and_actual_stock_seeking(self):
        p, options = config(max_normal_accel_mm_s2=80)
        p = replace(p, events=(*p.events[:3], p.events[4]))
        for lowering in (None, LoweringOptions()):
            with self.subTest(lowering=lowering), PostedRun(p, lowering=lowering, **options) as run:
                run.wait()
                policy = run.posting_manifest()['nominal_normal_acceleration']
                self.assertEqual(policy['limit_mm_s2'], 80)
                self.assertEqual(policy['source_feed_curves_checked'], 0)
                self.assertEqual(policy['decoded_feed_curves_checked'], 0)
                final, _, _ = run._read_post(run.posting_status()['chunks'] - 1)
                checks = final['nominal_normal_acceleration']
                self.assertEqual(checks['source_feed_curves_checked'], 1)
                self.assertEqual(checks['decoded_feed_curves_checked'], 1)
                self.assertEqual(checks['source_rapid_speed_unknown'], 0)
                self.assertEqual(checks['decoded_rapid_speed_unknown'], 0)
                self.assertIn('segment_interior', checks['scope'])
                player = Player(run)
                initial = player.state.heights.tobytes()
                player.seek(Cursor(1))
                cut = player.state.heights.tobytes()
                self.assertNotEqual(cut, initial)
                self.assertGreater(player.state.volume()['removed_lower_mm3'], 0)
                player.seek(Cursor())
                self.assertEqual(player.state.heights.tobytes(), initial)
                player.seek(Cursor(1))
                self.assertEqual(player.state.heights.tobytes(), cut)

    def test_curve_limit_rejection_keeps_prior_stock_and_posted_prefix(self):
        import math

        p, options = config(max_normal_accel_mm_s2=40)
        first = p.events[4]
        bad = Motion(2, 'curve', 't1', Arc((10, 0, 3), (6, 0, 3), (8, 0, 3), math.pi), 600)
        # Direct posting commits up to 64 events atomically. Put the invalid
        # motion in the next batch so an earlier stock/code prefix exists.
        p = replace(p, events=(*p.events[:3], first, *([Coolant('off')] * 60), bad))
        for lowering in (None, LoweringOptions()):
            with self.subTest(lowering=lowering), PostedRun(p, lowering=lowering, batch_size=1, **options) as run:
                with self.assertRaisesRegex(SimulationError, 'source motion 2 nominal normal acceleration'):
                    run.wait()
                self.assertEqual(run.status()['available'], 1)
                self.assertEqual(run.posting_status()['checked_motions'], 1)
                self.assertFalse(run.posting_status()['finished'])
                with self.assertRaises(NotReady):
                    run.posted_motion(1)
                player = Player(run)
                player.seek(Cursor(1))
                self.assertEqual(player.state.position, (10, 0, 3))
                self.assertGreater(player.state.volume()['removed_lower_mm3'], 0)
                final, _, _ = run._read_post(run.posting_status()['chunks'] - 1)
                checks = final['nominal_normal_acceleration']
                self.assertEqual(checks['source_feed_curves_checked'], 1)
                self.assertEqual(checks['decoded_feed_curves_checked'], 1)

    def test_actual_checked_control_effects_survive_the_retained_journal(self):
        p, options = config()
        expected = (
            *p.events[:3],
            Coolant('flood'),
            p.events[4],  # flood from off: no M9 first
            Coolant('off'),
            Coolant('mist'),
            *p.events[6:],
        )
        for lowering in (None, LoweringOptions()):
            with self.subTest(lowering=lowering), PostedRun(p, lowering=lowering, **options) as run:
                run.wait()
                events = []
                for index in range(run.posting_status()['chunks']):
                    data, _, _ = run._read_post(index)
                    chunk = Chunk(json.dumps(data).encode())
                    events.extend(chunk.events)
                    for check, mapping in zip(data['checked'], data['mappings']):
                        self.assertEqual(check['event_index'], mapping['event_index'])
                        self.assertEqual(check['line_interval'], mapping['line_interval'])
                        if check['control_effects']:
                            self.assertIsNone(mapping['source_motion_id'])
                            self.assertIsNone(mapping['source_fraction_interval'])
                self.assertEqual(tuple(events), expected)
                self.assertEqual(run.status()['available'], 1)
                self.assertEqual(run.posting_manifest()['identity']['abi'], 5 if lowering is None else 6)
                self.assertNotIn('nominal_normal_acceleration', run.posting_manifest())

    def test_direct_arbitrary_ids_actual_stock_and_source_nc_seek(self):
        p, options = config()
        move = replace(p.events[4], id=2**63 + 53)
        p = replace(p, events=(*p.events[:3], move))
        with PostedRun(p, checkpoint_every=1, **options) as run:
            run.wait()
            self.assertTrue(run.status()['posting']['finished'])
            record = run.posted_motion(0)
            self.assertEqual(record['motion']['id'], move.id)
            self.assertEqual(record['mapping']['source_motion_id'], move.id)
            self.assertTrue(record['stock_committed'])
            self.assertIn('X10', record['nc'][0]['text'])
            page = run.nc_lines(1, limit=256)
            self.assertEqual([item['number'] for item in page], list(range(1, len(page) + 1)))
            self.assertTrue(page[-1]['text'].startswith('M2'))
            manifest = run.posting_manifest()
            self.assertEqual(manifest['identity']['revision'], p.revision)
            self.assertEqual(manifest['definition']['machine']['name'], options['machine'].name)
            player = Player(run)
            before = player.state.heights.tobytes()
            self.assertIsNone(player.posting['context'])
            player.seek(Cursor(0, 0.5))
            self.assertEqual(player.posting['context']['mapping']['source_motion_id'], str(move.id))
            self.assertEqual(player.posting['context']['nominal_source_fraction'], 0.5)
            self.assertNotEqual(player.state.heights.tobytes(), before)
            half = player.state.heights.tobytes()
            player.seek(Cursor(1))
            player.seek(Cursor(0, 0.5))
            self.assertEqual(player.state.heights.tobytes(), half)
            player.seek(Cursor())
            self.assertEqual(player.state.heights.tobytes(), before)
            total = sum(f.stat().st_size for f in run.directory.iterdir() if f.is_file())
            self.assertEqual(total, run.status()['retained_bytes'])
            with self.assertRaises(NotReady):
                run.posted_motion(1)
            with self.assertRaises(ValueError):
                run.nc_lines(0)

    def test_lowering_source_intervals_original_hash_and_progress_are_retained(self):
        p, options = config()
        curve = Helix(p.initial_position_mm, (5, 0, 2), (0, 0, 3), 2 * 3.141592653589793)
        move = Motion(42, 'helix', 't1', curve, 1200)
        p = replace(p, events=(*p.events[:3], move))
        with PostedRun(p, lowering=LoweringOptions(), checkpoint_every=16, **options) as run:
            run.wait()
            count = run.status()['available']
            self.assertGreater(count, 64)
            first = run.posted_motion(0)
            last = run.posted_motion(count - 1)
            self.assertEqual(first['mapping']['source_motion_id'], 42)
            self.assertEqual(first['mapping']['source_event_sha256'], last['mapping']['source_event_sha256'])
            self.assertEqual(first['mapping']['source_fraction_interval'][0], 0)
            self.assertEqual(last['mapping']['source_fraction_interval'][1], 1)
            self.assertLessEqual(run.status()['posting']['max_chunk_bytes'], 512 * 1024)
            player = Player(run, compact=True)
            player.seek(Cursor(17, 0.25))
            mapping = player.posting['context']['mapping']
            a, b = mapping['source_fraction_interval']
            self.assertEqual(mapping['source_motion_id'], '42')
            self.assertAlmostEqual(player.posting['context']['nominal_source_fraction'], a + (b - a) * 0.25)
            self.assertLessEqual(mapping['deviation_bound_mm'], options['machine'].tolerance_mm)

    def test_computed_stock_and_displayed_prefixes_are_distinct(self):
        p, options = config()
        entered = threading.Event()
        release = threading.Event()

        class Held(PostedRun):
            def _append_post(self, *args):
                data = super()._append_post(*args)
                if any(c['motion'] is not None for c in data['checked']):
                    entered.set()
                    release.wait(3)
                return data

        with Held(p, **options) as run:
            try:
                self.assertTrue(entered.wait(2))
                self.assertEqual(run.status()['available'], 0)
                self.assertEqual(run.status()['posting']['checked_motions'], 1)
                with self.assertRaises(NotReady):
                    run.posted_motion(0)
                self.assertFalse(run.posted_motion(0, computed=True)['stock_committed'])
                self.assertIsNone(Player(run).posting['context'])
            finally:
                release.set()
            run.wait()
            self.assertTrue(run.posted_motion(0)['stock_committed'])

    def test_findings_at_displayed_prefix_have_nc_and_nominal_source_intervals(self):
        p, options = config(traverse_feed_mm_min=3000)
        move = Motion(42, 'clearance', 't1', Line(p.initial_position_mm, (10, 0, 3)), kind='rapid')
        p = replace(p, events=(*p.events[:2], move))
        with PostedRun(p, verification=StreamChecks(stock_contact=StockContactOptions()), **options) as run:
            run.wait()
            player = Player(run, compact=True)
            before = player.state.heights.tobytes()
            self.assertEqual(player.verification['samples'], [])
            player.seek(Cursor(1))
            sample = player.verification['samples'][0]
            self.assertEqual(sample['motion_id'], '42')
            self.assertEqual(sample['posted']['mapping']['source_motion_id'], '42')
            self.assertEqual(sample['posted']['nominal_source_fraction_interval'], sample['fraction_interval'])
            self.assertTrue(sample['posted']['mapping']['line_interval'][0] > 0)
            self.assertEqual(player.state.heights.tobytes(), before)
            self.assertEqual(run.posted_motion(0)['motion']['feed_mm_min'], 3000)
            player.seek(Cursor())
            self.assertEqual(player.verification['samples'], [])

    def test_unexamined_work_limits_keep_uncertainty_and_whole_source_scope(self):
        p, options = config(traverse_feed_mm_min=3000)
        start = (-5, 1, 6)
        end = (1, -5, 6)
        options['stock'] = StockSpec((0, 0, 0), (20, 20, 10), 0.5)
        options['machine'] = replace(
            options['machine'], tools=(replace(options['machine'].tools[0], position_after_mm=start),)
        )
        move = Motion(42, 'clearance', 't1', Line(start, end), kind='rapid')
        p = replace(
            p, initial_position_mm=start, events=(p.events[0], replace(p.events[1], position_after_mm=start), move)
        )
        with PostedRun(p, verification=StreamChecks(stock_contact=StockContactOptions(max_nodes=1)), **options) as run:
            run.wait()
            result = run.verification_status()
            self.assertEqual(result['progress']['uncertain'], 1)
            self.assertTrue(result['samples'])
            for sample in result['samples']:
                self.assertEqual(sample['status'], 'uncertain')
                self.assertEqual(sample['posted']['mapping']['source_motion_id'], '42')
                self.assertEqual(sample['posted']['nominal_source_fraction_interval'], sample['fraction_interval'])
            # The base retained schema also permits a work-limit summary without
            # a localized finding/ID. Join it to the whole move, not a fake witness.
            from unittest.mock import patch
            from camlib.playback import RecordedRun

            summary = {
                **result,
                'samples': [dict(kind='motion', completed=1, detail='Work limit; inspect full report')],
            }
            with patch.object(RecordedRun, 'verification_status', return_value=summary):
                fallback = run.verification_status()['samples'][0]
            self.assertEqual(fallback['motion_id'], '42')
            self.assertEqual(fallback['status'], 'uncertain')
            self.assertEqual(fallback['posted']['nominal_source_fraction_interval'], [0, 1])

    def test_corrupt_and_foreign_journals_do_not_publish_a_new_seek(self):
        p, options = config()
        p = replace(p, events=p.events[:5])
        with (
            PostedRun(p, run_id='same-explicit-id', **options) as one,
            PostedRun(p, run_id='same-explicit-id', **options) as two,
        ):
            one.wait()
            two.wait()
            player = Player(one)
            before = player.state.heights.tobytes()
            posting = player.posting
            for name in ('post-manifest.json', 'post-chunks.bin', 'post-chunks.idx', 'post-motions.idx'):
                path = one.directory / name
                original = path.read_bytes()
                offset = len(original) - 1
                chunk = struct.unpack_from('<Q', (one.directory / 'post-motions.idx').read_bytes())[0]
                if name == 'post-chunks.bin':
                    entry = one._post_entry(chunk)
                    offset = entry[0] + entry[1] - 1
                elif name == 'post-chunks.idx':
                    size = len(original) // one.status()['posting']['chunks']
                    offset = (chunk + 1) * size - 1
                changed = bytearray(original)
                changed[offset] ^= 1
                path.write_bytes(changed)
                try:
                    with self.subTest(file=name), self.assertRaises((ProtocolError, NotReady)):
                        player.seek(Cursor(1))
                    self.assertEqual(player.cursor, Cursor())
                    self.assertEqual(player.state.heights.tobytes(), before)
                    self.assertFalse(player.state.finished)
                    self.assertEqual(player.posting, posting)
                finally:
                    path.write_bytes(original)
            path = one.directory / 'post-chunks.bin'
            original = path.read_bytes()
            path.write_bytes((two.directory / path.name).read_bytes())
            with self.assertRaises(ProtocolError):
                player.seek(Cursor(1))
            path.write_bytes(original)
            player.seek(Cursor(1))
            self.assertTrue(player.state.finished)

    def test_cancel_closes_cooperative_source_and_retains_checked_partial_prefix(self):
        p, options = config()

        class Events:
            def __init__(self):
                self.events = iter(p.events[:5])
                self.gate = threading.Event()
                self.waiting = threading.Event()
                self.closed = False

            def __iter__(self):
                return self

            def __next__(self):
                try:
                    return next(self.events)
                except StopIteration:
                    self.waiting.set()
                    self.gate.wait(5)
                    raise StopIteration

            def cancel(self):
                self.gate.set()

            def close(self):
                self.closed = True

        events = Events()
        with PostedRun(p, events, lowering=LoweringOptions(), batch_size=1, **options) as run:
            run.wait_available(1)
            self.assertTrue(events.waiting.wait(2))
            run.cancel()
            self.assertTrue(run.wait_stopped(2))
            self.assertTrue(events.closed)
            status = run.status()
            self.assertTrue(status['cancelled'])
            self.assertFalse(status['finished'])
            self.assertFalse(status['posting']['finished'])
            self.assertEqual(status['available'], 1)
            self.assertTrue(run.posted_motion(0)['stock_committed'])
            Player(run).seek(Cursor(1))

    def test_post_failure_and_shared_quota_keep_previous_stock_code_prefix(self):
        p, options = config()

        class FailSecond(GCodePost):
            def on_motion(self, motion):
                if motion.id == 2:
                    self.emit('G1 X999 Y0 Z3 F1200')
                else:
                    super().on_motion(motion)

        moves = (replace(p.events[4], id=1), Motion(2, 'pocket', 't1', Line((10, 0, 3), (15, 0, 3)), 1200))
        p = replace(p, events=(*p.events[:3], *moves))
        with PostedRun(p, post=FailSecond, lowering=LoweringOptions(), batch_size=1, **options) as run:
            with self.assertRaisesRegex(SimulationError, 'deviation'):
                run.wait()
            self.assertEqual(run.status()['available'], 1)
            self.assertEqual(run.status()['posting']['checked_motions'], 1)
            self.assertFalse(run.status()['posting']['finished'])
            Player(run).seek(Cursor(1))
        with PostedRun(p, retained_bytes_limit=1024, **options) as run:
            with self.assertRaisesRegex(SimulationError, 'byte budget'):
                run.wait()
            self.assertEqual(run.status()['available'], 0)
            self.assertLessEqual(run.status()['retained_bytes'], 1024)

    def test_tool_pose_reset_is_the_table_premise_without_a_fabricated_sweep(self):
        # A table machine's tool change leaves the new tool at its declared
        # pose: the replay accepts that pose as the premise it is (as for
        # z_home, #65), without inserting a sweep along the unmodelled change
        # route. A plan that disagrees with the table is still refused.
        from camlib.motion import ToolChange

        def program(planned):
            p, options = config()
            second = replace(
                options['machine'].tools[0], tool='second', number=3, length_register=8, position_after_mm=(0, 0, 12)
            )
            tool = replace(p.tools[0], id='second')
            events = (
                *p.events[:3],
                p.events[4],
                Spindle(0),
                ToolChange('second', 3, 8, planned),
                Spindle(1200),
                Motion(2, 'second', 'second', Line(planned, (0, 0, 3)), 1200),
            )
            p = replace(p, tools=(*p.tools, tool), events=events)
            options['machine'] = replace(options['machine'], tools=(*options['machine'].tools, second))
            return p, options

        p, options = program((0, 0, 12))
        with PostedRun(p, lowering=LoweringOptions(), batch_size=1, **options) as run:
            run.wait()
            # Exactly the two planned motions: nothing fabricated.
            self.assertEqual(run.status()['available'], 2)
            self.assertEqual(tuple(run.posted_motion(1)['motion']['curve']['start']), (0, 0, 12))
        p, options = program((0, 0, 10))
        with PostedRun(p, lowering=LoweringOptions(), batch_size=1, **options) as run:
            with self.assertRaisesRegex(SimulationError, 'tool-change pose deviation'):
                run.wait()

    def test_native_operation_events_reuse_planner_and_cleanup_both_processes(self):
        from test_operations import small
        from test_simulation import tool
        from camlib.motion import Program
        from camlib.post import ToolSetting

        t = tool()
        header = Program('native-posted', 'source-revision', (t,), (0, 0, 12), ())
        source = Simulation(
            StockSpec((0, 0, 0), (20, 20, 10), 0.5),
            (t,),
            revision=header.revision,
            initial_position_mm=header.initial_position_mm,
            operations=(small(),),
        )
        machine = Machine(
            'Native source',
            (ToolSetting.for_tool(t, number=1, length_register=1, position_after_mm=header.initial_position_mm),),
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
        with PostedRun(
            header,
            OperationEvents(source),
            machine=machine,
            stock=StockSpec((0, 0, 0), (20, 20, 10), 0.5),
            lowering=LoweringOptions(),
        ) as run:
            run.wait()
            self.assertGreater(run.status()['available'], 10)
            self.assertTrue(run.status()['posting']['finished'])
            player = Player(run)
            player.seek(Cursor(run.status()['available']))
            self.assertGreater(player.state.volume()['removed_lower_mm3'], 100)
        self.assertIsNotNone(source._process.poll())

    def test_source_cleanup_also_runs_when_constructor_or_stock_setup_fails(self):
        p, options = config()

        class Events:
            closed = False

            def __iter__(self):
                return iter(p.events)

            def close(self):
                self.closed = True

        source = Events()
        with self.assertRaises(ValueError):
            PostedRun(p, source, retained_bytes_limit=0, **options)
        self.assertTrue(source.closed)
        source = Events()
        options['stock'] = StockSpec((0, 0, 0), (20, 20, -1), 0.5)
        with PostedRun(p, source, **options) as run:
            with self.assertRaises(SimulationError):
                run.wait()
            self.assertTrue(source.closed)
            self.assertEqual(run.status()['posting']['checked_motions'], 0)


if __name__ == '__main__':
    unittest.main()
