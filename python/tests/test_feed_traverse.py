"""A programmed feed must never weaken clearance intent or disappear in playback."""

from camlib.postprocessor import GCodeDialect

from dataclasses import asdict, replace
import json
import math
import unittest

from camlib._buffers import native
from camlib.motion import Arc, Line, Motion
from camlib.post import (
    Cancellation,
    Chunk,
    GCodeDecoder,
    GCodePost,
    LoweringOptions,
    Machine,
    NativeGCodeDecoder,
    PostEngine,
    post_events,
    post_program,
)
from camlib.playback import Cursor, Player, RecordedRun
from camlib.simulation import Simulation, VerificationUpdate
from camlib.verification import MachineLimits, Scene, Verifier
from test_post import fixture
from test_stock_contact import config, rapids


def job(**changes):
    p, s = fixture()
    moves = (
        Motion(42, 'clearance', 't1', Line(p.initial_position_mm, (10, 0, 3)), kind='rapid'),
        Motion(43, 'clearance', 't1', Line((10, 0, 3), (10, 5, 3)), 300, 'rapid'),
        Motion(44, 'clearance', 't1', Line((10, 5, 3), (15, 5, 3)), kind='rapid'),
    )
    return replace(
        p,
        events=(*p.events[:2], *moves),
    ), Machine(
        'Generic feed traverse',
        s.tools,
        **{
            'output_units': 'mm',
            **(
                {
                    'dialect': GCodeDialect.linuxcnc(),
                    'decimals': (3 if (changes).get('output_units', 'mm') == 'mm' else 4),
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
                    **changes,
                }
            ),
        },
    )


class FeedTraverseTest(unittest.TestCase):
    def test_actual_feed_units_roles_source_hashes_and_normal_python_hooks(self):
        class Observed(GCodePost):
            def on_motion(self, motion):
                self.seen = getattr(self, 'seen', []) + [(motion.feed_mm_min, self.source_event.feed_mm_min)]
                super().on_motion(motion)

        for units in ('mm', 'inch'):
            for decoder in (GCodeDecoder, NativeGCodeDecoder):
                for lowering in (None, LoweringOptions()):
                    with self.subTest(units=units, decoder=decoder, lowered=lowering is not None):
                        p, m = job(output_units=units, traverse_feed_mm_min=600)
                        engine = PostEngine(m, Observed, decoder=decoder, lowering=lowering)
                        begin = engine.begin(p, run_id='traverse')
                        chunks = list(engine.consume_events(p.events))
                        engine.finish()
                        self.assertEqual(engine.post.seen, [(600, None), (300, 300), (600, None)])
                        moves = [move for c in chunks for move in c.motions]
                        self.assertEqual([v.kind for v in moves], ['rapid'] * 3)
                        for actual, feed in zip(moves, (600, 300, 600)):
                            self.assertAlmostEqual(actual.feed_mm_min, feed, delta=m.feed_tolerance_mm_min)
                        text = ''.join(c.text for c in chunks)
                        self.assertNotIn('G0 ', text)
                        self.assertEqual(text.count('G1 '), 3)
                        # Serialized proof and public cached views retain the same actual F.
                        self.assertEqual([v for c in chunks for v in Chunk(c.raw).motions], moves)
                        mappings = [x for c in chunks for x in c.data['mappings'] if x['source_motion_id'] is not None]
                        plain = list(post_program(p, replace(m, traverse_feed_mm_min=None), lowering=lowering))
                        original_maps = [
                            x for c in plain for x in c.data['mappings'] if x['source_motion_id'] is not None
                        ]
                        self.assertEqual(
                            [x['source_event_sha256'] for x in mappings],
                            [x['source_event_sha256'] for x in original_maps],
                        )
                        self.assertTrue(all(x['deviation_bound_mm'] <= m.tolerance_mm for x in mappings))
                        self.assertNotEqual(begin.data['identity'], plain[0].data['identity'])
                        self.assertIsNone(p.events[2].feed_mm_min)

    def test_native_checker_rejects_wrong_feed_g0_and_geometry_and_rolls_back(self):
        class Changed(GCodePost):
            line = None

            def on_motion(self, motion):
                if self.line is None:
                    super().on_motion(motion)
                else:
                    self.emit(self.line)

        for lowering in (None, LoweringOptions()):
            p, m = job(traverse_feed_mm_min=600)
            e = PostEngine(m, Changed, lowering=lowering)
            e.begin(p)
            list(e.consume_events(p.events[:2]))
            if lowering:
                e.start(p.events[2])
            for line in (
                'G0 X10 Y0 Z3',
                'G1 X10 Y0 Z3 F1200',
                'G1 X10 Y0 Z3',
                'G1 X10 Y0 Z3 F0',
                'G1 X10.02 Y0 Z3 F600',
            ):
                e.post.line = line
                with self.assertRaises(ValueError):
                    e.drain() if lowering else e.consume((p.events[2],))
                self.assertEqual(e.position_mm, p.initial_position_mm)
            e.post.line = None
            actual = e.drain() if lowering else e.consume((p.events[2],))
            self.assertEqual(actual.motions[0].feed_mm_min, 600)
            self.assertEqual(actual.data['checked'][0]['event_index'], 2)
            e.finish()
        # No policy means no permission to substitute G1 for source G0.
        p, m = job()
        e = PostEngine(m, Changed)
        e.begin(p)
        e.consume(p.events[:2])
        e.post.line = 'G1 X10 Y0 Z3 F600'
        with self.assertRaisesRegex(ValueError, 'feed differs'):
            e.consume((p.events[2],))

    def test_policy_limits_are_checked_in_python_and_native_settings(self):
        p, m = job()
        engine = PostEngine(m)
        begin = engine.begin(p)
        settings = dict(
            capabilities=m.capabilities(),
            tolerance_mm=m.tolerance_mm,
            feed_tolerance_mm_min=m.feed_tolerance_mm_min,
            work_to_machine=asdict(m.work_to_machine),
            tip_travel_mm=None,
            definition={'test': 'raw native settings'},
            run_id='native-traverse',
            decoded_initial_position_mm=p.initial_position_mm,
        )
        lines = [line['text'] for line in begin.data['lines']]
        header = replace(p, events=()).to_dict()
        for feed in (0, -1, 10001):
            with self.assertRaisesRegex(ValueError, 'traverse_feed'):
                replace(m, traverse_feed_mm_min=feed)
            with self.assertRaisesRegex(ValueError, 'traverse feed'):
                native.IntentSession(
                    json.dumps(header), json.dumps(dict(settings, traverse_feed_mm_min=feed)), json.dumps(lines)
                )
        for feed in (float('nan'), float('inf')):
            with self.assertRaises(ValueError):
                replace(m, traverse_feed_mm_min=feed)
        # Source-programmed feeds cannot bypass the configured limit either.
        for feed in (0, -1, 10001):
            invalid = replace(p.events[2], feed_mm_min=feed)
            with self.assertRaises(ValueError):
                list(post_program(replace(p, events=(*p.events[:2], invalid)), m))
        curved = replace(
            p.events[2], curve=Arc(p.initial_position_mm, (0, 5, 3), (0, 0, 3), math.pi / 2), feed_mm_min=600
        )
        with self.assertRaisesRegex(ValueError, 'line'):
            list(post_program(replace(p, events=(*p.events[:2], curved)), m))

    def test_bounded_stream_and_cancellation_do_not_consume_or_commit_more(self):
        p, m = job(traverse_feed_mm_min=600)
        produced = 0
        token = Cancellation()

        def source():
            nonlocal produced
            for event in (
                *p.events[:2],
                *(
                    Motion(i + 1, 'clearance', 't1', Line((5 + i, 0, 3), (6 + i, 0, 3)), kind='rapid')
                    for i in range(200)
                ),
            ):
                produced += 1
                yield event

        stream = post_events(replace(p, events=()), source(), m, cancel=token)
        next(stream)
        self.assertEqual(produced, 0)
        chunk = next(stream)
        self.assertEqual(produced, 64)
        self.assertEqual(len(chunk.motions), 62)
        self.assertLessEqual(len(chunk.raw), 512 * 1024)
        self.assertTrue(all(v.feed_mm_min == 600 and v.kind == 'rapid' for v in chunk.motions))
        token.cancel()
        with self.assertRaisesRegex(ValueError, 'cancelled'):
            next(stream)
        self.assertEqual(produced, 64)

    def test_posted_traverse_stream_keeps_stock_and_reports_cutter_collision(self):
        p, s = fixture()
        source = rapids()
        table = replace(s.tools[0], position_after_mm=source[0].curve.start)
        header = replace(p, events=(), initial_position_mm=source[0].curve.start)
        m = Machine(
            'Traverse stock example',
            (table,),
            traverse_feed_mm_min=600,
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
        events = (*p.events[:1], replace(p.events[1], position_after_mm=header.initial_position_mm), *source)

        def decoded():
            for chunk in post_events(header, events, m):
                yield from chunk.motions

        with Simulation(**config()) as sim:
            before = sim.state.heights.tobytes()
            updates = list(sim.run(decoded()))
            reports = [u for u in updates if isinstance(u, VerificationUpdate)]
            self.assertEqual([r.status for r in reports], ['violation', 'clear', 'clear'])
            self.assertTrue(all(r.check['stock_contact']['scope'] == 'rapid_assembly' for r in reports))
            self.assertEqual(list(reports[0].findings())[0]['part'], 'cutter')
            self.assertEqual(sim.state.heights.tobytes(), before)
            self.assertEqual(sim.state.volume()['removed_upper_mm3'], 0)
            self.assertEqual(sim.state.completed, 3)

    def test_playback_uses_actual_feed_and_partial_seek_remains_stock_preserving(self):
        motions = [replace(v, feed_mm_min=600) for v in rapids()]
        with RecordedRun(**config(), motions=motions, rapid_mm_min=6000, checkpoint_every=1) as run:
            run.wait()
            self.assertEqual(run.motion_time(0), (0, 3))
            player = Player(run, compact=True)
            before = player.state.heights.tobytes()
            player.seek_seconds(1.5)
            self.assertEqual(player.cursor, Cursor(0, 0.5))
            self.assertEqual(player.state.position, (10, 5, 6))
            self.assertEqual(player.state.heights.tobytes(), before)
            player.seek(Cursor(3))
            self.assertEqual(player.verification['progress']['violation'], 1)
            player.seek(Cursor())
            self.assertEqual(player.state.heights.tobytes(), before)
        with RecordedRun(**config(), motions=rapids(), rapid_mm_min=6000) as run:
            run.wait()
            self.assertEqual(run.motion_time(0), (0, 0.3))

    def test_static_feed_limits_and_legacy_post_keep_explicit_clearance_semantics(self):
        p, s = fixture()
        motion = replace(p.events[4], kind='rapid', feed_mm_min=600)
        p = replace(p, events=(*p.events[:2], motion))
        for lowering in (None, LoweringOptions(chord_tolerance_mm=0.00025)):
            chunks = list(post_program(p, s, lowering=lowering))
            moves = [v for c in chunks for v in c.motions]
            self.assertEqual(len(moves), 1)
            self.assertEqual(moves[0].kind, 'rapid')
            self.assertEqual(moves[0].feed_mm_min, 600)
            # Y and Z stay where the tool change placed them: no words (#367).
            self.assertRegex(''.join(c.text for c in chunks), r'G1 X10(?:\.0+)? F600(?:\.0+)?\n')
        scene = Scene('traverse-limits', '1', machine=MachineLimits((-100, -100, -100), (100, 100, 100), 500))
        with Verifier(scene) as verifier:
            report = verifier.check(p.tools[0], motion)
            self.assertEqual(report['status'], 'violation')
            self.assertEqual(report['findings'][0]['kind'], 'feed_limit')
            self.assertTrue(report['coverage']['declared_fed_rate_checked'])


if __name__ == '__main__':
    unittest.main()
