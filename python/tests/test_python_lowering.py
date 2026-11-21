"""Generic Python posts use the shared native bounded curve cursor/certificates."""

from camlib.postprocessor import GCodeDialect

from dataclasses import replace
import json
import math
from pathlib import Path
import sys
import tempfile
import unittest

from camlib.motion import Motion, Line, Arc, Helix, Spindle
from camlib.post import (
    Machine,
    LoweringOptions,
    PostEngine,
    GCodePost,
    GCodeDecoder,
    Cancellation,
    post_program,
    post_events,
)
from test_post import fixture
from test_python_posts import PipePost, PipeDecoder


def source(*, turns=8):
    p, s = fixture()
    event = Motion(42, 'helix', 't1', Helix((5, 0, 3), (5, 0, -13), (0, 0, 3), turns * math.tau), 1200, 'plunge')
    p = replace(p, events=(*p.events[:3], event))
    m = Machine(
        'Python line-only machine',
        s.tools,
        arcs=False,
        helices=False,
        planes=('xy',),
        dialect=GCodeDialect.linuxcnc(),
        output_units='mm',
        decimals=3,
        max_feed_mm_min=10000,
        max_spindle_rpm=30000,
        max_sweep_rad=6.283185307179586,
        max_tool_number=99999,
        max_length_register=99999,
        tool_changes=True,
        flood=True,
        mist=True,
        tool_change_position_mode='table',
        rapid_mm_min=6000,
    )
    return p, m


class PythonLowering(unittest.TestCase):
    def test_original_curve_limit_cannot_be_evaded_by_lines_and_scope_tracks_real_segments(self):
        p, m = source(turns=2)
        limited = replace(m, max_normal_accel_mm_s2=70)
        engine = PostEngine(limited, lowering=LoweringOptions())
        engine.begin(p)
        for event in p.events[:3]:
            engine.start(event)
            engine.drain()
        with self.assertRaisesRegex(ValueError, 'source motion.*normal acceleration'):
            engine.start(p.events[-1])
        self.assertFalse(engine.pending)
        # Same id, explicitly revised feed can retry after failed start.
        engine.start(replace(p.events[-1], feed_mm_min=1000))
        output = []
        token = Cancellation()
        token.cancel()
        with self.assertRaisesRegex(ValueError, 'cancelled'):
            engine.drain(cancel=token)
        while engine.pending:
            output.append(engine.drain(7))
        end = engine.finish()
        scope = end.data['nominal_normal_acceleration']
        self.assertEqual(scope['source_feed_curves_checked'], 1)
        self.assertEqual(scope['decoded_feed_curves_checked'], sum(len(c.motions) for c in output))
        self.assertTrue(all(isinstance(move.curve, Line) for c in output for move in c.motions))
        self.assertTrue(all(len(c.raw) <= 512 * 1024 for c in output))
        self.assertEqual(end.data['progress']['source_events_completed'], len(p.events))

    def begin_motion(self, post=GCodePost, **settings):
        p, m = source()
        engine = PostEngine(m, post, lowering=LoweringOptions(), **settings)
        engine.begin(p, run_id='bounded-run')
        for event in p.events[:3]:
            engine.start(event)
            engine.drain()
        engine.start(p.events[-1])
        return p, engine

    def test_normal_python_hooks_receive_original_context_and_stream_checked_source_maps(self):
        class UserPost(GCodePost):
            def __init__(self, m):
                super().__init__(m)
                self.segments = 0

            def on_motion(self, motion):
                assert isinstance(motion.curve, Line)
                assert isinstance(self.source_event.curve, Helix)
                assert self.source_event.id == 42
                assert 0 <= self.source_fraction[0] < self.source_fraction[1] <= 1
                self.segments += 1
                self.comment('original helix')
                super().on_motion(motion)

        p, e = self.begin_motion(UserPost)
        first = e.drain()
        self.assertEqual(len(first.motions), 64)
        self.assertTrue(e.pending)
        chunks = [first]
        while e.pending:
            chunks.append(e.drain())
        end = e.finish()
        self.assertEqual(end.data['progress']['source_events_completed'], len(p.events))
        self.assertEqual(first.data['identity']['abi'], 6)
        self.assertEqual(first.data['scope'], 'python_declared_decoder_lowered_nominal_intent_only')
        fraction = 0.0
        posted = 0
        hashes = set()
        for chunk in chunks:
            self.assertLessEqual(len(chunk.raw), 512 * 1024)
            self.assertLessEqual(len(chunk.motions), 64)
            for mapping, checked in zip(chunk.data['mappings'], chunk.data['checked']):
                posted += 1
                self.assertEqual(mapping['source_motion_id'], 42)
                self.assertEqual(mapping['posted_motion_id'], posted)
                self.assertEqual(checked['motion']['id'], posted)
                self.assertEqual(mapping['source_fraction_interval'][0], fraction)
                fraction = mapping['source_fraction_interval'][1]
                self.assertLessEqual(checked['deviation_bound_mm'], e.machine.tolerance_mm)
                hashes.add(mapping['source_event_sha256'])
            self.assertTrue(all(line['motion_id'] == 42 for line in chunk.data['lines']))
        self.assertEqual(len(hashes), 1)
        self.assertEqual(len(hashes.pop()), 64)
        self.assertEqual(fraction, 1.0)
        self.assertEqual(e.post.segments, posted)
        self.assertEqual(end.data['progress']['posted_motions'], posted)

    def test_all_planes_senses_full_circles_multiturn_helices_and_custom_decoder(self):
        p, m = source()
        for plane, (u, v, a) in {'xy': (0, 1, 2), 'xz': (2, 0, 1), 'yz': (1, 2, 0)}.items():
            for sign in (1, -1):
                for turns in (1, 2):
                    start = p.initial_position_mm
                    center = list(start)
                    center[u] -= 5
                    end = list(start)
                    end[a] -= 2 if turns == 2 else 0
                    curve = (Helix if turns == 2 else Arc)(
                        start, tuple(end), tuple(center), sign * turns * math.tau, plane
                    )
                    event = Motion(42, 'curve', 't1', curve, 1200)
                    job = replace(p, events=(*p.events[:3], event))
                    for post, decoder in ((GCodePost, GCodeDecoder), (PipePost, PipeDecoder)):
                        chunks = list(post_program(job, m, post=post, decoder=decoder, lowering=LoweringOptions()))
                        moves = [move for c in chunks for move in c.motions]
                        self.assertGreater(len(moves), 64)
                        self.assertTrue(all(isinstance(move.curve, Line) for move in moves))
                        self.assertLessEqual(math.dist(moves[-1].curve.end, end), m.tolerance_mm)
                        self.assertEqual(chunks[-1].data['progress']['source_events_completed'], len(job.events))
                        self.assertEqual(moves[-1].curve.end[u], start[u])
                        self.assertEqual(moves[-1].curve.end[v], start[v])

    def test_preview_caps_pending_lifecycle_and_modified_intervals_are_atomic(self):
        p, e = self.begin_motion()
        preview = json.loads(e._session.preview(7))
        self.assertEqual(preview, json.loads(e._session.preview(7)))
        with self.assertRaises(ValueError):
            e.start(Spindle(0))
        with self.assertRaises(ValueError):
            e.finish()
        with self.assertRaises(ValueError):
            e.consume((Spindle(0),))
        for n in (0, 65):
            with self.assertRaises(ValueError):
                e.drain(n)
        events = [i['event'] for i in preview]
        ranges = [list(i['source_fraction_interval']) for i in preview]
        _, _, blocks, actual, _, _ = e._format(events, [p.events[-1]] * 7, ranges, None)
        ranges[0][1] += 1e-5
        with self.assertRaisesRegex(ValueError, 'interval'):
            e._session.check_lowered(json.dumps(ranges), json.dumps(actual), json.dumps(blocks))
        self.assertEqual(json.loads(e._session.preview(7)), preview)
        first = e.drain(7)
        self.assertEqual(first.motions[0].id, 1)
        self.assertEqual(first.data['mappings'][0]['source_fraction_interval'][0], 0.0)
        m = e.machine
        capped = PostEngine(m, lowering=LoweringOptions(max_segments_per_source=4))
        capped.begin(p)
        for event in p.events[:3]:
            capped.start(event)
            capped.drain()
        with self.assertRaisesRegex(ValueError, 'budget'):
            capped.start(p.events[-1])
        self.assertFalse(capped.pending)
        # Failed start did not consume the original source ID or state.
        capped.start(replace(p.events[-1], curve=Line(p.initial_position_mm, (6, 0, 3))))
        self.assertEqual(capped.drain().data['checked'][0]['event_index'], 3)
        capped.finish()

    def test_bad_output_cancelled_hook_and_retry_preserve_every_state(self):
        token = Cancellation()

        class Bad(GCodePost):
            def __init__(self, m):
                super().__init__(m)
                self.calls = 0
                self.mode = 'bad'

            def on_motion(self, event):
                self.calls += 1
                if self.mode == 'bad':
                    self.emit('G1 X99 Y0 Z0 F1200')
                else:
                    if self.mode == 'cancel':
                        token.cancel()
                    super().on_motion(event)

        _, e = self.begin_motion(Bad)
        preview = e._session.preview()
        with self.assertRaisesRegex(ValueError, 'deviation'):
            e.drain()
        self.assertEqual(e.post.calls, 0)
        self.assertEqual(e._session.preview(), preview)
        e.post.mode = 'cancel'
        with self.assertRaisesRegex(ValueError, 'cancelled'):
            e.drain(cancel=token)
        self.assertEqual(e.post.calls, 0)
        self.assertEqual(e._session.preview(), preview)
        e.post.mode = 'ok'
        first = e.drain()
        self.assertEqual(first.motions[0].id, 1)
        self.assertEqual(e.post.calls, 64)
        self.assertGreater(first.data['progress']['pending_source_fraction'], 0)

    def test_run_policy_identity_source_hash_and_total_budget_are_independent(self):
        p, m = source()
        options = LoweringOptions()

        def begin(run, policy):
            engine = PostEngine(m, lowering=policy)
            return engine, engine.begin(p, run_id=run)

        e, a = begin('run-a', options)
        _, b = begin('run-b', options)
        _, c = begin('run-a', replace(options, chord_tolerance_mm=0.00125))
        self.assertNotEqual(a.data['identity'], b.data['identity'])
        self.assertNotEqual(a.data['identity'], c.data['identity'])
        hashes = []
        for motion_id in (42, 43):
            engine, _ = begin('hash-run', options)
            for event in p.events[:3]:
                engine.start(event)
                engine.drain()
            engine.start(replace(p.events[-1], id=motion_id))
            hashes.append(engine.drain(1).data['mappings'][0]['source_event_sha256'])
        self.assertNotEqual(*hashes)
        capped, _ = begin('capped', replace(options, max_total_segments=4))
        for event in p.events[:3]:
            capped.start(event)
            capped.drain()
        with self.assertRaisesRegex(ValueError, 'job segment'):
            capped.start(p.events[-1])
        self.assertFalse(capped.pending)
        with self.assertRaisesRegex(ValueError, 'chord_tolerance'):
            PostEngine(replace(m, tolerance_mm=0.001, decimals=5), lowering=options).begin(p)

    def test_iterator_does_not_pull_next_source_until_current_curve_is_drained(self):
        p, m = source()
        seen = []

        def events():
            for e in p.events:
                seen.append(e)
                yield e
            raise AssertionError('pulled after cancellation')

        token = Cancellation()
        stream = post_events(p, events(), m, lowering=LoweringOptions(), cancel=token)
        self.assertEqual(next(stream).data['lowering_policy']['chord_tolerance_mm'], 0.0025)
        self.assertFalse(seen)
        for _ in range(4):
            chunk = next(stream)
        self.assertEqual(len(seen), 4)
        self.assertEqual(len(chunk.motions), 64)
        self.assertLess(chunk.data['progress']['pending_source_fraction'], 1.0)
        token.cancel()
        with self.assertRaisesRegex(ValueError, 'cancelled'):
            next(stream)
        self.assertEqual(len(seen), 4)

    def test_full_pocket_at_practical_defaults_retains_real_findings_and_allowance(self):
        examples = str(Path(__file__).resolve().parents[2] / 'examples')
        sys.path.insert(0, examples)
        try:
            from support.post_pocket import run
        finally:
            sys.path.remove(examples)
        with tempfile.TemporaryDirectory() as directory:
            raw = run(Path(directory) / 'raw.nc', profile='python', lowering=True)
            reserved = run(Path(directory) / 'reserved.nc', profile='python', lowering=True, radial_allowance_mm=0.01)
        self.assertEqual(raw['source_motions'], 352)
        self.assertGreater(raw['posted_scene_stock_counts']['violation'], 0)
        self.assertIsNotNone(raw['issue_samples'][0]['source_mapping'])
        self.assertEqual(reserved['source_motions'], 352)
        self.assertEqual(
            reserved['posted_scene_stock_counts'], dict(clear=reserved['motions'], violation=0, uncertain=0)
        )
        self.assertLessEqual(reserved['max_continuous_deviation_bound_mm'], 0.01)
        self.assertTrue(reserved['posted_material']['complete'])
        self.assertFalse(reserved['first_posted_motion']['source_native_exited'])


if __name__ == '__main__':
    unittest.main()
