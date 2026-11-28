"""Independent machine-axis velocity oracles and actual source/post/stock flows."""

from camlib.postprocessor import GCodeDialect

from dataclasses import replace
import math
import unittest

from camlib.motion import Arc, Helix, Line, Motion, WorkFrame
from camlib.post import (
    Machine,
    PostEngine,
    GCodePost,
    GCodeDecoder,
    NativeGCodeDecoder,
    LoweringOptions,
    Cancellation,
    post_program,
)
from camlib.verification import MachineLimits, Scene, Verifier
from camlib.posted import PostedRun
from camlib.playback import Cursor, Player
from camlib.simulation import VerificationStream, ProtocolError
from test_post import fixture
from test_simulation import tool, moves, simulation
from test_stream_verification import checks, modified
from test_posted_playback import config


class AxisFeedTest(unittest.TestCase):
    def test_python_scene_rotates_345_line_and_reports_whole_arc_interior(self):
        p, s = fixture()
        m = Machine(
            'axis limits',
            s.tools,
            tip_travel_mm=((-100,) * 3, (100,) * 3),
            max_feed_mm_min=2000,
            max_axis_feed_mm_min=(700, 700, 700),
            dialect=GCodeDialect.linuxcnc(),
            output_units='mm',
            decimals=3,
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
        limits = m.verification_limits()
        self.assertEqual(limits.max_axis_feed_mm_min, (700, 700, 700))
        line = Motion(1, 'feed', 't1', Line((0, 0, 0), (3, 4, 0)), 1000)
        with Verifier(Scene('axis', '1', machine=limits)) as v:
            r = v.check(tool(), line)
            self.assertEqual(r['status'], 'violation')
            self.assertTrue(r['coverage']['nominal_axis_feed_checked'])
            self.assertEqual(r['findings'][0]['kind'], 'axis_feed_limit')
            self.assertIn('axis Y', r['findings'][0]['detail'])
            rapid = v.check(tool(), replace(line, kind='rapid', feed_mm_min=None))
            self.assertFalse(rapid['coverage'].get('nominal_axis_feed_checked', False))
        frame = WorkFrame(origin_mm=(10, 20, 30), x=(0, 1, 0), y=(-1, 0, 0))
        with Verifier(Scene('rotated', '1', machine=limits, work_to_machine=frame)) as v:
            self.assertIn('axis X', v.check(tool(), line)['findings'][0]['detail'])
        q = math.sqrt(2)
        arc = replace(line, curve=Arc((q, q, 0), (-q, q, 0), (0, 0, 0), math.pi / 2))
        with Verifier(Scene('arc', '1', machine=replace(limits, max_axis_feed_mm_min=(900, 900, 900)))) as v:
            self.assertEqual(v.check(tool(), arc)['status'], 'violation')
        self.assertNotIn('max_axis_feed_mm_min', replace(limits, max_axis_feed_mm_min=None).to_dict())
        with Verifier(Scene('boundary', '1', machine=replace(limits, max_axis_feed_mm_min=(600, 800, 900)))) as v:
            self.assertEqual(v.check(tool(), line)['status'], 'uncertain')

    def test_machine_limits_are_frozen_validated_and_default_identity_unchanged(self):
        p, s = fixture()
        baseline = Machine(
            'baseline',
            s.tools,
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
        a = list(post_program(p, baseline, run_id='same'))
        b = list(post_program(p, replace(baseline, max_axis_feed_mm_min=None), run_id='same'))
        self.assertEqual([c.raw for c in a], [c.raw for c in b])
        self.assertNotIn('nominal_axis_feed', a[0].data)
        values = [1500, 1500, 1500]
        machine = replace(baseline, max_axis_feed_mm_min=values)
        values[0] = 1
        self.assertEqual(machine.max_axis_feed_mm_min, (1500, 1500, 1500))
        enabled = list(post_program(p, machine, run_id='same'))
        self.assertNotEqual(a[0].data['identity'], enabled[0].data['identity'])
        for bad in (
            (0, 1000, 1000),
            (-1, 1000, 1000),
            (math.nan, 1000, 1000),
            (math.inf, 1000, 1000),
            (True, 1000, 1000),
            (1000, 1000),
            (1000,) * 4,
        ):
            with self.subTest(bad=bad):
                with self.assertRaises(ValueError):
                    replace(baseline, max_axis_feed_mm_min=bad)
                with self.assertRaises(ValueError):
                    MachineLimits((-100,) * 3, (100,) * 3, 2000, max_axis_feed_mm_min=bad)
        # A large declared limit is the machine's value, not malformed:
        # camlib states no upper bound of its own for it (#429).
        self.assertEqual(replace(baseline, max_axis_feed_mm_min=(1e8, 1000, 1000)).max_axis_feed_mm_min[0], 1e8)
        MachineLimits((-100,) * 3, (100,) * 3, 2000, max_axis_feed_mm_min=(1e8, 1000, 1000))

    def test_full_turns_all_planes_senses_pitch_units_and_decoders(self):
        p, s = fixture()
        frame = WorkFrame(origin_mm=(10, 20, 30), x=(0, 1, 0), y=(-1, 0, 0))
        for units in ('mm', 'inch'):
            for plane, (u, v, w) in {'xy': (0, 1, 2), 'xz': (2, 0, 1), 'yz': (1, 2, 0)}.items():
                for sign in (-1, 1):
                    for pitch in (0, -2.54, 2.54):
                        start = (25.4, 0, 0)
                        center = list(start)
                        center[u] -= 2.54
                        end = list(start)
                        end[w] += pitch
                        curve = (Helix if pitch else Arc)(start, tuple(end), tuple(center), sign * math.tau, plane)
                        event = Motion(1, 'curve', 't1', curve, 600)
                        job = replace(
                            p,
                            frame=frame,
                            initial_position_mm=start,
                            events=(p.events[0], replace(p.events[1], position_after_mm=start), p.events[2], event),
                        )
                        m = Machine(
                            'any mill',
                            tuple(replace(t, position_after_mm=start) for t in s.tools),
                            frame=frame,
                            work_to_machine=frame,
                            output_units=units,
                            max_axis_feed_mm_min=(700, 700, 700),
                            dialect=GCodeDialect.linuxcnc(),
                            decimals=(3 if units == 'mm' else 4),
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
                        for decoder in (GCodeDecoder, NativeGCodeDecoder):
                            output = list(post_program(job, m, decoder=decoder))
                            scope = output[-1].data['nominal_axis_feed']
                            self.assertEqual(
                                (scope['source_feed_curves_checked'], scope['decoded_feed_curves_checked']), (1, 1)
                            )
                            self.assertEqual(scope['source_rapid_speed_unknown'], 0)
                            self.assertIn('machine_axis_segment_interior', scope['scope'])
                            self.assertEqual(len([motion for c in output for motion in c.motions]), 1)

    def test_source_interior_arc_violation_cannot_be_hidden_by_lowering(self):
        p, s = fixture()
        q = math.sqrt(2)
        start = (q, q, 3)
        event = Motion(1, 'arc', 't1', Arc(start, (-q, q, 3), (0, 0, 3), math.pi / 2), 1000)
        p = replace(
            p,
            initial_position_mm=start,
            events=(p.events[0], replace(p.events[1], position_after_mm=start), p.events[2], event),
        )
        m = Machine(
            'interior extrema',
            tuple(replace(t, position_after_mm=start) for t in s.tools),
            max_axis_feed_mm_min=(900, 900, 900),
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
        for lowering in (None, LoweringOptions()):
            engine = PostEngine(m, lowering=lowering)
            engine.begin(p)
            if lowering is None:
                engine.consume(p.events[:3])
                with self.assertRaisesRegex(ValueError, 'source motion.*axis X'):
                    engine.consume((event,))
                good = engine.consume((replace(event, feed_mm_min=800),))
            else:
                for e in p.events[:3]:
                    engine.start(e)
                    engine.drain()
                with self.assertRaisesRegex(ValueError, 'source motion.*axis X'):
                    engine.start(event)
                self.assertFalse(engine.pending)
                engine.start(replace(event, feed_mm_min=800))
                while engine.pending:
                    good = engine.drain(7)
            self.assertEqual(good.data['nominal_axis_feed']['source_feed_curves_checked'], 1)
            engine.finish()

    def test_decoded_feed_is_independently_limited_atomic_and_retryable(self):
        class Faster(GCodePost):
            def __init__(self, m):
                super().__init__(m)
                self.fail = True
                self.calls = 0

            def on_motion(self, motion):
                self.calls += 1
                super().on_motion(replace(motion, feed_mm_min=motion.feed_mm_min + 0.09) if self.fail else motion)

        p, s = fixture()
        event = replace(
            p.events[4],
            feed_mm_min=600,
        )
        m = Machine(
            'decoded feed',
            s.tools,
            max_axis_feed_mm_min=(600.05, 1000, 1000),
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
        for decoder in (GCodeDecoder, NativeGCodeDecoder):
            engine = PostEngine(m, Faster, decoder=decoder)
            engine.begin(p)
            engine.consume(p.events[:3])
            with self.assertRaisesRegex(ValueError, 'decoded motion.*axis X'):
                engine.consume((event,))
            self.assertEqual(engine.position_mm, p.initial_position_mm)
            self.assertEqual(engine.post.calls, 0)
            engine.post.fail = False
            token = Cancellation()
            token.cancel()
            with self.assertRaisesRegex(ValueError, 'cancelled'):
                engine.consume((event,), cancel=token)
            good = engine.consume((event,))
            scope = good.data['nominal_axis_feed']
            self.assertEqual((scope['source_feed_curves_checked'], scope['decoded_feed_curves_checked']), (1, 1))
            self.assertEqual(engine.post.calls, 1)
            engine.finish()
        boundary = PostEngine(replace(m, max_axis_feed_mm_min=(600, 1000, 1000)))
        boundary.begin(p)
        boundary.consume(p.events[:3])
        with self.assertRaisesRegex(ValueError, 'source motion.*Boundary'):
            boundary.consume((event,))

    def test_decoded_coordinate_rounding_changes_axis_peak_and_is_rejected(self):
        p, s = fixture()
        event = Motion(1, 'rounding', 't1', Line(p.initial_position_mm, (6, 1.0004, 3)), 600)
        # Source X=600/sqrt(1+1.0004^2) ~424.179; emitted Y rounds to1,
        # making decoded X=600/sqrt(2) ~424.264 despite identical scalar F.
        machine = Machine(
            'rounded axis component',
            s.tools,
            max_axis_feed_mm_min=(424.22, 500, 500),
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
        self.assertLess(600 / math.hypot(1, 1.0004), 424.22)
        self.assertGreater(600 / math.sqrt(2), 424.22)
        for decoder in (GCodeDecoder, NativeGCodeDecoder):
            engine = PostEngine(machine, decoder=decoder)
            engine.begin(p)
            engine.consume(p.events[:3])
            with self.assertRaisesRegex(ValueError, 'decoded motion.*axis X'):
                engine.consume((event,))
            self.assertEqual(engine.position_mm, p.initial_position_mm)
            good = engine.consume((replace(event, feed_mm_min=599),))
            scope = good.data['nominal_axis_feed']
            self.assertEqual((scope['source_feed_curves_checked'], scope['decoded_feed_curves_checked']), (1, 1))
            engine.finish()

    def test_unknown_rapid_and_explicit_traverse_feed_scope(self):
        p, s = fixture()
        rapid = Motion(1, 'rapid', 't1', Line(p.initial_position_mm, (6, 0, 3)), None, 'rapid')
        p = replace(p, events=(*p.events[:3], rapid))
        for traverse in (None, 600):
            m = Machine(
                'rapid scope',
                s.tools,
                max_axis_feed_mm_min=(700, 700, 700),
                traverse_feed_mm_min=traverse,
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
            output = list(post_program(p, m))
            scope = output[-1].data['nominal_axis_feed']
            self.assertEqual(
                (scope['source_rapid_speed_unknown'], scope['decoded_rapid_speed_unknown']),
                (1, 1) if traverse is None else (0, 0),
            )
            self.assertEqual(scope['source_feed_curves_checked'], 0 if traverse is None else 1)
        with self.assertRaisesRegex(ValueError, 'source motion.*axis X'):
            list(post_program(p, replace(m, traverse_feed_mm_min=800)))

    def test_stream_scope_is_bound_and_retained_stock_removal_can_be_replayed(self):
        cfg = checks(material=False)
        cfg = replace(
            cfg,
            scene=replace(
                cfg.scene,
                machine=replace(cfg.scene.machine, max_feed_mm_min=2000, max_axis_feed_mm_min=(2000, 2000, 2000)),
            ),
        )
        with simulation(verification=cfg) as sim:
            motion, report = list(sim.motions(moves()[:1]))
            reader = VerificationStream(sim.manifest_frame)
            self.assertTrue(report.check['report']['coverage']['nominal_axis_feed_checked'])
            forged = modified(
                report.frame, lambda d: d['check']['report']['coverage'].update(nominal_axis_feed_checked=False)
            )
            with self.assertRaises(ProtocolError):
                reader.read(forged, 1, motion.motion)
            self.assertTrue(
                reader.read(report.frame, 1, motion.motion).check['report']['coverage']['nominal_axis_feed_checked']
            )
            sim.finish()
        p, options = config(max_axis_feed_mm_min=(1500, 1500, 1500))
        p = replace(p, events=(*p.events[:3], p.events[4]))
        for lowering in (None, LoweringOptions()):
            with PostedRun(p, lowering=lowering, **options) as run:
                run.wait()
                manifest = run.posting_manifest()
                self.assertEqual(manifest['nominal_axis_feed']['limits_mm_min'], [1500] * 3)
                final, _, _ = run._read_post(run.posting_status()['chunks'] - 1)
                self.assertEqual(final['nominal_axis_feed']['decoded_feed_curves_checked'], 1)
                player = Player(run)
                initial = player.state.heights.tobytes()
                player.seek(Cursor(1))
                cut = player.state.heights.tobytes()
                self.assertNotEqual(initial, cut)
                self.assertGreater(player.state.volume()['removed_lower_mm3'], 0)
                player.seek(Cursor())
                self.assertEqual(player.state.heights.tobytes(), initial)
                player.seek(Cursor(1))
                self.assertEqual(player.state.heights.tobytes(), cut)
