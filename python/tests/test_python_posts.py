from camlib.postprocessor import GCodeDialect
from dataclasses import replace
import itertools
import math
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

from camlib.motion import WorkOffset, ToolChange, Spindle, Motion, Line, Arc, Helix, WorkFrame
from camlib.post import (
    Machine,
    GCodePost,
    GCodeDecoder,
    NativeGCodeDecoder,
    PostEngine,
    ToolSetting,
    post_program,
    post_events,
    Cancellation,
)
from test_post import fixture


class ShopPost(GCodePost):
    version = 'shop-1'

    def on_motion(self, motion):
        self.comment('shop operation ' + motion.operation)
        super().on_motion(motion)


class PipePost(GCodePost):
    def on_motion(self, motion):
        self.emit('FEED|' + '|'.join(self.number(v) for v in (*motion.curve.end, motion.feed_mm_min)))


class PipeDecoder(GCodeDecoder):
    """Separate toy language decoder; reads coordinates from emitted text only."""

    version = 'pipe-demo-1'

    def read_line(self, line, context=None):
        if not line.startswith('FEED|'):
            return super().read_line(line, context)
        self.validate_initialized()
        if context is None or self.active is None or self.length is None:
            raise ValueError('pipe motion state')
        x, y, z, f = (float(v) for v in line.split('|')[1:])
        end = (x, y, z)
        motion = Motion(
            context['id'], context['operation'], self.active.tool, Line(self.position, end), f, context['kind']
        )
        self.position = end
        return [motion]


class PythonPosts(unittest.TestCase):
    def test_finish_hook_sees_decoded_terminal_position_and_failed_finish_rolls_back(self):
        from camlib.post import LoweringOptions

        p, settings = fixture()
        end = (10.0004, 0, 3)
        p = replace(
            p, events=(*p.events[:4], Motion(1, 'finish-position', 't1', Line(p.initial_position_mm, end), 600))
        )

        class CoordinateFinish(GCodePost):
            def __init__(self, machine):
                super().__init__(machine)
                self.observed = []
                self.fail = True

            def on_finish(self):
                self.observed.append(tuple(self.position_mm))
                if self.fail:
                    self.emit('G1234')
                else:
                    self.comment('terminal ' + ','.join(str(v) for v in self.position_mm))
                    super().on_finish()

        for decoder in (GCodeDecoder, NativeGCodeDecoder):
            for lowering in (None, LoweringOptions()):
                engine = PostEngine(
                    Machine(
                        'Coordinate finish',
                        settings.tools,
                        decimals=3,
                        dialect=GCodeDialect.linuxcnc(),
                        output_units='mm',
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
                    ),
                    CoordinateFinish,
                    decoder=decoder,
                    lowering=lowering,
                )
                engine.begin(p)
                list(engine.consume_events(p.events))
                expected = tuple(engine.decoder.position)
                self.assertEqual(expected, (10.0, 0.0, 3.0))
                self.assertNotEqual(expected, end)
                with self.assertRaises(ValueError):
                    engine.finish()
                self.assertEqual(engine.post.observed, [])
                self.assertFalse(engine.finished)
                self.assertEqual(tuple(engine.decoder.position), expected)
                token = Cancellation()
                token.cancel()
                with self.assertRaisesRegex(ValueError, 'cancelled'):
                    engine.finish(cancel=token)
                self.assertEqual(engine.post.observed, [])
                engine.post.fail = False
                result = engine.finish()
                self.assertEqual(engine.post.observed, [expected])
                self.assertIn('terminal 10.0,0.0,3.0', result.text)

    def test_precision_collapsed_lines_keep_checked_source_maps_without_fake_sweeps(self):
        from camlib.post import LoweringOptions

        p, settings = fixture()
        start = p.initial_position_mm
        tiny = (start[0] + 0.0001, start[1], start[2])
        end = (start[0] + 1, start[1], start[2])
        job = replace(
            p,
            events=(
                *p.events[:4],
                Motion(1, 'micro-link', 't1', Line(start, tiny), 600),
                Motion(2, 'continue', 't1', Line(tiny, end), 600),
            ),
        )
        machine = Machine(
            'rounded coordinates',
            settings.tools,
            decimals=3,
            dialect=GCodeDialect.linuxcnc(),
            output_units='mm',
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
            for lowering in (None, LoweringOptions()):
                chunks = list(post_program(job, machine, decoder=decoder, lowering=lowering))
                moves = [m for c in chunks for m in c.motions]
                self.assertEqual(len(moves), 1)
                self.assertEqual(tuple(moves[0].curve.start), start)
                self.assertEqual(tuple(moves[0].curve.end), end)
                maps = [m for c in chunks for m in c.data['mappings'] if m['source_motion_id'] == 1]
                self.assertEqual(len(maps), 1)
                self.assertIsNone(maps[0]['posted_motion_id'])
                self.assertAlmostEqual(maps[0]['deviation_bound_mm'], 0.0001)
                self.assertEqual(maps[0]['source_fraction_interval'], [0, 1])

                class Suppress(GCodePost):
                    def on_motion(self, motion):
                        self.emit('G1 X5 Y0 Z3 F600')

                with self.assertRaisesRegex(ValueError, 'stationary output'):
                    list(post_program(job, machine, post=Suppress, decoder=decoder, lowering=lowering))

    def test_normal_acceleration_policy_checks_both_decoded_units_planes_senses_and_pitch(self):
        p, s = fixture()
        frame = WorkFrame(origin_mm=(10, 20, 30), x=(0, 1, 0), y=(-1, 0, 0))
        for units in ('mm', 'inch'):
            for plane, (u, v, w) in {'xy': (0, 1, 2), 'xz': (2, 0, 1), 'yz': (1, 2, 0)}.items():
                for sign in (-1, 1):
                    for helical in (False, True):
                        # Exact mm/inch-grid closure preserves the existing
                        # full-turn intent guard; do not widen its tolerance.
                        start = (25.4, 0, 0)
                        center = list(start)
                        center[u] -= 2.54
                        end = list(start)
                        if helical:
                            end[w] += 2.54
                        curve = (Helix if helical else Arc)(start, tuple(end), tuple(center), sign * math.tau, plane)
                        event = Motion(1, 'curve', 't1', curve, 600)
                        controls = (p.events[0], replace(p.events[1], position_after_mm=start), p.events[2])
                        job = replace(p, frame=frame, initial_position_mm=start, events=(*controls, event))
                        settings = tuple(
                            replace(
                                t,
                                position_after_mm=start,
                            )
                            for t in s.tools
                        )
                        machine = Machine(
                            'plain curve-limit machine',
                            settings,
                            frame=frame,
                            work_to_machine=frame,
                            output_units=units,
                            max_normal_accel_mm_s2=80,
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
                            chunks = list(post_program(job, machine, post=ShopPost, decoder=decoder))
                            checks = chunks[-1].data['nominal_normal_acceleration']
                            self.assertEqual(checks['limit_mm_s2'], 80)
                            self.assertEqual(
                                (checks['source_feed_curves_checked'], checks['decoded_feed_curves_checked']), (1, 1)
                            )
                            self.assertEqual(checks['source_rapid_speed_unknown'], 0)
                            self.assertIn('nominal_analytic_segment_interior', checks['scope'])
                            self.assertEqual(len([m for c in chunks for m in c.motions]), 1)
        baseline = Machine(
            'no curve limit',
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
        b = list(post_program(p, replace(baseline, max_normal_accel_mm_s2=None), run_id='same'))
        self.assertEqual([c.raw for c in a], [c.raw for c in b])
        self.assertNotIn('nominal_normal_acceleration', a[0].data)
        enabled = list(post_program(p, replace(baseline, max_normal_accel_mm_s2=80), run_id='same'))
        self.assertNotEqual(a[0].data['identity'], enabled[0].data['identity'])

    def test_decoded_radius_and_feed_rounding_limits_are_atomic_and_retryable(self):
        p, s = fixture()
        source = Motion(
            1, 'rounding', 't1', Arc(p.initial_position_mm, (2.9996, 2.0004, 3), (2.9996, 0, 3), math.pi / 2), 600
        )
        machine = Machine(
            'rounded-radius',
            s.tools,
            max_normal_accel_mm_s2=49.995,
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
            list(
                post_program(
                    replace(p, events=(*p.events[:3], source)),
                    replace(machine, max_normal_accel_mm_s2=None),
                    decoder=decoder,
                )
            )
            engine = PostEngine(machine, decoder=decoder)
            engine.begin(p)
            engine.consume(p.events[:3])
            with self.assertRaisesRegex(ValueError, 'decoded motion.*normal acceleration'):
                engine.consume((source,))
            self.assertEqual(engine.position_mm, p.initial_position_mm)
            # Retry the same id after lowering AUTHORED feed explicitly; no
            # machine or post tolerance is raised to force acceptance.
            good = engine.consume((replace(source, feed_mm_min=599),))
            self.assertEqual(good.data['checked'][0]['event_index'], 3)
            self.assertEqual(good.data['nominal_normal_acceleration']['source_feed_curves_checked'], 1)
            engine.finish()

        class SlightlyFaster(GCodePost):
            def __init__(self, m):
                super().__init__(m)
                self.fail = True
                self.calls = 0

            def on_motion(self, motion):
                self.calls += 1
                super().on_motion(replace(motion, feed_mm_min=motion.feed_mm_min + 0.09) if self.fail else motion)

        event = replace(source, curve=Arc(p.initial_position_mm, (2.999, 2.001, 3), (2.999, 0, 3), math.pi / 2))
        engine = PostEngine(replace(machine, max_normal_accel_mm_s2=49.985), SlightlyFaster)
        engine.begin(p)
        engine.consume(p.events[:3])
        with self.assertRaisesRegex(ValueError, 'decoded motion.*normal acceleration'):
            engine.consume((event,))
        self.assertEqual(engine.post.calls, 0)
        self.assertEqual(engine.position_mm, p.initial_position_mm)
        engine.post.fail = False
        chunk = engine.consume((event,))
        self.assertEqual(engine.post.calls, 1)
        self.assertEqual(chunk.data['nominal_normal_acceleration']['decoded_feed_curves_checked'], 1)
        engine.finish()

    def test_normal_acceleration_boundary_unknown_rapid_and_configured_traverse_scope(self):
        p, s = fixture()
        event = Motion(1, 'curve', 't1', Arc(p.initial_position_mm, (1, 0, 3), (3, 0, 3), math.pi), 600)
        machine = Machine(
            'boundary',
            s.tools,
            max_normal_accel_mm_s2=50,
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
        engine = PostEngine(machine)
        engine.begin(p)
        engine.consume(p.events[:3])
        with self.assertRaisesRegex(ValueError, 'source motion.*Boundary'):
            engine.consume((event,))
        token = Cancellation()
        token.cancel()
        with self.assertRaisesRegex(ValueError, 'cancelled'):
            engine.consume((replace(event, feed_mm_min=599),), cancel=token)
        chunk = engine.consume((replace(event, feed_mm_min=599),))
        self.assertEqual(chunk.data['nominal_normal_acceleration']['source_feed_curves_checked'], 1)
        engine.finish()
        rapid = Motion(1, 'travel', 't1', Line(p.initial_position_mm, (6, 0, 3)), None, 'rapid')
        job = replace(p, events=(*p.events[:3], rapid))
        for traverse in (None, 600):
            chunks = list(post_program(job, replace(machine, traverse_feed_mm_min=traverse)))
            scope = chunks[-1].data['nominal_normal_acceleration']
            self.assertEqual(
                (scope['source_rapid_speed_unknown'], scope['decoded_rapid_speed_unknown']),
                (1, 1) if traverse is None else (0, 0),
            )
            self.assertEqual(scope['source_feed_curves_checked'], 0 if traverse is None else 1)

    def machine(self, **changes):
        p, s = fixture()
        return p, Machine(
            'Any shop machine - no registration',
            s.tools,
            **{
                'dialect': GCodeDialect.linuxcnc(),
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
                'output_units': (changes).get('output_units', 'mm'),
                'decimals': 3 if (changes).get('output_units', 'mm') == 'mm' else 4,
                **(changes),
            },
        )

    def test_declared_tip_preserving_tool_change_keeps_rounded_pose_and_checks_its_bound(self):
        p, m = self.machine(tool_change_position_mode='preserve_tip')
        end = (10.0004, 0.0004, 3.0004)
        first = replace(p.events[4], curve=Line(p.initial_position_mm, end))
        change = ToolChange('t1', 2, 7, end)
        following = Motion(2, 'after-change', 't1', Line(end, (12, 0, 3)), 1200)
        job = replace(p, events=(*p.events[:4], first, Spindle(0), change, Spindle(12000), following))
        for decoder in (GCodeDecoder, NativeGCodeDecoder):
            chunks = list(post_program(job, m, decoder=decoder))
            checks = [c for chunk in chunks for c in chunk.data['checked']]
            control = next(c for c in checks if c['event_index'] == 6)
            self.assertIsNone(control['motion'])
            self.assertAlmostEqual(control['deviation_bound_mm'], math.sqrt(3) * 0.0004, places=11)
            self.assertEqual(tuple(control['control_effects'][0]['position_after_mm']), (10, 0, 3))
            with self.assertRaisesRegex(ValueError, 'tool-change pose deviation'):
                list(
                    post_program(
                        job,
                        replace(m, tool_change_position_mode='table'),
                        decoder=decoder,
                    )
                )
        with self.assertRaisesRegex(ValueError, 'position_mode'):
            replace(m, tool_change_position_mode='guess')

    def test_practical_accuracy_defaults_and_explicit_precision_overrides(self):
        p, m = self.machine()
        self.assertEqual((m.tolerance_mm, m.decimals, m.feed_tolerance_mm_min), (0.01, 3, 0.1))
        self.assertEqual(self.machine(output_units='inch')[1].decimals, 4)
        event = replace(
            p.events[4], curve=Line(p.initial_position_mm, (10.0004, 0.0004, 3.0004)), feed_mm_min=1200.01234
        )
        job = replace(p, events=(*p.events[:4], event))
        chunk = list(post_program(job, m))[1]
        self.assertIn('X10 Y0 Z3 F1200.012', chunk.text)
        self.assertLess(chunk.data['checked'][-1]['deviation_bound_mm'], 0.01)
        precise = replace(m, tolerance_mm=0.00001, decimals=6, feed_tolerance_mm_min=0.0001)
        tight = list(post_program(job, precise))[1]
        self.assertEqual(tight.motions[0].curve.end, event.curve.end)
        self.assertLess(tight.data['checked'][-1]['deviation_bound_mm'], 0.00001)
        with self.assertRaisesRegex(ValueError, 'rounding'):
            replace(m, tolerance_mm=0.00001)
        for value in (0, float('nan'), -1):
            with self.assertRaises(ValueError):
                replace(m, tolerance_mm=value)

        # A larger default remains an enforced budget, not permission to move freely.
        class Overshoot(GCodePost):
            def on_motion(self, motion):
                self.emit('G1 X10.02 Y0 Z3 F1200')

        with self.assertRaisesRegex(ValueError, 'deviation'):
            list(post_program(p, m, post=Overshoot))
        # Ordinary rounded IJK circles need not have identical binary64 radii.
        start = p.initial_position_mm
        center = (0.12345, 0.23456, 3)
        dx, dy = start[0] - center[0], start[1] - center[1]
        end = (center[0] - dy, center[1] + dx, 3)
        arc = Motion(1, 'rounded-circle', 't1', Arc(start, end, center, math.pi / 2), 1200)
        chunks = list(post_program(replace(p, events=(*p.events[:3], arc)), m))
        self.assertLess(chunks[1].data['checked'][-1]['deviation_bound_mm'], 0.01)

    def test_custom_numeric_formatters_keep_control_of_circular_words(self):
        p, m = self.machine()
        data = json.loads((Path(__file__).resolve().parents[2] / 'fixtures/motion/rounded-short-arc.json').read_text())
        arc = Arc(tuple(data['start']), tuple(data['end']), tuple(data['center']), data['sweep_rad'])
        job = replace(
            p,
            events=(
                *p.events[:3],
                Motion(1, 'approach', 't1', Line(p.initial_position_mm, arc.start), 300),
                Motion(2, 'rounding', 't1', arc, 150),
            ),
        )

        class Numbers(GCodePost):
            def number(self, value, *, dimensional=True):
                return f'{value:.6f}'

        class Words(GCodePost):
            def word(self, address, value, *, dimensional=True):
                return address + f'{value:.6f}'

        for post in (Numbers, Words):
            for decoder in (GCodeDecoder, NativeGCodeDecoder):
                chunks = list(post_program(job, m, post=post, decoder=decoder))
                text = ''.join(c.text for c in chunks)
                self.assertIn('J-0.037530', text)
                self.assertIn('X9.197757', text)

    def test_machine_and_normal_python_subclass_through_public_iterator(self):
        p, m = self.machine()
        chunks = list(post_program(p, machine=m, post=ShopPost, run_id='user-run'))
        self.assertIn('; shop operation pocket', chunks[1].text)
        self.assertEqual(chunks[1].motions, (p.events[4],))
        self.assertEqual(chunks[0].data['identity']['abi'], 5)
        self.assertEqual(chunks[0].data['scope'], 'python_declared_decoder_nominal_intent_only')
        lines = [item for c in chunks for item in c.data['lines']]
        self.assertEqual([item['number'] for item in lines], list(range(1, len(lines) + 1)))
        self.assertTrue(
            all(
                item['motion_id'] == 1 and item['operation'] == 'pocket'
                for item in lines
                if item['motion_id'] is not None
            )
        )
        self.assertTrue(chunks[-1].text.endswith('M2\n'))
        changed_data = chunks[1].data
        changed_data['checked'][4]['motion']['curve']['end'][0] = 99
        self.assertEqual(chunks[1].motions[0].curve.end, (10, 0, 3))
        changed = list(post_program(p, replace(m, revision='2'), run_id='user-run'))
        self.assertNotEqual(changed[0].data['identity'], chunks[0].data['identity'])

    def test_new_dialect_units_symbolic_commands_direct_t_and_per_tool_d(self):
        p, m = self.machine()
        d = replace(
            m.dialect,
            length_word='D',
            tool_change='direct_t',
            length_activation='tool_edge',
            parentheses_comments=False,
            fixed_modes=('center_incremental', 'rpm_mode', 'cycles_off'),
        ).with_commands(
            units_mm='G710',
            units_inch='G700',
            center_incremental=None,
            rpm_mode=None,
            cycles_off=None,
            length_on=None,
            length_off='D0',
            frame_reset='TRANS',
            exact_stop='G60',
            language_native='G290',
        )
        # Construct replacement together: D0 is validated against length_word.
        m = replace(m, dialect=d, planes=('xy',))
        chunks = list(post_program(p, m))
        text = ''.join(c.text for c in chunks)
        self.assertIn('G710\n', text)
        self.assertIn('TRANS\n', text)
        self.assertIn('T2\nD7\n', text)
        self.assertNotIn('G43', text)
        self.assertNotIn('G91.1', text)
        self.assertEqual(chunks[1].motions[0].curve, p.events[4].curve)

    def test_non_gcode_motion_decoder_is_plain_python_and_still_geometrically_checked(self):
        p, m = self.machine()
        chunks = list(post_program(p, m, post=PipePost, decoder=PipeDecoder))
        self.assertIn('FEED|10|0|3|1200', chunks[1].text)
        self.assertEqual(chunks[1].motions, (p.events[4],))

        class WrongPipe(PipePost):
            def on_motion(self, motion):
                self.emit('FEED|99|0|3|1200')

        with self.assertRaisesRegex(ValueError, 'deviation'):
            list(post_program(p, m, post=WrongPipe, decoder=PipeDecoder))

    def test_machine_tool_library_can_exceed_job_and_edge_numbers_are_per_tool(self):
        p, m = self.machine()
        second = replace(p.tools[0], id='second')
        extra = ToolSetting.for_tool(second, number=3, length_register=8, position_after_mm=p.initial_position_mm)
        list(post_program(p, replace(m, tools=(*m.tools, extra))))
        d = replace(m.dialect, length_word='D', length_activation='tool_edge').with_commands(
            length_off='D0', length_on=None
        )
        m = replace(m, dialect=d, tools=(replace(m.tools[0], length_register=1), replace(extra, length_register=1)))
        events = (
            WorkOffset(1),
            ToolChange('t1', 2, 1, p.initial_position_mm),
            Spindle(1000),
            p.events[4],
            Spindle(0),
            ToolChange('second', 3, 1, p.initial_position_mm),
            Spindle(1000),
            Motion(2, 'second', 'second', Line(p.initial_position_mm, (10, 0, 3)), 1200),
        )
        chunks = list(post_program(replace(p, tools=(*p.tools, second), events=events), m))
        self.assertEqual([v.tool for v in chunks[1].motions], ['t1', 'second'])
        self.assertEqual(chunks[1].text.count('D1'), 2)

    def test_hook_and_decoder_state_roll_back_on_bad_words_and_native_geometry_failure(self):
        p, m = self.machine()

        class Wrong(GCodePost):
            def __init__(self, machine):
                super().__init__(machine)
                self.calls = 0
                self.fail = True

            def on_motion(self, motion):
                self.calls += 1
                if self.fail:
                    self.emit('G1 X11 Y0 Z3 F1200')
                else:
                    super().on_motion(motion)

        e = PostEngine(m, Wrong)
        e.begin(p)
        with self.assertRaisesRegex(ValueError, 'deviation'):
            e.consume(p.events)
        self.assertEqual(e.post.calls, 0)
        self.assertEqual(e.position_mm, p.initial_position_mm)
        e.post.fail = False
        chunk = e.consume(p.events)
        self.assertEqual(e.post.calls, 1)
        self.assertEqual(chunk.data['checked'][0]['event_index'], 0)
        e.finish()

    def test_unknown_commands_embedded_lines_missing_initialization_and_early_end_reject(self):
        p, m = self.machine()

        class Missing(GCodePost):
            def on_begin(self, program):
                self.emit('G21 G90 G17')

        with self.assertRaisesRegex(ValueError, 'initialization'):
            list(post_program(p, m, post=Missing))
        for bad in ('M123', 'G1 X10 Y0 Z3 F1200\nM3', 'M2', 'G1 G20 X10 Y0 Z3 F1200'):

            class Bad(GCodePost):
                def on_motion(self, motion):
                    self.emit(bad)

            with self.assertRaises(ValueError, msg=bad):
                list(post_program(p, m, post=Bad))

        class EndBegin(GCodePost):
            def on_begin(self, program):
                super().on_begin(program)
                self.emit('M2')

        with self.assertRaises(ValueError):
            list(post_program(p, m, post=EndBegin))

    def test_custom_addresses_and_inches_are_decoded_without_brand_names(self):
        p, m = self.machine(output_units='inch', decimals=8)
        m = replace(m, dialect=replace(m.dialect, axes=('U', 'V', 'W'), centers=('A', 'B', 'C')))
        chunks = list(post_program(p, m))
        self.assertIn('U', chunks[1].text)
        self.assertNotIn(' X', chunks[1].text)
        self.assertLess(math.dist(chunks[1].motions[0].curve.end, p.events[4].curve.end), 0.001)
        self.assertIn('G20\n', chunks[0].text)

    def test_all_planes_senses_full_circles_and_helices_use_native_continuous_checks(self):
        p, m = self.machine()
        for plane, (u, v, a) in {'xy': (0, 1, 2), 'xz': (2, 0, 1), 'yz': (1, 2, 0)}.items():
            for sign in (1, -1):
                start = list(p.initial_position_mm)
                center = start[:]
                center[u] -= 5
                end = center[:]
                end[v] += sign * 5
                arc = Motion(
                    1, 'arc', 't1', Arc(tuple(start), tuple(end), tuple(center), sign * math.pi / 2, plane), 1200
                )
                he = end[:]
                he[a] -= 1
                helix = Motion(
                    2, 'helix', 't1', Helix(tuple(end), tuple(he), tuple(center), sign * math.tau, plane), 300, 'plunge'
                )
                program = replace(p, events=(*p.events[:3], arc, helix))
                chunks = list(post_program(program, m))
                self.assertEqual(len(chunks[1].motions), 2)
                self.assertIsInstance(chunks[1].motions[1].curve, Helix)
                self.assertLess(max(c['deviation_bound_mm'] or 0 for c in chunks[1].data['checked']), 0.001)

    def test_declared_machine_capabilities_table_and_continuous_tip_travel(self):
        p, m = self.machine(tip_travel_mm=((-10, -2, -10), (10, 2, 10)))
        arc = Motion(1, 'arc', 't1', Arc((5, 0, 3), (-5, 0, 3), (0, 0, 3), math.pi), 1200)
        e = PostEngine(m)
        e.begin(p)
        with self.assertRaisesRegex(ValueError, 'travel'):
            e.consume((*p.events[:3], arc))
        self.assertEqual(e.position_mm, p.initial_position_mm)
        for changed in [
            replace(m, arcs=False, helices=False),
            replace(m, max_feed_mm_min=10),
            replace(m, max_spindle_rpm=10),
        ]:
            with self.assertRaises(ValueError):
                list(post_program(replace(p, events=(*p.events[:3], arc)), changed))
        with self.assertRaises(ValueError):
            list(post_program(p, replace(m, tools=(replace(m.tools[0], gauge_length_mm=99),))))

    def test_one_travel_rule_a_limit_is_inside_and_a_word_rounded_beyond_it_is_refused(self):
        # 246: the posted-output check holds the decoded words (what the
        # controller executes) to the plan check's travel rule. Travel
        # X 0..300, Y 0..200, Z -300..-150 (machine).
        from camlib.verification import Scene, Verifier

        travel = ((0, 0, -300), (300, 200, -150))
        start = (5, 0, 3)

        def run(machine, *points):
            p, m = machine
            moves, at = [], start
            for i, point in enumerate(points):
                moves.append(Motion(i + 1, 'pocket', 't1', Line(at, point), 1200))
                at = point
            return ''.join(c.text for c in post_program(replace(p, events=(*p.events[:4], *moves)), m))

        def scene(m):
            return Scene('mill', '1', machine=m.verification_limits(), work_to_machine=m.work_to_machine)

        def planned(m, *points):
            with Verifier(scene(m)) as verifier:
                at = start
                for point in points:
                    report = verifier.check(fixture()[0].tools[0], Motion(1, 'pocket', 't1', Line(at, point), 1200))
                    if any(f['kind'] == 'machine_travel' for f in report['findings']):
                        return False
                    at = point
            return True

        on_grid = self.machine(tip_travel_mm=travel, work_to_machine=WorkFrame((100, 50, -200)))
        for decoder in (GCodeDecoder, NativeGCodeDecoder):
            # Into the corner of three limits: on them, so inside, in both checks.
            p, m = on_grid
            corner = (200, 150, 50)
            self.assertTrue(planned(m, corner))
            text = ''.join(
                c.text
                for c in post_program(
                    replace(p, events=(*p.events[:4], Motion(1, 'pocket', 't1', Line(start, corner), 1200))),
                    m,
                    decoder=decoder,
                )
            )
            self.assertIn('X200', text)
        # 1e-6 mm beyond a limit, written as it is (six decimals): refused, as the plan check reports it.
        fine = self.machine(tip_travel_mm=travel, work_to_machine=WorkFrame((100, 50, -200)), decimals=6)
        for beyond, axis, side in (
            ((200.000001, 150, 50), 'X', 'above'),
            ((200, 150.000001, 50), 'Y', 'above'),
            ((200, 150, 50.000001), 'Z', 'above'),
            ((-100.000001, 0, 3), 'X', 'below'),
        ):
            self.assertFalse(planned(fine[1], beyond))
            with self.assertRaisesRegex(ValueError, f'machine travel on machine {axis}: .* {side} its limit'):
                run(fine, beyond)
        run(fine, (200, 150, 50))
        # A work offset off the 0.001 mm output grid: the plan ends on the X
        # limit (machine X 300); its word, rounded to three decimals, is
        # 0.0004 mm beyond. Refused, never tolerated.
        off_grid = self.machine(tip_travel_mm=travel, work_to_machine=WorkFrame((100.0004, 50, -200)))
        end = (199.9996, 150, 50)
        self.assertTrue(planned(off_grid[1], end))
        with self.assertRaisesRegex(
            ValueError, 'machine X: tip at 300.000400 mm, above its limit 300.000000 mm.*rounded beyond'
        ):
            run(off_grid, end)
        # Written with a fourth decimal the position is exact and inside.
        self.assertIn('X199.9996', run((off_grid[0], replace(off_grid[1], decimals=4)), end))
        # A rotated work frame: the move itself is decided, not a box around
        # it (the corners of this diagonal's work-axis box are outside).
        angle = math.pi / 6
        c, s = math.cos(angle), math.sin(angle)
        frame = WorkFrame((140, 90, -200), (c, s, 0), (-s, c, 0), (0, 0, 1))

        def work(q):
            d = [q[i] - frame.origin_mm[i] for i in range(3)]
            return tuple(sum(d[i] * axis[i] for i in range(3)) for axis in (frame.x, frame.y, frame.z))

        turned = self.machine(tip_travel_mm=travel, work_to_machine=frame)
        a, b = work((1, 20, -195)), work((250, 180, -160))
        self.assertTrue(planned(turned[1], a, b))
        run(turned, a, b)
        out = work((300.002, 180, -160))
        self.assertFalse(planned(turned[1], a, out))
        with self.assertRaisesRegex(ValueError, 'machine travel on machine X'):
            run(turned, a, out)

    def test_a_decoded_arc_is_inside_travel_only_by_its_endpoint_gap(self):
        # 246: a decoded arc whose words leave an endpoint gap is not one
        # curve (controllers reconcile the two radii differently), so the
        # posted check admits it only when its nominal circle is inside the
        # limit by the gap. Travel Y 0..200 (machine), work Y 150 at the limit.
        from camlib.verification import Scene, Verifier

        travel = ((0, 0, -300), (300, 200, -150))
        start = (5, 0, 3)

        def program(machine, top, to_deg):
            # Radius 10 about (55, top - 10), clockwise from 180 degrees over the top to `to_deg`.
            p, m = machine
            cy, a = top - 10, math.radians(to_deg)
            begin, end = (45, cy, 3), (55 + 10 * math.cos(a), cy + 10 * math.sin(a), 3)
            arc = Motion(2, 'pocket', 't1', Arc(begin, end, (55, cy, 3), a - math.pi), 1200)
            return replace(p, events=(*p.events[:4], Motion(1, 'pocket', 't1', Line(start, begin), 1200), arc)), m, arc

        def planned(m, arc):
            scene = Scene('mill', '1', machine=m.verification_limits(), work_to_machine=m.work_to_machine)
            with Verifier(scene) as verifier:
                return verifier.check(fixture()[0].tools[0], arc)['status']

        def decoded_reach(p, m):
            # Independent of the checker: the highest work Y the decoded arc's
            # nominal circle reaches (it passes over the top), plus its gap.
            (arc,) = [x.curve for c in post_program(p, m) for x in c.motions if isinstance(x.curve, Arc)]
            radius = math.hypot(arc.start[0] - arc.center[0], arc.start[1] - arc.center[1])
            angle = math.atan2(arc.start[1] - arc.center[1], arc.start[0] - arc.center[0]) + arc.sweep_rad
            nominal = (arc.center[0] + radius * math.cos(angle), arc.center[1] + radius * math.sin(angle))
            gap = math.hypot(nominal[0] - arc.end[0], nominal[1] - arc.end[1])
            return arc.center[1] + radius + gap, gap

        for decimals in (3, 6):
            machine = self.machine(tip_travel_mm=travel, work_to_machine=WorkFrame((100, 50, -200)), decimals=decimals)
            # A half turn tangent to the limit: every word is exact, no gap, it posts.
            p, m, arc = program(machine, 150, 0)
            self.assertEqual(planned(m, arc), 'clear')
            self.assertRegex(''.join(c.text for c in post_program(p, m)), r'G0?2 .*I10')
            self.assertEqual(decoded_reach(p, m), (150, 0))
            # Tangent, ending at 60 degrees: the end is not on the output
            # grid. The plan is clear. Whatever words the post writes, the
            # program is released only if the decoded arc is inside by its gap:
            # at three decimals the post's closing words put the arc a few
            # micrometres inside; at six they leave a gap at the limit.
            p, m, arc = program(machine, 150, 60)
            self.assertEqual(planned(m, arc), 'clear')
            try:
                reach, gap = decoded_reach(p, m)
            except ValueError as refused:
                self.assertRegex(
                    str(refused),
                    'decoded arc of tool t1 is not inside configured machine travel on machine Y: its nominal '
                    'extreme at 200.000000 mm .* maximum 200.000000 mm.* endpoint gap of .*more decimals.*lower it to lines',
                )
                self.assertEqual(decimals, 6)
            else:
                self.assertLessEqual(reach, 150)
                self.assertEqual(decimals, 3)
            # Well inside the limit (by far more than a rounding gap): it posts, inside by its gap.
            p, m, arc = program(machine, 149.9, 60)
            reach, gap = decoded_reach(p, m)
            self.assertLess(gap, 10**-decimals)
            self.assertLessEqual(reach, 150)

    def test_gauge_line_travel_checks_each_decoded_tool_at_its_own_length(self):
        # 176: the gauge line travels Z -400..0 (machine); the work origin is
        # at machine Z -300. The 60 mm tool's tip reaches work Z 240, the
        # 115 mm tool's work Z 185.
        from camlib.tools import HolderSection

        p, m = self.machine()
        short = p.tools[0]
        self.assertEqual(short.gauge_length_mm, 60)
        long = replace(short, id='long', gauge_length_mm=115, holders=(HolderSection(35, 115, 4, 4),))
        start = p.initial_position_mm
        m = replace(
            m,
            tools=(*m.tools, ToolSetting.for_tool(long, number=3, length_register=8, position_after_mm=start)),
            work_to_machine=WorkFrame((0, 0, -300)),
            gauge_travel_mm=((-500, -500, -400), (500, 500, 0)),
        )

        def program(short_top, long_top):
            def rise(first, tool, top):
                peak = (start[0], start[1], top)
                return (
                    Motion(first, 'rise', tool, Line(start, peak), 1200),
                    Motion(first + 1, 'rise', tool, Line(peak, start), 1200),
                )

            return replace(
                p,
                tools=(short, long),
                events=(
                    *p.events[:3],
                    *rise(1, 't1', short_top),
                    Spindle(0),
                    ToolChange('long', 3, 8, start),
                    Spindle(12000),
                    *rise(3, 'long', long_top),
                    Spindle(0),
                ),
            )

        for decoder in (GCodeDecoder, NativeGCodeDecoder):
            # Each tool up to its own Z home, on its limit.
            chunks = list(post_program(program(240, 185), m, decoder=decoder))
            self.assertIn('Z240', ''.join(c.text for c in chunks))
            # The long tool where only the short one can go.
            with self.assertRaisesRegex(ValueError, r'tool long \(gauge length 115 mm\).*machine Z'):
                list(post_program(program(240, 240), m, decoder=decoder))
            with self.assertRaisesRegex(ValueError, r'tool t1 \(gauge length 60 mm\).*machine Z'):
                list(post_program(program(240.001, 185), m, decoder=decoder))
        # One tip box cannot describe both tools: the short tool's refuses the
        # long tool's low reach, the long tool's the short tool's home.
        for tip_top, moves in ((-60, (240, -200)), (-115, (240, 185))):
            boxed = replace(m, gauge_travel_mm=None, tip_travel_mm=((-500, -500, tip_top - 400), (500, 500, tip_top)))
            with self.assertRaisesRegex(ValueError, 'travel'):
                list(post_program(program(*moves), boxed))
        list(post_program(program(240, -215), m))

    def test_streams_before_source_exhaustion_cancels_before_pulling_next_batch(self):
        p, m = self.machine()
        produced = 0

        def events():
            nonlocal produced
            for event in p.events[:3]:
                produced += 1
                yield event
            for i in range(200):
                produced += 1
                yield Motion(i + 1, 'long', 't1', Line((5 + i, 0, 3), (6 + i, 0, 3)), 1200)

        token = Cancellation()
        stream = post_events(replace(p, events=()), events(), m, cancel=token)
        next(stream)
        self.assertEqual(produced, 0)
        chunk = next(stream)
        self.assertEqual(produced, 64)
        self.assertLess(len(chunk.raw), 512 * 1024)
        token.cancel()
        with self.assertRaisesRegex(ValueError, 'cancelled'):
            next(stream)
        self.assertEqual(produced, 64)

    def test_bounded_user_generators_and_lifecycle(self):
        p, m = self.machine()

        class Infinite(GCodePost):
            def on_motion(self, motion):
                return itertools.repeat('; endless post')

        e = PostEngine(m, Infinite)
        e.begin(p)
        with self.assertRaisesRegex(ValueError, 'limit'):
            e.consume(p.events)
        self.assertEqual(e.position_mm, p.initial_position_mm)
        e = PostEngine(m)
        e.begin(p)
        with self.assertRaises(ValueError):
            e.begin(p)
        with self.assertRaises(ValueError):
            e.consume(itertools.repeat(p.events[0], 65))
        e.consume(p.events)
        e.finish()
        with self.assertRaises(ValueError):
            e.finish()
        with self.assertRaises(ValueError):
            e.consume(())
        subprocess.run(
            [sys.executable, '-c', 'from camlib.postprocessor import Machine; from camlib.post import PostEngine'],
            check=True,
        )

    def test_single_motion_blocks_retain_all_word_modal_state_and_feed_rejections(self):
        p, m = self.machine()
        bad_blocks = (
            'G1 X10 Y0 Z3 F1200 H7',
            'G1 X10 Y0 Z3 F1200 T2',
            'G1 X10 Y0 Z3 F1200 I1',
            'G1 X10 Y0 Z3 F1200 Q1',
            'G1 X10 X11 Y0 Z3 F1200',
            'G1 G1 X10 Y0 Z3 F1200',
            'G1 G90 X10 Y0 Z3 F1200',
            'G1 X10 Y0 Z3',
            'G1 X10 Y0 Z3 F0',
            'G0 X10 Y0 Z3 F1200',
            'G1 X10 Y0 Z3 F1200junk',
        )
        for text in bad_blocks:

            class Bad(GCodePost):
                def on_motion(self, motion):
                    self.emit(text)

            e = PostEngine(m, Bad)
            e.begin(p)
            with self.assertRaises(ValueError, msg=text):
                e.consume(p.events)
            self.assertEqual(e.position_mm, p.initial_position_mm)
        # Single-motion shortcut also follows arbitrary declared codes/addresses.
        d = replace(m.dialect, axes=('U', 'V', 'W'), feed_word='E').with_commands(linear='G101')
        chunks = list(post_program(p, replace(m, dialect=d)))
        self.assertIn('G101 U10 V0 W3 E1200', chunks[1].text)
        self.assertEqual(chunks[1].motions, (p.events[4],))

    def test_immutable_decoder_tables_and_custom_mutable_state_keep_batch_isolation(self):
        p, m = self.machine()
        e = PostEngine(m)
        e.begin(p)
        with self.assertRaises(TypeError):
            e.decoder.rules[('G', 999.0)] = 'linear'
        for mutable_table in (False, True):

            class UserDecoder(GCodeDecoder):
                def __init__(self, machine, program):
                    super().__init__(machine, program)
                    if mutable_table:
                        self.rules = dict(self.rules)
                    self.observed = {'blocks': []}

                def read_line(self, line, context=None):
                    self.observed['blocks'].append(line)
                    if mutable_table:
                        self.rules[('G', 900.0 + len(self.observed['blocks']))] = 'linear'
                    return super().read_line(line, context)

            class Bad(GCodePost):
                def __init__(self, machine):
                    super().__init__(machine)
                    self.fail = True

                def on_motion(self, motion):
                    if self.fail:
                        self.emit('G1 X99 Y0 Z3 F1200')
                    else:
                        super().on_motion(motion)

            e = PostEngine(m, Bad, decoder=UserDecoder)
            e.begin(p)
            before = list(e.decoder.observed['blocks'])
            rules = dict(e.decoder.rules)
            with self.assertRaisesRegex(ValueError, 'deviation'):
                e.consume(p.events)
            self.assertEqual(e.decoder.observed['blocks'], before)
            self.assertEqual(dict(e.decoder.rules), rules)
            e.post.fail = False
            e.consume(p.events)
            e.finish()
            self.assertGreater(len(e.decoder.observed['blocks']), len(before))

    def test_actual_full_pocket_uses_python_hooks_and_independently_checked_posted_stock(self):
        examples = str(Path(__file__).resolve().parents[2] / 'examples')
        sys.path.insert(0, examples)
        try:
            from support.post_pocket import run
            from python_post import MyShopPost
        finally:
            sys.path.remove(examples)
        with tempfile.TemporaryDirectory() as directory:
            result = run(
                Path(directory) / 'pocket.nc',
                profile='python',
                post_type=MyShopPost,
                machine_options=dict(decimals=6, tolerance_mm=0.001),
            )
        self.assertEqual(result['source_motions'], 352)
        self.assertEqual(result['motions'], 352)
        self.assertFalse(result['first_posted_motion']['source_native_exited'])
        self.assertFalse(result['first_posted_motion']['source_finished'])
        self.assertEqual(result['posted_scene_stock_counts'], dict(clear=352, violation=0, uncertain=0))
        self.assertEqual(result['source_heights_sha256'], result['posted_heights_sha256'])
        self.assertTrue(result['posted_material']['complete'])
        self.assertEqual(result['posted_material']['residual']['definite_cells'], 12)


if __name__ == '__main__':
    unittest.main()
