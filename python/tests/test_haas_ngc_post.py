from dataclasses import replace
import math
import json
from pathlib import Path
import sys
import tempfile
import unittest

from camlib.motion import Arc, Coolant, Helix, Line, Motion, Spindle, Stop, ToolChange, WorkFrame, WorkOffset
from camlib.post import Cancellation, LoweringOptions, PostEngine, ToolSetting
from test_post import fixture
from haas_ngc_oracle import Oracle

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'examples'))
try:
    from haas_ngc_post import define_machine, HaasNGCPost, HaasNGCDecoder, post_haas, post_events_haas
finally:
    sys.path.pop(0)


class HaasNGCTest(unittest.TestCase):
    def machine(self, **settings):
        p, s = fixture()
        p = replace(p, events=(*p.events[:5], Coolant(), Spindle(0), Stop()))
        m = define_machine(
            'User-defined Haas2020 test setup',
            s.tools,
            traverse_feed_mm_min=settings.pop('traverse_feed_mm_min', 3000),
            coordinate_system=settings.pop('coordinate_system', 'fanuc'),
            h_t_agreement=settings.pop('h_t_agreement', False),
            g92_shift_mm=settings.pop('g92_shift_mm', (0, 0, 0)),
            rotation_scaling=settings.pop('rotation_scaling', True),
            work_to_machine=WorkFrame(origin_mm=(100, 200, 300)),
            **{
                'max_feed_mm_min': 10000,
                'max_spindle_rpm': 30000,
                'max_sweep_rad': 6.283185307179586,
                'tool_changes': True,
                'flood': True,
                'tool_change_position_mode': 'table',
                'rapid_mm_min': 6000,
                'output_units': (settings).get('output_units', 'mm'),
                'decimals': 3 if (settings).get('output_units', 'mm') == 'mm' else 4,
                'dwell_max_s': 1e5,
                **(settings),
            },
        )
        return p, m

    def oracle(self, p, m, chunks):
        oracle = Oracle(
            tools=m.tools,
            initial_position=p.initial_position_mm,
            program_number=m.program_number,
            work_offset=m.work_offset,
            output_units=m.output_units,
            coordinate_system=m.coordinate_system,
            h_t_agreement=m.h_t_agreement,
            rotation_scaling=m.rotation_scaling,
            optional_stop_enabled=m.optional_stop_enabled,
        )
        observed = []
        for c in chunks:
            for line in c.text.splitlines():
                observed.extend(oracle.read(line))
        actual = [v for c in chunks for v in c.motions]
        self.assertTrue(oracle.ended)
        self.assertEqual(len(observed), len(actual))
        for x, y in zip(observed, actual):
            self.assertEqual(x.tool, y.tool)
            self.assertEqual(x.curve, y.curve)
            self.assertEqual(x.feed_mm_min, y.feed_mm_min)
        return oracle

    def test_rounded_short_arc_emits_exact_radii_and_independent_oracle_agrees(self):
        p, m = self.machine(curve_policy='exact_ijk')
        data = json.loads((Path(__file__).resolve().parents[2] / 'fixtures/motion/rounded-short-arc.json').read_text())
        arc = Arc(tuple(data['start']), tuple(data['end']), tuple(data['center']), data['sweep_rad'])
        events = (
            *p.events[:3],
            Motion(1, 'approach', 't1', Line(p.initial_position_mm, arc.start), 300),
            Motion(2, 'rounded', 't1', arc, 150),
        )
        job = replace(p, events=events)
        chunks = list(post_haas(job, m))
        self.oracle(job, m, chunks)
        actual = [v for c in chunks for v in c.motions][-1].curve
        self.assertIsInstance(actual, Arc)
        self.assertEqual(math.dist(actual.start, actual.center), math.dist(actual.end, actual.center))
        self.assertGreater(math.dist(actual.center, arc.center), 0.01)
        # Centre displacement is not path error: the complete short arc stays
        # within the original output budget, without controller radius repair.
        for c in chunks:
            for checked in c.data['checked']:
                self.assertLessEqual(checked.get('deviation_bound_mm') or 0, 0.01)

    def test_literal_golden_full_circles_and_helix_match_manual_geometry(self):
        p, m = self.machine(curve_policy='exact_ijk')
        points = ((5, 0, 3), (0, 5, 3), (5, 0, 3), (5, 0, 3), (5, 0, 2))
        curves = (
            Arc(points[0], points[1], (0, 0, 3), math.pi / 2),
            Arc(points[1], points[2], (0, 0, 3), -math.pi / 2),
            Arc(points[2], points[3], (0, 0, 3), math.tau),
            Helix(points[3], points[4], (0, 0, 3), -math.tau),
        )
        moves = tuple(
            Motion(i + 1, 'haas-golden', 't1', c, 300 if isinstance(c, Helix) else 1200) for i, c in enumerate(curves)
        )
        p = replace(p, events=(*p.events[:3], *moves, Spindle(0)))
        chunks = list(post_haas(p, m))
        golden = (Path(__file__).resolve().parents[2] / 'fixtures/posts/O1000.NC').read_text()
        self.assertEqual(''.join(c.text for c in chunks), golden)
        self.assertEqual([v.curve for c in chunks for v in c.motions], list(curves))
        self.oracle(p, m, chunks)

    def test_units_options_setting33_and_feed_traverses(self):
        for units in ('mm', 'inch'):
            for option in (True, False):
                for convention in ('fanuc', 'haas'):
                    p, m = self.machine(output_units=units, rotation_scaling=option, coordinate_system=convention)
                    motion = Motion(1, 'clearance', 't1', Line(p.initial_position_mm, (10, 0, 3)), kind='rapid')
                    p = replace(p, events=(*p.events[:2], motion))
                    chunks = list(post_haas(p, m))
                    text = ''.join(c.text for c in chunks)
                    self.assertNotIn('G0 ', text)
                    self.assertNotIn('G91.1', text)
                    self.assertNotIn('G97', text)
                    self.assertEqual('G69\n' in text, option)
                    self.assertEqual('G50\n' in text, option)
                    actual = [v for c in chunks for v in c.motions][0]
                    self.assertEqual(actual.kind, 'rapid')
                    self.assertAlmostEqual(actual.feed_mm_min, 3000, delta=0.1)
                    self.assertEqual(
                        chunks[0].data['definition']['machine']['controller']['coordinate_system'], convention
                    )
                    self.oracle(p, m, chunks)

    def test_exact_curves_both_senses_mm_inch_and_full_circle_no_endpoint_words(self):
        for units in ('mm', 'inch'):
            p, m = self.machine(curve_policy='exact_ijk', output_units=units)
            start = (5.08, 0, 2.54) if units == 'inch' else p.initial_position_mm
            radius = 5.08 if units == 'inch' else 5
            m = replace(m, tools=(replace(m.tools[0], position_after_mm=start),))
            controls = (p.events[0], replace(p.events[1], position_after_mm=start), p.events[2])
            for sign in (-1, 1):
                for full in (False, True):
                    end = start if full else (start[0] - radius, sign * radius, start[2])
                    center = (start[0] - radius, 0, start[2])
                    arc = Motion(
                        1, 'native', 't1', Arc(start, end, center, sign * (math.tau if full else math.pi / 2)), 1200
                    )
                    helix = Motion(
                        2,
                        'native',
                        't1',
                        Helix(
                            end,
                            (end[0], end[1], end[2] - (0.508 if units == 'inch' else 1)),
                            (center[0], 0, end[2]),
                            sign * math.tau,
                        ),
                        300,
                    )
                    job = replace(p, initial_position_mm=start, events=(*controls, arc, helix, Spindle(0)))
                    chunks = list(post_haas(job, m))
                    self.oracle(job, m, chunks)
                    last = [item['text'] for c in chunks for item in c.data['lines'] if item['motion_id'] == 2][-1]
                    self.assertNotIn(' X', last)
                    self.assertNotIn(' Y', last)
                    self.assertIn(' Z', last)

    def test_alternate_work_offset_program_and_required_final_framing(self):
        p, m = self.machine(work_offset=2, program_number=60021)
        p = replace(p, events=(WorkOffset(2), *p.events[1:]))
        chunks = list(post_haas(p, m))
        self.oracle(p, m, chunks)
        self.assertIn('O60021\n', chunks[0].text)

        class Unclosed(HaasNGCPost):
            omit = True

            def on_finish(self):
                super().on_finish()
                if self.omit:
                    self._lines.pop()

        e = PostEngine(m, Unclosed, decoder=HaasNGCDecoder)
        e.begin(p)
        e.consume(p.events)
        before = e.position_mm
        with self.assertRaisesRegex(ValueError, 'finish'):
            e.finish()
        self.assertFalse(e.finished)
        self.assertFalse(e.decoder.end_seen)
        self.assertEqual(e.position_mm, before)
        e.post.omit = False
        end = e.finish()
        self.assertTrue(end.text.endswith('M30\n%\n'))
        prefix = chunks[0].text.splitlines() + [
            item['text']
            for c in chunks[1:]
            for item in c.data['lines']
            if item['event_index'] is not None and item['event_index'] < 3
        ]
        for bad in ('G0 X10 Y0 Z3', 'G64', 'G52 X1 Y0 Z0', 'G187 P3 E1', 'M7', 'G3 X5. Y0. I-5. J0. F1200.'):
            oracle = Oracle(
                tools=m.tools,
                initial_position=p.initial_position_mm,
                work_offset=2,
                program_number=60021,
                rotation_scaling=True,
            )
            for line in prefix:
                oracle.read(line)
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                oracle.read(bad)

    def test_initialization_omissions_and_invalid_machine_bindings_reject(self):
        p, m = self.machine()
        for omitted in ('%', 'O1000', 'G52 X0 Y0 Z0', 'G100 X0 Y0 Z0', 'G187', 'G61', 'G69', 'G50', 'G49', 'G80'):

            class Omit(HaasNGCPost):
                def on_begin(self, program):
                    super().on_begin(program)
                    self._lines = [line for line in self._lines if line != omitted]

            with self.subTest(omitted=omitted), self.assertRaises(ValueError):
                PostEngine(m, Omit, decoder=HaasNGCDecoder).begin(p)
        for changes in (
            dict(traverse_feed_mm_min=None),
            dict(h_t_agreement=True),
            dict(g92_shift_mm=(1, 0, 0)),
            dict(coordinate_system='linuxcnc'),
            dict(rotation_scaling='absent'),
            dict(decimals=6),
            dict(program_number=100000),
            dict(mirror_settings_xyz=(True, False, False)),
            dict(dialect=m.dialect.with_commands(exact_stop='G61.1')),
            dict(dialect=m.dialect.with_commands(frame_reset='G92.1')),
            dict(dialect=m.dialect.with_commands(flood='M7')),
        ):
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                replace(m, **changes)

    def test_hostile_actual_words_and_wrong_curve_geometry_roll_back(self):
        p, m = self.machine(curve_policy='exact_ijk')
        e = PostEngine(m, HaasNGCPost, decoder=HaasNGCDecoder)
        e.begin(p)
        e.consume(p.events[:3])
        context = dict(id=1, operation='hostile', kind='cut')
        bad = (
            'G0 X10 Y0 Z3',
            'G91.1',
            'G92.1',
            'G97',
            'G61.1',
            'G64',
            'G187 P3 E0.1',
            'G52',
            'G52 X1 Y0 Z0',
            'G100',
            'G18',
            'G19',
            'G92 X0',
            'G51 P2',
            'G68 R90',
            'G41 D1',
            'G44 H7',
            'G43 H0',
            'M7',
            'M2',
            'M0',
            'M6',
            'G53 G1 Z0 F100',
            'G93',
            'G1 X10 Y0 Z3',
            'G1 X10.0001 Y0. Z3. F1200.',
            'G3 X5. Y0. I-5. J0. F1200.',
            'G3 X0. Y5. R5. F1200.',
        )
        for line in bad:
            with self.subTest(line=line), self.assertRaises(ValueError):
                e._clone_decoder().read_line(line, context)

        class Wrong(HaasNGCPost):
            fail = True

            def on_motion(self, motion):
                if self.fail:
                    self.emit('G1 X10.02 Y0. Z3. F1200.')
                else:
                    super().on_motion(motion)

        e = PostEngine(m, Wrong, decoder=HaasNGCDecoder)
        e.begin(p)
        e.consume(p.events[:4])
        with self.assertRaisesRegex(ValueError, 'deviation'):
            e.consume((p.events[4],))
        self.assertEqual(e.position_mm, p.initial_position_mm)
        e.post.fail = False
        self.assertEqual(e.consume((p.events[4],)).motions[0].curve.end, (10, 0, 3))

    def test_linear_acceleration_and_reference_path_agree_on_actual_words_and_state(self):
        for units in ('mm', 'inch'):
            p, m = self.machine(output_units=units)
            e = PostEngine(m, HaasNGCPost, decoder=HaasNGCDecoder)
            prefix = [line['text'] for line in e.begin(p).data['lines']]
            prefix += [line['text'] for line in e.consume(p.events[:3]).data['lines']]
            fast = HaasNGCDecoder(m, p)
            slow = HaasNGCDecoder(m, p)
            slow._linear_block = None
            fast.read(prefix)
            slow.read(prefix)
            context = dict(id=1, operation='actual-words', kind='rapid')
            for line in (
                'G1 X10. Y0. Z3. F600.',
                'G1 X11.001 Y-2.001 Z3. F600.012',
                'g1x12y0z3f600',
                'G1 X10 Y0 Z3 F0',
                'G1 X10 Y0 Z3 F-1',
                'G1 X10.00001 Y0 Z3 F600',
                'G1 X1e2 Y0 Z3 F600',
                'G1 X10000000000 Y0 Z3 F600',
                'G1 X10 Y0 Z3 F600 M5',
                'G1 X10 X11 Y0 Z3 F600',
                'G0 X10 Y0 Z3',
            ):
                results = []
                for decoder in (fast, slow):
                    try:
                        results.append(('ok', decoder.read_line(line, context)))
                    except ValueError:
                        results.append(('error', None))
                self.assertEqual(results[0], results[1], line)
                self.assertEqual(fast.position, slow.position)
            for decoder in (fast, slow):
                decoder.read(('M5', 'M9', 'G49', 'M30'))
                with self.assertRaises(ValueError):
                    decoder.read_line('G1 X10 Y0 Z3 F600', context)
                decoder.read_line('%')
                self.assertTrue(decoder.ended)

    def test_explicit_decimals_survive_all_Haas_numeric_setting_conventions(self):
        for units in ('mm', 'inch'):
            p, m = self.machine(output_units=units)
            chunks = list(post_haas(p, m))
            prefix = chunks[0].text.splitlines() + [
                item['text']
                for c in chunks[1:]
                for item in c.data['lines']
                if item['event_index'] is not None and item['event_index'] < 3
            ]
            actual = [v for c in chunks for v in c.motions]
            for default_float in (False, True):
                for feed_notation in ('default', 'integer', '.1', '.01', '.001', '.0001'):
                    oracle = Oracle(
                        tools=m.tools,
                        initial_position=p.initial_position_mm,
                        output_units=units,
                        rotation_scaling=True,
                        default_to_float=default_float,
                        scale_integer_f=feed_notation,
                    )
                    observed = []
                    for c in chunks:
                        for line in c.text.splitlines():
                            observed.extend(oracle.read(line))
                    self.assertEqual([x.curve for x in observed], [x.curve for x in actual])
                    self.assertEqual([x.feed_mm_min for x in observed], [x.feed_mm_min for x in actual])
            oracle = Oracle(
                tools=m.tools, initial_position=p.initial_position_mm, output_units=units, rotation_scaling=True
            )
            for line in prefix:
                oracle.read(line)
            wrong = oracle.read('G1 X10 Y0 Z3 F600')[0]
            self.assertAlmostEqual(wrong.curve.end[0], 0.01 if units == 'mm' else 0.0254)
            self.assertAlmostEqual(wrong.feed_mm_min, 0.6 if units == 'mm' else 1.524)
            e = PostEngine(m, HaasNGCPost, decoder=HaasNGCDecoder)
            e.begin(p)
            e.consume(p.events[:3])
            for line in ('G1 X10 Y0. Z3. F600.', 'G1 X10. Y0. Z3. F600', 'G3 I-5 J0. F600.'):
                with self.assertRaisesRegex(ValueError, 'decimal points'):
                    e._clone_decoder().read_line(line, dict(id=1, operation='ambiguous', kind='cut'))

    def test_T_H_agreement_multiple_tools_and_stop_side_effects(self):
        p, m = self.machine()
        for event in (Stop(), Stop(True), ToolChange('t1', 2, 7, p.initial_position_mm)):
            with self.assertRaisesRegex(ValueError, 'spindle'):
                list(post_haas(replace(p, events=(*p.events[:4], event)), m))
        with self.assertRaisesRegex(ValueError, 'source spindle/coolant off'):
            list(post_haas(replace(p, events=(*p.events[:4], Spindle(0), p.events[1])), m))
        with self.assertRaisesRegex(ValueError, 'optional stop'):
            list(post_haas(replace(p, events=(*p.events[:2], Stop(True))), m))
        p = replace(p, events=(*p.events[:2], Stop(True)))
        self.oracle(
            p, replace(m, optional_stop_enabled=True), list(post_haas(p, replace(m, optional_stop_enabled=True)))
        )
        second = replace(p.tools[0], id='second')
        settings = (
            replace(m.tools[0], length_register=2),
            ToolSetting.for_tool(second, number=3, length_register=3, position_after_mm=p.initial_position_mm),
        )
        m = replace(m, tools=settings, h_t_agreement=True)
        events = (
            WorkOffset(),
            ToolChange('t1', 2, 2, p.initial_position_mm),
            Spindle(1200),
            Motion(1, 'first', 't1', Line(p.initial_position_mm, (10, 0, 3)), 600),
            Spindle(0),
            Coolant(),
            ToolChange('second', 3, 3, p.initial_position_mm),
            Spindle(1500, 'ccw'),
            Motion(2, 'second', 'second', Line(p.initial_position_mm, (10, 0, 3)), 600),
            Spindle(0),
        )
        p = replace(p, tools=(*p.tools, second), events=events)
        chunks = list(post_haas(p, m))
        self.oracle(p, m, chunks)

    def test_bounded_multiturn_allplane_lowering_and_cancel_preserve_source(self):
        p, m = self.machine()
        for plane, (u, v, a) in {'xy': (0, 1, 2), 'xz': (2, 0, 1), 'yz': (1, 2, 0)}.items():
            start = p.initial_position_mm
            end = list(start)
            end[a] -= 2
            center = list(start)
            center[u] -= 5
            motion = Motion(42, 'multiturn', 't1', Helix(start, tuple(end), tuple(center), -4 * math.tau, plane), 1200)
            job = replace(p, events=(*p.events[:3], motion))
            seen = []

            def events():
                for event in job.events:
                    seen.append(event)
                    yield event
                raise AssertionError('source requested after cancellation')

            token = Cancellation()
            stream = post_events_haas(job, events(), m, cancel=token)
            next(stream)
            self.assertFalse(seen)
            for _ in range(4):
                chunk = next(stream)
            self.assertEqual(len(seen), 4)
            self.assertEqual(len(chunk.motions), 64)
            self.assertLessEqual(len(chunk.raw), 512 * 1024)
            self.assertTrue(all(x['source_motion_id'] == 42 for x in chunk.data['mappings']))
            token.cancel()
            with self.assertRaisesRegex(ValueError, 'cancelled'):
                next(stream)
            chunks = list(post_haas(job, m))
            self.oracle(job, m, chunks)
            with self.assertRaisesRegex(ValueError, 'budget'):
                list(post_haas(job, m, lowering=LoweringOptions(max_segments_per_source=4)))

    def test_real_cadquery_pocket_independently_checks_posted_stock_and_literal_NC(self):
        sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'examples'))
        try:
            from support.post_pocket import run
        finally:
            sys.path.pop(0)
        captured = {}

        def builder(header, tools):
            machine = define_machine(
                'Haas local review',
                tools,
                traverse_feed_mm_min=3000,
                coordinate_system='fanuc',
                h_t_agreement=True,
                g92_shift_mm=(0, 0, 0),
                rotation_scaling=False,
                work_to_machine=WorkFrame(origin_mm=(100, 200, 300)),
                frame=header.frame,
                output_units='mm',
                decimals=3,
                max_feed_mm_min=10000,
                max_spindle_rpm=30000,
                max_sweep_rad=6.283185307179586,
                tool_changes=True,
                flood=True,
                tool_change_position_mode='table',
                rapid_mm_min=6000,
            )
            captured.update(machine=machine, header=header)
            return machine

        with tempfile.TemporaryDirectory() as directory:
            nc = Path(directory) / 'O1000.NC'
            result = run(
                nc,
                profile='python',
                lowering=True,
                radial_allowance_mm=0.01,
                post_type=HaasNGCPost,
                decoder_type=HaasNGCDecoder,
                machine_builder=builder,
            )
            tools = tuple(
                ToolSetting(**{**item, 'position_after_mm': tuple(item['position_after_mm'])})
                for item in result['machine']['tools']
            )
            oracle = Oracle(tools=tools, initial_position=tools[0].position_after_mm, h_t_agreement=True)
            decoder = HaasNGCDecoder(captured['machine'], captured['header'])
            count = 0
            for line in nc.read_text().splitlines():
                observed = oracle.read(line)
                actual = [
                    e
                    for e in decoder.read_line(line, dict(id=count + 1, operation='actual-file', kind='cut'))
                    if isinstance(e, Motion)
                ]
                self.assertEqual(
                    [(e.tool, e.curve, e.feed_mm_min) for e in observed],
                    [(e.tool, e.curve, e.feed_mm_min) for e in actual],
                )
                count += len(observed)
            self.assertTrue(decoder.ended)
        self.assertTrue(oracle.ended)
        self.assertEqual(count, result['motions'])
        self.assertEqual(result['source_motions'], 352)
        self.assertEqual(result['motions'], 816)
        self.assertEqual(result['posted_scene_stock_counts'], dict(clear=816, violation=0, uncertain=0))
        self.assertTrue(result['posted_material']['complete'])
        self.assertGreater(result['posted_material']['residual']['definite_cells'], 0)
        self.assertFalse(result['first_posted_motion']['source_finished'])
        self.assertFalse(result['first_posted_motion']['source_native_exited'])


if __name__ == '__main__':
    unittest.main()
