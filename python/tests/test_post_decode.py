"""Differential declared-dialect semantics, not controller/hardware qualification."""

from camlib.postprocessor import GCodeDialect, arc_radii

from dataclasses import asdict, replace
import json
import math
import random
import unittest

from camlib._buffers import native
from camlib.motion import Helix, Motion
from camlib.post import Machine, GCodePost, GCodeDecoder, NativeGCodeDecoder, post_program, PostEngine
from test_post import fixture


class MotionDecoder(unittest.TestCase):
    def setup_decoder(self, machine, cls):
        program, _ = fixture()
        decoder = cls(machine, program)
        post = GCodePost(machine)
        decoder.read(post._call('on_begin', program, limit=32))
        for event in program.events[:3]:
            decoder.read(post._call('on_' + event.to_dict()['kind'], event))
        return decoder

    def machine(self, **values):
        _, setup = fixture()
        return Machine(
            'Unregistered differential machine',
            setup.tools,
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
                'output_units': (values).get('output_units', 'mm'),
                'decimals': 3 if (values).get('output_units', 'mm') == 'mm' else 4,
                **(values),
            },
        )

    def compare_block(self, machine, line):
        outcomes = []
        for cls in (GCodeDecoder, NativeGCodeDecoder):
            decoder = self.setup_decoder(machine, cls)
            try:
                result = decoder.read_line(line, dict(id=1, operation='case', kind='cut'))
            except ValueError:
                outcomes.append(None)
            else:
                outcomes.append(
                    (
                        result,
                        decoder.position,
                        decoder.units,
                        decoder.plane,
                        decoder.rpm,
                        decoder.coolant,
                        decoder.ended,
                    )
                )
        self.assertEqual(outcomes[0] is None, outcomes[1] is None, msg=line)
        if outcomes[0] is not None:
            self.assertEqual(outcomes[0], outcomes[1], msg=line)

    def test_actual_word_lexer_and_state_semantics_match_independent_python_reference(self):
        machine = self.machine()
        valid = (
            'G1X10Y0Z3F1200',
            'n1 g01 x+10. y-0 z3 f+1200',
            'N0 N1 G1 X.5 Y-.25 F100 ; comment',
            'G1 X(comments)10 Y0 Z3 F1200',
            'G0 X10 Y0 Z3',
            'G90 G21 G17',
            'G1 X10 Y0 Z3 F1200 (end)',
            'M5',
            'G49',
            'M9',
            'G1 X10 Y0 Z3 F1200; (unclosed after semicolon',
        )
        invalid = (
            'G1 X10 X11 F1200',
            'G1 G1 X10 F1200',
            'G0 G1 X10 F1200',
            'G1 G90 X10 F1200',
            'G1 G20 X10 F1200',
            'G1 M5 X10 F1200',
            'G1 X10 H7 F1200',
            'G1 X10 D0 F1200',
            'G0 X10 F1200',
            'G1 X10 I1 F1200',
            'G1 X10 J1 F1200',
            'G1 X10 K1 F1200',
            'G1 X10 F0',
            'G1 X10 F-1',
            'G1 X10',
            'G999 X10 F1200',
            'G1 X1..2 F1200',
            'G1 X. F1200',
            'G1 X1e2 F1200',
            'G1 Xnan F1200',
            'G1 X1000000001 F1200',
            'G1 X10 F1200\n',
            'G1 X10 F1200\r',
            'G1 X10 F1200\x7f',
            'G1 X10 F1200é',
            'G1 X10 F1200 (unclosed',
            'G1 X10 F1200 ((nested))',
            'G1 X10 F1200 )',
            'G1 X10 F1200 (semi;)',
            'G1 X1(comment)2 F1200',
            'N-1 G1 X10 F1200',
            'N.1 G1 X10 F1200',
        )
        for line in (*valid, *invalid):
            self.compare_block(machine, line)
        rng = random.Random(431)
        for _ in range(250):
            end = [rng.uniform(-100, 100) for _ in range(3)]
            line = 'N27 G1 ' + ' '.join(axis + format(v, '.7f') for axis, v in zip(('X', 'Y', 'Z'), end)) + ' F1234.567'
            self.compare_block(machine, line)
        for units in ('mm', 'inch'):
            m = self.machine(output_units=units)
            m = replace(
                m,
                dialect=replace(m.dialect, axes=('U', 'V', 'W'), centers=('A', 'B', 'C'), feed_word='E').with_commands(
                    linear='G101'
                ),
            )
            for line in ('G101 U1 V2 W3 E45', 'G101 U1 V2 W3 F45', 'G101 U1 V2 W3 E45 (ok)'):
                self.compare_block(m, line)

    def test_arcs_all_planes_both_senses_helix_radius_and_actual_text_agree(self):
        p, _ = fixture()
        for units in ('mm', 'inch'):
            start = (0.2 * 25.4, 0.0, 0.1 * 25.4) if units == 'inch' else p.initial_position_mm
            controls = list(p.events[:3])
            controls[1] = replace(controls[1], position_after_mm=start)
            for plane, (u, v, a) in {'xy': (0, 1, 2), 'xz': (2, 0, 1), 'yz': (1, 2, 0)}.items():
                for sign in (-1, 1):
                    for full in (False, True):
                        center = list(start)
                        radius = 5.08 if units == 'inch' else 5
                        center[u] -= radius
                        end = list(start) if full else list(center)
                        if not full:
                            end[v] += sign * radius
                        end[a] -= 0.5
                        event = Motion(
                            1,
                            'arc',
                            't1',
                            Helix(start, tuple(end), tuple(center), sign * (math.tau if full else math.pi / 2), plane),
                            1234.5,
                        )
                        job = replace(p, initial_position_mm=start, events=(*controls, event))
                        m = self.machine(output_units=units, decimals=7)
                        m = replace(m, tools=(replace(m.tools[0], position_after_mm=start),))
                        outputs = [
                            list(post_program(job, m, decoder=cls)) for cls in (GCodeDecoder, NativeGCodeDecoder)
                        ]
                        self.assertEqual([c.text for c in outputs[0]], [c.text for c in outputs[1]])
                        left = outputs[0][1].motions[0]
                        right = outputs[1][1].motions[0]
                        self.assertEqual(left.curve.start, right.curve.start)
                        self.assertEqual(left.curve.end, right.curve.end)
                        self.assertEqual(left.curve.center, right.curve.center)
                        self.assertAlmostEqual(left.curve.sweep_rad, right.curve.sweep_rad, places=13)
                        self.assertEqual(left.feed_mm_min, right.feed_mm_min)
        for line in (
            'G2 X5 Y0 I-5 J0 F1200',
            'G3 X0 Y5 I-5 J0 F1200',
            'G2 X5 Y0 Z2 I-5 J0 F1200',
            'G2 X0 Y5 I-5 F1200',
            'G2 X0 Y5 I-5 J0 K1 F1200',
            'G2 X0 Y6 I-5 J0 F1200',
            'G2 X5 Y0 I0 J0 F1200',
        ):
            self.compare_block(self.machine(), line)

    def test_native_candidate_copies_keep_position_and_mutable_python_state_isolated(self):
        p, _ = fixture()
        m = self.machine()

        class Stateful(NativeGCodeDecoder):
            def __init__(self, m, p):
                super().__init__(m, p)
                self.log = []

            def read_line(self, line, context=None):
                self.log.append(line)
                return super().read_line(line, context)

        e = PostEngine(m, decoder=Stateful)
        e.begin(p)
        before = list(e.decoder.log)
        bad = replace(p, events=(*p.events[:4], replace(p.events[4], feed_mm_min=99999)))
        with self.assertRaises(ValueError):
            e.consume(bad.events)
        self.assertEqual(e.position_mm, p.initial_position_mm)
        self.assertEqual(e.decoder.log, before)
        e.consume(p.events)
        self.assertEqual(e.position_mm, (10, 0, 3))
        e.finish()
        self.assertGreater(len(e.decoder.log), len(before))
        for cls in (GCodeDecoder, NativeGCodeDecoder):
            d = cls(m, p)
            with self.assertRaises(ValueError):
                d.read_line('G1 X10 Y0 Z3 F1200', dict(id=1, operation='bad', kind='cut'))
            d = self.setup_decoder(m, cls)
            d.ended = True
            with self.assertRaises(ValueError):
                d.read_line('G1 X10 Y0 Z3 F1200', dict(id=1, operation='bad', kind='cut'))

    def test_batch_word_formatter_matches_python_and_honors_user_overrides(self):
        rng = random.Random(762)
        values = [0.0, -0.0, 0.125, -0.125, 0.005, -0.005, 999999.999, 1e-9, -1e-9] + [
            rng.uniform(-1000, 1000) for _ in range(200)
        ]
        for units in ('mm', 'inch'):
            for decimals in (2, 3, 5, 9):
                if units == 'inch' and decimals == 2:
                    continue
                p = GCodePost(self.machine(output_units=units, decimals=decimals, tolerance_mm=0.1))
                for value in values:
                    self.assertEqual(p.words('X', [value]), [p.word('X', value)])
        program, _ = fixture()
        for method in ('number', 'word'):

            class UserPost(GCodePost):
                def __init__(self, m):
                    super().__init__(m)
                    self.helper_calls = 0

            original = getattr(GCodePost, method)

            def user_helper(self, *args, **kwargs):
                self.helper_calls += 1
                return original(self, *args, **kwargs)

            setattr(UserPost, method, user_helper)
            e = PostEngine(self.machine(), UserPost)
            e.begin(program)
            chunk = e.consume(program.events)
            self.assertEqual(chunk.motions, (program.events[4],))
            self.assertEqual(e.post.helper_calls, 5)  # spindle S plus X/Y/Z/F
        for args in (('X', [float('nan')], 1, 3), ('X', [1000001.0], 1, 3), ('XY', [1.0], 1, 3), ('X', [1.0], 2, 3)):
            with self.assertRaises(ValueError):
                native.format_post_words(*args)

    def test_checked_chunk_views_match_authoritative_raw_and_snapshot_custom_objects(self):
        from camlib.post import Chunk
        from camlib.motion import Line

        p, _ = fixture()
        m = self.machine()

        class MutableDecoder(GCodeDecoder):
            def read_line(self, line, context=None):
                result = super().read_line(line, context)
                if result and isinstance(result[0], Motion):
                    motion = result[0]
                    result = [replace(motion, curve=Line(list(motion.curve.start), list(motion.curve.end)))]
                    self.last = result[0]
                return result

        for cls in (NativeGCodeDecoder, GCodeDecoder, MutableDecoder):
            e = PostEngine(m, decoder=cls)
            begin = e.begin(p)
            chunk = e.consume(p.events)
            end = e.finish()
            if cls is MutableDecoder:
                e.decoder.last.curve.end[0] = 999
            for c in (begin, chunk, end):
                raw = Chunk(c.raw)
                self.assertEqual(c.text, raw.text)
                self.assertEqual(c.motions, raw.motions)
                self.assertEqual(c.data, raw.data)
            changed = chunk.data
            changed['checked'][4]['motion']['curve']['end'][0] = 999
            self.assertEqual(chunk.motions[0].curve.end, (10, 0, 3))

    def test_replacing_native_rule_table_uses_custom_python_semantics(self):
        for cls in (GCodeDecoder, NativeGCodeDecoder):
            decoder = self.setup_decoder(self.machine(), cls)
            decoder.rules = dict(decoder.rules)
            decoder.rules[('G', 1.0)] = 'rapid'
            moves = decoder.read_line('G1 X10 Y0 Z3', dict(id=1, operation='custom', kind='rapid'))
            self.assertEqual(moves[0].kind, 'rapid')
            self.assertIsNone(moves[0].feed_mm_min)
            self.assertEqual(moves[0].curve.end, (10, 0, 3))

    def test_equal_radius_words_are_decided_exactly_at_zero_tolerance(self):
        # Issue #169: from (9.995, 20), X29.999 Y20 I10.002 J0.006 has exactly
        # equal radii. Rebuilt in binary64 the two radius vectors differ in the
        # last bit, and glibc's hypot rejected the arc at zero tolerance while
        # macOS's accepted it. Decided on the words, every platform accepts it
        # and still refuses a one-unit mismatch.
        machine = self.machine()
        machine = replace(machine, dialect=replace(machine.dialect, radius_tolerance_mm=0, radius_relative_tolerance=0))
        for cls in (GCodeDecoder, NativeGCodeDecoder):
            for line, accepted in (
                ('G2 X29.999 Y20 I10.002 J0.006 F100', True),
                ('G2 X29.999 Y20.001 I10.002 J0.006 F100', False),
            ):
                with self.subTest(cls=cls.__name__, line=line):
                    decoder = self.setup_decoder(machine, cls)
                    decoder.position = (9.995, 20.0, decoder.position[2])
                    decoder.plane = 'xy'
                    context = dict(id=1, operation='case', kind='cut')
                    if accepted:
                        moves = decoder.read_line(line, context)
                        self.assertEqual(moves[0].curve.end[:2], (29.999, 20.0))
                    else:
                        with self.assertRaisesRegex(ValueError, 'radius mismatch'):
                            decoder.read_line(line, context)

    def test_python_and_native_decoders_decide_arc_radii_alike(self):
        # Each decoder has its own implementation of the radius rule (#169);
        # at zero tolerance they must accept and refuse the same words. Ends
        # are the start rotated about the centre (exactly equal radii), then
        # nudged by -1, 0 or +1 in the last decimal.
        machine = self.machine()
        machine = replace(machine, dialect=replace(machine.dialect, radius_tolerance_mm=0, radius_relative_tolerance=0))
        rng = random.Random(169)
        outcomes = []

        def word(n):
            return f'{n / 1000:.3f}'

        for _ in range(300):
            s = [rng.randint(-50000, 50000) for _ in range(2)]
            i = [rng.randint(-20000, 20000) or 1 for _ in range(2)]
            c = [s[0] + i[0], s[1] + i[1]]
            x, y = s[0] - c[0], s[1] - c[1]
            x, y = rng.choice([(-y, x), (-x, -y), (y, -x)])
            e = [c[0] + x + rng.randint(-1, 1), c[1] + y + rng.randint(-1, 1)]
            line = f'G2 X{word(e[0])} Y{word(e[1])} I{word(i[0])} J{word(i[1])} F100'
            accepted = []
            for cls in (GCodeDecoder, NativeGCodeDecoder):
                decoder = self.setup_decoder(machine, cls)
                decoder.position = (float(word(s[0])), float(word(s[1])), decoder.position[2])
                decoder.plane = 'xy'
                try:
                    decoder.read_line(line, dict(id=1, operation='case', kind='cut'))
                    accepted.append(True)
                except ValueError as error:
                    self.assertIn('radius mismatch', str(error))
                    accepted.append(False)
            self.assertEqual(accepted[0], accepted[1], line)
            outcomes.append(accepted[0])
        self.assertTrue(any(outcomes) and not all(outcomes))

    def test_arc_radii_are_exact_on_the_word_grid_and_match_the_engine(self):
        r1, r2, error = arc_radii((9.995, 20.0), (29.999, 20.0), (10.002, 0.006), 1.0)
        self.assertEqual((error, r1), (0.0, r2))
        inch = [w * 25.4 for w in (9.995, 20.0, 29.999, 20.0, 10.002, 0.006)]
        self.assertEqual(arc_radii(inch[0:2], inch[2:4], inch[4:6], 25.4)[2], 0.0)
        # A start no word wrote: binary64 with correctly rounded operations.
        start = (0.1 + 0.2, 20.0)
        r1, r2, error = arc_radii(start, (10.3, 20.0), (5.0, 0.0), 1.0)
        center = start[0] + 5.0
        x, ex = start[0] - center, 10.3 - center
        self.assertEqual((r1, r2), (math.sqrt(x * x), math.sqrt(ex * ex)))
        self.assertEqual(error, abs(r1 - r2))

    def test_compiled_decoder_config_and_physical_block_caps_are_checked(self):
        dialect = asdict(self.machine().dialect)
        fields = ('commands', 'axes', 'centers', 'feed_word', 'parentheses_comments', 'radius_tolerance_mm')
        config = {key: dialect[key] for key in fields}
        for change in (
            dict(axes=['X', 'X', 'Z']),
            dict(feed_word='XY'),
            dict(radius_tolerance_mm=0.01),
            dict(commands=dialect['commands'] * 5),
        ):
            with self.assertRaises(ValueError):
                native.MotionDecoder(json.dumps({**config, **change}))
        decoder = native.MotionDecoder(json.dumps(config))
        for text in (' ' * 1025, 'G1 X1 F100\n'):
            with self.assertRaises(ValueError):
                decoder.decode(text, (0, 0, 0), 1, 'xy')


if __name__ == '__main__':
    unittest.main()
