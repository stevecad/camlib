from dataclasses import replace
import math
from pathlib import Path
import sys
import tempfile
import unittest

from camlib.motion import Coolant, Helix, Line, Motion, Spindle, Stop, ToolChange, WorkFrame, WorkOffset
from camlib.post import Cancellation, LoweringOptions, PostEngine, ToolSetting, post_events
from fanuc_0imb_oracle import Oracle
from test_post import fixture

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'examples'))
try:
    from fanuc_0imb_post import define_machine, Fanuc0iMBPost, Fanuc0iMBDecoder, post_fanuc
finally:
    sys.path.pop(0)

OEM = dict(tool_change='M6', spindle_cw='M3', spindle_ccw='M4', spindle_off='M5', coolant_off='M9', flood='M8')


class Fanuc0iMBTest(unittest.TestCase):
    def machine(self, **settings):
        p, s = fixture()
        p = replace(p, events=(*p.events[:5], Coolant(), Spindle(0)))
        m = define_machine(
            'Three-axis test mill',
            settings.pop('tools', s.tools),
            startup_units=settings.pop('startup_units', 'mm'),
            startup_standard_modes=settings.pop('startup_standard_modes', True),
            g92_shift_mm=settings.pop('g92_shift_mm', (0, 0, 0)),
            m_codes=settings.pop('m_codes', OEM),
            traverse_feed_mm_min=3000,
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
                **(settings),
            },
        )
        return p, m

    def oracle(self, p, m, chunks, calculator=False):
        oracle = Oracle(
            tools=m.tools,
            initial_position=p.initial_position_mm,
            output_units=m.output_units,
            program_number=m.program_number,
            work_offset=m.work_offset,
            calculator_decimal=calculator,
            optional_stop_enabled=m.optional_stop_enabled,
            m_codes=dict(m.oem_m_codes),
            tool_change_gauge_z=(
                m.tool_change_home_z_mm - m.work_to_machine.origin_mm[2]
                if m.tool_change_position_mode == 'z_home'
                else None
            ),
        )
        observed = []
        for chunk in chunks:
            for line in chunk.text.splitlines():
                observed.extend(oracle.read(line))
        actual = [v for c in chunks for v in c.motions]
        self.assertTrue(oracle.ended)
        self.assertEqual(
            [(v.tool, v.curve, v.feed_mm_min) for v in observed], [(v.tool, v.curve, v.feed_mm_min) for v in actual]
        )
        for _, a, b in oracle.transitions:
            self.assertAlmostEqual(a, b, places=9)
        return oracle

    def test_literal_golden_explicit_length_transitions_and_source_maps(self):
        p, m = self.machine()
        chunks = list(post_fanuc(p, m))
        text = ''.join(c.text for c in chunks)
        self.assertEqual(text, (Path(__file__).resolve().parents[2] / 'fixtures/posts/F0IMB1000.NC').read_text())
        oracle = self.oracle(p, m, chunks)
        self.assertEqual(oracle.transitions, [(43, 63.0, 63.0), (49, 63.0, 63.0)])
        lines = [item for c in chunks for item in c.data['lines']]
        self.assertEqual([item['number'] for item in lines], list(range(1, len(lines) + 1)))
        self.assertTrue(any(item.get('motion_id') == 1 and item['operation'] == 'pocket' for item in lines))
        self.assertEqual([v for c in chunks for v in c.motions], [p.events[4]])

    def test_mm_inch_decimal_conventions_and_builder_functions(self):
        base, _ = fixture()
        tool = replace(
            base.tools[0], gauge_length_mm=76.2, holders=tuple(replace(h, z_max_mm=76.2) for h in base.tools[0].holders)
        )
        start = (5.08, 0, 2.54)
        end = (10.16, 0, 2.54)
        table = (ToolSetting.for_tool(tool, number=2, length_register=7, position_after_mm=start),)
        p = replace(
            base,
            tools=(tool,),
            initial_position_mm=start,
            events=(
                WorkOffset(),
                ToolChange(tool.id, 2, 7, start),
                Spindle(12000),
                Motion(1, 'cut', tool.id, Line(start, end), 1200),
                Spindle(0),
            ),
        )
        for units in ('mm', 'inch'):
            for mapped in (OEM, dict(OEM, tool_change='M26', flood='M28', coolant_off='M29')):
                _, m = self.machine(tools=table, startup_units=units, output_units=units, m_codes=mapped)
                chunks = list(post_fanuc(p, m))
                for calculator in (False, True):
                    self.oracle(p, m, chunks, calculator)
                text = ''.join(c.text for c in chunks)
                self.assertIn(mapped['tool_change'] + '\n', text)
                self.assertNotIn('G0 ', text)
                self.assertNotIn('G91.1', text)
                self.assertNotIn('G97', text)

    def test_actual_word_mutations_reject_modes_offsets_and_implicit_motion(self):
        p, m = self.machine()
        text = ''.join(c.text for c in post_fanuc(p, m))
        variants = (
            text.replace('G61\n', ''),
            text.replace('G21\n', 'G20\n'),
            text.replace('G52 X0. Y0. Z0.', 'G52 X1. Y0. Z0.'),
            text.replace('G54\n', 'G55\n'),
            text.replace('G1 G43 H7 Z3. F3000.', 'G43 H7'),
            text.replace('G1 G49 Z63. F3000.', 'G49'),
            text.replace('G1 G43 H7 Z3. F3000.', 'G1 G43 H7 Z4. F3000.'),
            text.replace('G1 G49 Z63. F3000.', 'G1 G49 Z3. F3000.'),
            text.replace('G1 G43 H7', 'G1 G43 H8'),
            text.replace('X10.', 'X10'),
            text.replace('G1 X10.', 'G0 X10.'),
            text.replace('G94', 'G95'),
            text.replace('G17', 'G18'),
        )
        for broken in variants:
            with self.subTest(broken=broken):
                decoder = Fanuc0iMBDecoder(m, p)
                with self.assertRaises(ValueError):
                    for line in broken.splitlines():
                        decoder.read_line(line, dict(id=1, operation='file', kind='cut'))
        # The literal manual oracle interprets an omitted decimal differently;
        # it does not merely duplicate the production strict grammar.
        raw = text.replace('X10.', 'X10')
        oracle = Oracle(tools=m.tools, initial_position=p.initial_position_mm)
        observed = [x for line in raw.splitlines() for x in oracle.read(line)]
        self.assertEqual(observed[0].curve.end[0], 0.01)

    def test_two_tools_cancel_before_atc_and_atomic_bad_output_retry(self):
        p, m = self.machine()
        second = replace(
            p.tools[0],
            id='second',
            gauge_length_mm=70,
            holders=tuple(replace(h, z_max_mm=70) for h in p.tools[0].holders),
        )
        table = (*m.tools, ToolSetting.for_tool(second, number=3, length_register=8, position_after_mm=(5, 0, 4)))
        m = replace(m, tools=table)
        p = replace(
            p,
            tools=(*p.tools, second),
            events=(
                *p.events,
                ToolChange('second', 3, 8, (5, 0, 4)),
                Spindle(1200),
                Motion(2, 'second', 'second', Line((5, 0, 4), (10, 0, 4)), 600),
                Spindle(0),
            ),
        )
        chunks = list(post_fanuc(p, m))
        oracle = self.oracle(p, m, chunks)
        self.assertEqual(oracle.transitions, [(43, 63.0, 63.0), (49, 63.0, 63.0), (43, 74.0, 74.0), (49, 74.0, 74.0)])

        class Corrupt(Fanuc0iMBPost):
            bad = True

            def on_tool_change(self, event):
                super().on_tool_change(event)
                if self.bad:
                    self._lines[-1] = self._lines[-1].replace('Z3.', 'Z4.')

        engine = PostEngine(m, Corrupt, decoder=Fanuc0iMBDecoder)
        engine.begin(p)
        engine.consume(p.events[:1])
        with self.assertRaisesRegex(ValueError, 'physical tip'):
            engine.consume(p.events[1:2])
        engine.post.bad = False
        engine.consume(p.events[1:])
        engine.finish()

    def gauge_pose_job(self):
        """Two tools on a machine whose M06 leaves the gauge line at machine
        Z 380 (work Z 80: the work offset is Z 300), X and Y unmoved."""
        base, _ = fixture()
        first = base.tools[0]
        second = replace(
            first, id='second', gauge_length_mm=70, holders=tuple(replace(h, z_max_mm=70) for h in first.holders)
        )
        p = replace(
            base,
            tools=(first, second),
            initial_position_mm=(5, 0, 20),
            events=(
                WorkOffset(),
                ToolChange(first.id, 2, 7, (5, 0, 20)),
                Spindle(12000),
                Motion(1, 'pocket', first.id, Line((5, 0, 20), (10, 0, 20)), 1200),
                Spindle(0),
                ToolChange('second', 3, 8, (10, 0, 10)),
                Spindle(1200),
                Motion(2, 'second', 'second', Line((10, 0, 10), (15, 0, 10)), 600),
                Spindle(0),
            ),
        )
        return p, first, second

    def test_two_tools_post_from_the_declared_gauge_pose_without_measured_positions(self):
        p, first, second = self.gauge_pose_job()
        declared = (
            ToolSetting.for_tool(first, number=2, length_register=7),
            ToolSetting.for_tool(second, number=3, length_register=8),
        )
        self.assertIsNone(declared[0].position_after_mm)
        _, m = self.machine(tools=declared, tool_change_position_mode='z_home', tool_change_home_z_mm=380)
        chunks = list(post_fanuc(p, m))
        text = ''.join(c.text for c in chunks)
        # Each tip is the gauge line (work Z 80) less its gauge length, and
        # both compensation blocks of each tool keep the spindle there.
        for block in ('G1 G43 H7 Z20. F3000.', 'G1 G49 Z80. F3000.', 'G1 G43 H8 Z10. F3000.'):
            self.assertIn(block + '\n', text)
        self.assertEqual(text.count('G1 G49 Z80. F3000.\n'), 2)
        oracle = self.oracle(p, m, chunks)
        self.assertEqual(oracle.transitions, [(43, 80.0, 80.0), (49, 80.0, 80.0), (43, 80.0, 80.0), (49, 80.0, 80.0)])
        # The same machine declared by measured tip poses writes the same file.
        measured = (
            ToolSetting.for_tool(first, number=2, length_register=7, position_after_mm=(5, 0, 20)),
            ToolSetting.for_tool(second, number=3, length_register=8, position_after_mm=(10, 0, 10)),
        )
        _, table = self.machine(tools=measured)
        self.assertEqual(''.join(c.text for c in post_fanuc(p, table)), text)
        # The decoder holds the G43 block to the derived pose, and the plan to it.
        for broken in (
            text.replace('G1 G43 H8 Z10. F3000.', 'G1 G43 H8 Z20. F3000.'),
            text.replace('G1 G43 H8 Z10. F3000.', 'G1 G43 H7 Z10. F3000.'),
            text.replace('G1 G49 Z80. F3000.', 'G1 G49 Z70. F3000.'),
        ):
            self.assertNotEqual(broken, text)
            decoder = Fanuc0iMBDecoder(m, p)
            with self.assertRaises(ValueError):
                for line in broken.splitlines():
                    decoder.read_line(line, dict(id=1, operation='file', kind='cut'))
        # A plan that starts the second tool where the table pose of another
        # machine would put it is not what this machine does.
        moved = tuple(replace(e, position_after_mm=(10, 0, 20)) if e == p.events[5] else e for e in p.events)
        with self.assertRaises(ValueError):
            list(post_fanuc(replace(p, events=moved), m))

    def test_gauge_pose_and_measured_positions_must_not_disagree(self):
        p, first, second = self.gauge_pose_job()
        one = ToolSetting.for_tool(first, number=2, length_register=7)
        two = ToolSetting.for_tool(second, number=3, length_register=8)
        z_home = dict(tool_change_position_mode='z_home', tool_change_home_z_mm=380)
        # Neither declared: no measured positions and no gauge pose.
        with self.assertRaisesRegex(ValueError, 'position_after_mm.*z_home with tool_change_home_z_mm'):
            self.machine(tools=(one, two))
        with self.assertRaisesRegex(ValueError, 'tool_change_home_z_mm'):
            self.machine(tools=(one, two), tool_change_position_mode='z_home')
        with self.assertRaisesRegex(ValueError, 'z_home.*tool_change_home_z_mm.*table.*position_after_mm'):
            self.machine(tools=(one, two), tool_change_position_mode='preserve_tip')
        # Both declared and disagreeing: the stated tip is not the gauge line
        # less the gauge length (here a 70 mm tool entered as measured Z 20).
        wrong = replace(two, position_after_mm=(10, 0, 20))
        with self.assertRaisesRegex(ValueError, "'second'.*Z 20 .*gauge length 70.*Z 10"):
            self.machine(tools=(one, wrong), **z_home)
        # Both declared and agreeing: the same program.
        _, derived = self.machine(tools=(one, two), **z_home)
        _, both = self.machine(tools=(one, replace(two, position_after_mm=(10, 0, 10))), **z_home)
        self.assertEqual(''.join(c.text for c in post_fanuc(p, both)), ''.join(c.text for c in post_fanuc(p, derived)))
        # The derived tip must be on the output grid, as a measured one must.
        with self.assertRaisesRegex(ValueError, 'representable'):
            self.machine(tools=(one, two), tool_change_position_mode='z_home', tool_change_home_z_mm=380.0004)

    def test_allplane_multiturn_lowering_is_bounded_cancellable_and_attributed(self):
        p, m = self.machine()
        for plane, (u, v, a) in {'xy': (0, 1, 2), 'xz': (2, 0, 1), 'yz': (1, 2, 0)}.items():
            start = p.initial_position_mm
            end = list(start)
            end[a] -= 2
            center = list(start)
            center[u] -= 5
            motion = Motion(42, 'helix', 't1', Helix(start, tuple(end), tuple(center), -4 * math.tau, plane), 1200)
            job = replace(p, events=(*p.events[:3], motion))
            seen = []

            def events():
                for event in job.events:
                    seen.append(event)
                    yield event
                raise AssertionError('source consumed after cancellation')

            token = Cancellation()
            stream = post_events(
                job,
                events(),
                machine=m,
                post=Fanuc0iMBPost,
                decoder=Fanuc0iMBDecoder,
                lowering=LoweringOptions(),
                cancel=token,
            )
            next(stream)
            self.assertFalse(seen)
            for _ in range(4):
                chunk = next(stream)
            self.assertEqual(len(chunk.motions), 64)
            self.assertLessEqual(len(chunk.raw), 512 * 1024)
            self.assertTrue(all(x['source_motion_id'] == 42 for x in chunk.data['mappings']))
            token.cancel()
            with self.assertRaisesRegex(ValueError, 'cancelled'):
                next(stream)
            self.oracle(job, m, list(post_fanuc(job, m)))
            with self.assertRaisesRegex(ValueError, 'budget'):
                list(post_fanuc(job, m, lowering=LoweringOptions(max_segments_per_source=4)))

    def test_configuration_and_optional_stop_scope(self):
        for settings in (
            dict(startup_standard_modes=False),
            dict(g92_shift_mm=(1, 0, 0)),
            dict(startup_units='inch'),
            dict(program_number=9000),
            dict(tool_change_position_mode='preserve_tip'),
            dict(m_codes=dict(OEM, tool_change='M30')),
        ):
            with self.assertRaises(ValueError):
                self.machine(**settings)
        p, m = self.machine()
        job = replace(p, events=(*p.events, Stop(True)))
        with self.assertRaisesRegex(ValueError, 'optional'):
            list(post_fanuc(job, m))
        m = replace(m, optional_stop_enabled=True)
        self.oracle(job, m, list(post_fanuc(job, m)))
        bad = replace(m.tools[0], gauge_length_mm=60.0001)
        with self.assertRaisesRegex(ValueError, 'representable'):
            replace(m, tools=(bad,))

    def test_complete_pocket_posted_stock_and_literal_file(self):
        sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'examples'))
        try:
            from support.post_pocket import run
        finally:
            sys.path.pop(0)
        captured = {}

        def builder(header, tools):
            _, m = self.machine(tools=tools)
            m = replace(m, frame=header.frame)
            captured.update(machine=m, header=header)
            return m

        with tempfile.TemporaryDirectory() as directory:
            nc = Path(directory) / 'F0IMB1000.NC'
            result = run(
                nc,
                profile='python',
                lowering=True,
                radial_allowance_mm=0.01,
                post_type=Fanuc0iMBPost,
                decoder_type=Fanuc0iMBDecoder,
                machine_builder=builder,
            )
            m, p = captured['machine'], captured['header']
            oracle = Oracle(tools=m.tools, initial_position=p.initial_position_mm)
            decoder = Fanuc0iMBDecoder(m, p)
            count = 0
            for line in nc.read_text().splitlines():
                observed = oracle.read(line)
                actual = [
                    e
                    for e in decoder.read_line(line, dict(id=count + 1, operation='file', kind='cut'))
                    if isinstance(e, Motion)
                ]
                self.assertEqual(
                    [(e.tool, e.curve, e.feed_mm_min) for e in observed],
                    [(e.tool, e.curve, e.feed_mm_min) for e in actual],
                )
                count += len(actual)
            self.assertTrue(oracle.ended and decoder.ended)
        self.assertEqual(result['source_motions'], 352)
        self.assertEqual(count, result['motions'])
        self.assertEqual(result['posted_scene_stock_counts'], dict(clear=count, violation=0, uncertain=0))
        self.assertTrue(result['posted_material']['complete'])
        self.assertFalse(result['first_posted_motion']['source_finished'])
        self.assertFalse(result['first_posted_motion']['source_native_exited'])


if __name__ == '__main__':
    unittest.main()
