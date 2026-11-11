from dataclasses import replace
import math
import re
import json
from pathlib import Path
import sys
import unittest

from camlib.motion import WorkOffset, WorkFrame, ToolChange, Spindle, Motion, Line, Arc, Helix
from camlib.post import (
    Machine,
    GCodeDialect,
    PostEngine,
    ToolSetting,
    Cancellation,
    LoweringOptions,
)
from test_post import fixture
from test_tapping import CHANGE, job as tap_job, tapping, tool_table as tap_table
from siemens808d_oracle import Oracle

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'examples'))
try:
    from siemens808d_post import define_machine, Siemens808DPost, Siemens808DDecoder, post_808d, post_events_808d
finally:
    sys.path.pop(0)

OEM = dict(off='M9', flood='M8', mist='M7')  # explicit test PLC bindings, not inferred


class Siemens808D(unittest.TestCase):
    def machine(self, **settings):
        p, s = fixture()
        return p, define_machine(
            'Test OEM 808D',
            s.tools,
            tool_change=settings.pop('tool_change', 'm6'),
            coolant_codes=settings.pop('coolant_codes', OEM),
            work_to_machine=WorkFrame(origin_mm=(100, 200, 300)),
            **{
                'max_feed_mm_min': 10000,
                'max_spindle_rpm': 30000,
                'max_sweep_rad': 6.283185307179586,
                'tool_changes': True,
                'tool_change_position_mode': 'table',
                'rapid_mm_min': 6000,
                'output_units': (settings).get('output_units', 'mm'),
                'decimals': 3 if (settings).get('output_units', 'mm') == 'mm' else 4,
                **(settings),
            },
        )

    def oracle(self, p, m, chunks):
        oracle = Oracle(
            tools=m.tools,
            initial_position=p.initial_position_mm,
            tool_change=m.dialect.tool_change,
            coolant_codes=OEM,
            stop_window=m.stop_window,
            work_offset=m.work_offset,
        )
        observed = []
        for chunk in chunks:
            for line in chunk.text.splitlines():
                observed.extend(e for e in oracle.read(line) if isinstance(e, Motion))
        actual = [motion for chunk in chunks for motion in chunk.motions]
        self.assertTrue(oracle.ended)
        self.assertEqual(len(observed), len(actual))
        for x, y in zip(observed, actual):
            self.assertEqual(x.tool, y.tool)
            self.assertEqual(x.curve, y.curve)
            self.assertEqual(x.feed_mm_min, y.feed_mm_min)
            self.assertEqual(x.kind == 'rapid', y.kind == 'rapid')
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
        chunks = list(post_808d(job, m))
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

    def test_golden_start_tool_end_and_literal_independent_interpretation(self):
        expected = [
            'G290',
            'COMPOF',
            'G710',
            'G90',
            'G17',
            'G94',
            'G40',
            'D0',
            'MCALL',
            'TRANS',
            'G60',
            'M5',
            'M9',
            'G601',
            'RNDM=0',
            'CFTCP',
        ]
        for mode in ('m6', 'direct_t'):
            p, m = self.machine(tool_change=mode)
            chunks = list(post_808d(p, m, run_id='native-reference'))
            self.assertEqual(chunks[0].text.splitlines(), expected)
            text = ''.join(c.text for c in chunks)
            self.assertIn('T2\nM6 D7\n' if mode == 'm6' else 'T2 D7\n', text)
            self.assertTrue(text.endswith('M1\nM9\nD0\nM2\n'))  # spindle already stopped by its event; coolant still on
            for code in ('G21', 'G20', 'G91.1', 'G97', 'G49', 'G43', 'G80', 'G92.1', 'G61.1'):
                self.assertNotIn(code + '\n', text)
            self.assertEqual(chunks[0].data['identity']['abi'], 6)
            self.assertIn('4.90', chunks[0].data['definition']['machine']['reference'])
            self.oracle(p, m, chunks)

    def test_complete_native_golden_file_matches_literal_manual_geometry(self):
        p, m = self.machine(curve_policy='exact_ijk')
        points = ((5, 0, 3), (0, 5, 3), (5, 0, 3), (5, 0, 3), (5, 0, 2))
        curves = (
            Arc(points[0], points[1], (0, 0, 3), math.pi / 2),
            Arc(points[1], points[2], (0, 0, 3), -math.pi / 2),
            Arc(points[2], points[3], (0, 0, 3), math.tau),
            Helix(points[3], points[4], (0, 0, 3), -math.tau),
        )
        moves = tuple(
            Motion(i + 1, 'native-golden', 't1', curve, 300 if isinstance(curve, Helix) else 1200)
            for i, curve in enumerate(curves)
        )
        job = replace(p, events=(*p.events[:3], *moves, Spindle(0)))
        chunks = list(post_808d(job, m))
        expected = (Path(__file__).resolve().parents[2] / 'fixtures/posts/EXACT490.MPF').read_text()
        self.assertEqual(''.join(c.text for c in chunks), expected)
        self.oracle(job, m, chunks)

    def test_native_exact_circles_helix_units_and_coarse_stop_window(self):
        p, _ = fixture()
        for units in ('mm', 'inch'):
            start = (0.2 * 25.4, 0.0, 0.1 * 25.4) if units == 'inch' else p.initial_position_mm
            radius = 5.08 if units == 'inch' else 5
            center = (start[0] - radius, 0, start[2])
            p0, m = self.machine(curve_policy='exact_ijk', output_units=units, stop_window='coarse')
            m = replace(m, tools=(replace(m.tools[0], position_after_mm=start),))
            controls = list(p0.events[:3])
            controls[1] = replace(controls[1], position_after_mm=start)
            for sign in (-1, 1):
                for full in (False, True):
                    end = (start[0], 0, start[2]) if full else (center[0], sign * radius, start[2])
                    arc = Motion(
                        42, 'circle', 't1', Arc(start, end, center, sign * (math.tau if full else math.pi / 2)), 1200
                    )
                    he = (end[0], end[1], end[2] - (0.02 * 25.4 if units == 'inch' else 0.5))
                    hcenter = (center[0], center[1], end[2])
                    helix = Motion(43, 'helix', 't1', Helix(end, he, hcenter, sign * math.tau), 300, 'plunge')
                    job = replace(p0, initial_position_mm=start, events=(*controls, arc, helix))
                    chunks = list(post_808d(job, m))
                    self.assertEqual(sum(len(c.motions) for c in chunks), 2)
                    text = ''.join(c.text for c in chunks)
                    self.assertIn('G602\n', text)
                    self.assertIn('TURN=0', text)
                    self.assertIn('G700\n' if units == 'inch' else 'G710\n', text)
                    self.oracle(job, m, chunks)

    def test_nonzero_stored_offset_and_native_mode_reselection_preserve_tool_state(self):
        p, m = self.machine(curve_policy='exact_ijk', work_offset=2)
        events = list(p.events)
        events[0] = WorkOffset(2)
        events[2] = Spindle(12000, 'ccw')
        job = replace(p, events=tuple(events))
        chunks = list(post_808d(job, m))
        text = ''.join(c.text for c in chunks)
        self.assertIn('G55\n', text)
        self.assertIn('M4 S12000\n', text)
        self.oracle(job, m, chunks)
        decoder = Siemens808DDecoder(m, job)
        post = Siemens808DPost(m)
        decoder.read(post._call('on_begin', job, limit=32))
        for event in job.events[:3]:
            decoder.read(post._call('on_' + event.to_dict()['kind'], event))
        before = (decoder.active.tool, decoder.selected, decoder.length, decoder.offset, decoder.position)
        decoder.read_line('G290')
        self.assertEqual(
            (decoder.active.tool, decoder.selected, decoder.length, decoder.offset, decoder.position), before
        )
        for body in ('G291', 'G290 G710'):
            with self.assertRaises(ValueError):
                decoder.read_line(body)
            self.assertEqual(
                (decoder.active.tool, decoder.selected, decoder.length, decoder.offset, decoder.position), before
            )

    def test_multiple_edges_share_T_with_edge_only_selection_and_D1_across_tools(self):
        p, m = self.machine()
        edge2 = replace(
            p.tools[0], id='edge2', gauge_length_mm=61, holders=(replace(p.tools[0].holders[0], z_max_mm=61),)
        )
        tool2 = replace(p.tools[0], id='other-tool')
        poses = (p.initial_position_mm, (10, 0, 4), (11, 0, 3))
        tools = (
            ToolSetting.for_tool(p.tools[0], number=5, length_register=1, position_after_mm=poses[0]),
            ToolSetting.for_tool(edge2, number=5, length_register=2, position_after_mm=poses[1]),
            ToolSetting.for_tool(tool2, number=6, length_register=1, position_after_mm=poses[2]),
        )
        m = replace(m, tools=tools)
        events = [WorkOffset(1)]
        for i, (assembly, setting, end) in enumerate(
            zip((p.tools[0], edge2, tool2), tools, ((10, 0, 3), (11, 0, 4), (12, 0, 3)))
        ):
            if i:
                events.append(Spindle(0))
            events.extend(
                (
                    ToolChange(assembly.id, setting.number, setting.length_register, setting.position_after_mm),
                    Spindle(1200),
                    Motion(i + 1, 'edges', assembly.id, Line(setting.position_after_mm, end), 600),
                )
            )
        job = replace(p, tools=(p.tools[0], edge2, tool2), events=tuple(events))
        chunks = list(post_808d(job, m))
        text = ''.join(c.text for c in chunks)
        self.assertEqual(text.count('T5\n'), 1)
        self.assertEqual(text.count('M6 D1\n'), 2)
        self.assertIn('M5\nD2\nM3 S1200', text)
        self.assertEqual([x.tool for c in chunks for x in c.motions], ['t1', 'edge2', 'other-tool'])
        self.oracle(job, m, chunks)
        with self.assertRaises(ValueError):
            replace(m, tools=(*tools, replace(tools[1], tool='duplicate')))
        with self.assertRaises(ValueError):
            Machine(
                'global-H',
                tools,
                dialect=GCodeDialect(),
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

    def test_unmodeled_circle_correction_rejects_and_default_bounded_curve_path_works(self):
        p, m = self.machine(curve_policy='exact_ijk')
        start = p.initial_position_mm
        center = (0.12345, 0.23456, start[2])
        x = start[0] - center[0]
        y = start[1] - center[1]
        angle = math.pi / 3
        end = (
            center[0] + x * math.cos(angle) - y * math.sin(angle),
            center[1] + x * math.sin(angle) + y * math.cos(angle),
            start[2],
        )
        arc = Motion(42, 'rounded', 't1', Arc(start, end, center, angle), 1200)
        job = replace(p, events=(*p.events[:3], arc))
        # Zero radius tolerance: any emitted circle must have EXACTLY equal
        # start/end radii in the decimal words themselves. (The word search
        # finds such words for this arc; arcs without them are rejected, see
        # post_rounding::tests in Rust.)
        from decimal import Decimal

        exact = list(post_808d(job, m))
        blocks = [item for c in exact for item in c.text.splitlines() if item.startswith(('G2 ', 'G3 '))]
        self.assertEqual(len(blocks), 1)
        w = {k: Decimal(v) for k, v in re.findall(r'([XYIJ])(-?[0-9.]+)', blocks[0])}
        sx, sy = (Decimal(repr(v)) for v in start[:2])
        cx, cy = sx + w['I'], sy + w['J']
        self.assertEqual((sx - cx) ** 2 + (sy - cy) ** 2, (w['X'] - cx) ** 2 + (w['Y'] - cy) ** 2)
        self.assertTrue(
            all(
                x['deviation_bound_mm'] is None or x['deviation_bound_mm'] <= 0.01
                for c in exact
                for x in c.data['checked']
            )
        )
        _, m = self.machine()
        chunks = list(post_808d(job, m))
        moves = [x for c in chunks for x in c.motions]
        self.assertGreater(len(moves), 1)
        self.assertTrue(all(isinstance(x.curve, Line) for x in moves))
        self.assertTrue(all((x['deviation_bound_mm'] or 0) <= 0.01 for c in chunks for x in c.data['checked']))
        self.oracle(job, m, chunks)

    def test_startup_omissions_and_hostile_controller_semantics_are_rejected(self):
        p, m = self.machine(curve_policy='exact_ijk')
        for removed in ('G290', 'G710', 'G17', 'MCALL', 'TRANS', 'COMPOF', 'G60', 'G601', 'RNDM=0', 'CFTCP', 'D0'):

            class Missing(Siemens808DPost):
                def on_begin(self, program):
                    super().on_begin(program)
                    self._lines.remove(removed)

            with self.assertRaises(ValueError, msg=removed):
                PostEngine(m, Missing, decoder=Siemens808DDecoder).begin(p)
        hostile = (
            'G291',
            'G290 G710',
            'G70',
            'G71',
            'G18',
            'G19',
            'G43 H7',
            'G49',
            'G80',
            'G64',
            'TRANS X1',
            'ATRANS X1',
            'ROT RPL=1',
            'SCALE X2',
            'COMPCAD',
            'CFC',
            'RNDM=.01',
            'G1 X10 Y0 Z3 F1200 G291',
            'G2 X10 Y0 CR=5 F1200',
            'G2 X10 Y0 I=AC(0) J0 F1200',
        )
        for body in hostile:

            class Bad(Siemens808DPost):
                def on_motion(self, motion):
                    self.emit(body)

            e = PostEngine(m, Bad, decoder=Siemens808DDecoder)
            e.begin(p)
            with self.assertRaises(ValueError, msg=body):
                e.consume(p.events)
            self.assertEqual(e.position_mm, p.initial_position_mm)
        with self.assertRaises(ValueError):
            replace(m, dialect=m.dialect.with_commands(units_mm='G71'))
        with self.assertRaises(ValueError):
            replace(m, planes=('xy', 'xz'))
        with self.assertRaises(ValueError):
            replace(m, tools=(replace(m.tools[0], length_register=10),))
        with self.assertRaises(ValueError):
            replace(m, work_to_machine=WorkFrame(x=(1, 0, 0), y=(0, 0, 1), z=(0, -1, 0)))

    def test_actual_T_D_corruption_rolls_back_and_explicit_OEM_configuration_is_required(self):
        p, m = self.machine(curve_policy='exact_ijk')

        class Wrong(Siemens808DPost):
            def __init__(self, machine):
                super().__init__(machine)
                self.fail = True

            def on_tool_change(self, event):
                super().on_tool_change(event)
                if self.fail:
                    self._lines[-1] = self._lines[-1].replace('D7', 'D8')

        e = PostEngine(m, Wrong, decoder=Siemens808DDecoder)
        e.begin(p)
        with self.assertRaisesRegex(ValueError, 'edge'):
            e.consume(p.events)
        self.assertIsNone(e.post._active_number)
        e.post.fail = False
        chunk = e.consume(p.events)
        self.assertEqual(chunk.motions, (p.events[4],))
        e.finish()
        with self.assertRaises(TypeError):
            define_machine('missing OEM inputs', m.tools)
        for args in (dict(coolant_codes={}), dict(coolant_codes=OEM, tool_change='guess')):
            with self.assertRaises(ValueError):
                define_machine('bad', m.tools, work_to_machine=WorkFrame(), **({'tool_change': 'm6'} | args))
        for code in ('M17', 'M19', 'M30', 'M40', 'M45', 'M70'):
            with self.assertRaisesRegex(ValueError, 'reserved'):
                self.machine(coolant_codes={'off': code})
        with self.assertRaisesRegex(ValueError, 'reserved'):
            replace(m, dialect=m.dialect.with_commands(coolant_off='M30'))
        with self.assertRaises(ValueError):
            replace(m, dialect=m.dialect.with_commands(rpm_mode='G97'))
        with self.assertRaises(ValueError):
            replace(m, max_spindle_rpm=100000)
        with self.assertRaises(ValueError):
            replace(m, max_feed_mm_min=100000)
        _, dry = self.machine(coolant_codes={'off': 'M9'})
        with self.assertRaises(ValueError):
            list(post_808d(p, dry))

    def test_software_coordinate_range_is_checked_on_formatted_and_actual_words(self):
        p, m = self.machine(curve_policy='exact_ijk')
        move = replace(p.events[4], curve=Line(p.initial_position_mm, (100000, 0, 3)))
        job = replace(p, events=(*p.events[:4], move))
        with self.assertRaisesRegex(ValueError, 'range'):
            list(post_808d(job, m))

        class Raw(Siemens808DPost):
            def on_motion(self, motion):
                self.emit('G1 X100000 Y0 Z3 F1200')

        engine = PostEngine(m, Raw, decoder=Siemens808DDecoder)
        engine.begin(job)
        with self.assertRaisesRegex(ValueError, 'decoded native reference coordinate range'):
            engine.consume(job.events)
        self.assertEqual(engine.position_mm, p.initial_position_mm)

    def test_multiturn_non_G17_source_streams_lowered_with_caps_cancel_and_maps(self):
        p, m = self.machine()
        start = p.initial_position_mm
        end = (start[0], start[1] - 2, start[2])
        center = (start[0], start[1], start[2] - 5)
        helix = Motion(42, 'multiturn', 't1', Helix(start, end, center, 4 * math.tau, 'xz'), 1200)
        job = replace(p, events=(*p.events[:3], helix))
        seen = []

        def events():
            for event in job.events:
                seen.append(event)
                yield event
            raise AssertionError('must not request more source after cancelled partial curve')

        token = Cancellation()
        stream = post_events_808d(job, events(), m, cancel=token)
        next(stream)
        self.assertFalse(seen)
        for _ in range(4):
            chunk = next(stream)
        self.assertEqual(len(seen), 4)
        self.assertEqual(len(chunk.motions), 64)
        self.assertLess(chunk.data['progress']['pending_source_fraction'], 1)
        self.assertTrue(all(x['source_motion_id'] == 42 for x in chunk.data['mappings']))
        token.cancel()
        with self.assertRaisesRegex(ValueError, 'cancelled'):
            next(stream)
        chunks = list(post_808d(job, m))
        self.oracle(job, m, chunks)
        with self.assertRaisesRegex(ValueError, 'budget'):
            list(post_808d(job, m, lowering=LoweringOptions(max_segments_per_source=4)))

    def test_real_cadquery_pocket_posts_all_source_and_checks_decoded_stock_and_literal_NC(self):
        import tempfile

        sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'examples'))
        try:
            from support.post_pocket import run
        finally:
            sys.path.pop(0)

        def builder(header, tools):
            return define_machine(
                'Bound test OEM native808D',
                tools,
                tool_change='m6',
                coolant_codes=OEM,
                work_to_machine=WorkFrame(origin_mm=(100, 200, 300)),
                frame=header.frame,
                output_units='mm',
                decimals=3,
                max_feed_mm_min=10000,
                max_spindle_rpm=30000,
                max_sweep_rad=6.283185307179586,
                tool_changes=True,
                tool_change_position_mode='table',
                rapid_mm_min=6000,
            )

        with tempfile.TemporaryDirectory() as directory:
            nc = Path(directory) / 'pocket.mpf'
            result = run(
                nc,
                profile='python',
                lowering=True,
                radial_allowance_mm=0.01,
                post_type=Siemens808DPost,
                decoder_type=Siemens808DDecoder,
                machine_builder=builder,
            )
            text = nc.read_text()
            settings = result['machine']
            tools = tuple(
                ToolSetting(**{**item, 'position_after_mm': tuple(item['position_after_mm'])})
                for item in settings['tools']
            )
            oracle = Oracle(
                tools=tools, initial_position=tools[0].position_after_mm, tool_change='m6', coolant_codes=OEM
            )
            count = 0
            for line in text.splitlines():
                count += sum(isinstance(e, Motion) for e in oracle.read(line))
        self.assertTrue(oracle.ended)
        self.assertEqual(count, result['motions'])
        self.assertEqual(result['source_motions'], 352)
        self.assertGreater(result['motions'], 352)
        self.assertEqual(result['posted_scene_stock_counts'], dict(clear=result['motions'], violation=0, uncertain=0))
        self.assertTrue(result['posted_material']['complete'])
        self.assertGreater(result['posted_material']['residual']['definite_cells'], 0)
        self.assertFalse(result['first_posted_motion']['source_finished'])
        self.assertFalse(result['first_posted_motion']['source_native_exited'])
        self.assertIn('siemens808d_post.Siemens808DPost', result['definition']['post'])
        self.assertIn('examples/siemens808d_post.py', result['source_sha256'])


def tap_machine(j, **settings):
    values = dict(
        tool_change='m6',
        coolant_codes=OEM,
        work_to_machine=WorkFrame(),
        max_feed_mm_min=10000,
        max_spindle_rpm=30000,
        max_sweep_rad=6.283185307179586,
        tool_changes=True,
        tool_change_position_mode='table',
        rapid_mm_min=6000,
        output_units='mm',
        decimals=3,
        position_controlled_spindle=True,
        tap_spindle_position_deg=90,
    )
    values.update(settings)
    return define_machine('Test OEM 808D', tap_table(j), **values)


def tap_nc(hand='right', post=Siemens808DPost, **settings):
    j = tap_job(lambda tap: tapping(tap, hand=hand), tool_change=CHANGE)
    m = tap_machine(j, **settings)
    text = j.post_nc('top', machine=m, post=post, decoder=Siemens808DDecoder, accept={'unchecked_machine'}).text
    return j, m, text.splitlines()


def cycle84_with(index, change):
    """A deliberately wrong post: one CYCLE84 parameter rewritten."""

    class Wrong(Siemens808DPost):
        def emit(self, *words):
            if len(words) == 1 and str(words[0]).startswith('CYCLE84('):
                fields = words[0][8:-1].split(',')
                fields[index] = change(fields[index])
                words = ('CYCLE84(' + ','.join(fields) + ')',)
            return super().emit(*words)

    return Wrong


class Siemens808DTapping(unittest.TestCase):
    """#95: CYCLE84 rigid tapping (07/2018 manual pp.166-169), both hands."""

    def test_golden_cycle84_both_hands_and_independent_expansion(self):
        for hand, sdac, pit, spindle in (('right', '3', '1', 'M3'), ('left', '4', '-1', 'M4')):
            with self.subTest(hand):
                j, m, nc = tap_nc(hand)
                cycles = [i for i, line in enumerate(nc) if line.startswith('CYCLE84')]
                # RTP = RFP = the current Z, no SDIS; DP absolute; SDAC and the
                # sign of PIT are the hand; POSS 90; SST = SST1 = 600; AXN 3.
                self.assertEqual(
                    [nc[i : i + 2] for i in cycles],
                    [[f'CYCLE84(11,11,,4,,,{sdac},,{pit},90,600,600,3,0,0,0)', '; CYCLE84 retracted']] * 2,
                )
                for i in cycles:
                    # The hole position is approached before the call (p.156).
                    self.assertRegex(nc[i - 1], r'^G[01] .*Z11\b')
                    turning = next(line for line in reversed(nc[:i]) if re.match(r'M[345]\b', line))
                    self.assertEqual(turning, f'{spindle} S600')
                self.assertFalse(any(re.search(r'G33|G63|SPOS', line) for line in nc))
                header = j.compile().setup('top').program_header()
                oracle = Oracle(
                    tools=m.tools,
                    initial_position=header.initial_position_mm,
                    tool_change='m6',
                    coolant_codes=OEM,
                    spindle_position_deg=90,
                )
                taps = [e for line in nc for e in oracle.read(line) if isinstance(e, Motion) and e.kind == 'tap']
                self.assertTrue(oracle.ended)
                m_function = 3 if hand == 'right' else 4
                self.assertEqual(
                    oracle.taps,
                    [(hand, m_function, (x, 8.0, 11.0), (x, 8.0, 4.0), 600.0, 600.0) for x in (8.0, 20.0)],
                )
                # Down and back for each hole, and the spindle left as it ran.
                self.assertEqual(
                    [(e.curve.start[2], e.curve.end[2], e.feed_mm_min) for e in taps],
                    [(11.0, 4.0, 600.0), (4.0, 11.0, 600.0)] * 2,
                )

    def test_wrong_hand_or_direction_is_refused(self):
        def flipped(text):
            return text[1:] if text.startswith('-') else '-' + text

        for hand in ('right', 'left'):
            with self.subTest(hand):
                # A lead of the other sign taps the other hand (p.168).
                with self.assertRaisesRegex(Exception, 'lead sign .thread hand. differs from the running spindle'):
                    tap_nc(hand, post=cycle84_with(8, flipped))
                # SDAC leaving the spindle reversed after the cycle (p.167).
                with self.assertRaisesRegex(Exception, 'SDAC must restore the running spindle direction'):
                    tap_nc(hand, post=cycle84_with(6, lambda v: {'3': '4', '4': '3'}[v]))

        class Reversed(Siemens808DPost):
            # The spindle started the wrong way round, the cycle agreeing with it.
            def on_spindle(self, event):
                super().on_spindle(replace(event, direction='ccw') if event.rpm else event)

        # The intent check: the decoded spindle direction is not the planned one.
        with self.assertRaisesRegex(Exception, 'decoded effects differ from source'):
            tap_nc('right', post=Reversed)

    def test_wrong_cycle_parameters_are_refused(self):
        for index, change, message in (
            (0, lambda v: '12', 'RTP and RFP must be the current Z'),
            (1, lambda v: '10', 'RTP and RFP must be the current Z'),
            (2, lambda v: '2', 'no SDIS, DPR, DTB or MPIT'),
            (4, lambda v: '7', 'no SDIS, DPR, DTB or MPIT'),
            (7, lambda v: '6', 'no SDIS, DPR, DTB or MPIT'),
            (3, lambda v: '12', 'final depth DP below'),
            (9, lambda v: '0', 'POSS differs from the declared'),
            (10, lambda v: '500', 'SST and SST1 must repeat the running spindle speed'),
            (11, lambda v: '0', 'SST and SST1 must repeat the running spindle speed'),
            (12, lambda v: '1', 'AXN=3'),
            (15, lambda v: '1', 'VARI=0'),
            (8, lambda v: '0', 'PIT must be 0.001..2000'),
            (8, lambda v: 'R1', 'plain numbers'),
        ):
            with self.subTest(index=index, message=message), self.assertRaisesRegex(Exception, message):
                tap_nc(post=cycle84_with(index, change))

        class Trailing(Siemens808DPost):
            def tap(self, motion):
                super().tap(motion)
                if motion.curve.end[2] > motion.curve.start[2]:
                    self.emit('G1 Z11 F100')

        with self.assertRaisesRegex(Exception, 'CYCLE84 retracts by itself'):
            tap_nc(post=Trailing)

    def test_undeclared_premises_refuse_tapping(self):
        for settings, message in (
            (dict(position_controlled_spindle=False, tap_spindle_position_deg=None), 'position_controlled_spindle'),
            (dict(tap_spindle_position_deg=None), 'tap_spindle_position_deg'),
        ):
            with self.subTest(settings=settings), self.assertRaisesRegex(Exception, message):
                tap_nc(**settings)
        j = tap_job(tapping, tool_change=CHANGE)
        for settings in (
            dict(position_controlled_spindle=False),
            dict(position_controlled_spindle=1),
            dict(tap_spindle_position_deg=360),
            dict(tap_spindle_position_deg=-1),
            dict(tap_spindle_position_deg=True),
            dict(tap_spindle_position_deg=0.00001),
        ):
            with self.subTest(settings=settings), self.assertRaises(ValueError):
                tap_machine(j, **settings)
        # The manual gives the lead in mm only.
        with self.assertRaisesRegex(Exception, 'tapping needs output_units="mm"'):
            tap_nc(output_units='inch', decimals=4)


if __name__ == '__main__':
    unittest.main()
