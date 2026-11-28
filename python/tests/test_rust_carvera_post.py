"""Carvera Python/Rust bytes, actual-word checks, stock replay and refusal."""

from dataclasses import replace
import json
import unittest

from camlib._buffers import native
from camlib.simulation import SimulationError
from camlib.motion import WorkFrame
from camlib.post import LoweringOptions, ToolSetting, PostEngine, post_job, post_job_input
from test_rust_post import ROOT, example, plain, python_release


def machine_for(job):
    header = job.compile().setup('top').program_header()
    tools = tuple(
        ToolSetting.for_tool(t, number=i, length_register=i, position_after_mm=header.initial_position_mm)
        for i, t in enumerate(header.tools, 1)
    )
    # Synthetic setup: clearance gauge Z equals the manufacturer's -3 mm.
    length = tools[0].gauge_length_mm
    frame = WorkFrame(origin_mm=(0, 0, -3 - header.initial_position_mm[2] - length))
    return example('carvera_post').define_machine(
        'synthetic Carvera',
        tools,
        initial_gauge_length_mm=length,
        unmodified_coordinates=True,
        automatic_tool_length=True,
        work_to_machine=frame,
        tool_change_position_mode='table',
        output_units='mm',
        decimals=3,
        max_feed_mm_min=10000,
        max_spindle_rpm=30000,
        max_tool_number=6,
        max_length_register=6,
        tool_changes=True,
        flood=False,
        mist=True,
        rapid_mm_min=6000,
        dwell_max_s=1e5,
    )


def drill_job(**options):
    from test_drill_cycles import job

    return job(coolant='mist', **options)


class RustCarveraPostTest(unittest.TestCase):
    def assert_job(self, job, machine=None, accept=('unchecked_machine',), lowering=None):
        c = example('carvera_post')
        m = machine or machine_for(job)
        py = python_release(job, m, set(accept), post=c.CarveraPost, decoder=c.CarveraDecoder, lowering=lowering)
        source = post_job_input(job, 'top', machine=m, accept=accept, lowering=lowering)
        rs = post_job(source)
        self.assertEqual(rs['released'], py['released'])
        self.assertEqual(rs['findings'], plain(py['findings']))
        self.assertEqual(rs['posting']['definition']['machine'], plain(m.to_dict()))
        self.assertEqual(rs['posting']['definition']['post_version'], c.CarveraPost.version)
        self.assertEqual(rs['posting']['definition']['decoder_version'], c.CarveraDecoder.version)
        if rs['released']:
            self.assertEqual(rs['program'], py['text'])
            self.assertEqual(rs['motions'], py['motions'])
        else:
            self.assertIsNone(rs['program'])
            self.assertEqual(rs['refused']['statuses'], py['statuses'])
        return rs, source

    def test_drilling_explicit_moves_and_gate(self):
        for options in ({}, dict(dwell_s=0.5), dict(peck_mm=1.5), dict(peck_mm=1.5, peck_mode='chip_break')):
            result, _ = self.assert_job(drill_job(**options))
            self.assertNotRegex(result['program'], r'G(?:8[0-9]|73|43|49|20|61|64)\b')
            self.assertIn('T1 M6\n', result['program'])
            self.assertIn('M7\n', result['program'])
            self.assertTrue(result['program'].endswith('G53 G0 Z-3\nM30\n'))
        self.assertFalse(self.assert_job(drill_job(), accept=())[0]['released'])

    def test_fixture(self):
        result, source = self.assert_job(drill_job(dwell_s=0.5))
        source = plain(source)
        source['manifest']['run']['id'] = 'carvera-drill-fixture'
        path = ROOT / 'fixtures/posts/rust/carvera-drill'
        self.assertEqual(source, json.loads(path.with_suffix('.json').read_text()))
        self.assertEqual(result['program'], path.with_suffix('.nc').read_text())

    def test_exact_native_arcs(self):
        from test_cutter_compensation import job
        from camlib.planar import PlanarRegion, Component, Boundary

        j = job(compensation='computer', coolant='mist')
        setup = j.setups[0]
        node = setup.operations[0]
        budget = 1 / 128
        region = PlanarRegion('round', (Component(Boundary.circle((16, 16), 7 - budget / 2)),), 0)
        node = replace(node, operation=replace(node.operation, region=region, boundary_tolerance_mm=budget))
        j = replace(j, setups=(replace(setup, operations=(node,)),))
        result, source = self.assert_job(j, lowering=LoweringOptions(native_arcs=True))
        self.assertGreater(result['posting']['arc_output']['native'], 0)
        for key in ('lines_policy', 'lines_machine', 'lines_precision'):
            self.assertEqual(result['posting']['arc_output'][key], 0)
        self.assertRegex(result['program'], r'(?m)^G[23] ')
        path = ROOT / 'fixtures/posts/rust/carvera-arcs'
        source = plain(source)
        source['manifest']['run']['id'] = 'carvera-arcs-fixture'
        self.assertEqual(source, json.loads(path.with_suffix('.json').read_text()))
        self.assertEqual(result['program'], path.with_suffix('.nc').read_text())

    def test_native_arcs_and_helices_in_all_planes(self):
        import math
        from camlib.motion import Arc, Helix, Line, Motion, Program, ToolChange, WorkOffset, Spindle
        from camlib.post import post_events

        c = example('carvera_post')
        j = drill_job()
        header = j.compile().setup('top').program_header()
        m = machine_for(j)
        start = (5, 5, 12)
        setting = replace(m.tools[0], position_after_mm=start)
        m = replace(m, tools=(setting,))
        for plane, center, axis, quarter_end in (
            ('xy', (0, 5, 12), 2, (0, 10, 12)),
            ('xz', (5, 5, 7), 1, (10, 5, 7)),
            ('yz', (5, 0, 12), 0, (5, 0, 17)),
        ):
            for helical in (False, True):
                end = list(quarter_end)
                if helical:
                    end[axis] -= 1
                end = tuple(end)
                curve = (Helix if helical else Arc)(start, end, center, math.pi / 2, plane)
                p = Program(
                    'carvera-program',
                    '1',
                    header.tools,
                    start,
                    (
                        WorkOffset(1),
                        ToolChange(setting.tool, setting.number, setting.length_register, start),
                        Spindle(6000),
                        Motion(1, 'arc', setting.tool, curve, 300),
                        Motion(2, 'return', setting.tool, Line(end, (6, 5, 12)), 300),
                    ),
                )
                py = ''.join(
                    chunk.text for chunk in post_events(p, p.events, m, post=c.CarveraPost, decoder=c.CarveraDecoder)
                )
                source = dict(program=p.to_dict(), machine=m.to_dict())
                rs = json.loads(native.post_program(json.dumps(source)))
                self.assertEqual(rs['program'], py)
                self.assertIn('G3 ', py)
                self.assertNotIn('G43', py)
                self.assertEqual(py.count('G17\n'), 1)
                if plane == 'xy' and helical:
                    path = ROOT / 'fixtures/posts/rust/carvera-program'
                    self.assertEqual(plain(source), json.loads(path.with_suffix('.json').read_text()))
                    self.assertEqual(py, path.with_suffix('.nc').read_text())

    def test_reference_limits(self):
        import math

        m = machine_for(drill_job())
        for changes, message in (
            (dict(max_sweep_rad=math.pi + 0.01), '180'),
            (dict(max_tool_number=7), '1..6'),
            (dict(max_length_register=7), '1..6'),
        ):
            with self.subTest(changes=changes), self.assertRaisesRegex(ValueError, message):
                replace(m, **changes)
            source = plain(post_job_input(drill_job(), 'top', machine=m))
            source['machine'].update(changes)
            with self.assertRaisesRegex(ValueError, message):
                post_job(source)

    def test_air_requires_immediately_preceding_m400(self):
        c = example('carvera_post')
        j = drill_job()
        m = machine_for(j)
        header = j.compile().setup('top').program_header()
        d = c.CarveraDecoder(m, header)
        for line in ('G21', 'G90', 'G17', 'G94', 'M5'):
            d.read_line(line)
        with self.assertRaisesRegex(ValueError, 'M400'):
            d.read_line('M9')
        d.read_line('M400')
        d.read_line('M9')
        for line in ('M5', 'G17', '', 'M400', 'M30'):
            d = c.CarveraDecoder(m, header)
            d.read_line('M400')
            with self.subTest(line=line), self.assertRaisesRegex(ValueError, 'immediately'):
                d.read_line(line)
        for line in ('M7', 'M07', 'M9', 'M09', 'M400 P1'):
            d = c.CarveraDecoder(m, header)
            with self.subTest(line=line), self.assertRaises(ValueError):
                d.read_line(line)
        result, _ = self.assert_job(j)
        lines = result['program'].splitlines()
        for i, line in enumerate(lines):
            if line in ('M7', 'M9'):
                self.assertEqual(lines[i - 1], 'M400')
            if line == 'M400':
                self.assertIn(lines[i + 1], ('M7', 'M9'))

    def test_retract_at_posted_resolution(self):
        c = example('carvera_post')
        j = drill_job()
        header = j.compile().setup('top').program_header()
        m = replace(machine_for(j), work_to_machine=WorkFrame(origin_mm=(0, 0, -88.4)))
        header = replace(header, initial_position_mm=(0, 0, 25.4))
        self.assertNotEqual(25.4 - 88.4 + 60, -3)
        for delta, accepted in ((0, True), (0.00049, True), (0.00051, False), (-0.00051, False)):
            d = c.CarveraDecoder(m, replace(header, initial_position_mm=(0, 0, 25.4 + delta)))
            post = c.CarveraPost(m)
            for line in post._call('on_begin', header, limit=32):
                d.read_line(line)
            if accepted:
                d.read_line('G53 G0 Z-3')
            else:
                with self.assertRaisesRegex(ValueError, 'plan the retract'):
                    d.read_line('G53 G0 Z-3')

    def test_mutated_tool_word_is_caught_by_decode_and_compare(self):
        from camlib.motion import Program, WorkOffset, ToolChange
        from camlib.post import post_events

        c = example('carvera_post')
        j = drill_job()
        header = j.compile().setup('top').program_header()
        m = machine_for(j)
        other = replace(header.tools[0], id='other')
        m = replace(m, tools=(*m.tools, replace(m.tools[0], tool='other', number=2, length_register=2)))
        p = Program(
            'mutated',
            '1',
            (*header.tools, other),
            header.initial_position_mm,
            (WorkOffset(1), ToolChange(m.tools[0].tool, 1, 1, header.initial_position_mm)),
        )

        class Broken(c.CarveraPost):
            def on_tool_change(self, event):
                super().on_tool_change(replace(event, number=2))

        self.assertTrue(list(post_events(p, p.events, m, post=c.CarveraPost, decoder=c.CarveraDecoder)))
        with self.assertRaises(ValueError):
            list(post_events(p, p.events, m, post=Broken, decoder=c.CarveraDecoder))

    def test_plane_modes_and_planner_documentation(self):
        c = example('carvera_post')
        post = c.CarveraPost(machine_for(drill_job()))
        post._limit = 32
        for code in ('G17', 'G17', 'G18', 'G18', 'G19', 'G19', 'G17'):
            post.emit(code)
        self.assertEqual(post._lines, ['G17', 'G18', 'G19', 'G17'])

    def test_planner_premise_is_geometric_not_zero_corner_speed(self):
        doc = (ROOT / 'docs/carvera-post.md').read_text(encoding='utf-8')
        self.assertNotIn('minimum planner speed configured to zero', doc)
        self.assertIn('does not actually deviate', doc)

    def test_refusals(self):
        c = example('carvera_post')
        j = drill_job()
        m = machine_for(j)
        with self.assertRaisesRegex(ValueError, 'requires mm'):
            replace(m, output_units='inch')
        source = plain(post_job_input(j, 'top', machine=m))
        source['machine']['output_units'] = 'inch'
        with self.assertRaisesRegex(ValueError, 'requires mm'):
            post_job(source)
        for post, decoder in ((None, None), (c.CarveraPost, None), (None, c.CarveraDecoder)):
            kw = dict(post=post) if post else {}
            if decoder:
                kw['decoder'] = decoder
            with self.assertRaisesRegex(ValueError, 'requires matching post and decoder'):
                PostEngine(m, **kw)
        for change in (dict(unmodified_coordinates=False), dict(automatic_tool_length=False), dict(flood=True)):
            with self.assertRaises(ValueError):
                replace(m, **change)
        # Do not insert a G53 move outside the plan, even by micrometres.
        for m in (
            replace(m, initial_gauge_length_mm=m.initial_gauge_length_mm + 0.001),
            replace(m, work_to_machine=WorkFrame(origin_mm=(0, 0, 0))),
        ):
            with self.assertRaisesRegex(SimulationError, 'ValueError: .*plan the retract'):
                j.post_nc('top', machine=m, post=c.CarveraPost, decoder=c.CarveraDecoder)
            with self.assertRaisesRegex(ValueError, 'plan the retract'):
                post_job(post_job_input(j, 'top', machine=m))

    def test_decoder_rejects_unsupported_and_unretracted_end(self):
        c = example('carvera_post')
        j = drill_job()
        m = machine_for(j)
        header = j.compile().setup('top').program_header()

        def decoder():
            d = c.CarveraDecoder(m, header)
            for line in ('G21', 'G90', 'G17', 'G94', 'G54', 'M5', 'M400', 'M9'):
                d.read_line(line)
            return d

        for line in (
            'G20',
            'G43 H1',
            'G49',
            'G61',
            'G64',
            'G81 X1 Y1 Z0 R2 F100',
            'M8',
            'M4 S1000',
            'M0',
            'm06',
            'h1',
            't1',
            'M+6',
            'M030',
            'G53 G0 Z-3\n',
            'M1',
            'M6',
            'T1 M6 C1',
            'T1',
            'H1',
            'M30',
            'G53 Z-3',
            'G53 G0 Z-2',
        ):
            with self.subTest(line=line), self.assertRaises(ValueError):
                decoder().read_line(line)
        d = decoder()
        d.read_line('G53 G0 Z-3')
        d.read_line('M30')
        with self.assertRaises(ValueError):
            d.read_line('G21')
