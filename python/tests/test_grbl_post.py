"""GRBL 1.1h post (examples/grbl_post.py): GRBL's words, line size and arithmetic."""

from dataclasses import replace
import math
from pathlib import Path
import sys
import tempfile
import unittest

from camlib.motion import Arc, Coolant, Line, Motion, Spindle, Stop, ToolChange, WorkOffset
from camlib.post import LoweringOptions, ToolSetting, post_program
from camlib.simulation import Simulation, StockSpec
from test_post import fixture

EXAMPLES = Path(__file__).resolve().parents[2] / 'examples'
sys.path.insert(0, str(EXAMPLES))
try:
    from grbl_post import GrblDecoder, GrblPost, define_machine
finally:
    sys.path.remove(str(EXAMPLES))

ROUTER = dict(
    output_units='mm',
    decimals=6,
    tolerance_mm=0.001,
    feed_tolerance_mm_min=0.0001,
    max_feed_mm_min=10000,
    max_spindle_rpm=30000,
    rapid_mm_min=6000,
    flood=True,
    mist=False,
)


def grbl_fixture(**machine):
    """The post fixture as a GRBL job: tool 2 preloaded, no mist or M1."""
    p, m = fixture()
    events = [replace(p.events[1], length_register=1) if i == 1 else e for i, e in enumerate(p.events)]
    events = [e for e in events if not (getattr(e, 'mode', None) == 'mist' or isinstance(e, Stop))]
    p = replace(p, events=tuple(events))
    tool = ToolSetting.for_tool(p.tools[0], number=2, length_register=1, position_after_mm=p.initial_position_mm)
    g = define_machine(
        'GRBL router',
        tool,
        work_offset_machine_mm=(1000, -2000, 100),
        initial_machine_mm=(1005, -2000, 103),
        **{**ROUTER, **machine},
    )
    return p, g


def post(p, g, post_type=GrblPost, **options):
    return list(post_program(p, machine=g, post=post_type, decoder=GrblDecoder, **options))


class GrblPostTest(unittest.TestCase):
    def test_output_is_grbl_not_linuxcnc(self):
        p, g = grbl_fixture()
        chunks = post(p, g)
        text = ''.join(c.text for c in chunks)
        lines = text.splitlines()
        self.assertIn('G61', lines)
        self.assertIn('T2', lines)
        for code in ('M6', 'G43 H1', 'G97', 'G61.1'):
            self.assertNotIn(code, lines)
        self.assertTrue(all(len(line) <= 79 for line in lines))
        self.assertEqual(chunks[1].motions[0].curve.end, (10.0, 0.0, 3.0))
        self.assertTrue(text.endswith('M9\nG49\nM2\n'))

    def test_words_grbl_does_not_read_are_refused(self):
        p, g = grbl_fixture()
        for block in ('G61.1', 'T2 M6', 'G43 H1', 'G59.1', '(' + 'x' * 79 + ')'):

            class Bad(GrblPost):
                def on_tool_change(self, event, block=block):
                    self.emit('T2')
                    self.emit(block)

            with self.assertRaises(ValueError, msg=block):
                post(p, g, Bad)
        mist = replace(p, events=(*p.events, Coolant('mist')))
        with self.assertRaises(ValueError):
            post(mist, g)
        with self.assertRaisesRegex(ValueError, 'M1'):
            post(replace(p, events=(*p.events, Stop(True))), g)
        arc = Motion(2, 'pocket', 't1', Arc((10, 0, 3), (0, 10, 3), (0, 0, 3), math.pi / 2), 1200)
        program = replace(p, events=(*p.events[:5], arc))
        with self.assertRaises(ValueError):
            post(program, g)  # GRBL is posted with lines only
        lowered = post(program, g, lowering=LoweringOptions(chord_tolerance_mm=0.00025))
        text = ''.join(c.text for c in lowered)
        self.assertFalse(any(line.split()[0] in ('G2', 'G3') for line in text.splitlines() if line))

    def test_mm_and_inch_decode_grbl_binary32_endpoints(self):
        for units in ('mm', 'inch'):
            p, g = grbl_fixture(output_units=units, decimals=9, feed_tolerance_mm_min=0.01)
            events = (*p.events[:4], replace(p.events[4], curve=Line((5, 0, 3), (10.123456, 0, 3))))
            chunk = post(replace(p, events=events), g)[1]
            self.assertNotEqual(chunk.motions[0].curve.end[0], 10.123456)
            self.assertLess(chunk.data['checked'][-1]['deviation_bound_mm'], 0.001)
            # Eight digits at most: GRBL ignores the rest.
            self.assertTrue(
                all(
                    sum(ch.isdigit() for ch in word) <= 8
                    for line in chunk.text.splitlines()
                    for word in line.split()[1:]
                )
            )

    def test_binary32_start_initializes_downstream_simulation(self):
        p, _ = grbl_fixture()
        start = (5.1, 0.0, 3.0)
        p = replace(
            p,
            initial_position_mm=start,
            events=(
                WorkOffset(),
                ToolChange('t1', 2, 1, start),
                Spindle(12000),
                Motion(1, 'pocket', 't1', Line(start, (10.0, 0.0, 3.0)), 1200),
            ),
        )
        tool = ToolSetting.for_tool(p.tools[0], number=2, length_register=1, position_after_mm=start)
        g = define_machine(
            'GRBL router', tool, work_offset_machine_mm=(1000.0, 0, 0), initial_machine_mm=(1005.1, 0, 3), **ROUTER
        )
        chunks = post(p, g)
        actual = tuple(chunks[0].data['decoded_initial_position_mm'])
        self.assertNotEqual(actual, start)
        self.assertLess(math.dist(actual, start), g.tolerance_mm)
        self.assertEqual(chunks[1].motions[0].curve.start, actual)
        with Simulation(
            StockSpec((0, 0, 0), (20, 5, 2), 0.5), p.tools, revision='grbl-start', initial_position_mm=actual
        ) as sim:
            list(sim.motions(chunks[1].motions))
            sim.finish()
            self.assertEqual(sim.state.completed, 1)
        with self.assertRaisesRegex(ValueError, 'pose'):
            far = define_machine(
                'GRBL router', tool, work_offset_machine_mm=(1000.0, 0, 0), initial_machine_mm=(1006, 0, 3), **ROUTER
            )
            post(p, far)

    def test_example_linear_program_rechecks_actual_decoded_stock(self):
        sys.path.insert(0, str(EXAMPLES))
        try:
            from grbl_linear import run
        finally:
            sys.path.remove(str(EXAMPLES))
        with tempfile.TemporaryDirectory() as tmp:
            nc = Path(tmp) / 'grbl.ngc'
            result = run(nc)
            self.assertEqual(result['motions'], 20)
            self.assertEqual(result['posted_checks'], dict(clear=20, violation=0, uncertain=0))
            self.assertGreater(result['max_continuous_deviation_bound_mm'], 0)
            self.assertLess(result['max_continuous_deviation_bound_mm'], 0.001)
            self.assertEqual(result['source_heights_sha256'], result['posted_heights_sha256'])
            self.assertTrue(nc.read_text().endswith('M2\n'))

    def test_lowered_pocket_keeps_gouges_visible_and_allowance_reserves_post_error(self):
        from support.post_pocket import run

        with tempfile.TemporaryDirectory() as tmp:
            unsafe = run(Path(tmp) / 'unsafe.ngc', profile='grbl')
            self.assertEqual(unsafe['source_motions'], 400)
            self.assertGreater(unsafe['motions'], 400)
            self.assertGreater(unsafe['posted_scene_stock_counts']['violation'], 0)
            sample = unsafe['issue_samples'][0]
            self.assertEqual(sample['posted_motion_id'], sample['source_mapping']['posted_motion_id'])
            finding = sample['findings'][0]
            self.assertIsNotNone(finding.get('witness') or finding['finding']['witness'])
            reserved = run(Path(tmp) / 'reserved.ngc', profile='grbl', radial_allowance_mm=0.001)
            self.assertEqual(reserved['source_motions'], 400)
            self.assertEqual(
                reserved['posted_scene_stock_counts'], dict(clear=reserved['motions'], violation=0, uncertain=0)
            )
            self.assertTrue(reserved['posted_material']['complete'])


if __name__ == '__main__':
    unittest.main()
