from dataclasses import replace
import importlib.util
import math
from pathlib import Path
import sys
import unittest

from camlib.motion import Arc, Coolant, Helix, Motion, Spindle, WorkFrame
from camlib.post import ToolSetting
from test_post import fixture

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'examples'))
try:
    from centroid_post import CentroidDecoder, CentroidPost, define_machine, post_centroid
finally:
    sys.path.pop(0)


def machine(p, s, **settings):
    return define_machine(
        'Acorn test mill',
        s.tools,
        traverse_feed_mm_min=3000,
        work_to_machine=WorkFrame(origin_mm=(100, 200, 300)),
        decimals=settings.pop('decimals', 3),
        **{
            'output_units': 'mm',
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
            'rapid_mm_min': 6000,
            **(settings),
        },
    )


class CentroidPostTest(unittest.TestCase):
    def test_program_structure_follows_the_manual(self):
        p, s = fixture()
        p = replace(
            p,
            events=(
                *p.events[:5],
                Motion(2, 'pocket', 't1', Arc((10, 0, 3), (10, 0, 3), (12, 0, 3), -math.tau), 600),
                Motion(3, 'pocket', 't1', Helix((10, 0, 3), (10, 0, 2), (12, 0, 3), math.tau), 600),
                Coolant('off'),
                Spindle(0),
            ),
        )
        m = machine(p, s)
        chunks = list(post_centroid(p, m))
        text = ''.join(c.text for c in chunks)
        lines = text.splitlines()
        self.assertIn('G61', lines)  # decelerate/stop: smoothing off
        self.assertIn('G52 X0 Y0 Z0', lines)
        self.assertFalse(any(item.startswith('G0 ') or item == 'G0' for item in lines))
        self.assertEqual(lines[-1], 'M30')
        self.assertNotIn('M2', lines)
        self.assertEqual(lines[lines.index('T2') + 1], 'M6')
        self.assertIn('G43 H7', lines)
        self.assertTrue(all('(' not in item for item in lines))
        # The decoded motions are exactly the source motions (lines lowered).
        decoded = [v for c in chunks for v in c.motions]
        self.assertGreater(len(decoded), 3)
        self.assertEqual(decoded[0].curve.end, (10, 0, 3))

    def test_the_same_program_in_inches(self):
        # A control that runs in inches: G20, four decimals, and the decoded
        # motions are still the source's within the machine's tolerance.
        p, s = fixture()
        m = machine(p, s, output_units='inch', decimals=4)
        chunks = list(post_centroid(p, m))
        lines = ''.join(c.text for c in chunks).splitlines()
        self.assertIn('G20', lines)
        self.assertNotIn('G21', lines)
        metric = [v for c in post_centroid(p, machine(p, s)) for v in c.motions]
        decoded = [v for c in chunks for v in c.motions]
        self.assertEqual(len(decoded), len(metric))
        for a, b in zip(decoded, metric):
            self.assertLess(max(abs(x - y) for x, y in zip(a.curve.end, b.curve.end)), m.tolerance_mm)
        # Words are inches: nothing as large as the millimetre values.
        xs = [
            float(w[1:]) for line in lines for w in line.split(' ') if w[:1] == 'X' and line[:2] in ('G1', 'G2', 'G3')
        ]
        mm = [v.curve.end[0] for v in metric]
        self.assertLess(max(xs), max(mm) / 20)

    def test_rigid_tapping_writes_q_lead_on_every_line(self):
        spec = importlib.util.spec_from_file_location('tapping_test', ROOT / 'python/tests/test_tapping.py')
        tapping = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(tapping)
        job = tapping.job(tapping.tapping)
        header = job.compile().setup('top').program_header()
        # CNC12 M6 moves Z home: the table states each tool's tip pose after
        # M6. Here: the drill at the start pose, the tap above the last hole.
        poses = (header.initial_position_mm, (20, 8, 12))
        tools = tuple(
            ToolSetting.for_tool(t, number=i, length_register=i, position_after_mm=pose)
            for i, (t, pose) in enumerate(zip(header.tools, poses), 1)
        )
        m = define_machine(
            'Acorn',
            tools,
            traverse_feed_mm_min=3000,
            work_to_machine=WorkFrame(),
            decimals=3,
            rigid_tapping=True,
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
            rapid_mm_min=6000,
        )
        nc = tapping.posted_text(job, m, CentroidPost, CentroidDecoder)
        cycles = [i for i, line in enumerate(nc) if line.startswith('G84')]
        self.assertEqual([nc[i] for i in cycles], ['G84 X8 Y8 Z4 R11 Q1.', 'G84 X20 Y8 Z4 R11 Q1.'])
        for i in cycles:
            self.assertEqual(nc[i + 1], 'G80')
        plain = define_machine(
            'Acorn',
            tools,
            traverse_feed_mm_min=3000,
            work_to_machine=WorkFrame(),
            decimals=3,
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
            rapid_mm_min=6000,
        )
        with self.assertRaisesRegex(Exception, 'declares no tapping cycle'):
            tapping.posted_text(job, plain, CentroidPost, CentroidDecoder)

    def test_decoder_refuses_dogleg_restart_and_undotted_lead(self):
        p, s = fixture()
        m = machine(p, s, rigid_tapping=True)
        decoder = CentroidDecoder(m, p)
        for line, message in (
            ('G0 X1', 'dogleg'),
            ('M2', 'restarts'),
            ('G52 X1 Y0 Z0', 'zero XYZ G52'),
            ('G84 X1 Y1 Z-1 R1 Q1', 'decimal point'),
        ):
            with self.assertRaisesRegex(ValueError, message):
                decoder.read_line(line)
        with self.assertRaisesRegex(ValueError, 'documented CNC12 subset'):
            replace(m, rigid_tapping=False)


if __name__ == '__main__':
    unittest.main()
