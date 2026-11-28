"""LNC milling reference post (#182): A-LNC Milling Programming Manual V01.02.000."""

from dataclasses import replace
import math
from pathlib import Path
import sys
import tempfile
import unittest

from camlib.motion import Arc, Coolant, Cycle, Helix, Line, Motion, Spindle, ToolChange, WorkFrame
from camlib.post import LoweringOptions, PostEngine, ToolSetting
from lnc_mill_oracle import Oracle
from test_drill_cycles import job as drill_job
from test_drill_cycles import boring_job
from test_post import fixture
from test_tapping import CHANGE, job as tap_job, posted_text, tapping, tool_table as tap_table

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'examples'))
try:
    from lnc_mill_post import LNCMillDecoder, LNCMillPost, define_machine, post_lnc
finally:
    sys.path.pop(0)

OEM = dict(tool_change='M6', spindle_cw='M3', spindle_ccw='M4', spindle_off='M5', coolant_off='M9', flood='M8')
SETTINGS = dict(
    max_feed_mm_min=10000,
    max_spindle_rpm=30000,
    max_sweep_rad=math.pi,
    tool_changes=True,
    flood=True,
    tool_change_position_mode='table',
    rapid_mm_min=6000,
    output_units='mm',
    decimals=3,
    max_tool_number=24,
)


def lnc(tools, **settings):
    values = dict(
        startup_standard_modes=True,
        g92_shift_mm=(0, 0, 0),
        m_codes=OEM,
        arc_radius_tolerance_mm=0.002,
        traverse_feed_mm_min=3000,
        work_to_machine=WorkFrame(),
        dwell_max_s=1e5,
        **SETTINGS,
    )
    values.update(settings)
    return define_machine('LNC test mill', tools, **values)


def arcs_program():
    """A line and one exact arc in each plane (every word exact at 0.001 mm)."""
    p, _ = fixture()
    a = (10, 0, 3)
    b = (20, 0, 3)  # XY half circle about (15, 0, 3), clockwise
    c = (25, 0, 8)  # XZ quarter about (20, 0, 8)
    d = (25, 5, 3)  # YZ quarter about (25, 0, 3)
    events = (
        *p.events[:4],
        Motion(1, 'pocket', 't1', Line((5, 0, 3), a), 1200),
        Motion(2, 'pocket', 't1', Arc(a, b, (15, 0, 3), -math.pi, 'xy'), 1200),
        Motion(3, 'pocket', 't1', Arc(b, c, (20, 0, 8), -math.pi / 2, 'xz'), 1200),
        Motion(4, 'pocket', 't1', Arc(c, d, (25, 0, 3), -math.pi / 2, 'yz'), 1200),
        Coolant(),
        Spindle(0),
    )
    return replace(p, events=events)


class LNCMillPostTest(unittest.TestCase):
    def machine(self, **settings):
        p = arcs_program()
        return p, lnc(fixture()[1].tools, work_to_machine=WorkFrame(origin_mm=(100, 200, 300)), **settings)

    def oracle(self, p, m, chunks):
        oracle = Oracle(
            tools=m.tools,
            initial_position=p.initial_position_mm,
            radius_tolerance_mm=m.arc_radius_tolerance_mm,
            work_offset=m.work_offset,
            m_codes=dict(m.oem_m_codes),
        )
        observed = [v for c in chunks for line in c.text.splitlines() for v in oracle.read(line)]
        actual = [v for c in chunks for v in c.motions]
        self.assertTrue(oracle.ended)
        self.assertEqual(len(observed), len(actual))
        for o, a in zip(observed, actual):
            self.assertEqual((o.tool, type(o.curve), o.feed_mm_min), (a.tool, type(a.curve), a.feed_mm_min))
            for key in ('start', 'end', 'center'):
                if hasattr(a.curve, key):
                    for x, y in zip(getattr(o.curve, key), getattr(a.curve, key)):
                        self.assertAlmostEqual(x, y, places=9)
            if isinstance(a.curve, Arc):
                self.assertEqual(o.curve.plane, a.curve.plane)
                self.assertAlmostEqual(o.curve.sweep_rad, a.curve.sweep_rad, places=9)
        return oracle

    def test_literal_program_native_arcs_in_every_plane(self):
        p, m = self.machine()
        chunks = list(post_lnc(p, m))
        text = ''.join(c.text for c in chunks)
        self.assertEqual(text, (ROOT / 'fixtures/posts/LNC-MILL-ARCS.NC').read_text())
        oracle = self.oracle(p, m, chunks)
        self.assertEqual(oracle.transitions, [(43, 63.0), (49, 63.0)])
        # Metric, no inch, no rapid, no framing the manual does not define.
        for absent in ('G20', 'G0 ', 'G53', 'G91.1', 'G97', '%', 'O'):
            self.assertNotIn(absent, text)
        self.assertEqual(text.splitlines()[-1], 'M30')
        self.assertEqual(
            [v.curve for c in chunks for v in c.motions], [e.curve for e in p.events if isinstance(e, Motion)]
        )

    def test_actual_word_mutations_are_refused(self):
        p, m = self.machine()
        text = ''.join(c.text for c in post_lnc(p, m))
        arc = next(line for line in text.splitlines() if line.startswith('G2 '))
        variants = (
            text.replace('G21\n', 'G20\n'),
            text.replace('G61\n', ''),
            text.replace('G54\n', 'G55\n'),
            text.replace('G52 X0. Y0. Z0.', 'G52 X1. Y0. Z0.'),
            text.replace('G1 G43 H7 Z3. F3000.', 'G43 H7'),
            text.replace('G1 G43 H7 Z3. F3000.', 'G1 G43 H7 Z4. F3000.'),
            text.replace('G1 G49 Z63. F3000.', 'G49'),
            text.replace('G1 G43 H7', 'G1 G43 H8'),
            text.replace('G1 G43 H7 Z3. F3000.\n', 'G1 G43 H7 Z3. F3000.\nG53 Z0.\n'),
            text.replace('X10.', 'X10'),
            text.replace('G1 X10.', 'G0 X10.'),
            text.replace('G94', 'G95'),
            # Arc end point off the circle by more than parameter 50048.
            text.replace(arc, arc.replace('I5.', 'I5.01')),
            text.replace('G18\n', ''),
            text + 'G1 X0. F100.\n',
        )
        for broken in variants:
            with self.subTest(broken=broken):
                self.assertNotEqual(broken, text)
                decoder = LNCMillDecoder(m, p)
                with self.assertRaises(ValueError):
                    for line in broken.splitlines():
                        decoder.read_line(line, dict(id=1, operation='file', kind='cut'))

    def test_radius_error_within_parameter_50048_is_accepted(self):
        p, m = self.machine()
        text = ''.join(c.text for c in post_lnc(p, m))
        arc = next(line for line in text.splitlines() if line.startswith('G2 '))
        oracle = Oracle(tools=m.tools, initial_position=p.initial_position_mm, radius_tolerance_mm=0.002)
        ok = text.replace(arc, arc.replace('I5.', 'I5.001'))
        for line in ok.splitlines():
            oracle.read(line)
        strict = Oracle(tools=m.tools, initial_position=p.initial_position_mm, radius_tolerance_mm=0)
        with self.assertRaisesRegex(ValueError, '510204'):
            for line in ok.splitlines():
                strict.read(line)

    def test_helices_and_large_sweeps_are_lowered_or_split(self):
        p, m = self.machine()
        start = p.initial_position_mm
        helix = Motion(9, 'helix', 't1', Helix(start, (start[0], start[1], 1), (0, 0, 3), -2 * math.tau, 'xy'), 1200)
        full = Motion(10, 'circle', 't1', Arc(start, start, (0, 0, 3), -math.tau, 'xy'), 1200)
        for motion in (helix, full):
            job = replace(p, events=(*p.events[:4], motion, Coolant(), Spindle(0)))
            # Lowering is explicit: without it the post refuses.
            with self.assertRaises(ValueError):
                list(post_lnc(job, m))
            chunks = list(post_lnc(job, m, lowering=LoweringOptions(native_arcs=True)))
            text = ''.join(c.text for c in chunks)
            self.oracle(job, m, chunks)
            for line in text.splitlines():
                if line.startswith(('G2 ', 'G3 ')):
                    self.assertNotIn('Z', line)

    def test_cycles_after_an_xz_arc_select_g17(self):
        p, m = self.machine(peck_return_mm=0.254)
        a, b = (10, 0, 3), (15, 0, 8)
        events = (
            *p.events[:4],
            Motion(1, 'pocket', 't1', Line((5, 0, 3), a), 1200),
            Motion(2, 'pocket', 't1', Arc(a, b, (15, 0, 3), math.pi / 2, 'xz'), 1200),
            Cycle('drill', (15, 0), 1, 5, 8, 150, 3),
            Motion(3, 'drill', 't1', Line(b, (15, 0, 5)), None),
            Motion(4, 'drill', 't1', Line((15, 0, 5), (15, 0, 1)), 150),
            Motion(5, 'drill', 't1', Line((15, 0, 1), b), None),
            Coolant(),
            Spindle(0),
        )
        job = replace(
            p,
            events=tuple(
                replace(e, kind='rapid') if isinstance(e, Motion) and e.feed_mm_min is None else e for e in events
            ),
        )
        text = ''.join(c.text for c in post_lnc(job, m)).splitlines()
        g81 = next(i for i, line in enumerate(text) if line.startswith('G81'))
        self.assertEqual(text[g81 - 2 : g81 + 2], ['G17', 'G98', 'G81 X15. Y0. Z1. R5. F150.', 'G80'])
        broken = '\n'.join(text[: g81 - 2] + text[g81 - 1 :])
        decoder = LNCMillDecoder(m, job)
        with self.assertRaisesRegex(ValueError, 'G17'):
            for line in broken.splitlines():
                decoder.read_line(line, dict(id=1, operation='file', kind='cut'))

    def test_drilling_cycles_with_declared_parameters(self):
        tools = drill_job().compile().setup('top').program_header()
        table = tuple(
            replace(t, position_after_mm=tools.initial_position_mm)
            for t in __import__('test_drill_cycles').tool_table(drill_job())
        )
        m = lnc(table, peck_return_mm=0.254, chip_break_return_mm=0.254)
        for drilling, block in (
            ({}, 'G81 X8. Y8. Z4. R11. F150.'),
            (dict(dwell_s=0.25), 'G82 X8. Y8. Z4. R11. P250 F150.'),
            (dict(peck_mm=2), 'G83 X8. Y8. Z4. R11. Q2. F150.'),
            (dict(peck_mm=2, peck_mode='chip_break'), 'G73 X8. Y8. Z4. R11. Q2. F150.'),
        ):
            with self.subTest(block=block):
                nc = posted_text(drill_job(**drilling), m, LNCMillPost, LNCMillDecoder)
                self.assertIn(block, nc)
                self.assertEqual(nc[nc.index(block) - 1], 'G98')
        # Without parameter 160031 declared, pecking holes are explicit moves.
        plain = lnc(table)
        nc = posted_text(drill_job(peck_mm=2), plain, LNCMillPost, LNCMillDecoder)
        self.assertFalse(any(line.startswith('G83') for line in nc))
        # A different operation return amount refuses.
        with self.assertRaisesRegex(Exception, 'return amount'):
            posted_text(drill_job(peck_mm=2, reentry_clearance_mm=0.5), m, LNCMillPost, LNCMillDecoder)

    def test_boring_cycles(self):
        j = boring_job()
        header = j.compile().setup('top').program_header()
        # Tip poses after M6: the drill at the start, the reamer over the last hole.
        poses = (header.initial_position_mm, (20, 8, 12))
        table = tuple(
            ToolSetting.for_tool(t, number=i, length_register=i, position_after_mm=pose)
            for i, (t, pose) in enumerate(zip(header.tools, poses), 1)
        )
        m = lnc(table)
        nc = posted_text(j, m, LNCMillPost, LNCMillDecoder)
        self.assertIn('G85 X8. Y8. Z4. R11. F100.', nc)

    def test_rigid_tapping_m29_and_m28(self):
        for hand, code, spindle in (('right', 'G84', 'M3'), ('left', 'G74', 'M4')):
            with self.subTest(hand):
                j = tap_job(lambda tap: tapping(tap, hand=hand), tool_change=CHANGE)
                m = lnc(tap_table(j), rigid_tapping_m29=True)
                nc = j.post_nc(
                    'top', machine=m, post=LNCMillPost, decoder=LNCMillDecoder, accept={'unchecked_machine'}
                ).text.splitlines()
                cycles = [i for i, line in enumerate(nc) if line.startswith(code)]
                self.assertEqual(
                    [nc[i - 1 : i + 3] for i in cycles],
                    [
                        ['M29 S600', f'{code} X8. Y8. Z4. R11. F600.', 'M28', 'G80'],
                        ['M29 S600', f'{code} X20. Y8. Z4. R11. F600.', 'M28', 'G80'],
                    ],
                )
                spin = next(line for line in reversed(nc[: cycles[0]]) if line.split()[0] in ('M3', 'M4', 'M5'))
                self.assertEqual(spin.split()[0], spindle)
                text = '\n'.join(nc) + '\n'
                header = j.compile().setup('top').program_header()
                for broken in (
                    text.replace('M29 S600\n', '', 1),
                    text.replace('M28\n', '', 1),
                    text.replace('M29 S600', 'M29 S500', 1),
                ):
                    decoder = LNCMillDecoder(m, header)
                    with self.assertRaises(ValueError):
                        for line in broken.splitlines():
                            decoder.read_line(line, dict(id=1, operation='file', kind='cut'))
        # Without the rigid tapping premise, tapping is refused.
        j = tap_job(tapping, tool_change=CHANGE)
        with self.assertRaisesRegex(Exception, 'tapping cycle'):
            j.post_nc(
                'top', machine=lnc(tap_table(j)), post=LNCMillPost, decoder=LNCMillDecoder, accept={'unchecked_machine'}
            )

    def test_two_tools_and_corrupted_output_retry(self):
        p, m = self.machine()
        second = replace(
            p.tools[0],
            id='second',
            gauge_length_mm=70,
            holders=tuple(replace(h, z_max_mm=70) for h in p.tools[0].holders),
        )
        table = (*m.tools, ToolSetting.for_tool(second, number=3, length_register=8, position_after_mm=(25, 5, 4)))
        m = replace(m, tools=table)
        p = replace(
            p,
            tools=(*p.tools, second),
            events=(
                *p.events,
                ToolChange('second', 3, 8, (25, 5, 4)),
                Spindle(1200),
                Motion(5, 'second', 'second', Line((25, 5, 4), (30, 5, 4)), 600),
                Spindle(0),
            ),
        )
        chunks = list(post_lnc(p, m))
        oracle = self.oracle(p, m, chunks)
        self.assertEqual(oracle.transitions, [(43, 63.0), (49, 63.0), (43, 74.0), (49, 74.0)])

        class Corrupt(LNCMillPost):
            bad = True

            def on_tool_change(self, event):
                super().on_tool_change(event)
                if self.bad:
                    self._lines[-1] = self._lines[-1].replace('Z3.', 'Z4.')

        engine = PostEngine(m, Corrupt, decoder=LNCMillDecoder)
        engine.begin(p)
        engine.consume(p.events[:1])
        with self.assertRaisesRegex(ValueError, 'physical tip'):
            engine.consume(p.events[1:2])
        engine.post.bad = False
        engine.consume(p.events[1:])
        engine.finish()

    def test_configuration_refusals(self):
        tools = fixture()[1].tools
        for settings in (
            dict(output_units='inch', decimals=4),
            dict(decimals=4),
            dict(startup_standard_modes=False),
            dict(g92_shift_mm=(1, 0, 0)),
            dict(max_sweep_rad=math.tau),
            dict(helices=True),
            dict(tool_change_position_mode='preserve_tip'),
            dict(arc_radius_tolerance_mm=-1),
            dict(arc_radius_tolerance_mm=None),
            dict(m_codes=dict(OEM, tool_change='M30')),
            dict(m_codes=dict(OEM, tool_change='M98')),
            dict(m_codes=dict(OEM, flood='M29'), rigid_tapping_m29=True),
            dict(m_codes=dict(OEM, flood='M19'), fine_bore_edge_deg=0),
            dict(m_codes=dict(OEM, flood='M5')),
            dict(max_length_register=401),
        ):
            with self.subTest(settings=settings):
                with self.assertRaises((ValueError, TypeError)):
                    lnc(tools, **settings)

    def test_complete_pocket_posted_stock_and_literal_file(self):
        sys.path.insert(0, str(ROOT / 'examples'))
        try:
            from support.post_pocket import run
        finally:
            sys.path.pop(0)
        captured = {}

        def builder(header, tools):
            m = replace(lnc(tools), frame=header.frame)
            captured.update(machine=m, header=header)
            return m

        with tempfile.TemporaryDirectory() as directory:
            nc = Path(directory) / 'LNC.NC'
            result = run(
                nc,
                profile='python',
                lowering=False,
                radial_allowance_mm=0.01,
                post_type=LNCMillPost,
                decoder_type=LNCMillDecoder,
                machine_builder=builder,
            )
            m, p = captured['machine'], captured['header']
            oracle = Oracle(tools=m.tools, initial_position=p.initial_position_mm, radius_tolerance_mm=0.002)
            decoder = LNCMillDecoder(m, p)
            count = 0
            for line in nc.read_text().splitlines():
                observed = oracle.read(line)
                actual = [
                    e
                    for e in decoder.read_line(line, dict(id=count + 1, operation='file', kind='cut'))
                    if isinstance(e, Motion)
                ]
                self.assertEqual(len(observed), len(actual))
                count += len(actual)
            self.assertTrue(oracle.ended)
            # Native arcs, read back by the oracle as the manual defines them.
            self.assertTrue(any(line.startswith(('G2 ', 'G3 ')) for line in nc.read_text().splitlines()))
        self.assertEqual(count, result['motions'])
        self.assertEqual(result['source_motions'], result['motions'])
        self.assertEqual(result['posted_scene_stock_counts'], dict(clear=count, violation=0, uncertain=0))
        self.assertTrue(result['posted_material']['complete'])


if __name__ == '__main__':
    unittest.main()
