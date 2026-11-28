from dataclasses import replace
import math
import unittest
from pathlib import Path
import tempfile
import sys

from camlib.motion import Program, WorkOffset, ToolChange, Spindle, Coolant, Motion, Line, Arc, Helix, Stop
from camlib.post import (
    Cancellation,
    GCodeDialect,
    GCodePost,
    Machine,
    NativeGCodeDecoder,
    PostEngine,
    ToolSetting,
    post_events,
    post_program,
)
from test_simulation import tool


def fixture():
    """A short program and a LinuxCNC machine that posts it (fine precision)."""
    t = tool()
    p = Program(
        'post-job',
        'r1',
        (t,),
        (5, 0, 3),
        (
            WorkOffset(1),
            ToolChange('t1', 2, 7, (5, 0, 3)),
            Spindle(12000),
            Coolant('flood'),
            Motion(1, 'pocket', 't1', Line((5, 0, 3), (10, 0, 3)), 1200),
            Coolant('mist'),
            Spindle(0),
            Stop(True),
        ),
    )
    m = Machine(
        'post-machine',
        (ToolSetting.for_tool(t, number=2, length_register=7, position_after_mm=p.initial_position_mm),),
        dialect=GCodeDialect.linuxcnc(),
        output_units='mm',
        decimals=6,
        tolerance_mm=0.001,
        feed_tolerance_mm_min=0.0001,
        max_feed_mm_min=10000,
        max_spindle_rpm=30000,
        rapid_mm_min=6000,
        planes=('xy', 'xz', 'yz'),
        arcs=True,
        helices=True,
        max_sweep_rad=math.tau,
        max_tool_number=99999,
        max_length_register=99999,
        tool_changes=True,
        flood=True,
        mist=True,
        tool_change_position_mode='table',
    )
    return p, m


class PostTest(unittest.TestCase):
    def test_lifecycle_checked_mapping_and_actual_decoded_motion(self):
        p, m = fixture()
        chunks = list(post_program(p, machine=m))
        self.assertEqual(len(chunks), 3)
        self.assertEqual(chunks[0].data['scope'], 'python_declared_decoder_nominal_intent_only')
        self.assertEqual(chunks[-1].text, 'M9\nG49\nM2\n')  # spindle already stopped by its event
        self.assertEqual(chunks[1].motions, (p.events[4],))
        lines = [line for chunk in chunks for line in chunk.data['lines']]
        self.assertEqual([item['number'] for item in lines], list(range(1, len(lines) + 1)))
        mapped = [item for item in lines if item['motion_id'] is not None]
        self.assertEqual((mapped[0]['operation'], mapped[0]['motion_id']), ('pocket', 1))
        self.assertIn('G43 H7', [item['text'] for item in lines])
        engine = PostEngine(m, GCodePost, decoder=NativeGCodeDecoder)
        engine.begin(p)
        with self.assertRaisesRegex(ValueError, 'begun'):
            engine.begin(p)
        list(engine.consume_events(p.events))
        engine.finish()
        with self.assertRaises(ValueError):
            engine.finish()
        with self.assertRaises(ValueError):
            list(engine.consume_events(()))

    def test_every_written_block_is_decoded_and_checked(self):
        """A post's output is read back independently: wrong words fail, any
        equivalent spelling the controller reads the same way passes."""
        p, m = fixture()

        class WrongCoordinate(GCodePost):
            def on_motion(self, motion):
                self.emit('G1 X11 Y0 Z3 F1200')

        class WrongRegister(GCodePost):
            def on_tool_change(self, event):
                self.emit('T2')
                self.emit('M6')
                self.emit('G43 H2')

        class CustomWords(GCodePost):
            def on_motion(self, motion):
                self.emit('g1x10y0z3f1200 (custom python post)')

        with self.assertRaisesRegex(ValueError, 'deviation'):
            list(post_program(p, machine=m, post=WrongCoordinate))
        with self.assertRaisesRegex(ValueError, 'register'):
            list(post_program(p, machine=m, post=WrongRegister))
        chunks = list(post_program(p, machine=m, post=CustomWords))
        self.assertEqual(chunks[1].motions[0].curve, p.events[4].curve)

    def test_cancellation_precision_tool_table_and_source_identity(self):
        p, m = fixture()
        token = Cancellation()
        token.cancel()
        with self.assertRaisesRegex(ValueError, 'cancelled'):
            list(post_program(p, machine=m, cancel=token))
        first = list(post_program(p, machine=m))[0].data['identity']
        changed = list(post_program(replace(p, revision='r2'), machine=m))[0].data['identity']
        self.assertNotEqual(first, changed)
        bad = replace(p, events=(*p.events[:4], Motion(1, 'pocket', 't1', Line((5, 0, 3), (5.0001, 0, 3)), 1200)))
        with self.assertRaisesRegex(ValueError, 'decimals'):
            list(post_program(bad, machine=replace(m, decimals=2)))
        with self.assertRaisesRegex(ValueError, 'tool table'):
            list(post_program(p, machine=replace(m, tools=(replace(m.tools[0], gauge_length_mm=999),))))
        with self.assertRaises(TypeError):
            list(post_events(p, p.events, machine=object()))

    def test_stream_iterator_posts_before_source_is_exhausted(self):
        p, m = fixture()
        produced = 0

        def events():
            nonlocal produced
            for event in p.events:
                produced += 1
                yield event

        stream = post_events(replace(p, events=()), events(), machine=m)
        first = next(stream)
        self.assertEqual(produced, 0)
        self.assertIn('G91.1', first.text)
        next(stream)
        self.assertEqual(produced, len(p.events))
        next(stream)
        with self.assertRaises(StopIteration):
            next(stream)

    def test_planes_full_circles_helix_and_output_units(self):
        p, m = fixture()
        arc = Motion(1, 'pocket', 't1', Arc((5, 0, 3), (-5, 0, 3), (0, 0, 3), math.pi), 1200)
        helix = Motion(2, 'pocket', 't1', Helix((-5, 0, 3), (-5, 0, 2), (0, 0, 3), -math.tau), 300, 'plunge')
        program = replace(p, events=(*p.events[:3], arc, helix))
        chunks = list(post_program(program, machine=replace(m, output_units='inch', decimals=8)))
        motions = chunks[1].motions
        self.assertIsInstance(motions[0].curve, Arc)
        self.assertIsInstance(motions[1].curve, Helix)
        self.assertEqual(motions[0].curve.start[2], motions[0].curve.end[2])
        self.assertLess(abs(motions[1].curve.sweep_rad + math.tau), 1e-12)
        self.assertTrue(all(c['deviation_bound_mm'] < 0.001 for c in chunks[1].data['checked'] if c['motion']))

    def test_actual_pocket_posts_progressively_and_rechecks_decoded_stock(self):
        examples = str(Path(__file__).resolve().parents[2] / 'examples')
        sys.path.insert(0, examples)
        try:
            from support.post_pocket import run
        finally:
            sys.path.remove(examples)
        with tempfile.TemporaryDirectory() as tmp:
            nc = Path(tmp) / 'pocket.ngc'
            result = run(nc)
            self.assertEqual(result['motions'], 400)
            self.assertFalse(result['first_posted_motion']['source_finished'])
            self.assertFalse(result['first_posted_motion']['source_native_exited'])
            self.assertEqual(result['source_heights_sha256'], result['posted_heights_sha256'])
            self.assertEqual(result['posted_scene_stock_counts'], dict(clear=400, violation=0, uncertain=0))
            self.assertTrue(result['posted_material']['complete'])
            self.assertEqual(result['posted_material']['residual']['definite_cells'], 12)
            self.assertTrue(nc.read_text().endswith('M2\n'))


if __name__ == '__main__':
    unittest.main()
