"""Actual decoded control effects survive native comparison and transactions."""

from camlib.postprocessor import GCodeDialect

from copy import deepcopy
from dataclasses import FrozenInstanceError, replace
import json
import unittest

from camlib.motion import Coolant, Line, Motion, Spindle, Stop, ToolChange
from camlib.post import (
    Cancellation,
    Chunk,
    GCodeDecoder,
    GCodePost,
    LoweringOptions,
    Machine,
    PostEngine,
    ToolSetting,
    post_program,
)
from test_post import fixture


class CheckedPostControls(unittest.TestCase):
    def fixture(self, **changes):
        program, setup = fixture()
        return program, Machine(
            'Python control-event mill',
            setup.tools,
            **{
                'dialect': GCodeDialect.linuxcnc(),
                'output_units': 'mm',
                'decimals': 3,
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
                **changes,
            },
        )

    def assert_views(self, chunk):
        decoded = Chunk(chunk.raw)
        self.assertEqual(chunk.events, decoded.events)
        self.assertEqual(chunk.motions, tuple(e for e in decoded.events if isinstance(e, Motion)))
        self.assertEqual(chunk.text, decoded.text)
        return decoded.events

    def test_actual_effects_precede_normalization_and_keep_control_provenance(self):
        p, m = self.fixture()
        # A stopped spindle has no meaningful direction; actual M5 decodes cw.
        p = replace(p, events=(*p.events[:6], Spindle(0, 'ccw'), p.events[7]))
        chunks = list(post_program(p, m, run_id='controls'))
        self.assertEqual(chunks[0].data['identity']['abi'], 5)
        self.assertEqual(self.assert_views(chunks[0]), ())
        events = self.assert_views(chunks[1])
        expected = (
            *p.events[:3],
            Coolant('flood'),
            p.events[4],  # flood from off: no M9 first
            Coolant('off'),
            Coolant('mist'),
            Spindle(0),
            p.events[7],
        )
        self.assertEqual(events, expected)
        self.assertEqual(self.assert_views(chunks[-1]), ())
        data = chunks[1].data
        for source, check, mapping in zip(p.events, data['checked'], data['mappings']):
            self.assertEqual(check['event_index'], mapping['event_index'])
            self.assertEqual(check['line_interval'], mapping['line_interval'])
            lines = [
                item
                for item in data['lines']
                if check['line_interval'][0] <= item['number'] <= check['line_interval'][1]
            ]
            self.assertTrue(lines)
            self.assertTrue(all(item['event_index'] == check['event_index'] for item in lines))
            if isinstance(source, Motion):
                self.assertEqual(check['control_effects'], [])
            else:
                self.assertIsNone(check['motion'])
                self.assertIsNone(check['deviation_bound_mm'])
                self.assertIsNone(mapping['source_fraction_interval'])
                self.assertIsNone(mapping['source_motion_id'])
                self.assertIsNone(mapping['posted_motion_id'])
                self.assertTrue(all(item['motion_id'] is None and item['operation'] is None for item in lines))
                self.assertEqual(len(mapping['source_event_sha256']), 64)
        self.assertEqual(data['checked'][6]['control_effects'], [Spindle(0).to_dict()])
        changed = replace(p, events=(*p.events[:7], Stop(False)))
        other = list(post_program(changed, m, run_id='controls'))[1].data
        self.assertNotEqual(data['mappings'][7]['source_event_sha256'], other['mappings'][7]['source_event_sha256'])

    def test_multiple_tools_edges_and_declared_pose_resets_in_direct_and_lowered_views(self):
        p, m = self.fixture()
        # Same physical T, different cutting edges with independent tip poses.
        edge = replace(p.tools[0], id='edge2')
        pose = (20, 10, 8)
        setting = ToolSetting.for_tool(edge, number=2, length_register=8, position_after_mm=pose)
        dialect = replace(m.dialect, length_word='D', length_activation='tool_edge').with_commands(
            length_off='D0', length_on=None
        )
        m = replace(m, tools=(*m.tools, setting), dialect=dialect)
        large = (1 << 63) + 29
        events = (
            *p.events[:3],
            replace(p.events[4], id=large),
            Spindle(0),
            ToolChange('edge2', 2, 8, pose),
            Spindle(900),
            Motion(large + 1, 'edge-pass', 'edge2', Line(pose, (21, 10, 8)), 400),
            Spindle(0),
            Stop(),
        )
        p = replace(p, tools=(*p.tools, edge), events=events)
        for lowering in (None, LoweringOptions()):
            with self.subTest(lowering=lowering):
                chunks = list(post_program(p, m, lowering=lowering))
                actual = tuple(e for chunk in chunks for e in self.assert_views(chunk))
                controls = tuple(e for e in actual if not isinstance(e, Motion))
                self.assertEqual(controls, tuple(e for e in events if not isinstance(e, Motion)))
                moves = tuple(e for e in actual if isinstance(e, Motion))
                self.assertEqual([e.id for e in moves], [large, large + 1] if lowering is None else [1, 2])
                self.assertEqual(moves[1].curve.start, pose)
                transfer = next(e for e in actual if isinstance(e, ToolChange) and e.tool == 'edge2')
                self.assertEqual(transfer.position_after_mm, pose)
                self.assertEqual(transfer.length_register, 8)
                # The reset is a control only; there is no interpolated connecting motion.
                self.assertEqual(len(moves), 2)
                self.assertEqual(chunks[0].data['identity']['abi'], 5 if lowering is None else 6)

    def test_native_control_mismatch_rejection_does_not_advance_events_or_lines(self):
        p, m = self.fixture()
        e = PostEngine(m)
        begin = e.begin(p)
        source = [v.to_dict() for v in p.events[:2]]
        _, _, blocks, actual, _, _ = e._format(source, p.events[:2], [(0.0, 1.0)] * 2, None)
        for change in ('tool', 'number', 'length_register', 'position_after_mm'):
            forged = deepcopy(actual)
            forged[1][0][change] = {'tool': 'other', 'number': 3, 'length_register': 8, 'position_after_mm': [6, 0, 3]}[
                change
            ]
            with self.subTest(change=change), self.assertRaises(ValueError):
                e._session.consume(json.dumps(source), json.dumps(forged), json.dumps(blocks))
            self.assertEqual(e.position_mm, p.initial_position_mm)
        good = e.consume(p.events[:2])
        self.assert_views(good)
        self.assertEqual(good.data['checked'][0]['event_index'], 0)
        self.assertEqual(good.data['lines'][0]['number'], begin.data['lines'][-1]['number'] + 1)
        self.assertEqual(good.events, p.events[:2])

    def test_late_failure_and_cancel_roll_back_candidate_control_and_python_states(self):
        p, m = self.fixture()
        token = Cancellation()

        class Candidate(GCodePost):
            def __init__(self, machine):
                super().__init__(machine)
                self.mode = 'bad'
                self.changes = 0

            def on_tool_change(self, event):
                self.changes += 1
                super().on_tool_change(event)

            def on_motion(self, event):
                if self.mode == 'bad':
                    self.emit('G1 X99 Y0 Z3 F1200')
                else:
                    if self.mode == 'cancel':
                        token.cancel()
                    super().on_motion(event)

        e = PostEngine(m, Candidate)
        begin = e.begin(p)
        for mode, error in (('bad', 'deviation'), ('cancel', 'cancelled')):
            e.post.mode = mode
            with self.assertRaisesRegex(ValueError, error):
                e.consume(p.events, cancel=token)
            self.assertEqual(e.post.changes, 0)
            self.assertIsNone(e.decoder.active)
            self.assertEqual(e.position_mm, p.initial_position_mm)
        e.post.mode = 'ok'
        committed = e.consume(p.events)
        self.assert_views(committed)
        self.assertEqual(committed.data['checked'][0]['event_index'], 0)
        self.assertEqual(committed.data['lines'][0]['number'], begin.data['lines'][-1]['number'] + 1)
        self.assertEqual(e.post.changes, 1)

    def test_lowered_control_retry_preserves_pending_source_hash_and_actual_effects(self):
        p, m = self.fixture()
        token = Cancellation()

        class Candidate(GCodePost):
            def __init__(self, machine):
                super().__init__(machine)
                self.cancel = True

            def on_tool_change(self, event):
                super().on_tool_change(event)
                if self.cancel:
                    token.cancel()

        e = PostEngine(m, Candidate, lowering=LoweringOptions())
        e.begin(p)
        e.start(p.events[0])
        first = e.drain()
        e.start(p.events[1])
        preview = e._session.preview()
        with self.assertRaisesRegex(ValueError, 'cancelled'):
            e.drain(cancel=token)
        self.assertTrue(e.pending)
        self.assertEqual(e._session.preview(), preview)
        self.assertIsNone(e.decoder.active)
        e.post.cancel = False
        chunk = e.drain()
        self.assertFalse(e.pending)
        self.assertEqual(self.assert_views(chunk), (p.events[1],))
        mapping = chunk.data['mappings'][0]
        self.assertEqual(mapping['event_index'], 1)
        self.assertEqual(mapping['line_interval'][0], first.data['lines'][-1]['number'] + 1)
        # Source controls are not assigned synthetic motion IDs/fractions by lowering.
        self.assertIsNone(mapping['posted_motion_id'])
        self.assertIsNone(mapping['source_fraction_interval'])

    def test_live_and_raw_events_own_mutable_decoder_results(self):
        p, m = self.fixture()

        class MutableTool:
            def __init__(self, event):
                self.data = event.to_dict()

            def to_dict(self):
                return self.data

        class MutableDecoder(GCodeDecoder):
            def __init__(self, machine, program):
                super().__init__(machine, program)
                self.returned = []

            def read_line(self, line, context=None):
                events = super().read_line(line, context)
                out = []
                for event in events:
                    if isinstance(event, ToolChange):
                        event = MutableTool(event)
                        self.returned.append(event)
                    out.append(event)
                return out

        e = PostEngine(m, decoder=MutableDecoder)
        e.begin(p)
        chunk = e.consume(p.events)
        before = self.assert_views(chunk)
        e.decoder.returned[0].data['position_after_mm'] = [99, 0, 3]
        changed = chunk.data
        changed['checked'][1]['control_effects'][0]['tool'] = 'wrong'
        self.assertEqual(self.assert_views(chunk), before)
        with self.assertRaises(FrozenInstanceError):
            chunk.events[1].tool = 'wrong'

    def test_old_abi_rejects_complete_event_view_instead_of_dropping_controls(self):
        p, m = self.fixture()
        chunks = list(post_program(p, m))
        for abi in (1, 2, 3, 4):
            data = chunks[1].data
            data['identity']['abi'] = abi
            for check in data['checked']:
                check.pop('control_effects')
            old = Chunk(json.dumps(data).encode())
            self.assertEqual(old.motions, chunks[1].motions)
            with self.assertRaisesRegex(ValueError, 'ABI5/6'):
                old.events
        # A received malformed new record must not invent a motion/control order.
        for controls in ([], [p.events[4].to_dict()]):
            data = chunks[1].data
            data['checked'][1]['control_effects'] = controls
            with self.assertRaises(ValueError):
                Chunk(json.dumps(data).encode()).events


if __name__ == '__main__':
    unittest.main()
