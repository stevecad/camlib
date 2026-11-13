"""Native2 typed controls, Python stock buffers and read-only query transport."""

from dataclasses import replace
import json
from pathlib import Path
import struct
import tempfile
import unittest
from unittest.mock import patch

from camlib.motion import Coolant, Line, Motion, Spindle, Stop, ToolChange
from camlib.post import GCodeDialect, Machine, PostEngine, ToolSetting, post_events
from camlib.simulation import (
    ControlUpdate,
    Frame,
    PrefixSession,
    ProtocolError,
    Simulation,
    SimulationError,
    StockSpec,
    StockState,
    Update,
    VerificationUpdate,
)
from camlib.verification import StockContactOptions, StreamChecks
from test_post import fixture
from test_simulation import checksum


def job(pose=(5, 0, 3)):
    p, s = fixture()
    second = replace(p.tools[0], id='edge2')
    other_pose = (2, 8, 6)
    settings = (
        replace(s.tools[0], position_after_mm=pose),
        ToolSetting.for_tool(second, number=2, length_register=8, position_after_mm=other_pose),
    )
    dialect = replace(GCodeDialect(), length_word='D', length_activation='tool_edge').with_commands(
        length_off='D0', length_on=None
    )
    machine = Machine(
        'Unregistered control transport mill',
        settings,
        dialect=dialect,
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
    first = replace(p.events[4], id=2**63 + 53, curve=Line(pose, (10, 0, 3)))
    second_move = Motion(first.id + 1, 'edge-pass', 'edge2', Line(other_pose, (10, 8, 6)), 900)
    events = (
        p.events[0],
        replace(p.events[1], position_after_mm=pose),
        p.events[2],
        Coolant('flood'),
        first,
        Spindle(0),
        Stop(True),
        ToolChange('edge2', 2, 8, other_pose),
        Spindle(4000),
        second_move,
        Spindle(0),
        Coolant(),
        Stop(),
    )
    return replace(p, tools=(*p.tools, second), events=events), machine


def simulation(p, **options):
    return Simulation(
        StockSpec((0, -10, 0), (20, 10, 10), 0.5),
        p.tools,
        revision=p.revision,
        initial_position_mm=p.initial_position_mm,
        stream_version=2,
        **options,
    )


def snapshot(state):
    return (
        state.completed,
        state.position,
        state.last_motion_id,
        state.fraction,
        state.finished,
        state.control_cursor,
        state.controls,
        state.heights.tobytes(),
        state._token,
    )


def reframe(frame, payload):
    raw = bytearray(frame.raw[:20]) + payload
    struct.pack_into('<I', raw, 12, len(payload))
    return Frame.parse(checksum(raw))


def setup_events(p, m):
    e = PostEngine(m)
    e.begin(p)
    return e.consume(p.events[:3]).events


class SimulationControls(unittest.TestCase):
    def test_actual_checked_python_post_streams_controls_and_removed_stock_before_finish(self):
        p, m = job()
        exhausted = False

        def posted():
            nonlocal exhausted
            for chunk in post_events(p, p.events, m):
                yield from chunk.events
            exhausted = True

        with simulation(p) as sim:
            stream = sim.run_events(posted(), batch_size=3)
            controls = []
            moves = []
            previous = sim.state.heights.tobytes()
            for update in stream:
                if isinstance(update, ControlUpdate):
                    self.assertEqual(sim.state.heights.tobytes(), previous)
                    controls.append(update)
                    self.assertEqual(update.control_cursor, sim.state.control_cursor)
                    if update.event['kind'] == 'tool_change':
                        self.assertEqual(
                            sim.state.controls['selected']['length_register'], update.event['length_register']
                        )
                        owned = sim.state.controls
                        owned['selected']['tool'] = 'foreign'
                        self.assertEqual(sim.state.controls['selected']['tool'], update.event['tool'])
                else:
                    self.assertIsInstance(update, Update)
                    moves.append(update)
                    if len(moves) == 1:
                        self.assertFalse(exhausted)
                        self.assertFalse(sim.state.finished)  # streamed: before the run's end
                        self.assertGreater(sim.state.volume()['removed_lower_mm3'], 0)
                    previous = sim.state.heights.tobytes()
            self.assertTrue(exhausted)
            self.assertTrue(sim.state.finished)
            self.assertEqual([v.motion['id'] for v in moves], [2**63 + 53, 2**63 + 54])
            self.assertEqual([v.control_cursor[0] for v in controls], list(range(1, 12)))  # no M9 before the first M8
            self.assertEqual(sim.state.controls['unverified_tool_routes'], 2)
            self.assertEqual(sim.state.controls['spindle_rpm'], 0)
            self.assertEqual(sim.state.completed, 2)

    def test_native_and_python_checkpoints_match_after_initial_reset_and_later_tool_change(self):
        p, m = job((4, 1, 5))
        e = PostEngine(m)
        e.begin(p)
        with simulation(p) as sim, tempfile.TemporaryDirectory() as directory:
            for events in (
                e.consume(p.events[:2]).events,
                e.consume(p.events[2:5]).events,
                e.consume(p.events[5:9]).events,
            ):
                list(sim.events(events))
                sim._send(dict(command='checkpoint'))
                native = sim._next()
                path = Path(directory) / 'state.bin'
                sim.state.write_checkpoint(path)
                # The producer's checkpoint adds its committed cleared columns
                # (CAMCOL01) after the cells; everything before is identical.
                ours, theirs = path.read_bytes(), bytes(native.raw)
                self.assertEqual(ours[:12], theirs[:12])
                self.assertEqual(theirs[20 : len(ours)], ours[20:])
                self.assertEqual(theirs[len(ours) : len(ours) + 8], b'CAMCOL01')
                other = StockState(sim.manifest_frame)
                other.restore_checkpoint(path)
                self.assertEqual(other.heights.tobytes(), sim.state.heights.tobytes())
                self.assertEqual((other.position, other.controls), (sim.state.position, sim.state.controls))
                if sim.state.completed == 0:
                    self.assertEqual(other.position, (4, 1, 5))
                    self.assertNotEqual(other.position, p.initial_position_mm)
                packed = Path(directory) / 'state.gz'
                other.write_checkpoint(packed, compressed=True)
                again = StockState(sim.manifest_frame)
                again.restore_checkpoint(packed, compressed=True)
                self.assertEqual(again.controls, sim.state.controls)
                self.assertEqual(again.control_cursor, sim.state.control_cursor)
            with PrefixSession(sim.state.canonical_manifest) as query:
                _, update = query.preview(sim.state, p.events[9], 0.371)
                self.assertEqual(update.motion['tool'], 'edge2')
                self.assertEqual(query.checkpoint_loads, 1)

    def test_controls_motion_preview_and_end_reject_bad_or_substituted_prefix_atomically(self):
        p, m = job()
        with simulation(p) as sim:
            setup = list(sim.events(setup_events(p, m)))
            move = p.events[4]
            sim._send(dict(command='partial', motion=move.to_dict()['motion'], fraction=0.37))
            preview = sim._next()
            movement = next(sim.events([move]))
            end = sim.finish()
            r = StockState(sim.manifest_frame)
            for update in setup[:2]:
                r.accept(update.frame)
            original = snapshot(r)
            bad = bytearray(setup[2].frame.raw)
            bad[-1] ^= 1
            for candidate in (bytes(bad), setup[0].frame.raw):
                with self.assertRaises(ProtocolError):
                    r.accept(candidate)
                self.assertEqual(snapshot(r), original)
            # Valid typed spindle effect, same count/final pose, different prefix.
            frame = setup[2].frame
            payload = frame.payload.tobytes()
            identity = 2 + len(sim.state.manifest['run']['id']) + 2 + len(p.revision) + 32
            offset = identity + 8 + 40 + 24
            effect = json.dumps(Spindle(8000).to_dict(), separators=(',', ':')).encode()
            altered = reframe(frame, payload[:offset] + struct.pack('<I', len(effect)) + effect)
            r.accept(altered)
            before = snapshot(r)
            for operation in (lambda: r.accept(movement.frame), lambda: r.partial(preview), lambda: r.accept(end)):
                with self.assertRaisesRegex(ProtocolError, 'control prefix'):
                    operation()
                self.assertEqual(snapshot(r), before)
            proper = StockState(sim.manifest_frame)
            for update in setup:
                proper.accept(update.frame)
            proper.accept(movement.frame)
            before = snapshot(proper)
            broken = bytearray(end.payload)
            broken[-1] ^= 1
            with self.assertRaisesRegex(ProtocolError, 'control prefix'):
                proper.accept(reframe(end, broken))
            self.assertEqual(snapshot(proper), before)
            proper.accept(end)
            self.assertTrue(proper.finished)

    def test_foreign_control_and_bad_checkpoint_state_keep_old_buffers_and_control_owner(self):
        p, m = job()
        with (
            simulation(p, run_id='one') as one,
            simulation(p, run_id='two') as two,
            tempfile.TemporaryDirectory() as directory,
        ):
            foreign = next(two.events(setup_events(p, m)[:1]))
            before = snapshot(one.state)
            with self.assertRaisesRegex(ProtocolError, 'foreign'):
                one.state.accept(foreign.frame)
            self.assertEqual(snapshot(one.state), before)
            list(one.events(setup_events(p, m)))
            path = Path(directory) / 'checkpoint'
            one.state.write_checkpoint(path)
            original = path.read_bytes()
            before = snapshot(one.state)
            # Recompute CRC to exercise native typed-state validation, not only CRC.
            frame = Frame.parse(original)
            payload = frame.payload.tobytes()
            identity = 2 + len('one') + 2 + len(p.revision) + 32
            offset = identity + struct.calcsize('<Q3dBQQI')
            (size,) = struct.unpack_from('<I', payload, offset)
            state = json.loads(payload[offset + 4 : offset + 4 + size])
            state['selected']['tool'] = 'unknown'
            raw = json.dumps(state, separators=(',', ':')).encode()
            path.write_bytes(
                reframe(frame, payload[:offset] + struct.pack('<I', len(raw)) + raw + payload[offset + 4 + size :]).raw
            )
            with self.assertRaisesRegex(ProtocolError, 'control checkpoint'):
                one.state.restore_checkpoint(path)
            self.assertEqual(snapshot(one.state), before)
            for raw in (original[:-1], original[:-1] + bytes([original[-1] ^ 1])):
                path.write_bytes(raw)
                with self.assertRaises(ProtocolError):
                    one.state.restore_checkpoint(path)
                self.assertEqual(snapshot(one.state), before)
            path.write_bytes(original)
            one.state.restore_checkpoint(path)
            self.assertEqual(one.state.controls, before[6])

    def test_query_resynchronizes_same_move_count_and_full_stock_and_partial_views_own_prefix(self):
        p, m = job((4, 1, 5))
        with simulation(p) as sim:
            list(sim.events(setup_events(p, m)))
            base = sim.state.clone()
            with PrefixSession(sim.state.canonical_manifest) as query:
                shown, _ = query.preview(sim.state, p.events[4], 0.5, compact=True)
                self.assertEqual(query.checkpoint_loads, 1)
                self.assertEqual(shown.control_cursor, sim.state.control_cursor)
                self.assertEqual(shown.controls, sim.state.controls)
                before = sim.state.heights.tobytes()
                list(sim.events([Stop(True)]))
                self.assertEqual(sim.state.heights.tobytes(), before)
                with self.assertRaisesRegex(ProtocolError, 'base changed'):
                    shown.display_patch((20, 20), None)
                query.preview(sim.state, p.events[4], 0.75)
                self.assertEqual(query.checkpoint_loads, 2)
                list(sim.events([Spindle(0), p.events[7], Spindle(4000)]))
                shown, _ = query.preview(sim.state, p.events[9], 0.5, compact=True)
                self.assertEqual(query.checkpoint_loads, 3)
                self.assertEqual(shown.controls['selected']['tool'], 'edge2')
                self.assertEqual(base.controls['selected']['tool'], 't1')
                self.assertEqual(base.control_cursor[0], 3)
                self.assertEqual(sim.state.completed, 0)

    def test_mixed_file_replay_and_bad_acknowledgement_do_not_publish_or_reuse_unknown_cache(self):
        p, m = job()
        with simulation(p) as sim, tempfile.TemporaryDirectory() as directory:
            updates = list(sim.events(setup_events(p, m)))
            path = Path(directory) / 'history.bin'
            first = b''.join(u.frame.raw for u in updates)
            path.write_bytes(first)
            with PrefixSession(sim.state.canonical_manifest) as query:
                original = query._next

                def bad_ack():
                    frame = original()
                    payload = bytearray(frame.payload)
                    payload[-1] ^= 1
                    return reframe(frame, payload)

                before = snapshot(sim.state)
                with (
                    patch.object(query, '_next', side_effect=bad_ack),
                    self.assertRaisesRegex(ProtocolError, 'control prefix'),
                ):
                    query.replay(sim.state, path, 0, len(first))
                self.assertEqual(snapshot(sim.state), before)
                self.assertIsNone(query._loaded_token)
                self.assertEqual(query._cached_complete, -1)
                query.preview(sim.state, p.events[4], 0.5)
                self.assertEqual(query.checkpoint_loads, 1)
            with PrefixSession(sim.state.canonical_manifest) as query:
                query.replay(sim.state, path, 0, len(first))
                query.preview(sim.state, p.events[4], 0.5)
                self.assertEqual(query.checkpoint_loads, 0)
                self.assertEqual(query.replayed_chunks, 3)
                more = list(sim.events([p.events[4], Spindle(0), p.events[7], Spindle(4000)]))
                second = b''.join(u.frame.raw for u in more)
                path.write_bytes(first + second)
                query.replay(sim.state, path, len(first), len(second))
                shown, _ = query.preview(sim.state, p.events[9], 0.371)
                self.assertEqual(query.replayed_chunks, 7)
                self.assertEqual(query.checkpoint_loads, 0)
                self.assertNotEqual(shown.heights.tobytes(), sim.state.heights.tobytes())

    def test_cancellation_and_source_failure_close_owned_iterators_and_keep_partial_prefix(self):
        p, m = job()
        closed = []
        seen = []

        def events():
            try:
                for event in (*setup_events(p, m), p.events[4]):
                    seen.append(event)
                    yield event
                raise RuntimeError('source failed')
            finally:
                closed.append(True)

        with simulation(p) as sim:
            stream = sim.run_events(events(), batch_size=2)
            next(stream)
            next(stream)
            self.assertEqual(len(seen), 2)
            sim.cancel()
            with self.assertRaises(SimulationError):
                next(stream)
            self.assertEqual(len(seen), 2)
            self.assertTrue(closed)
            self.assertFalse(sim.state.finished)
            self.assertEqual(sim.state.control_cursor[0], 2)
            self.assertIsNotNone(sim._process.poll())
        closed.clear()
        seen.clear()
        with simulation(p) as sim:
            with self.assertRaisesRegex(RuntimeError, 'source failed'):
                list(sim.run_events(events(), batch_size=1))
            self.assertTrue(closed)
            self.assertEqual(sim.state.completed, 1)
            self.assertFalse(sim.state.finished)

    def test_event_bounds_control_only_finish_and_actual_motion_check_coverage(self):
        p, m = job()
        with simulation(p) as sim:
            before = snapshot(sim.state)
            with self.assertRaisesRegex(ValueError, '4096'):
                list(sim.events([Stop()] * 4097))
            self.assertEqual(snapshot(sim.state), before)
            updates = list(sim.run_events([Stop(), ToolChange('edge2', 2, 8, (100, 100, 50))], batch_size=1))
            self.assertEqual(len(updates), 2)
            self.assertTrue(sim.state.finished)
            self.assertEqual(sim.state.completed, 0)
            self.assertEqual(sim.state.position, (100, 100, 50))
            self.assertEqual(sim.state.heights.tobytes(), before[7])
        with simulation(p, verification=StreamChecks(stock_contact=StockContactOptions())) as sim:
            actual = (e for chunk in post_events(p, p.events, m) for e in chunk.events)
            updates = list(sim.run_events(actual, batch_size=4))
            self.assertEqual(sum(isinstance(u, ControlUpdate) for u in updates), 11)
            self.assertEqual(sum(isinstance(u, VerificationUpdate) for u in updates), 2)
            self.assertEqual(sim.verification_update.progress['through_completed'], 2)
            self.assertEqual(sim.state.controls['unverified_tool_routes'], 2)

    def test_version1_rejects_event_api_before_pulling_and_contexts_are_immutable(self):
        p, m = job()
        seen = []
        with Simulation(
            StockSpec((0, 0, 0), (20, 20, 10), 0.5), p.tools, revision='1', initial_position_mm=p.initial_position_mm
        ) as sim:

            def source():
                seen.append(True)
                yield Stop()

            with self.assertRaisesRegex(ValueError, 'stream_version=2'):
                next(sim.events(source()))
            self.assertFalse(seen)
            self.assertIsNone(sim.state.controls)
        with simulation(p) as sim:
            context = sim.state._control_context
            with self.assertRaises(AttributeError):
                context.extra = 'mutable'
            with self.assertRaisesRegex(ValueError, 'missing field.*cursor'):
                context.restored(b'{}', 0, p.initial_position_mm)
            self.assertEqual(sim.state.control_cursor[0], 0)


if __name__ == '__main__':
    unittest.main()
