"""Retain owned checked NC/source maps alongside native stock/check playback.

See docs/posts.md. Integrity of an owned producer, not authentication
or qualification of arbitrary imported NC/proof bytes.
"""

from copy import deepcopy
from dataclasses import replace
import collections
import hashlib
import json
import math
import struct
import uuid

from .playback import RecordedRun, NotReady
from .post import Cancellation, GCodePost, Machine, NativeGCodeDecoder, PostEngine
from .simulation import MaterialStage, OperationUpdate, VerificationUpdate, ProtocolError, SimulationError, _Decoder


_LIMIT = 512 * 1024
_MANIFEST_LIMIT = 2 * 1024 * 1024
_FRAME = struct.Struct('<8sQI32s32s')
_CHUNK = struct.Struct('<QIIQQQQ32s')
_MOVE = struct.Struct('<QIIQ32s')


def _encode(value):
    return json.dumps(value, sort_keys=True, allow_nan=False, separators=(',', ':')).encode()


def _sha(raw):
    return hashlib.sha256(raw).digest()


_KINDS = ('rapid', 'cut', 'plunge', 'lead', 'link', 'retract', 'tap', 'shift', 'thread')
_CURVES = ('line', 'arc', 'helix')
_PLANES = ('xy', 'xz', 'yz')


def _motion_digest(m):
    """Digest of a motion's exact values in a fixed binary layout (identical
    for the posted proof and the committed stock frame of the same motion)."""
    c = m['curve']
    feed = m['feed_mm_min']
    parts = [
        struct.pack('<Q', m['id']),
        m['operation'].encode(),
        b'\0',
        m['tool'].encode(),
        b'\0',
        struct.pack('<BBd', _KINDS.index(m['kind']), feed is not None, 0.0 if feed is None else feed),
        struct.pack('<B6d', _CURVES.index(c['kind']), *c['start'], *c['end']),
    ]
    if c['kind'] != 'line':
        parts.append(struct.pack('<3dBd', *c['center'], _PLANES.index(c['plane']), c['sweep_rad']))
    return hashlib.sha256(b''.join(parts)).digest()


def check_tool_change_pose(machine, config):
    """Before posting: the setup's tool-change pose (``Setup.tool_change``,
    where the planner starts each new tool) must be where the machine's tool
    change leaves it (``Machine.tool_change_position_mode`` and the tool
    table's ``position_after_mm``). Compares every pose fixed before the run;
    the posted output check compares each actual change again. Raises
    ValueError naming the tool, both poses and the setup to declare."""
    if 'turning' in config:
        return
    mode = machine.tool_change_position_mode
    change = config.get('tool_change')
    gauges = {t['id']: t['gauge_length_mm'] for t in config['tools']}
    used = list(dict.fromkeys(op['tool'] for op in config['operations']))
    initial = tuple(config['initial_position_mm'])
    tolerance = machine.tolerance_mm

    def planned(tool, first):
        # (x or None, y or None, z) the planner gives the tip after the change.
        if change is None:
            return initial if first else None
        if 'tips_mm' in change:
            return tuple(change['tips_mm'][tool]) if tool in change['tips_mm'] else None
        xy = change.get('xy_mm') or (initial[:2] if first else (None, None))
        return (*xy, change['gauge_z_mm'] - gauges[tool])

    def describe():
        if change is None:
            return 'Setup.tool_change is None: the tip stays where it was'
        if 'tips_mm' in change:
            return 'Setup.tool_change is a ToolChangeTable'
        return (
            f'Setup.tool_change is ToolChangePosition(gauge_z_mm={change["gauge_z_mm"]}, xy_mm={change.get("xy_mm")})'
        )

    if mode == 'preserve_tip':
        if change is not None:
            raise ValueError(
                f'the machine keeps the tip where it was at a tool change (tool_change_position_mode="preserve_tip"), '
                f'but {describe()}: plan with Setup(tool_change=None)'
            )
        return
    if mode == 'z_home':
        gauge_z = machine.tool_change_home_z_mm - machine.work_to_machine.origin_mm[2]
        fix = f'plan with Setup(tool_change=ToolChangePosition(gauge_z_mm={gauge_z}))'
        if change is not None and ('tips_mm' in change or change.get('xy_mm') is not None):
            raise ValueError(
                f'M6 sends the spindle to Z home and keeps XY (tool_change_position_mode="z_home"), '
                f'but {describe()}: {fix}'
            )
        tips = {tool: {(None, None, gauge_z - gauges[tool])} for tool in used}
        source = f'Z home: gauge line at work Z {gauge_z}'
    else:
        tips = {s.tool: {tuple(float(v) for v in s.position_after_mm)} for s in machine.tools if s.tool in gauges}
        missing = [tool for tool in used if tool not in tips]
        if missing:
            raise ValueError(f'the machine tool table has no entry for tool(s) {missing}: add a ToolSetting for each')
        source = 'its tool table position_after_mm'
        fix = 'plan with Setup(tool_change=ToolChangeTable.from_machine(machine))'
    for k, tool in enumerate(used):
        plan = planned(tool, k == 0)
        if plan is None:
            continue

        def agrees(tip):
            return all(a is None or b is None or abs(a - b) <= tolerance for a, b in zip(plan, tip))

        if not any(agrees(tip) for tip in tips[tool]):
            (shown,) = tips[tool]
            raise ValueError(
                f'tool {tool}: the setup starts it at {plan} after its tool change ({describe()}), '
                f'but the machine leaves it at {shown} '
                f'(tool_change_position_mode="{mode}", {source}); {fix}'
            )


def _is_stop(events):
    kinds = [e['kind'] for e in events]
    return sorted(kinds) == ['coolant', 'spindle'] and all(
        e.get('mode') == 'off' if e['kind'] == 'coolant' else e.get('rpm') == 0 for e in events
    )


def _begin_modal(events):
    """((work offset, tool assignment, spindle, coolant), declared pose) of a
    standard operation start, or None for anything else (never merged)."""
    by = {e['kind']: e for e in events}
    if len(by) != len(events) or set(by) != {'work_offset', 'tool_change', 'spindle', 'coolant'}:
        return None
    t, s = by['tool_change'], by['spindle']
    if s.get('rpm', 0) == 0:
        return None
    return (
        (
            by['work_offset']['number'],
            (t['tool'], t['number'], t['length_register']),
            (s['rpm'], s.get('direction')),
            by['coolant']['mode'],
        ),
        tuple(t['position_after_mm']),
    )


class OperationEvents:
    """Take ownership of a native operation Simulation and expose its typed events.

    Cancellation forwards to that process. Source check updates are not a claim
    about posted stock; PostedRun performs and retains its own configured checks.
    ``reuse_active_tool`` omits identical native boundary tool assignments only
    when they declare no change to the current pose. Real changes remain events.
    With it, an operation that continues with the same tool, spindle speed and
    direction, coolant and work offset also omits the previous operation's
    spindle/coolant stop and its own restart (as commercial CAM does between
    sections). Any difference keeps every stop and start. Segmented runs never
    merge across a segment boundary.
    """

    def __init__(self, simulation, *, reuse_active_tool=False, on_operation=None, on_verification=None):
        self.simulation = simulation
        self._updates = iter(simulation.run_operations())
        self._controls = collections.deque()
        self._on_verification = on_verification
        self._reuse_active_tool = reuse_active_tool
        self._active_tool = None
        self._on_operation = on_operation
        self.segmented = False
        self.boundary = None
        self._pending_boundary = None
        self._held = None
        self._modal = None
        # Whether a poster may read events ahead of its output (Job.post_nc,
        # which collects the whole text); interactive playback leaves it off.
        self.prefetch = False
        self._position = tuple(simulation.state.manifest['initial_position_mm'])

    def __iter__(self):
        return self

    def ready(self):
        """Whether the next event comes from memory, without waiting for the
        simulation: a buffered control event (not a tool assignment that may
        be dropped as a repeat), or a committed motion already decoded."""
        if self.segmented and self.boundary is not None:
            return False
        if self._controls:
            return not (self._reuse_active_tool and self._controls[0]['kind'] == 'tool_change')
        if self._held is not None or self._pending_boundary is not None:
            return False
        buffered = getattr(self.simulation, 'buffered_motion', None)
        return buffered is not None and buffered()

    def __next__(self):
        while True:
            if self.segmented and self.boundary is not None:
                raise StopIteration
            try:
                if not self._controls:
                    raise StopIteration
                event = self._controls.popleft()
                if self._reuse_active_tool and event['kind'] == 'tool_change':
                    assignment = (event['tool'], event['number'], event['length_register'])
                    # Native operation boundaries declare the assembly anew.
                    # Keep a real reassignment or pose reset; omit only an
                    # identical assignment that leaves the current pose intact.
                    same = assignment == self._active_tool and tuple(event['position_after_mm']) == self._position
                    self._active_tool = assignment
                    self._position = tuple(event['position_after_mm'])
                    if same:
                        continue
                return event
            except StopIteration:
                pass
            if self._held is not None and self.segmented:
                held, self._held = self._held, None
                self._controls = collections.deque(held)
                continue
            if self._pending_boundary is not None:
                boundary = self._pending_boundary
                self._pending_boundary = None
                if self.segmented:
                    self.boundary = boundary
                    raise StopIteration
            try:
                update = next(self._updates)
            except StopIteration:
                if self._held is None:
                    raise
                held, self._held = self._held, None
                self._controls = collections.deque(held)
                continue
            if isinstance(update, OperationUpdate):
                if self._on_operation is not None:
                    self._on_operation(update)
                events = list(update.events)
                if update.stage == "end" and self._reuse_active_tool and self._held is None and _is_stop(events):
                    self._held, events = events, []
                elif update.stage == "begin":
                    events = self._continue(events)
                self._controls = collections.deque(events)
                if update.stage == "end":
                    self._pending_boundary = update.operation
            elif isinstance(update, VerificationUpdate):
                if self._on_verification is not None:
                    self._on_verification(update)
            else:
                self._position = tuple(update.motion['curve']['end'])
                return update.motion

    def _continue(self, events):
        """Begin events after any held stop: all-or-nothing modal continuation."""
        begin = _begin_modal(events)
        held, self._held = self._held, None
        modal = None if begin is None else begin[0]
        # Same offset, tool, spindle and coolant as the running state, and the
        # tool change declares no pose change: nothing would change on the machine.
        if (
            held is not None
            and modal is not None
            and modal == self._modal
            and modal[1] == self._active_tool
            and begin[1] == self._position
        ):
            return []
        self._modal = modal
        return (held or []) + events

    def cancel(self):
        self.simulation.cancel()

    def close(self):
        try:
            self._updates.close()
        finally:
            self.simulation.close()


class PostedRun(RecordedRun):
    """Normal Python posting -> bounded checked journal -> actual stock playback.

    Owns the event iterator and closes it on completion/failure/cancel. A blocking
    source may implement cancel(), or supply cancel_source, for cooperative stop.
    Use OperationEvents for native operations. No machine connection is made.
    """

    def __init__(
        self,
        program,
        events=None,
        *,
        machine,
        stock,
        post=GCodePost,
        decoder=NativeGCodeDecoder,
        lowering=None,
        cancel_source=None,
        cancel=None,
        **options,
    ):
        if not isinstance(machine, Machine):
            raise ValueError('PostedRun requires an ordinary Python Machine')
        if set(options) & {'motions', 'revision', 'initial_position_mm', 'tools', 'operations', 'tool_change'}:
            raise ValueError('posted tools/revision/pose come from the checked program; operations use OperationEvents')
        if 'rapid_mm_min' in options and options['rapid_mm_min'] != machine.rapid_mm_min:
            raise ValueError("the posted replay's rapid rate is the machine's (Machine.rapid_mm_min)")
        options['rapid_mm_min'] = machine.rapid_mm_min
        if machine.tool_change_position_mode == 'z_home':
            # The decoded program's tool changes go to Z home: the replay
            # stock run accepts exactly that pose.
            from .simulation import ToolChangePosition

            options['tool_change'] = ToolChangePosition(
                machine.tool_change_home_z_mm - machine.work_to_machine.origin_mm[2]
            )
        elif machine.tool_change_position_mode == 'table' and 'turning' not in options:
            # Each decoded tool change puts the tool at its table pose: the
            # replay stock run accepts exactly those poses.
            from .simulation import ToolChangeTable

            options['tool_change'] = ToolChangeTable.from_machine(machine, tools=[t.id for t in program.tools])
        owner = program.events if events is None else events
        self._events = iter(owner)
        self._event_owner = owner
        self._event_closed = False
        try:
            self._cancel_source = cancel_source if cancel_source is not None else getattr(owner, 'cancel', None)
            self._source_header = deepcopy(replace(program, events=()))
            run_id = options.pop('run_id', None) or uuid.uuid4().hex
            self._post_cancel = cancel if cancel is not None else Cancellation()
            self._engine = PostEngine(
                deepcopy(machine), post or GCodePost, decoder=decoder or NativeGCodeDecoder, lowering=lowering
            )
            self._output_tolerance_mm = machine.tolerance_mm
            self._begin = self._engine.begin(self._source_header, run_id=run_id, cancel=self._post_cancel)
            begin = self._begin.data
            self._post_identity = deepcopy(begin['identity'])
            self._post_run_id = run_id
            self._post_binding = None
            self._post_chunks = 0
            self._post_count = 0
            self._post_lines = 0
            self._post_finished = False
            self._post_largest = 0
            self._post_last_id = None
            self._post_scheduling = deepcopy(begin.get('motion_scheduling', {}).get('summary'))
            if (
                isinstance(owner, OperationEvents)
                and owner.simulation.state.manifest['run']['revision'] != program.revision
            ):
                raise ValueError('native source revision differs from posted program')
            config = options.get('verification')
            if hasattr(config, 'to_dict'):
                config = config.to_dict()
            self._material_stages = {
                c['after_operation'] for c in (config or {}).get('material', ()) if c['after_operation'] is not None
            }
            if self._material_stages:
                if not isinstance(owner, OperationEvents):
                    raise ValueError('posted operation material stages require OperationEvents')
                known = {op['id'] for op in owner.simulation.state.manifest['operations']}
                if not self._material_stages <= known:
                    raise ValueError('unknown source operation material stage')
                owner.segmented = True
            super().__init__(
                stock,
                self._source_header.tools,
                self._posted_motions(),
                revision=program.revision,
                initial_position_mm=tuple(begin['decoded_initial_position_mm']),
                run_id=run_id,
                **options,
            )
        except BaseException:
            self._close_events()
            raise

    def _posted_motions(self):
        try:
            manifest = dict(
                version=1,
                journal_id=uuid.uuid4().hex,
                run_id=self._post_run_id,
                identity=self._post_identity,
                source_header=self._source_header.to_dict(),
                definition=self._begin.data['definition'],
                lowering_policy=self._begin.data.get('lowering_policy'),
                stock_manifest_sha256=hashlib.sha256(self.wait_ready().payload).hexdigest(),
                scope=(
                    'Owned checked Python post; retained nominal source/NC attribution and actual stock checks, '
                    'not controller execution'
                ),
            )
            for key in ('nominal_normal_acceleration', 'nominal_axis_feed', 'motion_scheduling'):
                if key in self._begin.data:
                    manifest[key] = self._begin.data[key]
            raw = _encode(manifest)
            if len(raw) > _MANIFEST_LIMIT:
                raise SimulationError('posted manifest exceeds bounded size')
            self._store(self.directory / 'post-manifest.json', raw)
            with self._condition:
                self._stock_identity = (
                    self._post_run_id,
                    self._source_header.revision,
                    _sha(self.wait_ready().payload),
                )
                self._post_binding = _sha(raw)
            with (
                (self.directory / 'post-chunks.bin').open('xb') as chunks,
                (self.directory / 'post-chunks.idx').open('xb') as index,
                (self.directory / 'post-motions.idx').open('xb') as moves,
            ):
                self._append_post(self._begin, chunks, index, moves)
                self._begin = None
                while True:
                    for chunk in self._engine.consume_events(self._events, cancel=self._post_cancel):
                        data = self._append_post(chunk, chunks, index, moves)
                        # Motions from the authoritative raw proof: `data` is
                        # this module's own parse of chunk.raw, never a cache
                        # supplied by the caller.
                        yield from (check['motion'] for check in data['checked'] if check['motion'] is not None)
                        del data
                    boundary = getattr(self._event_owner, 'boundary', None)
                    if boundary is None:
                        break
                    if boundary in self._material_stages:
                        yield MaterialStage(boundary)
                    self._event_owner.boundary = None
                self._append_post(self._engine.finish(cancel=self._post_cancel), chunks, index, moves)
                with self._condition:
                    self._post_finished = True
                    self._condition.notify_all()
        finally:
            self._close_events()

    def _close_events(self):
        if self._event_closed:
            return
        self._event_closed = True
        close = getattr(self._events, 'close', None)
        try:
            if close is not None:
                close()
        finally:
            try:
                if self._event_owner is not self._events:
                    close = getattr(self._event_owner, 'close', None)
                    if close is not None:
                        close()
            finally:
                self._events = iter(())
                self._event_owner = None
                self._cancel_source = None

    def _produce(self):
        try:
            super()._produce()
        finally:
            try:
                self._close_events()
            except BaseException as error:
                with self._condition:
                    if not self._cancel.is_set() and self._error is None:
                        self._error = f'source cleanup: {type(error).__name__}: {error}'
                        self._finished = False

    def _append_post(self, chunk, chunks, index, moves):
        if self._cancel.is_set():
            raise SimulationError('posting cancelled before journal commit')
        raw = chunk.raw
        if not isinstance(raw, bytes) or not 0 < len(raw) <= _LIMIT:
            raise ProtocolError('invalid checked post chunk size')
        data = json.loads(raw)
        if data['identity'] != self._post_identity or data['run_id'] != self._post_run_id:
            raise ProtocolError('foreign post run/revision/configuration')
        lines = data['lines']
        checked = data['checked']
        maps = data['mappings']
        if (
            not lines
            and checked
            and len(checked) == len(maps)
            and all(c['motion'] is None and c['line_interval'][0] > c['line_interval'][1] for c in checked)
        ):
            # Events written in a later block (a compensation change carried by
            # its move): checked, nothing to replay or journal yet.
            return data
        if not lines or len(checked) != len(maps) or len(checked) > 64:
            raise ProtocolError('invalid bounded post proof layout')
        if [nc_line['number'] for nc_line in lines] != list(
            range(self._post_lines + 1, self._post_lines + len(lines) + 1)
        ):
            raise ProtocolError('noncontiguous post lines')
        rows = []
        last = self._post_last_id
        for n, (check, mapping) in enumerate(zip(checked, maps)):
            if check['motion'] is None:
                continue
            motion = check['motion']
            self._validate_link(data, n, mapping)
            if last is not None and motion['id'] <= last:
                raise ProtocolError('posted IDs are not increasing')
            last = motion['id']
            rows.append(_MOVE.pack(self._post_chunks, n, n, last, _motion_digest(motion)))
        scheduling = data.get('motion_scheduling', {}).get('summary')
        if (scheduling is None) != (self._post_scheduling is None):
            raise ProtocolError('missing/foreign scheduling summary')
        if scheduling is not None:
            if (
                scheduling['policy_sha256'] != self._post_scheduling['policy_sha256']
                or scheduling['decoded_motions'] != self._post_count + len(rows)
                or any(
                    not math.isfinite(scheduling[k]) or scheduling[k] < self._post_scheduling[k]
                    for k in (
                        'completed_source_events',
                        'completed_source_motions',
                        'limited_source_motions',
                        'source_exact_stop_seconds',
                        'decoded_exact_stop_seconds',
                    )
                )
            ):
                raise ProtocolError('invalid scheduling prefix progression')
        frame = _FRAME.pack(b'CAMPOST1', self._post_chunks, len(raw), self._post_binding, _sha(raw)) + raw
        descriptor = _CHUNK.pack(
            chunks.tell(),
            len(frame),
            0,
            lines[0]['number'],
            lines[-1]['number'],
            self._post_count,
            len(rows),
            _sha(frame),
        )
        self._quota(len(frame) + len(descriptor) + sum(map(len, rows)))
        positions = [f.tell() for f in (chunks, index, moves)]
        try:
            chunks.write(frame)
            index.write(descriptor)
            moves.write(b''.join(rows))
            for f in (chunks, index, moves):
                f.flush()
            with self._condition:
                if self._cancel.is_set():
                    raise SimulationError('posting cancelled; unpublished journal append rolled back')
                self._bytes += len(frame) + len(descriptor) + sum(map(len, rows))
                self._post_chunks += 1
                self._post_lines += len(lines)
                self._post_count += len(rows)
                self._post_scheduling = deepcopy(scheduling)
                self._post_last_id = last
                self._post_largest = max(self._post_largest, len(raw))
                self._condition.notify_all()
        except BaseException:
            for f, pos in zip((chunks, index, moves), positions):
                f.seek(pos)
                f.truncate()
                f.flush()
            raise
        return data

    @staticmethod
    def _validate_link(data, index, mapping):
        check = data['checked'][index]
        motion = check['motion']
        if (
            mapping['event_index'] != check['event_index']
            or mapping['posted_motion_id'] != motion['id']
            or mapping['line_interval'] != check['line_interval']
            or mapping['deviation_bound_mm'] != check['deviation_bound_mm']
            or not isinstance(mapping['source_event_sha256'], str)
            or len(mapping['source_event_sha256']) != 64
        ):
            raise ProtocolError('post source/check link mismatch')
        try:
            bytes.fromhex(mapping['source_event_sha256'])
        except ValueError:
            raise ProtocolError('invalid source event digest') from None
        if 'motion_scheduling' in data:
            records = [r for r in data['motion_scheduling']['records'] if r['check_index'] == index]
            if len(records) != 1:
                raise ProtocolError('missing/duplicate scheduling motion record')
            record = records[0]
            if (
                record['event_index'] != mapping['event_index']
                or record['source_motion_id'] != mapping['source_motion_id']
                or record['source_fraction_interval'] != mapping['source_fraction_interval']
                or record['decoded_feed_mm_min'] != motion['feed_mm_min']
            ):
                raise ProtocolError('scheduling source/decoded link mismatch')
            for key in (
                'authored_feed_mm_min',
                'configured_feed_mm_min',
                'scheduled_feed_mm_min',
                'decoded_feed_mm_min',
            ):
                value = record[key]
                if value is not None and (not math.isfinite(value) or value <= 0):
                    raise ProtocolError('invalid scheduling feed')
            original = record['configured_feed_mm_min']
            scheduled = record['scheduled_feed_mm_min']
            if (original is None) != (scheduled is None) or original is not None and scheduled > original:
                raise ProtocolError('scheduling increased or invented feed')
            digest = record['scheduled_source_event_sha256']
            if not isinstance(digest, str) or len(digest) != 64:
                raise ProtocolError('invalid scheduled source digest')
            try:
                bytes.fromhex(digest)
            except ValueError:
                raise ProtocolError('invalid scheduled source digest') from None
        a, b = mapping['source_fraction_interval']
        if not (math.isfinite(a) and math.isfinite(b) and 0 <= a < b <= 1):
            raise ProtocolError('invalid nominal source fraction')
        start, end = check['line_interval']
        if check.get('cycle_event_index') is not None:
            # A move inside a canned-cycle block: the cycle's lines carry it.
            if end != start - 1 or check['cycle_event_index'] >= check['event_index']:
                raise ProtocolError('NC line/source-motion link mismatch')
            return
        selected = [nc_line for nc_line in data['lines'] if start <= nc_line['number'] <= end]
        if (
            not selected
            or len(selected) != end - start + 1
            or len(selected) > 8
            or any(
                nc_line['motion_id'] != mapping['source_motion_id']
                or nc_line['operation'] != motion['operation']
                or nc_line['event_index'] != check['event_index']
                for nc_line in selected
            )
        ):
            raise ProtocolError('NC line/source-motion link mismatch')

    def posting_status(self):
        with self._condition:
            return dict(
                run_id=self._post_run_id,
                identity=deepcopy(self._post_identity),
                checked_motions=self._post_count,
                chunks=self._post_chunks,
                lines=self._post_lines,
                finished=self._post_finished,
                max_chunk_bytes=self._post_largest,
                **({'arc_output': arc} if (arc := self._arc_output_status()) is not None else {}),
                **({'motion_scheduling': deepcopy(self._post_scheduling)} if self._post_scheduling is not None else {}),
            )

    def _arc_output_status(self):
        # Live lowered runs report their engine's counts; reopened snapshots
        # report the counts saved with them.
        saved = getattr(self, '_post_arc_output', None)
        if saved is not None:
            return deepcopy(saved)
        engine = getattr(self, '_engine', None)
        return dict(engine.arc_output) if getattr(engine, 'lowering', None) is not None else None

    def status(self):
        with self._condition:
            value = super().status()
            value['posting'] = self.posting_status()
            return value

    def posting_manifest(self):
        with self._condition:
            binding = self._post_binding
        if binding is None:
            raise NotReady('posted journal manifest not published')
        p = self.directory / 'post-manifest.json'
        if not p.is_file() or p.stat().st_size > _MANIFEST_LIMIT:
            raise ProtocolError('missing/oversized posted manifest')
        raw = p.read_bytes()
        if _sha(raw) != binding:
            raise ProtocolError('foreign/corrupt posted manifest')
        value = json.loads(raw)
        if value['stock_manifest_sha256'] != self._stock_identity[2].hex():
            raise ProtocolError('posted/stock manifest binding mismatch')
        return value

    def _verify_manifest(self):
        """posting_manifest's checks for each chunk read: the bytes on disk
        are read and hashed every time; their parse and stock binding check
        are reused only for the exact digest already verified."""
        with self._condition:
            binding = self._post_binding
        if binding is None:
            raise NotReady('posted journal manifest not published')
        p = self.directory / 'post-manifest.json'
        if not p.is_file() or p.stat().st_size > _MANIFEST_LIMIT:
            raise ProtocolError('missing/oversized posted manifest')
        digest = _sha(p.read_bytes())
        if digest != binding:
            raise ProtocolError('foreign/corrupt posted manifest')
        if self.__dict__.get('_manifest_verified') != (digest, self._stock_identity[2]):
            self.posting_manifest()
            self.__dict__['_manifest_verified'] = (digest, self._stock_identity[2])

    def _post_entry(self, index):
        with self._condition:
            count = self._post_chunks
        if type(index) is not int or not 0 <= index < count:
            raise NotReady('post chunk not published')
        with (self.directory / 'post-chunks.idx').open('rb') as f:
            f.seek(index * _CHUNK.size)
            raw = f.read(_CHUNK.size)
        if len(raw) != _CHUNK.size:
            raise ProtocolError('missing post chunk index')
        row = _CHUNK.unpack(raw)
        if row[2] or not _FRAME.size < row[1] <= _FRAME.size + _LIMIT or not 1 <= row[3] <= row[4] or row[6] > 64:
            raise ProtocolError('invalid post chunk index')
        return row

    def _read_post(self, index):
        self._verify_manifest()
        offset, length, _, first, last, ordinal, count, digest = self._post_entry(index)
        path = self.directory / 'post-chunks.bin'
        if offset + length > path.stat().st_size:
            raise ProtocolError('missing post chunk extent')
        with path.open('rb') as f:
            f.seek(offset)
            raw = f.read(length)
        if len(raw) != length or _sha(raw) != digest:
            raise ProtocolError('post chunk integrity mismatch')
        # Every read verifies the bytes on disk; only the parse of bytes with
        # this exact verified digest is reused (sequential NC paging reads
        # each chunk several times).
        cache = self.__dict__.setdefault('_post_cache', {})
        cached = cache.get((index, digest))
        if cached is not None:
            return cached
        magic, seq, size, binding, checksum = _FRAME.unpack_from(raw)
        payload = raw[_FRAME.size :]
        if (
            magic != b'CAMPOST1'
            or seq != index
            or size != len(payload)
            or binding != self._post_binding
            or _sha(payload) != checksum
        ):
            raise ProtocolError('foreign/corrupt post frame')
        data = json.loads(payload)
        if (
            data['identity'] != self._post_identity
            or data['run_id'] != self._post_run_id
            or len(data['checked']) != len(data['mappings'])
        ):
            raise ProtocolError('post proof identity/layout mismatch')
        if [nc_line['number'] for nc_line in data['lines']] != list(range(first, last + 1)) or sum(
            c['motion'] is not None for c in data['checked']
        ) != count:
            raise ProtocolError('post chunk/index range mismatch')
        if len(cache) >= 8:
            cache.pop(next(iter(cache)))
        cache[(index, digest)] = (data, ordinal, count)
        return data, ordinal, count

    def posted_motion(self, index, *, computed=False):
        with self._condition:
            published = self._post_count
            available = self._available
        if type(index) is not int or not 0 <= index < (published if computed else available):
            raise NotReady('posted motion not available in requested prefix')
        with (self.directory / 'post-motions.idx').open('rb') as f:
            f.seek(index * _MOVE.size)
            raw = f.read(_MOVE.size)
        if len(raw) != _MOVE.size:
            raise ProtocolError('missing posted motion index')
        chunk, n, m, move_id, digest = _MOVE.unpack(raw)
        data, ordinal, count = self._read_post(chunk)
        if not ordinal <= index < ordinal + count or not n < len(data['checked']) or m != n:
            raise ProtocolError('invalid post motion/check index')
        check = deepcopy(data['checked'][n])
        mapping = deepcopy(data['mappings'][m])
        motion = check['motion']
        if motion is None or motion['id'] != move_id or _motion_digest(motion) != digest:
            raise ProtocolError('posted motion identity/digest mismatch')
        if sum(c['motion'] is not None for c in data['checked'][:n]) != index - ordinal:
            raise ProtocolError('posted motion ordinal mismatch')
        self._validate_link(data, n, mapping)
        if index < available:
            frame = self.read_chunk(index)
            d = _Decoder(frame.payload)
            if (
                frame.kind not in (1, 10)
                or (d.string(128), d.string(512), d.take(32).tobytes()) != self._stock_identity
                or d.unpack('<3Q') != (index, index, index + 1)
                or _motion_digest(d.motion()) != digest
            ):
                raise ProtocolError('posted proof differs from committed stock motion')
        a, b = mapping['line_interval']
        result = dict(
            ordinal=index,
            stock_committed=index < available,
            motion=motion,
            mapping=mapping,
            nc=[dict(nc_line) for nc_line in data['lines'] if a <= nc_line['number'] <= b],
        )
        if 'motion_scheduling' in data:
            result['motion_scheduling'] = deepcopy(
                next(r for r in data['motion_scheduling']['records'] if r['check_index'] == n)
            )
        return result

    def check_machine(self, *, timeout=300):
        """Check every retained actual decoded event against the bound model.

        Replays stock with the native kernel. This checks declared geometry
        and controller routes, not the physical setup on a real machine.
        """
        from ._buffers import native

        self.wait(timeout)
        if not self.posting_status()["finished"]:
            raise ValueError("machine check needs the finished checked program")
        manifest = self.posting_manifest()
        model = manifest["definition"]["machine"].get("crash_model")
        if model is None:
            return dict(
                status="unchecked_machine",
                complete=False,
                findings=[
                    dict(
                        kind="unchecked_machine",
                        bodies=["machine", "program"],
                        detail="no declared machine crash model; supply Machine.crash_model or explicitly accept unchecked_machine",
                    )
                ],
            )
        # Configuration errors are actionable errors, never clear results.
        session = native.MachineCrashSession(
            _encode(model).decode(),
            self.wait_ready().payload.tobytes().decode(),
            _encode(manifest["definition"]).decode(),
        )
        try:
            for index in range(self._post_chunks):
                data, _, _ = self._read_post(index)
                session.push(_encode(data).decode(), getattr(self, "_post_cancel", None))
            return json.loads(
                session.finish(self.posting_status()["checked_motions"], getattr(self, "_post_cancel", None))
            )
        except ValueError as error:
            return json.loads(session.incomplete(str(error)))

    def export(self, path=None, *, accept=(), timeout=300):
        """Write the finished program to ``path`` (atomically) once its checks
        allow it: every violation or uncertain result, from the motion checks
        and from material (a definite gouge is a violation; a possible gouge
        or an incomplete analysis is uncertain), must be accepted by kind in
        ``accept``, or ``camlib.export.ExportRefused`` is raised and nothing
        is written. Returns the findings, accepted kinds and unchecked checks.
        """
        from pathlib import Path

        from .export import accepted, gate, material_status, unchecked

        kinds = accepted(accept)
        self.wait(timeout)
        if not self.posting_status()["finished"]:
            raise ValueError("export needs the finished, checked program")
        findings = []
        start = 0
        while page := list(self.verification_records(start, limit=256)):
            for update in page:
                status = update.status
                detail = list(update.findings())
                if update.kind == "material":
                    # The material report itself is the finding (as for a lathe).
                    report, proof = update.check["report"], update.check.get("sweep_proof")
                    status = material_status(report, proof)
                    detail = [report if proof is None else {**report, "sweep_proof": proof}]
                if status not in (None, "clear"):
                    findings.append(dict(completed=update.completed, status=status, findings=detail))
            start += len(page)
        material = None
        if self._config.get("turning") is not None:
            # A lathe program: its replayed stock compared with the part.
            from .playback import Cursor, Player

            player = Player(self)
            player.seek(Cursor(self.status()["available"]))
            material = player.state.material()
            status = material_status(material)
            if status != "clear":
                findings.append(dict(completed=player.cursor.completed, status=status, findings=[material]))
        machine = self.check_machine(timeout=timeout)
        if machine["status"] != "clear":
            # Preserve each severity: accepting uncertainty must not accept a
            # definite collision elsewhere in the same machine report.
            groups = {}
            for finding in machine["findings"]:
                status = finding.get("status", machine["status"])
                groups.setdefault(status, []).append(finding)
            for status, detail in groups.items():
                findings.append(dict(completed=None, status=status, findings=detail))
        gate(findings, kinds)
        released_statuses = {f["status"] for f in findings}
        # Missing-model acceptance has its own explicit machine report and
        # unchecked entry; keep geometric findings focused on checked results.
        findings = [f for f in findings if f["status"] != "unchecked_machine"]
        text = None
        if path is None:
            number = 1
            parts = []
            while page := self.nc_lines(number, limit=256):
                parts.extend(line["text"] + "\n" for line in page)
                number += len(page)
            text = "".join(parts)
        target = Path(path) if path is not None else None
        if target is not None:
            from tempfile import NamedTemporaryFile

            temporary = None
            try:
                with NamedTemporaryFile(
                    mode="w",
                    encoding="ascii",
                    dir=target.parent,
                    prefix=target.name + ".",
                    suffix=".writing",
                    delete=False,
                ) as output:
                    temporary = Path(output.name)
                    number = 1
                    while page := self.nc_lines(number, limit=256):
                        output.writelines(line["text"] + "\n" for line in page)
                        number += len(page)
                temporary.replace(target)
            finally:
                if temporary is not None:
                    temporary.unlink(missing_ok=True)
        return dict(
            path=str(target) if target is not None else None,
            text=text,
            motions=self.posting_status()["checked_motions"],
            machine=machine,
            lines=number - 1,
            findings=tuple(findings),
            accepted=tuple(sorted(kinds & released_statuses)),
            unchecked=unchecked(self._verification_config(), turning=self._config.get('turning') is not None)
            + (("machine",) if machine["status"] == "unchecked_machine" else ()),
            material=material,
        )

    def nc_lines(self, start=1, *, limit=32):
        if type(start) is not int or start < 1 or type(limit) is not int or not 1 <= limit <= 256:
            raise ValueError('NC page requires start>=1 and limit 1..256')
        with self._condition:
            count = self._post_chunks
            lines = self._post_lines
        if start > lines:
            return []
        low, high = 0, count
        while low < high:
            middle = (low + high) // 2
            if self._post_entry(middle)[4] < start:
                low = middle + 1
            else:
                high = middle
        out = []
        while low < count and len(out) < limit:
            data, _, _ = self._read_post(low)
            out.extend(dict(nc_line) for nc_line in data['lines'] if start <= nc_line['number'] < start + limit)
            low += 1
            if data['lines'][-1]['number'] >= start + limit - 1:
                break
        return out

    @staticmethod
    def _display_mapping(mapping):
        return {
            **mapping,
            'source_motion_id': str(mapping['source_motion_id']),
            'posted_motion_id': str(mapping['posted_motion_id']),
        }

    def cursor_metadata(self, cursor, motion):
        context = None
        if motion is not None:
            index = cursor.completed if cursor.fraction else cursor.completed - 1
            value = self.posted_motion(index)
            if _encode(value['motion']) != _encode(motion):
                raise ProtocolError('displayed motion/source mapping mismatch')
            mapping = value['mapping']
            a, b = mapping['source_fraction_interval']
            context = dict(
                posted_motion_id=str(motion['id']),
                operation=motion['operation'],
                mapping=self._display_mapping(mapping),
                nominal_source_fraction=a + (b - a) * (cursor.fraction if cursor.fraction else 1),
                nc=[{**nc_line, 'motion_id': str(nc_line['motion_id'])} for nc_line in value['nc']],
            )
        return dict(
            run_id=self._post_run_id,
            identity=deepcopy(self._post_identity),
            tolerance_mm=self._output_tolerance_mm,
            context=context,
        )

    def verification_status(self, completed=None, *, sample_limit=8):
        result = super().verification_status(completed, sample_limit=sample_limit)
        for sample in result.get('samples', []):
            if not sample.get('completed'):
                continue
            item = self.posted_motion(sample['completed'] - 1)
            if sample['kind'] == 'motion':
                if sample.get('motion_id', item['motion']['id']) != item['motion']['id']:
                    raise ProtocolError('finding/post attribution mismatch')
                sample['motion_id'] = str(item['motion']['id'])
                sample.setdefault('operation', item['motion']['operation'])
                sample.setdefault('tool', item['motion']['tool'])
                sample.setdefault('fraction_interval', [0.0, 1.0])
                sample.setdefault('status', 'uncertain')
                sample.setdefault('seconds', self.motion_time(sample['completed'] - 1)[1])
                mapping = item['mapping']
                a, b = mapping['source_fraction_interval']
                u, v = sample.get('fraction_interval', (0, 1))
                sample['posted'] = dict(
                    mapping=self._display_mapping(mapping),
                    nominal_source_fraction_interval=[a + (b - a) * u, a + (b - a) * v],
                )
            else:
                sample['posted_through'] = dict(
                    posted_motion_id=str(item['motion']['id']), line_interval=item['mapping']['line_interval']
                )
        return result

    def cancel(self):
        callback = self._cancel_source
        self._post_cancel.cancel()
        try:
            super().cancel()
        finally:
            if callback is not None:
                callback()
