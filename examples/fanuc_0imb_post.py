"""Ordinary Python FANUC 0i-MB B-63844EN/02 reference subset.

See docs/fanuc-0imb-post.md. This edition is not a claim for other FANUC series.
"""

from dataclasses import dataclass, replace
import math
import re

from camlib.post import Machine, GCodeDialect, GCodePost, NativeGCodeDecoder, LoweringOptions, post_events

_OEM = {'tool_change', 'spindle_cw', 'spindle_ccw', 'spindle_off', 'coolant_off', 'flood'}
_WORD = re.compile(r'([A-Z])([+-]?(?:[0-9]+(?:\.[0-9]*)?|\.[0-9]+))')


def _dialect(m_codes, peck_return_mm=None, chip_break_return_mm=None, fine_bore_edge_deg=None, rigid_tapping=False):
    if set(m_codes) != _OEM or len(set(m_codes.values())) != len(_OEM):
        raise ValueError('bind distinct OEM tool/spindle/flood/off M functions')
    if any(not re.fullmatch(r'M[1-9][0-9]?', c) or int(c[1:]) in (1, 2, 30, 98, 99) for c in m_codes.values()):
        raise ValueError('OEM functions need unreserved M codes below 100')
    # Canned drilling: G81, G82 (P whole milliseconds), G98 initial-level
    # return; G83/G73 only with their return amounts (parameters 5115/5114).
    commands = (
        GCodeDialect(oriented_edge_deg=fine_bore_edge_deg)
        .with_commands(
            rapid=None,
            arc_cw=None,
            arc_ccw=None,
            plane_xz=None,
            plane_yz=None,
            center_incremental=None,
            rpm_mode=None,
            mist=None,
            frame_reset='G52',
            exact_stop='G61',
            end='M30',
            tap_synchronized=None,
            dwell=None,
            # §14.4: start-up (G01 G41/G42 D X Y) and cancel (G01 G40 X Y) are
            # written in the block of the straight XY lead move.
            peck_cycle='G83' if peck_return_mm is not None else None,
            chip_break_cycle='G73' if chip_break_return_mm is not None else None,
            # §13.1.3 G76: oriented spindle stop (OSS) and shift Q in the
            # direction of parameter 5101 RD1/RD2, only with the declared edge.
            fine_bore_cycle='G76' if fine_bore_edge_deg is not None else None,
            # §13.2.1/13.2.2 rigid tapping, only with parameter No. 5200#0 (G84)
            # = 1 ("Specify G84 for rigid tapping"), so no M29 is needed: G84
            # (right hand, spindle CW in, CCW out) and G74 (left hand, CCW in,
            # CW out) X Y Z R F, F = lead x S in G94, cancelled by G80. The
            # spindle turns in the cycle's direction beforehand (§13.1.2).
            tap_cycle='G84' if rigid_tapping else None,
            tap_cycle_left='G74' if rigid_tapping else None,
            **m_codes,
        )
        .commands
    )
    return replace(
        GCodeDialect(),
        commands=commands,
        fixed_modes=('center_incremental', 'rpm_mode'),
        radius_tolerance_mm=0,
        cycle_dwell_unit='ms',
        peck_reentry_mm=peck_return_mm,
        chip_break_lift_mm=chip_break_return_mm,
        oriented_edge_deg=fine_bore_edge_deg,
        compensation_with_move=True,
    )


@dataclass(frozen=True, kw_only=True)
class ReferenceMachine(Machine):
    startup_units: str
    startup_standard_modes: bool
    g92_shift_mm: tuple
    oem_m_codes: tuple
    program_number: int = 1000
    optional_stop_enabled: bool = False
    # Parameter 5115 (G83 return above the last peck) and 5114 (G73 retract), mm.
    peck_return_mm: float | None = None
    chip_break_return_mm: float | None = None
    # G76: where the boring bar's edge points (work XY degrees) after the
    # oriented spindle stop. Parameter 5101 RD1/RD2 must shift the opposite
    # way (e.g. edge 0 = +X, shift -X).
    fine_bore_edge_deg: float | None = None
    # Parameter No. 5200#0 (G84) = 1: G84/G74 are rigid tapping cycles.
    rigid_tapping: bool = False
    reference: str = 'FANUC 0i-MB;B-63844EN/02 February2003;length-offset-A'

    def __post_init__(self):
        super().__post_init__()
        object.__setattr__(self, 'g92_shift_mm', tuple(self.g92_shift_mm))
        if self.startup_units != self.output_units:
            raise ValueError('startup/output units must agree; changing stored offset units is not modeled')
        if self.startup_standard_modes is not True or self.g92_shift_mm != (0, 0, 0):
            raise ValueError(
                'declare length-off, standard Cartesian/no-transform/no-macro/no-AI startup and zero G92 shift'
            )
        if type(self.optional_stop_enabled) is not bool:
            raise ValueError('declare optional-stop switch state')
        if type(self.rigid_tapping) is not bool:
            raise ValueError('declare rigid tapping (parameter No. 5200#0 G84) as a boolean')
        if type(self.program_number) is not int or not 1 <= self.program_number <= 7999:
            raise ValueError('reference program number must be 1..7999')
        if self.traverse_feed_mm_min is None or self.arcs or self.helices or self.planes != ('xy',):
            raise ValueError('reference uses G17 and explicit G1 traverse/curve lowering')
        if self.tool_change_position_mode != 'table':
            raise ValueError('declare physical post-ATC tip poses in the tool table')
        if self.decimals > (3 if self.output_units == 'mm' else 4):
            raise ValueError('reference emits at most IS-B precision')
        if self.max_tool_number > 200 or self.max_length_register > 200:
            raise ValueError('reference table subset is T/H1..200')
        if (
            self.work_to_machine.x != (1, 0, 0)
            or self.work_to_machine.y != (0, 1, 0)
            or self.work_to_machine.z != (0, 0, 1)
        ):
            raise ValueError('stored work offset requires Cartesian translation')
        if (
            self.dialect
            != _dialect(
                dict(self.oem_m_codes),
                self.peck_return_mm,
                self.chip_break_return_mm,
                self.fine_bore_edge_deg,
                self.rigid_tapping,
            )
            or self.mist
        ):
            raise ValueError('reference dialect differs from declared FANUC/OEM subset')
        # Offset memory A keeps one value per number for both H and D: a D
        # register must never be one of the H registers.
        lengths = {t.length_register for t in self.tools}
        if any(t.diameter_register in lengths for t in self.tools):
            raise ValueError(
                'D (cutter compensation) registers must differ from every H register (offset memory A shares them)'
            )
        scale = 25.4 if self.output_units == 'inch' else 1
        for tool in self.tools:
            # Explicit compensation blocks preserve the physical spindle-axis pose.
            # Require exact output-grid table values rather than hide a tiny move.
            for value in (*tool.position_after_mm, tool.gauge_length_mm):
                if abs(round(value / scale, self.decimals) * scale - value) > 1e-9:
                    raise ValueError('tool table poses and gauge lengths must be representable at output precision')


def define_machine(
    name,
    tools,
    *,
    startup_units,
    startup_standard_modes,
    g92_shift_mm,
    m_codes,
    traverse_feed_mm_min,
    work_to_machine,
    peck_return_mm=None,
    chip_break_return_mm=None,
    fine_bore_edge_deg=None,
    rigid_tapping=False,
    **settings,
):
    """Define the installed machine in ordinary Python; no controller writes.

    startup_standard_modes means length-offset A, G49, Cartesian, no scaling,
    rotation, mirror, macro/AI modes. H values equal declared physical gauge lengths.
    OEM ATC routes are external; table entries state actual tip poses after M6.
    """
    return ReferenceMachine(
        name,
        tools,
        dialect=_dialect(m_codes, peck_return_mm, chip_break_return_mm, fine_bore_edge_deg, rigid_tapping),
        rigid_tapping=rigid_tapping,
        planes=('xy',),
        peck_return_mm=peck_return_mm,
        chip_break_return_mm=chip_break_return_mm,
        fine_bore_edge_deg=fine_bore_edge_deg,
        arcs=False,
        helices=False,
        mist=False,
        max_tool_number=200,
        max_length_register=200,
        startup_units=startup_units,
        startup_standard_modes=startup_standard_modes,
        g92_shift_mm=g92_shift_mm,
        oem_m_codes=tuple(sorted(m_codes.items())),
        traverse_feed_mm_min=traverse_feed_mm_min,
        work_to_machine=work_to_machine,
        **settings,
    )


class Fanuc0iMBPost(GCodePost):
    version = 'fanuc-0imb-2003-1'

    def words(self, addresses, values, *, dimensional=True):
        result = super().words(addresses, values, dimensional=dimensional)
        if dimensional:
            result = [w if '.' in w else w + '.' for w in result]
        return result

    def on_begin(self, program):
        self.active_tool = None
        self.emit('%')
        self.emit('O' + str(self.machine.program_number))
        super().on_begin(program)
        self._lines = [('G52 X0. Y0. Z0.' if line == 'G52' else line) for line in self._lines]

    def _cancel_length(self):
        if self.active_tool is not None:
            z = self.position_mm[2] + self.active_tool.gauge_length_mm
            self.emit('G1 G49', *self.words(('Z', 'F'), (z, self.machine.traverse_feed_mm_min)))
            self.active_tool = None

    def on_tool_change(self, event):
        self._cancel_length()
        self.active_tool = next(
            t for t in self.machine.tools if t.number == event.number and t.length_register == event.length_register
        )
        self.emit('T' + str(event.number))
        self.emit(self.machine.dialect.code('tool_change'))
        self.emit(
            'G1 G43',
            'H' + str(event.length_register),
            *self.words(('Z', 'F'), (self.active_tool.position_after_mm[2], self.machine.traverse_feed_mm_min)),
        )

    def on_finish(self):
        # Stop codes only where the tracked state is on or unknown.
        if getattr(self, '_spindle_on', True) is not False:
            self.emit(self.machine.dialect.code('spindle_off'))
        if getattr(self, '_coolant_mode', None) != 'off':
            self.emit(self.machine.dialect.code('coolant_off'))
        self._cancel_length()
        self.emit('M30')
        self.emit('%')


class Fanuc0iMBDecoder(NativeGCodeDecoder):
    """Read actual words; explicit compensation blocks must preserve physical tip."""

    version = 'fanuc-0imb-2003-1'

    def __init__(self, machine, program):
        super().__init__(machine, program)
        self.opened = self.numbered = self.end_seen = self.closed = False

    def validate_initialized(self):
        super().validate_initialized()
        if not self.opened or not self.numbered:
            raise ValueError('FANUC percent/O framing is missing')

    def read_line(self, line, context=None):
        if (
            self.ended
            or len(line) > 1024
            or not line.isascii()
            or any(ord(c) < 32 and c != '\t' or ord(c) == 127 for c in line)
        ):
            raise ValueError('invalid FANUC physical block/end')
        body = re.sub(r'\([^()]*\)', ' ', line.strip().upper()).strip()
        if '(' in body or ')' in body or ';' in body:
            raise ValueError('unsupported FANUC comment/block syntax')
        if not body:
            return []
        if body == '%':
            if not self.opened:
                self.opened = True
            elif self.end_seen:
                self.closed = self.ended = True
            else:
                raise ValueError('closing percent requires M30')
            return []
        if not self.opened or self.end_seen:
            raise ValueError('invalid FANUC program framing')
        if re.fullmatch(r'O[0-9]{1,4}', body):
            if self.numbered or int(body[1:]) != self.machine.program_number:
                raise ValueError('unexpected O number')
            self.numbered = True
            return []
        if not self.numbered:
            raise ValueError('FANUC O number required')
        compact = re.sub(r'\s+', '', body)
        pieces = _WORD.findall(compact)
        if ''.join(a + b for a, b in pieces) != compact:
            raise ValueError('unsupported FANUC words')
        gs = [float(v) for w, v in pieces if w == 'G']
        values = {w: float(v) for w, v in pieces if w != 'G'}
        if len(values) != len(pieces) - len(gs) or any(not math.isfinite(v) or abs(v) > 1e9 for v in values.values()):
            raise ValueError('duplicate/nonfinite FANUC words')
        precision = 3 if self.machine.output_units == 'mm' else 4
        for word, text in pieces:
            if word in 'XYZIJKFRQ':
                if (
                    '.' not in text
                    or len(text.partition('.')[2]) > precision
                    or abs(float(text)) * 10**precision >= 1e8
                ):
                    raise ValueError('FANUC dimensional words need explicit decimals within IS-B/eight-digit range')
        if gs in ([1.0, 43.0], [1.0, 49.0]):
            activating = gs[-1] == 43
            if (
                set(values) != ({'H', 'Z', 'F'} if activating else {'Z', 'F'})
                or self.active is None
                or self.units is None
            ):
                raise ValueError('invalid explicit length compensation block')
            if (
                self.rpm
                or self.coolant != 'off'
                or values['F'] <= 0
                or values['F'] * self.units > self.machine.max_feed_mm_min
            ):
                raise ValueError('compensation transition needs stopped spindle/coolant and valid feed')
            self.validate_initialized()
            if self.offset != self.machine.work_offset:
                raise ValueError('work offset required before length activation')
            expected = self.position[2] if activating else self.position[2] + self.active.gauge_length_mm
            if abs(values['Z'] * self.units - expected) > 1e-9:
                raise ValueError('length compensation block would move the physical tip')
            if activating:
                return [self._length(values['H'])]
            if self.length is None:
                raise ValueError('length cancellation without active offset')
            return super().read_line('G49', context)
        if len(gs) == 2 and gs[0] == 1 and gs[1] in (40, 41, 42):
            # §14.4: cutter compensation start-up/cancel with its G01 XY move.
            return super().read_line(body, context)
        if len(gs) > 1:
            raise ValueError('unsupported combined G functions')
        g = gs[0] if gs else None
        if g == 43 or g == 49 and self.length is not None:
            raise ValueError('bare G43/G49 can move the tool; explicit Z/F compensation block required')
        if g in (20, 21) and g != (21 if self.machine.output_units == 'mm' else 20):
            raise ValueError('file units differ from installed offset units')
        if g == 52:
            if set(values) != {'X', 'Y', 'Z'} or any(values[a] != 0 for a in 'XYZ'):
                raise ValueError('only explicit all-axis local zero supported')
            self.ready.add('frame_reset')
            return []
        if values.get('M') in (0, 1, int(self.machine.dialect.code('tool_change')[1:])):
            if self.rpm or self.coolant != 'off':
                raise ValueError('stop spindle/coolant before stop/ATC')
            if values.get('M') == int(self.machine.dialect.code('tool_change')[1:]) and self.length is not None:
                raise ValueError('cancel length explicitly before ATC')
            if values.get('M') == 1 and not self.machine.optional_stop_enabled:
                raise ValueError('optional stop disabled')
        if 'S' in values and values['S'] != int(values['S']):
            raise ValueError('RPM must be integral')
        result = super().read_line(body, context)
        if values.get('M') == 30:
            self.end_seen = True
            self.ended = False
        return result


def post_fanuc(program, machine, *, lowering=None, **options):
    return post_events(
        program,
        program.events,
        machine=machine,
        post=Fanuc0iMBPost,
        decoder=Fanuc0iMBDecoder,
        lowering=lowering or LoweringOptions(),
        **options,
    )
