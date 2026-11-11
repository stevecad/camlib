"""Ordinary Python FANUC 0i-MB B-63844EN/02 reference subset.

See docs/fanuc-0imb-post.md. This edition is not a claim for other FANUC series.
"""

from dataclasses import dataclass, replace
import math
import re

from camlib.post import Machine, GCodeDialect, GCodePost, NativeGCodeDecoder, LoweringOptions, post_events

_OEM = {'tool_change', 'spindle_cw', 'spindle_ccw', 'spindle_off', 'coolant_off', 'flood'}
_WORD = re.compile(r'([A-Z])([+-]?(?:[0-9]+(?:\.[0-9]*)?|\.[0-9]+))')


def _dialect(
    m_codes,
    peck_return_mm=None,
    chip_break_return_mm=None,
    fine_bore_edge_deg=None,
    rigid_tapping=False,
    rigid_tapping_m29=False,
    inspection_plus_fast_feed_mm_min=None,
    inspection_plus_switching=False,
):
    if set(m_codes) != _OEM or len(set(m_codes.values())) != len(_OEM):
        raise ValueError('bind distinct OEM tool/spindle/flood/off M functions')
    reserved = (1, 2, 30, 98, 99) + ((29,) if rigid_tapping_m29 else ())
    if any(not re.fullmatch(r'M[1-9][0-9]?', c) or int(c[1:]) in reserved for c in m_codes.values()):
        raise ValueError('OEM functions need unreserved M codes below 100 (M29 selects rigid tapping when declared)')
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
            # §13.2.1/13.2.2 rigid tapping: G84 (right hand, spindle CW in,
            # CCW out) and G74 (left hand, CCW in, CW out) X Y Z R F, F = lead
            # x S in G94, cancelled by G80. The spindle turns in the cycle's
            # direction beforehand (§13.1.2). Rigid mode is selected either by
            # parameter No. 5200#0 (G84) = 1 ("Specify G84 for rigid
            # tapping", rigid_tapping) or, with 5200#0 = 0, by "M29 S*****
            # before a tapping command" (rigid_tapping_m29), the form of the
            # manual's examples; the post writes that M29 block.
            tap_cycle='G84' if rigid_tapping or rigid_tapping_m29 else None,
            tap_cycle_left='G74' if rigid_tapping or rigid_tapping_m29 else None,
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
        # Renishaw Inspection Plus macros (G65 P98xx, custom macro B), only
        # where installed: their fast feed #119 and probe switching.
        probing=None if inspection_plus_fast_feed_mm_min is None else 'inspection_plus',
        probe_fast_feed_mm_min=inspection_plus_fast_feed_mm_min,
        probe_switching=inspection_plus_switching,
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
    # Parameter No. 5200#0 (G84) = 0: rigid mode is selected by M29 S in the
    # block before G84/G74 (§13.2.1/13.2.2).
    rigid_tapping_m29: bool = False
    # Renishaw Inspection Plus installed (H-2000-6031): its fast feed #119
    # (settings macro O9724), and whether cycles switch the probe on/off
    # with O9832/O9833. None: no probing.
    inspection_plus_fast_feed_mm_min: float | None = None
    inspection_plus_switching: bool = False
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
        if type(self.rigid_tapping) is not bool or type(self.rigid_tapping_m29) is not bool:
            raise ValueError('declare rigid tapping (parameter No. 5200#0 G84) as booleans')
        if self.rigid_tapping and self.rigid_tapping_m29:
            raise ValueError(
                'declare one rigid tapping premise: rigid_tapping (5200#0 = 1) or rigid_tapping_m29 (5200#0 = 0)'
            )
        if type(self.program_number) is not int or not 1 <= self.program_number <= 7999:
            raise ValueError('reference program number must be 1..7999')
        if self.traverse_feed_mm_min is None or self.arcs or self.helices or self.planes != ('xy',):
            raise ValueError('reference uses G17 and explicit G1 traverse/curve lowering')
        # M06 runs the machine tool builder's tool-change macro (B-63844EN/02
        # pages 114-116: M functions are the builder's), so where it leaves
        # the spindle is a declared premise: each tool's measured tip pose
        # ("table"), or the gauge line's machine Z with X and Y unmoved
        # ("z_home"), from which each tip follows by its gauge length.
        if self.tool_change_position_mode not in ('table', 'z_home'):
            raise ValueError(
                'declare where M06 leaves the spindle: tool_change_position_mode="z_home" with '
                "tool_change_home_z_mm (the gauge line's machine Z after M06; X and Y stay), or "
                'tool_change_position_mode="table" with each ToolSetting.position_after_mm (the measured tip pose)'
            )
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
                self.rigid_tapping_m29,
                self.inspection_plus_fast_feed_mm_min,
                self.inspection_plus_switching,
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
            after_z = self.tip_z_after_change_mm(tool)
            stated = tool.position_after_mm
            if self.tool_change_position_mode == 'z_home' and stated is not None and abs(stated[2] - after_z) > 1e-9:
                # Two declarations of the same tip: never pick one.
                raise ValueError(
                    f'tool {tool.tool!r}: position_after_mm states the tip at Z {stated[2]} after M06, but '
                    f'tool_change_home_z_mm {self.tool_change_home_z_mm} (work Z '
                    f'{after_z + tool.gauge_length_mm}) less its gauge length {tool.gauge_length_mm} is Z '
                    f'{after_z}: correct the gauge length or the home Z, or leave position_after_mm out '
                    '(z_home derives it)'
                )
            for value in (*(stated or ()), after_z, tool.gauge_length_mm):
                if abs(round(value / scale, self.decimals) * scale - value) > 1e-9:
                    raise ValueError('tool table poses and gauge lengths must be representable at output precision')

    def tip_z_after_change_mm(self, tool):
        """Work Z of ``tool``'s tip after M06: its measured table pose, or
        (z_home) the gauge line's declared Z less the tool's gauge length."""
        if self.tool_change_position_mode == 'z_home':
            return self.tool_change_home_z_mm - self.work_to_machine.origin_mm[2] - tool.gauge_length_mm
        return tool.position_after_mm[2]


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
    rigid_tapping_m29=False,
    inspection_plus_fast_feed_mm_min=None,
    inspection_plus_switching=False,
    **settings,
):
    """Define the installed machine in ordinary Python; no controller writes.

    startup_standard_modes means length-offset A, G49, Cartesian, no scaling,
    rotation, mirror, macro/AI modes. H values equal declared physical gauge lengths.
    OEM ATC routes are external. Declare where M6 leaves the spindle:
    tool_change_position_mode="table" (each ToolSetting.position_after_mm is
    the measured tip pose), or "z_home" with tool_change_home_z_mm (the gauge
    line's machine Z after M6, X and Y unmoved): each tip is then that Z less
    the tool's gauge length, and position_after_mm is left out.
    rigid_tapping declares parameter No. 5200#0 = 1 (G84/G74 alone are rigid);
    rigid_tapping_m29 declares 5200#0 = 0 with rigid mode selected by M29 S.
    """
    return ReferenceMachine(
        name,
        tools,
        dialect=_dialect(
            m_codes,
            peck_return_mm,
            chip_break_return_mm,
            fine_bore_edge_deg,
            rigid_tapping=rigid_tapping,
            rigid_tapping_m29=rigid_tapping_m29,
            inspection_plus_fast_feed_mm_min=inspection_plus_fast_feed_mm_min,
            inspection_plus_switching=inspection_plus_switching,
        ),
        rigid_tapping=rigid_tapping,
        rigid_tapping_m29=rigid_tapping_m29,
        inspection_plus_fast_feed_mm_min=inspection_plus_fast_feed_mm_min,
        inspection_plus_switching=inspection_plus_switching,
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
            *self.words(
                ('Z', 'F'),
                (self.machine.tip_z_after_change_mm(self.active_tool), self.machine.traverse_feed_mm_min),
            ),
        )

    def tap(self, motion):
        # §13.2.1/13.2.2 with 5200#0 = 0: "M29 S***** before a tapping
        # command", S the running speed, directly before G84/G74: an S word or
        # axis move between them is P/S alarm 203/204 (Appendix G).
        c = motion.curve
        if self.machine.rigid_tapping_m29 and c.end[2] < c.start[2] and getattr(self, '_rpm', 0):
            self.emit('M29', self.word(self.machine.dialect.speed_word, self._rpm, dimensional=False))
        super().tap(motion)

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
        # M29 S read; the next block must be the G84/G74 it selects.
        self.rigid_armed = False

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
        tapping = gs in ([84.0], [74.0])
        if self.rigid_armed and not tapping:
            # Appendix G: P/S 203 (position of M29 or S) and 204 (axis move
            # between the M29 block and G84/G74).
            raise ValueError('M29 must be directly followed by its G84/G74 rigid tapping block')
        # Without the premise, M29 is an ordinary (OEM-bound or undeclared) M code.
        if values.get('M') == 29 and self.machine.rigid_tapping_m29:
            if gs or set(values) != {'M', 'S'}:
                raise ValueError('write M29 S alone in the block before G84/G74')
            if self.cycle_active or self.rpm <= 0 or values['S'] != self.rpm:
                raise ValueError('M29 S must repeat the running spindle speed, outside a canned cycle')
            self.rigid_armed = True
            return []
        if tapping:
            if self.machine.rigid_tapping_m29 and not self.rigid_armed:
                raise ValueError('rigid tapping under 5200#0 = 0 needs M29 S in the block before G84/G74')
            self.rigid_armed = False
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
