"""Ordinary Python LNC milling reference (A-LNC Milling Machine Series, LNC-M8000).

Source: A-LNC Milling Machine Series Programming Manual, V01.02.000
(4408230009), 2016/05, Advantech-LNC Technology Co., Ltd. Page numbers below
are the manual's printed pages. See docs/lnc-mill-post.md. This edition is not
a claim for other LNC series or for an unspecified machine builder's PLC.
"""

from dataclasses import dataclass, replace
import math
import re

from camlib.post import Machine, GCodeDialect, GCodePost, NativeGCodeDecoder, post_events

# The manual's M table (p.3) lists only M00, M01, M02, M30, M98 and M99.
# Spindle, coolant and tool change are PLC (machine builder) functions: the
# machine declares them; nothing is assumed.
_OEM = {'tool_change', 'spindle_cw', 'spindle_ccw', 'spindle_off', 'coolant_off', 'flood'}
# M28/M29: rigid tapping off/on (p.76); M19: the spindle orientation G76 runs
# (p.83). They are reserved whenever the machine declares those cycles.
_RESERVED = (0, 1, 2, 30, 98, 99)
_WORD = re.compile(r'([A-Z])([+-]?(?:[0-9]+(?:\.[0-9]*)?|\.[0-9]+))')


def _dialect(
    m_codes,
    arc_radius_tolerance_mm,
    peck_return_mm=None,
    chip_break_return_mm=None,
    fine_bore_edge_deg=None,
    rigid_tapping_m29=False,
):
    if set(m_codes) != _OEM or len(set(m_codes.values())) != len(_OEM):
        raise ValueError('bind distinct builder M functions for tool change, spindle CW/CCW/off, flood and coolant off')
    reserved = _RESERVED + ((28, 29) if rigid_tapping_m29 else ()) + ((19,) if fine_bore_edge_deg is not None else ())
    if any(not re.fullmatch(r'M[1-9][0-9]{0,2}', c) or int(c[1:]) in reserved for c in m_codes.values()):
        raise ValueError(
            'builder functions need M codes 1..999 other than M0/M1/M2/M30/M98/M99 '
            '(and M28/M29 with rigid tapping, M19 with fine boring)'
        )
    if (
        not isinstance(arc_radius_tolerance_mm, (int, float))
        or isinstance(arc_radius_tolerance_mm, bool)
        or not math.isfinite(arc_radius_tolerance_mm)
        or not 0 <= arc_radius_tolerance_mm <= 0.1
    ):
        raise ValueError('declare parameter 50048 (G02/G03 end-point radius error) in mm, 0..0.1')
    commands = (
        GCodeDialect(oriented_edge_deg=fine_bore_edge_deg)
        .with_commands(
            # G00 positions each axis at its own rapid rate (parameters
            # 60286-60317, p.5); the manual does not say the path is straight,
            # so traverses are written as G01 at the declared traverse feed.
            rapid=None,
            units_inch=None,
            # p.7: I/J/K are always incremental from the start point; there is
            # no G90.1/G91.1 and no constant surface speed on this mill.
            center_incremental=None,
            rpm_mode=None,
            mist=None,
            # p.48: G52 X0 Y0 Z0 clears the local coordinate system.
            frame_reset='G52',
            # p.53: G61 exact stop mode (G64 is the default).
            exact_stop='G61',
            end='M30',
            tap_synchronized=None,
            # G04 X is seconds but X is an axis word here; P is integer
            # milliseconds (p.9). Drilling dwell uses G82/G89 P instead.
            dwell=None,
            # p.108 G83 re-entry (parameter 160031) and p.63 G73 retract
            # (parameter 160030): written only with the declared values.
            peck_cycle='G83' if peck_return_mm is not None else None,
            chip_break_cycle='G73' if chip_break_return_mm is not None else None,
            # p.83 G76: oriented stop (M19) and shift Q away from the edge.
            fine_bore_cycle='G76' if fine_bore_edge_deg is not None else None,
            # p.71/p.76: rigid tapping is selected by M29 S before G84/G74
            # and cancelled by M28.
            tap_cycle='G84' if rigid_tapping_m29 else None,
            tap_cycle_left='G74' if rigid_tapping_m29 else None,
            **m_codes,
        )
        .commands
    )
    return replace(
        GCodeDialect(),
        commands=commands,
        fixed_modes=('center_incremental', 'rpm_mode'),
        # p.7: alarm 510204 when the start and end radii differ by more than
        # parameter 50048. No relative allowance is documented.
        radius_tolerance_mm=arc_radius_tolerance_mm,
        radius_relative_tolerance=0,
        cycle_dwell_unit='ms',
        peck_reentry_mm=peck_return_mm,
        chip_break_lift_mm=chip_break_return_mm,
        oriented_edge_deg=fine_bore_edge_deg,
        compensation_with_move=True,
    )


@dataclass(frozen=True, kw_only=True)
class ReferenceMachine(Machine):
    startup_standard_modes: bool
    g92_shift_mm: tuple
    oem_m_codes: tuple
    # Parameter 50048: accepted G02/G03 end-point radius error, mm.
    arc_radius_tolerance_mm: float
    optional_stop_enabled: bool = False
    # Parameter 160031 (G83 re-entry clearance) and 160030 (G73 retract), mm.
    peck_return_mm: float | None = None
    chip_break_return_mm: float | None = None
    # G76: where the boring bar's edge points (work XY degrees) after M19.
    # The shift-direction parameter must move the bar the opposite way.
    fine_bore_edge_deg: float | None = None
    # Rigid tapping by M29 S before G84/G74, cancelled by M28 (p.76).
    rigid_tapping_m29: bool = False
    reference: str = 'A-LNC Milling Programming Manual V01.02.000 (4408230009) 2016/05'

    def __post_init__(self):
        super().__post_init__()
        object.__setattr__(self, 'g92_shift_mm', tuple(self.g92_shift_mm))
        if self.output_units != 'mm':
            # p.18: G21 is the only unit command; there is no G20 (p.1-2).
            raise ValueError('the LNC milling manual documents metric input only (G21): output_units must be mm')
        if self.startup_standard_modes is not True or self.g92_shift_mm != (0, 0, 0):
            raise ValueError('declare standard startup (G49, G40, G15, G50, G50.1, G69, G67, G64) and a zero G92 shift')
        if type(self.optional_stop_enabled) is not bool or type(self.rigid_tapping_m29) is not bool:
            raise ValueError('declare the optional-stop switch and rigid tapping as booleans')
        if self.traverse_feed_mm_min is None or self.helices:
            # p.7 defines circular arcs in one plane only; helical moves are lowered.
            raise ValueError('the LNC reference writes traverses as G01 and lowers helices')
        if self.arcs and self.max_sweep_rad > math.pi:
            # Full circles (end = start) are not described on p.7: at most 180 degrees per block.
            raise ValueError('LNC native arcs are limited to 180 degrees per block (max_sweep_rad <= pi)')
        if self.tool_change_position_mode != 'table':
            raise ValueError('declare physical post-M6 tip poses in the tool table')
        if self.decimals > 3:
            raise ValueError('the least input increment is 0.001 mm (p.18): at most 3 decimals')
        if self.max_length_register > 400:
            raise ValueError('H registers are 1..400 (p.38)')
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
                self.arc_radius_tolerance_mm,
                self.peck_return_mm,
                self.chip_break_return_mm,
                self.fine_bore_edge_deg,
                self.rigid_tapping_m29,
            )
            or self.mist
        ):
            raise ValueError('reference dialect differs from the declared LNC/builder subset')
        for tool in self.tools:
            # Explicit compensation blocks preserve the physical spindle pose:
            # require exact output-grid values rather than hide a tiny move.
            for value in (*tool.position_after_mm, tool.gauge_length_mm):
                if abs(round(value, self.decimals) - value) > 1e-9:
                    raise ValueError('tool table poses and gauge lengths must be representable at output precision')


def define_machine(
    name,
    tools,
    *,
    startup_standard_modes,
    g92_shift_mm,
    m_codes,
    arc_radius_tolerance_mm,
    traverse_feed_mm_min,
    work_to_machine,
    arcs=True,
    peck_return_mm=None,
    chip_break_return_mm=None,
    fine_bore_edge_deg=None,
    rigid_tapping_m29=False,
    **settings,
):
    """Define the installed machine in ordinary Python; no controller writes.

    m_codes binds the builder's PLC functions. arc_radius_tolerance_mm is
    parameter 50048. peck_return_mm / chip_break_return_mm are parameters
    160031 / 160030. H values equal the declared physical gauge lengths; table
    entries state actual tip poses after M6.
    """
    planes = ('xy', 'xz', 'yz') if arcs else ('xy',)
    return ReferenceMachine(
        name,
        tools,
        dialect=_dialect(
            m_codes,
            arc_radius_tolerance_mm,
            peck_return_mm,
            chip_break_return_mm,
            fine_bore_edge_deg,
            rigid_tapping_m29,
        ),
        arc_radius_tolerance_mm=arc_radius_tolerance_mm,
        rigid_tapping_m29=rigid_tapping_m29,
        planes=planes,
        peck_return_mm=peck_return_mm,
        chip_break_return_mm=chip_break_return_mm,
        fine_bore_edge_deg=fine_bore_edge_deg,
        arcs=arcs,
        helices=False,
        mist=False,
        max_length_register=400,
        startup_standard_modes=startup_standard_modes,
        g92_shift_mm=g92_shift_mm,
        oem_m_codes=tuple(sorted(m_codes.items())),
        traverse_feed_mm_min=traverse_feed_mm_min,
        work_to_machine=work_to_machine,
        **settings,
    )


class LNCMillPost(GCodePost):
    version = 'lnc-mill-v01.02-1'

    def words(self, addresses, values, *, dimensional=True):
        # Decimal points on every dimensional word: no word depends on how the
        # control reads an undotted value.
        result = super().words(addresses, values, dimensional=dimensional)
        if dimensional:
            result = [w if '.' in w else w + '.' for w in result]
        return result

    def on_begin(self, program):
        self.active_tool = None
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
        # p.38: G43 H; the Z word in the same block keeps the physical tip
        # where it is (a zero-length move), whatever the control does with a
        # bare G43.
        self.emit(
            'G1 G43',
            'H' + str(event.length_register),
            *self.words(('Z', 'F'), (self.active_tool.position_after_mm[2], self.machine.traverse_feed_mm_min)),
        )

    def tap(self, motion):
        c = motion.curve
        if self.machine.rigid_tapping_m29:
            if c.end[2] < c.start[2]:
                # G17 before M29: nothing may come between M29 and G84/G74.
                self.cycle_plane()
                # p.76 example: M29 S<rpm> in the block before G84/G74.
                if getattr(self, '_rpm', 0):
                    self.emit('M29', self.word(self.machine.dialect.speed_word, self._rpm, dimensional=False))
            else:
                # p.76 example: M28 cancels rigid tapping before G80.
                self.emit('M28')
        super().tap(motion)

    def on_finish(self):
        if getattr(self, '_spindle_on', True) is not False:
            self.emit(self.machine.dialect.code('spindle_off'))
        if getattr(self, '_coolant_mode', None) != 'off':
            self.emit(self.machine.dialect.code('coolant_off'))
        self._cancel_length()
        self.emit('M30')


class LNCMillDecoder(NativeGCodeDecoder):
    """Read actual words. Imports no formatter code."""

    version = 'lnc-mill-v01.02-1'

    def __init__(self, machine, program):
        super().__init__(machine, program)
        self.end_seen = False
        # M29 S read; the next block must be the G84/G74 it selects.
        self.rigid_armed = False
        # A G84/G74 rigid cycle is open: M28 must come before its G80.
        self.rigid_open = False

    def read_line(self, line, context=None):
        if (
            self.end_seen
            or len(line) > 1024
            or not line.isascii()
            or any(ord(c) < 32 and c != '\t' or ord(c) == 127 for c in line)
        ):
            raise ValueError('invalid LNC physical block, or a block after M30')
        body = re.sub(r'\([^()]*\)', ' ', line.strip().upper()).strip()
        if '(' in body or ')' in body or ';' in body or '%' in body:
            raise ValueError('unsupported LNC comment or block syntax')
        if not body:
            return []
        compact = re.sub(r'\s+', '', body)
        pieces = _WORD.findall(compact)
        if ''.join(a + b for a, b in pieces) != compact:
            raise ValueError('unsupported LNC words')
        gs = [float(v) for w, v in pieces if w == 'G']
        values = {w: float(v) for w, v in pieces if w != 'G'}
        if len(values) != len(pieces) - len(gs) or any(not math.isfinite(v) or abs(v) > 1e9 for v in values.values()):
            raise ValueError('duplicate or nonfinite LNC words')
        if 53.0 in gs:
            # p.50: G53 cancels the active tool length compensation. This
            # reference never writes it.
            raise ValueError('G53 cancels tool length compensation (p.50); the LNC reference does not write G53')
        tapping = gs in ([84.0], [74.0])
        if self.rigid_armed and not tapping:
            raise ValueError('M29 S must be directly followed by its G84/G74 rigid tapping block')
        rigid = self.machine.rigid_tapping_m29
        if rigid and values.get('M') == 29:
            if gs or set(values) != {'M', 'S'}:
                raise ValueError('write M29 S alone in the block before G84/G74')
            if self.cycle_active or self.rpm <= 0 or values['S'] != self.rpm:
                raise ValueError('M29 S must repeat the running spindle speed, outside a canned cycle')
            self.rigid_armed = True
            return []
        if rigid and values.get('M') == 28:
            if gs or set(values) != {'M'} or not self.rigid_open:
                raise ValueError('M28 cancels an open G84/G74 rigid tapping cycle, alone in its block')
            self.rigid_open = False
            return []
        if tapping:
            if rigid and not self.rigid_armed:
                raise ValueError('rigid tapping needs M29 S in the block before G84/G74 (p.76)')
            self.rigid_armed = False
        if gs == [80.0] and rigid and self.rigid_open:
            raise ValueError('M28 must cancel rigid tapping before G80 (p.76)')
        for word, text in pieces:
            if word in 'XYZIJKFRQ':
                if '.' not in text or len(text.partition('.')[2]) > 3 or abs(float(text)) >= 1e5:
                    raise ValueError('LNC dimensional words need a decimal point, at most 0.001 mm resolution')
            if word == 'P' and ('.' in text or not 1 <= float(text) <= 99999999):
                raise ValueError('P is whole milliseconds without a decimal point, 1..99999999 (p.9)')
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
            if not 1 <= values.get('H', 1) <= 400:
                raise ValueError('H is 1..400 (p.38)')
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
            return super().read_line(body, context)
        if len(gs) > 1:
            raise ValueError('unsupported combined G functions')
        g = gs[0] if gs else None
        if g in (43.0, 44.0) or g == 49 and self.length is not None:
            raise ValueError('length compensation needs the explicit G1 G43 H Z F / G1 G49 Z F block')
        if g == 52:
            if set(values) != {'X', 'Y', 'Z'} or any(values[a] != 0 for a in 'XYZ'):
                raise ValueError('only an explicit all-axis G52 zero is supported')
            self.ready.add('frame_reset')
            return []
        tool_change = int(self.machine.dialect.code('tool_change')[1:])
        if values.get('M') in (0, 1, tool_change):
            if self.rpm or self.coolant != 'off':
                raise ValueError('stop the spindle and coolant before a program stop or tool change')
            if values.get('M') == tool_change and self.length is not None:
                raise ValueError('cancel length compensation explicitly before M6')
            if values.get('M') == 1 and not self.machine.optional_stop_enabled:
                raise ValueError('optional stop disabled')
        if 'S' in values and values['S'] != int(values['S']):
            raise ValueError('RPM must be integral')
        result = super().read_line(body, context)
        if tapping and rigid:
            self.rigid_open = True
        if values.get('M') == 30:
            self.end_seen = True
        return result


def post_lnc(program, machine, *, lowering=None, **options):
    # Native arcs by default; pass LoweringOptions to lower curves explicitly.
    return post_events(
        program,
        program.events,
        machine=machine,
        post=LNCMillPost,
        decoder=LNCMillDecoder,
        lowering=lowering,
        **options,
    )
