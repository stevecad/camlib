"""Ordinary Python 808D ISO milling reference, February 2016 manuals.

See docs/siemens808d-iso-post.md for declared machine settings and evidence.
The milling H map is $TC_DPH, not the native per-tool D cutting-edge number.
"""

from dataclasses import dataclass, replace
import math
import re

from camlib.motion import Line, Motion
from camlib.post import Machine, GCodeDialect, GCodePost, NativeGCodeDecoder, LoweringOptions, post_events

_WORDS = re.compile(r'([A-Z])\s*([+-]?(?:[0-9]+(?:\.[0-9]*)?|\.[0-9]+))')
_PREFIX = ('G290', 'COMPOF', 'RNDM=0', 'CFTCP', 'G601', 'G291')
_RESETS = ('G50', 'G69', 'G50.1 X0. Y0. Z0.')
_RESERVED = {0, 1, 2, 3, 4, 5, 6, 17, 19, 29, 30, 40, 41, 42, 43, 44, 45, 70, 98, 99}


def _dialect(tool_change, coolant_codes):
    if not set(coolant_codes) <= {'off', 'flood', 'mist'} or 'off' not in coolant_codes:
        raise ValueError('declare OEM off and optional flood/mist coolant M functions')
    for code in coolant_codes.values():
        if not re.fullmatch(r'M(?:0|[1-9][0-9]?)', code) or int(code[1:]) in _RESERVED:
            raise ValueError('coolant must use a free OEM M function in M0..M99')
    return replace(
        GCodeDialect(), tool_change=tool_change, radius_tolerance_mm=0, fixed_modes=('center_incremental', 'rpm_mode')
    ).with_commands(
        rapid=None,
        plane_xz=None,
        plane_yz=None,
        center_incremental=None,
        rpm_mode=None,
        exact_stop='G61',
        end='M30',
        tap_synchronized=None,
        dwell=None,
        radius_left=None,
        radius_right=None,
        drill_cycle=None,
        drill_dwell_cycle=None,
        peck_cycle=None,
        chip_break_cycle=None,
        cycle_return_initial=None,
        bore_cycle=None,
        bore_dwell_cycle=None,
        bore_stop_cycle=None,
        coolant_off=coolant_codes['off'],
        flood=coolant_codes.get('flood'),
        mist=coolant_codes.get('mist'),
    )


@dataclass(frozen=True, kw_only=True)
class ReferenceMachine(Machine):
    reference: str = '808D ADVANCED ISO milling; programming and parameter manuals 02/2016'
    startup_units: str
    extern_toolprog_mode: int
    cutting_edge_default: int
    tool_corr_move_mode: int
    tool_length_const: int
    optional_stop_enabled: bool = False
    curve_policy: str = 'linear'
    # G84/G74 premises (ISO manual pp.126, 130, 143-147). None: undeclared,
    # and tapping is refused.
    # "G84 can be used if the spindle ... is technically able to be operated
    # in the position-controlled spindle mode" (p.143, p.145 for G74).
    position_controlled_spindle: bool = False
    # GUD _ZSFI[1]: 2 and 3 turn G84/G74 into deep-hole tapping (p.147).
    tap_deep_hole_mode: int | None = None
    # GUD _ZSFI[2]: retraction speed in percent of the tapping speed (p.144).
    tap_retract_speed_percent: int | None = None
    # GUD _ZSFR[0], the safety clearance (pp.130, 132): the cycle taps from
    # the R plane "shifted by the amount of the safety clearance" (p.144).
    tap_safety_clearance_mm: float | None = None
    # Whether the spindle turns again at the earlier speed and direction after
    # the cycle. The table on p.126 gives the direction; the speed is unstated.
    tap_restores_spindle: bool | None = None

    def __post_init__(self):
        super().__post_init__()
        clearance = self.tap_safety_clearance_mm
        if (
            type(self.position_controlled_spindle) is not bool
            or self.tap_restores_spindle is not None
            and type(self.tap_restores_spindle) is not bool
            or any(
                v is not None and type(v) is not int for v in (self.tap_deep_hole_mode, self.tap_retract_speed_percent)
            )
            or clearance is not None
            and (
                isinstance(clearance, bool)
                or not isinstance(clearance, (int, float))
                or not math.isfinite(clearance)
                or clearance < 0
                or abs(round(clearance, 3) - clearance) > 1e-12
            )
        ):
            raise ValueError(
                'tapping premises: position_controlled_spindle and tap_restores_spindle are booleans, '
                'tap_deep_hole_mode (_ZSFI[1]) and tap_retract_speed_percent (_ZSFI[2]) integers, '
                'tap_safety_clearance_mm (_ZSFR[0]) a length >= 0 mm on the 0.001 mm grid'
            )
        if self.startup_units != self.output_units:
            raise ValueError('set output units to the existing controller units; runtime unit switching is not modeled')
        if type(self.extern_toolprog_mode) is not int or self.extern_toolprog_mode not in (0, 4, 64, 68):
            raise ValueError('ISO milling reference accepts MD10890 bits2/6 only; bit2 is turning-only')
        if (
            type(self.cutting_edge_default) is not int
            or self.cutting_edge_default != 0
            or type(self.tool_corr_move_mode) is not int
            or self.tool_corr_move_mode != 0
            or type(self.tool_length_const) is not int
            or self.tool_length_const != 17
        ):
            raise ValueError(
                'reference needs MD20270=0, MD20382=0 and SD42940=17 for explicit deferred Z length compensation'
            )
        if type(self.optional_stop_enabled) is not bool:
            raise ValueError('declare optional stop enablement')
        if self.traverse_feed_mm_min is None:
            raise ValueError('declare traverse_feed_mm_min; independent-axis G0 is not a straight swept path')
        if self.planes != ('xy',) or self.curve_policy not in ('linear', 'exact_ijk'):
            raise ValueError('reference uses G17 and linear/exact_ijk curves')
        if (
            self.arcs != (self.curve_policy == 'exact_ijk')
            or self.helices != self.arcs
            or self.max_sweep_rad > math.tau
        ):
            raise ValueError('reference curve capabilities must match single-turn policy')
        if self.decimals > (3 if self.output_units == 'mm' else 4):
            raise ValueError('reference emits at most IS-B precision: 3 metric or 4 inch decimals')
        if (
            self.work_to_machine.x != (1, 0, 0)
            or self.work_to_machine.y != (0, 1, 0)
            or self.work_to_machine.z != (0, 0, 1)
        ):
            raise ValueError('reference stored work offset must be a Cartesian translation')
        d = self.dialect
        coolant = {'off': d.code('coolant_off')}
        coolant.update({c.effect: c.code for c in d.commands if c.effect in ('flood', 'mist')})
        if d != _dialect(d.tool_change, coolant):
            raise ValueError('reference dialect differs from documented ISO milling subset')
        if self.flood != ('flood' in coolant) or self.mist != ('mist' in coolant):
            raise ValueError('coolant capabilities must match explicit OEM bindings')

    def require_tapping(self):
        """Refuse G84/G74 unless every setting the cycle depends on is declared."""
        if not self.position_controlled_spindle:
            raise ValueError(
                'G84/G74 are rigid tapping cycles and need a position-controlled spindle (ISO manual pp.143, 145): '
                'declare position_controlled_spindle=True'
            )
        missing = [
            name
            for name in (
                'tap_deep_hole_mode',
                'tap_retract_speed_percent',
                'tap_safety_clearance_mm',
                'tap_restores_spindle',
            )
            if getattr(self, name) is None
        ]
        if missing:
            raise ValueError(
                'G84/G74 depend on settings the program cannot set: declare '
                + ', '.join(missing)
                + ' (GUD _ZSFI[1], _ZSFI[2], _ZSFR[0] and the spindle state after the cycle)'
            )
        if self.tap_deep_hole_mode != 0:
            raise ValueError(
                'this reference taps in one pass: _ZSFI[1] = 2 or 3 selects deep-hole tapping (p.147) and the '
                'manual describes no other value, so only tap_deep_hole_mode=0 is accepted'
            )
        if self.tap_retract_speed_percent != 100:
            raise ValueError(
                'this reference retracts at the tapping speed: only tap_retract_speed_percent=100 (_ZSFI[2], p.144) '
                'is accepted'
            )
        if self.tap_restores_spindle is not True:
            raise ValueError(
                'the manual does not state the spindle speed after G84/G74: this reference needs a machine that '
                'turns the spindle again as before the cycle (tap_restores_spindle=True)'
            )
        if self.output_units != 'mm':
            raise ValueError('the unit of the safety clearance GUD under G20 is not stated: tapping needs mm output')


def define_machine(
    name,
    tools,
    *,
    startup_units,
    extern_toolprog_mode,
    cutting_edge_default,
    tool_corr_move_mode,
    tool_length_const,
    tool_change,
    coolant_codes,
    traverse_feed_mm_min,
    work_to_machine,
    **settings,
):
    """Bind installed settings without changing the controller or tool table.

    Each ToolSetting.length_register is its global ISO H ($TC_DPH) mapping.
    position_after_mm declares the physical tip pose after the OEM tool change;
    MD20382=0 means G43 H / G49 without axes must not traverse the length offset.
    Tapping (G84/G74) needs position_controlled_spindle, tap_deep_hole_mode,
    tap_retract_speed_percent, tap_safety_clearance_mm and tap_restores_spindle
    declared (see ReferenceMachine); without them a tapping motion is refused.
    """
    d = _dialect(tool_change, coolant_codes)
    curve_policy = settings.pop('curve_policy', 'linear')
    return ReferenceMachine(
        name,
        tools,
        dialect=d,
        planes=('xy',),
        arcs=curve_policy == 'exact_ijk',
        helices=curve_policy == 'exact_ijk',
        curve_policy=curve_policy,
        startup_units=startup_units,
        extern_toolprog_mode=extern_toolprog_mode,
        cutting_edge_default=cutting_edge_default,
        tool_corr_move_mode=tool_corr_move_mode,
        tool_length_const=tool_length_const,
        flood='flood' in coolant_codes,
        mist='mist' in coolant_codes,
        traverse_feed_mm_min=traverse_feed_mm_min,
        work_to_machine=work_to_machine,
        **settings,
    )


class Siemens808DISOPost(GCodePost):
    version = '808d-iso-milling-2016-1'

    def on_begin(self, program):
        for line in _PREFIX:
            self.emit(line)
        super().on_begin(program)
        # Cancel scale/rotation/mirror before resetting the coordinate system.
        i = self._lines.index('G92.1')
        self._lines[i : i + 1] = [*_RESETS, 'G92.1 X0. Y0. Z0.']

    def words(self, addresses, values, *, dimensional=True):
        words = super().words(addresses, values, dimensional=dimensional)
        # Literal decimals remove dependence on calculator/standard notation
        # and IS-B/IS-C input scaling, including the distinct F scaling factors.
        return [w if '.' in w else w + '.' for w in words] if dimensional else words

    def tap(self, motion):
        """Rigid tapping by G84 (right hand, p.143) or G74 (left hand, p.145).

        G98 G84 X Y Z R F, then G80: the cycle moves to R plus the safety
        clearance, stops the spindle oriented in axis mode, taps to Z, reverses
        back and retracts to the initial plane (pp.128, 144). No M29 is
        written: the cycles are rigid by themselves in ISO milling mode.
        """
        m = self.machine
        c = motion.curve
        if c.end[2] > c.start[2]:
            self.emit('G80')  # p.149: deselects the modal cycle
            return
        m.require_tapping()
        direction = getattr(self, '_spindle_direction', None)
        rpm = getattr(self, '_rpm', 0)
        if direction not in ('cw', 'ccw') or not rpm:
            raise ValueError('tapping needs the spindle running in the thread hand (M3 for G84, M4 for G74)')
        if c.start[:2] != c.end[:2] or not c.end[2] < c.start[2]:
            raise ValueError('G84/G74 tap straight down Z in G17')
        # p.145/147: in G94 F is the feed per minute, the lead times S.
        # R is the plane the safety clearance is added to: the cycle then
        # starts tapping, and ends, at the current Z.
        self.emit(
            'G98',
            'G74' if direction == 'ccw' else 'G84',
            *self.words(
                ('X', 'Y', 'Z', 'R', 'F'),
                (c.end[0], c.end[1], c.end[2], c.start[2] - m.tap_safety_clearance_mm, motion.feed_mm_min),
            ),
        )


class Siemens808DISODecoder(NativeGCodeDecoder):
    version = '808d-iso-milling-2016-nominal-1'

    def __init__(self, machine, program):
        super().__init__(machine, program)
        self.prefix = 0
        self.resets = set()

    def validate_initialized(self):
        super().validate_initialized()
        if self.prefix != len(_PREFIX) or self.resets != {50, 69, 50.1}:
            raise ValueError('incomplete ISO language/compressor/feed/scale/rotation/mirror initialization')

    def read_line(self, line, context=None):
        if (
            self.ended
            or len(line) > 1024
            or not line.isascii()
            or any(ord(c) < 32 and c != '\t' or ord(c) == 127 for c in line)
        ):
            raise ValueError('invalid ISO physical block or code after end')
        body = line.partition(';')[0].strip().upper()
        # Native parentheses are expressions, ISO parentheses are comments.
        if self.prefix == len(_PREFIX):
            body = re.sub(r'\([^()]*\)', ' ', body).strip()
        if '(' in body or ')' in body:
            raise ValueError('invalid comment for current language')
        body = re.sub(r'^N[0-9]{1,8}(?=\s|[A-Z]|$)', '', body).strip()
        if not body:
            return []
        if self.prefix < len(_PREFIX):
            if body != _PREFIX[self.prefix]:
                raise ValueError('expected standalone native preparation then G291')
            self.prefix += 1
            return []
        if self.tap_return is not None and body != 'G80':
            raise ValueError('G84/G74 must be deselected by G80 (p.149) before any other block')
        if body == 'G291':
            return []  # language reselection preserves tool/length/work offset
        cycle = re.fullmatch(r'G98\s+G(84|74)\s+(.+)', body)
        if cycle is not None:
            return [self._tap_cycle(cycle[1], cycle[2], context)]
        pieces = _WORDS.findall(body)
        if _WORDS.sub('', body).strip() or len({w for w, v in pieces}) != len(pieces):
            raise ValueError('unsupported or duplicate ISO words')
        values = {w: float(v) for w, v in pieces}
        if any(not math.isfinite(v) or abs(v) > 1e9 for v in values.values()) or 'G' in values and 'M' in values:
            raise ValueError('invalid or mixed ISO commands')
        g, m = values.get('G'), values.get('M')
        if g in (0, 290, 291):
            raise ValueError('G0/unprepared language changes are outside this reference')
        precision = 3 if self.machine.output_units == 'mm' else 4
        if any(w in 'XYZIJKF' and ('.' not in v or len(v.partition('.')[2]) > precision) for w, v in pieces):
            raise ValueError('ISO dimensional words require explicit decimals within IS-B precision')
        if g in (20, 21):
            if set(values) != {'G'} or g != (21 if self.machine.startup_units == 'mm' else 20):
                raise ValueError('unit block must be standalone and preserve declared startup units')
        if g in (50, 69):
            if set(values) != {'G'}:
                raise ValueError('scale/rotation cancel must be standalone')
            self.resets.add(g)
            return []
        if g in (50.1, 92.1):
            if set(values) != {'G', 'X', 'Y', 'Z'} or any(values[a] != 0 for a in 'XYZ'):
                raise ValueError('mirror/frame reset requires explicit zero XYZ')
            if g == 50.1:
                self.resets.add(g)
            else:
                if self.resets != {50, 69, 50.1}:
                    raise ValueError('cancel transforms before coordinate reset')
                self.ready.add('frame_reset')
            return []
        if (m in (0, 1, 6) or 'T' in values) and (self.rpm or self.coolant != 'off'):
            raise ValueError('explicit spindle/coolant off required before OEM stop/tool change')
        if m == 1 and not self.machine.optional_stop_enabled:
            raise ValueError('optional stop must be declared enabled')
        if 'S' in values and values['S'] != int(values['S']):
            raise ValueError('reference spindle RPM must be integral')
        return super().read_line(body, context)

    def _tap_cycle(self, code, rest, context):
        """Expand G98 G84/G74 X Y Z R F into the cycle's moves (pp.128, 144).

        Read from the written words alone. The cycle approaches R plus the
        declared safety clearance, which must be the current Z (no move), taps
        to Z at F, reverses back and returns to the initial plane (G98), also
        the current Z. G84 is the right-hand and G74 the left-hand cycle: the
        running spindle direction must be that hand.
        """
        m = self.machine
        m.require_tapping()
        pieces = _WORDS.findall(rest)
        if _WORDS.sub('', rest).strip() or sorted(w for w, v in pieces) != ['F', 'R', 'X', 'Y', 'Z']:
            raise ValueError('tapping cycle block is G98 G84/G74 X Y Z R F, each word once')
        if any('.' not in v or len(v.partition('.')[2]) > 3 for w, v in pieces):
            raise ValueError('ISO dimensional words require explicit decimals within IS-B precision')
        values = {w: float(v) for w, v in pieces}
        self.validate_initialized()
        if (
            context is None
            or context.get('kind') != 'tap'
            or self.active is None
            or self.length is None
            or self.offset is None
            or self.tool_pending
        ):
            raise ValueError('NC tapping cycle with unknown attribution/tool/length/offset')
        if self.cycle_active or self.units != 1 or self.plane != 'xy':
            raise ValueError('G84/G74 are read in mm and G17, outside another cycle')
        need = 'cw' if code == '84' else 'ccw'
        if self.rpm <= 0 or self.direction != need:
            raise ValueError(
                'tapping cycle hand differs from the running spindle direction: '
                'G84 is right-hand (M3), G74 left-hand (M4)'
            )
        start = self.position
        if values['X'] != start[0] or values['Y'] != start[1]:
            raise ValueError('tapping cycle must start at its hole position')
        if abs(values['R'] + m.tap_safety_clearance_mm - start[2]) > 1e-9:
            raise ValueError('tapping cycle R plus the declared safety clearance must be the current Z')
        if not values['Z'] < values['R']:
            raise ValueError('tapping cycle must go down')
        if not 0 < values['F'] <= m.max_feed_mm_min:
            raise ValueError('tapping cycle needs a positive feed within the machine maximum')
        bottom = (start[0], start[1], values['Z'])
        self.position = bottom
        self.tap_return = (start, bottom, values['F'])
        self.cycle_active = True
        return Motion(context['id'], context['operation'], self.active.tool, Line(start, bottom), values['F'], 'tap')


def post_iso(program, machine, *, lowering=None, **options):
    return post_events_iso(program, program.events, machine, lowering=lowering, **options)


def post_events_iso(header, events, machine, *, lowering=None, **options):
    if lowering is None and machine.curve_policy == 'linear':
        lowering = LoweringOptions()
    return post_events(
        header,
        events,
        machine=machine,
        post=Siemens808DISOPost,
        decoder=Siemens808DISODecoder,
        lowering=lowering,
        **options,
    )
