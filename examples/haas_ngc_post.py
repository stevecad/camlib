"""Ordinary Python Haas NGC2020 three-axis reference. See docs/haas-ngc-post.md.

Explicit setup premises, actual-word decoding and native continuous checks;
no controller connection or machine qualification.
"""

from dataclasses import dataclass, replace
from itertools import islice
import math
import re

from camlib.motion import Arc, Helix, Motion
from camlib.post import Machine, GCodeDialect, GCodePost, NativeGCodeDecoder, LoweringOptions, post_events


_WORDS = re.compile(r'([A-Z])\s*([+-]?(?:\d+(?:\.\d*)?|\.\d+))')
_EXPECTED = {
    'units_mm': 'G21',
    'units_inch': 'G20',
    'absolute': 'G90',
    'plane_xy': 'G17',
    'feed_per_minute': 'G94',
    'radius_off': 'G40',
    'length_off': 'G49',
    'length_on': 'G43',
    'cycles_off': 'G80',
    'frame_reset': 'G52',
    'exact_stop': 'G61',
    'linear': 'G1',
    'arc_cw': 'G2',
    'arc_ccw': 'G3',
    'spindle_cw': 'M3',
    'spindle_ccw': 'M4',
    'spindle_off': 'M5',
    'tool_change': 'M6',
    'flood': 'M8',
    'coolant_off': 'M9',
    'stop': 'M0',
    'optional_stop': 'M1',
    'end': 'M30',
}


_CYCLE_EFFECTS = (
    'dwell',
    'drill_cycle',
    'drill_dwell_cycle',
    'peck_cycle',
    'chip_break_cycle',
    'cycle_return_initial',
    'bore_cycle',
    'bore_dwell_cycle',
)


def _cycle_codes(setting_22_mm, setting_52_mm):
    """The documented canned cycles this machine's settings allow."""
    codes = {
        'dwell': 'G4',
        'drill_cycle': 'G81',
        'drill_dwell_cycle': 'G82',
        'cycle_return_initial': 'G98',
        'bore_cycle': 'G85',
        'bore_dwell_cycle': 'G89',
    }
    if setting_22_mm is not None:
        codes['chip_break_cycle'] = 'G73'
        if setting_52_mm == 0:
            codes['peck_cycle'] = 'G83'
    return codes


@dataclass(frozen=True, kw_only=True)
class ReferenceMachine(Machine):
    coordinate_system: str
    h_t_agreement: bool
    g92_shift_mm: tuple
    rotation_scaling: bool
    program_number: int = 1000
    optional_stop_enabled: bool = False
    mirror_settings_xyz: tuple = (False, False, False)
    curve_policy: str = 'linear'
    # Installed rigid tapping option: G84 X Y Z R F (right hand) and G74
    # X Y Z R F (reverse tap, left hand), F = pitch x S (G94), cancelled by
    # G80 (96-8210 Rev N: 5.8.2 p. 209, G74 p. 351, G84 p. 361, feed formula
    # p. 304). Without it tapping is refused.
    rigid_tapping: bool = False
    # Cutter compensation (wear): G01 G41/G42 D X Y on the lead-in move and
    # G40 G01 X Y on the lead-out (Haas: no start or cancel on an arc). The D
    # register holds only the wear, as a diameter or a radius per Setting 40;
    # Setting 58 (Fanuc or YASNAC) does not change a wear-only offset on the
    # tangent lead moves camlib writes.
    cutter_compensation: bool = False
    # Canned drilling (96-8210 Rev N, G-codes chapter): G81 drill, G82 spot
    # drill (dwell P), G85 bore in/out, G89 bore with dwell (P), G98 return,
    # cancelled by G80, always written. P is written in seconds with a decimal
    # point (G04 note: "G04 P10. is a dwell of 10 seconds; G04 P10 is a dwell
    # of 10 milliseconds"). G73/G83 rely on installed settings:
    # Setting 22 (Can Cycle Delta Z): the G73 chip-clearing retract and the G83
    # rapid re-entry above the previous peck (figures F7.21, F7.29).
    # Setting 52 (G83 Retract Above R): G83 clears chips at R + Setting 52;
    # camlib models the retract to R, so G83 needs Setting 52 = 0.
    setting_22_mm: float | None = None
    setting_52_mm: float | None = None
    reference: str = 'Haas NGC milling;96-8210 RevN December2020'

    def __post_init__(self):
        super().__post_init__()
        object.__setattr__(self, 'g92_shift_mm', tuple(islice(iter(self.g92_shift_mm), 4)))
        object.__setattr__(self, 'mirror_settings_xyz', tuple(islice(iter(self.mirror_settings_xyz), 4)))
        if self.coordinate_system not in ('fanuc', 'haas') or any(
            type(v) is not bool
            for v in (self.h_t_agreement, self.rotation_scaling, self.optional_stop_enabled, *self.mirror_settings_xyz)
        ):
            raise ValueError('declare Setting33 fanuc/haas and boolean machine options')
        if self.g92_shift_mm != (0, 0, 0) or self.mirror_settings_xyz != (False, False, False):
            raise ValueError('reference requires declared zero G92 and mirror Settings45..47 OFF')
        if type(self.program_number) is not int or not 1 <= self.program_number <= 99999:
            raise ValueError('reference O program number must be 1..99999')
        if self.traverse_feed_mm_min is None:
            raise ValueError('declare positive traverse_feed_mm_min; G0 dogleg is not this path model')
        if self.curve_policy not in ('linear', 'exact_ijk') or self.planes != ('xy',):
            raise ValueError('reference uses G17 and linear/exact_ijk curves')
        if (
            self.arcs != (self.curve_policy == 'exact_ijk')
            or self.helices != (self.curve_policy == 'exact_ijk')
            or self.max_sweep_rad > math.tau
        ):
            raise ValueError('reference curve capabilities must match single-turn policy')
        if self.decimals > (3 if self.output_units == 'mm' else 4):
            raise ValueError('reference coordinate precision is at most 3 metric/4 inch decimals')
        if self.max_tool_number > 200 or self.max_length_register > 200:
            raise ValueError('reference physical T/H table subset is 1..200')
        if self.h_t_agreement and any(t.number != t.length_register for t in self.tools):
            raise ValueError('Setting15 ON requires matching T/H')
        if (
            self.work_to_machine.x != (1, 0, 0)
            or self.work_to_machine.y != (0, 1, 0)
            or self.work_to_machine.z != (0, 0, 1)
        ):
            raise ValueError('reference stored work offsets require Cartesian translation only')
        d = self.dialect
        expected = dict(
            _EXPECTED,
            **({'tap_cycle': 'G84', 'tap_cycle_left': 'G74'} if self.rigid_tapping is True else {}),
            **_cycle_codes(self.setting_22_mm, self.setting_52_mm),
            **({'radius_left': 'G41', 'radius_right': 'G42'} if self.cutter_compensation is True else {}),
        )
        for name in ('setting_22_mm', 'setting_52_mm'):
            value = getattr(self, name)
            if value is not None and (
                type(value) not in (int, float) or not math.isfinite(value) or not 0 <= value <= 100
            ):
                raise ValueError(f'{name} must be a declared distance in mm (0..100)')
        if self.setting_22_mm is not None and self.setting_22_mm <= 0:
            raise ValueError('setting_22_mm (Setting 22, Can Cycle Delta Z) must be positive')
        if (d.peck_reentry_mm, d.chip_break_lift_mm) != (self.setting_22_mm, self.setting_22_mm):
            raise ValueError('G73/G83 return amounts must be Setting 22 (setting_22_mm)')
        if type(self.rigid_tapping) is not bool or type(self.cutter_compensation) is not bool:
            raise ValueError('declare the rigid tapping and cutter compensation options as booleans')
        if d.compensation_with_move != self.cutter_compensation:
            raise ValueError('Haas starts and cancels cutter compensation in the block of its XY move')
        if {c.effect: c.code for c in d.commands} != expected or len(d.commands) != len(expected):
            raise ValueError('reference command mapping differs from documented Haas subset')
        if (d.axes, d.centers, d.tool_word, d.length_word, d.feed_word, d.speed_word) != (
            ('X', 'Y', 'Z'),
            ('I', 'J', 'K'),
            'T',
            'H',
            'F',
            'S',
        ):
            raise ValueError('reference uses XYZ/IJK/T/H/F/S')
        if d.tool_change != 'm6' or d.length_activation != 'explicit' or d.radius_tolerance_mm != 0 or self.mist:
            raise ValueError('reference uses explicit M6/G43/H, exact-IJK and flood/off (M7 is shower)')
        if set(d.fixed_modes) != {'center_incremental', 'rpm_mode'} or not d.parentheses_comments:
            raise ValueError('reference fixed modes are incremental IJK and RPM only')
        if d.work_offsets != ('G54', 'G55', 'G56', 'G57', 'G58', 'G59'):
            raise ValueError('reference supports six stored work offsets')


def define_machine(
    name,
    tools,
    *,
    traverse_feed_mm_min,
    coordinate_system,
    h_t_agreement,
    g92_shift_mm,
    rotation_scaling,
    work_to_machine,
    **settings,
):
    """Bind the actual setup in normal Python; premises are captured in run identity.

    rotation_scaling declares the installed option: omit unavailable G50/G69,
    otherwise explicitly cancel those modes. G92 must already be zero. Stored
    table poses include explicit G43/H activation; tool-change routes are external.
    """
    rigid = settings.get('rigid_tapping', False)
    comp = settings.get('cutter_compensation', False)
    delta_z = settings.get('setting_22_mm')
    cycles = _cycle_codes(delta_z, settings.get('setting_52_mm'))
    d = replace(
        GCodeDialect(),
        radius_tolerance_mm=0,
        fixed_modes=('center_incremental', 'rpm_mode'),
        compensation_with_move=comp is True,
        cycle_dwell_unit='s',
        dwell_resolution_s=0.001,
    ).with_commands(
        rapid=None,
        plane_xz=None,
        plane_yz=None,
        center_incremental=None,
        rpm_mode=None,
        frame_reset='G52',
        exact_stop='G61',
        mist=None,
        end='M30',
        tap_synchronized=None,
        tap_cycle='G84' if rigid is True else None,
        tap_cycle_left='G74' if rigid is True else None,
        radius_left='G41' if comp is True else None,
        radius_right='G42' if comp is True else None,
        # G86 stops the spindle at the bottom; the manual does not say it
        # restarts, which camlib's G86 model assumes: not declared.
        bore_stop_cycle=None,
        **{effect: cycles.get(effect) for effect in _CYCLE_EFFECTS},
    )
    # The pecking return amounts are Setting 22, set with the commands so the
    # dialect is validated once with both.
    d = replace(d, peck_reentry_mm=delta_z, chip_break_lift_mm=delta_z)
    curve_policy = settings.pop('curve_policy', 'linear')
    return ReferenceMachine(
        name,
        tools,
        dialect=d,
        planes=('xy',),
        arcs=curve_policy == 'exact_ijk',
        helices=curve_policy == 'exact_ijk',
        max_tool_number=settings.pop('max_tool_number', 200),
        max_length_register=settings.pop('max_length_register', 200),
        mist=False,
        traverse_feed_mm_min=traverse_feed_mm_min,
        coordinate_system=coordinate_system,
        h_t_agreement=h_t_agreement,
        g92_shift_mm=g92_shift_mm,
        rotation_scaling=rotation_scaling,
        work_to_machine=work_to_machine,
        curve_policy=curve_policy,
        **settings,
    )


class HaasNGCPost(GCodePost):
    version = 'haas-ngc-2020-2'

    def on_begin(self, program):
        self.emit('%')
        self.emit('O' + str(self.machine.program_number))
        super().on_begin(program)
        # G52 needs explicit axis values, unlike a fictitious reset opcode.
        self._lines = [('G52 X0 Y0 Z0' if line == 'G52' else line) for line in self._lines if line != 'G61']
        if self.machine.rotation_scaling:
            self.emit('G50')
            self.emit('G69')
        self.emit('G100 X0 Y0 Z0')
        self.emit('G187')
        self.emit('G61')

    def on_motion(self, motion):
        c = motion.curve
        if not isinstance(c, Arc):
            return super().on_motion(motion)
        if c.plane != 'xy':
            raise ValueError('native Haas reference arc requires G17')
        if motion.feed_mm_min is None:
            raise ValueError('Haas circular motion requires positive explicit feed')
        self.emit('G17')
        full = c.start[:2] == c.end[:2]
        axes = ([] if full else [0, 1]) + ([2] if isinstance(c, Helix) else [])
        addresses = ['XYZ'[i] for i in axes] + ['I', 'J', 'F']
        end, offsets = self.circular_values(c)
        values = [end[i] for i in axes] + [offsets[i] for i in (0, 1)] + [motion.feed_mm_min]
        self.emit('G3' if c.sweep_rad > 0 else 'G2', *self.words(addresses, values))

    def on_cycle(self, event):
        m = self.machine
        if event.cycle in ('peck', 'chip_break'):
            code = 'G83' if event.cycle == 'peck' else 'G73'
            if m.setting_22_mm is None:
                raise ValueError(
                    f'Haas {code} needs Setting 22 (Can Cycle Delta Z): declare setting_22_mm with the installed value'
                )
            if event.cycle == 'peck' and m.setting_52_mm is None:
                raise ValueError('Haas G83 needs Setting 52 (G83 Retract Above R): declare setting_52_mm')
            if event.cycle == 'peck' and m.setting_52_mm != 0:
                raise ValueError(
                    f'Haas G83 with Setting 52 = {m.setting_52_mm} mm clears chips above R, which camlib does not '
                    'model: set Setting 52 to 0 (setting_52_mm=0) or drill without canned_cycle'
                )
            source = event.reentry_mm if event.cycle == 'peck' else event.lift_mm
            if abs(source - m.setting_22_mm) > 1e-9:
                name = 'reentry_clearance_mm' if event.cycle == 'peck' else 'chip_break_lift_mm'
                raise ValueError(
                    f'the operation {name} {source} mm differs from Setting 22 (Can Cycle Delta Z) '
                    f'{m.setting_22_mm} mm: set {name}={m.setting_22_mm}'
                )
        return super().on_cycle(event)

    def words(self, addresses, values, *, dimensional=True):
        words = super().words(addresses, values, dimensional=dimensional)
        # Settings77/162 reinterpret undotted numbers. Explicit decimals make
        # actual coordinates/IJK/F independent of both installed settings.
        return [w if '.' in w else w + '.' for w in words] if dimensional else words

    def on_finish(self):
        super().on_finish()
        self.emit('%')


class HaasNGCDecoder(NativeGCodeDecoder):
    """Independent actual-block model; no formatter/source geometry input."""

    version = 'haas-ngc-2020-nominal-1'

    def __init__(self, machine, program):
        super().__init__(machine, program)
        self.opened = False
        self.number = None
        self.end_seen = False
        self.closed = False
        self.rotation_off = not machine.rotation_scaling
        self.scaling_off = not machine.rotation_scaling
        self.mirror_off = False
        self.smoothing_reset = False
        precision = 3 if machine.output_units == 'mm' else 4
        scalar = rf'[+-]?(?:[0-9]+\.[0-9]{{0,{precision}}}|0)'
        self._linear_block = re.compile('G1 ' + ' '.join(a + scalar for a in 'XYZF'))

    def validate_initialized(self):
        super().validate_initialized()
        if not (
            self.opened
            and self.number == self.machine.program_number
            and self.rotation_off
            and self.scaling_off
            and self.mirror_off
            and self.smoothing_reset
        ):
            raise ValueError('incomplete Haas program/rotation/scaling/mirror/G187 initialization')

    def read_line(self, line, context=None):
        # This bounded grammar establishes only the extra reference syntax and
        # precision premises. The independent native reader still parses every
        # actual coordinate/F and checks readiness/tool/state/words. All other
        # block shapes use the full Python reference path below.
        if (
            not self.end_seen
            and len(line) <= 1024
            and self._linear_block is not None
            and self._linear_block.fullmatch(line)
        ):
            return super().read_line(line, context)
        if (
            self.ended
            or len(line) > 1024
            or not line.isascii()
            or any(ord(c) < 32 and c != '\t' or ord(c) == 127 for c in line)
        ):
            raise ValueError('invalid Haas physical block or code after closing percent')
        body = line.partition(';')[0].strip().upper()
        body = re.sub(r'\([^()]*\)', ' ', body).strip()
        if '(' in body or ')' in body:
            raise ValueError('invalid Haas comment')
        if not body:
            return []
        if body == '%':
            if not self.opened:
                self.opened = True
            elif self.end_seen:
                self.closed = True
                self.ended = True
            else:
                raise ValueError('closing percent requires explicit M30')
            return []
        if not self.opened or self.end_seen:
            raise ValueError('Haas framing requires percent/O/program/M30/percent')
        if re.fullmatch(r'O[0-9]{1,5}', body):
            if self.number is not None or int(body[1:]) != self.machine.program_number:
                raise ValueError('unexpected/repeated O program')
            self.number = int(body[1:])
            return []
        if self.number is None:
            raise ValueError('Haas program number required')
        body = re.sub(r'^N[0-9]{1,5}(?=\s|[A-Z]|$)', '', body).strip()
        pieces = _WORDS.findall(body)
        if _WORDS.sub('', body).strip():
            raise ValueError('unsupported/duplicate Haas words')
        if (
            self.machine.cutter_compensation
            and [w for w, v in pieces].count('G') == 2
            and (pieces[0] == ('G', '1') and pieces[1][0] == 'G' and float(pieces[1][1]) in (40, 41, 42))
        ):
            # G01 G41/G42 D X Y (start-up) or G01 G40 X Y (cancel): one block.
            pieces = [pieces[0], *pieces[2:]]
        if len({w for w, v in pieces}) != len(pieces):
            raise ValueError('unsupported/duplicate Haas words')
        values = {w: float(v) for w, v in pieces}
        if (
            'N' in values
            or any(not math.isfinite(v) or abs(v) > 1e9 for v in values.values())
            or 'G' in values
            and 'M' in values
        ):
            raise ValueError('invalid/mixed Haas command')
        g = values.get('G')
        m = values.get('M')
        if g == 0:
            raise ValueError('G0 dogleg rejected; use configured feed traverse')
        precision = 3 if self.machine.output_units == 'mm' else 4
        if any(w in 'XYZIJKF' and '.' not in text and float(text) != 0 for w, text in pieces):
            raise ValueError('Haas nonzero dimensional words need explicit decimal points (Settings77/162)')
        if any(w == 'P' and '.' not in text for w, text in pieces):
            # G04 note: "G04 P10. is a dwell of 10 seconds; G04 P10 is a dwell
            # of 10 milliseconds". This reference reads P only in seconds.
            raise ValueError('Haas dwell P without a decimal point is milliseconds; seconds need a decimal point')
        if any(w in 'XYZIJKF' and len(v.partition('.')[2]) > precision for w, v in pieces):
            raise ValueError('Haas reference coordinate/feed precision exceeded')
        if g in (20, 21) and g != (21 if self.machine.output_units == 'mm' else 20):
            raise ValueError('Haas file units differ from configured output units')
        if g in (52, 100):
            if set(values) != {'G', 'X', 'Y', 'Z'} or any(values[a] != 0 for a in 'XYZ'):
                raise ValueError('only explicit zero XYZ G52/G100 reset supported')
            if g == 52:
                self.ready.add('frame_reset')
            else:
                self.mirror_off = True
            return []
        if g in (50, 69, 187):
            if set(values) != {'G'} or g in (50, 69) and not self.machine.rotation_scaling:
                raise ValueError('unsupported Haas option/smoothing parameters')
            if g == 50:
                self.scaling_off = True
            elif g == 69:
                self.rotation_off = True
            else:
                self.smoothing_reset = True
            return []
        if m in (0, 1, 6) and (self.rpm != 0 or self.coolant != 'off'):
            raise ValueError('Haas stop/tool change requires explicit source spindle/coolant off')
        if m == 1 and not self.machine.optional_stop_enabled:
            raise ValueError('optional stop not declared enabled (Setting17/panel)')
        if 'S' in values and values['S'] != int(values['S']):
            raise ValueError('reference spindle RPM must be integral')
        result = super().read_line(body, context)
        for event in result:
            if isinstance(event, Motion) and isinstance(event.curve, Arc):
                if event.curve.start[:2] == event.curve.end[:2] and ('X' in values or 'Y' in values):
                    raise ValueError('Haas full circle must omit planar endpoint words')
        if m == 30:
            self.end_seen = True
            self.ended = False  # only closing % can complete this file
        return result

    def _length(self, number):
        if self.machine.h_t_agreement and (self.active is None or number != self.active.number):
            raise ValueError('Haas Setting15 T/H disagreement')
        return super()._length(number)


def post_haas(program, machine, *, lowering=None, **options):
    return post_events_haas(program, program.events, machine, lowering=lowering, **options)


def post_events_haas(header, events, machine, *, lowering=None, **options):
    if lowering is None and machine.curve_policy == 'linear':
        lowering = LoweringOptions()
    return post_events(
        header, events, machine=machine, post=HaasNGCPost, decoder=HaasNGCDecoder, lowering=lowering, **options
    )
