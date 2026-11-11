"""Ordinary Python reference post for SINUMERIK 808D ADVANCED native milling4.90.

Manual/provenance/supported subset: docs/siemens808d-post.md. OEM toolchange and
coolant bindings are required inputs. This example never connects to a machine.
"""

from dataclasses import dataclass, replace
import math
import re

from camlib.motion import Helix, Line, Motion
from camlib.post import Machine, GCodeDialect, GCodePost, NativeGCodeDecoder, LoweringOptions, post_events


_RESERVED_M = frozenset((0, 1, 2, 3, 4, 5, 6, 17, 19, 30, 40, 41, 42, 43, 44, 45, 70))
_WORD_LIMIT_MM = 99999.999
# CYCLE84 parameters (manual 166-167); the two PSYS entries are internal.
_CYCLE84 = (
    'RTP',
    'RFP',
    'SDIS',
    'DP',
    'DPR',
    'DTB',
    'SDAC',
    'MPIT',
    'PIT',
    'POSS',
    'SST',
    'SST1',
    'AXN',
    'PSYS1',
    'PSYS2',
    'VARI',
    'DAM',
    'VRT',
)
_NUMBER = re.compile(r'[+-]?(?:[0-9]+(?:\.[0-9]*)?|\.[0-9]+)')


def _check_coolant(codes):
    if set(codes) - {'off', 'flood', 'mist'} or 'off' not in codes:
        raise ValueError('declare OEM coolant off and optional flood/mist codes')
    if any(not isinstance(code, str) or not re.fullmatch(r'M[0-9]{1,2}', code) for code in codes.values()):
        raise ValueError('reference coolant codes must be documented OEM M0..M99')
    if any(int(code[1:]) in _RESERVED_M for code in codes.values()):
        raise ValueError('OEM coolant binding cannot reinterpret a reserved native M function')


@dataclass(frozen=True, kw_only=True)
class ReferenceMachine(Machine):
    reference: str = '808D ADVANCED milling4.90 native;6FC5398-4DP10-0BA7'
    curve_policy: str = 'linear'
    stop_window: str = 'fine'
    # CYCLE84 "can be used if the spindle ... is technically able to be
    # operated in the position-controlled spindle operation" (manual 167).
    position_controlled_spindle: bool = False
    # POSS, the oriented spindle stop CYCLE84 makes before tapping (manual
    # 166, 168), in degrees: 0 <= angle < 360 (SPOS, manual 109).
    tap_spindle_position_deg: float | None = None

    def __post_init__(self):
        super().__post_init__()
        angle = self.tap_spindle_position_deg
        if type(self.position_controlled_spindle) is not bool or (
            angle is not None
            and (
                isinstance(angle, bool)
                or not isinstance(angle, (int, float))
                or not 0 <= angle < 360
                or abs(round(angle, 4) - angle) > 1e-12
            )
        ):
            raise ValueError(
                'declare position_controlled_spindle as a boolean and tap_spindle_position_deg '
                '(CYCLE84 POSS) as 0 <= degrees < 360 with at most 4 decimals'
            )
        if angle is not None and not self.position_controlled_spindle:
            raise ValueError('tap_spindle_position_deg (CYCLE84 POSS) needs position_controlled_spindle=True')
        if self.curve_policy not in ('linear', 'exact_ijk') or self.stop_window not in ('fine', 'coarse'):
            raise ValueError('choose linear/exact_ijk curves and fine/coarse stop window')
        if (
            self.planes != ('xy',)
            or self.max_tool_number > 32000
            or self.max_length_register > 9
            or len(self.tools) > 64
        ):
            raise ValueError('reference subset requires G17, T1..32000, D1..9 and <=64 configured tool/edge fields')
        if math.dist(self.work_to_machine.z, (0, 0, 1)) > 1e-12:
            raise ValueError('reference subset requires vertical machine tool axis')
        if self.dialect.length_activation != 'tool_edge' or self.dialect.radius_tolerance_mm != 0:
            raise ValueError('reference uses per-tool D and uncorrected exact-IJK circles only')
        if self.arcs != (self.curve_policy == 'exact_ijk') or self.helices != (self.curve_policy == 'exact_ijk'):
            raise ValueError('curve capabilities must match reference policy')
        expected = {
            'units_mm': 'G710',
            'units_inch': 'G700',
            'absolute': 'G90',
            'plane_xy': 'G17',
            'feed_per_minute': 'G94',
            'radius_off': 'G40',
            'length_off': 'D0',
            'cycles_off': 'MCALL',
            'frame_reset': 'TRANS',
            'exact_stop': 'G60',
            'language_native': 'G290',
            'compression_off': 'COMPOF',
            'rapid': 'G0',
            'linear': 'G1',
            'arc_cw': 'G2',
            'arc_ccw': 'G3',
            'spindle_cw': 'M3',
            'spindle_ccw': 'M4',
            'spindle_off': 'M5',
            'tool_change': 'M6',
            'stop': 'M0',
            'optional_stop': 'M1',
            'end': 'M2',
        }
        if any(self.dialect.code(effect) != code for effect, code in expected.items()):
            raise ValueError('native reference command mapping differs from documented subset')
        if (
            self.dialect.axes != ('X', 'Y', 'Z')
            or self.dialect.centers != ('I', 'J', 'K')
            or (self.dialect.tool_word, self.dialect.length_word, self.dialect.feed_word, self.dialect.speed_word)
            != ('T', 'D', 'F', 'S')
        ):
            raise ValueError('native reference uses XYZ/IJK/T/D/F/S')
        if set(self.dialect.fixed_modes) != {'center_incremental', 'rpm_mode'} or self.dialect.parentheses_comments:
            raise ValueError('only native center/RPM invariants are fixed; parentheses are expressions')
        forbidden = {'center_incremental', 'rpm_mode', 'length_on'}
        if any(command.effect in forbidden for command in self.dialect.commands):
            raise ValueError('reference native invariants have no ISO/LinuxCNC opcode aliases')
        names = {'coolant_off': 'off', 'flood': 'flood', 'mist': 'mist'}
        _check_coolant({names[c.effect]: c.code for c in self.dialect.commands if c.effect in names})
        if max(self.max_feed_mm_min, self.max_spindle_rpm) > _WORD_LIMIT_MM or self.max_sweep_rad > math.tau:
            raise ValueError('native reference exceeds documented feed/RPM or single-turn subset')
        if any(not math.isfinite(v) or abs(v) > _WORD_LIMIT_MM for t in self.tools for v in t.position_after_mm):
            raise ValueError('reference tool/edge work poses exceed bounded software coordinate range')
        if self.dialect.work_offsets != ('G54', 'G55', 'G56', 'G57', 'G58', 'G59'):
            raise ValueError('native reference supports six explicit stored work offsets')

    def require_tapping(self):
        """Refuse CYCLE84 unless every premise it depends on is declared."""
        if not self.position_controlled_spindle or self.tap_spindle_position_deg is None:
            raise ValueError(
                'CYCLE84 rigid tapping needs a position-controlled spindle (manual p.167): declare '
                'position_controlled_spindle=True and tap_spindle_position_deg (POSS, the oriented stop angle)'
            )
        if self.output_units != 'mm':
            raise ValueError(
                'the manual gives the CYCLE84 thread lead PIT in mm only (p.166): tapping needs output_units="mm"'
            )


def define_machine(
    name, tools, *, tool_change, coolant_codes, work_to_machine, curve_policy='linear', stop_window='fine', **settings
):
    """Define the machine in Python. Required OEM bindings are not inferred.

    coolant_codes is {'off': 'M...', 'flood': 'M...', 'mist': 'M...'}; flood/mist
    optional. The caller supplies the actual PLC meanings and tool/edge poses.
    Linear policy uses bounded G1 for curves. exact_ijk permits only circles whose
    emitted start/end radii match exactly; unspecified controller corrections reject.
    Tapping (CYCLE84) needs position_controlled_spindle=True and
    tap_spindle_position_deg (POSS); without them a tapping motion is refused.
    """
    _check_coolant(coolant_codes)
    d = replace(
        GCodeDialect(),
        length_word='D',
        tool_change=tool_change,
        length_activation='tool_edge',
        parentheses_comments=False,
        radius_tolerance_mm=0,
        fixed_modes=('center_incremental', 'rpm_mode'),
    ).with_commands(
        units_mm='G710',
        units_inch='G700',
        center_incremental=None,
        rpm_mode=None,
        length_off='D0',
        length_on=None,
        cycles_off='MCALL',
        frame_reset='TRANS',
        exact_stop='G60',
        language_native='G290',
        compression_off='COMPOF',
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
    return ReferenceMachine(
        name,
        tools,
        dialect=d,
        planes=('xy',),
        arcs=curve_policy == 'exact_ijk',
        helices=curve_policy == 'exact_ijk',
        max_tool_number=32000,
        max_length_register=9,
        flood='flood' in coolant_codes,
        mist='mist' in coolant_codes,
        work_to_machine=work_to_machine,
        curve_policy=curve_policy,
        stop_window=stop_window,
        **settings,
    )


class Siemens808DPost(GCodePost):
    version = '808d-advanced-490-native-2'

    def on_begin(self, program):
        self._active_number = None
        super().on_begin(program)
        self.emit('G601' if self.machine.stop_window == 'fine' else 'G602')
        self.emit('RNDM=0')
        self.emit('CFTCP')  # manual99: F acts at cutter center

    def on_tool_change(self, event):
        if self._active_number == event.number:
            self.emit('D' + str(event.length_register))
        elif self.machine.dialect.tool_change == 'direct_t':
            self.emit('T' + str(event.number), 'D' + str(event.length_register))
        else:
            self.emit('T' + str(event.number))
            self.emit('M6', 'D' + str(event.length_register))
        self._active_number = event.number

    def words(self, addresses, values, *, dimensional=True):
        if any(abs(v) > _WORD_LIMIT_MM for v in values):
            raise ValueError('native reference coordinate/feed range exceeded')
        return super().words(addresses, values, dimensional=dimensional)

    def on_motion(self, motion):
        super().on_motion(motion)
        if isinstance(motion.curve, Helix):
            line = self._lines.pop()
            self.emit(line, 'TURN=0')  # zero additional turns; manual98-99/345

    def tap(self, motion):
        """Rigid tapping by CYCLE84 (manual 166-169), one call per hole.

        CYCLE84(RTP, RFP, SDIS, DP, DPR, DTB, SDAC, MPIT, PIT, POSS, SST, SST1,
        AXN, 0, 0, VARI): the cycle stops the spindle oriented at POSS (SPOS),
        taps to DP at SST, reverses back to RFP+SDIS at SST1, retracts to RTP
        and reprograms the earlier speed with the direction SDAC (p.167).
        """
        m = self.machine
        c = motion.curve
        if c.end[2] > c.start[2]:
            # The call already retracted (p.167); a comment carries the return.
            self.comment('CYCLE84 retracted')
            return
        m.require_tapping()
        direction = getattr(self, '_spindle_direction', None)
        rpm = getattr(self, '_rpm', 0)
        if direction not in ('cw', 'ccw') or not rpm:
            raise ValueError('tapping needs the spindle running in the thread hand (M3 right, M4 left) before CYCLE84')
        if c.start[:2] != c.end[:2] or not c.end[2] < c.start[2]:
            raise ValueError('CYCLE84 taps straight down the tool axis (G17, AXN=3)')
        if max(abs(c.start[2]), abs(c.end[2])) > _WORD_LIMIT_MM:
            raise ValueError('native reference coordinate/feed range exceeded')
        # p.166/168: PIT is the lead as a value, 0.001..2000 mm; its sign is
        # the direction of rotation in the thread (+ as M3, - as M4).
        lead = motion.feed_mm_min / rpm
        if not 0.001 <= lead <= 2000:
            raise ValueError('CYCLE84 thread lead PIT must be 0.001..2000 mm (manual p.166)')
        pit = ('-' if direction == 'ccw' else '') + f'{lead:.6f}'.rstrip('0').rstrip('.')
        plane = self.number(c.start[2])
        speed = self.number(rpm, dimensional=False)
        fields = (
            plane,  # RTP: the retraction plane is the starting level
            plane,  # RFP
            '',  # SDIS: no safety clearance (p.158 example N60)
            self.number(c.end[2]),  # DP, absolute
            '',  # DPR: the depth is absolute
            '',  # DTB: no dwell (p.168)
            '4' if direction == 'ccw' else '3',  # SDAC: the direction afterwards
            '',  # MPIT: the lead is given by PIT
            pit,
            f'{m.tap_spindle_position_deg:.4f}'.rstrip('0').rstrip('.'),  # POSS
            speed,  # SST
            speed,  # SST1
            '3',  # AXN: third axis of G17, Z (p.168-169)
            '0',
            '0',
            '0',  # VARI: one pass
        )
        # p.154: a cycle call stands in its own block; trailing parameters
        # (DAM, VRT: deep-hole tapping only) may be left off.
        self.emit('CYCLE84(' + ','.join(fields) + ')')


class Siemens808DDecoder(NativeGCodeDecoder):
    """Strict native4.90 subset; independently reads actual emitted blocks.

    Canonical tool changes explicitly select D before motion. Implicit D1 at tool
    activation is tracked, but an omitted explicit D is rejected in this subset.
    Nonzero rounding/compression, ISO, tilted length planes and circle correction
    are unsupported. No actual hardware or machine-data discovery is implied.
    """

    version = '808d-advanced-490-native-model-1'

    def __init__(self, machine, program):
        super().__init__(machine, program)
        self.native_mode = False
        self.window = None
        self.rounding_off = False
        self.circle_feed_off = False
        self.implicit_edge = None

    def validate_initialized(self):
        super().validate_initialized()
        if (
            not self.native_mode
            or self.window != self.machine.stop_window
            or not self.rounding_off
            or not self.circle_feed_off
        ):
            raise ValueError('native language, stop window, RNDM=0 and CFTCP must be explicit')

    def read_line(self, line, context=None):
        if self.ended:
            raise ValueError('NC after program end')
        if len(line) > 1024 or not line.isascii() or any(ord(c) < 32 and c != '\t' or ord(c) == 127 for c in line):
            raise ValueError('invalid native physical block')
        body = line.split(';', 1)[0].strip().upper()
        if not body:
            return []
        if self.tap_return is not None:
            raise ValueError('CYCLE84 retracts by itself: no block belongs to its return')
        if body.startswith('N'):
            match = re.match(r'N[0-9]{1,8}(?=\s|[A-Z]|$)', body)
            if match is None:
                raise ValueError('native block number must be integer 0..99999999')
            body = body[match.end() :].strip()
        if body.startswith('CYCLE84'):
            if not self.native_mode:
                raise ValueError('select native G290 before native instructions')
            return [self._cycle84(body, context)]
        if re.search(r'G\s*0*29[01](?:\.0*)?(?![0-9.])', body):
            if body != 'G290':
                raise ValueError('only standalone G290 native mode supported; ISO is separate')
            self.native_mode = True
        if not self.native_mode:
            raise ValueError('select native G290 before native instructions')
        if body in ('G601', 'G602'):
            self.window = 'fine' if body == 'G601' else 'coarse'
            if self.window != self.machine.stop_window:
                raise ValueError('exact-stop window differs from machine declaration')
            return []
        if body == 'CFTCP':
            self.circle_feed_off = True
            return []
        if re.fullmatch(r'RNDM\s*=\s*0(?:\.0*)?', body):
            self.rounding_off = True
            return []
        has_turn = 'TURN' in body
        if has_turn:
            match = re.fullmatch(r'(G[23]\s+.+?)\s+TURN\s*=\s*0', body)
            if match is None:
                raise ValueError('only explicit TURN=0 native helix subset supported')
            body = match[1]
        if body == 'D0':
            self.implicit_edge = None
        result = super().read_line(body, context)
        for event in result:
            curve = getattr(event, 'curve', None)
            if curve is not None:
                values = curve.end
                if hasattr(curve, 'center'):
                    values = (*values, *(c - a for c, a in zip(curve.center, curve.start)))
                if any(abs(v) > _WORD_LIMIT_MM for v in values):
                    raise ValueError('decoded native reference coordinate range exceeded')
        helical = any(isinstance(getattr(event, 'curve', None), Helix) for event in result)
        if helical != has_turn:
            raise ValueError('native helix subset requires explicit TURN=0; no TURN on other forms')
        return result

    def _cycle84(self, body, context):
        """Expand a CYCLE84 call into the moves the manual gives (p.167).

        Read from the written parameters alone: the approach to RFP+SDIS and
        the final retraction to RTP must be no moves (all at the current Z),
        the lead and speed give the synchronized feed, and the sign of the
        lead and SDAC must both be the running spindle direction.
        """
        m = self.machine
        m.require_tapping()
        match = re.fullmatch(r'CYCLE84\(([^()]*)\)', body)
        if match is None:
            raise ValueError('CYCLE84 stands alone in its block with one parameter list (manual p.154)')
        fields = [f.strip() for f in match[1].split(',')]
        if not 16 <= len(fields) <= len(_CYCLE84) or any(f and not _NUMBER.fullmatch(f) for f in fields):
            raise ValueError('CYCLE84 needs its parameters up to VARI as plain numbers (manual p.166-167)')
        p = dict.fromkeys(_CYCLE84)
        p.update((name, float(f)) for name, f in zip(_CYCLE84, fields) if f)
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
        if self.units != 1 or self.plane != 'xy':
            raise ValueError('CYCLE84 is read in mm (G710) and G17 only')
        start = self.position
        if p['RTP'] is None or p['RFP'] is None or abs(p['RTP'] - start[2]) > 1e-9 or abs(p['RFP'] - start[2]) > 1e-9:
            raise ValueError('CYCLE84 RTP and RFP must be the current Z: the cycle starts and ends where it is called')
        if p['SDIS'] or p['DPR'] is not None or p['DTB'] or p['MPIT']:
            raise ValueError('CYCLE84 subset: no SDIS, DPR, DTB or MPIT (absolute depth DP and lead PIT only)')
        if p['DP'] is None or not p['DP'] < start[2] or abs(p['DP']) > _WORD_LIMIT_MM:
            raise ValueError('CYCLE84 needs an absolute final depth DP below the reference plane')
        if p['AXN'] != 3 or p['PSYS1'] != 0 or p['PSYS2'] != 0 or p['VARI'] != 0 or p['DAM'] or p['VRT']:
            raise ValueError('CYCLE84 subset: AXN=3 (Z in G17), internal parameters 0 and VARI=0 (one pass)')
        if p['POSS'] is None or abs(p['POSS'] - m.tap_spindle_position_deg) > 1e-9:
            raise ValueError('CYCLE84 POSS differs from the declared tap_spindle_position_deg')
        if self.rpm <= 0 or p['SST'] != self.rpm or p['SST1'] != self.rpm:
            raise ValueError('CYCLE84 SST and SST1 must repeat the running spindle speed')
        lead = p['PIT']
        if lead is None or not 0.001 <= abs(lead) <= 2000:
            raise ValueError('CYCLE84 thread lead PIT must be 0.001..2000 mm (manual p.166)')
        # p.168: positive lead taps right-hand (as M3), negative left-hand
        # (as M4). p.167: afterwards the spindle turns as SDAC says.
        hand = 'cw' if lead > 0 else 'ccw'
        if hand != self.direction:
            raise ValueError(
                'CYCLE84 lead sign (thread hand) differs from the running spindle direction: '
                'positive PIT is right-hand (M3), negative left-hand (M4)'
            )
        if p['SDAC'] != (3 if self.direction == 'cw' else 4):
            raise ValueError('CYCLE84 SDAC must restore the running spindle direction (3 after M3, 4 after M4)')
        bottom = (start[0], start[1], p['DP'])
        feed = abs(lead) * self.rpm
        self.position = bottom
        self.tap_return = (start, bottom, feed)
        return Motion(context['id'], context['operation'], self.active.tool, Line(start, bottom), feed, 'tap')

    def _activate(self, number):
        super()._activate(number)
        self.length = 1
        self.implicit_edge = 1  # manual115; no traverse before explicit D

    def _length(self, number):
        event = super()._length(number)
        self.implicit_edge = None
        return event


def post_808d(program, machine, *, lowering=None, **options):
    return post_events_808d(program, program.events, machine, lowering=lowering, **options)


def post_events_808d(header, events, machine, *, lowering=None, **options):
    if lowering is None and machine.curve_policy == 'linear':
        lowering = LoweringOptions()
    return post_events(
        header, events, machine=machine, post=Siemens808DPost, decoder=Siemens808DDecoder, lowering=lowering, **options
    )
