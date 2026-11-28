"""Makera Carvera three-axis nominal post; see docs/carvera-post.md.

Sources: MakeraInc/CarveraProfiles CAM_Post_Processors/Fusion360-profiles/Carvera.cps
and https://carvera-community.gitbook.io/docs/firmware/supported-commands.
No controller connection. Machine offsets, calibration and routes are premises.
"""

from dataclasses import dataclass, replace
import math
import re

from camlib.post import Machine, GCodeDialect, GCodePost, NativeGCodeDecoder

# Only documented words. M4 is not wired on stock Carvera; M7 is AIR, not
# liquid mist. M30 does not stop spindle/air: those must be stopped explicitly.
_COMMANDS = dict(
    units_mm='G21',
    absolute='G90',
    plane_xy='G17',
    plane_xz='G18',
    plane_yz='G19',
    feed_per_minute='G94',
    rapid='G0',
    linear='G1',
    arc_cw='G2',
    arc_ccw='G3',
    spindle_cw='M3',
    spindle_off='M5',
    tool_change='M6',
    mist='M7',
    coolant_off='M9',
    end='M30',
    dwell='G4',
)
_FIXED = ('center_incremental', 'rpm_mode', 'radius_off', 'cycles_off', 'frame_reset', 'exact_stop')


def _dialect(dwell_max_s=None):
    d = GCodeDialect()
    return replace(
        d,
        commands=tuple(type(d.commands[0])(k, v) for k, v in _COMMANDS.items()),
        fixed_modes=_FIXED,
        radius_tolerance_mm=0,
        dwell_max_s=dwell_max_s,
    )


@dataclass(frozen=True, kw_only=True)
class ReferenceMachine(Machine):
    # Before the first M6, the loaded assembly's gauge length (zero for an
    # empty spindle). Needed to prove the first G53 retract adds no motion.
    initial_gauge_length_mm: float
    unmodified_coordinates: bool
    automatic_tool_length: bool

    def __post_init__(self):
        if self.output_units != 'mm':
            raise ValueError('Carvera requires mm: G20 halts the firmware')
        super().__post_init__()
        if self.dialect != _dialect(self.dialect.dwell_max_s):
            raise ValueError('Carvera command mapping differs from the documented subset')
        if not self.unmodified_coordinates or not self.automatic_tool_length:
            raise ValueError('Carvera requires unmodified coordinates and automatic M6 tool-length calibration')
        if not math.isfinite(self.initial_gauge_length_mm) or not 0 <= self.initial_gauge_length_mm <= 1e6:
            raise ValueError('Carvera initial_gauge_length_mm must be finite and nonnegative')
        f = self.work_to_machine
        if (tuple(f.x), tuple(f.y), tuple(f.z)) != ((1, 0, 0), (0, 1, 0), (0, 0, 1)):
            raise ValueError('Carvera work offsets require Cartesian translation only')
        if self.tool_change_position_mode != 'table' or self.flood:
            raise ValueError('Carvera requires declared table poses after M6 and no flood coolant')
        if self.planes != ('xy', 'xz', 'yz') or not self.arcs or not self.helices or self.max_sweep_rad > math.pi:
            raise ValueError('Carvera requires native XYZ-plane arcs and helices of at most 180 degrees')

        # Carvera.cps error6: stock ATC has six slots; no manual-change path.
        if self.max_tool_number > 6 or self.max_length_register > 6:
            raise ValueError('Carvera reference tool and length slots must be 1..6')

    def to_dict(self):
        result = super().to_dict()
        fields = ('initial_gauge_length_mm', 'unmodified_coordinates', 'automatic_tool_length')
        result['controller'] = dict(name='carvera', **{key: result.pop(key) for key in fields})
        return result


def define_machine(
    name,
    tools,
    *,
    initial_gauge_length_mm,
    unmodified_coordinates,
    automatic_tool_length,
    dwell_max_s=None,
    **machine,
):
    """Declare actual limits/offsets/tool table; mist=True explicitly means AIR.

    Initial and final poses must already have gauge-line machine Z=-3.
    Plan the retracts in the setup; the post never adds unverified travel.
    ``dwell_max_s`` has no default: declare it to write G4 dwells.
    """
    return ReferenceMachine(
        name,
        tools,
        dialect=_dialect(dwell_max_s),
        planes=('xy', 'xz', 'yz'),
        arcs=True,
        helices=True,
        max_sweep_rad=math.pi,
        initial_gauge_length_mm=initial_gauge_length_mm,
        unmodified_coordinates=unmodified_coordinates,
        automatic_tool_length=automatic_tool_length,
        **machine,
    )


class CarveraPost(GCodePost):
    controller_name = 'carvera'
    version = 'carvera-nominal-2'

    def emit(self, *words):
        if len(words) == 1:
            code = words[0]
            if code in ('G17', 'G18', 'G19') and self.modal('carvera_plane', code) is None:
                return
            # Switch.cpp toggles outputs at parse time. Makera's coolant
            # sequences explicitly drain the planner before M7 and M9.
            if code in ('M7', 'M9'):
                super().emit('M400')
        super().emit(*words)

    def on_begin(self, program):
        self.geometry_tolerance_mm = program.tolerance_mm
        for code in ('G21', 'G90', 'G17', 'G94', 'M5', 'M9'):
            self.emit(code)
        self._spindle_on = False
        self._coolant_mode = 'off'

    def on_tool_change(self, event):
        self.emit('G53 G0 Z-3')
        self.emit(f'T{event.number}', 'M6')

    def on_beam(self, event):
        raise ValueError('Carvera reference supports milling only; beam control is not declared')

    def on_finish(self):
        if self._spindle_on:
            self.emit('M5')
        if self._coolant_mode != 'off':
            self.emit('M9')
        self.emit('G53 G0 Z-3')
        self.emit('M30')


class CarveraDecoder(NativeGCodeDecoder):
    """Independent actual-word reader. No formatter helpers or source geometry."""

    controller_name = 'carvera'
    version = 'carvera-nominal-2'

    def __init__(self, machine, program):
        super().__init__(machine, program)
        self.ready.add('length')  # M6 measures/applies TLO; no G49 initialization.
        self.retracted = False
        self.air_sync_pending = False

    def validate_initialized(self):
        super().validate_initialized()
        if self.air_sync_pending:
            raise ValueError('Carvera M400 must immediately precede M7 or M9')

    def read_line(self, line, context=None):
        if self.ended:
            raise ValueError('Carvera code after M30')
        if len(line) > 1024 or not line.isascii() or any(ord(ch) < 32 and ch != '\t' or ord(ch) == 127 for ch in line):
            raise ValueError('invalid Carvera physical block')
        line = line.partition(';')[0].strip().upper()
        if self.air_sync_pending:
            if line not in ('M7', 'M9'):
                raise ValueError('Carvera M400 must immediately precede M7 or M9')
            self.air_sync_pending = False
            return super().read_line(line, context)
        if line in ('M7', 'M9'):
            raise ValueError('Carvera air change requires immediately preceding M400')
        if line == 'M400':
            self.air_sync_pending = True
            return []
        # G53 must explicitly select G0: firmware docs say the following G0/G1
        # uses MCS. Do not depend on the preceding modal motion command.
        if line == 'G53 G0 Z-3':
            self.validate_initialized()
            length = self.active.gauge_length_mm if self.active else self.machine.initial_gauge_length_mm
            z = self.position[2] + self.machine.work_to_machine.origin_mm[2] + length
            # Compare the posted machine coordinate at half one output unit,
            # not binary64 cancellation noise (25.4 - 88.4 + 60).
            if abs(z + 3) > 0.5 / 10**self.machine.decimals:
                raise ValueError('Carvera G53 retract requires planned gauge-line machine Z=-3; plan the retract')
            self.retracted = True
            return []
        # Independently recognize only the combined M6 syntax. The common
        # reader selects the actual table tool, then its automatic length.
        change = re.fullmatch(r'T([1-9][0-9]*) M6', line)
        if change:
            if not self.retracted or self.rpm != 0 or self.coolant != 'off':
                raise ValueError('Carvera M6 requires G53 retract and spindle/air off')
            self._activate(int(change.group(1)))
            events = [self._length(self.active.length_register)]
            self.retracted = False
            return events
        if line == 'M30' and (not self.retracted or self.rpm != 0 or self.coolant != 'off'):
            raise ValueError('Carvera M30 requires G53 retract and spindle/air off')
        if line == 'M30':
            self.ended = True
            return []
        # Reject alternate M6 shapes and H words, even if the generic reader
        # could interpret them as a separate length-register activation.
        words = re.findall(r'([A-Z])\s*([+-]?(?:[0-9]+(?:\.[0-9]*)?|\.[0-9]+))', line)
        if any(a in 'HT' or a == 'M' and float(v) in (6, 7, 9, 30, 400) for a, v in words):
            raise ValueError(
                'Carvera requires T<n> M6 without H; air requires standalone M400 immediately before M7 or M9'
            )
        events = super().read_line(line, context)
        if any(getattr(e, 'curve', None) is not None for e in events):
            self.retracted = False
        return events
