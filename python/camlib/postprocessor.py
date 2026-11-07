"""Ordinary Python machines, posts, and independently decoded nominal output.

No controller-name registry. A dialect declares command semantics; a post owns
formatting. Native code validates typed state and complete geometric trajectories.
This is a declared controller model, not external interpreter/machine qualification.
"""

from __future__ import annotations

from copy import deepcopy
from dataclasses import asdict, dataclass, field, replace
from itertools import islice
import math
import json
import re
import uuid
from types import MappingProxyType

from ._buffers import native
from .dynamics import MotionPolicy
from .motion import (
    Arc,
    Helix,
    Line,
    Motion,
    ToolChange,
    WorkOffset,
    Spindle,
    Coolant,
    Stop,
    Dwell,
    Cycle,
    CutterCompensation,
    SpindleOrient,
    Beam,
    HeightControl,
    SurfaceSpeed,
    FeedPerRev,
    WorkFrame,
)


@dataclass(frozen=True)
class Command:
    effect: str
    code: str


_DEFAULT_COMMANDS = tuple(
    Command(effect, code)
    for effect, code in (
        ('rapid', 'G0'),
        ('linear', 'G1'),
        ('arc_cw', 'G2'),
        ('arc_ccw', 'G3'),
        ('units_mm', 'G21'),
        ('units_inch', 'G20'),
        ('absolute', 'G90'),
        ('center_incremental', 'G91.1'),
        ('plane_xy', 'G17'),
        ('plane_xz', 'G18'),
        ('plane_yz', 'G19'),
        ('feed_per_minute', 'G94'),
        ('rpm_mode', 'G97'),
        ('radius_off', 'G40'),
        ('radius_left', 'G41'),
        ('radius_right', 'G42'),
        ('length_off', 'G49'),
        ('cycles_off', 'G80'),
        ('frame_reset', 'G92.1'),
        ('exact_stop', 'G61.1'),
        ('length_on', 'G43'),
        ('spindle_cw', 'M3'),
        ('spindle_ccw', 'M4'),
        ('spindle_off', 'M5'),
        ('tool_change', 'M6'),
        ('mist', 'M7'),
        ('flood', 'M8'),
        ('coolant_off', 'M9'),
        ('stop', 'M0'),
        ('optional_stop', 'M1'),
        ('end', 'M2'),
        # Dwell: G4 with the time in seconds in dwell_word (LinuxCNC, GRBL).
        ('dwell', 'G4'),
        # Canned drilling cycles (X Y Z R F; Q peck; P dwell), G98 return to the
        # initial level, cancelled by G80 (LinuxCNC interp_cycles.cc).
        ('drill_cycle', 'G81'),
        ('drill_dwell_cycle', 'G82'),
        ('peck_cycle', 'G83'),
        ('chip_break_cycle', 'G73'),
        ('cycle_return_initial', 'G98'),
        # Boring: feed out (G85), dwell and feed out (G89), spindle stop and
        # rapid out (G86, the spindle restarts in its direction).
        ('bore_cycle', 'G85'),
        ('bore_dwell_cycle', 'G89'),
        ('bore_stop_cycle', 'G86'),
        # LinuxCNC rigid tapping: G33.1 Z K(pitch) taps to Z, reverses the spindle
        # and returns to the start, synchronized to the spindle.
        ('tap_synchronized', 'G33.1'),
    )
)
# Canned tapping cycle (FANUC/Haas style G84 X Y Z R F, cancelled by G80).
# Declared per machine with with_commands(tap_cycle='G84', tap_synchronized=None).
# Oriented spindle stop (M19) and the fine boring cycle (G76 X Y Z R Q [P] F:
# feed to Z, dwell, oriented stop, shift Q away from the edge, rapid out,
# shift back, spindle CW) need the machine's orientation: declared per
# machine with oriented_edge_deg (LinuxCNC's G76 is a lathe threading cycle).
# Torch height control on/off (e.g. QtPlasmaC M63/M62 P2), with an optional
# word (height_control_word) written after the code.
# Lathe threading: a spindle-synchronized line (LinuxCNC G33 Z K<lead>,
# FANUC/Centroid G32 Z F<lead>), the lead in thread_lead_word.
_CYCLE_EFFECTS = frozenset(
    ('tap_cycle', 'fine_bore_cycle', 'spindle_orient', 'height_control_on', 'height_control_off', 'thread_synchronized')
)
# Lathe spindle and feed modes, declared per machine: constant surface
# speed (G96 S<m/min or ft/min>, its rpm limit as G50 S in its own block
# (css_limit) or a word in the G96 block (css_limit_word, LinuxCNC D)) and
# feed per revolution (G95; Centroid/FANUC lathes G99, with G98 per minute).
_LATHE_EFFECTS = frozenset(('css_on', 'css_limit', 'feed_per_rev'))
_GROUPS = {
    **dict.fromkeys(('css_on', 'rpm_mode'), 'speed_mode'),
    **dict.fromkeys(('feed_per_rev', 'feed_per_minute'), 'feed_mode'),
    **dict.fromkeys(
        (
            'rapid',
            'linear',
            'arc_cw',
            'arc_ccw',
            'tap_synchronized',
            'tap_cycle',
            'thread_synchronized',
            'drill_cycle',
            'drill_dwell_cycle',
            'peck_cycle',
            'chip_break_cycle',
            'bore_cycle',
            'bore_dwell_cycle',
            'bore_stop_cycle',
            'fine_bore_cycle',
        ),
        'motion',
    ),
    **dict.fromkeys(('units_mm', 'units_inch'), 'units'),
    **dict.fromkeys(('plane_xy', 'plane_xz', 'plane_yz'), 'plane'),
    **dict.fromkeys(('spindle_cw', 'spindle_ccw', 'spindle_off', 'spindle_orient'), 'spindle'),
    **dict.fromkeys(('flood', 'mist', 'coolant_off'), 'coolant'),
    **dict.fromkeys(('length_on', 'length_off'), 'length'),
    **dict.fromkeys(('radius_off', 'radius_left', 'radius_right'), 'radius'),
    **dict.fromkeys(('stop', 'optional_stop', 'end'), 'stop'),
    **dict.fromkeys(('height_control_on', 'height_control_off'), 'height_control'),
}
_REQUIRED = frozenset(
    (
        'units',
        'absolute',
        'center_incremental',
        'plane',
        'feed_per_minute',
        'rpm_mode',
        'radius_off',
        'length',
        'cycles_off',
        'frame_reset',
        'exact_stop',
        'spindle_off',
        'coolant_off',
    )
)
# Lathe X mode codes (LinuxCNC G7 diameter / G8 radius), written in the
# header when declared.
_OPTIONAL_EFFECTS = frozenset(('language_native', 'compression_off', 'lathe_diameter_mode', 'lathe_radius_mode'))
_NUMBER = re.compile(r'[+-]?(?:[0-9]+(?:\.[0-9]*)?|\.[0-9]+)')
_WORD = re.compile(r'([A-Za-z])\s*([+-]?(?:[0-9]+(?:\.[0-9]*)?|\.[0-9]+))')
_INVALID_BLOCK = re.compile(r'[^\t\x20-\x7e]')
_STATE_ONLY = (
    frozenset(
        (
            'units_mm',
            'units_inch',
            'absolute',
            'center_incremental',
            'plane_xy',
            'plane_xz',
            'plane_yz',
            'feed_per_minute',
            'rpm_mode',
            'radius_off',
            'length_off',
            'cycles_off',
            'frame_reset',
            'exact_stop',
            'cycle_return_initial',
        )
    )
    | _OPTIONAL_EFFECTS
)
_DRILL_CYCLES = {
    'drill': 'drill_cycle',
    'dwell': 'drill_dwell_cycle',
    'peck': 'peck_cycle',
    'chip_break': 'chip_break_cycle',
    'bore': 'bore_cycle',
    'bore_dwell': 'bore_dwell_cycle',
    'bore_stop': 'bore_stop_cycle',
    'fine_bore': 'fine_bore_cycle',
}
_DWELL_CYCLES = frozenset(('drill_dwell_cycle', 'bore_dwell_cycle'))
_BORE_CYCLES = frozenset(('bore_cycle', 'bore_dwell_cycle', 'bore_stop_cycle', 'fine_bore_cycle'))


def _kind(rapid, kind):
    """Decoded role: a rapid keeps its source role only as an oriented
    shift (the program validator requires the oriented stop before it)."""
    return kind if not rapid or kind == 'shift' else 'rapid'


@dataclass(frozen=True)
class GCodeDialect:
    """Declared word semantics, independent of any brand/profile name.

    Commands may be remapped or omitted. Unknown effects/syntax reject; there is
    no arbitrary "ignore this M code" escape. Subclass GCodeDecoder for a different
    language/model. A physical controller still needs manual/conformance evidence.
    """

    commands: tuple[Command, ...] = _DEFAULT_COMMANDS
    work_offsets: tuple[str, ...] = ('G54', 'G55', 'G56', 'G57', 'G58', 'G59')
    axes: tuple[str, ...] = ('X', 'Y', 'Z')
    centers: tuple[str, ...] = ('I', 'J', 'K')
    tool_word: str = 'T'
    length_word: str = 'H'
    feed_word: str = 'F'
    speed_word: str = 'S'
    parentheses_comments: bool = True
    # m6 preselection, direct_t, or lathe_t: one word T<nn><oo> selects tool
    # nn and applies its offset oo (FANUC/Centroid lathes, T0101).
    tool_change: str = 'm6'
    length_activation: str = 'explicit'  # explicit or tool_edge (per-tool D)
    # Accepted mismatch between an arc's start and end radius, as LinuxCNC
    # checks it (interp_arc.cc arc_data_ijk): |r1 - r2| <= radius_tolerance_mm,
    # or |r1 - r2| / max(r1, r2) <= radius_relative_tolerance with
    # |r1 - r2| <= 100 x radius_tolerance_mm. Defaults: LinuxCNC's documented
    # CENTER_ARC_RADIUS_TOLERANCE_MM (0.00127) and SPIRAL_RELATIVE_TOLERANCE
    # (0.001), so a stock LinuxCNC accepts every arc. Declare the machine's INI
    # value if it differs; 0 demands exactly consistent words.
    radius_tolerance_mm: float = 0.00127
    radius_relative_tolerance: float = 0.001
    fixed_modes: tuple[str, ...] = ()
    # Word carrying the thread lead in a tap_cycle block (Centroid rigid G84
    # takes Q on every line); None writes the synchronized feed F instead.
    tap_pitch_word: str | None = None
    # Word of the D register in G41/G42 blocks.
    compensation_word: str = 'D'
    # G41/G42 and G40 go in the block of the straight XY move that turns
    # compensation on or off (FANUC 0i §14.4 start-up/cancel, Haas), not on a
    # line of their own (LinuxCNC, Centroid).
    compensation_with_move: bool = False
    # Lathes: G41/G42 carry no register word (the T word's offset holds the
    # nose radius and tip direction).
    compensation_register: bool = True
    # Dwell time word (seconds), its resolution and its maximum.
    dwell_word: str = 'P'
    dwell_resolution_s: float = 0.001
    dwell_max_s: float = 1e5
    # Canned drilling cycles: unit of the G82 dwell word P ('s', or 'ms' as
    # an integer: FANUC), and the controller's fixed return amounts: G83
    # rapids back down to peck_reentry_mm above the last peck, G73 rapids up
    # chip_break_lift_mm (LinuxCNC: 0.010 in for both; FANUC parameters
    # 5115/5114; Centroid parameters 83/73).
    cycle_dwell_unit: str = 's'
    peck_reentry_mm: float | None = 0.254
    chip_break_lift_mm: float | None = 0.254
    # Where the boring bar's cutting edge points after an oriented spindle
    # stop (M19, or inside G76), in work XY degrees from +X: the machine's
    # orientation angle plus how the bar is set in the spindle. G76 shifts the
    # opposite way, which the controller's shift direction must agree with
    # (FANUC parameter 5101 RD1/RD2, Centroid parameter 136). A setup premise.
    oriented_edge_deg: float | None = None
    # Beam tools (plasma torch, laser head) use the spindle words: the beam
    # fires with beam_effect ('spindle_cw' M3, or 'spindle_ccw' M4: GRBL
    # dynamic laser power) and S = power x beam_s_max / 100, and M5 turns it
    # off. None: beam tools are refused.
    beam_s_max: float | None = None
    beam_effect: str = 'spindle_cw'
    # Lathes: X words are diameters (arc center I stays a radius, as FANUC
    # and Centroid lathes read it). False: X is the radius.
    x_diameter: bool = False
    thread_lead_word: str = 'K'
    height_control_word: str | None = None
    # The CSS rpm limit word in the G96 block (LinuxCNC: 'D', G96 D<max>
    # S<speed>); None when the dialect declares css_limit (G50 S<max> in its
    # own block) or no constant surface speed.
    css_limit_word: str | None = None

    def __post_init__(self):
        for key in ('commands', 'work_offsets', 'axes', 'centers', 'fixed_modes'):
            values = tuple(islice(iter(getattr(self, key)), 129))
            if len(values) > 128:
                raise ValueError('dialect definition cap')
            object.__setattr__(self, key, values)
        if len(self.axes) != 3 or len(self.centers) != 3 or not 1 <= len(self.work_offsets) <= 9:
            raise ValueError('three Cartesian axes and 1..9 work offsets required')
        letters = (*self.axes, *self.centers, self.tool_word, self.length_word, self.feed_word, self.speed_word)
        if len(set(letters)) != len(letters) or any(not re.fullmatch('[A-Z]', v) or v in 'GMN' for v in letters):
            raise ValueError('dialect word addresses must be distinct uppercase letters')
        if self.tap_pitch_word is not None and (
            not re.fullmatch('[A-Z]', self.tap_pitch_word) or self.tap_pitch_word in (*letters, 'G', 'M', 'N', 'R')
        ):
            raise ValueError('tap_pitch_word must be a free uppercase letter')
        if (
            not re.fullmatch('[A-Z]', self.dwell_word)
            or self.dwell_word in (*letters, 'G', 'M', 'N')
            or not (math.isfinite(self.dwell_resolution_s) and 0 < self.dwell_resolution_s <= 1)
            or not (math.isfinite(self.dwell_max_s) and self.dwell_resolution_s <= self.dwell_max_s <= 1e5)
        ):
            raise ValueError('invalid declared dwell word/resolution/maximum')
        if not re.fullmatch('[A-Z]', self.thread_lead_word) or self.thread_lead_word in (
            *self.axes,
            self.tool_word,
            self.length_word,
            self.speed_word,
            'G',
            'M',
            'N',
        ):
            # A center word (LinuxCNC G33 K) or the feed word (G32 F) is fine:
            # a threading block has neither an arc center nor a feed.
            raise ValueError(
                'thread_lead_word must be an uppercase letter that is not an axis, tool, length or speed word'
            )
        if self.cycle_dwell_unit not in ('s', 'ms'):
            raise ValueError("cycle_dwell_unit must be 's' or 'ms'")
        for value in (self.peck_reentry_mm, self.chip_break_lift_mm):
            if value is not None and not (math.isfinite(value) and 0 < value <= 100):
                raise ValueError('canned cycle return amounts must be positive millimetres')
        declared = {c.effect for c in self.commands}
        if self.beam_s_max is not None and not (math.isfinite(self.beam_s_max) and 0 < self.beam_s_max <= 1e6):
            raise ValueError('beam_s_max must be a positive S value')
        if self.beam_effect not in ('spindle_cw', 'spindle_ccw'):
            raise ValueError("beam_effect must be 'spindle_cw' or 'spindle_ccw'")
        if self.height_control_word is not None and not re.fullmatch(r'[A-Z][0-9]+', self.height_control_word):
            raise ValueError('height_control_word must be one address with an integer, e.g. P2')
        if self.oriented_edge_deg is not None and not (
            math.isfinite(self.oriented_edge_deg) and abs(self.oriented_edge_deg) <= 360
        ):
            raise ValueError('oriented_edge_deg must be finite, within +-360 degrees')
        if declared & {'spindle_orient', 'fine_bore_cycle'} and self.oriented_edge_deg is None:
            raise ValueError(
                'declare oriented_edge_deg (where the bar\'s edge points after the '
                'oriented stop) with spindle_orient/fine_bore_cycle'
            )
        if (
            'peck_cycle' in declared
            and self.peck_reentry_mm is None
            or 'chip_break_cycle' in declared
            and self.chip_break_lift_mm is None
        ):
            raise ValueError(
                'declare the controller return amount (peck_reentry_mm/chip_break_lift_mm) of each pecking cycle'
            )
        if self.tool_change not in ('m6', 'direct_t', 'lathe_t') or self.length_activation not in (
            'explicit',
            'tool_edge',
        ):
            raise ValueError('unsupported declared tool/length policy')
        if not math.isfinite(self.radius_tolerance_mm) or not 0 <= self.radius_tolerance_mm <= 0.005:
            raise ValueError('invalid declared circle radius tolerance')
        if not math.isfinite(self.radius_relative_tolerance) or not 0 <= self.radius_relative_tolerance <= 0.001:
            raise ValueError('invalid declared relative circle radius tolerance')
        if 'css_on' in declared:
            if ('css_limit' in declared) == (self.css_limit_word is not None):
                raise ValueError(
                    'declare the constant surface speed rpm limit one way: css_limit (G50 S, own block) '
                    'or css_limit_word (in the G96 block), not both'
                )
            if 'rpm_mode' not in declared or 'rpm_mode' in self.fixed_modes:
                raise ValueError('constant surface speed needs the rpm mode (G97) to cancel it')
        if self.css_limit_word is not None and (
            not re.fullmatch('[A-Z]', self.css_limit_word) or self.css_limit_word in (*letters, 'G', 'M', 'N')
        ):
            raise ValueError('css_limit_word must be a free uppercase letter')
        if 'feed_per_rev' in declared and ('feed_per_minute' not in declared or 'feed_per_minute' in self.fixed_modes):
            raise ValueError('feed per revolution needs the feed per minute mode (G94/G98) to return to')
        known = {v.effect for v in _DEFAULT_COMMANDS} | _OPTIONAL_EFFECTS | _CYCLE_EFFECTS | _LATHE_EFFECTS
        if not set(self.fixed_modes) <= _REQUIRED - {'units', 'plane', 'length', 'spindle_off', 'coolant_off'}:
            raise ValueError('only invariant modal semantics may be declared fixed')
        codes = []
        for command in (*self.commands, *(Command('work_offset', code) for code in self.work_offsets)):
            if command.effect not in known | {'work_offset'}:
                raise ValueError('unknown command effect')
            if not re.fullmatch(r'(?:[GM][0-9]+(?:\.[0-9]+)?|[A-Z_]+)', command.code) and not (
                command.effect == 'length_off' and command.code == self.length_word + '0'
            ):
                raise ValueError('command must be a numeric G/M code or standalone keyword')
            token = (command.code[0], float(command.code[1:])) if command.code[1:2].isdigit() else command.code
            if token in codes:
                raise ValueError('duplicate dialect command code')
            codes.append(token)
        if any(not re.fullmatch(r'G[0-9]+(?:\.[0-9]+)?', s) for s in self.work_offsets):
            raise ValueError('work offsets require numeric G codes')

    def compensation_word_free(self):
        return bool(re.fullmatch('[A-Z]', self.compensation_word)) and self.compensation_word not in (
            *self.axes,
            *self.centers,
            self.tool_word,
            self.length_word,
            self.feed_word,
            self.speed_word,
            'G',
            'M',
            'N',
        )

    @classmethod
    def linuxcnc(cls, **changes):
        """The LinuxCNC 2.9 dialect (RS274NGC words, G54-G59, M6/G43, arc
        tolerances from its documented defaults), with optional changes."""
        return replace(cls(), **changes)

    def radius_accepts(self, r1, r2):
        """Whether the controller accepts circular words whose start and end
        radii are r1 and r2 (see radius_tolerance_mm)."""
        error = abs(r1 - r2)
        return error <= self.radius_tolerance_mm or (
            error <= 100 * self.radius_tolerance_mm and error <= self.radius_relative_tolerance * max(r1, r2)
        )

    def code(self, effect):
        for command in self.commands:
            if command.effect == effect:
                return command.code
        raise ValueError(f'dialect does not declare {effect}')

    def with_commands(self, **codes):
        """Copy with selected semantic codes changed; None removes a command."""
        commands = [
            Command(c.effect, codes.get(c.effect, c.code))
            for c in self.commands
            if codes.get(c.effect, c.code) is not None
        ]
        commands.extend(
            Command(k, v) for k, v in codes.items() if k not in {c.effect for c in self.commands} and v is not None
        )
        return replace(self, commands=tuple(commands))


@dataclass(frozen=True)
class Machine:
    """A real machine, as its owner declares it. The engine assumes nothing:
    the controller dialect, output units and precision, feed/spindle/rapid
    limits, tool and register ranges, tool-change behaviour and capabilities
    (planes, arcs, helices, tool changes, coolant) are all required."""

    name: str
    tools: tuple[ToolSetting, ...]
    dialect: GCodeDialect = field(kw_only=True)
    work_offset: int = 1
    frame: WorkFrame = WorkFrame()
    work_to_machine: WorkFrame = WorkFrame()
    tip_travel_mm: tuple | None = None
    output_units: str = field(kw_only=True)
    decimals: int = field(kw_only=True)
    tolerance_mm: float = 0.01
    feed_tolerance_mm_min: float = 0.1
    traverse_feed_mm_min: float | None = None
    max_feed_mm_min: float = field(kw_only=True)
    max_spindle_rpm: float = field(kw_only=True)
    planes: tuple[str, ...] = field(kw_only=True)
    arcs: bool = field(kw_only=True)
    helices: bool = field(kw_only=True)
    max_sweep_rad: float = field(kw_only=True)
    max_tool_number: int = field(kw_only=True)
    max_length_register: int = field(kw_only=True)
    tool_changes: bool = field(kw_only=True)
    flood: bool = field(kw_only=True)
    mist: bool = field(kw_only=True)
    revision: str = '1'
    # Tip pose after a tool change: 'table' (each ToolSetting states it),
    # 'preserve_tip' (it stays where it was), or 'z_home' (M6 sends the
    # spindle gauge line to machine Z tool_change_home_z_mm, XY unchanged).
    tool_change_position_mode: str = field(kw_only=True)
    tool_change_home_z_mm: float | None = None
    # The machine's rapid traverse rate: nominal time of G0 moves in playback.
    rapid_mm_min: float = field(kw_only=True)
    # Nominal source/decoded curve limit, not a controller timing/blending model.
    max_normal_accel_mm_s2: float | None = None
    max_axis_feed_mm_min: tuple | None = None
    motion_policy: MotionPolicy | None = None
    schedule_post_feeds: bool = False

    def __post_init__(self):
        if self.schedule_post_feeds and self.motion_policy is None:
            raise ValueError('schedule_post_feeds requires an explicit motion_policy')
        if self.motion_policy is not None and not isinstance(self.motion_policy, MotionPolicy):
            raise TypeError('motion_policy must be MotionPolicy')
        if not isinstance(self.tools, (list, tuple)):
            raise TypeError('machine tools must be a list or tuple (a finite collection)')
        tools = tuple(replace(t, position_after_mm=tuple(t.position_after_mm)) for t in self.tools)
        object.__setattr__(self, 'tools', tools)
        for name in ('frame', 'work_to_machine'):
            value = getattr(self, name)
            object.__setattr__(
                self, name, WorkFrame(*(tuple(getattr(value, key)) for key in ('origin_mm', 'x', 'y', 'z')))
            )
        if not isinstance(self.planes, (list, tuple)):
            raise TypeError('machine planes must be a list or tuple')
        if len(set(self.planes)) != len(self.planes) or not set(self.planes) <= {'xy', 'xz', 'yz'}:
            raise ValueError("machine planes are distinct values from 'xy', 'xz', 'yz'")
        object.__setattr__(self, 'planes', tuple(self.planes))
        if not self.name or len(self.name) > 128 or not self.revision or len(self.revision) > 128:
            raise ValueError('machine needs bounded name/revision')
        if not tools or not isinstance(self.dialect, GCodeDialect):
            raise ValueError('machine needs at least one tool and a declared dialect')
        if self.tool_change_position_mode not in ('table', 'preserve_tip', 'z_home'):
            raise ValueError('tool_change_position_mode must be table, preserve_tip or z_home')
        if (self.tool_change_position_mode == 'z_home') != (self.tool_change_home_z_mm is not None) or (
            self.tool_change_home_z_mm is not None
            and not (math.isfinite(self.tool_change_home_z_mm) and abs(self.tool_change_home_z_mm) <= 1e6)
        ):
            raise ValueError(
                "z_home tool changes need tool_change_home_z_mm (the gauge line's machine Z after M6), and only they"
            )
        if self.tool_change_position_mode == 'z_home' and tuple(self.work_to_machine.z) != (0, 0, 1):
            raise ValueError('z_home tool changes need a work frame aligned with machine Z')
        if not math.isfinite(self.rapid_mm_min) or not 0 < self.rapid_mm_min <= 1e6:
            raise ValueError('rapid_mm_min must be the positive, finite rapid traverse rate')
        if self.output_units not in ('mm', 'inch') or not isinstance(self.decimals, int) or not 2 <= self.decimals <= 9:
            raise ValueError('invalid machine output units/precision')
        for name in ('tolerance_mm', 'feed_tolerance_mm_min'):
            value = getattr(self, name)
            if not math.isfinite(value) or not 1e-9 <= value <= 0.1:
                raise ValueError(f'{name} must be finite and in [1e-9, .1]')
        if self.traverse_feed_mm_min is not None and (
            not math.isfinite(self.traverse_feed_mm_min) or not 0 < self.traverse_feed_mm_min <= self.max_feed_mm_min
        ):
            raise ValueError('traverse_feed_mm_min must be positive, finite and within machine feed limits')
        if self.max_normal_accel_mm_s2 is not None and (
            isinstance(self.max_normal_accel_mm_s2, bool)
            or not math.isfinite(self.max_normal_accel_mm_s2)
            or not 1e-6 <= self.max_normal_accel_mm_s2 <= 1e9
        ):
            raise ValueError('max_normal_accel_mm_s2 must be finite and in [1e-6, 1e9]')
        if self.max_axis_feed_mm_min is not None:
            limits = tuple(islice(iter(self.max_axis_feed_mm_min), 4))
            if len(limits) != 3 or any(
                isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) or not 1e-6 <= v <= 1e7
                for v in limits
            ):
                raise ValueError('max_axis_feed_mm_min requires three finite positive axis limits in [1e-6, 1e7]')
            object.__setattr__(self, 'max_axis_feed_mm_min', limits)
        unit = 25.4 if self.output_units == 'inch' else 1
        if math.sqrt(3) * 0.5 * unit * 10**-self.decimals > self.tolerance_mm:
            raise ValueError('coordinate rounding exceeds tolerance_mm; increase decimals or tolerance')
        if not 1 <= self.work_offset <= len(self.dialect.work_offsets):
            raise ValueError('work offset absent from dialect')
        if len({t.tool for t in tools}) != len(tools):
            raise ValueError('machine tool identities must be unique')
        physical_keys = {
            (t.number, t.length_register) if self.dialect.length_activation == 'tool_edge' else t.number for t in tools
        }
        if len(physical_keys) != len(tools):
            raise ValueError('machine tool numbers (or tool/edge pairs) must be unique')
        if self.dialect.length_activation == 'explicit' and len({t.length_register for t in tools}) != len(tools):
            raise ValueError('global length registers must be unique (tool_edge registers are per tool)')
        for t in tools:
            if not 1 <= t.number <= self.max_tool_number or not 1 <= t.length_register <= self.max_length_register:
                raise ValueError('tool/length number exceeds machine limits')
        # Frozen dataclasses must not retain caller-owned lists in identity/state.
        if self.tip_travel_mm is not None:
            object.__setattr__(self, 'tip_travel_mm', tuple(tuple(p) for p in self.tip_travel_mm))

    def schedule(self, program, *, adjust_feeds=True, cancel=None):
        """Limit feeds and estimate motion time using the explicitly authored policy."""
        from .dynamics import schedule

        return schedule(program, self, adjust_feeds=adjust_feeds, cancel=cancel)

    def schedule_events(self, header, events, *, adjust_feeds=True, cancel=None):
        """Collect a native/typed event source for complete-program lookahead."""
        from .dynamics import schedule_events

        return schedule_events(header, events, self, adjust_feeds=adjust_feeds, cancel=cancel)

    def verification_limits(self):
        """Source nominal-curve limits; controller/junction dynamics stay unchecked."""
        from .verification import MachineLimits

        if self.tip_travel_mm is None:
            raise ValueError('verification_limits requires explicit tip_travel_mm')
        return MachineLimits(
            *self.tip_travel_mm, self.max_feed_mm_min, self.max_normal_accel_mm_s2, self.max_axis_feed_mm_min
        )

    def capabilities(self):
        return dict(
            id='python_machine',
            planes=self.planes,
            arcs=self.arcs,
            helices=self.helices,
            max_sweep_rad=self.max_sweep_rad,
            max_feed_mm_min=self.max_feed_mm_min,
            max_spindle_rpm=self.max_spindle_rpm,
            max_tool_number=self.max_tool_number,
            max_length_register=self.max_length_register,
            max_work_offset=len(self.dialect.work_offsets),
            tool_changes=self.tool_changes,
            flood=self.flood,
            mist=self.mist,
        )


def _typed(data):
    if data['kind'] == 'motion':
        m = dict(data['motion'])
        c = dict(m.pop('curve'))
        kind = c.pop('kind')
        for k in ('start', 'end', 'center'):
            if k in c:
                c[k] = tuple(c[k])
        return Motion(curve={'line': Line, 'arc': Arc, 'helix': Helix}[kind](**c), **m)
    value = dict(data)
    kind = value.pop('kind')
    if 'position_after_mm' in value:
        value['position_after_mm'] = tuple(value['position_after_mm'])
    return {
        'tool_change': ToolChange,
        'work_offset': WorkOffset,
        'spindle': Spindle,
        'coolant': Coolant,
        'stop': Stop,
        'dwell': Dwell,
        'cycle': Cycle,
        'cutter_compensation': CutterCompensation,
        'spindle_orient': SpindleOrient,
        'beam': Beam,
        'height_control': HeightControl,
        'surface_speed': SurfaceSpeed,
        'feed_per_rev': FeedPerRev,
    }[kind](**value)


_ATOMS = (int, float, complex, str, bytes, bool, type(None))


def _immutable(value):
    t = type(value)
    if t in _ATOMS:
        return True
    if t is tuple or t is frozenset:
        return all(_immutable(v) for v in value)
    return False


class _TransactionCopy:
    """Deep copies for transactional batches (the post and decoder are copied
    before every checked batch so a refused batch changes nothing). Values
    that cannot change (atoms, and tuples and frozensets of them) are
    shared; every other value is deep-copied, as copy.deepcopy would. A
    subclass with its own pickling state (__getstate__/__setstate__,
    __reduce__, __slots__) is copied by copy.deepcopy's own rules."""

    def __deepcopy__(self, memo):
        import copy

        cls = type(self)
        if (
            cls.__reduce_ex__ is not object.__reduce_ex__
            or cls.__reduce__ is not object.__reduce__
            or cls.__getstate__ is not object.__getstate__
            or hasattr(cls, '__setstate__')
            or any('__slots__' in vars(k) for k in cls.__mro__)
        ):
            return copy._reconstruct(self, memo, *self.__reduce_ex__(4))
        new = cls.__new__(cls)
        memo[id(self)] = new
        state = new.__dict__
        for key, value in self.__dict__.items():
            state[key] = value if _immutable(value) else deepcopy(value, memo)
        return new


def _machine_memo(machine):
    """The machine and the immutable definitions it owns (dialect, tool
    table), shared by every copy of a post or decoder bound to it."""
    memo = {id(machine): machine, id(machine.dialect): machine.dialect}
    for tool in machine.tools:
        memo[id(tool)] = tool
    return memo


class PostProcessor(_TransactionCopy):
    """Subclass in any normal .py file. Hooks emit blocks or return an iterable.

    Mutable hook state must belong to this instance and be deepcopy-compatible.
    External side effects/global variables are not transactional or sandboxed.
    """

    version = '1'

    def __init__(self, machine):
        self.machine = machine
        self._lines = []
        self._limit = 8
        self._modal = {}

    def emit(self, *words):
        line = ' '.join(str(w) for w in words if w is not None and w != '')
        if len(self._lines) >= self._limit or len(line) > 1024 or _INVALID_BLOCK.search(line):
            raise ValueError('post hook output exceeds block/ASCII/1024byte limit')
        self._lines.append(line)

    def number(self, value, *, dimensional=True):
        if not math.isfinite(value) or abs(value) > 1e6:
            raise ValueError('invalid post number')
        value = value / (25.4 if dimensional and self.machine.output_units == 'inch' else 1)
        result = f'{value:.{self.machine.decimals}f}'.rstrip('0').rstrip('.')
        return '0' if result in ('', '-0') else result

    def word(self, address, value, *, dimensional=True):
        return address + self.number(value, dimensional=dimensional)

    def words(self, addresses, values, *, dimensional=True):
        """Batch the standard numeric helper; ordinary number/word overrides win."""
        d = self.machine.dialect
        if d.x_diameter and dimensional and d.axes[0] in addresses:
            values = [2 * v if a == d.axes[0] else v for a, v in zip(addresses, values)]
        if (
            getattr(self.number, '__func__', None) is PostProcessor.number
            and getattr(self.word, '__func__', None) is PostProcessor.word
        ):
            unit = 25.4 if dimensional and self.machine.output_units == 'inch' else 1
            return native.format_post_words(''.join(addresses), values, unit, self.machine.decimals)
        return [self.word(a, v, dimensional=dimensional) for a, v in zip(addresses, values)]

    def modal(self, group, word):
        """Return a word only when changed. Decoder independently checks omissions."""
        if self._modal.get(group) == word:
            return None
        self._modal[group] = word
        return word

    def comment(self, text):
        self.emit('; ' + text)

    def on_begin(self, program):
        raise NotImplementedError

    def on_work_offset(self, event):
        raise NotImplementedError

    def on_tool_change(self, event):
        raise NotImplementedError

    def on_spindle(self, event):
        raise NotImplementedError

    def on_coolant(self, event):
        raise NotImplementedError

    def on_stop(self, event):
        raise NotImplementedError

    def on_dwell(self, event):
        raise NotImplementedError

    def on_cycle(self, event):
        """Write nothing: the hole's moves follow as explicit blocks."""
        return []

    def on_cutter_compensation(self, event):
        raise NotImplementedError

    def on_spindle_orient(self, event):
        raise NotImplementedError

    def on_beam(self, event):
        raise NotImplementedError

    def on_height_control(self, event):
        raise NotImplementedError

    def on_surface_speed(self, event):
        raise NotImplementedError

    def on_feed_per_rev(self, event):
        raise NotImplementedError

    def on_motion(self, motion):
        raise NotImplementedError

    def on_finish(self):
        raise NotImplementedError

    def _call(self, name, *args, limit=8):
        self._lines = []
        self._limit = limit
        covered = getattr(self, '_covered', 0)
        if covered:
            # A written canned cycle carries the moves and dwell of its hole.
            if name not in ('on_motion', 'on_dwell', 'on_spindle', 'on_spindle_orient'):
                raise ValueError('a canned cycle covers only the moves, dwell and spindle stop/restart of its hole')
            edge = self.machine.dialect.oriented_edge_deg
            if name == 'on_spindle_orient' and abs((args[0].edge_angle_deg - edge + 180) % 360 - 180) > 1e-9:
                raise ValueError(
                    f'the operation\'s fine-bore edge_angle_deg {args[0].edge_angle_deg} differs '
                    f'from this machine\'s oriented_edge_deg {edge}'
                )
            self._covered = covered - 1
            return []
        if getattr(self, '_cycle_open', False) and name != 'on_cycle':
            self.emit(self.machine.dialect.code('cycles_off'))
            self._cycle_open = False
        if name == 'on_tool_change':
            # The active tool's table entry (its D register), for any post.
            e = args[0]
            self._tool_setting = next(
                (
                    t
                    for t in self.machine.tools
                    if t.tool == e.tool and t.number == e.number and t.length_register == e.length_register
                ),
                None,
            )
        returned = getattr(self, name)(*args)
        if returned is not None:
            if isinstance(returned, str):
                self.emit(returned)
            else:
                for line in islice(iter(returned), limit + 1):
                    self.emit(line)
        if (
            not self._lines
            and name != 'on_cycle'
            and not (name == 'on_cutter_compensation' and self.machine.dialect.compensation_with_move)
        ):
            raise ValueError(f'{name} emitted no blocks')
        return self._lines[:]


class GCodePost(PostProcessor):
    """Default formatter using the machine's declared dialect; normal Python."""

    version = '2'

    def on_begin(self, program):
        self.geometry_tolerance_mm = program.tolerance_mm
        d = self.machine.dialect
        for optional in ('language_native', 'compression_off'):
            if any(c.effect == optional for c in d.commands):
                self.emit(d.code(optional))
        mode = 'lathe_diameter_mode' if d.x_diameter else 'lathe_radius_mode'
        if any(c.effect == mode for c in d.commands):
            self.emit(d.code(mode))
        # The machine's first plane (G18 on a lathe).
        plane = 'plane_xy' if 'xy' in self.machine.planes else 'plane_' + self.machine.planes[0]
        for effect in (
            'units_mm' if self.machine.output_units == 'mm' else 'units_inch',
            'absolute',
            'center_incremental',
            plane,
            'feed_per_minute',
            'rpm_mode',
            'radius_off',
            'length_off',
            'cycles_off',
            'frame_reset',
            'exact_stop',
            'spindle_off',
            'coolant_off',
        ):
            if effect not in d.fixed_modes:
                self.emit(d.code(effect))
        # Explicitly stopped above, or the dialect declares it the controller's
        # startup mode. Tracked so later output omits only provably redundant codes.
        self._spindle_on = False
        self._coolant_mode = 'off'

    def on_work_offset(self, event):
        self.emit(self.machine.dialect.work_offsets[event.number - 1])

    def on_tool_change(self, event):
        d = self.machine.dialect
        if d.tool_change == 'lathe_t':
            if not (1 <= event.number <= 99 and 1 <= event.length_register <= 99):
                raise ValueError('T<nn><oo> needs a tool and an offset number of 1..99')
            self.emit(f'{d.tool_word}{event.number:02d}{event.length_register:02d}')
            return
        self.emit(d.tool_word + str(event.number))
        if d.tool_change == 'm6':
            self.emit(d.code('tool_change'))
        if d.length_activation == 'explicit':
            self.emit(d.code('length_on'), d.length_word + str(event.length_register))
        else:
            self.emit(d.length_word + str(event.length_register))

    def on_spindle(self, event):
        d = self.machine.dialect
        if event.rpm != 0 and getattr(self, '_css', False):
            # Out of constant surface speed first: S is an rpm again.
            self.emit(d.code('rpm_mode'))
            self._css = False
        self.emit(
            d.code('spindle_off') if event.rpm == 0 else d.code('spindle_' + event.direction),
            None if event.rpm == 0 else self.word(d.speed_word, event.rpm, dimensional=False),
        )
        self._spindle_on = event.rpm != 0
        self._rpm = event.rpm

    def on_coolant(self, event):
        # As commercial posts do: M9 to stop, or before switching between two
        # different active modes; a mode is switched on directly from off.
        d = self.machine.dialect
        current = getattr(self, '_coolant_mode', None)
        if event.mode == 'off' or current not in ('off', event.mode):
            self.emit(d.code('coolant_off'))
        if event.mode != 'off':
            self.emit(d.code(event.mode))
        self._coolant_mode = event.mode

    def on_stop(self, event):
        self.emit(self.machine.dialect.code('optional_stop' if event.optional else 'stop'))

    def on_cutter_compensation(self, event):
        """G41/G42 with the active tool's D register (its wear), or G40."""
        d = self.machine.dialect
        if event.side == 'off':
            if d.compensation_with_move:
                self._pending_compensation = (d.code('radius_off'),)
            else:
                self.emit(d.code('radius_off'))
            return
        code = d.code('radius_left' if event.side == 'left' else 'radius_right')
        if not d.compensation_register:
            words = (code,)
        else:
            if not d.compensation_word_free():
                raise ValueError(
                    f'this dialect uses {d.compensation_word} for another word: '
                    'it cannot write cutter compensation registers'
                )
            tool = getattr(self, '_tool_setting', None)
            if tool is None or tool.diameter_register is None:
                raise ValueError(
                    'cutter compensation needs the tool\'s D register: declare '
                    'ToolSetting(diameter_register=...) holding its wear'
                )
            words = (code, d.compensation_word + str(tool.diameter_register))
        # With the move: written by on_motion in the next (straight XY) block.
        if d.compensation_with_move:
            self._pending_compensation = words
        else:
            self.emit(*words)

    def on_beam(self, event):
        """Beam on with its power as S (the dialect's beam_s_max at 100%), or
        off (M5)."""
        d = self.machine.dialect
        if d.beam_s_max is None:
            raise ValueError('this dialect declares no beam (beam_s_max): it cannot post plasma or laser cutting')
        if event.power_percent == 0:
            self.emit(d.code('spindle_off'))
            self._spindle_on = False
            return
        self.emit(
            d.code(d.beam_effect), self.word(d.speed_word, event.power_percent * d.beam_s_max / 100, dimensional=False)
        )
        self._spindle_on = True

    def on_height_control(self, event):
        d = self.machine.dialect
        effect = 'height_control_on' if event.on else 'height_control_off'
        if not any(c.effect == effect for c in d.commands):
            raise ValueError('this dialect declares no torch height control (height_control_on/off)')
        self.emit(d.code(effect), *((d.height_control_word,) if d.height_control_word else ()))

    def on_surface_speed(self, event):
        """G96 S<surface speed> with M3/M4 (m/min; ft/min in inch output), its
        rpm limit as G50 S in the block before (css_limit) or the
        css_limit_word in the G96 block (LinuxCNC G96 D)."""
        d = self.machine.dialect
        commands = {c.effect for c in d.commands}
        if 'css_on' not in commands:
            raise ValueError('this dialect declares no constant surface speed (css_on, e.g. G96)')
        speed = event.m_min / (0.3048 if self.machine.output_units == 'inch' else 1)
        limit = self.word(d.css_limit_word or d.speed_word, event.max_rpm, dimensional=False)
        if 'css_limit' in commands:
            self.emit(d.code('css_limit'), limit)
        self.emit(
            d.code('css_on'),
            *(() if 'css_limit' in commands else (limit,)),
            self.word(d.speed_word, speed, dimensional=False),
            d.code('spindle_' + event.direction),
        )
        self._spindle_on = True
        self._css = True
        self._rpm = event.max_rpm

    def on_feed_per_rev(self, event):
        """G95 F<feed per revolution>, or G94 back to feeds per minute. The
        written F must keep the feed within the machine's feed tolerance at
        its top spindle speed."""
        d = self.machine.dialect
        m = self.machine
        if event.mm_rev == 0:
            self.emit(d.code('feed_per_minute'))
            self._feed_rev = None
            return
        if not any(c.effect == 'feed_per_rev' for c in d.commands):
            raise ValueError('this dialect declares no feed per revolution (feed_per_rev, e.g. G95)')
        if m.schedule_post_feeds:
            raise ValueError('feeds per revolution are not rescheduled per minute: post without schedule_post_feeds')
        word = self.word(d.feed_word, event.mm_rev)
        written = float(word[len(d.feed_word) :]) * (25.4 if m.output_units == 'inch' else 1)
        if abs(written - event.mm_rev) > m.feed_tolerance_mm_min / max(m.max_spindle_rpm, 1):
            raise ValueError(
                f'feed per revolution {event.mm_rev} mm is written {word}: not within the feed '
                f'tolerance at {m.max_spindle_rpm} rpm; use a feed representable at '
                f'{m.decimals} decimals ({m.output_units})'
            )
        self.emit(d.code('feed_per_rev'), word)
        self._feed_rev = event.mm_rev

    def on_spindle_orient(self, event):
        """M19: the machine's oriented stop must leave the edge where the
        operation planned its shifts."""
        d = self.machine.dialect
        if not any(c.effect == 'spindle_orient' for c in d.commands) or d.oriented_edge_deg is None:
            raise ValueError(
                'this dialect declares no oriented spindle stop (spindle_orient, '
                'oriented_edge_deg): fine boring cannot be posted'
            )
        if abs((event.edge_angle_deg - d.oriented_edge_deg + 180) % 360 - 180) > 1e-9:
            raise ValueError(
                f'the operation\'s fine-bore edge_angle_deg {event.edge_angle_deg} differs from '
                f'this machine\'s oriented_edge_deg {d.oriented_edge_deg}'
            )
        self.emit(d.code('spindle_orient'))
        self._spindle_on = False

    def on_dwell(self, event):
        # Rounded up to the controller's dwell resolution: never shorter.
        d = self.machine.dialect
        step = d.dwell_resolution_s
        ticks = math.ceil(event.seconds / step - 1e-9)
        if not 1 <= ticks or ticks * step > d.dwell_max_s + 1e-12:
            raise ValueError(f'dwell {event.seconds}s outside the controller range {step}..{d.dwell_max_s}s')
        digits = max(0, -math.floor(math.log10(step) + 1e-9))
        # Always with a decimal point: no control reads it as milliseconds.
        self.emit(d.code('dwell'), d.dwell_word + f'{ticks * step:.{digits}f}'.rstrip('0'))

    def on_cycle(self, event):
        """One cycle block per hole (G98 once), or nothing when the dialect
        declares no code for this cycle: the moves then follow explicitly."""
        d = self.machine.dialect
        effect = _DRILL_CYCLES[event.cycle]
        if getattr(self, '_feed_rev', None):
            raise ValueError('canned cycles are written with feeds per minute: return to G94 first')
        # A cycle needs its own code, G98 return and G80 cancel.
        if not {effect, 'cycle_return_initial', 'cycles_off'} <= {c.effect for c in d.commands}:
            return []
        for kind, source, machine, name in (
            ('peck', event.reentry_mm, d.peck_reentry_mm, 'reentry_clearance_mm'),
            ('chip_break', event.lift_mm, d.chip_break_lift_mm, 'chip_break_lift_mm'),
        ):
            if event.cycle == kind and abs(source - machine) > 1e-9:
                raise ValueError(
                    f'the operation {name} {source} mm differs from this controller\'s '
                    f'{d.code(effect)} return amount {machine} mm: set {name}={machine}'
                )
        if getattr(self, '_cycle_open', False) and getattr(self, '_cycle_initial', None) != event.initial_z_mm:
            # FANUC keeps the initial level from where the cycle mode began.
            self.emit(d.code('cycles_off'))
        self._cycle_initial = event.initial_z_mm
        if word := self.modal('cycle_return', d.code('cycle_return_initial')):
            self.emit(word)
        addresses = [d.axes[0], d.axes[1], d.axes[2], 'R']
        values = [*event.center_mm, event.z_mm, event.r_mm]
        words = self.words(addresses, values)
        for q in (event.peck_mm, event.shift_mm):
            if q is not None:
                q = self.word('Q', q)
                # Always with a decimal point: no control reads it as an increment count.
                words.append(q if '.' in q else q + '.')
        if event.dwell_s is not None:
            words.append(self.cycle_dwell(event.dwell_s))
        words.extend(self.words((d.feed_word,), (event.feed_mm_min,)))
        self.emit(d.code(effect), *words)
        self._cycle_open = True
        self._covered = event.covers

    def cycle_dwell(self, seconds):
        """The G82 P word, rounded up to the controller resolution."""
        d = self.machine.dialect
        if d.cycle_dwell_unit == 'ms':
            ms = math.ceil(seconds * 1000 - 1e-9)
            if not 1 <= ms <= d.dwell_max_s * 1000:
                raise ValueError(f'dwell {seconds}s outside the controller range')
            return f'P{ms}'
        step = d.dwell_resolution_s
        ticks = math.ceil(seconds / step - 1e-9)
        if not 1 <= ticks or ticks * step > d.dwell_max_s + 1e-12:
            raise ValueError(f'dwell {seconds}s outside the controller range {step}..{d.dwell_max_s}s')
        digits = max(0, -math.floor(math.log10(step) + 1e-9))
        return 'P' + f'{ticks * step:.{digits}f}'.rstrip('0')

    def on_motion(self, motion):
        d = self.machine.dialect
        c = motion.curve
        if motion.kind == 'tap':
            return self.tap(motion)
        if motion.kind == 'thread':
            # The lead (feed per spindle revolution), not a feed.
            if not any(x.effect == 'thread_synchronized' for x in d.commands):
                raise ValueError('this dialect declares no spindle-synchronized threading (thread_synchronized)')
            rpm = getattr(self, '_rpm', 0)
            if not rpm or isinstance(c, Arc):
                raise ValueError('threading needs a running spindle and a straight move')
            addresses = [d.axes[0], d.axes[2]]
            values = [c.end[0], c.end[2]]
            self.emit(
                d.code('thread_synchronized'),
                *self.words(addresses, values),
                *self.words((d.thread_lead_word,), (motion.feed_mm_min / rpm,)),
            )
            return
        if isinstance(c, Arc):
            self.emit(d.code('plane_' + c.plane))
            effect = 'arc_ccw' if c.sweep_rad > 0 else 'arc_cw'
        else:
            effect = 'rapid' if motion.kind in ('rapid', 'shift') and motion.feed_mm_min is None else 'linear'
        axes = (0, 1, 2)
        if isinstance(c, Arc):
            u, v, ax = {'xy': (0, 1, 2), 'xz': (2, 0, 1), 'yz': (1, 2, 0)}[c.plane]
            axes = (0, 1, 2) if isinstance(c, Helix) else tuple(i for i in axes if i != ax)
        addresses = [d.axes[i] for i in axes]
        values = [c.end[i] for i in axes]
        position = getattr(self, 'position_mm', None)
        # Vertical in the plan: this motion, or the source it was lowered from
        # (a lowered segment starts at the actual decoded position).
        source = getattr(getattr(self, 'source_event', None), 'curve', None)
        vertical = (c.start[0] == c.end[0] and c.start[1] == c.end[1]) or (
            isinstance(source, Line) and source.start[0] == source.end[0] and source.start[1] == source.end[1]
        )
        if (
            not isinstance(c, Arc)
            and position is not None
            and vertical
            and all(abs(position[k] - c.end[k]) <= self.machine.tolerance_mm / 2 for k in (0, 1))
        ):
            # A straight vertical move keeps the controller's current XY (an
            # arc's representable end may differ by a word): writing the source
            # XY would add a sideways step to a move leaving the surface it
            # touches. The decoded block is still checked against the source.
            values = [position[i] if i < 2 else c.end[i] for i in axes]
        if isinstance(c, Arc):
            # Actual decoded start accounts for earlier rounding.
            addresses.extend(d.centers[i] for i in (u, v))
            end, offsets = self.circular_values(c)
            values = [end[i] for i in axes] + [offsets[i] for i in (u, v)]
        if motion.feed_mm_min is not None:
            per_rev = getattr(self, '_feed_rev', None)
            if per_rev and motion.kind in ('rapid', 'shift'):
                raise ValueError('a programmed traverse feed cannot be written in feed per revolution mode')
            addresses.append(d.feed_word)
            values.append(per_rev or motion.feed_mm_min)
        carried = getattr(self, '_pending_compensation', None) or ()
        if carried and (isinstance(c, Arc) or effect not in ('rapid', 'linear')):
            raise ValueError('cutter compensation turns on/off over a straight G0/G1 move')
        self._pending_compensation = None
        self.emit(d.code(effect), *carried, *self.words(addresses, values))

    def tap(self, motion):
        """A tapping pair: the downward motion is the whole cycle; the return
        is part of it (G33.1 returns by itself; G84 needs G80 afterwards)."""
        d = self.machine.dialect
        c = motion.curve
        commands = {x.effect for x in d.commands}
        if getattr(self, '_feed_rev', None) or getattr(self, '_css', False):
            raise ValueError('tapping is written at a fixed rpm with feeds per minute (G97, G94)')
        if c.end[2] > c.start[2]:
            if 'tap_cycle' in commands:
                self.emit(d.code('cycles_off'))
            elif d.parentheses_comments:
                self.comment('G33.1 returns to the start')
            else:
                raise ValueError('tapping return needs a block: declare tap_cycle (G84/G80) or comments')
            return
        rpm = getattr(self, '_rpm', 0)
        if not rpm:
            raise ValueError('tapping needs the spindle running')
        scale = 25.4 if self.machine.output_units == 'inch' else 1
        if 'tap_synchronized' in commands:
            # Pitch per spindle revolution, written exactly enough that the
            # decoded feed (pitch x rpm) matches the programmed feed.
            pitch = motion.feed_mm_min / rpm / scale
            self.emit(
                d.code('tap_synchronized'),
                self.word(d.axes[2], c.end[2]),
                'K' + (f'{pitch:.6f}'.rstrip('0').rstrip('.')),
            )
        elif 'tap_cycle' in commands and d.tap_pitch_word is not None:
            # Lead per spindle revolution, always with a decimal point so no
            # control reinterprets it as implied decimals.
            lead = f'{motion.feed_mm_min / rpm / scale:.6f}'.rstrip('0')
            self.emit(
                d.code('tap_cycle'),
                *self.words((d.axes[0], d.axes[1], d.axes[2], 'R'), (c.end[0], c.end[1], c.end[2], c.start[2])),
                d.tap_pitch_word + lead,
            )
        elif 'tap_cycle' in commands:
            self.emit(
                d.code('tap_cycle'),
                *self.words(
                    (d.axes[0], d.axes[1], d.axes[2], 'R', d.feed_word),
                    (c.end[0], c.end[1], c.end[2], c.start[2], motion.feed_mm_min),
                ),
            )
        else:
            raise ValueError('this machine dialect declares no tapping cycle (tap_synchronized or tap_cycle)')

    def circular_values(self, curve):
        """Return representable endpoint and incremental IJK values in mm.

        Custom circular block layouts can use this with ``words``. Selection
        keeps the configured precision and complete-curve error budget. The
        independent decoder still reads actual emitted words. Overridden numeric
        formatters retain control and remain subject to those same checks.
        """
        if (
            getattr(self.number, '__func__', None) is PostProcessor.number
            and getattr(self.word, '__func__', None) is PostProcessor.word
        ):
            try:
                end, offsets = native.round_circular_words(
                    _json(curve.to_dict()),
                    tuple(self.position_mm),
                    25.4 if self.machine.output_units == 'inch' else 1,
                    self.machine.decimals,
                    self.geometry_tolerance_mm,
                    self.machine.tolerance_mm,
                    self.machine.dialect.radius_tolerance_mm,
                    self.machine.dialect.radius_relative_tolerance,
                )
            except ValueError as error:
                raise ValueError(
                    f'{error}. Arc {curve.start} -> {curve.end} cannot be written as G2/G3 at '
                    f'{self.machine.decimals} decimals ({self.machine.output_units}). Use '
                    f'lowering=LoweringOptions(native_arcs=True) to emit checked line segments for '
                    f'only such arcs, or Machine(decimals={self.machine.decimals + 1}) if the controller '
                    f'accepts that precision.'
                ) from error
            return tuple(end), tuple(offsets)
        return curve.end, tuple(c - s for c, s in zip(curve.center, self.position_mm))

    def on_finish(self):
        d = self.machine.dialect
        for effect in ('spindle_off', 'coolant_off', 'length_off', 'end'):
            # Omit a stop only when this post knows the state is already off.
            if effect == 'spindle_off' and getattr(self, '_spindle_on', True) is False:
                continue
            if effect == 'coolant_off' and getattr(self, '_coolant_mode', None) == 'off':
                continue
            self.emit(d.code(effect))


class GCodeDecoder(_TransactionCopy):
    """Actual-output parser, independent of formatter helpers/source geometry.

    The supplied dialect defines semantics. Only attribution (ID/operation/kind)
    is read from context; coordinates/feed/tool/offset/spindle come from NC words.
    Unknown words/modal effects fail instead of becoming unverifiable no-ops.
    """

    version = 'configured_gcode_v2'

    def __init__(self, machine, program):
        self.machine = machine
        self.position = tuple(program.initial_position_mm)
        self.units = None
        self.plane = None
        self.ready = set(machine.dialect.fixed_modes)
        self.active = None
        self.selected = None
        self.length = None
        self.tool_pending = False
        self.offset = None
        self.rpm = 0
        self.coolant = 'off'
        self.ended = False
        self.direction = 'cw'
        self.tap_return = None
        self.cycle_active = False
        # Lathe modes: constant surface speed (m/min, rpm limit), its G50
        # limit, and the feed per revolution in effect.
        self.css = None
        self.css_max = None
        self.feed_rev = None
        # A decoded drilling cycle's moves (end, feed, rapid) and dwell
        # ('dwell', seconds), handed out one per covered source event.
        self.cycle_queue = []
        self.cycle_initial = None
        self.compensation = 'off'

        # Beam tools: the spindle words fire and stop their beam.
        def shape(t):
            c = t['cutter'] if isinstance(t, dict) else t.cutter
            s = c['shape'] if isinstance(c, dict) else dict(c.shape)
            return s.get('kind')

        self.beams = frozenset((t['id'] if isinstance(t, dict) else t.id) for t in program.tools if shape(t) == 'beam')
        rules = {}
        for c in machine.dialect.commands:
            key = (c.code[0], float(c.code[1:])) if c.code[1:2].isdigit() else c.code
            rules[key] = c.effect
        for i, code in enumerate(machine.dialect.work_offsets):
            rules[(code[0], float(code[1:]))] = ('work_offset', i + 1)
        self.rules = MappingProxyType(rules)
        self._compiled_rules = self.rules
        self._required = _REQUIRED | {c.effect for c in machine.dialect.commands if c.effect in _OPTIONAL_EFFECTS}

    def validate_initialized(self):
        required = self._required
        if not required <= self.ready:
            raise ValueError('incomplete explicit decoder modal initialization: ' + str(sorted(required - self.ready)))

    def read(self, lines, context=None):
        if not lines and self.cycle_queue:
            return [self._cycle_effect(context)]
        if self.cycle_queue:
            raise ValueError('canned cycle moves still pending')
        result = []
        pending = self.tap_return
        for line in lines:
            result.extend(self.read_line(line, context))
        if pending is not None:
            # The block after a tapping cycle is its return: G33.1 returns by
            # itself; a G84 cycle returns to R and must then be cancelled (G80).
            if context is None or context.get('kind') != 'tap' or result or self.cycle_active:
                raise ValueError('a tapping cycle must be followed by its return (and G80 for a canned cycle)')
            start, bottom, feed = pending
            result.append(
                Motion(context['id'], context['operation'], self.active.tool, Line(bottom, start), feed, 'tap')
            )
            self.position = start
            self.tap_return = None
        return result

    def read_line(self, line, context=None):
        if self.ended:
            raise ValueError('NC after program end')
        if len(line) > 1024 or _INVALID_BLOCK.search(line):
            raise ValueError('invalid NC physical block')
        body = line.split(';', 1)[0].strip().upper()
        d = self.machine.dialect
        if '(' in body or ')' in body:
            if not d.parentheses_comments:
                raise ValueError('parentheses are not comments in this dialect')
            body = re.sub(r'\([^()]*\)', ' ', body)
            if '(' in body or ')' in body:
                raise ValueError('nested/unclosed NC comment')
        if not body.strip():
            return []
        if body.strip() in self.rules and isinstance(self.rules[body.strip()], str):
            effects = [self.rules[body.strip()]]
            values = {}
        else:
            effects = []
            values = {}
            tokens = _WORD.findall(body)
            if _WORD.sub('', body).strip():
                raise ValueError('unsupported NC syntax')
            for word, number in tokens:
                value = float(number)
                if not math.isfinite(value) or abs(value) > 1e9:
                    raise ValueError('invalid NC value')
                if word in 'GM' or (word, value) in self.rules:
                    try:
                        effects.append(self.rules[(word, value)])
                    except KeyError:
                        raise ValueError(f'undeclared controller command {word}{number}') from None
                elif word == 'N':
                    if value < 0 or value != int(value):
                        raise ValueError('invalid line number')
                else:
                    if word in values:
                        raise ValueError('duplicate NC word')
                    values[word] = value
        if self.tap_return is not None and (len(effects) != 1 or effects[0] != 'cycles_off'):
            raise ValueError('a tapping cycle must be followed by its return (and G80 for a canned cycle)')
        if len(effects) == 1 and effects[0] in ('rapid', 'linear', 'arc_cw', 'arc_ccw'):
            if self.cycle_active:
                raise ValueError('canned cycle still active: cancel it (G80) before other motion')
            motion = self._motion(effects[0], values, context)
            return [] if motion is None else [motion]
        radius = {'radius_left', 'radius_right', 'radius_off'}
        if (
            d.compensation_with_move
            and len(effects) == 2
            and effects[0] in ('rapid', 'linear')
            and effects[1] in radius
        ):
            # Start-up/cancel in the move block: the change, then the move.
            if self.cycle_active:
                raise ValueError('canned cycle still active: cancel it (G80) before other motion')
            change = self._compensation(effects[1], values)
            motion = self._motion(
                effects[0],
                {k: v for k, v in values.items() if k != d.compensation_word or effects[1] == 'radius_off'},
                context,
            )
            if motion is None or motion.curve.start == motion.curve.end:
                raise ValueError('cutter compensation needs a move in its block')
            return [*change, motion]
        if len(effects) == 1 and effects[0] in ('tap_synchronized', 'tap_cycle'):
            return [self._tap(effects[0], values, context)]
        if len(effects) == 1 and effects[0] == 'thread_synchronized':
            return [self._thread(values, context)]
        if len(effects) == 1 and effects[0] in _DRILL_CYCLES.values():
            return self._drill_cycle(effects[0], values, context)
        if self.cycle_active and effects != ['cycles_off']:
            raise ValueError('canned cycle still active: cancel it (G80) first')
        if 'css_on' in effects:
            return self._surface_speed(effects, values)
        if effects == ['css_limit']:
            s = values.get(d.speed_word)
            if set(values) != {d.speed_word} or not 0 < s <= self.machine.max_spindle_rpm:
                raise ValueError('the CSS limit block sets S (rpm) alone, within the machine maximum')
            self.css_max = s
            return []
        if effects == ['feed_per_rev']:
            f = values.get(d.feed_word)
            if set(values) != {d.feed_word} or not f > 0:
                raise ValueError('feed per revolution needs its F word alone')
            self.feed_rev = f * self.units
            return [FeedPerRev(self.feed_rev)]
        groups = [_GROUPS.get(e, e) if isinstance(e, str) else 'work' for e in effects]
        if len(groups) != len(set(groups)):
            raise ValueError('conflicting modal commands in one block')
        # Restrict semantic event order deliberately: independent state setters may
        # share a block; tool/spindle/coolant/motion/end transitions remain separate.
        state_only = _STATE_ONLY
        transitions = [e for e in effects if not isinstance(e, str) or e not in state_only]
        if len(transitions) > 1 or transitions and len(effects) > 1:
            raise ValueError('separate controller state transitions into individual blocks')
        output = []
        used = set()
        for effect in effects:
            if effect in ('units_mm', 'units_inch'):
                self.units = 1 if effect == 'units_mm' else 25.4
                self.ready.add('units')
            elif effect in ('plane_xy', 'plane_xz', 'plane_yz'):
                self.plane = effect[6:]
                if self.machine.planes and self.plane not in self.machine.planes:
                    raise ValueError('plane not supported by machine definition')
                self.ready.add('plane')
            elif effect == 'length_off':
                self.length = None
                self.ready.add('length')
            elif effect == 'cycles_off':
                self.cycle_active = False
                self.ready.add(effect)
            elif effect in ('radius_left', 'radius_right') or effect == 'radius_off' and self.compensation != 'off':
                if d.compensation_with_move:
                    raise ValueError('this controller turns cutter compensation on/off in the block of an XY move')
                used.add(d.compensation_word)
                output.extend(self._compensation(effect, values))
            elif effect == 'rpm_mode':
                self.css = None
                self.ready.add(effect)
            elif effect == 'feed_per_minute':
                self.ready.add(effect)
                if self.feed_rev is not None:
                    self.feed_rev = None
                    output.append(FeedPerRev(0))
            elif effect in state_only:
                self.ready.add(effect)
            elif isinstance(effect, tuple):
                number = effect[1]
                if number != self.machine.work_offset:
                    raise ValueError('NC work offset differs from machine setup')
                self.offset = number
                output.append(WorkOffset(number))
            elif effect == 'tool_change':
                if d.tool_change != 'm6' or self.selected is None:
                    raise ValueError('unexpected M6/tool selection')
                self._activate(self.selected)
            elif effect == 'length_on':
                if d.length_activation != 'explicit':
                    raise ValueError('unexpected global length command')
                used.add(d.length_word)
                output.append(self._length(values.get(d.length_word)))
            elif effect == 'spindle_orient':
                if self.active is None or self.tool_pending or d.oriented_edge_deg is None:
                    raise ValueError('NC oriented spindle stop without tool or declared orientation')
                self.rpm = 0
                output.append(SpindleOrient(d.oriented_edge_deg))
            elif (
                effect in ('spindle_off', 'spindle_cw', 'spindle_ccw')
                and self.active is not None
                and self.active.tool in self.beams
            ):
                if d.beam_s_max is None:
                    raise ValueError('NC beam without a declared beam_s_max')
                if effect == 'spindle_off':
                    self.rpm = 0
                    output.append(Beam(0))
                else:
                    if effect != d.beam_effect or self.tool_pending:
                        raise ValueError('NC beam word differs from the declared beam_effect')
                    used.add(d.speed_word)
                    s = values.get(d.speed_word)
                    if s is None or not 0 < s <= d.beam_s_max:
                        raise ValueError('NC beam power S outside 0..beam_s_max')
                    self.rpm = s
                    output.append(Beam(s / d.beam_s_max * 100))
            elif effect in ('height_control_on', 'height_control_off'):
                if d.height_control_word is not None:
                    word = d.height_control_word[0]
                    used.add(word)
                    if values.get(word) != float(d.height_control_word[1:]):
                        raise ValueError('NC height control word differs')
                output.append(HeightControl(effect == 'height_control_on'))
            elif effect in ('spindle_off', 'spindle_cw', 'spindle_ccw'):
                if effect == 'spindle_off':
                    self.rpm = 0
                    self.ready.add('spindle_off')
                else:
                    if self.active is None or self.tool_pending:
                        raise ValueError('NC spindle without assigned tool/length')
                    if self.css is not None:
                        raise ValueError('S is a surface speed in G96 mode: G97 before an rpm')
                    used.add(d.speed_word)
                    self.rpm = values.get(d.speed_word)
                    if self.rpm is None or not 0 < self.rpm <= self.machine.max_spindle_rpm:
                        raise ValueError('invalid NC spindle')
                    self.direction = 'ccw' if effect == 'spindle_ccw' else 'cw'
                output.append(Spindle(self.rpm, 'ccw' if effect == 'spindle_ccw' else 'cw'))
            elif effect in ('coolant_off', 'flood', 'mist'):
                if effect == 'coolant_off':
                    self.coolant = 'off'
                    self.ready.add('coolant_off')
                else:
                    if self.coolant != 'off':
                        raise ValueError('exclusive coolant change requires off first')
                    self.coolant = effect
                output.append(Coolant(self.coolant))
            elif effect in ('stop', 'optional_stop'):
                output.append(Stop(effect == 'optional_stop'))
            elif effect == 'dwell':
                seconds = values.get(d.dwell_word)
                used.add(d.dwell_word)
                if seconds is None or not 0 < seconds <= d.dwell_max_s:
                    raise ValueError('NC dwell needs a positive time')
                output.append(Dwell(seconds))
            elif effect == 'end':
                if (
                    self.rpm != 0
                    or self.coolant != 'off'
                    or self.length is not None
                    or self.tool_pending
                    or self.compensation != 'off'
                ):
                    raise ValueError('program end requires stopped spindle/coolant and cancelled length')
                self.ended = True
            else:
                raise ValueError('unsupported declared effect')
        if d.tool_word in values:
            if effects:
                raise ValueError('select tool in its own block')
            used.add(d.tool_word)
            number = values[d.tool_word]
            if number != int(number):
                raise ValueError('tool must be integer')
            self.selected = int(number)
            if d.tool_change == 'lathe_t':
                # T<nn><oo>: tool nn with its offset oo, in one word.
                self.selected = int(number) // 100
                self._activate(self.selected)
                output.append(self._length(int(number) % 100))
            elif d.tool_change == 'direct_t':
                self._activate(self.selected)
        if d.length_word in values and d.length_word not in used:
            if d.length_activation != 'tool_edge' or effects and effects != ['tool_change']:
                raise ValueError('unexpected length word')
            used.add(d.length_word)
            number = values[d.length_word]
            if number == 0:
                self.length = None
                self.ready.add('length')
            else:
                output.append(self._length(number))
        if set(values) - used:
            raise ValueError('unused/unsupported NC words: ' + str(sorted(set(values) - used)))
        return output

    def _motion(self, effect, values, context):
        d = self.machine.dialect
        used = set()
        self.validate_initialized()
        if context is None or self.active is None or self.length is None or self.offset is None or self.tool_pending:
            raise ValueError('NC motion with unknown attribution/tool/length/offset')
        start = self.position
        end = list(start)
        for i, axis in enumerate(d.axes):
            if axis in values:
                end[i] = values[axis] * self.units * (0.5 if i == 0 and d.x_diameter else 1)
                used.add(axis)
        end = tuple(end)
        feed = None
        if effect != 'rapid':
            feed = values.get(d.feed_word)
            used.add(d.feed_word)
            if feed is None or feed <= 0:
                raise ValueError('NC motion needs positive explicit feed')
            feed *= self.units
        if feed is not None and self.feed_rev is not None:
            if abs(feed - self.feed_rev) > 1e-12 * max(1, feed):
                raise ValueError('NC feed differs from the feed per revolution in effect (G95 F)')
        curve = Line(start, end)
        if effect in ('arc_cw', 'arc_ccw'):
            u, v, a = {'xy': (0, 1, 2), 'xz': (2, 0, 1), 'yz': (1, 2, 0)}[self.plane]
            center = list(start)
            for i in (u, v):
                word = d.centers[i]
                if word not in values:
                    raise ValueError('NC arc needs both center offsets')
                center[i] += values[word] * self.units
                used.add(word)
            x, y = start[u] - center[u], start[v] - center[v]
            ex, ey = end[u] - center[u], end[v] - center[v]
            if math.hypot(x, y) < 1e-6 or not d.radius_accepts(math.hypot(x, y), math.hypot(ex, ey)):
                raise ValueError('NC arc radius mismatch/collapse exceeds declared decoder model')
            angle = math.atan2(x * ey - y * ex, x * ex + y * ey)
            sweep = angle % math.tau if effect == 'arc_ccw' else -((-angle) % math.tau)
            if sweep == 0:
                sweep = math.tau if effect == 'arc_ccw' else -math.tau
            curve = (Arc if end[a] == start[a] else Helix)(start, end, tuple(center), sweep, self.plane)
        if feed is not None and self.feed_rev is not None:
            # Per minute: at the highest spindle speed along the move.
            top = native.css_rpm_bound(_json(curve.to_dict()), *self.css) if self.css else self.rpm
            if not top > 0:
                raise ValueError('feed per revolution with the spindle stopped')
            feed = self.feed_rev * top
        motion = Motion(
            context['id'],
            context['operation'],
            self.active.tool,
            curve,
            feed,
            _kind(effect == 'rapid', context['kind']),
        )
        if set(values) - used:
            raise ValueError('unused/unsupported NC words: ' + str(sorted(set(values) - used)))
        self.position = end
        # A coordinate block can legitimately round to the current position.
        # Native intent checking must prove that its whole source line fits the
        # output tolerance; it never becomes a fake zero-length stock sweep.
        return None if effect in ('rapid', 'linear') and end == start else motion

    def _compensation(self, effect, values):
        """G41/G42 with the active tool's D register, or G40 while active."""
        d = self.machine.dialect
        if effect == 'radius_off':
            if self.compensation == 'off':
                self.ready.add(effect)
                return []
            self.compensation = 'off'
            self.ready.add(effect)
            return [CutterCompensation('off', self._compensation_plane())]
        if self.active is None or self.tool_pending or self.compensation != 'off':
            raise ValueError('cutter compensation needs an active tool and must be off before it is turned on')
        if d.compensation_register:
            if not d.compensation_word_free():
                raise ValueError('compensation word is not free in this dialect')
            if (
                values.get(d.compensation_word) != self.active.diameter_register
                or self.active.diameter_register is None
            ):
                raise ValueError('cutter compensation D register differs from the active tool\'s declared register')
        self.compensation = 'left' if effect == 'radius_left' else 'right'
        return [CutterCompensation(self.compensation, self._compensation_plane())]

    def _compensation_plane(self):
        # G18 on a lathe: XZ; otherwise the mill's XY.
        return 'xz' if self.plane == 'xz' else 'xy'

    def _surface_speed(self, effects, values):
        """G96 S<speed> M3/M4 with its rpm limit (G50 before, or a word here)."""
        d = self.machine.dialect
        commands = {c.effect for c in d.commands}
        rest = [e for e in effects if e != 'css_on']
        if len(rest) != 1 or rest[0] not in ('spindle_cw', 'spindle_ccw'):
            raise ValueError('constant surface speed is written with its spindle direction in one block')
        if self.active is None or self.tool_pending or self.active.tool in self.beams:
            raise ValueError('NC constant surface speed without an assigned (non-beam) tool')
        words = {d.speed_word}
        if 'css_limit' in commands:
            limit = self.css_max
        else:
            words.add(d.css_limit_word)
            limit = values.get(d.css_limit_word)
        s = values.get(d.speed_word)
        if (
            set(values) != words
            or s is None
            or s <= 0
            or limit is None
            or not 0 < limit <= self.machine.max_spindle_rpm
        ):
            raise ValueError('constant surface speed needs S and its rpm limit within the machine maximum')
        m_min = s * (0.3048 if self.units == 25.4 else 1)
        self.css = (m_min, limit)
        self.rpm = limit
        self.direction = 'ccw' if rest[0] == 'spindle_ccw' else 'cw'
        return [SurfaceSpeed(m_min, limit, self.direction)]

    def _drill_cycle(self, effect, values, context):
        """Expand a drilling/boring cycle block (G98 return) into the moves
        and spindle/dwell effects the controller makes, from its actual words
        (LinuxCNC interp_cycles.cc; FANUC 0i-MB 13.1; Centroid 12.28)."""
        d = self.machine.dialect
        used = {*d.axes, 'R', d.feed_word}
        self.validate_initialized()
        if (
            context is not None
            or self.active is None
            or self.length is None
            or self.offset is None
            or self.tool_pending
        ):
            raise ValueError('NC drilling cycle with unknown tool/length/offset')
        if self.feed_rev is not None:
            raise ValueError('drilling cycle in feed per revolution mode')
        if 'cycle_return_initial' not in self.ready:
            raise ValueError('drilling cycle without G98 initial-level return')
        if any(w not in values for w in used):
            raise ValueError('drilling cycle needs X Y Z R F')
        x, y, z = (values[a] * self.units for a in d.axes)
        r = values['R'] * self.units
        feed = values[d.feed_word] * self.units
        start = self.position
        z0 = start[2]
        if not (z < r <= z0) or feed <= 0:
            raise ValueError('drilling cycle needs Z < R <= the initial level and a positive feed')
        if self.cycle_active and z0 != self.cycle_initial:
            # FANUC keeps the initial level from where the cycle mode began.
            raise ValueError('a chained canned cycle must start at the same initial level (cancel with G80 first)')
        boring = effect in _BORE_CYCLES
        if boring and z0 != r:
            raise ValueError('boring cycles are written from the R plane (initial level = R)')
        dwell = None
        if effect in _DWELL_CYCLES:
            if 'P' not in values:
                raise ValueError('dwell cycle needs its dwell P')
            used.add('P')
            dwell = values['P']
            if d.cycle_dwell_unit == 'ms':
                if dwell != int(dwell):
                    raise ValueError('dwell P is whole milliseconds')
                dwell /= 1000
            if not 0 < dwell <= d.dwell_max_s:
                raise ValueError('invalid drilling cycle dwell')
        q = None
        if effect in ('peck_cycle', 'chip_break_cycle', 'fine_bore_cycle'):
            if 'Q' not in values or values['Q'] <= 0:
                raise ValueError('pecking/fine boring cycle needs a positive Q')
            used.add('Q')
            q = values['Q'] * self.units
        if effect == 'fine_bore_cycle' and 'P' in values:
            # G76 takes an optional dwell at the bottom.
            used.add('P')
            dwell = values['P']
            if d.cycle_dwell_unit == 'ms':
                if dwell != int(dwell):
                    raise ValueError('dwell P is whole milliseconds')
                dwell /= 1000
            if not 0 < dwell <= d.dwell_max_s:
                raise ValueError('invalid drilling cycle dwell')
        if set(values) - used:
            raise ValueError('unused/unsupported NC words: ' + str(sorted(set(values) - used)))
        if effect == 'bore_stop_cycle' and (self.rpm <= 0 or self.direction not in ('cw', 'ccw')):
            raise ValueError('G86 needs the spindle running')
        if effect == 'fine_bore_cycle' and (self.rpm <= 0 or self.direction != 'cw' or d.oriented_edge_deg is None):
            raise ValueError('G76 needs the spindle running clockwise and a declared oriented_edge_deg')
        # Items: (end, feed or None, rapid), ('dwell', s), ('spindle', rpm, direction).
        items = [((x, y, z0), None, True), ((x, y, r), None, True)]
        if q is not None and effect != 'fine_bore_cycle':
            current = r - q
            while current > z:
                items.append(((x, y, current), feed, False))
                if effect == 'peck_cycle':
                    items.append(((x, y, r), None, True))
                    items.append(((x, y, current + d.peck_reentry_mm), None, True))
                else:
                    items.append(((x, y, current + d.chip_break_lift_mm), None, True))
                current -= q
        items.append(((x, y, z), feed, False))
        if dwell is not None:
            items.append(('dwell', dwell))
        if effect == 'bore_stop_cycle':
            items += [('spindle', 0, self.direction), ((x, y, z0), None, True), ('spindle', self.rpm, self.direction)]
        elif effect == 'fine_bore_cycle':
            # FANUC 0i-MB 13.1.3: oriented stop, shift away from the edge,
            # rapid out, shift back, spindle clockwise.
            a = math.radians(d.oriented_edge_deg)
            s = (x - q * math.cos(a), y - q * math.sin(a))
            items += [
                ('orient',),
                ((*s, z), None, True),
                ((*s, z0), None, True),
                ((x, y, z0), None, True),
                ('spindle', self.rpm, 'cw'),
            ]
        elif boring:
            items.append(((x, y, r), feed, False))
        items.append(((x, y, z0), None, True))
        queue = []
        position = start
        for item in items:
            if isinstance(item[0], str):
                queue.append(item)
            elif item[0] != position:
                queue.append(item)
                position = item[0]
        self.cycle_queue = queue
        self.cycle_active = True
        self.cycle_initial = z0
        return []

    def _cycle_effect(self, context):
        item = self.cycle_queue.pop(0)
        if item[0] in ('dwell', 'spindle', 'orient'):
            if context is not None:
                raise ValueError('canned cycle control effect expected, not a move')
            if item[0] == 'dwell':
                return Dwell(item[1])
            if item[0] == 'orient':
                self.rpm = 0
                return SpindleOrient(self.machine.dialect.oriented_edge_deg)
            self.rpm = item[1]
            return Spindle(item[1], item[2])
        if context is None:
            raise ValueError('canned cycle move expected, not a control event')
        end, feed, rapid = item
        motion = Motion(
            context['id'],
            context['operation'],
            self.active.tool,
            Line(self.position, end),
            feed,
            _kind(rapid, context['kind']),
        )
        self.position = end
        return motion

    def _tap(self, effect, values, context):
        """Decode a tapping cycle at the current XY from its actual words."""
        d = self.machine.dialect
        used = set()
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
        if self.cycle_active:
            raise ValueError('canned cycle already active')
        if self.css is not None or self.feed_rev is not None:
            raise ValueError('tapping needs G97 and G94')
        if self.rpm <= 0 or self.direction != 'cw':
            raise ValueError('tapping needs the spindle running clockwise')
        start = self.position
        for i in (0, 1):
            if d.axes[i] in values:
                used.add(d.axes[i])
                if values[d.axes[i]] * self.units != start[i]:
                    raise ValueError('tapping cycle must start at its hole position')
        z = values.get(d.axes[2])
        used.add(d.axes[2])
        if z is None:
            raise ValueError('tapping cycle needs Z')
        bottom = (start[0], start[1], z * self.units)
        if effect == 'tap_synchronized':
            pitch = values.get('K')
            used.add('K')
            if pitch is None or pitch <= 0:
                raise ValueError('G33.1 needs a positive K pitch')
            feed = pitch * self.units * self.rpm
        else:
            r = values.get('R')
            used.add('R')
            if r is None or r * self.units != start[2]:
                raise ValueError('tapping cycle R plane must be the current Z')
            if d.tap_pitch_word is not None:
                lead = values.get(d.tap_pitch_word)
                used.add(d.tap_pitch_word)
                if lead is None or lead <= 0:
                    raise ValueError('rigid tapping cycle needs a positive lead')
                feed = lead * self.units * self.rpm
            else:
                feed = values.get(d.feed_word)
                used.add(d.feed_word)
                if feed is None or feed <= 0:
                    raise ValueError('tapping cycle needs a positive feed')
                feed *= self.units
            self.cycle_active = True
        if not bottom[2] < start[2]:
            raise ValueError('tapping cycle must go down')
        if set(values) - used:
            raise ValueError('unused/unsupported NC words: ' + str(sorted(set(values) - used)))
        self.position = bottom
        self.tap_return = (start, bottom, feed)
        return Motion(context['id'], context['operation'], self.active.tool, Line(start, bottom), feed, 'tap')

    def _thread(self, values, context):
        """A spindle-synchronized line: feed = lead x rpm."""
        d = self.machine.dialect
        word = d.thread_lead_word
        self.validate_initialized()
        if context is None or self.active is None or self.length is None or self.offset is None or self.tool_pending:
            raise ValueError('NC threading move with unknown attribution/tool/length/offset')
        if self.cycle_active or self.rpm <= 0 or self.css is not None:
            raise ValueError('threading needs the spindle running at a fixed rpm (G97) and no active cycle')
        lead = values.get(word)
        if lead is None or lead <= 0:
            raise ValueError('threading move needs a positive lead')
        start = self.position
        end = list(start)
        for i, axis in enumerate(d.axes):
            if axis in values:
                end[i] = values[axis] * self.units * (0.5 if i == 0 and d.x_diameter else 1)
        if set(values) - {*d.axes, word}:
            raise ValueError('unused/unsupported NC words in threading move')
        end = tuple(end)
        self.position = end
        return Motion(
            context['id'],
            context['operation'],
            self.active.tool,
            Line(start, end),
            lead * self.units * self.rpm,
            'thread',
        )

    def _activate(self, number):
        if self.rpm != 0:
            raise ValueError('stop spindle before selecting tool')
        choices = [t for t in self.machine.tools if t.number == number]
        self.active = next((t for t in choices if t.length_register == 1), choices[0] if choices else None)
        if self.active is None:
            raise ValueError('NC tool absent from machine table')
        if self.machine.tool_change_position_mode == 'table':
            self.position = tuple(self.active.position_after_mm)
        elif self.machine.tool_change_position_mode == 'z_home':
            # M6 sends the spindle to machine Z home; XY stays.
            m = self.machine
            gauge_z = m.tool_change_home_z_mm - m.work_to_machine.origin_mm[2]
            self.position = (self.position[0], self.position[1], gauge_z - self.active.gauge_length_mm)
        self.length = None
        self.tool_pending = True

    def _length(self, number):
        if self.active is None:
            raise ValueError('NC length without active tool')
        if self.machine.dialect.length_activation == 'tool_edge':
            edge = next(
                (t for t in self.machine.tools if t.number == self.active.number and t.length_register == number), None
            )
            if edge is None:
                raise ValueError('NC cutting edge absent from active tool table')
            self.active = edge
            # The table is an explicit pose premise after this tool/edge selection.
            if self.machine.tool_change_position_mode == 'table':
                self.position = tuple(edge.position_after_mm)
        elif number != self.active.length_register or not self.tool_pending:
            raise ValueError('NC tool/length register differs from configured table')
        self.length = int(number)
        self.tool_pending = False
        self.ready.add('length')
        t = self.active
        return ToolChange(t.tool, t.number, int(number), tuple(self.position))


class NativeGCodeDecoder(GCodeDecoder):
    """Same declared Python control model with immutable native motion-word decoding.

    The accelerator reads actual text/position/modes, never expected geometry.
    Custom language decoders can still subclass/use the reference GCodeDecoder.
    """

    version = 'configured_motion_native_v1'

    def __init__(self, machine, program):
        super().__init__(machine, program)
        dialect = asdict(machine.dialect)
        self._motion_decoder = native.MotionDecoder(
            _json(
                {
                    key: dialect[key]
                    for key in (
                        'commands',
                        'axes',
                        'centers',
                        'feed_word',
                        'parentheses_comments',
                        'radius_tolerance_mm',
                        'radius_relative_tolerance',
                    )
                }
            )
        )

    def read_line(self, line, context=None):
        if (
            self.rules is not self._compiled_rules
            or self.tap_return is not None
            or self.cycle_active
            or self.machine.dialect.x_diameter
            or self.feed_rev is not None
        ):
            return super().read_line(line, context)
        if self.ended:
            raise ValueError('NC after program end')
        decoded = self._motion_decoder.decode(line, self.position, self.units, self.plane)
        if decoded is None:
            return super().read_line(line, context)
        self.validate_initialized()
        if context is None or self.active is None or self.length is None or self.offset is None or self.tool_pending:
            raise ValueError('NC motion with unknown attribution/tool/length/offset')
        kind, end, center, sweep, feed, rapid = decoded
        if kind == 0 and tuple(end) == tuple(self.position):
            return []
        curve = (
            Line(self.position, end)
            if kind == 0
            else (Arc if kind == 1 else Helix)(self.position, end, center, sweep, self.plane)
        )
        motion = Motion(
            context['id'], context['operation'], self.active.tool, curve, feed, _kind(rapid, context['kind'])
        )
        self.position = end
        return [motion]


def _is_closed_curve(event):
    """Whether a source event is an arc or helix whose start and end coincide
    in its plane (a full turn)."""
    if not _is_curve(event):
        return False
    curve = _event_data(event)['motion']['curve']
    u, v = {'xy': (0, 1), 'xz': (0, 2), 'yz': (1, 2)}[curve['plane']]
    return curve['start'][u] == curve['end'][u] and curve['start'][v] == curve['end'][v]


def _is_curve(event):
    """Whether a source event is an arc or helix motion (in any accepted
    form: typed, an event dict, or a native Motion body)."""
    data = _event_data(event)
    return (
        isinstance(data, dict) and data.get('kind') == 'motion' and data['motion']['curve']['kind'] in ('arc', 'helix')
    )


class PostEngine:
    """Owns one run, bounded atomic batches and separately owned post/decoder state."""

    def __init__(self, machine, post=GCodePost, *, decoder=NativeGCodeDecoder, lowering=None):
        self.machine = machine
        self.lowering = lowering
        # Started source events not yet fully checked, oldest first.
        self._sources = []
        # Lowered runs: count of source arcs/helices by output form and reason.
        self.arc_output = dict(native=0, lines_policy=0, lines_machine=0, lines_precision=0)
        self.post = post(machine) if isinstance(post, type) else deepcopy(post, {id(machine): machine})
        if self.post.machine != machine:
            raise ValueError('post uses a different machine definition')
        self.post.machine = machine
        self.decoder_type = decoder
        self.decoder = None
        self._session = None
        self.finished = False

    def begin(self, program, *, run_id=None, cancel=None):
        if self._session is not None:
            raise ValueError('post engine already begun')
        m = self.machine
        if program.frame != m.frame:
            raise ValueError('program and machine work frame differ')
        if any(
            not any(t.id == s.tool and t.gauge_length_mm == s.gauge_length_mm for s in m.tools) for t in program.tools
        ):
            raise ValueError('machine tool table does not match program assemblies/lengths')
        post = deepcopy(self.post, _machine_memo(m))
        decoder = self.decoder_type(m, program)
        post.geometry_tolerance_mm = program.tolerance_mm
        lines = post._call('on_begin', replace(program, events=()), limit=32)
        actions = decoder.read(lines)
        if decoder.ended or any(
            not isinstance(a, (Spindle, Coolant))
            or (isinstance(a, Spindle) and a.rpm != 0)
            or (isinstance(a, Coolant) and a.mode != 'off')
            for a in actions
        ):
            raise ValueError('initialization must not move/select tools or change source setup')
        decoder.validate_initialized()
        definition = dict(
            machine=asdict(m),
            post=type(post).__module__ + '.' + type(post).__qualname__,
            post_version=post.version,
            decoder=self.decoder_type.__module__ + '.' + self.decoder_type.__qualname__,
            decoder_version=decoder.version,
        )
        for key in ('max_normal_accel_mm_s2', 'max_axis_feed_mm_min', 'motion_policy'):
            if getattr(m, key) is None:
                definition['machine'].pop(key)
        if not m.schedule_post_feeds:
            definition['machine'].pop('schedule_post_feeds')
        settings = dict(
            capabilities=m.capabilities(),
            tolerance_mm=m.tolerance_mm,
            feed_tolerance_mm_min=m.feed_tolerance_mm_min,
            traverse_feed_mm_min=m.traverse_feed_mm_min,
            work_to_machine=asdict(m.work_to_machine),
            tip_travel_mm=m.tip_travel_mm,
            definition=definition,
            run_id=run_id or uuid.uuid4().hex,
            decoded_initial_position_mm=decoder.position,
            lowering=asdict(self.lowering) if self.lowering is not None else None,
        )
        for key in ('max_normal_accel_mm_s2', 'max_axis_feed_mm_min'):
            if getattr(m, key) is not None:
                settings[key] = getattr(m, key)
        if m.schedule_post_feeds:
            from .dynamics import _settings

            scheduling = _settings(m, True)
            scheduling['policy']['junction_mode'] = 'exact_stop'
            settings['motion_scheduling'] = scheduling
        session = native.IntentSession(_json(replace(program, events=()).to_dict()), _json(settings), _json(lines))
        session.check_cancel(cancel)
        first = Chunk(session.take_begin())
        data = first.data
        data['definition'] = definition
        first = Chunk(_json(data).encode())
        self._session = session
        self.post = post
        self.decoder = decoder
        return first

    @property
    def position_mm(self):
        if self.decoder is None:
            raise ValueError('begin engine first')
        return self.decoder.position

    @property
    def pending(self):
        return self._session is not None and self._session.has_pending()

    def _clone_decoder(self):
        memo = _machine_memo(self.machine)
        rules = getattr(self.decoder, '_compiled_rules', None)
        if isinstance(rules, MappingProxyType):
            memo[id(rules)] = rules
        return deepcopy(self.decoder, memo)

    def _default_motions(self, post, decoder):
        """Native formatter/decoder for unmodified default post and decoder
        classes (same output, decoded independently); None otherwise."""
        P, D = type(post), type(decoder)
        if (
            self.machine.schedule_post_feeds
            or self.machine.traverse_feed_mm_min is not None
            or self.machine.dialect.x_diameter
            or any(
                getattr(P, name) is not getattr(GCodePost, name)
                for name in ('on_motion', 'circular_values', 'words', 'number', 'word', 'emit', 'tap', '_call')
            )
            or any(
                getattr(D, name) is not getattr(NativeGCodeDecoder, name)
                for name in ('read', 'read_line', '_motion', 'validate_initialized')
            )
            or decoder.rules is not decoder._compiled_rules
        ):
            return None
        native_post = self.__dict__.get('_native_default')
        if native_post is None:
            d = self.machine.dialect
            config = asdict(d)
            native_post = native.DefaultMotionPost(
                _json(
                    dict(
                        decoder={
                            key: config[key]
                            for key in (
                                'commands',
                                'axes',
                                'centers',
                                'feed_word',
                                'parentheses_comments',
                                'radius_tolerance_mm',
                                'radius_relative_tolerance',
                            )
                        },
                        codes={c.effect: c.code for c in d.commands},
                        allowed_planes=list(self.machine.planes),
                        unit=25.4 if self.machine.output_units == 'inch' else 1,
                        decimals=self.machine.decimals,
                        geometry_tolerance=post.geometry_tolerance_mm,
                        output_tolerance=self.machine.tolerance_mm,
                    )
                )
            )
            self._native_default = native_post
        return native_post

    def _format(self, events, originals, ranges, cancel, renumber=False):
        """renumber: a lowered batch's motion ids follow the decoded motions
        so far (a segment that rounds to no move takes no id), as the
        checker numbers them."""
        post = deepcopy(self.post, _machine_memo(self.machine))
        decoder = self._clone_decoder()
        blocks = []
        actual = []
        effects = []
        fast = self._default_motions(post, decoder)
        if fast is not None:
            return self._format_default(fast, post, decoder, events, originals, ranges, cancel, renumber)
        next_id = None
        for event, original, interval in zip(events, originals, ranges):
            self._session.check_cancel(cancel)
            kind = event['kind']
            if renumber and kind == 'motion':
                next_id = event['motion']['id'] if next_id is None else next_id
                event = dict(event, motion=dict(event['motion'], id=next_id))
            typed = _typed(event) if self.lowering is not None or self.machine.schedule_post_feeds else original
            if (
                isinstance(typed, Motion)
                and typed.kind in ('rapid', 'shift')
                and typed.feed_mm_min is None
                and self.machine.traverse_feed_mm_min is not None
            ):
                typed = replace(typed, feed_mm_min=self.machine.traverse_feed_mm_min)
            post.position_mm = decoder.position
            post.source_event = _typed(original) if isinstance(original, dict) else original
            post.source_fraction = interval if kind == 'motion' else None
            block = post._call('on_' + ('motion' if kind == 'motion' else kind), typed)
            if post.machine is not self.machine:
                raise ValueError('hook changed the machine definition')
            context = None if kind != 'motion' else {k: event['motion'][k] for k in ('id', 'operation', 'kind')}
            decoded = decoder.read(block, context)
            if decoder.ended or decoder.tool_pending:
                raise ValueError('event ended program or left a partial tool transition')
            if renumber and kind == 'motion' and any(isinstance(e, Motion) for e in decoded):
                next_id += 1
            # Own immutable snapshots before releasing the GIL in native checking.
            owned = [_typed(e.to_dict()) for e in decoded]
            blocks.append(block)
            actual.append([e.to_dict() for e in owned])
            effects.extend(owned)
        return post, decoder, blocks, actual, tuple(effects)

    def _format_default(self, fast, post, decoder, events, originals, ranges, cancel, renumber=False):
        """As _format, with runs of non-tapping motions formatted and decoded
        natively; other events use the Python hooks."""
        blocks = []
        actual = []
        effects = []
        next_id = None
        i = 0
        while i < len(events):
            self._session.check_cancel(cancel)
            j = i
            while (
                j < len(events)
                and events[j]['kind'] == 'motion'
                and events[j]['motion']['kind'] not in ('tap', 'thread')
            ):
                j += 1
            if (
                j > i
                and decoder.tap_return is None
                and not decoder.cycle_active
                and decoder.feed_rev is None
                and not getattr(post, '_feed_rev', None)
                and not getattr(post, '_cycle_open', False)
                and not decoder.cycle_queue
                and not getattr(post, '_pending_compensation', None)
            ):
                decoder.validate_initialized()
                if decoder.active is None or decoder.length is None or decoder.offset is None or decoder.tool_pending:
                    raise ValueError('NC motion with unknown attribution/tool/length/offset')
                lines, decoded, position, plane = fast.motions(
                    _json([e['motion'] for e in events[i:j]]), decoder.position, decoder.units, decoder.plane
                )
                for event, block, words in zip(events[i:j], lines, decoded):
                    blocks.append(block)
                    if renumber and next_id is None:
                        next_id = event['motion']['id']
                    if words is None:
                        actual.append([])
                        continue
                    (kind, end, center, sweep, feed, rapid), arc_plane = words
                    m = event['motion']
                    if renumber:
                        m = dict(m, id=next_id)
                        next_id += 1
                    start = decoder.position
                    curve = (
                        Line(start, end)
                        if kind == 0
                        else (Arc if kind == 1 else Helix)(start, end, center, sweep, arc_plane)
                    )
                    motion = Motion(m['id'], m['operation'], decoder.active.tool, curve, feed, _kind(rapid, m['kind']))
                    decoder.position = end
                    actual.append([motion.to_dict()])
                    effects.append(motion)
                decoder.position = position
                decoder.plane = plane
                if plane is not None:
                    decoder.ready.add('plane')
                i = j
                continue
            # One event through the Python hooks.
            event, original, interval = events[i], originals[i], ranges[i]
            kind = event['kind']
            if renumber and kind == 'motion':
                next_id = event['motion']['id'] if next_id is None else next_id
                event = dict(event, motion=dict(event['motion'], id=next_id))
            typed = _typed(event) if self.lowering is not None or self.machine.schedule_post_feeds else original
            post.position_mm = decoder.position
            post.source_event = _typed(original) if isinstance(original, dict) else original
            post.source_fraction = interval if kind == 'motion' else None
            block = post._call('on_' + ('motion' if kind == 'motion' else kind), typed)
            if post.machine is not self.machine:
                raise ValueError('hook changed the machine definition')
            context = None if kind != 'motion' else {k: event['motion'][k] for k in ('id', 'operation', 'kind')}
            decoded = decoder.read(block, context)
            if decoder.ended or decoder.tool_pending:
                raise ValueError('event ended program or left a partial tool transition')
            if renumber and kind == 'motion' and any(isinstance(e, Motion) for e in decoded):
                next_id += 1
            owned = [_typed(e.to_dict()) for e in decoded]
            blocks.append(block)
            actual.append([e.to_dict() for e in owned])
            effects.extend(owned)
            i += 1
        return post, decoder, blocks, actual, tuple(effects)

    @staticmethod
    def _checked_chunk(raw, blocks, effects):
        # Called only after native validation/commit. Snapshot from the exact event
        # records submitted to the checker, not from source geometry or arbitrary
        # custom decoder objects; raw proof metadata remains authoritative/available.
        out = Chunk(raw)
        out.__dict__['text'] = ''.join(line + '\n' for block in blocks for line in block)
        out.__dict__['events'] = effects
        out.__dict__['motions'] = tuple(e for e in effects if isinstance(e, Motion))
        return out

    def consume(self, events, *, cancel=None):
        """Direct mode only: one bounded source batch produces one chunk."""
        if self._session is None or self.finished:
            raise ValueError('generic post lifecycle: begin/finish')
        if self.lowering is not None:
            raise ValueError('lowering uses start/drain or consume_events')
        self._session.check_cancel(cancel)
        events = _batch(events)
        source = [_event_data(e) for e in events]
        fast = self._default_motions(self.post, self.decoder)
        d = self.decoder
        if (
            fast is not None
            and source
            and d.tap_return is None
            and not d.cycle_active
            and not getattr(self.post, '_cycle_open', False)
            and not d.cycle_queue
            and all(e['kind'] == 'motion' and e['motion']['kind'] != 'tap' for e in source)
        ):
            # Default post: one native call formats, decodes and checks the batch.
            d.validate_initialized()
            if d.active is None or d.length is None or d.offset is None or d.tool_pending:
                raise ValueError('NC motion with unknown attribution/tool/length/offset')
            raw, position, plane = self._session.consume_default(
                _json(source), fast, d.position, d.units, d.plane, d.active.tool, cancel
            )
            d.position, d.plane = tuple(position), plane
            if plane is not None:
                d.ready.add('plane')
            return Chunk(raw)
        originals = [
            e
            if isinstance(
                e,
                (
                    Motion,
                    ToolChange,
                    WorkOffset,
                    Spindle,
                    Coolant,
                    Stop,
                    Dwell,
                    Cycle,
                    CutterCompensation,
                    SpindleOrient,
                    Beam,
                    HeightControl,
                    SurfaceSpeed,
                    FeedPerRev,
                ),
            )
            else _typed(d)
            for e, d in zip(events, source)
        ]
        prepared = (
            json.loads(self._session.preview_source(_json(source), cancel))
            if self.machine.schedule_post_feeds
            else source
        )
        post, decoder, blocks, actual, effects = self._format(prepared, originals, [(0.0, 1.0)] * len(source), cancel)
        out = self._checked_chunk(
            self._session.consume(_json(source), _json(actual), _json(blocks), cancel), blocks, effects
        )
        self.post = post
        self.decoder = decoder
        return out

    def start(self, event, *, cancel=None):
        """Accept one original source event; no expanded motion list is retained.
        Drain the pending source first."""
        if self.pending:
            raise ValueError('drain pending source before start/finish')
        self._start(event, cancel=cancel)

    def _start(self, event, *, cancel=None):
        """Start a source event, queued behind any pending ones."""
        if self._session is None or self.finished:
            raise ValueError('generic start lifecycle')
        source = _event_data(event)
        # The typed event only where it is read: an arc's output form here,
        # the hooks' source_event when formatted (typed lazily there).
        original = _typed(source) if _is_curve(source) else source
        reason = self._arc_output(original)
        self._session.start(_json(source), cancel, reason == 'native')
        if reason is not None:
            self.arc_output[reason] += 1
        self._sources.append(original)

    def _start_many(self, events, *, cancel=None):
        """_start for several events in one native call (all or none); on an
        error they are started one by one, so the error names its event."""
        if len(events) == 1:
            self._start(events[0], cancel=cancel)
            return
        if self._session is None or self.finished:
            raise ValueError('generic start lifecycle')
        sources, originals, reasons = [], [], []
        for event in events:
            source = _event_data(event)
            original = _typed(source) if _is_curve(source) else source
            sources.append(source)
            originals.append(original)
            reasons.append(self._arc_output(original))
        try:
            self._session.start_many(_json(sources), [r == 'native' for r in reasons], cancel)
        except ValueError:
            for event in events:
                self._start(event, cancel=cancel)
            return
        for reason in reasons:
            if reason is not None:
                self.arc_output[reason] += 1
        self._sources.extend(originals)

    def _arc_output(self, event):
        """How a lowered source arc/helix is written; None for other events.

        'native': one G2/G3 block (LoweringOptions(native_arcs=True), machine
        accepts the curve, exact words exist). Otherwise checked lines, with
        the reason: 'lines_policy' (native_arcs off), 'lines_machine' (plane,
        helix or sweep not accepted) or 'lines_precision' (no exactly
        consistent words at the configured decimals). The native session
        re-checks capabilities and certifies every decoded block regardless.
        """
        if not isinstance(event, Motion) or not isinstance(event.curve, Arc):
            return None
        if not getattr(self.lowering, 'native_arcs', False):
            return 'lines_policy'
        c = event.curve
        m = self.machine
        if (
            c.plane not in m.planes
            or abs(c.sweep_rad) > m.max_sweep_rad
            or not (m.helices if isinstance(c, Helix) else m.arcs)
        ):
            return 'lines_machine'
        post = deepcopy(self.post, _machine_memo(m))
        post.position_mm = self.decoder.position
        try:
            post.circular_values(c)
        except ValueError:
            return 'lines_precision'
        return 'native'

    def drain(self, max_segments=64, *, cancel=None):
        """Format/decode/check at most 64 planned segments, then atomically commit."""
        if self._session is None or self.finished:
            raise ValueError('generic drain lifecycle')
        preview = json.loads(self._session.preview(max_segments, cancel))
        events = [p['event'] for p in preview]
        ranges = [p['source_fraction_interval'] for p in preview]
        post, decoder, blocks, actual, effects = self._format(
            events, [self._sources[p['source']] for p in preview], ranges, cancel, renumber=True
        )
        out = self._checked_chunk(
            self._session.check_lowered(_json(ranges), _json(actual), _json(blocks), cancel), blocks, effects
        )
        self.post = post
        self.decoder = decoder
        # Sources whose last segment was in this batch are complete.
        del self._sources[: sum(r[1] == 1.0 for r in ranges)]
        return out

    def consume_events(self, events, *, cancel=None):
        """Yield bounded chunks in either mode, polling before pulling sources."""
        if self._session is None or self.finished or self.pending:
            raise ValueError('drain pending source before consuming events')
        queue = isinstance(events, (tuple, list)) or getattr(events, 'prefetch', False) is True
        # A source that says which events are in memory (ready()) queues
        # them, never waiting for more while some are queued.
        ready = getattr(events, 'ready', None)
        ready = ready if callable(ready) else None
        iterator = iter(events)
        held = None
        while True:
            self._session.check_cancel(cancel)
            if self.lowering is not None:
                # Sources that cannot block (a sequence) or that allow it
                # (prefetch) are queued so one checked batch covers up to 64
                # segments of several; any other iterator is pulled only once
                # the current source is drained. Feed scheduling and arcs
                # (their output form depends on the decoded start) take the
                # queue alone.
                limit = 64 if (queue or ready is not None) and not self.machine.schedule_post_feeds else 1
                done = False
                batch = []
                while len(self._sources) + len(batch) < limit:
                    queued = self._sources or batch
                    if held is None:
                        if queued and not queue and not ready():
                            break
                        self._session.check_cancel(cancel)
                        try:
                            held = next(iterator)
                        except StopIteration:
                            done = True
                            break
                    # Only a native arc's output form depends on the decoded
                    # start (exact words there); lowered to lines it does not.
                    # A closed curve's lowered chain ends at its decoded
                    # start, known once the queued sources are output.
                    if queued and (
                        getattr(self.lowering, 'native_arcs', False) and _is_curve(held) or _is_closed_curve(held)
                    ):
                        break
                    batch.append(held)
                    held = None
                if batch:
                    self._start_many(batch, cancel=cancel)
                if self.pending:
                    yield self.drain(cancel=cancel)
                if done:
                    while self.pending:
                        yield self.drain(cancel=cancel)
                    return
            else:
                batch = tuple(islice(iterator, 64))
                if not batch:
                    return
                yield self.consume(batch, cancel=cancel)

    def finish(self, *, cancel=None):
        if self._session is None or self.finished or self.pending:
            raise ValueError('generic finish lifecycle: drain pending source')
        self._session.check_cancel(cancel)
        post = deepcopy(self.post, _machine_memo(self.machine))
        decoder = self._clone_decoder()
        post.position_mm = decoder.position
        lines = post._call('on_finish')
        actions = decoder.read(lines)
        if not decoder.ended or any(
            not isinstance(a, (Spindle, Coolant))
            or isinstance(a, Spindle)
            and a.rpm != 0
            or isinstance(a, Coolant)
            and a.mode != 'off'
            for a in actions
        ):
            raise ValueError('finish needs stopped spindle/coolant and end; no extra motion/effects')
        out = self._checked_chunk(self._session.finish(_json(lines), cancel), [lines], ())
        self.post = post
        self.decoder = decoder
        self.finished = True
        return out


def stream_program(
    header, events, machine, *, post=GCodePost, decoder=NativeGCodeDecoder, cancel=None, run_id=None, lowering=None
):
    engine = PostEngine(machine, post, decoder=decoder, lowering=lowering)
    yield engine.begin(header, run_id=run_id, cancel=cancel)
    yield from engine.consume_events(events, cancel=cancel)
    yield engine.finish(cancel=cancel)


# Import after declarations so camlib.post and camlib.postprocessor can each be
# imported first (camlib.post re-exports the public authoring API).
from .post import Chunk, ToolSetting, _json, _event_data, _batch  # noqa: E402
