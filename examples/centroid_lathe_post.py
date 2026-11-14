"""Centroid CNC12 lathe post (Acorn, AcornSix, Hickory and Oak lathes).

Reference: Centroid CNC12 Lathe Operator's Manual (v5.x): T word (8.2.9),
G-codes (chapter 9: G0-G3 with I/K always radius values, G4 P seconds, G32
lead in F, G50 S CSS limit, G52, G61, G96/G97, G98/G99) and M-codes
(chapter 10: M3-M9, M30). See docs/centroid-lathe-post.md.

Program shape:

    G21 G18 G98 G97 G40 G80        header, one code per block
    G52 X0 Z0, G61, M5, M9         no local shift, exact path, all off
    G54
    T0101                          tool 01, offset 01 (a new tool number runs
                                   the tool change: Z home, spindle and coolant off)
    G50 S3000 / G96 S200 M3        constant surface speed with its rpm limit
    (or M3 S<rpm>)
    G99 F0.25                      feed per revolution (G98 per minute)
    M8
    G0 X.. / G0 Z..                rapids along one axis at a time
    G1/G2/G3 X Z [I K] F           X a diameter; I, K radius values, incremental
    G32 X Z F<lead>                threading
    M9, M5, M30

There is no G90 or G91.1: on this lathe G90 is the diameter cutting cycle,
X/Z are always absolute (U/W incremental) and I/K always incremental. There
is no G43/G49: T<tt><oo> applies the offset.

Premises (declared, not checked on the machine):
- Parameter 55 = 0: X words are diameters.
- Tool offsets are touched off on a diameter and a face: the program moves
  each tool's imaginary tip (TurningSetup controlled_point="tip").
- A new T number moves Z home and stops the spindle and coolant (the M6
  macro): the job's initial position is where the change leaves the tool;
  each tool's ToolSetting.position_after_mm is that position
  (tool_change_position_mode="table").
- G0 runs each axis at its own maximum rate: a rapid moving both X and Z
  would not follow a straight line, so rapids move one axis at a time
  (refused otherwise; an axis that only settles onto its rounded word,
  within the machine's tolerance, is not a second move).
- M30 ends the program like M2 (moves Z home, then restarts).
- Tool-nose compensation in the control (TurnFinish compensation="wear"):
  G41/G42 T<oo> (9.15) names the active offset, whose nose radius holds
  only the wear and whose tip direction is set in the Offset Library.
- Arcs need exactly consistent words: post with
  lowering=LoweringOptions(native_arcs=True) if an arc is refused.
No controller connection; verify on the machine before cutting.
"""

from dataclasses import dataclass, replace
import re

from camlib.post import Machine, GCodeDialect, GCodePost, NativeGCodeDecoder

_REFERENCE = 'Centroid CNC12 Lathe Operator\'s Manual (v5.x)'


def _dialect():
    commands = (
        GCodeDialect()
        .with_commands(
            absolute=None,
            center_incremental=None,
            feed_per_minute='G98',
            frame_reset='G52',
            exact_stop='G61',
            length_on=None,
            length_off=None,
            tool_change=None,
            end='M30',
            tap_synchronized=None,
            drill_cycle=None,
            drill_dwell_cycle=None,
            peck_cycle=None,
            chip_break_cycle=None,
            cycle_return_initial=None,
            bore_cycle=None,
            bore_dwell_cycle=None,
            bore_stop_cycle=None,
            thread_synchronized='G32',
            css_on='G96',
            css_limit='G50',
            feed_per_rev='G99',
        )
        .commands
    )
    return replace(
        GCodeDialect(),
        commands=commands,
        parentheses_comments=False,
        radius_tolerance_mm=0,
        fixed_modes=('absolute', 'center_incremental'),
        x_diameter=True,
        tool_change='lathe_t',
        thread_lead_word='F',
        # 9.15: G41/G42 name the offset whose nose radius applies (G41 T03),
        # on their own line; G40 cancels.
        compensation_register=False,
        # 9.4: G4 P seconds, 0.01 to 327.67 s.
        dwell_resolution_s=0.01,
        dwell_max_s=327.67,
    )


@dataclass(frozen=True, kw_only=True)
class CentroidLatheMachine(Machine):
    reference: str = _REFERENCE

    def __post_init__(self):
        super().__post_init__()
        if self.dialect != _dialect() or self.planes != ('xz',):
            raise ValueError('dialect differs from the CNC12 lathe subset (XZ arcs, diameter X, T<tt><oo>)')
        if self.tool_change_position_mode != 'table' or self.traverse_feed_mm_min is not None:
            raise ValueError(
                'a new T number moves Z home: declare each tool\'s position after the change '
                '(tool_change_position_mode="table"); rapids are G0'
            )


def define_machine(name, tools, **settings):
    """``tools``: ToolSetting per turret station (number = tt, length_register
    = oo), position_after_mm where the tool change leaves the tool."""
    return CentroidLatheMachine(
        name, tuple(tools), dialect=_dialect(), planes=('xz',), **{'tool_change_position_mode': 'table', **settings}
    )


class CentroidLathePost(GCodePost):
    version = 'centroid-cnc12-lathe-1'

    def on_begin(self, program):
        d = self.machine.dialect
        self.geometry_tolerance_mm = program.tolerance_mm
        for effect in (
            'units_mm' if self.machine.output_units == 'mm' else 'units_inch',
            'plane_xz',
            'feed_per_minute',
            'rpm_mode',
            'radius_off',
            'cycles_off',
        ):
            self.emit(d.code(effect))
        # G52 with zero shifts cancels any local offset (9.17); G61 stops at
        # every block and turns smoothing off (9.20): the exact path.
        self.emit('G52 X0 Z0')
        for effect in ('exact_stop', 'spindle_off', 'coolant_off'):
            self.emit(d.code(effect))
        self._spindle_on = False
        self._coolant_mode = 'off'

    def on_cutter_compensation(self, event):
        # 9.15: "G41 T03 ; compensate to the left by the nose radius that
        # corresponds to T03 in the Offset Library": the active offset.
        d = self.machine.dialect
        if event.side == 'off':
            self.emit(d.code('radius_off'))
            return
        tool = getattr(self, '_tool_setting', None)
        if tool is None:
            raise ValueError('CNC12 lathe: tool-nose compensation needs an active tool offset')
        code = d.code('radius_left' if event.side == 'left' else 'radius_right')
        self.emit(code, f'T{tool.length_register:02d}')

    def on_motion(self, motion):
        c = motion.curve
        if c.start[1] != 0 or c.end[1] != 0:
            raise ValueError('a lathe has no Y axis: every move stays at y = 0')
        if motion.kind in ('rapid', 'shift') and c.start[0] != c.end[0] and c.start[2] != c.end[2]:
            raise ValueError('CNC12 G0 runs each axis at its own rate: a lathe rapid moves along X or Z only')
        super().on_motion(motion)
        # No Y words. (The header's G18 stays the plane: no arc writes another.)
        y = self.machine.dialect.axes[1]
        self._lines = [' '.join(w for w in line.split(' ') if not w.startswith(y)) for line in self._lines]

    def on_finish(self):
        d = self.machine.dialect
        if getattr(self, '_coolant_mode', None) != 'off':
            self.emit(d.code('coolant_off'))
        if getattr(self, '_spindle_on', False):
            self.emit(d.code('spindle_off'))
        self.emit(d.code('end'))


class CentroidLatheDecoder(NativeGCodeDecoder):
    """Reads the CNC12 lathe words back into the generic decoder's model."""

    version = 'centroid-cnc12-lathe-nominal-1'

    def __init__(self, machine, program):
        super().__init__(machine, program)
        # No length codes: the T word applies the offset before any motion.
        self.ready.add('length')

    def read_line(self, line, context=None):
        d = self.machine.dialect
        text = line.partition(';')[0].strip()
        compensation = re.fullmatch(r'(G4[12]) T(\d\d)', text)
        if compensation:
            # The offset named must be the active one (its nose radius).
            if int(compensation.group(2)) != self.length or self.compensation != 'off':
                raise ValueError('G41/G42 T<oo> must name the active offset, compensation off before')
            self.compensation = 'left' if compensation.group(1) == 'G41' else 'right'
            from camlib.motion import CutterCompensation

            return [CutterCompensation(self.compensation, 'xz')]
        if text == 'G52 X0 Z0':
            self.ready.add('frame_reset')
            return []
        words = text.split(' ')
        if words[0] == d.code('rapid'):
            x = next((float(w[1:]) for w in words if w.startswith('X')), None)
            z = next((float(w[1:]) for w in words if w.startswith('Z')), None)
            if (
                x is not None
                and z is not None
                # Each axis at its own rate keeps the tool inside the rectangle of
                # the two moves. Where the smaller side is within the machine's
                # tolerance (an axis only settling onto its rounded word: inch
                # output from a position stated in millimetres), every path in it
                # is within that tolerance of the straight line.
                and min(abs(x * self.units / 2 - self.position[0]), abs(z * self.units - self.position[2]))
                > self.machine.tolerance_mm
            ):
                raise ValueError('G0 moving X and Z together does not follow a straight line on CNC12')
        if text == d.code('end'):
            # The offset stays with the tool; the program ends.
            self.length = None
        return super().read_line(line, context)
