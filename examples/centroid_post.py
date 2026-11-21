"""Ordinary Python Centroid CNC12 milling post (Acorn, AcornSix, Hickory, Oak,
Allin1DC, MPU11, M400, M39). See docs/centroid-post.md.

Reference: Centroid CNC12 Milling Machine Operators Manual, CNC12 v5.42+.
Explicit setup premises, actual-word decoding and native continuous checks;
no controller connection or machine qualification.
"""

from dataclasses import dataclass, replace
import re

from camlib.post import Machine, GCodeDialect, GCodePost, NativeGCodeDecoder, LoweringOptions, post_events

_WORDS = re.compile(r'([A-Z])\s*([+-]?(?:\d+(?:\.\d*)?|\.\d+))')
# Manual chapter 12/13: G0 dogleg (Z first up, last down) is not this path
# model, so traverse uses G1 at a declared feed. M30 is the documented end of
# program code (posted at the end of every program; default action none);
# M2 restarts the program and is not used.
_EXPECTED = {
    'units_mm': 'G21',
    'units_inch': 'G20',
    'absolute': 'G90',
    'plane_xy': 'G17',
    'plane_xz': 'G18',
    'plane_yz': 'G19',
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
    'mist': 'M7',
    'flood': 'M8',
    'coolant_off': 'M9',
    'stop': 'M0',
    'optional_stop': 'M1',
    'end': 'M30',
    'dwell': 'G4',
    # 12.28-12.35, 12.44: canned drilling with initial-point return.
    'drill_cycle': 'G81',
    'drill_dwell_cycle': 'G82',
    'cycle_return_initial': 'G98',
    # 12.37-12.38: G85 boring, G89 with dwell (CNC12 has no G86).
    'bore_cycle': 'G85',
    'bore_dwell_cycle': 'G89',
    # 12.15: cutter compensation, D register on the G41/G42 line.
    'radius_left': 'G41',
    'radius_right': 'G42',
}


@dataclass(frozen=True, kw_only=True)
class CentroidMachine(Machine):
    """Installed CNC12 mill premises.

    rigid_tapping: Parameter 36 enables rigid tapping (spindle encoder); G84
    is then written with Q = thread lead on every line.
    ijk_incremental: Parameter 2 bit 0 clear (I/J/K incremental, the default).
    Tool table poses are the tip positions after M6 (which moves Z home first).
    """

    rigid_tapping: bool = False
    # Parameter 83 (G83 rapid-to-clearance amount) and Parameter 73 (G73
    # retract, G10 P73), in mm. Declare them to write G83/G73; otherwise
    # pecking holes are written as explicit moves.
    peck_clearance_mm: float | None = None
    chip_break_retract_mm: float | None = None
    # 13.16: M19 is a custom macro (no default action). Declare where the
    # boring bar's edge points (work XY degrees) after it to fine bore with
    # M19 and explicit shift moves. G76 is not written: the manual does not
    # say whether it shifts back or restarts the spindle (12.31).
    oriented_edge_deg: float | None = None
    ijk_incremental: bool = True
    reference: str = 'Centroid CNC12 Milling Machine Operators Manual v5.42+'

    def __post_init__(self):
        super().__post_init__()
        if type(self.rigid_tapping) is not bool or self.ijk_incremental is not True:
            raise ValueError('declare rigid tapping (Parameter 36) and incremental IJK (Parameter 2 bit 0 clear)')
        if self.traverse_feed_mm_min is None:
            raise ValueError('declare traverse_feed_mm_min: G0 doglegs are not this path model')
        if self.tool_change_position_mode not in ('table', 'z_home'):
            raise ValueError(
                'M6 moves Z home: declare z_home with the home Z, or the tip poses after M6 in the tool table'
            )
        d = self.dialect
        expected = dict(
            _EXPECTED,
            **({'tap_cycle': 'G84'} if self.rigid_tapping else {}),
            **({'peck_cycle': 'G83'} if self.peck_clearance_mm is not None else {}),
            **({'chip_break_cycle': 'G73'} if self.chip_break_retract_mm is not None else {}),
            **({'spindle_orient': 'M19'} if self.oriented_edge_deg is not None else {}),
        )
        if {c.effect: c.code for c in d.commands} != expected or len(d.commands) != len(expected):
            raise ValueError('command mapping differs from the documented CNC12 subset')
        if d.parentheses_comments or d.tap_pitch_word != ('Q' if self.rigid_tapping else None):
            raise ValueError('CNC12 comments start with ";"; rigid G84 takes its lead in Q')
        if set(d.fixed_modes) != {'center_incremental', 'rpm_mode'}:
            raise ValueError('incremental IJK and RPM spindle mode are fixed premises')
        if (d.dwell_word, d.dwell_resolution_s, d.dwell_max_s, d.cycle_dwell_unit) != ('P', 0.01, 327.67, 's'):
            raise ValueError('CNC12 dwell is P seconds, 0.01 to 327.67 s (12.4, 12.28)')
        if (d.peck_reentry_mm, d.chip_break_lift_mm) != (self.peck_clearance_mm, self.chip_break_retract_mm):
            raise ValueError('G83/G73 return amounts are Parameters 83 and 73')
        if d.oriented_edge_deg != self.oriented_edge_deg:
            raise ValueError('the M19 edge direction is the machine\'s oriented_edge_deg')


def define_machine(
    name,
    tools,
    *,
    traverse_feed_mm_min,
    work_to_machine,
    rigid_tapping=False,
    peck_clearance_mm=None,
    chip_break_retract_mm=None,
    oriented_edge_deg=None,
    **settings,
):
    """Bind the installed machine in ordinary Python; no controller writes."""
    commands = (
        GCodeDialect(oriented_edge_deg=oriented_edge_deg)
        .with_commands(
            rapid=None,
            center_incremental=None,
            rpm_mode=None,
            frame_reset='G52',
            exact_stop='G61',
            end='M30',
            tap_synchronized=None,
            tap_cycle='G84' if rigid_tapping else None,
            bore_stop_cycle=None,
            peck_cycle='G83' if peck_clearance_mm is not None else None,
            chip_break_cycle='G73' if chip_break_retract_mm is not None else None,
            spindle_orient='M19' if oriented_edge_deg is not None else None,
        )
        .commands
    )
    # radius_tolerance_mm=0: CNC12's handling of an arc whose end radius
    # differs from its start radius is not documented, so arcs need exact words.
    d = replace(
        GCodeDialect(),
        commands=commands,
        parentheses_comments=False,
        radius_tolerance_mm=0,
        fixed_modes=('center_incremental', 'rpm_mode'),
        tap_pitch_word='Q' if rigid_tapping else None,
        # 12.4: G4 P seconds, 0.01 to 327.67 s.
        dwell_resolution_s=0.01,
        dwell_max_s=327.67,
        peck_reentry_mm=peck_clearance_mm,
        chip_break_lift_mm=chip_break_retract_mm,
        oriented_edge_deg=oriented_edge_deg,
    )
    return CentroidMachine(
        name,
        tools,
        dialect=d,
        traverse_feed_mm_min=traverse_feed_mm_min,
        work_to_machine=work_to_machine,
        rigid_tapping=rigid_tapping,
        peck_clearance_mm=peck_clearance_mm,
        chip_break_retract_mm=chip_break_retract_mm,
        oriented_edge_deg=oriented_edge_deg,
        **{'tool_change_position_mode': 'table', **settings},
    )


class CentroidPost(GCodePost):
    version = 'centroid-cnc12-mill-1'

    def on_begin(self, program):
        super().on_begin(program)
        # G52 with explicit zero shifts cancels any local offset (12.20).
        # G61 decelerates and stops every block and turns smoothing off
        # (12.23), so the programmed path is followed exactly.
        self._lines = [('G52 X0 Y0 Z0' if line == 'G52' else line) for line in self._lines]


class CentroidDecoder(NativeGCodeDecoder):
    """Independent actual-block model of the CNC12 subset."""

    version = 'centroid-cnc12-mill-nominal-1'

    def read_line(self, line, context=None):
        body = line.partition(';')[0].strip().upper()
        if body.startswith(':'):
            return []  # visible comment line (11.2.12)
        pieces = _WORDS.findall(body)
        values = {w: float(v) for w, v in pieces}
        if values.get('G') == 0:
            raise ValueError('G0 dogleg rejected; use the configured feed traverse')
        if values.get('M') == 2:
            raise ValueError('M2 restarts the program on CNC12; end with M30')
        if values.get('G') == 52:
            if set(values) != {'G', 'X', 'Y', 'Z'} or any(values[a] != 0 for a in 'XYZ'):
                raise ValueError('only an explicit zero XYZ G52 reset is supported')
            self.ready.add('frame_reset')
            # 12.20: G52 shifts the local coordinate system (zero cancels the
            # shift); it moves no axis, the X/Y/Z words are the shift.
            self.hold_axes()
            return []
        if values.get('G') in (73, 83, 84) and any('.' not in text for w, text in pieces if w == 'Q'):
            # Parameters 245/246 may reinterpret an undotted Q (implied decimals).
            raise ValueError('write Q with a decimal point')
        return super().read_line(line, context)


def post_centroid(program, machine, *, lowering=None, **options):
    return post_events(
        program,
        program.events,
        machine=machine,
        post=CentroidPost,
        decoder=CentroidDecoder,
        lowering=LoweringOptions() if lowering is None else lowering,
        **options,
    )
