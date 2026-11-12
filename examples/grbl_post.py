"""GRBL 1.1h post: one preloaded tool, straight lines, GRBL's own arithmetic.

Reference: GRBL 1.1h source (gcode.c for the accepted words and modes,
nuts_bolts.c read_float for number reading, config.h LINE_BUFFER_SIZE 80) and
its laser-mode documentation ($30 maximum spindle speed, $32 laser mode).

Program shape:

    G21 (G20), G90, G91.1, G17, G94, G40, G49, G80, G92.1, G61, M5, M9
    G54                     G54..G59 only
    T<n>                    the tool number only: no tool changer, no H offsets
    S<rpm> M3               or, in laser mode, S<power> M4
    G0/G1 X Y Z [F]         lines only: arcs are posted as checked segments
    M5, M9, G49, M2

Premises (declared, not checked on the machine):
- One tool, loaded and touched off before the program: its length is part of
  the work offset (no G43). ``work_offset_machine_mm`` is that offset in
  machine coordinates and ``initial_machine_mm`` the machine position at the
  start; their binary32 difference is the decoded start.
- GRBL reads every number with its own eight-digit decimal routine and keeps
  positions as binary32 machine coordinates. The decoder reproduces exactly
  that arithmetic (camlib's native GRBL model), so a posted program is checked
  where GRBL will actually move.
- Lines are at most 79 characters (LINE_BUFFER_SIZE 80); numbers use at most
  eight digits, so precision drops for large values.
- GRBL ignores M1: optional stops are refused.
- Laser mode ($32=1): the beam is on with M4 S<power>, scaled by
  ``beam_s_max`` = $30.
- Marking (``ThermalMark``) only on a machine declared with
  ``beam_mark="power"``: GRBL has no marking process, a mark is the same
  M4 S<power> at the marking power (Grbl v1.1 Laser Mode: S sets the laser
  power; M4 dynamic power is "very useful for clean, precise engraving").
  The marking power, feed and height are the caller's.
No controller connection; verify on the machine before cutting.
"""

from dataclasses import dataclass, replace
import math

from camlib._buffers import native
from camlib.motion import Line, Motion
from camlib.post import Machine, GCodeDialect, GCodePost
from camlib.postprocessor import GCodeDecoder, _WORD, _kind

_REFERENCE = "GRBL 1.1h (gcode.c, nuts_bolts.c read_float, config.h)"
_LINE_LIMIT = 79


def _dialect(beam_s_max=None, beam_mark=None):
    commands = (
        GCodeDialect()
        .with_commands(
            rpm_mode=None,
            exact_stop="G61",
            length_on=None,
            tool_change=None,
            optional_stop=None,
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
            tap_synchronized=None,
        )
        .commands
    )
    return replace(
        GCodeDialect(),
        commands=commands,
        # GRBL has no G97: rpm mode is its only spindle mode.
        fixed_modes=("rpm_mode",),
        tool_change="direct_t",
        beam_effect="spindle_ccw",
        beam_s_max=beam_s_max,
        beam_mark=beam_mark,
        radius_tolerance_mm=0,
    )


@dataclass(frozen=True, kw_only=True)
class GrblMachine(Machine):
    """A GRBL machine: the controller facts are fixed here, the machine's own
    (units, precision, limits, coolant) come from the caller."""

    work_offset_machine_mm: tuple
    initial_machine_mm: tuple
    reference: str = _REFERENCE

    def __post_init__(self):
        super().__post_init__()
        if self.dialect.beam_mark not in (None, "power"):
            raise ValueError('GRBL has no marking process: a laser marks by its power word (beam_mark="power")')
        if self.dialect != _dialect(self.dialect.beam_s_max, self.dialect.beam_mark):
            raise ValueError("dialect differs from the GRBL 1.1h subset")
        if (
            len(self.tools) != 1
            or not 1 <= self.tools[0].number <= 255
            or self.tools[0].length_register != 1
            or self.arcs
            or self.helices
            or self.tool_changes
            or self.tool_change_position_mode != "preserve_tip"
        ):
            raise ValueError(
                "GRBL: one preloaded tool (T1..255, length slot 1), lines only, no tool changes, the tip stays put"
            )
        for value in (*self.work_offset_machine_mm, *self.initial_machine_mm):
            if not (math.isfinite(value) and abs(value) <= 1e6) or len(self.work_offset_machine_mm) != 3:
                raise ValueError("GRBL needs finite three-axis work offset and initial machine positions")


def define_machine(
    name, tool, *, work_offset_machine_mm, initial_machine_mm, beam_s_max=None, beam_mark=None, **machine
):
    """``tool``: the preloaded tool's ToolSetting (its position_after_mm is the
    program's initial position). ``machine``: units, precision, limits and
    coolant of this machine (camlib assumes none of them). ``beam_mark="power"``
    declares that this laser marks (``ThermalMark``) with its power word."""
    if beam_mark not in (None, "power"):
        raise ValueError('GRBL has no marking process: a laser marks by its power word (beam_mark="power")')
    return GrblMachine(
        name,
        (tool,),
        dialect=_dialect(beam_s_max, beam_mark),
        planes=("xy",),
        arcs=False,
        helices=False,
        max_sweep_rad=math.tau,
        tool_changes=False,
        max_tool_number=255,
        max_length_register=1,
        tool_change_position_mode="preserve_tip",
        work_offset_machine_mm=tuple(work_offset_machine_mm),
        initial_machine_mm=tuple(initial_machine_mm),
        **machine,
    )


class GrblPost(GCodePost):
    version = "grbl-1.1h-1"

    def number(self, value, *, dimensional=True):
        # GRBL reads at most eight digits of a number: drop decimals until the
        # written value fits, so what it reads is what was written.
        if not math.isfinite(value) or abs(value) > 1e6:
            raise ValueError("invalid post number")
        value = value / (25.4 if dimensional and self.machine.output_units == "inch" else 1)
        for places in range(self.machine.decimals, -1, -1):
            text = f"{value:.{places}f}".rstrip("0").rstrip(".")
            text = "0" if text in ("", "-0") else text
            if sum(c.isdigit() for c in text) <= 8:
                return text
        raise ValueError("GRBL eight-digit numeric budget exceeded")

    def emit(self, *words):
        super().emit(*words)
        if len(self._lines[-1]) > _LINE_LIMIT:
            raise ValueError(f"GRBL lines are at most {_LINE_LIMIT} characters")

    def on_motion(self, motion):
        # A move vertical in the plan is written with Z only: GRBL keeps the
        # X and Y it holds. Rewriting them could not reproduce its binary32
        # values (its 8-digit reader rounds again), so a retract could step
        # sideways off the surface it leaves.
        c = motion.curve
        source = getattr(getattr(self, "source_event", None), "curve", None)
        position = getattr(self, "position_mm", None)
        vertical = isinstance(c, Line) and (
            c.start[:2] == c.end[:2] or (isinstance(source, Line) and source.start[:2] == source.end[:2])
        )
        if (
            vertical
            and position is not None
            and all(abs(position[k] - c.end[k]) <= self.machine.tolerance_mm / 2 for k in (0, 1))
        ):
            d = self.machine.dialect
            rapid = motion.kind in ("rapid", "shift") and motion.feed_mm_min is None
            feed = () if rapid else (self.word(d.feed_word, motion.feed_mm_min),)
            self.emit(d.code("rapid" if rapid else "linear"), self.word(d.axes[2], c.end[2]), *feed)
            return
        super().on_motion(motion)

    def on_tool_change(self, event):
        # The preloaded tool: its number only (GRBL has no M6 or H offsets).
        self.emit(self.machine.dialect.tool_word + str(event.number))

    def on_stop(self, event):
        if event.optional:
            raise ValueError("GRBL ignores M1: optional stops are not supported")
        super().on_stop(event)


class GrblDecoder(GCodeDecoder):
    """Reads posted GRBL blocks with GRBL's own arithmetic."""

    version = "grbl-1.1h-binary32-1"

    def __init__(self, machine, program):
        super().__init__(machine, program)
        start = tuple(
            native.grbl_start_mm(m, o) for m, o in zip(machine.initial_machine_mm, machine.work_offset_machine_mm)
        )
        if math.dist(start, program.initial_position_mm) > machine.tolerance_mm:
            raise ValueError("GRBL initial binary32 machine/work-offset pose disagrees with the program start")
        self.position = start

    def read_line(self, line, context=None):
        if len(line) > _LINE_LIMIT:
            raise ValueError(f"GRBL lines are at most {_LINE_LIMIT} characters")

        def grbl(match):
            word, number = match.group(1), match.group(2)
            if word.upper() in "GMN":
                return match.group(0)
            return word + repr(native.grbl_number(number))

        output = super().read_line(_WORD.sub(grbl, line), context)
        if self.tool_pending and self.active is not None:
            # T selects the preloaded tool; its length is metadata slot 1.
            output.append(self._length(self.active.length_register))
        return output

    def _motion(self, effect, values, context):
        if effect not in ("rapid", "linear"):
            raise ValueError("GRBL is posted with lines only")
        d = self.machine.dialect
        self.validate_initialized()
        if context is None or self.active is None or self.length is None or self.offset is None or self.tool_pending:
            raise ValueError("NC motion with unknown attribution/tool/length/offset")
        used = set()
        start = self.position
        end = list(start)
        for i, axis in enumerate(d.axes):
            if axis in values:
                end[i] = native.grbl_axis_mm(values[axis], self.units, self.machine.work_offset_machine_mm[i])
                used.add(axis)
        end = tuple(end)
        feed = None
        if effect == "linear":
            used.add(d.feed_word)
            if values.get(d.feed_word, 0) <= 0:
                raise ValueError("NC motion needs positive explicit feed")
            feed = native.grbl_feed_mm_min(values[d.feed_word], self.units)
        if set(values) - used:
            raise ValueError("unused/unsupported NC words: " + str(sorted(set(values) - used)))
        motion = Motion(
            context["id"],
            context["operation"],
            self.active.tool,
            Line(start, end),
            feed,
            _kind(effect == "rapid", context["kind"]),
        )
        self.position = end
        return None if end == start else motion
