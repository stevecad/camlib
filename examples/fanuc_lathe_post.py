"""FANUC lathe post, G-code system A (0i-TF and similar T-series controls).

Reference: Autodesk's FANUC Turning post (fanuc turning.cps, revision 44226,
the program structure and words FANUC lathes are commonly posted with) and
the conventions of G-code system A: no G90/G91 (X/Z absolute, U/W
incremental; G90 is the outside diameter cutting cycle), G98/G99 feed per
minute/revolution, G50 S spindle limit, G96/G97, G32 F lead, T<tt><oo>.
FANUC's own manual for the control (e.g. B-64604EN) is the authority: check
the premises below against it and the machine builder's documentation.
See docs/fanuc-lathe-post.md.

Program shape:

    %
    O1001
    G21, G18, G98, G97, G40, G80   one code per block
    G54
    T0101                          tool 01 with offset 01
    G50 S3000 / G96 S200 M3        constant surface speed with its limit
    (or M3 S<rpm>)
    G99 F0.25                      feed per revolution
    M8
    G0 X.. / G0 Z..                one axis per rapid
    G1/G2/G3 X Z [I K] F           X a diameter; I, K radius values, incremental
    G32 X Z F<lead>                threading
    M9, M5, M30
    %

Premises (declared, not checked on the machine):
- G-code system A; X words are diameters (parameter DIA, No. 1006#3, set
  for X); I is a radius value.
- Dimensional words are written with a decimal point (without one FANUC
  reads a least-increment count).
- G00 runs each axis at its own rate unless linear-interpolation positioning
  is set (parameter LRP, No. 1401#1): rapids move one axis at a time
  (refused otherwise).
- No local coordinate system (G52) and no tool-nose radius compensation
  active at the start; no smoothing that departs from the programmed path.
- Tool-nose radius compensation (compensation="wear"): the program is the
  nose path already; the offset's R holds only the wear and T the tip
  direction (3 outside, 2 inside for right-hand tools on a rear turret).
- Tool offsets are touched off on a diameter and a face: the program moves
  each tool's imaginary tip (TurningSetup controlled_point="tip").
- The tool change T<tt><oo> needs no turret position: each tool's
  ToolSetting.position_after_mm is the job's start position, where the
  program parks the tool before the change.
No controller connection; verify on the machine before cutting.
"""

from dataclasses import dataclass, replace
import re

from camlib.post import Machine, GCodeDialect, GCodePost, NativeGCodeDecoder

_REFERENCE = "Autodesk FANUC Turning post rev 44226; FANUC G-code system A"


def _dialect():
    commands = (
        GCodeDialect()
        .with_commands(
            absolute=None,
            center_incremental=None,
            feed_per_minute="G98",
            frame_reset=None,
            exact_stop=None,
            length_on=None,
            length_off=None,
            tool_change=None,
            end="M30",
            dwell=None,
            tap_synchronized=None,
            drill_cycle=None,
            drill_dwell_cycle=None,
            peck_cycle=None,
            chip_break_cycle=None,
            cycle_return_initial=None,
            bore_cycle=None,
            bore_dwell_cycle=None,
            bore_stop_cycle=None,
            thread_synchronized="G32",
            css_on="G96",
            css_limit="G50",
            feed_per_rev="G99",
        )
        .commands
    )
    return replace(
        GCodeDialect(),
        commands=commands,
        parentheses_comments=True,
        radius_tolerance_mm=0,
        fixed_modes=("absolute", "center_incremental", "frame_reset", "exact_stop"),
        x_diameter=True,
        tool_change="lathe_t",
        thread_lead_word="F",
        # Tool-nose radius compensation (TurnFinish compensation="wear"):
        # G41/G42 in the block of the entry move, G40 in the retract's; the
        # T offset holds the nose radius (its wear) and tip direction.
        compensation_with_move=True,
        compensation_register=False,
    )


@dataclass(frozen=True, kw_only=True)
class FanucLatheMachine(Machine):
    program_number: int = 1001
    reference: str = _REFERENCE

    def __post_init__(self):
        super().__post_init__()
        if self.dialect != _dialect() or self.planes != ("xz",):
            raise ValueError("dialect differs from the FANUC lathe subset (system A, XZ arcs, diameter X)")
        if self.tool_change_position_mode != "table" or self.traverse_feed_mm_min is not None:
            raise ValueError(
                'declare each tool\'s position after its change (tool_change_position_mode="table"); rapids are G0'
            )
        if type(self.program_number) is not int or not 1 <= self.program_number <= 9999:
            raise ValueError("FANUC program numbers are O1..O9999")


def define_machine(name, tools, *, program_number=1001, **settings):
    """``tools``: ToolSetting per turret station (number = tt, length_register
    = oo), position_after_mm where the program parks the tool for its change."""
    return FanucLatheMachine(
        name,
        tuple(tools),
        dialect=_dialect(),
        planes=("xz",),
        program_number=program_number,
        **{"tool_change_position_mode": "table", **settings},
    )


class FanucLathePost(GCodePost):
    version = "fanuc-lathe-system-a-1"

    def word(self, address, value, *, dimensional=True):
        result = super().word(address, value, dimensional=dimensional)
        return result if not dimensional or "." in result else result + "."

    def words(self, addresses, values, *, dimensional=True):
        # Always a decimal point on dimensional words.
        result = super().words(addresses, values, dimensional=dimensional)
        return [w if not dimensional or "." in w else w + "." for w in result]

    def circular_values(self, curve):
        # The decimal point this post appends leaves every word's value as
        # the standard formatter writes it: arcs get the standard search for
        # exactly consistent end and I/K words (radius_tolerance_mm=0), and
        # an arc without such words is refused (or lowered to lines with
        # LoweringOptions(native_arcs=True)) instead of written inconsistent.
        return self.exact_circular_values(curve)

    def on_begin(self, program):
        d = self.machine.dialect
        self.geometry_tolerance_mm = program.tolerance_mm
        self.emit("%")
        self.emit("O" + str(self.machine.program_number))
        for effect in (
            "units_mm" if self.machine.output_units == "mm" else "units_inch",
            "plane_xz",
            "feed_per_minute",
            "rpm_mode",
            "radius_off",
            "cycles_off",
            "spindle_off",
            "coolant_off",
        ):
            self.emit(d.code(effect))
        self._spindle_on = False
        self._coolant_mode = "off"

    def on_motion(self, motion):
        c = motion.curve
        if c.start[1] != 0 or c.end[1] != 0:
            raise ValueError("a lathe has no Y axis: every move stays at y = 0")
        if motion.kind in ("rapid", "shift") and c.start[0] != c.end[0] and c.start[2] != c.end[2]:
            raise ValueError("G00 may run each axis at its own rate: a lathe rapid moves along X or Z only")
        super().on_motion(motion)
        y = self.machine.dialect.axes[1]
        self._lines = [
            " ".join(w for w in line.split(" ") if not w.startswith(y))
            for line in self._lines
            if line != self.machine.dialect.code("plane_xz")
        ]

    def on_finish(self):
        d = self.machine.dialect
        if getattr(self, "_coolant_mode", None) != "off":
            self.emit(d.code("coolant_off"))
        if getattr(self, "_spindle_on", False):
            self.emit(d.code("spindle_off"))
        self.emit(d.code("end"))
        self.emit("%")


class FanucLatheDecoder(NativeGCodeDecoder):
    """Reads the FANUC lathe words back into the generic decoder's model."""

    version = "fanuc-lathe-system-a-nominal-1"

    def __init__(self, machine, program):
        super().__init__(machine, program)
        # No length codes: the T word applies the offset before any motion.
        self.ready.add("length")
        self.opened = self.numbered = self.closed = False

    def validate_initialized(self):
        super().validate_initialized()
        if not (self.opened and self.numbered):
            raise ValueError("FANUC % and O-number framing is missing")

    def read_line(self, line, context=None):
        d = self.machine.dialect
        text = line.strip()
        if self.closed:
            raise ValueError("NC after the closing %")
        if text == "%":
            if self.opened and self.ended:
                self.closed = True
            elif self.opened:
                raise ValueError("% only opens and closes the program")
            self.opened = True
            return []
        if re.fullmatch(r"O\d{1,4}", text):
            if not self.opened or self.numbered or int(text[1:]) != self.machine.program_number:
                raise ValueError("the O number follows the opening % and matches the machine's program number")
            self.numbered = True
            return []
        # Dimensional words carry a decimal point.
        bare = [w for w in text.split(" ") if w[:1] in ("X", "Z", "I", "K", "F") and "." not in w]
        if bare:
            raise ValueError(f"dimensional word without a decimal point: {bare[0]}")
        words = text.split(" ")
        if words[0] == d.code("rapid"):
            x = next((float(w[1:]) for w in words if w.startswith("X")), None)
            z = next((float(w[1:]) for w in words if w.startswith("Z")), None)
            if (
                x is not None
                and z is not None
                and x * self.units / 2 != self.position[0]
                and z * self.units != self.position[2]
            ):
                raise ValueError("G00 moving X and Z together may not follow a straight line")
        if text == d.code("end"):
            # The offset stays with the tool; the program ends.
            self.length = None
        return super().read_line(line, context)
