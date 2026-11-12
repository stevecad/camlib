"""WinCNC plasma post (WinCNC 3.x with its torch height control).

References: WinCNC Users Guide 3.1r5 (Command Reference: G0-G4, G20/G21/G22,
G40, G49, G90, G92; Plasma Torch Height Control, p. 53-56; comment
characters) and WinCNC's own plasma posts (SheetCAM "WinCNC Plasma THC",
Fusion "wincnc_plasma.cps"). See docs/wincnc-plasma-post.md.

Program shape (as WinCNC's posts write it):

    G90 G22 ...                    absolute, millimetres (G21 is CENTIMETRES on WinCNC)
    M65                            start macro (plasma setup)
    G0 X Y                         to the pierce point
    M61                            torch on: touch, pierce height, pierce delay,
                                   cut height, THC per the material setup
    G1/G2/G3 X Y [I J] F           the cut (I/J always relative)
    M62                            torch off
    ... M66                        end macro

Premises (declared, not checked on the machine):
- M61-M66 are user-defined macros in CNC.MAC; the defaults are those of
  WinCNC's posts. Pass your machine's codes to define_machine.
- ThermalCut with height_mode="fixed", pierce_delay_s=0 and
  height_control=False: pierce delay, heights and THC belong to the WinCNC
  material setup. Start the job with the torch at the cut height, so no Z
  word is written.
- WinCNC arcs are XY only (no plane words); its velocity smoothing (G9)
  must keep the path within the machine tolerance.
- Arcs need exactly consistent words: post with
  lowering=LoweringOptions(native_arcs=True). At most half a turn per arc
  (as WinCNC's Fusion post).
No controller connection; verify on the machine before cutting.
"""

from dataclasses import dataclass, replace
import math

from camlib.post import Machine, GCodeDialect, GCodePost, NativeGCodeDecoder

_REFERENCE = 'WinCNC Users Guide 3.1r5; WinCNC plasma posts (SheetCAM, Fusion)'


def _dialect(end):
    commands = (
        GCodeDialect()
        .with_commands(
            units_mm='G22',
            units_inch='G20',
            center_incremental=None,
            plane_xz=None,
            plane_yz=None,
            feed_per_minute=None,
            rpm_mode=None,
            cycles_off=None,
            exact_stop=None,
            frame_reset='G92',
            length_on=None,
            tool_change=None,
            spindle_ccw=None,
            mist=None,
            flood=None,
            coolant_off=None,
            stop=None,
            optional_stop=None,
            end=end,
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
        )
        .commands
    )
    # I/J are always relative; feeds are units per minute; no canned cycles;
    # exact path within tolerance (smoothing setting premise).
    return replace(
        GCodeDialect(),
        commands=commands,
        parentheses_comments=True,
        radius_tolerance_mm=0,
        fixed_modes=('center_incremental', 'feed_per_minute', 'rpm_mode', 'cycles_off', 'exact_stop'),
        beam_s_max=1,
    )


@dataclass(frozen=True, kw_only=True)
class WinCNCPlasmaMachine(Machine):
    start_code: str = 'M65'
    torch_on_code: str = 'M61'
    torch_off_code: str = 'M62'
    end_code: str = 'M66'
    reference: str = _REFERENCE

    def __post_init__(self):
        super().__post_init__()
        codes = (self.start_code, self.torch_on_code, self.torch_off_code, self.end_code)
        if len(set(codes)) != 4 or any(not (c.startswith('M') and c[1:].isdigit()) for c in codes):
            raise ValueError('declare four distinct M macro codes (start, torch on, torch off, end)')
        if len(self.tools) != 1 or self.planes != ('xy',) or self.max_sweep_rad > math.pi:
            raise ValueError('WinCNC plasma: one torch, XY arcs of at most half a turn')
        if self.dialect != _dialect(self.end_code) or self.flood or self.mist:
            raise ValueError('dialect differs from the WinCNC plasma subset')


def define_machine(
    name, torch, *, start_code='M65', torch_on_code='M61', torch_off_code='M62', end_code='M66', **settings
):
    """``torch``: the torch's ToolSetting. The macro codes are those of your
    CNC.MAC (defaults: WinCNC's own plasma posts)."""
    return WinCNCPlasmaMachine(
        name,
        (torch,),
        dialect=_dialect(end_code),
        flood=False,
        mist=False,
        planes=('xy',),
        max_sweep_rad=math.pi,
        tool_change_position_mode='preserve_tip',
        start_code=start_code,
        torch_on_code=torch_on_code,
        torch_off_code=torch_off_code,
        end_code=end_code,
        **settings,
    )


class WinCNCPlasmaPost(GCodePost):
    version = 'wincnc-3.1-plasma-1'

    def words(self, addresses, values, *, dimensional=True):
        # As WinCNC's posts: dimensional words always with a decimal point.
        result = super().words(addresses, values, dimensional=dimensional)
        return [w if not dimensional or '.' in w else w + '.' for w in result]

    def on_begin(self, program):
        d = self.machine.dialect
        self.geometry_tolerance_mm = program.tolerance_mm
        self.emit('G90')
        self.emit(d.code('units_mm' if self.machine.output_units == 'mm' else 'units_inch'))
        # G92 (no local coordinates) is written once, as the work offset's
        # block, right after this header and before any motion.
        self.emit('G40')
        self.emit('G49')
        # WinCNC arcs are XY only and it has no plane words: XY is the plane.
        self._plane = 'xy'
        self._spindle_on = False
        self._coolant_mode = 'off'

    def on_work_offset(self, event):
        # WinCNC's work coordinates with no local offset (G92 alone). Its
        # G54 selects a Z head, not a work system, and is never written.
        if event.number != 1:
            raise ValueError('WinCNC plasma: work offset 1 only')
        self.emit('G92')

    def on_tool_change(self, event):
        # The start macro sets up the plasma torch.
        self.emit(self.machine.start_code)

    def on_beam(self, event):
        m = self.machine
        self.emit(m.torch_off_code if event.power_percent == 0 else m.torch_on_code)
        self._spindle_on = event.power_percent != 0

    def on_height_control(self, event):
        raise ValueError(
            'WinCNC plasma: torch height control follows the material setup (M61 starts it '
            'by its THC mode and active delay); plan with height_control=False'
        )

    def on_dwell(self, event):
        raise ValueError(
            'WinCNC plasma: the pierce delay belongs to the material setup (M61); plan with pierce_delay_s=0'
        )

    def on_motion(self, motion):
        c = motion.curve
        if c.start[2] != c.end[2]:
            raise ValueError(
                'WinCNC plasma programs carry no Z: plan with height_mode="fixed" and start '
                'the job with the torch at the cut height'
            )
        super().on_motion(motion)
        z = self.machine.dialect.axes[2]
        # No Z words.
        self._lines = [' '.join(w for w in line.split(' ') if not w.startswith(z)) for line in self._lines]

    def on_finish(self):
        # As WinCNC's posts: the end macro alone (no length offset was ever
        # turned on: the start macro only sets up the torch).
        if getattr(self, '_spindle_on', False):
            self.emit(self.machine.torch_off_code)
        self.emit(self.machine.end_code)


class WinCNCPlasmaDecoder(NativeGCodeDecoder):
    """Reads the WinCNC plasma words back into the generic decoder's model."""

    def __init__(self, machine, program):
        super().__init__(machine, program)
        # WinCNC: XY arcs only, and the torch macros own spindle-like and
        # coolant state (none is on at the start).
        self.plane = 'xy'
        self.ready |= {'plane', 'spindle_off', 'coolant_off'}

    def validate_initialized(self):
        # The frame reset (G92) is the work offset's own block: until that
        # block it may be pending; every motion needs it (offset set).
        if self.offset is None and 'frame_reset' not in self.ready:
            self.ready.add('frame_reset')
            try:
                super().validate_initialized()
            finally:
                self.ready.discard('frame_reset')
            return
        super().validate_initialized()

    def read_line(self, line, context=None):
        from camlib.motion import WorkOffset

        m = self.machine
        text = line.strip()
        if text == 'G92':
            # No local coordinates: WinCNC's work coordinates, offset 1.
            self.ready.add('frame_reset')
            self.offset = 1
            return [WorkOffset(1)]
        if text == m.start_code:
            # The plasma setup macro selects the torch.
            torch = m.tools[0]
            self.selected = torch.number
            self._activate(torch.number)
            return [self._length(torch.length_register)]
        if text == m.torch_on_code:
            return super().read_line('M3 S1', context)
        if text == m.torch_off_code:
            return super().read_line('M5', context)
        if text == m.end_code:
            # The torch was selected by a macro, not a length offset.
            self.length = None
            return super().read_line(line, context)
        return super().read_line(line, context)
