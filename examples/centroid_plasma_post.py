"""Centroid CNC12 plasma post (Acorn/AcornSix/Hickory with the Centroid THC).

Reference: Centroid Acorn CNC Plasma Users Manual, CNC12 v5.1+ (rev 54):
program format (p. 3), plasma M-codes (p. 48), plasma G-codes (p. 49) and the
Plasma Intercon program example (p. 94). See docs/centroid-plasma-post.md.

A CNC12 plasma program carries geometry only; the material profile (loaded
by M65) supplies pierce height, pierce delay, cut height, THC mode and the
default feed:

    G21 ... header, M65            load the material profile
    G0 X Y                         to the pierce point (torch off)
    M61                            touch off, pierce, go to cut height (THC per profile)
    G1/G2/G3 X Y [I J] F           the cut
    M62                            torch off, raise to the safe height
    ... M30

Premises (declared here, not checked on the machine):
- ThermalCut with height_mode="fixed", pierce_delay_s=0 and
  height_control=False: Z, the pierce delay and torch height control belong
  to the profile (M61 enables THC by its THC mode and active delay).
- The job starts with the torch at the cut height in the program frame, so
  no Z word is written (the manual's programs have none).
- G0 rapids run with the torch raised by M62; CNC12 may move the axes
  independently, which does not matter over a flat plate (tipped-up parts
  are not modeled).
- I/J are incremental (Parameter 2 bit 0 clear); arcs need exactly
  consistent words (radius_tolerance_mm=0): post with
  lowering=LoweringOptions(native_arcs=True).
No controller connection; verify on the machine before cutting.
"""

from dataclasses import dataclass, replace

from camlib.post import Machine, GCodeDialect, GCodePost, NativeGCodeDecoder

_REFERENCE = 'Centroid Acorn CNC Plasma Users Manual rev 54, CNC12 v5.1+'


def _dialect():
    commands = (
        GCodeDialect()
        .with_commands(
            center_incremental=None,
            rpm_mode=None,
            frame_reset='G52',
            exact_stop='G61',
            end='M30',
            tap_synchronized=None,
            mist=None,
            flood=None,
            dwell=None,
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
    # M61 fires the torch: decoded as the beam at full power (S1 = 100%).
    return replace(
        GCodeDialect(),
        commands=commands,
        parentheses_comments=False,
        radius_tolerance_mm=0,
        fixed_modes=('center_incremental', 'rpm_mode'),
        beam_s_max=1,
    )


@dataclass(frozen=True, kw_only=True)
class CentroidPlasmaMachine(Machine):
    reference: str = _REFERENCE

    def __post_init__(self):
        super().__post_init__()
        if len(self.tools) != 1:
            raise ValueError('CNC12 plasma: one torch (its material profile is loaded by M65)')
        if self.dialect != _dialect() or self.flood or self.mist or self.traverse_feed_mm_min is not None:
            raise ValueError('dialect differs from the CNC12 plasma subset (G0 rapids, no coolant)')
        if self.planes != ('xy',):
            raise ValueError('CNC12 plasma programs are XY only')


def define_machine(name, torch, **settings):
    """``torch``: the torch's ToolSetting (any number; M65 selects it)."""
    return CentroidPlasmaMachine(
        name,
        (torch,),
        dialect=_dialect(),
        flood=False,
        mist=False,
        planes=('xy',),
        tool_change_position_mode='preserve_tip',
        **settings,
    )


class CentroidPlasmaPost(GCodePost):
    version = 'centroid-cnc12-plasma-1'

    def on_begin(self, program):
        super().on_begin(program)
        # G52 with explicit zero shifts cancels any local offset.
        self._lines = [('G52 X0 Y0 Z0' if line == 'G52' else line) for line in self._lines]

    def on_tool_change(self, event):
        # The torch is selected by its material profile.
        self.emit('M65')

    def on_beam(self, event):
        self.emit('M62' if event.power_percent == 0 else 'M61')
        self._spindle_on = event.power_percent != 0

    def on_beam_mark(self, event):
        raise ValueError(
            'CNC12 plasma: this post declares no marking method (M61 runs the cutting profile); '
            'a ThermalMark cannot be posted for it'
        )

    def on_height_control(self, event):
        raise ValueError(
            'CNC12 plasma: torch height control follows the material profile (M61 enables it '
            'by its THC mode and active delay); plan with height_control=False'
        )

    def on_dwell(self, event):
        raise ValueError(
            'CNC12 plasma: the pierce delay belongs to the material profile (M61); plan with pierce_delay_s=0'
        )

    def on_motion(self, motion):
        c = motion.curve
        if c.start[2] != c.end[2]:
            raise ValueError(
                'CNC12 plasma programs carry no Z: plan with height_mode="fixed" and start '
                'the job with the torch at the cut height'
            )
        super().on_motion(motion)
        # Z is unchanged: drop its word (the manual's programs have none).
        z = self.machine.dialect.axes[2]
        self._lines = [' '.join(w for w in line.split(' ') if not w.startswith(z)) for line in self._lines]

    def on_finish(self):
        # As the manual's programs: M30 alone. The torch is already off
        # (M62) and no length offset was ever on (M65 loads a profile).
        if getattr(self, '_spindle_on', False):
            self.emit('M62')
        self.emit(self.machine.dialect.code('end'))


class CentroidPlasmaDecoder(NativeGCodeDecoder):
    """Reads the plasma macros back: M65 selects the torch, M61 fires it,
    M62 turns it off."""

    def read_line(self, line, context=None):
        text = line.partition(';')[0].strip()
        if text == 'M65':
            torch = self.machine.tools[0]
            return [
                *super().read_line(f'T{torch.number}', context),
                *super().read_line('M6', context),
                *super().read_line(f'G43 H{torch.length_register}', context),
            ]
        if text == 'M61':
            return super().read_line('M3 S1', context)
        if text == 'M62':
            return super().read_line('M5', context)
        if text == 'M30':
            # The torch was selected by its profile, not a length offset.
            self.length = None
            return super().read_line(line, context)
        if text == 'G52 X0 Y0 Z0':
            # Cancels any local offset: the frame reset.
            return super().read_line('G52', context)
        return super().read_line(line, context)
