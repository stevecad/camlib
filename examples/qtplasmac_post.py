"""LinuxCNC QtPlasmaC plasma post (LinuxCNC 2.9 QtPlasmaC documentation).

Writes ``ThermalCut`` programs as QtPlasmaC expects them and reads its own
output back for checking. Setup premises, declared here and in
docs/qtplasmac-post.md:

- The torch is QtPlasmaC tool 0 (``T0 M6``, ``G43 H0``); camlib's tool table
  entry for it is number 1, length register 1. Z belongs to QtPlasmaC, so
  the length offset does not move the cut.
- QtPlasmaC owns Z: its loader removes Z motion, probes the plate and handles
  pierce and cut heights from the material file. Plan with
  ``height_mode="fixed"`` (one Z, the cut height).
- ``M52 P1`` (paused motion) is in the preamble, as the documentation requires.
  Path blending is set explicitly with ``G64 P<blend_mm>``; the blend must be
  within half of the machine's tolerance (the loader would otherwise insert
  ``G64 P0.1``).
- Torch on ``M3 $0 S1``, off ``M5 $0``; torch height control off/on with
  ``M62 P2``/``M63 P2`` (synchronized with motion).
- Feeds are written as F words; QtPlasmaC warns if they differ from the
  selected material's cut feed rate.
- Marking (``ThermalMark``) with the torch, only on a machine that declares
  its materials: ``mark_material`` and ``cut_material`` are the numbers of
  two entries of this machine's own material file, and ``material_wait_s``
  the wait for a change. QtPlasmaC documents automatic material changes
  (``M190 P<n>`` then ``M66 P3 L3 Q<seconds>``, "in the order shown", then
  the feed and ``M3 $0 S1``) and a material's cut mode "3 = Gouge/Mark"
  with its cut amps (used with PowerMax communications). What the marking
  material holds (current, cut mode, heights, feed) is the machine
  owner's; camlib only selects it before the marks and selects the cutting
  material again before every cut that follows. Without the declaration a
  mark is refused. The scribe (tool 1, ``M3 $1 S1``) and spotting
  (``M3 $2 S1``) are not written.

No controller connection; verify on the machine before cutting.
"""

from dataclasses import dataclass, replace

from camlib.motion import Beam, BeamMark
from camlib.post import Machine, GCodeDialect, GCodePost, NativeGCodeDecoder


def _dialect():
    commands = (
        GCodeDialect(beam_s_max=1, height_control_word='P2')
        .with_commands(
            exact_stop='G64',
            center_incremental=None,
            rpm_mode=None,
            mist=None,
            flood=None,
            height_control_on='M63',
            height_control_off='M62',
        )
        .commands
    )
    return replace(
        GCodeDialect(beam_s_max=1, height_control_word='P2'),
        commands=commands,
        fixed_modes=('center_incremental', 'rpm_mode'),
    )


@dataclass(frozen=True, kw_only=True)
class QtPlasmaCMachine(Machine):
    blend_mm: float = 0.005
    # Marking with the torch: the numbers of this machine's marking and
    # cutting materials (its material file) and the seconds to wait for a
    # material change (the Q of M66 P3 L3). All three, or none (no marking).
    mark_material: int | None = None
    cut_material: int | None = None
    material_wait_s: int | None = None
    reference: str = 'LinuxCNC 2.9 QtPlasmaC documentation'

    def __post_init__(self):
        super().__post_init__()
        materials = (self.mark_material, self.cut_material, self.material_wait_s)
        if any(v is not None for v in materials):
            if (
                any(type(v) is not int for v in materials)
                or self.mark_material < 0
                or self.cut_material < 0
                or self.mark_material == self.cut_material
                or not 1 <= self.material_wait_s <= 60
            ):
                raise ValueError(
                    'QtPlasmaC marking: declare mark_material and cut_material (two different material '
                    'numbers of the machine) and material_wait_s (1..60 whole seconds) together'
                )
        if not 0 < self.blend_mm <= self.tolerance_mm / 2:
            raise ValueError('G64 blend must be positive and within half of tolerance_mm')
        if len(self.tools) != 1 or self.tools[0].number != 1 or self.tools[0].length_register != 1:
            raise ValueError('QtPlasmaC: one torch, entered as number 1 / length register 1 (posted T0 / H0)')
        if self.dialect != _dialect() or self.flood or self.mist:
            raise ValueError('dialect differs from the QtPlasmaC subset')


def define_machine(
    name, tool, *, blend_mm=0.005, mark_material=None, cut_material=None, material_wait_s=None, **settings
):
    """``tool``: the torch ToolSetting (number 1, length register 1).
    ``mark_material``, ``cut_material`` and ``material_wait_s`` declare
    marking with the torch (see the module documentation)."""
    return QtPlasmaCMachine(
        name,
        (tool,),
        dialect=_dialect(),
        flood=False,
        mist=False,
        blend_mm=blend_mm,
        mark_material=mark_material,
        cut_material=cut_material,
        material_wait_s=material_wait_s,
        tool_change_position_mode='preserve_tip',
        **settings,
    )


class QtPlasmaCPost(GCodePost):
    version = 'qtplasmac-2.9-1'

    def on_begin(self, program):
        super().on_begin(program)
        blend = f'{self.machine.blend_mm:.4f}'.rstrip('0')
        self._lines = [f'G64 P{blend}' if line == 'G64' else line for line in self._lines]
        self.emit('M52 P1')

    def on_tool_change(self, event):
        self.emit('T0 M6')
        self.emit('G43 H0')

    def _select_material(self, number):
        """The material change, in the documented order, when the program
        has not selected this material already."""
        if getattr(self, '_material', None) != number:
            self.emit(f'M190 P{number}')
            self.emit(f'M66 P3 L3 Q{self.machine.material_wait_s}')
            self._material = number

    def on_beam(self, event):
        if event.power_percent != 0 and self.machine.cut_material is not None:
            self._select_material(self.machine.cut_material)
        self.emit('M5 $0' if event.power_percent == 0 else 'M3 $0 S1')
        self._spindle_on = event.power_percent != 0

    def on_beam_mark(self, event):
        if self.machine.mark_material is None:
            raise ValueError(
                'this QtPlasmaC machine declares no marking: declare mark_material, cut_material and '
                'material_wait_s (the machine\'s own marking and cutting materials)'
            )
        if event.power_percent != 100:
            raise ValueError(
                'QtPlasmaC fires the torch at the selected material\'s current: plan the mark with power_percent=100'
            )
        self._select_material(self.machine.mark_material)
        self.emit('M3 $0 S1')
        self._spindle_on = True

    def on_finish(self):
        super().on_finish()
        self._lines = ['M5 $0' if line == 'M5' else line for line in self._lines]


class QtPlasmaCDecoder(NativeGCodeDecoder):
    """Reads the QtPlasmaC words back into the generic decoder's subset.
    The material a torch-on runs with is read from the material change
    blocks alone: it decides whether the torch marks or cuts."""

    # The material selected by the program (None: not yet), and one whose
    # change still waits for its M66.
    material = None
    material_pending = None

    def read_line(self, line, context=None):
        text = line.strip()
        if text == 'M52 P1':
            return []
        m = self.machine
        if self.material_pending is not None:
            if text != f'M66 P3 L3 Q{m.material_wait_s}':
                raise ValueError('a QtPlasmaC material change (M190) must be followed by its wait (M66 P3 L3 Q)')
            self.material, self.material_pending = self.material_pending, None
            return []
        if text.startswith(('M190', 'M66')):
            number = text[6:] if text.startswith('M190 P') else ''
            if (
                m.mark_material is None
                or not number.isdigit()
                or int(number) not in (m.mark_material, m.cut_material)
                or self.rpm != 0
            ):
                raise ValueError(
                    'QtPlasmaC material change: only M190 P<declared mark or cut material>, with the torch off'
                )
            self.material_pending = int(number)
            return []
        if text == 'M3 $0 S1' and m.mark_material is not None:
            if self.material is None:
                raise ValueError('QtPlasmaC torch on before the program selected its material')
            events = super().read_line('M3 S1', context)
            if self.material == m.mark_material:
                events = [BeamMark(e.power_percent) if isinstance(e, Beam) else e for e in events]
            return events
        if text.startswith('G64 P'):
            if float(text[5:]) > self.machine.blend_mm + 1e-12:
                raise ValueError('G64 blend exceeds the declared blend_mm')
            return super().read_line('G64', context)
        if text == 'T0 M6':
            return [*super().read_line('T1', context), *super().read_line('M6', context)]
        if text == 'G43 H0':
            return super().read_line('G43 H1', context)
        if text == 'M3 $0 S1':
            return super().read_line('M3 S1', context)
        if text == 'M5 $0':
            return super().read_line('M5', context)
        if '$' in text:
            raise ValueError('unsupported QtPlasmaC spindle selector')
        return super().read_line(line, context)
