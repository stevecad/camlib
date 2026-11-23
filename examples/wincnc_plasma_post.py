"""WinCNC plasma post (WinCNC 3.x with its torch height control).

References: WinCNC Users Guide 3.1r5 (Command Reference: G0-G4, G20/G21/G22,
G40, G49, G90, G92, L36 "Torch Height Control On", L37 "Torch Height Control
Off"; Plasma Torch Height Control, p. 53-56; comment characters) and WinCNC's own plasma posts (SheetCAM "WinCNC Plasma THC",
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
- A table whose own programs have no start macro and are run in the local
  coordinates the operator sets at the sheet (ShopSabre plasma tables with
  ProNest's post: `G90`, the cuts between M61 and M62, `M66`) is declared
  with start_code=None and local_coordinates="keep": no M65 and no G92 are
  written, so the operator's sheet zero stays in force. Each is replaced by
  a comment that states the premise to the operator (and to the decoder):
  "(WORK ZERO AS SET AT THE MACHINE)" and "(TORCH <id>)".
- material_feed_mm_min: the "Feed Rate" of the material setup chosen at the
  control (Users Guide, Plasma Torch Height Control: "XY axes feed rate for
  this material"). A program with no F word cuts at that feed, which is how
  ProNest's programs for these tables are written. Declared, no F word is
  written, and a cut planned at any other feed (a small-hole or corner
  feed) is refused: the program cannot say it. Left undeclared, every
  cutting move carries its F word and overrides the material's feed.
- program_height_control=True: ThermalCut(height_control=True) is posted
  with WinCNC's L36/L37 (height control on over each loop's cut, off over
  leads, small holes and slowed corners), as ProNest's programs for these
  tables carry them. L37 is written before the first torch-on, so the
  program does not depend on the state it starts in, and L36 before the
  end macro, so it leaves height control on as it is found. Without it a
  height control command is refused (M61 runs it by the material setup).
- ThermalCut with height_mode="fixed", pierce_delay_s=0 and
  height_control=False: pierce delay, heights and THC belong to the WinCNC
  material setup. Start the job with the torch at the cut height, so no Z
  word is written.
- edge_start_cut_z_mm: opt in to the native edge-start sequence documented
  by WinCNC's public demo plasma macros. The selected material's THR must
  represent this Z in the plan's coordinates, and sheet Z must already be
  established. Requires program_height_control=True. L90 G0 Z{THR} returns
  to cut height, ~L38 fires without touch-off, and G4 D supplies the edge
  delay. With material feed declared, F{THF} selects it at the edge start.
- WinCNC arcs are XY only (no plane words); its velocity smoothing (G9)
  must keep the path within the machine tolerance.
- control_kerf: how the control applies the kerf of a
  ThermalCut(compensation="control") program, which carries the part side
  of the kerf path. "g41": the program says it, `G41 O<kerf/2>` on a block
  of its own before the straight lead-in and `G40` on its own before the
  straight lead-out (Users Guide, Controller 3.1, revision 3.1.04,
  pp. 138-139; ShopSabre's own plasma post writes it so). "material": the
  owner declares that the control applies half the kerf of the material
  setup chosen at it (material_kerf_mm) to the left of travel with no word
  in the program, from the first move after M61 to the last before M62.
  None (default): compensation="control" is refused. The cutcomp settings
  of WINCNC.INI (Users Guide p. 72: T0, no entry move inserted, the
  default; a positive D) are premises; cutcomp_arc_radius_mm and
  cutcomp_arc_angle_deg (R and C, defaults 200 in and 0.5 degrees) and
  cutcomp_full_circle_mm (F, Versions 3.1.07b) are replayed by the decoder.
- Arcs: post with lowering=LoweringOptions(native_arcs=True). At most half
  a turn per arc (as WinCNC's Fusion post). WinCNC takes arcs whose start
  and end radius differ by up to its arc_err setting (WINCNC.INI; Users
  Guide 3.1r5, System Configuration: default .01 unit of measure); declare
  arc_radius_tolerance_mm (at most that, and at most 0.005 mm) and arcs
  whose rounded words differ that little stay arcs. Left at 0, only
  exactly consistent words are written and other arcs become checked
  lines.
No controller connection; verify on the machine before cutting.
"""

from dataclasses import dataclass, replace
import math

from camlib.post import Machine, GCodeDialect, GCodePost, NativeGCodeDecoder

_REFERENCE = 'WinCNC Users Guide 3.1r5; WinCNC plasma posts (SheetCAM, Fusion)'


_KEEP_LOCAL = '(WORK ZERO AS SET AT THE MACHINE)'
# WinCNC Users Guide 3.1r5, Command Reference: L36 Torch Height Control On,
# L37 Torch Height Control Off.
_THC_ON, _THC_OFF = 'L36', 'L37'


def _torch_comment(machine):
    return f'(TORCH {machine.tools[0].tool.upper()})'


def _dialect(end, arc_radius_tolerance_mm=0):
    # G41/G42 O#: "O# is the compensation amount on machines without a tool
    # changer" (Users Guide, Controller 3.1, revision 3.1.04, pp. 138-139).
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
        radius_tolerance_mm=arc_radius_tolerance_mm,
        radius_relative_tolerance=0,
        arc_word_mismatch_mm=arc_radius_tolerance_mm,
        fixed_modes=('center_incremental', 'feed_per_minute', 'rpm_mode', 'cycles_off', 'exact_stop'),
        beam_s_max=1,
        compensation_amount_word='O',
    )


@dataclass(frozen=True, kw_only=True)
class WinCNCPlasmaMachine(Machine):
    start_code: str | None = 'M65'
    # "cancel": G92 alone, which removes local coordinates (the program runs
    # in WinCNC's work coordinates). "keep": nothing is written, and the
    # program runs in the local coordinates the operator has set.
    local_coordinates: str = 'cancel'
    # Height control switched from the program with L36/L37.
    program_height_control: bool = False
    # The material setup's feed rate: no F word is written, and every cut
    # must be planned at this feed.
    material_feed_mm_min: float | None = None
    # Opt in to native edge starts. THR in the selected material must put
    # the torch at this Z in the plan's coordinates; sheet Z must already
    # be established. This is not the controller's local Z value.
    edge_start_cut_z_mm: float | None = None
    # The start/end radius mismatch an arc's words may have: WinCNC takes
    # arcs within its arc_err setting (WINCNC.INI, "allowable errors in arc
    # specifications", default .01 unit of measure). 0: exactly consistent
    # words only, and arcs that have none are posted as checked lines.
    arc_radius_tolerance_mm: float = 0
    # How the control applies the kerf of a compensation="control" program:
    # "g41" (G41 O<kerf/2> in the program), "material" (the owner declares
    # the control applies its material setup's kerf, material_kerf_mm, to
    # the left of travel with no word), or None (refused).
    control_kerf: str | None = None
    material_kerf_mm: float | None = None
    # WINCNC.INI cutcomp R# and C# (Users Guide p. 72, defaults 200 in and
    # 0.5 degrees): an arc of a larger radius turning less is cut as a line
    # under compensation. F# (Versions 3.1.07b; no documented default): an
    # arc whose ends are closer is taken as a full circle. None: a
    # compensated arc is refused until it is declared (0 when not set).
    cutcomp_arc_radius_mm: float = 200 * 25.4
    cutcomp_arc_angle_deg: float = 0.5
    cutcomp_full_circle_mm: float | None = None
    torch_on_code: str = 'M61'
    torch_off_code: str = 'M62'
    end_code: str = 'M66'
    reference: str = _REFERENCE

    def __post_init__(self):
        super().__post_init__()
        codes = (self.torch_on_code, self.torch_off_code, self.end_code)
        codes += () if self.start_code is None else (self.start_code,)
        if len(set(codes)) != len(codes) or any(
            not (isinstance(c, str) and c.startswith('M') and c[1:].isdigit()) for c in codes
        ):
            raise ValueError(
                'declare distinct M macro codes (torch on, torch off, end, and the start macro or start_code=None)'
            )
        if self.material_feed_mm_min is not None and not (
            math.isfinite(self.material_feed_mm_min) and 0 < self.material_feed_mm_min <= self.max_feed_mm_min
        ):
            raise ValueError("material_feed_mm_min must be positive and within the machine's max_feed_mm_min")
        if self.local_coordinates not in ('cancel', 'keep'):
            raise ValueError('local_coordinates must be "cancel" (G92 is written) or "keep" (it is not)')
        if self.edge_start_cut_z_mm is not None and (
            not math.isfinite(self.edge_start_cut_z_mm) or not self.program_height_control
        ):
            raise ValueError('edge_start_cut_z_mm needs a finite cut Z and program_height_control=True')
        if len(self.tools) != 1 or self.planes != ('xy',) or self.max_sweep_rad > math.pi:
            raise ValueError('WinCNC plasma: one torch, XY arcs of at most half a turn')
        if self.dialect != _dialect(self.end_code, self.arc_radius_tolerance_mm) or self.flood or self.mist:
            raise ValueError('dialect differs from the WinCNC plasma subset')
        if self.control_kerf not in (None, 'g41', 'material'):
            raise ValueError(
                'control_kerf must be "g41" (G41 O<kerf/2> in the program), "material" (the control applies '
                'its material setup\'s kerf with no word) or None'
            )
        if (self.control_kerf == 'material') != (self.material_kerf_mm is not None) or (
            self.material_kerf_mm is not None
            and not (math.isfinite(self.material_kerf_mm) and 0 < self.material_kerf_mm <= 100)
        ):
            raise ValueError('control_kerf="material" needs the material setup\'s kerf: a positive material_kerf_mm')
        if not (
            math.isfinite(self.cutcomp_arc_radius_mm)
            and self.cutcomp_arc_radius_mm > 0
            and math.isfinite(self.cutcomp_arc_angle_deg)
            and 0 <= self.cutcomp_arc_angle_deg < 180
            and (
                self.cutcomp_full_circle_mm is None
                or math.isfinite(self.cutcomp_full_circle_mm)
                and self.cutcomp_full_circle_mm >= 0
            )
        ):
            raise ValueError('cutcomp settings: a positive R, an angle C of 0..180 degrees and F of zero or more')

    @property
    def compensation_unwritten(self):
        """The control applies the kerf from its material setup, with no
        word in the program (control_kerf="material", the owner's
        declaration)."""
        return self.control_kerf == 'material'


def define_machine(
    name,
    torch,
    *,
    start_code='M65',
    torch_on_code='M61',
    torch_off_code='M62',
    end_code='M66',
    local_coordinates='cancel',
    program_height_control=False,
    arc_radius_tolerance_mm=0,
    material_feed_mm_min=None,
    edge_start_cut_z_mm=None,
    control_kerf=None,
    material_kerf_mm=None,
    cutcomp_arc_radius_mm=200 * 25.4,
    cutcomp_arc_angle_deg=0.5,
    cutcomp_full_circle_mm=None,
    **settings,
):
    """``torch``: the torch's ToolSetting. The macro codes are those of your
    CNC.MAC (defaults: WinCNC's own plasma posts). ``start_code=None``: the
    table has no start macro. ``local_coordinates="keep"``: no G92 is
    written, so the program runs in the local coordinates the operator set
    at the sheet. ``program_height_control=True``: height control is
    switched from the program with L36/L37 (plan with height_control=True).
    ``arc_radius_tolerance_mm``: the start/end radius mismatch arc words may
    have, at most the table's arc_err (WINCNC.INI) and at most 0.005 mm.
    ``material_feed_mm_min``: the feed rate of the material setup chosen at
    the control; every cut must be planned at it.
    ``edge_start_cut_z_mm``: opt in to native edge starts at the declared Z
    in plan coordinates. The selected material's THR must represent that
    height and sheet Z must already be established. Requires
    program_height_control=True; see docs/wincnc-plasma-post.md.
    ``control_kerf``: how the control applies the kerf of a
    ThermalCut(compensation="control") program: "g41" (``G41 O<kerf/2>``
    written, WinCNC Users Guide 3.1.04 pp. 138-139) or "material" (the
    control applies its material setup's kerf, ``material_kerf_mm``, to the
    left of travel with no word: the owner's declaration). The cutcomp
    settings of WINCNC.INI: ``cutcomp_arc_radius_mm`` and
    ``cutcomp_arc_angle_deg`` (R, C) and ``cutcomp_full_circle_mm`` (F, 0
    when not set; needed for compensated arcs)."""
    return WinCNCPlasmaMachine(
        name,
        (torch,),
        dialect=_dialect(end_code, arc_radius_tolerance_mm),
        flood=False,
        mist=False,
        planes=('xy',),
        max_sweep_rad=math.pi,
        tool_change_position_mode='preserve_tip',
        start_code=start_code,
        local_coordinates=local_coordinates,
        program_height_control=program_height_control,
        arc_radius_tolerance_mm=arc_radius_tolerance_mm,
        material_feed_mm_min=material_feed_mm_min,
        edge_start_cut_z_mm=edge_start_cut_z_mm,
        control_kerf=control_kerf,
        material_kerf_mm=material_kerf_mm,
        cutcomp_arc_radius_mm=cutcomp_arc_radius_mm,
        cutcomp_arc_angle_deg=cutcomp_arc_angle_deg,
        cutcomp_full_circle_mm=cutcomp_full_circle_mm,
        torch_on_code=torch_on_code,
        torch_off_code=torch_off_code,
        end_code=end_code,
        **settings,
    )


class WinCNCPlasmaPost(GCodePost):
    version = 'wincnc-3.1-plasma-2'

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
        self._edge_position = tuple(program.initial_position_mm)
        self._edge_delay = False

    def on_work_offset(self, event):
        # WinCNC's work coordinates with no local offset (G92 alone). Its
        # G54 selects a Z head, not a work system, and is never written.
        if event.number != 1:
            raise ValueError('WinCNC plasma: work offset 1 only')
        if self.machine.local_coordinates == 'cancel':
            self.emit('G92')
        else:
            # Nothing changes the coordinates: the program says so.
            self.emit(_KEEP_LOCAL)

    def on_tool_change(self, event):
        # The start macro sets up the plasma torch (a table without one has
        # the torch set up already: it is its only tool).
        if self.machine.start_code is not None:
            self.emit(self.machine.start_code)
        else:
            self.emit(_torch_comment(self.machine))
        self._edge_position = tuple(event.position_after_mm)

    def on_beam(self, event):
        m = self.machine
        self._edge_delay = False
        if event.power_percent == 0:
            self._stretch_open = False
        if event.power_percent != 0 and m.program_height_control and getattr(self, '_thc', None) is None:
            # Height control is off over a lead-in: before the first cut
            # that is the program's to say, whatever state it starts in.
            self.emit(_THC_OFF)
            self._thc = False
        self.emit(m.torch_off_code if event.power_percent == 0 else m.torch_on_code)
        self._spindle_on = event.power_percent != 0

    def on_beam_start(self, event):
        m = self.machine
        if m.edge_start_cut_z_mm is None:
            return super().on_beam_start(event)
        if self._edge_position[2] != m.edge_start_cut_z_mm:
            raise ValueError('WinCNC edge start: planned Z differs from edge_start_cut_z_mm')
        if event.power_percent != 100:
            raise ValueError('WinCNC edge start: the material setup supplies full torch power (100 percent)')
        # WinCNC's public demo: torchon.mac calls M63 (touch) BEFORE L38,
        # and waits for arc-good / pierce delay AFTER it. torchcut.mac uses
        # L90 G0 Z{THR}. Use those native primitives without the touch or
        # pierce steps. The tilde bypasses a CNC.MAC override (Guide p.112).
        # https://www.wincnc.net/webfiles/Demo/WinDemo.exe
        if getattr(self, '_thc', None) is not False:
            self.emit(_THC_OFF)
        self._thc = False
        self.emit('L90 G0 Z{THR}')
        self.emit('~L38')
        if m.material_feed_mm_min is not None:
            # M61 normally establishes this feed; an edge start bypasses it.
            self.emit('F{THF}')
        self._spindle_on = True
        self._edge_delay = True

    def on_cutter_compensation(self, event):
        m = self.machine
        if event.side != 'off' and event.amount_mm is not None:
            if m.control_kerf is None:
                raise ValueError(
                    'WinCNC plasma: declare how the control applies the kerf of a compensation="control" '
                    'program: control_kerf="g41" (G41 O<kerf/2> written, Users Guide 3.1.04 pp. 138-139) or '
                    '"material" (the control applies its material setup\'s kerf with no word)'
                )
            if m.control_kerf == 'material':
                if event.side != 'left':
                    raise ValueError(
                        'WinCNC plasma: this control applies its kerf to the left of travel: plan with '
                        'reverse=False (outlines clockwise, holes counter-clockwise)'
                    )
                if abs(2 * event.amount_mm - m.material_kerf_mm) > m.tolerance_mm:
                    raise ValueError(
                        f'WinCNC plasma: the plan\'s kerf ({2 * event.amount_mm:g} mm) differs from the material '
                        f'setup\'s ({m.material_kerf_mm:g} mm, material_kerf_mm): plan with the torch\'s kerf as '
                        'the control applies it'
                    )
            self._stretch_open = True
        elif event.side != 'off' and m.control_kerf == 'material':
            # Wear compensation (G41/G42 with the D register) on a control that
            # offsets every cut by its material setup's kerf: it would offset
            # twice (#391).
            raise ValueError(
                'WinCNC plasma: this control applies its material setup\'s kerf itself (control_kerf="material"): '
                'plan the cut with compensation="control", not wear compensation'
            )
        return super().on_cutter_compensation(event)

    def on_beam_mark(self, event):
        raise ValueError(
            'WinCNC plasma: this post declares no marking method (M61 fires the torch with the '
            'cutting material setup); a ThermalMark cannot be posted for it'
        )

    def on_height_control(self, event):
        if not self.machine.program_height_control:
            raise ValueError(
                'WinCNC plasma: torch height control follows the material setup (M61 starts it '
                'by its THC mode and active delay); plan with height_control=False, or declare '
                'program_height_control=True to switch it with L36/L37'
            )
        self.emit(_THC_ON if event.on else _THC_OFF)
        self._thc = event.on

    def on_dwell(self, event):
        if self._edge_delay:
            from decimal import Decimal

            # Guide p.137: G4 D is seconds. Preserve the planned delay;
            # it is distinct from the material setup's pierce delay.
            # WinCNC numeric words use decimal notation, not Python's
            # exponent notation for small values.
            self.emit(f'G4 D{Decimal(str(event.seconds)):f}')
            self._edge_delay = False
            return
        raise ValueError(
            'WinCNC plasma: the pierce delay belongs to the material setup (M61); plan with pierce_delay_s=0'
        )

    def on_motion(self, motion):
        self._edge_delay = False
        c = motion.curve
        if c.start[2] != c.end[2]:
            raise ValueError(
                'WinCNC plasma programs carry no Z: plan with height_mode="fixed" and start '
                'the job with the torch at the cut height'
            )
        m = self.machine
        material = m.material_feed_mm_min
        if (
            material is not None
            and motion.kind != 'rapid'
            and abs(motion.feed_mm_min - material) > m.feed_tolerance_mm_min
        ):
            raise ValueError(
                f'WinCNC plasma: a move is planned at {motion.feed_mm_min:g} mm/min, but this machine cuts at its '
                f"material setup's feed ({material:g} mm/min, material_feed_mm_min) and writes no F word. Plan every "
                'cut at that feed (no small-hole or corner feed), or leave material_feed_mm_min undeclared so the '
                'program states its feeds'
            )
        if (
            m.control_kerf == 'material'
            and getattr(self, '_spindle_on', False)
            and motion.kind != 'rapid'
            and not getattr(self, '_stretch_open', False)
        ):
            raise ValueError(
                'WinCNC plasma: this control offsets every cut by its material kerf (control_kerf="material"): '
                'only ThermalCut(compensation="control") loops can be cut on it, without crop, skeleton, chain '
                'or micro_joints'
            )
        super().on_motion(motion)
        self._edge_position = tuple(c.end)
        d = m.dialect
        # No Z words; no F words where the material setup sets the feed.
        dropped = (d.axes[2], d.feed_word) if material is not None else (d.axes[2],)
        self._lines = [' '.join(w for w in line.split(' ') if not w.startswith(dropped)) for line in self._lines]

    def on_finish(self):
        # As WinCNC's posts: the end macro alone (no length offset was ever
        # turned on: the start macro only sets up the torch).
        if getattr(self, '_spindle_on', False):
            self.emit(self.machine.torch_off_code)
        if getattr(self, '_thc', None) is False:
            # Left on, as ProNest's programs leave it: a program that
            # never switches it runs with height control afterwards.
            self.emit(_THC_ON)
        self.emit(self.machine.end_code)


class WinCNCPlasmaDecoder(NativeGCodeDecoder):
    """Reads the WinCNC plasma words back into the generic decoder's model."""

    def __init__(self, machine, program):
        super().__init__(machine, program)
        # WinCNC: XY arcs only, and the torch macros own spindle-like and
        # coolant state (none is on at the start).
        self.plane = 'xy'
        self.ready |= {'plane', 'spindle_off', 'coolant_off'}
        self._edge_stage = None
        self._edge_delay = False
        self._thc = None

    def unwritten_compensation(self):
        m = self.machine
        return ('left', m.material_kerf_mm / 2) if m.control_kerf == 'material' else None

    def replay_compensation(self, side, amount_mm, curves):
        from camlib.motion import Arc, Line

        m = self.machine
        written = []
        for c in curves:
            if isinstance(c, Arc):
                if m.cutcomp_full_circle_mm is None:
                    raise ValueError(
                        'WinCNC plasma: declare cutcomp_full_circle_mm (WINCNC.INI cutcomp F#, 0 when it is not '
                        'set): under compensation an arc whose ends are closer is taken as a full circle'
                    )
                if math.dist(c.start[:2], c.end[:2]) < m.cutcomp_full_circle_mm:
                    raise ValueError(
                        f'WinCNC plasma: the compensated arc from {c.start[:2]} to {c.end[:2]} ends closer to its '
                        'start than cutcomp F#: the control would cut a full circle'
                    )
                radius = math.dist(c.start[:2], c.center[:2])
                if radius > m.cutcomp_arc_radius_mm and abs(math.degrees(c.sweep_rad)) < m.cutcomp_arc_angle_deg:
                    # cutcomp R#/C#: cut as a line (Users Guide p. 72).
                    c = Line(c.start, c.end)
            written.append(c)
        return super().replay_compensation(side, amount_mm, written)

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
        from camlib.motion import Beam, BeamStart, Dwell, HeightControl, WorkOffset

        m = self.machine
        text = line.strip()
        if self._edge_stage == 'fire' and text != '~L38':
            raise ValueError('WinCNC edge start: cut-height positioning must be followed by native ~L38')
        if self._edge_stage == 'feed' and text != 'F{THF}':
            raise ValueError('WinCNC edge start: missing material feed F{THF}')
        if text == 'L90 G0 Z{THR}':
            if m.edge_start_cut_z_mm is None or self.rpm or self._thc is not False:
                raise ValueError('WinCNC edge start: declare cut Z, turn the torch off and disable THC first')
            self.validate_initialized()
            if self.position[2] != m.edge_start_cut_z_mm:
                raise ValueError('WinCNC edge start: decoded Z differs from edge_start_cut_z_mm')
            self._edge_stage = 'fire'
            return []
        if text == '~L38':
            if self._edge_stage != 'fire':
                raise ValueError('WinCNC edge start: native ~L38 requires cut-height positioning first')
            events = super().read_line('M3 S1', context)
            if len(events) != 1 or not isinstance(events[0], Beam):
                raise ValueError('WinCNC edge start: active tool must be a torch')
            self._edge_stage = 'feed' if m.material_feed_mm_min is not None else None
            self._edge_delay = True
            return [BeamStart(events[0].power_percent)]
        if text == 'F{THF}':
            if self._edge_stage != 'feed':
                raise ValueError('WinCNC edge start: unexpected material feed')
            self._edge_stage = None
            return []
        if text.startswith('G4 '):
            import re

            match = re.fullmatch(r'G4 D([0-9]+(?:\.[0-9]+)?)', text)
            if not self._edge_delay or not match:
                raise ValueError('WinCNC plasma: only the delay immediately after an edge start may be programmed')
            seconds = float(match[1])
            if not math.isfinite(seconds) or seconds <= 0:
                raise ValueError('WinCNC edge start: delay must be finite and positive')
            self._edge_delay = False
            return [Dwell(seconds)]
        self._edge_delay = False
        words = text.split(' ')
        if words[0] in ('G41', 'G42'):
            # What the machine's declaration rules out (#391): any G41/G42 on a
            # control that applies its material setup's kerf itself (it would
            # offset twice), and an amount word (O) unless the control is
            # declared to take it. Wear compensation, G41/G42 with the head's
            # D register, stays the generic decoder's to check.
            if m.control_kerf == 'material':
                raise ValueError(
                    'WinCNC plasma: this control applies its material setup\'s kerf itself (control_kerf="material"): '
                    'a program for it carries no G41/G42'
                )
            if m.control_kerf != 'g41' and any(w.startswith('O') for w in words[1:]):
                raise ValueError(
                    'WinCNC plasma: G41/G42 with an amount (O) needs control_kerf="g41" (the control offsetting by '
                    'the written amount)'
                )
        if text == 'G92' and m.local_coordinates == 'cancel':
            # No local coordinates: WinCNC's work coordinates, offset 1.
            self.ready.add('frame_reset')
            self.offset = 1
            return [WorkOffset(1)]
        if m.local_coordinates == 'keep' and text == _KEEP_LOCAL:
            # The operator's local coordinates stay in force: offset 1.
            self.ready.add('frame_reset')
            self.offset = 1
            return [WorkOffset(1)]
        if text == 'G92' and m.local_coordinates == 'keep':
            raise ValueError('WinCNC plasma: this machine keeps its local coordinates; G92 would remove them')
        if text == (m.start_code if m.start_code is not None else _torch_comment(m)):
            # The plasma setup macro selects the torch.
            torch = m.tools[0]
            self.selected = torch.number
            self._activate(torch.number)
            return [self._length(torch.length_register)]
        if text in (_THC_ON, _THC_OFF):
            if not m.program_height_control:
                raise ValueError('WinCNC plasma: this machine does not switch height control from the program')
            # With the torch off it only sets the state the next cut starts
            # in; with it on it is the plan's height control command.
            self._thc = text == _THC_ON
            return [HeightControl(self._thc)] if self.rpm else []
        if text == m.torch_on_code:
            return super().read_line('M3 S1', context)
        words = text.split(' ')
        if m.material_feed_mm_min is not None and words[0] in ('G1', 'G2', 'G3'):
            d = m.dialect
            if any(w.startswith(d.feed_word) for w in words[1:]):
                raise ValueError(
                    "WinCNC plasma: this machine cuts at its material setup's feed; an F word overrides it"
                )
            # The controller cuts at the material setup's feed.
            unit = 25.4 if m.output_units == 'inch' else 1
            line = f'{text} {d.feed_word}{m.material_feed_mm_min / unit!r}'
        if text == m.torch_off_code:
            return super().read_line('M5', context)
        if text == m.end_code:
            # The torch was selected by a macro, not a length offset.
            self.length = None
            return super().read_line(line, context)
        return super().read_line(line, context)
