# Postprocessors

A postprocessor turns camlib's typed motion (lines, arcs, helices, tool
changes, spindle, coolant) into G-code for one controller. In camlib it is
ordinary Python: a `Machine` describes the controller and tool table, and a
`GCodePost` subclass decides how each event is written.

Every block a post writes is **decoded again** by an independent reader and
compared with the planned motion before it is accepted. A post that writes the
wrong coordinate, feed, plane or modal state makes the run fail. It cannot
produce a quietly different program.

## Machine crash check at export

`Machine.crash_model` holds a required physical `camlib.machine.MachineModel`
for release through `Job.post_nc` or `PostedRun.export`. Missing checking refuses
by default; `accept={"unchecked_machine"}` is an explicit recorded override.
Both APIs replay actual decoded output against stock before export. See
[the full model contract and limits](machine.md), including mesh fixtures,
head/table axes, lathe equipment, rapid envelopes and tool-change routes.
Low-level `post_events` remains a stockless formatter, not a release gate.

## Machine

```python
import math
from camlib.post import GCodeDialect, Machine, ToolSetting

machine = Machine(
    "My mill",
    tools=(ToolSetting.for_tool(tool, number=1, length_register=1,
                                position_after_mm=(0, 0, 50)),),
    # Required: camlib assumes nothing about the machine.
    dialect=GCodeDialect.linuxcnc(),  # the controller's G/M codes
    output_units="mm",          # or "inch"
    decimals=3,                 # what the controller reads (3 mm / 4 inch on FANUC/Haas)
    max_feed_mm_min=10000, max_spindle_rpm=24000, rapid_mm_min=10000,
    planes=("xy", "xz", "yz"), arcs=True, helices=True, max_sweep_rad=math.tau,
    max_tool_number=99, max_length_register=99, tool_changes=True,
    flood=True, mist=False, tool_change_position_mode="table",
    # Optional: engine accuracy and checks.
    tolerance_mm=0.01,          # total allowed deviation of posted vs planned path
    tip_travel_mm=((-100, -100, -100), (100, 100, 100)),  # travel check
)
```

`tip_travel_mm` is the travel of one tool's tip (machine coordinates). With
tools of different gauge lengths declare the machine's own axis travel
instead, `gauge_travel_mm=((x_min, y_min, z_min), (x_max, y_max, z_max))`
(the spindle gauge line, as the controller's soft limits state it): each
tool's decoded moves are then checked against that box lowered by the tool's
gauge length, and `Machine.verification_limits()` gives the planned-move
checks the same limits. See
[Machine travel with tools of different lengths](python.md).

The decoded words are compared with the travel by the same rule as the
planned moves ([the travel rule](python.md#the-travel-rule)): a position on a
limit is inside, a decoded position beyond one is refused however small the
step. Words are rounded to `decimals`, so a planned position on a limit is
written exactly only when the work offset and the limit are multiples of the
output step; otherwise it can round beyond the limit and the post refuses,
naming the axis, the decoded position and the limit.
An arc whose rounded words leave its end off the circle drawn from the start
about the centre (an endpoint gap) must be inside the limit by that gap: an
arc tangent to a travel limit posts only when its words are exact.

Key settings:

- **`tools`**: the machine tool table. `position_after_mm` is where the tool
  tip is after a tool change (`tool_change_position_mode="table"`, which
  requires it; the other modes do not read it and it may be left out); the next
  operation travels at the higher of that height and its clearance before it
  descends (no rapid moves sideways while descending), so the declared pose
  must match the real machine. Plan a multi-tool job with
  `Setup(tool_change=ToolChangeTable.from_machine(machine))` so each new tool
  starts at its table pose (see [Getting started](getting-started.md#more-than-one-tool)). With
  `tool_change_position_mode="preserve_tip"` the tip is instead assumed to
  return to where it was. With `tool_change_position_mode="z_home"` and
  `tool_change_home_z_mm` (the spindle gauge line's machine Z after M6), M6
  sends Z home and leaves X and Y where they were (Centroid CNC12). Plan the
  job with the matching `Setup(tool_change=ToolChangePosition(gauge_z_mm=...))`
  (the same height in work coordinates: home Z minus the work offset Z), so
  each new tool's first move comes down from there and is checked like any
  other rapid. With `tool_change_position_mode="preserve_spindle"` no axis
  moves at the tool change: the spindle gauge line stays, so the new tip is
  the old tip moved along Z by the old gauge length minus the new one. Plan
  with `Setup(tool_change=ToolChangeInPlace())`, or with
  `ToolChangeInPlace(retract_gauge_z_mm=...)` to have the program retract Z
  to a stated height before each change (see
  [LinuxCNC tool changes](#linuxcnc-tool-changes)). On a lathe,
  `tool_change_position_mode="preserve_carriage"` declares a `T<nn><oo>`
  word that moves no axis and only applies the new tool's offset: the
  controlled point moves by the new tool's `ToolSetting.carriage_to_tip_mm`
  (x as a radius, z) minus the old one's, which the mode requires of every
  tool. Plan with `Setup(tool_change=TurnToolChange.carriage_kept())` (see
  [Lathe tool changes](operations.md#lathe-tool-changes)). Where the
  offsets are only in the control, `tool_change_tip_within_mm=(x radius, z)`
  on the same mode states one bound instead (how far any tool's controlled
  point on the tool post can be from any other's): no `carriage_to_tip_mm`
  is read, each `T` word after the first is followed by a rapid of X and Z
  to the start position, and the decoder refuses any other block there.
  Plan with `TurnToolChange.carriage_kept(tip_within_mm=...)`, the same
  value. `job.post` and
  `job.post_nc` refuse a setup whose tool-change pose disagrees with the
  machine's, before planning.
- **`diameter_register`** (per `ToolSetting`): the D register holding the
  tool's wear, written with `G41`/`G42` for `Contour(compensation="wear")`.
  `ThermalCut(compensation="control")` writes no register: the dialect's
  `compensation_amount_word` carries the amount.
- **`decimals`**: output precision. 3 mm / 4 inch is what FANUC/Haas-class
  controls accept. LinuxCNC and GRBL accept more.
- **Curves**: `arcs`, `helices`, `planes` and `max_sweep_rad` declare which
  G2/G3 moves the controller takes.
- `dialect`: the G/M codes. `GCodeDialect.linuxcnc()` is LinuxCNC 2.9; the
  controller posts in `examples/` declare their own.
- **`rapid_mm_min`**: the machine's rapid rate (nominal time of G0 moves).
- **Frames**: `work_offset` (1 → G54), `frame` (the program frame) and
  `work_to_machine` (where work zero sits in machine coordinates, for the
  travel and Z-home checks).
- **Traverse**: `traverse_feed_mm_min` writes rapids as G1 at that feed, for
  controllers whose G0 does not move in a straight line (Centroid doglegs).
- **Limits**: `max_feed_mm_min`, `max_spindle_rpm`, `max_tool_number`,
  `max_length_register`, `tool_changes` (False: one preloaded tool), `flood`,
  `mist`; `feed_tolerance_mm_min` (0.1) is the allowed feed difference.
  These declared values are the only machine limits a program is held to
  (#429): the motion checks themselves bound nothing but numeric safety
  (finite numbers, signs, coordinates within ±1e6 mm, sweeps up to 1000
  turns). A feed per revolution, a thread lead, a surface speed or a dwell
  has no bound of camlib's own; a feed per revolution is held to
  `max_feed_mm_min` move by move, as mm/rev times the highest rpm along the
  move, and a dwell to the dialect's `dwell_max_s` when it is written. A
  refusal names the operation, the value, the limit and the field it comes
  from, with the event (and move) index after the words, for example
  `face: the programmed feed, the feed per revolution 250 mm/rev at 2500 rpm
  (the highest along the move), 625000 mm/min, is above the machine's
  maximum feed 10000 mm/min (Machine.max_feed_mm_min) (move 3, event 7)`.
- **Dynamics**: `max_normal_accel_mm_s2` and `max_axis_feed_mm_min` report curve
  feeds the machine cannot reach. `motion_policy=MotionPolicy(
  tangential_accel_mm_s2, rapid_feed_mm_min, junction_mode="exact_stop" or
  "tangent")` enables `machine.schedule(program)` (time estimate, feeds that
  respect the limits) and `schedule_post_feeds=True` (posts the scheduled feeds).
  Optional `per_block_s=0.0` adds caller-declared seconds to every segment ending
  at rest, including rapids. It must be finite and between 0 and 10 seconds.
  Exact-stop mode adds it to every motion block; tangent mode adds it only at
  stops, including the final endpoint. Checked posting reports exact-stop
  source and decoded totals separately, so lowering one source motion into
  several decoded motions can increase the decoded stop cost.
  This term models a fixed stop cost separately from tangential acceleration.
  It does not infer a controller's cost or model speed-dependent settling.
  Zero preserves the previous schedule identity.

  For a synthetic example, consider three consecutive 20 mm lines at
  1200 mm/min with 100 mm/s² tangential acceleration and no feed reserve.
  Each exact-stop line takes 1.2 s: two 0.2 s ramps cover 4 mm, and the
  remaining 16 mm takes 0.8 s. If an example controller spends another
  0.125 s at each stop, `MotionPolicy(100, 6000, feed_reserve_fraction=0,
  per_block_s=0.125)` predicts 3.975 s for the three lines instead of 3.6 s.
  With `junction_mode="tangent"`, the continuous 60 mm path takes 3.2 s plus
  one final stop, giving 3.325 s. The cost is chosen for this example and is
  not a measured controller value. The runnable synthetic case is
  `fixed_stop_cost_matches_synthetic_controller_timeline` in `tests/dynamics.rs`.
- `revision`: your machine definition's version, recorded with the run.

## Dialect

`GCodeDialect` declares what each word means; nothing is guessed:

| Field | Meaning |
| --- | --- |
| `commands` | `Command(effect, code)` pairs; change them with `dialect.with_commands(effect=code or None)` |
| `work_offsets`, `axes`, `centers` | G54.. codes; X Y Z; I J K |
| `tool_word`, `length_word`, `feed_word`, `speed_word` | T, H, F, S |
| `tool_change` | `"m6"` (T then M6) or `"direct_t"` (T changes the tool) |
| `length_activation` | `"explicit"` (G43 H) or `"tool_edge"` (per-tool edge registers) |
| `parentheses_comments` | Whether `( )` are comments (else `;`) |
| `fixed_modes` | Modes the controller always has, so they are not written |
| `radius_tolerance_mm` | Allowed start/end radius mismatch of written arcs (0 to 0.005; default LinuxCNC's 0.00127) |
| `radius_relative_tolerance` | Relative mismatch also allowed up to 100 × `radius_tolerance_mm` (0 to 0.001; default LinuxCNC's 0.001) |
| `arc_word_mismatch_mm` | Start/end radius mismatch the post may *write* (0 to `radius_tolerance_mm`; default 0: exactly consistent words only) |
| `dwell_word`, `dwell_resolution_s`, `dwell_max_s` | Dwell word, step and range |
| `cycle_dwell_unit`, `peck_reentry_mm`, `chip_break_lift_mm` | Canned-cycle dwell unit and the controller's peck return amounts |
| `tap_pitch_word` | Lead word of a rigid tapping cycle (Centroid Q), or feed |
| `tap_cycle`, `tap_cycle_left` | Canned rigid tapping, right hand (`G84`) and left hand (`G74`), declared with `with_commands`. The spindle turns in the cycle's direction beforehand (M3 for G84, M4 for G74, FANUC 0i-MB §13.1.2); the decoder refuses a cycle under the other direction. Without `tap_cycle_left` (and without `tap_synchronized` G33.1) a left-hand tap is refused |
| `compensation_word` | D register word of G41/G42 |
| `compensation_with_move` | G41/G42/G40 written in the block of the move that turns compensation on or off (FANUC, Haas), not alone |
| `compensation_amount_word` | The word of a whole compensation amount for `ThermalCut(compensation="control")`: `G41`/`G42` and this word with half the kerf, in output units at the linear resolution, on a block of their own, and `G40` on its own (WinCNC `O`: "G41 O# ... O# is the compensation amount on machines without a tool changer", Users Guide, Controller 3.1, revision 3.1.04, pp. 138-139). The post writes the part side of the planned beam path (`camlib.postprocessor.written_path`); the decoder reads the stretch from `G41`/`G42` to the move after `G40` whole and replays the controller's offset of what was written (`GCodeDecoder.replay_compensation`, `camlib.controller_offset`), each move checked against the plan. None (default): `compensation="control"` is refused. Not with `compensation_with_move` |
| `oriented_edge_deg` | Where the boring bar's edge points after the oriented spindle stop |
| `thread_lead_word` | Lathe threading (`thread_synchronized`, e.g. G33/G32): the word carrying the lead (K for LinuxCNC G33, F for G32) |
| `tool_change="lathe_t"` | Lathe tool words `T<nn><oo>` (tool and offset, T0101) |
| `x_diameter` | Lathes: X words are diameters (arc I stays a radius). LinuxCNC lathes: declare `lathe_radius_mode="G8"` or `lathe_diameter_mode="G7"` to write the mode in the header |
| `css_on`, `css_limit`, `css_limit_word` | Constant surface speed (`SurfaceSpeed` events): `G96 S<m/min>` (ft/min in inch output) with M3/M4, its rpm limit either in the same block (`css_limit_word="D"`: LinuxCNC `G96 D<max> S<speed>`) or as `G50 S<max>` in the block before (`css_limit="G50"`: FANUC, Centroid); exactly one of the two. A `Spindle` after it writes the rpm mode (G97) first |
| `compensation_register` | False for lathes: G41/G42 written without a register word (the T offset holds the nose radius and tip direction) |
| `feed_per_rev` | Feed per revolution (`FeedPerRev` events): `G95 F<mm/rev>` (Centroid/FANUC lathes `G99`, with `feed_per_minute="G98"` and `cycle_return_initial=None`), back to `G94` at 0. The written F must keep the feed within `feed_tolerance_mm_min` at `max_spindle_rpm`, otherwise the post refuses (e.g. a metric feed in inch output) |
| `beam_s_max`, `beam_effect` | Plasma/laser: S at full power and the spindle code that fires the beam (`spindle_cw` M3, `spindle_ccw` M4); M5 turns it off. None refuses beam tools |
| `beam_mark` | How the machine marks with its beam ([`ThermalMark`](operations.md#plasma-and-laser-marking-thermalmark), `BeamMark` events), declared by its owner. `None` (default): a marking program is refused. `"power"`: the beam word at the marking power (a laser; GRBL v1.1 laser mode scales the laser power with S). `"mode"`: the controller has a marking process; its codes are declared with `with_commands(mark_mode="M..", cut_mode="M..")` and written with the beam off before the program's first marking beam, before its first cutting beam, and at every change. camlib knows no such code. See [Beam marking](#beam-marking) |
| `height_control_word` | Word after the `height_control_on`/`off` codes (e.g. `P2` for QtPlasmaC `M63 P2`/`M62 P2`) |
| `probing` | Probing back end: `None` (probing cycles are refused), `"g38"` (LinuxCNC G38.x) or `"inspection_plus"` (Renishaw Inspection Plus macros); see [Probing](#probing) |
| `probe_fast_feed_mm_min`, `probe_switching` | Inspection Plus: the installed fast feed #119, and whether each cycle switches the probe on and off (`G65 P9832`/`P9833`) |

`LoweringOptions(native_arcs=..., chord_tolerance_mm=0.0025,
max_segments_per_source=..., max_total_segments=...)` bounds the curve-to-line
conversion.

## Probing

A probing cycle (`camlib.motion.Probe`, from the [probing
operations](operations.md#probing-and-tool-setting)) covers its moves. The
post writes the whole cycle as the dialect's probing blocks; the decoder reads
every move back from those blocks alone (only the feature description and the
attribution come from the source), then each decoded move is compared with the
plan, and the program validator checks them against the cycle's layout.

**LinuxCNC (`probing="g38"`)**, from the LinuxCNC 2.9 G-code reference
(`docs/src/gcode/g-code.adoc`, `overview.adoc`):

```
G38.3 Z5 F1000                         ; protected move: stops on a trip...
O100 IF [#5070 NE 0]                   ; ...#5070 = 1 when the probe tripped
(ABORT, camlib: protected probe move obstructed)
O100 ENDIF
G38.2 Y0 F100                          ; measure: errors if nothing is touched
#<probe1> = [#5062 + 2]                ; error against the expected tip Y (-2)
G1 Y-7 F1000                           ; back
...
G10 L2 P1 X[#5221 + #<probe2>] Y[#5222 + #<probe1>]   ; G54 += error
```

`G38.2` ends `search_mm` beyond the expected contact; `#5061`-`#5063` hold
the trip point "in the current work coordinate system", so each error is
offset-independent. A feature measured from both sides averages its two
errors (the centre). `(ABORT,)` stops the program with an error
(`interp_convert.cc`). Tool length: `G10 L10 P<H> Z[#5422 - #<probe1>]`
then `G43 H<H>` (the register then makes the current Z what it truly is);
tool radius: `G10 L1 P<entry> R[<declared radius> + [#<probe1> -
#<probe2>] / 2]`. Premises: a probe input wired to `motion.probe-input`, no
G5x rotation, and a probe (or tool setter) that is the active input when the
cycle runs.

**Renishaw Inspection Plus (`probing="inspection_plus"`)**, from the
*Inspection Plus software programming manual* H-2000-6031 and H-2000-6222
(chapters 5 and 7), for FANUC-style custom macro controls:

| Cycle | Calls |
| --- | --- |
| Protected positioning | `G65 P9810 X Y Z F` |
| Surface | `G65 P9811 X|Y|Z<surface> S<n>. Q<search>.` |
| Outside corner | Two `O9811` calls: the Y face, then the X face |
| Inside corner | `G65 P9815 X<corner> Y<corner> S Q` (no I/J: parallel to the axes) |
| Bore | `G65 P9814 D<diameter> S Q` |
| Pocket | `G65 P9812 X|Y<width> S Q` |

`S1`..`S6` correct G54..G59 ("New work offset = active work offset + error");
`Q` is the overtravel, the search beyond the expected position. The decoder
expands each call into the moves the manual describes: O9811 measures from its
start toward the surface along its axis and returns; O9814 measures +X, -X,
+Y, -Y from the centre; O9812 + and - from the centre; O9815 the Y wall, then
the X wall, and returns. The measuring move runs at 0.6 x the installed fast
feed #119 and the move back at #119 (0.6 x #119 in Z) (chapter 9,
"Feedrates"): the plan must use those feeds, or the post refuses. The gauging
touch at 30 mm/min after the back-off `#506` lies on the same line. A boss or
web (`O9814`/`O9812` with Z and R) and tool setting are refused: their macro
moves are not taken from a source here; post them with `probing="g38"`.
Premises: the macros installed (some controls embed them behind an M code),
the probe calibrated with `O9801`-`O9804`, and the macros restoring the modal
state they change.

## Beam marking

A `ThermalMark` fires the beam with `BeamMark(power_percent)` and feeds
`mark` moves under it. How a controller marks is not something camlib knows:
the machine declares it, and without a declaration the post refuses.

```python
# A laser: marking is the beam word at the marking power.
laser = GCodeDialect(beam_s_max=1000, beam_mark="power")           # M3 S120 ... M5
# A controller with a marking process: its own codes, declared.
plasma = GCodeDialect(beam_s_max=1, beam_mark="mode").with_commands(mark_mode="M..", cut_mode="M..")
```

| Declared | Written | Decoded |
| --- | --- | --- |
| `"mode"` | `mark_mode`, then the beam word; `cut_mode` before the next cutting beam (and before the program's first cut: the controller may have been left in its marking process) | The decoder tracks the process from `mark_mode`/`cut_mode` alone (only with the beam off). A beam word under the marking process decodes as a marking beam, under the cutting process as a cutting beam, and before any selection is refused |
| `"power"` | The beam word with S at the marking power; every cut's beam word states the cutting power again | The power word is decoded and compared (within 0.5 %). No word tells a marking beam from a cutting one, so its role is the source event's, as a fed move's kind is |

The decoded program then goes through the same typed checks as the plan: a
`mark` move needs the marking beam, a cutting move the cutting beam, fired
again after the marks. So a post that leaves the marking process selected
for a cut, selects the wrong process, or writes the wrong power is refused
(`python/tests/test_thermal_mark.py` posts with such broken posts). The
decoded program is also replayed on the stock, which refuses a mark over cut
sheet.

A marking current, gas or consumable is the controller's (its process or
material table); the program only selects the process. Controllers whose
marking needs more than one code per selection get their own post:
[QtPlasmaC](qtplasmac-post.md) selects the machine's declared marking and
cutting materials.

## Arcs that cannot be written exactly

On a 3-decimal grid some arcs cannot be written with exactly equal start and
end radius. camlib never emits a slightly inconsistent circle. Choose
explicitly how to post with `lowering=`:

| Option | Result |
| --- | --- |
| *(none)* | Error naming the arc, if such an arc occurs |
| `LoweringOptions(native_arcs=True)` | Exact arcs as G2/G3; the rest as checked line segments within `chord_tolerance_mm` (0.0025). Counts and reasons in `status()["posting"]["arc_output"]` |
| `LoweringOptions()` | Every curve as checked line segments |
| `Machine(decimals=4)` | More precision, if the controller accepts it |

Rounded arc words rarely put the end exactly on the start circle. Controllers
accept a small start/end radius mismatch and cut a spiral between the two radii:

| Controller | Documented acceptance |
| --- | --- |
| LinuxCNC | Mismatch ≤ T, or ≤ 0.1 % of the radius while ≤ 100 T, with T = `CENTER_ARC_RADIUS_TOLERANCE_MM` (documented default 0.00127 mm; INI minimum 0.001) (`interp_arc.cc` `arc_data_ijk`) |
| FANUC 0i | Parameter 3410 (machine-specific) |
| Centroid CNC12 | Not documented, so the example post uses `radius_tolerance_mm=0` |

A representable arc end can differ from the planned end by part of a word.
A full circle (an adaptive orbit, a closed helix turn) always closes exactly:
its end word repeats the controller's actual start, so start and end radius
are identical and only the I/J rounding remains (checked against the planned
circle like any arc). When a full circle becomes line segments instead, the
last segment ends on that same start. A straight vertical move that follows (a retract out of the cut, a plunge) keeps
the controller's current XY when that is within half the posting tolerance, so
it never adds a sideways step while leaving or entering a surface; the decoded
block is still checked against its source motion.

`GCodeDialect.radius_tolerance_mm` and `radius_relative_tolerance` set the
mismatch the post may write, with LinuxCNC's rule: at most
`radius_tolerance_mm`, or at most `radius_relative_tolerance` of the radius
while at most 100 × `radius_tolerance_mm`. The defaults are LinuxCNC's
documented ones (0.00127 mm and 0.1 %), so a stock LinuxCNC accepts every
block; declare your INI's `CENTER_ARC_RADIUS_TOLERANCE_MM` if it differs (at
most 0.005 here). `radius_tolerance_mm=0` demands exactly consistent words, as
the controller posts above without a documented tolerance use.

What the controller accepts and what the post writes are declared apart. By
default the post writes only exactly consistent words (start and end radius
equal on the word grid), whatever the controller would accept; an arc that
has none within the posting tolerance at the output's decimals is posted as
checked lines (`native_arcs=True`) or refused. Shallow arcs of a large radius
at four decimals of an inch are the usual case. A dialect that declares
`arc_word_mismatch_mm` (at most `radius_tolerance_mm`) lets the post write
words that differ in radius by up to that much, for arcs of at most half a
turn, so those arcs stay arcs. Such a block is certified with the mismatch
added to its deviation bound: how the controller blends the two radii is not
assumed, only that its path lies between the circle through the start and the
circle through the end about the written centre. The posted program's stock
and machine checks then replay the block as one exact circle, through the
written start and end with its centre moved along the chord onto its
bisector, which lies in that same band.

The radii are decided on the words themselves, so every platform writes the
same program (issue #169). When the start, end and centre offsets are all
words (up to 9 decimals, or half of one for X written as a diameter), the
squared radii are compared as exact integers on the word grid: words with
equal radii have a mismatch of exactly 0, with no epsilon. A start that no
word wrote (a declared tool-change position, for example) is a binary64 value,
and the radii are computed from it with correctly rounded arithmetic only,
never the platform's `hypot`. The post and the decoders use the same rule,
each with its own implementation (the word chooser, the Rust decoders and
the Python decoder), and a test checks that they agree bit for bit.

The certified
deviation of a posted arc
includes the spiral: the start/end radius difference of the written words is
added to the bound before it is compared with the posting tolerance.

## Axis words a straight move leaves out

A straight move (G0/G1) leaves out the word of an axis it does not move
while that axis is still where the controller placed it itself (#367):

- At the program start every axis is where the controller placed it (the
  machine's start pose).
- A tool change whose declared mode moves axes to a machine position places
  them again, from its change block (`M6`, or the `T` word with
  `tool_change="direct_t"`/`"lathe_t"`): Z with
  `tool_change_position_mode="z_home"`, every axis with `"table"`.
  `"preserve_tip"` and `"preserve_spindle"` place none. A lathe's
  `"preserve_carriage"` moves no axis but puts every axis under the new
  tool's offset, at a work position no block has commanded: every axis
  counts as placed after its `T` word.
- An axis leaves that state with the first block that writes its word: a
  move that changes it, an arc or helix (they always name their plane's
  axes), a canned cycle, a tapping or threading block, or a block a
  controller post writes itself (a FANUC `G1 G43 H.. Z..`, a Carvera
  `G53 G0 Z-3`). A probing cycle leaves every axis written, since its macros
  move axes their blocks need not name.
- A block that only sets an offset or a mode, naming axis letters as its
  parameters (`G52 X0 Y0 Z0`, Haas `G100 X0 Y0 Z0`), writes no axis word:
  the decoder reads it as moving no axis (`GCodeDecoder.hold_axes`), and
  the post and the machine check both leave the axes it names placed. A
  block no decoder classifies so counts every axis it names.

An axis word commands the axis to that work coordinate through the
controller's *actual* work offset, even at the value it already has. Where
the controller placed the axis itself, the program's nominal offset and the
real one differ by the calibration interval, so the word would move the axis
by that error; at a travel limit, past it. A mill homed at X 0 and Y 0 (its
travel minimums) facing from its home pose writes

```
G0 Z10
G0 X-24 Y-17.6 Z10
```

and not `G0 X-250 Y-200 Z10` first, which LinuxCNC refuses as beyond the soft
limit when the real offset is 0.02 mm the other way. The word is left out
only when the value the block would write equals the controller's current
position exactly (the vertical-move rule above keeps the current XY, so a
move vertical in the plan qualifies), and never in a block that turns cutter
compensation on or off. A move that would keep no axis word at all (it moves
nothing) is written whole, since a bare `G0` is no motion block. Every decoder reads an absent word as the axis's
current position, and the machine check holds an axis no block has written
exactly at its declared position ([machine coverage](machine.md)), so the
first moves from a pose on a travel limit are checked inside it. Only the
blocks before each axis is first written change; the toolpaths do not.

The default post (`GCodePost.on_motion`, and every controller post in
`examples/` built on it), the native default formatter and the Rust posts
(default, Haas NGC, Carvera) apply the rule alike. The state is
`self._placed` (one flag per work axis), kept by `PostProcessor._call` from
the blocks each hook writes; a post that writes its own straight-move blocks
can read it.

## Writing your own post

Subclass `GCodePost` and override the hooks you need. Each hook writes lines
with `self.emit(...)` and `self.comment(...)`:

```python
from camlib.post import GCodePost

class MyShopPost(GCodePost):
    version = "my-shop-1"

    def on_tool_change(self, event):
        self.comment(f"Tool {event.number}")
        super().on_tool_change(event)
```

Hooks: `on_begin(program)`, `on_work_offset`, `on_tool_change`,
`on_spindle`, `on_coolant`, `on_stop`, `on_dwell`, `on_motion(motion)`,
`on_cycle`, `on_cutter_compensation`, `on_spindle_orient`, `on_beam`,
`on_beam_mark`, `on_height_control`, `on_surface_speed`, `on_feed_per_rev`, `on_finish()`. The default `on_dwell` writes the dialect's `dwell` code (G4)
with the time in seconds in `dwell_word` (P), rounded up to
`dwell_resolution_s` and limited to `dwell_max_s`.

Plane selection is modal (G17/G18/G19, modal group 2 in the LinuxCNC G-code
reference; FANUC, Haas and Centroid read it the same way), so the program
selects a plane once, in its header, and again only where an arc or a canned
cycle needs another one: a lathe program has one `G18`, and a mill program
with XY arcs only has one `G17`. A post that writes arcs itself calls
`self.select_plane("xy" | "xz" | "yz")` before the arc block; it writes the
dialect's code only when another plane is active (a block that is a plane
code alone, written by any hook, sets the active plane). The decoder keeps
its own plane from the blocks it reads and decodes every arc in it, so a
plane code that was needed and is missing gives an arc that differs from the
plan, and the program is refused.

Arc words come from `circular_values(curve)`: with the standard `number` and
`word` it searches the representable end and centre words for ones whose
start and end radius agree within the dialect's radius tolerance. A post
that overrides `word` or `number` gets the arc's plain rounded values
instead (its own formatter is in control), which at `radius_tolerance_mm=0`
its decoder will usually refuse. If the override keeps every word's value
(it only appends a decimal point, as the FANUC lathe post does), also
override `circular_values` to return `self.exact_circular_values(curve)`.

Canned drilling cycles (`Drilling(canned_cycle=True)`) are written by
`on_cycle`: `G98` once, then `G81`/`G82`/`G83`/`G73 X Y Z R [Q] [P] F` per
hole and `G80` before the next other block. The dialect declares the codes
(`drill_cycle`, `drill_dwell_cycle`, `peck_cycle`, `chip_break_cycle`,
`cycle_return_initial`), the G82 dwell unit (`cycle_dwell_unit`, `"s"` or
`"ms"`) and the controller's fixed return amounts (`peck_reentry_mm`,
`chip_break_lift_mm`; LinuxCNC 0.254 mm). Cycles drill along Z, so on a
machine that declares G17 the post selects `G17` before a cycle block (drilling,
boring, fine boring or G84/G74 tapping) when an arc left G18 or G19 active, and
the decoder refuses a cycle block outside G17: in G18 LinuxCNC drills along Y
("performed with respect to the currently-selected plane", #183). A cycle block and the hole's moves
it covers are checked in one batch, with or without `lowering=`; a cycle
covering more than 63 events is refused (post that hole with
`canned_cycle=False`). Boring cycles (`Reaming(canned_cycle=True)`)
use `bore_cycle` (G85), `bore_dwell_cycle` (G89) and `bore_stop_cycle` (G86,
whose spindle stop and restart are part of the block); they start at the R
plane so every controller runs them alike. Fine boring needs the machine's
oriented spindle stop: declare `oriented_edge_deg` (where the bar's edge points
after it, work XY degrees) with `spindle_orient` (M19, written before explicit
shift moves) and/or `fine_bore_cycle` (G76 X Y Z R Q [P] F, whose shift
direction parameter must point opposite the edge). LinuxCNC's `G76` is a lathe
threading cycle, so the default dialect declares neither. A cycle the dialect lacks is written
as its explicit moves. The decoder expands each cycle block back into the
controller's moves, and each is checked against the planned motion it covers.
Pass it with `job.post(..., post=MyShopPost)`. See
[`examples/python_post.py`](../examples/python_post.py).

## Controller examples

These are ordinary Python files in [`examples/`](../examples). Each targets one
documented controller generation and lists the settings it assumes. Verify
them against your machine before use:

| File | Controller | Notes |
| --- | --- | --- |
| `GCodeDialect.linuxcnc()` | LinuxCNC 2.9 | `GCodeDialect.linuxcnc(beam_s_max=...)` writes a beam as `M3 S`/`M5`; `probing="g38"` writes probing cycles |
| `grbl_post.py` | GRBL 1.1h | Lines only (arcs as checked segments), one preloaded tool, checked with GRBL's own binary32 arithmetic; vertical moves are written with Z only. `define_machine(..., beam_s_max=1000)` (= `$30`) writes a laser as `M4 S` (laser mode `$32=1`); with `beam_mark="power"` it also marks, as `M4 S<marking power>`. `grbl_linear.py` is a complete example |
| `carvera_post.py` | Makera Carvera (metric milling) | Python and Rust named pair; [setup and retract premises](carvera-post.md) |
| `haas_ngc_post.py` | Haas NGC (2020 manual) | [setup notes](haas-ngc-post.md) |
| `fanuc_0imb_post.py` | FANUC 0i-MB | [setup notes](fanuc-0imb-post.md); `inspection_plus_fast_feed_mm_min=` posts probing as Renishaw Inspection Plus macros |
| `lnc_mill_post.py` | LNC milling, LNC-M8000 (A-LNC programming manual V01.02) | [setup notes](lnc-mill-post.md); native G2/G3 in G17/G18/G19 checked against parameter 50048; canned cycles and M29 rigid tapping with the declared parameters |
| `siemens808d_post.py` | SINUMERIK 808D native | [setup notes](siemens808d-post.md); CYCLE84 rigid tapping in both hands with the declared spindle premises |
| `siemens808d_iso_post.py` | SINUMERIK 808D ISO mode | [setup notes](siemens808d-iso-post.md); G84/G74 rigid tapping with the declared GUD settings |
| `centroid_post.py` | Centroid CNC12 mill (Acorn, Oak, ...) | [setup notes](centroid-post.md) |
| `qtplasmac_post.py` | LinuxCNC 2.9 QtPlasmaC (plasma) | [setup notes](qtplasmac-post.md) |
| `centroid_plasma_post.py` | Centroid CNC12 plasma (Acorn THC) | [setup notes](centroid-plasma-post.md) |
| `wincnc_plasma_post.py` | WinCNC 3.x plasma (WinCNC THC) | [setup notes](wincnc-plasma-post.md) |
| `centroid_lathe_post.py` | Centroid CNC12 lathe (diameter X, T0101, G96/G50, G99, G32) | [setup notes](centroid-lathe-post.md) |
| `fanuc_lathe_post.py` | FANUC lathe, G-code system A (%, O number, decimal points, T0101, G50/G96, G99, G32) | [setup notes](fanuc-lathe-post.md) |

Add `examples` to `PYTHONPATH`, or copy the file next to your code. The
controller examples write curves as checked line segments by default.

### LinuxCNC tool changes

`GCodeDialect.linuxcnc()` writes a tool change as `T<n> M6` and then
`G43 H<n>`. What the machine does at `M6` is not fixed by the G-code: it is
the machine's configuration, so camlib assumes none and you declare it in
`tool_change_position_mode`. From the LinuxCNC documentation:

- [`M6`](https://linuxcnc.org/docs/stable/html/gcode/m-code.html#mcode:m6):
  "The tool change may include axis motion", "If configured in the INI file
  some axis positions may move when a M6 is issued", and "The tool length
  offset is not changed by M6, use G43 after the M6 to change the tool length
  offset."
- [`[EMCIO]`](https://linuxcnc.org/docs/stable/html/config/ini-config.html):
  `TOOL_CHANGE_POSITION` ("the XYZ location to move to when performing a tool
  change"), `TOOL_CHANGE_QUILL_UP = 1` ("The Z axis will be moved to machine
  zero prior to the tool change") and `TOOL_CHANGE_AT_G30 = 1` move the
  machine at `M6`.
- [`G43`](https://linuxcnc.org/docs/stable/html/gcode/g-code.html#gcode:g43):
  "G43 does not cause any motion. The next time a compensated axis is moved,
  that axis's endpoint is the compensated location."

| Your machine at `M6` | `tool_change_position_mode` | Setup |
|---|---|---|
| None of the `[EMCIO]` moves is set and the tool is changed where the spindle stands (a manual change, or a changer that comes to the spindle) | `"preserve_spindle"` | `ToolChangeInPlace()` |
| `TOOL_CHANGE_QUILL_UP = 1` only: Z goes to machine zero, X and Y stay | `"z_home"` with `tool_change_home_z_mm` (the gauge line's machine Z there) | `ToolChangePosition(gauge_z_mm=...)` |
| `TOOL_CHANGE_POSITION`, `TOOL_CHANGE_AT_G30`, or a remapped `M6` that ends at a fixed pose | `"table"`: each tool's tip at that pose in `position_after_mm` | `ToolChangeTable.from_machine(machine)` |

With `"preserve_spindle"` the spindle stays and only the tool differs, so the
tip moves by the difference of the gauge lengths (`ToolSetting.gauge_length_mm`,
the lengths in the controller's tool table): changing a 75 mm tool for a
115 mm one puts the tip 40 mm lower, and `G43 H` then makes the controller's
Z agree with it without moving. camlib plans, checks and decodes the program
with that pose: an operation whose tool would arrive below its clearance
height is refused with the change named, and the new tool's first moves are
checked against stock and fixtures like any others. The mode needs the work Z axis along machine Z, and is refused
for lathe turrets and for controllers that select a cutting edge with `T`/`D`.

Because M6 moves nothing on such a machine, the program itself must take the
spindle clear before the change when the next tool is longer. State where:
`Setup(tool_change=ToolChangeInPlace(retract_gauge_z_mm=150))` plans, before
every tool change, one rapid of the old tool straight up until the spindle
gauge line is at work Z 150 (the 75 mm tool's tip at Z 75); the 115 mm tool
is then changed in with its tip at Z 35. The post writes that rapid as an
ordinary block before the change (here after an operation that stopped at
Z 12 over X20 Y8), and the decoder reads it back as one:

```
G1 X20 Y8 Z12 F600
G0 X20 Y8 Z75
M9
M5
G54
T2
M6
G43 H2
```

The retract is a planned move like any other: it is streamed with the run,
checked against stock, fixtures and the machine's travel, and compared with
the decoded program. camlib assumes no retract height: without the value the
tool is changed where the last operation stopped, as before, and a job whose
longer tool would then start below its clearance height is refused. A stated
retract only goes up (a value below where an operation leaves the spindle is
refused), and a value too low for the next tool's clearance height is refused
naming the change and the height it needs. The value is a work Z of the
spindle gauge line, so it is the same for every tool; it is not the
controller's `G53`/`G28`/`G30` and no machine-coordinate block is written.

### Program end position

A post ends a program by stopping the spindle and coolant and writing its
end code; it adds no move of its own (the Carvera post writes its
`G53 G0 Z-3` only where the plan already ends at that height, see its
notes), so the tool is left where the last operation retracted to. State where the program should end instead with
`Setup(end_position=EndPosition(gauge_z_mm=..., xy_mm=None))`: the planner
adds a Z rapid until the spindle gauge line is at that work Z and, when
`xy_mm` is given, an XY rapid, after the last operation. Every mill post
writes them as its ordinary rapids (`G0`, or `G1` at the declared traverse
feed on a machine that declares one), before the stop and the end code, and
its decoder reads them back like any other block. With the default post
(LinuxCNC dialect), a 60 mm tool and
`EndPosition(gauge_z_mm=100, xy_mm=(-10, 30))`:

```
G1 X20 Y8 Z12 F600
G0 X20 Y8 Z40
G0 X-10 Y30 Z40
M9
M5
G49
M2
```

and without an end position, byte for byte what it was:

```
G1 X20 Y8 Z12 F600
M9
M5
G49
M2
```

The positions are work coordinates of the active work offset, written as
plain `G0` blocks: no `G53`, `G28` or `G30` block is written, since where
those send a machine is its configuration. The moves are checked like any
others (stock contact, fixtures, avoid areas, the declared travel, and the
machine check on the decoded program), and an end position a check refuses
is reported on that move. Lathe posts are unchanged: a lathe program already
ends with the tool at the turret index position the setup starts from.
Plasma and laser setups take no end position.

## Posting without Python (Rust, WebAssembly)

The default post (`GCodePost` with `NativeGCodeDecoder`) also exists in Rust,
for applications without a Python runtime, such as a WebAssembly build of the
engine (issue #120). It is a port of the Python default post and decoder, not a
second design: for the same job and machine it writes the same program, byte
for byte (`python/tests/test_rust_post.py` compares them on the examples and on
jobs with canned cycles, tapping, cutter compensation, dwells, plasma words,
probing cycles on both probing back ends (LinuxCNC G38.x and Inspection Plus
macros, #147), lowering, inch output and traverse feeds). Posts you write in Python stay
Python. The default post, dialects declared as data, and the explicitly named
Haas NGC and Makera Carvera reference posts run in Rust. Other controller subclasses remain Python.

The machine and its dialect cross as JSON: `Machine.to_dict()`, the same form
a posted run records as its `definition["machine"]`, with the #116
`crash_model` inside. Every field is required, except the optional settings
`to_dict()` leaves out when unset (`max_normal_accel_mm_s2`,
`max_axis_feed_mm_min`, `motion_policy`, `schedule_post_feeds`, `crash_model`, `controller`).
A JSON machine is checked with the Python rules and messages.

| Entry point | What it does |
| --- | --- |
| `camlib::post_job::run(input_json, cancel)` (Rust) | The whole `Job.post_nc` run for one setup: plans and streams its operations, posts them, decodes every block and compares it with its source event, replays the decoded program on the stock with the setup's checks, checks it against the machine model, and applies the export gate. No file, process or thread |
| `cam-post` | The same, as a command: the JSON input on stdin, the JSON report on stdout (errors on stderr, exit status 1). Separate from `cam-sim-stream` |
| `camlib.post.post_job_input(job, setup_id, machine=..., lowering=None, accept=())` | Builds that JSON input from a Python job |
| `camlib.post.post_job(input)` | Runs the Rust post in this process; returns the report |
| `camlib::gcode::post_program(input_json, cancel)` (Rust) | The counterpart of `post_events`: posts an explicit `Program`'s events with the decode-and-compare check, no stock replay and no export gate |

The input is `{"manifest": ..., "program_id": ..., "frame": ..., "machine":
..., "lowering": ..., "accept": [...], "run_id": ...}`: the setup's stream
manifest (what `cam-sim-stream` reads), the setup id and work frame, the
machine, optional `LoweringOptions` fields and the finding kinds accepted for
release (`uncertain`, `violation`, `unchecked_machine`). The report holds
`released`, `program` (the NC text, only when released), `refused` (statuses
and message), `findings` (as `PostedRun.export` lists them: stock contact,
scene and machine limits per move, material, machine model), `accepted`,
`unchecked`, `machine` (the #116 report) and `posting` (run identity,
definition, arc output). A block that does not decode to its planned event is
an error, as in Python: nothing is released.

Not supported yet in Rust, and refused with a message: lathe setups (the
turning replay) and `schedule_post_feeds`.

### Named controller posts

`Machine.to_dict()` may include a `controller` object with a required `name`.
Omitting it selects the existing default formatter/decoder and preserves its
program bytes and definition. Unknown names and unknown options are refused.
The first named pair is `"name": "haas_ngc"`: the 2020 reference in
[Haas NGC](haas-ngc-post.md), not a VF-2 machine preset or later firmware alias.
`examples/haas_ngc_post.py`'s `ReferenceMachine.to_dict()` packs its additional
fields into this object, including Setting 22/52, program number, curve policy,
rotation/scaling, tapping and compensation options. All are explicit in JSON;
there are no inferred machine values. Complete synthetic inputs are in
[`fixtures/posts/rust/haas-drill.json`](../fixtures/posts/rust/haas-drill.json)
and [`haas-program.json`](../fixtures/posts/rust/haas-program.json). The base machine and `crash_model` stay
in their existing locations. Rust validates the same reference premises.

**Compatibility:** Haas Python posting definitions now record reference-specific
fields under `definition["machine"]["controller"]`, rather than flat under
`definition["machine"]`. For example, `coordinate_system` moves to
`definition["machine"]["controller"]["coordinate_system"]`. Consumers of stored
metadata must update those paths. Base-machine fields stay at their existing
paths; Python machine attributes and the emitted Haas program are unchanged.
Calling Python posting with a declared Haas machine and omitted/default post
classes now refuses instead of emitting a default-post program.

The name selects controller formatter hooks and an independent actual-word
decoder guard. The Haas formatter extends the common numeric and event hooks;
the Haas decoder parses framing, resets, numeric notation and controller side
effects independently, then uses the common independent motion/cycle reader.
It imports no formatter code. Both use the existing engine, intent checker,
posted-stock replay, machine check and release gate. This extends the #142/#159
posting path, respecting decisions 4–7 and 11; it adds no parallel job or
verification path. Controller-specific program and decoder versions enter the
posting identity from the selected variant. Rust's controller enum lives in
`gcode::controller`; every controller-specific formatter and decoder hook matches
it exhaustively. Adding a variant requires explicit hook and identity choices.

Python classes and user subclasses continue to execute as Python. A declared
controller requires an explicitly matching `post` and `decoder`: both classes
must declare the same `controller_name` as `machine.controller.name`. Python
refuses a default or mismatched pair before producing a posting definition or
any blocks. For Haas use `job.post_nc(..., machine=machine,
post=HaasNGCPost, decoder=HaasNGCDecoder)`. Rust selects the matching pair directly
from the same declaration; its JSON accepts no separate post override. Machines
without a controller continue to use the existing default post. This prevents
the same declared machine from silently using the default in Python and Haas
in Rust. Custom Python subclasses remain Python; the marker declares their
controller contract and is inherited by controller-specific subclasses. Serialization does not compile or dispatch
Python code. To run the shipped Haas reference in Rust, pass its machine's
`to_dict()` through `post_job_input`/`post_job` or `cam-post`; supply explicit
`LoweringOptions()` for the convenience iterator's linear curve policy.
Changes to the reference must keep both ports byte-identical by tests. Moving
all editable Python posts into a mandatory registry would require a separate
issue arguing a change to the documented Python-post contract.

Haas supports G81/G82/G83/G73, G84/G74, G85/G89, dwell and compensation in lead
blocks under its declared settings. It does not declare G86 (#97): as in Python,
stop-boring uses explicit source moves; declaring G86 is refused. No real Haas
machine or controller execution is established by these nominal checks.

## What a passing post does and does not prove

Passing means the emitted words, decoded independently, describe the planned
motion within `tolerance_mm`, with explicit units, plane, offsets and modal
state. It does **not** prove the controller's real settings (work offsets, tool
length table, option parameters, tool-change routine) match the machine
definition. Those are inputs you supply.

### Checked in LinuxCNC's own interpreter

The decoder that checks a posted program is camlib's reading of the
controller's manual. For LinuxCNC there is a second, independent reader: the
controller's own interpreter. CI runs posted LinuxCNC programs through
`rs274`, the standalone interpreter shipped with LinuxCNC (2.9.4, Debian's
`linuxcnc-uspace`), which executes a program and prints the canonical
commands it would send to the machine, and compares those with the plan
(`python/tests/test_linuxcnc_interpreter.py`):

- every planned motion, in order, is executed by interpreter moves that stay
  on its curve and end where it ends, within the post's own deviation bound
  for that motion plus the interpreter's print resolution (0.0001 mm);
- a rapid is a traverse, anything else a feed at the planned rate;
- while it feeds, the spindle, the coolant and the tool are the plan's;
- the interpreter accepts every block and moves nowhere the plan does not.

What runs, and what is compared besides the path:

| | jobs | also compared |
|---|---|---|
| Milling (`GCodeDialect.linuxcnc()`) | facing and pockets, arcs, two tools with a tool change, drilling as G81/G83/G73 cycles and as explicit moves, a contour with G41 wear compensation, G33.1 rigid tapping | spindle speed and direction, coolant, tool |
| Turning (G7 or G8) | facing, roughing, finishing with arcs, grooving and parting off, drilling on the axis, boring, threading outside and inside | X as a diameter or a radius; constant surface speed and its rpm limit (`G96 D S`); feed per revolution (`G95`); each threading pass synchronised (`G33`) with the planned lead |
| Plasma (QtPlasmaC post) | a plate with holes and a washer at a fixed height and with programmed heights; marking part numbers, then cutting | the torch on for every cut and lead and off for every rapid (`M3 $0 S1`, `M5 $0`); torch height control on and off where planned (`M63 P2`, `M62 P2`); feed hold (`M52 P1`) |

Each group also has tests that change one word of a program (a coordinate
by 0.01 mm, a surface speed, its limit, a feed, a thread's lead, a torch-off
removed, height control switched the wrong way) and require the comparison to
report it.

Limits, so this is not read as more than it is:

- It is the interpreter, not a machine: it says nothing about a real
  machine's limits, tool table, offsets, kinematics or tool-change routine,
  and for plasma nothing about QtPlasmaC's own components (probing,
  piercing, arc-OK, the height controller itself). No post has been run on a
  real machine ([#20](https://github.com/stevecad/camlib/issues/20)).
- The interpreter is started at the position the `Machine` declares the
  tool is at when the program begins (a canned cycle takes its initial plane
  from there), with a tool table of zero offsets.
- A rigid tap's lead (the `K` of `G33.1`) is not in the interpreter's
  output; camlib's decoder checks it.
- QtPlasmaC changes material with `M190`, a script of its configuration and
  not a code of the interpreter: the standalone interpreter stops at it
  (`Unknown m code used: M190`, which a test records). A marking program is
  therefore run without its `M190` blocks. Which material is selected, and
  that the cutting material is restored before the cuts, is checked by
  camlib's decoder only.

The post lifecycle follows FreeCAD CAM's
[postprocessor](https://github.com/FreeCAD/FreeCAD/blob/920d14763241267340e755f3d3b6ece8ef60fa43/src/Mod/CAM/Path/Post/Processor.py)
concepts.

The named `"name": "carvera"` pair targets metric Makera Carvera milling.
Its explicit tool-length, coordinate and planned-retract premises are described
in [Carvera setup notes](carvera-post.md). Python requires `CarveraPost` with
`CarveraDecoder`; Rust dispatches the same named pair from the machine JSON.
