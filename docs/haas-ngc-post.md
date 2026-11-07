# Haas NGC2020 Python reference post

This ordinary Python example targets the three-axis milling subset in the
Haas96-8210 RevN/December2020 manual. It is not a native controller registry,
CHC/later-firmware alias, external interpreter result or machine qualification.

## Ordinary Python use

```python
import math
from camlib.motion import WorkFrame
from camlib.post import ToolSetting
from haas_ngc_post import define_machine, post_haas  # examples/ on PYTHONPATH

# camlib assumes nothing about the machine: declare it.
declared = dict(
    output_units="mm",
    decimals=3,
    max_feed_mm_min=10000,
    max_spindle_rpm=12000,
    max_sweep_rad=math.tau,
    tool_changes=True,
    flood=True,
    tool_change_position_mode="table",
    rapid_mm_min=10000,
)
machine = define_machine(
    "My Haas setup",
    (ToolSetting.for_tool(tool, number=1, length_register=1,
                          position_after_mm=program.initial_position_mm),),
    traverse_feed_mm_min=3000, coordinate_system="fanuc", h_t_agreement=True,
    g92_shift_mm=(0, 0, 0), rotation_scaling=False,
    work_to_machine=WorkFrame(origin_mm=(100, 200, 300)), frame=program.frame, **declared)
for chunk in post_haas(program, machine):
    output.write(chunk.text)
    simulate(chunk.motions)  # independently decoded actual words/feeds
```

The numbers above are example setup bindings. Supply the real stored offsets,
options and tool table for a qualified configuration. `HaasNGCPost` and
`HaasNGCDecoder` are normal Python classes; use them or your subclasses directly
with generic `PostEngine`/`post_events`. The convenience iterators fix this
reference's classes and use bounded line lowering by default. Machine options
include `curve_policy="exact_ijk"`, units, precision/budget, travel, feed/RPM
limits, six stored offsets, program number and optional-stop enablement.
`rigid_tapping=True` declares the installed rigid tapping option: each tapping
pair is written as `G84 X Y Z R F` (right hand) or, for `Tapping(hand="left")`,
`G74 X Y Z R F` (reverse tap; 96-8210 Rev N §5.8.2, p. 351), with F = pitch × S
in G94 and R = the current Z, so G98/G99 return to the same plane, and cancelled
with `G80`; without it, tapping is refused. The spindle is commanded in the
cycle's direction beforehand (M3 for G84, M4 for G74) and the decoder checks it.

Cutter compensation (`Contour(compensation="wear")`) needs
`cutter_compensation=True`: `G1 G41 D… X… Y… F…` on the lead-in move and
`G1 G40 X… Y… F…` on the lead-out, as Haas requires (no start or cancel on an
arc or without an XY move). The D register holds only the wear: a diameter or
a radius per Setting 40, geometry zero. The wear-only offset on camlib's
tangent leads is the same under Setting 58 Fanuc or YASNAC. Source: Haas Mill
Programming Manual (Haas Factory Outlet / Productivity Inc., rev. 06/26),
cutter compensation chapter.

## Dwell and canned drilling cycles

From the Haas Mill Operator's Manual 96-8210 Rev N (G-codes chapter, §5.8),
holes with `canned_cycle=True` are written as one block per hole after `G98`,
cancelled by `G80`:

| Cycle | Block | Notes |
| --- | --- | --- |
| Drill, spot drill | `G81 X Y Z R F` | |
| Dwell, counterbore | `G82 X Y Z R P F` | P in seconds |
| Peck | `G83 X Y Z R Q F` | Needs `setting_22_mm` (Setting 22, Can Cycle Delta Z: re-entry above the last peck) and `setting_52_mm=0` (Setting 52, G83 Retract Above R) |
| Chip break | `G73 X Y Z R Q F` | Needs `setting_22_mm` (the chip-clearing retract) |
| Boring | `G85 X Y Z R F` | Feeds back out |
| Boring with dwell | `G89 X Y Z R P F` | P in seconds |

A standalone dwell (`Drilling(dwell_s=...)` without a cycle) is `G4 P`. Every P
is written in seconds with a decimal point, to the millisecond: Haas reads
`P10.` as 10 seconds but `P10` as 10 milliseconds (G04 note), and the decoder
refuses an undotted P.

Settings are declared, never assumed. A peck or chip-break hole with
`canned_cycle=True` is refused, naming Setting 22 or Setting 52, when the setting
it needs is not declared. A nonzero Setting 52 is refused too: G83 then clears
chips above R, which camlib does not model. The operation's
`reentry_clearance_mm` / `chip_break_lift_mm` must equal Setting 22. `G86`
(bore and stop) is not declared: the manual does not say the spindle restarts
after the cycle, so those holes are written as explicit moves.

## Semantics and setup premises


Use the generic Machine/PostEngine, typed clearance-feed policy and native full
continuous comparison. Default bounded linear lowering handles all source planes,
arcs/helices/multiple turns. Optional exact_ijk supports G17 circles and <=one-turn
helices with exactly matching decoded radii; no undocumented center correction.
Full-circle planar endpoint words are omitted as the manual requires; a full helix
retains its Z endpoint. Plain G1/F clearance moves retain rapid assembly clearance
and no-removal intent; actual F stays in checks and playback.

Bind Setting33 (FANUC/HAAS), Setting15 T/H agreement, a positive traverse feed,
full work-to-machine translation, installed Rotation and Scaling option, actual tool/H/gauge/pose tables, and a declared
zero G92 shift. G52 X0 Y0 Z0 clears G52; it does not clear an unknown G92 shift.
No fabricated G92.1 reset or G91.1/G97/G61.1 aliases. Fixed Cartesian three-axis
setup excludes dynamic work offsets/coupled kinematics; the reference clears
programmable mirror image explicitly, and rotation/scaling when the installed
option supports G50/G69. With that option absent those modes are unavailable and
its cancel codes are omitted. Settings45..47 must be declared OFF. Stored work offsets,
length geometry+wear, G92 and machine setup remain external measured premises.

Initialization: framed `%`/O program, explicit units/G90/G17/G94/G40/G49/G80,
G52 X0 Y0 Z0, conditional G50/G69, G100 X0 Y0 Z0, plain G187 reset, G61 exact stop,
M5/M9. Decoder requires the closing percent after M30 before file completion.
G61 supplies the documented nominal stop at every commanded endpoint. No G64,
G187 P/E blending model or automatic corner rounding/chamfer words. This proves
nominal paths under the declared model, not servo/acceleration/physical accuracy.

M6 and M0/M1 stop spindle/coolant on Haas. Require explicit source spindle/coolant
off before them so those side effects cannot silently change the typed state.
Optional stop requires an explicitly enabled setting/panel premise. Tool changes
select T then M6 then positive G43/H; T=H is enforced when Setting15 is declared ON.
The M6 Z/tool-change route and post-selection pose are external premises, not
swept-motion verified. Any OEM macros must preserve the initialized units, frames
and path policy; their instructions are not interpreted by this reference.
M7 is shower coolant here, so it is not relabeled as mist;
the reference supports M8/M9 flood/off only. Finish M5/M9/G49/M30/closing `%`.

Coordinate precision is at most3metric/4inch decimal places, consistent with the
manual's minimum programmable dimensions. Dimensional words always include a
decimal point, including integral coordinates/IJK/F, so Setting162 DefaultToFloat
and Setting77 ScaleIntegerF cannot reinterpret them. The decoder rejects ambiguous
nonzero undotted words; zero reset words are invariant under either convention.
F is explicit on every motion. Physical
T/H table subset is1..200 (H table authority below); larger ATM groups/options are
outside this reference. Program numbers use the five-digit-or-shorter O notation;
the owner must choose an available nonreserved program slot. Machine-specific
feed/RPM/travel limits remain configurable. Supported exact circles are a bounded
reference option; general normal jobs use the default bounded line policy.

## References and local checks

The primary reference is [Haas 96-8210 Rev N, December 2020](https://haascnc.com/content/dam/haascnc/en/service/manual/operator/english---mill-ngc---operator%27s-manual---2020.pdf).
Relevant PDF pages: 330–335 interpolation; 354/359 length compensation;
365/387 work shifts; 370 optional rotation; 432–434 stops/tool changes/coolant;
466/467/470 units and settings 15/33; 472 mirror settings; 476/484 numeric notation;
287 H geometry/wear table. The later 100.21.000.1100 persistent-length behavior
requires separate evidence and a model; this 2020 profile does not qualify it.

`python/tests/test_haas_ngc_post.py` and `python/tests/haas_ngc_oracle.py` compare
literal emitted words, settings/units, curve lowering, retry/cancellation and
actual posted stock. The literal oracle applies both Setting162 states and all
six Setting77 conventions. Local nominal checks do not establish external
controller or machine qualification. General G92 rebasing, blending, cycles,
macros, probing, additional axes, returns and mid-program restart are outside
this profile. See [postprocessor architecture](posts.md) for workflow references.
