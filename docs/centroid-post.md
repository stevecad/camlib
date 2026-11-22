# Centroid CNC12 milling post

`examples/centroid_post.py` writes programs for Centroid CNC12 mill controls
(Acorn, AcornSix, Hickory, Oak, Allin1DC, MPU11, M400, M39), following the
*Centroid CNC12 Milling Machine Operators Manual* for CNC12 v5.42 and later.
Like the other controller posts, it is ordinary Python: `CentroidPost` writes
blocks, and `CentroidDecoder` reads the written blocks back independently so
that every move is checked against the source toolpath.

```python
import math
from centroid_post import define_machine, post_centroid

# camlib assumes nothing about the machine: declare it.
declared = dict(
    output_units="mm",
    max_feed_mm_min=10000,
    max_spindle_rpm=12000,
    planes=("xy", "xz", "yz"),
    arcs=True,
    helices=True,
    max_sweep_rad=math.tau,
    max_tool_number=99,
    max_length_register=99,
    tool_changes=True,
    flood=True,
    mist=False,
    rapid_mm_min=10000,
)
machine = define_machine("Acorn mill", tool_settings, traverse_feed_mm_min=3000,
                         work_to_machine=frame, rigid_tapping=True, decimals=3, **declared)
for chunk in post_centroid(program, machine):
    ...
```

## What it writes, and why

| Topic | Choice | Manual |
| --- | --- | --- |
| Traverse | `G1` at `traverse_feed_mm_min`; `G0` is refused | 12.1: `G0` moves Z first when going up and last when going down (a dogleg), not a straight line |
| Path mode | `G61` at the start | 12.23–12.24: `G61` decelerates and stops every block and turns smoothing off; smoothing (`G64`) is the default otherwise |
| Arcs | `G2`/`G3` with incremental `I J K` | 12.3: I/J/K are incremental unless Parameter 2 bit 0 is set (a declared premise). CNC12 documents no start/end radius tolerance, so words must be exactly consistent (`radius_tolerance_mm=0`); post with `lowering=LoweringOptions(native_arcs=True)` so other arcs become checked segments |
| Local offset | `G52 X0 Y0 Z0` | 12.20 |
| Comments | `;` | 11.2.13; parentheses are not comments |
| Tool change | `T… M6`, then `G43 H…` | M6 moves Z home first (13.2): declare `tool_change_position_mode="z_home"` with `tool_change_home_z_mm`, and plan with `Setup(tool_change=ToolChangePosition(...))`; or state each tip pose after M6 in the tool table |
| Program end | `M30` | 13.19: M30 is the end-of-program code; `M2` restarts the program and is refused |
| Cutter compensation | `G41 D…`/`G42 D…` on its own line before the lead-in, `G40` before the lead-out | 12.15: it takes effect over the next move; D holds the wear |
| Dwell | `G4 P<seconds>` | 12.4: 0.01 to 327.67 s; rounded up to 0.01 s |
| Drilling | `G98`, then `G81 X Y Z R F` or `G82 … P<seconds> F` per hole, then `G80` | 12.28, 12.32, 12.34, 12.44 |
| Boring | `G85 X Y Z R F`, or `G89 … P<seconds> F` with dwell | 12.37–12.38; CNC12 has no G86, so stop-boring is written as `M5`, a traverse out and `M3` |
| Fine boring | `M19`, traverse moves away from the edge, out and back, then `M3` | 13.16: M19 is a custom macro; declare `oriented_edge_deg` (where the bar's edge points after it). G76 (12.31) is not written: the manual does not say whether it shifts back or restarts the spindle |
| Peck drilling | `G83 X Y Z R Q F` | 12.35: written only with `peck_clearance_mm` = Parameter 83; Q always has a decimal point (Parameters 245/246) |
| Chip-break drilling | `G73 X Y Z R Q F` | 12.29: written only with `chip_break_retract_mm` = Parameter 73 (set by `G10 P73 R`); the post never writes G10 |
| Rigid tapping | `G84 X Y Z R Q<lead>`, then `G80` | 12.36: Q must be on every line and is written with a decimal point; requires `rigid_tapping=True` (Parameter 36) |

Tapping overruns: the manual warns that the tap may cut a little below the
programmed Z while the spindle reverses. camlib's `Tapping` requires the
drilled hole to extend below the tap depth by `bottom_clearance_mm` (one
pitch by default); set it to suit the machine.

Program end position: with `Setup(end_position=EndPosition(...))` the end
moves are written before `M9`/`M5`/`M30` as this post's traverses (CNC12
rapids are written as `G1` at the declared `traverse_feed_mm_min`, like every
other rapid here) and decoded the same way. See
[Posts](posts.md#program-end-position).

Centroid's lathe controls (T-series) have their own post: [Centroid lathe post](centroid-lathe-post.md).
The premises (traverse feed, tool poses after M6, Parameters 2 and 36) must
match the installed machine; nothing is read from the control.
