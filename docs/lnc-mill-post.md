# LNC milling Python reference

`examples/lnc_mill_post.py` is an ordinary Python machine, post and decoder for
one bounded three-axis subset of LNC milling controls (LNC-M8000 series). The
source is the **A-LNC Milling Machine Series Programming Manual, V01.02.000
(4408230009), 2016/05, Advantech-LNC Technology Co., Ltd.** Page numbers below
are the manual's printed pages. The manual is not redistributed here. This
reference does not establish compatibility with other LNC series or with a
machine builder's PLC configuration, and it has not run on a real LNC control
(#20).

## What the manual fixes

| Subject | Pages | What the post does |
| --- | --- | --- |
| G-code and M-code tables | 1–3 | Writes only codes listed there, plus the builder functions you declare |
| Units | 18 | `G21` only; there is no G20, so inch output refuses |
| Least increment | 18 | 0.001 mm: at most 3 decimals, a decimal point on every dimensional word |
| G00 | 5 | Not written. Each axis runs at its own rapid rate and the manual does not say the path is straight, so traverses are `G1` at the declared `traverse_feed_mm_min` |
| G02/G03 | 7 | Native arcs in G17, G18 and G19 with incremental I/J/K, at most 180° per block |
| Arc end-point check | 7 | Alarm 510204 when the radii differ by more than parameter 50048: declared as `arc_radius_tolerance_mm`, with no relative allowance |
| G04 | 9 | Not written (X is seconds but X is an axis word; P is integer ms). Drilling dwell uses G82/G89 P |
| G43/G49 | 38 | `G1 G43 H Z F` and `G1 G49 Z F`, the Z keeping the physical tip still. H is 1..400 |
| G52 | 48 | `G52 X0. Y0. Z0.` clears the local coordinate system at the start |
| G53 | 50 | Not written: G53 **cancels the active tool length compensation**. The decoder refuses it |
| G54–G59 | 51 | The setup's work offset |
| G61/G64 | 53 | `G61` exact stop (G64 is the default) |
| Canned cycles | 63–170 | One block per hole after `G98`, cancelled by `G80`, always in G17 (the post selects G17 after a G18/G19 arc) |
| Rigid tapping | 71, 76 | `M29 S` before G84/G74 and `M28` before G80, as the manual's example |
| Program end | 171 | `M30`. No `%` or `O` line: the manual's sample program has neither |

The manual's M table (p.3) lists only M00, M01, M02, M30, M98 and M99. Spindle,
coolant, tool change and spindle orientation are the machine builder's PLC
functions, so you declare them (`m_codes`). They must be distinct, between M1
and M999, and not one of the codes the manual reserves (M28/M29 with rigid
tapping, M19 with fine boring).

## Machine parameters you declare

Several cycle motions are set by controller parameters, not by the program.
camlib checks the posted program against your values, so they are required;
a cycle whose parameter is not declared is written as explicit moves.

| Parameter | Meaning | Machine field |
| --- | --- | --- |
| 50048 | Accepted G02/G03 end-point radius error | `arc_radius_tolerance_mm` (required) |
| 160031 | G83 re-entry clearance above the last peck (p.108) | `peck_return_mm` |
| 160030 | G73 retract after each peck (p.63) | `chip_break_return_mm` |
| G76 shift direction | Direction of the Q shift after M19 (p.83) | `fine_bore_edge_deg`: where the bar's edge points; the parameter must shift the opposite way |
| — | Rigid tapping with M29 | `rigid_tapping_m29=True`; without it tapping refuses |

The operation's `reentry_clearance_mm` or `chip_break_lift_mm` must equal the
declared value, or posting stops and says which to set.

## Two errors in the manual

- **G76 shift direction.** The argument list names parameter 170003 for the
  offset direction (p.83), and step 6 of the same page names 160010. Declare
  the direction your machine actually shifts (`fine_bore_edge_deg`) and check
  it on the machine.
- **M29 with G74.** The G74 page (p.71) says that M29 before G74 gives
  "right-hand thread rigid tapping". G74 is the left-hand cycle (the spindle
  reverses into the hole), and its own operation steps and the p.76 example
  say so. The post writes G74 under the counterclockwise spindle for
  left-hand taps and G84 under clockwise for right-hand taps.

## Define a machine

```python
import math
from camlib.motion import WorkFrame
from lnc_mill_post import define_machine, post_lnc

machine = define_machine(
    "LNC mill", tools,
    startup_standard_modes=True,
    g92_shift_mm=(0, 0, 0),
    m_codes=dict(tool_change="M6", spindle_cw="M3", spindle_ccw="M4",
                 spindle_off="M5", coolant_off="M9", flood="M8"),
    arc_radius_tolerance_mm=0.002,   # your parameter 50048
    traverse_feed_mm_min=3000,
    work_to_machine=WorkFrame(),
    peck_return_mm=0.5,              # your parameter 160031
    chip_break_return_mm=0.2,        # your parameter 160030
    rigid_tapping_m29=True,
    output_units="mm", decimals=3,
    max_feed_mm_min=10000, max_spindle_rpm=12000,
    max_sweep_rad=math.pi,
    tool_changes=True, flood=True,
    tool_change_position_mode="table", rapid_mm_min=10000,
    max_tool_number=24,              # your magazine (T range is the builder's)
)
for chunk in post_lnc(program, machine):
    consume(chunk.text, chunk.motions)
```

Or pass `post=LNCMillPost, decoder=LNCMillDecoder` to `Job.post_nc`. Arcs
are written natively. Helices and arcs over 180 degrees are refused unless
you ask for checked line segments with
`post_lnc(program, machine, lowering=LoweringOptions(native_arcs=True))`. Copy
`examples/lnc_mill_post.py` beside your script or add `examples` to Python's
module path. The M codes above are examples, not LNC assignments: use your
builder's.

`startup_standard_modes=True` declares that the control starts in G49, G40,
G15, G50, G50.1, G69, G67 and G64 with no G92 shift. The H register at each
tool's `length_register` must hold that tool's declared gauge length. The
tool table states the actual tip pose after M6; the builder's tool-change
route itself is a machine premise for your crash model.

A program looks like:

```
G21
G90
G17
G94
G40
G49
G80
G52 X0. Y0. Z0.
G61
M5
M9
G54
T2
M6
G1 G43 H7 Z3. F3000.
M3 S12000
M8
G1 X10. Z3. F1200.
G2 X20. Y0. I5. J0. F1200.
...
M5
M9
G1 G49 Z63. F3000.
M30
```

## Program end position

With `Setup(end_position=EndPosition(gauge_z_mm=..., xy_mm=...))` the planned
end moves (a Z rapid to the gauge height, then an XY rapid) are written as
this post's ordinary traverses (`G1` at the declared `traverse_feed_mm_min`,
like every other rapid here) after the last operation, before the spindle
and coolant stop, and decoded like any other block. The length-cancel block
that follows states the gauge line's Z, which is then the stated one. No
reference-point return is written. See [Posts](posts.md#program-end-position).

## Checks

Every posted block is read back by `LNCMillDecoder`, which imports no
formatter code, and compared with the plan. It refuses G20, G53, G0, bare
G43/G44/G49, a compensation block that would move the tip, an arc whose end
point is off the circle by more than parameter 50048, a dimensional word
without a decimal point, a P word with a decimal point, a canned cycle outside
G17, G84/G74 without the `M29 S` block directly before it (repeating the
running speed), and G80 before `M28`.

`python/tests/test_lnc_mill_post.py` also runs every program through
`lnc_mill_oracle.py`, a literal reading of the manual's words (lines, arcs in
each plane, compensation blocks) that shares no code with the post or the
decoder. It covers a literal program (`fixtures/posts/LNC-MILL-ARCS.NC`),
mutations of every word class, cycles with and without declared parameters,
rigid tapping in both hands, two tools, and a complete pocket whose posted
program is replayed through the stock and material checks.
