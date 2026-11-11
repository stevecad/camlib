# FANUC 0i-MB Python reference

`examples/fanuc_0imb_post.py` is an ordinary Python machine/post/decoder extension.
It supports one bounded three-axis milling subset of **FANUC 0i-MB,
B-63844EN/02, February 2003**. It does not establish compatibility with 0i-F,
0i-F Plus, 30i/31i/32i, or an unspecified machine builder's configuration.

## Primary reference and acquisition state

The [FANUC-authored operator's manual, public PDF mirror](https://manuals.plus/m/1e49b0bc11890a1b3918373ae05dafb5209088805ba3e59e8120b907f8f8edc7.pdf)
was inspected through the web PDF reader on 2026-11-02. The indexed document has
905 PDF pages; its revision record is PDF page883. Direct downloads returned403,
so there is **no locally acquired complete PDF or whole-PDF SHA256 claim**. This
is primary manufacturer-authored content delivered by a third-party mirror, not
a manufacturer-portal acquisition. Reader excerpts and failed acquisition attempts
are retained privately; the manual is not redistributed in the library.

| Subject | Printed pages (PDF pages) |
| --- | --- |
| Rapid versus linear interpolation | 37–42 (61–66) |
| Feed per minute / exact-stop mode | 61,65–66 (85,89–90) |
| Stored/local work coordinates | 79–89 (103–113) |
| Absolute, unit and decimal conventions | 92,96–97 (116,120–121) |
| Builder-defined M functions and stops | 114–116 (138–140) |
| Program framing | 118–119 (142–143) |
| Length-offset A and activation/cancellation | 199–206 (223–230) |
| Dwell word P (whole milliseconds, no decimal point) | 70 (§5.5) |
| Canned drilling G73/G81/G82/G83, G98, G80 | 134–149 (§13.1–13.1.6) |
| Canned boring G85/G86/G89 | 156–159, 164–165 (§13.1.9–13.1.10, §13.1.13) |
| Left-handed tapping G74; rigid tapping G84/G74 | 140 (§13.1.2), 169–175 (§13.2–13.2.2) |

G43/H and G49 can cause axis movement. The reference therefore uses explicit
G1 compensation blocks whose spindle-axis endpoints preserve the declared tip
position. G52 clears only the local shift; a zero G92 shift remains a setup
premise. Units agree with startup offset-table units. Decimal points avoid DPI
parameter ambiguity. G61 selects exact-stop mode without asserting zero physical
servo error. M-code functions and ATC routes depend on the builder.

## Canned drilling cycles

Drilling and reaming/boring with `canned_cycle=True` are written as one block per hole after
`G98`, cancelled by `G80`:

| Cycle | Block | Notes |
| --- | --- | --- |
| Drill, spot drill | `G81 X Y Z R F` | |
| Dwell, counterbore | `G82 X Y Z R P F` | P in whole milliseconds, rounded up |
| Peck | `G83 X Y Z R Q F` | Written only with `peck_return_mm` = parameter 5115 (d) |
| Chip break | `G73 X Y Z R Q F` | Written only with `chip_break_return_mm` = parameter 5114 (d) |
| Boring | `G85 X Y Z R F` | Feeds back out (§13.1.9) |
| Boring with dwell | `G89 X Y Z R P F` | P in whole milliseconds (§13.1.13) |
| Stop boring | `G86 X Y Z R F` | Spindle stop at depth, rapid out, spindle restart (§13.1.10) |
| Fine boring | `G76 X Y Z R Q P F` | Written only with `fine_bore_edge_deg` declared: where the bar's edge points after the oriented spindle stop. Parameter 5101 RD1/RD2 must shift the opposite way; Q has a decimal point, P in whole milliseconds (§13.1.3) |

Without the parameter value declared, pecking holes are written as explicit
moves. The operation's `reentry_clearance_mm` or `chip_break_lift_mm` must equal
the declared value, or posting stops with that message. The small-hole G83 mode
(§13.1.7) needs the M code from parameter 5163, which this post never writes.
Standalone dwell (`G04`) is not declared; drilling dwell uses `G82`.

Rigid tapping needs `rigid_tapping=True`, the premise that parameter No.
5200#0 (G84) is 1, so G84 and G74 are rigid tapping cycles without `M29`
(§13.2.1–13.2.2). A right-hand tap is written `G84 X Y Z R F` under `M3`, a
left-hand tap (`Tapping(hand="left")`) `G74 X Y Z R F` under `M4`, each
followed by `G80`. F = lead × S (G94).

Machines that keep 5200#0 = 0 declare `rigid_tapping_m29=True` instead: rigid
mode is then selected by `M29 S…` in the block before each cycle, the form of
the manual's examples (§13.2.1–13.2.2), and S repeats the running spindle speed:

```
M29 S600
G84 X8. Y8. Z4. R11. F600.
G80
```

The decoder requires the `M29 S` block directly before every `G84`/`G74`
(an S word or axis move between them is P/S alarm 203/204, Appendix G),
refuses a cycle without it and an `M29` whose S differs from the running
speed, and refuses `M29` under `rigid_tapping=True`. Under this premise M29
cannot be bound to an OEM function. The two premises are exclusive; with
neither, tapping is refused.
Cutter compensation is started and cancelled in the block of the straight XY
lead move (§14.4): `G1 G41 D… X… Y… F…` and `G1 G40 X… Y… F…`. Offset memory A
keeps one value per number for H and D, so every tool's D register must
differ from all H registers (checked).

## Define a machine

```python
import math
from camlib.motion import WorkFrame
from fanuc_0imb_post import define_machine, post_fanuc

# camlib assumes nothing about the machine: declare it.
declared = dict(
    output_units="mm",
    decimals=3,
    max_feed_mm_min=10000,
    max_spindle_rpm=12000,
    max_sweep_rad=math.tau,
    tool_changes=True,
    # Where the builder's M6 leaves the spindle (see "Tool change pose").
    tool_change_position_mode="z_home",
    tool_change_home_z_mm=gauge_line_machine_z_after_m6,  # measured on this machine
    rapid_mm_min=10000,
)
machine = define_machine(
    "Three-axis mill", tools,
    startup_units="mm",
    startup_standard_modes=True,
    g92_shift_mm=(0, 0, 0),
    m_codes=dict(tool_change="M6", spindle_cw="M3", spindle_ccw="M4",
                 spindle_off="M5", coolant_off="M9", flood="M8"),
    traverse_feed_mm_min=3000,
    work_to_machine=WorkFrame(), **declared)
for chunk in post_fanuc(program, machine):
    consume(chunk.text, chunk.motions)
```

Copy `examples/fanuc_0imb_post.py` beside the calling script or add `examples` to
Python's module search path. These are example builder bindings, not universal
FANUC M-code assignments. `tools` contains ordinary `ToolSetting` entries. The
controller offset-table contents at each configured H register must equal the
declared physical gauge length; H7 selects register 7, not a length of 7 mm.
Where the tool change leaves the tip is declared as described under
[Tool change pose](#tool-change-pose).
`startup_standard_modes=True` explicitly declares length-offset A with G49,
Cartesian coordinates, no scale/rotation/mirror, and no macro/AI contour mode.
The profile does not alter controller parameters or offsets.

## Tool change pose

M06 runs the machine tool builder's tool-change macro (M functions are the
builder's, pages 114–116), so the manual does not say where the spindle is
afterwards. camlib needs it: the first block after `M6` is the `G1 G43 H… Z…`
block, whose Z word must be the tip's position at that moment so that
activating the offset moves nothing. Declare it in one of two ways; any other
`tool_change_position_mode` is refused.

| Declaration | What it states | Each tool's tip after M6 |
| --- | --- | --- |
| `tool_change_position_mode="z_home"`, `tool_change_home_z_mm=<Z>` | One fact about the machine: M6 leaves the spindle gauge line at this machine Z, and X and Y where they were | work Z = `tool_change_home_z_mm` − work offset Z − the tool's gauge length |
| `tool_change_position_mode="table"` | One measured pose per tool: `ToolSetting.position_after_mm` | as stated |

With `z_home`, a tool table entry needs only the tool, its T and H numbers and
its gauge length (`ToolSetting.for_tool(tool, number=3, length_register=8)`).
Plan the job with the matching
`Setup(tool_change=ToolChangePosition(gauge_z_mm=<home Z minus work offset Z>))`,
as for any `z_home` machine (see [Postprocessors](posts.md)). For a gauge line
at work Z 80, a 60 mm tool and a 70 mm tool are written:

```
T2
M6
G1 G43 H7 Z20. F3000.
...
G1 G49 Z80. F3000.
T3
M6
G1 G43 H8 Z10. F3000.
```

A `z_home` entry that also states `position_after_mm` must state the same tip
Z (home Z − work offset Z − gauge length); if the two disagree, posting stops
and names the tool and both values, since one of the gauge length, the home Z
and the measured pose is wrong. Its X and Y are not read: `z_home` keeps X and
Y where the change was called. A `table` entry without `position_after_mm` is
refused. The derived tip Z, like a measured one, must be representable at the
output precision. A machine whose M6 also moves X and Y to a fixed position is
declared with `table`, each entry `(x, y, gauge line Z − gauge length)`.

The decoder does not take the pose from the post: it places the tip from the
same machine declaration at `M6` and refuses a `G43 H Z` block whose Z differs
from it, and the posted-output check compares each decoded tool change with
the planned one.

Output uses G17/G90/G94/G40/G49/G80, explicit G52 zero, G54–G59, G61, and G1.
Curves/helixes in any source plane are lowered with the existing adjustable
continuous-deviation budget (default0.01mm total,0.0025mm chord). G0, native arcs,
cycles, reference returns, transformations and controller compensation paths are
outside this profile. It emits at most3metric/4inch decimals, bounded T/H1–200,
and O1–7999. Tool-table poses/gauge lengths must be representable at that precision
so a compensation-only transition does not silently introduce a tool-tip move.

For example, a tip at Z3 with60mm gauge length activates with
`G1 G43 H7 Z3. F3000.` and cancels with `G1 G49 Z63. F3000.`. Both retain
spindle-axis Z63. The actual-word decoder and independent test oracle reject an
incorrect compensation endpoint, a bare moving G43/G49, or the wrong H entry.
Subsequent motion/stock checks operate on the unchanged physical tip position.
The declared ATC route itself is still an external machine premise.

## Probing with Renishaw Inspection Plus

Probing is not a FANUC function: it runs through Renishaw's Inspection Plus
macros (custom macro B, `G65`), where they are installed. Declare them with
`define_machine(..., inspection_plus_fast_feed_mm_min=<#119>,
inspection_plus_switching=<bool>)`: the fast feed #119 from the settings macro
O9724, and whether each cycle switches the probe on and off with
`G65 P9832`/`P9833`. Without them, probing cycles are refused. The calls,
their decoding and their premises are in
[Postprocessors: Probing](posts.md#probing) (Renishaw H-2000-6031 chapters 5,
7 and 9): surfaces, outside and inside corners, bores and pockets correcting
G54..G59 (`S1`..`S6`); bosses, webs and tool setting are refused here.
Arguments are written with their decimal point (`G65 P9814 D12. S1. Q2.`), as
the manual's examples are, so no argument depends on the least input
increment.

## Local conformance

`python/tests/test_fanuc_0imb_post.py`, its literal `fanuc_0imb_oracle.py`, and
`fixtures/posts/F0IMB1000.NC` exercise:

- Literal output, physical compensation endpoints, mm/inch and both DPI meanings.
- Alternate explicit OEM functions, two tools, stops and atomic output retry.
- Two tools from a declared gauge-line pose (`z_home`) without measured
  positions: the same file as the equivalent measured table, and refusal of a
  disagreeing or missing declaration.
- Hostile modes, units, WCS, H values, implicit moves and dimensional notation.
- Bounded all-plane/multiturn lowering, source attribution and cancellation.
- A complete synthetic pocket: independently decoded output and actual native
  stock/contact/material checks while the source is still producing.

The batch also found a generic lifecycle bug: `PostEngine.finish()` supplied the
start of the last event to coordinate-aware finish hooks. It now supplies the
actual decoded terminal position, including output rounding. The generic
regression checks both decoders, direct/lowered posting, rollback and cancellation.

This is local nominal conformance. There has been no external FANUC interpreter,
CNC GUIDE or physical machine qualification. Newer controller generations are outside this profile.
