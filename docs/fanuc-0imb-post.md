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
followed by `G80`. F = lead × S (G94). Without the premise, tapping is
refused; the `M29 S` form is not written.
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
    tool_change_position_mode="table",
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
The supplied poses are the actual tip positions after ATC.
`startup_standard_modes=True` explicitly declares length-offset A with G49,
Cartesian coordinates, no scale/rotation/mirror, and no macro/AI contour mode.
The profile does not alter controller parameters or offsets.

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

## Local conformance

`python/tests/test_fanuc_0imb_post.py`, its literal `fanuc_0imb_oracle.py`, and
`fixtures/posts/F0IMB1000.NC` exercise:

- Literal output, physical compensation endpoints, mm/inch and both DPI meanings.
- Alternate explicit OEM functions, two tools, stops and atomic output retry.
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
