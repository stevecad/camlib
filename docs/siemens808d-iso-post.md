# Siemens 808D ISO milling Python reference

`examples/siemens808d_iso_post.py` defines a machine, formatter and actual-word
reader in ordinary Python. It uses the generic engine's bounded lowering,
transactional checks, attribution and streaming; there is no new Rust controller
branch. This profile is distinct from `siemens808d_post.py` (native language).

The authority is the acquired **02/2016 ISO Turning/Milling manual** (software
4.7.1/4.6.2), supplemented by the same-edition parameter and native milling manuals.
Primary programming reference: [02/2016 ISO manual](https://cache.industry.siemens.com/dl/files/339/109736339/att_880241/v1/SINUMERIK_808D_ADVANCED_ISO_022016_eng.pdf).
These editions do not establish the installed version of any particular machine.

## Use from Python

```python
import math
from camlib.motion import WorkFrame
from siemens808d_iso_post import define_machine, post_iso

# tools contains ToolSetting objects with the actual T -> ISO H mapping and
# declared tip pose after each OEM tool change. program is a camlib Program.
# camlib assumes nothing about the machine: declare it.
declared = dict(
    decimals=3,
    max_feed_mm_min=10000,
    max_spindle_rpm=12000,
    max_sweep_rad=math.tau,
    max_tool_number=99,
    max_length_register=99,
    tool_changes=True,
    tool_change_position_mode="table",
    rapid_mm_min=10000,
)
machine = define_machine(
    "Three-axis mill", tools,
    startup_units="mm", output_units="mm",
    extern_toolprog_mode=0, cutting_edge_default=0,
    tool_corr_move_mode=0, tool_length_const=17,
    tool_change="m6", coolant_codes={"off": "M9", "flood": "M8"},
    traverse_feed_mm_min=3000,
    work_to_machine=WorkFrame(origin_mm=(100, 200, 300)), **declared)
for chunk in post_iso(program, machine):
    consume_checked_nc(chunk.text, chunk.motions, chunk.data)
```

The values above illustrate a configured machine, not universal machine defaults.
The example module must be on Python's import path. `post_events_iso` accepts a
header and streamed event iterator, and accepts the generic cancellation and
lowering options. Default curves are lowered with the existing 0.0025 mm chord
allocation inside the adjustable 0.01 mm source-to-output budget. `curve_policy=
"exact_ijk"` emits G17 circles and helices up to one revolution when representable
within that same budget, with zero radius-repair allowance. Other-plane and
multiturn paths can use bounded line lowering. No canned drilling cycle is emitted;
tapping uses G84/G74 (below).

## Semantics and setup premises

| Behavior | Manual evidence and implementation |
| --- | --- |
| Language/state | ISO p8: standalone G290/G291 preserve tools, lengths and work offsets. Startup enters native mode, disables compression and modal rounding, selects cutter-center feed and the fine stop window, then enters ISO mode. Native prep uses the 02/2016 native manual's COMPOF, RNDM=0, CFTCP and G601. No mixed-language blocks are accepted. |
| Units | ISO p110: G20/G21 are standalone; changing units requires additional frame/offset handling. This profile emits the already active declared units and rejects a runtime change. Work/tool tables and the declared pose are supplied in mm through the generic API. |
| Numeric notation | ISO pp9–11: MD10884 and MD10886 change undotted numbers. All dimensional XYZ/IJK/F words include a decimal, at up to three metric/four inch decimal places, so both calculator/standard and IS-B/C interpretations agree. |
| ISO milling length | ISO pp114–116: H selects the global `$TC_DPH[t,d]` map, G43 adds length, G49 cancels. Parameter manual MD10890 pp50–51 explicitly scopes bit2's H/D selection to **turning**; the general ISO p8 note is not used to invent a milling D mode. Bits2/6 configurations are recorded; radius compensation remains G40. |
| Offset traversal | MD20270=0 means no implicit cutting edge after T; MD20382=0 means offset traversal requires the corresponding axis to be programmed. SD42940=17 binds length to Z. Thus the standalone G43 H and G49 blocks do not introduce an unreported axis move. Other settings need a different decoder/motion model. |
| Tool changes | Explicit OEM direct-T or T/M6 choice. H must agree with the supplied table. Post-change physical tip poses are supplied by the caller, as for the existing generic engine; OEM tool-change routes are not simulated by this reference. |
| Work frames | ISO pp104–113: G50/G69/G50.1 cancel scale, rotation and mirroring before G92.1 X0. Y0. Z0. resets all three G92/G52 shifts. Bare G92.1 is insufficient. Stored G54–G59 offsets must match the supplied Cartesian translation. |
| Traverse | ISO pp95–96: G0 can move axes independently unless MD20732 enables interpolation. This reference emits explicitly fed G1 traverses and checks their actual paths. It does not guess MD20732. |
| Circles/helix | ISO pp97–100: G17, G2/G3, signed incremental I/J, absolute endpoints, explicit F; one-turn arcs/helix, no native TURN syntax, radius R or contour chamfer/rounding words. |
| Stop/feed/spindle | ISO pp122–125: G94, explicit S with M3/M4, G61. Coolant codes are declared OEM functions. Tool changes and stops require spindle/coolant already off; optional stop requires declared enablement. Shutdown explicitly stops spindle/coolant and cancels length before M30. |

## Rigid tapping: G84 and G74

A `Tapping` operation posts `G98 G84 X Y Z R F` (right hand) or `G98 G74 ...`
(left hand) per hole, each followed by `G80`. In ISO milling mode these cycles
are rigid by themselves: the manual titles them "drilling a right-hand (left-hand)
thread without any compensating chuck" and the cycle itself makes the oriented
spindle stop and the change to axis mode. No M29 is written. M29 is listed
(p.123) only as "changeover of spindle to the axis/open-loop control mode", its
number set by MD20095, and no page asks for it before G84/G74.

The cycle depends on settings the program cannot set. Each is a declared premise,
and tapping is refused while any is undeclared:

```python
machine = define_machine(
    ...,
    position_controlled_spindle=True,  # pp.143, 145
    tap_deep_hole_mode=0,              # GUD _ZSFI[1]
    tap_retract_speed_percent=100,     # GUD _ZSFI[2]
    tap_safety_clearance_mm=2,         # GUD _ZSFR[0]
    tap_restores_spindle=True,         # the spindle turns as before after the cycle
)
```

For an M6 x 1 hole from Z11 to Z4 at 600 rpm with a 2 mm safety clearance:

```
M3 S600                      (M4 S600 and G74 for a left-hand tap)
G1 X8. Y8. Z11. F3000.
G98 G84 X8. Y8. Z4. R9. F600.
G80
```

| Behaviour | Page | What the manual states, and what the post does |
| --- | --- | --- |
| Rigid cycles, hand | 91, 126, 143, 145 | G84 is the "right tapping cycle" and G74 the "left tapping cycle" (group 9); "with G84 you can produce rigid tapping", "with G74 you can produce left rigid tapping". The decoder requires M3 running for G84 and M4 for G74, the planned hand |
| Spindle requirement | 143, 145 | The cycles "can be used if the spindle ... is technically able to be operated in the position-controlled spindle mode": `position_controlled_spindle=True` |
| Words | 143, 145 | `G84 X Y Z R P F K`: X, Y the hole, Z the bottom, R the R plane, F the cutting feed. The dwell `P` and repetitions `K` are not written; the manual's examples omit them. "The drilling cycle is executed only if an axis motion ... is programmed": X, Y, Z and R are always written |
| Feed and lead | 145, 147 | In G94 F is the feed per minute (the example: F1000 at S1000 is a 1.0 mm lead). The post writes the planned feed, lead x rpm; the decoder reads the feed from F |
| Sequence | 144, 146 | "Approach of reference plane shifted by the amount of the safety clearance with G0", oriented spindle stop and axis mode, tapping to depth, dwell, "retraction to the reference plane and reversion of direction of rotation brought forward by the safety clearance", "retraction to the retraction plane with G0" |
| Safety clearance | 130, 132, 144 | "The safety clearance can be specified with GUD _ZSFR[0]". The post writes R as the starting Z minus `tap_safety_clearance_mm`, so the approach to R plus the clearance is no move and tapping starts where the plan starts it; the decoder checks exactly that |
| Return plane | 92, 128 | G98 returns to the initial plane, G99 to point R; "normally G98" is not relied on: G98 is written in the cycle block, as the manual's examples write G98/G99 with the cycle. The initial plane is the starting Z |
| Deep-hole variants | 147-149 | `_ZSFI[1]` = 2 or 3 turns G84/G74 into deep-hole tapping with a Q infeed. This reference taps in one pass and accepts only `tap_deep_hole_mode=0` |
| Retraction speed | 144, 146 | `_ZSFI[2]` scales the retraction speed (120 is 120 %). Only `tap_retract_speed_percent=100` is accepted, so the return is at the tapping feed |
| Deselection | 128, 129, 149 | The cycles are modal; G80 deselects them. `G80` follows every hole, and the decoder refuses any other block before it |
| Spindle afterwards | 126 | The cycle table gives "spindle turns in the opposite direction" after the reversed return, so the spindle turns again in its earlier direction. The speed afterwards is not stated: `tap_restores_spindle=True` is the declared premise that it is the earlier speed |

Not stated by the 02/2016 ISO manual, and therefore refused or declared rather
than assumed: what `_ZSFI[1]` values other than 2 and 3 select (only 0 is
accepted), whether `_ZSFI[2]` = 0 means 100 % (only 100 is accepted), the unit
of `_ZSFR[0]` under G20 (tapping is refused with inch output), the spindle speed
after the cycle (declared), and that the G84/G74 pages name `_ZSFR[0]` (they say
"the safety clearance"; the GUD is named on pp.130 and 132 for G73 and G76).

`Siemens808DISODecoder` expands the cycle block from its written words into the
tap to Z and the return, and refuses a cycle whose hand is not the running
spindle direction, whose R plus the declared clearance is not the current Z, or
that is not followed by `G80`.

Acquired manuals and locally checked interpretation are separate from external
interpreter and physical machine qualification. Neither external nor physical
qualification is claimed. Workholding, OEM routes, persistent external/handwheel
shifts and calibration remain supplied machine/setup premises. This profile does
not add general five-axis, probing, macros, transformations, drilling or boring
cycles or machine return commands.

## Evidence

`python/tests/test_siemens808d_iso_post.py` exercises unit/notation combinations,
nonzero stored offsets, T/M6 and direct-T, exact arcs/full circles/helices, bounded
all-plane/multiturn lowering, fed traverses, cancellation/attribution, hostile
modal edits, atomic retry, and G84/G74 tapping in both hands (golden blocks,
deliberately wrong posts, undeclared premises). `siemens808d_iso_oracle.py` independently interprets
literal NC without importing the post, dialect or decoder; its integer-word
scaling follows the milling table and can detect omitted decimal points.
A complete pocket runs through native planning, this post, independent literal
interpretation, continuous scene/contact checks and actual posted stock removal.
These are local nominal checks, not external interpreter or machine qualification.
See [postprocessor architecture](posts.md) for workflow references.
