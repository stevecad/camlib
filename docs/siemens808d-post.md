# SINUMERIK808D ADVANCED native reference post

`examples/siemens808d_post.py` is an ordinary Python machine definition, post class,
decoder and iterator adapter using the generic camlib engine. No Rust controller
registry/branch or copied upstream post code. Target reference: **native milling
software4.90**,07/2018 manual6FC5398-4DP10-0BA7. This is a documented local subset,
not ISO mode, all Siemens software, external firmware conformance or qualification
for any physical machine whose installed version and builder configuration have
not been established.

## Define and post in Python

```python
import math
from camlib.motion import WorkFrame
from camlib.post import ToolSetting
from siemens808d_post import define_machine, post_808d

# camlib assumes nothing about the machine: declare it.
declared = dict(
    output_units="mm",
    decimals=3,
    max_sweep_rad=math.tau,
    tool_changes=True,
    tool_change_position_mode="table",
    rapid_mm_min=10000,
)
machine = define_machine(
    'My808D setup',
    tools=(ToolSetting.for_tool(tool, number=2, length_register=1,
                               position_after_mm=known_tool_change_tip_pose),),
    tool_change='m6',                 # or 'direct_t': match actual machine data
    coolant_codes={'off': 'M9', 'flood': 'M8'},  # actual OEM PLC meanings required
    work_to_machine=WorkFrame(origin_mm=known_base_and_work_offset),
    frame=program.frame,
    max_feed_mm_min=8000,
    max_spindle_rpm=12000, **declared)
for chunk in post_808d(program, machine):
    nc_file.write(chunk.text)
    # Checked decoded motions and original source/fraction/NC mappings.
    simulation.consume(chunk.motions)
```

Use `PYTHONPATH=python:examples` to import this example module. The program's
ToolChange numbers/edge IDs/poses must match the selected machine bindings;
there is no inference/remapping of hardware tool tables. Supply a ToolAssembly
for each cutting-edge geometry/length used in this job and a ToolSetting for
its `(T,D)` pair. Multiple edges may share T; D1 may repeat across different T.
Up to64configured fields, T1..32000,D1..9 in this reference subset. Normalized
programmed endpoints/incremental centers and feed/RPM limits are bounded to99,999.999
(mm,mm/min,RPM) for this subset; actual machine travel/speed limits can be lower.
These software bounds are independently checked on output, not assumed from formatting. Actual stored
calibrations/geometry, base/stored frame and post-selection poses are premises.

The engine's generic per-tool-edge support now accepts distinct `(T,D)` entries,
D-only edge reselection and bundled M6+Dn. Explicit-H dialects retain their global
register/unique-T rules. Source/tool/edge identity and native state still validate.
This is useful to any Python dialect with per-tool edge semantics, not a Siemens
branch. Example post emits `Tn Dd` for direct activation or `Tn` then `M6 Dd`
for preselection/M6. Same-T changes emit only `Dd`. Explicit coactivation avoids
assuming a valid D1 entry when selecting another edge. The decoder tracks implicit
D1 on activation but requires an explicit edge before machining in this subset.

## Supported subset and checks

Startup explicitly selects G290 in its own block; G710/G700 for coordinate **and
feed** units, G90,G17,G94,G40,D0,MCALL,TRANS,COMPOF,G60 with G601/G602,RNDM=0 and
CFTCP. M5 and the caller's OEM coolant-off code are explicit. Shutdown stops
spindle/coolant, cancels D and ends with M2. Native S is RPM and unqualified I/J/K
are relative to the arc start; these native invariants replace LinuxCNC-only
G97/G91.1. No G20/G21/G43/G49/G80/G92.1/G61.1 aliases are emitted as Siemens codes.

The declared length/cutter axis stays vertical/G17. G18/G19 are rejected because
they change the tool-length axis, not merely arc formatting. Initial/reset base
frames, kinematic transforms/couplings and any OEM tool-change macro effects must
match the fixed Cartesian setup; TRANS clears programmable frame components, not
an unknown base frame. The supplied OEM tool-change contract must preserve these initialized modes and
frames (or a custom post/decoder must model and restore them). Tool-change routes
and servo motion are not inferred from endpoint/table values. G60/windows describe nominal stop behavior; real window
sizes are machine data, not zero-error part accuracy or machine-cycle-time proof.

Default `curve_policy='linear'` runs the existing native bounded curve lowering
through ordinary Python hooks. All source planes, circles, helices and multiturn
curves are retained as attributed G1 segments; no G18/G19 switch for the vertical
tool. At most64segments/512KiB per chunk, one pending source, cancellation/rollback,
source hashes, separate posted IDs and continuous source-interval error bounds.
Defaults remain0.01mm total post deviation/0.0025mm chord budget and unit-selected
formatting. Stock allowance is an explicit separate planner input.

`curve_policy='exact_ijk'` opts into native G17 circles and <=one-turn helices,
with explicit endpoints and incremental I/J. Helices include `TURN=0` (zero
additional revolutions). Radii must match **exactly in the declared binary64
model** after output quantization. The manual says accepted mismatched circles
have their center adjusted internally, but does not specify an algorithm here;
the example must not pretend the uncorrected circle is the actual result. Such
cases reject and should use bounded linear policy. This zero-mismatch structural
check is not a request for submicron machine accuracy. `CR`, `AR`, `AC/IC`, `CT`,
`CIP`, extra TURN counts, transforms, blending/compression/rounding, cycles other
than `CYCLE84` (below) and ISO syntax remain unsupported. Native center-repair/blend models and external/machine conformance are outside
this reference profile.

`post_events_808d(header, iterator, machine)` supports a live source. To use the
core API directly, pass `post=Siemens808DPost`, `decoder=Siemens808DDecoder`, and
`lowering=LoweringOptions()` for the linear policy. Machine/post/decoder choices
and revisions are included in run definitions; this is not attestation of arbitrary
Python dependencies. Custom Python posts remain trusted executable code.

## Rigid tapping: CYCLE84

A `Tapping` operation posts one `CYCLE84` call per hole, in either hand. It needs
two declared premises; without them a tapping motion is refused:

```python
machine = define_machine(
    ...,
    position_controlled_spindle=True,   # the spindle can run position controlled
    tap_spindle_position_deg=0,         # POSS: the oriented stop before tapping
)
```

For an M6 x 1 hole from Z11 to Z4 at 600 rpm the blocks are

```
M3 S600                                       (M4 S600 for a left-hand tap)
G1 X8 Y8 Z11 F...                             (the hole position, at the R level)
CYCLE84(11,11,,4,,,3,,1,0,600,600,3,0,0,0)    (left hand: ...,4,,-1,0,...)
; CYCLE84 retracted
```

The comment carries the return half of the tapping pair, which the call itself
performs. What each word is, and where the manual fixes it (07/2018 milling
manual, printed pages):

| Behaviour | Page | What the manual states, and what the post does |
| --- | --- | --- |
| Call form | 154 | A cycle call stands in its own block; a parameter is omitted with a comma placeholder and the list may end early. The post writes the list to `VARI` and leaves `DAM`, `VRT` (deep-hole tapping only) off |
| Position and modes | 153, 156 | The hole position is approached before the call; the plane and a tool length compensation must be active; the G functions active before the cycle stay active after it. The decoder requires G17 and an active tool edge |
| `RTP`, `RFP`, `SDIS` | 157-158, 167 | The cycle moves with G0 to `RFP + SDIS` and at the end with G0 to `RTP`. Both planes are written as the current Z with no `SDIS` (p.158: equal planes forbid only the relative depth `DPR`), so neither is a move; the decoder refuses anything else |
| `DP`, `DPR`, `DTB` | 157, 166, 168 | `DP` is the absolute final depth; `DPR` (relative) and the dwell `DTB` are omitted ("when tapping blind holes, it is recommended that you omit the dwell time") |
| Thread lead and hand | 166, 168 | `PIT` is the lead as a value, 0.001-2000 mm; "positive value: right (same as M3), negative value: left (same as M4)". `MPIT` (thread size) is omitted. The post writes feed / rpm, negative under M4 |
| Spindle orientation | 102, 167, 168 | G331 needs the spindle in position control by `SPOS` first (p.102); CYCLE84 does this itself: "the spindle is stopped with orientation in the cycle by using the command SPOS", at `POSS`. The post writes the declared `tap_spindle_position_deg` and no `SPOS` block |
| Speeds | 166, 168 | `SST` is the tapping speed (the G331 block), `SST1` the retraction speed (0 means `SST`). Both are written as the running spindle speed |
| Tool axis | 168-169 | `AXN=3` is the third axis of the current plane: Z in G17. The two internal parameters are 0 |
| One pass | 169 | `VARI=0` is tapping in one pass; 1 and 2 are deep-hole tapping, not written |
| Retraction | 167, 169 | "Retraction to the reference plane brought forward by the safety clearance, speed SST1 and direction reversal"; the reversal is automatic |
| Spindle afterwards | 166, 167 | "Spindle mode is reinitiated by reprogramming the spindle speed active before the cycle was called and the direction of rotation programmed under SDAC" (3, 4 or 5). The post writes 3 after M3 and 4 after M4, so the spindle turns afterwards as before |
| Spindle requirement | 167 | "CYCLE84 can be used if the spindle ... is technically able to be operated in the position-controlled spindle operation": `position_controlled_spindle=True` |

`Siemens808DDecoder` expands the call from its written parameters into the tap
to `DP` and the return, and refuses it unless the sign of `PIT` and `SDAC` are
both the running spindle direction, `SST`/`SST1` repeat the running speed, `POSS`
is the declared angle and no block follows before the return. The manual states
the lead in mm only, so tapping is refused with inch output. Not stated by the
manual and not relied on: the units of `PIT` under G700, and how a spindle is
returned from position control after a bare G331/G332 pair (which is why the
post writes the cycle, whose spindle state afterwards is documented). Machine
data for the spindle "must be configured" (p.154); whether a given machine's
spindle taps rigidly is the declared premise.

## References and local checks

The primary reference is the [07/2018 native milling manual](https://cache.industry.siemens.com/dl/files/981/109759981/att_959788/v1/808D_ADV_milling_op_man_0718_en-US.pdf),
6FC5398-4DP10-0BA7, software 4.90. Printed/PDF pages coincide: 79–81 planes and
units; 83–88 frames; 90–99 interpolation and helix; 105–106 stop windows;
114–117 tool/edge activation and length axes; 123–124/338–339 builder M functions;
133 modal call cancellation; 337 language switching; 102-103 G331/G332 and 108-109
SPOS; 152-158 cycle call conditions and the shared drilling parameters; 166-169
CYCLE84. The [02/2016 ISO manual](https://cache.industry.siemens.com/dl/files/339/109736339/att_880241/v1/SINUMERIK_808D_ADVANCED_ISO_022016_eng.pdf)
page 8 additionally documents standalone language switching and preserved state.
That older ISO edition does not establish native 4.90 semantics.

`python/tests/test_siemens808d_post.py` and `python/tests/siemens808d_oracle.py`
check literal output, units/frames/tool edges, bounded lowering, cancellation,
hostile modal changes, actual stock, and CYCLE84 tapping in both hands (golden
blocks, deliberately wrong posts, undeclared premises). `fixtures/posts/EXACT490.MPF` is a
literal native quarter/full-circle/helix fixture. The oracle imports motion
contracts without the post/dialect/decoder implementation. Machine data, OEM
routes, physical stop windows, later firmware and external qualification remain
separate requirements. See [postprocessor architecture](posts.md) for workflow
references and [ISO profile](siemens808d-iso-post.md) for the separate language.
