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

Key settings:

- **`tools`**: the machine tool table. `position_after_mm` is where the tool
  tip is after a tool change (`tool_change_position_mode="table"`); the next
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
  other rapid. `job.post` and `job.post_nc` refuse a setup whose tool-change
  pose disagrees with the machine's, before planning.
- **`diameter_register`** (per `ToolSetting`): the D register holding the
  tool's wear, written with `G41`/`G42` for `Contour(compensation="wear")`.
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
- **Dynamics**: `max_normal_accel_mm_s2` and `max_axis_feed_mm_min` report curve
  feeds the machine cannot reach. `motion_policy=MotionPolicy(
  tangential_accel_mm_s2, rapid_feed_mm_min, junction_mode="exact_stop" or
  "tangent")` enables `machine.schedule(program)` (time estimate, feeds that
  respect the limits) and `schedule_post_feeds=True` (posts the scheduled feeds).
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
| `dwell_word`, `dwell_resolution_s`, `dwell_max_s` | Dwell word, step and range |
| `cycle_dwell_unit`, `peck_reentry_mm`, `chip_break_lift_mm` | Canned-cycle dwell unit and the controller's peck return amounts |
| `tap_pitch_word` | Lead word of a rigid tapping cycle (Centroid Q), or feed |
| `tap_cycle`, `tap_cycle_left` | Canned rigid tapping, right hand (`G84`) and left hand (`G74`), declared with `with_commands`. The spindle turns in the cycle's direction beforehand (M3 for G84, M4 for G74, FANUC 0i-MB §13.1.2); the decoder refuses a cycle under the other direction. Without `tap_cycle_left` (and without `tap_synchronized` G33.1) a left-hand tap is refused |
| `compensation_word` | D register word of G41/G42 |
| `compensation_with_move` | G41/G42/G40 written in the block of the move that turns compensation on or off (FANUC, Haas), not alone |
| `oriented_edge_deg` | Where the boring bar's edge points after the oriented spindle stop |
| `thread_lead_word` | Lathe threading (`thread_synchronized`, e.g. G33/G32): the word carrying the lead (K for LinuxCNC G33, F for G32) |
| `tool_change="lathe_t"` | Lathe tool words `T<nn><oo>` (tool and offset, T0101) |
| `x_diameter` | Lathes: X words are diameters (arc I stays a radius). LinuxCNC lathes: declare `lathe_radius_mode="G8"` or `lathe_diameter_mode="G7"` to write the mode in the header |
| `css_on`, `css_limit`, `css_limit_word` | Constant surface speed (`SurfaceSpeed` events): `G96 S<m/min>` (ft/min in inch output) with M3/M4, its rpm limit either in the same block (`css_limit_word="D"`: LinuxCNC `G96 D<max> S<speed>`) or as `G50 S<max>` in the block before (`css_limit="G50"`: FANUC, Centroid); exactly one of the two. A `Spindle` after it writes the rpm mode (G97) first |
| `compensation_register` | False for lathes: G41/G42 written without a register word (the T offset holds the nose radius and tip direction) |
| `feed_per_rev` | Feed per revolution (`FeedPerRev` events): `G95 F<mm/rev>` (Centroid/FANUC lathes `G99`, with `feed_per_minute="G98"` and `cycle_return_initial=None`), back to `G94` at 0. The written F must keep the feed within `feed_tolerance_mm_min` at `max_spindle_rpm`, otherwise the post refuses (e.g. a metric feed in inch output) |
| `beam_s_max`, `beam_effect` | Plasma/laser: S at full power and the spindle code that fires the beam (`spindle_cw` M3, `spindle_ccw` M4); M5 turns it off. None refuses beam tools |
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
`on_height_control`, `on_surface_speed`, `on_feed_per_rev`, `on_finish()`. The default `on_dwell` writes the dialect's `dwell` code (G4)
with the time in seconds in `dwell_word` (P), rounded up to
`dwell_resolution_s` and limited to `dwell_max_s`.

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
| `grbl_post.py` | GRBL 1.1h | Lines only (arcs as checked segments), one preloaded tool, checked with GRBL's own binary32 arithmetic; vertical moves are written with Z only. `define_machine(..., beam_s_max=1000)` (= `$30`) writes a laser as `M4 S` (laser mode `$32=1`). `grbl_linear.py` is a complete example |
| `carvera_post.py` | Makera Carvera (metric milling) | Python and Rust named pair; [setup and retract premises](carvera-post.md) |
| `haas_ngc_post.py` | Haas NGC (2020 manual) | [setup notes](haas-ngc-post.md) |
| `fanuc_0imb_post.py` | FANUC 0i-MB | [setup notes](fanuc-0imb-post.md); `inspection_plus_fast_feed_mm_min=` posts probing as Renishaw Inspection Plus macros |
| `lnc_mill_post.py` | LNC milling, LNC-M8000 (A-LNC programming manual V01.02) | [setup notes](lnc-mill-post.md); native G2/G3 in G17/G18/G19 checked against parameter 50048; canned cycles and M29 rigid tapping with the declared parameters |
| `siemens808d_post.py` | SINUMERIK 808D native | [setup notes](siemens808d-post.md) |
| `siemens808d_iso_post.py` | SINUMERIK 808D ISO mode | [setup notes](siemens808d-iso-post.md) |
| `centroid_post.py` | Centroid CNC12 mill (Acorn, Oak, ...) | [setup notes](centroid-post.md) |
| `qtplasmac_post.py` | LinuxCNC 2.9 QtPlasmaC (plasma) | [setup notes](qtplasmac-post.md) |
| `centroid_plasma_post.py` | Centroid CNC12 plasma (Acorn THC) | [setup notes](centroid-plasma-post.md) |
| `wincnc_plasma_post.py` | WinCNC 3.x plasma (WinCNC THC) | [setup notes](wincnc-plasma-post.md) |
| `centroid_lathe_post.py` | Centroid CNC12 lathe (diameter X, T0101, G96/G50, G99, G32) | [setup notes](centroid-lathe-post.md) |
| `fanuc_lathe_post.py` | FANUC lathe, G-code system A (%, O number, decimal points, T0101, G50/G96, G99, G32) | [setup notes](fanuc-lathe-post.md) |

Add `examples` to `PYTHONPATH`, or copy the file next to your code. The
controller examples write curves as checked line segments by default.

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

The post lifecycle follows FreeCAD CAM's
[postprocessor](https://github.com/FreeCAD/FreeCAD/blob/920d14763241267340e755f3d3b6ece8ef60fa43/src/Mod/CAM/Path/Post/Processor.py)
concepts.

The named `"name": "carvera"` pair targets metric Makera Carvera milling.
Its explicit tool-length, coordinate and planned-retract premises are described
in [Carvera setup notes](carvera-post.md). Python requires `CarveraPost` with
`CarveraDecoder`; Rust dispatches the same named pair from the machine JSON.
