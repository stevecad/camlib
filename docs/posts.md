# Postprocessors

A postprocessor turns camlib's typed motion (lines, arcs, helices, tool
changes, spindle, coolant) into G-code for one controller. In camlib it is
ordinary Python: a `Machine` describes the controller and tool table, and a
`GCodePost` subclass decides how each event is written.

Every block a post writes is **decoded again** by an independent reader and
compared with the planned motion before it is accepted. A post that writes the
wrong coordinate, feed, plane or modal state makes the run fail. It cannot
produce a quietly different program.

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
  tip is after a tool change; the next operation travels at the higher of
  that height and its clearance before it descends (no rapid moves sideways
  while descending), so the declared pose must match the real machine. With
  `tool_change_position_mode="preserve_tip"` the tip is instead assumed to
  return to where it was. With `tool_change_position_mode="z_home"` and
  `tool_change_home_z_mm` (the spindle gauge line's machine Z after M6), M6
  sends Z home and leaves X and Y where they were (Centroid CNC12). Plan the
  job with the matching `Setup(tool_change=ToolChangePosition(gauge_z_mm=...))`
  (the same height in work coordinates: home Z minus the work offset Z), so
  each new tool's first move comes down from there and is checked like any
  other rapid.
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

`LoweringOptions(native_arcs=..., chord_tolerance_mm=0.0025,
max_segments_per_source=..., max_total_segments=...)` bounds the curve-to-line
conversion.

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
the controller posts above without a documented tolerance use. The certified
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
`chip_break_lift_mm`; LinuxCNC 0.254 mm). Boring cycles (`Reaming(canned_cycle=True)`)
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
| `GCodeDialect.linuxcnc()` | LinuxCNC 2.9 | `GCodeDialect.linuxcnc(beam_s_max=...)` writes a beam as `M3 S`/`M5` |
| `grbl_post.py` | GRBL 1.1h | Lines only (arcs as checked segments), one preloaded tool, checked with GRBL's own binary32 arithmetic; vertical moves are written with Z only. `define_machine(..., beam_s_max=1000)` (= `$30`) writes a laser as `M4 S` (laser mode `$32=1`). `grbl_linear.py` is a complete example |
| `haas_ngc_post.py` | Haas NGC (2020 manual) | [setup notes](haas-ngc-post.md) |
| `fanuc_0imb_post.py` | FANUC 0i-MB | [setup notes](fanuc-0imb-post.md) |
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

## What a passing post does and does not prove

Passing means the emitted words, decoded independently, describe the planned
motion within `tolerance_mm`, with explicit units, plane, offsets and modal
state. It does **not** prove the controller's real settings (work offsets, tool
length table, option parameters, tool-change routine) match the machine
definition. Those are inputs you supply.

The post lifecycle follows FreeCAD CAM's
[postprocessor](https://github.com/FreeCAD/FreeCAD/blob/920d14763241267340e755f3d3b6ece8ef60fa43/src/Mod/CAM/Path/Post/Processor.py)
concepts.
