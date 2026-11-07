# Operations

All operations live in `camlib.operations`. Each is an immutable dataclass:
build it with keyword arguments, put it in a `Setup`, and the Rust engine
plans it. Units are mm, mm/min, rpm and degrees; Z values are absolute.

## Common parameters

Every cutting operation has these. Cutting data (feeds, spindle speed and
coolant) and the tool's T and H numbers depend on the tool, material and
machine, so they have no defaults: pass them as keyword arguments.

| Parameter | Default | Meaning |
| --- | --- | --- |
| `id` | (required) | Unique name in the setup |
| `tool` | (required) | A `ToolAssembly` from the setup's tool list |
| `clearance_z_mm` | (required) | Safe height above everything, clamps included |
| `feed_mm_min` | (required) | Cutting feed |
| `plunge_mm_min` | (required) | Vertical entry feed (not on every operation) |
| `tolerance_mm` | 0.00001 | Numerical geometry tolerance (not a machining accuracy) |
| `spindle_rpm` | (required) | Spindle speed |
| `coolant` | (required) | `"flood"`, `"mist"` or `"off"` |
| `tool_number` | (required) | T number written to the program |
| `length_register` | (required) | H (tool length offset) register |
| `work_offset` | 1 | 1..9 → G54..G59.x |
| `retract_z_mm` | `clearance_z_mm` | Rapid travel height between passes (Pocket, RestPocket, AdaptivePocket, Roughing, Contour, Chamfer, Facing, TrochoidalSlot, Bore, Parallel, Spiral, Radial, Project, Waterline, Trace) |
| `feed_z_mm` | `retract_z_mm` | Rapid descent stops here and plunge feed begins; leaving a pass, feed changes to rapid here |
| `boundary_tolerance_mm` | 0.01 | Allowed construction error of offset boundaries (Contour, Chamfer, Bore, TrochoidalSlot, adaptive clearing) |
| `reverse` | off | Run the passes the other way: Facing and Parallel start each pass from the opposite end, Waterline cuts its contours the other way, Contour/Chamfer (`None` = from `cutting_sense`) reverse the loop direction |

Heights follow the usual CAM convention: an operation starts and ends at
`clearance_z_mm`, travels between passes at `retract_z_mm`, and descends at
rapid only to `feed_z_mm`. Required order: stock top < `feed_z_mm` ≤
`retract_z_mm` ≤ `clearance_z_mm`; since the stock only loses material, rapids
at these heights cannot touch stock. Operations that do not support them yet
reject them with an error instead of ignoring them.

Straight-up retracts out of a cut are rapids by default, as commercial CAM
does. The engine converts a retract only when the stock proves it clear: it
starts exactly where the last cut left the tool, so that pose is already empty,
and the stock has no overhangs, so everything above it is empty too. Any other
retract stays at feed; after an undercut (a necked tool under a ledge) the
proof no longer applies and retracts stay fed. The stock-contact check, when
configured, verifies each rapid again. `Setup(rapid_retracts=False)` keeps
every retract at feed.

## Machining limits: avoid areas and containment

Any operation can be limited to where it may cut:

```python
from camlib import AvoidArea, Limits, OperationNode, Setup

clamp = AvoidArea(clamp_footprint, top_z_mm=18)        # a PlanarRegion and its top
setup = Setup("top", stock, tools, (OperationNode(pocket), OperationNode(finish,
              limits=Limits(contain=window))), initial_position_mm, avoid=(clamp,))
```

- **Avoid areas** (`Setup(avoid=...)` for every operation, or per operation)
  keep the whole cutter out of the area's footprint while its tip is below the
  area's top: clamps, fixtures, features to leave alone.
- **Containment** (`contain=`) keeps the whole cutter inside a region while it
  cuts.

How each operation honours them:

| Operation | Behaviour |
| --- | --- |
| Pocket, adaptive, roughing | The region to clear becomes (region ∩ containment) − avoid areas, so the planner routes around them; a 0.02 mm clearance is kept from their edges |
| Facing | Planned as a raster pocket over the stock (grown so the tool still runs past its edges) restricted by the limits, entering from the stock edge; needs `stepover_mm` at most the tool radius |
| Contour, chamfer, trace | Blocked stretches of a loop are lifted over (to the highest avoid top, at least the retract height); blocked starts, leads and entries are refused, as is lifting with wear compensation |
| Parallel and its patterns, waterline, flat areas | Pass stretches whose cutter is not proven inside the limits are left out; the rest is cut as separate runs |
| Drilling, bore, reaming, tapping, thread milling, slots | A hole or slot whose footprint enters an avoid area or leaves the containment is refused, naming it |

Every rapid and link is checked against the avoid areas; one that passes
through an area below its top is refused (raise the clearance or retract
height). Each check is a proof: samples must clear by at least half their
spacing, so nothing between samples can touch.

## Facing: `Facing`

Required: `floor_z_mm`, `stepdown_mm`, `stepover_mm`, `clearance_z_mm`.
Faces the whole rectangular stock top down to `floor_z_mm`, entering from
outside the stock. Options: `axis` (`"x"`/`"y"`), `mode` (`"zigzag"` or
`"one_way"`), `entry_air_gap_mm` (1), `edge_overlap_mm` (default 10% of the
tool diameter), `reverse`. Flat end mills.

## Pocket: `Pocket`, `RestPocket`

Required: `region`, `top_z_mm`, `floor_z_mm`, `stepdown_mm`, `stepover_mm`
(at most the tool radius), `clearance_z_mm`.

`region` is a `PlanarRegion`: outer boundaries with holes. **Holes are islands
that are kept.** Boundaries are polygons (`Boundary.polygon`) or line/arc loops
(`Boundary.circle`, or bulge vertices). Flat, center-cutting end mills.

- Allowances: `radial_allowance_mm`, `bottom_allowance_mm` leave stock on walls/floor.
- Direction: wall passes are climb milled with a clockwise spindle (pocket walls
  counter-clockwise, islands clockwise); `boundary_clockwise=True` reverses
  them. The interior raster runs both ways.
- Entry: `entry=None` plunges straight down (center-cutting tool required);
  `HelicalEntry(angle_deg=3, radius_mm=None, ...)` ramps in on a helix, written
  one full turn per G2/G3 block; `StockEdgeEntry()` enters from outside the
  stock where possible, helical otherwise.
- Heights: `retract_z_mm` (travel between passes) and `feed_z_mm` (where rapid
  approach changes to plunge feed, and where leaving a cut changes from feed to
  rapid). Both default to `clearance_z_mm`, the most conservative behaviour.
  Required order: stock top < `feed_z_mm` ≤ `retract_z_mm` ≤ `clearance_z_mm`.
- `RestPocket` cuts only material left by earlier operations (it reads the
  simulated stock), typically with a smaller tool. `rest_tolerance_mm` (0.01)
  is how much leftover is ignored.
- Links between separate regions (longer than ordinary stepovers) stay at
  cutting depth only through stock proven already cleared, and only when that
  is faster than leaving at `feed_z_mm` and plunging again (the default) or up
  to `max_stay_down_mm` (0: always leave). A `feed_z_mm` just above the stock
  makes leaving cheap: in a corner rest pass after a 6 mm pocket it cut
  feed-rate links from 2178 mm to 58 mm.

Every planned tool-centre path is checked against the original region before
any motion is emitted: if geometry construction ever went wrong, the operation
fails instead of cutting into an island or wall.

## Adaptive pocket: `AdaptivePocket`

Same region and depths as `Pocket`, cleared with bounded tool engagement
(`max_engagement_deg`, default 90): no full-width slotting, for hard
materials and high-speed toolpaths. `retract_z_mm`/`feed_z_mm` work as for
`Pocket`; it relinks through its own proven-clear re-entries, so
`max_stay_down_mm` is refused.

- `cutting_sense`: `"climb"` (default) or `"conventional"` with a clockwise
  spindle (`clockwise` is the older orbit-direction switch).
- Orbits: `orbit_radius_mm` (default half the tool radius). Set
  `min_orbit_radius_mm` below it with `local_orbit_tolerance_mm` (for example
  0.05) to size each orbit from the clearance around it instead.
- Narrow features and corner rest it cannot reach are reported.
- `path_strategy="stock_front"` selects a newer strategy still under
  qualification; it stops with an error where its coverage is unresolved.

## 3D roughing: `Roughing`, `RestRoughing`

Required: `target` (a `SurfaceMesh` of the part, or use
`Roughing.from_solid(...)` with CadQuery), `top_z_mm`, `floor_z_mm`,
`stepdown_mm`, `clearance_z_mm`, `clearing`.

Clears stock in Z layers while staying above the part surface plus
`radial_allowance_mm`/`bottom_allowance_mm`. Each layer protects the part's
projection above it (its "shadow"); holes in that projection smaller than a
disk of half the cutter's radius are protected too, since the cutter cannot
enter them (the layer report's `shadow.filled_holes`). A target mesh whose faces
were tessellated separately leaves such slivers between neighbouring faces; a
gap that still makes the layer's topology ambiguous is refused with its
location. `clearing` is one of:

- `RasterClearing(stepover_mm, routing=None)`: parallel passes;
  `routing="economical"` orders and joins the rows to shorten links,
- `OffsetClearing(stepover_mm, coverage_margin_mm=0.01)`: contour-parallel
  passes, with that much overlap reserved for the coverage proof,
- `AdaptiveClearing(max_radial_engagement_mm)`: engagement-limited, with the
  same orbit, `cutting_sense` and `path_strategy` options as `AdaptivePocket`.

Default entry is `StockEdgeEntry()`. `retract_z_mm`/`feed_z_mm` work for raster
and offset clearing. `RestRoughing` machines only what earlier operations left
(`rest_tolerance_mm`, 0.01, is ignored), never repeating strokes proven already
cleared. Further options:

- `omit_empty_cuts=True` leaves out passes whose whole sweep is proven to cut
  only air (reported).
- `require_part_at_first_depth=True` refuses a first layer above the top of the
  part (face the stock first).
- `clearing_tolerance_mm` sets the allowed construction error of each layer's
  clearing region; the operation fails if it cannot be kept.

## Contour: `Contour`

Required: `region` (the part footprint to **keep**), `floor_z_mm`,
`stepdown_mm`, `clearance_z_mm`. Cuts along the outside (`boundaries="outer"`),
inside of holes (`"holes"`) or both (`"all"`). Optional tangent leads
(`lead_in_mm`, `lead_out_mm`) and allowances. It cuts a band one tool wide
next to the wall, not all the stock outside it. `cutting_sense` is `"climb"`
(default) or `"conventional"` for a clockwise spindle; `reverse` overrides it
with an explicit loop direction.

`compensation="wear"` posts the same tool-center path with cutter compensation
so the operator can trim size at the machine: `G41` (climb) or `G42`
(conventional) with the tool's D register, turned on over the lead-in and off
(`G40`) before the lead-out. Both leads are required and must be longer than
the largest wear. The D register must hold only the wear (the difference from
the programmed diameter), never the full radius. Declare it per tool with
`ToolSetting(..., diameter_register=...)`; `GCodePost` and the controller
examples write it (GRBL has no cutter compensation and refuses it). While it
is on, every move travels in XY (ramps may change Z); straight Z moves are
refused.

`tabs=Tabs(width_mm, height_mm, count=4)` (or `spacing_mm=...`) leaves holding
tabs on a through-cut part: on passes below the tab top (`floor_z_mm +
height_mm`) the tool rises over each tab, which stays `width_mm` wide on the
part edge. `shape="triangle"` (default) ramps at 45 degrees; `"rectangle"` goes
straight up and over (not allowed with wear compensation). Tabs are spread
evenly along each loop, away from its start; the operation refuses tabs that
do not fit.

## Drilling: `Drilling`

Required: `centers_mm` (list of (x, y)), `depth_z_mm`, `clearance_z_mm`.
`depth_reference="tip"` (default) means `depth_z_mm` is the tip;
`"full_diameter"` means it is where the drill reaches full diameter. `peck_mm`
enables pecking (`peck_mode="full_retract"` or `"chip_break"`). Drills or
center-cutting flat mills. `retract_feed_mm_min` is the feed out of the hole
where the move is fed. A tip below the stock bottom is refused unless
`allow_breakthrough_mm` allows that much (through holes); the report gives the
breakthrough. Without `canned_cycle` the moves are written as explicit lines.

`dwell_s` pauses at full depth in each hole, as a G82 cycle does: use it for
spot drilling and for counterbores cut with a flat end mill. The dwell is a
separate event after the last cut into the hole, posted as `G4 P<seconds>`
(rounded up to the controller's dwell resolution, never shorter). Posts whose
dialect declares no dwell refuse the program instead of dropping the pause.

`canned_cycle=True` moves exactly as the controller's canned cycle would, so a
post can write one block per hole:

| Settings | Cycle | Motion |
| --- | --- | --- |
| no `peck_mm` | `G81` | feed to depth, rapid out |
| `dwell_s` | `G82` | feed to depth, dwell, rapid out |
| `peck_mm`, `peck_mode="full_retract"` | `G83` | pecks of `peck_mm` from the R plane (`retract_z_mm`), rapid to R after each, rapid back down to `reentry_clearance_mm` above the last peck |
| `peck_mm`, `peck_mode="chip_break"` | `G73` | pecks from R, rapid up `chip_break_lift_mm` after each |

All cycles return to the clearance height (`G98`). The return amounts are
controller settings (LinuxCNC: 0.254 mm, i.e. 0.010 in), so set
`reentry_clearance_mm`/`chip_break_lift_mm` to the machine's value; the post
refuses a mismatch. Posts without the cycle write the same moves explicitly.
The stock checks prove each rapid inside a hole clear from the plunge that
drilled it. Pecking with a dwell has no standard cycle and is refused.

## Bore milling: `Bore`

Required: `centers_mm`, `diameter_mm`, `floor_z_mm`, `stepdown_mm`,
`clearance_z_mm`. Mills full round holes with a helical path, widening in
`radial_stepover_mm` steps if the hole is much larger than the tool.
`entry_feed_mm_min` (150) is the feed of the helical entry.

## Reaming and boring: `Reaming`

Required: `centers_mm`, `pilot_diameter_mm`, `depth_z_mm`, `clearance_z_mm`.
The usual G85 cycle: feed from the R plane (`feed_z_mm`) down to
`depth_z_mm`, then `retract_feed_mm_min` back out. The reamer or boring bar is
modeled as a flat cutter of its diameter (not center-cutting); the stock must
show each pilot hole open below the depth first, or the operation stops with an
error.

| Settings | Cycle | At depth |
| --- | --- | --- |
| default | `G85` | feed back out |
| `dwell_s` | `G89` | dwell, feed back out |
| `spindle_stop=True` | `G86` | stop the spindle, rapid out, restart it |
| `fine_bore=FineBore(shift_mm, edge_angle_deg)` | `G76` | (dwell), oriented spindle stop, shift away from the wall, rapid out, shift back, restart |

`canned_cycle=True` runs each hole from the R plane as the controller's
cycle (the R plane is then also the height between holes, so `feed_z_mm` must
equal `retract_z_mm`; it feeds out at `feed_mm_min`), written as one block per
hole where the machine declares it and as explicit moves elsewhere.

### Fine boring

`fine_bore` stops the spindle at depth with the bar's cutting edge pointing
along `edge_angle_deg` (work XY, degrees from +X), moves the bar `shift_mm` the
opposite way so the stopped edge clears the finished wall, rapids out to R,
shifts back over the hole and restarts the spindle. Use a
`camlib.tools.BoringBar(id, diameter=, bar_diameter=, reach=, overall_length=)`:
it is modeled as an insert of the bore diameter on a neck of the bar diameter.
`shift_mm` must leave the bar inside the bore: `shift_mm + bar_diameter/2 <=
diameter/2` (less a small guard). The stock-contact check proves every shift
again: the stopped bar is assumed to lie within its bar diameter except the
cutting edge.

The edge direction is a setup premise. The post compares it with the
machine's `oriented_edge_deg` (where the edge points after the oriented stop)
and refuses a mismatch. Machines without a declared oriented stop cannot post
fine boring. The FANUC example writes `G76` (`fine_bore_edge_deg=`); the
Centroid example writes `M19` and explicit moves (`oriented_edge_deg=`).

## Tapping: `Tapping`

Required: `centers_mm`, `pitch_mm`, `depth_z_mm`, `clearance_z_mm`. Rigid
tapping of drilled holes with a `camlib.tools.Tap(id,
drill_diameter=, thread_length=, overall_length=, shank_diameter=)`. Each hole
is tapped from the R plane (`feed_z_mm`, default the retract height) to
`depth_z_mm` and back at the synchronized feed `pitch_mm` × `spindle_rpm`.

- `hand="right"` (default; a thread is right-hand unless designated LH, ISO
  965-1 / ASME B1.1) turns the spindle clockwise (M3) into the hole;
  `hand="left"` turns it counterclockwise (M4). The controller reverses the
  spindle for the return.
- The post writes one controller cycle per hole and its decoder reads it back
  for checking: `G33.1 Z… K<pitch>` with the default dialect (LinuxCNC),
  or `G84 X… Y… Z… R… F…` followed by `G80` on machines that declare
  `tap_cycle` (the Haas example with `rigid_tapping=True`). Machines that
  declare neither refuse tapping.
- Left-hand taps: `G33.1` under `M4` (LinuxCNC reverses whatever direction
  is running), or `G74 X… Y… Z… R… F…` and `G80` on machines that declare
  `tap_cycle_left` (the FANUC 0i-MB example with `rigid_tapping=True`).
  Machines with neither refuse a left-hand tap. The decoder reads the hand
  back: `G84` needs the spindle clockwise, `G74` counterclockwise, and the
  decoded spindle direction must match the plan.
- Each hole must already be drilled below the tap depth plus
  `bottom_clearance_mm` (default one pitch: synchronized taps such as G33.1
  overrun while the spindle reverses) in the stock.
- The tap is modeled by its drill diameter, so the stock keeps the thread
  flanks (conservative). Feed limits that would change the synchronized feed
  are refused.

## Thread milling: `ThreadMilling`

Required: `centers_mm`, `internal`, `diameter_mm`, `start_diameter_mm`,
`pitch_mm`, `top_z_mm`, `bottom_z_mm`, `clearance_z_mm`. Helical thread
milling with a single-profile thread mill (`camlib.tools.ThreadMill(id,
diameter=, tooth_height=, neck_diameter=, neck_length=, overall_length=)`):

- `diameter_mm` is the final diameter of the tooth tip: the major diameter of
  an internal thread, the minor diameter of an external one (for ISO metric
  external threads, major − 1.2269 × pitch). `start_diameter_mm` is the
  drilled hole or the boss.
- One helical turn per pitch between `bottom_z_mm` and `top_z_mm` (tool tip),
  entered and left on tangent semicircles: from the hole center, or from
  outside the boss.
- `right_hand` (default) and `climb` (default) set the direction, as usual
  for a clockwise spindle: climb milling a right-hand thread runs
  counter-clockwise and bottom-up in a hole, clockwise and top-down around a
  boss.
- `radial_passes` splits the thread depth into equal steps; `spring_passes`
  repeat the final pass.

Before the tool goes down, the current stock must show the descent column
open down to the start depth (the hole drilled, or the space beside the
boss cleared); otherwise the operation stops with an error. The neck must
clear the thread crests. The stock model shows the swept tooth: below
material that stays above it (the groove), the top height is unchanged.
`feed_mm_min` is the feed at the tool center; the cutting edge moves faster
inside a hole (by diameter / (diameter − tool diameter)).

## Trochoidal slot: `TrochoidalSlot`

Required: `start_mm` and `end_mm` (straight), or `path_mm` (along a
centreline; pass `None` for both points), and `width_mm`, `top_z_mm`,
`floor_z_mm`, `stepdown_mm`, `clearance_z_mm`. Slots whose width is more than
the tool diameter and at most twice it, cut with looping low-engagement passes:
full orbits at stations along the centreline, joined by links on the
centreline's offset. `entry_feed_mm_min` (150) is the entry feed.

```python
# An S-shaped slot: two half turns of radius 7, tangent where they meet.
TrochoidalSlot("s", tool, None, None, 6, 10, 8, 1, 12,
               path_mm=((6, 12, 1), (20, 12, -1), (34, 12, 0)), ...)
```

- `path_mm` vertices are `(x, y, bulge_to_next)`, bulge = tan(sweep / 4) in
  [-1, 1] (at most a half turn per arc, as in `Boundary`); the last bulge is 0.
- The engagement limit holds on curves too: after each orbit, everything left
  lies outside its disk, and the next link and orbit stay within the chord to
  the next station plus the orbit radius, so `max_engagement_deg` bounds the
  exposed edge angle whatever the curvature.
- The step follows the outside of each bend, where the gap between two orbits'
  disks is widest, so the swept width stays within `boundary_tolerance_mm` of
  the slot width.
- Every vertex gets an orbit. At a corner, a straight link inside that orbit's
  disk joins the two offsets (`corner_links` in the report).
- Links run on the left of the centreline for `clockwise=True`, on the right
  otherwise. An arc bending toward that side needs a radius above the orbit
  radius (width/2 − tool radius), otherwise the slot is refused, naming the
  radius. Reverse `clockwise`, or use a narrower slot or a larger tool.

## V-carve: `VCarve`

Required: `region` (letters or a logo as a `PlanarRegion`), `top_z_mm`,
`clearance_z_mm`, and a V bit (`ChamferMill`). For each point of the outline
the bit goes to the largest disk inside the region that touches it, as deep as
its cone must be to be that wide at `top_z_mm`, so the walls are sharp: convex
corners run out to the surface, concave corners are swept around. Every
interior point lies in one of those disks, so the region is cleared; between
outline samples (`sample_mm`, 0.05) the carve is checked against the outline
and refined where it would cross it. Parts narrower than a flat-tipped bit's
tip are left (reported). `max_depth_mm` limits the depth; a region too wide
for it is refused, since its middle needs a flat clearing tool.

## Trace / engrave: `Trace`

Required: `paths`, `top_z_mm`, `depth_z_mm`, `stepdown_mm`, `clearance_z_mm`.
The tool center follows each path with no radius offset (text and logo
engraving with a chamfer tool, edge breaks, slots along a path). Depth passes
run from `top_z_mm` (at most the stock top) to the tool tip at `depth_z_mm`.

```python
from camlib import Trace, TracePath, TraceLine, TraceArc

path = TracePath((4, 4), (TraceLine((14, 4)), TraceArc((14, 7), 180)))
op = Trace("engrave", tool, (path,), top_z_mm=6, depth_z_mm=5.5,
           stepdown_mm=0.5, clearance_z_mm=8, feed_mm_min=400, plunge_mm_min=100,
           spindle_rpm=18000, coolant="off", tool_number=3, length_register=3)
```

A `TraceArc(center, sweep_deg)` is given by its center and sweep in degrees
(positive is counter-clockwise), so its end point and radius are always consistent. A path
is closed when its last segment ends on its start. Arcs stay native G2/G3
where the controller and precision allow. Nothing protects the part except the
configured checks, since the path is the programmer's. Supports
`retract_z_mm`/`feed_z_mm`.

## Chamfer: `Chamfer`

Required: `region`, `top_z_mm`, `width_mm`, `clearance_z_mm`. Chamfers the
top edges of a region with a chamfer mill (`ChamferMill`). The depth comes from
the tool's angle and tip. `boundaries` selects outer edges, holes or both.
`cutting_sense` is `"climb"` (default) or `"conventional"`; `reverse` overrides
it. Wider chamfers are cut in steps of `width_step_mm` (default: one pass).
`tip_clearance_mm` (0.5) cuts the chamfer's lower edge that far out along the
cone from the tool's tip (radially), so the flank cuts, not the point. `lead_in_mm`/`lead_out_mm` add tangent
leads. A tool that does not cut at its center needs its entry proven cleared
in the stock first.

## Deburr: `Deburr`

Required: `surface` (the finished part as a `SurfaceMesh`), `levels`,
`width_mm`, `clearance_z_mm`, and a chamfer mill (`ChamferMill`). Breaks the
horizontal edges of a finished part: every boundary of every upward-facing
planar face. `camlib.cadquery.deburr_levels(solid)` makes the levels (the
faces at each height, exported exactly):

```python
from camlib import Deburr
from camlib.cadquery import deburr_levels, export_surface_mesh

op = Deburr("deburr", c90, export_surface_mesh(part), deburr_levels(part), 0.5, 30,
            feed_mm_min=600, plunge_mm_min=200, spindle_rpm=10000,
            tool_number=2, length_register=2, coolant="flood")
```

- The tool axis runs on the outward offset of each face boundary, so the cone
  breaks the edge `width_mm` wide on the face and the chamfer depth down a
  vertical wall (`wall_depth_mm` in the report: equal for 90 degrees),
  whatever the edge's curvature. `tip_clearance_mm` (0.5) keeps the point out
  of the cut.
- Nothing else is touched, by an exact proof for the whole tool (cone,
  neck, shank and holder) against the part above the face, below the break
  and anything in the break's height band that is not the face or a vertical
  wall under its boundary. The proof allows for the mesh's `surface_error_mm`
  and two stock cell diagonals.
- A stretch that cannot be proven is not cut. The tool retracts and
  re-enters where cutting can be proven again (entries and exits are proven
  too). Each stretch is reported in `skipped`: concave stretches, where a
  wall rises from the face, in `concave_length_mm` (nothing to break); convex
  edges the tool cannot reach in `unreachable_length_mm`.
- The stock must already be machined to the part. A non-center-cutting
  chamfer mill enters only where the stock is proven clear under its tip.
- Not yet: ball-end deburring, and edges that are not horizontal (sloped
  or vertical edges need tilting the tool, see the multi-axis issues).

## Parallel finishing: `Parallel`

Required: `surface` (`SurfaceMesh`), `floor_z_mm`, `stepover_mm`,
`clearance_z_mm`. Ball end mill raster along `axis` following the surface.
`path_error_mm` (0.005) bounds how far the path may deviate between computed
points; `safety_margin_mm` (0.005) keeps the tool that far above the exact
no-gouge height; `stock_to_leave_mm` leaves material. In a direct comparison
with OpenCamLib's drop-cutter on the same mesh, camlib's tip stayed between
+0.0046 and +0.0099 mm above the exact no-gouge height.

Depth: when the remaining stock stands no higher above `floor_z_mm` than the
cutting length, every pass is within the flutes. Where it stands higher (a
part taller than the flutes, after roughing), each feed move is checked
instead: the neck, shank and holder sections stay above the committed stock
under them, so only a pass that really cuts deeper than the flutes is refused
(the message names where). `steep_and_shallow` can then take the part's
lowest point as its floor.

Flat and bull-nose end mills work too (raster and the spiral, radial and
project patterns). Their contact with each triangle is exact (facet, edge and
vertex), and every move between computed points is proven not to gouge: per
triangle, the contact along a straight move is concave, so its highest point
is found exactly and compared with the tool. Their finishing error is sampled
along each move (`max_sampled_path_excess_mm` in the report, where a ball end
mill reports the certified `max_certified_path_excess_mm`). Slope limits, rest
finishing, pencil and scallop passes need a ball end mill and are refused for
other cutters.

## Morph and flow finishing: `Morph`, `flow_paths`

`Morph(id, tool, surface, floor_z_mm, stepover_mm, clearance_z_mm, from_path,
to_path)` cuts passes interpolated between two XY guide curves (TracePaths,
both followed from their starts), no two neighbouring passes more than
`stepover_mm` apart anywhere: a blend between two edges or rails. Passes are
followed on the surface like `Project`, with the same proofs.

`camlib.cadquery.flow_paths(face, stepover_mm=..., along="u")` turns a CAD
face's own parameter lines into XY paths at most `stepover_mm` apart, for
`Project`: flow finishing that follows the face's shape. The face only shapes
the passes; the surface mesh decides the heights and the clearance proof.

## Flat-area finishing: `Flat`

Required: `surface`, `floor_z_mm`, `stepover_mm`, `clearance_z_mm`. Finds the
horizontal faces above `floor_z_mm` (faces within `level_tolerance_mm`, 0.001,
form one level) and finishes each level at its exact height plus
`stock_to_leave_mm`, top level first, with a flat center-cutting end mill in
raster rows along `axis`. A cut is kept only where the highest point of the
part under the tool's swept disk is not above the cut (computed exactly per
facet); each run starts with a vertical plunge. Near walls the tool stays a
radius away; the report gives the levels, runs, cut length and any row length
that could not be proven clear (`dropped_length_mm`).

## Spiral, radial and project finishing: `Spiral`, `Radial`, `Project`

Same surface engine, parameters and checks as `Parallel` (ball end mill;
every chord between computed points is certified for path error and
continuous clearance), along other XY patterns:

- `Spiral(..., stepover_mm, clearance_z_mm, center_mm)`: one Archimedean
  spiral, ring pitch `stepover_mm`, inward by default (`outward=True`,
  `clockwise=True` to change). `inner_radius_mm` (0) and `outer_radius_mm`
  (default: reach the whole stock) bound it. Suits round, domed areas.
- `Radial(..., clearance_z_mm, center_mm, angular_step_deg)`: straight spokes
  from the center; the step is rounded down so spokes divide 360 evenly,
  starting at `start_angle_deg` (0).
  `inner_radius_mm` avoids overlapping spokes at the center; `alternate`
  (default) runs every other spoke inward to shorten links.
- `Project(..., clearance_z_mm, paths)`: `TracePath` lines and arcs followed
  on the surface, with the ball resting on it (plus `stock_to_leave_mm`).
  The paths are not offset by the tool radius and must lie within the stock.

Patterns are clipped to the stock grown by the cutter radius. Each run
starts with a vertical plunge onto the surface at `plunge_mm_min` and ends
with a vertical retract (rows of `Parallel` enter from outside the stock).

### Scallop: `Scallop`

`Scallop(id, tool, surface, floor_z_mm, stepover_mm, clearance_z_mm)` cuts
offsets of the stock outline (grown by the tool radius) measured along the
surface, `stepover_mm` apart, so the stepover stays about constant on steep
and shallow areas alike (the usual scallop / constant-stepover strategy). The
distance is computed over the ball-center surface on a sample grid (8
neighbours, so within about 8% of the true surface distance); each offset is
cut as a pass with every chord lifted and checked like `Parallel`.

### Pencil: `Pencil`

`Pencil(id, tool, surface, floor_z_mm, clearance_z_mm, min_angle_deg=20,
sample_mm=None)` follows the concave creases (inside corners, valleys) where
the ball touches two faces at once. On a grid of ball contacts (`sample_mm`,
default a quarter of the tool radius) a crease is where the contact normal
turns by at least `min_angle_deg` and, across it, one contact point lies
above the other side's tangent plane; convex edges, around which the ball
rolls, are not creases. Creases are located by bisection to `path_error_mm`,
linked into polylines, and cut like `Project` paths (every chord lifted and
checked). Creases shorter than the tool radius are skipped.

### Rest finishing

`Parallel`, `Spiral`, `Radial`, `Project`, `Pencil` and `Scallop` take
`rest_threshold_mm` (off by default). With it, passes are kept only where the
ball would reach more than that far into stock that certainly remains above
the target (plus stock to leave) after the preceding operations: rest
machining from the stock model, whichever tools left the material. It uses
the stock's lower bounds, so detection is limited by the stock cell size
(use a finer stock for finishing). It needs the stock, so it runs in a job
after the operations that left the material.

### Slope limits and steep-and-shallow

`Parallel`, `Spiral`, `Radial`, `Project` and `Waterline` take
`slope_from_deg` (0) and `slope_to_deg` (90): the usual contact-angle limits.
The angle is between the tool axis and the ball's contact normal (0 on flats,
90 against vertical walls). Passes are kept only where it lies within the
limits (located to within a quarter of the tool radius); each kept run starts
with a vertical plunge onto the surface. Over a convex edge the contact
normal turns gradually, so a shallow pass may roll over the edge until the
limit.

`camlib.steep_and_shallow(id, tool, surface, floor_z_mm, clearance_z_mm,
top_z_mm=, bottom_z_mm=, stepdown_mm=, stepover_mm=, threshold_deg=45,
overlap_deg=5, shallow="parallel")` returns the two operations of the usual
steep-and-shallow strategy: waterline above `threshold_deg − overlap_deg` and
parallel (or spiral, with `center_mm`) below `threshold_deg + overlap_deg`.

## Waterline finishing: `Waterline`

Required: `surface`, `floor_z_mm`, `top_z_mm`, `bottom_z_mm`, `stepdown_mm`,
`clearance_z_mm`. Ball end mill contours at constant Z, the complement of
parallel finishing on steep walls. A flat end mill's contours are the exact
offset of the part's outline above each level; a bull nose's are built the
same way in thin bands over its corner height, never closer than allowed and
at most a few hundredths of the corner radius farther out.

`simplify` (default on) removes contour points that are not needed within
`contour_error_mm`; `reverse` cuts the contours the other way;
`safety_margin_mm` and `stock_to_leave_mm` work as for `Parallel`.

`ramp=True` (ramp finishing) cuts each level as one continuous descent along
its contour from the level above, instead of a flat lap and a step down; the
lowest level is then cut once more flat. Each ramp follows a contour proven
clear at its own level and stays above that level, so it cannot gouge. `contour_error_mm` (0.01) bounds the contour
approximation as a *normal* (3D) distance to the surface. Z levels are computed
in parallel. `contour_method="quad_tree"` (default) scales to dense meshes;
`"support"` builds exact per-facet polygons and suits small or convex meshes.
On a 51,200-triangle dome, contours agreed with OpenCamLib's waterline and kept
0.003 to 0.010 mm normal clearance to the true surface.

## Plasma and laser cutting: `ThermalCut`

Required: `parts` (a `PlanarRegion`: each component is a part to keep, with its
holes), `feed_mm_min`, `cut_height_mm`, and a beam tool:
`camlib.Torch(id, kerf_width=...)` or `camlib.LaserHead(id, kerf_width=...)`
(the kerf is the beam's diameter; the nozzle and body are collision geometry).
The setup's stock is the sheet (its top and thickness).

```python
torch = camlib.Torch("torch", kerf_width=1.5)
cut = camlib.ThermalCut("plasma", torch, parts, feed_mm_min=2500, cut_height_mm=1.5,
                        lead_in=camlib.Lead(length_mm=2, radius_mm=3),
                        lead_out=camlib.Lead(length_mm=1, radius_mm=2),
                        overcut_mm=-0.5, pierce_delay_s=0.5, power_percent=100,
                        height_control=True, small_holes=camlib.SmallHoles(feed_percent=60),
                        tool_number=1, length_register=1)
```

- **Kerf compensation**: the beam center runs half a kerf (plus half of
  `boundary_tolerance_mm`) outside each part. `compensation="wear"` posts the
  same path with G41/G42, turned on over the straight lead-in and off over the
  straight lead-out, so the operator can trim the kerf at the controller.
- **Order**: holes before their part's outline; a part lying in another part's
  hole before that hole; parts nearest first. The report lists the order.
- **Direction**: outlines clockwise, holes counter-clockwise (the part on the
  right of travel, the good side of a plasma arc with clockwise swirl);
  `reverse=True` swaps both.
- **Leads**: `Lead(length_mm, radius_mm)`: a straight part and a tangent arc,
  both in the scrap. `overcut_mm` cuts past the start; a negative value starts
  the lead-out that far before the loop closes (negative overburn, the
  Hypertherm recommendation for holes with an arc lead-in). A loop starts at
  its first vertex; where the leads there would leave the scrap (a corner of a
  straight-edged hole), they move to the middle of the longest edge whose
  leads are clear (reported).
- **Pierce**: each pierce point keeps `pierce_clearance_mm` (default one kerf)
  beyond half a kerf from every part. In a hole too small for the leads, the
  pierce moves to the hole center (reported).
- **Small holes** (`SmallHoles(feed_percent=60, max_diameter_mm=None,
  height_control_off=True)`): holes up to `max_diameter_mm` (default 1.5 ×
  the sheet thickness, Hypertherm's definition) are cut at the reduced feed,
  with torch height control off.
- **Heights**: `height_mode="fixed"` (default) holds one Z, the cut height above
  the sheet top: the controller handles probing, pierce height and height
  control (QtPlasmaC, lasers). `height_mode="program"` writes the moves itself:
  up to `travel_height_mm`, down to `pierce_height_mm`, beam on, `pierce_delay_s`,
  down to `cut_height_mm` at `plunge_mm_min`.
- **Posts**: generic G-code (`M3 S`/`M5`, declared THC words), GRBL laser
  (`M4 S`, [examples/grbl_post.py](../examples/grbl_post.py)), [QtPlasmaC](qtplasmac-post.md), [Centroid CNC12
  plasma](centroid-plasma-post.md) and [WinCNC plasma](wincnc-plasma-post.md).
  Torch height control itself runs in the controller (arc voltage); posts
  only switch it on and off where the controller takes that from the
  program (QtPlasmaC `M63/M62 P2`), and leave it to the material setup on
  Centroid and WinCNC.
- **Height control**: `height_control=True` turns torch height control on after
  the lead-in and off before the lead-out of each loop.
- **Micro-joints**: `MicroJoints(width_mm, count=... or spacing_mm=...,
  holes=False)` leave uncut bridges on outlines; the beam turns off over each
  and fires again after it (no pierce clearance there). Not with wear
  compensation.
- `power_percent` is the laser power (posted as S).

Every lead and kerf path is proven not to touch any part, and every pierce
crater to stay clear of them, before any motion is produced. The simulated
sheet is cut through wherever the kerf passes. Not modeled: tipped-up parts
after they are cut free (plan travel heights accordingly), bevel, marking and
common-line cutting.

## Turning (lathe): a lathe `Setup`

Lathe coordinates: x is the radius from the spindle axis, z runs along it.
Programs move each tool's imaginary tip (`controlled_point="tip"`, the
default): the nose center one nose radius toward the diameter (toward the
axis outside, away from it in a bore) and toward the face it cuts toward
(-z for right-hand tools; threading inserts on their center line). That is
the point offsets touched off on a diameter and a face measure, so a finish
along a diameter is posted at that diameter. `controlled_point="nose_center"`
posts the nose center instead. The start position `initial_position_mm`
(radius, 0, z) names the controlled point; each tool's arrives there after
its change (the turret index position).
The part is its half-section `profile`: points `(radius, z, bulge)` from the
front face (on the axis) toward the chuck; the bulge is tan(sweep/4) of the
arc to the next point (positive: G3). A part with a hole also declares its
`bore`: the hole's half-section `(radius, z, bulge)` from the front face to
its deepest point (a drill point included). Drills and boring tools are
checked to stay inside it, on both sides of the axis.

A lathe setup is an ordinary `camlib.Setup` whose stock is a
`TurnStockSpec`, tools `TurnToolSetting`, operations the `Turn*` operations,
with its turning definition in `turning=camlib.TurningSetup(...)`. It runs,
plays back, posts and exports like any other setup: each operation is planned
when the run reaches it, every move is checked as it is planned and streamed
with the exact stock change it makes (`Simulation`, `RecordedRun`/`Player`,
`Job.post`, `Job.post_nc`, `run.export`).

```python
# Every tool value is stated: an ISO 1832 insert on an ISO 5608 holder style,
# and camlib's rectangular holder (or your holder's outline).
insert = camlib.TurningInsert.iso_code("CNMG120408", holder="PCLNR2525M12")  # style L: 95 degrees
holder = camlib.TurningHolder.rectangle(insert=insert, nose_radius_mm=insert.nose_radius_mm, side="outer",
                                        width_mm=25, length_mm=150, offset_mm=0)
tool = camlib.TurningTool("cnmg", insert=insert, side="outer", hand="right", holder=holder)
lathe = camlib.Setup(
    "lathe",
    stock=camlib.TurnStockSpec(z_min_mm=-60, z_max_mm=2, outer_radius_mm=20),
    tools=(camlib.TurnToolSetting(tool, number=1, offset=1),),
    # Feeds per revolution and speeds have no defaults: set them for the insert.
    operations=(camlib.TurnFace("face", "cnmg", z_mm=0, feed_mm_rev=0.15, rpm=1000),
                camlib.TurnRough("rough", "cnmg", depth_of_cut_mm=2, allowance_mm=0.3,
                                 feed_mm_rev=0.25, rpm=1000),
                camlib.TurnFinish("finish", "cnmg", feed_mm_rev=0.1, rpm=1200)),
    initial_position_mm=(30, 0, 20),
    turning=camlib.TurningSetup(
        profile=((0, 0, 0), (10, 0, 0), (10, -20, 0), (15, -20, 0), (15, -40, 0)),
        chuck_z_mm=-50, clearance_mm=2, tolerance_mm=1e-3))
job = camlib.Job("shaft", "1", (lathe,))
with job.record("lathe", rapid_mm_min=6000) as run:   # playback: Player(run)
    run.wait()
program = job.post_nc("lathe", machine=machine)        # checked NC text
```

Playback seeks to any point, within a move too: the stock shown partway
through a move is the planner's move kernel on that first part of the move,
from the stock at its start (as milling and thermal runs sweep the cutter to
that fraction). The stock is the per-z radius bounds of the outside and of
the bore, plus removed rings (`state.stock()`). `Job.post` replays the
posted program on the stock with the planner's own move kernel, so a posted
move is checked exactly like a planned one.

Material check: each operation's end report carries `material`, the stock
after it compared with the part (`state.material()` for any committed
stock; `PostedProgram.material` and `run.export(...)["material"]` for the
program's final stock). Gouges are excluded per move: every insert position
is checked against the part and bore as it is made (`gouge.proof`), so a
gouge stops planning or replay. Leftover material is compared per z bin of
the stock (its radius bounds against the part's radius range over the bin,
outside and in the bore; definite or possible) and per declared face groove
(its bottom corners keep the blade's nose radii). Leftover material is
reported, not refused: a sharp inside corner keeps the nose radius, a
roughing setup its allowance.

- `TurnFace(z_mm, stepdown_mm)`: passes across the axis down to `z_mm`.
- `TurnRough(depth_of_cut_mm, allowance_mm, retract_mm)`: axial passes (G71
  type I), each ending where the part plus allowance is reached, retracting
  at 45 degrees. Besides the depth-of-cut steps there is a pass at every
  diameter of the profile, so each one is left at the allowance (never up
  to a depth of cut more).
- `TurnGroove(z_from_mm, z_to_mm, bottom_radius_mm, peck_mm=None,
  overlap_mm=0.2)`: plunges of a `GrooveInsert` tool across a groove that is
  in the profile (internal grooves with a `side="inner"` tool: in the
  `bore`, plunging outward from inside the open hole, entered and left
  along the axis); `TurnPartOff(z_mm)`: the blade's +z side at the part's back
  face, plunged to the axis (the blade must be deeper than the stock radius;
  the holder is checked). A rapid back out along the plunge it just made is
  clear by construction.
- `TurnThread(z_start_mm, z_end_mm, major_radius_mm, pitch_mm, depth_mm,
  passes=6, infeed_angle_deg=29.5, spring_passes=1)`: single-point threading
  with a `ThreadInsert` tool, spindle-synchronized passes of constant chip
  area. The thread must start where the diameter starts (the passes run
  through everything before it), and its run-out (depth × tan of half the
  profile angle) must be free of shoulders. The stock keeps its crests (only
  its solid bound moves to the root). With a boring-side tool
  (`side="inner"`) the thread is internal: the `bore` is its minor
  diameter, the passes cut out to `major_radius_mm`, pull out toward the
  axis and leave along it inside the crests; a blind bore needs room past
  `z_end_mm` for the run-out.
- `TurnFaceGroove(inner_radius_mm, outer_radius_mm, bottom_z_mm,
  peck_mm=None, overlap_mm=0.2)`: axial plunges of a
  `FaceGrooveInsert(width_mm, depth_mm)` tool (blade pointing -z, its
  reference the inner corner) across an annular groove in the front face.
  The groove is declared on the setup, `TurningSetup(face_grooves=(FaceGroove(r_in,
  r_out, bottom_z),))`, within the flat front face; below the face the blade
  is checked to stay inside it. The stock records the ring as removed
  (`state.stock()["rings"]`; its radius bounds still count it as possible
  material, which keeps other tools' checks conservative).
- `TurnDrill(depth_z_mm, peck_mm=None)`: a `DrillInsert(diameter_mm,
  point_angle_deg, flute_length_mm)` tool on the axis, its tip to
  `depth_z_mm`, in full-retract pecks if set (back in to just above the last
  depth). The point and body stay inside the `bore`, the holder clear of the
  stock; the stock's hole (inner bounds, cone at the bottom) is updated.
- `TurnBoreRough(z_end_mm, depth_of_cut_mm=1.5, allowance_mm=0.2,
  retract_mm=0.5)`: inside roughing with a boring tool: axial passes opening
  the hole already there (stock bore or drilled) by at most the depth of
  cut, with a pass at every bore diameter, each from the front to where the
  bore less the allowance is reached or `z_end_mm`, retracting at 45 degrees
  toward the axis. `TurnBoreFinish(z_end_mm, allowance_mm=0)`: along the
  bore from the front face to `z_end_mm`, then out at 45 degrees and along
  the axis. Posted as `G33 Z… K<lead>` (LinuxCNC)
  or `G32 Z… F<lead>` where the dialect declares `thread_synchronized` and
  `thread_lead_word`; dialects without them refuse it.
- `TurnFinish(allowance_mm)`: the profile from the front face, the nose
  center a nose radius (plus allowance) outside it: rounded at convex
  corners, trimmed at concave ones. A feature too small for the nose is
  refused. `compensation="wear"` (also on `TurnBoreFinish`) posts the same
  path with tool-nose radius compensation on (G42 outside, G41 inside,
  cutting toward the chuck), turned on over the straight entry and off
  over the retract, so the control applies only the wear in its offset's
  nose radius (tip direction from the offset).
- Tools: `TurningTool(id, insert=, side=, hand=, holder=, nose_radius_mm=)`,
  every value required (a missing one is refused by name; nothing stands in
  for a tool that isn't yours). Geometry is in the tool's XZ frame: x away
  from the spindle axis, z along it, drawn as an outside, right-hand tool
  cutting toward -z (camlib mirrors it for `side="inner"` and
  `hand="left"`).
  - Inserts: `InsertOutline(outline, corner, cutting, programming_point_mm)`
    for any shape (round, trigon, a real insert's back edge or wiper):
    `outline` is `turning.Outline(start_mm, segments)` of `turning.Line(to_mm)`
    and `turning.Arc(center_mm, radius_mm, start_deg, end_deg)` (closed, not
    crossing itself); `corner` is the index of the arc that is the cutting
    corner (its center the nose center, its radius the nose radius, a circle
    that must lie in the insert); `cutting` the indices of the segments that
    may cut (every other segment is checked to stay clear of the stock when
    it leads, like an insert's back); `programming_point_mm` the point offsets
    are touched off with (the imaginary tip). Constructors:
    `TurningInsert.iso(included_angle_deg=, approach_angle_deg=,
    edge_length_mm=, nose_radius_mm=)` (camlib's wedge: nose, two edges,
    straight back) and `TurningInsert.iso_code(code, approach_angle_deg= |
    holder=)` (the whole insert, every corner rounded, from an ISO 1832
    designation: shapes C, D, V, S, T, W negative, R; the approach angle from
    an ISO 5608 holder style letter). `GrooveInsert(width, depth)`,
    `FaceGrooveInsert(width, depth)`, `ThreadInsert(profile_angle, depth)`
    and `DrillInsert(diameter, point_angle, flute_length)` keep their
    parameters (their operations plan from them) and take `nose_radius_mm`.
  - Holders: `TurningHolder(outline, bar=None)`, an outline in the same
    frame, anywhere relative to the insert (a head ahead of it, a stepped
    head), and for a boring bar a `TurningBar(diameter_mm, axis_x_mm, z_mm)`.
    `TurningHolder.rectangle(insert=, nose_radius_mm=, side=, width_mm=,
    length_mm=, offset_mm=)` builds camlib's rectangle: outside, a shank
    behind the insert's reach; boring, an axial bar `width_mm` across, its
    top `offset_mm` under the nose center, `length_mm` out of the bore.
  - ISO values: shapes and corner angles from ISO 1832:1991 section 4.1
    table 1; size codes (inscribed circle per shape) and corner radius codes
    from the Sumitomo Electric Hardmetal general catalog, section B
    "Indexable Insert Identification Code", tables 5 and 7, p. B3, which
    reproduce ISO 1832's designations (edge length from the inscribed circle
    by each shape's geometry, matching the tabulated lengths); holder styles
    (cutting edge angle) from ISO 5608:1995 section 4.3 table 3. Round
    sizes 12 and 25 each name two diameters and are refused (give an
    outline).
- Checks on every move: the swept insert against the part (a tolerance of
  touching allowed), so a profile the insert's trailing edge cannot follow is
  refused; drills and boring tools inside the `bore` on both sides of the
  axis; the holder (outline and bar, swept like the insert, in convex
  pieces when it is not convex) against the part, its bore and the stock
  left after the move; non-cutting insert segments that lead against the
  stock; rapids
  against the stock (inside tools: within the hole the stock certainly has
  open); nothing past the chuck face. The turning stock (outer and inner
  radius bounds per z bin) is updated by each cut and returned.
- Speeds and feeds: each operation runs at `rpm` (G97), or at constant
  surface speed with `surface_speed_m_min` (G96) and `rpm` as its spindle
  limit (G50). Feeds are per minute (`feed_mm_rev × rpm`) unless the job sets
  `feed_per_rev=True` (G95), which constant surface speed needs. Each feed
  move then carries the per-revolution feed times the highest spindle speed
  along it (the move's least radius), which the program validator
  recomputes. Threading and drilling run at a fixed rpm.

A lathe setup posts (`job.post_nc`, `job.post`) with any `Machine` whose `planes` include `"xz"`: arcs
are written in G18. `GCodeDialect(x_diameter=True)` writes X as a diameter
(arc I stays a radius, as on FANUC and Centroid lathes); for LinuxCNC declare
`lathe_radius_mode="G8"` (radius X) with `with_commands`. The [Centroid CNC12 lathe post](centroid-lathe-post.md)
is a complete controller example (diameter X, T0101, G96/G50, G99, G32), as
is the [FANUC lathe post](fanuc-lathe-post.md) (G-code system A).

From CAD: `camlib.cadquery.turning_section(solid)` returns the `profile` and
`bore` of a solid turned about its Z axis (front face toward +Z), lines and
arcs kept exact. It refuses a solid that is not a solid of revolution (its
volume differs from the half-section revolved: flats, cross holes, keyways),
closed voids, cavities on the axis behind solid material, undercuts and
other curve types. Rectangular grooves in the front face come back as
`s.face_grooves` (`TurningSetup.from_section(s, ...)` takes all three).

```python
s = camlib.cadquery.turning_section(part.val())
turning = camlib.TurningSetup.from_section(s, chuck_z_mm=-50, clearance_mm=2, tolerance_mm=1e-3)
```

The turning stock is removed by everything the insert sweeps: its nose
exactly and its straight sides. Material may reach an insert only through
its cutting edges: a move whose back (non-cutting) side leads into stock is
refused, and stock beyond the edges' reach is left for the holder check, so
a depth of cut beyond the cutting edge is refused.

## Tools

`camlib.tools`: `EndMill`, `BallEndMill`, `BullEndMill(corner_radius=...)`,
`Drill(point_angle=118)`, `ChamferMill(angle=90, tip_diameter=0)`,
`ThreadMill(diameter=, tooth_height=, neck_diameter=, neck_length=)`,
`Tap(drill_diameter=, thread_length=, shank_diameter=)`,
`BoringBar(diameter=, bar_diameter=, reach=, insert_length=1)`,
`Torch(kerf_width=, nozzle_diameter=6, body_diameter=30)` and
`LaserHead(kerf_width=, nozzle_diameter=3, body_diameter=25)` (complete
assemblies). The cutter helpers take
`diameter`, `flute_length`, `overall_length` and optional `shank_diameter`,
`flutes`, `center_cutting`, `material`, `name`; `neck=Neck(diameter, length)`
models a reduced neck above the flutes. A
`ToolAssembly(id, cutter, stickout_mm, gauge_length_mm, holders)` adds how far
the tool sticks out of the holder, the gauge length (spindle gauge line to tip,
the length offset) and `HolderSection(z_min_mm, z_max_mm, radius_min_mm,
radius_max_mm)` cone sections above the tip, which the collision checks use.

## Speeds and feeds

Every operation requires `spindle_rpm` and `feed_mm_min` (and its other
feeds). To compute them from the tool maker's cutting data:

```python
from camlib.cutting import cutting_data

data = cutting_data(tool, surface_speed_m_min=200, feed_per_tooth_mm=0.05,
                    radial_engagement_mm=1.5, max_rpm=12000)
op = Pocket(..., spindle_rpm=data.spindle_rpm, feed_mm_min=data.feed_mm_min)
```

It applies n = 1000·Vc/(π·D) and vf = n·z·fz, raises fz for radial chip
thinning below 50% engagement (`chip_thinning=False` turns that off), uses the
effective diameter of a ball end mill at a shallow `axial_depth_mm`, and caps the
speed at `max_rpm` (reported as `rpm_limited`). The result (`CuttingData`) also
gives the `effective_diameter_mm` it used. camlib does not pick cutting data.

## Not supported

Undercuts (other than thread
grooves and the stopped boring bar's shift) and 4/5-axis motion. Surfaces must
be reachable from above (a part seen as a height map).
