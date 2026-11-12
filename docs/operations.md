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
| `one_way` | off | Parallel: cut every row in the same direction instead of alternating (see [Parallel finishing](#parallel-finishing-parallel)); Facing has `mode="one_way"` |

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
  stock where possible, helical otherwise; `RampEntry(angle_deg)` ramps in
  along the cutting path (see [Ramp entry](#ramp-entry-rampentry)), the entry
  for slots and narrow pockets a helix does not fit.
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

**A top below the remaining stock** (`Pocket` and `AdaptivePocket`, issue
#129). `top_z_mm` may be below the stock's original top when earlier
operations removed the material: the depth (and the cutting-length check) is
then measured from `top_z_mm`. Every stock cell meeting the `region` must be at
or below it. Stock may stand higher outside the region, for example a low
pocket beside a tall wall, when the tool assembly clears it:

- Before any motion, the neck, shank and every holder section are checked
  against that stock for every tool centre the cutter can occupy in the region
  and every tip at or above the operation's floor (plus bottom allowance).
  A section that would meet it refuses the operation with a typed reach
  refusal (below) naming the section, the obstruction's top and the depth.
- `clearance_z_mm` must then be above all remaining stock, and a planned move
  that leaves the region's tool centres below that height is refused.
- The plan's report gives `stock_top.outside_upper_z_mm` (the highest such
  stock) and `assembly_checked_cells`.

This is conservative (every pose at the lowest tip anywhere in the region);
configure stock-contact checking (`StreamChecks(stock_contact=...)`) to check
every move's assembly against the actual stock independently.

**Reach refusals are typed.** When a tool cannot reach the depth an operation
needs, the run raises `camlib.simulation.ReachRefused` (a `SimulationError`)
whose `refusal` is a `ReachRefusal`: `operation`, `tool`, `limit`
(`"cutting_length"`, `"neck"`, `"shank"` or `"holder"`, with
`holder_section`), `depth_needed_mm`, `depth_available_mm`, `top_z_mm` (the
material top the depth is measured from: the operation's top, or the
obstruction's) and `z_mm` (the lowest tip Z needed); `depth_needed_mm =
top_z_mm - z_mm`. Pocket, adaptive pocket, slot, contour, drilling, bore,
facing, chamfer, waterline, trace, parallel finishing and roughing layers
refuse their cutting-length depth this way, and pockets their neck, shank and
holder sections. Parallel finishing and waterline, when the remaining stock
stands taller than the flutes, refuse a feed move whose neck, shank or holder
section would meet that stock the same way (`z_mm` is the move's lowest tip). In Rust, `camlib::reach::ReachRefusal::parse` reads it from
the error text.

### Ramp entry: `RampEntry`

`RampEntry(angle_deg, min_length_mm=None, feed_mm_min=None)` enters by
descending along the pass the tool is about to cut, never steeper than
`angle_deg` to the XY plane, at the entry feed (`feed_mm_min`, default the
operation's `plunge_mm_min`). The pass is then cut at full depth, which removes
the sloped floor the ramp left. `angle_deg` is a fact about the tool and
material, so there is no default.

References:

- FreeCAD CAM's Ramp Entry dressup
  ([`Path/Dressup/Gui/RampEntry.py`](https://github.com/FreeCAD/FreeCAD/blob/9623213a59dd3cda20fb573f8295691af23b6b68/src/Mod/CAM/Path/Dressup/Gui/RampEntry.py),
  generator
  [`Path/Base/Generator/ramp_entry.py`](https://github.com/FreeCAD/FreeCAD/blob/9623213a59dd3cda20fb573f8295691af23b6b68/src/Mod/CAM/Path/Base/Generator/ramp_entry.py)):
  a plunge is replaced by a descent along "the path that comes after the
  plunge" over the XY length `plungelen / tan(angle)`. The pocket's one-leg
  ramp is its "Ramp Method 2" ("Travel at start depth along the path, for a
  distance required", then "Ramp backwards along the path at rampangle,
  arriving exactly at the bottom of the plunge"). On a path shorter than that
  length its ramp helper turns round at the end of the path and keeps
  descending ("going back-and-forth"), which is the pocket's zigzag. On a
  closed path its "Helix like path" method descends in whole loops,
  `max(ceil(height / pitch), ceil(height / length / tan(angle)))` of them
  (`pitch`: "Maximum height for each helix turn"), which is the contour's ramp
  with `stepdown_mm` as the pitch.
- Autodesk Fusion,
  [2D Pocket reference](https://help.autodesk.com/cloudhelp/ENU/Fusion-CAM/files/GUID67DC60DE-EE42-4BC4-B50D-B28D938D146E.htm)
  and
  [2D Contour reference](https://help.autodesk.com/cloudhelp/ENU/Fusion-CAM/files/GUID75B6821B-DE26-4E3B-AF10-4A54131CD9E4.htm):
  "Ramp Type" (Zig-Zag, Profile, Helix, Plunge, ...), "Ramping Angle (deg):
  Specifies the maximum ramping angle of the helix during the cut", and
  "Maximum Ramp Stepdown: Specifies the maximum stepdown per revolution on the
  ramping profile. This parameter allows the tool load to be constrained when
  doing full-width cuts during ramping."

Differences from FreeCAD, on purpose: camlib's zigzag gives every leg the whole
path at one equal slope (shallower than the angle) so the last leg ends at the
start of the cut, where FreeCAD keeps the angle and returns to the start at
depth; and camlib never leaves a plunge where no ramp fits (FreeCAD logs "plunge
will remain as such"): it refuses.

- **`Pocket`**: every descent into a level is a ramp (the first pass of the
  level, and every pass reached by a retract rather than a short link at depth).
  The ramp runs along that pass and the passes continuing it without a gap
  (lines, and arcs as helical arcs), and ends at the start of the pass. When
  the path is at least depth / tan(angle) long it is one leg at exactly the
  angle; when it is shorter the tool zigzags back and forth over the whole
  path, every leg at the same, shallower slope.
- **`min_length_mm`** is the shortest span the tool may reverse a ramp on. It
  has no default: neither reference defines one, and camlib assumes no cutting
  data. A one-leg ramp never reverses and does not need it; a pocket where a
  ramp has to reverse is refused until it is declared, and refused where the
  pass is shorter than declared.
- The first level's ramp starts at the pocket top. Each later one starts 0.01
  mm above the level before, at a point of the path that level has cut, and
  the planner requires the simulated stock to show that path cut before it
  descends straight to the start: the straight descent is through air, and
  every move below it is a ramp. The same holds on stock earlier operations
  have cut, and for a pocket whose `top_z_mm` is below the stock top.
- **`Contour`**: each loop is cut in one continuous descent instead of a pass
  per level. The tool comes down to the stock top at the lead-in point, leads
  in there, descends around the closed loop in whole laps to the floor (as many
  as keep the slope within the angle and each lap within `stepdown_mm`), then
  cuts one lap flat at the floor before the lead-out. A closed loop needs no
  reversal, so `min_length_mm` does not apply, and the tool never comes
  straight down into material. With `compensation="wear"` G41/G42 goes on over
  the lead-in at the stock top and stays on down the whole descent.
- Ramp moves are `plunge` moves at the entry feed, swept through the stock like
  any other move.
- The pocket report gives `ramp_entries` and `max_ramp_angle_deg` (the steepest
  ramp made, never above `angle_deg`).

Refused, before any motion:

- A pocket pass the ramp has to reverse on, with `min_length_mm` not set or
  larger than the pass. camlib does not plunge instead: the message names the
  pocket, the place and the level. A smaller angle does not help; use
  `HelicalEntry`, `entry=None` (plunge), a smaller cutter, or declare
  `min_length_mm` for the tool.
- More than 1000 zigzag legs or laps (an angle far too small for the path).
- `RampEntry` on a `Pocket` with `rest_tolerance_mm` (`RestPocket`): rest
  passes are short leftover pieces a ramp does not fit.
- `RampEntry` on a `Contour` with holding tabs, or with limits that block part
  of a loop: the tool comes back down from a tab or a lift steeper than the
  ramp angle. A `Contour` accepts no other entry type.
- `RampEntry` on `Roughing` and `AdaptivePocket`: their passes are linked and
  their engagement proven from stock-edge and helical entries, and a ramp along
  the first pass is a full-width slot outside that model.

Refused while running, as a planner defect, should it ever happen: a ramp whose
start the simulated stock does not show cut by the level above. The planner
stops rather than descend straight into material.

## Adaptive pocket: `AdaptivePocket`

Same region and depths as `Pocket`, cleared with a bounded radial engagement:
no full-width slotting, for hard materials and high-speed toolpaths.
`retract_z_mm`/`feed_z_mm` work as for `Pocket`.

- **It follows the stock** (`path_strategy="engagement_front"`, the default):
  one depth level at a time, each pass runs along the boundary between cut
  and uncut stock inside the region, so nothing is cut twice. The region's
  boundary is a wall.
- **`stepover_mm` is the radial width of each pass**, at most the cutter
  radius. Its contact angle, `acos(1 - stepover / r)`, is the limit every
  cutting stroke is proven against (see `AdaptiveClearing` under
  [3D roughing](#3d-roughing-roughing-restroughing) for the proof and the
  report fields, which each level carries). A `max_engagement_deg` below that
  angle narrows the width further.
- **Entry:** each level enters by a helix into uncut material, with `entry`'s
  helix (`HelicalEntry`) or `HelicalEntry`'s defaults when none is given, and
  straight down only through stock the committed cuts prove is gone.
- `cutting_sense`: `"climb"` (default) or `"conventional"` with a clockwise
  spindle.
- **Links and entries** take the options `Roughing`'s adaptive clearing has
  (described with their proofs under
  [3D roughing](#3d-roughing-roughing-restroughing)). All four are unset by
  default. The link options change feeds only: the path is the same, move
  for move. The entry options change how the cutter comes down, never a
  cutting pass.
  - `link_feed_mm_min`: between passes the cutter returns at depth across
    stock it has already cut (a stay-down link). Without this option those
    links run at `feed_mm_min`. With it, each stretch of a link (at most
    2 mm) whose whole cutter footprint the committed stock's cell bounds
    prove clear of stock runs at this feed; a stretch that may touch stock
    (beside a wall's skin, or entering the next pass) keeps `feed_mm_min`.
    There is no default: declare the feed your machine should reposition at.
    On a 50 x 30 x 8 mm pocket (10 mm cutter, 3 mm stepover, 600 mm/min) the
    links are 705 mm against 1,333 mm of cutting; see
    [benchmarks](benchmarks.md) for measured cycle times.
  - `link_chip_thinning` (needs `link_feed_mm_min`): stretches that may touch
    stock lightly feed between the two rates, by radial chip thinning from a
    proven bound.
  - `rapid_descent_clearance_mm`: a level below one already cut is entered
    by a rapid down to this far above the height the committed cuts prove
    clear for the whole tool (and above every stock cell bound under it),
    then the plunge feed. Without it every descent feeds from `feed_z_mm`.
    An entry beside the material it cuts comes down a few stock cells back
    from it and feeds across at depth, which adds a short link.
  - `ramp_clearance_mm`: each helix starts this far above the material it
    meets, never above `feed_z_mm`.
  The wall pass is entered by a plunge and its lead-in arc at `feed_mm_min`;
  it has no stay-down link. `path_strategy="disk_front"` and `"stock_front"`
  refuse all four by name.
- Orbit options, `max_stay_down_mm` and `rest_tolerance_mm` do not apply and
  are refused. Corners the cutter cannot reach are left and reported as the
  level's residual area.
- `path_strategy="disk_front"` is the earlier planner: it covers the region
  with orbits of `orbit_radius_mm` (default half the tool radius) whatever
  the stock holds. Its orbits overlap, so it cuts many times longer (33.7 m
  against 1.1 m on a 50 x 30 x 8 mm pocket with a 10 mm cutter). With it,
  `clockwise` sets the orbit direction, `max_engagement_deg` (default 90)
  bounds the total contact, and `min_orbit_radius_mm` with
  `local_orbit_tolerance_mm` sizes each orbit from the clearance around it.
- `path_strategy="stock_front"` is a development strategy; it stops with an
  error where its coverage is unresolved.

## Surface mesh inputs

Every operation taking a `SurfaceMesh` (roughing, deburr, flat, parallel,
waterline, steep-and-shallow and the other surface finishing strategies)
requires version 1, millimetres, at least one nondegenerate triangle, and a
finite `surface_error_mm` in **0–0.1 mm, inclusive**. This is the caller's
asserted bound on the mesh's error from the intended surface; 0 means the
facets themselves are authoritative. If the bound exceeds 0.1 mm, supply a
finer mesh with a justified bound; do not reduce the stated error without
improving the mesh. Python checks this range at construction, and the engine
checks it for every mesh input. Preparation errors name the failed check.

## 3D roughing: `Roughing`, `RestRoughing`

Required: `target` (a `SurfaceMesh` of the part, or use
`Roughing.from_solid(...)` with CadQuery), `top_z_mm`, `floor_z_mm`,
`stepdown_mm`, `clearance_z_mm`, `clearing`.

The target follows the [surface mesh input limits](#surface-mesh-inputs),
including the inclusive 0.1 mm `surface_error_mm` maximum.

Clears stock in Z layers while staying above the part surface plus
`radial_allowance_mm`/`bottom_allowance_mm`. Each layer protects the part's
projection above it (its "shadow"), simplified within `contour_tolerance_mm`
(default a quarter of the radial allowance; 0 keeps the exact shadow) and
grown by it, so it still contains the exact shadow while its moves no longer
follow every facet of a fine mesh (at most twice that tolerance more stock
is left; the layer report's `shadow.simplified` gives the tolerance and the
exact shadow's vertex count). Because the simplified shadow is bigger, it can
narrow a gap the cutter still fits until no entry reaches it; that layer then
keeps the exact shadow, as it would with `contour_tolerance_mm` 0, and
`shadow.simplified.reason` says so with the entry refusal it avoided. Holes in that projection smaller than a
disk of half the cutter's radius are protected too, since the cutter cannot
enter them (the layer report's `shadow.filled_holes`). A target mesh whose faces
were tessellated separately leaves such slivers between neighbouring faces; a
gap that still makes the layer's topology ambiguous is refused with its
location. `clearing` is one of:

- `RasterClearing(stepover_mm, routing=None)`: parallel passes;
  `routing="economical"` orders and joins the rows to shorten links,
- `OffsetClearing(stepover_mm, coverage_margin_mm=0.01)`: contour-parallel
  passes, with that much overlap reserved for the coverage proof,
- `AdaptiveClearing(max_radial_engagement_mm)`: engagement-limited clearing
  that follows the boundary between cut and uncut stock
  (`path_strategy="engagement_front"`, the default). Passes start from air
  beside the stock or from stock already cleared (proven clear by the
  committed cuts) and helix only into enclosed material; each pass starts at
  the upstream end of the material it cuts; short moves between passes stay
  down when the link is clear.
  - **The engagement is proven, not estimated.** The declared width `w` on a
    cutter of radius `r` is a contact angle, `acos(1 - w/r)`: the arc of the
    cutter in material in a straight pass of that width. Steering runs on a
    fine grid built from the stock model, and then every cutting and
    stay-down link stroke is proven exactly against the committed cleared
    sweeps: its contact angle, not counting the cutter's own trail, is at
    most that limit. The layer report gives `engagement_limit_deg`,
    `proven_max_engagement_deg` (and its width, `proven_max_engagement_mm`),
    `engagement_proved_strokes` and `engagement_replans`: a pass with an
    unproven stroke is planned again 5% narrower.
  - **Confined material is entry motion.** In a pocket barely wider than the
    cutter no step width keeps the contact within the limit, just as the
    helix that opens it cuts a full slot. Strokes still unproven at half the
    declared width are cut as lead moves at the entry feed (the helix feed,
    else the plunge feed) and counted (`confined_strokes`,
    `confined_stroke_mm`, `confined_max_engagement_deg`). This needs an entry
    that allows a helix; with `StockEdgeEntry(helix=None)` the operation
    stops and names the place.
  - Every cutting and link move is also checked exactly against the layer
    region before it is emitted, and the run fails rather than emit one that
    does not fit. Orbit options and `omit_empty_cuts` do not apply, the entry
    must be `StockEdgeEntry`, and the direction is `cutting_sense` (`"climb"`
    by default, `"conventional"`); `clockwise` does not apply.
  - `path_strategy="disk_front"` is refused in `Roughing`. It covers a region
    it takes to be solid with small orbits and never reads the stock;
    `Roughing`'s region is the stock grown by a margin, so it orbited through
    the air beside the stock (#167: 75 m of a 137 m path on a 130 x 90 mm
    stock). It remains `AdaptivePocket`'s strategy, where the region's
    boundary is a wall. `path_strategy="stock_front"` is a development
    strategy with the same options as `AdaptivePocket`.
  With `rapid_descent_clearance_mm`
  (opt-in, on `Roughing`), a descent into a column that the committed cuts
  prove clear for the whole tool assembly is rapid down to that far above
  the proven height, and only the rest is fed; otherwise every entry feeds
  down from `feed_z_mm` through material already cut. An entry beside the
  material it cuts (where the whole-cell stock bounds still hold that
  material) then plunges a few stock cells back, where the bounds let the
  rapid come lower, and feeds across at depth. A helical entry ramps only
  through material: above the height the committed cuts or the stock cell
  bounds prove clear over its whole bore it descends straight down, and a
  bore that is clear to the layer gets a straight plunge (reported as
  `air_entries`). `ramp_clearance_mm` (opt-in, on `Roughing`) starts each
  helix that far above the material it meets, never above `feed_z_mm`.
  `link_feed_mm_min` (opt-in, on `Roughing`) feeds the stay-down links at
  that rate wherever the committed stock's whole-cell upper bounds prove the
  cutter's footprint clear of stock (checked in stretches of at most 2 mm);
  any stretch that may touch stock keeps `feed_mm_min`. Reported per layer as
  `fast_links` and `fast_link_mm`. `link_chip_thinning` (opt-in, needs
  `link_feed_mm_min`) also speeds up stretches that may touch stock lightly:
  a tooth at angle t from the feed direction cuts a chip fz·cos(t) and meets
  material only at lateral offset r·sin(t) from the line, so from the
  smallest lateral offset s of any stock cell the cutter can reach whose
  upper bound is above the link depth (cell extent included), no chip is
  thicker than that of a side cut with ae = r − s. The stretch feeds at
  `feed_mm_min` · f(engagement) / f(r − s), with f(ae) = 2·√(ae/D − (ae/D)²)
  (Sandvik Coromant, radial chip thinning), rounded down to 10 mm/min and
  capped at `link_feed_mm_min`, so no chip is thicker than at `feed_mm_min`
  and the clearing's radial engagement. Reported as `thinned_links` and
  `thinned_link_mm`.

Default entry is `StockEdgeEntry()`. `retract_z_mm`/`feed_z_mm` work for raster,
offset and engagement_front clearing. `RestRoughing` machines only what earlier operations left
(`rest_tolerance_mm`, 0.01, is ignored), never repeating strokes proven already
cleared. Further options:

- `omit_empty_cuts=True` leaves out passes whose whole sweep is proven to cut
  only air (reported).
- `require_part_at_first_depth=True` refuses a first layer above the top of the
  part (face the stock first).
- `clearing_tolerance_mm` sets the allowed construction error of each layer's
  clearing region; the operation fails if it cannot be kept.
- `stepup_mm` adds step-up layers: after each full `stepdown_mm` layer, layers
  at most `stepup_mm` apart climb back up through that layer's depth, each
  clearing only a band next to the part (the layer's shadow grown by three
  times the cutter radius plus radial allowance, which contains every point
  the full layer could not reach). The stair a full stepdown leaves on sloped
  and stepped walls shrinks to one step-up. Each step-up layer is reported
  with `step_up_of_z_mm`, the full layer it climbs from. Must be positive and
  smaller than `stepdown_mm`.

Total roughing depth may exceed the flute length: each deeper layer is
prepared for its actual increment below the preceding layer, with a reserve
for conservative stock bounds. Above the flutes, every emitted move must keep
the declared shank and holder clear of committed stock. A move is admitted
when one of these proves each section above the flutes clear over the whole
move:

- whole-cell stock bounds (nothing in reach stands as high as the section);
- the cutter sweeps already committed. The section's sweep, everything within
  its radius of the move's path, must lie inside the union of the sweeps cut
  at or below the section's bottom, or outside the stock. A straight-shank
  end mill (shank as wide as the cutter) is therefore clear wherever the
  layers above cut: on a path that repeats theirs, exactly, with no margin
  (a line may run along several collinear earlier moves; an arc must be the
  same arc), and anywhere between their passes, as on a drafted or stepped
  wall where each layer's passes lie inside those above. Apart from exact
  repeats, the section is grown by the move's path error and every sweep is
  shrunk by its own (both of the order of the stock sweep's `tolerance_mm`);
  no other allowance is made, and nothing is inferred from samples.

A near-repeat is still refused when it cannot be proven by those certificates
(#228). A deeper pass offset by even one nanometre from an earlier path is
not that path: with a shank as wide as the cutter, its outer edge can extend
past the earlier nominal sweep. An offset smaller than the two paths' combined
error does not supply physical clearance. No tolerance is added to admit it,
and the planner does not silently snap or change the requested path.
`engagement::cover::tests::a_path_on_several_collinear_sweeps_is_proven_without_margin`
pins this limitation: R6 cuts at Z5 on Y = `0.1 + 0.2`, with a section on the
deeper Z1 pass whose bottom is Z5.1. The exact repeated line passes; the same
section offset by one ulp or +/-0.000001 mm is an expected refusal. Resolving
such a job needs a proven path/assembly change, not acceptance of unproven
clearance.

The same proof decides the stock sweep's cutting-length check and the
stock-contact verification of the shank, so a move the planner admits is not
flagged afterwards. A move whose shank or holder would enter stock that no
layer above removed is refused with a typed reach refusal naming the layer,
the tool part, the stock it would meet and the tip height. A wider shank or
low holder can therefore still prevent reaching the requested floor. In the
terms of Autodesk Fusion's shaft and holder modes (Fusion help, "Shaft and
Holder Modes reference": separate shaft and holder clearances; "Fail on
collision" aborts the toolpath when the safe distance is violated), this is
fail-on-collision with no added clearance, judged against the stock as the
run has cut it: the tool body may pass exactly where the cutter has been, and
nowhere else.

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

`Tabs(width_mm, height_mm, positions_mm=[(x, y), ...])` puts one tab at each
given point instead, for example to keep tabs off a feature or to hold a part
where it must stay held. This is the "at points" tab positioning of commercial
CAM (Fusion 2D Contour, "Tab Positioning: At points"; the tag positions of the
FreeCAD CAM Tag dressup). Positions are XY points on the part boundary, in the
region's coordinates; arc-length parameters are not accepted, because they
depend on where a loop starts and which way it is cut, and a point does not.

- A point is taken as the nearest point of the boundary when it lies within
  `position_tolerance_mm` (default 0.01) of it. A point farther from every
  selected boundary is refused with its distance; it is never moved further.
  A point within the tolerance of two loops is refused as ambiguous.
- Each loop gets the tabs whose points lie on it. **A loop with no point gets
  no tab**: a hole's slug or a part whose loop has no tab is cut free. The
  operation's report lists every such loop in `diagnostics`.
- The tab is centred where the tool is nearest to the point; a point on a
  convex corner centres the tab on the corner.
- The width, height and shape rules are those of evenly spaced tabs, and so is
  the room a tab needs: tabs whose centres are closer along the tool path than
  the tab width plus the tool diameter (plus both ramps for triangular tabs)
  overlap and are refused, as is a tab that close to the loop start, where the
  tool enters and leaves. Both refusals name the points.
- `positions_mm` cannot be combined with `count` or `spacing_mm`.

`entry=RampEntry(angle_deg)` replaces the plunge at the start of each pass by
one continuous ramp around each loop, from the stock top to the floor (see
[Ramp entry](#ramp-entry-rampentry)), at no more than the angle and no more
than `stepdown_mm` per lap, followed by one lap at the floor. Not combined with
tabs or with limits that block part of a loop (both refused before any
motion).

### A mesh part's outline: `Engine.silhouette_region`

`Contour` takes a planar region. For a part given as a `SurfaceMesh`,
`camlib.Engine().silhouette_region(surface, z_mm, max_error_mm=...)` returns
its **silhouette at a depth** as such a region: the plan-view shadow of
everything at or above `z_mm`. That is the area a flat end mill with its tip
at `z_mm` must stay beside, since the cutter stands over its tip; it is not
the cross-section at `z_mm` (an overhang above the depth is in it).

```python
with camlib.Engine() as engine:
    out = engine.silhouette_region(mesh, z_mm=5, max_error_mm=0.0025)
region = camlib.PlanarRegion.from_dict(out["region"])
wall = camlib.Contour("wall-z5", tool, region, floor_z_mm=5, stepdown_mm=2, clearance_z_mm=15, ...)
```

- At or below the part's lowest point it is the whole part's outline, the
  "silhouette" selection of commercial CAM (Autodesk Fusion: "the boundary of
  the part when viewed from the Z axis of the work coordinate system", also
  for mesh bodies; FreeCAD CAM's Profile of a whole model uses the same
  projected outline). One `Contour` then cuts around the part at every depth.
- Where the outline changes with depth (steps, drafted walls, bosses), make
  one `Contour` per depth, each with that depth's silhouette and floor. A
  silhouette contains the silhouettes of all higher depths, so every pass of
  such a contour, from the stock top down to its floor, stays beside the part.
- To *finish* walls that change with depth, use [`Waterline`](#waterline-finishing-waterline)
  with a flat end mill: its level contours are offsets of the part's shadow
  above each level (the constant-Z "Contour" finishing of commercial CAM).
  `silhouette_region` is for the 2D operations and what only they have: wear
  compensation, tabs, leads, or the outline as a pocket or limit region.

**What it guarantees.** The region is conservative. Facets at or above
`z_mm − surface_error_mm − ` a numerical reserve are projected (the report's
`protected_from_z_mm`), facets seen edge-on (vertical walls) are enclosed
rather than dropped, and the pieces are joined by the exact planar kernel. The
region's `boundary_error_mm` (also in the report) states how far the intended
surface at or above `z_mm` may lie outside the region: the mesh's declared
error, the numerical reserve and the enclosure, about 0.0008 mm more than
`surface_error_mm` at the default `tolerance_mm`. `Contour` adds a region's
error to its clearance and refuses a region whose error its
`boundary_tolerance_mm` cannot hold. Its edges are straight (the mesh's
facets); the outline is as fine as the mesh.

A horizontal face within that reserve of `z_mm` is at the depth, so it is in
the silhouette: for the outline of a boss standing on a face at Z3, ask a
little above the face (`z_mm=3.01` with a 0.001 mm mesh) and cut the face
itself with `Flat` or a pocket.

**Refused:** a silhouette whose error exceeds `max_error_mm` (the message
gives its parts; supply a finer mesh), a depth with nothing at or above it,
an invalid mesh, and a mesh whose facets' projections touch within the
construction tolerance without sharing vertices (not watertight), as for
`Roughing`. `max_error_mm` has no default.

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
  `tap_cycle_left` (the FANUC 0i-MB example with `rigid_tapping=True`, or
  `rigid_tapping_m29=True`, which writes `M29 S…` before each cycle).
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
If the shared undercut-neck check refuses a move, its diagnostic gives the
neck radius, stock cell half-diagonal, path error and conservative clearance
lower bound at the failing cell. A positive cell-centre clearance also gives
the strict square-cell size threshold at that centre. This is a local
resolution hint, not the exact wall clearance or a guarantee for a new grid:
rerun from the original stock at the finer resolution and check every move.
If the current centre has no positive clearance bound, no local cell-size
threshold is given. New stock bounds may still resolve that uncertain cell.
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
engraving with a chamfer tool, edge breaks, slots along a path), or runs
beside it with `side_offset` (below). Depth passes
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

### Cutter on one side of an open path: `side_offset`

`side_offset=TraceSideOffset(side, radial_allowance_mm=0)` keeps a flat end
mill beside each path instead of on it: the tool centre runs at the cutter
radius plus the allowance to the `"left"` or `"right"` of the direction of
travel. This is a contour along an open chain of edges (an open edge, a step),
which `Contour` cannot take because it needs a closed region. It is the
sideways compensation of commercial CAM: Autodesk Fusion's Trace offsets "to
the left and right of the selected contours" as its 2D Contour does ("Trace
milling strategy" and "Side Compensation with Trace reference" in the Fusion
help), and FreeCAD CAM's Profile cuts beside open edges, stopping at their
ends.

```python
from camlib import Trace, TracePath, TraceLine, TraceArc, TraceSideOffset

edge = TracePath((6, 10), (TraceLine((20, 10)), TraceLine((20, 18)), TraceArc((24, 18), -90)))
op = Trace("step", tool, (edge,), top_z_mm=10, depth_z_mm=8.5,
           stepdown_mm=0.5, clearance_z_mm=12, feed_mm_min=400, plunge_mm_min=100,
           spindle_rpm=12000, coolant="off", tool_number=1, length_register=1,
           side_offset=TraceSideOffset("left", radial_allowance_mm=0.2))
```

- The offset path is exact: lines move parallel, arcs keep their centre, a
  joint that turns away from the cutter is rounded by an arc about it (the
  cutter rolls around the corner, staying in contact), and a joint that turns
  towards the cutter is trimmed where the neighbouring offsets meet. Arcs stay
  arcs.
- The path starts and ends on the normals through the path's two ends: the
  cutter does not wrap around them. Extend the path if the cut should run past
  an end.
- With a clockwise spindle (M3), `"left"` is climb milling and `"right"`
  conventional, as with G41 and G42.
- Refused, with the path and segment named, rather than cut differently: an
  arc curving towards the cutter that is not larger than the offset; a segment
  the cutter cannot reach at this offset (a notch or corner tighter than the
  cutter); a path that folds back on itself or passes closer to itself than
  the offset; a closed path (use `Contour`); a cutter that is not a flat end
  mill; a negative allowance. Use a smaller tool or allowance, or split the
  path.
- The run's report gives `side` and `side_offset_mm` (radius plus allowance).
  Without `side_offset` the operation and its serialisation are unchanged.
- Controller wear compensation (G41/G42) is not available on Trace; `Contour`
  has it for closed profiles.

## Undercut: `Undercut`

Required: `paths`, `tip_z_mm`, `entry_mm`, `clearance_z_mm`; optional
`exit_mm`. An undercut cutter (`TSlotCutter`, `KeyseatCutter`,
`LollipopCutter`, `DovetailCutter`) follows tool-centre paths (`TracePath`) at
one tip height, under material left by earlier operations: the wide bottom of
a T-slot under a slot milled first, a keyseat in a wall, a dovetail on a slot
or pocket wall. As established practice (Machinery's Handbook, milling
cutters; cutter makers' application guides), the cutter goes down to depth
in open space (`entry_mm`: outside the part, or a cleared pocket), cuts
sideways and goes back up in open space (`exit_mm`, or back along each open
path to its start and the entry). It never plunges into material.

```python
from camlib import TracePath, TraceLine, TSlotCutter, Undercut, DexelStockSpec

cutter = TSlotCutter("tslot", diameter=10, width=2, neck_diameter=3,
                     neck_length=10, overall_length=50, shank_diameter=6)
op = Undercut("t-slot", tool, (TracePath((-8, 10), (TraceLine((38, 10)),)),),
              tip_z_mm=3, entry_mm=(-8, 10), exit_mm=(38, 10), clearance_z_mm=20,
              feed_mm_min=300, plunge_mm_min=100, spindle_rpm=6000,
              coolant="flood", tool_number=2, length_register=2)
```

Undercut operations, and lollipop and dovetail cutters in any operation,
need tri-dexel stock (`DexelStockSpec`); a height-field setup refuses them.
The plan proves, on the run's height view, that the columns where the tool
goes down and up are open for the whole assembly, and refuses otherwise. The
tri-dexel sweep refuses a move whose neck, shank or holder may touch
material (a neck wider than the slot above). Compare the result with the
part with `DexelStockState.compare_solid` (a closed mesh of the part); the
stream's material stages compare the height view, which cannot hold an
undercut.

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

Required: `surface` (the finished part as a `SurfaceMesh`), `levels`, the
break size, `clearance_z_mm`, and a chamfer mill (`ChamferMill`), ball-end
mill (`BallEndMill`) or bull-nose mill (`BullEndMill`). Breaks the horizontal
edges of a finished part (every boundary of every upward-facing planar face)
and, given the part's convex `edges`, sloped edges with a ball-end mill,
accounting for every convex edge.
`camlib.cadquery.deburr_levels(solid)` makes the levels (the faces at each
height, exported exactly):

```python
from camlib import Deburr
from camlib.cadquery import deburr_levels, export_surface_mesh

op = Deburr("deburr", c90, export_surface_mesh(part), deburr_levels(part), 0.5, 30,
            feed_mm_min=600, plunge_mm_min=200, spindle_rpm=10000,
            tool_number=2, length_register=2, coolant="flood")
```

- The tool axis runs on the outward offset of each face boundary, so the
  break is the tool's own profile seen across the edge, whatever the edge's
  curvature. A face point s inside the boundary is at least the offset plus
  s from the path (exactly that on a convex stretch).
- **Chamfer mill:** the cone breaks the edge `width_mm` wide on the face and
  the chamfer depth down a vertical wall (`wall_depth_mm` in the report:
  equal for 90 degrees). The axis is `tip_clearance_mm` (0.5) beyond the
  tip's flat, which keeps the point out of the cut.
- **Ball-end or bull-nose mill:** the break is a concave arc of the tool's
  corner radius rc (`break_profile` "arc"). Across the edge, the corner's
  circle has its centre c outward of the wall and c above the face, on the
  edge's outward bisector. The tool tip is then rc - c below the face and
  the axis r - rc + c outward of the boundary (r the tool radius). Because
  c >= 0, only the corner arc reaches the part: the flat bottom stays
  outside the wall and the body above the corner stays above the face. Give
  the size in one of two ways:
  - `width_mm` L, the leg: equal legs L on the face and down a vertical
    wall, c = (sqrt(2 rc^2 - L^2) - L) / 2. L is at most rc.
  - `break_radius_mm` rho (with `width_mm=None`): the largest break that
    stays inside a fillet of radius rho. The corner circle touches the
    fillet's circle, c = rc / sqrt(2) - rho (1 - 1 / sqrt(2)), and rho is at
    most rc (1 + sqrt(2)). The legs are then L = sqrt(rc^2 - c^2) - c, less
    than rho.

  The report gives the legs (`face_width_mm`, `wall_depth_mm`), the axis
  offset (`axis_offset_mm`) and `fillet_radius_mm`: the smallest fillet
  radius whose fillet contains the break. Checking the stock against the
  part filleted with that radius shows no gouge. The break's lower edge
  must be at least `tip_clearance_mm` out from the end of the tool's flat
  (c >= `tip_clearance_mm`, the ball's centre line for a ball), so the
  point does not cut. A larger break is refused.
- **Drafted walls** (chamfer mill): a planar wall hanging off the boundary
  at draft beta below alpha, the cone's half angle, is found in the part
  mesh. Each of its points is depth tan(beta) out from the boundary. The
  cone ends the break on it at depth h = width / (tan alpha - tan beta),
  the slant leg h / cos beta. The level's axis moves out by h tan(beta) for
  the steepest such wall, so the break's lower edge there keeps the tip
  clearance and the width on the face stays exact (`level_details`:
  `axis_offset_mm`, `tip_z_mm`, `max_wall_draft_deg`). The proof excludes
  such a wall only down to that depth. The cone reaches a point at depth d
  only if d (tan alpha - tan beta) < width (plus the proof's margins), so
  nothing deeper is cut. A wall drafted too steeply for the cone, at or near
  its half angle or beyond its radius, stays an obstacle, and the stretch is
  reported with that reason. With a ball-end or bull-nose mill, drafted
  walls stay obstacles and are reported (use a chamfer mill).
- **Fillets:** `deburr_levels` lists the boundary edges where a face meets
  a fillet tangentially (`DeburrLevel.broken_edges`, from OpenCascade's
  `BRepOffset_Analyse`). There is no sharp edge there, so the stretch is
  reported as already broken (`already_broken_length_mm`), not unreachable.
- Nothing else is touched, by an exact proof for the whole tool (cone or
  corner arc, neck, shank and holder) against the part above the face,
  below the break and anything in the break's height band that is not the
  face, a vertical wall under its boundary or (to the depth above) a drafted
  wall hanging off it. The arc's contact is a
  concave maximisation like the cone's. It is bounded without a slope
  bound, because a full quarter arc is vertical at its rim, and tested
  against dense sampling. The proof allows for the mesh's `surface_error_mm`
  and two stock cell diagonals.
- A stretch that cannot be proven is not cut. The tool retracts and
  re-enters where cutting can be proven again (entries and exits are proven
  too). Each stretch is reported in `skipped`: concave stretches, where a
  wall rises from the face, in `concave_length_mm` (nothing to break); convex
  edges the tool cannot reach in `unreachable_length_mm`.
- The stock must already be machined to the part. A non-center-cutting
  tool enters only where the stock is proven clear under its tip.
- **Every convex edge** (`edges`, from `camlib.cadquery.deburr_edges`, which
  lists each convex edge as horizontal, vertical or sloped with its faces'
  normals; convexity from OpenCascade's `BRepOffset_Analyse`): the report's
  `edges` accounts for each one as `cut`, `partly cut` or `skipped`, with
  the reasons for what was not cut. A horizontal edge on a level's boundary
  takes the level's result.
- **Sloped edges** (and horizontal edges under a sloped face), when straight
  between planar faces, are broken with a ball-end mill. The ball's centre
  runs on the edge moved s out along the outward bisector b of the faces'
  normals (cos g = b . n), which leaves a cylindrical cove with equal legs
  L = sqrt(R^2 - s^2 cos^2 g) - s sin g along both faces, whatever the
  slope. `width_mm` gives L; `break_radius_mm` rho gives s = R + rho -
  rho / cos g, the cove touching a rho fillet. For a 90 degree edge these
  are the horizontal formulas. The proof is against the part with the two
  faces' strips along the edge replaced by the cove itself (radius R plus
  the proof's margins), so the whole assembly may touch nothing else. That
  matters for the shank on a steep edge. Near the edge's ends, where the
  ball would reach the next face, the path stays unproven and is reported.
- Skipped with a reason: vertical edges (breaking one needs the tool's side
  run along it), edges with a face facing down (they need a tilted tool,
  #5/#6), curved edges that are not on a level, and edges off the levels
  with a chamfer or bull-nose mill.

## Parallel finishing: `Parallel`

Required: `surface` (`SurfaceMesh`), `floor_z_mm`, `stepover_mm`,
`clearance_z_mm`. Ball end mill raster along `axis` following the surface.
`path_error_mm` (0.005) bounds how far the path may deviate between computed
points; `safety_margin_mm` (0.005) keeps the tool that far above the exact
no-gouge height; `stock_to_leave_mm` leaves material. In a direct comparison
with OpenCamLib's drop-cutter on the same mesh, camlib's tip stayed between
+0.0046 and +0.0099 mm above the exact no-gouge height.

Pass direction: rows run along `axis`, stepping from the low to the high end
of the other axis. By default they alternate (zigzag: the first row from the
low end of `axis`, or from the high end with `reverse=True`, the next one
back). `one_way=True` cuts every row in the same direction (the first row's):
after each row the tool leaves the stock, retracts, returns over the part at
`retract_z_mm` with a rapid and enters the next row from the same side. The
return is a link like any other (never a feed move over the surface) and is
checked as one: stock contact, fixtures and machine limits. It applies to
slope-limited and rest finishing rows too, and is refused for the other
patterns, which have their own direction. The rows are zigzag's rows, each
held to the same path error, safety margin and clearance proof (either way a
row lies between the exact no-gouge height and `safety_margin_mm +
path_error_mm` above it), so the surface left is zigzag's within that
tolerance; only the direction and the longer links differ. Other CAM systems
offer the same choice for parallel passes: Autodesk Fusion's Parallel strategy has a Direction parameter "to create only
climb cuts or only conventional cuts"
([Generate a Parallel toolpath](https://help.autodesk.com/cloudhelp/ENU/Fusion-CAM/files/3D-PARALLEL-STEPS.htm)),
and FreeCAD CAM's 3D Surface operation separates the `Line` cut pattern (one
direction) from `ZigZag`
([Surface.py](https://github.com/FreeCAD/FreeCAD/blob/920d14763241267340e755f3d3b6ece8ef60fa43/src/Mod/CAM/Path/Op/Surface.py)).
With one-way rows the uncut material is always on the same side of the tool.
The spindle turns clockwise (M3), so material on the right of the travel is
climb milling and on the left conventional: `axis="x"` cuts conventional and
`axis="y"` climb, and `reverse=True` swaps each. (On a slope the ball also
cuts uphill or downhill; that depends on the surface, not on this option.)
The report has `one_way: true` when it is set.

Depth: when the remaining stock stands no higher above `floor_z_mm` than the
cutting length, every pass is within the flutes. Where it stands higher (a
part taller than the flutes, after roughing), each feed move is checked
instead: the neck, shank and holder sections stay above the committed stock
under them, so only a pass that really cuts deeper than the flutes is refused
with a typed reach refusal naming the section, the obstruction's top and the
tip Z (the message also names where). The check is conservative to whole
stock cells: a cell counts when it stands above a section's bottom (at the
move's lowest tip) and any point of it may lie within the section's radius of
the tool axis anywhere on the move. `steep_and_shallow` can then take the
part's lowest point as its floor. Waterline is admitted the same way.

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

### Geodesic: `Geodesic`

`Geodesic(id, tool, surface, floor_z_mm, stepover_mm, clearance_z_mm, seed)`
cuts passes whose contact paths are `stepover_mm` apart measured along the
part's surface (contact mode, as Cimatron and ModuleWorks define geodesic
contact paths), so the stepover, and with it the scallop, stays even on
steep and shallow areas alike. `seed`:

- `"boundary"`: passes work inward from the drive surface's boundary;
- `("point", (x, y))`: rings around the surface point there, the first half
  a stepover out;
- `("curve", ((x, y), ...))`: the curve's surface points are the first pass,
  the others follow it along the surface.

How it is computed:

- **The drive surface** is the target mesh's upward-facing triangles: the
  faces a three-axis ball can touch, welded into one connected mesh. Its
  boundary (open edges, and edges against walls or downward faces) is
  explicit: passes end there, never rolling over an edge.
- **The distance** is the exact geodesic distance on that mesh (window
  propagation: Mitchell, Mount and Papadimitriou 1987; Surazhsky et al.
  2005), from polygonal sources (Bommes and Kobbelt 2007). Within a face it
  is the minimum of point-source and line-source distances, each valid in
  its own region, so iso-curves are found exactly; a straight chord through
  the air is never used.
- **The passes** are the exact iso-curves at `(k + 1/2) x stepover` from a
  point seed (`k x stepover` from a boundary or curve seed), as polylines
  within a quarter of `path_error_mm` of the curve (the cutter path's
  smoothing takes another quarter: each pass within half the path error of
  its level, so neighbouring passes are the stepover apart within the path
  error). Where an iso-curve turns back
  inside one triangle (it rounds a corner where the nearest source changes)
  the triangle is split until each piece is a simple arc.
- **Coverage** is certified, not estimated: every point of the drive surface
  is proven to lie within half a stepover (plus the path error) of a pass by
  the exact distance from the passes themselves (branch and bound over each
  face's distance functions). Where the levels leave a point farther than
  that (a peak between levels, the sharp corners of offsets from a
  boundary, a ridge), a fill pass is added there, and fills along one ridge
  are chained into a single pass along it (the medial axis of a pocket-like
  region: corner bisectors and a centre line).
- **Cutter positions**: ball centre = contact + R x the surface normal (the
  vertex normals interpolated, so a faceted mesh does not shake the path).
  Where the normal jumps (a convex crease or corner) the ball rolls round the
  point: its centre sweeps the arc between the two normals. Along a crease
  the pass runs on, the normal cone's bisector is used; along the drive
  surface's boundary, the boundary face's own normal. Positions are lifted
  onto the part with the certified ball contact; where the ball cannot touch
  a stretch of a pass (a concave crease narrower than the ball) the pass is
  trimmed exactly there and bridged straight only across a gap no wider than
  the ball, otherwise split.
- **Linking** stays down wherever the dropped ball touches the drive surface
  all along the link: a tangent lane change to the next ring (entered two
  gaps on, a 1:2 slope), else a straight link of at most two stepovers, else
  along the boundary pass or back along the ring just cut; only otherwise a
  retract.

Cancellation is checked while constructing a level, including subdivision
inside one facet, path chaining and simplification. A cancelled contour returns
an error; its partial paths cannot become a successful plan.

Verification, independent of the distance field, in the operation report
(`geodesic`):

- `spacing_mm` [min, mean, max]: the geodesic spacing between neighbouring
  passes' achieved contacts, computed afresh by dropping the ball along the
  cutter path as cut (at most a quarter stepover apart, not only at its
  vertices), from each pass's contact chains to the next.
- `planned_coverage_mm`: the certified bound above.
- `unreachable_samples`: pass vertices whose intended contact the ball
  cannot touch (the certified drop lifts it above that contact by more than
  the mesh's surface error): a concave crease narrower than the ball. No
  stepover is owed where no position touches, so they are left out of the
  spacing and counted, not hidden; `bridged_spans` counts the trimmed gaps
  bridged.
- `off_drive_samples`: achieved contacts on a wall or the base.
- `over_stepover_samples` and `flat_scallop_estimate_mm`, a flat-surface
  estimate of the scallop at the largest spacing. Convex areas leave more,
  concave areas less, and creases are not modelled. The material check is
  authoritative for residual stock.

Nothing is rescaled to fit. Limits, so that what it cuts is clean:

- **Slope.** The drive surface is the faces within `slope_from_deg` ..
  `slope_to_deg`, and for `Geodesic` `slope_to_deg` defaults to 80 (not 90):
  where the ball touches near its equator (the foot of a fillet into a
  wall), a hair of sideways movement moves it far up or down, and the cutter
  would chatter along the wall. Steeper areas and walls belong to
  `Waterline` (or steep-and-shallow). The passes are planned inside the
  range, not cut afterwards.
- **Mesh.** The surface mesh must be a clean tessellation: triangles thinner
  than a thousandth of its surface error (a tessellator's fan of slivers at a
  sphere's pole, say) are refused with their location, since no distance
  across them is reliable. Re-export or repair the mesh (for a sphere, put
  its pole off the surface).
- **Concave creases** narrower than the ball (a sharp internal corner) cannot
  be reached in contact mode: the passes are trimmed there and bridged
  straight (reported as `unreachable_samples` and `bridged_spans`). For such
  parts use `mode="tool_centre"`, which runs smoothly over them.

`mode="tool_centre"` (explicit; the default is `"contact"`) measures the
stepover along the surface the ball's centre moves on instead of the part's
surface (Cimatron's "tool center", ModuleWorks' geodesic tool-centre paths).
That surface is the certified drop's ball-centre height, sampled a quarter
stepover apart wherever the resting ball touches the drive surface as
contact mode lets it (rolling over the edges between its faces, never off
its boundary), its edge found exactly by bisection, and triangulated; the
same exact distance, certified coverage, fills and links run on it, and the
passes are the cutter path itself. It runs smoothly over internal corners
the ball cannot reach into (a concave crease narrower than the ball), where
contact mode leaves a band. Equal centre spacing is not equal contact
spacing: it differs by (rho +- R) / rho with the curvature (closer contacts
on convex areas, merging in a crease, jumping across it). The report gives
`mode`, `centre_spacing_mm` (measured on that surface, from each level's
passes to the next) and `spacing_mm` (the contact spacing it produces).

To see how a plan was built, set `CAMLIB_GEODESIC_TRACE` to a file path (or
`-` for standard error): it records the planning phases and their times,
each coverage round and the fills it adds, positions that cannot touch their
contact and why, links that retract, and verification samples narrower than
the stepover. It is off by default and costs nothing then.

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

`Parallel`, `Spiral`, `Radial`, `Project`, `Pencil`, `Scallop` and `Geodesic` take
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

That outline is the part's shadow at or above the level, the same conservative
region 3D roughing protects its layers with and
[`silhouette_region`](#a-mesh-parts-outline-enginesilhouette_region) returns: facets are
taken from the level less `surface_error_mm` and a numerical reserve
(33 × `tolerance_mm`), facets seen edge-on (vertical walls) are enclosed in a
box of 32 × `tolerance_mm` rather than dropped, and a mesh whose facets the
planar kernel cannot join is refused. The contour is offset from it by the
cutter radius, `stock_to_leave_mm`, `safety_margin_mm` and the shadow's stated
error (its `boundary_error_mm`: the mesh's `surface_error_mm` plus at most
about 80 × `tolerance_mm`, under 0.001 mm at the default tolerance); the
report's `shadow_error_mm` is the largest such error over the levels cut.

`simplify` (default on) removes contour points that are not needed within
`contour_error_mm`; `reverse` cuts the contours the other way;
`safety_margin_mm` and `stock_to_leave_mm` work as for `Parallel`.

`ramp=True` (ramp finishing) cuts each level as one continuous descent along
its contour from the level above, instead of a flat lap and a step down; the
lowest level is then cut once more flat. Each ramp follows a contour proven
clear at its own level and stays above that level, so it cannot gouge.

Depth: when the remaining stock stands no higher above `bottom_z_mm` than the
cutting length, every level is within the flutes. Where it stands higher (a
part taller than the flutes, finished from roughed stock, e.g. a mesh stock
from `StockSpec.from_mesh`), each feed move is checked instead, as for
`Parallel`: the levels run top-down, so the levels above have already left
the finished wall (plus stock to leave) beside the tool, and the neck, shank
and holder sections must stay above the committed stock under them. A part
that narrows upward (drafted walls, a cone or dome) is then finished to its
foot with a short tool; beside an undrafted wall taller than the flutes, or
from unroughed stock, the shank or holder would rub, and the first such move
is refused with a typed reach refusal naming the section and the level's Z.
The report's `reach_proof` says when this check is in force. `contour_error_mm` (0.01) bounds the contour
approximation as a *normal* (3D) distance to the surface. Z levels are computed
in parallel. `contour_method="quad_tree"` (default) scales to dense meshes;
`"support"` builds exact per-facet polygons and suits small or convex meshes.
On a 51,200-triangle dome, contours agreed with OpenCamLib's waterline and kept
0.003 to 0.010 mm normal clearance to the true surface.

## Supplied toolpath: `SuppliedToolpath`

A toolpath planned somewhere else (another CAM system, a fork of camlib, your
own planner) run **exactly as given**, as one more operation of the job.
Exchanging a toolpath as cutter-location data, apart from the system that
planned it, is standard practice: CLDATA (ISO 4343) and APT CL files carry a
toolpath to the system that checks, simulates and posts it. Here the data is
camlib's own typed motion instead of a text format.

```python
from camlib import SuppliedToolpath, ToolpathMove
from camlib.motion import Arc, Helix, Line

moves = (
    ToolpathMove("rapid", Line((6, 6, 8), (6, 6, 5.5))),
    ToolpathMove("plunge", Line((6, 6, 5.5), (6, 6, 4)), feed_mm_min=150),
    ToolpathMove("cut", Line((6, 6, 4), (12, 6, 4)), feed_mm_min=600),
    ToolpathMove("cut", Arc((12, 6, 4), (12, 10, 4), center=(12, 8, 4), sweep_rad=math.pi), feed_mm_min=500),
    ToolpathMove("retract", Line((12, 10, 4), (12, 10, 8)), feed_mm_min=300),
)
op = SuppliedToolpath("outside", tool, moves, clearance_z_mm=8,
                      spindle_rpm=12000, coolant="flood", tool_number=7, length_register=9)
```

Required: `id`, `tool`, `motions`, `clearance_z_mm`, `spindle_rpm`, `coolant`,
`tool_number`, `length_register`; `tolerance_mm` (0.00001) and `work_offset`
(1) as for every operation. There is no operation feed: each move states its
own.

A `ToolpathMove(kind, curve, feed_mm_min=None)` is the tool tip's move in the
setup's work frame: `kind` is `"rapid"`, `"cut"`, `"plunge"`, `"lead"`,
`"link"` or `"retract"`, and `curve` a `camlib.motion.Line`, `Arc` or `Helix`
(mm; `sweep_rad` in radians, positive counter-clockwise about the plane's
normal). `ToolpathMove.from_motion(m)` takes the move of a
`camlib.motion.Motion` or of a streamed motion dictionary (`Update.motion`),
leaving its id, operation and tool behind.

**The same run.** The moves go through the stream every planned operation
uses, so everything after it is the same code: they are swept through the
stock the operations before left, and the next operation plans from what they
leave; each gets the per-move checks (holder, shank, fixtures, machine limits,
rapids and links in stock); the material check compares the stock with the
target and proves each move against it; export is refused while a finding is
not accepted; and the posted program is decoded back and compared with them.
A helix or arc the machine cannot take is refused by the post, as for a
planned move.

**Nothing is changed.** No re-linking, no smoothing, no feed substitution; a
fed retract stays fed whatever `Setup(rapid_retracts=...)` says. The moves
leave the stream with the same kind, feed and coordinates, bit for bit.

**What camlib adds:** only the travel to the first supplied point, as planned
operations travel to theirs. The operation starts where the one before ended
(or at the tool-change pose), at or above `clearance_z_mm`; from there one
vertical rapid goes to the first point's height and one horizontal rapid to
the point. Either is left out when it has no length. They are ordinary moves
of the operation and are checked like the rest. The report gives
`approach_moves` and `first_supplied_motion_id`: supplied move `i` is the
motion with that id plus `i`, which is how a finding's motion maps back to
your list.

**Refused, with the move's index** (0-based; the message reads
`supplied toolpath <id>: move <i> of <n>: ...`), before any motion:

| Rule | Why |
| --- | --- |
| The kind is one of the six above | Tapping, oriented shifts, threading and probing moves belong to cycles whose events only their own operations write |
| A rapid is a line | A controller's rapid has no arc form |
| Every move but a rapid states a positive, finite `feed_mm_min` | camlib supplies no feed. A rapid may state one (a fed traverse) or none |
| The curve is valid within `tolerance_mm`: finite coordinates within 1e6 mm, a line at least 1e-9 mm long, a circle centre in the start's plane, an arc that stays at its plane height and a helix that leaves it, a radius of at least 1e-6 mm, and an end within `tolerance_mm` of where the centre and sweep arrive | The rule for every motion in a run (the run's `SweepOptions.tolerance_mm` applies to each move as well) |
| Each move starts **exactly** where the one before ends (the same coordinates) | The stream's continuity rule. camlib does not bridge or snap a gap, however small: supply the connecting move |
| The first move starts and the last move ends at or above `clearance_z_mm`, which is above the stock top | Operations start and end at clearance; the toolpath brings its own approach and retract |
| With [machining limits](#machining-limits-avoid-areas-and-containment): every move stays out of the avoid areas below their tops, and every cutting move inside the containment | Limits apply to every operation |

Also refused: a plasma/laser beam or a probe as the tool (a beam needs its
pierce, lead and beam events, which only `ThermalCut` plans), a supplied
toolpath in a lathe setup (turning moves are checked by the turning planner
as it makes them), and a number that is not finite (in Python, before the job
is built).

**Guard rails a planner would have provided:**

- The setup must check stock contact
  (`StreamChecks(stock_contact=StockContactOptions())`), or the run is
  refused: no planner proof stands behind a supplied rapid, link, shank or
  holder.
- A cutter that is not center cutting descends only through air: a fed move
  that goes down is refused, with its index, unless the stock proves nothing
  remains above its lowest point under the whole move. camlib has no
  ramp-angle data for such cutters.
- Spindle speed, coolant, work offset, tool number and length register are
  declared on the operation and written like any other operation's.

camlib does not judge the strategy: chip load, engagement, cutting direction
and whether a cutter suits a move are the supplier's. Configure a scene
(fixtures, machine limits) and a material check against the part to have
those checked; what is not configured is listed as unchecked at export.

**Size.** There is no cap on the number of moves ([decision 12](DECISIONS.md)):
they are limited by memory (checked as the operation is built) and by the
run manifest, which holds them and is at most 4 GiB (about 150 bytes a line
move). They are streamed one at a time. The moves are part of the job's
identity: the operation's input digest and regeneration cache key cover every
coordinate, computed in one pass.

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
after they are cut free (plan travel heights accordingly), bevel and
common-line cutting.

## Plasma and laser marking: `ThermalMark`

Lines traced on the sheet surface with the beam in the machine's marking
process, without cutting through: part numbers, bend lines, layout marks.
Required: the `parts` of the sheet (the cut's `parts`), the `paths` to mark
(`TracePath` values of `TraceLine`/`TraceArc`, as for [`Trace`](#trace--engrave-trace);
camlib has no fonts: text arrives as its strokes), and the marking values of
this machine and material. None of them has a default.

```python
torch = camlib.Torch("torch", kerf_width=1.5)
seven = camlib.TracePath((26, 92), (camlib.TraceLine((34, 92)), camlib.TraceLine((29, 80))))
mark = camlib.ThermalMark("numbers", torch, parts, (seven,), feed_mm_min=4000, mark_height_mm=2.5,
                          power_percent=100, mark_width_mm=0.8, tool_number=1, length_register=1)
cut = camlib.ThermalCut("plasma", torch, parts, ...)
setup = camlib.Setup("sheet", stock=sheet, tools=(torch,), operations=(mark, cut), ...)
```

| Parameter | Meaning |
| --- | --- |
| `feed_mm_min`, `mark_height_mm` | Marking feed, and the nozzle (or focus) height above the sheet top |
| `power_percent` | A laser's marking power (posted as S). A plasma torch fires at the current its controller's marking process sets: give 100 |
| `mark_width_mm` | The width of the marked line on this material: that width must stay inside the part |
| `edge_clearance_mm` | Extra distance kept between a mark's edge and its part's edges (default 0) |
| `allow_off_part` | Marks may lie on the scrap or cross parts (remnant labels, layout lines); default False |
| `height_mode`, `travel_height_mm` | `"fixed"` (default): one Z, the marking height (the controller owns Z). `"program"`: travel between paths at `travel_height_mm` |

What sets a mark's depth and width belongs to the machine ("You can vary the
depth and width of marks by varying torch speed, output current, and
torch-to-workpiece height": Hypertherm Powermax45 SYNC Operator Manual,
*Marking process guidelines*), so camlib takes all of it from the caller.

- **Path**: the beam centre follows each path, with no kerf compensation, one
  beam-on per path, in the order given. A marking move is a line or an XY arc
  at the marking height; its kind is `mark`.
- **Marks run before cuts**, while each part is still held by the sheet and
  has not moved (ProNest sequences scribing with its "Punch/Scribe first"
  setting): put the `ThermalMark` before the `ThermalCut`. A part with a mark
  must not have been cut: read from the stock when the operation is planned,
  no cell at or inside its outline may be cut through. Refused otherwise,
  also with `allow_off_part`.
- **On its part**: every mark's width, with `edge_clearance_mm`, is proven
  inside one part (not on the scrap, in a hole, across an edge or from one
  part to another) before any motion is produced, by the same proof as the
  containment limits. `allow_off_part=True` skips this proof, and the report
  then says `containment: "not_checked"` instead of `"proven"`.
- **Stock**: a marking move removes no material. The stock model itself
  refuses one that passes (with the beam's width, the tool's kerf) over a
  cell that is or may be cut through, or beyond the sheet: in the planned run
  and again when the posted program is replayed. The run still carries every
  marking move (kind `mark`), so an application can draw the marks.
- **Typed motion**: a path's beam is `BeamMark(power_percent)`, turned off
  with `Beam(0)`. Only `mark` moves are fed while it is on; a cutting move
  under it, a `mark` move under the cutting beam, and firing either beam
  while the other is on are all refused, in the plan and in the decoded
  program.
- **Posts**: the machine declares how it marks, or the post refuses the
  program ([Posts](posts.md): `beam_mark`). Declared: generic G-code
  (`beam_mark="power"` or `"mode"`), GRBL laser (`define_machine(...,
  beam_mark="power")`) and [QtPlasmaC](qtplasmac-post.md) (declared marking
  and cutting materials). Centroid CNC12 plasma and WinCNC plasma refuse:
  no marking method is declared for them.

Not modeled: the marked line's depth or appearance, a separate scribe or
marker tool with its own offsets (QtPlasmaC tool 1), spotting (a timed torch
pulse) and powder marking.

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
# and camlib's rectangular holder (or your holder's outline), with its body
# off the centre plane: the shank's height (25 mm below the cutting face, its
# clamp 3 mm above) and the seat's rake and inclination from the holder's
# catalogue page (-6/-6 for a typical negative P-clamp holder: check yours).
insert = camlib.TurningInsert.iso_code("CNMG120408", holder="PCLNR2525M12")  # style L: 95 degrees
holder = camlib.TurningHolder.rectangle(insert=insert, nose_radius_mm=insert.nose_radius_mm, side="outer",
                                        width_mm=25, length_mm=150, offset_mm=0, y_mm=(-25, 3))
tool = camlib.TurningTool("cnmg", insert=insert, side="outer", hand="right", holder=holder,
                          seat=camlib.TurningSeat(rake_deg=-6, inclination_deg=-6))
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
  bore from the front face to `z_end_mm` (arcs as arcs, like `TurnFinish`),
  then out at 45 degrees and along the axis. Posted as `G33 Z… K<lead>` (LinuxCNC)
  or `G32 Z… F<lead>` where the dialect declares `thread_synchronized` and
  `thread_lead_word`; dialects without them refuse it.
- `TurnFinish(allowance_mm)`: the profile from the front face, the nose
  center a nose radius (plus allowance) outside it: rounded at convex
  corners, trimmed at concave ones. Tangent offset segments share one
  represented endpoint, so rounding at their join does not introduce a tiny
  connecting move (#242). A feature too small for the nose is
  refused (a concave arc no larger than the nose radius plus allowance
  included). Arcs of that path are planned as arcs and posted as one G2/G3
  block each (G18): the profile's own arcs, offset by the nose radius (a
  fillet of radius R is followed at R - nose radius, a ball end or convex
  radius at R + nose radius), and the nose rolling round each sharp convex
  corner. The program's point (the imaginary tip) is the nose center moved
  by a fixed offset, so its arcs have the same radius and sweep. An arc move
  is checked like the chords it replaces: in pieces within a quarter of the
  setup tolerance of the arc, each piece's insert, holder and body against
  the part, the chuck and the stock left by the pieces before it, and the
  stock is bounded on both sides of the arc. These stay chords within a
  quarter of the tolerance, as lines: an arc flatter than that over its
  whole length, an arc crossing the end of the cut (the chuck clearance, or
  `z_end_mm` in a bore), and any arc followed with a grooving blade.
  Roughing passes are straight and unchanged. `compensation="wear"` (also on `TurnBoreFinish`) posts the same
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
  - Holders: `TurningHolder(outline, bar=None, y_mm=)`, an outline in the
    same frame, anywhere relative to the insert (a head ahead of it, a
    stepped head), its extent across the centre plane `y_mm=(low, high)`,
    and for a boring bar a `TurningBar(diameter_mm, axis_x_mm, z_mm,
    axis_y_mm)`. `TurningHolder.rectangle(insert=, nose_radius_mm=, side=,
    width_mm=, length_mm=, offset_mm=, y_mm=)` builds camlib's rectangle:
    outside, a shank behind the insert's reach; boring, an axial bar
    `width_mm` across, its top `offset_mm` under the nose center,
    `length_mm` out of the bore.
  - Holders from the maker's model (#110): `TurningHolder.from_model(model)`
    with a `TurningToolModel`; see [Tool bodies from a model](#tool-bodies-from-a-model).
  - The body off the centre plane (#103), required for every tool: a
    point of the tool at x out and y across the plane lies at radius
    √(x² + y²), so in a bore a thick insert's heel reaches past its
    outline. `TurningTool(..., seat=TurningSeat(rake_deg, inclination_deg),
    insert_thickness_mm=, insert_clearance_deg=, edge_deg=,
    curved_for_diameter_mm=, insert_front_clearance_deg=,
    insert_side_clearance_deg=, insert_size_tolerance_mm=)`:
    - `seat`: the holder seat's orthogonal rake and inclination (ISO 3002-1,
      about the major cutting edge), from the holder maker (ISO 5608 codes
      do not carry them; ISO 5610 gives only recommended values).
      `TurningSeat(0, 0)` for a flat seat. Not for a drill.
    - The insert's thickness below its cutting face and its normal clearance
      (its sides taper inward by thickness x tan(clearance)). From
      `TurningInsert.iso_code` they default to the code's worst case: ISO
      1832:1985 §3.6 gives the thickness symbol as the thickness in mm
      "disregarding any decimals" (T1, T3 for 1.98, 3.97 mm), so `04` is
      4.00 to 5.00 mm, plus the tolerance class's ±0.025 or ±0.13 mm
      (§3.3); the clearance letter is §3.2's (N 0, A 3, B 5, C 7, P 11,
      D 15, E 20, F 25, G 30 degrees; O must be declared). Declare the
      insert's catalogue thickness to tighten it.
    - `insert_front_clearance_deg`, `insert_side_clearance_deg`: a grooving
      blade's (`GrooveInsert`, `FaceGrooveInsert`) own clearance below its
      front edge and below its sides, e.g. 10 degrees at the front and 3 at
      the sides. ISO 1832 gives an indexable insert one normal clearance,
      so every other insert takes `insert_clearance_deg` alone (per-flank
      angles on one are refused); a grooving blade is not an ISO 1832
      shape, and its maker lists its relief per face (Kaiser Tool Company's
      THINBIT catalogue pages "O.D./I.D. Grooving Inserts" and "Parting
      Inserts" give "side clearance" and "back clearance" per insert, "both
      sides"), and a face grooving insert's two sides may differ (US
      10,780,505 B2, Sandvik: its two side clearance surfaces, "the second
      clearance angle is larger than the first"). Declared as one angle,
      such a blade must take its least, and a groove its front clears is
      refused. Give the two together, from the catalogue, instead of
      `insert_clearance_deg`: the side value is one angle for both sides or
      a pair, for a `GrooveInsert` (the side at its leading corner, toward
      -Z for a right-hand tool; the other side), for a `FaceGrooveInsert`
      (the inner side, toward the axis; the outer side). Each flank then
      recedes below the face at its own angle; the rounded corners between
      them recede at the least of the three (`insert_clearance_deg` is then
      that least: left out, or given as exactly it; another value is
      refused as a contradiction), so the checked body contains the real
      one whatever blend is ground there. Nothing is assumed: a blade
      declared with one angle is checked with that angle on every side, as
      before. A refusal names the flank whose body reaches the material
      (`the blade's front flank, 3 degrees of clearance`, `outer side
      flank`, `side flank toward -Z`). Where it matters: the front in an
      inside groove (below the centre plane it reaches √(x² + y²) toward
      the groove's bottom), the outer side of a straight face grooving
      blade at the groove's outer wall (it needs sin(2 x clearance) ≥
      thickness / wall radius), the sides of a blade on an inclined seat.
    - `edge_deg`: the major edge's direction (the rake's axis), known for
      ISO, grooving, threading and face grooving inserts; declared for a
      drawn `InsertOutline`.
    - `curved_for_diameter_mm`: a face grooving blade curved to follow the
      groove, its maker's (min, max) diameter range; every groove it cuts
      must lie in it. Without it the blade is checked as straight.
    - A drill on the axis is a solid of revolution: its outline is its exact
      section, so it takes no seat or insert body (its holder's `y_mm`
      still).

    The checks use the body's convex pieces: their XZ projection against
    material outside the tool's radius (outside turning: a point's radius
    is at least its |x|), and the radial image (z, √(x² + y²)) of their
    vertices against material beyond it (in a bore; the radius is convex, so
    exact at the vertices and never under-reporting). The insert's body
    must also reach the stock left by a cut no farther than its cutting
    face does: a heel rubbing the wall just cut is refused.
  - The insert's in-plane size tolerance (#113). An insert is not exactly
    its nominal size: its ISO 1832 tolerance class (the code's third
    letter) bounds its inscribed circle d and its corner dimension m, as
    well as its thickness. `TurningInsert.iso_code` reads the class and
    the tool is then checked as **the largest insert the tolerance allows,
    relative to its measured nose**; `insert_size_tolerance_mm=(d, m)` on
    the `TurningTool` (or `size_tolerance_mm=` on `iso_code`) states the ±
    tolerances instead (the insert maker's values; `(0, 0)` for an insert
    measured to be exactly its outline).
    - What d and m are (ISO 1832:2017, 5.3, figures 1 and 2): d is the
      diameter of the inscribed circle; m, for an insert with an odd number
      of sides (triangle), is the distance from a corner's tip to the side
      opposite it, and for an even number of sides (square, rhombic, the
      80 degree hexagon W) the distance along the corner's bisector from
      the inscribed circle to the corner's tip. The class does not
      tolerance the corner radius or the corner angles: the nose radius is
      the tool's declared value, and the sides keep their directions.
    - The values (ISO 1832:2017 tables 3, 4 and 5, ± mm on d / m):
      classes A 0.025 / 0.005, F 0.013 / 0.005, C 0.025 / 0.013,
      H 0.013 / 0.013, E and G 0.025 / 0.025; J, K, L: d by size, m 0.005,
      0.013, 0.025; M and N: d and m by size; U: d and m by size. By
      inscribed circle, for shapes S, T, C, W (table 4; d for J to N, d
      for U, m for M and N, m for U): 4.76 to 10 mm 0.05, 0.08, 0.08,
      0.13; 12.7 mm 0.08, 0.13, 0.13, 0.2; 15.875 and 19.05 mm 0.1, 0.18,
      0.15, 0.27; 25.4 mm 0.13, 0.25, 0.18, 0.38; 31.75 mm 0.15, 0.25,
      0.2, 0.38. For classes M and N of shape D (table 5; d, m): 5.56 to
      9.525 mm 0.05, 0.11; 12.7 mm 0.08, 0.15; 15.875 and 19.05 mm 0.1,
      0.18; of shape V: 6.35 to 9.525 mm 0.05, 0.16; 12.7 mm 0.08, 0.25.
      Where the standard gives no value (D and V in classes J, K, L and U;
      a size not in its table), the code is refused, naming
      `size_tolerance_mm`: camlib does not guess one.
    - Why the tolerance is not a growth of the outline. The tool is
      measured at its nose (presetting, touch-off): the program moves the
      nose's imaginary tip (or its centre, `controlled_point`), and the
      operator's offsets put the real nose there whatever the insert's
      size. The seat locates the insert by its flanks, so the two cutting
      edges at the nose keep their lines. Relative to the measured nose,
      the nose arc and the two edge lines are therefore fixed, and a
      larger insert reaches farther back along those lines: only the far
      sides and far corners move. The cut (the nose, the edges, the
      material they remove, the posted program) is the same as without the
      tolerance; a finishing pass with the edge along the profile is
      accepted as before.
    - How m is measured, and what that leaves open. ISO 1832's figures
      draw an insert of perfect form, whose inscribed circle touches every
      side. ISO 883:1985 annex B (triangular and square inserts) gives the
      method: a triangular insert lies on a surface plate on the side
      opposite the corner, and a dial gauge reads the corner's tip (B.1);
      a square insert rests in a 90 degree vee-block on the two sides of
      the corner opposite, and the dial gauge, zeroed on a precision roller
      of the nominal d lying in the same vee, reads the corner's tip
      (B.2). For rhombic and hexagonal inserts no such annex was read. On
      a real insert, then, figure 2's circle can be the insert's own
      largest circle or a nominal roller against the two opposite sides,
      and the two give different bounds. The envelope must never be
      smaller than the real insert, so it is the **union of every
      reading**, for every shape; it may be tightened only with a source
      that settles the measurement for the shape in question.
    - The envelope (derivation and tests in `src/turning/size.rs`). With u
      the nose's bisector into the insert, ε its corner angle, a = n·u for
      a far side of outward normal n and c = b·u for a far corner of
      bisector b, three readings are taken:
      - (A) Size: a true-to-form insert of inscribed circle d ± Δd is the
        nominal one scaled about the point where the edge lines meet, so
        the side moves out Δd (a / sin(ε/2) + 1)/2 (Δd for a rhombic or
        square insert's far sides, whose distance from the edge lines is
        d; 1.5 Δd for a triangle's). The scaled inserts are nested: their
        union is the largest.
      - (B) m from the insert's own inscribed circle (for the triangle:
        from the opposite side, exactly ISO 883 B.1). The nose is a corner
        m checks and is where the tool is measured, so a nose standing Δm
        farther from its datum puts the insert's body Δm farther back.
        Even number of sides (the far sides tangent to the circle): the
        side moves out up to |a| Δm + (1 + a) Δd/2, and a far corner like
        the nose has its tip up to (|c| + 1) Δm + (1 + c) Δd/2 beyond
        nominal (never beyond the sharp corner of its sides). Triangle:
        the far side moves out up to Δm, and each far corner's tip,
        measured from the fixed edge line opposite it, up to Δm.
      - (C) m by vee-block and roller (ISO 883 B.2; applied to rhombic and
        hexagonal inserts by analogy, a vee of the opposite corner's
        angle). Each far side has its own displacement (a rhombic insert's
        two widths need not be equal: d moves a far side, and m at the
        corners moves it further). The inscribed circle is at least
        d - Δd, and at every corner like the nose the dial reads within
        ±Δm: the corner's tip less the roller's top, each moving with the
        two sides it touches. These conditions are linear in the
        displacements, and the largest displacement of each side and tip
        is found exactly. For a rhombic or square insert: one far side up
        to 2 Δm sin(ε/2) + Δd out (the other Δd in), the far corner's tip
        up to Δm.
      - The union: each far side is moved out by the largest of the three;
        far corners unlike the nose keep their rounding between the moved
        sides; a far corner like the nose is the sharp corner of its moved
        sides cut off square at the farthest its tip can be under any
        reading (m locates the tip, not the corner's form). For a CNMG1204
        of class M (d ±0.08, m ±0.13): far sides (A) 0.08, (B) 0.149, (C)
        0.247 mm, so 0.247; the far 80 degree corner's tip (A) 0.124, (B)
        0.34, (C) 0.13 mm, so 0.34. The tests sample inserts admissible
        under each reading, built from the shape's angles and the
        definitions above alone, and compare: every one lies inside the
        envelope, and each far side and checked corner of the envelope is
        reached by one.
      - Not bounded by d and m, and so assumed: the sides keep their
        directions (the class does not tolerance the corner angles; a side
        that tilts about an obtuse corner of a rhombic insert, which m
        does not check, has no bound from the designation), and the
        corner radius is the code's.
    - What uses it. The insert's body (its taper, thickness and seat as
      above) is built on the envelope and checked against the part, the
      bore, the chuck and the stock; a refusal that the nominal insert
      would not have caused says so and names the tolerance. An insert at
      the small end of its tolerance has its far corner nearer the nose:
      the bands the far sides and corners can lie in, between the least
      and the largest insert, must be clear of the stock where they lead
      (the depth of cut must end short of where the least insert's cutting
      edge ends). The stock model follows: material is certainly cut only
      by what every insert of the tolerance has (the nose, and the two
      edges at it as far as the least insert's reach), and may be cut
      wherever the largest insert's sides sweep.
    - Covered: outlines of the ISO form (straight sides each tangent to
      the inscribed circle, rounded corners) with three, four or six
      sides: C, D, V, S, T and W from `iso_code`, and a drawn
      `InsertOutline` of that form (the 75 and 86 degree rhombic E and M,
      the hexagon H) with a declared tolerance. Not covered, and refused
      by name when a tolerance is declared for them: the pentagon (P) and
      the octagon (O), outlines with a side that is not tangent to the
      inscribed circle (parallelograms A, B, K, rectangles L, wiper edges,
      `TurningInsert.iso`'s wedge), and round inserts (R): a round
      insert's d is its nose diameter, so its tolerance is a nose radius
      error, which changes the cut itself and is not a matter of the far
      side of the body; `iso_code` gives a round insert no size tolerance.
      A drawn outline, a grooving, face grooving or threading blade and a
      drill have no class: they are checked as declared, so state the
      largest outline (width, diameter) the tool can have.
    - Sources: the tolerance tables and the figures defining m, ISO
      1832:2017 5.3 as shown in the standard's public preview
      (standards.iteh.ai, "ISO 1832:2017" sample; the ISO 1832:1991
      preview, 4.3, gives the same values, without shape V); the method
      of measuring m, ISO 883:1985 annex B with its figures 2 and 3, as
      shown in that standard's public preview (standards.iteh.ai, "ISO
      883:1985" sample); the same
      class table and class M values by shape and inscribed circle in
      Mitsubishi Materials' technical information "3. Symbol for
      Tolerance Class" (turning insert identification); the same class
      table in Kyocera's general catalogue, section B "Turning Indexable
      Inserts Identification System".
  - Solid (one-piece) boring bars (#114): a solid carbide boring bar, a
    brazed tool or a ground HSS bar has no insert and no seat; its tip is
    ground into the end of a round bar. Declared as an insert body (a prism
    of the head's outline as deep as the head, clearance 0) it is sound but
    keeps its full-width corners at full depth, and is refused in the small
    bores these tools are made for. Declare the head instead:
    `TurningTool(id, insert=InsertOutline(...), side="inner", hand=,
    holder=, solid_bar=SolidBarHead(diameter_mm=, axis_x_mm=, axis_y_mm=,
    clearance_deg=, min_bore_diameter_mm=, max_bore_depth_mm=))`, with no
    seat and no insert thickness or clearance. `insert` is the head's
    outline at its cutting face (convex: the head only; its nose arc, the
    side and end cutting edges in `cutting`), and the holder is what lies
    behind the head (the plain bar as a `TurningBar`, the sleeve as the
    outline).
    - The body below the cutting face is the cylinder of `diameter_mm`
      about the axis (`axis_x_mm` in the outline's frame, `axis_y_mm`
      across the centre plane, the cutting face at 0), cut by the cutting
      face and by the ground face below each segment of the outline, which
      recedes under the face at that segment's `clearance_deg` (each angle
      declares that the whole head lies behind that plane; a rounded
      corner's angle applies to every tangent plane of the corner, so it
      must be at most that of the two straight sides it joins, or 0, and
      anything else is refused). It is
      convex, so the checks above apply unchanged. They need vertices, so
      the cylinder is checked as the prism of a polygon circumscribed about
      its circle in 2 degree steps (every side tangent, the corners
      1 / cos 1° = 1.00015 radii out: it contains the cylinder and stands
      at most 0.000076 x the diameter outside it), like the round bar of a
      holder and the nose arcs of every insert.
    - Catalogue values (KYOCERA, "EZ Bar: Series for Small Internal
      Machining" catalogue, 2022: p. 12 "EZ Bar Dimensions" tabulates DMIN,
      DCON, H, LF, LU, WF, WF2 and RE per bar, e.g. EZBR 055050ST-008H:
      5.5, 5.0, 4.80, 58.2, 25.0, 2.44, 0.33, 0.08 mm; p. 8 "Bar
      Tolerances" gives the offset (WF), the cutting edge height (Y, e.g.
      "+0.06 / 0 mm") and "Min. Bore Dia.: Same as Shank Dia." for the HP
      bars, "Different from Shank Dia." for the ST bars):

      | Catalogue | Declared as |
      |---|---|
      | Minimum bore diameter (DMIN) | `min_bore_diameter_mm` |
      | Usable length, maximum bore depth (LU) | `max_bore_depth_mm`, from the head's front end |
      | Offset from the bar's centreline to the cutting edge (WF, f) | `axis_x_mm` = the edge's x in the outline + WF |
      | Cutting edge height above the bar's centre (Y) | `axis_y_mm` = -Y (its tolerance's worst case) |
      | Shank diameter (DCON) | the holder's `TurningBar`; `diameter_mm` where the head keeps the bar's surface |
      | Corner radius (RE) | the outline's corner arc |

      The tables give neither the head's outline nor the clearance angles
      of its ground faces (the drawings show them, e.g. a 5 degree end
      cutting edge angle): they are declared, from the tool's drawing or by
      measuring. An angle that is not known is declared 0, and a head whose
      section is not known is declared as the catalogue's promise itself:
      `diameter_mm` = DMIN, the axis DMIN / 2 from the cutting edge at the
      cutting face's height, every clearance 0 (the head enters a bore of
      DMIN with its edge on the wall, so it lies in that cylinder). Both
      contain the real head.
    - The catalogue's premises are held besides the geometry: a cutting
      move that leaves a bore below `min_bore_diameter_mm` where the corner
      cuts, and any move with the head's front end more than
      `max_bore_depth_mm` behind the front of the stock, are refused. A
      declaration that contradicts itself is refused too: an outline that
      reaches off the bar's section at the cutting face, and a minimum
      bore the declared head cannot enter with its edge on the wall (the
      message gives the least bore it does enter). A body point `a` behind
      the edge and `y` below the face enters a bore of diameter
      (a² + y²) / a, so a face receding at the angle α clears the bore's
      curvature only down to (bore radius) x sin 2α below the face: the
      rest must be the bar's own round or a steeper face.
    - The polygon stands outside the true cylinder by up to 0.000076 x its
      diameter: a head declared exactly tangent to the bore it cuts (no
      relief, the edge on the bar's surface, a bore of the bar's own
      diameter) is planned only with a `tolerance_mm` of at least that.
    - Boring only (`TurnBoreRough`, `TurnBoreFinish`, and any operation
      that takes a boring tool with an `InsertOutline`): solid grooving,
      face grooving and threading bars are refused as solid heads; declare
      such a blade as a `GrooveInsert`, `FaceGrooveInsert` or `ThreadInsert`
      with its insert body.
  - ISO values: shapes and corner angles from ISO 1832:1991 section 4.1
    table 1; size codes (inscribed circle per shape) and corner radius codes
    from the Sumitomo Electric Hardmetal general catalog, section B
    "Indexable Insert Identification Code", tables 5 and 7, p. B3, which
    reproduce ISO 1832's designations (edge length from the inscribed circle
    by each shape's geometry, matching the tabulated lengths); holder styles
    (cutting edge angle) from ISO 5608:1995 section 4.3 table 3. Round
    sizes 12 and 25 each name two diameters and are refused (give an
    outline).
- Checks on every move: the swept insert body against the part (a
  tolerance of touching allowed), so a profile the insert's trailing edge
  cannot follow is refused; drills and boring tools inside the `bore` on both sides of the
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

#### Tool bodies from a model

A grooving blade's carrier, a modified boring bar, a holder with a clamp or
a coolant block that sticks out: no outline or code describes them, and
their maker's 3D model is the body to check. `TurningToolModel` is that
body; the tool takes it as its holder (`TurningHolder.from_model(model)`),
and every check that used the holder outline's body (against the part, its
bore, the chuck and the stock left after each cut) uses the model's.

```python
import camlib
from camlib.cadquery import step_solids, tool_body_from_step

# Which solids of the file are the insert? (order of step_solids(path))
model = tool_body_from_step(
    "holder.step",
    max_error_mm=0.005,
    placement=camlib.TurningModelPlacement(
        nose_center_mm=(100, 50, 20),   # the model point of the nose centre
        x_axis=(0, -1, 0),              # the lathe's +X in the model
        z_axis=(1, 0, 0),               # the lathe's +Z in the model
    ),
    axial_resolution_mm=0.25,
    insert_solids=(2,),
    corner_region_mm=1.0,
    position_tolerance_mm=0.02,
    nose_radius_tolerance_mm=0.02,
    edge_angle_tolerance_deg=0.25,
)
tool = camlib.TurningTool("groove2", insert=camlib.GrooveInsert(2, 22), nose_radius_mm=0.2,
                          side="outer", hand="right",
                          holder=camlib.TurningHolder.from_model(model),
                          seat=camlib.TurningSeat(0, 0), insert_thickness_mm=3,
                          insert_clearance_deg=7)
tool.validate()   # the meshes, the placement and the insert cross-check
```

- **The engine takes a mesh, never STEP.** `TurningToolModel(body=,
  placement=, axial_resolution_mm=, insert_check=)`: `body` is a
  `SurfaceMesh` (the input type of parts) in the model's own coordinates.
  `camlib.cadquery.tool_body_from_step` (and `tool_body_from_shape` for
  CadQuery solids) reads the file with CadQuery/OpenCascade and tessellates
  each solid within `max_error_mm`; a mesh from any other kernel works the
  same way.
- **What the model is.** `body` is the tool away from the cut: holder, bar,
  shim, clamp, coolant block, without the insert. The insert stays the
  declared one (ISO code, outline or blade, with its thickness, clearance
  and seat): its exact outline is what cuts, and a mesh known only to its
  error cannot certify a contact at the cutting edge more closely than that
  error. Insert material left in `body` meets the stock at the edge and is
  refused there ("the holder would hit the stock"): name the insert's
  solids in `insert_solids`. An insert fused with its holder in one solid
  cannot be told apart; split the solid in CAD.
- **Placing a model.** A maker's model has its origin and axes anywhere, so
  the placement is declared and nothing is assumed or fitted:
  `TurningModelPlacement(nose_center_mm=, x_axis=, z_axis=)` gives the
  model point that is the nose centre on the cutting face (a drill: its tip
  on the axis) and the model's unit directions of the lathe's +X (radial,
  away from the spindle axis on the tool's side) and +Z, as the tool is
  mounted. Y completes the right-handed frame (Y = Z x X): the cutting face
  looks toward +Y. A frame given at the theoretical tip (the sharp corner
  the offsets are touched off with) is one nose radius from the nose centre
  along X and along Z. A left-hand or boring tool is placed as it really
  sits: no mirroring is applied to a model.
- **The cross-check.** A model placed wrongly is checked in the wrong
  place. Where the model has the insert (`insert_solids`, or
  `TurningModelInsertCheck(insert=mesh, ...)`), its cutting corner must
  agree with the declared insert's body, both measured the same way within
  `corner_region_mm` of the nose centre along X and Z (at least two nose
  radii; small enough that only the insert is in it), on the declared
  insert's cutting face as its seat tilts it (so the model must be drawn
  on its seat, and the comparison does not depend on the insert's thickness
  or clearance below the face):
  - the nose radius, from three supports across the nose arc, within
    `nose_radius_tolerance_mm`;
  - the direction of each straight edge at the corner, from the model's
    reach across the edge at two stations along it, within
    `edge_angle_tolerance_deg`;
  - the position of the corner, from its reach in every direction in which
    the declared corner is outermost, along the cutting face and raised 45
    degrees from it (the centre height), within `position_tolerance_mm`.

  A model outside one is refused before any move, with what was measured
  ("its model's cutting corner is 0.5000 mm beyond the declared insert's
  toward (X ..., Y ..., Z ...)"): it is misplaced or a different insert,
  and camlib does not align it. The tolerances have no defaults and must
  cover the insert mesh's own error. A model without its insert cannot be
  compared: `without_insert_reason=` (`TurningModelWithoutInsert(reason=)`)
  states where its placement comes from, and the check is then the
  caller's.
- **The errors a model adds**, all on the safe side (the checked body
  contains the real one; a tight fit may be refused, never passed):
  - `surface_error_mm`: every point of the real surface lies within it of
    the mesh. The body is grown by a cube of that half-size, so it reaches
    up to that much farther along X, Y and Z (up to √3 times it at a
    corner).
  - `axial_resolution_mm`: the body is checked in slices across Z of this
    width. In a bore each slice is as large across as the body gets
    anywhere in it (its convex section, which loses nothing: only the
    largest radius can meet a bore wall); outside, each slice is the convex
    hull of its projection on XZ. The body is over-estimated along Z by
    less than one slice (inside), or by the hollows of its underside within
    one slice (outside). A neck, a relief behind the head or a block far
    along the bar stay as they are: the model is not replaced by its hull.
  - The mesh must be a closed surface. Faces tessellated separately need
    not share vertices: an edge with no face on its other side within
    twice the mesh's error is refused as open; camlib does not repair it.
  - The stated error is the caller's: `export_surface_mesh` bounds it for
    planar and analytic faces and refuses surfaces it cannot bound.
- The checks run on the slices, not the triangles. A model is reduced once
  (and kept: a run asks for a tool's body at every operation); consecutive
  slices of one section are one prism, so a straight bar is a single piece
  however many triangles run along it. A move's work then grows with the
  prisms' vertices: the facets round each distinct section. A coarser
  `axial_resolution_mm` or tessellation gives fewer.

Milling holders have the same need. A holder turns with the spindle, so the
body to check is what its model sweeps about the tool axis:
`camlib.tools.holder_sections_from_mesh(mesh, tip_mm=, axis=, stickout_mm=,
gauge_length_mm=, axial_resolution_mm=)` returns the `HolderSection`
cylinders of a `ToolAssembly` (per slice along the axis, the largest radius
the model reaches, the mesh's error added), from a mesh such as
`camlib.cadquery.step_mesh(path, max_error_mm=)` with its declared
placement (the model point of the tool tip and the unit direction of the
axis toward the spindle). A model that starts below the declared stickout,
does not span the range, or is open is refused.

A lathe setup posts (`job.post_nc`, `job.post`) with any `Machine` whose `planes` include `"xz"`: arcs
are written in G18. `GCodeDialect(x_diameter=True)` writes X as a diameter
(arc I stays a radius, as on FANUC and Centroid lathes); for LinuxCNC declare
`lathe_radius_mode="G8"` (radius X) with `with_commands`.
Arc direction follows the coordinates, not the picture: camlib's x is the
radius on the tool's side of the axis, and an arc turning from +Z toward +X
is `G3`, the other way `G2` (counterclockwise and clockwise seen from +Y of
a right-handed X, Y, Z: LinuxCNC G-code reference, G2/G3; FANUC operator's
manual, circular interpolation in the ZpXp plane). A front-turret machine
drawn with X downward shows the same arcs mirrored, with the same words.
Premise: the machine's X+ points from the spindle axis toward the tool; a
machine cutting at negative X is not modelled. Finishing a profile with arcs
therefore needs a machine with `arcs=True` and exactly writable arc words,
or an explicit `lowering=` choice ([Arcs that cannot be written
exactly](posts.md#arcs-that-cannot-be-written-exactly)):
`LoweringOptions(native_arcs=True)` keeps the writable arcs and turns the
rest into checked line segments, `LoweringOptions()` every arc. A chord of
an arc round a convex feature passes inside the arc, toward the part, by up
to `chord_tolerance_mm` (0.0025), and the posted program's replay checks
the insert against the part with the setup's `tolerance_mm`: keep the chord
tolerance below it, or the lowered program is refused as cutting into the
part. The [Centroid CNC12 lathe post](centroid-lathe-post.md)
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

## Probing and tool setting

camlib plans, simulates and posts probing cycles; it never reads their
results back from a machine (it never talks to one). Each operation is one
cycle with a touch probe, `camlib.ProbeTool(id, ball_diameter=,
stem_diameter=, stylus_length=, body_diameter=, body_length=)` (no defaults:
the ball is the calibrated effective diameter; the stem is checked as the
shank and the body as a holder section), or, for tool setting, with the
cutting tool being measured:

| Operation | Measures | Stores |
| --- | --- | --- |
| `ProbeSurface(start_mm, axis, surface_mm)` | One surface square to X, Y or Z, from `start_mm` along that axis and back (Z from above) | That axis of `update_work_offset` |
| `ProbeOutsideCorner(corner_mm, z_mm, material, inset_mm, clearance_mm)` | The Y face, then the X face, each `inset_mm` along it from the corner and from `clearance_mm` away, over the clearance plane | X and Y |
| `ProbeInsideCorner(corner_mm, z_mm, material, clearance_mm)` | The Y wall, then the X wall, from one start `clearance_mm` from both | X and Y |
| `ProbeBore(center_mm, z_mm, diameter_mm)` | +X, -X, +Y, -Y from the centre | X and Y (the centre) |
| `ProbeBoss(center_mm, z_mm, diameter_mm, clearance_mm)` | +X, -X, +Y, -Y from outside, over the clearance plane | X and Y (the centre) |
| `ProbePocket(center_mm, z_mm, axis, width_mm)` | + and - across X or Y from the centre | That axis (the centre) |
| `ProbeWeb(center_mm, z_mm, axis, width_mm, clearance_mm)` | + and - across X or Y from outside | That axis (the centre) |
| `ToolSetLength(contact_mm, clearance_mm)` | The tool's tip down onto a tool setter's contact face | The tool's length register |
| `ToolSetDiameter(setter_center_mm, setter_top_z_mm, setter_diameter_mm, depth_mm, clearance_mm)` | The cutter's side against the setter's round face from +X and from -X | The tool table radius |

`material` gives the side of each face where the material lies (+1 or -1
per axis). `z_mm` is the probe tip: the probe's controlled point is the
bottom of the stylus ball, so an XY touch happens one ball radius above it.
Every operation takes `clearance_z_mm` (rapids reach the cycle over it; a
stylus cycle's protected retract height), `search_mm`, `measure_feed_mm_min`,
`return_feed_mm_min`, `tool_number`, `length_register` and `work_offset`.
Stylus cycles also take `position_feed_mm_min` and `update_work_offset` (the
offset corrected: new offset = active offset + measured error, in the axes
the feature measures; Renishaw Inspection Plus `S`, LinuxCNC `G10 L2`). Tool
setting takes `spindle_rpm` (0: stopped) and `spindle_direction`: camlib
assumes neither.

```python
probe = camlib.ProbeTool("probe", ball_diameter=4, stem_diameter=3, stylus_length=50,
                         body_diameter=40, body_length=60)
corner = camlib.ProbeOutsideCorner(
    "datum", probe, corner_mm=(0, 0), z_mm=5, material=(1, 1), inset_mm=5, clearance_mm=5,
    clearance_z_mm=20, search_mm=2, position_feed_mm_min=1000, measure_feed_mm_min=100,
    return_feed_mm_min=1000, tool_number=20, length_register=20, update_work_offset=1)
```

The moves of a cycle:

- **Protected positioning** (`probe_position`): every stylus move below the
  clearance plane that is not measuring stops the machine if the stylus
  triggers (LinuxCNC `G38.3` and a trip check; Inspection Plus `O9810`).
- **Measuring** (`probe_measure`, then `probe_touch`): the surface is expected
  within `search_mm` of its nominal position. The measuring move is planned up
  to `search_mm` before the expected contact, where nothing may be touched (the
  whole assembly is checked against stock and scene bodies like a rapid), then
  over the last `search_mm` to the expected contact, where only the stylus ball
  (or the measured cutter) may touch: the stem and body stay checked. The
  controller searches `search_mm` beyond the expected contact.
- **Back** (`probe_return`) to where the measuring move started, at
  `return_feed_mm_min`; the ball is not checked (it starts touching).

Probing moves never remove simulated stock, and their whole assembly is
checked for stock contact without the removal certificates a cutting move
earns. Avoid areas are honoured like rapids. A probe never cuts: any other
operation with a probe tool is refused, as is a stylus cycle with a cutting
tool, a running spindle under a probe, a measuring move no longer than its
search, and a probing height at or above the clearance plane.

What a cycle stores (the operation report's `stores`) is a premise for the
rest of the program, recorded in it; camlib cannot check it. Also premises:
the probe's calibration (ball size, the trigger point as the length), the
controller's stopping distance after a trigger within the stylus overtravel,
the features existing within `search_mm` of nominal, and no G5x rotation. The
tool setter is declared in the setup's work coordinates (declare its body as
a scene fixture to check the moves around it), so a program refuses a tool
length set after it probed its own work offset. `ToolSetDiameter` stores the
full radius, not wear: camlib's wear compensation on that tool afterwards is
refused.

Posts write cycles through the dialect's declared probing back end
([Postprocessors](posts.md#probing)): LinuxCNC `G38.x`, or Renishaw Inspection
Plus macros on FANUC. The decoder reads every move back from the written
blocks, and each is compared with the plan.

Layout references: LinuxCNC G-code reference (G38.n, G10 L1/L2/L10,
numbered parameters, `docs/src/gcode` in LinuxCNC 2.9); Renishaw *Inspection
Plus software programming manual* H-2000-6031 and H-2000-6222 (chapters 5 and
7: O9810, O9811, O9812, O9814, O9815; chapter 9 feedrates). Probing on tilted
work planes waits for #2 and #5.

## Tools

`camlib.tools`: `EndMill`, `BallEndMill`, `BullEndMill(corner_radius=...)`,
`Drill(point_angle=118)`, `ChamferMill(angle=90, tip_diameter=0)`,
`ThreadMill(diameter=, tooth_height=, neck_diameter=, neck_length=)`,
`Tap(drill_diameter=, thread_length=, shank_diameter=)`,
`TSlotCutter(diameter=, width=, neck_diameter=, neck_length=)` and
`KeyseatCutter` (a cutting disk on a narrower neck),
`LollipopCutter(diameter=, neck_diameter=, neck_length=, cutting_length=None)`
(a sphere on a neck), `DovetailCutter(diameter=, angle=, cutting_length=)`
(widest at the tip, the flank `angle` degrees from the tip face),
`BoringBar(diameter=, bar_diameter=, reach=, insert_length=1)`,
`Torch(kerf_width=, nozzle_diameter=6, body_diameter=30)` and
`LaserHead(kerf_width=, nozzle_diameter=3, body_diameter=25)` and
`ProbeTool(ball_diameter=, stem_diameter=, stylus_length=, body_diameter=,
body_length=)` (complete assemblies). The cutter helpers take
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

4/5-axis motion. Supplied toolpaths for turning and for plasma/laser cutting
(`SuppliedToolpath` is three-axis milling). Surface finishing needs surfaces reachable from above (a
part seen as a height map). Undercuts are cut with `Undercut` on tri-dexel
stock only; lollipop finishing along an undercut edge (a surface strategy)
is not supported yet.
