# Nesting sheet parts: `Nest`

`camlib.Nest` places copies of parts on a sheet for plasma or laser cutting and
returns their placements and the placed parts as one `PlanarRegion`, which is
the `parts` of a [`ThermalCut`](operations.md#plasma-and-laser-cutting-thermalcut).
Nesting is a planning step on geometry only: it produces no motion, and the
same request always gives the same nest.

Nothing is assumed about the machine or the job. The caller declares the sheet,
the kerf and leads of the cut, the spacing between parts, the margin to the
sheet's edge, and for every part its quantity and the rotations it may take.

```python
import dataclasses
import camlib
from camlib.planar import Boundary, Component, PlanarRegion

sheet = PlanarRegion("sheet", (Component(Boundary.polygon(((0, 0), (1250, 0), (1250, 625), (0, 625)))),))
torch = camlib.Torch("torch", kerf_width=1.5)
cut = camlib.ThermalCut("plasma", torch, None, feed_mm_min=2500, cut_height_mm=1.5,
                        lead_in=camlib.Lead(length_mm=2, radius_mm=3),
                        lead_out=camlib.Lead(length_mm=1, radius_mm=2),
                        pierce_delay_s=0.5, power_percent=100, tool_number=1, length_register=1)

nest = camlib.Nest.for_cut(
    cut, sheet,
    parts=(camlib.NestPart(bracket, quantity=12, rotations_deg=(0, 90, 180, 270)),
           camlib.NestPart(washer, quantity=30, rotations_deg=(0,))),
    part_spacing_mm=3, edge_margin_mm=10, resolution_mm=1)
result = nest.run()                       # refused if a part does not fit
print(result.utilisation, result.check["min_part_distance_mm"])
cut = dataclasses.replace(cut, parts=result.region)   # cut the nest
```

## Inputs

| Field | Meaning |
|---|---|
| `sheet` | A `PlanarRegion` of one outline; its holes are areas already cut out of a remnant |
| `parts` | `NestPart(region, quantity, rotations_deg, mirror=False, priority=0, fill=False)`: a `PlanarRegion` of one outline with holes, the number of copies, the allowed rotations (degrees counter-clockwise; `(0,)` keeps the part as drawn), and whether it may also be turned over. Parts of a higher `priority` are placed first (nearest the start corner). `fill=True` makes a filler: placed after every other part, as many copies as fit up to `quantity`, with no refusal for the rest (the result's `filled` lists `(part, copies)`). `kit="name"` keeps parts together: parts with the same kit are placed one after another, all of them or none (a kit that does not fit whole is left out whole: a refusal, or listed as unplaced with `allow_unplaced`, so on several sheets it moves to the next one whole). Kits take their place among the other parts by their highest priority, then their largest part. The result's `order` lists the copies as placed, and `kits` each kit's parts, whether it was placed and the box round its copies |
| `kerf_mm`, `lead_in`, `lead_out`, `pierce_clearance_mm` | What the cut will use. `Nest.for_cut(cut, ...)` reads them, and the tolerances, from a `ThermalCut` |
| `chain` | The cut's `ChainCutting`, if it has one (`Nest.for_cut` reads it). The layout is the same with or without it; `Nest.remnant` gives up half of `max_link_mm` more round every part, since a link cuts through the scrap between two parts' leads |
| `corner_loops` | The cut's `CornerLoops`, if it has them (`Nest.for_cut` reads it): room is kept for the loops as for the leads, `overshoot / tan(min angle / 4)` beyond the kerf path's corner |
| `part_spacing_mm` | Material left between what is cut for two parts |
| `edge_margin_mm` | Material left between what is cut for a part and the sheet's outline or holes |
| `resolution_mm` | Column spacing of the placement (see Limits) |
| `lead_room` | `"all_round"` (default): room for leads is kept all round every part, and the cut may start its loops anywhere. `"at_leads"`: only where each outline will start; the result's `lead_starts` must be given to the cut (see below). `overcut_mm` is the cut's overcut, part of that room (`Nest.for_cut` reads it) |
| `part_in_part` | Parts may be placed in holes of other parts (default `True`) |
| `allow_unplaced` | Return a partial nest listing what did not fit instead of refusing (default `False`) |
| `search_iterations`, `seed` | Exchanges of the placement order to try (default 0) |
| `start_corner` | The corner of the sheet's bounding box the fill starts from: `"lower_left"` (default), `"lower_right"`, `"upper_left"`, `"upper_right"` |
| `fill_axis` | The axis parts are stacked along from the start corner before the nest advances along the other: `"y"` (default) fills a column, then the next column in x; `"x"` fills a row, then the next row in y |

There are no defaults for the sheet, kerf, leads, spacing, margin, resolution,
quantities or rotations.

## What a nest guarantees

A thermal cut runs the beam center half a kerf plus an allowance (the parts'
`boundary_error_mm` and half the cut's `boundary_tolerance_mm`) outside each
part. A lead starts where its loop starts; its pierce point lies the lead's arc
radius along the loop and radius + length away from it, so at most
`hypot(radius, radius + length)` from the loop, and the pierce crater reaches
`pierce_clearance_mm` beyond half a kerf round it. The other lead shapes
(`Lead.line`, `Lead.arc`, `Lead.line_arc`, [operations](operations.md)) reach
no further from the loop than their farthest point: their length for a
straight lead, the far end of the arc or the straight part tangent to it
otherwise. The larger of the lead-in's
and the lead-out's reach is the result's `lead_reach_mm`; with the cut's
`corner_loops` declared it is at least the loops' reach too (a loop's far
side is `overshoot / tan(t / 4)` from the point where the kerf path's edges
meet at a corner turning by `t`, the most at `min_angle_deg`; that point is
`offset / cos(t / 2)` from the drawn corner, the most at `max_angle_deg`).
Everything cut for a
part therefore lies within `kerf + allowance + lead_reach` of the part, and a
nest keeps

- any two parts at least `2 × (kerf + allowance + lead_reach) + part_spacing_mm`
  apart: the kerfs, leads and pierce craters of two parts stay
  `part_spacing_mm` apart wherever the cut starts its loops (see Limits for
  center pierces in small holes);
- every part at least `kerf + allowance + lead_reach + edge_margin_mm` (plus
  the sheet's `boundary_error_mm`) from the sheet's outline and holes.

Both distances are **proven on the finished layout by a check that does not
trust the planner** (`src/nesting/check.rs`). It reads the placed parts as the
lines and arcs that will be cut and

- confirms each placed part is a rigid (or, where declared, mirrored) copy of
  its source, measured in the part's own frame, at the reported placement;
- computes the distance between every two parts, and from every part to the
  sheet's outline and holes, in closed form (line-line, line-arc, arc-arc;
  crossings by exact orientation predicates; nothing is sampled), adding a
  bound on floating-point rounding to each requirement;
- decides containment by winding numbers: a part inside another part's outline
  is accepted only when it lies in one of that part's holes and `part_in_part`
  allows it; a part must lie on the sheet and must not cover a sheet hole.

A layout that fails is refused; it is never returned with a warning.
`result.check` reports the required and the smallest found distances.
`Nest.verify(placements, region)` runs the same check on a layout from anywhere
(an edited or manual nest).

The thermal cut then proves its own leads, kerf paths and pierce craters clear
of every part, as it does for any `parts`.

### Lead room kept only at the leads

`Nest(lead_room="at_leads")` (default `"all_round"`) keeps room for leads and
pierce craters only where each outline will start, so parts stand a kerf
path apart everywhere else and the sheet takes more of them. Each part gets
one start, on its drawn outline: the middle of its longest straight edge that
is as long as the room is wide, or, for a part with no such edge (a round
one), its first vertex. The room is a rectangle on the edge there:

- along the edge, either way: the larger of `lead_in radius + kerf / 2 +
  pierce_clearance` and `|overcut_mm| + lead_out radius + kerf / 2` (the same
  both ways, so it holds whichever way the loop is cut, turned or mirrored);
- out from the edge: the kerf path's offset plus the larger of
  `lead_in radius + length + kerf / 2 + pierce_clearance` and
  `lead_out radius + length + kerf / 2`.

That holds the lead-in, the pierce crater and the lead-out exactly as the cut
builds them. For the other lead shapes the room is the same with each lead's
own reach: along the edge, the farthest its points lie along it from the
start (half a kerf beyond, or the pierce clearance beyond half a kerf round
the pierce), and out from it the farthest they lie out. With in-line leads
both ways (`Lead.line(length, 0)`), which at a corner start leave along the
previous edge's extension, the lead-out's room is its length and the kerf
path's offset every way from the start. The nest then keeps

- any two parts at least `2 × (kerf + allowance) + part_spacing_mm` apart
  (plus the reach of `corner_loops`, which need room at every corner);
- every room at least `kerf + allowance + part_spacing_mm` from every other
  part, and `part_spacing_mm` from every other room;
- every part `kerf + allowance + edge_margin_mm` and every room
  `edge_margin_mm` from the sheet's outline and holes.

The result's `lead_starts` gives each placed part's start, in the order of
`placements`. **Give them to the cut**:
`ThermalCut(lead_placement=LeadPlacement(keep_off_distance_mm=0,
starts=result.lead_starts))`, with the same leads, overcut and pierce
clearance the nest was planned with (`Nest.for_cut` reads them). The cut then
starts every outline there, proves the leads clear of every part as always,
and refuses rather than move a lead whose room it was not given. Holes keep
their leads inside themselves, so `part_in_part` is not planned with it (a
hole's leads point at the part nested in it): the nest refuses the two
together.

The independent check builds every room again from the request's leads and
the declared start (it does not take the planner's), requires the start to
lie on its part's outline, and proves the three distances above in closed
form; `result.check["lead_room"]` reports the required and the smallest
found. `Nest.verify(placements, region, lead_starts)` checks a layout from
anywhere the same way.

Measured on the test parts (a 70 × 50 mm plate with holes, an 18 mm washer;
kerf 1 mm, 1 mm lead-in with a 1 mm arc, 1 mm spacing), filling a sheet with
as many copies as fit:

| Sheet and parts | Room all round | Room at the leads |
| --- | ---: | ---: |
| 400 × 250 mm, washers | 131 (29.6 %) | 204 (46.1 %) |
| 400 × 250 mm, plates | 18 (41.7 %) | 21 (48.6 %) |
| 200 × 120 mm, one plate then washers | 21 washers (29.4 %) | 34 washers (41.7 %) |

The distance required between two parts falls from 8.48 mm to 3.01 mm. The
gain is largest for small parts, where the lead room was most of the gap.

## Placement

Bottom-left fill, after Burke, Hellier, Kendall and Whitwell, "A new
bottom-left-fill heuristic algorithm for the two-dimensional irregular packing
problem" (Operations Research 54(3), 2006): parts are placed one at a time,
largest area first, each at the lowest free position of the leftmost column
that has one; columns are `resolution_mm` apart. The left is the sheet's
smallest x, the bottom its smallest y.

The lowest free position of a column is exact for the planning outlines (each
part grown by its clearance, arcs enclosed in polygons). The positions where a
part would overlap a placed part are a slice of the two parts' no-fit polygon
(Bennell and Oliveira, "The geometry of nesting problems: a tutorial", European
Journal of Operational Research 184(2), 2008). camlib uses the decomposition
approach described there: both outlines are cut into convex pieces
(trapezoids), each pair of pieces forbids one interval of the column, and the
part goes to the lowest position outside all of them. The area outside the
sheet's outline and inside its holes is handled as obstacles in the same way.

Of a part's allowed rotations (and mirror image) the one that ends furthest
left is taken. Parts fall into holes of placed parts, and into notches of a
remnant, whenever that is the lowest free position.

`search_iterations` exchanges of two parts in the placement order are tried
(the paper's hill climbing): an exchange is kept when fewer parts are left over
or the nest ends further left. The exchanges come from `seed` by a fixed
generator, and the search is bounded by iterations, not time, so it is
reproducible. Rotations of multiples of 90° are exact; other angles use the
platform's sine and cosine, so their last digits may differ between platforms.

## Parts that do not fit

`run()` raises `ValueError` naming every part that does not fit and how many
copies. With `allow_unplaced=True` it returns the partial nest and lists them
in `result.unplaced` as `(part, quantity)`. No part is dropped silently: placed
and unplaced copies always add up to the declared quantities.

## Result

| Field | Meaning |
|---|---|
| `placements` | `NestPlacement(part, copy, rotation_deg, mirrored, translation_mm)`: a point `p` of the part moves to `R(rotation_deg) · M(p) + translation_mm`, where `M` mirrors x when `mirrored` |
| `region` | The placed parts, one component per placement in the same order: the `parts` of the `ThermalCut` |
| `unplaced` | `(part, quantity)` that did not fit |
| `utilisation` | Placed part area over sheet area (`placed_area_mm2`, `sheet_area_mm2`) |
| `used_bounds_mm` | The box round the placed parts |
| `lead_reach_mm` | How far leads and pierce craters may reach beyond a kerf's edge |
| `check` | The independent check's report |
| `no_lead` | The parts' edges kept free of leads, as placed (below) |
| `marks`, `mark_placements` | The parts' marks as placed (below) and, for each, the `(placement, mark)` it came from |

## Part labels

`NestPart(..., marks=(...))` declares marks on a part in the part's own
coordinates: `TracePath` lines and arcs (a part number in your own stroke
letters, a bend line, a layout mark). camlib has no lettering of its own, so
the strokes are the caller's. The nest places them with every copy, turned
with it and mirrored with a part that is turned over, and returns them as
`result.marks`: the `paths` of a `ThermalMark` run before the cut, which
proves every mark inside its part at the mark's width and refuses one that
is not.

```python
seven = camlib.TracePath((26, 12), (camlib.TraceLine((34, 12)), camlib.TraceLine((29, 0))))
result = camlib.Nest.for_cut(cut, sheet, (camlib.NestPart(plate, 4, (0, 90), marks=(seven,)),),
                             part_spacing_mm=2, edge_margin_mm=5, resolution_mm=1).run()
marking = camlib.ThermalMark("labels", torch, result.region, result.marks, feed_mm_min=4000,
                             mark_height_mm=2.5, power_percent=15, mark_width_mm=0.8,
                             tool_number=1, length_register=1)
```

`NestPart(..., no_lead=(...))` declares, the same way, the stretches of a
part's outline that must not carry a lead (a seen or welded edge): paths
along the drawn edge in the part's coordinates. They are placed with every
copy and returned as `result.no_lead`, the `keep_off` of the cut's
`LeadPlacement` ([operations](operations.md)):

```python
edge = camlib.TracePath((0, 0), (camlib.TraceLine((70, 0)),))          # the part's bottom edge
result = camlib.Nest.for_cut(cut, sheet, (camlib.NestPart(plate, 4, (0, 90), no_lead=(edge,)),),
                             part_spacing_mm=2, edge_margin_mm=5, resolution_mm=1).run()
cut = dataclasses.replace(cut, parts=result.region,
                          lead_placement=camlib.LeadPlacement(result.no_lead, keep_off_distance_mm=3, prefer="corner"))
```

## Where the nest starts

`start_corner` and `fill_axis` choose which part of the sheet is used first,
and so where the remnant is left. The fill is the same bottom-left fill,
planned in a frame in which the sheet is reflected and/or its axes exchanged
(one of the eight symmetries of the square) and mapped back; a part's
rotation and mirror flag are never changed by it, and the independent check
runs on the sheet itself, as for any layout.

```python
nest = camlib.Nest.for_cut(cut, sheet, parts, part_spacing_mm=2, edge_margin_mm=5, resolution_mm=1,
                           start_corner="upper_right", fill_axis="x")   # rows from the top right
```

## Remnants

`Nest.remnant(result, remnant_margin_mm=...)` gives what is left of the sheet
after the nest is cut, as a `Remnant`: `region` (a `PlanarRegion` in the
sheet's own coordinates), `pieces` (its separate pieces, largest first, each
usable as the `sheet` of the next `Nest`), `area_mm2`, `sheet_area_mm2`,
`cut_reach_mm` and `check`.

It is the sheet less every part's outline grown by everything the cut can
reach beyond it (the kerf, the allowance and the leads' and pierce craters'
reach: the same distance the nest's guarantee uses) and by
`remnant_margin_mm`. The band is taken all round each part, also where no
lead was placed, so the remnant never claims material the cut may have
removed; it can leave out a little that is still there. Its outline is built
to 0.01 mm (curved edges as chords), with the parts grown by eight times
that first, so the coarser edge only gives material up. A part goes with the
slugs of its holes and whatever was nested inside it. `remnant_margin_mm` has
no default: it is what you give up for the heat-affected edge, dross, warp
and how exactly the sheet is put back on the table. The result is checked at
points of every part's outline before it is returned, and
`python/tests/test_nesting.py` checks it against the simulated stock of the
cut (no cell the torch removed lies in the remnant).

```python
left = nest.remnant(result, remnant_margin_mm=3)
camlib.dxf.write_regions(left.pieces[0], "remnant-0412.dxf")     # keep it with the plate

# Next job, the plate back against the same reference corner:
(sheet,) = camlib.dxf.read_parts("remnant-0412.dxf", unit_mm=1, join_tolerance_mm=0.001)
nest = camlib.Nest.for_cut(cut, sheet, parts, part_spacing_mm=2, edge_margin_mm=5, resolution_mm=1)
```

A remnant is the planned one, not a measured one, and it is in the
coordinates of the sheet it was cut from: it is only as good as the plate's
position when it is put back. `Nest.remnant` also takes the `parts` region of
any `ThermalCut` (a hand layout). Parts held by micro-joints count as removed.

With the cut's crop cut ([operations](operations.md), `ThermalCut(crop=...)`),
`Nest.remnant(result, remnant_margin_mm=..., crop=cut.crop)` also removes the
crop cut's kerf, its pierce clearance and the margin to both sides of it, so
the offcut it frees is one of the `pieces`, apart from the skeleton. The
crop cut is checked like the parts: points along it lie that far outside the
remnant (`check["crop"]`).

With the cut's skeleton cut-up (`ThermalCut(skeleton=...)`),
`Nest.remnant(result, remnant_margin_mm=..., skeleton=cut.skeleton)` removes
the whole area it divides, out to where its cuts run off and the margin
beyond: what is cut into pieces to be lifted is no remnant. Give the cut-up
the area the nest used and what lies beyond it stays the remnant; give it the
whole sheet and nothing is left. The construction is checked: a grid of
points over the area lies outside the remnant.

## Cut report

`camlib.sheet_report(reports, nest=, remnant=, rapid_mm_min=, consumables=)`
adds up what cutting one sheet takes, from the reports of its `ThermalCut`
and `ThermalMark` operations (`OperationUpdate.report` of each operation's
last update):

| Field | Meaning |
|---|---|
| `parts`, `pierces`, `cut_length_mm` | Parts cut, beam starts and the kerf's length (with a crop cut's) |
| `mark_starts`, `mark_length_mm` | Marks made and their length |
| `rapid_length_mm` | Travel between cuts and marks, in the sheet's plane |
| `feed_seconds`, `dwell_seconds` | Every fed move at its programmed feed (small holes at their reduced feed); the pierce delays |
| `rapid_seconds`, `seconds` | The rapids at `rapid_mm_min`, and the total. `None` without a rapid rate |
| `part_area_mm2`, `sheet_area_mm2`, `utilisation` | From `nest` (a `NestResult`); `None` without one |
| `remnant_area_mm2` | From `remnant` (a `Remnant`); `None` without one |
| `consumables` | Per declared `Consumable`: the share of its life this sheet uses |

The times are nominal: they leave out acceleration, height moves and what
the controller does on its own (height sensing before a pierce, corner
slow-down), so a real sheet takes longer. `camlib.Consumable(name,
life_pierces=, life_cut_mm=)` declares a wearing part's life; camlib has no
consumable figures of its own. The share is given by pierces and by cut
length separately, and they are not combined: how the two add up is the
torch maker's knowledge, not camlib's.

```python
report = camlib.sheet_report(reports, nest=result, remnant=left, rapid_mm_min=machine_rapid,
                             consumables=(camlib.Consumable("nozzle", life_pierces=400),))
print(report.cut_length_mm, report.pierces, report.seconds, report.utilisation)
```

## DXF

`camlib.dxf.read_parts(path, unit_mm=, join_tolerance_mm=)` reads the closed
outlines of a drawing as parts: one `PlanarRegion` per outline with the holes
drawn inside it (an outline inside a hole is a part of its own), ready for
`NestPart`. It reads `LINE`, `ARC`, `CIRCLE`, `LWPOLYLINE` and 2D `POLYLINE`
entities with their bulges, in any order and direction, and keeps arcs as
arcs.

- `unit_mm` is the size of a drawing unit in millimetres (1, or 25.4 for
  inches). There is no default, and a file whose `$INSUNITS` says otherwise is
  refused.
- `join_tolerance_mm` is how far apart two ends may be and still be one
  corner. There is no default. The largest gap closed becomes the region's
  `boundary_error_mm`, which the cut adds to its allowance. An end with
  nothing to join, or three ends at one point, is refused with the entity and
  the point.
- Splines, ellipses, blocks, hatches and anything off the Z = 0 plane are
  refused by name (explode them to lines and arcs in the drawing). Text and
  dimensions are refused too unless their types are listed in `ignore=`;
  `layers=` restricts reading to named layers.

`camlib.dxf.write_regions(regions, path)` writes regions as closed polylines
in millimetres (`$INSUNITS` 4), outlines on layer `OUTLINE` and holes on
`HOLE`, with full precision; `read_parts(path, unit_mm=1, ...)` reads them
back. This is 2D line and arc geometry only: camlib holds no CAD kernel.

## Limits

- **A heuristic, not an optimum.** Bottom-left fill gives a good nest quickly;
  it does not find the densest one, and camlib makes no claim about how close
  it comes. The search improves the order only, by exchanges.
- **Resolution.** Columns are `resolution_mm` apart, and curved planning
  outlines stand up to a quarter of `resolution_mm` outside the required
  clearance. A finer resolution nests tighter and takes longer; more than a
  million columns across the sheet are refused.
- **Leads.** By default room for leads and pierce craters is reserved all
  round every part, which is safe for any lead position and wastes material.
  `lead_room="at_leads"` reserves it only where each outline starts (above);
  the start is then fixed per part (its longest straight edge), not chosen to
  suit the neighbours, and holes still keep their leads inside themselves.
- **Center pierces.** In a hole too small for its leads the `ThermalCut`
  pierces at the hole's center, which is further from the loop than
  `lead_reach_mm`. A part nested in such a hole keeps the part distance to the
  hole's edge, but its spacing to that pierce and lead is not part of the
  nest's proof: the cut proves them clear of the part, or refuses.
- **Declared rotations only.** No free rotation.
- **Several sheets.** `Nest.run_sheets(sheets)` nests over full sheets and
  remnants in the order given: each takes what fits of what is left. It
  returns the nests of the sheets used and the parts left over (refused
  unless `allow_unplaced`). It does not choose among sheets or balance them.
- **Not modeled:** common-line cutting between neighbours (place parts at
  `ThermalCut.common_line_spacing_mm()` yourself), grain direction beyond the
  allowed rotations, heat distribution and cut order (the `ThermalCut` orders
  the loops: its `part_order`), tip-up of cut parts (its `avoid_cut_parts`).
