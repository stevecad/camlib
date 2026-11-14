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
| `parts` | `NestPart(region, quantity, rotations_deg, mirror=False, priority=0, fill=False)`: a `PlanarRegion` of one outline with holes, the number of copies, the allowed rotations (degrees counter-clockwise; `(0,)` keeps the part as drawn), and whether it may also be turned over. Parts of a higher `priority` are placed first (nearest the start corner). `fill=True` makes a filler: placed after every other part, as many copies as fit up to `quantity`, with no refusal for the rest (the result's `filled` lists `(part, copies)`) |
| `kerf_mm`, `lead_in`, `lead_out`, `pierce_clearance_mm` | What the cut will use. `Nest.for_cut(cut, ...)` reads them, and the tolerances, from a `ThermalCut` |
| `corner_loops` | The cut's `CornerLoops`, if it has them (`Nest.for_cut` reads it): room is kept for the loops as for the leads, `overshoot / tan(min angle / 4)` beyond the kerf path's corner |
| `part_spacing_mm` | Material left between what is cut for two parts |
| `edge_margin_mm` | Material left between what is cut for a part and the sheet's outline or holes |
| `resolution_mm` | Column spacing of the placement (see Limits) |
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
`pierce_clearance_mm` beyond half a kerf round it. The larger of the lead-in's
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
- **Leads.** Room for leads and pierce craters is reserved all round every
  part, not only where the cut will put them. This is safe for any lead
  position and wastes material compared with nesting at the kerf and placing
  leads into gaps afterwards.
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
- **Not modeled:** common-line cutting between neighbours, grain direction
  beyond the allowed rotations, heat distribution and cut order (the
  `ThermalCut` orders the loops: its `part_order`), tip-up of cut parts,
  skeleton cut-up.
