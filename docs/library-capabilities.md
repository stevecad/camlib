# Capabilities and limits

What camlib does, what it checks, and where its responsibility ends.

## Inputs and outputs

You supply a job: stock, a work frame, tool assemblies and an ordered list of
operations. Geometry is either authored directly (planar line/arc regions with
islands, hole centres, triangle-mesh surfaces with a stated error) or extracted
from CadQuery solids with the optional adapter. A solid alone does not choose
operations, tools or workholding; you do.

camlib produces typed motion (lines, arcs, helices) with operation and tool
identity, simulated stock removal streamed alongside it, check results, and
G-code through a Python postprocessor. It never connects to a machine.

## Operations

See [Operations](operations.md) for parameters. Summary:

| Operation | Scope |
| --- | --- |
| Facing | Rectangular stock top; flat mills; zig-zag or one-way |
| Pocket / rest pocket | Line/arc regions with islands; plunge, helical or stock-edge entry; rest pocket cuts only what the simulated stock still holds |
| Adaptive pocket | Engagement-limited clearing of the same regions |
| 3D roughing / rest roughing | Layered clearing above a mesh part: raster, offset, adaptive; air passes can be omitted |
| Contour | Outer walls and/or hole walls of a part: climb (default) or conventional, tangent leads, wear compensation (G41/G42), holding tabs (triangle or rectangle) |
| 3D finishing cutters | Ball (all patterns, certified error), flat and bull-nose (raster, spiral, radial, project, waterline; exact contacts, proven no-gouge moves, sampled error) |
| Parallel / spiral / radial / project / pencil / scallop finishing | Mesh surfaces; slope (contact-angle) limits; rest finishing from the stock model |
| Waterline, ramp finishing | Constant-Z contours, or one continuous descent per level (`ramp=True`); steep-and-shallow pairs waterline with parallel or spiral |
| Flat-area finishing | Horizontal faces of a mesh part at their exact heights, flat end mills |
| Morph, flow | Passes between two guide curves; CAD face parameter lines (CadQuery) followed with Project |
| V-carve | Maximal inscribed disks along the outline, sharp corners, overcut-checked between samples |
| Machining limits | Avoid areas (clamps, fixtures) with a top height and containment regions, for every operation: region operations plan around them, profiles lift over them, finishing leaves out what is not proven allowed, holes/slots are refused; rapids and links checked |
| Drilling, pecking | Tip or full-diameter depth; full-retract or chip-break pecks; dwell at depth (G4 or G82); canned cycles G81/G82/G83/G73, decoded and checked move by move |
| Reaming, boring | Feed out (G85), dwell (G89), spindle stop and rapid out (G86) or fine boring with an oriented stop and shift (G76, or M19 and explicit moves), as canned cycles where declared; pilot hole proven open first; every shift proven inside the bore |
| Bore milling | Helical full-hole clearing |
| Trochoidal slot | Straight slots, width between 1x and 2x tool diameter |
| Thread milling | Internal/external helical threads; hole or boss proven in the stock first |
| Tapping | Rigid right- and left-hand tapping cycles (G33.1 under M3/M4, or G84/G74 and G80 where declared); drilled hole proven first |
| Chamfer | Top edges, depth from the chamfer tool's angle and tip; stepped widths, leads |
| Trace / engrave | Open or closed line/arc paths at depth, no radius offset |
| Feeds and speeds | `cutting_data`: rpm and feed from surface speed and feed per tooth, chip thinning, ball effective diameter |
| Plasma and laser cutting | Sheet parts with holes: kerf compensation (computed or G41/G42 wear), arc/line leads from the scrap, overcut or negative overburn, pierce clearance (center pierce in small holes), inner features and nested parts first, small-hole feed and THC off, micro-joints, fixed or programmed pierce/cut heights; every lead, kerf path and pierce proven clear of the parts; sheet simulated cut through |
| Turning (lathe) | Facing, outside roughing (axial passes, every diameter left at its allowance), finishing, external, internal and face grooving, parting off, external and internal single-point threading, drilling on the axis (pecks), inside roughing and finishing (boring bars) of a turned profile and bore with insert/holder/bar geometry; every move checked against the part, its bore, the stock and the chuck; axisymmetric stock with outer and inner bounds; constant surface speed (G96 with its G50/D limit) and feed per revolution (G95/G99); G18 output, radius or diameter X, T0101 words; a lathe setup is an ordinary `Setup` streamed like every run (per-move stock, playback with partial moves, cancellation, posted replay on the stock) with a material check (leftover material against the part) after each operation |

Not supported: trochoidal curved slots, undercuts,
4th/5th axis. Unsupported tool/operation
combinations and impossible dimensions are rejected before any motion.

## Linking and heights

- Every operation starts and ends at `clearance_z_mm`, travels at
  `retract_z_mm` and changes from rapid to feed at `feed_z_mm` (stock top <
  feed ≤ retract ≤ clearance), so rapids at these heights cannot touch stock.
- Straight-up retracts are rapids by default, each proven clear from where
  the cut left the tool (`Setup(rapid_retracts=False)` keeps them fed).
- Pocket links stay at depth only through stock proven cleared, when faster
  than leaving, or up to `max_stay_down_mm`.
- `Setup(tool_change=ToolChangePosition(...))` declares where a new tool
  arrives (e.g. M6 to Z home); its first move is checked like any rapid.
- Avoid areas and containment regions apply to every operation and to every
  rapid and link.

## Geometry

- Planar regions keep exact arcs where possible. Offsets and booleans report
  their approximation error.
- Before an offset that shrinks the machinable area, boundaries are simplified
  conservatively (at most 0.002 mm, only ever shrinking the machinable area)
  so that dense, noisy outlines from tessellated CAD or mesh slices offset
  correctly. The amount used is reported and added to the error budget.
- Every pocket/roughing tool-centre path is checked against the **original**
  boundary before motion is produced; a construction error fails the operation.
- 3D surfaces are treated as a part seen from above (a height map with a
  protected base). Parallel and waterline finishing complement each other;
  neither guarantees every surface is reachable. Sharp valleys, steep walls and
  tool size leave material, which remaining-stock reports show.

The CadQuery adapter exports planar faces (lines, circles, and bounded
approximations of ellipses/B-splines) and surfaces made of planes, cylinders,
cones, spheres, tori and extrusions. General B-spline surface patches are not
supported. Load STEP/BREP in CadQuery first; camlib does not parse STEP.

## Stock simulation and checks

Stock is a grid of columns with lower/upper height bounds. It models top-down
removal well and reports removed and remaining volume as bounds. It cannot
represent overhangs or enclosed voids. Cell size is independent of machining
tolerance. Display meshes are approximations, never evidence.

Configured checks examine every line/arc/helix continuously (not just its
endpoints) against:

- the evolving stock (rapids must not touch it),
- fixtures (`Prism(role="fixture")`) and protected parts
  (`Prism(role="protected_target")`),
- cutter, shank and holder geometry,
- declared machine travel and feed limits.

Also available:

- **Material checks** (`MaterialCheck(target, after_operation=...)`): the
  actual stock after an operation (or at the end) against the part, reporting
  gouges and remaining material.
- **Machine dynamics** (`MachineLimits`, `Machine.max_normal_accel_mm_s2`,
  `max_axis_feed_mm_min`): curve feeds that need more acceleration or axis
  speed than declared are reported; with a `MotionPolicy` the program can be
  time-estimated and its feeds scheduled (`Machine.schedule`,
  `schedule_post_feeds`).
- **Operation reports**: each operation reports what it did and what it
  left (remaining-stock estimates, dropped lengths, construction errors,
  link lengths).

Results are reported per move as clear, violation or uncertain. Anything not
configured is listed as not checked: the overall status is always
`"partial"`, never a blanket certificate. Checks report findings per move
while the run continues; exporting the machine-ready program (`job.post_nc`,
`run.export`) refuses it while any violation or uncertain result remains,
unless that kind is accepted on purpose (`accept={"uncertain"}`), and lists
the checks that were not configured. Lathe setups run every check always:
each move against the part, bore, chuck and stock as it is planned or
replayed, and the stock against the part (material) after each operation.

## Tolerances

All dimensions are mm. Manufacturing budgets are separate from numerical guards:

| Setting | Default |
| --- | --- |
| Posted path deviation (`Machine.tolerance_mm`) | 0.01 mm |
| Curve-to-line chord (`LoweringOptions.chord_tolerance_mm`) | 0.0025 mm |
| CAD source approximation | 0.005 mm |
| Parallel finishing path error / safety margin | 0.005 / 0.005 mm |
| Waterline contour error / safety margin | 0.01 / 0.005 mm |
| Boundary and rest budgets | 0.01 mm |
| Boundary simplification before erosion | 0.002 mm (conservative side only) |
| Numerical geometry tolerance (`tolerance_mm`) | 0.00001 mm |

The numerical tolerance is not a machine accuracy. Source error, path error,
post rounding, stock resolution, scallop height and stock-to-leave are
different quantities; setting one does not replace another.

## Posting

Machines and posts are Python. The engine handles units, modal state, work
frames, work and length offsets, tools, spindle/coolant, controller curve
capabilities and optional curve-to-line conversion, then independently decodes
the emitted text. Output precision defaults to 3 decimals (mm). See
[Postprocessors](posts.md). Physical execution (tool-change routines,
controller parameters, real offsets) is outside what camlib can check.

| Feature | Output |
| --- | --- |
| Arcs and helices | G2/G3 where the controller declares them and the rounded words stay within tolerance (spiral mismatch included); otherwise checked line segments |
| Dwell | G4 with the controller's unit and resolution, never shorter |
| Drilling and boring cycles | G81/G82/G83/G73, G85/G89/G86, G76 (with G98, G80); decoded back into moves and checked |
| Rigid tapping | G33.1 (M3 right, M4 left), or G84 / G74 with feed or lead word |
| Lathe threading | G33 K<lead> or G32 F<lead> (spindle-synchronized), decoded back |
| Lathe spindle and feed modes | G96 S (with G50 S or D limit) / G97, G95 or G99 F per revolution / G94 or G98, decoded back; per-move feed bounds checked |
| Cutter compensation | G41/G42 with the tool's D register (wear), G40: on their own line (LinuxCNC, Centroid) or in the lead move's block (FANUC, Haas) |
| Oriented spindle stop | M19 (declared per machine) for fine boring |
| Beam on/off, power | Plasma `M3 S1`/`M5`, laser `M4 S<power>` (GRBL laser mode) or `M3 S`, from the declared S at full power |
| Torch height control | Declared on/off codes (e.g. `M63 P2`/`M62 P2`) |
| Tool changes | Table poses, preserved tip, or Z home |
| Redundant codes | Omitted only where the tracked state proves them redundant |

LinuxCNC is the `GCodeDialect.linuxcnc()` dialect of an ordinary `Machine`.
Python controller posts (examples): GRBL 1.1h (laser mode `M4 S`), Haas NGC,
FANUC 0i-MB, FANUC lathe (G-code system A), Siemens 808D (native and ISO),
Centroid CNC12 (mill, lathe and plasma), LinuxCNC QtPlasmaC, WinCNC plasma.

## Streaming, replay and reuse

Motion and stock stream together with bounded buffers, cancellation, pause,
speed control and seeking. Mid-operation events (dwells, cycles, spindle
stops, compensation) stream with the motion. Jobs reuse unchanged operations
(`RegenerationCache`, size-bounded) and recompute what a change affects, with
dependencies between operations (`OperationNode(depends_on=...)`) and source
identities (`CadSource`). Finished runs can be [saved and
reopened](persistence.md), and a posted program can be replayed from its
decoded G-code.

## CAD input (CadQuery, optional)

- Planar regions from faces (`export_floor_region`), surface meshes from
  solids with a stated error (`export_surface_mesh`), `Roughing.from_solid`.
- `flow_paths(face, ...)`: a face's parameter lines as XY passes for flow
  finishing.
- `turning_section(solid)`: the turned profile and bore of a solid of
  revolution about Z for a lathe setup (`TurningSetup.from_section`), proven axisymmetric by its volume.
- The host application (any CadQuery program) supplies solids and persistent references;
  camlib never parses STEP itself.

## Size, memory and time

There are no caps on job contents: setups, operations, tools, scene bodies,
material stages, motion and layer counts, triangles, faces, hole patterns and
stock cells are limited only by the data formats (32-bit frame lengths and
stock cell indices) and by the machine. Operation lists must be finite
collections (a list or tuple, not an endless iterator).

Transport limits bound single messages, not jobs:

| Limit | Value |
|---|---|
| Posting batch (planned source segments per check/commit) | 64 |
| Authored typed events per `consume` batch | 4096 |
| Encoded post chunk | 512 KiB |
| One operation's external inputs (JSON) | 64 KiB |
| Strings in stream frames | 65535 bytes (u16 length) |
| Stream frames | 32-bit length |

- **Memory.** Large allocations are checked against the memory actually
  available before they are made: the smaller of the operating system's free
  memory and the process's cgroup limit (containers, `systemd-run -p
  MemoryMax=`). Set `CAMLIB_MEMORY_BUDGET_BYTES` to give camlib an explicit
  budget instead. A job that would not fit fails with an error naming what was
  too large (for example a stock grid or raster stroke set) and how to shrink it.
- **Untrusted input.** Lengths read from streams and saved files are never
  allocated up front; reads grow with the bytes that actually arrive.
- **Time.** Long work is bounded by cancellation, not by work caps. Streamed
  runs of every machine type (milling, thermal cutting and lathe setups:
  `Simulation`, `RecordedRun`, posted runs) stop when closed or cancelled
  (a lathe run's planner polls it before every move); posting takes a
  `camlib.post.Cancellation`.

## Reference

FreeCAD CAM ([pinned source](https://github.com/FreeCAD/FreeCAD/tree/920d14763241267340e755f3d3b6ece8ef60fa43/src/Mod/CAM))
was used as the reference for workflow (job → tools → operations →
simulation → post). This does not claim feature parity.
