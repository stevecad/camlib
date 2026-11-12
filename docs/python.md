# Python API map

The common names are also available from the top level (`camlib.Job`,
`camlib.Pocket`, `camlib.EndMill`, `camlib.Machine`, ...). They load on first
use, so `import camlib` is cheap and does not import CadQuery. At the top
level `Setup` is the job setup (`camlib.jobs.Setup`).

| Module | What it holds |
| --- | --- |
| `camlib.machine` | Required physical `MachineModel`, `Axis`, `MachineBody`, `Box`, `ConvexMesh`, `ToolCalibration`, `ToolChangeRoute`, `AllowedContact`, `MachineCheckOptions`; [coverage and required inputs](machine.md) |
| `camlib.jobs` | `Job(id, revision, setups)`, `Setup`, `PostedProgram` (result of `job.post_nc`: text, line/motion counts, findings, accepted kinds, unchecked checks, and a lathe setup's `material` check; a program with unaccepted violation/uncertain results raises `camlib.export.ExportRefused` instead, see `accept=`), `OperationNode(op, sources=, depends_on=, limits=)`, `RegenerationCache(directory, max_entry_bytes=, max_total_bytes=)`, `CadSource` (document id, revision, geometry hash and CadQuery/OCP versions of a CAD input) |
| `camlib.operations` | Operations and entry/clearing strategies ([Operations](operations.md)), `Tabs`, `FineBore`, `ThermalCut`, `ThermalMark`, `Lead`, `SmallHoles`, `MicroJoints`, `CommonLine`, `steep_and_shallow`, `SuppliedToolpath` and `ToolpathMove` (a toolpath planned elsewhere, run unchanged: [supplied toolpath](operations.md#supplied-toolpath-suppliedtoolpath)) |
| `camlib.nesting` | `Nest`, `NestPart`, `NestPlacement`, `NestResult`: place sheet parts for a `ThermalCut` and prove the layout ([nesting](nesting.md)) |
| `camlib.limits` | `Limits(contain=, avoid=)`, `AvoidArea(region, top_z_mm)` |
| `camlib.cutting` | `cutting_data` → `CuttingData`: spindle speed and feed from surface speed and feed per tooth (chip thinning, ball effective diameter, rpm cap) |
| `camlib.tools` | `EndMill`, `BallEndMill`, `BullEndMill`, `Drill`, `ChamferMill`, `ThreadMill`, `TSlotCutter`, `KeyseatCutter`, `LollipopCutter`, `DovetailCutter`, `Tap`, `BoringBar`, `Torch`, `LaserHead`, `Neck`, `ToolAssembly`, `HolderSection`, `holder_sections_from_mesh` |
| `camlib.planar` | `PlanarRegion` (`components`, reported `boundary_error_mm`), `Component`, `Boundary` (line/arc regions) |
| `camlib.surface` | `SurfaceMesh` (triangle surfaces with a finite stated `surface_error_mm` in the inclusive range 0–0.1 mm; checked at construction) |
| `camlib.simulation` | `StockSpec(min, max, cell_size, max_cells=, mesh=)` and `StockSpec.from_mesh(mesh, cell_size)`, `InitialStock` (declared whole-cell blank bounds, [preparation](preparation.md)), `StockMesh` (closed stock solid: `from_triangles`, `transformed`; `StockState.stock_mesh()` returns the finished stock), `DexelStockSpec(min, max, cell_size, max_rays=, max_intervals=)` (tri-dexel stock: undercuts), `SweepOptions` (chord error, tip step and work budgets of the stock sweep), `Simulation`, `ToolChangePosition`, `ToolChangeTable`, `ProgressUpdate` ([planning progress](#planning-progress)), `SimulationError`, `ReachRefused` and its typed `ReachRefusal` ([reach refusals](operations.md#pocket-pocket-restpocket)), `CamWarning`, low-level streaming |
| `camlib.verification` | `StreamChecks(scene=, options=, material=, stock_contact=)`, `StockContactOptions` (resolution, work and finding limits), `Scene(id, revision, bodies, machine, work_to_machine)`, `Prism` (fixture or protected part; `z_error_mm`, `representation`), `MachineLimits` (travel of one tip or of the gauge line, feed, acceleration, axis feeds), `MaterialCheck(target, after_operation=)` |
| `camlib.playback` | `Player`, `Cursor` (operation, motion and `fraction` along it), `RecordedRun` (replay, seek, save) |
| `camlib.dexel_state` | `DexelStockState` (a tri-dexel run's committed stock: `volume()`, `heights` (the height view), `ray(axis, index)`, `rays(axis)`, `grid`, `preview(frame, fraction)`, `compare_solid(mesh, tolerance_mm)` (leftover material and over-cut against a closed mesh of the part, undercuts included), checkpoints), `heights_at` |
| `camlib.post` | `Machine`, `ToolSetting`, `GCodeDialect`, `GCodePost`, `NativeGCodeDecoder`, `LoweringOptions`, `post_program`, `post_events` (post an event list you built), `post_job_input`/`post_job` (the Rust default post, see [posts](posts.md#posting-without-python-rust-webassembly)) |
| `camlib.posted` | `PostedRun`: replays a posted program from its decoded G-code (what the controller will do), with stock and checks |
| `camlib.dynamics` | `MotionPolicy` (acceleration, rapid feed, junction mode) and feed scheduling / time estimates |
| `camlib.motion` | Typed motion and events (`Motion`, `Line`, `Arc`, `Helix`, `Dwell`, `Cycle`, `SpindleOrient`, `CutterCompensation`, `Beam`, `BeamMark`, `HeightControl`, `SurfaceSpeed`, `FeedPerRev`, ...), `WorkFrame(origin_mm, x, y, z)` |
| `camlib.turning` | Lathe setups (`Setup(turning=...)`): `TurningSetup`, `TurnStockSpec`, `TurningTool`, `TurningInsert` (`iso`, `iso_code` constructors), `InsertOutline`, `TurningHolder` (`rectangle`, `from_model`), `TurningToolModel`, `TurningModelPlacement`, `TurningModelInsertCheck`, `TurningModelWithoutInsert`, `TurningBar`, `Outline`/`Line`/`Arc`, `GrooveInsert`, `ThreadInsert`, `TurnToolSetting`, `TurnFace`, `TurnRough`, `TurnFinish`, `TurnGroove`, `TurnPartOff`, `TurnThread`, `DrillInsert`, `TurnDrill`, `TurnBoreRough`, `TurnBoreFinish`, `FaceGroove`, `FaceGrooveInsert`, `TurnFaceGroove` |
| `camlib.cadquery` | Optional CadQuery adapter (`export_floor_region`, `export_surface_mesh`, `flow_paths`, `turning_section`, `tool_body_from_step`, `tool_body_from_shape`, `step_solids`, `step_mesh`) |

Typical flow, as in [Getting started](getting-started.md):

1. Build tools, geometry and operations.
2. `Setup(...)` → `Job(...)`.
3. `job.post(setup_id, machine=...)` to plan, simulate, check and post in one
   stream with a playback journal; `job.post_nc(setup_id, machine=..., path=...)`
   to write the same checked program directly; or `job.regenerate(cache)` /
   `job.record(setup_id, rapid_mm_min=...)` to plan and simulate without posting
   (the rapid rate times G0 moves in playback; posting takes it from the `Machine`).
4. Supply `Machine.crash_model` before machine-ready export (missing coverage requires explicit `accept={"unchecked_machine"}`).
5. `Player` to replay stock and read check results; `run.nc_lines()` for G-code;
   `run.export(path, accept=...)` to write the finished program once its checks allow it.

`Setup(avoid=(AvoidArea(region, top_z_mm), ...), contain=region)` limits every
operation of the setup, and `OperationNode(op, limits=Limits(...))` adds limits
to one operation; see [Operations](operations.md#machining-limits-avoid-areas-and-containment).

`Setup(tool_change=ToolChangePosition(gauge_z_mm=..., xy_mm=None))` declares
where the machine leaves a new tool (in work coordinates; `xy_mm=None` keeps X
and Y). `Setup(tool_change=ToolChangeTable({tool_id: (x, y, z), ...}))` gives
each tool's tip after its change instead (`ToolChangeTable.from_machine(machine)`
takes them from a `"table"` machine's `position_after_mm`).
`Setup(tool_change=ToolChangeInPlace())` is a machine that changes the tool
where the spindle stands: the tip moves along Z by the old gauge length minus
the new one. Without any of them, each tool's tip starts where the previous
operation ended. See
[Posts](posts.md) for the matching machine setting.

`SuppliedToolpath(id, tool, motions, clearance_z_mm, ...)` puts moves planned
outside camlib into a job between planned operations: `motions` are
`ToolpathMove(kind, curve, feed_mm_min)` with `camlib.motion.Line`/`Arc`/`Helix`
curves, or `camlib.motion.Motion` objects. They are serialised once
(`to_dict()` returns the same move list each time) and are part of the
operation's input digest and cache key. See
[Operations](operations.md#supplied-toolpath-suppliedtoolpath) for the rules.

`job.regenerate(cache)` yields motion, stock and operation updates as they are
computed (and planning progress when asked, see [Planning progress](#planning-progress)). A `RegenerationCache` reuses operations whose inputs did not change
and recomputes the rest. `examples/cadquery_job.py` shows the same job built
from a CadQuery solid (pocket floor selected from the CAD face).

## Planning progress

Planning one operation can take seconds to minutes. A run tells an application
which operation it is planning and how far along it is with
`camlib.simulation.ProgressUpdate`:

| Field | Meaning |
| --- | --- |
| `operation`, `index`, `operations` | The operation being planned: its id, and its index among the setup's `operations` operations |
| `basis` | What `fraction` counts: `"layers"`, `"area"`, `"passes"`, or `None` |
| `done`, `total` | Depth levels (`"layers"`, `"area"`) or finishing passes (`"passes"`) done and planned, to show "layer 3 of 8" |
| `fraction` | Completed fraction, 0 to 1; `None` when `basis` is `None` |
| `area_left_mm2`, `area_start_mm2` | `"area"` only: the reachable material area left in the current level and at the level's start |

The fraction is only what the planner counts, never an estimate of time:

- `"layers"`: depth levels done of levels planned (`Roughing`, `Pocket`,
  `AdaptivePocket`, `Waterline`, `Contour`, `Chamfer`), unless the operation
  clears with the engagement front.
- `"area"`: adaptive clearing with the engagement front
  (`path_strategy="engagement_front"` of `AdaptiveClearing` in `Roughing`, or
  of `AdaptivePocket`).
  Levels done plus, in the current level, the share of its reachable material
  area already cleared, over the levels planned:
  `(done + 1 - area_left_mm2 / area_start_mm2) / total`. The area is measured
  when the planner refreshes its candidate starts, so it moves in steps.
- `"passes"`: surface finishing (`Parallel` and its patterns): passes done of
  passes planned.
- `None`: every other operation (drilling, facing, turning, ...) and an
  operation reused from a `RegenerationCache`. The update still says which
  operation is running.

Within an operation the fraction never decreases (if a refresh finds more
reachable material, the fraction holds while `area_left_mm2` shows the new
figure) and it is 1 in the operation's last update. An operation's first
update has no fraction and arrives before its `OperationUpdate` `begin`:
preparing the plan is part of planning.

Progress is asked for, so code that handles only the other update types keeps
working unchanged:

```python
for update in simulation.run_operations(progress_interval_s=0.25):
    if isinstance(update, ProgressUpdate):
        show(update.operation, update.index, update.operations, update.fraction)
    ...
```

| Where | How |
| --- | --- |
| `Simulation.run_operations(progress_interval_s=...)` | Yields `ProgressUpdate` between the other updates; none without the argument |
| `job.regenerate(cache, progress_interval_s=...)` | The same, as `JobUpdate(setup, ProgressUpdate)` |
| `job.record(...)` (`RecordedRun`) | `run.planning_progress()` returns the latest update (asked every 0.25 s; `progress_interval_s=None` turns it off) |
| `job.post(...)`, `job.post_nc(...)` | `on_progress=callback` is called with each update, on the thread that produces the run (`progress_interval_s`, 0.25 s unless given) |

`progress_interval_s` is 0.001 to 1 second: an update goes out when an
operation's planning starts and ends and at that interval between, from a
timer in the stream process, so also while the planner is busy before the
operation's first move or between two moves. `0` sends one whenever
the planner's counts change instead (after a move), which is deterministic.

A progress update is not part of the run. It carries no frame, is never
written to a recorded or saved run or a regeneration cache, and is not in
`status()`. The motions, stock, checks, checkpoints and posted program are
the same with or without it.

## Accuracy and work budgets

These rarely need changing; each failure names the budget it hit.

| Object | Fields |
| --- | --- |
| `SweepOptions` (stock simulation) | `chord_error_mm` (0.002), `max_tip_step_mm` (0.1) sampling of curved/sloped sweeps; `max_segments`, `max_scratch_cells`, `max_changed_cells`, `max_cell_visits` work limits |
| `StockSpec` | `max_cells` caps the stock grid size |
| `DexelStockSpec` | `max_rays` caps the three ray grids together; `max_intervals` the stored intervals (two per uncut ray) |
| `StockContactOptions` | `spatial_resolution_mm` (0.001) smallest region examined; `max_nodes`, `max_cell_reads` work per move; `max_findings` per move |
| verification `Options` | the same for fixture/part checks, and `target_floor_tolerance_mm` (floor allowance on protected parts) |
| `MachineLimits` | Travel, declared once: `min_tip_mm`, `max_tip_mm` (the tip travel box of one tool length) or `min_gauge_mm`, `max_gauge_mm` (`MachineLimits.gauge_line(min, max, max_feed_mm_min)`: the spindle gauge line's travel; each tool is checked against it lowered by its gauge length, see below). Limits included: a tip on a limit, such as a mill at home in a corner of its travel, is inside, and every move (line, arc or helix, in any work frame) is decided as inside or as a violation at the point that leaves; travel is never reported `uncertain`. A declared end is compared exactly: the motion tolerance is not a travel allowance (see [the travel rule](#the-travel-rule)). `max_feed_mm_min`, `max_normal_accel_mm_s2`, `max_axis_feed_mm_min` |
| `RegenerationCache` | `max_entry_bytes`, `max_total_bytes` disk limits |

**Machine travel with tools of different lengths.** A machine's travel is its
axes' travel: the controller's soft limits act on the machine position (the
spindle gauge line), whatever tool length offset is active (LinuxCNC INI
`MIN_LIMIT`/`MAX_LIMIT`; FANUC stored stroke check 1, parameters 1320/1321: B-63534EN/02 section 6.3, p. 855–856).
The tip of a tool sits its gauge length below the gauge line, so a short tool
reaches higher and a long tool lower. A single tip box
(`min_tip_mm`/`max_tip_mm`, `Machine.tip_travel_mm`) is the travel of one
gauge length only. For a job whose tools differ, declare the gauge line's
travel instead (`MachineLimits.gauge_line(...)`, `Machine.gauge_travel_mm`):
every planned move, and every move decoded from the posted program, is then
checked against its own tool's travel (the gauge box lowered along machine Z
by that tool's `gauge_length_mm`), and a finding names the tool and the axis.
Declare one or the other, never both. Gauge-line travel needs the work Z axis
along machine Z and spindle-held tools: a tilted work frame, a beam (plasma or
laser) tool or a lathe is refused, and declares a tip box.

### The travel rule

Every check that compares a position with the machine's travel uses one rule
(`src/travel.rs`), so they cannot disagree about a position on a limit: the
planned-move check (`MachineLimits`), the posted-program check
(`Machine.tip_travel_mm`/`gauge_travel_mm`) and the kinematic machine model's
axis travel ([`MachineModel.axes`](machine.md)), including its start position
and route waypoints.

- **The travel box is closed.** A position on a limit is inside: a machine at
  home sits on its limits, often on three at once.
- **Declared positions are exact.** The ends of every move, a tool-change
  pose and the start pose are compared as declared. The only allowance is the
  bound on the binary64 evaluation of the work-to-machine transform
  (`origin + x·X + y·Y + z·Z`: three products and three sums, at most
  `5 · 2⁻⁵³ · (|origin| + |x·X| + |y·Y| + |z·Z|)` on each machine axis, about
  1e-13 mm at a metre of travel), plus one rounding of `limit − gauge length`
  on machine Z with gauge-line travel. A line end 1e-6 mm beyond a limit is a
  violation. `Options.tolerance_mm` and the post's `tolerance_mm` are not
  travel allowances.
- **An arc or helix is tested on its nominal circle too.** Besides its two
  declared ends (exact), it is tested where each machine coordinate of its
  nominal circle is extreme, and at the nominal end. The nominal circle is
  drawn from the start about the centre, and the declared end may be off it by
  the move's own endpoint gap (the gap of this move, not the tolerance, which
  only bounds it). What the gap means depends on whose arc it is:
  - *A planned arc* is camlib's own representation of one curve, known to
    within its gap: a nominal point is a violation only beyond the gap plus
    the evaluation bound of a circle point (about 1e-13 mm for a 10 mm
    radius). A planned arc whose end lies on its circle is held as exactly as
    a line.
  - *A decoded arc* is what the controller will execute, and controllers
    reconcile a start radius and an end radius that disagree in different
    ways (averaging them, spiralling between them, keeping the centre). The
    executed path is only known to lie within the gap of the nominal circle,
    so the posted check admits the arc only when every nominal extreme is
    inside the limit by the gap. The gap shrinks what is admissible; it is
    never an allowance. So an arc exactly tangent to a limit is clear in the
    plan check and posts when its words are exact (no gap), and is refused
    when rounding leaves any gap. The refusal names the axis, the nominal
    extreme, the gap and the limit; the remedies are more decimals, keeping
    the arc the gap inside the limit, or a
    machine without arcs, whose arcs are lowered to lines.
- **The posted program is held to its decoded words.** The controller
  executes the words, rounded to the machine's `decimals`, and stops on its
  travel limit (LinuxCNC `MIN_LIMIT`/`MAX_LIMIT`; FANUC stored stroke check 1,
  B-63534EN/02 section 6.3). A decoded position beyond a limit is refused,
  however small the step; rounding is never given a tolerance. A position on
  a limit is written exactly when the work offset and the limit are on the
  output grid (multiples of the output step). If they are not, a planned
  position on the limit can round beyond it, and the post refuses with the
  axis, the decoded position and the limit: keep the plan one output step
  inside, or put the offset and the limit on the grid.

- **A calibration interval is judged whole.** The kinematic machine model
  declares how far the work frame and each tool's gauge offset may be from
  their expected values, so there a decoded position is an interval on each
  machine axis: inside only if the whole interval is inside the closed
  travel, a violation only if the whole interval is beyond a limit, and
  `uncertain` while it straddles one (the finding states the interval and the
  limit). The interval is the declared calibration alone; no tolerance widens
  it. A decoded arc's endpoint gap is such an interval about its nominal
  circle: the posted check refuses an arc that is not inside by its gap, and
  the machine model reports it `uncertain`, or a `violation` once it is
  beyond the limit by more than the gap. Positions declared in machine
  coordinates (the model's start position and route waypoints) involve no
  arithmetic and are compared exactly.

A rotated work frame is decided on the move itself, not on a box around it.
Through a general rotation the machine coordinate of a position is a rounded
computation, so "on the limit" holds only to that evaluation bound; with the
work axes along the machine's it is exact.

`CadSource.from_shape(id, revision, shape, document_id=None, reference="body")`
records where CAD input came from (document, reference, geometry SHA-256,
CadQuery/OCP versions, BREP format), so cached results are reused only for the
same geometry.
