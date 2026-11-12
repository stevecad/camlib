# Python API map

The common names are also available from the top level (`camlib.Job`,
`camlib.Pocket`, `camlib.EndMill`, `camlib.Machine`, ...). They load on first
use, so `import camlib` is cheap and does not import CadQuery. At the top
level `Setup` is the job setup (`camlib.jobs.Setup`).

| Module | What it holds |
| --- | --- |
| `camlib.machine` | Required physical `MachineModel`, `Axis`, `MachineBody`, `Box`, `ConvexMesh`, `ToolCalibration`, `ToolChangeRoute`, `AllowedContact`, `MachineCheckOptions`; [coverage and required inputs](machine.md) |
| `camlib.jobs` | `Job(id, revision, setups)`, `Setup`, `PostedProgram` (result of `job.post_nc`: text, line/motion counts, findings, accepted kinds, unchecked checks, and a lathe setup's `material` check; a program with unaccepted violation/uncertain results raises `camlib.export.ExportRefused` instead, see `accept=`), `OperationNode(op, sources=, depends_on=, limits=)`, `RegenerationCache(directory, max_entry_bytes=, max_total_bytes=)`, `CadSource` (document id, revision, geometry hash and CadQuery/OCP versions of a CAD input) |
| `camlib.operations` | Operations and entry/clearing strategies ([Operations](operations.md)), `Tabs`, `FineBore`, `ThermalCut`, `Lead`, `SmallHoles`, `MicroJoints`, `steep_and_shallow`, `SuppliedToolpath` and `ToolpathMove` (a toolpath planned elsewhere, run unchanged: [supplied toolpath](operations.md#supplied-toolpath-suppliedtoolpath)) |
| `camlib.limits` | `Limits(contain=, avoid=)`, `AvoidArea(region, top_z_mm)` |
| `camlib.cutting` | `cutting_data` → `CuttingData`: spindle speed and feed from surface speed and feed per tooth (chip thinning, ball effective diameter, rpm cap) |
| `camlib.tools` | `EndMill`, `BallEndMill`, `BullEndMill`, `Drill`, `ChamferMill`, `ThreadMill`, `TSlotCutter`, `KeyseatCutter`, `LollipopCutter`, `DovetailCutter`, `Tap`, `BoringBar`, `Torch`, `LaserHead`, `Neck`, `ToolAssembly`, `HolderSection`, `holder_sections_from_mesh` |
| `camlib.planar` | `PlanarRegion` (`components`, reported `boundary_error_mm`), `Component`, `Boundary` (line/arc regions) |
| `camlib.surface` | `SurfaceMesh` (triangle surfaces with a finite stated `surface_error_mm` in the inclusive range 0–0.1 mm; checked at construction) |
| `camlib.simulation` | `StockSpec(min, max, cell_size, max_cells=, mesh=)` and `StockSpec.from_mesh(mesh, cell_size)`, `InitialStock` (declared whole-cell blank bounds, [preparation](preparation.md)), `StockMesh` (closed stock solid: `from_triangles`, `transformed`; `StockState.stock_mesh()` returns the finished stock), `DexelStockSpec(min, max, cell_size, max_rays=, max_intervals=)` (tri-dexel stock: undercuts), `SweepOptions` (chord error, tip step and work budgets of the stock sweep), `Simulation`, `ToolChangePosition`, `ToolChangeTable`, `SimulationError`, `ReachRefused` and its typed `ReachRefusal` ([reach refusals](operations.md#pocket-pocket-restpocket)), `CamWarning`, low-level streaming |
| `camlib.verification` | `StreamChecks(scene=, options=, material=, stock_contact=)`, `StockContactOptions` (resolution, work and finding limits), `Scene(id, revision, bodies, machine, work_to_machine)`, `Prism` (fixture or protected part; `z_error_mm`, `representation`), `MachineLimits` (travel of one tip or of the gauge line, feed, acceleration, axis feeds), `MaterialCheck(target, after_operation=)` |
| `camlib.playback` | `Player`, `Cursor` (operation, motion and `fraction` along it), `RecordedRun` (replay, seek, save) |
| `camlib.dexel_state` | `DexelStockState` (a tri-dexel run's committed stock: `volume()`, `heights` (the height view), `ray(axis, index)`, `rays(axis)`, `grid`, `preview(frame, fraction)`, `compare_solid(mesh, tolerance_mm)` (leftover material and over-cut against a closed mesh of the part, undercuts included), checkpoints), `heights_at` |
| `camlib.post` | `Machine`, `ToolSetting`, `GCodeDialect`, `GCodePost`, `NativeGCodeDecoder`, `LoweringOptions`, `post_program`, `post_events` (post an event list you built), `post_job_input`/`post_job` (the Rust default post, see [posts](posts.md#posting-without-python-rust-webassembly)) |
| `camlib.posted` | `PostedRun`: replays a posted program from its decoded G-code (what the controller will do), with stock and checks |
| `camlib.dynamics` | `MotionPolicy` (acceleration, rapid feed, junction mode) and feed scheduling / time estimates |
| `camlib.motion` | Typed motion and events (`Motion`, `Line`, `Arc`, `Helix`, `Dwell`, `Cycle`, `SpindleOrient`, `CutterCompensation`, `Beam`, `HeightControl`, `SurfaceSpeed`, `FeedPerRev`, ...), `WorkFrame(origin_mm, x, y, z)` |
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
computed. A `RegenerationCache` reuses operations whose inputs did not change
and recomputes the rest. `examples/cadquery_job.py` shows the same job built
from a CadQuery solid (pocket floor selected from the CAD face).

## Accuracy and work budgets

These rarely need changing; each failure names the budget it hit.

| Object | Fields |
| --- | --- |
| `SweepOptions` (stock simulation) | `chord_error_mm` (0.002), `max_tip_step_mm` (0.1) sampling of curved/sloped sweeps; `max_segments`, `max_scratch_cells`, `max_changed_cells`, `max_cell_visits` work limits |
| `StockSpec` | `max_cells` caps the stock grid size |
| `DexelStockSpec` | `max_rays` caps the three ray grids together; `max_intervals` the stored intervals (two per uncut ray) |
| `StockContactOptions` | `spatial_resolution_mm` (0.001) smallest region examined; `max_nodes`, `max_cell_reads` work per move; `max_findings` per move |
| verification `Options` | the same for fixture/part checks, and `target_floor_tolerance_mm` (floor allowance on protected parts) |
| `MachineLimits` | Travel, declared once: `min_tip_mm`, `max_tip_mm` (the tip travel box of one tool length) or `min_gauge_mm`, `max_gauge_mm` (`MachineLimits.gauge_line(min, max, max_feed_mm_min)`: the spindle gauge line's travel; each tool is checked against it lowered by its gauge length, see below). Limits included: a tip on a limit, such as a mill at home, is inside. `max_feed_mm_min`, `max_normal_accel_mm_s2`, `max_axis_feed_mm_min` |
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

`CadSource.from_shape(id, revision, shape, document_id=None, reference="body")`
records where CAD input came from (document, reference, geometry SHA-256,
CadQuery/OCP versions, BREP format), so cached results are reused only for the
same geometry.
