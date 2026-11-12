# Architecture

How camlib fits together, for developers (human or AI) changing it. User
documentation starts at [getting-started.md](getting-started.md); the reasons
behind the design are in [DECISIONS.md](DECISIONS.md).

## Layers

| Layer | Where | Role |
|---|---|---|
| Rust engine | `src/` (crate `camlib`) | Geometry, toolpath planning, stock, verification, posting checks |
| Stream process | `src/bin/sim_stream.rs` (`cam-sim-stream`) | Runs a setup and streams binary frames on stdout |
| Rust post | `src/gcode/`, `src/post_job.rs`, `src/bin/post_job.rs` (`cam-post`) | The default post, its decoder and the export gate without Python (JSON in, JSON out; issue #120) |
| Python extension | `bindings/python/` (`camlib._native`) | Buffer kernels, posting intent and decoding, region engine, native receivers |
| Python API | `python/camlib/` | Jobs and setups, operations, tools, machines and posts, runs, playback, export |

The Python API is the public interface. It declares inputs and consumes results,
while the Rust side does the planning and the checking.

## One run, end to end

1. **Declare.** A `camlib.jobs.Job(id, revision, setups)` holds `Setup`s. A
   setup has stock (`StockSpec` for milling and thermal, a box or a closed mesh
   given with `StockSpec.from_mesh`; `TurnStockSpec` for a lathe), tools, operations in order and a start position. A lathe setup also
   has `turning=TurningSetup(...)` (the part profile, chuck and tolerances).
2. **Compile.** `Job.compile()` validates the job and turns each setup into a
   manifest (JSON). The job revision binds a digest of the whole content, so any
   change to inputs is a new revision.
3. **Run.** `camlib.simulation.Simulation(**setup.simulation_options())` starts
   `cam-sim-stream`, sends the manifest, and reads frames back.
   - Milling and thermal: `regeneration::run` (`src/regeneration.rs`) plans
     each operation with `OperationPlan::prepare_with_stock`, pulls moves with
     `next_motion_with_stock`, sweeps each move through the stock and sends it
     with its stock change. Operations can be served from a regeneration cache.
     A `SuppliedToolpath` (`src/supplied.rs`, #157) is one more
     `OperationPlan`: it hands out the caller's moves unchanged, after the
     standard rapid travel to the first, so nothing downstream tells them
     from planned moves. Such a run must configure the stock contact check.
   - Lathe: `regeneration::run_turning` plans each operation with
     `turning::ops::TurnSession` when the run reaches it. Every move carries its
     exact `TurnDelta`.
4. **Receive.** `simulation.stock_state()` picks the receiver: `StockState`
   (height field, validated by native buffer kernels),
   `turn_state.TurnStockState` (the native `TurnReceiver`) or
   `dexel_state.DexelStockState` (the native `DexelReceiver`). **Every frame is
   checked against the receiver's own stock before it commits**: cursor,
   attribution, continuity, each changed cell's old value, material only removed.
5. **Consume.** `Simulation.run_operations()` yields:
   - `Update`: one move and its stock change;
   - `OperationUpdate`: an operation's begin, mid-operation events and end,
     with its report;
   - `VerificationUpdate`: per-move check results.
6. **Keep and replay.** `camlib.playback.RecordedRun` records frames and
   checkpoints to a directory. `Player` seeks by restoring the nearest
   checkpoint and replaying, including partway through a move. A run can be
   saved and reopened without replanning (`persistence.py`).
7. **Post.** See *Posting* below. `Job.post_nc` gives checked NC text, and
   `Job.post` gives a `posted.PostedRun` whose posted program is replayed on the
   stock. Both refuse unaccepted findings at export (`camlib/export.py`).

## Frames

Binary frames: a 20-byte header (`CAMLSTRM` magic, version, kind, length,
CRC-32), then the payload. Kinds (`src/stream.rs`):

| Kind | Name | Holds |
|---|---|---|
| 1 | Chunk | One milling or thermal move and its height-field cell changes |
| 2 | Checkpoint | The full committed stock and cursor, with the tool in the spindle at that position (#168: a tool change in place is measured from exactly it; a checkpoint written before it was recorded still restores, and such a change is then refused); a producer's also ends with its material checks' per-move proof states (`CAMPRF01` section, #108) |
| 3 | Manifest | The canonical manifest (its SHA-256 binds every later frame) |
| 4 | End | The run finished; final cursor and position |
| 5 | Preview | A partial move's stock (not committed) |
| 6 | Restored | A cursor restored from a checkpoint |
| 7 | Operation | An operation's begin, events or end, with its report (JSON) |
| 8 | Verification | A check record for a move or a material stage (JSON) |
| 9 | Control | Typed control events (version 2 runs) |
| 10 | Turn chunk | One lathe move and its turning stock change |
| 11 | Dexel chunk | One milling move on tri-dexel stock and the rays it changed |

Every frame carries the run id, revision and manifest digest. A frame from
another run or revision is refused.

## Stock models

Stock is **bounds, not samples**. Each cell holds what is *certainly* there and
what *may* be there, and removal moves both conservatively. That is how a check
can say clear, violation or uncertain honestly.

- **Height field** (`src/stock.rs`, `Stock`): an XY grid of `HeightBounds`
  (lower: material certainly up to here; upper: no material above here). Used by
  milling and thermal runs. Moves are swept with the full tool assembly
  (cutter and holder). A run starts from the whole box, or from a closed mesh
  (`src/stock_mesh.rs`, manifest `stock` with a `mesh`; always build a run's
  starting stock with `RunStockSpec::initial_stock`, since the grid alone is
  the box). The mesh is converted conservatively: a cell's upper bound is the
  highest point of the mesh over the cell's open footprint (facets lying only
  on a cell side belong to the neighbour), and its lower bound is claimed only
  where no other surface lies between the grid's bottom plane and that height
  and facets on that plane close the column from below. Undercuts and voids
  become possible material. `Stock::finished_mesh` returns the closed mesh of
  each cell's upper bound, without T-junctions, which reads back on the same
  grid as the same bounds; it is approximate (`HEIGHT_FIELD_APPROXIMATION`).
  Beside the cells, a natively committed flat cut leaves a record of the
  column it emptied (path, cutter radius, floor). Whole cells cannot show
  that a tool body as wide as the cutter fits where the cutter has been, so
  `Stock::history_clearance` proves it from those records: the body's sweep
  along a move lies inside the union of the emptied columns
  (`src/engagement/cover.rs` states the claim and the proof; exact where the
  move runs on earlier paths, otherwise with the sweeps' own path error and
  no other allowance). The roughing planner's shank check, the sweep's
  cutting-length check and stock contact all use this one proof (#200).
- **Turning** (`src/turning/stock.rs`, `TurnStock`): per z bin, bounds of the
  outer radius and of the bore radius, plus rings certainly removed inside the
  material (face grooves). Lathe moves carry their changes computed by the
  turning planner's move kernel (`Planner::emit` for a line, `emit_arc` for
  an XZ arc), which also checks the insert against the part, bore and chuck,
  and the holder and rapids against the stock. An arc is checked in pieces
  (each inside the triangle of its ends and their tangents' corner, within a
  quarter of the tolerance), each against the stock the pieces before it
  leave, and committed as one move. The insert that is checked is its body
  off the centre plane (`src/turning/body.rs`) and, where the tool declares
  its insert's size tolerance (ISO 1832 d and m), everything an insert of
  that tolerance can occupy relative to its measured nose
  (`src/turning/size.rs`): the nose and the two cutting edges at it are
  fixed, so the cut and the program do not change with it.

- **Tri-dexel** (`src/dexel.rs`, `DexelStock`; issue #1): three grids of rays
  along X, Y and Z, each ray a prism (its cell's rectangle times its line)
  holding two sorted interval lists: possible material (outside it the
  cross-section is empty) and certain material (inside it the cross-section
  is full). It holds undercuts. Grids combine only conservatively: a check is
  clear when one grid proves it. The sweep kernel (`src/dexel/sweep.rs`)
  removes the cutting part's sweep conservatively per ray (outer interval
  from certain, inner from possible) and refuses a move whose neck, shank or
  holder may meet possible material unless one grid proves that sweep
  empty. The run keeps a **height view**: per Z ray, the top of the certain
  material from the stock bottom (lower) and of the possible material
  (upper). It is a valid height-field bound, so planners, the retract
  policy, height-field stock contact, the material verifier and the machine
  replay read it unchanged; its native proof records are added only for moves
  that pass under no material. Frames carry ray changes; receivers derive
  the view themselves. Partial moves are swept from the receiver's stock, as
  for turning.

Stock only loses material. A receiver refuses any change that adds material, or
whose old value doesn't match its own stock.

## Verification

- **Per move** (`src/stream_verification.rs`, records of kind 8): scene bodies
  (fixtures, protected parts as prisms), stock contact (rapids and links must not
  touch stock), machine limits. On tri-dexel stock, contact the height view
  leaves uncertain is clear when one ray grid proves the swept assembly misses
  possible material (`DexelStock::proves_clear`); violations come from the
  height view's certain material.
- **Tri-dexel material** (`src/dexel/material.rs`,
  `DexelStock::compare_solid`, Python `DexelStockState.compare_solid`):
  dexel stock against a closed triangle mesh of the part with a stated
  error, undercuts included. Per ray, the part's certain and possible
  intervals come from the centre line's crossings (exact orientations) and
  the band where the ray's prism, grown by the error, meets a facet. A point's
  3D distance never exceeds its distance along a ray, so excess within
  tolerance along the rays proves the check; a cell counts as possible
  excess only when all three grids allow it there. A definite finding needs a
  witness whose exact distance to the mesh, less its error, exceeds the
  tolerance. In runs, the stream's material stages still compare the height
  view with `Target`/`SurfaceTarget`. Where two cut faces meet at an edge,
  every grid's bound spans a whole cell, so the tolerance the rays can
  prove there is about a cell's diagonal; choose `cell_size_mm` against the
  comparison tolerance.
- **Material** (`src/material.rs`, `camlib.material.Verifier`): stock against a
  target (`Target`: planar region and floor; `SurfaceTarget`: mesh). It reports
  leftover material (residual) and over-cut (gouge) as certain and possible
  volumes with per-cell findings. A `SurfaceTarget`'s upper bound over a
  cell takes only facets whose XY projection meets the cell (exact test), so
  a cell inside a through hole is bounded by the target's base. A cell that
  straddles a vertical wall is `possible` in the cell comparison: its
  whole-cell stock bounds hold both the wall and the cut-away side, and cannot
  tell a cut tangent to the wall from one into it. In a run, a material check
  also proves every motion against the target (`src/material/sweep_proof.rs`,
  issues #72 and #106). The tool is eroded by the comparison tolerance less the
  target's declared error. Against a `SurfaceTarget` mesh, the exact section
  contact of `flat::Index::sections_chord_clear` proves it clear. A ball or
  bull-nose end is proven with its real shape (#107): eroded, it is a smaller
  bull lifted by the erosion, and `flat::Index::bull_chord_bound_clear` bounds
  its contact (concave along each edge and chord, so no Lipschitz bound is
  needed at the corner's vertical rim); a corner smaller than the erosion
  falls back to the enclosing flat end, which is sound but conservative. Against a
  prismatic `Target`, the floor is checked exactly and the walls by the region's
  1-Lipschitz signed clearance on bisected chords; the boundary error and floor
  error are the declared error. A possible
  gouge cell that no unproven motion touches is then within tolerance. At such
  cells "within tolerance" is a depth into the part (distance), not a vertical
  height. The report itself stays the cell comparison, reproducible from the
  stock alone; the record's `sweep_proof`, beside it, gives the proven and
  unproven motions and the decided cells. `export.material_status(report,
  sweep_proof)` is clear when the proof decides every possible-only gouge cell.
  A cell is decided only when every motion since the raw stock was proven:
  a producer's checkpoint keeps each proof's state (history flag, proven and
  unproven counts, and the touched cells, one bit per cell, run-length coded
  when smaller), under the frame's run identity and checksum, so a stream
  resumed from it (`StreamProducer::from_checkpoint`, `--resume`, or
  `restore_checkpoint`) decides the same cells as one that never stopped. A
  checkpoint without it (a consumer's) resumes with `history_complete: false`,
  and decides nothing. Work budgets: `max_tile_nodes` and `max_boundary_tests`
  left unset grow with the grid to an upper bound on the work the check can
  need (the full tile tree; per cell, the target's hierarchy nodes, facets
  and edge tests, or per tile the region's segments), capped at the allowed
  maximums. An incomplete report's `stats.stopped_by` names the budget that
  stopped it and, when that bound is within the maximum, the value that
  completes it.
  Lathe runs report
  `turning::ops::material_report` after every operation; there, gouges are
  excluded per move by the planner's exact checks.
- **Export gate** (`camlib/export.py`): a program with unaccepted `violation`
  or `uncertain` results is refused (`ExportRefused`), and nothing is written.
  Callers accept a kind explicitly (`accept={"uncertain"}`). Exports also list
  the checks that were not configured. The refusal names the first result: a
  contact's kind and object, or a material result's target with its gouge
  assessment (and residual, when material is left); its `findings` hold the
  motion findings or the material report.

## Posting

1. A `camlib.motion.Program` (typed events: motions, tool changes, spindle,
   coolant, cycles, compensation) and a declared `camlib.post.Machine` (dialect,
   units, decimals, limits, tools, capabilities) go to
   `postprocessor.PostEngine`.
2. A native intent session (`src/post_intent.rs`, `IntentSession` in the
   bindings) tracks modal state and batches source moves.
3. A post class (`GCodePost` and subclasses; the controller posts in
   `examples/*_post.py`) writes words.
4. An **independent decoder** (`GCodeDecoder`, `NativeGCodeDecoder`, or a
   controller decoder) reads the written blocks back into motion, which is
   compared with the source move within the machine's tolerance. A mismatch
   fails the post. The decoder never trusts the post that produced the output.
5. Without Python, `src/gcode/` holds a port of the default post and decoder
   (`GCodePost`, `NativeGCodeDecoder`) driven by the same intent checker, and
   `src/post_job.rs` the whole `Job.post_nc` run (stock replay, machine model,
   export gate). Their output must stay byte-identical to the Python default
   post's (`python/tests/test_rust_post.py`); change both together.
6. Arcs that can't be written exactly at the machine's precision are refused,
   or become checked segments with `LoweringOptions(native_arcs=True)`.
   `camlib.dynamics.schedule` gives acceleration-aware timing and optional feed
   scheduling.

## Identity and determinism

- The manifest's canonical bytes and their SHA-256 bind each run, and a job
  revision binds a digest of its content.
- The same inputs give the same toolpaths. The benchmark (`bench/`) runs from
  frozen geometry, and CI fails when a run's toolpath data differs from the
  committed baseline (`bench/results/baseline.json`), on any machine.

## What is proven and what is not

- **Proven by camlib:** planned moves against stock, fixtures and machine
  limits (where configured); stock against the target, and in a run every
  motion against a surface target (no cut deeper than the comparison
  tolerance into the part); posted output against
  the plan (decoded); finishing chords between samples (certified, not just
  sampled endpoints).
- **Not proven:** posts have not been run on real machines (issue #20); camlib
  cannot know the real work offsets, tool lengths or clamps. A probing cycle
  (`src/probe.rs`) records what it stores on the controller (a work offset, a
  tool length or radius) as a premise; the program after it is checked with
  the declared values, not the measured ones. Display data (stock
  meshes, previews) is approximate. "Possible" material bounds include the stock
  grid's and the target mesh's resolution.

## Traps

- A clear straight rapid does not prove a controller's dogleg rapid clear.
  `tests/verification.rs` and `python/tests/test_machine_clearance_regressions.py`
  include a clamp missed by the diagonal but hit by an axis-first route, and
  a tall fixture hit by a short assembly's holder while its tip stays clear.
  The CNC12 mill decoder rejects G0, and its post uses declared feed traverses
  instead (see [the controller notes](centroid-post.md)). The generic motion
  checker still checks the supplied curve; it does not infer a controller's
  rapid route. A long tool clearing these fixtures proves only the declared
  assembly clear, not the spindle nose or head. The declared machine checker
  (#116, `src/machine`) checks those bodies, independent-axis rapid envelopes,
  machine coordinates and complete caller-supplied tool-change/index sweeps
  against actual decoded output and native stock replay. Export requires it,
  or explicit `unchecked_machine` acceptance. See [machine coverage](machine.md)
  for the exact physical-input contract and unsupported paths.

- Some arcs cannot be written with equal start and end radii at 3 decimals; see
  `docs/posts.md` and issue #58.
- Coarse stock cells inflate "possible" bounds. Compare certain values, or use a
  finer `cell_size_mm`. A finer grid does not decide cells along vertical walls
  cut by the side of a cutter; the per-move proof does, but only when the
  comparison tolerance exceeds the target mesh's `surface_error_mm` plus the
  program's own resolution. At 3 decimals a tangent pass can sit up to
  0.0007 mm into the wall, and a default CadQuery mesh carries about 0.0009 mm,
  so the default 0.001 mm cannot prove tangent walls; 0.005 mm can. Stock and
  target approximations also add up: the stock sweep's
  `SweepOptions.chord_error_mm` (0.002 mm by default) and the target's
  `surface_error_mm` both widen "possible" bounds against the comparison
  tolerance.
- Different CadQuery/OCP versions tessellate the same solid differently. That
  is why the benchmark freezes its geometry (`bench/fixtures/`).
- The Python extension targets CPython 3.12+ (abi3); the Rust MSRV is 1.88.
- Some tests exercise large inputs on purpose: run suites with memory and time
  limits.
- `main` requires CI checks by name. CI skips build jobs for Markdown-only
  changes, and skipped required jobs count as passing.

## Glossary

| Term | Meaning |
|---|---|
| Tip / controlled point | The point a program moves: the tool tip for milling; for turning, each tool's imaginary tip (or nose centre) |
| Nose centre | The centre of a turning insert's nose radius |
| Certain / possible | Lower and upper bounds of stock or of a check result |
| Certified chord | A straight move between two samples proven within tolerance of the surface along its whole length |
| Air cut | A feed move that removes no stock |
| Protected move | A probe move that stops the machine if the stylus triggers (LinuxCNC G38.3 with a trip check, Renishaw O9810) |
| Touch zone | The last `search_mm` of a probing measuring move, where the stylus is expected to touch; only there is the ball exempt from contact checks |
| Rest machining | Cutting only material left by earlier operations, read from the stock |
| R plane | The retract height of a drilling cycle |
| Lowering | Rewriting moves a controller can't take exactly (for example arcs) as checked segments |
| Intent session | The native posting state machine that tracks modal state and source moves |
| Wear compensation | Controller cutter compensation (G41/G42) carrying only the wear; camlib still plans the exact tool-centre path |
| Turret pose | A lathe's tool-change position; every tool's controlled point starts there |
| Tool change in place | A mill's tool change that moves no axis (`tool_change_position_mode="preserve_spindle"`, `ToolChangeInPlace`): the spindle gauge line stays, so the tip moves along Z by the old gauge length minus the new one |
| Export gate | Refusing to write a program while unaccepted check results remain |
