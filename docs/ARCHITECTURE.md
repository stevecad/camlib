# Architecture

How camlib fits together, for developers (human or AI) changing it. User
documentation starts at [getting-started.md](getting-started.md); the reasons
behind the design are in [DECISIONS.md](DECISIONS.md).

## Layers

| Layer | Where | Role |
|---|---|---|
| Rust engine | `src/` (crate `camlib`) | Geometry, toolpath planning, stock, verification, posting checks |
| Stream process | `src/bin/sim_stream.rs` (`cam-sim-stream`) | Runs a setup and streams binary frames on stdout |
| Python extension | `bindings/python/` (`camlib._native`) | Buffer kernels, posting intent and decoding, region engine, native receivers |
| Python API | `python/camlib/` | Jobs and setups, operations, tools, machines and posts, runs, playback, export |

The Python API is the public interface. It declares inputs and consumes results,
while the Rust side does the planning and the checking.

## One run, end to end

1. **Declare.** A `camlib.jobs.Job(id, revision, setups)` holds `Setup`s. A
   setup has stock (`StockSpec` for milling and thermal, `TurnStockSpec` for a
   lathe), tools, operations in order and a start position. A lathe setup also
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
   - Lathe: `regeneration::run_turning` plans each operation with
     `turning::ops::TurnSession` when the run reaches it. Every move carries its
     exact `TurnDelta`.
4. **Receive.** `simulation.stock_state()` picks the receiver: `StockState`
   (height field, validated by native buffer kernels) or
   `turn_state.TurnStockState` (the native `TurnReceiver`). **Every frame is
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
| 2 | Checkpoint | The full committed stock and cursor |
| 3 | Manifest | The canonical manifest (its SHA-256 binds every later frame) |
| 4 | End | The run finished; final cursor and position |
| 5 | Preview | A partial move's stock (not committed) |
| 6 | Restored | A cursor restored from a checkpoint |
| 7 | Operation | An operation's begin, events or end, with its report (JSON) |
| 8 | Verification | A check record for a move or a material stage (JSON) |
| 9 | Control | Typed control events (version 2 runs) |
| 10 | Turn chunk | One lathe move and its turning stock change |

Every frame carries the run id, revision and manifest digest. A frame from
another run or revision is refused.

## Stock models

Stock is **bounds, not samples**. Each cell holds what is *certainly* there and
what *may* be there, and removal moves both conservatively. That is how a check
can say clear, violation or uncertain honestly.

- **Height field** (`src/stock.rs`, `Stock`): an XY grid of `HeightBounds`
  (lower: material certainly up to here; upper: no material above here). Used by
  milling and thermal runs. Moves are swept with the full tool assembly
  (cutter and holder).
- **Turning** (`src/turning/stock.rs`, `TurnStock`): per z bin, bounds of the
  outer radius and of the bore radius, plus rings certainly removed inside the
  material (face grooves). Lathe moves carry their changes computed by the
  turning planner's move kernel (`Planner::emit`), which also checks the insert
  against the part, bore and chuck, and the holder and rapids against the
  stock.

Stock only loses material. A receiver refuses any change that adds material, or
whose old value doesn't match its own stock.

## Verification

- **Per move** (`src/stream_verification.rs`, records of kind 8): scene bodies
  (fixtures, protected parts as prisms), stock contact (rapids and links must not
  touch stock), machine limits.
- **Material** (`src/material.rs`, `camlib.material.Verifier`): stock against a
  target (`Target`: planar region and floor; `SurfaceTarget`: mesh). It reports
  leftover material (residual) and over-cut (gouge) as certain and possible
  volumes with per-cell findings. Lathe runs report
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
5. Arcs that can't be written exactly at the machine's precision are refused,
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
  limits (where configured); stock against the target; posted output against
  the plan (decoded); finishing chords between samples (certified, not just
  sampled endpoints).
- **Not proven:** posts have not been run on real machines (issue #20); camlib
  cannot know the real work offsets, tool lengths or clamps. Display data (stock
  meshes, previews) is approximate. "Possible" material bounds include the stock
  grid's and the target mesh's resolution.

## Traps

- Some arcs cannot be written with equal start and end radii at 3 decimals; see
  `docs/posts.md` and issue #58.
- Coarse stock cells inflate "possible" bounds. Compare certain values, or use a
  finer `cell_size_mm`.
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
| Rest machining | Cutting only material left by earlier operations, read from the stock |
| R plane | The retract height of a drilling cycle |
| Lowering | Rewriting moves a controller can't take exactly (for example arcs) as checked segments |
| Intent session | The native posting state machine that tracks modal state and source moves |
| Wear compensation | Controller cutter compensation (G41/G42) carrying only the wear; camlib still plans the exact tool-centre path |
| Turret pose | A lathe's tool-change position; every tool's controlled point starts there |
| Export gate | Refusing to write a program while unaccepted check results remain |
