# Adding or changing a toolpath

A checklist for adding a new operation or changing an existing one. Read
[ARCHITECTURE.md](ARCHITECTURE.md) first, and follow the rules in
[AGENTS.md](../AGENTS.md). A toolpath is not done until every applicable item
below is done.

## 1. Know the reference

- Find how established CAM and the controller manuals define the operation:
  parameters, entry and exit, ordering, what is refused. Don't invent behaviour;
  cite the reference in the module's doc comment or the docs.
- Write down what the operation guarantees (for example "never gouges the part",
  "stepover never exceeds X") and how that will be checked independently.

## 2. Rust: plan the moves (milling and thermal)

1. **The operation type** in its own module under `src/` (for example
   `src/slot.rs`). Use `#[derive(Serialize, Deserialize)]` with
   `#[serde(deny_unknown_fields)]`, and give every parameter an explicit name
   and unit (`_mm`, `_deg`, `_mm_min`). Avoid defaults that stand in for machine
   or tool facts.
2. **Register it** in `pocket::Operation` (`src/pocket.rs`), and cover it in
   `id`, `tool`, `limits`, `work_offset`, `map_geometry` (for mesh inputs) and,
   for hole cycles, `declares_retract_feed`.
3. **The plan** (`XxxPlan`) with the shared interface, and register it in
   `operation::OperationPlan` (`src/operation.rs`): `prepare`, `report`,
   `prefix_events`, `suffix_events`, `next_motion_with_stock` (one move at a
   time, reading the current stock) and `finish_report`, plus `take_events` for
   events between moves (dwells, compensation).
4. **Validation and refusals.** Every invalid or impossible input is refused
   before any motion, with a message that names the problem and the fix.
5. **Machining limits** (avoid areas and containment): honour
   `Operation::limits()` the way the closest existing operation does. Region
   operations restrict their region, profiles lift over avoid areas, holes refuse
   blocked footprints. Rapids and links are also checked by the shared link guard.
6. **Cancellation.** Poll the cancel signal in long loops.
7. **Regeneration.** Check that `regeneration::semantic_operation` captures
   every parameter that changes the output (cache keys depend on it).

## 2b. Lathe operations

Lathe operations are variants of `turning::ops::TurnOperation`
(`src/turning/ops.rs`), planned by `TurnSession::operation`. Every move goes
through `Planner::emit`, which checks the insert against the part, bore and
chuck and the holder and rapids against the stock, and records the move's exact
stock change. Never bypass `emit`.

## 3. Stock and verification

- Milling and thermal moves are swept through the height-field stock by the
  stream: the tool assembly you declare is what removes material. If the
  operation removes material in a way a swept tool does not model, say so and
  model it conservatively.
- Add an **independent check** for the operation's guarantee: the material
  verifier against a target part, stock contact for rapids and links, or an
  analytic check in the tests. The check must not reuse the planner's own
  geometry code.

## 4. Posting

- If the operation only produces moves already supported (lines, arcs,
  helices, cycles), posting works as is.
- New event types or cycles need: the typed event (`src/motion.rs` and
  `python/camlib/motion.py`), dialect support in `GCodeDialect`, the post's
  words, **and the decoder**. The posted program must decode back to the planned
  motion. Controllers that can't do it must refuse, not approximate.

## 5. Python API

- A dataclass in `python/camlib/operations.py` (or `turning.py`) with `to_dict`
  matching the Rust names, exported through `_EXPORTS` in
  `python/camlib/__init__.py`.
- Keep it ordinary Python: plain values, clear errors, no hidden state.

## 6. Tests

- **Rust:** an integration test in `tests/` (for example `tests/slot.rs`,
  added to the list in `tests/main.rs`: the integration tests are one binary)
  or unit tests next to the code. Cover the guarantee, the refusals and an edge
  case.
- **Python:** `python/tests/test_<operation>.py` running it end to end through
  `Simulation` or a `Job`, with the independent check, plus posting where it
  adds new output. Reuse or add a synthetic part in `python/tests/support/`.
- Tests must fail without the change. Never weaken an existing test.

## 7. Benchmark

- Add a workload to `bench/workloads.py` that maps the operation id to its
  toolpath name and optimization issue, built from a test part. If it uses CAD
  geometry, regenerate the frozen fixtures (`CAMLIB_BENCH_REFRESH=1`) and commit
  them.
- Run `python bench/run.py --label <name>` and `bench/compare.py` against
  `bench/results/baseline.json`. Update the baseline and `docs/benchmarks.md`
  (`bench/report.py`) in the same PR, and explain the change.

## 8. Documentation and roadmap

- `docs/operations.md`: parameters, behaviour, what is refused, with the
  reference.
- `docs/library-capabilities.md`: the capability table.
- `README.md` if the feature list changes.
- Close the issue from the PR (`Closes #N`). Don't edit `ROADMAP.md`: it lists
  open issues only, and feature PRs leave it alone so they don't conflict.
