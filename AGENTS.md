# AGENTS.md

Instructions for anyone, human or AI agent, working on camlib. Read this file
and [ROADMAP.md](ROADMAP.md) before changing anything, then:

- [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md): how a run flows end to end
  (frames, stock models, verification, posting), the invariants, what is proven
  and what is not, known traps, and a glossary.
- [docs/DECISIONS.md](docs/DECISIONS.md): why camlib is built this way. Don't undo
  a decision without an issue arguing the case.
- [docs/CONTRIBUTING-TOOLPATHS.md](docs/CONTRIBUTING-TOOLPATHS.md): the checklist
  for adding or changing a toolpath.

## What camlib is

camlib is a **headless CAM engine**: a library that applications embed to turn
parts into checked machine programs.

- A **Rust engine** does the work: geometry, toolpath planning, stock simulation,
  verification and posting checks.
- An **ordinary Python API** (`python/camlib`, bindings in `bindings/python`)
  drives it: jobs, setups, tools, machines and postprocessors are plain Python
  objects.
- It covers **three-axis milling, turning (lathe) and plasma/laser cutting**. All
  three use one `Setup` and one streamed run: every move arrives with the stock it
  removes, so applications can play back, seek, cancel and save a run.
- It **checks its own output**: moves against stock, fixtures and machine limits,
  stock against the target part, and every posted program decoded again and
  compared with the plan. A program with unaccepted findings is refused at export.
- It **writes G-code through postprocessors** that are first-class code: typed
  motion, declared controller capabilities, explicit modal state, independent
  decoding.

The bar is simple: **camlib drives real machine tools, so it must not be wrong.**
A crash, gouge or wrong G-code word can destroy a machine or hurt someone.

## What camlib is not

Do not add these. Open an issue to discuss first if you believe one is needed.

- **No UI.** No viewer, web app, demo server or GUI in this repository. User
  interfaces belong in applications that use camlib. A UI here would take over
  the project's focus.
- **No machine control.** camlib produces files. It never connects to,
  streams to or reads from a machine or controller.
- **No CAD kernel and no STEP parsing.** Geometry comes from the caller (meshes,
  planar regions, turned profiles), optionally through the CadQuery adapter
  (`camlib.cadquery`).
- **No assumed machines, tools or cutting data.** camlib has no default machine,
  controller, tool, feed or speed. The caller declares them (`camlib.post.Machine`,
  tool assemblies, operation feeds). Do not add defaults that silently stand in for
  a machine's real values.
- **No application integrations or adapters** for specific products. Those live
  in the applications.
- **No 4/5-axis yet.** It is planned (see the roadmap); do not add partial
  rotary-axis behaviour outside those issues.

## Correctness rules (non-negotiable)

1. **Follow established practice, never invent it.** Controller behaviour comes
   from the controller's manual or source (LinuxCNC, FANUC, Haas, Siemens,
   Centroid, GRBL), and machining strategy from established CAM practice. Cite
   the source in code comments or docs.
2. **No silent fallbacks.** If camlib cannot do something exactly, it refuses
   with a clear message that names the problem and how to fix it. Any alternate
   behaviour must be explicit, opt-in, deterministic, verified and reported.
3. **Refuse rather than approximate.** Never relax a tolerance, drop a check or
   change what reaches the machine to make a test pass or dodge a model limit.
   Fix the model.
4. **Verify independently.** Every new strategy or post needs a check that does
   not trust the code it checks: geometric verification against the part or
   stock, or decoding the posted output back and comparing it with the plan.
5. **Posted programs contain only what the controller needs.** No redundant
   blocks or artifacts written to satisfy camlib's own model.
6. **Display data is approximate; verification is authoritative.** Never claim
   an animation, a sample or a mesh proves a sweep is safe.
7. **Measure, don't assert.** Performance claims come from the benchmark
   (`bench/`) in release mode, on the same machine, sequentially.

## How to work an issue

Every change starts from a GitHub issue, and only one person works an issue
at a time.

1. **Claim an issue before you start.** Prefer an existing open issue: known
   problems get solved before new ones are added. Take the next unclaimed issue
   in [ROADMAP.md](ROADMAP.md) order. Issues are grouped into GitHub milestones
   by phase. Work the lowest open phase first, and inside a phase the order
   listed, unless an issue says it can run in parallel. Do not start an issue
   whose dependencies are still open.
   - **Skip claimed issues.** An issue with the `claimed` label is taken; pick
     another.
   - **Claim it:** add the `claimed` label and comment that you are working on
     it (`gh issue edit N --add-label claimed`, then `gh issue comment N`).
     Claim before writing code, so nobody else starts the same problem.
   - **Nothing fits?** Open a new issue describing the problem and its
     acceptance criteria, then claim it the same way.
   - **Stopping before it is done?** Remove the label and comment what is
     finished and what is left, so someone else can pick it up.
2. **Read the whole issue.** Its "Acceptance" section is the definition of done.
   If the proposed approach turns out wrong, say so in the issue with evidence
   before taking another route.
3. **Branch and open a pull request.** `main` is protected: every change lands
   through a PR with all CI checks passing. Reference the issue (`Closes #N`).
4. **Tests come with the change.** Add tests that fail without the change, in
   Rust (`tests/`, or unit tests next to the code) and in Python (`python/tests/`)
   where the API changes. Never delete or weaken an existing test to make a change
   pass; if a test was wrong, explain why in the PR.
5. **Benchmark toolpath changes.** Run `python bench/run.py --label <name>` and
   `python bench/compare.py bench/results/baseline.json
   bench/results/<name>.json`. CI fails the benchmark job when toolpaths differ
   from the committed baseline. If a change is meant to change toolpaths, update
   the baseline in the same PR, regenerate `docs/benchmarks.md` with
   `bench/report.py`, and explain in the PR what changed and why it is better.
6. **Update the docs** (`docs/`, `README.md`) for any user-visible change.
7. **Keep PRs focused**: one issue, or a tightly related group. Note anything
   left undone in the issue rather than half-finishing it silently. Anything a
   PR leaves out gets its own issue.

## Commands

```sh
# Build the engine and Python extension (needs a Rust toolchain; MSRV 1.88).
PYO3_PYTHON=$(which python) cargo build --release --workspace --all-features

# Rust tests, lints and formatting.
cargo test --release --workspace --all-features --lib --tests
cargo test --release --doc
cargo clippy --workspace --all-targets --all-features -- -D warnings
cargo fmt --check

# Python tests (need CadQuery/OCP for the CAD-adapter tests).
PYTHONPATH=python python -m unittest discover -s python/tests

# Python lint and formatting.
ruff check python examples bench
ruff format --check python examples bench

# Runnable examples (synthetic inspection files, without a physical machine model).
PYTHONPATH=python python examples/standalone_job.py out --accept-unchecked-machine
PYTHONPATH=python python examples/cadquery_job.py out-cad --accept-unchecked-machine

# Toolpath benchmarks (need CadQuery for target parts).
python bench/run.py --label <name>
python bench/compare.py bench/results/baseline.json bench/results/<name>.json
python bench/report.py bench/results/<name>.json
```

Also run both examples in fresh output directories without
`--accept-unchecked-machine`: each must refuse export and leave no NC file. Report which operating systems were actually tested;
a local pass does not establish another platform's result.

On Windows, see [docs/windows.md](docs/windows.md) for the toolchain and the
PowerShell equivalents of these commands.

Run long test suites with a memory and time limit, using whatever your system
provides: some tests exercise large inputs on purpose, and an unbounded run
can exhaust a machine.

## Evidence before publishing

Use the existing test suite for library, CLI and installed-package behavior.
`python/tests/test_package.py` runs both example entry points, including explicit
inspection export and default refusal, in fresh directories containing spaces.
The same tests run locally and against the installed wheel; do not maintain
separate unchecked copies of these commands.

Before pushing, inspect callers of changed behavior: examples, documented
commands and package entry points as well as library tests. For a timing or
platform failure, reproduce the triggering condition deterministically and keep
a regression that fails before the fix. Never rely on a passing retry alone.

Record the tested commit (or exact dirty-tree snapshot), commands/profiles,
package mode, operating systems, results and log locations. Check again after
any source change and verify that the pushed head matches the tested source.
Report each required platform/check as passed, failed, pending or not run.
Keep an unrun or failed platform explicit; do not describe the whole change as
verified or ready to merge while a required check is unresolved. Never weaken a machine
check or silently add an acceptance to obtain a pass.

## Repository map

| Path | What it holds |
|---|---|
| `src/` | The Rust engine: geometry (`planar`, `surface`), operations (`pocket`, `adaptive`, `roughing`, `finishing`, `waterline`, `holes`, `thermal`, `turning`, …), `stock`, `stream` (streamed runs), `verification`, `material`, posting (`post*`, `dynamics`) |
| `bindings/python/` | The PyO3 extension (`camlib._native`) |
| `python/camlib/` | The Python API: jobs, operations, tools, machines and posts, simulation and playback, verification, CadQuery adapter |
| `tests/` | Rust integration tests |
| `python/tests/` | Python tests; `support/` holds synthetic test parts |
| `fixtures/` | Synthetic regression inputs |
| `examples/` | API examples and controller postprocessors (`*_post.py`) |
| `bench/` | Toolpath benchmark: workloads, runner, report, results, frozen geometry |
| `docs/` | User documentation, per-controller post notes, `benchmarks.md` |

## What does not belong in a commit

Build outputs, wheels, logs, local run directories, screenshots, editor or
agent scratch files, credentials, and personal paths or data. Keep fixtures
synthetic and small.
