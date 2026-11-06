# Design decisions

Why camlib is built the way it is. These decisions are deliberate: don't undo
one without opening an issue that argues the case. Each entry gives the
decision, the reason and what follows from it.

## 1. A headless engine, with no UI in this repository

**Why:** camlib is a library that applications embed. A UI here would become
what people judge and work on, and pull the project away from the engine.
**So:** no viewers, demo servers or GUIs here; they live in applications.
Examples stay small API scripts.

## 2. A Rust engine with an ordinary Python API

**Why:** toolpath generation and verification must be fast; CAM users and
applications script in Python. **So:** heavy work lives in Rust (`src/`), and
the Python API (`python/camlib`) is plain dataclasses and functions over it.
Python never reimplements engine rules; it declares inputs and checks results.

## 3. One `Setup` and one stream for every machine type

**Why:** milling, thermal cutting and turning all need the same things:
streamed moves with their stock, playback, posting, verification and export.
Separate paths drift apart and double the work. **So:** a lathe setup is an
ordinary `Setup` with turning stock (there is no separate lathe job type), and
every machine type uses the same frames, receivers, playback, posting and export
gate. New machine types extend these; they don't add side paths.

## 4. The engine assumes no machine, tool or cutting data

**Why:** camlib supports many controllers and machine types, and a default that
is right for one is wrong, and dangerous, for another. **So:** machines declare
their dialect, units, decimals, limits, tools and capabilities
(`camlib.post.Machine`), and operations declare feeds and speeds. There are no
silent defaults that stand in for those facts.

## 5. Refuse rather than approximate; no silent fallbacks

**Why:** camlib drives real machine tools. An output that is quietly different
from what was asked for is worse than an error. **So:** impossible or unsupported
requests are refused with a message naming the problem and the fix. Alternate
behaviour is explicit, opt-in, deterministic, verified and reported (for example
`LoweringOptions(native_arcs=True)`).

## 6. Independent verification, including decoding posted output

**Why:** a check that trusts the code it checks proves nothing. **So:** moves
are checked against the stock, scene and limits by separate code; the stock is
compared with the target part by the material verifier; every posted program is
decoded by a decoder that doesn't trust the post and compared with the plan.

## 7. An export gate on check results

**Why:** findings in a report are easy to miss; a program that reaches the
machine is not. **So:** exporting (`Job.post_nc`, `PostedRun.export`) refuses a
program with unaccepted `violation` or `uncertain` results, and nothing is
written. Accepting is explicit and per kind, and exports list the checks that
were not configured.

## 8. Stock as certain and possible bounds

**Why:** a sampled stock can't tell "clear" from "not checked". **So:** every
stock cell holds bounds; removal is conservative in both directions; checks
report clear, violation or uncertain. Display meshes are approximate and never
used for verification.

## 9. Streamed runs with stock per move

**Why:** applications need to show material removal as it is planned, pause,
seek (also within a move), cancel, and save runs. **So:** runs stream every move
with its stock change; receivers re-check every frame; checkpoints make seeking
cheap; previews show partial moves without committing them.

## 10. Wear compensation means controller compensation codes, never a different path

**Why:** operators trim size at the machine with wear offsets; the planned path
must still be exact. **So:** camlib plans the exact tool-centre path and, with
`compensation="wear"`, only adds G41/G42 and G40 at the declared places, with the
register holding the wear. It doesn't track tool wear over time (that needs a
machine connection, which camlib doesn't have).

## 11. No legacy code before 1.0

**Why:** an unreleased engine has no users to keep compatible. Old and new paths
side by side double the testing and confuse contributors. **So:** a replaced
design is removed, not kept beside its replacement. There is one posting path,
one job model and one stream.

## 12. No count caps; only transport limits

**Why:** caps on setups, operations, tools or bodies are arbitrary and silently
limit real jobs. **So:** counts are limited only by the data formats and memory.
Transport limits (frame sizes, batch sizes) are documented. Inputs must be
finite collections; nothing is truncated.

## 13. A public benchmark with frozen geometry

**Why:** optimizations must be shown, not claimed, and comparisons must not
depend on whose machine ran them. **So:** `bench/` measures toolpath data and
engine time for every toolpath; CAD-derived geometry is frozen in
`bench/fixtures/`; CI compares toolpath data with the committed baseline on
every change; timing compares only on the same machine.

## 14. Features first, then the optimization round

**Why:** a complete, correct engine is worth more than a fast partial one, and
optimizing code that is still changing is wasted effort. **So:** the roadmap
finishes correctness, then milling, turning and plasma/laser, then multi-axis,
and optimizes every toolpath last, against the benchmark.

## 15. Multi-axis waits for its foundations

**Why:** rotary axes touch the stock model, kinematics, verification and
posting; partial support would be unsafe. **So:** 4/5-axis work starts with the 3D
stock model (#1) and declared machine kinematics (#2), in roadmap order.

## 16. Pull requests only, with green CI

**Why:** every change must be tested before it reaches `main`. **So:** `main`
is protected for everyone. Changes land through PRs with the required checks
passing; Markdown-only changes skip the build jobs.

## 17. Toolchains: current stable, declared minimums

**Why:** users build with current tools, on Linux and Windows. **So:** CI builds
and tests on current stable Rust on Linux and native Windows x64, checks the
declared minimum (MSRV 1.88), and installs and tests a Windows wheel; the Python
extension targets CPython 3.12+ (abi3); lint rules are explicit so tool upgrades
don't change what is checked. Other platforms (macOS, ARM64) are welcome
contributions with their own CI.
