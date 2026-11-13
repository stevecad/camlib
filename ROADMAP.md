# Roadmap

The order in which camlib's open issues are closed. Read [AGENTS.md](AGENTS.md)
first: it says what camlib is, what it is not, and how to work an issue.

## How to use this roadmap

- **Work the lowest open phase first**, and inside a phase, top to bottom.
  Each phase is a GitHub milestone, so `gh issue list --milestone "Phase 2:
  Complete milling"` lists its issues.
- **Never start an issue whose dependencies are open.** Each issue lists its
  dependencies; the order below already respects them.
- **Correctness bugs jump the queue.** A bug that makes camlib produce a wrong,
  unsafe or refused-but-valid program goes into Phase 1 and is fixed before
  anything else.
- **"Any time" issues** may run in parallel with the current phase.
- **This file lists open issues only, and feature PRs don't edit it.** An
  issue is done when its PR closes it (`Closes #N`); the milestone shows
  progress. Keeping feature PRs out of this file keeps them from conflicting
  with each other here. It changes only to place new issues or change the
  order, in a roadmap-only PR, which also sweeps out closed issues.

## Why this order

camlib already covers three-axis milling, turning and plasma/laser. What's left
is ordered by real dependency, not by size:

1. **Correctness before features.** A program camlib refuses or gets wrong
   matters more than anything new.
2. **Finish one process before the next.** Milling comes first: its pieces
   (cutter compensation, tabs, avoid areas, the 3D stock) feed the others, and a
   complete, testable process is worth more than three partial ones.
3. **Then turning, then plasma/laser.** Each needs only the shared core.
4. **Multi-axis builds on foundations.** The 3D stock (#1, Phase 2) and machine
   kinematics (#2) come first; everything rotary depends on them.
5. **The optimization round is last.** Features first, then make every toolpath
   faster and better against the published benchmark (`docs/benchmarks.md`).

## Phase 1: Correctness

Refused-but-valid programs first, the most common setups first.

- #167 Roughing with AdaptiveClearing: no entry found, or the posted journal grows past 1 GB
- #160 Roughing: the shank/holder guard can't prove clearance cleared by earlier layers unless a move exactly repeats one
- #168 Tool change: no mode for a controller whose M6 doesn't move (LinuxCNC T M6 G43 H)
- #176 Machine travel: one tip box per scene doesn't fit tools of different gauge lengths

## Phase 2: Complete milling

Entry, tab and path options shops expect, then the larger additions.

- #171 Pocket and Contour: ramp entry along the path
- #172 Tabs at given positions
- #174 Trace with a side offset (an open chain of edges, cutter on one side)
- #175 Parallel: rows all cut one way
- #173 Contour following the part's outline at each depth (silhouette)
- #157 Milling: an operation that runs a caller-supplied toolpath through the same simulation, checks and posting
- #288 Milling tools: face mills and high-feed mills (real cutter profiles, depth-of-cut and entry limits)

## Phase 3: Complete turning

- #14 Turning: post profile arcs as native G2/G3 instead of chords
- #110 Tool and holder bodies from supplied models (mesh in the engine; STEP via the CadQuery adapter)
- #113 Turning: apply the insert's in-plane size tolerance (ISO 1832 d and m) to its body
- #170 Turning tools: one clearance angle for every side of the insert
- #114 Turning: solid (one-piece) boring bars with a ground head on a round bar

## Phase 4: Complete plasma/laser

- #16 Plasma/laser: automatic part nesting
- #17 Plasma/laser: common-line cutting (needs #16 for automatic placements)
- #18 Plasma/laser: marking and etching

## Phase 5: Multi-axis

In dependency order: kinematics, then indexed work, then continuous motion.

- #2 Machine kinematics: rotary axes (A/B/C), table and head configurations
- #3 4th axis: indexed (3+1) machining (needs #2)
- #5 5-axis indexed (3+2) machining with tilted work planes (needs #2)
- #4 4th axis: continuous rotary machining (wrapping and rotary toolpaths) (needs #1, #2)
- #6 5-axis simultaneous machining (tool axis control, swarf, multi-axis finishing) (needs #1, #2, #5)
- #15 Mill-turn: live tooling with C and Y axes on a lathe (needs #1, #2, #4)
- #19 Plasma/laser: bevel cutting (needs #1, #2, #6)

## Phase 6: Optimization

Every toolpath, measured against the published baseline (`bench/`). The
shared machinery comes first because most toolpaths use it. Then milling (by
use: 2D, holes, 3D), turning and thermal.

- #23 Shared toolpath machinery: linking, ordering, arc fitting and dynamics-aware feeds
- #24 Optimize Facing
- #25 Optimize Pocket and RestPocket
- #26 Optimize AdaptivePocket and adaptive clearing
- #190 AdaptivePocket: plan from the stock (engagement_front), then remove disk_front
- #191 Retire the stock_front development strategy
- #192 engagement_front: planning time and time to the first move
- #193 engagement_front: multi-resolution steering grid for small cutters on large stock
- #28 Optimize Contour (and Tabs)
- #34 Optimize TrochoidalSlot
- #35 Optimize Chamfer
- #36 Optimize Trace (engrave)
- #37 Optimize VCarve
- #29 Optimize Drilling
- #30 Optimize Bore (helical bore milling)
- #31 Optimize Reaming and fine boring
- #32 Optimize Tapping
- #33 Optimize ThreadMilling
- #27 Optimize 3D Roughing and RestRoughing
- #38 Optimize Flat-area finishing
- #39 Optimize Parallel finishing (and rest finishing)
- #46 Optimize Waterline finishing (and ramp mode)
- #47 Optimize steep-and-shallow finishing (after #39 and #46)
- #40 Optimize Spiral finishing
- #41 Optimize Radial finishing
- #42 Optimize Project finishing
- #43 Optimize Morph and flow finishing
- #44 Optimize Scallop finishing
- #45 Optimize Pencil finishing
- #118 Optimize Geodesic finishing
- #49 Optimize turning: facing (TurnFace)
- #50 Optimize turning: roughing (TurnRough)
- #51 Optimize turning: finishing (TurnFinish) (after #14)
- #52 Optimize turning: grooving (TurnGroove)
- #53 Optimize turning: parting off (TurnPartOff)
- #54 Optimize turning: threading (TurnThread)
- #55 Optimize turning: drilling on the axis (TurnDrill)
- #56 Optimize turning: boring (TurnBoreRough, TurnBoreFinish) (after #14)
- #57 Optimize turning: face grooving (TurnFaceGroove)
- #48 Optimize ThermalCut (plasma and laser)

## Any time

These can run in parallel with any phase.

- #20 Validate every controller post on real machines (start with the LinuxCNC simulator in CI)
- #21 Release: PyPI distribution name and published wheels
- #188 Checking retracts to the top of Z travel: exact setup values, or a bound?
- #194 CI: the Windows Rust test job takes about an hour, twice per merge
- #95 Siemens 808D posts: tapping (CYCLE84 / G331, ISO G84/G74)
- #97 Haas NGC post: G86 needs a source for spindle restart
- #169 Arc radius check at zero tolerance depends on the platform's hypot
- #177 FANUC 0i-MB post with several tools needs measured tip positions
