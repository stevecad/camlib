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
- **When an issue closes,** tick it here in the same PR. When a new issue is
  opened, it is placed in a phase by dependency.

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

No open correctness issues.

## Phase 2: Complete milling

Smaller, independent items first, then the 3D stock and what needs it.

- [ ] #12 Geodesic finishing
- [ ] #7 Probing: work offset, feature and tool-setting cycles (3-axis; tilted planes come with #5)
- [ ] #92 Deburr: ball-end and bull-nose tools
- [ ] #93 Deburr: edges next to drafted or filleted walls
- [ ] #94 Deburr: sloped and vertical edges (3-axis part needs #92; tilted tool with #5/#6)
- [ ] #107 Material check: prove ball and bull-nose ends with their real shape (shares contact work with #92)
- [ ] #1 3D stock model (tri-dexel) for undercuts and multi-axis machining
- [ ] #8 Undercut milling tools: T-slot, keyseat, lollipop and dovetail cutters (needs #1)

## Phase 3: Complete turning

- [ ] #14 Turning: post profile arcs as native G2/G3 instead of chords
- [ ] #110 Tool and holder bodies from supplied models (mesh in the engine; STEP via the CadQuery adapter)
- [ ] #113 Turning: apply the insert's in-plane size tolerance (ISO 1832 d and m) to its body
- [ ] #114 Turning: solid (one-piece) boring bars with a ground head on a round bar

## Phase 4: Complete plasma/laser

- [ ] #16 Plasma/laser: automatic part nesting
- [ ] #17 Plasma/laser: common-line cutting (needs #16 for automatic placements)
- [ ] #18 Plasma/laser: marking and etching

## Phase 5: Multi-axis

In dependency order: kinematics, then indexed work, then continuous motion.

- [ ] #2 Machine kinematics: rotary axes (A/B/C), table and head configurations
- [ ] #3 4th axis: indexed (3+1) machining (needs #2)
- [ ] #5 5-axis indexed (3+2) machining with tilted work planes (needs #2)
- [ ] #4 4th axis: continuous rotary machining (wrapping and rotary toolpaths) (needs #1, #2)
- [ ] #6 5-axis simultaneous machining (tool axis control, swarf, multi-axis finishing) (needs #1, #2, #5)
- [ ] #15 Mill-turn: live tooling with C and Y axes on a lathe (needs #1, #2, #4)
- [ ] #19 Plasma/laser: bevel cutting (needs #1, #2, #6)

## Phase 6: Optimization

Every toolpath, measured against the published baseline (`bench/`). The
shared machinery comes first because most toolpaths use it. Then milling (by
use: 2D, holes, 3D), turning and thermal.

- [ ] #23 Shared toolpath machinery: linking, ordering, arc fitting and dynamics-aware feeds
- [ ] #24 Optimize Facing
- [ ] #25 Optimize Pocket and RestPocket
- [ ] #26 Optimize AdaptivePocket and adaptive clearing
- [ ] #28 Optimize Contour (and Tabs)
- [ ] #34 Optimize TrochoidalSlot
- [ ] #35 Optimize Chamfer
- [ ] #36 Optimize Trace (engrave)
- [ ] #37 Optimize VCarve
- [ ] #29 Optimize Drilling
- [ ] #30 Optimize Bore (helical bore milling)
- [ ] #31 Optimize Reaming and fine boring
- [ ] #32 Optimize Tapping
- [ ] #33 Optimize ThreadMilling
- [ ] #27 Optimize 3D Roughing and RestRoughing
- [ ] #38 Optimize Flat-area finishing
- [ ] #39 Optimize Parallel finishing (and rest finishing)
- [ ] #46 Optimize Waterline finishing (and ramp mode)
- [ ] #47 Optimize steep-and-shallow finishing (after #39 and #46)
- [ ] #40 Optimize Spiral finishing
- [ ] #41 Optimize Radial finishing
- [ ] #42 Optimize Project finishing
- [ ] #43 Optimize Morph and flow finishing
- [ ] #44 Optimize Scallop finishing
- [ ] #45 Optimize Pencil finishing
- [ ] #49 Optimize turning: facing (TurnFace)
- [ ] #50 Optimize turning: roughing (TurnRough)
- [ ] #51 Optimize turning: finishing (TurnFinish) (after #14)
- [ ] #52 Optimize turning: grooving (TurnGroove)
- [ ] #53 Optimize turning: parting off (TurnPartOff)
- [ ] #54 Optimize turning: threading (TurnThread)
- [ ] #55 Optimize turning: drilling on the axis (TurnDrill)
- [ ] #56 Optimize turning: boring (TurnBoreRough, TurnBoreFinish) (after #14)
- [ ] #57 Optimize turning: face grooving (TurnFaceGroove)
- [ ] #48 Optimize ThermalCut (plasma and laser)

## Any time

These can run in parallel with any phase.

- [ ] #20 Validate every controller post on real machines (start with the LinuxCNC simulator in CI)
- [ ] #21 Release: PyPI distribution name and published wheels
- [ ] #64 SurfaceMesh: document the 0.1 mm `surface_error_mm` limit, and say which check failed
- [ ] #66 Undercut neck check: give the clearance and the cell size it needs
- [ ] #95 Siemens 808D posts: tapping (CYCLE84 / G331, ISO G84/G74)
- [ ] #96 FANUC 0i-MB post: M29 S rigid tapping form
- [ ] #97 Haas NGC post: G86 needs a source for spindle restart
- [ ] #108 Material check: keep the per-move proof across resumed streams; scale work budgets

## Done

- [x] #22 Optimization baseline: benchmark workloads and toolpath quality metrics
- [x] #58 Posting: adaptive orbits cannot be posted at 3 decimals (native_arcs lowering refuses closed circles)
- [x] #10 Left-hand tapping
- [x] #9 Trochoidal slots along curved paths
- [x] #13 Haas NGC post: dwell and canned drilling cycles
- [x] #11 Deburr finishing (edge breaking along sharp edges)
- [x] #65 Multi-tool jobs: post fails at the first tool change unless `Setup.tool_change` matches the tool table
- [x] #87 Project along flow_paths(along="u") fails: duplicate last row gives a zero-length link
- [x] #88 steep_and_shallow: Waterline and Parallel floor rules conflict for parts taller than the cutting length
- [x] #72 Material check stays uncertain on a correctly machined prismatic part (wall cells, through-hole interiors)
- [x] #106 Material check: per-move gouge proof for prismatic targets
- [x] #89 Roughing runtime and program length scale with mesh facets, not the allowance
- [x] #100 Turning: a thread plans and simulates but Job.post stops after the first pass
- [x] #101 Turning: a face groove as wide as the face-grooving blade fails with 'line must have nonzero length'
- [x] #102 Turning: an inside groove after facing fails with 'needs an open hole in front of it'
- [x] #103 Turning: tools need an insert thickness and out-of-plane holder bodies
