# Roadmap

The order of camlib's remaining work. Read [AGENTS.md](AGENTS.md) for the
engineering and contribution rules. The first-release checklist is
[issue #21](https://github.com/stevecad/camlib/issues/21).

## How to use this roadmap

- Until v0.1.0, follow the release order below. The phase lists also retain
  later work; an open feature or optimization is not automatically a release gate.
- Correctness defects in the release scope take priority. Respect existing
  claims, explicit parked status and each issue's dependencies. Independent
  release/workflow tasks may proceed while an owned fix is in progress.
- Phase milestones describe the backlog. Closed work is omitted; refresh
  this file in roadmap-only PRs, never alongside toolpath implementation.
- New evidence of unsafe output or a broken advertised workflow promotes an
  otherwise deferred issue into the release gates. No tolerance or verification
  check is weakened to meet a release date.

## Ordered work for v0.1.0

Scope: three-axis milling, turning and the implemented plasma/laser operations,
with the Python `camlib` API distributed as `cam-engine`. Multi-axis and the
broad optimization round follow the first release.

The tool-body and cross-platform correctness work (#330, #245), face and
high-feed mills (#288), and the benchmark workflow (#329) are complete.
For v0.1, retract verification uses declared setup values (#188), and
turning uses the declared installed insert geometry (#222).

1. **Qualify a candidate under #21.** Select an exact main commit. Check the
   capability matrix and setup premises. Pass required CI, engine verification,
   posted-program decoding and replay, interpreter tests, and benchmark
   expectations. Rehearse clean installation and example export/refusal tests
   from the Linux x86_64/aarch64 wheels, Windows x64 wheel and sdist without
   publishing. Verify the publication configuration.
2. **Publish under #21.** Reconcile the changelog, capabilities and installation
   docs with the evidence, then publish the maintainer-selected `v0.1.0`
   commit and verify a clean public-package installation.

Voluntary machine runs belong in #20 and can follow publication. Record
controller previews, air runs and measured cuts separately for the post,
controller version and setup tested. Mark missing hardware coverage as
untested. A demonstrated correctness defect in the release scope blocks
release regardless of how it was found.

## Phase 1: Correctness

Parked, as recorded in the issue:

- #252 Turning rub-check arc pieces and stale end bins: conservative refusals;
  resume when a real job supplies a reproducer. The saved WIP over-claims
  clearance and regresses performance; it is not qualified code to merge.

## Phase 2: Complete milling

Face and high-feed milling shipped on main under #288.

## Phase 3: Complete turning

Follow-ups to the completed turning feature issues:

- #336 extends verification to a declared range of insert geometries.
- #331 Nonconvex supplied-insert comparison: subsequent work; #330 refuses
  unsupported comparisons rather than certifying a convex hull as the outline.

## Phase 4: Complete plasma/laser

Standard plasma sheet cutting and edge starts are delivered. These additions
are subsequent work, not gates on the already supported release scope:

- #306 Second-tool scribe/marker support with a declared offset and documented
  controller behavior; the edge-start item is already complete.
- #305 Laser per-feature power/feed, then corner ramps; fly cutting where
  documented by the controller. Respect the issue's internal dependencies.

## Phase 5: Multi-axis

Subsequent releases, in dependency order: kinematics, then indexed work,
then continuous motion.

- #2 Machine kinematics: rotary axes (A/B/C), table and head configurations
- #3 4th axis: indexed (3+1) machining (needs #2)
- #5 5-axis indexed (3+2) machining with tilted work planes (needs #2)
- #4 4th axis: continuous rotary machining (wrapping and rotary toolpaths) (needs #1, #2)
- #6 5-axis simultaneous machining (tool axis control, swarf, multi-axis finishing) (needs #1, #2, #5)
- #15 Mill-turn: live tooling with C and Y axes on a lathe (needs #1, #2, #4)
- #19 Plasma/laser: bevel cutting (needs #1, #2, #6)

## Phase 6: Optimization

Subsequent work in the existing operation order. #190's default planner change
is delivered; its removal of `disk_front` and pocket-specific independent
coverage remain. #191 follows #190. #328 bounds small regions on large stock;
#193 still addresses a genuinely large active front. Existing claims, including
#34 and #323, stay with their owners.

Every toolpath, measured against the published baseline (`bench/`). The
shared machinery comes first because most toolpaths use it. Then milling (by
use: 2D, holes, 3D), turning and thermal.

- #23 Shared toolpath machinery: linking, ordering, arc fitting and dynamics-aware feeds
- #24 Optimize Facing
- #25 Optimize Pocket and RestPocket
- #26 Optimize AdaptivePocket and adaptive clearing
- #190 AdaptivePocket: plan from the stock (engagement_front), then remove disk_front
- #191 Retire the stock_front development strategy
- #193 engagement_front: multi-resolution steering grid for small cutters on large stock
- #28 Optimize Contour (and Tabs)
- #34 Optimize TrochoidalSlot
- #35 Optimize Chamfer
- #36 Optimize Trace (engrave)
- #37 Optimize VCarve
- #29 Optimize Drilling
- #323 Drilling: bounded nearest-neighbour multi-start ordering (follow-up to #319; existing claim)
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

Independent release and developer-workflow tasks:

- #20 Voluntary controller-specific reports from physical machine runs.
- #21 First-release candidate qualification and publication.
