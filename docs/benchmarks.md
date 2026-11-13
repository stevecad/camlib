# Toolpath benchmarks

Measured toolpath quality and engine performance for every camlib toolpath, so
a change can be shown to be better or worse, and why. Each toolpath's
optimization issue links here.

## How these numbers are made

- **Workloads** are the regression suite's synthetic parts (`bench/workloads.py`).
  Their CAD-derived meshes and regions are frozen in `bench/fixtures/`, so every
  machine plans from identical geometry and the toolpath data is reproducible
  exactly. `python bench/compare.py BASELINE.json NEW.json` lists every toolpath
  change and, on the same machine, engine-time ratios.
  Every number describes a toolpath whose correctness is tested. Each maps
  its operations to toolpaths and issues. `dense` workloads use the same parts
  at production density (finer stepover or stepdown, or a large stock) to make
  planning time meaningful.
- **Toolpath data** is deterministic for the same inputs:
  - *Cut / rapid mm*: path length of feed moves and of rapids.
  - *Air cut*: feed moves that removed no stock in the simulation (cutting air).
    Every feed move counts, so this includes the stay-down links of adaptive
    clearing: moves at depth that take the cutter to its next pass through
    stock already cut. Those are repositioning, not a pass that missed;
    results that record engagement (`bench/engagement.py`) show them
    separately in an engagement table.
  - *Removed*: certain material removed by the operation (simulated stock).
    Tapping and single-point threading show none (and their moves count as air
    cutting): the stock model conservatively keeps thread flanks, the tap being
    modeled by its drill diameter and a thread keeping its crests.
  - *Cycle time*: `camlib.dynamics` on a declared benchmark machine
    (1000 mm/s² tangential acceleration, rapids 6000 mm/min).
    *Exact stop* stops at every block; *tangent* carries speed through tangent
    moves. Neither models controller blending or jerk.
  - *Blocks*: lines of G-code posted for a LinuxCNC-style controller at 3
    decimals (arcs not exactly writable there become checked segments).
  - *Leftover / over-cut*: material left above, and removed below, the
    workload's target part, from camlib's material verifier, as certain /
    possible bounds in mm³. Certain values are proven; possible values include
    the stock grid's and the target mesh's resolution. A workload that
    deliberately leaves material (allowances, a partial setup) shows it as
    leftover. Compare these between runs, not against zero.
- **Engine performance** is wall-clock time, the median of 3 runs each, run sequentially:
  - *Engine s* (per operation): from the operation's begin to its end in the
    stream, planning and streamed simulation together.
  - *Run s*: the whole run.
  - *First move s*: time until the first move arrived.
  - *Post s*: posting the whole program with independent decoding and checks.
  - *Peak memory*: the native engine's peak resident memory.

Reproduce: `python bench/run.py --label <name>` (needs CadQuery for the target
parts), then `python bench/report.py bench/results/<name>.json`. Results are
comparable only on the same machine; CI runs the same benchmark on every push.

## Environment

- Commit `a8df5c1ab728`
- Intel(R) Xeon(R) Silver 4112 CPU @ 2.60GHz (16 threads), Linux-6.17.0-35-generic-x86_64-with-glibc2.39
- rustc 1.100.0 (0ef1bc386 2026-11-09), Python 3.12.13

## Per toolpath

| Toolpath | Issue | Workload | Moves | Cut mm | Air cut | Rapid mm | Plunges | Retracts | Removed mm³ | Cycle s (exact / tangent) | Engine s |
|---|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| ThermalCut common line | [#17](https://github.com/stevecad/camlib/issues/17) | `plasma-common-line` (plasma) | 34 | 518 | 0 (0%) | 355 | 4 | 4 | 3035 | 18.3 / 18.0 | 0.011 |
| Facing | [#24](https://github.com/stevecad/camlib/issues/24) | `facing` (face) | 20 | 442 | 42 (10%) | 31 | 2 | 2 | 1920 | 47.1 / 47.1 | 0.007 |
| Facing | [#24](https://github.com/stevecad/camlib/issues/24) | `facing-large` (face) | 140 | 21474 | 394 (2%) | 207 | 2 | 2 | 119999 | 2153.3 / 2153.3 | 0.390 |
| Facing | [#24](https://github.com/stevecad/camlib/issues/24) | `rest-pocket` (face) | 18 | 256 | 16 (6%) | 6 | 1 | 1 | 384 | 26.9 / 26.9 | 0.001 |
| Pocket | [#25](https://github.com/stevecad/camlib/issues/25) | `pocket` (clear-pocket) | 352 | 3412 | 0 (0%) | 1137 | 48 | 48 | 8698 | 508.3 / 508.3 | 0.040 |
| Pocket | [#25](https://github.com/stevecad/camlib/issues/25) | `pocket-dense` (clear-pocket) | 9088 | 79836 | 292 (0%) | 26383 | 1216 | 1216 | 8699 | 11873.8 / 11873.4 | 0.680 |
| Pocket | [#25](https://github.com/stevecad/camlib/issues/25) | `rest-pocket` (pocket) | 30 | 99 | 8 (8%) | 44 | 4 | 4 | 124 | 16.7 / 16.7 | 0.002 |
| RestPocket | [#25](https://github.com/stevecad/camlib/issues/25) | `rest-pocket` (rest) | 82 | 206 | 117 (57%) | 37 | 4 | 8 | 2 | 26.3 / 26.1 | 0.004 |
| AdaptivePocket | [#26](https://github.com/stevecad/camlib/issues/26) | `adaptive-pocket` (adaptive) | 528 | 1122 | 519 (46%) | 51 | 6 | 6 | 472 | 144.3 / 144.0 | 0.465 |
| Roughing | [#27](https://github.com/stevecad/camlib/issues/27) | `rough-parallel` (rough) | 2927 | 37872 | 33811 (89%) | 212 | 12 | 12 | 1984 | 8900.3 / 8898.9 | 0.082 |
| Roughing | [#27](https://github.com/stevecad/camlib/issues/27) | `roughing-adaptive` (rough) | 5067 | 16332 | 3267 (20%) | 441 | 30 | 30 | 5803 | 1808.3 / 1807.4 | 9.043 |
| Roughing | [#27](https://github.com/stevecad/camlib/issues/27) | `roughing-fine-mesh` (rough) | 3295 | 7879 | 6464 (82%) | 200 | 3 | 3 | 5423 | 554.3 / 531.4 | 0.156 |
| Roughing | [#27](https://github.com/stevecad/camlib/issues/27) | `roughing-offset` (rough) | 924 | 10665 | 6089 (57%) | 250 | 5 | 8 | 5776 | 1439.8 / 1436.8 | 0.054 |
| Roughing | [#27](https://github.com/stevecad/camlib/issues/27) | `roughing-raster` (rough) | 2169 | 27414 | 21241 (77%) | 614 | 13 | 24 | 5803 | 7932.3 / 7931.6 | 0.081 |
| Contour | [#28](https://github.com/stevecad/camlib/issues/28) | `chamfer` (rough-profile) | 40 | 358 | 0 (0%) | 148 | 8 | 8 | 2243 | 55.8 / 55.7 | 0.010 |
| Contour | [#28](https://github.com/stevecad/camlib/issues/28) | `contour` (profile) | 40 | 358 | 0 (0%) | 148 | 8 | 8 | 2192 | 55.8 / 55.7 | 0.010 |
| Drilling | [#29](https://github.com/stevecad/camlib/issues/29) | `holes` (pilot) | 20 | 62 | 46 (74%) | 36 | 6 | 6 | 111 | 11.8 / 11.8 | 0.001 |
| Drilling | [#29](https://github.com/stevecad/camlib/issues/29) | `reaming` (drill) | 8 | 44 | 23 (52%) | 25 | 2 | 2 | 313 | 11.3 / 11.3 | 0.002 |
| Drilling | [#29](https://github.com/stevecad/camlib/issues/29) | `tapping` (drill) | 8 | 44 | 23 (52%) | 25 | 2 | 2 | 313 | 11.3 / 11.3 | 0.002 |
| Bore | [#30](https://github.com/stevecad/camlib/issues/30) | `holes` (finish-holes) | 42 | 370 | 44 (12%) | 36 | 2 | 2 | 1177 | 39.2 / 39.2 | 0.019 |
| Bore | [#30](https://github.com/stevecad/camlib/issues/30) | `thread-milling` (bore) | 21 | 185 | 12 (6%) | 16 | 1 | 1 | 661 | 19.6 / 19.6 | 0.041 |
| Reaming | [#31](https://github.com/stevecad/camlib/issues/31) | `reaming` (bore) | 8 | 28 | 14 (50%) | 26 | 2 | 3 | 19 | 17.4 / 17.4 | 0.001 |
| Tapping | [#32](https://github.com/stevecad/camlib/issues/32) | `tapping` (tap) | 10 | 28 | 28 (100%) | 28 | 0 | 2 | 0 | 3.5 / 3.5 | 0.001 |
| ThreadMilling | [#33](https://github.com/stevecad/camlib/issues/33) | `thread-milling` (thread) | 14 | 140 | 97 (69%) | 0 | 2 | 2 | 23 | 28.1 / 28.1 | 0.274 |
| TrochoidalSlot | [#34](https://github.com/stevecad/camlib/issues/34) | `trochoidal-slot` (slot) | 356 | 1153 | 18 (2%) | 42 | 2 | 2 | 269 | 125.4 / 121.9 | 0.021 |
| Chamfer | [#35](https://github.com/stevecad/camlib/issues/35) | `chamfer` (top-bevel) | 10 | 82 | 0 (0%) | 19 | 2 | 2 | 37 | 10.9 / 10.9 | 0.018 |
| Trace | [#36](https://github.com/stevecad/camlib/issues/36) | `trace` (engrave) | 24 | 108 | 0 (0%) | 60 | 4 | 4 | 170 | 15.5 / 15.5 | 0.003 |
| VCarve | [#37](https://github.com/stevecad/camlib/issues/37) | `vcarve` (carve) | 1986 | 97 | 21 (22%) | 17 | 1 | 1 | 107 | 28.8 / 20.5 | 2.804 |
| Flat | [#38](https://github.com/stevecad/camlib/issues/38) | `flat` (flats) | 156 | 680 | 176 (26%) | 459 | 39 | 39 | 1273 | 165.2 / 165.2 | 0.004 |
| Parallel | [#39](https://github.com/stevecad/camlib/issues/39) | `parallel` (finish-roof) | 3321 | 1723 | 543 (32%) | 427 | 41 | 41 | 1508 | 334.8 / 316.9 | 0.164 |
| Parallel | [#39](https://github.com/stevecad/camlib/issues/39) | `parallel-dense` (finish-roof) | 14023 | 8457 | 3170 (37%) | 2005 | 201 | 201 | 1511 | 1618.0 / 1555.9 | 0.773 |
| Parallel | [#39](https://github.com/stevecad/camlib/issues/39) | `rough-parallel` (parallel) | 3321 | 1805 | 625 (35%) | 508 | 41 | 41 | 292 | 368.4 / 350.5 | 0.175 |
| Spiral | [#40](https://github.com/stevecad/camlib/issues/40) | `spiral` (spiral) | 2552 | 1060 | 84 (8%) | 740 | 30 | 31 | 1505 | 225.9 / 225.5 | 0.098 |
| Spiral | [#40](https://github.com/stevecad/camlib/issues/40) | `spiral-dense` (spiral) | 12767 | 5291 | 513 (10%) | 3603 | 152 | 152 | 1510 | 1125.9 / 1123.9 | 0.450 |
| Radial | [#41](https://github.com/stevecad/camlib/issues/41) | `radial` (radial) | 610 | 477 | 3 (1%) | 215 | 24 | 24 | 1452 | 104.7 / 104.0 | 0.033 |
| Project | [#42](https://github.com/stevecad/camlib/issues/42) | `project` (project) | 97 | 61 | 0 (0%) | 24 | 2 | 2 | 413 | 11.3 / 11.2 | 0.009 |
| Morph | [#43](https://github.com/stevecad/camlib/issues/43) | `morph` (morph) | 441 | 770 | 0 (0%) | 201 | 25 | 25 | 1234 | 142.8 / 141.2 | 0.084 |
| Scallop | [#44](https://github.com/stevecad/camlib/issues/44) | `scallop` (scallop) | 2898 | 1076 | 134 (12%) | 451 | 34 | 34 | 1505 | 205.2 / 187.0 | 0.140 |
| Scallop | [#44](https://github.com/stevecad/camlib/issues/44) | `scallop-dense` (scallop) | 27025 | 6962 | 2606 (37%) | 4357 | 589 | 589 | 1511 | 1678.2 / 1565.8 | 1.975 |
| Pencil | [#45](https://github.com/stevecad/camlib/issues/45) | `pencil` (pencil) | 67 | 53 | 0 (0%) | 16 | 1 | 1 | 562 | 8.6 / 8.6 | 0.005 |
| Waterline | [#46](https://github.com/stevecad/camlib/issues/46) | `waterline` (finish-levels) | 112 | 224 | 0 (0%) | 60 | 4 | 4 | 1719 | 33.0 / 33.0 | 0.148 |
| Waterline | [#46](https://github.com/stevecad/camlib/issues/46) | `waterline-dense` (finish-levels) | 1847 | 3192 | 0 (0%) | 414 | 56 | 56 | 1730 | 460.2 / 460.2 | 1.131 |
| Steep-and-shallow | [#47](https://github.com/stevecad/camlib/issues/47) | `steep-and-shallow` (finish-shallow) | 780 | 847 | 52 (6%) | 216 | 24 | 24 | 1256 | 156.6 / 152.9 | 0.086 |
| Steep-and-shallow | [#47](https://github.com/stevecad/camlib/issues/47) | `steep-and-shallow` (finish-steep) | 866 | 1035 | 68 (7%) | 476 | 23 | 23 | 249 | 171.5 / 168.1 | 0.436 |
| ThermalCut | [#48](https://github.com/stevecad/camlib/issues/48) | `plasma` (plasma) | 66 | 757 | 0 (0%) | 397 | 6 | 6 | 4689 | 26.5 / 25.5 | 0.016 |
| TurnFace | [#49](https://github.com/stevecad/camlib/issues/49) | `lathe-face-groove` (face) | 6 | 18 | 2 (11%) | 84 | 0 | 2 | 531 | 8.4 / 8.4 | 0.006 |
| TurnFace | [#49](https://github.com/stevecad/camlib/issues/49) | `lathe-groove` (face) | 8 | 50 | 4 (8%) | 53 | 0 | 2 | 2513 | 20.8 / 20.8 | 0.004 |
| TurnFace | [#49](https://github.com/stevecad/camlib/issues/49) | `lathe-shaft` (face) | 8 | 50 | 4 (8%) | 53 | 0 | 2 | 2513 | 20.8 / 20.8 | 0.003 |
| TurnFace | [#49](https://github.com/stevecad/camlib/issues/49) | `lathe-thread` (face) | 8 | 50 | 4 (8%) | 53 | 0 | 2 | 2513 | 20.8 / 20.8 | 0.004 |
| TurnRough | [#50](https://github.com/stevecad/camlib/issues/50) | `lathe-groove` (rough) | 28 | 229 | 4 (2%) | 296 | 0 | 14 | 33227 | 59.4 / 59.4 | 0.005 |
| TurnRough | [#50](https://github.com/stevecad/camlib/issues/50) | `lathe-shaft` (rough) | 26 | 229 | 4 (2%) | 260 | 0 | 13 | 33227 | 58.8 / 58.8 | 0.005 |
| TurnRough | [#50](https://github.com/stevecad/camlib/issues/50) | `lathe-thread` (rough) | 26 | 229 | 4 (2%) | 260 | 0 | 13 | 33227 | 58.8 / 58.8 | 0.004 |
| TurnFinish | [#51](https://github.com/stevecad/camlib/issues/51) | `lathe-shaft` (finish) | 12 | 61 | 15 (25%) | 108 | 1 | 1 | 1152 | 32.1 / 32.0 | 0.003 |
| TurnFinish | [#51](https://github.com/stevecad/camlib/issues/51) | `lathe-thread` (finish) | 12 | 61 | 15 (25%) | 108 | 1 | 1 | 1152 | 32.1 / 32.0 | 0.003 |
| TurnGroove | [#52](https://github.com/stevecad/camlib/issues/52) | `lathe-groove` (groove) | 43 | 38 | 26 (68%) | 77 | 20 | 1 | 841 | 58.3 / 58.3 | 0.006 |
| TurnPartOff | [#53](https://github.com/stevecad/camlib/issues/53) | `lathe-groove` (part) | 5 | 22 | 0 (0%) | 107 | 1 | 1 | 1287 | 34.8 / 34.8 | 0.003 |
| TurnThread | [#54](https://github.com/stevecad/camlib/issues/54) | `lathe-thread` (thread) | 31 | 155 | 22 (14%) | 293 | 0 | 7 | 0 | 12.9 / 12.9 | 0.003 |
| TurnDrill | [#55](https://github.com/stevecad/camlib/issues/55) | `lathe-bore` (drill) | 12 | 34 | 0 (0%) | 194 | 3 | 4 | 5324 | 21.6 / 21.6 | 0.004 |
| TurnBore | [#56](https://github.com/stevecad/camlib/issues/56) | `lathe-bore` (bore_finish) | 8 | 29 | 3 (10%) | 88 | 1 | 3 | 332 | 19.4 / 19.4 | 0.003 |
| TurnBore | [#56](https://github.com/stevecad/camlib/issues/56) | `lathe-bore` (bore_rough) | 9 | 53 | 1 (3%) | 92 | 0 | 4 | 4009 | 22.8 / 22.8 | 0.003 |
| TurnFaceGroove | [#57](https://github.com/stevecad/camlib/issues/57) | `lathe-face-groove` (fg) | 29 | 22 | 22 (100%) | 98 | 12 | 13 | 308 | 75.0 / 75.0 | 0.004 |
| Geodesic | [#118](https://github.com/stevecad/camlib/issues/118) | `geodesic` (geodesic) | 5336 | 767 | 79 (10%) | 22 | 1 | 1 | 1506 | 131.0 / 127.9 | 0.266 |
| Geodesic | [#118](https://github.com/stevecad/camlib/issues/118) | `geodesic-boundary` (geodesic) | 713 | 712 | 5 (1%) | 53 | 3 | 3 | 1504 | 84.2 / 81.0 | 0.085 |
| Roughing | [#202](https://github.com/stevecad/camlib/issues/202) | `roughing-adaptive-large` (rough) | 3994 | 61340 | 5499 (9%) | 23424 | 174 | 174 | 2418061 | 4700.3 / 4698.0 | 11.635 |

## Engagement of adaptive clearing

Adaptive clearing holds the cutter's *contact angle*: the part of its rim that is
in material. Each operation declares a limit, acos(1 − w/r) for its radial width
w, and proves every cutting stroke within it. This table shows how close to that
limit the cutter actually runs, measured from the streamed moves alone
(`bench/engagement.py`, which reads no plan and no planner report):

- The contact angle at a point of the path is the part of the leading half of
  the rim (the 180° ahead of the direction of travel) that lies in material:
  inside the stock the operation started on, and not within the cutter radius of
  any earlier move's centre line where that move's tip was at or below this
  one's. Earlier moves are tested in closed form (lines, arcs, helices), not on
  a grid; the starting stock is the simulated stock when the operation began.
- The rim's leading half is sampled every 0.5°, so one contact arc reads within
  0.5° of its true angle, without bias. The path is evaluated at steps of 0.1 of
  the cutter radius, each weighted by its length.
- *Cutting mm*: horizontal moves of kind `cut` (wall passes included). The
  shares are of this length: *over* is more than 1° above the limit, *within
  10°* is from 10° under the limit to 1° over it, then 10–20° under, more than
  20° under, and *nothing*: cutting moves whose rim touches no material.
- *Links*: stay-down moves at depth between passes, and the share of their
  length that touches nothing. *Entry feed*: horizontal moves made at the
  entry feed because their contact is not proven within the limit, with their
  mean contact angle. Helical entries are not measured (a helix cuts a full
  slot).
- The measure goes through the platform's sin/cos/atan2, so the last digit can
  differ between platforms. `bench/compare.py` lists differences without
  failing on them.

| Toolpath | Issue | Workload | Limit ° | Cutting mm | Mean ° | Max ° | Over | Within 10° | 10–20° under | >20° under | Nothing | Link mm (touching nothing) | Entry feed mm (mean °) |
|---|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| AdaptivePocket | [#26](https://github.com/stevecad/camlib/issues/26) | `adaptive-pocket` (adaptive) | 75.5 | 540 | 49.2 | 75.0 | 0.0% | 30.0% | 30.4% | 36.8% | 2.8% | 497 (95.4%) | 16 (103.9) |
| Roughing | [#27](https://github.com/stevecad/camlib/issues/27) | `roughing-adaptive` (rough) | 41.4 | 15454 | 36.0 | 41.5 | 0.0% | 96.7% | 1.9% | 1.3% | 0.1% | 491 (57.9%) | 28 (84.2) |
| Roughing | [#202](https://github.com/stevecad/camlib/issues/202) | `roughing-adaptive-large` (rough) | 60.0 | 50013 | 52.6 | 60.0 | 0.0% | 83.3% | 9.6% | 6.7% | 0.4% | 6395 (92.7%) | 32 (87.3) |

## Per workload

| Workload | What | Moves | Cycle s (exact / tangent) | Blocks | Leftover mm³ | Over-cut mm³ | Run s | First move s | Post s | Peak memory MiB |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| `facing` | Face a 40 x 24 block 2 mm | 20 | 47.1 / 47.1 | 43 | 0.0 / 254.0 | 0.0 / 0.0 | 0.011 | 0.004 | 0.003 | 42 |
| `pocket` | Pocket around a round island (CAD region) | 352 | 508.3 / 508.3 | 375 | 3.8 / 537.6 | 0.0 / 489.6 | 0.051 | 0.010 | 0.034 | 47 |
| `rest-pocket` | Face, 4 mm pocket, 2 mm rest pocket | 130 | 69.9 / 69.6 | 169 | 0.0 / 267.0 | 0.0 / 66.0 | 0.011 | 0.004 | 0.016 | 41 |
| `adaptive-pocket` | Adaptive clearing of an L pocket | 528 | 144.3 / 144.0 | 551 | 0.0 / 481.6 | 0.0 / 56.0 | 0.471 | 0.056 | 5.131 | 387 |
| `roughing-raster` | 3D raster roughing around a frustum | 2169 | 7932.3 / 7931.6 | 2192 | 930.8 / 1468.4 | 0.0 / 83.1 | 0.094 | 0.012 | 0.285 | 391 |
| `roughing-offset` | 3D offset roughing around a frustum | 924 | 1439.8 / 1436.8 | 947 | 930.8 / 1495.2 | 0.0 / 83.1 | 0.075 | 0.019 | 0.376 | 391 |
| `roughing-adaptive` | 3D adaptive roughing around a frustum | 5067 | 1808.3 / 1807.4 | 5090 | 930.8 / 1468.4 | 0.0 / 83.1 | 9.060 | 1.881 | 54.487 | 412 |
| `roughing-fine-mesh` | 3D offset roughing of a filleted boss from a fine mesh | 3295 | 554.3 / 531.4 | 3318 | 816.3 / 1380.1 | 0.0 / 0.4 | 0.412 | 0.246 | 13.726 | 409 |
| `contour` | Outer and inner walls of an annulus | 40 | 55.8 / 55.7 | 63 | 0.0 / 825.6 | 0.0 / 430.2 | 0.015 | 0.005 | 0.010 | 386 |
| `holes` | Pilot drilling then helical bore milling | 62 | 51.0 / 50.9 | 93 | 0.0 / 413.1 | 0.0 / 125.1 | 0.027 | 0.005 | 0.014 | 390 |
| `reaming` | Drill then ream two holes | 16 | 28.7 / 28.7 | 46 | – | – | 0.011 | 0.007 | 0.006 | 44 |
| `tapping` | Drill then rigid-tap two holes | 18 | 14.8 / 14.8 | 49 | – | – | 0.010 | 0.006 | 0.007 | 44 |
| `thread-milling` | Bore then thread-mill M14 x 1.5 | 35 | 47.7 / 47.7 | 66 | – | – | 0.340 | 0.008 | 0.015 | 46 |
| `trochoidal-slot` | Straight trochoidal slot | 356 | 125.4 / 121.9 | 379 | 0.0 / 407.8 | 0.0 / 53.6 | 0.028 | 0.005 | 0.110 | 42 |
| `chamfer` | Chamfer the top edges of an annulus | 50 | 66.7 / 66.6 | 81 | 0.0 / 719.2 | 0.0 / 343.4 | 0.036 | 0.006 | 0.015 | 387 |
| `trace` | Engrave a rectangle and a hook | 24 | 15.5 / 15.5 | 47 | – | – | 0.011 | 0.007 | 0.005 | 42 |
| `vcarve` | V-carve an L-shaped letter | 1986 | 28.8 / 20.5 | 2009 | – | – | 2.829 | 0.025 | 0.180 | 48 |
| `flat` | Flat areas of a stepped part | 156 | 165.2 / 165.2 | 179 | 28.5 / 173.5 | 0.0 / 14.3 | 0.012 | 0.007 | 0.012 | 386 |
| `rough-parallel` | Rough then parallel ball finish a roof | 6248 | 9268.7 / 9249.5 | 6279 | 0.0 / 146.1 | 0.0 / 99.6 | 0.272 | 0.012 | 0.527 | 399 |
| `parallel` | Parallel finishing of a roof | 3321 | 334.8 / 316.9 | 3344 | 0.0 / 146.1 | 0.0 / 99.6 | 0.170 | 0.004 | 0.193 | 392 |
| `spiral` | Spiral finishing of a roof | 2552 | 225.9 / 225.5 | 2575 | 0.0 / 149.9 | 0.0 / 79.0 | 0.105 | 0.005 | 0.202 | 390 |
| `radial` | Radial finishing of a roof | 610 | 104.7 / 104.0 | 633 | 9.8 / 202.6 | 0.0 / 67.2 | 0.039 | 0.005 | 0.041 | 387 |
| `project` | Project curves onto a roof | 97 | 11.3 / 11.2 | 120 | 985.2 / 1241.2 | 0.0 / 7.6 | 0.014 | 0.005 | 0.008 | 385 |
| `morph` | Morph between two edges of a roof | 441 | 142.8 / 141.2 | 464 | 194.2 / 420.2 | 0.0 / 52.7 | 0.091 | 0.005 | 0.028 | 385 |
| `scallop` | Scallop finishing of a roof | 2898 | 205.2 / 187.0 | 2921 | 0.0 / 149.9 | 0.0 / 79.1 | 0.173 | 0.032 | 0.198 | 392 |
| `geodesic` | Geodesic finishing of a roof, rings from a point | 5336 | 131.0 / 127.9 | 5359 | 0.0 / 148.6 | 0.0 / 65.6 | 2.389 | 2.108 | 0.370 | 398 |
| `geodesic-boundary` | Geodesic finishing of a roof, inward from its boundary | 713 | 84.2 / 81.0 | 736 | 0.0 / 150.2 | 0.0 / 81.9 | 0.336 | 0.259 | 0.052 | 388 |
| `pencil` | Pencil the inside corner of a stepped part | 67 | 8.6 / 8.6 | 90 | 648.6 / 884.2 | 0.0 / 27.3 | 0.028 | 0.023 | 0.008 | 387 |
| `waterline` | Waterline finishing of a tapered island | 112 | 33.0 / 33.0 | 135 | 369.9 / 809.3 | 0.0 / 89.3 | 0.158 | 0.150 | 0.013 | 390 |
| `steep-and-shallow` | Steep-and-shallow finishing of a roof | 1646 | 328.1 / 321.0 | 1677 | 0.0 / 149.8 | 0.0 / 152.0 | 0.534 | 0.419 | 0.143 | 389 |
| `facing-large` | Face a 300 x 200 block 2 mm (production size) | 140 | 2153.3 / 2153.3 | 163 | – | – | 0.445 | 0.047 | 0.011 | 87 |
| `roughing-adaptive-large` | 3D adaptive roughing of a 420 x 300 mm plate with bosses (production size) | 3994 | 4700.3 / 4698.0 | 4151 | 87492.3 / 138763.0 | 0.0 / 17399.6 | 11.699 | 2.292 | 43.591 | 179 |
| `pocket-dense` | Pocket around a round island, 0.4 mm stepover, 0.25 mm stepdown | 9088 | 11873.8 / 11873.4 | 9111 | 3.8 / 536.3 | 0.0 / 489.6 | 0.690 | 0.010 | 0.566 | 60 |
| `parallel-dense` | Parallel finishing of a roof, 0.1 mm stepover | 14023 | 1618.0 / 1555.9 | 14046 | 0.0 / 143.9 | 0.0 / 101.0 | 0.778 | 0.005 | 0.814 | 417 |
| `spiral-dense` | Spiral finishing of a roof, 0.15 mm stepover | 12767 | 1125.9 / 1123.9 | 12790 | 0.0 / 144.2 | 0.0 / 86.6 | 0.456 | 0.006 | 0.777 | 416 |
| `scallop-dense` | Scallop finishing of a roof, 0.15 mm stepover | 27025 | 1678.2 / 1565.8 | 27048 | 0.0 / 143.6 | 0.0 / 81.0 | 2.998 | 1.024 | 1.801 | 446 |
| `waterline-dense` | Waterline finishing of a tapered island, 0.1 mm stepdown | 1847 | 460.2 / 460.2 | 1870 | 369.9 / 797.6 | 0.0 / 100.4 | 1.144 | 0.376 | 0.113 | 395 |
| `plasma` | Plasma-cut a plate with holes and a nested washer | 66 | 26.5 / 25.5 | 270 | – | – | 0.095 | 0.079 | 0.080 | 60 |
| `plasma-common-line` | Plasma-cut a row of four flush rectangles with common lines | 34 | 18.3 / 18.0 | 73 | – | – | 0.066 | 0.055 | 0.008 | 60 |
| `lathe-shaft` | Face, rough and finish a stepped shaft | 46 | 111.6 / 111.6 | 71 | 0.9 / 194.2 | 0.0 / 0.0 | 0.038 | 0.011 | 0.005 | 45 |
| `lathe-groove` | Rough, groove and part off a shaft | 84 | 173.2 / 173.2 | 115 | 857.3 / 1386.6 | 0.0 / 0.0 | 0.039 | 0.011 | 0.008 | 46 |
| `lathe-thread` | Turn and single-point thread M30 x 2 | 77 | 124.5 / 124.5 | 109 | 0.9 / 194.2 | 0.0 / 0.0 | 0.042 | 0.009 | 0.008 | 46 |
| `lathe-bore` | Drill on the axis, rough and finish bore | 29 | 63.8 / 63.8 | 61 | 9596.7 / 10035.8 | 0.0 / 0.0 | 0.021 | 0.008 | 0.004 | 46 |
| `lathe-face-groove` | Face and cut a face groove | 35 | 83.3 / 83.3 | 66 | 3122.0 / 3254.7 | 0.0 / 0.0 | 0.022 | 0.010 | 0.007 | 46 |
