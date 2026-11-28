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
    Controller-specific workloads name their post in the workload description;
    plasma-edge-starts uses the WinCNC post with independent decoding.
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

- Measured at 2026-11-28T20:40:36.006083+00:00
- Commit `43909a9ccad19ca1208f2a4fc6f10fce4b5b2b21`
- Intel(R) Xeon(R) Platinum 8370C CPU @ 2.80GHz (4 threads), Linux-7.0.0-1020-azure-x86_64-with-glibc2.43
- rustc 1.100.0 (0ef1bc386 2026-11-09), Python 3.12.16

## Per toolpath

| Toolpath | Issue | Workload | Moves | Cut mm | Air cut | Rapid mm | Plunges | Retracts | Removed mm³ | Cycle s (exact / tangent) | Engine s |
|---|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| ThermalCut common line | [#17](https://github.com/stevecad/camlib/issues/17) | `plasma-common-line` (plasma) | 34 | 518 | 0 (0%) | 355 | 4 | 4 | 3035 | 18.3 / 18.0 | 0.008 |
| Facing | [#24](https://github.com/stevecad/camlib/issues/24) | `facing` (face) | 20 | 442 | 42 (10%) | 31 | 2 | 2 | 1920 | 47.1 / 47.1 | 0.007 |
| Facing | [#24](https://github.com/stevecad/camlib/issues/24) | `facing-axis-auto` (face) | 20 | 442 | 42 (10%) | 31 | 2 | 2 | 1920 | 47.1 / 47.1 | 0.007 |
| Facing | [#24](https://github.com/stevecad/camlib/issues/24) | `facing-axis-x` (face) | 32 | 550 | 74 (13%) | 61 | 2 | 2 | 1920 | 58.4 / 58.4 | 0.007 |
| Facing | [#24](https://github.com/stevecad/camlib/issues/24) | `facing-large` (face) | 140 | 21474 | 394 (2%) | 207 | 2 | 2 | 119999 | 2153.3 / 2153.3 | 0.213 |
| Facing | [#24](https://github.com/stevecad/camlib/issues/24) | `indexable-facing` (face) | 60 | 969 | 153 (16%) | 68 | 6 | 6 | 960 | 140.3 / 140.3 | 0.089 |
| Facing | [#24](https://github.com/stevecad/camlib/issues/24) | `rest-pocket` (face) | 18 | 256 | 16 (6%) | 6 | 1 | 1 | 384 | 26.9 / 26.9 | 0.002 |
| AdaptivePocket | [#25](https://github.com/stevecad/camlib/issues/25) | `adaptive-pocket-corner-cap` (adaptive) | 267 | 593 | 271 (46%) | 18 | 3 | 3 | 472 | 80.8 / 80.7 | 0.148 |
| Pocket | [#25](https://github.com/stevecad/camlib/issues/25) | `pocket` (clear-pocket) | 352 | 3892 | 480 (12%) | 657 | 48 | 48 | 8698 | 547.2 / 547.2 | 0.021 |
| Pocket | [#25](https://github.com/stevecad/camlib/issues/25) | `pocket-dense` (clear-pocket) | 9088 | 90932 | 11388 (13%) | 15287 | 1216 | 1216 | 8699 | 12764.7 / 12764.4 | 0.415 |
| Pocket | [#25](https://github.com/stevecad/camlib/issues/25) | `rest-pocket` (pocket) | 30 | 117 | 26 (22%) | 26 | 4 | 4 | 124 | 18.0 / 18.0 | 0.002 |
| RestPocket | [#25](https://github.com/stevecad/camlib/issues/25) | `rest-pocket` (rest) | 66 | 124 | 58 (47%) | 77 | 10 | 20 | 2 | 24.6 / 24.6 | 0.004 |
| AdaptivePocket | [#26](https://github.com/stevecad/camlib/issues/26) | `adaptive-pocket` (adaptive) | 528 | 1143 | 540 (47%) | 30 | 6 | 6 | 472 | 145.7 / 145.4 | 0.290 |
| Roughing | [#27](https://github.com/stevecad/camlib/issues/27) | `indexable-roughing` (rough) | 918 | 28888 | 24107 (83%) | 313 | 6 | 6 | 829 | 8604.7 / 8604.2 | 3.972 |
| Roughing | [#27](https://github.com/stevecad/camlib/issues/27) | `indexable-roughing-exterior-feed` (rough) | 918 | 28888 | 24107 (83%) | 313 | 6 | 6 | 829 | 4992.0 / 4991.3 | 3.962 |
| Roughing | [#27](https://github.com/stevecad/camlib/issues/27) | `rough-parallel` (rough) | 2927 | 37902 | 33841 (89%) | 182 | 12 | 12 | 1984 | 8902.7 / 8901.3 | 0.049 |
| Roughing | [#27](https://github.com/stevecad/camlib/issues/27) | `roughing-adaptive` (rough) | 5067 | 16542 | 3477 (21%) | 230 | 30 | 30 | 5803 | 1824.7 / 1823.8 | 5.553 |
| Roughing | [#27](https://github.com/stevecad/camlib/issues/27) | `roughing-fine-mesh` (rough) | 3278 | 7855 | 6439 (82%) | 200 | 3 | 3 | 5423 | 550.2 / 527.3 | 0.068 |
| Roughing | [#27](https://github.com/stevecad/camlib/issues/27) | `roughing-offset` (rough) | 924 | 10701 | 6125 (57%) | 215 | 5 | 8 | 5776 | 1442.6 / 1439.6 | 0.045 |
| Roughing | [#27](https://github.com/stevecad/camlib/issues/27) | `roughing-raster` (rough) | 2169 | 27581 | 21408 (78%) | 447 | 13 | 24 | 5803 | 7945.3 / 7944.5 | 0.038 |
| Contour | [#28](https://github.com/stevecad/camlib/issues/28) | `chamfer` (rough-profile) | 40 | 414 | 56 (14%) | 92 | 8 | 8 | 2243 | 60.1 / 60.1 | 0.006 |
| Contour | [#28](https://github.com/stevecad/camlib/issues/28) | `contour` (profile) | 40 | 414 | 56 (14%) | 92 | 8 | 8 | 2192 | 60.1 / 60.1 | 0.004 |
| Drilling | [#29](https://github.com/stevecad/camlib/issues/29) | `drilling-clearance-pecks` (holes) | 38 | 126 | 109 (87%) | 41 | 12 | 12 | 184 | 18.6 / 18.6 | 0.013 |
| drilling | [#29](https://github.com/stevecad/camlib/issues/29) | `drilling-data-manual` (holes) | 8 | 14 | 0 (0%) | 41 | 2 | 2 | 184 | 6.2 / 6.2 | 0.002 |
| drilling | [#29](https://github.com/stevecad/camlib/issues/29) | `drilling-data-recommended` (holes) | 8 | 14 | 0 (0%) | 41 | 2 | 2 | 184 | 6.2 / 6.2 | 0.003 |
| Drilling | [#29](https://github.com/stevecad/camlib/issues/29) | `drilling-decreasing-pecks` (holes) | 19 | 77 | 62 (80%) | 32 | 6 | 6 | 184 | 12.9 / 12.9 | 0.008 |
| Drilling | [#29](https://github.com/stevecad/camlib/issues/29) | `drilling-g98` (holes) | 44 | 17 | 0 (0%) | 154 | 14 | 14 | 184 | 10.9 / 10.9 | 0.011 |
| Drilling | [#29](https://github.com/stevecad/camlib/issues/29) | `drilling-g99` (holes) | 44 | 17 | 0 (0%) | 136 | 14 | 15 | 184 | 10.7 / 10.7 | 0.011 |
| Drilling | [#29](https://github.com/stevecad/camlib/issues/29) | `drilling-given-order` (holes) | 60 | 186 | 141 (76%) | 164 | 18 | 18 | 525 | 35.0 / 35.0 | 0.005 |
| Drilling | [#29](https://github.com/stevecad/camlib/issues/29) | `drilling-multistart-baseline` (holes) | 90 | 279 | 211 (76%) | 223 | 27 | 27 | 582 | 52.3 / 52.2 | 0.004 |
| Drilling | [#29](https://github.com/stevecad/camlib/issues/29) | `drilling-multistart-nearest` (holes) | 90 | 279 | 211 (76%) | 204 | 27 | 27 | 582 | 52.1 / 52.1 | 0.004 |
| Drilling | [#29](https://github.com/stevecad/camlib/issues/29) | `drilling-ordered` (holes) | 60 | 186 | 141 (76%) | 85 | 18 | 18 | 525 | 34.2 / 34.2 | 0.006 |
| drilling | [#29](https://github.com/stevecad/camlib/issues/29) | `drilling-preparation-bound` (holes) | 8 | 30 | 16 (53%) | 26 | 2 | 2 | 175 | 7.8 / 7.8 | 0.003 |
| drilling | [#29](https://github.com/stevecad/camlib/issues/29) | `drilling-preparation-bound` (pilot) | 8 | 13 | 8 (57%) | 26 | 2 | 2 | 8 | 3.7 / 3.7 | 0.002 |
| drilling | [#29](https://github.com/stevecad/camlib/issues/29) | `drilling-preparation-bound` (spot) | 8 | 7 | 5 (64%) | 25 | 2 | 2 | 0 | 2.1 / 2.1 | 0.001 |
| drilling | [#29](https://github.com/stevecad/camlib/issues/29) | `drilling-preparation-manual` (holes) | 8 | 30 | 16 (53%) | 26 | 2 | 2 | 175 | 7.8 / 7.8 | 0.003 |
| drilling | [#29](https://github.com/stevecad/camlib/issues/29) | `drilling-preparation-manual` (pilot) | 8 | 13 | 8 (57%) | 26 | 2 | 2 | 8 | 3.7 / 3.7 | 0.001 |
| drilling | [#29](https://github.com/stevecad/camlib/issues/29) | `drilling-preparation-manual` (spot) | 8 | 7 | 5 (64%) | 25 | 2 | 2 | 0 | 2.1 / 2.1 | 0.002 |
| Drilling | [#29](https://github.com/stevecad/camlib/issues/29) | `drilling-r-plane-pecks` (holes) | 37 | 117 | 100 (86%) | 32 | 12 | 12 | 184 | 17.5 / 17.5 | 0.013 |
| Drilling | [#29](https://github.com/stevecad/camlib/issues/29) | `holes` (pilot) | 20 | 62 | 46 (74%) | 36 | 6 | 6 | 111 | 11.8 / 11.8 | 0.001 |
| Drilling | [#29](https://github.com/stevecad/camlib/issues/29) | `reaming` (drill) | 8 | 44 | 23 (52%) | 25 | 2 | 2 | 313 | 11.3 / 11.3 | 0.003 |
| Drilling | [#29](https://github.com/stevecad/camlib/issues/29) | `tapping` (drill) | 8 | 44 | 23 (52%) | 25 | 2 | 2 | 313 | 11.3 / 11.3 | 0.003 |
| Bore | [#30](https://github.com/stevecad/camlib/issues/30) | `holes` (finish-holes) | 42 | 370 | 44 (12%) | 36 | 2 | 2 | 1177 | 39.2 / 39.2 | 0.015 |
| Bore | [#30](https://github.com/stevecad/camlib/issues/30) | `thread-milling` (bore) | 21 | 185 | 12 (6%) | 16 | 1 | 1 | 661 | 19.6 / 19.6 | 0.029 |
| Reaming | [#31](https://github.com/stevecad/camlib/issues/31) | `reaming` (bore) | 8 | 28 | 14 (50%) | 26 | 2 | 3 | 19 | 17.4 / 17.4 | 0.001 |
| Tapping | [#32](https://github.com/stevecad/camlib/issues/32) | `tapping` (tap) | 10 | 28 | 28 (100%) | 28 | 0 | 2 | 0 | 3.5 / 3.5 | 0.001 |
| ThreadMilling | [#33](https://github.com/stevecad/camlib/issues/33) | `thread-milling` (thread) | 14 | 140 | 97 (69%) | 0 | 2 | 2 | 23 | 28.1 / 28.1 | 0.187 |
| TrochoidalSlot | [#34](https://github.com/stevecad/camlib/issues/34) | `trochoidal-slot` (slot) | 1060 | 1063 | 410 (39%) | 35 | 10 | 2 | 269 | 137.7 / 127.3 | 0.038 |
| Chamfer | [#35](https://github.com/stevecad/camlib/issues/35) | `chamfer` (top-bevel) | 10 | 89 | 7 (8%) | 12 | 2 | 2 | 37 | 11.4 / 11.4 | 0.015 |
| Trace | [#36](https://github.com/stevecad/camlib/issues/36) | `trace` (engrave) | 24 | 119 | 11 (9%) | 49 | 4 | 4 | 170 | 16.2 / 16.2 | 0.003 |
| VCarve | [#37](https://github.com/stevecad/camlib/issues/37) | `vcarve` (carve) | 1986 | 97 | 21 (22%) | 17 | 1 | 1 | 107 | 28.8 / 20.5 | 1.834 |
| Flat | [#38](https://github.com/stevecad/camlib/issues/38) | `flat` (flats) | 197 | 329 | 39 (12%) | 55 | 3 | 3 | 38 | 40.4 / 38.9 | 0.005 |
| Parallel | [#39](https://github.com/stevecad/camlib/issues/39) | `parallel` (finish-roof) | 3321 | 1723 | 543 (32%) | 427 | 41 | 41 | 1508 | 334.8 / 316.9 | 0.100 |
| Parallel | [#39](https://github.com/stevecad/camlib/issues/39) | `parallel-dense` (finish-roof) | 14023 | 8457 | 3170 (37%) | 2005 | 201 | 201 | 1511 | 1618.0 / 1555.8 | 0.525 |
| Parallel | [#39](https://github.com/stevecad/camlib/issues/39) | `rough-parallel` (parallel) | 3321 | 1805 | 625 (35%) | 508 | 41 | 41 | 292 | 368.4 / 350.5 | 0.103 |
| Spiral | [#40](https://github.com/stevecad/camlib/issues/40) | `spiral` (spiral) | 2552 | 1231 | 294 (24%) | 569 | 30 | 31 | 23 | 239.6 / 239.2 | 0.061 |
| Spiral | [#40](https://github.com/stevecad/camlib/issues/40) | `spiral-dense` (spiral) | 12767 | 6209 | 1971 (32%) | 2684 | 152 | 152 | 29 | 1199.3 / 1197.3 | 0.301 |
| Radial | [#41](https://github.com/stevecad/camlib/issues/41) | `radial` (radial) | 745 | 616 | 189 (31%) | 103 | 159 | 24 | 10 | 124.2 / 123.1 | 0.034 |
| Project | [#42](https://github.com/stevecad/camlib/issues/42) | `project` (project) | 184 | 90 | 31 (34%) | 12 | 88 | 2 | 2 | 18.9 / 18.7 | 0.010 |
| Morph | [#43](https://github.com/stevecad/camlib/issues/43) | `morph` (morph) | 466 | 1028 | 181 (18%) | 69 | 25 | 25 | 1239 | 166.3 / 164.6 | 0.072 |
| Scallop | [#44](https://github.com/stevecad/camlib/issues/44) | `scallop` (scallop) | 3481 | 1338 | 556 (42%) | 284 | 606 | 34 | 23 | 257.0 / 237.7 | 0.129 |
| Scallop | [#44](https://github.com/stevecad/camlib/issues/44) | `scallop-dense` (scallop) | 34950 | 8587 | 5191 (60%) | 3261 | 8437 | 589 | 29 | 1992.0 / 1869.0 | 1.673 |
| Pencil | [#45](https://github.com/stevecad/camlib/issues/45) | `pencil` (pencil) | 68 | 67 | 8 (12%) | 13 | 1 | 1 | 3 | 10.0 / 10.0 | 0.114 |
| Waterline | [#46](https://github.com/stevecad/camlib/issues/46) | `waterline` (finish-levels) | 251 | 421 | 33 (8%) | 19 | 73 | 1 | 58 | 78.7 / 78.7 | 0.304 |
| Waterline | [#46](https://github.com/stevecad/camlib/issues/46) | `waterline-dense` (finish-levels) | 1967 | 2942 | 45 (2%) | 19 | 120 | 1 | 63 | 347.5 / 347.0 | 2.009 |
| Steep-and-shallow | [#47](https://github.com/stevecad/camlib/issues/47) | `steep-and-shallow` (finish-shallow) | 804 | 1053 | 433 (41%) | 46 | 24 | 24 | 23 | 173.7 / 170.0 | 0.059 |
| Steep-and-shallow | [#47](https://github.com/stevecad/camlib/issues/47) | `steep-and-shallow` (finish-steep) | 2532 | 1748 | 717 (41%) | 256 | 1051 | 13 | 0 | 350.4 / 342.4 | 1.364 |
| ThermalCut | [#48](https://github.com/stevecad/camlib/issues/48) | `plasma` (plasma) | 66 | 757 | 0 (0%) | 397 | 6 | 6 | 4689 | 26.5 / 25.5 | 0.011 |
| TurnFace | [#49](https://github.com/stevecad/camlib/issues/49) | `lathe-bar-over-size` (face) | 8 | 55 | 4 (7%) | 53 | 0 | 2 | 3181 | 22.8 / 22.8 | 0.003 |
| TurnFace | [#49](https://github.com/stevecad/camlib/issues/49) | `lathe-face-groove` (face) | 6 | 18 | 2 (11%) | 84 | 0 | 2 | 531 | 8.4 / 8.4 | 0.006 |
| TurnFace | [#49](https://github.com/stevecad/camlib/issues/49) | `lathe-groove` (face) | 8 | 50 | 4 (8%) | 53 | 0 | 2 | 2513 | 20.8 / 20.8 | 0.002 |
| TurnFace | [#49](https://github.com/stevecad/camlib/issues/49) | `lathe-second-chucking` (face) | 8 | 42 | 4 (10%) | 53 | 0 | 2 | 1608 | 17.6 / 17.6 | 0.002 |
| TurnFace | [#49](https://github.com/stevecad/camlib/issues/49) | `lathe-shaft` (face) | 8 | 50 | 4 (8%) | 53 | 0 | 2 | 2513 | 20.8 / 20.8 | 0.002 |
| TurnFace | [#49](https://github.com/stevecad/camlib/issues/49) | `lathe-thread` (face) | 8 | 50 | 4 (8%) | 53 | 0 | 2 | 2513 | 20.8 / 20.8 | 0.002 |
| TurnRough | [#50](https://github.com/stevecad/camlib/issues/50) | `lathe-bar-over-size` (rough) | 30 | 250 | 5 (2%) | 285 | 0 | 15 | 42660 | 64.2 / 64.2 | 0.004 |
| TurnRough | [#50](https://github.com/stevecad/camlib/issues/50) | `lathe-groove` (rough) | 28 | 229 | 4 (2%) | 296 | 0 | 14 | 33227 | 59.4 / 59.4 | 0.003 |
| TurnRough | [#50](https://github.com/stevecad/camlib/issues/50) | `lathe-second-chucking` (rough) | 14 | 72 | 2 (3%) | 95 | 0 | 7 | 6396 | 18.9 / 18.9 | 0.002 |
| TurnRough | [#50](https://github.com/stevecad/camlib/issues/50) | `lathe-shaft` (rough) | 26 | 229 | 4 (2%) | 260 | 0 | 13 | 33227 | 58.8 / 58.8 | 0.003 |
| TurnRough | [#50](https://github.com/stevecad/camlib/issues/50) | `lathe-thread` (rough) | 26 | 229 | 4 (2%) | 260 | 0 | 13 | 33227 | 58.8 / 58.8 | 0.003 |
| TurnFinish | [#51](https://github.com/stevecad/camlib/issues/51) | `lathe-bar-over-size` (finish) | 12 | 69 | 23 (33%) | 105 | 1 | 1 | 1165 | 35.8 / 35.8 | 0.002 |
| TurnFinish | [#51](https://github.com/stevecad/camlib/issues/51) | `lathe-second-chucking` (finish) | 11 | 41 | 17 (42%) | 78 | 1 | 1 | 517 | 21.7 / 21.7 | 0.002 |
| TurnFinish | [#51](https://github.com/stevecad/camlib/issues/51) | `lathe-shaft` (finish) | 12 | 61 | 15 (25%) | 108 | 1 | 1 | 1152 | 32.1 / 32.0 | 0.002 |
| TurnFinish | [#51](https://github.com/stevecad/camlib/issues/51) | `lathe-thread` (finish) | 12 | 61 | 15 (25%) | 108 | 1 | 1 | 1152 | 32.1 / 32.0 | 0.002 |
| TurnGroove | [#52](https://github.com/stevecad/camlib/issues/52) | `lathe-groove` (groove) | 43 | 38 | 26 (68%) | 77 | 20 | 1 | 841 | 58.3 / 58.3 | 0.003 |
| TurnPartOff | [#53](https://github.com/stevecad/camlib/issues/53) | `lathe-groove` (part) | 5 | 22 | 0 (0%) | 107 | 1 | 1 | 1287 | 34.8 / 34.8 | 0.002 |
| TurnThread | [#54](https://github.com/stevecad/camlib/issues/54) | `lathe-thread` (thread) | 31 | 155 | 22 (14%) | 293 | 0 | 7 | 0 | 12.9 / 12.9 | 0.002 |
| TurnDrill | [#55](https://github.com/stevecad/camlib/issues/55) | `lathe-bore` (drill) | 12 | 34 | 0 (0%) | 194 | 3 | 4 | 5324 | 21.6 / 21.6 | 0.004 |
| TurnBore | [#56](https://github.com/stevecad/camlib/issues/56) | `lathe-bore` (bore_finish) | 8 | 29 | 3 (10%) | 88 | 1 | 3 | 332 | 19.4 / 19.4 | 0.002 |
| TurnBore | [#56](https://github.com/stevecad/camlib/issues/56) | `lathe-bore` (bore_rough) | 9 | 53 | 1 (3%) | 92 | 0 | 4 | 4009 | 22.8 / 22.8 | 0.002 |
| TurnFaceGroove | [#57](https://github.com/stevecad/camlib/issues/57) | `lathe-face-groove` (fg) | 29 | 22 | 22 (100%) | 98 | 12 | 13 | 308 | 75.0 / 75.0 | 0.003 |
| Geodesic | [#118](https://github.com/stevecad/camlib/issues/118) | `geodesic` (geodesic) | 5337 | 775 | 92 (12%) | 13 | 1 | 1 | 24 | 131.7 / 128.6 | 0.155 |
| Geodesic | [#118](https://github.com/stevecad/camlib/issues/118) | `geodesic-boundary` (geodesic) | 801 | 748 | 45 (6%) | 31 | 90 | 3 | 23 | 91.8 / 88.6 | 0.067 |
| Roughing | [#202](https://github.com/stevecad/camlib/issues/202) | `roughing-adaptive-large` (rough) | 3994 | 65864 | 10023 (15%) | 18900 | 174 | 174 | 2418061 | 4911.7 / 4909.4 | 6.903 |
| ThermalCut edge starts | [#306](https://github.com/stevecad/camlib/issues/306) | `plasma-edge-starts` (plasma) | 26 | 247 | 0 (0%) | 117 | 0 | 0 | 1462 | 8.3 / 7.6 | 0.007 |
| Flat | [#371](https://github.com/stevecad/camlib/issues/371) | `flat-cad-walls` (flats) | 100 | 283 | 53 (19%) | 95 | 9 | 5 | 35 | 38.0 / 37.9 | 0.006 |
| Facing | [#422](https://github.com/stevecad/camlib/issues/422) | `facing-clamps` (face) | 66 | 1696 | 146 (9%) | 188 | 3 | 3 | 5727 | 91.6 / 91.3 | 0.007 |

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
| AdaptivePocket | [#25](https://github.com/stevecad/camlib/issues/25) | `adaptive-pocket-corner-cap` (adaptive) | 75.5 | 270 | 49.2 | 75.0 | 0.0% | 30.0% | 30.4% | 36.8% | 2.8% | 249 (95.4%) | 8 (103.9) |
| AdaptivePocket | [#26](https://github.com/stevecad/camlib/issues/26) | `adaptive-pocket` (adaptive) | 75.5 | 540 | 49.2 | 75.0 | 0.0% | 30.0% | 30.4% | 36.8% | 2.8% | 497 (95.4%) | 16 (103.9) |
| Roughing | [#27](https://github.com/stevecad/camlib/issues/27) | `roughing-adaptive` (rough) | 41.4 | 15454 | 36.0 | 41.5 | 0.0% | 96.7% | 1.9% | 1.3% | 0.1% | 491 (57.9%) | 28 (84.2) |
| Roughing | [#202](https://github.com/stevecad/camlib/issues/202) | `roughing-adaptive-large` (rough) | 60.0 | 50013 | 52.6 | 60.0 | 0.0% | 84.2% | 9.0% | 6.4% | 0.4% | 6395 (92.7%) | 32 (87.3) |

## Per workload

| Workload | What | Moves | Cycle s (exact / tangent) | Blocks | Leftover mm³ | Over-cut mm³ | Run s | First move s | Post s | Peak memory MiB |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| `indexable-facing` | Synthetic 45-degree face mill, six insert-depth layers | 60 | 140.3 / 140.3 | 83 | 0.0 / 53.3 | 0.0 / 0.0 | 0.091 | 0.006 | 0.004 | 52 |
| `indexable-roughing` | Synthetic curved high-feed insert, layered raster roughing | 918 | 8604.7 / 8604.2 | 941 | 0.9 / 129.6 | 0.0 / 30.6 | 3.977 | 0.009 | 0.066 | 48 |
| `indexable-roughing-exterior-feed` | Synthetic high-feed raster roughing with a declared exterior link feed | 918 | 4992.0 / 4991.3 | 941 | 0.9 / 129.6 | 0.0 / 30.6 | 3.974 | 0.008 | 0.066 | 48 |
| `drilling-data-manual` | Synthetic declared drill rates written explicitly | 8 | 6.2 / 6.2 | 27 | – | – | 0.008 | 0.005 | 0.005 | 56 |
| `drilling-data-recommended` | Same synthetic rates selected from declared tool/material data | 8 | 6.2 / 6.2 | 27 | – | – | 0.008 | 0.006 | 0.003 | 56 |
| `drilling-preparation-manual` | Explicit spot, pilot and final stages without preparation metadata | 24 | 13.5 / 13.5 | 63 | – | – | 0.017 | 0.008 | 0.004 | 56 |
| `drilling-preparation-bound` | The same stages with checked source-bound preparation | 24 | 13.5 / 13.5 | 63 | – | – | 0.017 | 0.008 | 0.004 | 56 |
| `facing` | Face a 40 x 24 block 2 mm | 20 | 47.1 / 47.1 | 43 | 0.0 / 254.0 | 0.0 / 0.0 | 0.014 | 0.006 | 0.003 | 48 |
| `facing-axis-x` | Face a 24 x 40 block along X | 32 | 58.4 / 58.4 | 55 | 0.0 / 0.0 | 0.0 / 63.5 | 0.012 | 0.004 | 0.003 | 48 |
| `facing-axis-auto` | Face the same block with automatic axis selection | 20 | 47.1 / 47.1 | 43 | 0.0 / 0.0 | 0.0 / 63.5 | 0.013 | 0.005 | 0.003 | 48 |
| `facing-clamps` | Face a 100 x 60 block 1 mm round two clamps over its X- and X+ edges | 66 | 91.6 / 91.3 | 89 | 190.0 / 273.1 | 0.0 / 140.1 | 0.021 | 0.014 | 0.126 | 49 |
| `pocket` | Pocket around a round island (CAD region) | 352 | 547.2 / 547.2 | 375 | 3.8 / 537.6 | 0.0 / 489.6 | 0.027 | 0.006 | 0.018 | 53 |
| `rest-pocket` | Face, 4 mm pocket, 2 mm rest pocket | 114 | 69.4 / 69.4 | 153 | 0.0 / 267.0 | 0.0 / 66.0 | 0.013 | 0.004 | 0.007 | 47 |
| `adaptive-pocket` | Adaptive clearing of an L pocket | 528 | 145.7 / 145.4 | 551 | 0.0 / 481.6 | 0.0 / 56.0 | 0.293 | 0.026 | 3.014 | 579 |
| `adaptive-pocket-corner-cap` | L pocket with a hard cap on confined corner strokes | 267 | 80.8 / 80.7 | 290 | 0.0 / 72.0 | 0.0 / 56.0 | 0.153 | 0.025 | 1.504 | 578 |
| `roughing-raster` | 3D raster roughing around a frustum | 2169 | 7945.3 / 7944.5 | 2192 | 930.8 / 1468.4 | 0.0 / 83.1 | 0.046 | 0.007 | 0.147 | 583 |
| `roughing-offset` | 3D offset roughing around a frustum | 924 | 1442.6 / 1439.6 | 947 | 930.8 / 1495.2 | 0.0 / 83.1 | 0.059 | 0.014 | 0.214 | 583 |
| `roughing-adaptive` | 3D adaptive roughing around a frustum | 5067 | 1824.7 / 1823.8 | 5090 | 930.8 / 1468.4 | 0.0 / 83.1 | 5.560 | 1.137 | 34.094 | 605 |
| `roughing-fine-mesh` | 3D offset roughing of a filleted boss from a fine mesh | 3278 | 550.2 / 527.3 | 3301 | 816.3 / 1380.1 | 0.0 / 0.4 | 0.252 | 0.176 | 8.848 | 601 |
| `contour` | Outer and inner walls of an annulus | 40 | 60.1 / 60.1 | 63 | 0.0 / 825.6 | 0.0 / 430.2 | 0.007 | 0.003 | 0.010 | 578 |
| `holes` | Pilot drilling then helical bore milling | 62 | 51.0 / 50.9 | 93 | 0.0 / 413.1 | 0.0 / 125.1 | 0.019 | 0.003 | 0.010 | 582 |
| `drilling-given-order` | Six-hole given-order control with fixed final hole | 60 | 35.0 / 35.0 | 83 | – | – | 0.013 | 0.006 | 0.004 | 50 |
| `drilling-ordered` | Same six-hole pattern with bounded dynamics-aware 2-opt | 60 | 34.2 / 34.2 | 83 | – | – | 0.011 | 0.005 | 0.004 | 49 |
| `drilling-multistart-baseline` | Nine-hole baseline 2-opt with fixed final hole | 90 | 52.3 / 52.2 | 113 | – | – | 0.010 | 0.005 | 0.005 | 50 |
| `drilling-multistart-nearest` | Same nine-hole pattern with bounded nearest-neighbour multi-start | 90 | 52.1 / 52.1 | 113 | – | – | 0.010 | 0.006 | 0.005 | 50 |
| `drilling-clearance-pecks` | Declared fixed explicit pecks, clearance travel with scene/stock checks | 38 | 18.6 / 18.6 | 61 | – | – | 0.022 | 0.009 | 0.003 | 55 |
| `drilling-r-plane-pecks` | Same fixed explicit pecks with proven R-plane travel | 37 | 17.5 / 17.5 | 60 | – | – | 0.019 | 0.006 | 0.003 | 55 |
| `drilling-decreasing-pecks` | Same R-plane pattern with caller-declared decreasing explicit pecks | 19 | 12.9 / 12.9 | 42 | – | – | 0.016 | 0.008 | 0.003 | 55 |
| `drilling-g98` | Declared fixed canned pecks, G98 clearance return with scene/stock checks | 44 | 10.9 / 10.9 | 27 | – | – | 0.019 | 0.008 | 0.005 | 55 |
| `drilling-g99` | Same fixed canned pecks with G99 and proven R-plane travel | 44 | 10.7 / 10.7 | 28 | – | – | 0.018 | 0.006 | 0.005 | 56 |
| `reaming` | Drill then ream two holes | 16 | 28.7 / 28.7 | 46 | – | – | 0.012 | 0.006 | 0.004 | 50 |
| `tapping` | Drill then rigid-tap two holes | 18 | 14.8 / 14.8 | 49 | – | – | 0.012 | 0.006 | 0.003 | 50 |
| `thread-milling` | Bore then thread-mill M14 x 1.5 | 35 | 47.7 / 47.7 | 66 | – | – | 0.223 | 0.005 | 0.009 | 52 |
| `trochoidal-slot` | Straight trochoidal slot | 1060 | 137.7 / 127.3 | 1083 | 0.0 / 407.8 | 0.0 / 53.7 | 0.040 | 0.003 | 10.699 | 50 |
| `chamfer` | Chamfer the top edges of an annulus | 50 | 71.6 / 71.5 | 81 | 0.0 / 719.2 | 0.0 / 343.4 | 0.026 | 0.004 | 0.011 | 580 |
| `trace` | Engrave a rectangle and a hook | 24 | 16.2 / 16.2 | 47 | – | – | 0.008 | 0.004 | 0.005 | 48 |
| `vcarve` | V-carve an L-shaped letter | 1986 | 28.8 / 20.5 | 2009 | – | – | 1.846 | 0.012 | 0.084 | 55 |
| `flat` | Flat areas of a stepped part | 197 | 40.4 / 38.9 | 224 | 0.0 / 103.1 | 0.0 / 48.0 | 0.062 | 0.055 | 0.600 | 582 |
| `flat-cad-walls` | CAD floor trims and final wall contours of a stepped part | 100 | 38.0 / 37.9 | 132 | 1.3 / 105.8 | 0.0 / 37.3 | 0.019 | 0.013 | 0.090 | 581 |
| `rough-parallel` | Rough then parallel ball finish a roof | 6248 | 9271.1 / 9251.8 | 6279 | 0.0 / 146.1 | 0.0 / 99.6 | 0.156 | 0.007 | 0.337 | 592 |
| `parallel` | Parallel finishing of a roof | 3321 | 334.8 / 316.9 | 3344 | 0.0 / 146.1 | 0.0 / 99.6 | 0.103 | 0.003 | 0.136 | 586 |
| `spiral` | Spiral finishing of a roof | 2552 | 239.6 / 239.2 | 2575 | 0.0 / 149.9 | 0.0 / 79.0 | 0.072 | 0.011 | 0.114 | 589 |
| `radial` | Radial finishing of a roof | 745 | 124.2 / 123.1 | 768 | 1.1 / 162.7 | 0.0 / 67.3 | 0.043 | 0.010 | 0.034 | 584 |
| `project` | Project curves onto a roof | 184 | 18.9 / 18.7 | 207 | 5.0 / 170.8 | 0.0 / 30.9 | 0.022 | 0.010 | 0.010 | 583 |
| `morph` | Morph between two edges of a roof | 466 | 166.3 / 164.6 | 489 | 192.9 / 415.7 | 0.0 / 52.8 | 0.076 | 0.003 | 0.022 | 581 |
| `scallop` | Scallop finishing of a roof | 3481 | 257.0 / 237.7 | 3504 | 0.0 / 149.9 | 0.0 / 79.1 | 0.156 | 0.026 | 0.151 | 589 |
| `geodesic` | Geodesic finishing of a roof, rings from a point | 5337 | 131.7 / 128.6 | 5360 | 0.0 / 148.6 | 0.0 / 65.6 | 1.223 | 1.077 | 0.233 | 596 |
| `geodesic-boundary` | Geodesic finishing of a roof, inward from its boundary | 801 | 91.8 / 88.6 | 824 | 0.0 / 150.2 | 0.0 / 81.9 | 0.216 | 0.148 | 0.036 | 585 |
| `pencil` | Pencil the inside corner of a stepped part | 68 | 10.0 / 10.0 | 91 | 34.2 / 55.7 | 0.0 / 4.7 | 0.309 | 0.271 | 0.005 | 591 |
| `waterline` | Waterline finishing of a tapered island | 251 | 78.7 / 78.7 | 274 | 33.2 / 250.0 | 0.0 / 95.1 | 0.312 | 0.304 | 0.013 | 584 |
| `steep-and-shallow` | Steep-and-shallow finishing of a roof | 3336 | 524.1 / 512.4 | 3367 | 0.0 / 149.8 | 0.0 / 152.0 | 1.450 | 1.342 | 0.145 | 589 |
| `facing-large` | Face a 300 x 200 block 2 mm (production size) | 140 | 2153.3 / 2153.3 | 163 | – | – | 0.242 | 0.028 | 0.007 | 91 |
| `roughing-adaptive-large` | 3D adaptive roughing of a 420 x 300 mm plate with bosses (production size) | 3994 | 4911.7 / 4909.4 | 4151 | 87492.3 / 138763.0 | 0.0 / 17399.6 | 6.931 | 1.172 | 30.171 | 131 |
| `pocket-dense` | Pocket around a round island, 0.4 mm stepover, 0.25 mm stepdown | 9088 | 12764.7 / 12764.4 | 9111 | 3.8 / 536.3 | 0.0 / 489.6 | 0.421 | 0.006 | 0.407 | 66 |
| `parallel-dense` | Parallel finishing of a roof, 0.1 mm stepover | 14023 | 1618.0 / 1555.8 | 14046 | 0.0 / 143.9 | 0.0 / 101.0 | 0.527 | 0.003 | 0.590 | 613 |
| `spiral-dense` | Spiral finishing of a roof, 0.15 mm stepover | 12767 | 1199.3 / 1197.3 | 12790 | 0.0 / 144.2 | 0.0 / 86.6 | 0.313 | 0.012 | 0.559 | 611 |
| `scallop-dense` | Scallop finishing of a roof, 0.15 mm stepover | 34950 | 1992.0 / 1869.0 | 34973 | 0.0 / 143.6 | 0.0 / 81.0 | 2.310 | 0.636 | 1.509 | 649 |
| `waterline-dense` | Waterline finishing of a tapered island, 0.1 mm stepdown | 1967 | 347.5 / 347.0 | 1990 | 33.2 / 244.7 | 0.0 / 102.5 | 2.018 | 1.980 | 0.086 | 588 |
| `plasma` | Plasma-cut a plate with holes and a nested washer | 66 | 26.5 / 25.5 | 270 | – | – | 0.050 | 0.040 | 0.062 | 67 |
| `plasma-common-line` | Plasma-cut a row of four flush rectangles with common lines | 34 | 18.3 / 18.0 | 73 | – | – | 0.038 | 0.030 | 0.006 | 66 |
| `plasma-edge-starts` | Two squares with a sheet-edge start and an ordinary pierce, posted for WinCNC | 26 | 8.3 / 7.6 | 46 | – | – | 0.026 | 0.019 | 0.008 | 67 |
| `lathe-shaft` | Face, rough and finish a stepped shaft | 46 | 111.6 / 111.6 | 71 | 0.9 / 194.2 | 0.0 / 0.0 | 0.020 | 0.004 | 0.006 | 53 |
| `lathe-groove` | Rough, groove and part off a shaft | 84 | 173.2 / 173.2 | 115 | 857.3 / 1386.6 | 0.0 / 0.0 | 0.020 | 0.005 | 0.008 | 52 |
| `lathe-thread` | Turn and single-point thread M30 x 2 | 77 | 124.5 / 124.5 | 109 | 0.9 / 194.2 | 0.0 / 0.0 | 0.023 | 0.004 | 0.009 | 53 |
| `lathe-bore` | Drill on the axis, rough and finish bore | 29 | 63.8 / 63.8 | 61 | 9596.7 / 10035.8 | 0.0 / 0.0 | 0.019 | 0.008 | 0.004 | 53 |
| `lathe-face-groove` | Face and cut a face groove | 35 | 83.3 / 83.3 | 66 | 3122.0 / 3254.7 | 0.0 / 0.0 | 0.020 | 0.012 | 0.005 | 52 |
| `lathe-second-chucking` | Second chucking of a stepped shaft: turned around, held on its finished diameter, on the stock the first left | 33 | 58.2 / 58.2 | 58 | 2.1 / 361.3 | 0.0 / 0.0 | 0.017 | 0.005 | 0.005 | 53 |
| `lathe-bar-over-size` | First chucking from a bar well over the largest diameter: the finish feeds out up the wall roughing left | 50 | 122.7 / 122.7 | 75 | 21832.2 / 22373.6 | 0.0 / 0.0 | 0.022 | 0.005 | 0.006 | 53 |
