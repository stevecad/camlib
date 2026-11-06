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

- Commit `d05c0df8d74d`
- Intel(R) Xeon(R) Silver 4112 CPU @ 2.60GHz (16 threads), Linux-6.17.0-35-generic-x86_64-with-glibc2.39
- rustc 1.99.1 (58cf8f9c3 2026-10-13), Python 3.12.13

## Per toolpath

| Toolpath | Issue | Workload | Moves | Cut mm | Air cut | Rapid mm | Plunges | Retracts | Removed mm³ | Cycle s (exact / tangent) | Engine s |
|---|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| Facing | [#24](https://github.com/stevecad/camlib/issues/24) | `facing` (face) | 20 | 442 | 42 (10%) | 31 | 2 | 2 | 1920 | 47.1 / 47.1 | 0.006 |
| Facing | [#24](https://github.com/stevecad/camlib/issues/24) | `facing-large` (face) | 140 | 21474 | 394 (2%) | 207 | 2 | 2 | 119999 | 2153.3 / 2153.3 | 0.345 |
| Facing | [#24](https://github.com/stevecad/camlib/issues/24) | `rest-pocket` (face) | 18 | 256 | 16 (6%) | 6 | 1 | 1 | 384 | 26.9 / 26.9 | 0.001 |
| Pocket | [#25](https://github.com/stevecad/camlib/issues/25) | `pocket` (clear-pocket) | 352 | 3412 | 0 (0%) | 1137 | 48 | 48 | 8694 | 508.3 / 508.3 | 0.026 |
| Pocket | [#25](https://github.com/stevecad/camlib/issues/25) | `pocket-dense` (clear-pocket) | 9088 | 79836 | 292 (0%) | 26383 | 1216 | 1216 | 8694 | 11873.8 / 11873.4 | 0.336 |
| Pocket | [#25](https://github.com/stevecad/camlib/issues/25) | `rest-pocket` (pocket) | 30 | 99 | 8 (8%) | 44 | 4 | 4 | 124 | 16.7 / 16.7 | 0.001 |
| RestPocket | [#25](https://github.com/stevecad/camlib/issues/25) | `rest-pocket` (rest) | 82 | 206 | 117 (57%) | 37 | 4 | 8 | 2 | 26.3 / 26.1 | 0.002 |
| AdaptivePocket | [#26](https://github.com/stevecad/camlib/issues/26) | `adaptive-pocket` (adaptive) | 3752 | 12192 | 8713 (71%) | 18 | 2 | 2 | 461 | 1262.9 / 1225.6 | 0.077 |
| Roughing | [#27](https://github.com/stevecad/camlib/issues/27) | `rough-parallel` (rough) | 2927 | 37872 | 34195 (90%) | 212 | 12 | 12 | 1984 | 8900.3 / 8898.9 | 0.051 |
| Roughing | [#27](https://github.com/stevecad/camlib/issues/27) | `roughing-adaptive` (rough) | 97138 | 164270 | 108718 (66%) | 251 | 8 | 8 | 5761 | 17533.4 / 16572.5 | 2.192 |
| Roughing | [#27](https://github.com/stevecad/camlib/issues/27) | `roughing-offset` (rough) | 924 | 10665 | 6073 (57%) | 250 | 5 | 8 | 5766 | 1439.8 / 1436.8 | 0.027 |
| Roughing | [#27](https://github.com/stevecad/camlib/issues/27) | `roughing-raster` (rough) | 2169 | 27414 | 21291 (78%) | 614 | 13 | 24 | 5766 | 7932.3 / 7931.6 | 0.047 |
| Contour | [#28](https://github.com/stevecad/camlib/issues/28) | `chamfer` (rough-profile) | 40 | 358 | 0 (0%) | 148 | 8 | 8 | 2236 | 55.8 / 55.7 | 0.009 |
| Contour | [#28](https://github.com/stevecad/camlib/issues/28) | `contour` (profile) | 40 | 358 | 0 (0%) | 148 | 8 | 8 | 2186 | 55.8 / 55.7 | 0.006 |
| Drilling | [#29](https://github.com/stevecad/camlib/issues/29) | `holes` (pilot) | 20 | 62 | 46 (74%) | 36 | 6 | 6 | 111 | 11.8 / 11.8 | 0.002 |
| Drilling | [#29](https://github.com/stevecad/camlib/issues/29) | `reaming` (drill) | 8 | 44 | 23 (52%) | 25 | 2 | 2 | 313 | 11.3 / 11.3 | 0.002 |
| Drilling | [#29](https://github.com/stevecad/camlib/issues/29) | `tapping` (drill) | 8 | 44 | 23 (52%) | 25 | 2 | 2 | 313 | 11.3 / 11.3 | 0.002 |
| Bore | [#30](https://github.com/stevecad/camlib/issues/30) | `holes` (finish-holes) | 42 | 370 | 44 (12%) | 36 | 2 | 2 | 1177 | 39.2 / 39.2 | 0.018 |
| Bore | [#30](https://github.com/stevecad/camlib/issues/30) | `thread-milling` (bore) | 21 | 185 | 12 (6%) | 16 | 1 | 1 | 660 | 19.6 / 19.6 | 0.035 |
| Reaming | [#31](https://github.com/stevecad/camlib/issues/31) | `reaming` (bore) | 8 | 28 | 14 (50%) | 26 | 2 | 3 | 19 | 17.4 / 17.4 | 0.001 |
| Tapping | [#32](https://github.com/stevecad/camlib/issues/32) | `tapping` (tap) | 10 | 28 | 28 (100%) | 28 | 0 | 2 | 0 | 3.5 / 3.5 | 0.001 |
| ThreadMilling | [#33](https://github.com/stevecad/camlib/issues/33) | `thread-milling` (thread) | 14 | 140 | 97 (69%) | 0 | 2 | 2 | 23 | 28.1 / 28.1 | 0.250 |
| TrochoidalSlot | [#34](https://github.com/stevecad/camlib/issues/34) | `trochoidal-slot` (slot) | 356 | 1153 | 15 (1%) | 42 | 2 | 2 | 269 | 125.4 / 121.9 | 0.008 |
| Chamfer | [#35](https://github.com/stevecad/camlib/issues/35) | `chamfer` (top-bevel) | 10 | 82 | 0 (0%) | 19 | 2 | 2 | 38 | 10.9 / 10.9 | 0.019 |
| Trace | [#36](https://github.com/stevecad/camlib/issues/36) | `trace` (engrave) | 24 | 108 | 0 (0%) | 60 | 4 | 4 | 170 | 15.5 / 15.5 | 0.002 |
| VCarve | [#37](https://github.com/stevecad/camlib/issues/37) | `vcarve` (carve) | 1986 | 97 | 21 (22%) | 17 | 1 | 1 | 107 | 28.8 / 20.5 | 2.487 |
| Flat | [#38](https://github.com/stevecad/camlib/issues/38) | `flat` (flats) | 156 | 680 | 176 (26%) | 459 | 39 | 39 | 1270 | 165.2 / 165.2 | 0.005 |
| Parallel | [#39](https://github.com/stevecad/camlib/issues/39) | `parallel` (finish-roof) | 3321 | 1723 | 543 (32%) | 427 | 41 | 41 | 1508 | 334.8 / 316.9 | 0.131 |
| Parallel | [#39](https://github.com/stevecad/camlib/issues/39) | `parallel-dense` (finish-roof) | 14023 | 8457 | 3170 (37%) | 2005 | 201 | 201 | 1511 | 1618.0 / 1555.9 | 0.645 |
| Parallel | [#39](https://github.com/stevecad/camlib/issues/39) | `rough-parallel` (parallel) | 3321 | 1805 | 625 (35%) | 508 | 41 | 41 | 292 | 368.4 / 350.5 | 0.142 |
| Spiral | [#40](https://github.com/stevecad/camlib/issues/40) | `spiral` (spiral) | 1658 | 1033 | 85 (8%) | 740 | 30 | 31 | 1499 | 214.3 / 214.3 | 0.103 |
| Spiral | [#40](https://github.com/stevecad/camlib/issues/40) | `spiral-dense` (spiral) | 8269 | 5159 | 532 (10%) | 3603 | 152 | 152 | 1509 | 1067.8 / 1067.7 | 0.381 |
| Radial | [#41](https://github.com/stevecad/camlib/issues/41) | `radial` (radial) | 610 | 477 | 3 (1%) | 215 | 24 | 24 | 1452 | 104.7 / 104.0 | 0.032 |
| Project | [#42](https://github.com/stevecad/camlib/issues/42) | `project` (project) | 36 | 59 | 0 (0%) | 24 | 2 | 2 | 357 | 10.4 / 10.4 | 0.005 |
| Morph | [#43](https://github.com/stevecad/camlib/issues/43) | `morph` (morph) | 441 | 770 | 0 (0%) | 201 | 25 | 25 | 1234 | 142.8 / 141.2 | 0.078 |
| Scallop | [#44](https://github.com/stevecad/camlib/issues/44) | `scallop` (scallop) | 2704 | 1056 | 131 (12%) | 451 | 34 | 34 | 1503 | 201.3 / 183.8 | 0.112 |
| Scallop | [#44](https://github.com/stevecad/camlib/issues/44) | `scallop-dense` (scallop) | 17456 | 6780 | 2419 (36%) | 4357 | 589 | 589 | 1511 | 1580.3 / 1471.1 | 1.741 |
| Pencil | [#45](https://github.com/stevecad/camlib/issues/45) | `pencil` (pencil) | 47 | 53 | 0 (0%) | 16 | 1 | 1 | 562 | 8.5 / 8.5 | 0.005 |
| Waterline | [#46](https://github.com/stevecad/camlib/issues/46) | `waterline` (finish-levels) | 112 | 224 | 0 (0%) | 60 | 4 | 4 | 1719 | 33.0 / 33.0 | 0.121 |
| Waterline | [#46](https://github.com/stevecad/camlib/issues/46) | `waterline-dense` (finish-levels) | 1847 | 3192 | 0 (0%) | 414 | 56 | 56 | 1730 | 460.2 / 460.2 | 0.973 |
| Steep-and-shallow | [#47](https://github.com/stevecad/camlib/issues/47) | `steep-and-shallow` (finish-shallow) | 780 | 847 | 52 (6%) | 215 | 24 | 24 | 1256 | 156.6 / 152.9 | 0.081 |
| Steep-and-shallow | [#47](https://github.com/stevecad/camlib/issues/47) | `steep-and-shallow` (finish-steep) | 964 | 668 | 22 (3%) | 1006 | 40 | 40 | 249 | 186.2 / 180.8 | 0.432 |
| ThermalCut | [#48](https://github.com/stevecad/camlib/issues/48) | `plasma` (plasma) | 66 | 757 | 0 (0%) | 397 | 6 | 6 | 4689 | 26.5 / 25.5 | 0.015 |
| TurnFace | [#49](https://github.com/stevecad/camlib/issues/49) | `lathe-face-groove` (face) | 6 | 18 | 2 (11%) | 84 | 0 | 2 | 531 | 8.4 / 8.4 | 0.003 |
| TurnFace | [#49](https://github.com/stevecad/camlib/issues/49) | `lathe-groove` (face) | 8 | 50 | 4 (8%) | 53 | 0 | 2 | 2513 | 20.8 / 20.8 | 0.003 |
| TurnFace | [#49](https://github.com/stevecad/camlib/issues/49) | `lathe-shaft` (face) | 8 | 50 | 4 (8%) | 53 | 0 | 2 | 2513 | 20.8 / 20.8 | 0.003 |
| TurnFace | [#49](https://github.com/stevecad/camlib/issues/49) | `lathe-thread` (face) | 8 | 50 | 4 (8%) | 53 | 0 | 2 | 2513 | 20.8 / 20.8 | 0.003 |
| TurnRough | [#50](https://github.com/stevecad/camlib/issues/50) | `lathe-groove` (rough) | 28 | 229 | 4 (2%) | 296 | 0 | 14 | 33227 | 59.4 / 59.4 | 0.004 |
| TurnRough | [#50](https://github.com/stevecad/camlib/issues/50) | `lathe-shaft` (rough) | 26 | 229 | 4 (2%) | 260 | 0 | 13 | 33227 | 58.8 / 58.8 | 0.004 |
| TurnRough | [#50](https://github.com/stevecad/camlib/issues/50) | `lathe-thread` (rough) | 26 | 229 | 4 (2%) | 260 | 0 | 13 | 33227 | 58.8 / 58.8 | 0.003 |
| TurnFinish | [#51](https://github.com/stevecad/camlib/issues/51) | `lathe-shaft` (finish) | 76 | 61 | 17 (27%) | 108 | 1 | 1 | 1152 | 32.2 / 32.2 | 0.005 |
| TurnFinish | [#51](https://github.com/stevecad/camlib/issues/51) | `lathe-thread` (finish) | 76 | 61 | 17 (27%) | 108 | 1 | 1 | 1152 | 32.2 / 32.2 | 0.005 |
| TurnGroove | [#52](https://github.com/stevecad/camlib/issues/52) | `lathe-groove` (groove) | 43 | 38 | 26 (68%) | 77 | 20 | 1 | 841 | 58.3 / 58.3 | 0.004 |
| TurnPartOff | [#53](https://github.com/stevecad/camlib/issues/53) | `lathe-groove` (part) | 5 | 22 | 0 (0%) | 107 | 1 | 1 | 1287 | 34.8 / 34.8 | 0.002 |
| TurnThread | [#54](https://github.com/stevecad/camlib/issues/54) | `lathe-thread` (thread) | 31 | 155 | 22 (14%) | 293 | 0 | 7 | 0 | 12.9 / 12.9 | 0.002 |
| TurnDrill | [#55](https://github.com/stevecad/camlib/issues/55) | `lathe-bore` (drill) | 12 | 34 | 0 (0%) | 194 | 3 | 4 | 5324 | 21.6 / 21.6 | 0.003 |
| TurnBore | [#56](https://github.com/stevecad/camlib/issues/56) | `lathe-bore` (bore_finish) | 8 | 29 | 3 (10%) | 88 | 1 | 3 | 332 | 19.4 / 19.4 | 0.003 |
| TurnBore | [#56](https://github.com/stevecad/camlib/issues/56) | `lathe-bore` (bore_rough) | 9 | 53 | 1 (3%) | 92 | 0 | 4 | 4009 | 22.8 / 22.8 | 0.003 |
| TurnFaceGroove | [#57](https://github.com/stevecad/camlib/issues/57) | `lathe-face-groove` (fg) | 29 | 22 | 22 (100%) | 98 | 12 | 13 | 308 | 75.0 / 75.0 | 0.003 |

## Per workload

| Workload | What | Moves | Cycle s (exact / tangent) | Blocks | Leftover mm³ | Over-cut mm³ | Run s | First move s | Post s | Peak memory MiB |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| `facing` | Face a 40 x 24 block 2 mm | 20 | 47.1 / 47.1 | 43 | 0.0 / 254.0 | 0.0 / 0.0 | 0.009 | 0.004 | 0.003 | 26 |
| `pocket` | Pocket around a round island (CAD region) | 352 | 508.3 / 508.3 | 383 | 3.8 / 541.5 | 0.0 / 489.6 | 0.035 | 0.009 | 0.023 | 31 |
| `rest-pocket` | Face, 4 mm pocket, 2 mm rest pocket | 130 | 69.9 / 69.6 | 169 | 0.0 / 267.0 | 0.0 / 66.0 | 0.007 | 0.004 | 0.010 | 26 |
| `adaptive-pocket` | Adaptive clearing of an L pocket | 3752 | 1262.9 / 1225.6 | refused ¹ | 4.8 / 492.8 | 0.0 / 49.6 | 0.082 | 0.004 | – | 390 |
| `roughing-raster` | 3D raster roughing around a frustum | 2169 | 7932.3 / 7931.6 | 2400 | 922.0 / 1505.5 | 0.0 / 195.5 | 0.062 | 0.011 | 0.190 | 390 |
| `roughing-offset` | 3D offset roughing around a frustum | 924 | 1439.8 / 1436.8 | 1219 | 922.0 / 1505.5 | 0.0 / 195.5 | 0.045 | 0.015 | 0.268 | 389 |
| `roughing-adaptive` | 3D adaptive roughing around a frustum | 97138 | 17533.4 / 16572.5 | refused ¹ | 922.0 / 1510.5 | 0.0 / 190.5 | 2.237 | 0.016 | – | 565 |
| `contour` | Outer and inner walls of an annulus | 40 | 55.8 / 55.7 | 79 | 0.0 / 831.6 | 0.0 / 458.2 | 0.010 | 0.004 | 0.010 | 384 |
| `holes` | Pilot drilling then helical bore milling | 62 | 51.0 / 50.9 | 111 | 0.0 / 413.1 | 0.0 / 143.3 | 0.026 | 0.006 | 0.012 | 387 |
| `reaming` | Drill then ream two holes | 16 | 28.7 / 28.7 | 46 | – | – | 0.011 | 0.006 | 0.004 | 28 |
| `tapping` | Drill then rigid-tap two holes | 18 | 14.8 / 14.8 | 49 | – | – | 0.008 | 0.005 | 0.004 | 28 |
| `thread-milling` | Bore then thread-mill M14 x 1.5 | 35 | 47.7 / 47.7 | 85 | – | – | 0.295 | 0.009 | 0.011 | 30 |
| `trochoidal-slot` | Straight trochoidal slot | 356 | 125.4 / 121.9 | 555 | 0.0 / 407.8 | 0.0 / 53.6 | 0.012 | 0.004 | 0.082 | 26 |
| `chamfer` | Chamfer the top edges of an annulus | 50 | 66.7 / 66.6 | 101 | 0.0 / 725.6 | 0.0 / 379.2 | 0.035 | 0.006 | 0.013 | 385 |
| `trace` | Engrave a rectangle and a hook | 24 | 15.5 / 15.5 | 49 | – | – | 0.007 | 0.004 | 0.004 | 26 |
| `vcarve` | V-carve an L-shaped letter | 1986 | 28.8 / 20.5 | 2009 | – | – | 2.505 | 0.018 | 0.111 | 32 |
| `flat` | Flat areas of a stepped part | 156 | 165.2 / 165.2 | 179 | 28.5 / 175.8 | 0.0 / 14.3 | 0.011 | 0.006 | 0.010 | 385 |
| `rough-parallel` | Rough then parallel ball finish a roof | 6248 | 9268.7 / 9249.5 | 6549 | 0.0 / 146.1 | 0.0 / 99.6 | 0.202 | 0.010 | 0.474 | 397 |
| `parallel` | Parallel finishing of a roof | 3321 | 334.8 / 316.9 | 3344 | 0.0 / 146.1 | 0.0 / 99.6 | 0.139 | 0.004 | 0.169 | 391 |
| `spiral` | Spiral finishing of a roof | 1658 | 214.3 / 214.3 | 1681 | 0.6 / 155.3 | 0.0 / 83.6 | 0.118 | 0.006 | 0.111 | 388 |
| `radial` | Radial finishing of a roof | 610 | 104.7 / 104.0 | 633 | 9.8 / 202.6 | 0.0 / 67.2 | 0.041 | 0.004 | 0.035 | 385 |
| `project` | Project curves onto a roof | 36 | 10.4 / 10.4 | 59 | 1052.6 / 1297.2 | 0.0 / 8.2 | 0.010 | 0.004 | 0.004 | 384 |
| `morph` | Morph between two edges of a roof | 441 | 142.8 / 141.2 | 464 | 194.2 / 420.2 | 0.0 / 52.7 | 0.083 | 0.004 | 0.025 | 385 |
| `scallop` | Scallop finishing of a roof | 2704 | 201.3 / 183.8 | 2727 | 0.0 / 151.3 | 0.0 / 80.2 | 0.163 | 0.050 | 0.151 | 389 |
| `pencil` | Pencil the inside corner of a stepped part | 47 | 8.5 / 8.5 | 70 | 648.7 / 884.3 | 0.0 / 27.4 | 0.025 | 0.020 | 0.007 | 386 |
| `waterline` | Waterline finishing of a tapered island | 112 | 33.0 / 33.0 | 135 | 367.3 / 809.3 | 0.0 / 144.4 | 0.127 | 0.122 | 0.008 | 389 |
| `steep-and-shallow` | Steep-and-shallow finishing of a roof | 1744 | 342.8 / 333.7 | 1775 | 0.0 / 149.8 | 0.0 / 100.3 | 0.542 | 0.417 | 0.114 | 389 |
| `facing-large` | Face a 300 x 200 block 2 mm (production size) | 140 | 2153.3 / 2153.3 | 163 | – | – | 0.390 | 0.045 | 0.009 | 69 |
| `pocket-dense` | Pocket around a round island, 0.4 mm stepover, 0.25 mm stepdown | 9088 | 11873.8 / 11873.4 | 9175 | 3.8 / 541.5 | 0.0 / 489.6 | 0.345 | 0.010 | 0.533 | 44 |
| `parallel-dense` | Parallel finishing of a roof, 0.1 mm stepover | 14023 | 1618.0 / 1555.9 | 14046 | 0.0 / 143.9 | 0.0 / 101.0 | 0.649 | 0.005 | 0.825 | 417 |
| `spiral-dense` | Spiral finishing of a roof, 0.15 mm stepover | 8269 | 1067.8 / 1067.7 | 8292 | 0.0 / 145.9 | 0.0 / 94.9 | 0.387 | 0.008 | 0.506 | 404 |
| `scallop-dense` | Scallop finishing of a roof, 0.15 mm stepover | 17456 | 1580.3 / 1471.1 | 17479 | 0.0 / 143.6 | 0.0 / 81.3 | 5.676 | 3.937 | 1.197 | 416 |
| `waterline-dense` | Waterline finishing of a tapered island, 0.1 mm stepdown | 1847 | 460.2 / 460.2 | 1870 | 367.2 / 797.6 | 0.0 / 155.5 | 0.986 | 0.359 | 0.105 | 393 |
| `plasma` | Plasma-cut a plate with holes and a nested washer | 66 | 26.5 / 25.5 | 294 | – | – | 0.084 | 0.069 | 0.070 | 44 |
| `lathe-shaft` | Face, rough and finish a stepped shaft | 110 | 111.7 / 111.7 | 135 | 0.9 / 194.2 | 0.0 / 0.0 | 0.021 | 0.006 | 0.008 | 28 |
| `lathe-groove` | Rough, groove and part off a shaft | 84 | 173.2 / 173.2 | 115 | 857.3 / 1386.6 | 0.0 / 0.0 | 0.024 | 0.006 | 0.006 | 28 |
| `lathe-thread` | Turn and single-point thread M30 x 2 | 141 | 124.7 / 124.7 | 173 | 0.9 / 194.2 | 0.0 / 0.0 | 0.025 | 0.006 | 0.010 | 28 |
| `lathe-bore` | Drill on the axis, rough and finish bore | 29 | 63.8 / 63.8 | 61 | 9596.7 / 10035.8 | 0.0 / 0.0 | 0.020 | 0.008 | 0.004 | 28 |
| `lathe-face-groove` | Face and cut a face groove | 35 | 83.3 / 83.3 | 66 | 3122.0 / 3254.7 | 0.0 / 0.0 | 0.014 | 0.006 | 0.004 | 28 |

¹ The posting engine refused these programs at 3 decimals:

- `adaptive-pocket`: lowering cannot preserve declared planar closure at controller precision.
- `roughing-adaptive`: lowering cannot preserve declared planar closure at controller precision.
