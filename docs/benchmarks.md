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

- Commit `a98a3d5e7917`
- Intel(R) Xeon(R) Silver 4112 CPU @ 2.60GHz (16 threads), Linux-6.17.0-35-generic-x86_64-with-glibc2.39
- rustc 1.99.1 (58cf8f9c3 2026-10-13), Python 3.12.13

## Per toolpath

| Toolpath | Issue | Workload | Moves | Cut mm | Air cut | Rapid mm | Plunges | Retracts | Removed mm³ | Cycle s (exact / tangent) | Engine s |
|---|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| Facing | [#24](https://github.com/stevecad/camlib/issues/24) | `facing` (face) | 20 | 442 | 42 (10%) | 31 | 2 | 2 | 1920 | 47.1 / 47.1 | 0.006 |
| Facing | [#24](https://github.com/stevecad/camlib/issues/24) | `facing-large` (face) | 140 | 21474 | 394 (2%) | 207 | 2 | 2 | 119999 | 2153.3 / 2153.3 | 0.359 |
| Facing | [#24](https://github.com/stevecad/camlib/issues/24) | `rest-pocket` (face) | 18 | 256 | 16 (6%) | 6 | 1 | 1 | 384 | 26.9 / 26.9 | 0.001 |
| Pocket | [#25](https://github.com/stevecad/camlib/issues/25) | `pocket` (clear-pocket) | 352 | 3412 | 0 (0%) | 1137 | 48 | 48 | 8694 | 508.3 / 508.3 | 0.025 |
| Pocket | [#25](https://github.com/stevecad/camlib/issues/25) | `pocket-dense` (clear-pocket) | 9088 | 79836 | 292 (0%) | 26383 | 1216 | 1216 | 8694 | 11873.8 / 11873.4 | 0.335 |
| Pocket | [#25](https://github.com/stevecad/camlib/issues/25) | `rest-pocket` (pocket) | 30 | 99 | 8 (8%) | 44 | 4 | 4 | 124 | 16.7 / 16.7 | 0.001 |
| RestPocket | [#25](https://github.com/stevecad/camlib/issues/25) | `rest-pocket` (rest) | 82 | 206 | 117 (57%) | 37 | 4 | 8 | 2 | 26.3 / 26.1 | 0.002 |
| AdaptivePocket | [#26](https://github.com/stevecad/camlib/issues/26) | `adaptive-pocket` (adaptive) | 3752 | 12192 | 8713 (71%) | 18 | 2 | 2 | 461 | 1262.9 / 1225.6 | 0.068 |
| Roughing | [#27](https://github.com/stevecad/camlib/issues/27) | `rough-parallel` (rough) | 2927 | 37872 | 34195 (90%) | 212 | 12 | 12 | 1984 | 8900.3 / 8898.9 | 0.065 |
| Roughing | [#27](https://github.com/stevecad/camlib/issues/27) | `roughing-adaptive` (rough) | 97138 | 164270 | 108718 (66%) | 251 | 8 | 8 | 5761 | 17533.4 / 16572.5 | 2.688 |
| Roughing | [#27](https://github.com/stevecad/camlib/issues/27) | `roughing-fine-mesh` (rough) | 3295 | 7879 | 6462 (82%) | 200 | 3 | 3 | 5418 | 554.3 / 531.4 | 0.080 |
| Roughing | [#27](https://github.com/stevecad/camlib/issues/27) | `roughing-offset` (rough) | 924 | 10665 | 6073 (57%) | 250 | 5 | 8 | 5766 | 1439.8 / 1436.8 | 0.028 |
| Roughing | [#27](https://github.com/stevecad/camlib/issues/27) | `roughing-raster` (rough) | 2169 | 27414 | 21291 (78%) | 614 | 13 | 24 | 5766 | 7932.3 / 7931.6 | 0.043 |
| Contour | [#28](https://github.com/stevecad/camlib/issues/28) | `chamfer` (rough-profile) | 40 | 358 | 0 (0%) | 148 | 8 | 8 | 2236 | 55.8 / 55.7 | 0.009 |
| Contour | [#28](https://github.com/stevecad/camlib/issues/28) | `contour` (profile) | 40 | 358 | 0 (0%) | 148 | 8 | 8 | 2186 | 55.8 / 55.7 | 0.006 |
| Drilling | [#29](https://github.com/stevecad/camlib/issues/29) | `holes` (pilot) | 20 | 62 | 46 (74%) | 36 | 6 | 6 | 111 | 11.8 / 11.8 | 0.001 |
| Drilling | [#29](https://github.com/stevecad/camlib/issues/29) | `reaming` (drill) | 8 | 44 | 23 (52%) | 25 | 2 | 2 | 313 | 11.3 / 11.3 | 0.002 |
| Drilling | [#29](https://github.com/stevecad/camlib/issues/29) | `tapping` (drill) | 8 | 44 | 23 (52%) | 25 | 2 | 2 | 313 | 11.3 / 11.3 | 0.002 |
| Bore | [#30](https://github.com/stevecad/camlib/issues/30) | `holes` (finish-holes) | 42 | 370 | 44 (12%) | 36 | 2 | 2 | 1177 | 39.2 / 39.2 | 0.017 |
| Bore | [#30](https://github.com/stevecad/camlib/issues/30) | `thread-milling` (bore) | 21 | 185 | 12 (6%) | 16 | 1 | 1 | 660 | 19.6 / 19.6 | 0.036 |
| Reaming | [#31](https://github.com/stevecad/camlib/issues/31) | `reaming` (bore) | 8 | 28 | 14 (50%) | 26 | 2 | 3 | 19 | 17.4 / 17.4 | 0.001 |
| Tapping | [#32](https://github.com/stevecad/camlib/issues/32) | `tapping` (tap) | 10 | 28 | 28 (100%) | 28 | 0 | 2 | 0 | 3.5 / 3.5 | 0.001 |
| ThreadMilling | [#33](https://github.com/stevecad/camlib/issues/33) | `thread-milling` (thread) | 14 | 140 | 97 (69%) | 0 | 2 | 2 | 23 | 28.1 / 28.1 | 0.264 |
| TrochoidalSlot | [#34](https://github.com/stevecad/camlib/issues/34) | `trochoidal-slot` (slot) | 356 | 1153 | 15 (1%) | 42 | 2 | 2 | 269 | 125.4 / 121.9 | 0.009 |
| Chamfer | [#35](https://github.com/stevecad/camlib/issues/35) | `chamfer` (top-bevel) | 10 | 82 | 0 (0%) | 19 | 2 | 2 | 38 | 10.9 / 10.9 | 0.018 |
| Trace | [#36](https://github.com/stevecad/camlib/issues/36) | `trace` (engrave) | 24 | 108 | 0 (0%) | 60 | 4 | 4 | 170 | 15.5 / 15.5 | 0.002 |
| VCarve | [#37](https://github.com/stevecad/camlib/issues/37) | `vcarve` (carve) | 1986 | 97 | 21 (22%) | 17 | 1 | 1 | 107 | 28.8 / 20.5 | 2.480 |
| Flat | [#38](https://github.com/stevecad/camlib/issues/38) | `flat` (flats) | 156 | 680 | 176 (26%) | 459 | 39 | 39 | 1270 | 165.2 / 165.2 | 0.006 |
| Parallel | [#39](https://github.com/stevecad/camlib/issues/39) | `parallel` (finish-roof) | 3321 | 1723 | 543 (32%) | 427 | 41 | 41 | 1508 | 334.8 / 316.9 | 0.131 |
| Parallel | [#39](https://github.com/stevecad/camlib/issues/39) | `parallel-dense` (finish-roof) | 14023 | 8457 | 3170 (37%) | 2005 | 201 | 201 | 1511 | 1618.0 / 1555.9 | 0.634 |
| Parallel | [#39](https://github.com/stevecad/camlib/issues/39) | `rough-parallel` (parallel) | 3321 | 1805 | 625 (35%) | 508 | 41 | 41 | 292 | 368.4 / 350.5 | 0.177 |
| Spiral | [#40](https://github.com/stevecad/camlib/issues/40) | `spiral` (spiral) | 2552 | 1060 | 84 (8%) | 740 | 30 | 31 | 1505 | 225.9 / 225.5 | 0.087 |
| Spiral | [#40](https://github.com/stevecad/camlib/issues/40) | `spiral-dense` (spiral) | 12767 | 5291 | 513 (10%) | 3603 | 152 | 152 | 1510 | 1125.9 / 1123.9 | 0.490 |
| Radial | [#41](https://github.com/stevecad/camlib/issues/41) | `radial` (radial) | 610 | 477 | 3 (1%) | 215 | 24 | 24 | 1452 | 104.7 / 104.0 | 0.035 |
| Project | [#42](https://github.com/stevecad/camlib/issues/42) | `project` (project) | 97 | 61 | 0 (0%) | 24 | 2 | 2 | 413 | 11.3 / 11.2 | 0.006 |
| Morph | [#43](https://github.com/stevecad/camlib/issues/43) | `morph` (morph) | 441 | 770 | 0 (0%) | 201 | 25 | 25 | 1234 | 142.8 / 141.2 | 0.079 |
| Scallop | [#44](https://github.com/stevecad/camlib/issues/44) | `scallop` (scallop) | 2898 | 1076 | 134 (12%) | 451 | 34 | 34 | 1505 | 205.2 / 187.0 | 0.109 |
| Scallop | [#44](https://github.com/stevecad/camlib/issues/44) | `scallop-dense` (scallop) | 27025 | 6962 | 2606 (37%) | 4357 | 589 | 589 | 1511 | 1678.2 / 1565.8 | 1.822 |
| Pencil | [#45](https://github.com/stevecad/camlib/issues/45) | `pencil` (pencil) | 67 | 53 | 0 (0%) | 16 | 1 | 1 | 562 | 8.6 / 8.6 | 0.004 |
| Waterline | [#46](https://github.com/stevecad/camlib/issues/46) | `waterline` (finish-levels) | 112 | 224 | 0 (0%) | 60 | 4 | 4 | 1719 | 33.0 / 33.0 | 0.127 |
| Waterline | [#46](https://github.com/stevecad/camlib/issues/46) | `waterline-dense` (finish-levels) | 1847 | 3192 | 0 (0%) | 414 | 56 | 56 | 1730 | 460.2 / 460.2 | 0.995 |
| Steep-and-shallow | [#47](https://github.com/stevecad/camlib/issues/47) | `steep-and-shallow` (finish-shallow) | 780 | 847 | 52 (6%) | 215 | 24 | 24 | 1256 | 156.6 / 152.9 | 0.080 |
| Steep-and-shallow | [#47](https://github.com/stevecad/camlib/issues/47) | `steep-and-shallow` (finish-steep) | 964 | 668 | 22 (3%) | 1006 | 40 | 40 | 249 | 186.2 / 180.8 | 0.392 |
| ThermalCut | [#48](https://github.com/stevecad/camlib/issues/48) | `plasma` (plasma) | 66 | 757 | 0 (0%) | 397 | 6 | 6 | 4689 | 26.5 / 25.5 | 0.015 |
| TurnFace | [#49](https://github.com/stevecad/camlib/issues/49) | `lathe-face-groove` (face) | 6 | 18 | 2 (11%) | 84 | 0 | 2 | 531 | 8.4 / 8.4 | 0.003 |
| TurnFace | [#49](https://github.com/stevecad/camlib/issues/49) | `lathe-groove` (face) | 8 | 50 | 4 (8%) | 53 | 0 | 2 | 2513 | 20.8 / 20.8 | 0.003 |
| TurnFace | [#49](https://github.com/stevecad/camlib/issues/49) | `lathe-shaft` (face) | 8 | 50 | 4 (8%) | 53 | 0 | 2 | 2513 | 20.8 / 20.8 | 0.003 |
| TurnFace | [#49](https://github.com/stevecad/camlib/issues/49) | `lathe-thread` (face) | 8 | 50 | 4 (8%) | 53 | 0 | 2 | 2513 | 20.8 / 20.8 | 0.003 |
| TurnRough | [#50](https://github.com/stevecad/camlib/issues/50) | `lathe-groove` (rough) | 28 | 229 | 4 (2%) | 296 | 0 | 14 | 33227 | 59.4 / 59.4 | 0.004 |
| TurnRough | [#50](https://github.com/stevecad/camlib/issues/50) | `lathe-shaft` (rough) | 26 | 229 | 4 (2%) | 260 | 0 | 13 | 33227 | 58.8 / 58.8 | 0.003 |
| TurnRough | [#50](https://github.com/stevecad/camlib/issues/50) | `lathe-thread` (rough) | 26 | 229 | 4 (2%) | 260 | 0 | 13 | 33227 | 58.8 / 58.8 | 0.003 |
| TurnFinish | [#51](https://github.com/stevecad/camlib/issues/51) | `lathe-shaft` (finish) | 76 | 61 | 17 (27%) | 108 | 1 | 1 | 1152 | 32.2 / 32.2 | 0.005 |
| TurnFinish | [#51](https://github.com/stevecad/camlib/issues/51) | `lathe-thread` (finish) | 76 | 61 | 17 (27%) | 108 | 1 | 1 | 1152 | 32.2 / 32.2 | 0.005 |
| TurnGroove | [#52](https://github.com/stevecad/camlib/issues/52) | `lathe-groove` (groove) | 43 | 38 | 26 (68%) | 77 | 20 | 1 | 841 | 58.3 / 58.3 | 0.007 |
| TurnPartOff | [#53](https://github.com/stevecad/camlib/issues/53) | `lathe-groove` (part) | 5 | 22 | 0 (0%) | 107 | 1 | 1 | 1287 | 34.8 / 34.8 | 0.004 |
| TurnThread | [#54](https://github.com/stevecad/camlib/issues/54) | `lathe-thread` (thread) | 31 | 155 | 22 (14%) | 293 | 0 | 7 | 0 | 12.9 / 12.9 | 0.003 |
| TurnDrill | [#55](https://github.com/stevecad/camlib/issues/55) | `lathe-bore` (drill) | 12 | 34 | 0 (0%) | 194 | 3 | 4 | 5324 | 21.6 / 21.6 | 0.003 |
| TurnBore | [#56](https://github.com/stevecad/camlib/issues/56) | `lathe-bore` (bore_finish) | 8 | 29 | 3 (10%) | 88 | 1 | 3 | 332 | 19.4 / 19.4 | 0.003 |
| TurnBore | [#56](https://github.com/stevecad/camlib/issues/56) | `lathe-bore` (bore_rough) | 9 | 53 | 1 (3%) | 92 | 0 | 4 | 4009 | 22.8 / 22.8 | 0.003 |
| TurnFaceGroove | [#57](https://github.com/stevecad/camlib/issues/57) | `lathe-face-groove` (fg) | 29 | 22 | 22 (100%) | 98 | 12 | 13 | 308 | 75.0 / 75.0 | 0.003 |
| Geodesic | [#118](https://github.com/stevecad/camlib/issues/118) | `geodesic` (geodesic) | 5336 | 767 | 79 (10%) | 22 | 1 | 1 | 1506 | 131.0 / 127.9 | 0.222 |
| Geodesic | [#118](https://github.com/stevecad/camlib/issues/118) | `geodesic-boundary` (geodesic) | 713 | 712 | 5 (1%) | 53 | 3 | 3 | 1504 | 84.2 / 81.0 | 0.079 |

## Per workload

| Workload | What | Moves | Cycle s (exact / tangent) | Blocks | Leftover mm³ | Over-cut mm³ | Run s | First move s | Post s | Peak memory MiB |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| `facing` | Face a 40 x 24 block 2 mm | 20 | 47.1 / 47.1 | 43 | 0.0 / 254.0 | 0.0 / 0.0 | 0.011 | 0.005 | 0.003 | 26 |
| `pocket` | Pocket around a round island (CAD region) | 352 | 508.3 / 508.3 | 383 | 3.8 / 541.5 | 0.0 / 489.6 | 0.035 | 0.009 | 0.024 | 31 |
| `rest-pocket` | Face, 4 mm pocket, 2 mm rest pocket | 130 | 69.9 / 69.6 | 169 | 0.0 / 267.0 | 0.0 / 66.0 | 0.008 | 0.003 | 0.011 | 26 |
| `adaptive-pocket` | Adaptive clearing of an L pocket | 3752 | 1262.9 / 1225.6 | 5781 | 4.8 / 492.8 | 0.0 / 49.6 | 0.073 | 0.005 | 1.349 | 390 |
| `roughing-raster` | 3D raster roughing around a frustum | 2169 | 7932.3 / 7931.6 | 2400 | 930.8 / 1505.5 | 0.0 / 83.1 | 0.054 | 0.010 | 0.188 | 389 |
| `roughing-offset` | 3D offset roughing around a frustum | 924 | 1439.8 / 1436.8 | 1219 | 930.8 / 1505.5 | 0.0 / 83.1 | 0.048 | 0.018 | 0.293 | 388 |
| `roughing-adaptive` | 3D adaptive roughing around a frustum | 97138 | 17533.4 / 16572.5 | 146997 | 930.8 / 1510.5 | 0.0 / 78.1 | 2.738 | 0.017 | 31.231 | 564 |
| `roughing-fine-mesh` | 3D offset roughing of a filleted boss from a fine mesh | 3295 | 554.3 / 531.4 | 4874 | 816.3 / 1385.3 | 0.0 / 0.4 | 0.314 | 0.227 | 10.079 | 406 |
| `contour` | Outer and inner walls of an annulus | 40 | 55.8 / 55.7 | 79 | 0.0 / 831.6 | 0.0 / 430.2 | 0.011 | 0.005 | 0.010 | 384 |
| `holes` | Pilot drilling then helical bore milling | 62 | 51.0 / 50.9 | 111 | 0.0 / 413.1 | 0.0 / 125.1 | 0.025 | 0.005 | 0.013 | 388 |
| `reaming` | Drill then ream two holes | 16 | 28.7 / 28.7 | 46 | – | – | 0.010 | 0.006 | 0.004 | 29 |
| `tapping` | Drill then rigid-tap two holes | 18 | 14.8 / 14.8 | 49 | – | – | 0.009 | 0.005 | 0.004 | 29 |
| `thread-milling` | Bore then thread-mill M14 x 1.5 | 35 | 47.7 / 47.7 | 85 | – | – | 0.316 | 0.009 | 0.011 | 31 |
| `trochoidal-slot` | Straight trochoidal slot | 356 | 125.4 / 121.9 | 555 | 0.0 / 407.8 | 0.0 / 53.6 | 0.013 | 0.005 | 0.083 | 27 |
| `chamfer` | Chamfer the top edges of an annulus | 50 | 66.7 / 66.6 | 101 | 0.0 / 725.6 | 0.0 / 343.4 | 0.034 | 0.006 | 0.012 | 384 |
| `trace` | Engrave a rectangle and a hook | 24 | 15.5 / 15.5 | 49 | – | – | 0.006 | 0.004 | 0.004 | 26 |
| `vcarve` | V-carve an L-shaped letter | 1986 | 28.8 / 20.5 | 2009 | – | – | 2.498 | 0.019 | 0.107 | 33 |
| `flat` | Flat areas of a stepped part | 156 | 165.2 / 165.2 | 179 | 28.5 / 175.8 | 0.0 / 14.3 | 0.013 | 0.007 | 0.012 | 384 |
| `rough-parallel` | Rough then parallel ball finish a roof | 6248 | 9268.7 / 9249.5 | 6549 | 0.0 / 146.1 | 0.0 / 99.6 | 0.264 | 0.010 | 0.484 | 396 |
| `parallel` | Parallel finishing of a roof | 3321 | 334.8 / 316.9 | 3344 | 0.0 / 146.1 | 0.0 / 99.6 | 0.135 | 0.004 | 0.184 | 392 |
| `spiral` | Spiral finishing of a roof | 2552 | 225.9 / 225.5 | 2575 | 0.0 / 149.9 | 0.0 / 79.0 | 0.093 | 0.005 | 0.150 | 390 |
| `radial` | Radial finishing of a roof | 610 | 104.7 / 104.0 | 633 | 9.8 / 202.6 | 0.0 / 67.2 | 0.040 | 0.004 | 0.041 | 385 |
| `project` | Project curves onto a roof | 97 | 11.3 / 11.2 | 120 | 985.2 / 1241.2 | 0.0 / 7.6 | 0.013 | 0.005 | 0.007 | 383 |
| `morph` | Morph between two edges of a roof | 441 | 142.8 / 141.2 | 464 | 194.2 / 420.2 | 0.0 / 52.7 | 0.087 | 0.006 | 0.025 | 383 |
| `scallop` | Scallop finishing of a roof | 2898 | 205.2 / 187.0 | 2921 | 0.0 / 149.9 | 0.0 / 79.1 | 0.139 | 0.029 | 0.165 | 390 |
| `geodesic` | Geodesic finishing of a roof, rings from a point | 5336 | 131.0 / 127.9 | 5359 | 0.0 / 148.6 | 0.0 / 65.6 | 2.225 | 1.820 | 0.333 | 396 |
| `geodesic-boundary` | Geodesic finishing of a roof, inward from its boundary | 713 | 84.2 / 81.0 | 736 | 0.0 / 150.2 | 0.0 / 81.9 | 0.314 | 0.232 | 0.043 | 385 |
| `pencil` | Pencil the inside corner of a stepped part | 67 | 8.6 / 8.6 | 90 | 648.6 / 884.2 | 0.0 / 27.3 | 0.024 | 0.019 | 0.007 | 384 |
| `waterline` | Waterline finishing of a tapered island | 112 | 33.0 / 33.0 | 135 | 369.9 / 809.3 | 0.0 / 89.3 | 0.135 | 0.129 | 0.009 | 389 |
| `steep-and-shallow` | Steep-and-shallow finishing of a roof | 1744 | 342.8 / 333.7 | 1775 | 0.0 / 149.8 | 0.0 / 100.3 | 0.481 | 0.379 | 0.107 | 388 |
| `facing-large` | Face a 300 x 200 block 2 mm (production size) | 140 | 2153.3 / 2153.3 | 163 | – | – | 0.408 | 0.048 | 0.009 | 69 |
| `pocket-dense` | Pocket around a round island, 0.4 mm stepover, 0.25 mm stepdown | 9088 | 11873.8 / 11873.4 | 9175 | 3.8 / 541.5 | 0.0 / 489.6 | 0.344 | 0.010 | 0.532 | 44 |
| `parallel-dense` | Parallel finishing of a roof, 0.1 mm stepover | 14023 | 1618.0 / 1555.9 | 14046 | 0.0 / 143.9 | 0.0 / 101.0 | 0.638 | 0.004 | 0.838 | 415 |
| `spiral-dense` | Spiral finishing of a roof, 0.15 mm stepover | 12767 | 1125.9 / 1123.9 | 12790 | 0.0 / 144.2 | 0.0 / 86.6 | 0.498 | 0.007 | 0.748 | 417 |
| `scallop-dense` | Scallop finishing of a roof, 0.15 mm stepover | 27025 | 1678.2 / 1565.8 | 27048 | 0.0 / 143.6 | 0.0 / 81.0 | 2.998 | 1.026 | 1.552 | 444 |
| `waterline-dense` | Waterline finishing of a tapered island, 0.1 mm stepdown | 1847 | 460.2 / 460.2 | 1870 | 369.9 / 797.6 | 0.0 / 100.4 | 1.008 | 0.382 | 0.104 | 392 |
| `plasma` | Plasma-cut a plate with holes and a nested washer | 66 | 26.5 / 25.5 | 294 | – | – | 0.082 | 0.068 | 0.064 | 45 |
| `lathe-shaft` | Face, rough and finish a stepped shaft | 110 | 111.7 / 111.7 | 135 | 0.9 / 194.2 | 0.0 / 0.0 | 0.022 | 0.007 | 0.008 | 29 |
| `lathe-groove` | Rough, groove and part off a shaft | 84 | 173.2 / 173.2 | 115 | 857.3 / 1386.6 | 0.0 / 0.0 | 0.035 | 0.007 | 0.008 | 29 |
| `lathe-thread` | Turn and single-point thread M30 x 2 | 141 | 124.7 / 124.7 | 173 | 0.9 / 194.2 | 0.0 / 0.0 | 0.026 | 0.007 | 0.012 | 29 |
| `lathe-bore` | Drill on the axis, rough and finish bore | 29 | 63.8 / 63.8 | 61 | 9596.7 / 10035.8 | 0.0 / 0.0 | 0.019 | 0.007 | 0.004 | 29 |
| `lathe-face-groove` | Face and cut a face groove | 35 | 83.3 / 83.3 | 66 | 3122.0 / 3254.7 | 0.0 / 0.0 | 0.015 | 0.006 | 0.004 | 29 |
