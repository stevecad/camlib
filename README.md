# camlib

camlib is a CAM engine for three-axis milling, turning and plasma/laser
cutting that you embed in your own application.
You describe stock, tools and operations in Python (or Rust); camlib plans the
toolpaths, simulates material removal while it streams them, checks the moves,
and writes G-code through a postprocessor you can read and change.

- **Engine in Rust**: geometry, toolpath planning, stock simulation, checking.
- **Ordinary Python API**: jobs, tools, machines and postprocessors are plain
  Python objects. CadQuery is optional, for getting geometry from CAD solids.
- **No UI, no machine control.** camlib produces files; it never talks to a machine.

> **Safety.** camlib checks the program against the model you give it: stock,
> tools, declared fixtures/protected parts and machine limits. It cannot know
> your real work offsets, tool lengths, clamps or tool-change position. The
> controller posts follow their manufacturers' manuals and every posted program
> is decoded again and checked against the plan, but no post has yet been
> proven on a real machine. Always simulate the posted file in your own tools,
> dry-run above the part, and single-block the first run. MIT licensed, no
> warranty.

## What it can do

| Operation | Notes |
| --- | --- |
| Facing | Rectangular stock, zig-zag or one-way passes |
| Pocket, rest pocket | Line/arc regions with islands; plunge, helical or stock-edge entry; links kept at depth only through cleared stock |
| Adaptive pocket | Engagement-limited clearing |
| 3D roughing, rest roughing | Layered clearing above a triangle-mesh part (raster, offset or adaptive) |
| Contour | Outer walls / holes of a part: climb or conventional, leads, wear compensation (G41/G42), holding tabs |
| Drilling, peck drilling, spot/counterbore dwell | Canned cycles (G81/G82/G83/G73) where the machine declares them, explicit moves elsewhere |
| Reaming, boring, fine boring, bore milling | G85/G89/G86 from proven pilot holes; G76 or M19 fine boring with an oriented stop and shift proven inside the bore; helical bore milling |
| Trochoidal slot, chamfer, V-carve | Straight or curved slots; chamfer on top edges; sharp-cornered V-carving of letters and logos |
| Thread milling, tapping | Internal and external thread milling; rigid right- and left-hand tapping (G33.1, or G84/G74 where declared) |
| Trace / engrave | Open or closed line/arc paths at depth, no radius offset |
| Flat-area finishing | Horizontal faces at their exact heights, flat end mills |
| Parallel, spiral, radial, project, pencil, scallop, waterline, ramp, steep-and-shallow, morph, flow finishing | Triangle-mesh surfaces: ball (certified error), flat and bull-nose end mills; slope limits; rest finishing from the stock |
| Machining limits | Avoid areas (clamps) and containment regions for every operation, rapids and links |
| Plasma and laser cutting | Kerf-compensated sheet parts: leads, pierce clearance, inner features first, small holes, micro-joints, torch height control, programmed or controller heights |
| Feeds and speeds | rpm and feed from cutting data (chip thinning, ball effective diameter) |
| Turning (lathe) | Facing, outside and inside roughing and finishing (boring), grooving, parting off, external and internal threading, drilling on the axis, constant surface speed and feed per revolution; insert, holder and boring-bar checks against the part, bore, stock and chuck; a lathe setup is an ordinary `Setup` (`turning=TurningSetup(...)`), streamed, played back and posted like any other, with a material check after each operation |

Every setup (milling, plasma/laser and lathe) runs as one stream: each move
arrives with the stock it removes, and playback can pause, seek (also
partway through a move), cancel, and save and reopen a run. Every move can be
checked continuously against the evolving stock, fixtures, protected parts,
the tool holder and machine limits (lathe moves always are, against the part,
bore, chuck and stock), and the stock against the target part; posted G-code
is decoded again, compared with the plan and replayed on the stock. Exporting
the program refuses it while a violation or uncertain check result has not
been accepted on purpose.

Postprocessors: every machine is a `camlib.post.Machine` with a declared dialect
(`GCodeDialect.linuxcnc()` for LinuxCNC). Documented Python controller posts:
GRBL 1.1h (with laser mode), Haas NGC, FANUC 0i-MB and FANUC lathes (system A),
Siemens 808D (native and ISO) and Centroid CNC12 (mill, lathe and plasma),
LinuxCNC QtPlasmaC and WinCNC (plasma). Not
supported yet: 4/5-axis, undercuts.
Details and limits: [capabilities](docs/library-capabilities.md). Measured
toolpath quality and engine performance for every toolpath:
[benchmarks](docs/benchmarks.md).

## Install

Requires Python 3.12+ and a Rust toolchain (the package compiles the engine):

```sh
python -m pip install .
python examples/standalone_job.py out      # writes out/pocket.nc
```

For native Windows x64 installation, build tools and PowerShell commands, see
[Windows setup](docs/windows.md).

The CadQuery example (`examples/cadquery_job.py`) also needs CadQuery/OCP
installed in the same Python environment.

A portable Linux x86_64 wheel (manylinux_2_28: glibc 2.28 or newer, CPython
3.12+) is built in Docker from the committed tree:

```sh
packaging/build-manylinux-wheel.sh          # writes dist/camlib-*-manylinux_2_28_x86_64.whl
python -m pip install dist/camlib-*-manylinux_2_28_x86_64.whl
```

Start with **[Getting started](docs/getting-started.md)**, then
[Operations](docs/operations.md) and [Postprocessors](docs/posts.md).

## Rust

```sh
cargo build --release -p camlib
cargo run --release --example pocket
```

See `operation::OperationPlan`, `motion::Curve`, `tool::ToolAssembly` and
`stock::Stock`. All internal units are millimetres.

## Development

```sh
cargo test --release --workspace --all-features --lib --tests
cargo clippy --workspace --all-targets --all-features -- -D warnings
PYO3_PYTHON=$(which python) cargo build --release --workspace --all-features
PYTHONPATH=python python -m unittest discover -s python/tests -v   # needs CadQuery/OCP
```

Toolpath benchmarks (toolpath quality and planning time per toolpath, needs
CadQuery): `python bench/run.py --label <name>`, then
`python bench/report.py bench/results/<name>.json`.

Tests use synthetic inputs. The Python suite includes CAD-adapter tests and
therefore needs CadQuery/OCP. CI (`.github/workflows/ci.yml`) runs all of the
above on every push and pull request.

## License

[MIT](LICENSE).
