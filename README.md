> [!CAUTION]
> ## ⚠️ EXPERIMENTAL SOFTWARE: NOT PRODUCTION READY ⚠️
>
> **camlib is experimental and under active development. Its toolpaths and
> G-code have not been proven on real machines. Its declared-machine crash
> check covers nominal paths and caller-supplied physical geometry, not the
> actual setup or controller execution. See [coverage and limits](docs/machine.md).**
>
> **Review and test ALL G-code THOROUGHLY before running it on any machine:**
>
> - simulate the posted file in independent verification software;
> - check every work offset, tool length and tool number against your setup;
> - dry-run above the part, then single-block the first run with the rapid and
>   feed overrides turned down and a hand on feed hold / E-stop.
>
> A wrong program can destroy the machine, the tool, the fixture and the part,
> and can injure or kill. You use camlib entirely at your own risk. It is
> provided under the MIT license **with no warranty of any kind**.

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
| Trochoidal slot, chamfer, V-carve | Straight or curved slots; chamfer on top edges; deburring of horizontal and sloped edges; sharp-cornered V-carving of letters and logos |
| Thread milling, tapping | Internal and external thread milling; rigid right- and left-hand tapping (G33.1, or G84/G74 where declared) |
| Trace / engrave | Open or closed line/arc paths at depth, no radius offset; or the cutter beside an open path (left/right side offset) |
| Undercut | T-slot, keyseat, lollipop and dovetail cutters under material, on tri-dexel stock (3D stock that holds undercuts) |
| Flat-area finishing | Horizontal faces at their exact heights, flat end mills |
| Parallel, spiral, radial, project, pencil, scallop, waterline, ramp, steep-and-shallow, morph, flow finishing | Triangle-mesh surfaces: ball (certified error), flat and bull-nose end mills; slope limits; rest finishing from the stock |
| Machining limits | Avoid areas (clamps) and containment regions for every operation, rapids and links |
| Plasma and laser cutting | Kerf-compensated sheet parts: leads, pierce clearance, inner features first, small holes, micro-joints, torch height control, programmed or controller heights |
| Feeds and speeds | rpm and feed from cutting data (chip thinning, ball effective diameter) |
| Probing, tool setting | Work-offset probing of surfaces, corners, bores, bosses, pockets and webs, tool length and diameter on a tool setter; LinuxCNC G38 or Renishaw Inspection Plus macros, decoded back and checked |
| Turning (lathe) | Facing, outside and inside roughing and finishing (boring), grooving, parting off, external and internal threading, drilling on the axis, constant surface speed and feed per revolution; finishing follows fillets, ball ends and corner rounding with native G2/G3 arcs; insert, holder and boring-bar checks against the part, bore, stock and chuck; a lathe setup is an ordinary `Setup` (`turning=TurningSetup(...)`), streamed, played back and posted like any other, with a material check after each operation |

Every setup (milling, plasma/laser and lathe) runs as one stream: each move
arrives with the stock it removes, and playback can pause, seek (also
partway through a move), cancel, and save and reopen a run. Every move can be
checked continuously against the evolving stock, fixtures, protected parts,
the tool holder and machine limits (lathe moves always are, against the part,
bore, chuck and stock), and the stock against the target part; posted G-code
is decoded again, compared with the plan and replayed on the stock. Exporting
the program refuses it while a violation or uncertain check result has not
been accepted on purpose. A declared machine model is required at export;
missing coverage requires explicit `accept={"unchecked_machine"}`. Head/table
kinematics, convex mesh fixtures, independent-axis rapid envelopes, absolute
travel limits and complete declared tool-change/indexing routes apply to actual
decoded output. [Machine checking](docs/machine.md) explains conservative
refusals, overhang/rotary limits (#1/#2), and external setup premises.

Postprocessors: every machine is a `camlib.post.Machine` with a declared dialect
(`GCodeDialect.linuxcnc()` for LinuxCNC). Documented Python controller posts:
GRBL 1.1h (with laser mode), Haas NGC, [Makera Carvera](docs/carvera-post.md), FANUC 0i-MB and FANUC lathes (system A), LNC milling (LNC-M8000),
Siemens 808D (native and ISO) and Centroid CNC12 (mill, lathe and plasma),
LinuxCNC QtPlasmaC and WinCNC (plasma). Not
supported yet: 4/5-axis, undercuts.
Details and limits: [capabilities](docs/library-capabilities.md). Measured
toolpath quality and engine performance for every toolpath:
[benchmarks](docs/benchmarks.md).

Optional [checked recipe planning](docs/planning.md) compares supplied milling
strategies and cutter recipes, retains the reference, and can append bounded
native rest corrections from residual findings. It uses the existing checks
and explicit machine/cutting data; it produces advice and never releases NC.
The same guide covers opt-in engagement slowdown caps, declared initial-blank
bounds, and a host-side example for colour intent and confirmed blank placement.

Native [bounded preparation](docs/preparation.md) can screen supplied samples
and calibrated assemblies and enclose a declared top-down blank. These queries
produce explicit premises and advice; ordinary job verification still decides release.

## Install

Requires Python 3.12+ and a Rust toolchain (the package compiles the engine):

```sh
python -m pip install .
python examples/standalone_job.py out --accept-unchecked-machine  # synthetic inspection file
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

## Contributing

Every change starts from a GitHub issue, and only one person works an issue at a
time:

1. **Claim an issue.** Prefer an existing open issue, so known problems are
   solved first; follow the order in [ROADMAP.md](ROADMAP.md). Issues with the
   `claimed` label are taken: pick another. To claim one, add the `claimed`
   label and comment that you are working on it.
2. **No existing issue fits?** Open one describing the problem and what "done"
   means, then claim it.
3. **Work it through a pull request** that references the issue
   (`Closes #N`). `main` is protected; every change needs passing CI.
4. **Stopping early?** Remove the `claimed` label and comment on what is left.

[AGENTS.md](AGENTS.md) has the full rules (for people and AI agents alike):
what camlib is and is not, the correctness rules, and how to work an issue.

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
