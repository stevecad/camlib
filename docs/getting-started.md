# Getting started

This walks through one complete job in Python: a block of stock, one end mill,
a facing pass and a pocket, posted to LinuxCNC G-code. The full runnable
version is [`examples/standalone_job.py`](../examples/standalone_job.py).

## Conventions

- **Millimetres everywhere** inside camlib. Inch output is a postprocessor setting.
- **Absolute Z in the work coordinate system.** If stock is `Z 0..6`, its top is `Z 6`.
- **Tool positions are the tool tip** (the lowest point of the cutter).
- **Heights an operation uses:**
  - `top_z_mm`: where material starts (the stock top for the first operation),
  - `floor_z_mm`: the bottom of the cut,
  - `clearance_z_mm`: a height above everything (clamps included) for travel
    between operations. Optional `retract_z_mm` / `feed_z_mm` lower the travel
    and approach planes for pockets and roughing; see [Operations](operations.md).

## A complete job

```python
import math

from camlib.jobs import Job, Setup
from camlib.operations import Facing, Pocket
from camlib.planar import Boundary, Component, PlanarRegion
from camlib.post import GCodeDialect, LoweringOptions, Machine, ToolSetting
from camlib.simulation import StockSpec
from camlib.tools import EndMill, HolderSection, ToolAssembly
from camlib.verification import StockContactOptions, StreamChecks

# 1. Tool: a 6 mm flat end mill, 30 mm out of the holder.
mill = ToolAssembly(
    "mill6",
    EndMill("em6", diameter=6, flute_length=20, overall_length=60),
    stickout_mm=30, gauge_length_mm=70,
    holders=(HolderSection(z_min_mm=30, z_max_mm=70, radius_min_mm=10, radius_max_mm=10),),
)

# 2. Geometry: a 30 x 20 mm pocket with a round island (r = 4) left standing.
area = PlanarRegion("pocket", (Component(
    Boundary.polygon(((5, 5), (35, 5), (35, 25), (5, 25))),
    holes=(Boundary.circle((20, 15), 4),),
),))

# 3. Operations, in order.
#    Cutting data has no defaults: set it for your tool, material and machine.
cutting = dict(feed_mm_min=600, plunge_mm_min=150, spindle_rpm=10000, coolant="flood",
               tool_number=1, length_register=1)
face = Facing("face", mill, floor_z_mm=9, stepdown_mm=1, stepover_mm=3,
              clearance_z_mm=15, **cutting)
pocket = Pocket("pocket", mill, area, top_z_mm=9, floor_z_mm=5, stepdown_mm=1.5,
                stepover_mm=2.5, clearance_z_mm=15, **cutting)

# 4. Setup: stock block 40 x 30 x 10, the tool, operations, start position,
#    and checking of every move against the evolving stock.
setup = Setup("top",
              stock=StockSpec(min_mm=(0, 0, 0), max_mm=(40, 30, 10), cell_size_mm=0.25),
              tools=(mill,), operations=(face, pocket),
              initial_position_mm=(0, 0, 15),
              verification=StreamChecks(stock_contact=StockContactOptions()))
job = Job("getting-started", "1", (setup,))

# 5. Machine: camlib assumes nothing about it. State the controller, units,
#    limits and capabilities of your machine; these describe a LinuxCNC mill.
header = job.compile().setup("top").program_header()
machine = Machine("My mill", (ToolSetting.for_tool(
    header.tools[0], number=1, length_register=1,
    position_after_mm=header.initial_position_mm),),
    dialect=GCodeDialect.linuxcnc(), output_units="mm", decimals=3,
    max_feed_mm_min=10000, max_spindle_rpm=24000, rapid_mm_min=10000,
    planes=("xy", "xz", "yz"), arcs=True, helices=True, max_sweep_rad=math.tau,
    max_tool_number=99, max_length_register=99, tool_changes=True,
    flood=True, mist=False, tool_change_position_mode="table")

# 6. Plan, simulate, check and post in one stream. native_arcs=True writes
#    arcs as G2/G3 when exact at 3 decimals, else as checked lines (see below).
arcs = LoweringOptions(native_arcs=True)
with job.post("top", machine=machine, lowering=arcs) as run:
    run.wait()
    lines, start = [], 1
    while page := run.nc_lines(start, limit=256):
        lines.extend(line["text"] for line in page)
        start = page[-1]["number"] + 1

print(f"{len(lines)} lines of G-code")
```

`job.post(...)` plans each operation in the Rust engine, sweeps the tool
through the stock, runs the configured checks, formats G-code with the
postprocessor, and then *decodes the emitted text again* to confirm it
describes the planned motion. A planning, posting or decoding failure stops
the run with an error, so you never get a silently wrong file. Check *findings*
(below) are reported per move while the run continues; exporting the program
refuses it while any violation or uncertain result remains:

```python
exported = run.export("top.nc")  # raises camlib.export.ExportRefused on findings
```

Accept a kind only on purpose, after looking at it: `run.export("top.nc",
accept={"uncertain"})`. The result also names the checks you did not configure
(nothing was checked there).

To just write the program (no playback journal and no replay of the posted
output into stock), use `job.post_nc`. It formats, decodes and checks every
block exactly as `job.post` does and produces the same text, in less than half
the time:

```python
program = job.post_nc("top", machine=machine, path="top.nc")  # same acceptance rule
print(program.lines, program.motions, program.unchecked)
```

Milling material checks (`StreamChecks(material=...)`) need the posted-stock
replay, so they require `job.post`. Every posted move is also proven not to cut deeper into the part than the comparison tolerance,
which decides cells along walls the tool ran tangent to. Choose a tolerance
above the program's resolution and the mesh's surface error, for example
`MaterialCheck(target, options=camlib.material.Options(comparison_tolerance_mm=0.005))`
at 3 decimals; the default 0.001 mm leaves those cells uncertain. A lathe setup's material check runs with
every operation, so `job.post_nc` reports it too (`program.material`).

Lathe setups work the same way: a `Setup` with turning stock and
`turning=camlib.TurningSetup(...)` (see [Operations](operations.md), Turning)
records, plays back, posts and exports through these same calls.

## More than one tool

The job above uses one tool, so the tip simply starts where the machine is.
With several tools the planner must start each new tool where the machine's
tool change really leaves it, and the machine and the setup must say the same
thing. Choose the pair that matches your controller:

| Machine (`tool_change_position_mode`) | Setup (`Setup(tool_change=...)`) |
|---|---|
| `"table"`: each `ToolSetting.position_after_mm` is the tip after its change | `ToolChangeTable.from_machine(machine)` |
| `"z_home"`: M6 sends the gauge line to `tool_change_home_z_mm`, XY stays | `ToolChangePosition(gauge_z_mm=<home Z minus work offset Z>)` |
| `"preserve_tip"`: the tip stays where it was | `None` (the default) |

```python
from camlib import Drill, Drilling, ToolChangeTable

drill = ToolAssembly("drill5", Drill("drill5", diameter=5, point_angle=118,
                     flute_length=20, overall_length=60), 40, 80,
                     (HolderSection(40, 80, 10, 10),))
holes = Drilling("holes", drill, ((10, 10), (30, 20)), 4.0, 15,
                 feed_mm_min=150, retract_feed_mm_min=600, spindle_rpm=6000,
                 coolant="flood", tool_number=2, length_register=2)

# Where the machine's tool change leaves each tool, measured on your machine
# (here: the gauge line 100 mm above the work origin, over X0 Y0).
machine = Machine("My mill", (
    ToolSetting.for_tool(mill, number=1, length_register=1,
                         position_after_mm=(0, 0, 100 - mill.gauge_length_mm)),
    ToolSetting.for_tool(drill, number=2, length_register=2,
                         position_after_mm=(0, 0, 100 - drill.gauge_length_mm)),
    ), **machine_settings, tool_change_position_mode="table")

setup = Setup("top", stock=..., tools=(mill, drill),
              operations=(face, pocket, holes), initial_position_mm=(0, 0, 15),
              tool_change=ToolChangeTable.from_machine(machine))
```

`machine_settings` stands for the dialect, units, limits and capabilities of
step 5. `job.post` and `job.post_nc` refuse a setup that disagrees with the
machine before planning, naming the tool, both poses and the setup to declare.
Each posted tool change is also decoded and checked against the plan.

## Reading the results

Replay the run to the end to see the final stock and check summary:

```python
from camlib.playback import Cursor, Player

with job.post("top", machine=machine, lowering=arcs) as run:
    run.wait()
    player = Player(run)
    player.seek(Cursor(run.status()["available"]))
    print(player.state.volume())     # removed / remaining stock bounds, mm³
    print(player.verification)       # check summary, see below
```

The check summary always has `status: "partial"`. camlib never claims a blanket
"all clear": it reports exactly what was checked. Look at:

- `progress.violation`: moves that definitely hit something they must not
  (stock during a rapid, a fixture, a protected part). Must be 0.
- `progress.uncertain`: moves the check could not decide within its budgets.
  Treat as a failure until understood.
- `coverage`: what was checked. For example `machine_tip_and_feed_configured:
  false` means travel and feed limits were not supplied, so they were not checked.

**The finished part is only protected if you declare it.** Stock-contact
checking protects stock from rapids; to catch a cut into the part, add the part
(or a conservative prism of it) as a protected body:

```python
from camlib.verification import Prism, Scene, StockContactOptions, StreamChecks

island = PlanarRegion("island", (Component(Boundary.circle((20, 15), 4)),))
checks = StreamChecks(
    Scene("checks", "1", (Prism("island", island, z_min_mm=0, z_max_mm=9,
                                role="protected_target"),)),
    stock_contact=StockContactOptions())
```

Fixtures and clamps use `role="fixture"`.

Plans that are valid but probably wrong raise a `camlib.simulation.CamWarning`
(for example an operation that produced no cutting motion because the region is
smaller than the tool, or removed no stock because it lies outside the stock).
`warnings.simplefilter("error", CamWarning)` turns them into errors.

## Arcs and output precision

The default machine writes 3 decimals in mm (4 in inch), like FANUC/Haas-class
controls. Some arcs cannot be written *exactly* (equal start and end radius) on
that grid; the pocket above has some. Without a policy camlib stops with an
error naming the arc instead of emitting a slightly wrong circle. Choose one:

- `lowering=LoweringOptions(native_arcs=True)` (used above, recommended): keep
  every exactly representable arc as G2/G3 and write only the others as checked
  line segments, like the "linearize" fallback of commercial post processors.
  `run.status()["posting"]["arc_output"]` reports how many arcs took each form
  and why.
- `Machine(..., decimals=4)` if your controller accepts that precision.
- `lowering=LoweringOptions()`: all curves as checked line segments.

## Saving and reopening

A finished run can be saved and reopened later without replanning; see
[Saving and reopening runs](persistence.md).

## Next

- [Operations](operations.md): every operation, its parameters and limits.
- [Postprocessors](posts.md): machines, controllers and writing your own post.
- [Capabilities and limits](library-capabilities.md).
