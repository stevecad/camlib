# Centroid CNC12 lathe post

[`examples/centroid_lathe_post.py`](../examples/centroid_lathe_post.py)
writes lathe setup programs for Centroid CNC12 lathes (Acorn, AcornSix,
Hickory, Oak) in the format of the Centroid CNC12 Lathe Operator's Manual
(T word 8.2.9; G-codes chapter 9; M-codes chapter 10), and reads its own
output back for checking.

```python
import math
import centroid_lathe_post as c

header = job.compile().setup("lathe").program_header()   # a lathe Setup
tools = tuple(ToolSetting.for_tool(t, number=i, length_register=i,
                                   position_after_mm=header.initial_position_mm)
              for i, t in enumerate(header.tools, 1))
# camlib assumes nothing about the machine: declare it.
declared = dict(
    output_units="mm",
    decimals=3,
    max_feed_mm_min=5000,
    arcs=True,
    helices=False,
    max_sweep_rad=math.tau,
    max_tool_number=12,
    max_length_register=12,
    tool_changes=True,
    flood=True,
    mist=False,
    rapid_mm_min=8000,
)
machine = c.define_machine("Lathe", tools, max_spindle_rpm=4000, **declared)
nc = job.post_nc("lathe", machine=machine, post=c.CentroidLathePost,
                 decoder=c.CentroidLatheDecoder).text
```

| Item | Output | Premise |
| --- | --- | --- |
| Header | `G21 G18 G98 G97 G40 G80`, `G52 X0 Z0`, `G61`, `M5 M9`, `G54` | No `G90`: on this lathe G90 is the diameter cutting cycle; X/Z are always absolute. No `G91.1`: I/K are always incremental |
| Tool | `T<tt><oo>` (`T0101`) | A new tool number runs the machine's tool-change macro (spindle and coolant off). Where it leaves the tool is declared, see [Tool change](#tool-change). No `G43`/`G49` |
| Spindle | `M3 S<rpm>`, or `G50 S<max>` then `G96 S<m/min> M3`; `G97` before an rpm again | |
| Feeds | `G99 F<mm/rev>` with `TurningSetup(feed_per_rev=True)`, else `G98` and `F` per minute | |
| X words | Diameters; arc `I` and `K` radius values | Parameter 55 = 0 (diameter mode) |
| Tool point | Each tool's imaginary tip (`TurningSetup(controlled_point="tip")`, the default) | Offsets touched off on a diameter and a face (the manual's tool offset procedure); `"nose_center"` if the offsets are of the nose center |
| Rapids | `G0` along X or Z only | G0 runs each axis at its own rate; a rapid moving both is refused |
| Threading | `G32 X Z F<lead>` | Spindle encoder |
| End | `M9`, `M5`, `M30` | M30 ends like M2 (Z home, then restart) |
| Nose compensation | `TurnFinish(compensation="wear")`: `G42 T<oo>` (outside; `G41` inside) on its own line before the entry, `G40` after the cut | Manual 9.15: the T word names the Offset Library row whose nose radius applies (the active offset); that row's radius holds only the wear |

The planner switches coolant off before each tool change and on again after
it.

## Tool change

The `T<tt><oo>` word is the same on every CNC12 lathe; what the tool-change
macro behind it does is the machine's own (a turret that indexes, a gang
plate, a quick-change tool post). camlib assumes none of them: declare it on
the setup and on the machine (see
[Lathe tool changes](operations.md#lathe-tool-changes)).

| The macro | Setup | `define_machine(...)` |
|---|---|---|
| Brings each new tool's controlled point to the job's start position | `tool_change=None` (the tip kept) | `"table"` (the default here), each `position_after_mm=header.initial_position_mm`, as above |
| Moves no axis and only applies the new offset; the operator swaps the holder while the program waits (a quick-change tool post) | `TurnToolChange.carriage_kept()` and each `TurnToolSetting.carriage_to_tip_mm` | `tool_change_position_mode="preserve_carriage"`, each `ToolSetting.carriage_to_tip_mm` |
| The same, with the offsets only in the control's Offset Library | `TurnToolChange.carriage_kept(tip_within_mm=(radius, z))` | `tool_change_position_mode="preserve_carriage"`, `tool_change_tip_within_mm=(radius, z)` |
| Moves the carriage to a fixed machine X, Z | `TurnToolChange.machine_position(radius_mm, z_mm)` and each `carriage_to_tip_mm` | `"table"`, each `position_after_mm=change.tip_mm(setting)` |

A quick-change tool post whose macro moves nothing:

```python
from camlib.turning import TurnToolChange

settings = (camlib.TurnToolSetting(turning_tool, 1, 1, carriage_to_tip_mm=(-25, -40)),
            camlib.TurnToolSetting(blade, 2, 2, carriage_to_tip_mm=(-22, -52)))
lathe = camlib.Setup("lathe", tools=settings, tool_change=TurnToolChange.carriage_kept(), ...)
offsets = {s.tool.id: s.carriage_to_tip_mm for s in settings}
tools = tuple(ToolSetting.for_tool(t, number=i, length_register=i,
                                   carriage_to_tip_mm=offsets[t.id])
              for i, t in enumerate(header.tools, 1))
machine = c.define_machine("Lathe", tools, tool_change_position_mode="preserve_carriage",
                           max_spindle_rpm=4000, **declared)
```

`carriage_to_tip_mm` is each holder's controlled point from one common
reference on the carriage, `(radius, z)` in millimetres along the program
axes: the same geometry the control's Offset Library holds for the `T` word,
stated to camlib in that form (X as a radius). With the carriage kept, only
the differences between holders matter.

The program text does not say where the tool is after `T0202`: the next
block is an absolute move from wherever the change left the tool. With the
carriage kept that is the old position moved by the offset difference. The
planner checks the new tool standing there (against the bar, the part and
the chuck) and starts its approach there, and the decoder reads the posted
program back from the same point (a program planned for one tool change and
read with another is refused). An axis the first blocks after the `T` word
do not move gets no word, as after any tool change here: a word would
command it through the new offset. So two setups that differ only in the
tool change can post the same blocks; what differs is where camlib knows
the tool to be while they run, and so what it refuses. Before each change
the program stops the spindle and coolant and rapids the old tool to the
start position; it does not write a stop for the swap: that wait is the
macro's.

### Offsets only in the Offset Library

The `T<tt><oo>` word applies the offset of row `oo` of the Offset Library
(manual 8.2.9); the program never states the offsets themselves. Keeping a
second copy of them for camlib (`carriage_to_tip_mm`), in step with every
touch-off, is not something a shop does. State one bound for the tool post
instead: how far any tool's tip can be from any other's.

```python
change = TurnToolChange.carriage_kept(tip_within_mm=(30, 40))     # radius, z in mm
lathe = camlib.Setup("lathe", tools=(camlib.TurnToolSetting(turning_tool, 1, 1),
                                     camlib.TurnToolSetting(blade, 2, 2)),
                     initial_position_mm=(60, 0, 80), tool_change=change, ...)
tools = tuple(ToolSetting.for_tool(t, number=i, length_register=i)
              for i, t in enumerate(header.tools, 1))
machine = c.define_machine("Lathe", tools, tool_change_position_mode="preserve_carriage",
                           tool_change_tip_within_mm=(30, 40), max_spindle_rpm=4000, **declared)
```

The program then regroups after each tool change:

```
G0 X120 Z3.2       the turning tool out to the start radius,
G0 X120 Z80        then along Z to the start position (radius 60, z 80)
T0202              the offset changes, no axis moves: the blade's tip is
                   somewhere within 30 mm (radius) and 40 mm (z) of the start
G0 X120 Z80        the blade's tip to the start position
M3 S800
```

That rapid is the one `G0` block here that may move both axes at once: the
tool is not on a known line to begin with, and both axes have to be
commanded under the new offset. camlib has checked the blade, insert and
holder, everywhere in the box and so on every path the two axes can take to
the start position (see
[The carriage kept within a stated bound](operations.md#the-carriage-kept-within-a-stated-bound)):
a start position too close to the bar or the chuck for the bound is refused
before any program is written. Read back, the block after a `T` word must
be exactly this rapid; anything else is refused. The first tool of the
program is the one mounted at the start and gets no such block. The
premise is the same as with `carriage_to_tip_mm`: the tool-change macro
moves no axis. Check that on your machine; camlib cannot.

Finishing follows the profile's arcs, and the nose rolling round sharp
corners, with `G2`/`G3` blocks (`TurnFinish`, `TurnBoreFinish`). An arc
turning from +Z toward +X is `G3`, the other way `G2` (counterclockwise and
clockwise seen from +Y of the right-handed X, Y, Z system; CNC12 lathe
manual chapter 9, G2 and G3). X is a diameter and `I`, `K` are radius values
from the start to the centre. Premise: X+ points from the spindle axis
toward the tool. Check the direction on your machine before cutting.

Arcs need exactly consistent words (`radius_tolerance_mm=0`): post with
`LoweringOptions(native_arcs=True)` if an arc is refused (the arcs that
cannot be written exactly at the machine's decimals become checked line
segments), or `LoweringOptions()` for lines only. No controller
connection; verify on the machine before cutting.
