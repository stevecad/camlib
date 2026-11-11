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
| Tool | `T<tt><oo>` (`T0101`) | A new tool number runs the tool change (Z home, spindle and coolant off): each tool's `position_after_mm` is where the change leaves it, the job's initial position. No `G43`/`G49` |
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
