# FANUC lathe post (G-code system A)

[`examples/fanuc_lathe_post.py`](../examples/fanuc_lathe_post.py) writes
lathe setup programs for FANUC T-series lathes (0i-TF and similar) in G-code
system A, and reads its own output back for checking. Reference: Autodesk's
FANUC Turning post (`fanuc turning.cps`, revision 44226) for the program
structure and words; FANUC's manual for your control (e.g. B-64604EN) and the
machine builder's documentation are the authority for the premises below.

```python
import math
import fanuc_lathe_post as f

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
machine = f.define_machine("Lathe", tools, max_spindle_rpm=4000, program_number=1001, **declared)
nc = job.post_nc("lathe", machine=machine, post=f.FanucLathePost,
                 decoder=f.FanucLatheDecoder).text
```

| Item | Output | Premise |
| --- | --- | --- |
| Framing | `%`, `O1001` … `M30`, `%` | |
| Header | `G21 G18 G98 G97 G40 G80`, `M5 M9`, `G54` | System A: no `G90` (it is the diameter cutting cycle; X/Z absolute, U/W incremental); no G52 local system, nose-radius compensation or path smoothing active |
| Tool | `T<tt><oo>` (`T0101`) | Offsets touched off on a diameter and a face (the program moves the imaginary tip); each tool parked at the job's start position for its change |
| Spindle | `M3 S<rpm>`, or `G50 S<max>` then `G96 S<m/min> M3`; `G97` before an rpm again | |
| Feeds | `G99 F<mm/rev>` with `TurningSetup(feed_per_rev=True)`, else `G98` and `F` per minute | |
| Words | X diameters, `I` and `K` radius values (incremental); every dimensional word with a decimal point | Parameter DIA (No. 1006#3) for X |
| Rapids | `G0` along X or Z only | G00 may run each axis at its own rate (parameter LRP, No. 1401#1) |
| Threading | `G32 X Z F<lead>` | |
| Nose compensation | `TurnFinish(compensation="wear")`: `G1 G42 X Z F` on the entry, `G1 G40 X Z F` on the retract (G41 inside) | The offset's R holds only the wear, its T the tip direction |

Finishing follows the profile's arcs, and the nose rolling round sharp
corners, with `G2`/`G3` blocks (`TurnFinish`, `TurnBoreFinish`). An arc
turning from +Z toward +X is `G3`, the other way `G2`: counterclockwise and
clockwise seen from +Y of the right-handed X, Y, Z system (FANUC operator's
manual, circular interpolation G02/G03 in the ZpXp plane). X is a diameter
and `I`, `K` are radius values from the start to the centre. Premise: X+
points from the spindle axis toward the tool (a rear turret as drawn in the
manual, or a front turret whose picture is mirrored; the words are the
same). Check the direction on your machine before cutting.

Arcs need exactly consistent words (`radius_tolerance_mm=0`): post with
`LoweringOptions(native_arcs=True)` if an arc is refused (the arcs that
cannot be written exactly at the machine's decimals become checked line
segments), or `LoweringOptions()` for lines only. No controller
connection; verify on the machine before cutting.
