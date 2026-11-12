# Centroid CNC12 plasma post

[`examples/centroid_plasma_post.py`](../examples/centroid_plasma_post.py)
writes `ThermalCut` programs for Centroid Acorn/AcornSix/Hickory plasma
tables with the Centroid THC, in the format of the Centroid Acorn CNC Plasma
Users Manual (CNC12 v5.1+, rev 54: program format p. 3, plasma M-codes p. 48,
G-codes p. 49, Intercon example p. 94), and reads its own output back for
checking.

```python
import math
import centroid_plasma_post as c

torch = camlib.Torch("torch", kerf_width=1.5)
# camlib assumes nothing about the machine: declare it.
declared = dict(
    output_units="mm",
    decimals=3,
    max_feed_mm_min=15000,
    max_spindle_rpm=1,
    arcs=True,
    helices=False,
    max_sweep_rad=math.tau,
    max_tool_number=1,
    max_length_register=1,
    tool_changes=True,
    rapid_mm_min=15000,
)
machine = c.define_machine("Plasma table", ToolSetting.for_tool(torch, number=1, length_register=1,
                                                                 position_after_mm=start), **declared)
program = job.post_nc("sheet", machine=machine, post=c.CentroidPlasmaPost,
                      decoder=c.CentroidPlasmaDecoder,
                      lowering=camlib.LoweringOptions(native_arcs=True))
```

| Item | Output | Premise |
| --- | --- | --- |
| Header | `G21 G90 G17 G94 G40 G49 G80`, `G52 X0 Y0 Z0`, `G61`, `M5 M9`, `G54` | |
| Torch / profile | `M65` | Loads the material profile selected in the Profile Manager (pierce height, pierce delay, cut height, THC mode, default feed) |
| Each cut | `G0 X Y`, `M61`, the cut, `M62` | M61: touch-off and pierce cycle, then cut height; THC per the profile's THC mode and active delay. M62: torch off, raise to the safe height |
| Feeds | `F` on every cutting move | Overrides the profile's feed (the manual allows F in the program) |
| Z | None | Plan with `height_mode="fixed"` and start the job with the torch at the cut height, so no Z word is written; a Z move is refused |
| End | `M30` | |

A `ThermalMark` is refused: the post declares no marking method for CNC12
(`M61` runs the cutting profile).

Plan with `pierce_delay_s=0` and `height_control=False` (the post refuses a
dwell or THC command: both belong to the profile). Arcs need exactly
consistent words (`radius_tolerance_mm=0`; I/J incremental, Parameter 2 bit
0 clear): post with `LoweringOptions(native_arcs=True)`. Rapids run with the
torch raised by `M62`; CNC12 may move the axes independently, which does not
matter over a flat plate (tipped-up parts are not modeled). Verify on the
machine before cutting.
