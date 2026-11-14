# WinCNC plasma post

[`examples/wincnc_plasma_post.py`](../examples/wincnc_plasma_post.py) writes
`ThermalCut` programs for WinCNC 3.x plasma tables with WinCNC torch height
control, and reads its own output back for checking. References: the WinCNC
Users Guide 3.1r5 (command reference, plasma torch height control) and
WinCNC's own plasma posts (SheetCAM "WinCNC Plasma THC", Fusion
`wincnc_plasma.cps`).

```python
import wincnc_plasma_post as w

torch = camlib.Torch("torch", kerf_width=1.5)
# camlib assumes nothing about the machine: declare it.
declared = dict(
    output_units="mm",
    decimals=3,
    max_feed_mm_min=15000,
    max_spindle_rpm=1,
    arcs=True,
    helices=False,
    max_tool_number=1,
    max_length_register=1,
    tool_changes=True,
    rapid_mm_min=15000,
)
machine = w.define_machine("Plasma table", ToolSetting.for_tool(torch, number=1, length_register=1,
                                                                 position_after_mm=start), **declared)
program = job.post_nc("sheet", machine=machine, post=w.WinCNCPlasmaPost,
                      decoder=w.WinCNCPlasmaDecoder,
                      lowering=camlib.LoweringOptions(native_arcs=True))
```

| Item | Output | Premise |
| --- | --- | --- |
| Units | `G22` (mm) or `G20` (inch) | On WinCNC `G21` means centimetres |
| Header | `G90 G22 G40 G49`, then `G92` | `G92` alone removes local coordinates: written once, as the work offset (WinCNC's `G54` selects a Z head and is never written) |
| Start | `M65` | Plasma setup macro |
| Each cut | `G0 X Y`, `M61`, the cut, `M62` | M61 fires the torch: touch, pierce height, pierce delay, cut height, THC per the material setup; M62 turns it off |
| Arcs | `G2`/`G3 X Y I J`, at most half a turn | I/J are always relative in WinCNC; no plane words (XY only) |
| Inches | `output_units="inch"`, `decimals=4`: `G20`, every word in inches | A half-turn arc is kept as an arc only where its rounded inch words still span no more than half a turn; otherwise it is written as checked lines (with `LoweringOptions(native_arcs=True)`) |
| Feeds | `F` on every cutting move | |
| Z | None | Plan with `height_mode="fixed"` and start with the torch at the cut height |
| End | `M66` | End macro |

M61–M66 are user-defined macros in WinCNC's `CNC.MAC`. The defaults are the
codes of WinCNC's own posts; pass your machine's codes:
`define_machine(..., start_code="M65", torch_on_code="M61",
torch_off_code="M62", end_code="M66")`.

A `ThermalMark` is refused: the post declares no marking method for WinCNC
(`M61` fires the torch with the cutting material setup).

Torch height control runs in WinCNC (arc voltage); its settings (target
volts, THC active delay, THC mode) and the pierce and cut heights are in the
material setup. Plan with `pierce_delay_s=0` and `height_control=False`: the
post refuses a dwell or THC command. Arcs need exactly consistent words:
post with `LoweringOptions(native_arcs=True)`. WinCNC's velocity smoothing
(G9) must keep the path within the machine tolerance. Verify on the machine
before cutting.
