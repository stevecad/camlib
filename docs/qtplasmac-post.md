# LinuxCNC QtPlasmaC post

[`examples/qtplasmac_post.py`](../examples/qtplasmac_post.py) writes
`ThermalCut` and `ThermalMark` programs for LinuxCNC 2.9 QtPlasmaC and reads its own output back
for checking. Reference: the LinuxCNC 2.9 QtPlasmaC documentation (G-code
conventions, THC codes, G-code filter).

```python
import math
import qtplasmac_post as q

torch = camlib.Torch("torch", kerf_width=1.5)
setting = ToolSetting.for_tool(torch, number=1, length_register=1, position_after_mm=start)
# camlib assumes nothing about the machine: declare it.
declared = dict(
    output_units="mm",
    decimals=3,
    max_feed_mm_min=15000,
    max_spindle_rpm=1,
    planes=("xy",),
    arcs=True,
    helices=False,
    max_sweep_rad=math.tau,
    max_tool_number=1,
    max_length_register=1,
    tool_changes=True,
    rapid_mm_min=15000,
)
machine = q.define_machine("Plasma table", setting, blend_mm=0.005, **declared)
program = job.post_nc("sheet", machine=machine, post=q.QtPlasmaCPost, decoder=q.QtPlasmaCDecoder,
                      lowering=camlib.LoweringOptions(native_arcs=True))
```

| Item | Output | Premise |
| --- | --- | --- |
| Preamble | `G21 G90 G17 G94 G40 G49 G80 G92.1`, `G64 P<blend_mm>`, `M52 P1` | The documentation requires `M52 P1` (paused motion). `G64 P` is written explicitly (the loader would insert `G64 P0.1`); `blend_mm` must be at most half the machine tolerance |
| Torch | `T0 M6`, `G43 H0` | QtPlasmaC tool 0 is the torch (T1 is the scribe); camlib's table entry is number 1, register 1 |
| Torch on / off | `M3 $0 S1` / `M5 $0` | |
| Height control | `M62 P2` off, `M63 P2` on (synchronized with motion) | `ThermalCut(height_control=True)` |
| Z | One Z (the cut height) | QtPlasmaC removes Z motion and handles probing, pierce height, pierce delay and cut height from the material file: plan with `height_mode="fixed"`, `pierce_delay_s=0` |
| Feeds | `F` words | QtPlasmaC warns when they differ from the material's cut feed rate |

## Marking

QtPlasmaC documents no marking command for the torch. It documents materials
(the material file: cut amps, and a cut mode `3 = Gouge/Mark`, both acted on
with PowerMax communications) and automatic material changes: `M190 P<n>`,
then `M66 P3 L3 Q<seconds>` to wait for the change, then the feed and
`M3 $0 S1`, "in the order shown". So a `ThermalMark` is posted only on a
machine whose owner declares the two materials of its own material file:

```python
machine = q.define_machine("Plasma table", setting, blend_mm=0.005,
                           mark_material=7, cut_material=3, material_wait_s=2, **declared)
```

| Item | Output | Premise |
| --- | --- | --- |
| Before the first mark | `M190 P<mark_material>`, `M66 P3 L3 Q<material_wait_s>`, `M3 $0 S1` | The marking material holds this machine's marking current, cut mode, heights and feed; camlib only selects it |
| Before the first cut after marks, and before the first cut of any program | `M190 P<cut_material>`, `M66 P3 L3 Q<material_wait_s>`, `M3 $0 S1` | A material is selected once and stays until the other is needed |
| Feed | `F` on the first fed move after the change | QtPlasmaC warns when it differs from the material's cut feed rate |

Plan the mark with `power_percent=100` (the torch fires at the material's
current; the post refuses another value) and `height_mode="fixed"`. The
decoder reads the selected material from the `M190`/`M66` blocks alone
(only the two declared numbers, only with the torch off, each `M190` followed
by its `M66`): a torch-on under the marking material decodes as a marking
beam, under the cutting material as a cutting beam, and before any selection
is refused. A machine that declares no materials writes cut programs exactly
as before, and refuses a mark. Whether the mark is light enough is a property
of the material entry, not of the program: prove it on scrap.

Not written: the scribe (tool 1, `T1 M6`, `M3 $1 S1`: a second tool with its
own offsets) and spotting (`M3 $2 S1`).

## Not written

Velocity percentages (`M67`), hole sensing and QtPlasmaC's own overcut
(camlib plans small-hole feeds and overburn itself), cutter compensation
`G41.1`/`G42.1` (QtPlasmaC disables THC and overcut with it), and material
selection other than for marking as above. Verify on the machine before
cutting.
