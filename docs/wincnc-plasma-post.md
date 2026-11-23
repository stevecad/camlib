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
| Z | Controller-owned; explicit return to `THR` for edge starts | Plan with `height_mode="fixed"` and start with the torch at the cut height |
| End | `M66` | End macro |

M61–M66 are user-defined macros in WinCNC's `CNC.MAC`. The defaults are the
codes of WinCNC's own posts; pass your machine's codes:
`define_machine(..., start_code="M65", torch_on_code="M61",
torch_off_code="M62", end_code="M66")`.

Tables whose own programs carry no start macro and run in the coordinates
the operator sets at the sheet (ShopSabre plasma tables as ProNest posts for
them: `G90`, each cut between `M61` and `M62`, `M66`) are declared with
`define_machine(..., start_code=None, local_coordinates="keep")`:

| Declared | Written | Premise |
| --- | --- | --- |
| `local_coordinates="keep"` | `(WORK ZERO AS SET AT THE MACHINE)` in place of `G92` | `G92` alone removes local coordinates; on a table zeroed at the sheet with them the program must not. The program then runs wherever the operator set X0 Y0 |
| `start_code=None` | `(TORCH <id>)` in place of `M65` | The table has no plasma setup macro: the torch is its only tool and the material setup is chosen at the control |

`program_height_control=True` posts `ThermalCut(height_control=True)` with
WinCNC's own commands, `L36` (Torch Height Control On) and `L37` (Torch
Height Control Off; Users Guide 3.1r5, Command Reference), as ProNest's
programs for these tables carry them: on over each loop's cut, off over
leads, small holes and slowed corners. `L37` is written before the first
torch-on, so the program does not depend on the state it starts in, and `L36`
before the end macro, so height control is left on. Without the declaration a
height control command is refused, as before.

`arc_radius_tolerance_mm` (default 0) declares the start/end radius mismatch
arc words may have. WinCNC accepts arcs within its `arc_err` setting
(`WINCNC.INI`; Users Guide 3.1r5, System Configuration: "allowable errors in
arc specifications", default .01 unit of measure); declare at most that, and
at most 0.005 mm. With it, arcs whose rounded words differ that little stay
arcs (see [posts](posts.md): `arc_word_mismatch_mm`); at 0 only exactly
consistent words are written and other arcs become checked lines, which at
four decimals of an inch is most shallow arcs.

`material_feed_mm_min` declares the "Feed Rate" of the material setup chosen at
the control (Users Guide, Plasma Torch Height Control: "XY axes feed rate for
this material"). A program with no `F` word cuts at that feed, which is how
ProNest's programs for these tables are written. With it declared no `F` word
is written, the decoder cuts at the declared feed, and a cut planned at any
other feed (a `small_holes` or corner feed) is refused, naming the feed: the
program could not say it. Left undeclared, every cutting move carries its `F`
word, which overrides the material's feed.

Both comments state the premise to the operator; the decoder reads them as
the work offset and the torch, and refuses a `G92` on such a machine.

## Kerf applied by the control (`compensation="control"`)

`ThermalCut(compensation="control")` writes the part side of the kerf path
(the part outline, beyond the boundary margin) and leaves the kerf to the
control. How the control takes it is declared, as the table's owner knows it:

| Declared | Written | Source |
| --- | --- | --- |
| `control_kerf="g41"` | `G41 O<kerf/2>` on a block of its own before each straight lead-in, `G40` on its own before each straight lead-out; `O` in output units at the linear resolution | WinCNC Users Guide, Controller 3.1, revision 3.1.04, pp. 138-139: "G41 Cutter Compensation Left - G41 O# ... Starts to the left on the first move. O# is the compensation amount on machines without a tool changer", G42 likewise to the right, "G40 Cutter Compensation Off". ShopSabre's own plasma post (Autodesk post library, "shopsabre plasma", $Revision 44226, lines 490-504) writes `G41 O<kerfWidth/2>` alone, then the G1 lead-in, and `G40` alone, and refuses a compensation change on a rapid or an arc |
| `control_kerf="material", material_kerf_mm=...` | No compensation word | **The owner's declaration of their control**, not a WinCNC manual fact: the control applies half the kerf of the material setup chosen at it, to the left of travel, from the first move after the torch fires (`M61`) to the last before it goes off (`M62`), and the parts come out on size. Only `compensation="control"` loops can be cut on such a table (no crop, skeleton, chain or micro-joint cuts, which the control would offset too), in the usual direction (`reverse=False`), with the torch's kerf equal to `material_kerf_mm` |
| `control_kerf=None` (default) | | `compensation="control"` is refused |

Wear compensation (`compensation="wear"`, `G41/G42 D<register>` with the head's diameter register, the control trimming
only the wear) posts on a table that declares no `control_kerf` or `control_kerf="g41"`. It is refused with
`control_kerf="material"`: that control offsets every cut by its material setup's kerf, so it would offset twice. The
decoder refuses an amount word (`O`) unless `control_kerf="g41"`, and any G41/G42 with `control_kerf="material"` (#391).

The G41/G42 `O#` behaviour cited is documented for revision 3.1.04 of the
Users Guide (Controller 3.1); earlier WinCNC versions are not covered by
that citation, and no claim is made for them.

The premises of `WINCNC.INI` (Users Guide p. 72, `cutcomp=T# D# R# C#`):
`T0` (the default: "does not insert entry move") and a positive `D` (a
negative one swaps G41 and G42). The decoder replays the others:
`cutcomp_arc_radius_mm` and `cutcomp_arc_angle_deg` (R and C, defaults 200 in
and 0.5 degrees: an arc of a larger radius turning less is cut as a line), and
`cutcomp_full_circle_mm` (F, Versions Information List 3.1.07b: an arc whose
ends are closer is taken as a full circle; no documented default, so it must
be declared, 0 where it is not set, before a compensated arc is posted).

What camlib plans and checks (see [ThermalCut](operations.md)): the beam
path the control cuts from the written path. At an outside corner WinCNC
joins its offset elements at their intersection (Versions Information List
3.1.01a: arcs are offset and "a new intersection point" calculated), so the
kerf paths of the two edges run on to where they meet; the arc join a
control could take instead lies inside it, and both are proven clear of
every part. The pierce and the end of each lead-out are on the written path;
the written straight leads run along the planned ones, tangent into the lead
arcs, so where the control ends its start-up does not depend on its entry
type. The decoder reads each stretch from the compensation turning on to the
move after it turns off, and replays the control's offset of the written
words; every replayed move is compared with the plan.

A `ThermalMark` is refused: the post declares no marking method for WinCNC
(`M61` fires the torch with the cutting material setup).

Torch height control runs in WinCNC (arc voltage); its settings (target
volts, THC active delay, THC mode) and the pierce and cut heights are in the
material setup. Plan with `pierce_delay_s=0` and `height_control=False`: the
post refuses a dwell, and a THC command unless the machine declares
`program_height_control=True`. Arcs need exactly consistent words:
post with `LoweringOptions(native_arcs=True)`. WinCNC's velocity smoothing
(G9) must keep the path within the machine tolerance. Verify on the machine
before cutting.

Edge starts (`BeamStart`) are opt-in with
`edge_start_cut_z_mm=...` and `program_height_control=True`. Declare the
cutting Z in **the plan's coordinates**, for example 7.5 for stock whose
top is Z6 and a 1.5 mm cutting gap. The selected material's `THR` must put
the torch at that physical height, and the sheet's Z reference must already
be established before the program starts. Every edge start is checked
against the declared Z. Plan with fixed height and `height_control=True`
so THC stays off over the lead and can turn on over the part.

The sequence is derived from the plasma configuration distributed in
[WinCNC's public demo installer](https://www.wincnc.net/webfiles/Demo/WinDemo.exe)
(retrieved 2026-11-15): `torchon.mac` calls `M63` (touch-off) before `L38`,
then separately waits for arc-good and the pierce delay; `torchcut.mac`
returns to cut height with `L90 G0 Z{THR}`. The post writes:

```text
L37
L90 G0 Z{THR}
~L38
```

This disables THC, returns from the controller's travel/lift height to the
selected material's cut height, and fires without the touch-off or pierce
steps. `~L38` explicitly invokes the built-in command even if a macro
overrides `L38` (Users Guide 3.1r5, Macro File Configuration, p. 112).
The edge-start delay is written as `G4 D...` in seconds; the normal material
pierce delay is not applied. With `material_feed_mm_min` declared, an edge
start also writes `F{THF}` to select the material's feed, which `M61` would
otherwise do. This reads the controller setting instead of supplying a
numeric feed. Other cutting moves still carry no `F` words in this mode.

Ordinary pierces in the same program retain the declared torch-on macro
(`M61` by default), and both start types use the declared torch-off macro.
The independent decoder requires cut-height positioning before native
torch-on, verifies THC is off, checks the declared height and material-feed
sequence, and preserves `BeamStart` separately from a pierce. No table
macro file is needed to select this documented native sequence. As with
ordinary fixed-height posting, controller-owned Z moves and the material
setup are declared machine behavior, not measured by camlib.

Without `edge_start_cut_z_mm`, edge starts remain refused. The public
configuration establishes controller semantics, not physical validation on
a ShopSabre; no real-machine cut has been performed for this implementation.

Source archive SHA-256:
`0e12a83b508b4306ac262fcf8829b973f4184add68715610501fe336edb28887`.
