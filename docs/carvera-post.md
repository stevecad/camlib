# Makera Carvera milling post

`examples/carvera_post.py` supplies `ReferenceMachine`, `define_machine`,
`CarveraPost` and `CarveraDecoder`. The Rust/JSON equivalent selects
`"controller": {"name": "carvera", "initial_gauge_length_mm": ...,
"unmodified_coordinates": true, "automatic_tool_length": true}` inside the
ordinary `Machine.to_dict()` shape. Other machine fields retain their existing
locations. See the complete synthetic [drilling input](../fixtures/posts/rust/carvera-drill.json)
and [native-arc input](../fixtures/posts/rust/carvera-arcs.json). These fixtures
are test setups, not physical machine presets or cutting recommendations.

Python callers must explicitly pass `post=CarveraPost, decoder=CarveraDecoder`
to `job.post_nc`; omitted/default or mismatched pairs refuse. Rust selects both
from the controller declaration, through `post_job`, `cam-post` or the library.
Every block uses the existing independent decode/intent comparison, posted
stock replay, machine checks and export gate. No machine communication occurs.

## Controller decisions and sources

The primary references are the Community firmware's
[G-code table](https://carvera-community.gitbook.io/docs/firmware/supported-commands/gcodes),
[M-code table](https://carvera-community.gitbook.io/docs/firmware/supported-commands/mcodes),
[M6 documentation](https://carvera-community.gitbook.io/docs/firmware/supported-commands/mcodes/tool-change)
and Makera's [Carvera.cps](https://github.com/MakeraInc/CarveraProfiles/blob/main/CAM_Post_Processors/Fusion360-profiles/Carvera.cps)
(Community Post v1.4.6). No Makera implementation is copied.

- Initialize `G21 G90 G17 G94` (separate blocks), explicitly stop
  spindle and air with M5/M9, then select the source work offset (G54–G59). G20 is documented to halt the firmware: inch
  output refuses, without conversion.
- Write `T<n> M6`; M6 measures and applies tool length. No G43, G49 or H
  activation is written. `automatic_tool_length=true` declares calibration
  enabled and that measured lengths agree with the supplied assemblies.
  Post-change tip positions are explicit table poses; actual changer and
  measurement routes require the caller's machine crash model. This stock ATC
  reference requires both `max_tool_number` and metadata `max_length_register`
  at most 6, following the post's `error6` policy; manual-change profiles are
  outside this contract. `C1` is deliberately omitted: the Community M6 docs
  support C, but Makera's stock
  [ATCHandler.cpp](https://github.com/MakeraInc/CarveraFirmware/blob/master/src/modules/tools/atc/ATCHandler.cpp)
  M6 handler does not read C. `Carvera.cps` emits C1 only in its Community
  branch. Automatic calibration remains an explicit setup premise, including
  the length already active if stock firmware skips an unchanged tool.
- Write `G53 G0 Z-3` before M6 and before M30. The explicit G0 follows the
  firmware table's G53 semantics and Makera's `writeRetract` helper. The local
  research note's bare `G53 Z-3` omits that motion selection. The current
  Makera closing routine actually calls optional G28 through a different
  `writeRetract` branch; this reference consistently uses the explicit Z=-3
  policy requested for camlib.
- G2/G3 use incremental IJK and G17/G18/G19, with arcs/helices at most 180
  degrees (`maximumCircularSweep=toRad(180)` in Makera's post). The Community
  [Robot.cpp](https://github.com/Carvera-Community/Carvera_Community_Firmware/blob/master/src/modules/robot/Robot.cpp)
  full-circle branch bypasses the XZ direction flip used for partial arcs;
  G18 full circles/helices therefore have inconsistent direction and are
  excluded. Plane words are emitted only when the active plane changes. No documented
  radius-mismatch allowance is assumed: the declared radius tolerance is zero.
  Arcs that cannot be represented at the caller's precision refuse unless the
  caller explicitly requests the existing verified lowering policy. The native
  arc job uses integer arc endpoints/centres derived from binary-exact geometry;
  its tests require native output with zero fallback segments.
- Drilling uses explicit source moves, following the existing no-cycle dialect
  path (also used by GRBL). Canned-cycle words are absent. An operation may
  request cycles, but only its independently verified explicit travel is written;
  no alternate machining strategy is invented.
- M3/M5 are clockwise spindle/off; M4 is documented as unwired on stock Carvera
  and refuses. M7/M9 mean compressed air/off. Callers opt into `mist=True` and
  use coolant `mist` for AIR; there is no liquid mist/flood mapping. M8 refuses.
  Every M7/M9 is immediately preceded by M400, including initialization and
  finish. The firmware's
  [Switch.cpp](https://github.com/Carvera-Community/Carvera_Community_Firmware/blob/master/src/modules/tools/switch/Switch.cpp)
  changes outputs when parsing, without waiting for motion. Makera's coolant
  table specifies `[400,7]` / `[400,9]`. Both independent decoders reject air
  changes without the barrier, or M400 followed by any other block.
  M30 has no stopping side effects according to the table, so M5/M9 are handled
  explicitly. G4 P is seconds. No M0/M1, compensation, probing, tapping, beam,
  vacuum, rotary axes or undocumented path-control words are declared.

## Setup premises and retract contract

`unmodified_coordinates=true` declares zero G92, no levelling/flex compensation,
rotation or scaling, and ordinary Cartesian stored offsets matching
`work_to_machine`. There is no zero junction-deviation or minimum-planner-speed
premise. The firmware
[Planner.cpp](https://github.com/Carvera-Community/Carvera_Community_Firmware/blob/master/src/modules/robot/Planner.cpp)
explains that its corner-speed calculation "does not actually deviate" from the
path. As in the GRBL reference (`examples/grbl_post.py`), fixed `exact_stop`
readiness denotes the nominal geometric path model, not a command to stop at
every junction. No G61/G64, G40, G80, G91.1 or G97 is emitted. Firmware arc
segmentation and binary32 arithmetic are not emulated by this nominal model.
The post does not qualify controller firmware or machine configuration.

The setup must **plan its retracts**, including the initial and final safe poses.
At each G53 block the independent decoder compares at the posted resolution:

`abs(work_tip_z + work_to_machine.origin_mm[2] + current_gauge_length_mm + 3) <= 0.5 / 10**decimals`.

This accepts binary64 cancellation noise such as `25.4 - 88.4 + 60`, while
refusing a discrepancy greater than half one output unit. It does not relax
the independent motion/stock checks or the machine's posting tolerance.

Before the first M6, `initial_gauge_length_mm` declares the loaded assembly's
length (zero for an empty spindle). Afterwards it comes from the decoded tool
table. These safety blocks require the planned retract pose at the declared output
resolution. A setup
ending lower, or changing tools at another height, refuses with “plan the
retract”; the post never silently adds an unchecked machine-coordinate move.
`Setup(end_position=EndPosition(gauge_z_mm=<the work Z of machine Z-3>))`
plans the final retract: the program then ends at that gauge height (and at
the stated XY, when one is given) as checked moves, and the post writes its
`G53 G0 Z-3` there before M30.
This is a deliberate limitation of this first reference, rather than a claim
that arbitrary setup clearances are Carvera-safe. All tool changes require
spindle and air explicitly off. Physical routes, offsets and measured tool
lengths remain supplied premises; no real-machine execution is established.
