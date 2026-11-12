# Declared machine crash checking

`Machine.crash_model` is a `camlib.machine.MachineModel`. `PostedRun.export`
and `Job.post_nc` refuse output without it unless the caller explicitly passes
`accept={"unchecked_machine"}`. This acceptance is recorded in `accepted` and
`unchecked`, and the result's `machine` report states that no machine check ran.
Accepting `uncertain` does not accept a missing model or a definite collision.

`Job.post_nc` now uses the same actual decoded-output stock replay as `Job.post`.
Rounding that passes the post's geometric comparison can still cut into the
part; replay can refuse it. Material checks work through either export API.
The low-level `post_events`/`post_program` APIs are formatters with independent
intent checking. They have no stock or machine release gate and do not produce
a machine-clearance certificate.

As JSON, the model is the machine's `crash_model` in `Machine.to_dict()`, the
form the Rust post reads ([posts](posts.md#posting-without-python-rust-webassembly)).

## Declaration contract

Every `MachineModel` field is required. Physical values come from the owner,
machine builder and controller documentation; camlib supplies no machine.
All coordinates and geometry are in mm, including for inch NC output.
`version=1` and `units="mm"` identify the schema. See
`python/tests/test_machine.py` for complete **synthetic test** declarations of
a mill and a lathe, not dimensions suitable for a real machine.

| Input | Meaning |
| --- | --- |
| `id`, `revision`, `controller_reference` | Stable model identity and the manual/settings that establish its controller behavior |
| `process` | `milling` or `turning`; thermal equipment is refused |
| `axes` | Three `Axis(drive, min_mm, max_mm)`, XYZ machine-axis travel, limits included ([the travel rule](python.md#the-travel-rule)). Drive is `head`, `table` or `fixed`; a fixed axis has equal limits |
| `work_offset`, `work_to_machine` | Selected G54..G59.3 and the physical work-to-machine frame, matching the bound post definition. Work X/Y may rotate about upright Z; turning uses upright XZ. `Machine.frame` identifies the CAD/setup frame, not a second physical transform |
| `work_error_min_mm`, `work_error_max_mm` | A common calibration interval around the declared work frame, enclosing zero. Stock, fixtures and commanded work coordinates share this error; it cancels in relative checks where appropriate |
| `initial_axes_mm`, `initial_tool` | Actual expected initial machine-axis coordinates and preloaded tool ID, or explicit `None` for an empty spindle/turret station |
| `tools` | One `ToolCalibration` per program assembly: ID, H/edge register, `body_solids`, and ordered gauge-to-controlled-point XYZ bounds. Milling offsets are axial and contain the assembly gauge length. Lathe offsets are physical turret calibration, never the milling proxy's fake gauge geometry |
| `bodies` | `MachineBody(id, component, mount, solids, indexed_solids)` rigid convex unions. `Box` or closed outward-wound `ConvexMesh`; concave meshes must be decomposed by the caller. Open, degenerate, inconsistent or nonconvex meshes are refused |
| `absent_components` | Explicit reason for each machine component not fitted. It cannot also have a body |
| `allowed_contacts` | Pairs of IDs with a reason, for mechanical joints or clamping. No implicit exclusions, including for co-moving bodies. Contact between `@tool` and `@stock` cannot be excluded; declared clamping contacts such as chuck/stock may be allowed |
| `rapid_mode` | `linear`, or `axis_independent` for monotone independent-axis G0. The latter checks the entire endpoint XYZ box. Any possible collision is uncertain and refuses export. G1 feed traverses remain linear even if their source role is rapid |
| `tool_change_routes` | A `ToolChangeRoute` for every tool, including the first assignment: reference, absolute machine-axis waypoints before/after indexing, complete `index_envelopes`, and all body IDs those envelopes replace (including `@tool`) |
| `program_end_route_mm` | Absolute machine-axis waypoints for any controller-internal end/retract motion; an empty tuple explicitly declares none |
| `options` | Required numerical tolerance, spatial resolution, per-stage axis-test budget, finding cap and native `StockContactOptions`. Exhaustion or cancellation means incomplete and uncertain, never clear |

A mount is `fixed` (machine zero frame), `head` (moving carrier zero-axis frame),
`table` (moving table zero-axis frame), or `workpiece` (setup-local coordinates,
transformed by `work_to_machine` and carried by table axes). Positive table
motion moves the work negatively relative to the head. All body pairs are
checked unless expressly allowed. Bodies can include fixtures, vise jaws,
clamps, protected targets and auxiliary equipment as well as machine castings.
Do not omit the target part or a fixture because it was not configured in the
older optional prism checker.

Mills must declare a spindle/head, column, table and enclosure, or explicitly
explain each absence; spindle/head geometry is mandatory. Lathes must declare
or explicitly exclude turret, other tools, chuck, tailstock, quill, steady rest,
bed and enclosure; chuck geometry is mandatory. Turret and other-tool states
can vary with selection: `indexed_solids` maps **every** tool ID and the empty
state `""` to geometry, added to common `solids`. Put every changing body in
`replaces`. These bodies must be head-mounted. Include even an indexing body
that returns to the same final pose: its intermediate sweep still matters.

Waypoints contain three axis values; `None` retains that axis. Before-index
motion carries the old assembly; after-index motion carries the new one.
Index envelopes are a **caller-certified complete sweep**, including both tool
assemblies and all changing machine bodies. Endpoint containment is checked,
but does not prove that the intermediate indexing mechanism stays inside the
envelope. Camlib does not infer it from sampled rotary positions. The declared
route must finish at the independently decoded tool-change tip/gauge pose.

Milling tool/holder outer enclosures come from the existing `ToolAssembly`;
`body_solids` must be empty (a holder given as a model reaches the assembly as
its swept envelope: `camlib.tools.holder_sections_from_mesh`). Native turning
geometry shares the planner's hand,
seat, bore and controlled-point transforms; a turning holder given as a model
([Tool bodies from a model](operations.md#tool-bodies-from-a-model)) is
enclosed by the box of each of its slices. For curved face-grooving tools,
whose axisymmetric stock envelope does not locate the physical blade in 3D,
supply a complete tip-relative convex `body_solids` enclosure. Such custom
bodies are checked against stock conservatively too: their overlap cannot be
assumed to be permitted cutting contact. Controller wear compensation has no
declared physical displacement bound here and makes the check incomplete;
use computer compensation for a proven nominal path.

## What is checked

The model is copied into the post definition and its identity, saved with the
owned journal and checked again on reopening. The checker reads retained
**actual decoded controls and motions**, not source events or display meshes.
It owns a fresh native stock replay; wire deltas cannot grant geometric proof.

The initial pose, every line/arc/helix, rapid envelope, tool change and declared
end route are checked against machine-axis travel, all body pairs and evolving
stock. Cutting uses the existing milling or turning stock kernel; non-cutting
machine bodies must clear stock. Report findings name both bodies and the
motion ID/event index or controller route. For independent-axis envelopes, the
fraction is an enclosure parameter, not a reconstructed controller time. A
later incomplete phase preserves
any definite collision already found.

**Axis travel** is decided by [the travel rule](python.md#the-travel-rule),
the same one the planned-move check and the posted-program check use: the
travel is closed (a position on a limit is inside) and a decoded position is
exact. `options.tolerance_mm` plays no part in it. A decoded move's axis
position is known to the declared calibration interval (`work_error_*_mm` and
the tool's gauge-offset bounds), so it is an interval on each axis: `clear`
only if the whole interval is inside, a `violation` only if the whole interval
is beyond a limit, `uncertain` while it straddles one; the finding states the
interval and the limit. With no calibration interval a position is inside or
a violation, never uncertain. The initial position and route waypoints are
declared machine-axis positions: no interval, compared exactly. A decoded arc
whose words leave an endpoint gap is known to that gap either side of its
nominal circle.

Continuous separation uses convex face normals and edge cross products.
Linear relative translation is clipped over the whole time interval; curves
use interval displacement bounds and bounded subdivision. Point evaluations
can establish collision witnesses, never whole-path clearance. Tool prisms
circumscribe the native rotational sections; curved profiles and turning body
boxes may enclose extra space. Lathe machine-body/stock checks enclose each
radial stock bin with a box, including its bore and any removed ring: these
conservative bounds can refuse a physically clear case. They cannot prove a
physical collision merely because their outer bounds overlap. Milling stock
lower bounds can establish definite machine-body penetration. Unresolved
contact, tangency and budget exhaustion are uncertain and refused by default.

Stock simulation reuses the existing native stock and turning kernels.

## Limits and external premises

- The actual machine, work offset, H/D tables, tool lengths, fixture positions,
  clamping and controller options must match the declaration. Camlib cannot
  measure setup mistakes. Decoding checks declared register identities.
- This is the nominal, exact-stop geometric path. Servo following error,
  controller blending, flex, breakage, unmodelled attachments and non-monotone
  overshoot are not inferred; real-machine post qualification remains #20.
- Explicit G28/G30/G53/homing blocks are unsupported and refused by the
  independent decoder; they cannot bypass the check. Internal M6/end motion
  must be fully represented by the declared absolute routes and envelopes.
- Rotary/tilting machine kinematics are #2/#5. Indexing is covered only by the
  complete declared swept envelope above.
- Height-field stock cannot represent material under an overhang (#1). Model
  such material as protected convex solids or use a different stock engine;
  a height-field result is no proof beneath an omitted overhang.
- Camlib does not establish that input solids cover the real machine. A clear
  result applies to the declared geometry, calibration intervals, controller
  semantics and native stock model, not arbitrary imported NC or actual setup.

Sources: Eberly, [Method of Separating Axes](https://www.geometrictools.com/Documentation/MethodOfSeparatingAxes.pdf),
sections 4 and 6 (convex solids and continuous translation);
[LinuxCNC G-code](https://www.linuxcnc.org/docs/stable/html/gcode/g-code.html)
(G0 coordinated motion, G28 saved positions, G53 machine coordinates);
[Haas G00](https://www.haascnc.com/service/codes-settings.type%3Dgcode.machine%3Dmill.value%3DG00.html)
(independent-axis positioning). Controller behavior must be established for
the actual declared controller and its settings.
