# Indexable face and high-feed mills

`FaceMill` and `HighFeedMill` describe an assembled cutter: mounted cutting
profiles, a separate noncutting body, insert counts and operating limits. The
family name does not select cutting data or substitute a flat end mill.

All profile coordinates are millimetres: radius from the spindle axis and
height above the lowest cutting tip toward the spindle. Each
`MountedInsertGroup` contains the rotational cutting profile of identically
mounted inserts. Different axial settings (including wipers) use separate
groups. The model is the rotational envelope; it does not simulate individual
tooth marks, runout, vibration or deflection.

Two analytic meridian regions are supported:

- `AnnularFrustum(z_mm, inner_radius_mm, outer_radius_mm)`: inner and outer
  radii vary linearly between the two declared heights. A 45-degree shoulder
  has equal changes in outer radius and height. Equal inner and outer radius
  at one end gives a triangular section.
- `CircularRing(center_rz_mm, radius_mm, z_mm)`: a disk in the radius/height
  plane, clipped to the declared height interval, then revolved about the
  spindle. Its equation is `(r-cr)^2 + (z-cz)^2 <= radius^2`.

`RotationalProfile` is the union of its regions. It preserves central holes,
separate rings and axial gaps. These must be **mounted meridian profiles**.
Rotating an unmounted lathe insert outline or using its nominal corner radius
does not in general describe a tilted milling insert's assembled envelope.
Dimensions from an assembly drawing or a separately established assembled
profile can be supplied directly; a CAD model is not required.

The noncutting profile includes the body, insert backs and fasteners relevant
to clearance. Currently declare this as filled cylindrical/conical frusta
(zero inner radii). Shank and holder envelopes are added by `ToolAssembly`.
The body is checked against stock as noncutting material even when optional
run verification is absent. Part and fixture checks include it as a holder.
As with other tools, the caller must declare the complete assembly.

## Python declaration

This is a synthetic geometric example, **not cutting data for a real tool**:

```python
from camlib import (
    AnnularFrustum, FaceMill, IndexableGeometry, MountedInsertGroup,
    RotationalProfile, ToolAssembly, HolderSection,
)

mill = FaceMill(
    "face12",
    geometry=IndexableGeometry(
        inserts=(MountedInsertGroup(
            "primary", 3,
            RotationalProfile((AnnularFrustum((0, 2), (2, 2), (4, 6)),)),
        ),),
        noncutting=RotationalProfile((
            AnnularFrustum((2.5, 10), (0, 0), (5, 5)),
        )),
    ),
    diameter=12, cutting_height=2, overall_length=30, shank_diameter=6,
    max_depth_of_cut_mm=1, max_ramp_angle_deg=3,
    helical_hole_diameter_mm=(14, 18), material="synthetic",
)
assembly = ToolAssembly("mill", mill, 15, 40, (HolderSection(15, 40, 6, 6),))
```

For `HighFeedMill`, supply the actual low-angle/curved mounted profile using
the same geometry API. A `CircularRing((4, 1), 1, (0, 2))`, for example, is a
synthetic round meridian with a one-millimetre radius; it is not a model of an
arbitrary commercial high-feed insert.

`diameter` encloses the largest cutting diameter; `cutting_height` encloses the
cutting profile. Neither substitutes for `max_depth_of_cut_mm`. The latter is
the permitted axial engagement of this body/insert combination. Every layer
must respect it, even when the operation's total depth is greater than the
cutting height. Feeds and spindle speed remain explicit operation data and
are never increased automatically for axial chip thinning.

Nominal stepdown alone does not prove compliance: a previous pass's curved
shoulder can engage a higher portion of the insert on the next pass. The
runtime checks that contact against the current stock bounds. Unresolved
contact above the limit is refused, with its location and stock/limit heights.
The source profiles and limits must describe the same assembled combination.

`helical_hole_diameter_mm` is the inclusive **machined hole diameter** range,
not the tool-center orbit diameter. A shaped cutter can leave different hole
diameters at different heights. The engine checks an enclosing diameter range:
orbit diameter plus twice a lower bound on every occupied section's outer
radius, through orbit diameter plus the declared maximum cutting diameter.
For a frustum the lower bound is its smaller outer endpoint radius; for a
circular region it is its center radius. Unions use the smallest such bound.
This is conservative for clipped arcs and overlapping insert groups. Using
only the largest cutter diameter could falsely accept a too-small hole at
the tip. Set the range to `None` to prohibit helical cutting entry.
A zero maximum ramp angle prohibits cutting descent. Declaring helical limits
does not prove clearance: stock and the body are checked along the entry.

## Operations and checks

Facing uses the floor-cutting width for its row spacing and the full assembly
envelope for clearance. Pocket and raster/offset Roughing protect the target
using the maximum cutting envelope; their stock result retains the actual
shoulder and curved floor. Contour supports exterior air entry without tabs.
Supplied toolpaths undergo the same stock/body/depth/entry admission checks.

Straight plunges into material, excessive layer depths, steep entries and
helices outside the declared hole range are refused. Linear cutting ramps
are currently refused even within the angle limit: their reversal/exit
transition needs a model of the cutter's required lift. Use exterior air
entry or a permitted helix. Adaptive clearing, contour tabs/ramped contour
entries and other generators that have not been adapted explicitly refuse
these tools.

Clearance proofs may use an enclosing cylinder to prove separation. They do
not use that cylinder to erase stock. Body admission is conservative against
the stock before the motion; it does not assume an earlier portion of that
same motion cleared the body. If the enclosure cannot establish clearance,
the move is refused rather than approximating the cutter or removing stock
on the body's behalf.

Both height-field and tri-dexel stock sweeps retain annular occupancy. A
height field bounds a vertical column rather than representing every cavity;
tri-dexel retains finite intervals. The actual stock bounds feed the material
verifier, which reports the chamfered shoulder and material under a curved
insert. Nominal planar guide coverage is not a claim that a flat floor was
cut. Full-disk clearance certificates are not issued for these tools.

`Engine.inspect_tool(assembly, heights)` returns `annular_profile`, containing
separate `cutting` and `noncutting` interval sets at each height. Each set has
`certain` and `possible` radial intervals. The older single-radius `profile`
response cannot describe a ring and is not used for these cutters.

## Numerical contract

The profile routines intersect analytic half-planes and circles. Basic
binary64 arithmetic is rounded outward at each operation. A query returns
an inner set contained in the exact declared section and an outer set
containing it. Unresolved tangencies may have no inner interval; an empty
inner set is not evidence of clearance. Interval unions never bridge a gap
with a tolerance.

Sweeps use the engine's bounded chord approximation, widening possible
removal and contracting certain removal by path error and cell extent. For
a chord, distance to its path is continuous; every cell point reaches the
common interval between the bounded nearest and farthest distances. This
gives a certain axial sweep without pretending the cutter fills its center.
Cross-axis dexel rays use the outer swept disk minus the intersection of
the two endpoint inner disks, with opposite bounds for the subtracted hole.
Independent tests compare resulting stock bounds with closed-form shoulder
and circular-floor equations. These bounds concern the declared geometry;
manufacturing uncertainty must be included in that declaration.

## Sources and programming radii

[Sandvik's entering-angle guide](https://www.sandvik.coromant.com/en-gb/knowledge/milling/entering-angle-and-chip-thickness)
distinguishes straight-edge and round-insert chip thickness; a round insert's
effective angle depends on depth. Its
[face-milling guide](https://www.sandvik.coromant.com/en-gb/knowledge/milling/face-milling)
also describes wipers and their axial setting. Its
[ramping guide](https://www.sandvik.coromant.com/en-gb/knowledge/milling/milling-holes-cavities-pockets/ramping)
explains the lift needed before reversing a maximum-angle linear ramp.

The [Tungaloy DoTwistBall catalogue](https://tungaloy.com/wpdata/wp-content/uploads/507.pdf)
lists different depth, ramp and machined-hole limits for different inserts
on a body. Limits therefore belong to the assembled combination, not just
the cutter family or nominal diameter. Its
[programming-radius diagram](https://tungaloy.com/wpdata/wp-content/uploads/Milling-highfeed-DoTwistBall-img-12.pdf)
shows both uncut and overcut material relative to an RPG approximation.
An RPG value is not the actual mounted radius. camlib does not accept an RPG
as geometric evidence or turn scalar catalogue deviations into an assumed
universal three-dimensional bound. Supply the actual supported profile.
