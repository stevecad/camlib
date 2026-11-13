"""Turning (lathe) jobs: axisymmetric stock, turning tools and operations.

Coordinates are the lathe's: x is the radius from the spindle axis, z runs
along it. The part is given as its half-section ``profile``: points
``(radius, z, bulge)`` from the front face (on the axis) toward the chuck,
the bulge being tan(sweep / 4) of the arc to the next point (positive: G3).

A lathe setup is a ``camlib.jobs.Setup`` with ``TurnStockSpec`` stock and a
``TurningSetup``. Its run plans each operation natively when it is reached
and checks every move: the swept insert against the part (eroded by the
tolerance), the holder against the part, its bore and the stock, rapids
against the stock, nothing
past the chuck face. The run streams every move with the exact stock change
it makes, like a milling run: playback, posting and export work the same.
"""

from dataclasses import dataclass, field, replace
import json

from ._buffers import native


@dataclass(frozen=True)
class TurnStockSpec:
    z_min_mm: float
    z_max_mm: float
    outer_radius_mm: float
    bore_radius_mm: float = 0
    cell_mm: float = 0.25
    max_cells: int = 1_000_000

    def to_dict(self):
        return dict(
            version=1,
            units="mm",
            z_min_mm=self.z_min_mm,
            z_max_mm=self.z_max_mm,
            outer_radius_mm=self.outer_radius_mm,
            bore_radius_mm=self.bore_radius_mm,
            cell_mm=self.cell_mm,
            max_cells=self.max_cells,
        )


# Turning tools. Every geometric value is stated: nothing defaults to a tool
# that isn't the caller's (AGENTS.md: no assumed tools). Outlines are in the
# tool's XZ frame: x away from the spindle axis for an outside tool, z along
# the axis, drawn as an outside, right-hand tool cutting toward -z (a boring
# tool is the same drawing, mirrored by camlib; a left-hand tool too).


def _pair(value, what):
    pair = tuple(float(v) for v in value)
    if len(pair) != 2:
        raise ValueError(f"{what} must be an (x, z) pair in mm")
    return pair


def _size_tolerance(value, what):
    pair = tuple(float(v) for v in value)
    if len(pair) != 2:
        raise ValueError(
            f"{what} must be (d, m): the ± tolerances in mm on the insert's inscribed circle and on its corner "
            "dimension m (ISO 1832)"
        )
    return pair


@dataclass(frozen=True)
class Line:
    """A straight segment to ``to_mm`` (x, z)."""

    to_mm: tuple

    def to_dict(self):
        return dict(kind="line", to_mm=list(_pair(self.to_mm, "Line.to_mm")))


@dataclass(frozen=True)
class Arc:
    """A circular arc around ``center_mm`` of ``radius_mm`` (0: a sharp
    corner) from ``start_deg`` to ``end_deg`` (degrees from +x toward +z;
    increasing is counterclockwise), at most a full turn. It starts where
    the previous segment ends."""

    center_mm: tuple
    radius_mm: float
    start_deg: float
    end_deg: float

    def to_dict(self):
        return dict(
            kind="arc",
            center_mm=list(_pair(self.center_mm, "Arc.center_mm")),
            radius_mm=self.radius_mm,
            start_deg=self.start_deg,
            end_deg=self.end_deg,
        )


def _segment(data):
    data = dict(data)
    kind = data.pop("kind")
    if kind == "line":
        return Line(tuple(data["to_mm"]))
    return Arc(tuple(data["center_mm"]), data["radius_mm"], data["start_deg"], data["end_deg"])


@dataclass(frozen=True)
class Outline:
    """A closed outline: ``start_mm`` (x, z), then ``segments`` (``Line``,
    ``Arc``), the last ending at the start. It must not cross itself."""

    start_mm: tuple
    segments: tuple

    def to_dict(self):
        return dict(
            start_mm=list(_pair(self.start_mm, "Outline.start_mm")), segments=[s.to_dict() for s in self.segments]
        )

    @classmethod
    def from_dict(cls, data):
        return cls(tuple(data["start_mm"]), tuple(_segment(s) for s in data["segments"]))


@dataclass(frozen=True)
class InsertOutline:
    """A turning insert of any shape (round, trigon, a real insert's back
    edge or wiper): its ``outline``; ``corner``, the index of the arc
    segment that is the cutting corner (its center is the nose center, its
    radius the nose radius); ``cutting``, the indices of the segments that
    may cut (the corner and the edges next to it; every other segment must
    stay clear of the stock); and ``programming_point_mm``, the point the
    tool's offsets are touched off with (the imaginary tip), all in the
    tool's XZ frame. ``TurningInsert.iso`` and ``TurningInsert.iso_code``
    build one from ISO values."""

    outline: Outline
    corner: int
    cutting: tuple
    programming_point_mm: tuple
    # What the ISO constructors know of the body (#103): the major edge's
    # direction (from the holder's approach angle), and from an ISO 1832
    # code the worst-case thickness and the clearance. Not serialized: they
    # feed the TurningTool's body.
    edge_deg: float | None = None
    thickness_mm: float | None = None
    clearance_deg: float | None = None
    # From an ISO 1832 code: the tolerance (d, m) in mm on the inscribed
    # circle and the corner dimension, of the code's tolerance class (#113).
    size_tolerance_mm: tuple | None = None

    @property
    def nose_radius_mm(self):
        segment = self.outline.segments[self.corner]
        if not isinstance(segment, Arc):
            raise ValueError("InsertOutline.corner must be the index of an Arc segment (the nose)")
        return segment.radius_mm

    def to_dict(self):
        return dict(
            kind="outline",
            outline=self.outline.to_dict(),
            corner=self.corner,
            cutting=list(self.cutting),
            programming_point_mm=list(_pair(self.programming_point_mm, "programming_point_mm")),
        )

    @classmethod
    def from_dict(cls, data):
        return cls(
            Outline.from_dict(data["outline"]),
            data["corner"],
            tuple(data["cutting"]),
            tuple(data["programming_point_mm"]),
        )


class TurningInsert:
    """Constructors of turning inserts (``InsertOutline``) from ISO values.

    ``TurningInsert.iso(...)``: the wedge camlib has always modelled (nose,
    two edges, straight back) from its angles and edge length.
    ``TurningInsert.iso_code(code, ...)``: the whole insert from its ISO 1832
    designation. There are no defaults: every value is the caller's."""

    def __init__(self, *args, **kwargs):
        raise TypeError(
            "TurningInsert(...) no longer has default angles or edge length: use "
            "TurningInsert.iso(included_angle_deg=, approach_angle_deg=, edge_length_mm=, nose_radius_mm=), "
            "TurningInsert.iso_code(code, approach_angle_deg= or holder=), or an InsertOutline"
        )

    @staticmethod
    def iso(*, included_angle_deg, approach_angle_deg, edge_length_mm, nose_radius_mm):
        """A nose of ``nose_radius_mm`` between two edges of
        ``edge_length_mm`` at ``included_angle_deg`` (80 C, 55 D, 35 V, 90 S,
        60 T), on a holder with ``approach_angle_deg`` (the cutting edge
        angle, e.g. 93 or 95), closed straight across the back. The
        programming point is the imaginary tip."""
        data = native.turning_iso_insert(
            float(included_angle_deg), float(approach_angle_deg), float(edge_length_mm), float(nose_radius_mm)
        )
        insert = InsertOutline.from_dict(json.loads(data))
        return replace(insert, edge_deg=float(approach_angle_deg) - 90)

    @staticmethod
    def iso_code(code, *, approach_angle_deg=None, holder=None, size_tolerance_mm=None):
        """The whole insert from its ISO 1832 designation (e.g.
        ``"CNMG120408"``): shape C, D, V, S, T, W (negative) or R, size and
        nose radius; every corner rounded, the cutting corner at the nose.
        The approach angle comes from the holder: ``approach_angle_deg``, or
        ``holder`` as an ISO 5608 designation (e.g. ``"PCLNR2525M12"``, style
        L: 95 degrees). Values: ISO 1832:1991 and ISO 5608:1995 tables, and
        the size and corner radius tables of Sumitomo's code key (see
        docs/operations.md).

        The outline is the insert at its nominal size. Its tolerance class
        (the code's third letter) gives the tolerance on its inscribed
        circle d and its corner dimension m (ISO 1832:2017 tables 3 to 5):
        a ``TurningTool`` with this insert is checked as the largest insert
        that tolerance allows, relative to its measured nose (the nose and
        the two edges at it stay where they are; the far sides and corners
        move out). Where the standard gives no value for the code's class,
        shape and size, the code is refused: give
        ``size_tolerance_mm=(d, m)``, the ± tolerances from the insert
        maker's data (also to use a maker's closer tolerances). A round
        insert (R) has none: its d is its nose diameter."""
        if (approach_angle_deg is None) == (holder is None):
            raise ValueError(
                "TurningInsert.iso_code needs the holder's approach angle: approach_angle_deg= or holder= (an ISO 5608 code)"
            )
        if holder is not None:
            approach_angle_deg = native.turning_holder_approach(str(holder))
        data, _ = native.turning_iso_code(str(code), float(approach_angle_deg))
        thickness, clearance = native.turning_iso_insert_body(str(code))
        if size_tolerance_mm is None:
            size_tolerance_mm = native.turning_iso_size_tolerance(str(code))
        else:
            size_tolerance_mm = _size_tolerance(size_tolerance_mm, "size_tolerance_mm")
        return replace(
            InsertOutline.from_dict(json.loads(data)),
            edge_deg=float(approach_angle_deg) - 90,
            thickness_mm=thickness,
            clearance_deg=clearance,
            size_tolerance_mm=size_tolerance_mm,
        )


@dataclass(frozen=True)
class GrooveInsert:
    """A grooving or parting insert ``width_mm`` wide, ``depth_mm`` deep."""

    width_mm: float
    depth_mm: float

    def to_dict(self):
        return dict(kind="groove", width_mm=self.width_mm, depth_mm=self.depth_mm)


@dataclass(frozen=True)
class FaceGrooveInsert:
    """A face grooving insert: a blade pointing along -z into the front face,
    ``width_mm`` across in r, reaching ``depth_mm``; the nose radius at both
    corners. Its reference is the inner corner (toward the axis)."""

    width_mm: float
    depth_mm: float

    def to_dict(self):
        return dict(kind="face_groove", width_mm=self.width_mm, depth_mm=self.depth_mm)


@dataclass(frozen=True)
class ThreadInsert:
    """A threading insert: ``profile_angle_deg`` (60 metric and UN, 55
    Whitworth), reaching ``depth_mm`` beyond its nose."""

    profile_angle_deg: float
    depth_mm: float

    def to_dict(self):
        return dict(kind="thread", profile_angle_deg=self.profile_angle_deg, depth_mm=self.depth_mm)


@dataclass(frozen=True)
class DrillInsert:
    """A drill on the spindle axis: diameter, point angle (118, 140; 180
    flat) and flute length. Its reference point is the tip."""

    diameter_mm: float
    point_angle_deg: float
    flute_length_mm: float

    def to_dict(self):
        return dict(
            kind="drill",
            diameter_mm=self.diameter_mm,
            point_angle_deg=self.point_angle_deg,
            flute_length_mm=self.flute_length_mm,
        )


@dataclass(frozen=True)
class Bar:
    """A round boring bar: ``diameter_mm``, its axis at ``axis_x_mm`` in the
    tool frame (parallel to z) and ``axis_y_mm`` across the centre plane
    (the cutting face at 0, the insert below it), from ``z_mm[0]`` to
    ``z_mm[1]``. Checked as the round body it is, off the centre plane too."""

    diameter_mm: float
    axis_x_mm: float
    z_mm: tuple
    axis_y_mm: float

    def to_dict(self):
        return dict(diameter_mm=self.diameter_mm, axis_x_mm=self.axis_x_mm, z_mm=list(_pair(self.z_mm, "Bar.z_mm")))


@dataclass(frozen=True)
class Holder:
    """The holder (or boring bar) as an ``Outline`` in its insert's tool
    frame, anywhere relative to the insert (a head reaching ahead of it, a
    stepped or angled head), and a round ``bar`` for a boring bar. It is
    checked like the insert: against the part, its bore, the chuck and the
    stock. ``Holder.rectangle`` builds camlib's rectangular holder."""

    outline: Outline
    bar: Bar | None = None
    # The outline's extent across the centre plane, (low, high) mm: the
    # cutting face at 0, the insert body toward negative (#103). Required.
    y_mm: tuple | None = None
    # The body away from the cut from a supplied model (#110): checked in
    # place of the outline's body (``Holder.from_model``).
    model: "ToolModel | None" = None

    def to_dict(self):
        data = dict(outline=self.outline.to_dict())
        if self.bar is not None:
            data["bar"] = self.bar.to_dict()
        return data

    @classmethod
    def from_model(cls, model):
        """The holder as its maker's model (a ``ToolModel``: a mesh with its
        error, its declared placement and its insert cross-check): every
        check uses the model's body in place of an outline's. The
        ``outline`` and ``y_mm`` kept here are only the model's enclosing
        rectangle about the nose centre in the lathe's X and Z and its
        extent in Y (the tool format requires them); no check reads them."""
        if not isinstance(model, ToolModel):
            raise TypeError("Holder.from_model takes a ToolModel (camlib.TurningToolModel)")
        (x0, x1), (y0, y1), (z0, z1) = model.bounds_mm()
        outline = Outline((x0, z0), (Line((x1, z0)), Line((x1, z1)), Line((x0, z1)), Line((x0, z0))))
        return cls(outline, y_mm=(y0, y1), model=model)

    @classmethod
    def rectangle(cls, *, insert, nose_radius_mm, side, width_mm, length_mm, offset_mm, y_mm):
        """camlib's rectangular holder for ``insert``: outside, a shank
        ``width_mm`` along z and ``length_mm`` away from the cut, starting
        ``offset_mm`` behind the insert's reach (a turning insert's nose
        center toward +z; across a grooving or threading blade); boring
        (``side="inner"``), an axial bar ``width_mm`` across, its top
        ``offset_mm`` under the nose center, ``length_mm`` out of the bore;
        a drill's chuck ``width_mm`` across, ``offset_mm`` behind the
        flutes, ``length_mm`` long. ``y_mm``: its extent across the centre
        plane, (low, high) mm (an outside shank from its height below the
        face up to its clamp; a boring bar's section)."""
        data = native.turning_rectangle_holder(
            json.dumps(insert.to_dict()),
            float(nose_radius_mm),
            str(side),
            float(width_mm),
            float(length_mm),
            float(offset_mm),
        )
        data = json.loads(data)
        return cls(Outline.from_dict(data["outline"]), y_mm=tuple(float(v) for v in _pair(y_mm, "y_mm")))


# Tool bodies from supplied models (#110).


def _triple(value, what):
    triple = tuple(float(v) for v in value)
    if len(triple) != 3:
        raise ValueError(f"{what} must be three coordinates (x, y, z)")
    return triple


@dataclass(frozen=True, kw_only=True)
class ModelPlacement:
    """Where a tool model sits on the lathe, as the tool is mounted (camlib
    assumes nothing and fits nothing): ``nose_center_mm``, the model point
    that is the insert's nose centre on its cutting face (the theoretical
    tip a maker's frame may give is one nose radius from it along X and
    along Z; a drill: its tip on the axis); ``x_axis`` and ``z_axis``, the
    model directions of the lathe's +X (radial, away from the spindle axis
    on the tool's side) and +Z, unit and perpendicular. Y completes the
    right-handed frame (Y = Z x X): the cutting face toward +Y."""

    nose_center_mm: tuple
    x_axis: tuple
    z_axis: tuple

    def to_dict(self):
        return dict(
            nose_center_mm=list(_triple(self.nose_center_mm, "ModelPlacement.nose_center_mm")),
            x_axis=list(_triple(self.x_axis, "ModelPlacement.x_axis")),
            z_axis=list(_triple(self.z_axis, "ModelPlacement.z_axis")),
        )

    def place(self, point):
        """A model point in the lathe's (X, Y, Z) from the nose centre."""
        o, x, z = (_triple(v, "ModelPlacement") for v in (self.nose_center_mm, self.x_axis, self.z_axis))
        y = (z[1] * x[2] - z[2] * x[1], z[2] * x[0] - z[0] * x[2], z[0] * x[1] - z[1] * x[0])
        d = tuple(float(point[k]) - o[k] for k in range(3))
        return tuple(sum(d[k] * axis[k] for k in range(3)) for axis in (x, y, z))


@dataclass(frozen=True, kw_only=True)
class ModelInsertCheck:
    """The model's insert (``insert``: a ``SurfaceMesh`` in the model's
    coordinates, placed like its body) and how closely its cutting corner
    must agree with the declared insert, compared within
    ``corner_region_mm`` of the nose centre along X and Z (at least two nose
    radii; inside the holder's reach, so only the insert is in it):
    ``position_tolerance_mm`` (the corner, along the cutting face and in
    height), ``nose_radius_tolerance_mm`` and ``edge_angle_tolerance_deg``
    (each straight edge at the corner). They must cover the insert mesh's
    own error. A model outside one is refused before any move, as misplaced
    or as a different insert; it is never fitted. No value has a default."""

    insert: object
    corner_region_mm: float
    position_tolerance_mm: float
    nose_radius_tolerance_mm: float
    edge_angle_tolerance_deg: float

    def to_dict(self):
        return dict(
            kind="model",
            insert=self.insert.to_dict(),
            corner_region_mm=float(self.corner_region_mm),
            position_tolerance_mm=float(self.position_tolerance_mm),
            nose_radius_tolerance_mm=float(self.nose_radius_tolerance_mm),
            edge_angle_tolerance_deg=float(self.edge_angle_tolerance_deg),
        )


@dataclass(frozen=True, kw_only=True)
class ModelWithoutInsert:
    """A model that has no insert: its placement cannot be compared with
    the declared insert and rests on ``reason``, the caller's statement of
    where the placement comes from (e.g. the maker's cutting-point frame)."""

    reason: str

    def to_dict(self):
        return dict(kind="not_modelled", reason=str(self.reason))


@dataclass(frozen=True, kw_only=True)
class ToolModel:
    """A turning tool's body from its maker's model (#110).

    ``body``: a ``SurfaceMesh`` of the tool away from the cut (holder, bar,
    shim, clamp, coolant block), without the insert, in the model's own
    coordinates: a closed surface, every point of the real one within its
    ``surface_error_mm``, which is grown onto the body.
    ``camlib.cadquery.tool_body_from_step`` builds one from a STEP file.
    ``placement``: a ``ModelPlacement``. ``axial_resolution_mm``: the width
    of the slices across Z the body is checked in (it is over-estimated
    along Z by less than one slice). ``insert_check``: a
    ``ModelInsertCheck`` (the model's insert, cross-checked against the
    declared one) or a ``ModelWithoutInsert``.

    The insert stays the declared one (ISO code or outline, thickness,
    clearance, seat): it is what cuts. Use it as a tool's holder with
    ``Holder.from_model(model)``."""

    body: object
    placement: ModelPlacement
    axial_resolution_mm: float
    insert_check: object

    def __post_init__(self):
        if not isinstance(self.placement, ModelPlacement):
            raise TypeError("ToolModel.placement must be a ModelPlacement: camlib assumes no placement")
        if not isinstance(self.insert_check, (ModelInsertCheck, ModelWithoutInsert)):
            raise TypeError(
                "ToolModel.insert_check must be a ModelInsertCheck (the model's insert and its tolerances) "
                "or a ModelWithoutInsert (the reason the placement is right)"
            )

    def to_dict(self):
        return dict(
            body=self.body.to_dict(),
            placement=self.placement.to_dict(),
            axial_resolution_mm=float(self.axial_resolution_mm),
            insert_check=self.insert_check.to_dict(),
        )

    def bounds_mm(self):
        """The placed body's extent along the lathe's X, Y and Z from the
        nose centre (its mesh error included), as ((x0, x1), (y0, y1),
        (z0, z1))."""
        low, high = [float("inf")] * 3, [float("-inf")] * 3
        for triangle in self.body.triangles:
            for point in triangle:
                placed = self.placement.place(point)
                for k in range(3):
                    low[k] = min(low[k], placed[k])
                    high[k] = max(high[k], placed[k])
        error = float(self.body.surface_error_mm)
        return tuple((low[k] - error, high[k] + error) for k in range(3))


# Top-level names (camlib.TurningHolder, camlib.TurningBar; camlib.BoringBar
# is the milling boring bar).
TurningHolder = Holder
TurningBar = Bar
TurningToolModel = ToolModel
TurningModelPlacement = ModelPlacement
TurningModelInsertCheck = ModelInsertCheck
TurningModelWithoutInsert = ModelWithoutInsert

_REMOVED = ("holder_width_mm", "holder_length_mm", "holder_offset_mm")


@dataclass(frozen=True)
class Seat:
    """How the holder's seat tilts the insert: its orthogonal rake and its
    inclination (degrees, ISO 3002-1), about the major cutting edge. They
    are the holder maker's data: ISO 5608 designations do not carry them,
    and ISO 5610 gives only recommended values. ``Seat(0, 0)``: a flat seat.
    Negative rake tilts the face up away from the edge (the flank below the
    edge backs off); negative inclination makes the nose the lowest point of
    the major edge (the flank below the nose backs off), positive the
    highest. These are the signs of holder catalogues: a negative P-clamp
    holder is ``Seat(-6, -6)``."""

    rake_deg: float
    inclination_deg: float


TurningSeat = Seat


@dataclass(frozen=True)
class SolidBarHead:
    """The ground head of a solid (one-piece) boring bar (#114): a solid
    carbide boring bar, a brazed tool or a ground HSS bar. Such a tool has no
    insert and no seat: its tip is ground into the end of a round bar, so
    its body below the cutting face is the bar's cylinder cut by the ground
    faces, which narrows toward the bottom. The tool's ``insert`` is an
    ``InsertOutline``, the head's (convex) outline at its cutting face.

    Every value is required, from the tool's catalogue or drawing:

    - ``diameter_mm``: the diameter of the round the head is ground from:
      the bar's, where the head keeps the bar's surface (the catalogue's
      shank diameter), or of any cylinder along the bar that contains the
      head below its cutting face.
    - ``axis_x_mm``, ``axis_y_mm``: that cylinder's axis in the outline's
      frame: its x (the catalogue's f or WF is the distance from the bar's
      centreline to the cutting edge) and its offset across the centre plane
      (the cutting face at 0; minus the catalogue's cutting edge height
      above the bar's centre).
    - ``clearance_deg``: one angle per segment of the outline, at which the
      ground face below that side recedes under the cutting face: the end
      clearance below the end cutting edge, the side clearance below the
      side cutting edge, at most the lesser of the two below the nose
      between them, 0 where the head is not relieved (the cylinder alone
      bounds it there). Catalogue tables do not list them: an angle not
      known is declared 0, which contains the real head.
    - ``min_bore_diameter_mm``: the catalogue's minimum bore diameter
      (Dmin). No cut in a smaller bore, and a head that cannot itself enter
      it is refused as a contradiction.
    - ``max_bore_depth_mm``: the catalogue's maximum bore depth (the usable
      length), from the head's front end. No move deeper into the stock.

    The geometry is checked whatever the catalogue values say."""

    diameter_mm: float
    axis_x_mm: float
    axis_y_mm: float
    clearance_deg: tuple
    min_bore_diameter_mm: float
    max_bore_depth_mm: float

    def to_dict(self):
        return dict(
            diameter_mm=float(self.diameter_mm),
            axis_x_mm=float(self.axis_x_mm),
            axis_y_mm=float(self.axis_y_mm),
            clearance_deg=[float(a) for a in self.clearance_deg],
            min_bore_diameter_mm=float(self.min_bore_diameter_mm),
            max_bore_depth_mm=float(self.max_bore_depth_mm),
        )


class TurningTool:
    """A turning tool: ``id``, ``insert`` (an ``InsertOutline``, e.g. from
    ``TurningInsert.iso``/``iso_code``, or a ``GrooveInsert``,
    ``FaceGrooveInsert``, ``ThreadInsert`` or ``DrillInsert``), ``side``
    ("outer", or "inner" for a boring tool cutting outward), ``hand``
    ("right": cuts toward the chuck, or "left"), and ``holder`` (a
    ``Holder``, e.g. ``Holder.rectangle``). ``nose_radius_mm`` is the
    insert's: an ``InsertOutline`` states it (its corner's radius); the
    other inserts need it. Every value is required: a missing one is
    refused with its name.

    The body off the centre plane (#103) is required too, so every check
    sees the real tool, not its flat outline:

    - ``seat``: the holder's ``Seat`` (rake, inclination); not for a drill.
    - ``insert_thickness_mm``, ``insert_clearance_deg``: the insert's
      thickness below its cutting face and its normal clearance. From
      ``TurningInsert.iso_code`` they are the code's worst case (the largest
      thickness its symbol and tolerance class allow); give them to use the
      insert's real values. Not for a drill.
    - ``insert_front_clearance_deg``, ``insert_side_clearance_deg``: a
      grooving blade's (``GrooveInsert``, ``FaceGrooveInsert``) own
      clearance below its front edge and below its sides, from its maker's
      catalogue (they differ: e.g. 10 degrees at the front, 3 at the
      sides). The side value is one angle for both sides or a pair: for a
      ``GrooveInsert`` (the side at its leading corner, the other side),
      for a ``FaceGrooveInsert`` (the inner side, toward the axis; the
      outer side). Given together, instead of ``insert_clearance_deg``
      (which is then their least and applies to the corners between them);
      an ``insert_clearance_deg`` that is not their least is refused. Other
      inserts have one normal clearance (ISO 1832) and take only
      ``insert_clearance_deg``.
    - ``insert_size_tolerance_mm``: the insert's in-plane size tolerance,
      (d, m): ± mm on its inscribed circle and on its corner dimension m
      (ISO 1832). From ``TurningInsert.iso_code`` it is the code's
      tolerance class; give it to use the insert maker's values, or
      ``(0, 0)`` for an insert measured to be exactly its outline. With
      it, every check uses the largest insert the tolerance allows relative
      to its measured nose: the nose and the two cutting edges at it stay
      where they are (the tool is measured there), the far sides and
      corners move out. Only for an ``InsertOutline`` of the ISO form
      (straight sides tangent to one inscribed circle, rounded corners);
      any other outline is taken as given: state the largest outline the
      insert can have.
    - ``edge_deg``: the major cutting edge's direction in the tool frame,
      for an ``InsertOutline`` not built by ``TurningInsert`` (grooving,
      threading and face grooving inserts know theirs).
    - ``curved_for_diameter_mm``: a face grooving blade curved to follow
      the groove, its maker's (min, max) diameter range; without it the
      blade is checked as straight.
    - the holder's ``y_mm`` (and a ``Bar``'s ``axis_y_mm``), or the holder
      as its maker's model: ``Holder.from_model(ToolModel(...))`` (#110).

    A solid (one-piece) boring bar (#114) has no insert and no seat: give
    ``solid_bar=SolidBarHead(...)`` instead of the seat and insert values,
    with ``insert`` the head's outline at its cutting face and
    ``side="inner"``."""

    __slots__ = (
        "id",
        "insert",
        "nose_radius_mm",
        "side",
        "hand",
        "holder",
        "seat",
        "insert_thickness_mm",
        "insert_clearance_deg",
        "edge_deg",
        "curved_for_diameter_mm",
        "insert_front_clearance_deg",
        "insert_side_clearance_deg",
        "solid_bar",
        "insert_size_tolerance_mm",
    )

    def __init__(
        self,
        id,
        *,
        insert=None,
        side=None,
        hand=None,
        holder=None,
        nose_radius_mm=None,
        seat=None,
        insert_thickness_mm=None,
        insert_clearance_deg=None,
        edge_deg=None,
        curved_for_diameter_mm=None,
        insert_front_clearance_deg=None,
        insert_side_clearance_deg=None,
        solid_bar=None,
        insert_size_tolerance_mm=None,
        **removed,
    ):
        if removed:
            names = ", ".join(sorted(removed))
            hint = (
                "; the holder is now a Holder (Holder.rectangle(insert=, nose_radius_mm=, side=, width_mm=, length_mm=, offset_mm=) gives camlib's rectangle)"
                if set(removed) & set(_REMOVED)
                else ""
            )
            raise TypeError(f"TurningTool {id!r}: unknown value(s) {names}{hint}")
        missing = [
            name
            for name, value in (("insert", insert), ("side", side), ("hand", hand), ("holder", holder))
            if value is None
        ]
        if isinstance(insert, InsertOutline):
            if nose_radius_mm is None:
                nose_radius_mm = insert.nose_radius_mm
        elif nose_radius_mm is None:
            missing.append("nose_radius_mm")
        drill = isinstance(insert, DrillInsert)
        if solid_bar is not None:
            # A solid boring bar's ground head (#114): the whole body below
            # the cutting face, with no insert and no seat.
            if not isinstance(solid_bar, SolidBarHead) or not isinstance(insert, InsertOutline):
                raise TypeError(
                    f"TurningTool {id!r}: solid_bar is a SolidBarHead, for a boring head drawn as an InsertOutline "
                    "(the head's outline at its cutting face); solid grooving and threading bars are not modelled "
                    "as solid heads (give such a blade as a GrooveInsert or ThreadInsert with its insert body)"
                )
            given = [
                name
                for name, value in (
                    ("seat", seat),
                    ("insert_thickness_mm", insert_thickness_mm),
                    ("insert_clearance_deg", insert_clearance_deg),
                    ("edge_deg", edge_deg),
                    ("curved_for_diameter_mm", curved_for_diameter_mm),
                    ("insert_front_clearance_deg", insert_front_clearance_deg),
                    ("insert_side_clearance_deg", insert_side_clearance_deg),
                )
                if value is not None
            ]
            if given:
                raise TypeError(
                    f"TurningTool {id!r}: a solid boring bar has no insert and no seat: its head (solid_bar) is its "
                    f"whole body below the cutting face; leave out {', '.join(given)}"
                )
        if insert_front_clearance_deg is not None or insert_side_clearance_deg is not None:
            # A grooving blade's own front and side clearance (#170).
            if not isinstance(insert, (GrooveInsert, FaceGrooveInsert)):
                raise TypeError(
                    f"TurningTool {id!r}: insert_front_clearance_deg and insert_side_clearance_deg are a grooving "
                    "blade's (GrooveInsert, FaceGrooveInsert) only: other inserts have one normal clearance "
                    "(ISO 1832: insert_clearance_deg alone), a drill none"
                )
            if insert_front_clearance_deg is None or insert_side_clearance_deg is None:
                given, other = (
                    ("insert_side_clearance_deg", "insert_front_clearance_deg")
                    if insert_front_clearance_deg is None
                    else ("insert_front_clearance_deg", "insert_side_clearance_deg")
                )
                raise TypeError(
                    f"TurningTool {id!r}: {given} needs {other} too (a blade's front and side clearance are "
                    "declared together; for one angle on every side give insert_clearance_deg alone)"
                )
            insert_front_clearance_deg = float(insert_front_clearance_deg)
            if isinstance(insert_side_clearance_deg, (int, float)):
                insert_side_clearance_deg = (float(insert_side_clearance_deg),) * 2
            else:
                insert_side_clearance_deg = tuple(float(v) for v in insert_side_clearance_deg)
                if len(insert_side_clearance_deg) != 2:
                    raise TypeError(
                        f"TurningTool {id!r}: insert_side_clearance_deg is one angle for both sides or a pair "
                        "(the side at the blade's reference corner, the other side)"
                    )
            least = min(insert_front_clearance_deg, *insert_side_clearance_deg)
            if insert_clearance_deg is None:
                insert_clearance_deg = least
            elif float(insert_clearance_deg) != least:
                raise ValueError(
                    f"TurningTool {id!r}: insert_clearance_deg={insert_clearance_deg!r} contradicts "
                    f"insert_front_clearance_deg={insert_front_clearance_deg!r} and "
                    f"insert_side_clearance_deg={insert_side_clearance_deg!r}: it is the insert's least clearance "
                    f"({least!r} here); leave it out or give {least!r}"
                )
        if insert_size_tolerance_mm is not None:
            # The in-plane size tolerance (#113): an outline insert's.
            if not isinstance(insert, InsertOutline):
                raise TypeError(
                    f"TurningTool {id!r}: insert_size_tolerance_mm (ISO 1832 d and m) is an InsertOutline's; a "
                    "grooving, threading or drilling tool's size is its declared width, profile or diameter"
                )
            insert_size_tolerance_mm = _size_tolerance(insert_size_tolerance_mm, "insert_size_tolerance_mm")
        elif isinstance(insert, InsertOutline):
            insert_size_tolerance_mm = insert.size_tolerance_mm
        if not drill and insert is not None and solid_bar is None:
            if isinstance(insert, InsertOutline):
                insert_thickness_mm = insert.thickness_mm if insert_thickness_mm is None else insert_thickness_mm
                insert_clearance_deg = insert.clearance_deg if insert_clearance_deg is None else insert_clearance_deg
                edge_deg = insert.edge_deg if edge_deg is None else edge_deg
            else:
                # Grooving and threading edges run along z; a face grooving
                # blade's across r.
                edge_deg = (0.0 if isinstance(insert, FaceGrooveInsert) else 90.0) if edge_deg is None else edge_deg
            missing += [
                name
                for name, value in (
                    ("seat", seat),
                    ("insert_thickness_mm", insert_thickness_mm),
                    ("insert_clearance_deg", insert_clearance_deg),
                    ("edge_deg", edge_deg),
                )
                if value is None
            ]
        if holder is not None and getattr(holder, "y_mm", None) is None:
            missing.append("holder.y_mm (the holder's extent across the centre plane)")
        if missing:
            raise TypeError(
                f"TurningTool {id!r} needs {', '.join(missing)} (turning tools have no default values; "
                "the body off the centre plane is checked too)"
            )
        if drill and any(v is not None for v in (seat, insert_thickness_mm, insert_clearance_deg, edge_deg)):
            raise TypeError(f"TurningTool {id!r}: a drill's outline is its exact section; give no seat or insert body")
        for name, value in (
            ("id", id),
            ("insert", insert),
            ("nose_radius_mm", nose_radius_mm),
            ("side", side),
            ("hand", hand),
            ("holder", holder),
            ("seat", seat),
            ("insert_thickness_mm", insert_thickness_mm),
            ("insert_clearance_deg", insert_clearance_deg),
            ("edge_deg", edge_deg),
            ("curved_for_diameter_mm", None if curved_for_diameter_mm is None else tuple(curved_for_diameter_mm)),
            ("insert_front_clearance_deg", insert_front_clearance_deg),
            ("insert_side_clearance_deg", insert_side_clearance_deg),
            ("solid_bar", solid_bar),
            ("insert_size_tolerance_mm", insert_size_tolerance_mm),
        ):
            object.__setattr__(self, name, value)

    def __setattr__(self, name, value):
        raise AttributeError("TurningTool is immutable")

    def __eq__(self, other):
        return isinstance(other, TurningTool) and self.to_dict() == other.to_dict()

    def __hash__(self):
        return hash(json.dumps(self.to_dict(), sort_keys=True))

    def __repr__(self):
        return (
            f"TurningTool({self.id!r}, insert={self.insert!r}, nose_radius_mm={self.nose_radius_mm!r}, "
            f"side={self.side!r}, hand={self.hand!r}, holder={self.holder!r}, seat={self.seat!r}, "
            f"insert_thickness_mm={self.insert_thickness_mm!r}, insert_clearance_deg={self.insert_clearance_deg!r}, "
            f"edge_deg={self.edge_deg!r}, curved_for_diameter_mm={self.curved_for_diameter_mm!r}, "
            f"insert_front_clearance_deg={self.insert_front_clearance_deg!r}, "
            f"insert_side_clearance_deg={self.insert_side_clearance_deg!r}, "
            f"insert_size_tolerance_mm={self.insert_size_tolerance_mm!r}"
            + (f", solid_bar={self.solid_bar!r})" if self.solid_bar is not None else ")")
        )

    def to_dict(self):
        body = dict(
            holder_y_mm=[float(v) for v in self.holder.y_mm],
            bar_axis_y_mm=float(self.holder.bar.axis_y_mm) if self.holder.bar is not None else 0.0,
        )
        if getattr(self.holder, "model", None) is not None:
            body["model"] = self.holder.model.to_dict()
        if self.seat is not None:
            insert = dict(
                insert_thickness_mm=float(self.insert_thickness_mm),
                insert_clearance_deg=float(self.insert_clearance_deg),
                rake_deg=float(self.seat.rake_deg),
                inclination_deg=float(self.seat.inclination_deg),
                edge_deg=float(self.edge_deg),
            )
            if self.curved_for_diameter_mm is not None:
                insert["curved_for_diameter_mm"] = [float(v) for v in self.curved_for_diameter_mm]
            if self.insert_front_clearance_deg is not None:
                insert["blade_clearance"] = dict(
                    front_deg=self.insert_front_clearance_deg, side_deg=list(self.insert_side_clearance_deg)
                )
            if self.insert_size_tolerance_mm is not None:
                d, m = self.insert_size_tolerance_mm
                insert["size_tolerance"] = dict(d_mm=float(d), m_mm=float(m))
            body["insert"] = insert
        if self.solid_bar is not None:
            body["solid_bar"] = self.solid_bar.to_dict()
        return dict(
            id=self.id,
            insert=self.insert.to_dict(),
            nose_radius_mm=self.nose_radius_mm,
            side=self.side,
            hand=self.hand,
            holder=self.holder.to_dict(),
            body=body,
        )

    def validate(self):
        """Check the tool natively (outlines, corner, holder); raises
        ValueError naming what is wrong."""
        native.turning_validate_tool(json.dumps(self.to_dict()))
        return self


@dataclass(frozen=True)
class TurnToolSetting:
    """The tool in the turret: T number and offset (T0101 = 1, 1)."""

    tool: TurningTool
    number: int = 1
    offset: int = 1

    def to_dict(self):
        return dict(tool=self.tool.to_dict(), number=self.number, offset=self.offset)


def _op(kind, data):
    # Unset options (None) are left out: the engine's defaults.
    return dict(kind=kind, **{k: v for k, v in data.items() if v is not None})


@dataclass(frozen=True)
class TurnFace:
    """Face the stock end down to ``z_mm`` in ``stepdown_mm`` passes, across
    the axis."""

    id: str
    tool: str
    z_mm: float
    stepdown_mm: float = 1
    feed_mm_rev: float = field(kw_only=True)
    rpm: float = field(kw_only=True)
    surface_speed_m_min: float | None = None

    def to_dict(self):
        return _op("face", self.__dict__)


@dataclass(frozen=True)
class TurnRough:
    """Outside roughing in axial passes of ``depth_of_cut_mm`` (G71 type I),
    leaving ``allowance_mm``; each pass ends where the part (plus allowance)
    is reached and retracts ``retract_mm`` at 45 degrees."""

    id: str
    tool: str
    depth_of_cut_mm: float = 2
    allowance_mm: float = 0.3
    feed_mm_rev: float = field(kw_only=True)
    rpm: float = field(kw_only=True)
    surface_speed_m_min: float | None = None
    retract_mm: float = 0.5

    def to_dict(self):
        return _op("rough", self.__dict__)


@dataclass(frozen=True)
class TurnGroove:
    """Grooving with a ``GrooveInsert`` tool: plunges across the groove
    between ``z_from_mm`` and ``z_to_mm`` down to ``bottom_radius_mm``,
    overlapping by ``overlap_mm``, in pecks of ``peck_mm`` if set. The groove
    must be in the part profile (the plunges are checked against it). With a
    boring-side tool (``side="inner"``) the groove is internal: it must be in
    the job's ``bore``; the plunges go outward from inside the open hole,
    which the tool enters and leaves along the axis."""

    id: str
    tool: str
    z_from_mm: float
    z_to_mm: float
    bottom_radius_mm: float
    peck_mm: float | None = None
    overlap_mm: float = 0.2
    feed_mm_rev: float = field(kw_only=True)
    rpm: float = field(kw_only=True)
    surface_speed_m_min: float | None = None

    def to_dict(self):
        data = _op("groove", self.__dict__)
        if self.peck_mm is None:
            data.pop("peck_mm", None)
        return data


@dataclass(frozen=True)
class TurnPartOff:
    """Parting off with a ``GrooveInsert`` tool: its +z side at ``z_mm`` (the
    part's back face), plunged to the axis. The blade must be deeper than
    the stock radius there (its holder is checked against the stock)."""

    id: str
    tool: str
    z_mm: float
    peck_mm: float | None = None
    feed_mm_rev: float = field(kw_only=True)
    rpm: float = field(kw_only=True)
    surface_speed_m_min: float | None = None

    def to_dict(self):
        data = _op("part_off", self.__dict__)
        if self.peck_mm is None:
            data.pop("peck_mm", None)
        return data


@dataclass(frozen=True)
class TurnThread:
    """Single-point threading with a ``ThreadInsert`` tool: ``passes``
    spindle-synchronized passes (constant chip area) from ``z_start_mm`` to
    ``z_end_mm``, infeeding along ``infeed_angle_deg`` (0 radial, 29.5
    flank), plus ``spring_passes``. Outside: down to ``major_radius_mm -
    depth_mm``. Inside (a boring-side tool, the job's ``bore`` at the
    minor diameter): out to ``major_radius_mm``, pulling out toward the
    axis. Posted as G33 K<lead> (LinuxCNC) or G32 F<lead> where the
    dialect declares ``thread_synchronized``."""

    id: str
    tool: str
    z_start_mm: float
    z_end_mm: float
    major_radius_mm: float
    pitch_mm: float
    depth_mm: float
    passes: int = 6
    infeed_angle_deg: float = 29.5
    spring_passes: int = 1
    rpm: float = field(kw_only=True)
    surface_speed_m_min: float | None = None

    def to_dict(self):
        return _op("thread", self.__dict__)


@dataclass(frozen=True)
class FaceGroove:
    """An annular groove in the part's front face (declared on the job):
    between ``inner_radius_mm`` and ``outer_radius_mm``, from the face down to
    ``bottom_z_mm``, square corners."""

    inner_radius_mm: float
    outer_radius_mm: float
    bottom_z_mm: float

    def to_dict(self):
        return dict(self.__dict__)


@dataclass(frozen=True)
class TurnFaceGroove:
    """Face grooving with a ``FaceGrooveInsert`` tool: axial plunges across a
    groove declared in ``TurningJob.face_grooves`` (same radii and bottom),
    overlapping by ``overlap_mm``, in pecks of ``peck_mm`` if set. The blade
    is checked to stay inside the declared groove below the face."""

    id: str
    tool: str
    inner_radius_mm: float
    outer_radius_mm: float
    bottom_z_mm: float
    peck_mm: float | None = None
    overlap_mm: float = 0.2
    feed_mm_rev: float = field(kw_only=True)
    rpm: float = field(kw_only=True)
    surface_speed_m_min: float | None = None

    def to_dict(self):
        return _op("face_groove", self.__dict__)


@dataclass(frozen=True)
class TurnDrill:
    """Drilling on the spindle axis with a ``DrillInsert`` tool, the tip to
    ``depth_z_mm``, in pecks of ``peck_mm`` (out of the hole after each) if
    set. The job must declare the part's ``bore``: the drill is checked to
    stay inside it."""

    id: str
    tool: str
    depth_z_mm: float
    peck_mm: float | None = None
    feed_mm_rev: float = field(kw_only=True)
    rpm: float = field(kw_only=True)
    surface_speed_m_min: float | None = None

    def to_dict(self):
        data = _op("drill", self.__dict__)
        if self.peck_mm is None:
            data.pop("peck_mm", None)
        return data


@dataclass(frozen=True)
class TurnBoreRough:
    """Inside roughing with a boring tool (``side="inner"``): axial passes
    opening the existing hole by at most ``depth_of_cut_mm``, with a pass at
    every bore diameter, each from the front face to where the bore (less
    ``allowance_mm``) is reached or ``z_end_mm``, retracting ``retract_mm``
    at 45 degrees toward the axis. Needs the job's ``bore`` and an open hole
    (stock bore or a drilled one)."""

    id: str
    tool: str
    z_end_mm: float
    depth_of_cut_mm: float = 1.5
    allowance_mm: float = 0.2
    feed_mm_rev: float = field(kw_only=True)
    rpm: float = field(kw_only=True)
    surface_speed_m_min: float | None = None
    retract_mm: float = 0.5

    def to_dict(self):
        return _op("bore_rough", self.__dict__)


@dataclass(frozen=True)
class TurnBoreFinish:
    """Inside finishing along the job's ``bore`` from the front face to
    ``z_end_mm`` with a boring tool, the nose center ``nose radius +
    allowance_mm`` inside it; out toward the axis and along it."""

    id: str
    tool: str
    z_end_mm: float
    allowance_mm: float = 0
    feed_mm_rev: float = field(kw_only=True)
    rpm: float = field(kw_only=True)
    # "computer": the nose path; "wear": the same path with G41/G42 on, the
    # control applying only the wear in its offset's nose radius.
    compensation: str = "computer"
    surface_speed_m_min: float | None = None

    def to_dict(self):
        return _op("bore_finish", self.__dict__)


@dataclass(frozen=True)
class TurnFinish:
    """Outside finishing along the profile (front face first), the nose
    center ``nose radius + allowance_mm`` outside it."""

    id: str
    tool: str
    allowance_mm: float = 0
    feed_mm_rev: float = field(kw_only=True)
    rpm: float = field(kw_only=True)
    # "computer": the nose path; "wear": the same path with G41/G42 on, the
    # control applying only the wear in its offset's nose radius.
    compensation: str = "computer"
    surface_speed_m_min: float | None = None

    def to_dict(self):
        return _op("finish", self.__dict__)


@dataclass(frozen=True, kw_only=True)
class TurningSetup:
    """The turning definition of a lathe ``camlib.jobs.Setup`` (whose stock
    is a ``TurnStockSpec``, tools ``TurnToolSetting`` and operations the
    ``Turn*`` operations, in order).

    ``profile``: the part's half-section (radius, z, bulge) from the front
    face on the axis toward the chuck (``camlib.cadquery.turning_section``
    takes it from a solid). ``chuck_z_mm``: the chuck face; no tool part may
    reach below it (plus the clearance). ``clearance_mm``: the approach and
    retract clearance. ``tolerance_mm``: the tolerance of every move check.
    ``bore``: the part's hole (radius, z, bulge) from the front face to its
    deepest point; drilling and boring stay inside it.

    Every operation runs at ``rpm``, or at constant surface speed
    ``surface_speed_m_min`` (G96) with ``rpm`` as its spindle limit (G50);
    constant surface speed needs ``feed_per_rev=True`` (G95: each feed is
    ``feed_mm_rev``). Threading and drilling run at a fixed rpm.
    ``controlled_point``: the point the program moves and the start position
    names, "tip" (each tool's imaginary tip, where offsets touched off on a
    diameter and a face are measured) or "nose_center". ``face_grooves``:
    annular grooves in the front face (``FaceGroove``), cut by
    ``TurnFaceGroove``."""

    profile: tuple
    chuck_z_mm: float
    clearance_mm: float
    tolerance_mm: float
    bore: tuple | None = None
    feed_per_rev: bool = False
    controlled_point: str = "tip"
    face_grooves: tuple = ()

    @classmethod
    def from_section(cls, section, **settings):
        """From a ``camlib.cadquery.turning_section`` result: its profile,
        bore and face grooves; ``settings`` give the rest."""
        return cls(profile=section.profile, bore=section.bore, face_grooves=section.face_grooves, **settings)

    def to_dict(self, tools, operations):
        """The native turning setup with the setup's tools and operations."""
        result = dict(
            part=dict(points=[list(p) for p in self.profile]),
            chuck_z_mm=self.chuck_z_mm,
            tools=[t if isinstance(t, dict) else t.to_dict() for t in tools],
            operations=[o if isinstance(o, dict) else o.to_dict() for o in operations],
            clearance_mm=self.clearance_mm,
            tolerance_mm=self.tolerance_mm,
            feed_per_rev=self.feed_per_rev,
            controlled_point=self.controlled_point,
            face_grooves=[g.to_dict() for g in self.face_grooves],
        )
        if self.bore is not None:
            result["bore"] = dict(points=[list(p) for p in self.bore])
        return result


def proxy_tools(turning):
    """The posting stand-ins of a native turning setup's tools (the lathe
    run's tool assemblies: id and identity for the program header)."""
    return json.loads(native.turn_proxy_tools(json.dumps(turning)))
