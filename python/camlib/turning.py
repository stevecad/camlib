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

from dataclasses import dataclass, field
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
        return InsertOutline.from_dict(json.loads(data))

    @staticmethod
    def iso_code(code, *, approach_angle_deg=None, holder=None):
        """The whole insert from its ISO 1832 designation (e.g.
        ``"CNMG120408"``): shape C, D, V, S, T, W (negative) or R, size and
        nose radius; every corner rounded, the cutting corner at the nose.
        The approach angle comes from the holder: ``approach_angle_deg``, or
        ``holder`` as an ISO 5608 designation (e.g. ``"PCLNR2525M12"``, style
        L: 95 degrees). Values: ISO 1832:1991 and ISO 5608:1995 tables, and
        the size and corner radius tables of Sumitomo's code key (see
        docs/operations.md)."""
        if (approach_angle_deg is None) == (holder is None):
            raise ValueError(
                "TurningInsert.iso_code needs the holder's approach angle: approach_angle_deg= or holder= (an ISO 5608 code)"
            )
        if holder is not None:
            approach_angle_deg = native.turning_holder_approach(str(holder))
        data, _ = native.turning_iso_code(str(code), float(approach_angle_deg))
        return InsertOutline.from_dict(json.loads(data))


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
    tool frame (parallel to z), from ``z_mm[0]`` to ``z_mm[1]``. Its section
    through its axis bounds its reach on both sides of the spindle axis."""

    diameter_mm: float
    axis_x_mm: float
    z_mm: tuple

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

    def to_dict(self):
        data = dict(outline=self.outline.to_dict())
        if self.bar is not None:
            data["bar"] = self.bar.to_dict()
        return data

    @classmethod
    def rectangle(cls, *, insert, nose_radius_mm, side, width_mm, length_mm, offset_mm):
        """camlib's rectangular holder for ``insert``: outside, a shank
        ``width_mm`` along z and ``length_mm`` away from the cut, starting
        ``offset_mm`` behind the insert's reach (a turning insert's nose
        center toward +z; across a grooving or threading blade); boring
        (``side="inner"``), an axial bar ``width_mm`` across, its top
        ``offset_mm`` under the nose center, ``length_mm`` out of the bore;
        a drill's chuck ``width_mm`` across, ``offset_mm`` behind the
        flutes, ``length_mm`` long."""
        data = native.turning_rectangle_holder(
            json.dumps(insert.to_dict()),
            float(nose_radius_mm),
            str(side),
            float(width_mm),
            float(length_mm),
            float(offset_mm),
        )
        data = json.loads(data)
        return cls(Outline.from_dict(data["outline"]))


# Top-level names (camlib.TurningHolder, camlib.TurningBar; camlib.BoringBar
# is the milling boring bar).
TurningHolder = Holder
TurningBar = Bar

_REMOVED = ("holder_width_mm", "holder_length_mm", "holder_offset_mm")


class TurningTool:
    """A turning tool: ``id``, ``insert`` (an ``InsertOutline``, e.g. from
    ``TurningInsert.iso``/``iso_code``, or a ``GrooveInsert``,
    ``FaceGrooveInsert``, ``ThreadInsert`` or ``DrillInsert``), ``side``
    ("outer", or "inner" for a boring tool cutting outward), ``hand``
    ("right": cuts toward the chuck, or "left"), and ``holder`` (a
    ``Holder``, e.g. ``Holder.rectangle``). ``nose_radius_mm`` is the
    insert's: an ``InsertOutline`` states it (its corner's radius); the
    other inserts need it. Every value is required: a missing one is
    refused with its name."""

    __slots__ = ("id", "insert", "nose_radius_mm", "side", "hand", "holder")

    def __init__(self, id, *, insert=None, side=None, hand=None, holder=None, nose_radius_mm=None, **removed):
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
        if missing:
            raise TypeError(f"TurningTool {id!r} needs {', '.join(missing)} (turning tools have no default values)")
        for name, value in (
            ("id", id),
            ("insert", insert),
            ("nose_radius_mm", nose_radius_mm),
            ("side", side),
            ("hand", hand),
            ("holder", holder),
        ):
            object.__setattr__(self, name, value)

    def __setattr__(self, name, value):
        raise AttributeError("TurningTool is immutable")

    def __eq__(self, other):
        return isinstance(other, TurningTool) and self.to_dict() == other.to_dict()

    def __hash__(self):
        return hash(json.dumps(self.to_dict(), sort_keys=True))

    def __repr__(self):
        return f"TurningTool({self.id!r}, insert={self.insert!r}, nose_radius_mm={self.nose_radius_mm!r}, side={self.side!r}, hand={self.hand!r}, holder={self.holder!r})"

    def to_dict(self):
        return dict(
            id=self.id,
            insert=self.insert.to_dict(),
            nose_radius_mm=self.nose_radius_mm,
            side=self.side,
            hand=self.hand,
            holder=self.holder.to_dict(),
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
