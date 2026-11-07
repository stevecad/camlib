"""Native operation authoring. Geometry is in work XY; depths are absolute mm.

The baseline Pocket clears a horizontal prismatic line/arc region with a flat,
center-cutting end mill. A native iterator plans all passes and feeds the stock
stream directly; Python does not generate or transmit individual path edges.
"""

from dataclasses import dataclass, field
import itertools
import math

from ._resources import bounded_tuple


@dataclass(frozen=True)
class Waterline:
    """Native constant-tip-Z ball finishing; contour_error_mm is normal error."""

    id: str
    tool: object
    surface: object
    floor_z_mm: float
    top_z_mm: float
    bottom_z_mm: float
    stepdown_mm: float
    clearance_z_mm: float
    contour_error_mm: float = 0.01
    safety_margin_mm: float = 0.005
    stock_to_leave_mm: float = 0
    reverse: bool = False
    simplify: bool = True
    tolerance_mm: float = 1e-5
    feed_mm_min: float = field(kw_only=True)
    plunge_mm_min: float = field(kw_only=True)
    spindle_rpm: float = field(kw_only=True)
    work_offset: int = 1
    tool_number: int = field(kw_only=True)
    length_register: int = field(kw_only=True)
    coolant: str = field(kw_only=True)
    contour_method: str = "quad_tree"
    # Contact (slope) angle limits, degrees from the tool axis (0 flat, 90
    # vertical): keep only the parts of each contour within them.
    slope_from_deg: float = 0
    slope_to_deg: float = 90

    # Heights, as in commercial CAM: rapid travel between contours of a level
    # at retract_z_mm, rapid descent to feed_z_mm, feed below it; each level
    # ends at clearance. None keeps clearance_z_mm for both.
    # Required: stock top < feed <= retract <= clearance.
    retract_z_mm: float | None = None
    feed_z_mm: float | None = None
    # Ramp: descend continuously along each level's contour from the level
    # above, instead of cutting each level flat (finished by one flat lap).
    ramp: bool = False

    def to_dict(self):
        result = {field: getattr(self, field) for field in self.__dataclass_fields__}
        result.update(
            kind="waterline",
            tool=getattr(self.tool, "id", self.tool),
            surface=self.surface.to_dict() if hasattr(self.surface, "to_dict") else self.surface,
        )
        for key in ('retract_z_mm', 'feed_z_mm', 'rest_threshold_mm'):
            if result.get(key, 0) is None:
                result.pop(key)
        if not result['ramp']:
            result.pop('ramp')
        _drop_default_slopes(result)
        return result


@dataclass(frozen=True)
class Parallel:
    """Native ball-end surface scan; depths and allowances are in millimetres."""

    id: str
    tool: object
    surface: object
    floor_z_mm: float
    stepover_mm: float
    clearance_z_mm: float
    axis: str = "x"
    reverse: bool = False
    stock_to_leave_mm: float = 0
    path_error_mm: float = 0.005
    safety_margin_mm: float = 0.005
    entry_air_gap_mm: float = 0.5
    feed_mm_min: float = field(kw_only=True)
    plunge_mm_min: float = field(kw_only=True)
    tolerance_mm: float = 1e-5
    spindle_rpm: float = field(kw_only=True)
    work_offset: int = 1
    tool_number: int = field(kw_only=True)
    length_register: int = field(kw_only=True)
    coolant: str = field(kw_only=True)
    # Contact (slope) angle limits, degrees from the tool axis (0 flat, 90
    # vertical): cut only where the surface slope lies within them; each run
    # is then entered with a vertical plunge onto the surface.
    slope_from_deg: float = 0
    slope_to_deg: float = 90
    # Rest finishing: cut only where the stock left by the preceding
    # operations still stands more than this (mm) above the target.
    rest_threshold_mm: float | None = None

    # Heights, as in commercial CAM: rapid travel between rows at
    # retract_z_mm, rapid descent to feed_z_mm (or the row's air entry if
    # higher), feed below it. None keeps clearance_z_mm for both.
    # Required: stock top < feed <= retract <= clearance.
    retract_z_mm: float | None = None
    feed_z_mm: float | None = None

    def to_dict(self):
        result = {field: getattr(self, field) for field in self.__dataclass_fields__}
        result.update(
            kind="parallel",
            tool=getattr(self.tool, "id", self.tool),
            surface=self.surface.to_dict() if hasattr(self.surface, "to_dict") else self.surface,
        )
        for key in ('retract_z_mm', 'feed_z_mm', 'rest_threshold_mm'):
            if result.get(key, 0) is None:
                result.pop(key)
        _drop_default_slopes(result)
        return result


def _pattern_dict(op, kind, pattern_fields):
    result = {field: getattr(op, field) for field in op.__dataclass_fields__ if field not in pattern_fields}
    pattern = {field: getattr(op, field) for field in pattern_fields}
    if kind == "project":
        pattern["paths"] = [path.to_dict() if hasattr(path, "to_dict") else path for path in pattern["paths"]]
    if kind == "morph":
        pattern = {"from": pattern.pop("from_path"), "to": pattern.pop("to_path")}
        pattern = {k: v.to_dict() if hasattr(v, "to_dict") else v for k, v in pattern.items()}
    if pattern.get("outer_radius_mm", 0) is None:
        pattern.pop("outer_radius_mm")
    result.update(
        kind="parallel",
        pattern={"kind": kind, **pattern},
        tool=getattr(op.tool, "id", op.tool),
        surface=op.surface.to_dict() if hasattr(op.surface, "to_dict") else op.surface,
    )
    for key in ("retract_z_mm", "feed_z_mm", "rest_threshold_mm"):
        if result.get(key, 0) is None:
            result.pop(key)
    _drop_default_slopes(result)
    return result


def _drop_default_slopes(result):
    if result.get("slope_from_deg") == 0:
        result.pop("slope_from_deg")
    if result.get("slope_to_deg") == 90:
        result.pop("slope_to_deg")


def steep_and_shallow(
    id,
    tool,
    surface,
    floor_z_mm,
    clearance_z_mm,
    *,
    top_z_mm,
    bottom_z_mm,
    stepdown_mm,
    stepover_mm,
    threshold_deg=45,
    overlap_deg=5,
    shallow="parallel",
    center_mm=None,
    **common,
):
    """Steep-and-shallow finishing as in commercial CAM: waterline contours
    where the surface is steeper than ``threshold_deg``, and parallel (or
    spiral, with ``center_mm``) passes where it is shallower, overlapping by
    ``overlap_deg``. Returns the two operations (steep first) to add to a
    setup; both use ``common`` settings (feeds, allowances, heights, tool
    numbers)."""
    if not 0 < threshold_deg < 90 or not 0 <= overlap_deg < min(threshold_deg, 90 - threshold_deg):
        raise ValueError(
            f"threshold_deg {threshold_deg} must be within (0, 90) and overlap_deg "
            f"{overlap_deg} smaller than its distance to 0 and 90"
        )
    waterline_keys = set(Waterline.__dataclass_fields__)
    steep = Waterline(
        f"{id}-steep",
        tool,
        surface,
        floor_z_mm,
        top_z_mm,
        bottom_z_mm,
        stepdown_mm,
        clearance_z_mm,
        slope_from_deg=threshold_deg - overlap_deg,
        **{k: v for k, v in common.items() if k in waterline_keys},
    )
    if shallow == "parallel":
        cls, extra = Parallel, {}
        args = (f"{id}-shallow", tool, surface, floor_z_mm, stepover_mm, clearance_z_mm)
    elif shallow == "spiral":
        if center_mm is None:
            raise ValueError("a spiral shallow pass needs center_mm")
        cls, extra = Spiral, {}
        args = (f"{id}-shallow", tool, surface, floor_z_mm, stepover_mm, clearance_z_mm, center_mm)
    else:
        raise ValueError("shallow must be 'parallel' or 'spiral'")
    keys = set(cls.__dataclass_fields__)
    flat = cls(
        *args, slope_to_deg=threshold_deg + overlap_deg, **{k: v for k, v in common.items() if k in keys}, **extra
    )
    return steep, flat


@dataclass(frozen=True)
class Scallop:
    """Ball-end scallop (constant stepover) finishing: offsets of the stock
    outline measured along the surface, ``stepover_mm`` apart, so the
    stepover stays about constant on steep and shallow areas. Every chord is
    lifted onto the surface and checked like Parallel."""

    id: str
    tool: object
    surface: object
    floor_z_mm: float
    stepover_mm: float
    clearance_z_mm: float
    stock_to_leave_mm: float = 0
    path_error_mm: float = 0.005
    safety_margin_mm: float = 0.005
    entry_air_gap_mm: float = 0.5
    feed_mm_min: float = field(kw_only=True)
    plunge_mm_min: float = field(kw_only=True)
    tolerance_mm: float = 1e-5
    spindle_rpm: float = field(kw_only=True)
    work_offset: int = 1
    tool_number: int = field(kw_only=True)
    length_register: int = field(kw_only=True)
    coolant: str = field(kw_only=True)
    slope_from_deg: float = 0
    slope_to_deg: float = 90
    # Rest finishing: cut only where the stock left by the preceding
    # operations still stands more than this (mm) above the target.
    rest_threshold_mm: float | None = None
    retract_z_mm: float | None = None
    feed_z_mm: float | None = None

    def to_dict(self):
        return _pattern_dict(self, "scallop", ())


@dataclass(frozen=True)
class Pencil:
    """Ball-end pencil finishing: follows the concave creases (inside corners
    and valleys) where the ball touches two faces at once, found where the
    ball's contact normal turns by at least ``min_angle_deg``. Detection
    samples every ``sample_mm`` (default a quarter of the tool radius); every
    chord is then lifted onto the surface and checked like Parallel."""

    id: str
    tool: object
    surface: object
    floor_z_mm: float
    clearance_z_mm: float
    min_angle_deg: float = 20
    sample_mm: float | None = None
    stock_to_leave_mm: float = 0
    path_error_mm: float = 0.005
    safety_margin_mm: float = 0.005
    entry_air_gap_mm: float = 0.5
    feed_mm_min: float = field(kw_only=True)
    plunge_mm_min: float = field(kw_only=True)
    tolerance_mm: float = 1e-5
    spindle_rpm: float = field(kw_only=True)
    work_offset: int = 1
    tool_number: int = field(kw_only=True)
    length_register: int = field(kw_only=True)
    coolant: str = field(kw_only=True)
    slope_from_deg: float = 0
    slope_to_deg: float = 90
    # Rest finishing: cut only where the stock left by the preceding
    # operations still stands more than this (mm) above the target.
    rest_threshold_mm: float | None = None
    retract_z_mm: float | None = None
    feed_z_mm: float | None = None

    def to_dict(self):
        result = _pattern_dict(self, "pencil", ("min_angle_deg", "sample_mm"))
        if result["pattern"]["sample_mm"] is None:
            result["pattern"].pop("sample_mm")
        return result


@dataclass(frozen=True)
class Spiral:
    """Ball-end spiral finishing: one Archimedean spiral about ``center_mm``
    with ring pitch ``stepover_mm``, clipped to the stock (grown by the
    cutter radius). Each run inside the stock starts with a vertical plunge
    onto the surface. Every chord is checked like Parallel's rows."""

    id: str
    tool: object
    surface: object
    floor_z_mm: float
    stepover_mm: float
    clearance_z_mm: float
    center_mm: tuple
    inner_radius_mm: float = 0
    outer_radius_mm: float | None = None  # None: reach the whole stock
    outward: bool = False
    clockwise: bool = False
    stock_to_leave_mm: float = 0
    path_error_mm: float = 0.005
    safety_margin_mm: float = 0.005
    entry_air_gap_mm: float = 0.5
    feed_mm_min: float = field(kw_only=True)
    plunge_mm_min: float = field(kw_only=True)
    tolerance_mm: float = 1e-5
    spindle_rpm: float = field(kw_only=True)
    work_offset: int = 1
    tool_number: int = field(kw_only=True)
    length_register: int = field(kw_only=True)
    coolant: str = field(kw_only=True)
    slope_from_deg: float = 0
    slope_to_deg: float = 90
    # Rest finishing: cut only where the stock left by the preceding
    # operations still stands more than this (mm) above the target.
    rest_threshold_mm: float | None = None
    retract_z_mm: float | None = None
    feed_z_mm: float | None = None

    def to_dict(self):
        return _pattern_dict(
            self, "spiral", ("center_mm", "inner_radius_mm", "outer_radius_mm", "outward", "clockwise")
        )


@dataclass(frozen=True)
class Radial:
    """Ball-end radial finishing: straight spokes from ``center_mm``,
    ``angular_step_deg`` apart (rounded down so they divide 360 evenly)."""

    id: str
    tool: object
    surface: object
    floor_z_mm: float
    clearance_z_mm: float
    center_mm: tuple
    angular_step_deg: float
    inner_radius_mm: float = 0
    outer_radius_mm: float | None = None
    start_angle_deg: float = 0
    alternate: bool = True
    stock_to_leave_mm: float = 0
    path_error_mm: float = 0.005
    safety_margin_mm: float = 0.005
    entry_air_gap_mm: float = 0.5
    feed_mm_min: float = field(kw_only=True)
    plunge_mm_min: float = field(kw_only=True)
    tolerance_mm: float = 1e-5
    spindle_rpm: float = field(kw_only=True)
    work_offset: int = 1
    tool_number: int = field(kw_only=True)
    length_register: int = field(kw_only=True)
    coolant: str = field(kw_only=True)
    slope_from_deg: float = 0
    slope_to_deg: float = 90
    # Rest finishing: cut only where the stock left by the preceding
    # operations still stands more than this (mm) above the target.
    rest_threshold_mm: float | None = None
    retract_z_mm: float | None = None
    feed_z_mm: float | None = None

    def to_dict(self):
        return _pattern_dict(
            self,
            "radial",
            ("center_mm", "angular_step_deg", "inner_radius_mm", "outer_radius_mm", "start_angle_deg", "alternate"),
        )


@dataclass(frozen=True)
class Morph:
    """Passes morphed between two XY guide curves (``from_path`` and
    ``to_path``, TracePaths followed from their starts), at most
    ``stepover_mm`` apart, followed on the surface like Project: blends
    between two edges, rails or boundaries."""

    id: str
    tool: object
    surface: object
    floor_z_mm: float
    stepover_mm: float
    clearance_z_mm: float
    from_path: object
    to_path: object
    stock_to_leave_mm: float = 0
    path_error_mm: float = 0.005
    safety_margin_mm: float = 0.005
    entry_air_gap_mm: float = 0.5
    feed_mm_min: float = field(kw_only=True)
    plunge_mm_min: float = field(kw_only=True)
    tolerance_mm: float = 1e-5
    spindle_rpm: float = field(kw_only=True)
    work_offset: int = 1
    tool_number: int = field(kw_only=True)
    length_register: int = field(kw_only=True)
    coolant: str = field(kw_only=True)
    slope_from_deg: float = 0
    slope_to_deg: float = 90
    rest_threshold_mm: float | None = None
    retract_z_mm: float | None = None
    feed_z_mm: float | None = None

    def to_dict(self):
        return _pattern_dict(self, "morph", ("from_path", "to_path"))


@dataclass(frozen=True)
class Project:
    """Ball-end curves on a surface: XY ``paths`` (TracePath lines/arcs)
    followed with the ball resting on the surface (plus stock to leave).
    The curves are not offset by the tool radius."""

    id: str
    tool: object
    surface: object
    floor_z_mm: float
    clearance_z_mm: float
    paths: tuple
    stock_to_leave_mm: float = 0
    path_error_mm: float = 0.005
    safety_margin_mm: float = 0.005
    entry_air_gap_mm: float = 0.5
    feed_mm_min: float = field(kw_only=True)
    plunge_mm_min: float = field(kw_only=True)
    tolerance_mm: float = 1e-5
    spindle_rpm: float = field(kw_only=True)
    work_offset: int = 1
    tool_number: int = field(kw_only=True)
    length_register: int = field(kw_only=True)
    coolant: str = field(kw_only=True)
    slope_from_deg: float = 0
    slope_to_deg: float = 90
    # Rest finishing: cut only where the stock left by the preceding
    # operations still stands more than this (mm) above the target.
    rest_threshold_mm: float | None = None
    retract_z_mm: float | None = None
    feed_z_mm: float | None = None

    def to_dict(self):
        return _pattern_dict(self, "project", ("paths",))


@dataclass(frozen=True)
class Reaming:
    """Reaming or boring of existing holes, as the usual G85 cycle: feed down
    from the R plane (``feed_z_mm``, default the retract height) to
    ``depth_z_mm`` and feed back out. ``dwell_s`` dwells at depth (G89);
    ``spindle_stop`` stops the spindle at depth and leaves at rapid (G86).
    ``canned_cycle`` runs each hole from the R plane (then also the height
    between holes) as the controller's G85/G89/G86 cycle, written as one
    block where the machine declares it; it feeds out at ``feed_mm_min``.
    ``fine_bore=FineBore(shift_mm, edge_angle_deg)`` fine bores (G76): at
    depth (after ``dwell_s``) the spindle stops oriented with the edge along
    ``edge_angle_deg``, the bar shifts ``shift_mm`` away from the wall, rapids
    out to R, shifts back and the spindle restarts. It needs a
    ``camlib.tools.BoringBar`` (bar diameter) and a post that declares the
    machine's oriented stop.
    The tool (a reamer or boring bar, modeled as a flat cutter of its
    diameter) does not cut at its center: the stock must show each pilot
    hole of ``pilot_diameter_mm`` open below the depth first."""

    id: str
    tool: object
    centers_mm: object
    pilot_diameter_mm: float
    depth_z_mm: float
    clearance_z_mm: float
    feed_mm_min: float = field(kw_only=True)
    retract_feed_mm_min: float = field(kw_only=True)
    spindle_rpm: float = field(kw_only=True)
    tolerance_mm: float = 1e-5
    work_offset: int = 1
    tool_number: int = field(kw_only=True)
    length_register: int = field(kw_only=True)
    coolant: str = field(kw_only=True)
    retract_z_mm: float | None = None
    feed_z_mm: float | None = None
    dwell_s: float | None = None
    spindle_stop: bool = False
    canned_cycle: bool = False
    fine_bore: "FineBore | None" = None

    def __post_init__(self):
        object.__setattr__(self, "centers_mm", _freeze_hole_centers(self.centers_mm))

    def to_dict(self):
        result = {field: getattr(self, field) for field in self.__dataclass_fields__}
        result.update(kind="reaming", tool=getattr(self.tool, "id", self.tool))
        if self.fine_bore is not None:
            result["fine_bore"] = {"shift_mm": self.fine_bore.shift_mm, "edge_angle_deg": self.fine_bore.edge_angle_deg}
        for key in ("retract_z_mm", "feed_z_mm", "dwell_s", "fine_bore"):
            if result[key] is None:
                result.pop(key)
        for key in ("spindle_stop", "canned_cycle"):
            if not result[key]:
                result.pop(key)
        return result


@dataclass(frozen=True)
class FineBore:
    """Fine boring (G76) for :class:`Reaming`: the shift away from the wall
    at the bottom, and where the cutting edge points after the oriented stop
    (work XY, degrees from +X; the shift goes the opposite way). The edge
    direction is a setup premise the post checks against the machine's
    declared ``oriented_edge_deg``."""

    shift_mm: float
    edge_angle_deg: float


@dataclass(frozen=True)
class Tapping:
    """Rigid tapping of drilled holes with a ``camlib.tools.Tap``.

    ``hand``: ``"right"`` (default: a thread is right-hand unless designated
    LH, ISO 965-1 / ASME B1.1) turns the spindle clockwise (M3) into the hole;
    ``"left"`` counterclockwise (M4). The controller reverses it for the
    return. A machine without a left-hand cycle (G33.1, or a declared
    ``tap_cycle_left`` such as G74) refuses a left-hand tap at posting.

    Each hole is tapped from the R plane (``feed_z_mm``, default the retract
    height) to ``depth_z_mm`` and back at the synchronized feed
    ``pitch_mm`` x ``spindle_rpm``. The post writes one controller cycle per
    hole (G33.1 on LinuxCNC, G84 ... G80 where the machine declares it) and
    reads it back for checking. Each hole must already be drilled below the
    tap depth plus ``bottom_clearance_mm`` (default one pitch) in the stock.
    """

    id: str
    tool: object
    centers_mm: object
    pitch_mm: float
    depth_z_mm: float
    clearance_z_mm: float
    spindle_rpm: float = field(kw_only=True)
    bottom_clearance_mm: float | None = None
    tolerance_mm: float = 1e-5
    work_offset: int = 1
    tool_number: int = field(kw_only=True)
    length_register: int = field(kw_only=True)
    coolant: str = field(kw_only=True)
    retract_z_mm: float | None = None
    feed_z_mm: float | None = None
    hand: str = field(default="right", kw_only=True)

    def __post_init__(self):
        object.__setattr__(self, "centers_mm", _freeze_hole_centers(self.centers_mm))
        if self.hand not in ("right", "left"):
            raise ValueError(f"Tapping hand must be 'right' or 'left', not {self.hand!r}")

    def to_dict(self):
        result = {field: getattr(self, field) for field in self.__dataclass_fields__}
        result.update(kind="tapping", tool=getattr(self.tool, "id", self.tool))
        if result["hand"] == "right":
            result.pop("hand")
        for key in ("retract_z_mm", "feed_z_mm", "bottom_clearance_mm"):
            if result[key] is None:
                result.pop(key)
        return result


@dataclass(frozen=True)
class ThreadMilling:
    """Helical thread milling with a single-profile thread mill (see
    ``camlib.tools.ThreadMill``), internal (in a drilled hole) or external
    (around a boss).

    ``diameter_mm`` is the final diameter reached by the tooth tip: the major
    diameter of an internal thread, the minor diameter of an external one.
    ``start_diameter_mm`` is the existing hole or boss. The tool tip moves
    between ``bottom_z_mm`` and ``top_z_mm``, one turn per ``pitch_mm``, with
    tangent arc leads. Climb milling of a right-hand thread runs bottom-up in
    a hole and top-down around a boss. The current stock must show the hole
    drilled (or the boss cleared around) before the tool goes down.
    ``feed_mm_min`` is the programmed feed at the tool center.
    """

    id: str
    tool: object
    centers_mm: object
    internal: bool
    diameter_mm: float
    start_diameter_mm: float
    pitch_mm: float
    top_z_mm: float
    bottom_z_mm: float
    clearance_z_mm: float
    right_hand: bool = True
    climb: bool = True
    radial_passes: int = 1
    spring_passes: int = 0
    feed_mm_min: float = field(kw_only=True)
    plunge_mm_min: float = field(kw_only=True)
    tolerance_mm: float = 1e-5
    spindle_rpm: float = field(kw_only=True)
    work_offset: int = 1
    tool_number: int = field(kw_only=True)
    length_register: int = field(kw_only=True)
    coolant: str = field(kw_only=True)
    retract_z_mm: float | None = None
    feed_z_mm: float | None = None

    def __post_init__(self):
        object.__setattr__(self, "centers_mm", _freeze_hole_centers(self.centers_mm))

    def to_dict(self):
        result = {field: getattr(self, field) for field in self.__dataclass_fields__}
        result.update(kind="thread_mill", tool=getattr(self.tool, "id", self.tool))
        for key in ("retract_z_mm", "feed_z_mm"):
            if result[key] is None:
                result.pop(key)
        return result


@dataclass(frozen=True)
class Flat:
    """Flat-area finishing: finds the horizontal faces of ``surface`` above
    ``floor_z_mm`` and finishes each at its exact height (plus
    ``stock_to_leave_mm``) with a flat, center-cutting end mill, in raster
    rows ``stepover_mm`` apart along ``axis``. Each cut is kept only where
    the part under the tool's swept disk is proven not above it; each run is
    entered with a vertical plunge. Faces within ``level_tolerance_mm`` form
    one level."""

    id: str
    tool: object
    surface: object
    floor_z_mm: float
    stepover_mm: float
    clearance_z_mm: float
    axis: str = "x"
    stock_to_leave_mm: float = 0
    level_tolerance_mm: float = 0.001
    feed_mm_min: float = field(kw_only=True)
    plunge_mm_min: float = field(kw_only=True)
    tolerance_mm: float = 1e-5
    spindle_rpm: float = field(kw_only=True)
    work_offset: int = 1
    tool_number: int = field(kw_only=True)
    length_register: int = field(kw_only=True)
    coolant: str = field(kw_only=True)
    retract_z_mm: float | None = None
    feed_z_mm: float | None = None

    def to_dict(self):
        result = {field: getattr(self, field) for field in self.__dataclass_fields__}
        result.update(
            kind="flat",
            tool=getattr(self.tool, "id", self.tool),
            surface=self.surface.to_dict() if hasattr(self.surface, "to_dict") else self.surface,
        )
        for key in ("retract_z_mm", "feed_z_mm"):
            if result[key] is None:
                result.pop(key)
        return result


@dataclass(frozen=True)
class DeburrLevel:
    """The upward-facing planar faces of a part at one height ``z_mm``, as
    planar regions (unioned). ``camlib.cadquery.deburr_levels`` makes them
    from a solid."""

    regions: tuple
    z_mm: float

    def __post_init__(self):
        object.__setattr__(self, "regions", tuple(self.regions))

    def to_dict(self):
        return dict(regions=[r.to_dict() if hasattr(r, "to_dict") else r for r in self.regions], z_mm=float(self.z_mm))


@dataclass(frozen=True)
class Deburr:
    """Break the horizontal edges of a finished part with a chamfer mill.

    Each level's face boundaries are broken ``width_mm`` wide on the face
    (and the chamfer depth down a vertical wall), with the tool's cone. The
    rest of the part (``surface``, the finished part as a ``SurfaceMesh``) is
    proven untouched for the whole tool assembly; a stretch that cannot be
    proven is not cut and is reported (``unreachable_length_mm``; concave
    stretches, where a wall rises from the face, in ``concave_length_mm``).
    The stock must already be machined to the part.
    """

    id: str
    tool: object
    surface: object
    levels: tuple
    width_mm: float
    clearance_z_mm: float
    tip_clearance_mm: float = 0.5
    cutting_sense: str = "climb"
    boundary_tolerance_mm: float = 0.01
    tolerance_mm: float = 1e-5
    feed_mm_min: float = field(kw_only=True)
    plunge_mm_min: float = field(kw_only=True)
    spindle_rpm: float = field(kw_only=True)
    work_offset: int = 1
    tool_number: int = field(kw_only=True)
    length_register: int = field(kw_only=True)
    coolant: str = field(kw_only=True)
    retract_z_mm: float | None = None
    feed_z_mm: float | None = None

    def __post_init__(self):
        object.__setattr__(self, "levels", tuple(self.levels))

    def to_dict(self):
        result = {field: getattr(self, field) for field in self.__dataclass_fields__}
        result.update(
            kind="deburr",
            tool=getattr(self.tool, "id", self.tool),
            surface=self.surface.to_dict() if hasattr(self.surface, "to_dict") else self.surface,
            levels=[level.to_dict() if hasattr(level, "to_dict") else level for level in self.levels],
        )
        for key in ("retract_z_mm", "feed_z_mm"):
            if result[key] is None:
                result.pop(key)
        return result


@dataclass(frozen=True)
class Chamfer:
    """Bevel selected closed top edges with the tool's included angle/flat tip.

    width_mm is horizontal width; axial depth comes from the cutter profile.
    Non-center tools require a stock-proven cleared tip entry before any motion.
    """

    id: str
    tool: object
    region: object
    top_z_mm: float
    width_mm: float
    clearance_z_mm: float
    boundaries: str = "outer"
    tip_clearance_mm: float = 0.5
    width_step_mm: float | None = None
    boundary_tolerance_mm: float = 0.01
    tolerance_mm: float = 1e-5
    # None: from cutting_sense. True/False: against/along the boundary direction.
    reverse: bool | None = None
    lead_in_mm: float = 0
    lead_out_mm: float = 0
    feed_mm_min: float = field(kw_only=True)
    plunge_mm_min: float = field(kw_only=True)
    spindle_rpm: float = field(kw_only=True)
    work_offset: int = 1
    tool_number: int = field(kw_only=True)
    length_register: int = field(kw_only=True)
    coolant: str = field(kw_only=True)

    # Heights, as in commercial CAM: rapid travel between passes at
    # retract_z_mm, rapid descent to feed_z_mm, feed below it. None keeps
    # clearance_z_mm for both. Required: stock top < feed <= retract <= clearance.
    retract_z_mm: float | None = None
    feed_z_mm: float | None = None
    # 'climb' (default) or 'conventional' milling with a clockwise spindle.
    cutting_sense: str = "climb"

    def to_dict(self):
        result = {field: getattr(self, field) for field in self.__dataclass_fields__}
        result.update(
            kind="chamfer",
            tool=getattr(self.tool, "id", self.tool),
            region=self.region.to_dict() if hasattr(self.region, "to_dict") else self.region,
            reverse=_profile_reverse(self.reverse, result.pop("cutting_sense")),
        )
        for key in ('retract_z_mm', 'feed_z_mm'):
            if result[key] is None:
                result.pop(key)
        return result


def _profile_reverse(reverse, cutting_sense):
    """Engine profile direction. Outer boundaries run counter-clockwise and
    holes clockwise, which is conventional milling with a clockwise spindle:
    climb runs against them."""
    if cutting_sense not in ("climb", "conventional"):
        raise ValueError("cutting_sense must be 'climb' or 'conventional'")
    if reverse is not None:
        if type(reverse) is not bool:
            raise ValueError("reverse must be a bool or None")
        return reverse
    return cutting_sense == "climb"


def _freeze_hole_centers(values):
    # Any finite iterable; refused before it outgrows memory (~200 B/center).
    result = bounded_tuple(values, 200, "hole centers", lambda p: tuple(itertools.islice(iter(p), 3)))
    if not result:
        raise ValueError("hole operations need at least one center")
    if any(len(p) != 2 for p in result):
        raise ValueError("hole centers need exactly two coordinates")
    return result


@dataclass(frozen=True)
class Drilling:
    """Explicit axial moves; depth_z_mm is tip Z unless full_diameter is chosen."""

    id: str
    tool: object
    centers_mm: object
    depth_z_mm: float
    clearance_z_mm: float
    depth_reference: str = "tip"
    peck_mm: float | None = None
    peck_mode: str = "full_retract"
    retract_z_mm: float | None = None
    chip_break_lift_mm: float = 1
    reentry_clearance_mm: float = 0.5
    allow_breakthrough_mm: float = 0
    feed_mm_min: float = field(kw_only=True)
    retract_feed_mm_min: float = field(kw_only=True)
    tolerance_mm: float = 1e-5
    spindle_rpm: float = field(kw_only=True)
    work_offset: int = 1
    tool_number: int = field(kw_only=True)
    length_register: int = field(kw_only=True)
    coolant: str = field(kw_only=True)
    # Dwell at full depth (seconds), as the G82 cycle: spot drilling,
    # counterboring. Posted as G4 after the last cut into each hole.
    dwell_s: float | None = None
    # Move exactly as the controller's canned cycle (G81; G82 with dwell_s;
    # G83 full-retract or G73 chip-break pecks; G98 return): pecks measured
    # from the R plane, rapid retracts. Posts write one cycle block per hole
    # where the machine declares the cycle, explicit moves otherwise.
    # reentry_clearance_mm / chip_break_lift_mm must equal the controller's
    # return amount (LinuxCNC: 0.254 mm).
    canned_cycle: bool = False

    def __post_init__(self):
        object.__setattr__(self, "centers_mm", _freeze_hole_centers(self.centers_mm))

    def to_dict(self):
        result = {field: getattr(self, field) for field in self.__dataclass_fields__}
        result.update(kind="drilling", tool=getattr(self.tool, "id", self.tool))
        if result["dwell_s"] is None:
            del result["dwell_s"]
        if not result["canned_cycle"]:
            del result["canned_cycle"]
        return result


@dataclass(frozen=True)
class Bore:
    """Clear full cylindrical holes with helical entry and concentric widening."""

    id: str
    tool: object
    centers_mm: object
    diameter_mm: float
    floor_z_mm: float
    stepdown_mm: float
    clearance_z_mm: float
    radial_stepover_mm: float | None = None
    radial_allowance_mm: float = 0
    bottom_allowance_mm: float = 0
    boundary_tolerance_mm: float = 0.01
    clockwise: bool = True
    entry_feed_mm_min: float = field(kw_only=True)
    feed_mm_min: float = field(kw_only=True)
    retract_feed_mm_min: float = field(kw_only=True)
    tolerance_mm: float = 1e-5
    spindle_rpm: float = field(kw_only=True)
    work_offset: int = 1
    tool_number: int = field(kw_only=True)
    length_register: int = field(kw_only=True)
    coolant: str = field(kw_only=True)

    def __post_init__(self):
        object.__setattr__(self, "centers_mm", _freeze_hole_centers(self.centers_mm))

    # Heights, as in commercial CAM: rapid travel between holes at
    # retract_z_mm, rapid descent to feed_z_mm, entry feed below it. None keeps
    # clearance_z_mm for both. Required: stock top < feed <= retract <= clearance.
    retract_z_mm: float | None = None
    feed_z_mm: float | None = None

    def to_dict(self):
        result = {field: getattr(self, field) for field in self.__dataclass_fields__}
        result.update(kind="bore", tool=getattr(self.tool, "id", self.tool))
        for key in ('retract_z_mm', 'feed_z_mm'):
            if result[key] is None:
                result.pop(key)
        return result


@dataclass(frozen=True)
class Tabs:
    """Holding tabs left on a contour so a through-cut part stays attached.

    ``count`` tabs per loop (or one every ``spacing_mm`` of loop length),
    ``width_mm`` wide along the part edge and ``height_mm`` above the floor.
    ``shape`` is 'triangle' (45 degree ramps, the default) or 'rectangle'
    (straight up and over). Passes above the tab top are not changed."""

    width_mm: float
    height_mm: float
    count: int | None = 4
    spacing_mm: float | None = None
    shape: str = "triangle"

    def __post_init__(self):
        if (self.count is None) == (self.spacing_mm is None):
            raise ValueError("give tabs either count or spacing_mm")
        if self.shape not in ("triangle", "rectangle"):
            raise ValueError("tab shape must be 'triangle' or 'rectangle'")

    def to_dict(self):
        result = {"width_mm": self.width_mm, "height_mm": self.height_mm, "shape": self.shape}
        if self.count is not None:
            result["count"] = self.count
        else:
            result["spacing_mm"] = self.spacing_mm
        return result


@dataclass(frozen=True)
class Contour:
    """Cut selected outer/hole walls of a RETAINED part footprint.

    boundaries is outer/holes/all. A contour cuts a band, not all outside stock.
    The 0.01mm boundary budget includes an explicit half-budget safety margin.
    Leads are optional; every profile/lead is continuously checked before motion.
    """

    id: str
    tool: object
    region: object
    floor_z_mm: float
    stepdown_mm: float
    clearance_z_mm: float
    boundaries: str = "outer"
    radial_allowance_mm: float = 0
    bottom_allowance_mm: float = 0
    feed_mm_min: float = field(kw_only=True)
    plunge_mm_min: float = field(kw_only=True)
    boundary_tolerance_mm: float = 0.01
    tolerance_mm: float = 1e-5
    # None: from cutting_sense. True/False: against/along the boundary
    # direction (outer boundaries run counter-clockwise, holes clockwise).
    reverse: bool | None = None
    lead_in_mm: float = 0
    lead_out_mm: float = 0
    spindle_rpm: float = field(kw_only=True)
    work_offset: int = 1
    tool_number: int = field(kw_only=True)
    length_register: int = field(kw_only=True)
    coolant: str = field(kw_only=True)

    # Heights, as in commercial CAM: rapid travel between passes at
    # retract_z_mm, rapid descent to feed_z_mm, feed below it. None keeps
    # clearance_z_mm for both. Required: stock top < feed <= retract <= clearance.
    retract_z_mm: float | None = None
    feed_z_mm: float | None = None
    # "computer": the path is the tool center. "wear": the same path, posted
    # with G41/G42 (on over the lead-in, off over the lead-out) so the
    # controller adds the tool's wear register; needs both leads.
    compensation: str = "computer"
    # 'climb' (default) or 'conventional' milling with a clockwise spindle.
    cutting_sense: str = "climb"
    # Holding tabs (camlib.Tabs) on through-cuts; None: no tabs.
    tabs: object | None = None

    def to_dict(self):
        result = {field: getattr(self, field) for field in self.__dataclass_fields__}
        result.update(
            kind="contour",
            tool=getattr(self.tool, "id", self.tool),
            region=self.region.to_dict() if hasattr(self.region, "to_dict") else self.region,
            reverse=_profile_reverse(self.reverse, result.pop("cutting_sense")),
        )
        tabs = result.pop("tabs")
        if tabs is not None:
            result["tabs"] = tabs.to_dict() if hasattr(tabs, "to_dict") else dict(tabs)
        for key in ('retract_z_mm', 'feed_z_mm'):
            if result[key] is None:
                result.pop(key)
        if result['compensation'] == 'computer':
            result.pop('compensation')
        return result


@dataclass(frozen=True)
class Facing:
    """Level the whole stock rectangle; enter outside its cutting-axis edges.

    axis is x/y, mode is zigzag/one_way. None edge overlap uses 10% of diameter.
    The initial stock top is the depth reference. Cutter overtravel is intentional.
    """

    id: str
    tool: object
    floor_z_mm: float
    stepdown_mm: float
    stepover_mm: float
    clearance_z_mm: float
    feed_mm_min: float = field(kw_only=True)
    plunge_mm_min: float = field(kw_only=True)
    axis: str = "x"
    mode: str = "zigzag"
    reverse: bool = False
    entry_air_gap_mm: float = 1
    edge_overlap_mm: float | None = None
    tolerance_mm: float = 1e-5
    spindle_rpm: float = field(kw_only=True)
    work_offset: int = 1
    tool_number: int = field(kw_only=True)
    length_register: int = field(kw_only=True)
    coolant: str = field(kw_only=True)

    # Heights, as in commercial CAM: rapid travel between passes at
    # retract_z_mm, rapid descent to feed_z_mm, feed below it. None keeps
    # clearance_z_mm for both. Required: stock top < feed <= retract <= clearance.
    retract_z_mm: float | None = None
    feed_z_mm: float | None = None

    def to_dict(self):
        result = {field: getattr(self, field) for field in self.__dataclass_fields__}
        result.update(kind="facing", tool=getattr(self.tool, "id", self.tool))
        for key in ('retract_z_mm', 'feed_z_mm'):
            if result[key] is None:
                result.pop(key)
        return result


@dataclass(frozen=True)
class HelicalEntry:
    """Source-fitted helical entry; angle in degrees, lengths in mm.

    None radius fits automatically, preferring half cutter radius. Feed defaults
    to the operation's plunge feed; sense follows the operation. A floor circle
    and checked navigation reach the unchanged clearing strategy.
    """

    angle_deg: float = 3
    radius_mm: float | None = None
    feed_mm_min: float | None = None
    clockwise: bool | None = None

    def to_dict(self):
        import math

        result = dict(kind="helix", angle_rad=math.radians(self.angle_deg))
        for key in ("radius_mm", "feed_mm_min", "clockwise"):
            if getattr(self, key) is not None:
                result[key] = getattr(self, key)
        return result


@dataclass(frozen=True)
class StockEdgeEntry:
    """Prefer a whole cutter/seed disk outside stock; otherwise fit this helix.

    Set helix=None to require outside-stock entry for every component.
    """

    helix: HelicalEntry | None = HelicalEntry()

    def to_dict(self):
        if self.helix is None:
            return dict(kind="stock_edge")
        return dict(self.helix.to_dict(), kind="stock_edge_or_helix")


@dataclass(frozen=True)
class RasterClearing:
    stepover_mm: float
    routing: str | None = None

    def to_dict(self):
        result = dict(kind="raster", stepover_mm=self.stepover_mm)
        if self.routing is not None:
            result['routing'] = self.routing
        return result


@dataclass(frozen=True)
class OffsetClearing:
    """Native contour-offset fill; spacing bounds overlap, not engagement.

    coverage_margin_mm reserves cutter overlap for a whole-domain sweep check.
    """

    stepover_mm: float
    coverage_margin_mm: float = 0.01

    def to_dict(self):
        return dict(kind="offset", stepover_mm=self.stepover_mm, coverage_margin_mm=self.coverage_margin_mm)


@dataclass(frozen=True)
class AdaptiveClearing:
    """Maximum radial width in mm; Rust derives its circumference-angle bound.

    path_strategy='stock_front' is a development strategy: it can stream a
    productive prefix but raises if coverage/steering is unresolved. Its
    cutting_sense is 'climb' (default) or 'conventional' with the CW spindle.
    """

    max_radial_engagement_mm: float
    orbit_radius_mm: float | None = None
    min_orbit_radius_mm: float | None = None
    local_orbit_tolerance_mm: float | None = None
    boundary_tolerance_mm: float = 0.01

    path_strategy: str = "disk_front"
    cutting_sense: str | None = None

    def to_dict(self):
        result = dict(kind="adaptive", **{k: getattr(self, k) for k in self.__dataclass_fields__})
        if self.path_strategy == "disk_front":
            result.pop("path_strategy")
        if self.cutting_sense is None:
            result.pop("cutting_sense")
        return result


@dataclass(frozen=True)
class Roughing:
    """Clear stock around a retained target with native full-column protection.

    Target is a bounded SurfaceMesh. from_solid exports a real CadQuery solid
    with the bounded CAD surface adapter. Rust plans paths;
    Python does not author individual cutting moves.
    """

    id: str
    tool: object
    target: object
    top_z_mm: float
    floor_z_mm: float
    stepdown_mm: float
    clearance_z_mm: float
    clearing: RasterClearing | OffsetClearing | AdaptiveClearing
    feed_mm_min: float = field(kw_only=True)
    plunge_mm_min: float = field(kw_only=True)
    radial_allowance_mm: float = 0
    bottom_allowance_mm: float = 0
    tolerance_mm: float = 1e-5
    spindle_rpm: float = field(kw_only=True)
    work_offset: int = 1
    tool_number: int = field(kw_only=True)
    length_register: int = field(kw_only=True)
    coolant: str = field(kw_only=True)
    clockwise: bool = True
    entry: StockEdgeEntry | HelicalEntry = StockEdgeEntry()
    clearing_tolerance_mm: float | None = None
    require_part_at_first_depth: bool = False
    omit_empty_cuts: bool = False
    rest_tolerance_mm: float | None = None
    # See Pocket.retract_z_mm/feed_z_mm. Raster and offset clearing only.
    retract_z_mm: float | None = None
    feed_z_mm: float | None = None

    @classmethod
    def from_solid(cls, id, tool, solid, *, source=None, **parameters):
        from .cadquery import export_surface_mesh

        return cls(id, tool, export_surface_mesh(solid, source=source), **parameters)

    def to_dict(self):
        result = {key: getattr(self, key) for key in self.__dataclass_fields__}
        result.update(kind="roughing", tool=getattr(self.tool, "id", self.tool))
        if self.rest_tolerance_mm is None:
            result.pop('rest_tolerance_mm')
        if type(self.omit_empty_cuts) is not bool:
            raise ValueError('omit_empty_cuts must be a boolean')
        if not self.omit_empty_cuts:
            result.pop('omit_empty_cuts')
        if self.clearing_tolerance_mm is None:
            result.pop('clearing_tolerance_mm')
        if not self.require_part_at_first_depth:
            result.pop('require_part_at_first_depth')
        for key in ('retract_z_mm', 'feed_z_mm'):
            if result[key] is None:
                result.pop(key)
        for key in ("target", "clearing", "entry"):
            if hasattr(result[key], "to_dict"):
                result[key] = result[key].to_dict()
        return result


@dataclass(frozen=True)
class RestRoughing(Roughing):
    """Full-column protection with rest filtering from committed stock.

    Use RasterClearing with default routing or OffsetClearing. Offset guides
    retain their contour sense and are scheduled before subdivision/filtering.
    The whole-curve stock enclosure keeps uncertain pieces and reports its work/height
    tolerance. Smaller tools can remove stock left by earlier cutters without repeating
    proven-cleared cutting strokes.
    """

    rest_tolerance_mm: float = 0.01


@dataclass(frozen=True)
class Pocket:
    id: str
    tool: object
    region: object
    top_z_mm: float
    floor_z_mm: float
    stepdown_mm: float
    stepover_mm: float
    clearance_z_mm: float
    feed_mm_min: float = field(kw_only=True)
    plunge_mm_min: float = field(kw_only=True)
    radial_allowance_mm: float = 0
    bottom_allowance_mm: float = 0
    tolerance_mm: float = 1e-5
    spindle_rpm: float = field(kw_only=True)
    work_offset: int = 1
    tool_number: int = field(kw_only=True)
    length_register: int = field(kw_only=True)
    coolant: str = field(kw_only=True)
    rest_tolerance_mm: float | None = None
    boundary_clockwise: bool | None = None
    entry: HelicalEntry | StockEdgeEntry | None = None
    # Non-cutting planes (absolute Z), like the retract/feed heights of other
    # CAM systems. None keeps the default: travel, approach and departure all
    # at clearance_z_mm. Required order: top < feed <= retract <= clearance.
    retract_z_mm: float | None = None
    feed_z_mm: float | None = None
    # Longest link kept at cutting depth between regions (proven through
    # cleared stock only). None: link at depth when that is faster than
    # retracting to feed_z_mm and plunging again; 0: always retract.
    max_stay_down_mm: float | None = None

    def to_dict(self):
        # Avoid deep-copying arbitrary CAD source objects through asdict.
        result = {field: getattr(self, field) for field in self.__dataclass_fields__}
        result.update(
            kind="pocket",
            tool=getattr(self.tool, "id", self.tool),
            region=self.region.to_dict() if hasattr(self.region, "to_dict") else self.region,
        )
        if self.rest_tolerance_mm is None:
            result.pop('rest_tolerance_mm')
        if self.boundary_clockwise is None:
            result.pop('boundary_clockwise')
        if self.entry is None:
            result.pop('entry')
        else:
            result['entry'] = self.entry.to_dict() if hasattr(self.entry, 'to_dict') else self.entry
        for key in ('retract_z_mm', 'feed_z_mm', 'max_stay_down_mm'):
            if result[key] is None:
                result.pop(key)
        return result


@dataclass(frozen=True)
class RestPocket(Pocket):
    """Cull cleared strokes using committed stock and an explicit height budget."""

    rest_tolerance_mm: float = 0.01


@dataclass(frozen=True)
class AdaptivePocket(Pocket):
    """Region-based bounded engagement, with one seed per guide component/depth.

    None selects orbit radius equal to half the native cutter radius. Narrow
    features/corner rest are reported; plunge and seed orbit use plunge feed
    outside the steady engagement guarantee. Stock-clear jumps need zero added
    height tolerance, so rest_tolerance_mm must remain None.

    Set min_orbit_radius_mm below orbit_radius_mm and local_orbit_tolerance_mm
    (for example .05 mm) to size orbits from whole-guide clearance. This controls
    radius selection, separately from machining and boundary tolerances. None
    preserves the existing fixed/two-scale strategies.

    path_strategy='stock_front' selects the replacement under development; it
    raises on unresolved steering/coverage. Reachable-domain coverage and clear
    re-entry are implemented; broad geometry, dynamics and speed remain under
    qualification.
    cutting_sense='climb' (default) or 'conventional' describes material relative
    to feed with the CW spindle. With disk_front, clockwise sets the orbit direction.
    """

    orbit_radius_mm: float | None = None
    max_engagement_deg: float = 90
    boundary_tolerance_mm: float = 0.01
    clockwise: bool = True
    min_orbit_radius_mm: float | None = None
    local_orbit_tolerance_mm: float | None = None

    path_strategy: str = "disk_front"
    cutting_sense: str | None = None

    def to_dict(self):
        pocket = {field: getattr(self, field) for field in Pocket.__dataclass_fields__}
        pocket.update(
            tool=getattr(self.tool, "id", self.tool),
            region=self.region.to_dict() if hasattr(self.region, "to_dict") else self.region,
        )
        if pocket["rest_tolerance_mm"] is None:
            pocket.pop("rest_tolerance_mm")
        if pocket["boundary_clockwise"] is None:
            pocket.pop("boundary_clockwise")
        if pocket["entry"] is None:
            pocket.pop("entry")
        else:
            pocket["entry"] = self.entry.to_dict() if hasattr(self.entry, 'to_dict') else self.entry
        result = dict(
            pocket,
            kind="adaptive_pocket",
            max_engagement_deg=self.max_engagement_deg,
            boundary_tolerance_mm=self.boundary_tolerance_mm,
            clockwise=self.clockwise,
        )
        if self.path_strategy != "disk_front":
            result["path_strategy"] = self.path_strategy
        if self.cutting_sense is not None:
            result["cutting_sense"] = self.cutting_sense
        if self.orbit_radius_mm is not None:
            result["orbit_radius_mm"] = self.orbit_radius_mm
        if self.min_orbit_radius_mm is not None:
            result["min_orbit_radius_mm"] = self.min_orbit_radius_mm
        if self.local_orbit_tolerance_mm is not None:
            result["local_orbit_tolerance_mm"] = self.local_orbit_tolerance_mm
        return result


@dataclass(frozen=True)
class TraceLine:
    """Straight segment to ``end_mm`` (x, y)."""

    end_mm: tuple

    def to_dict(self):
        return {"kind": "line", "end_mm": [float(v) for v in self.end_mm]}


@dataclass(frozen=True)
class TraceArc:
    """Arc about ``center_mm`` (x, y) through ``sweep_deg`` degrees from the
    current point: positive is counter-clockwise, negative clockwise. The end
    point follows from the sweep, so the radius is always consistent."""

    center_mm: tuple
    sweep_deg: float

    def to_dict(self):
        return {
            "kind": "arc",
            "center_mm": [float(v) for v in self.center_mm],
            "sweep_rad": math.radians(self.sweep_deg),
        }


@dataclass(frozen=True)
class TracePath:
    """A path from ``start_mm`` (x, y) through ``segments`` (TraceLine/TraceArc).
    It is closed when the last segment ends on the start."""

    start_mm: tuple
    segments: tuple

    def __post_init__(self):
        object.__setattr__(self, "segments", tuple(self.segments))

    def to_dict(self):
        return {
            "start_mm": [float(v) for v in self.start_mm],
            "segments": [segment.to_dict() for segment in self.segments],
        }


@dataclass(frozen=True)
class VCarve:
    """V-carve a region (letters, a logo) with a V (chamfer) bit: for each
    point of the outline the bit goes to the largest disk inside the region
    that touches it, as deep as needed for its cone to be that wide at
    ``top_z_mm``. Sharp convex corners run out to the surface. ``max_depth_mm``
    limits the depth (a region too wide for it is refused: its middle needs a
    flat clearing tool)."""

    id: str
    tool: object
    region: object
    top_z_mm: float
    clearance_z_mm: float
    max_depth_mm: float | None = None
    sample_mm: float = 0.05
    feed_mm_min: float = field(kw_only=True)
    plunge_mm_min: float = field(kw_only=True)
    tolerance_mm: float = 1e-5
    spindle_rpm: float = field(kw_only=True)
    work_offset: int = 1
    tool_number: int = field(kw_only=True)
    length_register: int = field(kw_only=True)
    coolant: str = field(kw_only=True)
    retract_z_mm: float | None = None
    feed_z_mm: float | None = None

    def to_dict(self):
        result = {field: getattr(self, field) for field in self.__dataclass_fields__}
        result.update(
            kind="v_carve",
            tool=getattr(self.tool, "id", self.tool),
            region=self.region.to_dict() if hasattr(self.region, "to_dict") else self.region,
        )
        for key in ("max_depth_mm", "retract_z_mm", "feed_z_mm"):
            if result[key] is None:
                result.pop(key)
        return result


@dataclass(frozen=True)
class Lead:
    """Thermal cut lead: a straight part of ``length_mm`` and a tangent arc
    of ``radius_mm`` (either may be zero), always on the scrap side."""

    length_mm: float = 0
    radius_mm: float = 0


@dataclass(frozen=True)
class SmallHoles:
    """Small holes in thermal cutting: cut at ``feed_percent`` of the feed and,
    by default, with torch height control off. ``max_diameter_mm`` defaults
    to 1.5 x the sheet thickness (Hypertherm's definition)."""

    feed_percent: float = 60
    max_diameter_mm: float | None = None
    height_control_off: bool = True


@dataclass(frozen=True)
class MicroJoints:
    """Uncut bridges of ``width_mm`` that keep a cut part in the sheet:
    ``count`` per outline (or one per ``spacing_mm``); ``holes=True`` puts
    them on holes too. The beam turns off over each and fires again."""

    width_mm: float
    count: int | None = None
    spacing_mm: float | None = None
    holes: bool = False


@dataclass(frozen=True)
class ThermalCut:
    """Plasma or laser profile cutting of sheet parts (``parts``: a
    PlanarRegion whose components are the parts to keep, each with holes)
    with a ``Torch`` or ``LaserHead`` (its diameter is the kerf).

    Each loop is cut once with the beam center half a kerf outside the part:
    holes before their part's outline, parts nested in another part's hole
    before that hole. It is entered from a pierce point in the scrap over the
    ``lead_in`` and left over the ``lead_out``; ``overcut_mm`` continues past
    the start (negative: stops short, for holes). Every lead and kerf path is
    proven clear of every part and each pierce point keeps
    ``pierce_clearance_mm`` (default one kerf) from them; in a hole that is
    too small for the leads the pierce moves to the hole center.

    ``height_mode="fixed"`` (default) keeps one Z (the controller handles
    pierce and cut height); ``"program"`` moves to ``travel_height_mm``,
    ``pierce_height_mm`` and ``cut_height_mm`` above the sheet top itself.
    ``height_control`` turns torch height control on for each loop (not over
    leads, nor in ``small_holes``). ``compensation="wear"`` posts G41/G42 so
    the kerf can be trimmed at the controller."""

    id: str
    tool: object
    parts: object
    feed_mm_min: float
    cut_height_mm: float
    lead_in: Lead = Lead(2, 2)
    lead_out: Lead = Lead(1, 1)
    height_mode: str = "fixed"
    pierce_height_mm: float | None = None
    travel_height_mm: float | None = None
    plunge_mm_min: float | None = None
    pierce_delay_s: float = field(kw_only=True)
    power_percent: float = field(kw_only=True)
    overcut_mm: float = 0
    pierce_clearance_mm: float | None = None
    height_control: bool = False
    small_holes: SmallHoles | None = None
    micro_joints: MicroJoints | None = None
    reverse: bool = False
    compensation: str = "computer"
    boundary_tolerance_mm: float = 0.01
    tolerance_mm: float = 1e-5
    work_offset: int = 1
    tool_number: int = field(kw_only=True)
    length_register: int = field(kw_only=True)

    def to_dict(self):
        result = {field: getattr(self, field) for field in self.__dataclass_fields__}
        result.update(
            kind="thermal_cut",
            tool=getattr(self.tool, "id", self.tool),
            parts=self.parts.to_dict() if hasattr(self.parts, "to_dict") else self.parts,
            lead_in=dict(length_mm=self.lead_in.length_mm, radius_mm=self.lead_in.radius_mm),
            lead_out=dict(length_mm=self.lead_out.length_mm, radius_mm=self.lead_out.radius_mm),
        )
        if self.small_holes is not None:
            h = self.small_holes
            result["small_holes"] = dict(
                feed_percent=h.feed_percent,
                height_control_off=h.height_control_off,
                **({} if h.max_diameter_mm is None else dict(max_diameter_mm=h.max_diameter_mm)),
            )
        if self.micro_joints is not None:
            m = self.micro_joints
            result["micro_joints"] = {
                k: v
                for k, v in dict(width_mm=m.width_mm, count=m.count, spacing_mm=m.spacing_mm, holes=m.holes).items()
                if v is not None
            }
        for key in (
            "pierce_height_mm",
            "travel_height_mm",
            "plunge_mm_min",
            "pierce_clearance_mm",
            "small_holes",
            "micro_joints",
        ):
            if result[key] is None:
                result.pop(key)
        return result


@dataclass(frozen=True)
class Trace:
    """Trace/engrave: the tool center follows paths at depth, no radius offset.

    Depth passes run from ``top_z_mm`` (at most the stock top) down to the
    tool tip at ``depth_z_mm``. Use a chamfer (V) tool for engraving, or an end
    mill for slots along a path. Heights as for the other operations.
    """

    id: str
    tool: object
    paths: tuple
    top_z_mm: float
    depth_z_mm: float
    stepdown_mm: float
    clearance_z_mm: float
    feed_mm_min: float = field(kw_only=True)
    plunge_mm_min: float = field(kw_only=True)
    tolerance_mm: float = 1e-5
    spindle_rpm: float = field(kw_only=True)
    work_offset: int = 1
    tool_number: int = field(kw_only=True)
    length_register: int = field(kw_only=True)
    coolant: str = field(kw_only=True)
    retract_z_mm: float | None = None
    feed_z_mm: float | None = None

    def __post_init__(self):
        object.__setattr__(self, "paths", tuple(self.paths))

    def to_dict(self):
        result = {field: getattr(self, field) for field in self.__dataclass_fields__}
        result.update(
            kind="trace",
            tool=getattr(self.tool, "id", self.tool),
            paths=[path.to_dict() if hasattr(path, "to_dict") else path for path in self.paths],
        )
        for key in ('retract_z_mm', 'feed_z_mm'):
            if result[key] is None:
                result.pop(key)
        return result


@dataclass(frozen=True)
class TrochoidalSlot:
    """Slot of constant width with bounded steady cutting-edge engagement.

    Straight from ``start_mm`` to ``end_mm``, or along ``path_mm`` (pass None
    for both points): an open centreline of lines and arcs, vertices
    ``(x, y, bulge_to_next)`` with bulge = tan(sweep / 4) in [-1, 1] (at most a
    half turn per arc, as in ``camlib.planar.Boundary``) and the last bulge 0.
    Orbits sit at every vertex; the step follows the outside of each bend, and
    an arc bending toward the orbit side needs a radius above the orbit radius
    (width/2 - tool radius), otherwise the slot is refused.

    The plunge and first orbit at each depth use entry_feed_mm_min and are
    explicitly outside the engagement limit. Width must be in (D, 2D].
    boundary_tolerance_mm bounds side underclearance, not stock-grid resolution.
    """

    id: str
    tool: object
    start_mm: tuple
    end_mm: tuple
    width_mm: float
    top_z_mm: float
    floor_z_mm: float
    stepdown_mm: float
    clearance_z_mm: float
    feed_mm_min: float = field(kw_only=True)
    entry_feed_mm_min: float = field(kw_only=True)
    max_engagement_deg: float = 90
    boundary_tolerance_mm: float = 0.01
    tolerance_mm: float = 1e-5
    clockwise: bool = True
    spindle_rpm: float = field(kw_only=True)
    work_offset: int = 1
    tool_number: int = field(kw_only=True)
    length_register: int = field(kw_only=True)
    coolant: str = field(kw_only=True)

    # Heights, as in commercial CAM: rapid travel between passes at
    # retract_z_mm, rapid descent to feed_z_mm, feed below it. None keeps
    # clearance_z_mm for both. Required: stock top < feed <= retract <= clearance.
    retract_z_mm: float | None = None
    feed_z_mm: float | None = None
    path_mm: tuple | None = field(default=None, kw_only=True)

    def __post_init__(self):
        if self.path_mm is not None:
            object.__setattr__(self, "path_mm", tuple(tuple(float(v) for v in vertex) for vertex in self.path_mm))
            if any(len(vertex) != 3 for vertex in self.path_mm):
                raise ValueError("TrochoidalSlot path_mm vertices are (x, y, bulge_to_next)")
        if (self.path_mm is None) == (self.start_mm is None and self.end_mm is None) or (
            (self.start_mm is None) != (self.end_mm is None)
        ):
            raise ValueError(
                "TrochoidalSlot needs either start_mm and end_mm (straight) or path_mm (and None for both)"
            )

    def to_dict(self):
        result = {field: getattr(self, field) for field in self.__dataclass_fields__}
        result.update(kind="trochoidal_slot", tool=getattr(self.tool, "id", self.tool))
        for key in ('retract_z_mm', 'feed_z_mm', 'start_mm', 'end_mm', 'path_mm'):
            if result[key] is None:
                result.pop(key)
        if "path_mm" in result:
            result["path_mm"] = [list(vertex) for vertex in result["path_mm"]]
        return result
