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
    # Pass direction: False (default) alternates the rows (zigzag); True
    # cuts every row in the same direction along the axis (from the low end,
    # or the high end with reverse) and returns at retract_z_mm with a rapid
    # link, for a consistent climb or conventional cut (docs/operations.md).
    one_way: bool = False

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
        if result['one_way'] is False:
            result.pop('one_way')
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
class Geodesic:
    """Ball-end geodesic finishing (#12): passes at a constant stepover
    measured along the part's surface from ``seed``, so the scallop stays
    even on steep and shallow areas. ``seed``: ``"boundary"`` (passes work
    inward from the drive surface's boundary), ``("point", (x, y))`` (rings
    around a point, the first half a stepover out) or
    ``("curve", ((x, y), ...))`` (the curve itself is the first pass; the
    others follow it along the surface).

    The passes are exact iso-curves of the geodesic distance, every point is
    certified within half a stepover (plus the path error) of a pass, and
    the spacing between neighbouring passes is measured independently from
    fresh ball contacts; all in the operation report (``geodesic``). Every
    chord is lifted onto the surface and checked like Parallel.

    ``mode``: ``"contact"`` (default: the stepover measured along the part's
    surface) or ``"tool_centre"`` (measured along the surface the ball's
    centre moves on: smooth through internal corners and wall tops, but the
    contact spacing then varies with the curvature; both spacings are
    measured and reported).

    ``slope_to_deg`` defaults to 80 here: steeper faces (where the ball would
    touch near its equator and chatter along a wall) are left to Waterline.
    The passes are planned within the slope range."""

    id: str
    tool: object
    surface: object
    floor_z_mm: float
    stepover_mm: float
    clearance_z_mm: float
    seed: object
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
    slope_to_deg: float = 80
    rest_threshold_mm: float | None = None
    retract_z_mm: float | None = None
    feed_z_mm: float | None = None
    mode: str = "contact"

    def __post_init__(self):
        _geodesic_seed(self.seed)
        if self.mode not in ("contact", "tool_centre"):
            raise ValueError('Geodesic mode must be "contact" or "tool_centre"')

    def to_dict(self):
        result = _pattern_dict(self, "geodesic", ())
        result.pop("seed")
        result.pop("mode")
        result["pattern"]["seed"] = _geodesic_seed(self.seed)
        if self.mode == "tool_centre":
            result["pattern"]["tool_centre"] = True
        return result


def _geodesic_seed(seed):
    if seed == "boundary":
        return {"kind": "boundary"}
    if isinstance(seed, tuple) and len(seed) == 2 and seed[0] == "point" and len(seed[1]) == 2:
        x, y = seed[1]
        return {"kind": "point", "xy_mm": [float(x), float(y)]}
    if (
        isinstance(seed, tuple)
        and len(seed) == 2
        and seed[0] == "curve"
        and len(seed[1]) >= 2
        and all(len(p) == 2 for p in seed[1])
    ):
        return {"kind": "curve", "points_mm": [[float(x), float(y)] for x, y in seed[1]]}
    raise ValueError(
        'Geodesic seed must be "boundary", ("point", (x, y)) or ("curve", ((x, y), ...)) with two or more points'
    )


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
    the part under the tool's swept disk is proven not above it. A run with no
    stock above its height is entered with a vertical plunge. A run with
    stock above it is refused unless ``entry=RampEntry(angle_deg, ...)`` is
    declared: the tool then plunges to the stock's top over the whole run and
    ramps down along the run (Z-flat tools are never plunged into stock). Faces
    within ``level_tolerance_mm`` form one level."""

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
    # How a run under stock is entered: RampEntry(angle_deg, ...). None: no
    # stock stands above the runs, and one that does is refused.
    entry: object | None = None

    def to_dict(self):
        result = {field: getattr(self, field) for field in self.__dataclass_fields__}
        result.update(
            kind="flat",
            tool=getattr(self.tool, "id", self.tool),
            surface=self.surface.to_dict() if hasattr(self.surface, "to_dict") else self.surface,
        )
        entry = result.pop("entry")
        if entry is not None:
            result["entry"] = entry.to_dict() if hasattr(entry, "to_dict") else dict(entry)
        for key in ("retract_z_mm", "feed_z_mm"):
            if result[key] is None:
                result.pop(key)
        return result


@dataclass(frozen=True)
class DeburrBrokenEdge:
    """A boundary stretch of a deburr level's faces that is already broken:
    the face meets the next face tangentially (a fillet), so there is no
    sharp edge to break. ``points`` is a polyline in XY within ``error_mm``
    of the edge."""

    points: tuple
    error_mm: float

    def __post_init__(self):
        object.__setattr__(self, "points", tuple((float(x), float(y)) for x, y in self.points))

    def to_dict(self):
        return dict(points=[list(p) for p in self.points], error_mm=float(self.error_mm))


@dataclass(frozen=True)
class DeburrEdge:
    """One convex edge of a part, for ``Deburr(edges=...)``.
    ``camlib.cadquery.deburr_edges`` lists them from a solid.

    ``kind`` is ``"horizontal"``, ``"vertical"`` or ``"sloped"``; ``points``
    a polyline along the edge within ``error_mm`` (the ends of a straight
    edge); ``normals`` its two faces' outward unit normals at its midpoint,
    and ``planar`` whether each face is planar."""

    id: str
    kind: str
    points: tuple
    error_mm: float
    straight: bool
    normals: tuple
    planar: tuple

    def __post_init__(self):
        object.__setattr__(self, "points", tuple(tuple(float(v) for v in p) for p in self.points))
        object.__setattr__(self, "normals", tuple(tuple(float(v) for v in n) for n in self.normals))
        object.__setattr__(self, "planar", tuple(bool(p) for p in self.planar))

    def to_dict(self):
        return dict(
            id=self.id,
            kind=self.kind,
            points=[list(p) for p in self.points],
            error_mm=float(self.error_mm),
            straight=bool(self.straight),
            normals=[list(n) for n in self.normals],
            planar=list(self.planar),
        )


@dataclass(frozen=True)
class DeburrLevel:
    """The upward-facing planar faces of a part at one height ``z_mm``, as
    planar regions (unioned), and the stretches of their boundaries that are
    already broken (``DeburrBrokenEdge``: tangent fillets).
    ``camlib.cadquery.deburr_levels`` makes them from a solid."""

    regions: tuple
    z_mm: float
    broken_edges: tuple = ()

    def __post_init__(self):
        object.__setattr__(self, "regions", tuple(self.regions))
        object.__setattr__(self, "broken_edges", tuple(self.broken_edges))

    def to_dict(self):
        result = dict(
            regions=[r.to_dict() if hasattr(r, "to_dict") else r for r in self.regions], z_mm=float(self.z_mm)
        )
        if self.broken_edges:
            result["broken_edges"] = [e.to_dict() if hasattr(e, "to_dict") else e for e in self.broken_edges]
        return result


@dataclass(frozen=True)
class Deburr:
    """Break the edges of a finished part with a chamfer mill, a ball-end
    mill or a bull-nose mill: the horizontal edges of the ``levels``, and,
    with ``edges`` (``camlib.cadquery.deburr_edges``), every listed convex
    edge accounted for in the report, straight sloped edges broken with a
    ball-end mill.

    Each level's face boundaries are broken ``width_mm`` wide on the face. A
    chamfer mill's cone leaves a chamfer (the chamfer depth down a vertical
    wall); a ball-end or bull-nose mill leaves a concave arc of its corner
    radius with equal legs ``width_mm`` on the face and down a vertical wall.
    With a ball-end or bull-nose mill the size can be given instead as
    ``break_radius_mm``, the fillet the break must stay inside (pass
    ``width_mm=None``). The rest of the part (``surface``, the finished part
    as a ``SurfaceMesh``) is proven untouched for the whole tool assembly; a
    stretch that cannot be proven is not cut and is reported
    (``unreachable_length_mm``; concave stretches, where a wall rises from the
    face, in ``concave_length_mm``). The stock must already be machined to
    the part.
    """

    id: str
    tool: object
    surface: object
    levels: tuple
    width_mm: float | None
    clearance_z_mm: float
    tip_clearance_mm: float = 0.5
    cutting_sense: str = "climb"
    boundary_tolerance_mm: float = 0.01
    tolerance_mm: float = 1e-5
    break_radius_mm: float | None = None
    edges: tuple = ()
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
        object.__setattr__(self, "edges", tuple(self.edges))

    def to_dict(self):
        result = {field: getattr(self, field) for field in self.__dataclass_fields__}
        result.update(
            kind="deburr",
            tool=getattr(self.tool, "id", self.tool),
            surface=self.surface.to_dict() if hasattr(self.surface, "to_dict") else self.surface,
            levels=[level.to_dict() if hasattr(level, "to_dict") else level for level in self.levels],
            edges=[edge.to_dict() if hasattr(edge, "to_dict") else edge for edge in self.edges],
        )
        if not result["edges"]:
            result.pop("edges")
        for key in ("width_mm", "break_radius_mm", "retract_z_mm", "feed_z_mm"):
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

    ``count`` tabs per loop (4 by default), or one every ``spacing_mm`` of
    loop length, or one at each of ``positions_mm``; ``width_mm`` wide along
    the part edge and ``height_mm`` above the floor. ``shape`` is 'triangle'
    (45 degree ramps, the default) or 'rectangle' (straight up and over).
    Passes above the tab top are not changed.

    ``positions_mm`` are (x, y) points on the part boundary, in the region's
    coordinates (as tabs "at points" in commercial CAM). A point must lie
    within ``position_tolerance_mm`` of exactly one selected boundary loop and
    is taken as the nearest point on it; a point farther away is refused, and
    so are tabs that overlap or sit where the tool enters and leaves the loop.
    A loop with no point gets NO tab: a hole's slug, or a part, without a tab
    is cut free."""

    width_mm: float
    height_mm: float
    count: int | None = None
    spacing_mm: float | None = None
    shape: str = "triangle"
    positions_mm: tuple | None = None
    position_tolerance_mm: float = 0.01

    def __post_init__(self):
        if self.positions_mm is not None:
            if self.count is not None or self.spacing_mm is not None:
                raise ValueError("give tabs either positions_mm or count/spacing_mm, not both")
            points = tuple(tuple(float(n) for n in p) for p in self.positions_mm)
            if not points or any(len(p) != 2 for p in points):
                raise ValueError("tab positions_mm must be a non-empty sequence of (x, y) points")
            object.__setattr__(self, "positions_mm", points)
        elif self.count is None and self.spacing_mm is None:
            object.__setattr__(self, "count", 4)
        elif self.count is not None and self.spacing_mm is not None:
            raise ValueError("give tabs either count or spacing_mm")
        if self.shape not in ("triangle", "rectangle"):
            raise ValueError("tab shape must be 'triangle' or 'rectangle'")

    def to_dict(self):
        result = {"width_mm": self.width_mm, "height_mm": self.height_mm, "shape": self.shape}
        if self.positions_mm is not None:
            result["positions_mm"] = [list(p) for p in self.positions_mm]
            result["position_tolerance_mm"] = self.position_tolerance_mm
        elif self.count is not None:
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
    # None: a pass per level, plunging at its start. RampEntry(angle_deg):
    # each loop is one continuous descent around it, then a flat lap at depth.
    entry: object | None = None

    def to_dict(self):
        result = {field: getattr(self, field) for field in self.__dataclass_fields__}
        result.update(
            kind="contour",
            tool=getattr(self.tool, "id", self.tool),
            region=self.region.to_dict() if hasattr(self.region, "to_dict") else self.region,
            reverse=_profile_reverse(self.reverse, result.pop("cutting_sense")),
        )
        entry = result.pop("entry")
        if entry is not None:
            result["entry"] = entry.to_dict() if hasattr(entry, "to_dict") else dict(entry)
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
class RampEntry:
    """Ramp entry: descend along the cutting path at ``angle_deg`` to the plane.

    The tool comes down along the pass it is about to cut, never steeper than
    ``angle_deg``, at the entry feed (``feed_mm_min``, default the operation's
    plunge feed); the pass is then cut at full depth, which removes the sloped
    floor. ``angle_deg`` is the largest ramp angle for the tool and material:
    it has no default.

    Pocket: along the pass, one leg at the angle when the pass is long enough,
    back and forth over it when it is shorter than depth / tan(angle).
    ``min_length_mm`` is the shortest span the tool may reverse a ramp on. It
    has no default either: a ramp that has to reverse is refused until it is
    set, and refused on a pass shorter than it. Contour: one continuous descent
    around each closed loop from the stock top to the floor (a profile ramp),
    no lap deeper than the stepdown, then one lap flat at the floor.

    A ramp entry does not lift the need for a center-cutting cutter: every
    ramp ends by turning back along the slope it has just cut, which a
    cutter that is not center cutting cannot do without a lift camlib has no
    data for.

    RestPocket: each run of leftover pieces is entered along itself, from the
    lowest height the committed stock proves clear along that path (usually
    a zigzag: leftover pieces are short, so set ``min_length_mm``).

    AdaptivePocket and Roughing with AdaptiveClearing (engagement_front):
    entries from air or cleared stock come first; enclosed material is
    entered by a ramp along the longest straight path through its most open
    point (no longer than one leg needs), from the material top the stock
    proves, then one flat pass along it. The ramp and that pass are entry
    moves at the entry feed, a full-width slot like a helix's bore, and are
    not bounded by the engagement limit; every cutting pass after them is.
    Roughing with offset or raster clearing, and the disk_front and
    stock_front strategies, refuse a ramp entry.
    """

    angle_deg: float
    min_length_mm: float | None = None
    feed_mm_min: float | None = None

    def to_dict(self):
        import math

        result = dict(kind="ramp", angle_rad=math.radians(self.angle_deg))
        for key in ("min_length_mm", "feed_mm_min"):
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
    """Engagement-limited roughing: the cutter never takes more than
    max_radial_engagement_mm of radial width.

    The default path_strategy='engagement_front' follows the boundary
    between cut and uncut stock, entering from air or proven-cleared stock
    and helixing only into enclosed material. Every cutting and stay-down
    link stroke is proven exactly within the contact angle of that width,
    acos(1 - w/r) (the layer report's proven_max_engagement_deg). Orbit
    options do not apply to it.

    path_strategy='stock_front' is a development strategy: it can stream a
    productive prefix but raises if coverage/steering is unresolved. Its
    cutting_sense is 'climb' (default) or 'conventional' with the CW spindle.

    path_strategy='disk_front' (orbits over a whole region) is refused by
    Roughing: it does not read the stock, so around a part it orbits through
    the air beside the stock (#167). It remains AdaptivePocket's strategy.
    """

    max_radial_engagement_mm: float
    orbit_radius_mm: float | None = None
    min_orbit_radius_mm: float | None = None
    local_orbit_tolerance_mm: float | None = None
    boundary_tolerance_mm: float = 0.01

    path_strategy: str = "engagement_front"
    cutting_sense: str | None = None

    def to_dict(self):
        # The strategy is always written: a saved program states which
        # planner made it.
        result = dict(kind="adaptive", **{k: getattr(self, k) for k in self.__dataclass_fields__})
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
    # RampEntry: with AdaptiveClearing (engagement_front) only.
    entry: StockEdgeEntry | HelicalEntry | RampEntry = StockEdgeEntry()
    clearing_tolerance_mm: float | None = None
    require_part_at_first_depth: bool = False
    omit_empty_cuts: bool = False
    rest_tolerance_mm: float | None = None
    # See Pocket.retract_z_mm/feed_z_mm. Raster, offset and engagement_front.
    retract_z_mm: float | None = None
    feed_z_mm: float | None = None
    # Each layer's protected shadow simplified within this and grown by it
    # (still containing the exact shadow), so a fine mesh adds no moves.
    # None: a quarter of radial_allowance_mm; 0: off. At most the allowance;
    # leaves at most twice this more stock.
    contour_tolerance_mm: float | None = None
    # Step-up layers: after each full stepdown, climb back up through its depth
    # at most this far apart, cutting only a band next to the part.
    stepup_mm: float | None = None
    # engagement_front only: a descent into a column the committed sweeps
    # prove clear for the whole tool is rapid down to this far above the
    # proven height, then fed. None: rapid descent stops at feed_z_mm.
    rapid_descent_clearance_mm: float | None = None
    # engagement_front only: a rapid between passes travels this far above
    # the whole-cell stock bounds any tool section can meet over the
    # rectangle its two ends span (straight route and both axis-first
    # doglegs), when that is below retract_z_mm. Stock only: fixtures are
    # not seen by the planner (the run's verification still checks them).
    # None: rapids travel at retract_z_mm.
    travel_clearance_mm: float | None = None
    # engagement_front only: each helical entry starts this far above the
    # material its bore meets (never above feed_z_mm), so it is already
    # ramping when it reaches stock. None: the helix starts at that material.
    ramp_clearance_mm: float | None = None
    # engagement_front only: stay-down links at depth feed at this rate
    # (mm/min) where the committed stock bounds prove the cutter clear of
    # stock; elsewhere links keep feed_mm_min. None: links feed at feed_mm_min.
    link_feed_mm_min: float | None = None
    # engagement_front only, with link_feed_mm_min: a link stretch that may
    # touch stock feeds faster than feed_mm_min by radial chip thinning, from
    # a proven bound on how close stock lies to its line, so no chip exceeds
    # the one at feed_mm_min and the clearing's radial engagement; capped at
    # link_feed_mm_min.
    link_chip_thinning: bool = False

    @classmethod
    def from_solid(cls, id, tool, solid, *, source=None, **parameters):
        from .cadquery import export_surface_mesh

        return cls(id, tool, export_surface_mesh(solid, source=source), **parameters)

    def to_dict(self):
        result = {key: getattr(self, key) for key in self.__dataclass_fields__}
        result.update(kind="roughing", tool=getattr(self.tool, "id", self.tool))
        if self.contour_tolerance_mm is None:
            result.pop('contour_tolerance_mm')
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
        for key in (
            'retract_z_mm',
            'feed_z_mm',
            'stepup_mm',
            'rapid_descent_clearance_mm',
            'travel_clearance_mm',
            'ramp_clearance_mm',
            'link_feed_mm_min',
        ):
            if result[key] is None:
                result.pop(key)
        if not self.link_chip_thinning:
            result.pop('link_chip_thinning')
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
    entry: HelicalEntry | StockEdgeEntry | RampEntry | None = None
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
    """A pocket cleared with a bounded radial engagement.

    The default path_strategy='engagement_front' follows the boundary between
    cut and uncut stock inside the region, one depth level at a time (#190).
    stepover_mm is the radial width of each pass (at most the cutter radius);
    max_engagement_deg below acos(1 - stepover/r) narrows it further. Each
    level enters by a helix into uncut material (entry's helix, or
    HelicalEntry's default), or with entry=RampEntry(angle_deg) by a ramp
    along a straight path (for a pocket too narrow for a helix), and every
    cutting stroke is proven within that
    contact angle against the committed cleared sweeps. Nothing is cut twice.
    An area of the region no entry reaches refuses the pocket (#250), as it
    does a Pocket: where the cutter fits but no helix does and nothing there
    is clear of material to enter from (a slot barely wider than the
    cutter), the run raises when the level starts, before its first motion,
    with the place, the area's extent and the options (a RampEntry, a
    smaller cutter, an earlier operation that opens a start there). So does
    an area that leaves the cutter less play than this planner works in
    (about 0.19 mm a side with a 4 mm cutter; use a Pocket there), and a
    region the cutter fits nowhere in. Each level's report counts all the
    stock left in the region as its residual area.
    Orbit options, max_stay_down_mm and rest_tolerance_mm do not apply; the
    direction is cutting_sense ('climb' by default, or 'conventional').

    engagement_front's entry and link options are the ones Roughing's
    adaptive clearing has. All are unset by default. The link options change
    feeds only; the entry options change descents, never a cutting pass:

    - link_feed_mm_min: stay-down links at depth (the returns between passes)
      feed at this rate wherever the committed stock bounds prove the whole
      cutter footprint clear of stock; every other link stretch keeps
      feed_mm_min. None: every link feeds at feed_mm_min.
    - link_chip_thinning (with link_feed_mm_min): a link stretch that may
      touch stock feeds faster than feed_mm_min by radial chip thinning, from
      a proven bound on how close stock lies to its line, capped at
      link_feed_mm_min.
    - rapid_descent_clearance_mm: a descent into a column the committed cuts
      prove clear for the whole tool (a level below one already cut) is rapid
      down to this far above the proven height, then fed; an entry beside
      the material it cuts descends a few stock cells back and feeds across at
      depth. None: rapid descent stops at feed_z_mm.
    - ramp_clearance_mm: each helical entry starts this far above the
      material its bore meets (never above feed_z_mm). None: at that material.

    path_strategy='disk_front' is the earlier planner: it covers the region
    with orbits of orbit_radius_mm (None: half the cutter radius) whatever the
    stock holds, so its orbits overlap and it cuts far longer. With it,
    clockwise sets the orbit direction; set min_orbit_radius_mm below
    orbit_radius_mm and local_orbit_tolerance_mm (for example .05 mm) to size
    orbits from whole-guide clearance. path_strategy='stock_front' is a
    development strategy that raises on unresolved steering/coverage. Both
    refuse the four options above, and an area of the region the cutter
    fits in that no orbit does (cutter radius plus orbit radius).
    """

    orbit_radius_mm: float | None = None
    max_engagement_deg: float = 90
    boundary_tolerance_mm: float = 0.01
    clockwise: bool = True
    min_orbit_radius_mm: float | None = None
    local_orbit_tolerance_mm: float | None = None

    path_strategy: str = "engagement_front"
    cutting_sense: str | None = None
    # engagement_front only (see the class docstring); as on Roughing.
    rapid_descent_clearance_mm: float | None = None
    ramp_clearance_mm: float | None = None
    link_feed_mm_min: float | None = None
    link_chip_thinning: bool = False

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
        # The strategy is always written: a saved program states which planner made it.
        result["path_strategy"] = self.path_strategy
        if self.cutting_sense is not None:
            result["cutting_sense"] = self.cutting_sense
        if self.orbit_radius_mm is not None:
            result["orbit_radius_mm"] = self.orbit_radius_mm
        if self.min_orbit_radius_mm is not None:
            result["min_orbit_radius_mm"] = self.min_orbit_radius_mm
        if self.local_orbit_tolerance_mm is not None:
            result["local_orbit_tolerance_mm"] = self.local_orbit_tolerance_mm
        # Written only when set: a job without them is the job it was.
        for key in ("rapid_descent_clearance_mm", "ramp_clearance_mm", "link_feed_mm_min"):
            if getattr(self, key) is not None:
                result[key] = getattr(self, key)
        if type(self.link_chip_thinning) is not bool:
            raise ValueError("link_chip_thinning must be a boolean")
        if self.link_chip_thinning:
            result["link_chip_thinning"] = True
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
class CommonLine:
    """Common-line cutting for ``ThermalCut``: two parts whose facing straight
    edges are exactly ``ThermalCut.common_line_spacing_mm()`` apart (one kerf
    path between them) share a single cut along that edge. The shared edge
    must be a complete edge of both parts, at least ``min_length_mm`` long,
    with the parts flush at both ends (pairs and rows)."""

    min_length_mm: float


@dataclass(frozen=True)
class SpreadHeat:
    """A ``ThermalCut`` part order that spreads the heat: after a part, the
    nearest part whose outline starts at least ``min_distance_mm`` away is cut
    next (the farthest one when every remaining part is nearer), so
    consecutive parts are not neighbours. The distance is the shop's choice
    for the plate; there is no default."""

    min_distance_mm: float


@dataclass(frozen=True)
class AvoidCutParts:
    """Keep a ``ThermalCut``'s rapids ``clearance_mm`` from every part and
    slug already cut free (they can tip up between the table's slats): the
    travel goes round them, and a rapid with no way round is refused by name.
    The clearance is measured from the beam centre, so include the torch
    body's radius; there is no default. Pieces held by micro-joints are not
    counted as free."""

    clearance_mm: float


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
    the kerf can be trimmed at the controller.

    ``common_line=CommonLine(min_length_mm)`` cuts the shared straight edge
    of parts placed ``common_line_spacing_mm()`` apart once: holes first,
    then each common line (pierced in the scrap on its extension), then the
    one outline of the group. Parts whose kerf paths coincide in any other
    way are refused; the report lists the ``common_lines``.

    ``part_order`` chooses which part is cut next: ``"nearest"`` (default: the
    shortest travel), ``"given"`` (the order of ``parts``) or
    ``SpreadHeat(min_distance_mm)`` (consecutive parts kept apart, against
    heat building up in one place). In every order a part nested in another
    part's hole is cut before that part and a part's holes before its
    outline; the report gives the ``order``, the ``min_part_step_mm`` and the
    ``rapid_length_mm``.

    ``avoid_cut_parts=AvoidCutParts(clearance_mm)`` keeps every rapid that
    distance from the pieces already cut free; the report counts the
    ``rerouted_rapids``. It is not planned together with ``common_line``."""

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
    common_line: CommonLine | None = field(default=None, kw_only=True)
    part_order: object = field(default="nearest", kw_only=True)
    avoid_cut_parts: AvoidCutParts | None = field(default=None, kw_only=True)

    def common_line_spacing_mm(self):
        """The gap between two parts whose facing edges are a common line:
        twice the applied offset, so that the one kerf path between them is
        the compensated path of both (the kerf width plus the boundary
        allowance the cut leaves on each part)."""
        return self.tool.cutter.diameter_mm + 2 * self.parts.boundary_error_mm + self.boundary_tolerance_mm

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
        if self.common_line is not None:
            result["common_line"] = dict(min_length_mm=self.common_line.min_length_mm)
        if self.avoid_cut_parts is None:
            result.pop("avoid_cut_parts")
        else:
            result["avoid_cut_parts"] = dict(clearance_mm=self.avoid_cut_parts.clearance_mm)
        order = self.part_order
        if isinstance(order, SpreadHeat):
            result["part_order"] = dict(kind="spread", min_distance_mm=order.min_distance_mm)
        elif order in ("nearest", "given"):
            result["part_order"] = dict(kind=order)
        else:
            raise ValueError(f"ThermalCut part_order {order!r}: use 'nearest', 'given' or SpreadHeat(min_distance_mm)")
        for key in (
            "pierce_height_mm",
            "travel_height_mm",
            "plunge_mm_min",
            "pierce_clearance_mm",
            "small_holes",
            "micro_joints",
            "common_line",
        ):
            if result[key] is None:
                result.pop(key)
        return result


@dataclass(frozen=True)
class ThermalMark:
    """Plasma or laser marking: lines traced on the sheet surface with the
    beam in the machine's marking process, without cutting through (part
    numbers, bend lines, layout marks). The beam centre follows ``paths``
    (``TracePath`` values, in the order given) with no kerf compensation,
    one beam-on per path.

    Nothing is assumed about the machine: ``feed_mm_min``,
    ``mark_height_mm`` (nozzle above the sheet top), ``power_percent`` (a
    laser's marking power; a plasma torch marks at the current its
    controller's marking process sets) and ``mark_width_mm`` (the width of
    the marked line on this material) are the caller's, and the machine must
    declare how it marks (``GCodeDialect(beam_mark=...)``, or its controller
    post's setting) or the post refuses the program.

    Marking runs before cutting, while each part is still held by the sheet:
    put the ``ThermalMark`` before the ``ThermalCut``. ``parts`` is the
    cut's ``parts``. Every mark's width, with ``edge_clearance_mm`` around
    it, is proven inside one part (``allow_off_part=True`` skips that proof,
    for marks on the scrap, and the report says so); a part with a mark must
    not have been cut; and a marking move removes no stock and is refused
    over sheet that is cut through.

    ``height_mode="fixed"`` (default) keeps one Z (the controller owns the
    height); ``"program"`` travels between paths at ``travel_height_mm``."""

    id: str
    tool: object
    parts: object
    paths: tuple
    feed_mm_min: float
    mark_height_mm: float
    height_mode: str = "fixed"
    travel_height_mm: float | None = None
    power_percent: float = field(kw_only=True)
    mark_width_mm: float = field(kw_only=True)
    edge_clearance_mm: float = 0
    allow_off_part: bool = False
    tolerance_mm: float = 1e-5
    work_offset: int = 1
    tool_number: int = field(kw_only=True)
    length_register: int = field(kw_only=True)

    def __post_init__(self):
        object.__setattr__(self, "paths", tuple(self.paths))

    def to_dict(self):
        result = {field: getattr(self, field) for field in self.__dataclass_fields__}
        result.update(
            kind="thermal_mark",
            tool=getattr(self.tool, "id", self.tool),
            parts=self.parts.to_dict() if hasattr(self.parts, "to_dict") else self.parts,
            paths=[path.to_dict() if hasattr(path, "to_dict") else path for path in self.paths],
        )
        if result["travel_height_mm"] is None:
            result.pop("travel_height_mm")
        return result


@dataclass(frozen=True)
class TraceSideOffset:
    """Keep the cutter on one side of a Trace's open paths: the tool centre
    runs at the cutter radius plus ``radial_allowance_mm`` to the ``side``
    ("left" or "right") of the direction of travel."""

    side: str
    radial_allowance_mm: float = 0.0

    def __post_init__(self):
        if self.side not in ("left", "right"):
            raise ValueError(f"side must be 'left' or 'right', not {self.side!r}")

    def to_dict(self):
        return {"side": self.side, "radial_allowance_mm": float(self.radial_allowance_mm)}


@dataclass(frozen=True)
class Trace:
    """Trace/engrave: the tool center follows paths at depth, no radius offset.

    Depth passes run from ``top_z_mm`` (at most the stock top) down to the
    tool tip at ``depth_z_mm``. Use a chamfer (V) tool for engraving, or an end
    mill for slots along a path. Heights as for the other operations.

    With ``side_offset`` (a ``TraceSideOffset``) a flat end mill runs beside
    each open path instead, on the given side of the direction of travel:
    a contour along an open chain of edges (an open edge, a step).
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
    side_offset: object | None = None

    def __post_init__(self):
        object.__setattr__(self, "paths", tuple(self.paths))

    def to_dict(self):
        result = {field: getattr(self, field) for field in self.__dataclass_fields__}
        result.update(
            kind="trace",
            tool=getattr(self.tool, "id", self.tool),
            paths=[path.to_dict() if hasattr(path, "to_dict") else path for path in self.paths],
        )
        for key in ('retract_z_mm', 'feed_z_mm', 'side_offset'):
            if result[key] is None:
                result.pop(key)
        if hasattr(self.side_offset, "to_dict"):
            result["side_offset"] = self.side_offset.to_dict()
        return result


@dataclass(frozen=True)
class Undercut:
    """Undercut milling with a T-slot, keyseat, lollipop or dovetail cutter:
    the tool centre follows ``paths`` (``TracePath``) at the tip height
    ``tip_z_mm``, under material left by earlier operations.

    The tool goes down at ``entry_mm`` and back up at ``exit_mm`` (default:
    back along each open path to its start, then to the entry): columns that
    must be open for the whole tool, outside the part or in a cleared pocket.
    The plan refuses a column it cannot prove open. Needs tri-dexel stock
    (``DexelStockSpec``): its sweep refuses a move whose neck, shank or
    holder may touch material.
    """

    id: str
    tool: object
    paths: tuple
    tip_z_mm: float
    entry_mm: tuple
    clearance_z_mm: float
    exit_mm: tuple | None = None
    feed_mm_min: float = field(kw_only=True)
    plunge_mm_min: float = field(kw_only=True)
    tolerance_mm: float = 1e-5
    spindle_rpm: float = field(kw_only=True)
    work_offset: int = 1
    tool_number: int = field(kw_only=True)
    length_register: int = field(kw_only=True)
    coolant: str = field(kw_only=True)

    def __post_init__(self):
        object.__setattr__(self, "paths", tuple(self.paths))

    def to_dict(self):
        result = {field: getattr(self, field) for field in self.__dataclass_fields__}
        result.update(
            kind="undercut",
            tool=getattr(self.tool, "id", self.tool),
            paths=[path.to_dict() if hasattr(path, "to_dict") else path for path in self.paths],
            entry_mm=[float(v) for v in self.entry_mm],
        )
        if self.exit_mm is None:
            result.pop("exit_mm")
        else:
            result["exit_mm"] = [float(v) for v in self.exit_mm]
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


# Probing and tool setting (issue #7). camlib plans, simulates and posts the
# cycles; it never reads results back from a machine.


def _probe_point(value, n, what):
    point = tuple(float(v) for v in value)
    if len(point) != n or not all(math.isfinite(v) for v in point):
        raise ValueError(f"{what} needs {n} finite coordinates")
    return point


@dataclass(frozen=True)
class _Probing:
    """Common probing parameters. ``clearance_z_mm`` is the plane rapids
    reach the cycle over and, for stylus cycles, the protected retract
    height. The surface is expected within ``search_mm`` of nominal: the
    measuring move is checked clear until ``search_mm`` before the expected
    contact and searches ``search_mm`` beyond it."""

    id: str
    tool: object
    clearance_z_mm: float = field(kw_only=True)
    search_mm: float = field(kw_only=True)
    measure_feed_mm_min: float = field(kw_only=True)
    return_feed_mm_min: float = field(kw_only=True)
    tool_number: int = field(kw_only=True)
    length_register: int = field(kw_only=True)
    work_offset: int = field(default=1, kw_only=True)
    tolerance_mm: float | None = field(default=None, kw_only=True)

    def _dict(self, feature, result, **extra):
        data = {
            "kind": "probe",
            "id": self.id,
            "tool": getattr(self.tool, "id", self.tool),
            "tool_number": self.tool_number,
            "length_register": self.length_register,
            "work_offset": self.work_offset,
            "clearance_z_mm": self.clearance_z_mm,
            "feature": feature,
            "result": result,
            "search_mm": self.search_mm,
            "measure_feed_mm_min": self.measure_feed_mm_min,
            "return_feed_mm_min": self.return_feed_mm_min,
            **extra,
        }
        if self.tolerance_mm is not None:
            data["tolerance_mm"] = self.tolerance_mm
        return data


@dataclass(frozen=True)
class _StylusProbing(_Probing):
    """A cycle with a probe (``camlib.ProbeTool``) that corrects work offset
    ``update_work_offset`` (1 = G54 .. 9) by the measured error: new offset =
    active offset + error, in the axes the feature measures. Every move below
    the clearance plane that is not a measuring move is protected: the
    machine stops if the stylus triggers."""

    position_feed_mm_min: float = field(kw_only=True)
    update_work_offset: int = field(kw_only=True)

    def _stylus(self, feature):
        return self._dict(
            feature,
            {"kind": "work_offset", "number": self.update_work_offset},
            position_feed_mm_min=self.position_feed_mm_min,
        )


def _axis(axis, allowed):
    if axis not in allowed:
        raise ValueError(f"axis must be one of {allowed}")
    return axis


@dataclass(frozen=True)
class ProbeSurface(_StylusProbing):
    """One surface square to ``axis`` ('x', 'y' or 'z') at ``surface_mm``,
    measured from ``start_mm`` (the probe tip) along that axis toward it and
    back (Renishaw Inspection Plus O9811). A Z surface is probed from above."""

    start_mm: tuple = field(kw_only=True)
    axis: str = field(kw_only=True)
    surface_mm: float = field(kw_only=True)

    def __post_init__(self):
        object.__setattr__(self, "start_mm", _probe_point(self.start_mm, 3, "start_mm"))
        _axis(self.axis, ("x", "y", "z"))

    def to_dict(self):
        return self._stylus(
            {"kind": "surface", "start_mm": self.start_mm, "axis": self.axis, "surface_mm": self.surface_mm}
        )


@dataclass(frozen=True)
class ProbeOutsideCorner(_StylusProbing):
    """An outside corner at ``corner_mm``; the part's material lies on the
    ``material`` side ((+1 or -1, +1 or -1): +X and +Y of the corner for
    (1, 1)). Each face is touched ``inset_mm`` along it from the corner,
    from ``clearance_mm`` away, the probe tip at ``z_mm``: the Y face, then
    the X face, each from the clearance plane."""

    corner_mm: tuple = field(kw_only=True)
    z_mm: float = field(kw_only=True)
    material: tuple = field(kw_only=True)
    inset_mm: float = field(kw_only=True)
    clearance_mm: float = field(kw_only=True)

    def __post_init__(self):
        object.__setattr__(self, "corner_mm", _probe_point(self.corner_mm, 2, "corner_mm"))
        object.__setattr__(self, "material", tuple(int(v) for v in self.material))

    def to_dict(self):
        return self._stylus(
            {
                "kind": "outside_corner",
                "corner_mm": self.corner_mm,
                "z_mm": self.z_mm,
                "material": self.material,
                "inset_mm": self.inset_mm,
                "clearance_mm": self.clearance_mm,
            }
        )


@dataclass(frozen=True)
class ProbeInsideCorner(_StylusProbing):
    """An inside corner at ``corner_mm``; the walls lie on the ``material``
    side of the start point, which is ``clearance_mm`` from both walls. The
    Y wall, then the X wall, from that one start point and back (Renishaw
    Inspection Plus O9815 without I/J)."""

    corner_mm: tuple = field(kw_only=True)
    z_mm: float = field(kw_only=True)
    material: tuple = field(kw_only=True)
    clearance_mm: float = field(kw_only=True)

    def __post_init__(self):
        object.__setattr__(self, "corner_mm", _probe_point(self.corner_mm, 2, "corner_mm"))
        object.__setattr__(self, "material", tuple(int(v) for v in self.material))

    def to_dict(self):
        return self._stylus(
            {
                "kind": "inside_corner",
                "corner_mm": self.corner_mm,
                "z_mm": self.z_mm,
                "material": self.material,
                "clearance_mm": self.clearance_mm,
            }
        )


@dataclass(frozen=True)
class ProbeBore(_StylusProbing):
    """A bore of ``diameter_mm`` from its centre, the probe tip at ``z_mm``:
    +X, -X, +Y, -Y (Renishaw Inspection Plus O9814). Corrects X and Y."""

    center_mm: tuple = field(kw_only=True)
    z_mm: float = field(kw_only=True)
    diameter_mm: float = field(kw_only=True)

    def __post_init__(self):
        object.__setattr__(self, "center_mm", _probe_point(self.center_mm, 2, "center_mm"))

    def to_dict(self):
        return self._stylus(
            {"kind": "bore", "center_mm": self.center_mm, "z_mm": self.z_mm, "diameter_mm": self.diameter_mm}
        )


@dataclass(frozen=True)
class ProbeBoss(_StylusProbing):
    """A boss of ``diameter_mm`` from outside: +X, -X, +Y, -Y, each from
    ``clearance_mm`` outside it, reached over the clearance plane."""

    center_mm: tuple = field(kw_only=True)
    z_mm: float = field(kw_only=True)
    diameter_mm: float = field(kw_only=True)
    clearance_mm: float = field(kw_only=True)

    def __post_init__(self):
        object.__setattr__(self, "center_mm", _probe_point(self.center_mm, 2, "center_mm"))

    def to_dict(self):
        return self._stylus(
            {
                "kind": "boss",
                "center_mm": self.center_mm,
                "z_mm": self.z_mm,
                "diameter_mm": self.diameter_mm,
                "clearance_mm": self.clearance_mm,
            }
        )


@dataclass(frozen=True)
class ProbePocket(_StylusProbing):
    """A pocket (slot) ``width_mm`` wide across ``axis`` ('x' or 'y'), from
    its centre: + then - (Renishaw Inspection Plus O9812 without Z)."""

    center_mm: tuple = field(kw_only=True)
    z_mm: float = field(kw_only=True)
    axis: str = field(kw_only=True)
    width_mm: float = field(kw_only=True)

    def __post_init__(self):
        object.__setattr__(self, "center_mm", _probe_point(self.center_mm, 2, "center_mm"))
        _axis(self.axis, ("x", "y"))

    def to_dict(self):
        return self._stylus(
            {
                "kind": "pocket",
                "center_mm": self.center_mm,
                "z_mm": self.z_mm,
                "axis": self.axis,
                "width_mm": self.width_mm,
            }
        )


@dataclass(frozen=True)
class ProbeWeb(_StylusProbing):
    """A web (rib) ``width_mm`` wide across ``axis`` ('x' or 'y'), from
    outside: the + side then the - side, each from ``clearance_mm`` outside
    it, reached over the clearance plane."""

    center_mm: tuple = field(kw_only=True)
    z_mm: float = field(kw_only=True)
    axis: str = field(kw_only=True)
    width_mm: float = field(kw_only=True)
    clearance_mm: float = field(kw_only=True)

    def __post_init__(self):
        object.__setattr__(self, "center_mm", _probe_point(self.center_mm, 2, "center_mm"))
        _axis(self.axis, ("x", "y"))

    def to_dict(self):
        return self._stylus(
            {
                "kind": "web",
                "center_mm": self.center_mm,
                "z_mm": self.z_mm,
                "axis": self.axis,
                "width_mm": self.width_mm,
                "clearance_mm": self.clearance_mm,
            }
        )


@dataclass(frozen=True)
class _ToolSetting(_Probing):
    """Tool setting measures the cutting tool ``tool`` on a tool setter
    declared in this setup's work coordinates, and stores the result in the
    tool's ``length_register``. Declare the setter's body as a scene fixture
    to check the moves around it. ``spindle_rpm`` (0: stopped) and
    ``spindle_direction`` are the caller's: camlib assumes neither."""

    spindle_rpm: float = field(kw_only=True)
    spindle_direction: str | None = field(default=None, kw_only=True)

    def _tool(self, feature, result):
        extra = {"spindle_rpm": self.spindle_rpm}
        if self.spindle_direction is not None:
            extra["spindle_direction"] = self.spindle_direction
        return self._dict(feature, {"kind": result, "register": self.length_register}, **extra)


@dataclass(frozen=True)
class ToolSetLength(_ToolSetting):
    """Tool length: the tip comes down onto the setter's contact face at
    ``contact_mm`` (the tip position at contact) from ``clearance_mm`` above
    it. The controller's length register then holds the measured length (a
    premise for the rest of the program). Set tool lengths before probing
    the work offset in the same program: the setter is declared in that
    offset's coordinates."""

    contact_mm: tuple = field(kw_only=True)
    clearance_mm: float = field(kw_only=True)

    def __post_init__(self):
        object.__setattr__(self, "contact_mm", _probe_point(self.contact_mm, 3, "contact_mm"))

    def to_dict(self):
        return self._tool(
            {"kind": "tool_length", "contact_mm": self.contact_mm, "clearance_mm": self.clearance_mm},
            "tool_length",
        )


@dataclass(frozen=True)
class ToolSetDiameter(_ToolSetting):
    """Tool diameter: the cutter's side touches the setter's round contact
    face (``setter_diameter_mm`` across, top at ``setter_top_z_mm``) from +X
    and from -X, the tip ``depth_mm`` below the top, from ``clearance_mm``
    away, crossing over ``clearance_mm`` above the top. The controller's tool
    table entry then holds the measured radius (not wear: camlib's wear
    compensation is refused for this tool afterwards)."""

    setter_center_mm: tuple = field(kw_only=True)
    setter_top_z_mm: float = field(kw_only=True)
    setter_diameter_mm: float = field(kw_only=True)
    depth_mm: float = field(kw_only=True)
    clearance_mm: float = field(kw_only=True)

    def __post_init__(self):
        object.__setattr__(self, "setter_center_mm", _probe_point(self.setter_center_mm, 2, "setter_center_mm"))

    def to_dict(self):
        return self._tool(
            {
                "kind": "tool_diameter",
                "setter_center_mm": self.setter_center_mm,
                "setter_top_z_mm": self.setter_top_z_mm,
                "setter_diameter_mm": self.setter_diameter_mm,
                "depth_mm": self.depth_mm,
                "clearance_mm": self.clearance_mm,
            },
            "tool_radius",
        )


_TOOLPATH_CURVE_FIELDS = {
    "line": ("start", "end"),
    "arc": ("start", "end", "center"),
    "helix": ("start", "end", "center"),
}


@dataclass(frozen=True)
class ToolpathMove:
    """One move of a :class:`SuppliedToolpath`: its role, its path and its feed.

    ``kind`` is "rapid", "cut", "plunge", "lead", "link" or "retract";
    ``curve`` a ``camlib.motion.Line``, ``Arc`` or ``Helix`` in the setup's
    work frame (mm, radians). Every move but a rapid states its
    ``feed_mm_min``; a rapid may state one (a fed traverse) or none. Nothing
    here has a default that stands in for cutting data.
    """

    kind: str
    curve: object
    feed_mm_min: float | None = None

    @classmethod
    def from_motion(cls, motion):
        """The move of a ``camlib.motion.Motion`` or of a streamed motion
        dictionary (``Update.motion``). Its id, operation and tool are left
        behind: the supplied toolpath's own operation assigns them."""
        if isinstance(motion, dict):
            from .motion import Arc, Helix, Line

            motion = motion.get("motion", motion)
            curve = dict(motion["curve"])
            kind = curve.pop("kind")
            if kind not in _TOOLPATH_CURVE_FIELDS:
                raise ValueError(f"unknown curve kind {kind!r}")
            curve = {"line": Line, "arc": Arc, "helix": Helix}[kind](**curve)
            return cls(motion["kind"], curve, motion["feed_mm_min"])
        return cls(motion.kind, motion.curve, motion.feed_mm_min)

    def to_dict(self):
        curve = self.curve.to_dict() if hasattr(self.curve, "to_dict") else dict(self.curve)
        return {"kind": self.kind, "curve": curve, "feed_mm_min": self.feed_mm_min}


def _toolpath_move(id, index, move):
    """The move's manifest dictionary. Only what JSON cannot carry is checked
    here (a number that is not finite), so that the refusal names the move;
    every machining rule is the engine's."""
    if not isinstance(move, ToolpathMove):
        if not (hasattr(move, "curve") and hasattr(move, "kind") and hasattr(move, "feed_mm_min")):
            raise TypeError(
                f"supplied toolpath {id}: move {index}: expected a ToolpathMove or camlib.motion.Motion, "
                f"not {type(move).__name__}"
            )
        move = ToolpathMove.from_motion(move)
    try:
        data = move.to_dict()
        curve = data["curve"]
        fields = _TOOLPATH_CURVE_FIELDS.get(curve.get("kind"))
    except (TypeError, ValueError, AttributeError):
        fields = None
    if fields is None:
        raise ValueError(f"supplied toolpath {id}: move {index}: the curve must be a Line, Arc or Helix")
    values = [data["feed_mm_min"], curve.get("sweep_rad")]
    for name in fields:
        point = tuple(curve[name])
        if len(point) != 3:
            raise ValueError(f"supplied toolpath {id}: move {index}: {name} must be (x, y, z)")
        curve[name] = point
        values.extend(point)
    for value in values:
        if value is None:
            continue
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            raise ValueError(
                f"supplied toolpath {id}: move {index}: {value!r} is not a finite number "
                "(coordinates, sweep and feed must be finite)"
            )
    return data


@dataclass(frozen=True)
class SuppliedToolpath:
    """A toolpath planned elsewhere, run exactly as given.

    ``motions`` (``ToolpathMove`` or ``camlib.motion.Motion``) are the tool
    tip's moves in the setup's work frame. camlib changes nothing in them and
    treats them as its own: they are simulated against the stock the
    operations before left, checked move by move, compared with the target,
    refused at export on unaccepted findings, posted and decoded back. A move
    that cannot be run exactly is refused with its index.

    The toolpath starts and ends at or above ``clearance_z_mm`` (above the
    stock top), where the operation itself starts; camlib adds only the rapid
    travel to its first point. Each move starts exactly where the one before
    ends. Spindle speed, coolant, tool number and length register are
    declared as for any operation. The setup must check stock contact
    (``StreamChecks(stock_contact=StockContactOptions())``): no planner
    guarantee stands behind these moves. Three-axis milling only.
    """

    id: str
    tool: object
    motions: tuple
    clearance_z_mm: float
    spindle_rpm: float = field(kw_only=True)
    tool_number: int = field(kw_only=True)
    length_register: int = field(kw_only=True)
    coolant: str = field(kw_only=True)
    tolerance_mm: float = 1e-5
    work_offset: int = 1

    def __post_init__(self):
        if not isinstance(self.motions, (list, tuple)):
            raise TypeError("supplied toolpath motions must be a list or tuple (a finite collection)")
        # Roughly 1 kB a move, as Python objects and again as its manifest
        # dictionary: refused before it outgrows memory, never truncated.
        moves = bounded_tuple(self.motions, 1000, "supplied toolpath motions")
        object.__setattr__(self, "motions", moves)
        object.__setattr__(
            self,
            "_motion_dicts",
            bounded_tuple(
                (_toolpath_move(self.id, i, move) for i, move in enumerate(moves)),
                1000,
                "supplied toolpath motions",
            ),
        )

    def to_dict(self):
        result = {field: getattr(self, field) for field in self.__dataclass_fields__}
        # Built once (the operation is immutable); a job serialises it for
        # its manifest and its input digest, both in one pass over the moves.
        result.update(
            kind="supplied_toolpath",
            tool=getattr(self.tool, "id", self.tool),
            motions=self._motion_dicts,
        )
        return result
