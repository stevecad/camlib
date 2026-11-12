"""Immutable Python tool definitions, validated/prepared by the Rust engine.

All profile heights are millimetres above the tool tip toward the spindle.
Feeds/speeds belong to the program, not this geometry definition.
"""

from dataclasses import dataclass, asdict
import math


@dataclass(frozen=True)
class Neck:
    diameter_mm: float
    length_mm: float


@dataclass(frozen=True)
class Cutter:
    id: str
    name: str
    shape: tuple
    diameter_mm: float
    cutting_length_mm: float
    overall_length_mm: float
    shank_diameter_mm: float
    flutes: int
    center_cutting: bool
    material: str
    neck: Neck | None = None

    def to_dict(self):
        result = asdict(self)
        result["shape"] = dict(self.shape)
        return result

    @classmethod
    def from_dict(cls, data):
        data = dict(data)
        data["shape"] = tuple(data["shape"].items())
        if data.get("neck") is not None:
            data["neck"] = Neck(**data["neck"])
        return cls(**data)


def _cutter(
    id,
    shape,
    *,
    diameter,
    flute_length,
    overall_length,
    shank_diameter=None,
    flutes=3,
    center_cutting=True,
    material="carbide",
    name=None,
    neck=None,
):
    return Cutter(
        id,
        name or id,
        tuple(shape.items()),
        diameter,
        flute_length,
        overall_length,
        diameter if shank_diameter is None else shank_diameter,
        flutes,
        center_cutting,
        material,
        neck,
    )


def EndMill(id, **dimensions):
    return _cutter(id, {"kind": "flat"}, **dimensions)


def BallEndMill(id, **dimensions):
    return _cutter(id, {"kind": "ball"}, **dimensions)


def BullEndMill(id, *, corner_radius, **dimensions):
    return _cutter(id, {"kind": "bull", "corner_radius_mm": corner_radius}, **dimensions)


def Drill(id, *, point_angle=118, **dimensions):
    dimensions.setdefault("flutes", 2)
    return _cutter(id, {"kind": "drill", "point_angle_rad": math.radians(point_angle)}, **dimensions)


def ChamferMill(id, *, angle=90, tip_diameter=0, **dimensions):
    return _cutter(
        id,
        {"kind": "chamfer", "included_angle_rad": math.radians(angle), "tip_diameter_mm": tip_diameter},
        **dimensions,
    )


def ThreadMill(
    id,
    *,
    diameter,
    tooth_height,
    neck_diameter,
    neck_length,
    overall_length,
    shank_diameter=None,
    flutes=3,
    material="carbide",
    name=None,
):
    """Single-profile thread mill: a tooth of ``diameter`` and ``tooth_height``
    on a neck. Modeled as a flat, non-center-cutting cutter with a neck, so
    the stock shows the swept tooth envelope (not the thread form)."""
    return _cutter(
        id,
        {"kind": "flat"},
        diameter=diameter,
        flute_length=tooth_height,
        overall_length=overall_length,
        shank_diameter=shank_diameter,
        flutes=flutes,
        center_cutting=False,
        material=material,
        name=name,
        neck=Neck(neck_diameter, neck_length),
    )


def TSlotCutter(
    id,
    *,
    diameter,
    width,
    neck_diameter,
    neck_length,
    overall_length,
    shank_diameter=None,
    flutes=6,
    material="carbide",
    name=None,
):
    """T-slot cutter: a cutting disk of ``diameter`` and ``width`` (its
    height) on a narrower neck. It cuts the wide bottom of a T-slot under a
    slot milled first (``camlib.operations.Undercut``, tri-dexel stock)."""
    return _cutter(
        id,
        {"kind": "flat"},
        diameter=diameter,
        flute_length=width,
        overall_length=overall_length,
        shank_diameter=shank_diameter,
        flutes=flutes,
        center_cutting=False,
        material=material,
        name=name,
        neck=Neck(neck_diameter, neck_length),
    )


def KeyseatCutter(id, **dimensions):
    """Keyseat (Woodruff) cutter: a thin cutting disk on a neck, cutting a
    keyseat into a wall from the side. Dimensions as for ``TSlotCutter``."""
    return TSlotCutter(id, **dimensions)


def LollipopCutter(
    id,
    *,
    diameter,
    neck_diameter,
    neck_length,
    overall_length,
    cutting_length=None,
    shank_diameter=None,
    flutes=4,
    material="carbide",
    name=None,
):
    """Lollipop (ball-shank undercut) cutter: a sphere of ``diameter`` on a
    neck. ``cutting_length``: the sphere's height up to where it meets the
    neck (default: where the sphere's radius equals the neck's)."""
    if cutting_length is None:
        r, n = diameter / 2, neck_diameter / 2
        cutting_length = r + math.sqrt(max(r * r - n * n, 0))
    return _cutter(
        id,
        {"kind": "lollipop"},
        diameter=diameter,
        flute_length=cutting_length,
        overall_length=overall_length,
        shank_diameter=shank_diameter,
        flutes=flutes,
        center_cutting=False,
        material=material,
        name=name,
        neck=Neck(neck_diameter, neck_length),
    )


def DovetailCutter(
    id,
    *,
    diameter,
    angle,
    cutting_length,
    overall_length,
    neck_diameter=None,
    neck_length=None,
    shank_diameter=None,
    flutes=6,
    material="carbide",
    name=None,
):
    """Dovetail cutter: widest at its flat tip (``diameter``), the flank
    tapering toward the spindle at ``angle`` degrees from the tip face (a 60
    degree cutter cuts a 60 degree dovetail)."""
    neck = None if neck_diameter is None else Neck(neck_diameter, neck_length)
    return _cutter(
        id,
        {"kind": "dovetail", "flank_angle_rad": math.radians(angle)},
        diameter=diameter,
        flute_length=cutting_length,
        overall_length=overall_length,
        shank_diameter=shank_diameter,
        flutes=flutes,
        center_cutting=False,
        material=material,
        name=name,
        neck=neck,
    )


def BoringBar(
    id,
    *,
    diameter,
    bar_diameter,
    reach,
    overall_length,
    insert_length=1,
    shank_diameter=None,
    material="carbide",
    name=None,
):
    """Single-point boring bar: the insert cuts ``diameter`` at the end of a
    bar of ``bar_diameter`` reaching ``reach`` below the shank. Modeled as a
    flat, non-center-cutting cutter of the bore diameter (``insert_length``
    long) on a neck of the bar, so fine boring can prove its shift stays in
    the bore. The stopped bar lies within its diameter except the cutting
    edge."""
    if not 0 < insert_length < reach:
        raise ValueError("insert_length must be positive and shorter than reach")
    return _cutter(
        id,
        {"kind": "flat"},
        diameter=diameter,
        flute_length=insert_length,
        overall_length=overall_length,
        shank_diameter=shank_diameter,
        flutes=1,
        center_cutting=False,
        material=material,
        name=name,
        neck=Neck(bar_diameter, reach - insert_length),
    )


def _beam(id, kerf_width, nozzle_diameter, nozzle_length, body_diameter, body_length, name):
    if not 0 < kerf_width < nozzle_diameter <= body_diameter:
        raise ValueError("kerf_width < nozzle_diameter <= body_diameter")
    cutter = _cutter(
        id,
        {"kind": "beam"},
        diameter=kerf_width,
        flute_length=nozzle_length / 2,
        overall_length=nozzle_length,
        shank_diameter=nozzle_diameter,
        flutes=1,
        center_cutting=True,
        material="",
        name=name,
    )
    # The body (torch or laser head) above the nozzle, for collision checks.
    holder = (
        HolderSection(nozzle_length * 0.75, nozzle_length * 0.75 + body_length, body_diameter / 2, body_diameter / 2),
    )
    return ToolAssembly(id, cutter, nozzle_length * 0.75, nozzle_length * 0.75 + body_length, holder)


def Torch(id, *, kerf_width, nozzle_diameter=6, nozzle_length=20, body_diameter=30, body_length=150, name=None):
    """Plasma torch: the arc cuts a kerf of ``kerf_width`` through the whole
    sheet below the nozzle tip. The nozzle and torch body are collision
    geometry. Returns a ToolAssembly ready for ThermalCut."""
    return _beam(id, kerf_width, nozzle_diameter, nozzle_length, body_diameter, body_length, name)


def LaserHead(id, *, kerf_width, nozzle_diameter=3, nozzle_length=15, body_diameter=25, body_length=80, name=None):
    """Laser cutting head: the beam cuts a kerf of ``kerf_width`` through the
    sheet below the nozzle (focus) tip. Returns a ToolAssembly."""
    return _beam(id, kerf_width, nozzle_diameter, nozzle_length, body_diameter, body_length, name)


def ProbeTool(id, *, ball_diameter, stem_diameter, stylus_length, body_diameter, body_length, name=None):
    """A touch probe as a tool assembly: the stylus ball (``ball_diameter``,
    the calibrated effective diameter the controller's probe calibration
    holds), its stem (``stem_diameter``, thinner than the ball) reaching
    ``stylus_length`` from the ball's bottom (the controlled point) to the
    probe body, and the body (``body_diameter``) up ``body_length`` to the
    gauge line. Stylus, stem and body are checked against stock and fixtures
    on every probing move. A probe never cuts: only the probing operations
    (ProbeSurface, ProbeBore, ...) use it. Every dimension is the caller's:
    there are no defaults."""
    if not 0 < stem_diameter < ball_diameter < stylus_length:
        raise ValueError("a probe needs 0 < stem_diameter < ball_diameter < stylus_length")
    if not (body_diameter >= stem_diameter and body_length > 0):
        raise ValueError("the probe body must be at least as wide as the stem, with a positive length")
    gauge = stylus_length + body_length
    cutter = _cutter(
        id,
        {"kind": "probe"},
        diameter=ball_diameter,
        flute_length=ball_diameter,
        overall_length=gauge,
        shank_diameter=stem_diameter,
        flutes=1,
        center_cutting=False,
        material="",
        name=name,
    )
    holder = (HolderSection(stylus_length, gauge, body_diameter / 2, body_diameter / 2),)
    return ToolAssembly(id, cutter, stylus_length, gauge, holder)


def Tap(id, *, drill_diameter, thread_length, overall_length, shank_diameter, flutes=3, material="hss", name=None):
    """Tap modeled by its drill (minor) diameter: a flat, non-center-cutting
    cutter whose cutting length is the thread length. The stock then keeps
    the thread flanks (conservative). The pitch belongs to the operation."""
    return _cutter(
        id,
        {"kind": "flat"},
        diameter=drill_diameter,
        flute_length=thread_length,
        overall_length=overall_length,
        shank_diameter=shank_diameter,
        flutes=flutes,
        center_cutting=False,
        material=material,
        name=name,
    )


@dataclass(frozen=True)
class HolderSection:
    z_min_mm: float
    z_max_mm: float
    radius_min_mm: float
    radius_max_mm: float


def holder_sections_from_mesh(mesh, *, tip_mm, axis, stickout_mm, gauge_length_mm, axial_resolution_mm):
    """A milling holder's envelope from its model (#110): the
    ``HolderSection`` cylinders of a ``ToolAssembly`` from ``stickout_mm``
    up to ``gauge_length_mm``.

    A holder turns with the spindle, so what is checked is the solid its
    model sweeps about the tool axis. ``mesh``: a ``SurfaceMesh`` of the
    holder (a closed surface in the model's own coordinates; e.g.
    ``camlib.cadquery.step_mesh``). Its placement is declared: ``tip_mm``,
    the model point of the tool tip on the axis, and ``axis``, the model's
    unit direction from the tip toward the spindle. Each slice of
    ``axial_resolution_mm`` along the axis becomes a cylinder of the largest
    radius the model reaches in it, the mesh's ``surface_error_mm`` added
    (along the axis too), so the envelope contains the turning holder and
    over-estimates it along the axis by less than one slice.

    Refused: a model that starts below ``stickout_mm`` (the declared
    stickout is then wrong), one that does not span the range, an open
    mesh. The model above the gauge plane (the taper in the spindle) is not
    part of the assembly."""
    import json

    from ._buffers import native

    rows = native.holder_envelope_from_mesh(
        json.dumps(mesh.to_dict()),
        [float(v) for v in tip_mm],
        [float(v) for v in axis],
        [float(stickout_mm), float(gauge_length_mm)],
        float(axial_resolution_mm),
    )
    return tuple(HolderSection(z0, z1, radius, radius) for z0, z1, radius in rows)


@dataclass(frozen=True)
class ToolAssembly:
    id: str
    cutter: Cutter
    stickout_mm: float
    gauge_length_mm: float
    holders: tuple[HolderSection, ...]

    def to_dict(self):
        return {
            "id": self.id,
            "cutter": self.cutter.to_dict(),
            "stickout_mm": self.stickout_mm,
            "gauge_length_mm": self.gauge_length_mm,
            "holders": [asdict(s) for s in self.holders],
        }

    @classmethod
    def from_dict(cls, data):
        data = dict(data)
        data["cutter"] = Cutter.from_dict(data["cutter"])
        data["holders"] = tuple(HolderSection(**s) for s in data["holders"])
        return cls(**data)
