"""Small synthetic assembled profiles; these are not manufacturer cutting data."""

from camlib import (
    AnnularFrustum,
    CircularRing,
    FaceMill,
    HighFeedMill,
    IndexableGeometry,
    MountedInsertGroup,
    RotationalProfile,
    ToolAssembly,
    HolderSection,
)
from camlib.operations import Facing, Roughing, RasterClearing, StockEdgeEntry
from camlib.simulation import StockSpec
from camlib.surface import SurfaceMesh


def tool(round_insert=False):
    edge = CircularRing((4, 1), 1, (0, 2)) if round_insert else AnnularFrustum((0, 2), (2, 2), (4, 6))
    geometry = IndexableGeometry(
        (MountedInsertGroup('primary', 3, RotationalProfile((edge,))),),
        RotationalProfile((AnnularFrustum((2.5, 10), (0, 0), (4, 4) if round_insert else (5, 5)),)),
    )
    cutter = (HighFeedMill if round_insert else FaceMill)(
        'inserted',
        geometry=geometry,
        diameter=10 if round_insert else 12,
        cutting_height=2,
        overall_length=30,
        shank_diameter=6,
        # This full circular meridian can engage above the nominal stepdown
        # at a previous pass's curved shoulder. The declared limit must cover
        # that engagement too; a 1 mm limit is separately tested as a refusal.
        max_depth_of_cut_mm=2 if round_insert else 1,
        max_ramp_angle_deg=3,
        helical_hole_diameter_mm=(14, 18),
        material='synthetic',
    )
    return ToolAssembly('mill', cutter, 15, 40, (HolderSection(15, 40, 6, 6),))


def facing_job():
    mill = tool()
    op = Facing(
        'face',
        mill,
        floor_z_mm=3,
        stepdown_mm=0.5,
        stepover_mm=4,
        clearance_z_mm=12,
        edge_overlap_mm=1,
        feed_mm_min=500,
        plunge_mm_min=100,
        spindle_rpm=1000,
        tool_number=1,
        length_register=1,
        coolant='flood',
    )
    return dict(
        stock=StockSpec((-10, -8, 0), (10, 8, 6), 0.25),
        tools=(mill,),
        operations=(op,),
        revision='indexable-1',
        initial_position_mm=(0, 0, 12),
    )


def target():
    # Analytic horizontal floor; no tessellation error.
    return SurfaceMesh(
        (((-10, -8, 3), (10, -8, 3), (10, 8, 3)), ((-10, -8, 3), (10, 8, 3), (-10, 8, 3))), surface_error_mm=0
    )


def roughing_job():
    config = facing_job()
    mill = tool(True)
    op = Roughing(
        'rough',
        mill,
        roughing_target(),
        top_z_mm=6,
        floor_z_mm=3,
        stepdown_mm=0.5,
        clearance_z_mm=12,
        clearing=RasterClearing(3),
        entry=StockEdgeEntry(helix=None),
        feed_mm_min=500,
        plunge_mm_min=100,
        spindle_rpm=1000,
        tool_number=1,
        length_register=1,
        coolant='flood',
    )
    return dict(config, tools=(mill,), operations=(op,))


def roughing_target():
    # Retain a rectangular boss. The curved insert leaves material at its
    # foot that a flat end mill of the same maximum diameter would remove.
    triangles = list(target().triangles)
    bottom = ((0, -3, 3), (6, -3, 3), (6, 3, 3), (0, 3, 3))
    top = tuple((x, y, 6) for x, y, _ in bottom)
    triangles.extend(((top[0], top[1], top[2]), (top[0], top[2], top[3])))
    for i in range(4):
        j = (i + 1) % 4
        triangles.extend(((bottom[i], bottom[j], top[j]), (bottom[i], top[j], top[i])))
    return SurfaceMesh(tuple(triangles), surface_error_mm=0)
