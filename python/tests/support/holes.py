"""Synthetic CAM regression input."""

import math
from camlib.operations import Drilling, Bore
from camlib.tools import Drill, EndMill, HolderSection, ToolAssembly
from camlib.simulation import StockSpec


def cadquery_target():
    import cadquery as cq

    blank = cq.Workplane("XY").box(40, 24, 10, centered=(False, False, False))
    holes = cq.Workplane("XY").pushPoints(((10, 12), (28, 12))).circle(6).extrude(6).translate((0, 0, 4))
    return blank.cut(holes)


def hole_job():
    target = cadquery_target()
    edges = target.faces(">Z").edges("%CIRCLE").vals()
    centers = tuple(sorted(edge.arcCenter().toTuple()[:2] for edge in edges))
    radii = [edge.radius() for edge in edges]
    assert len(centers) == 2 and all(abs(r - 6) < 1e-7 for r in radii)
    drill = ToolAssembly(
        "drill4",
        Drill("drill-bit", diameter=4, flute_length=30, overall_length=50),
        35,
        60,
        (HolderSection(35, 60, 4, 4),),
    )
    mill = ToolAssembly(
        "mill4",
        EndMill("mill-bit", diameter=4, flute_length=30, overall_length=50),
        35,
        60,
        (HolderSection(35, 60, 4, 4),),
    )
    pilot = Drilling(
        "pilot",
        drill,
        centers,
        depth_z_mm=4,
        clearance_z_mm=12,
        peck_mm=2,
        feed_mm_min=150,
        retract_feed_mm_min=600,
        spindle_rpm=6000,
        tool_number=1,
        length_register=1,
        coolant="flood",
    )
    bore = Bore(
        "finish-holes",
        mill,
        centers,
        diameter_mm=12,
        floor_z_mm=4,
        stepdown_mm=2,
        clearance_z_mm=12,
        tool_number=2,
        length_register=2,
        entry_feed_mm_min=150,
        feed_mm_min=600,
        retract_feed_mm_min=600,
        spindle_rpm=12000,
        coolant="flood",
    )
    return dict(
        stock=StockSpec((0, 0, 0), (40, 24, 10), 0.2),
        tools=(drill, mill),
        operations=(pilot, bore),
        revision="holes-1",
        initial_position_mm=(0, 0, 12),
    )


def cadquery_drill_target(tip_z=4):
    """Independent 118-degree cone + full-diameter cylinder target."""
    import cadquery as cq

    blank = cq.Workplane("XY").box(40, 24, 10, centered=(False, False, False))
    height = 2 / math.tan(math.radians(59))
    for x, y in ((10, 12), (28, 12)):
        cone = cq.Solid.makeCone(0, 2, height, cq.Vector(x, y, tip_z), cq.Vector(0, 0, 1))
        cylinder = cq.Solid.makeCylinder(2, 10 - tip_z - height, cq.Vector(x, y, tip_z + height), cq.Vector(0, 0, 1))
        blank = blank.cut(cone).cut(cylinder)
    return blank
