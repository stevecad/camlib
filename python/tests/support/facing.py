"""Synthetic CAM regression input."""

from camlib.operations import Facing
from camlib.simulation import StockSpec
from camlib.tools import EndMill, HolderSection, ToolAssembly


def facing_job():
    tool = ToolAssembly(
        "face8", EndMill("em8", diameter=8, flute_length=30, overall_length=50), 35, 60, (HolderSection(35, 60, 5, 5),)
    )
    face = Facing(
        "face",
        tool,
        floor_z_mm=8,
        stepdown_mm=1,
        stepover_mm=6,
        clearance_z_mm=12,
        feed_mm_min=600,
        plunge_mm_min=150,
        spindle_rpm=12000,
        tool_number=1,
        length_register=1,
        coolant="flood",
    )
    return dict(
        stock=StockSpec((0, 0, 0), (40, 24, 10), 0.25),
        tools=(tool,),
        operations=(face,),
        revision="facing-1",
        initial_position_mm=(0, 0, 12),
    )


def cadquery_target():
    import cadquery as cq

    return cq.Workplane("XY").box(40, 24, 8, centered=(False, False, False))


# Two strap clamps over the ends of a 100 x 60 block (#422): their keep-out
# areas reach into the facing passes along the block's X- and X+ edges.
CLAMPS = ((-1, 20, 6, 40), (94, 20, 101, 40))
CLAMP_TOP_Z = 15


def clamped_facing_job():
    from camlib import AvoidArea, Limits
    from camlib.planar import Boundary, Component, PlanarRegion

    tool = ToolAssembly(
        "face10",
        EndMill("em10", diameter=10, flute_length=30, overall_length=60),
        35,
        60,
        (HolderSection(35, 60, 8, 8),),
    )
    face = Facing(
        "face",
        tool,
        floor_z_mm=9,
        stepdown_mm=1,
        stepover_mm=4,
        clearance_z_mm=20,
        feed_mm_min=1200,
        plunge_mm_min=400,
        spindle_rpm=9000,
        tool_number=1,
        length_register=1,
        coolant="flood",
    )

    def area(x0, y0, x1, y1, name):
        corners = ((x0, y0), (x1, y0), (x1, y1), (x0, y1))
        return AvoidArea(PlanarRegion(name, (Component(Boundary.polygon(corners)),)), CLAMP_TOP_Z)

    limits = Limits(avoid=tuple(area(*box, f"clamp_{i}") for i, box in enumerate(CLAMPS, 1)))
    return dict(
        stock=StockSpec((0, 0, 0), (100, 60, 10), 0.5),
        tools=(tool,),
        operations=(dict(face.to_dict(), limits=limits.to_dict()),),
        revision="clamped-facing-1",
        initial_position_mm=(-20, -20, 20),
    )
