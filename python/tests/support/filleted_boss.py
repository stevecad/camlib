"""Synthetic CAM regression input: a filleted boss for roughing on a fine mesh."""

from camlib.operations import OffsetClearing, Roughing
from camlib.simulation import StockSpec
from camlib.tools import EndMill, HolderSection, ToolAssembly


def part():
    """A 30 x 30 x 4 mm base with a 16 mm round boss 6 mm tall, its top edge
    rounded 3 mm: curved faces that a fine mesh splits into many facets."""
    import cadquery as cq

    base = cq.Workplane("XY").box(30, 30, 4, centered=False)
    boss = base.faces(">Z").workplane().center(15, 15).circle(8).extrude(6)
    return boss.faces(">Z").edges().fillet(3)


def roughing_job(max_error_mm=0.005):
    from camlib.cadquery import export_surface_mesh

    tool = ToolAssembly(
        "em6", EndMill("em6", diameter=6, flute_length=20, overall_length=50), 30, 60, (HolderSection(30, 60, 5, 5),)
    )
    operation = Roughing(
        "rough",
        tool,
        export_surface_mesh(part().val(), max_error_mm=max_error_mm),
        top_z_mm=11,
        floor_z_mm=4,
        stepdown_mm=2.5,
        clearance_z_mm=14,
        clearing=OffsetClearing(2.5),
        radial_allowance_mm=0.3,
        bottom_allowance_mm=0.2,
        feed_mm_min=1200,
        plunge_mm_min=300,
        spindle_rpm=10000,
        tool_number=1,
        length_register=1,
        coolant="flood",
    )
    return dict(
        stock=StockSpec((-1, -1, 0), (31, 31, 11), 0.25),
        tools=(tool,),
        operations=(operation,),
        revision="boss-1",
        initial_position_mm=(0, 0, 14),
    )
