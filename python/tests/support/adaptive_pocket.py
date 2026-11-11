"""Synthetic CAM regression input."""

from camlib.cadquery import export_floor_region
from camlib.operations import AdaptivePocket
from camlib.simulation import StockSpec
from camlib.tools import EndMill, HolderSection, ToolAssembly


def cavity():
    import cadquery as cq

    return (
        cq.Workplane("XY")
        .polyline([(3, 3), (23, 3), (23, 11), (11, 11), (11, 23), (3, 23)])
        .close()
        .extrude(2)
        .translate((0, 0, 8))
    )


def target():
    import cadquery as cq

    return cq.Workplane("XY").box(26, 26, 10, centered=(False, False, False)).cut(cavity())


def adaptive_job(path_strategy="engagement_front"):
    face = cavity().faces("<Z").val()
    # Export the same XY floor at the nominal final depth.
    region = export_floor_region(face, name="l-pocket", source=dict(model="CadQuery L-pocket", revision="adaptive-1"))
    tool = ToolAssembly(
        "t1", EndMill("em4", diameter=4, flute_length=20, overall_length=50), 30, 60, (HolderSection(30, 60, 4, 4),)
    )
    op = AdaptivePocket(
        "adaptive",
        tool,
        region,
        10,
        8,
        1,
        1.5,
        12,
        feed_mm_min=600,
        plunge_mm_min=150,
        spindle_rpm=12000,
        tool_number=1,
        length_register=1,
        coolant="flood",
        path_strategy=path_strategy,
    )
    return dict(
        stock=StockSpec((0, 0, 0), (26, 26, 10), 0.4),
        tools=(tool,),
        operations=(op,),
        revision="adaptive-1",
        initial_position_mm=(0, 0, 12),
    )
