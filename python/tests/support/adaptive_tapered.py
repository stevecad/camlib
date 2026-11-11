"""Synthetic CAM regression input."""

from camlib.cadquery import export_floor_region
from camlib.operations import AdaptivePocket
from camlib.simulation import StockSpec
from camlib.tools import EndMill, HolderSection, ToolAssembly

POINTS = (
    (3, 3),
    (13, 3),
    (13, 7.5),
    (21, 6.5),
    (21, 3),
    (31, 3),
    (31, 17),
    (21, 17),
    (21, 13.5),
    (13, 12.5),
    (13, 17),
    (3, 17),
)


def cavity():
    import cadquery as cq

    return cq.Workplane('XY').polyline(POINTS).close().extrude(1).translate((0, 0, 9))


def target():
    import cadquery as cq

    return cq.Workplane('XY').box(34, 20, 10, centered=(False, False, False)).cut(cavity())


def tapered_job():
    region = export_floor_region(
        cavity().faces('<Z').val(),
        name='tapered-rooms',
        source=dict(model='CadQuery tapered neck', revision='tapered-1'),
    )
    tool = ToolAssembly(
        't1', EndMill('em4', diameter=4, flute_length=20, overall_length=50), 30, 60, (HolderSection(30, 60, 4, 4),)
    )
    op = AdaptivePocket(
        'tapered',
        tool,
        region,
        10,
        9,
        1,
        1.5,
        12,
        orbit_radius_mm=1,
        path_strategy="disk_front",
        min_orbit_radius_mm=0.25,
        local_orbit_tolerance_mm=0.05,
        feed_mm_min=600,
        plunge_mm_min=150,
        spindle_rpm=12000,
        tool_number=1,
        length_register=1,
        coolant="flood",
    )
    return dict(
        stock=StockSpec((0, 0, 0), (34, 20, 10), 0.4),
        tools=(tool,),
        operations=(op,),
        revision='tapered-1',
        initial_position_mm=(0, 0, 12),
    )
