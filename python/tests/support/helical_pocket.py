"""Synthetic CAM regression input."""

from camlib.cadquery import export_floor_region
from camlib.operations import AdaptivePocket, HelicalEntry, Pocket
from camlib.simulation import StockSpec
from camlib.tools import EndMill, HolderSection, ToolAssembly


def cavity():
    import cadquery as cq

    outer = cq.Workplane('XY').rect(26, 18).extrude(4).translate((16, 12, 6))
    island = cq.Workplane('XY').circle(3).extrude(4).translate((16, 12, 6))
    return outer.cut(island)


def target():
    import cadquery as cq

    return cq.Workplane('XY').box(32, 24, 10, centered=False).cut(cavity())


def helical_job(adaptive=False):
    region = export_floor_region(
        cavity().faces('<Z').val(),
        name='helical-island',
        source=dict(model='CadQuery island pocket', revision='helical-1'),
    )
    tool = ToolAssembly(
        't1', EndMill('em4', diameter=4, flute_length=20, overall_length=50), 30, 60, (HolderSection(30, 60, 4, 4),)
    )
    cls = AdaptivePocket if adaptive else Pocket
    # The orbit planner: this job checks its entries and its orbit options.
    extra = (
        dict(
            path_strategy='disk_front',
            orbit_radius_mm=1,
            min_orbit_radius_mm=0.25,
            local_orbit_tolerance_mm=0.05,
        )
        if adaptive
        else {}
    )
    operation = cls(
        'helical',
        tool,
        region,
        10,
        6,
        2,
        1.5,
        12,
        entry=HelicalEntry(),
        **extra,
        feed_mm_min=600,
        plunge_mm_min=150,
        spindle_rpm=12000,
        tool_number=1,
        length_register=1,
        coolant='flood',
    )
    return dict(
        stock=StockSpec((0, 0, 0), (32, 24, 10), 0.4),
        tools=(tool,),
        operations=(operation,),
        revision='helical-1',
        initial_position_mm=(0, 0, 12),
    )
