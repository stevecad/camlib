"""Synthetic CAM regression input."""

from camlib.operations import AdaptiveClearing, OffsetClearing, RasterClearing, Roughing
from camlib.simulation import StockSpec
from camlib.tools import EndMill, HolderSection, ToolAssembly


def part():
    import cadquery as cq

    base = cq.Workplane('XY').box(24, 20, 2, centered=False).translate((4, 4, 0))
    # Construct the six actual planes: generic loft represents these sides as
    # BSplines, whose approximation is outside the bounded planar CAD adapter.
    bottom = [(8, 8, 2), (24, 8, 2), (24, 20, 2), (8, 20, 2)]
    top = [(11, 10, 10), (21, 10, 10), (21, 18, 10), (11, 18, 10)]
    rings = [list(reversed(bottom)), top]
    rings.extend([bottom[i], bottom[(i + 1) % 4], top[(i + 1) % 4], top[i]] for i in range(4))
    faces = [cq.Face.makeFromWires(cq.Wire.makePolygon(ring, close=True)) for ring in rings]
    frustum = cq.Workplane('XY').newObject([cq.Solid.makeSolid(cq.Shell.makeShell(faces))])
    opening = cq.Workplane('XY').box(4, 4, 8, centered=False).translate((14, 12, 2))
    return base.union(frustum).cut(opening)


def roughing_job(
    adaptive=False, *, routing=None, offset=False, omit_empty_cuts=False, stock_front=False, engagement_front=False
):
    # Adaptive roughing is engagement_front (Roughing refuses disk_front, #167).
    engagement_front = engagement_front or (adaptive and not stock_front)
    if engagement_front:
        clearing = AdaptiveClearing(0.25)
    if stock_front:
        adaptive = True
    if offset and (adaptive or routing is not None):
        raise ValueError('offset selects OffsetClearing; use it without --adaptive/--economical')
    if adaptive and routing is not None:
        raise ValueError('routing selects RasterClearing; use it without --adaptive')
    tool = ToolAssembly(
        't1', EndMill('em2', diameter=2, flute_length=20, overall_length=50), 30, 60, (HolderSection(30, 60, 4, 4),)
    )
    clearing = (
        clearing
        if engagement_front
        else AdaptiveClearing(
            0.25,
            orbit_radius_mm=0.5,
            min_orbit_radius_mm=0.2,
            local_orbit_tolerance_mm=0.05,
            path_strategy='stock_front',
        )
        if adaptive
        else OffsetClearing(0.8)
        if offset
        else RasterClearing(0.8, routing=routing)
    )
    operation = Roughing.from_solid(
        'rough',
        tool,
        part(),
        top_z_mm=10,
        floor_z_mm=2,
        stepdown_mm=2,
        clearance_z_mm=12,
        clearing=clearing,
        radial_allowance_mm=0.1,
        bottom_allowance_mm=0.2,
        omit_empty_cuts=omit_empty_cuts,
        source=dict(model='CadQuery retained frustum and enclosed opening', revision='roughing-1'),
        feed_mm_min=600,
        plunge_mm_min=150,
        spindle_rpm=12000,
        tool_number=1,
        length_register=1,
        coolant="flood",
    )
    return dict(
        stock=StockSpec((0, 0, 0), (32, 28, 10), 0.4),
        tools=(tool,),
        operations=(operation,),
        revision='roughing-1',
        initial_position_mm=(0, 0, 12),
    )
