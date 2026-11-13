"""Synthetic solids with B-spline faces, for the CadQuery adapter's tests.

Every kind of spline face the adapter bounds: polynomial and rational,
periodic, with poles, only C0 along a knot line, trimmed by an intersection
curve, and beside analytic faces.
"""

import math

import cadquery as cq


def nurbs(shape):
    """The shape with every face and edge rewritten as a (rational) B-spline."""
    from OCP.BRepBuilderAPI import BRepBuilderAPI_NurbsConvert

    return cq.Shape.cast(BRepBuilderAPI_NurbsConvert(shape.wrapped, True).Shape())


def ellipsoid(radius=3.0, stretch=1.5):
    """A sphere scaled along Z: one rational B-spline face with two poles."""
    return (
        cq.Workplane("XY")
        .sphere(radius)
        .val()
        .transformGeometry(cq.Matrix([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, stretch, 0], [0, 0, 0, 1]]))
    )


def saddle_face(nx=7, ny=6):
    """A 20 x 16 mm polynomial B-spline sheet: a saddle with a ripple."""

    def height(x, y, i, j):
        return 0.03 * (x * x - y * y) + 0.8 * math.sin(0.5 * i + 0.3 * j)

    points = [
        [cq.Vector(x, y, height(x, y, i, j)) for j in range(ny) for y in (-8 + 16 * j / (ny - 1),)]
        for i in range(nx)
        for x in (-10 + 20 * i / (nx - 1),)
    ]
    return cq.Face.makeSplineApprox(points)


def saddle_block(depth=9.0):
    """The saddle sheet extruded down: spline top and bottom, extruded
    spline sides."""
    from OCP.BRepPrimAPI import BRepPrimAPI_MakePrism
    from OCP.gp import gp_Vec

    return cq.Shape.cast(BRepPrimAPI_MakePrism(saddle_face().wrapped, gp_Vec(0, 0, -depth)).Shape())


def bored_saddle():
    """The saddle block with a bore through it: spline faces trimmed by the
    bore's intersection curves."""
    bore = cq.Workplane("XY").circle(3.2).extrude(30, both=True).translate((2, 1, 0))
    return cq.Workplane("XY").add(saddle_block()).cut(bore).val()


def ruled_loft():
    """A circle lofted to a rectangle with ruled faces (degree 1 along the loft)."""
    corners = [cq.Vector(-3, -2, 6), cq.Vector(3, -2, 6), cq.Vector(3, 2, 6), cq.Vector(-3, 2, 6)]
    wires = [cq.Wire.makeCircle(5, cq.Vector(0, 0, 0), cq.Vector(0, 0, 1)), cq.Wire.makePolygon(corners, close=True)]
    return cq.Solid.makeLoft(wires, True)


def smooth_loft():
    """An ellipse, a circle and an ellipse lofted smoothly: periodic-like
    spline faces closed around the loft."""
    return (
        cq.Workplane("XY")
        .ellipse(6, 4)
        .workplane(offset=5)
        .circle(3)
        .workplane(offset=5)
        .ellipse(2, 5)
        .loft(combine=True)
        .val()
    )


def folded_sheet_block(depth=6.0):
    """A degree 1 B-spline sheet with a crease along u = 1 (a C0 knot line),
    extruded down."""
    from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
    from OCP.BRepPrimAPI import BRepPrimAPI_MakePrism
    from OCP.Geom import Geom_BSplineSurface
    from OCP.gp import gp_Pnt, gp_Vec

    try:
        from OCP.TColgp import TColgp_Array2OfPnt as Points
        from OCP.TColStd import TColStd_Array1OfInteger as Integers, TColStd_Array1OfReal as Reals
    except ImportError:  # OCP 8 moved collection aliases.
        from OCP.collections import Array1_double as Reals, Array1_int as Integers, Array2_gp_Pnt as Points

    poles = Points(1, 3, 1, 2)
    for i, (x, z) in enumerate(((0.0, 0.0), (6.0, 2.5), (12.0, 0.5)), 1):
        for j, y in enumerate((0.0, 8.0), 1):
            poles.SetValue(i, j, gp_Pnt(x, y, z + 0.2 * y * (i - 2)))
    uknots, umults = Reals(1, 3), Integers(1, 3)
    for i, (knot, multiplicity) in enumerate(((0.0, 2), (1.0, 1), (2.0, 2)), 1):
        uknots.SetValue(i, knot)
        umults.SetValue(i, multiplicity)
    vknots, vmults = Reals(1, 2), Integers(1, 2)
    for i, (knot, multiplicity) in enumerate(((0.0, 2), (1.0, 2)), 1):
        vknots.SetValue(i, knot)
        vmults.SetValue(i, multiplicity)
    surface = Geom_BSplineSurface(poles, uknots, vknots, umults, vmults, 1, 1)
    face = BRepBuilderAPI_MakeFace(surface, 1e-7).Face()
    return cq.Shape.cast(BRepPrimAPI_MakePrism(face, gp_Vec(0, 0, -depth)).Shape())


def revolved_spline():
    """A spline profile revolved about an axis: a surface of revolution, for
    which the adapter has no bound."""
    return (
        cq.Workplane("XZ")
        .moveTo(0, 0)
        .lineTo(3, 0)
        .spline([(2.5, 2), (3, 4)], includeCurrent=True)
        .lineTo(0, 4)
        .close()
        .revolve(360, (0, 0, 0), (0, 1, 0))
        .val()
    )
