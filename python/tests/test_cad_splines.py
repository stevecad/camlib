"""The CadQuery adapter meshes B-spline and Bezier faces within a proven error (#240).

Checked here without the adapter's own bookkeeping, as for the analytic faces
(test_cad_watertight): facet edges matched on their bitwise coordinates, the
enclosed volume, true surface and edge points evaluated through OCP against
the facets, and facet points against the CAD faces. The derivative bounds are
compared with the surface's own second derivatives (OCP's D2), and the trim
bound with points of the trimming curve.

The cost is part of the contract: a bound that holds but is loose asks for
many times the facets the surface needs, and every operation then pays for
them. So the facet counts are held against the kernel's own (unproven)
triangulation at the same deflection.
"""

from functools import lru_cache
import math
import random
import unittest
from unittest import mock

try:
    import cadquery as cq
    import numpy as np
except ImportError:  # pragma: no cover - CI installs CadQuery
    cq = None

from camlib import _cad_surface as cad
from camlib.cadquery import export_surface_mesh

if cq is not None:
    from support import spline_solids
    from test_cad_watertight import (
        distance_to_mesh,
        distance_to_solid,
        edge_points,
        surface_points,
        unmatched_edges,
    )


def _tilted(shape):
    return shape.rotate((0, 0, 0), (1, 2, 3), 31).translate((200, -50, 17))


def _nurbs_filleted_box():
    box = cq.Workplane("XY").box(20, 15, 8, centered=(True, True, False)).edges().fillet(2).val()
    return spline_solids.nurbs(box)


# name: (solid, requested error, exact volume where one is known: the
# kernel's own integral of a rational spline solid is off by up to a percent).
SHAPES = {
    "ellipsoid": (lambda: spline_solids.ellipsoid(), 0.01, 4 / 3 * math.pi * 3 * 3 * 4.5),
    "ellipsoid, fine": (lambda: spline_solids.ellipsoid(), 0.002, 4 / 3 * math.pi * 3 * 3 * 4.5),
    # makeSphere is a hemisphere over a flat cap.
    "nurbs sphere": (lambda: spline_solids.nurbs(cq.Solid.makeSphere(4)), 0.01, 2 / 3 * math.pi * 4**3),
    "nurbs torus": (lambda: spline_solids.nurbs(cq.Solid.makeTorus(5, 1)), 0.01, 2 * math.pi**2 * 5),
    "nurbs filleted box": (_nurbs_filleted_box, 0.005, None),
    "saddle block": (spline_solids.saddle_block, 0.01, 20 * 16 * 9),
    "bored saddle": (spline_solids.bored_saddle, 0.005, None),
    "ruled loft": (spline_solids.ruled_loft, 0.01, None),
    "smooth loft": (spline_solids.smooth_loft, 0.01, None),
    "folded sheet": (spline_solids.folded_sheet_block, 0.01, 12 * 8 * 6),
    "tilted bored saddle": (lambda: _tilted(spline_solids.bored_saddle()), 0.01, None),
}


@lru_cache(None)
def solid(name):
    return SHAPES[name][0]()


@lru_cache(None)
def mesh(name):
    return export_surface_mesh(solid(name), max_error_mm=SHAPES[name][1])


def kernel_facets(shape, deflection):
    """Facets of the CAD kernel's own triangulation at a deflection."""
    from OCP.BRep import BRep_Tool
    from OCP.BRepBuilderAPI import BRepBuilderAPI_Copy
    from OCP.BRepMesh import BRepMesh_IncrementalMesh
    from OCP.TopLoc import TopLoc_Location

    copy = cq.Shape.cast(BRepBuilderAPI_Copy(shape.wrapped, True, False).Shape())
    BRepMesh_IncrementalMesh(copy.wrapped, deflection, False, 0.5, True)
    return sum(BRep_Tool.Triangulation_s(face.wrapped, TopLoc_Location()).NbTriangles() for face in copy.Faces())


def spline_faces(shape):
    from OCP.BRepAdaptor import BRepAdaptor_Surface
    from OCP.GeomAbs import GeomAbs_BezierSurface, GeomAbs_BSplineSurface

    for face in shape.Faces():
        surface = BRepAdaptor_Surface(face.wrapped)
        if surface.GetType() in (GeomAbs_BSplineSurface, GeomAbs_BezierSurface):
            yield surface


def second_derivatives(surface, u, v):
    """|Fu|, |Fv|, |Fuu|, |Fuv|, |Fvv| from the kernel's own evaluator."""
    from OCP.gp import gp_Pnt, gp_Vec

    point, vectors = gp_Pnt(), [gp_Vec() for _ in range(5)]
    surface.D2(u, v, point, *vectors)
    du, dv, duu, dvv, duv = (vector.Magnitude() for vector in vectors)
    return du, dv, duu, duv, dvv


@unittest.skipIf(cq is None, "needs CadQuery")
class SplineDerivativeBoundTest(unittest.TestCase):
    def test_bounds_cover_the_kernels_derivatives_on_every_box(self):
        generator = random.Random(240)
        checked = 0
        shapes = [solid(name) for name in ("ellipsoid", "nurbs torus", "saddle block", "ruled loft", "smooth loft")]
        for shape in shapes + [_tilted(shapes[0])]:
            for surface in spline_faces(shape):
                bounds = cad._spline_bounds(surface)
                self.assertEqual(cad._surface_bounds(surface), bounds.whole)
                low_u, high_u, low_v, high_v = bounds.domain
                for _ in range(40):
                    size = generator.choice((1.0, 0.3, 0.05, 0.004))
                    u0 = generator.uniform(low_u, high_u)
                    v0 = generator.uniform(low_v, high_v)
                    u1 = min(high_u, u0 + size * (high_u - low_u) * generator.random())
                    v1 = min(high_v, v0 + size * (high_v - low_v) * generator.random())
                    local = bounds.local(u0, u1, v0, v1)
                    for i in range(4):
                        for j in range(4):
                            u, v = u0 + (u1 - u0) * i / 3, v0 + (v1 - v0) * j / 3
                            actual = second_derivatives(surface, u, v)
                            # Across a C0 line only the first derivatives are bounded.
                            for k in range(5 if len(local) == 5 else 2):
                                self.assertLessEqual(actual[k], local[k] * (1 + 1e-9) + 1e-9, (k, u, v))
                            checked += 1
                    for k in range(5):
                        self.assertLessEqual(local[k], bounds.whole[k] * (1 + 1e-9))
        self.assertGreater(checked, 5000)

    def test_a_small_box_is_bounded_near_its_real_derivatives_also_on_a_rational_face(self):
        # The cost: a bound three times the real second derivative is three
        # times the facets. Term-by-term bounds of a rational patch's
        # quotient rule stay that far off however small the box.
        for name in ("nurbs torus", "ellipsoid", "saddle block"):
            surface = next(spline_faces(solid(name)))
            bounds = cad._spline_bounds(surface)
            low_u, high_u, low_v, high_v = bounds.domain
            span_u, span_v = high_u - low_u, high_v - low_v
            for fu, fv in ((0.31, 0.27), (0.62, 0.41), (0.13, 0.55)):
                u0, v0 = low_u + fu * span_u, low_v + fv * span_v
                u1, v1 = u0 + span_u / 500, v0 + span_v / 500
                local = bounds.local(u0, u1, v0, v1)
                actual = [
                    max(second_derivatives(surface, u, v)[k] for u in (u0, u1) for v in (v0, v1)) for k in range(5)
                ]
                for k in (2, 3, 4):
                    self.assertLessEqual(local[k], 1.35 * actual[k] + 1e-6 * bounds.whole[k], (name, k, fu, fv))

    def test_bounds_vanish_towards_a_pole_as_an_analytic_spheres_do(self):
        # Along a pole the longitude's derivatives vanish. A bound over the
        # whole patch does not show it, and asks for facets around the pole
        # down to the numeric resolution.
        surface = next(spline_faces(solid("ellipsoid")))
        bounds = cad._spline_bounds(surface)
        low_u, high_u, low_v, high_v = bounds.domain
        near = bounds.local(low_u + 0.2, low_u + 0.3, high_v - 1e-4, high_v)
        self.assertLess(near[2], 0.01 * bounds.whole[2])
        self.assertLess(near[0], 0.01 * bounds.whole[0])

    def test_interpolation_error_bounds_real_triangles_within_and_across_c0_lines(self):
        generator = random.Random(7)
        checked = kinked = 0
        for name in ("ellipsoid", "nurbs torus", "saddle block", "folded sheet", "ruled loft"):
            for surface in spline_faces(solid(name)):
                bounds = cad._spline_bounds(surface)
                low_u, high_u, low_v, high_v = bounds.domain
                for _ in range(30):
                    size = generator.choice((1.0, 0.2, 0.02))
                    uv = []
                    cu, cv = generator.uniform(low_u, high_u), generator.uniform(low_v, high_v)
                    for _ in range(3):
                        u = min(high_u, max(low_u, cu + size * (high_u - low_u) * (generator.random() - 0.5)))
                        v = min(high_v, max(low_v, cv + size * (high_v - low_v) * (generator.random() - 0.5)))
                        uv.append((u, v))
                    us, vs = [p[0] for p in uv], [p[1] for p in uv]
                    local = bounds.local(min(us), max(us), min(vs), max(vs))
                    kinked += len(local) == 6
                    bound = cad._interpolation_error(uv, local)
                    corners = [surface.Value(*p).Coord() for p in uv]
                    for weights in ((0.2, 0.3, 0.5), (0.05, 0.9, 0.05), (0.6, 0.1, 0.3), (0.5, 0.5, 0.0)):
                        p = tuple(sum(w * q[k] for w, q in zip(weights, uv)) for k in (0, 1))
                        linear = tuple(sum(w * q[k] for w, q in zip(weights, corners)) for k in range(3))
                        self.assertLessEqual(math.dist(surface.Value(*p).Coord(), linear), bound + 1e-9, (name, uv))
                        checked += 1
        self.assertGreater(checked, 1000)
        self.assertGreater(kinked, 10)  # the sphere's and the fold's C0 knot lines were crossed

    def test_a_periodic_surface_is_bounded_over_any_period_and_across_its_seam(self):
        surface = next(spline_faces(solid("nurbs torus")))
        self.assertTrue(surface.IsUPeriodic() and surface.IsVPeriodic())
        bounds = cad._spline_bounds(surface)
        period = surface.UPeriod()
        inside = bounds.local(0.3, 0.4, 1.0, 1.1)
        shifted = bounds.local(0.3 + period, 0.4 + period, 1.0 - surface.VPeriod(), 1.1 - surface.VPeriod())
        for a, b in zip(inside, shifted):
            self.assertAlmostEqual(a, b, delta=1e-9 * max(1.0, a))
        across = bounds.local(period - 0.05, period + 0.05, 1.0, 1.1)
        for u in (period - 0.05, period - 0.01, period + 0.01, period + 0.05):
            actual = second_derivatives(surface, u, 1.05)
            for k in range(2):
                self.assertLessEqual(actual[k], across[k] * (1 + 1e-9))
        # A periodic axis has no end to clamp to; a clamped one has.
        self.assertEqual(bounds.clamp((period + 1.0, 2.0)), (period + 1.0, 2.0))
        sheet = cad._spline_bounds(next(spline_faces(solid("saddle block"))))
        low_u, high_u, low_v, high_v = sheet.domain
        self.assertEqual(sheet.clamp((low_u - 1.0, high_v + 1.0)), (low_u, high_v))


def _kinked_trim():
    """A quadratic 2D B-spline with a corner (a C0 knot) at t = 0.5."""
    from OCP.Geom2d import Geom2d_BSplineCurve
    from OCP.gp import gp_Pnt2d

    try:
        from OCP.TColgp import TColgp_Array1OfPnt2d as Points
        from OCP.TColStd import TColStd_Array1OfInteger as Integers, TColStd_Array1OfReal as Reals
    except ImportError:  # OCP 8 moved collection aliases.
        from OCP.collections import Array1_double as Reals, Array1_gp_Pnt2d as Points, Array1_int as Integers
    points = Points(1, 5)
    for i, (x, y) in enumerate(((0, 0), (0.5, 1), (1, 0), (1.5, 1), (2, 0)), 1):
        points.SetValue(i, gp_Pnt2d(x, y))
    knots, multiplicities = Reals(1, 3), Integers(1, 3)
    for i, (knot, multiplicity) in enumerate(((0, 3), (0.5, 2), (1, 3)), 1):
        knots.SetValue(i, knot)
        multiplicities.SetValue(i, multiplicity)
    return Geom2d_BSplineCurve(points, knots, multiplicities, 2)


def _rational_trim():
    """A rational cubic 2D Bezier."""
    from OCP.Geom2d import Geom2d_BezierCurve
    from OCP.gp import gp_Pnt2d

    try:
        from OCP.TColgp import TColgp_Array1OfPnt2d as Points
        from OCP.TColStd import TColStd_Array1OfReal as Reals
    except ImportError:  # OCP 8 moved collection aliases.
        from OCP.collections import Array1_double as Reals, Array1_gp_Pnt2d as Points
    points, weights = Points(1, 4), Reals(1, 4)
    for i, ((x, y), weight) in enumerate((((0, 0), 1.0), ((1, 2), 0.4), ((3, 2.5), 2.0), ((4, 0), 1.0)), 1):
        points.SetValue(i, gp_Pnt2d(x, y))
        weights.SetValue(i, weight)
    return Geom2d_BezierCurve(points, weights)


@unittest.skipIf(cq is None, "needs CadQuery")
class TrimBoundTest(unittest.TestCase):
    def curves(self):
        from OCP.Geom2d import Geom2d_Circle, Geom2d_Line
        from OCP.Geom2dAdaptor import Geom2dAdaptor_Curve
        from OCP.gp import gp_Ax2d, gp_Dir2d, gp_Pnt2d

        yield "kinked spline", Geom2dAdaptor_Curve(_kinked_trim()), (0.0, 1.0)
        yield "rational Bezier", Geom2dAdaptor_Curve(_rational_trim()), (0.0, 1.0)
        circle = Geom2d_Circle(gp_Ax2d(gp_Pnt2d(1, 2), gp_Dir2d(1, 0)), 3.0)
        yield "circle", Geom2dAdaptor_Curve(circle, 0.0, 2.0), (0.0, 2.0)
        line = Geom2d_Line(gp_Pnt2d(0, 0), gp_Dir2d(0.6, 0.8))
        yield "line", Geom2dAdaptor_Curve(line, 0.0, 5.0), (0.0, 5.0)

    def measured(self, curve, t0, t1, a, b, shift):
        """The real distance either way between the span and the chord."""
        span = [tuple(x + d for x, d in zip(curve.Value(t0 + (t1 - t0) * n / 96).Coord(), shift)) for n in range(97)]
        chord = [tuple(p + n / 96 * (q - p) for p, q in zip(a, b)) for n in range(97)]
        return max(
            max(cad._segment_distance(p, a, b) for p in span),
            max(min(math.dist(p, q) for q in span) for p in chord),
        )

    def test_the_reach_covers_the_span_and_its_chord_either_way(self):
        shift = (0.25, -3.0)
        for name, curve, (first, last) in self.curves():
            reach = cad._trim_reach(curve)
            for f0, f1 in ((0.0, 1.0), (0.1, 0.45), (0.3, 0.9), (0.49, 0.52), (0.6, 0.6)):
                t0, t1 = first + f0 * (last - first), first + f1 * (last - first)
                on_curve = [tuple(x + d for x, d in zip(curve.Value(t).Coord(), shift)) for t in (t0, t1)]
                # Mesh nodes on the curve, and a hair off it (the kernel's own gap).
                for gap in ((0.0, 0.0), (0.002, -0.001)):
                    a = on_curve[0]
                    b = (on_curve[1][0] + gap[0], on_curve[1][1] + gap[1])
                    bound = reach(t0, t1, a, b, shift)
                    slack = 1e-12 + 0.02 * math.dist(a, b) / 96  # the sampling's own step
                    self.assertLessEqual(self.measured(curve, t0, t1, a, b, shift), bound + slack, (name, f0, f1))

    def test_a_short_span_of_a_spline_trim_is_bounded_near_its_real_sag(self):
        # The whole-curve derivative bound this replaces was 50 to 400 times
        # the sag on spline trims of imported parts, and the solid was
        # triangulated ever finer to get under it.
        for name, curve, (first, last) in self.curves():
            if name == "line":
                continue
            reach = cad._trim_reach(curve)
            for start in (0.07, 0.31, 0.83):
                t0 = first + start * (last - first)
                t1 = t0 + (last - first) / 64
                a, b = (curve.Value(t).Coord() for t in (t0, t1))
                sag = self.measured(curve, t0, t1, a, b, (0.0, 0.0))
                self.assertLessEqual(reach(t0, t1, a, b, (0.0, 0.0)), 2.1 * sag + 1e-12, (name, start))
        line = dict((name, curve) for name, curve, _ in self.curves())["line"]
        self.assertLessEqual(cad._trim_reach(line)(1.0, 4.0, (0.6, 0.8), (2.4, 3.2), (0.0, 0.0)), 1e-12)


def _creased_prism():
    """A prism over a closed wire of a creased quadratic B-spline (a C0 knot
    at its middle) and a straight edge."""
    from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeEdge, BRepBuilderAPI_MakeFace, BRepBuilderAPI_MakeWire
    from OCP.BRepPrimAPI import BRepPrimAPI_MakePrism
    from OCP.Geom import Geom_BSplineCurve
    from OCP.gp import gp_Pnt, gp_Vec

    try:
        from OCP.TColgp import TColgp_Array1OfPnt as Points
        from OCP.TColStd import TColStd_Array1OfInteger as Integers, TColStd_Array1OfReal as Reals
    except ImportError:  # OCP 8 moved collection aliases.
        from OCP.collections import Array1_double as Reals, Array1_gp_Pnt as Points, Array1_int as Integers
    points = Points(1, 5)
    for i, (x, y) in enumerate(((0, 0), (3, 6), (6, 2), (9, 7), (12, 0)), 1):
        points.SetValue(i, gp_Pnt(x, y, 0))
    knots, multiplicities = Reals(1, 3), Integers(1, 3)
    for i, (knot, multiplicity) in enumerate(((0, 3), (0.5, 2), (1, 3)), 1):
        knots.SetValue(i, knot)
        multiplicities.SetValue(i, multiplicity)
    curve = BRepBuilderAPI_MakeEdge(Geom_BSplineCurve(points, knots, multiplicities, 2)).Edge()
    closing = BRepBuilderAPI_MakeEdge(gp_Pnt(12, 0, 0), gp_Pnt(0, 0, 0)).Edge()
    wire = BRepBuilderAPI_MakeWire(curve, closing).Wire()
    face = BRepBuilderAPI_MakeFace(wire).Face()
    return cq.Shape.cast(BRepPrimAPI_MakePrism(face, gp_Vec(0, 0, 5)).Shape())


@unittest.skipIf(cq is None, "needs CadQuery")
class SplineSolidMeshTest(unittest.TestCase):
    def test_spline_solids_are_watertight_and_enclose_the_solids_volume(self):
        for name, (_, _, exact) in SHAPES.items():
            with self.subTest(shape=name):
                surface = mesh(name)
                self.assertIn("BSPLINE", {face["kind"] for face in surface.source["face_bounds"]})
                self.assertEqual(unmatched_edges(surface.triangles), [])
                array = np.array(surface.triangles)
                normals = np.cross(array[:, 1] - array[:, 0], array[:, 2] - array[:, 0])
                self.assertGreater(np.linalg.norm(normals, axis=1).min(), 2e-10)
                volume = np.einsum("ij,ij->", array[:, 0], normals) / 6
                area = np.linalg.norm(normals, axis=1).sum() / 2
                if exact is None:
                    exact = solid(name).Volume()
                self.assertLessEqual(abs(volume - exact), area * surface.surface_error_mm)
                self.assertGreater(volume, 0)

    def test_the_declared_error_bounds_the_measured_deviation_both_ways(self):
        for name in SHAPES:
            with self.subTest(shape=name):
                surface, shape = mesh(name), solid(name)
                declared = surface.surface_error_mm
                self.assertLessEqual(declared, SHAPES[name][1])
                reach = 4 * SHAPES[name][1]
                measured = dict(
                    surface_to_mesh=distance_to_mesh(surface_points(shape), surface.triangles, reach),
                    edges_to_mesh=distance_to_mesh(edge_points(shape), surface.triangles, reach),
                    mesh_to_surface=distance_to_solid(shape, surface.triangles),
                )
                for direction, value in measured.items():
                    self.assertLessEqual(value, declared, (direction, measured))
                source = surface.source
                for face in source["face_bounds"]:
                    bound = (
                        face["interpolation_error_mm"]
                        + face["trim_error_mm"]
                        + face["edge_weld_mm"]
                        + source["cad_topology_error_mm"]
                        + source["numerical_guard_mm"]
                    )
                    self.assertLessEqual(bound, declared)
                self.assertEqual(sum(f["facets"] for f in source["face_bounds"]), len(surface.triangles))

    def test_the_proven_mesh_costs_no_more_than_three_times_the_kernels_unproven_one(self):
        # Measured 1.4 to 2.3 times. Bounds from whole Bezier patches, the
        # quotient rule term by term and a seed at a quarter of the budget
        # gave 5 to 16 times, and no mesh at all around a pole.
        for name in SHAPES:
            with self.subTest(shape=name):
                facets = len(mesh(name).triangles)
                self.assertLessEqual(facets, 3 * kernel_facets(solid(name), SHAPES[name][1]), facets)

    def test_a_face_that_is_only_c0_is_divided_and_its_crease_is_on_the_mesh(self):
        surface = mesh("folded sheet")
        # The sheet and the two extruded sides that cross its crease.
        self.assertEqual(surface.source["faces_divided_at_c0"], 4)
        # The crease runs along u = 1 of the sheet, from (6, 0, 2.5) to
        # (6, 8, 2.5), and is an edge of the mesh: no facet spans the fold.
        vertices = {p for triangle in surface.triangles for p in triangle}
        for end in ((6.0, 0.0, 2.5), (6.0, 8.0, 2.5)):
            self.assertLess(min(math.dist(end, p) for p in vertices), 1e-9)
        crease = np.array([(6.0, 8.0 * n / 50, 2.5) for n in range(51)])
        self.assertLessEqual(distance_to_mesh(crease, surface.triangles, 0.04), 1e-9)

    def test_an_undivided_c0_face_is_still_meshed_within_the_error(self):
        # The kernel's division is not relied on: across a C0 knot line the
        # facets are bounded from the first derivatives. That takes facets as
        # small as the error itself (the reason the faces are divided), so a
        # coarse error here.
        shape = solid("folded sheet")
        divided = export_surface_mesh(shape, max_error_mm=0.1)
        with mock.patch.object(cad, "_divide_at_c0", lambda shape: (shape, 0)):
            surface = export_surface_mesh(shape, max_error_mm=0.1)
        self.assertEqual(surface.source["faces_divided_at_c0"], 0)
        self.assertEqual(unmatched_edges(surface.triangles), [])
        self.assertLessEqual(surface.surface_error_mm, 0.1)
        self.assertGreater(len(surface.triangles), 5 * len(divided.triangles))
        measured = max(
            distance_to_mesh(surface_points(shape), surface.triangles, 0.4),
            distance_to_mesh(edge_points(shape), surface.triangles, 0.4),
            distance_to_solid(shape, surface.triangles),
        )
        self.assertLessEqual(measured, surface.surface_error_mm)

    def test_an_extrusion_of_a_c0_profile_is_divided_too(self):
        # Without a spline face: the sides of a prism over a creased spline
        # wire. Undivided, each whole side is bounded from first derivatives.
        profile = _creased_prism()
        surface = export_surface_mesh(profile, max_error_mm=0.01)
        self.assertGreater(surface.source["faces_divided_at_c0"], 0)
        self.assertNotIn("BSPLINE", {face["kind"] for face in surface.source["face_bounds"]})
        self.assertEqual(unmatched_edges(surface.triangles), [])
        self.assertLessEqual(len(surface.triangles), 3 * kernel_facets(profile, 0.01))
        measured = max(
            distance_to_mesh(surface_points(profile), surface.triangles, 0.04),
            distance_to_mesh(edge_points(profile), surface.triangles, 0.04),
            distance_to_solid(profile, surface.triangles),
        )
        self.assertLessEqual(measured, surface.surface_error_mm)

    def test_an_imported_step_file_and_loose_tolerances_are_charged_and_meshed(self):
        import os
        import tempfile

        from OCP.BRepBuilderAPI import BRepBuilderAPI_Copy
        from OCP.ShapeFix import ShapeFix_ShapeTolerance
        from OCP.TopAbs import TopAbs_SHAPE

        with tempfile.TemporaryDirectory(prefix="camlib spline ") as directory:
            path = os.path.join(directory, "bored saddle.step")
            cq.exporters.export(cq.Workplane("XY").add(solid("bored saddle")), path)
            imported = cq.importers.importStep(path).val()
        loose = cq.Shape.cast(BRepBuilderAPI_Copy(imported.wrapped, True, False).Shape())
        ShapeFix_ShapeTolerance().SetTolerance(loose.wrapped, 2e-3, TopAbs_SHAPE)
        for shape, requested, topology in ((imported, 0.01, None), (loose, 0.02, 4e-3)):
            surface = export_surface_mesh(shape, max_error_mm=requested)
            self.assertEqual(unmatched_edges(surface.triangles), [])
            self.assertLessEqual(surface.surface_error_mm, requested)
            if topology is not None:
                self.assertAlmostEqual(surface.source["cad_topology_error_mm"], topology)
            measured = max(
                distance_to_mesh(surface_points(shape), surface.triangles, 4 * requested),
                distance_to_mesh(edge_points(shape), surface.triangles, 4 * requested),
                distance_to_solid(shape, surface.triangles),
            )
            self.assertLessEqual(measured, surface.surface_error_mm)
        with self.assertRaisesRegex(ValueError, "CAD topology/numerical error exceeds requested surface error"):
            export_surface_mesh(loose, max_error_mm=0.003)


@unittest.skipIf(cq is None, "needs CadQuery")
class MesherMismatchTest(unittest.TestCase):
    """The kernel's triangulation is a seed that must match every face's
    trims. Where it does not, the whole solid is triangulated again; a face
    is never meshed on its own, which would leave its edges unshared."""

    def failing(self, attempts, error=None):
        """`_face_data` that refuses the first face of the solid on the
        first `attempts` triangulations."""
        original = cad._face_data
        state = dict(passes=0, first=None)

        def face_data(face, cancel, spline=None):
            if state["first"] is None or face.wrapped.IsSame(state["first"]):
                state["first"] = face.wrapped
                state["passes"] += 1
                if state["passes"] <= attempts:
                    raise error or cad._MesherMismatch("CAD trim chains do not cover mesh boundary exactly once")
            return original(face, cancel, spline)

        return mock.patch.object(cad, "_face_data", face_data), state

    def test_the_solid_is_triangulated_again_finer_then_with_tightened_tolerances(self):
        shape = solid("ruled loft")
        plain = mesh("ruled loft")
        for attempts, tightened in ((1, False), (3, False), (4, True), (6, True)):
            with self.subTest(attempts=attempts):
                patch, state = self.failing(attempts)
                with patch:
                    surface = export_surface_mesh(shape, max_error_mm=0.01)
                self.assertEqual(state["passes"], attempts + 1)
                self.assertEqual(surface.source["tolerances_tightened_for_meshing"], tightened)
                expected = plain.source["seed_deflection_mm"] / 2 ** (attempts % 4)
                self.assertAlmostEqual(surface.source["seed_deflection_mm"], expected)
                self.assertEqual(unmatched_edges(surface.triangles), [])
                self.assertLessEqual(surface.surface_error_mm, 0.01)
                # The error is charged the solid's own tolerances either way.
                self.assertEqual(surface.source["cad_topology_error_mm"], plain.source["cad_topology_error_mm"])

    def test_a_face_no_triangulation_matches_is_refused_by_name(self):
        patch, state = self.failing(10**6)
        with (
            patch,
            self.assertRaisesRegex(
                ValueError,
                r"the CAD mesher cannot triangulate face 0 \(BSPLINE, [0-9.]+ mm2\) consistently with its trims: "
                r"CAD trim chains do not cover mesh boundary exactly once, at every triangulation tried",
            ),
        ):
            export_surface_mesh(solid("ruled loft"), max_error_mm=0.01)
        self.assertEqual(state["passes"], 8)

    def test_other_errors_of_a_face_are_not_retried_and_name_the_face(self):
        patch, state = self.failing(10**6, ValueError("nonfinite CAD derivative bound"))
        with patch, self.assertRaisesRegex(ValueError, r"^face 0 \(BSPLINE, [0-9.]+ mm2\): nonfinite CAD derivative"):
            export_surface_mesh(solid("ruled loft"), max_error_mm=0.01)
        self.assertEqual(state["passes"], 1)

    def test_cancellation_is_not_taken_for_a_mesher_mismatch(self):
        calls = []

        def cancel():
            calls.append(None)
            return len(calls) > 3

        with self.assertRaisesRegex(ValueError, "CAD surface export cancelled"):
            export_surface_mesh(solid("ruled loft"), max_error_mm=0.01, cancel=cancel)

    def test_unbounded_surfaces_and_facet_caps_name_the_face(self):
        with self.assertRaisesRegex(
            ValueError, r"^face 1 \(REVOLUTION, [0-9.]+ mm2\): no parametric interpolation bound for CAD surface"
        ):
            export_surface_mesh(spline_solids.revolved_spline(), max_error_mm=0.01)
        with self.assertRaisesRegex(
            ValueError,
            r"CAD surface exceeds max_facets 2000 at requested error; face \d+ \(BSPLINE, [0-9.]+ mm2\) has the most",
        ):
            export_surface_mesh(solid("ellipsoid"), max_error_mm=0.002, max_facets=2000)


if __name__ == "__main__":
    unittest.main()
