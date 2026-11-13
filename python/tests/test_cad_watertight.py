"""The CadQuery adapter meshes a solid watertight, within its declared error (#241).

The adapter refines the CAD kernel's triangulation until a proven
interpolation bound holds. It used to bisect each facet on its own and to
evaluate the nodes of a CAD edge on each face's own surface: neighbours were
left with T-junctions (a vertex on an edge the neighbour kept whole) and the
two faces of an edge with different points, 0.3 um apart on a typical part.
Projected, those seams are the slivers and near-touching boundaries that made
roughing refuse a layer.

Checked here without the adapter's own bookkeeping: facet edges are matched
on their bitwise coordinates, the enclosed volume is compared with the CAD
solid's, and true surface points evaluated through OCP (inside each face's
trims, off the mesher's nodes) are measured against the facets, and facet
points against the CAD faces, with exact point-to-triangle distances.
"""

from collections import Counter
from functools import lru_cache
import math
import os
import unittest

try:
    import cadquery as cq
    import numpy as np
except ImportError:  # pragma: no cover - CI installs CadQuery
    cq = None

from camlib.cadquery import export_surface_mesh


def _curved_part():
    from support import curved_solid

    return curved_solid.target().val()


def _filleted_box():
    return cq.Workplane("XY").box(20, 15, 8, centered=(True, True, False)).edges().fillet(2).val()


# name: (solid, requested error). Every analytic surface the adapter bounds,
# trimmed faces, seams, poles (sphere, cone apex, the fillet corners), a
# reversed face (the bore) and a solid in general position.
SHAPES = {
    "sphere": (lambda: cq.Workplane("XY").sphere(2).val().translate((0, 0, 2)), 0.01),
    "torus": (lambda: cq.Solid.makeTorus(5, 1).translate((0, 0, 1)), 0.01),
    "filleted box": (_filleted_box, 0.005),
    "filleted box, fine": (_filleted_box, 0.001),
    "frustum": (lambda: cq.Solid.makeCone(4, 2, 5), 0.002),
    "cone": (lambda: cq.Solid.makeCone(4, 0, 5), 0.005),
    "cylinder": (lambda: cq.Solid.makeCylinder(3, 5), 0.001),
    "bored block": (lambda: cq.Workplane("XY").box(12, 10, 6).faces(">Z").workplane().hole(5).val(), 0.002),
    "curved part": (_curved_part, 0.005),
    "tilted filleted box": (
        lambda: _filleted_box().rotate((0, 0, 0), (1, 2, 3), 31).translate((200, -50, 17)),
        0.005,
    ),
}


@lru_cache(None)
def solid(name):
    return SHAPES[name][0]()


@lru_cache(None)
def mesh(name):
    return export_surface_mesh(solid(name), max_error_mm=SHAPES[name][1])


def unmatched_edges(triangles):
    """Facet sides without exactly one facet running the other way between
    the same two points (bitwise): a watertight, oriented mesh has none."""
    sides = Counter()
    for a, b, c in triangles:
        sides[(a, b)] += 1
        sides[(b, c)] += 1
        sides[(c, a)] += 1
    return [side for side, n in sides.items() if n != 1 or sides.get((side[1], side[0]), 0) != 1]


def distances_to_triangles(p, triangles):
    """Exact distance from p to each triangle of an (n, 3, 3) array
    (Ericson, Real-Time Collision Detection, 5.1.5)."""
    a, b, c = triangles[:, 0], triangles[:, 1], triangles[:, 2]
    ab, ac = b - a, c - a
    ap, bp, cp = p - a, p - b, p - c
    d1, d2 = (ab * ap).sum(1), (ac * ap).sum(1)
    d3, d4 = (ab * bp).sum(1), (ac * bp).sum(1)
    d5, d6 = (ab * cp).sum(1), (ac * cp).sum(1)
    va, vb, vc = d3 * d6 - d5 * d4, d5 * d2 - d1 * d6, d1 * d4 - d3 * d2

    def ratio(numerator, denominator):
        return np.divide(numerator, denominator, out=np.zeros_like(numerator), where=denominator != 0)

    total = va + vb + vc
    closest = a + ab * ratio(vb, total)[:, None] + ac * ratio(vc, total)[:, None]
    for region, point in (
        ((va <= 0) & (d4 - d3 >= 0) & (d5 - d6 >= 0), b + (c - b) * ratio(d4 - d3, d4 - d3 + d5 - d6)[:, None]),
        ((vb <= 0) & (d2 >= 0) & (d6 <= 0), a + ac * ratio(d2, d2 - d6)[:, None]),
        ((vc <= 0) & (d1 >= 0) & (d3 <= 0), a + ab * ratio(d1, d1 - d3)[:, None]),
        ((d6 >= 0) & (d5 <= d6), c),
        ((d3 >= 0) & (d4 <= d3), b),
        ((d1 <= 0) & (d2 <= 0), a),
    ):
        closest[region] = point[region]
    return np.linalg.norm(closest - p, axis=1)


def distance_to_mesh(points, triangles, reach):
    """The largest distance from a point to the mesh. Only facets whose box,
    grown by `reach`, holds the point are measured: none there is a failure."""
    array = np.array(triangles)
    low, high = array.min(1) - reach, array.max(1) + reach
    worst = 0.0
    for p in points:
        near = np.all((low <= p) & (p <= high), axis=1)
        if not near.any():
            return math.inf
        worst = max(worst, distances_to_triangles(p, array[near]).min())
    return worst


def surface_points(shape, n=10):
    """True points of every face: a UV grid inside its trims (OCP's
    classifier), at offsets that are not the mesher's own nodes."""
    from OCP.BRepAdaptor import BRepAdaptor_Surface
    from OCP.BRepClass import BRepClass_FaceClassifier
    from OCP.gp import gp_Pnt2d
    from OCP.TopAbs import TopAbs_IN, TopAbs_ON

    points = []
    for face in shape.Faces():
        surface = BRepAdaptor_Surface(face.wrapped)
        u0, u1 = surface.FirstUParameter(), surface.LastUParameter()
        v0, v1 = surface.FirstVParameter(), surface.LastVParameter()
        for i in range(n + 1):
            for j in range(n + 1):
                u = u0 + (u1 - u0) * min(1.0, (i + 0.381966 * (i < n)) / n)
                v = v0 + (v1 - v0) * min(1.0, (j + 0.618034 * (j < n)) / n)
                if BRepClass_FaceClassifier(face.wrapped, gp_Pnt2d(u, v), 1e-9).State() in (TopAbs_IN, TopAbs_ON):
                    points.append(surface.Value(u, v).Coord())
    return np.array(points)


def edge_points(shape, n=24):
    """True points of every CAD edge curve (where two faces meet)."""
    from OCP.BRep import BRep_Tool
    from OCP.BRepAdaptor import BRepAdaptor_Curve

    points = []
    for edge in shape.Edges():
        if BRep_Tool.Degenerated_s(edge.wrapped):
            continue
        curve = BRepAdaptor_Curve(edge.wrapped)
        a, b = curve.FirstParameter(), curve.LastParameter()
        points.extend(curve.Value(a + (b - a) * (i + 0.381966 * (i < n)) / n).Coord() for i in range(n + 1))
    return np.array(points)


def distance_to_solid(shape, triangles, count=120):
    """Facet corners, centroids and edge midpoints against the CAD faces."""
    from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeVertex
    from OCP.BRepExtrema import BRepExtrema_DistShapeShape
    from OCP.gp import gp_Pnt

    array = np.array(triangles)
    picked = array[:: max(1, len(array) // count)]
    points = np.concatenate([picked[:, 0], picked.mean(1), (picked[:, 0] + picked[:, 1]) / 2])
    shell = cq.Compound.makeCompound(shape.Faces())
    worst = 0.0
    for p in points:
        vertex = BRepBuilderAPI_MakeVertex(gp_Pnt(*map(float, p))).Vertex()
        distance = BRepExtrema_DistShapeShape(vertex, shell.wrapped)
        assert distance.IsDone()
        worst = max(worst, distance.Value())
    return worst


@unittest.skipIf(cq is None, "needs CadQuery")
class WatertightCadMeshTest(unittest.TestCase):
    def test_every_facet_edge_is_shared_by_two_facets_with_identical_ends(self):
        for name in SHAPES:
            with self.subTest(shape=name):
                triangles = mesh(name).triangles
                self.assertEqual(unmatched_edges(triangles), [])
                array = np.array(triangles)
                # No zero-area facet (twice the area, as the engine measures it).
                normals = np.cross(array[:, 1] - array[:, 0], array[:, 2] - array[:, 0])
                self.assertGreater(np.linalg.norm(normals, axis=1).min(), 2e-10)
                # Closed and oriented outwards: the divergence theorem gives
                # the solid's volume, within the declared error over its area.
                volume = np.einsum("ij,ij->", array[:, 0], normals) / 6
                area = np.linalg.norm(normals, axis=1).sum() / 2
                exact = solid(name).Volume()
                self.assertLessEqual(abs(volume - exact), area * mesh(name).surface_error_mm)
                self.assertGreater(volume, 0)

    def test_planar_solids_were_and_stay_watertight(self):
        # Planar faces with straight edges take the adapter's exact path
        # (the kernel's facets; nothing is refined): unchanged, and closed.
        stepped = (
            cq.Workplane("XY")
            .polyline([(0, 0), (30, 0), (30, 12), (14, 12), (14, 20), (0, 20)])
            .close()
            .extrude(8)
            .faces(">Z")
            .workplane()
            .center(6, 6)
            .rect(5, 4)
            .cutThruAll()
        )
        for shape in (cq.Workplane("XY").box(10, 8, 5).val(), stepped.val()):
            surface = export_surface_mesh(shape)
            self.assertEqual(surface.source["adapter"], "planar_cad_solid_facets_v1")
            self.assertEqual(unmatched_edges(surface.triangles), [])
            array = np.array(surface.triangles)
            normals = np.cross(array[:, 1] - array[:, 0], array[:, 2] - array[:, 0])
            self.assertAlmostEqual(np.einsum("ij,ij->", array[:, 0], normals) / 6, shape.Volume(), places=6)

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
                # The report's parts add up to the declared error.
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

    def test_a_pole_is_one_vertex_and_its_collapsed_facets_are_left_out(self):
        # A sphere's poles and a cone's apex are whole edges in UV. Their
        # facets are segments in 3D: left out (the neighbours share the
        # segment), not turned into slivers a hair off the surface.
        for name, poles in (("sphere", ((0, 0, 0), (0, 0, 4))), ("cone", ((0, 0, 5),))):
            with self.subTest(shape=name):
                surface = mesh(name)
                self.assertGreater(sum(f["collapsed_pole_facets"] for f in surface.source["face_bounds"]), 0)
                vertices = {p for t in surface.triangles for p in t}
                for pole in poles:
                    near = [p for p in vertices if math.dist(p, pole) < 1e-6]
                    self.assertEqual(len(near), 1, near)

    def test_band_bounds_of_a_sphere_and_a_cone_cover_the_actual_interpolation_error(self):
        from OCP.BRepAdaptor import BRepAdaptor_Surface
        from camlib._cad_surface import _interpolation_error, _local_bounds, _surface_bounds

        sphere = cq.Workplane("XY").sphere(3).val().rotate((0, 0, 0), (1, 2, 3), 31).translate((20, -5, 7))
        cone = cq.Solid.makeCone(4, 0, 5).rotate((0, 0, 0), (3, 1, 2), 47)
        state = 12345
        for shape in (sphere, cone):
            face = next(f for f in shape.Faces() if f.geomType() != "PLANE")
            surface = BRepAdaptor_Surface(face.wrapped)
            local = _local_bounds(surface)
            whole = _surface_bounds(surface)
            u0, u1 = surface.FirstUParameter(), surface.LastUParameter()
            v0, v1 = surface.FirstVParameter(), surface.LastVParameter()
            tighter = 0
            for _ in range(400):
                uv = []
                for _ in range(3):
                    state = (state * 6364136223846793005 + 1442695040888963407) % 2**64
                    fu = (state >> 11) / 2**53
                    state = (state * 6364136223846793005 + 1442695040888963407) % 2**64
                    fv = ((state >> 11) / 2**53) ** 3  # many triangles at a pole/apex end
                    uv.append((u0 + (u1 - u0) * fu, v1 - (v1 - v0) * fv))
                v = [p[1] for p in uv]
                bounds = local(min(v), max(v))
                self.assertTrue(all(x <= y + 1e-12 for x, y in zip(bounds, whole)), (bounds, whole))
                bound = _interpolation_error(uv, bounds)
                tighter += bound < 0.5 * _interpolation_error(uv, whole)
                corners = [surface.Value(*p).Coord() for p in uv]
                for weights in (
                    (0.2, 0.3, 0.5),
                    (0.05, 0.9, 0.05),
                    (0.6, 0.1, 0.3),
                    (1 / 3, 1 / 3, 1 / 3),
                    (0.5, 0.5, 0),
                ):
                    p = tuple(sum(w * q[k] for w, q in zip(weights, uv)) for k in (0, 1))
                    flat = tuple(sum(w * q[k] for w, q in zip(weights, corners)) for k in range(3))
                    self.assertLessEqual(math.dist(surface.Value(*p).Coord(), flat), bound + 1e-9)
            self.assertGreater(tighter, 20)

    def test_roughing_plans_the_layers_whose_shadow_was_refused(self):
        # With the facet-by-facet refinement these two were refused: the
        # sphere at its first layer (Z3.5, "constructed short edge cannot be
        # removed": a sliver between a T-junction's two sides), the torus at
        # Z0.5 ("filled components overlap or nest outside holes").
        from camlib.material import Options, SurfaceTarget, Verifier
        from camlib.operations import RasterClearing, Roughing
        from camlib.simulation import OperationUpdate, Simulation, StockSpec
        from camlib.tools import EndMill, HolderSection, ToolAssembly

        cases = (
            ("sphere", cq.Workplane("XY").sphere(2).val().translate((0, 0, 2)), 0.01, 2, 0.5, (3.5, 1.0)),
            ("torus", cq.Solid.makeTorus(5, 2).translate((0, 0, 2)), 0.005, 3, 1.0, (0.5,)),
        )
        for name, shape, error, diameter, stepdown, levels in cases:
            with self.subTest(shape=name):
                surface = export_surface_mesh(shape, max_error_mm=error)
                self.assertEqual(unmatched_edges(surface.triangles), [])
                box = shape.BoundingBox()
                tool = ToolAssembly(
                    "em",
                    EndMill("em", diameter=diameter, flute_length=40, overall_length=70),
                    50,
                    80,
                    (HolderSection(50, 80, 10, 10),),
                )
                margin = 2 * diameter
                rough = Roughing(
                    "rough",
                    tool,
                    surface,
                    top_z_mm=box.zmax + 0.5,
                    floor_z_mm=box.zmin + 0.25,
                    stepdown_mm=stepdown,
                    clearance_z_mm=box.zmax + 10,
                    clearing=RasterClearing(diameter / 2, routing="economical"),
                    radial_allowance_mm=0.3,
                    bottom_allowance_mm=0.2,
                    omit_empty_cuts=True,
                    feed_mm_min=900,
                    plunge_mm_min=250,
                    spindle_rpm=6000,
                    tool_number=1,
                    length_register=1,
                    coolant="flood",
                )
                report = None
                with Simulation(
                    stock=StockSpec(
                        (box.xmin - margin, box.ymin - margin, box.zmin),
                        (box.xmax + margin, box.ymax + margin, box.zmax + 0.5),
                        0.25,
                    ),
                    tools=(tool,),
                    operations=(rough,),
                    revision="1",
                    initial_position_mm=(box.xmin - margin, box.ymin - margin, box.zmax + 10),
                    executable=os.environ.get("CAMLIB_TEST_ENGINE"),
                ) as sim:
                    for update in sim.run_operations():
                        if isinstance(update, OperationUpdate) and update.stage == "end":
                            report = update.report
                    self.assertTrue(sim.state.finished)
                    material = Verifier(SurfaceTarget(name, "1", surface, box.zmin), Options(max_findings=1)).check(
                        sim.state
                    )
                    removed = sim.state.volume()["removed_lower_mm3"]
                self.assertEqual(report["completed_layers"], len(report["layers"]))
                planned = [layer["shadow"]["tip_z_mm"] for layer in report["layers"]]
                for level in levels:
                    self.assertTrue(any(abs(z - level) < 0.01 for z in planned), (level, planned))
                # A watertight mesh leaves no slivers to fill.
                self.assertEqual(sum(layer["shadow"]["filled_holes"] for layer in report["layers"]), 0)
                self.assertTrue(material["complete"])
                self.assertEqual(material["gouge"]["definite_cells"], 0)
                self.assertGreater(removed, 10)


if __name__ == "__main__":
    unittest.main()
