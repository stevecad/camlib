"""Roughing parts with B-spline faces imported through the CadQuery adapter (#240).

An ellipsoid (one rational B-spline face with poles) and a saddle block with a
bore (polynomial spline faces trimmed by intersection curves, beside extruded
and cylindrical ones). Roughing takes the adapter's mesh; the result is
checked against a mesh the adapter had no part in: the CAD kernel's own
triangulation, face by face, at a fifth of the error it is declared with.
"""

import unittest

from camlib.material import Options, SurfaceTarget, Verifier
from camlib.operations import RasterClearing, Roughing
from camlib.simulation import OperationUpdate, Simulation, StockSpec
from camlib.surface import SurfaceMesh
from camlib.tools import EndMill, HolderSection, ToolAssembly

try:
    import cadquery as cq
except ImportError:  # pragma: no cover - CI installs CadQuery
    cq = None

TOOL = ToolAssembly(
    "em6",
    EndMill("em6", diameter=6, flute_length=30, overall_length=60),
    40,
    60,
    (HolderSection(40, 60, 12, 12),),
)
CUT = dict(feed_mm_min=900, plunge_mm_min=250, spindle_rpm=9000, tool_number=1, length_register=1, coolant="flood")


def kernel_mesh(solid, declared=0.01):
    """The kernel's triangulation of each face, at a fifth of `declared`."""
    triangles = []
    for face in solid.Faces():
        vertices, indices = face.tessellate(declared / 5, 0.1)
        for triangle in indices:
            a, b, c = (vertices[i] for i in triangle)
            # The kernel's facets on a pole have no area and hold no surface.
            if (b - a).cross(c - a).Length > 2e-10:
                triangles.append((a.toTuple(), b.toTuple(), c.toTuple()))
    return SurfaceMesh(triangles, declared, dict(adapter="kernel triangulation, face by face (test)"))


def parts():
    from support import spline_solids

    # 24 x 24 x 14.4 mm, its equator at Z-8: the upper half is roughed.
    ellipsoid = spline_solids.ellipsoid(12, 0.6).translate((20, 20, -8))
    # 20 x 16 mm, its spline top between Z-6.4 and Z-0.4, bored through.
    saddle = spline_solids.bored_saddle().translate((14, 12, -4))
    return (
        ("ellipsoid", ellipsoid, StockSpec((0, 0, -16), (40, 40, 0), 0.5), -8.0),
        ("bored saddle", saddle, StockSpec((0, 0, -15), (28, 24, 0), 0.5), -14.5),
    )


@unittest.skipIf(cq is None, "needs CadQuery")
class SplinePartRoughingTest(unittest.TestCase):
    def test_spline_parts_rough_layer_by_layer_and_keep_the_part(self):
        for name, solid, stock, floor in parts():
            with self.subTest(part=name):
                rough = Roughing.from_solid(
                    "rough",
                    TOOL,
                    solid,
                    top_z_mm=0,
                    floor_z_mm=floor,
                    stepdown_mm=2,
                    clearance_z_mm=10,
                    clearing=RasterClearing(3, routing="economical"),
                    radial_allowance_mm=0.3,
                    bottom_allowance_mm=0.2,
                    omit_empty_cuts=True,
                    **CUT,
                )
                self.assertIn("BSPLINE", {face["kind"] for face in rough.target.source["face_bounds"]})
                self.assertLessEqual(rough.target.surface_error_mm, 0.01)
                report = None
                with Simulation(
                    stock=stock,
                    tools=(TOOL,),
                    operations=(rough,),
                    revision="1",
                    initial_position_mm=(0, 0, 10),
                ) as sim:
                    moves = 0
                    for update in sim.run_operations():
                        moves += 1
                        if isinstance(update, OperationUpdate) and update.stage == "end":
                            report = update.report
                    self.assertTrue(sim.state.finished)
                    target = SurfaceTarget(name.replace(" ", "-"), "1", kernel_mesh(solid), stock.min_mm[2])
                    material = Verifier(target, Options(max_findings=1)).check(sim.state)
                self.assertGreater(moves, 50)
                self.assertEqual(report["completed_layers"], len(report["layers"]))
                self.assertGreater(report["completed_layers"], 3)
                # No stock cell is certainly below the part, and material came
                # off: every layer but those the part fills removed some.
                self.assertEqual(material["gouge"]["definite_cells"], 0)
                self.assertTrue(material["complete"])


if __name__ == "__main__":
    unittest.main()
