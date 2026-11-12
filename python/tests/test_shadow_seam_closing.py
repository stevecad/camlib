"""A mesh with seams below its declared error plans for every shadow consumer (#241).

The part is written by hand: a drafted boss in two faces that share a circle
and were subdivided on their own (180 chords against 240, one turned by
2e-8 of a turn, 0.75 nm along the circle), every vertex on the true circle.
The faces differ by at most 0.91 um along it, and where their vertices should
be one they are a hair apart: the slivers between them have an edge the
planar kernel cannot take ("constructed short edge cannot be removed"), so
every layer through the seam was refused. Declaring a 2 um surface error, the
mesh roughs, rest roughs, takes flat and bull-nose waterline levels and gives
silhouettes; the shadow report says what was closed. Declaring no error it
claims its seams are real, and is refused with the remedy.

The checks do not use the planner: the material verifier compares the stock
with the mesh, and an even-odd test written here puts sampled facet points
inside each silhouette.
"""

import math
import os
import unittest

from camlib import Engine
from camlib.material import Options, SurfaceTarget, Verifier
from camlib.operations import OffsetClearing, RasterClearing, RestRoughing, Roughing, Waterline
from camlib.simulation import OperationUpdate, Simulation, StockSpec
from camlib.surface import SurfaceMesh
from camlib.tools import BullEndMill, EndMill, HolderSection, ToolAssembly

WALL, CROWN = 180, 240
SEAM_MM = 6 * (1 - math.cos(math.pi / WALL))
CUT = dict(feed_mm_min=900, plunge_mm_min=250, spindle_rpm=8000, coolant="flood")


def two_faces(error):
    def at(r, turn, z):
        return (r * math.cos(math.tau * turn), r * math.sin(math.tau * turn), z)

    triangles = []
    for i in range(WALL):
        a, b = i / WALL, (i + 1) / WALL
        triangles.append((at(10, a, 0), at(10, b, 0), at(6, b, 4)))
        triangles.append((at(10, a, 0), at(6, b, 4), at(6, a, 4)))
    twist = 2e-8
    for i in range(CROWN):
        a, b = i / CROWN + twist, (i + 1) / CROWN + twist
        triangles.append((at(6, a, 4), at(6, b, 4), (0.0, 0.0, 6.0)))
    return SurfaceMesh(triangles, error, dict(model="two separately subdivided faces"))


def assembly(name, cutter):
    return ToolAssembly(name, cutter, 40, 70, (HolderSection(40, 70, 12, 12),))


ROUGHER = assembly("em6", EndMill("em6", diameter=6, flute_length=30, overall_length=60))
SMALLER = assembly("em3", EndMill("em3", diameter=3, flute_length=30, overall_length=60))
FLAT = assembly("flat4", EndMill("flat4", diameter=4, flute_length=30, overall_length=60))
BULL = assembly("bull4", BullEndMill("bull4", corner_radius=1, diameter=4, flute_length=30, overall_length=60))


def operations(surface):
    rough = Roughing(
        "rough",
        ROUGHER,
        surface,
        top_z_mm=6.5,
        floor_z_mm=0.25,
        stepdown_mm=1.25,
        clearance_z_mm=15,
        clearing=OffsetClearing(3.0),
        radial_allowance_mm=0.3,
        bottom_allowance_mm=0.2,
        omit_empty_cuts=True,
        tool_number=1,
        length_register=1,
        **CUT,
    )
    rest = RestRoughing(
        "rest",
        SMALLER,
        surface,
        top_z_mm=6.5,
        floor_z_mm=0.25,
        stepdown_mm=0.625,
        clearance_z_mm=15,
        clearing=RasterClearing(1.2),
        radial_allowance_mm=0.15,
        bottom_allowance_mm=0.2,
        tool_number=2,
        length_register=2,
        **CUT,
    )
    levels = [
        Waterline(
            f"levels-{tool.id}",
            tool,
            surface,
            0,
            6.2,
            0.4,
            0.8,
            15,
            stock_to_leave_mm=0.05,
            tool_number=number,
            length_register=number,
            **CUT,
        )
        for number, tool in ((3, FLAT), (4, BULL))
    ]
    return (rough, rest, *levels)


def run(surface):
    reports = {}
    with Simulation(
        stock=StockSpec((-16, -16, 0), (16, 16, 6.5), 0.25),
        tools=(ROUGHER, SMALLER, FLAT, BULL),
        operations=operations(surface),
        revision="1",
        initial_position_mm=(-16, -16, 15),
        executable=os.environ.get("CAMLIB_TEST_ENGINE"),
    ) as sim:
        for update in sim.run_operations():
            if isinstance(update, OperationUpdate) and update.stage == "end":
                reports[update.operation] = update.report
        assert sim.state.finished
        material = Verifier(SurfaceTarget("boss", "1", surface, 0), Options(max_findings=1)).check(sim.state)
    return reports, material


def covered(region, x, y):
    inside = False
    for component in region["components"]:
        for ring in (component["outer"], *component["holes"]):
            vertices = ring["vertices"]
            for a, b in zip(vertices, vertices[1:] + vertices[:1]):
                assert a[2] == 0, "a silhouette has straight edges only"
                if (a[1] > y) != (b[1] > y) and x < a[0] + (y - a[1]) / (b[1] - a[1]) * (b[0] - a[0]):
                    inside = not inside
    return inside


class ShadowSeamClosingTest(unittest.TestCase):
    def test_a_mesh_claiming_no_error_is_refused_with_the_remedy(self):
        exact = two_faces(0.0)
        with self.assertRaisesRegex(ValueError, "declare a surface error of at least half the widest seam"):
            with Engine() as engine:
                engine.silhouette_region(exact, 2.0, max_error_mm=0.01)
        with self.assertRaisesRegex(Exception, "not watertight"):
            run(exact)

    def test_seams_below_the_declared_error_are_closed_for_every_consumer(self):
        error = 0.002
        self.assertLess(SEAM_MM, error)
        surface = two_faces(error)
        reports, material = run(surface)
        self.assertEqual(set(reports), {"rough", "rest", "levels-flat4", "levels-bull4"})
        for name in ("rough", "rest"):
            report = reports[name]
            self.assertEqual(report["completed_layers"], len(report["layers"]))
            self.assertGreaterEqual(len(report["layers"]), 4)
            closed = 0
            for layer in report["layers"]:
                shadow = layer["shadow"]
                # With a cutter the slivers between the two chord sets are
                # holes too small for it and are filled; the closing is
                # reported where it joined something the filling did not.
                if "seam_closing" in shadow:
                    closed += 1
                    self.assertEqual(shadow["seam_closing"]["distance_mm"], error)
                    # The lunes between the two chord sets, and the rounding.
                    self.assertTrue(0 <= shadow["seam_closing"]["added_area_mm2"] < 0.05, shadow)
                self.assertLess(shadow["filled_hole_area_mm2"], 0.05, shadow)
        for name in ("levels-flat4", "levels-bull4"):
            self.assertGreater(reports[name]["levels"], 5, reports[name])
        # Nothing cut into the part, anywhere, by any of the four.
        self.assertTrue(material["complete"])
        self.assertEqual(material["gouge"]["definite_cells"], 0)
        self.assertEqual(material["gouge"]["volume_mm3"][0], 0)

    def test_every_silhouette_holds_the_part_at_or_above_its_depth(self):
        error = 0.002
        surface = two_faces(error)
        samples = [
            tuple(a[k] + u / 4 * (b[k] - a[k]) + v / 4 * (c[k] - a[k]) for k in range(3))
            for a, b, c in surface.triangles[::7]
            for u in range(5)
            for v in range(5 - u)
        ]
        with Engine() as engine:
            for z in (0.0, 1.0, 3.9, 3.9995, 4.0, 4.0005, 4.5, 5.9):
                out = engine.silhouette_region(surface, z, max_error_mm=0.01)
                region, report = out["region"], out["report"]
                self.assertEqual((report["components"], report["holes"]), (1, 0), z)
                above = [p for p in samples if p[2] >= z]
                self.assertGreater(len(above), 20)
                for x, y, _ in above:
                    self.assertTrue(covered(region, x, y), (z, x, y))
                # And it is not grown beyond the part by more than its error:
                # the part at or above the level is inside the cone's circle
                # at the level it protects from.
                level = report["protected_from_z_mm"]
                radius = 10 - max(level, 0) if level <= 4 else 6 * (6 - level) / 2
                for x, y, _ in region["components"][0]["outer"]["vertices"]:
                    self.assertLessEqual(math.hypot(x, y), radius + report["boundary_error_mm"] + 1e-9)


if __name__ == "__main__":
    unittest.main()
