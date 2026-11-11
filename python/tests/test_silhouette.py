"""The silhouette of a mesh part at a depth, for 2D operations (#173).

Contours cut at several depths, each from that depth's silhouette, follow a
stepped and drafted boss without gouging it (the run's material check and its
per-move proof), and every sampled point of the mesh at or above a depth lies
inside that depth's region (an even-odd test written here, not the engine's).
"""

import json
import math
import os
import unittest

from camlib import Engine
from camlib.export import material_status
from camlib.material import Options, SurfaceTarget
from camlib.operations import Contour
from camlib.planar import PlanarRegion
from camlib.simulation import Simulation, StockSpec, Update, VerificationUpdate
from camlib.tools import EndMill, HolderSection, ToolAssembly
from camlib.verification import MaterialCheck, StreamChecks
from support import silhouette as boss

TOOL = ToolAssembly(
    "t1", EndMill("em4", diameter=4, flute_length=30, overall_length=50), 35, 60, (HolderSection(35, 60, 4, 4),)
)
# Z3.01: beside the boss, just above the base's top face at Z3.
FLOORS = (7.0, 5.0, 3.01, 0.0)


def silhouette(z, mesh=None, **options):
    with Engine() as engine:
        return engine.silhouette_region(mesh or boss.part(), z, **{"max_error_mm": 0.0025, **options})


def outside_by(region, x, y):
    """How far (x, y) is outside the region (0 inside): even-odd crossings and
    distances to the boundary segments of the region's own vertices."""
    inside = False
    nearest = math.inf
    for component in region["components"]:
        for ring in (component["outer"], *component["holes"]):
            vertices = ring["vertices"]
            for a, b in zip(vertices, vertices[1:] + vertices[:1]):
                assert a[2] == 0, "a silhouette has straight edges only"
                if (a[1] > y) != (b[1] > y) and x < a[0] + (y - a[1]) / (b[1] - a[1]) * (b[0] - a[0]):
                    inside = not inside
                dx, dy = b[0] - a[0], b[1] - a[1]
                length2 = dx * dx + dy * dy
                t = max(0.0, min(1.0, ((x - a[0]) * dx + (y - a[1]) * dy) / length2)) if length2 else 0.0
                nearest = min(nearest, math.hypot(x - a[0] - t * dx, y - a[1] - t * dy))
    return 0.0 if inside else nearest


def rect_distance(rect, x, y):
    dx = max(rect[0] - x, x - rect[2])
    dy = max(rect[1] - y, y - rect[3])
    return math.hypot(dx, dy) if dx > 0 and dy > 0 else max(dx, dy)


def samples(mesh, n=10):
    for a, b, c in mesh.triangles:
        for i in range(n + 1):
            for j in range(n + 1 - i):
                u, v = i / n, j / n
                yield tuple(a[k] + u * (b[k] - a[k]) + v * (c[k] - a[k]) for k in range(3))


def contour(floor, region):
    return Contour(
        f"contour-z{floor}",
        TOOL,
        region,
        floor_z_mm=floor,
        stepdown_mm=4,
        clearance_z_mm=12,
        feed_mm_min=600,
        plunge_mm_min=150,
        spindle_rpm=12000,
        tool_number=1,
        length_register=1,
        coolant="flood",
    )


def run(operations):
    mesh = boss.part()
    checks = StreamChecks(
        material=(MaterialCheck(SurfaceTarget("boss", "1", mesh, 0), options=Options(comparison_tolerance_mm=0.005)),)
    )
    moves = []
    material = []
    with Simulation(
        StockSpec((0, 0, 0), (40, 40, 10), 0.25),
        (TOOL,),
        operations=tuple(operations),
        revision="silhouette-1",
        initial_position_mm=(0, 0, 12),
        verification=checks,
        executable=os.environ.get("CAMLIB_TEST_ENGINE"),
    ) as sim:
        for update in sim.run_operations():
            if isinstance(update, Update):
                moves.append(update.motion)
            elif isinstance(update, VerificationUpdate) and update.check["kind"] == "material":
                material.append(update.check)
        assert sim.state.finished
    return moves, material[-1]


class SilhouetteTest(unittest.TestCase):
    def test_every_mesh_point_at_or_above_a_depth_is_inside_that_depths_region(self):
        for error in (0.0, 0.002):
            mesh = boss.part(error)
            points = list(samples(mesh))
            for z in (-1, 0, 2.999, 3, 4.5, 6, 8.999, 9):
                out = silhouette(z, mesh, max_error_mm=0.01)
                region, report = out["region"], out["report"]
                bound = report["boundary_error_mm"]
                self.assertEqual(bound, region["boundary_error_mm"])
                self.assertTrue(error <= bound < error + 0.001, bound)
                self.assertEqual(region["source"]["adapter"], "surface_silhouette_v1")
                self.assertEqual(region["source"]["z_mm"], z)
                above = [p for p in points if p[2] >= z]
                self.assertGreater(len(above), 100)
                for x, y, _ in above:
                    self.assertLessEqual(outside_by(region, x, y), bound - error)
                # No bigger than the exact shadow of the level it states.
                self.assertLessEqual(z - report["protected_from_z_mm"], bound)
                exact = boss.outline(report["protected_from_z_mm"])
                for component in region["components"]:
                    self.assertEqual(component["holes"], [])
                    for x, y, _ in component["outer"]["vertices"]:
                        self.assertLessEqual(rect_distance(exact, x, y), bound)
                self.assertAlmostEqual(report["area_mm2"], (exact[2] - exact[0]) * (exact[3] - exact[1]), delta=0.2)
                again = json.loads(json.dumps(PlanarRegion.from_dict(region).to_dict()))
                self.assertEqual(again, region)

    def test_contours_from_each_depths_silhouette_follow_the_part_and_do_not_gouge_it(self):
        regions = {z: PlanarRegion.from_dict(silhouette(z)["region"]) for z in FLOORS}
        moves, material = run(contour(z, regions[z]) for z in FLOORS)
        report, proof = material["report"], material["sweep_proof"]
        self.assertTrue(report["complete"])
        self.assertEqual(report["gouge"]["definite_cells"], 0)
        self.assertEqual((proof["unproven_moves"], proof["history_complete"]), (0, True))
        self.assertEqual(material_status(report, proof), "clear")
        # Each depth's pass runs beside that depth's outline: the cutter
        # radius, the stated error and Contour's margin away, no farther.
        at_floor = {z: 0 for z in FLOORS}
        for move in moves:
            if move["kind"] != "cut" or "end" not in move["curve"]:
                continue
            for x, y, z in (move["curve"]["start"], move["curve"]["end"]):
                clear = rect_distance(boss.outline(z), x, y) - 2
                self.assertGreaterEqual(clear, 0)
                if z in at_floor:
                    self.assertLess(clear, 0.02)
                    at_floor[z] += 1
        self.assertTrue(all(count >= 8 for count in at_floor.values()), at_floor)

    def test_one_outline_for_every_depth_gouges_and_the_same_check_says_so(self):
        # What Contour could do before: the crown's outline cut down to the
        # foot of the boss, through its drafted walls.
        crown = PlanarRegion.from_dict(silhouette(8.5)["region"])
        _, material = run([contour(3.01, crown)])
        # (Not a definite violation: a mesh with a declared error bounds the
        # part from outside only. No motion into it is proven, so it is not
        # clear and export is refused.)
        self.assertGreater(material["sweep_proof"]["unproven_moves"], 0)
        self.assertEqual(material["sweep_proof"]["decided_cells"], 0)
        self.assertEqual(material_status(material["report"], material["sweep_proof"]), "uncertain")

    def test_refusals_name_the_problem(self):
        with self.assertRaisesRegex(ValueError, r"exceeds max_error_mm 0.01 .*surface_error_mm 0.02"):
            silhouette(5, boss.part(0.02), max_error_mm=0.01)
        self.assertIn("region", silhouette(5, boss.part(0.02), max_error_mm=0.025))
        with self.assertRaisesRegex(ValueError, "nothing at or above"):
            silhouette(9.5)
        with self.assertRaisesRegex(ValueError, "max_error_mm"):
            silhouette(5, max_error_mm=0)
        with self.assertRaises(TypeError):
            with Engine() as engine:
                engine.silhouette_region(boss.part(), 5)
        # Contour itself refuses a silhouette whose error its boundary
        # tolerance cannot hold (nothing is relaxed to make it fit).
        loose = PlanarRegion.from_dict(silhouette(5, boss.part(0.02), max_error_mm=0.025)["region"])
        with self.assertRaisesRegex(Exception, "uncertainty exceeds boundary tolerance"):
            run([contour(5, loose)])


if __name__ == "__main__":
    unittest.main()
