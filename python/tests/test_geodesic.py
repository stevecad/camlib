"""Geodesic finishing (#12): a constant stepover measured along the surface.

On a spherical dome (rings from its apex, meeting a flat base in a concave
crease, with walls) and a saddle (inward from its boundary, rings from a
point, and both ways from a curve), the cutter positions are solved so each
achieved contact lies on its pass's level, and the passes are checked
independently from the achieved contacts:

- no two neighbouring passes are farther apart along the surface than the
  stepover plus the path error;
- they are closer only where the ball rests on both sides of a concave facet
  edge (its contact jumps across a band no position touches): by at most
  twice that miss, itself within the band a ball of radius R leaves at the
  mesh's sharpest concave bend;
- every point of the surface is within half a stepover (plus the path error)
  of a planned pass, certified;
- the material check finds no gouge and the stock never goes below the
  surface.
"""

import math
import unittest

import camlib
from camlib.simulation import OperationUpdate, Simulation
from camlib.material import SurfaceTarget
from camlib.surface import SurfaceMesh
from camlib.tools import BallEndMill, HolderSection, ToolAssembly

BALL = ToolAssembly(
    "ball6",
    BallEndMill("ball6", diameter=6, flute_length=30, overall_length=70),
    40,
    80,
    (HolderSection(40, 80, 6, 6),),
)
SIZE = 40.0


def height_field(z, step=0.5):
    """A solid's surface over the 40 x 40 square: the height field z(x, y)
    on top and vertical walls down to the base (Z0) all round, as a part's
    exported mesh has (at the region's edge the ball rolls down the walls)."""
    n = int(SIZE / step)
    pts = [[(i * step, j * step, z(i * step, j * step)) for i in range(n + 1)] for j in range(n + 1)]
    triangles = []
    for j in range(n):
        for i in range(n):
            a, b, c, d = pts[j][i], pts[j][i + 1], pts[j + 1][i + 1], pts[j + 1][i]
            triangles += [(a, b, c), (a, c, d)]
    rim = [pts[0][i] for i in range(n)] + [pts[j][n] for j in range(n)]
    rim += [pts[n][i] for i in range(n, 0, -1)] + [pts[j][0] for j in range(n, 0, -1)]
    for k in range(len(rim)):
        p, q = rim[k], rim[(k + 1) % len(rim)]
        p0, q0 = (p[0], p[1], 0.0), (q[0], q[1], 0.0)
        triangles += [(p, q, q0), (p, q0, p0)]
    return triangles


def dome(x, y):
    # A sphere of radius 30 centred 12 below the base (apex at Z18), meeting
    # a flat base at Z0.5 near the square's corners (a crease).
    return max(0.5, math.sqrt(max(0.0, 30**2 - (x - 20) ** 2 - (y - 20) ** 2)) - 12)


def saddle(x, y):
    return 10 + ((x - 20) ** 2 - (y - 20) ** 2) / 60


def run(surface_z, seed, stepover=1.5, sag=0.01, mode="contact"):
    # Chords of a 0.5 mm grid sag at most about 0.25/(8 x 30) mm on the dome
    # (curvature radius 30) and less on the saddle: declared as 0.01 mm.
    surface = SurfaceMesh(height_field(surface_z), surface_error_mm=sag)
    op = camlib.Geodesic(
        "geodesic",
        BALL,
        surface,
        0,
        stepover,
        30,
        seed,
        feed_mm_min=1200,
        plunge_mm_min=300,
        spindle_rpm=12000,
        tool_number=1,
        length_register=1,
        coolant="flood",
        mode=mode,
    )
    target = SurfaceTarget("part", "1", surface, 0)
    config = dict(
        stock=camlib.StockSpec((0, 0, 0), (SIZE, SIZE, 20), 0.5),
        tools=(BALL,),
        operations=(op,),
        revision="1",
        initial_position_mm=(0, 0, 40),
        verification=camlib.StreamChecks(material=(camlib.MaterialCheck(target),)),
    )
    report = material = None
    with Simulation(**config) as sim:
        for update in sim.run_operations():
            if isinstance(update, OperationUpdate) and update.stage == "end":
                report = update.report
            check = getattr(update, "check", None)
            if isinstance(check, dict) and check.get("kind") == "material":
                material = check["report"]
        state = sim.state
        heights = list(state.heights)
        geometry = (state.nx, state.ny, state.dx, state.dy)
    return report, heights, geometry, material


def sharpest_bend(surface_z, step=0.5):
    """The largest angle between neighbouring facets of the height field's
    smooth part (its grid cells' two triangles and their neighbours)."""
    n = int(SIZE / step)

    def normal(a, b, c):
        u = [b[k] - a[k] for k in range(3)]
        v = [c[k] - a[k] for k in range(3)]
        w = (u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0])
        length = math.sqrt(sum(x * x for x in w))
        return [x / length for x in w]

    def p(i, j):
        return (i * step, j * step, surface_z(i * step, j * step))

    normals = {}
    for j in range(n):
        for i in range(n):
            a, b, c, d = p(i, j), p(i + 1, j), p(i + 1, j + 1), p(i, j + 1)
            normals[(i, j, 0)] = normal(a, b, c)
            normals[(i, j, 1)] = normal(a, c, d)
    worst = 0.0
    for (i, j, k), m in normals.items():
        for other in [(i, j, 1 - k), (i + 1, j, 1), (i, j + 1, 0)]:
            if other in normals:
                cos = max(-1.0, min(1.0, sum(x * y for x, y in zip(m, normals[other]))))
                worst = max(worst, math.acos(cos))
    return worst


PATH_ERROR = 0.005


class GeodesicTest(unittest.TestCase):
    def check(self, surface_z, seed):
        stepover, radius = 1.5, 3.0
        report, heights, (nx, ny, dx, dy), material = run(surface_z, seed, stepover)
        g = report["geodesic"]
        lo, mean, hi = g["spacing_mm"]
        miss = g["max_level_miss_mm"]
        self.assertGreater(g["samples"], 3000)
        self.assertEqual(g["unmeasured_samples"], 0)
        self.assertLess(g["off_drive_samples"], 0.001 * g["samples"])
        # Never farther apart than the stepover, except by contacts off
        # their level (each within the reported miss: the ball meets a
        # neighbouring facet first) and the chords' error.
        self.assertLessEqual(hi, stepover + 2 * miss + PATH_ERROR + 1e-9)
        # Closer only by straddled contacts, each within the band no ball
        # position touches at a concave bend of the facets.
        self.assertGreaterEqual(lo, stepover - 2 * miss - PATH_ERROR - 1e-9)
        self.assertLessEqual(miss, radius * sharpest_bend(surface_z))
        # Every point within half a stepover of a planned pass, certified.
        self.assertLessEqual(g["planned_coverage_mm"], stepover / 2 + PATH_ERROR + 1e-9)
        # A flat-surface estimate, labelled as such.
        self.assertAlmostEqual(g["flat_scallop_estimate_mm"], 3 - math.sqrt(9 - (hi / 2) ** 2), places=12)
        self.assertEqual(report["rows_completed"], report["rows"])
        # The material check: no gouge anywhere.
        self.assertEqual(material["gouge"]["definite_cells"], 0)
        # The stock never below the surface (its upper bound over each cell
        # at or above the surface's lowest point over the cell).
        for j in range(ny):
            for i in range(nx):
                x0, x1, y0, y1 = i * dx, (i + 1) * dx, j * dy, (j + 1) * dy
                low = min(surface_z(x, y) for x in (x0, x1) for y in (y0, y1))
                self.assertGreaterEqual(heights[2 * (j * nx + i) + 1], low - 0.011)
        return g

    def test_dome_rings_from_the_apex(self):
        g = self.check(dome, ("point", (20, 20)))
        self.assertEqual(g["seed"], "point")
        # The concave crease where the dome meets the base is narrower than
        # the ball, and the wall tops: those levels no position touches are
        # reported, never counted as spacing.
        self.assertGreater(g["unreachable_samples"], 0)

    def test_saddle_inward_from_the_boundary(self):
        g = self.check(saddle, "boundary")
        self.assertEqual(g["seed"], "boundary")
        self.assertEqual(g["unreachable_samples"], 0)

    def test_saddle_rings_from_a_point(self):
        self.check(saddle, ("point", (20, 20)))

    def test_saddle_both_ways_from_a_curve(self):
        g = self.check(saddle, ("curve", ((6.0, 33.0), (20.0, 21.0), (34.0, 8.0))))
        self.assertEqual(g["seed"], "curve")

    def test_dome_tool_centre(self):
        # Tool-centre mode (#119): the passes a stepover apart along the
        # surface the ball's centre moves on (measured), smooth over the
        # concave crease at the dome's foot; the contact spacing it gives is
        # reported alongside (closer on the convex dome), and nothing gouges.
        report, _, _, material = run(dome, ("point", (20, 20)), mode="tool_centre")
        g = report["geodesic"]
        self.assertEqual(g["mode"], "tool_centre")
        lo, mean, hi = g["centre_spacing_mm"]
        self.assertGreaterEqual(lo, 1.5 - PATH_ERROR)
        self.assertLessEqual(hi, 1.5 + PATH_ERROR)
        self.assertEqual(g["unreachable_samples"], 0)
        self.assertLess(g["spacing_mm"][1], 1.5)
        self.assertLessEqual(g["planned_coverage_mm"], 1.5 / 2 + PATH_ERROR + 1e-9)
        self.assertEqual(report["rows_completed"], report["rows"])
        self.assertEqual(material["gouge"]["definite_cells"], 0)

    def test_mode_values(self):
        with self.assertRaisesRegex(ValueError, "Geodesic mode"):
            camlib.Geodesic(
                "g",
                BALL,
                SurfaceMesh(height_field(saddle, 10)),
                0,
                1.5,
                30,
                "boundary",
                feed_mm_min=1,
                plunge_mm_min=1,
                spindle_rpm=1,
                tool_number=1,
                length_register=1,
                coolant="flood",
                mode="centre",
            )

    def test_seed_values(self):
        with self.assertRaisesRegex(ValueError, "Geodesic seed"):
            camlib.Geodesic(
                "g",
                BALL,
                SurfaceMesh(height_field(saddle, 10)),
                0,
                1.5,
                30,
                ("point", (1, 2, 3)),
                feed_mm_min=1,
                plunge_mm_min=1,
                spindle_rpm=1,
                tool_number=1,
                length_register=1,
                coolant="flood",
            )


if __name__ == "__main__":
    unittest.main()
