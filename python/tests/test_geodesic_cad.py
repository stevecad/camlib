"""Geodesic finishing (#12) on CAD parts tessellated by the CadQuery adapter.

The same independent checks as the analytic surfaces (test_geodesic): no two
neighbouring passes farther apart along the surface than the stepover plus
the path error, closer only by straddled contacts within the band a ball
cannot touch at the mesh's sharpest bend, every point within half a stepover
of a planned pass (certified), no gouge, the stock never below the target.
The parts: a block with filleted top edges (flat, cylindrical and spherical
faces meeting tangentially), a spherical bowl (concave, wider than the ball)
in a block, and a plate with a through-hole (geodesics bend around the hole;
its boundary has two loops).
"""

import math
import unittest

import camlib
from camlib.material import SurfaceTarget
from camlib.simulation import OperationUpdate, Simulation
from camlib.tools import BallEndMill, HolderSection, ToolAssembly

from support.geodesic_parts import PARTS, STEP_DIR, bowl, plate_with_hole, rounded_block

try:
    import cadquery as cq
except ImportError:  # pragma: no cover - CAD environment absent
    cq = None

BALL = ToolAssembly(
    "ball6",
    BallEndMill("ball6", diameter=6, flute_length=30, overall_length=70),
    40,
    80,
    (HolderSection(40, 80, 6, 6),),
)
STEPOVER, RADIUS, PATH_ERROR = 1.5, 3.0, 0.005


def sharpest_bend(triangles):
    """The largest angle between upward-facing triangles sharing an edge."""

    def normal(t):
        a, b, c = t
        u = [b[k] - a[k] for k in range(3)]
        v = [c[k] - a[k] for k in range(3)]
        w = (u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0])
        length = math.sqrt(sum(x * x for x in w))
        return [x / length for x in w] if length > 0 else None

    edges = {}
    for t in triangles:
        n = normal(t)
        if n is None or n[2] <= 0:
            continue
        for k in range(3):
            key = tuple(sorted((tuple(t[k]), tuple(t[(k + 1) % 3]))))
            edges.setdefault(key, []).append(n)
    worst = 0.0
    for ns in edges.values():
        if len(ns) == 2:
            cos = max(-1.0, min(1.0, sum(x * y for x, y in zip(*ns))))
            worst = max(worst, math.acos(cos))
    return worst


def run(part, seed):
    from camlib.cadquery import export_surface_mesh

    surface = export_surface_mesh(part, max_error_mm=0.005)
    lo = [min(p[k] for t in surface.triangles for p in t) for k in range(3)]
    hi = [max(p[k] for t in surface.triangles for p in t) for k in range(3)]
    op = camlib.Geodesic(
        "geodesic",
        BALL,
        surface,
        lo[2],
        STEPOVER,
        hi[2] + 20,
        seed,
        path_error_mm=PATH_ERROR,
        feed_mm_min=1200,
        plunge_mm_min=300,
        spindle_rpm=12000,
        tool_number=1,
        length_register=1,
        coolant="flood",
    )
    target = SurfaceTarget("part", "1", surface, 0)
    config = dict(
        stock=camlib.StockSpec(tuple(lo), (hi[0], hi[1], hi[2] + 1), 0.5),
        tools=(BALL,),
        operations=(op,),
        revision="1",
        initial_position_mm=(lo[0], lo[1], hi[2] + 30),
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
    return surface, report, material


@unittest.skipIf(cq is None, "CadQuery is not installed")
class GeodesicCadTest(unittest.TestCase):
    def check(self, part, seed):
        surface, report, material = run(part, seed)
        g = report["geodesic"]
        lo, _, hi = g["spacing_mm"]
        miss = g["max_level_miss_mm"]
        self.assertGreater(g["samples"], 1000)
        self.assertEqual(g["unmeasured_samples"], 0)
        self.assertLessEqual(hi, STEPOVER + 2 * miss + PATH_ERROR + 1e-9)
        self.assertGreaterEqual(lo, STEPOVER - 2 * miss - PATH_ERROR - 1e-9)
        self.assertLessEqual(miss, RADIUS * sharpest_bend(surface.triangles) + PATH_ERROR)
        self.assertLessEqual(g["planned_coverage_mm"], STEPOVER / 2 + PATH_ERROR + 1e-9)
        self.assertEqual(report["rows_completed"], report["rows"])
        self.assertEqual(material["gouge"]["definite_cells"], 0)
        return g

    def test_rounded_block_from_its_boundary(self):
        self.check(rounded_block(), "boundary")

    def test_rounded_block_from_its_centre(self):
        self.check(rounded_block(), ("point", (20, 15)))

    def test_bowl_from_its_bottom(self):
        self.check(bowl(), ("point", (22, 22)))

    def test_step_fixtures_match_the_parts(self):
        # fixtures/geodesic/*.step are exports of the CadQuery parts (for
        # use without CadQuery); each must still be the same solid.
        for name, make in PARTS.items():
            step = cq.importers.importStep(str(STEP_DIR / f"{name}.step")).val()
            part = make().val()
            self.assertAlmostEqual(step.Volume(), part.Volume(), delta=1e-6 * part.Volume(), msg=name)
            a, b = step.BoundingBox(), part.BoundingBox()
            for u, v in (
                (a.xmin, b.xmin),
                (a.ymin, b.ymin),
                (a.zmin, b.zmin),
                (a.xmax, b.xmax),
                (a.ymax, b.ymax),
                (a.zmax, b.zmax),
            ):
                self.assertAlmostEqual(u, v, delta=1e-6, msg=name)

    def test_plate_around_a_hole(self):
        self.check(plate_with_hole(), ("point", (6, 6)))
        self.check(plate_with_hole(), "boundary")


if __name__ == "__main__":
    unittest.main()
