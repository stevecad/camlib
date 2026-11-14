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

from support.geodesic_parts import PARTS, STEP_DIR, bowl, plate_with_hole, rounded_block, v_groove_block

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


def run(part, seed, mode="contact"):
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
        mode=mode,
        # Passes that start under the stock are ramped in, not plunged.
        entry=camlib.RampEntry(angle_deg=10, min_length_mm=0.5),
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
    cuts = []
    with Simulation(**config) as sim:
        for update in sim.run_operations():
            motion = getattr(update, "motion", None)
            if isinstance(motion, dict) and motion["kind"] == "cut" and motion["curve"]["kind"] == "line":
                cuts.append((motion["curve"]["start"], motion["curve"]["end"]))
            if isinstance(update, OperationUpdate) and update.stage == "end":
                report = update.report
            check = getattr(update, "check", None)
            if isinstance(check, dict) and check.get("kind") == "material":
                material = check["report"]
    return surface, report, material, cuts


@unittest.skipIf(cq is None, "CadQuery is not installed")
class GeodesicCadTest(unittest.TestCase):
    def check(self, part, seed):
        surface, report, material, cuts = run(part, seed)
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
        self.cuts = cuts
        return g

    def test_rounded_block_from_its_boundary(self):
        self.check(rounded_block(), "boundary")

    def test_rounded_block_from_its_centre(self):
        self.check(rounded_block(), ("point", (20, 15)))

    def test_bowl_from_its_bottom(self):
        self.check(bowl(), ("point", (22, 22)))

    def test_v_groove_tool_centre(self):
        # Tool-centre mode (#119) over a sharp internal corner: the ball
        # cannot reach into the groove's crease, so in contact mode the
        # passes leave a band there (unreachable contacts). On the surface
        # its centre moves on the passes run smoothly over it a stepover
        # apart (measured), every position touches, and the contact spacing
        # (contacts merge in the crease, jump across it) is reported.
        _, contact, _, _ = run(v_groove_block(), ("point", (20, 5)))
        self.assertGreater(contact["geodesic"]["unreachable_samples"], 0)
        _, report, material, _ = run(v_groove_block(), ("point", (20, 5)), mode="tool_centre")
        g = report["geodesic"]
        self.assertEqual(g["mode"], "tool_centre")
        lo, mean, hi = g["centre_spacing_mm"]
        self.assertGreaterEqual(lo, STEPOVER - PATH_ERROR)
        self.assertLessEqual(hi, STEPOVER + PATH_ERROR)
        self.assertEqual(g["unreachable_samples"], 0)
        self.assertGreater(g["spacing_mm"][2], STEPOVER)
        self.assertLessEqual(g["planned_coverage_mm"], STEPOVER / 2 + PATH_ERROR + 1e-9)
        self.assertEqual(report["rows_completed"], report["rows"])
        self.assertEqual(material["gouge"]["definite_cells"], 0)

    def test_degenerate_tessellation_is_refused(self):
        # A sphere tessellated with its pole on the surface: a tessellator
        # fans slivers there far thinner than the mesh's own accuracy, across
        # which no distance is reliable. Geodesic refuses it, naming the
        # problem, instead of planning on it.
        from camlib.cadquery import export_surface_mesh
        from camlib.simulation import SimulationError
        from camlib.surface import SurfaceMesh

        block = cq.Workplane("XY").box(44, 44, 12, centered=False)
        polar = block.cut(cq.Workplane("XY").sphere(25).translate((22, 22, 28)))
        exported = export_surface_mesh(polar, max_error_mm=0.05)

        def thinnest(triangles):
            def height(t):
                u = [b - a for a, b in zip(t[0], t[1])]
                v = [b - a for a, b in zip(t[0], t[2])]
                cross = (u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0])
                return math.hypot(*cross) / max(math.dist(t[i], t[(i + 1) % 3]) for i in range(3))

            return min(map(height, triangles))

        # camlib's own exporter used to make that fan (the pole and two nodes
        # of one meridian, in line, moved a micron apart to have an area); its
        # mesh is watertight now and the pole is one vertex (#241), so the fan
        # is written here: the pole (22, 22, 3) and two points 0.025 and
        # 0.05 mm down three meridians of the sphere, the second 20 nm aside.
        limit = exported.surface_error_mm / 1000
        self.assertGreater(thinnest(exported.triangles), limit)

        def on_sphere(polar, heading):
            r = 25 * math.sin(polar)
            return (22 + r * math.cos(heading), 22 + r * math.sin(heading), 28 - 25 * math.cos(polar))

        fan = [((22.0, 22.0, 3.0), on_sphere(0.001, a), on_sphere(0.002, a + 0.0004)) for a in (0.0, 2.1, 4.2)]
        self.assertLess(thinnest(fan), limit)
        surface = SurfaceMesh((*exported.triangles, *fan), exported.surface_error_mm, exported.source)
        op = camlib.Geodesic(
            "geodesic",
            BALL,
            surface,
            0,
            STEPOVER,
            32,
            ("point", (22, 22)),
            feed_mm_min=1200,
            plunge_mm_min=300,
            spindle_rpm=12000,
            tool_number=1,
            length_register=1,
            coolant="flood",
        )
        config = dict(
            stock=camlib.StockSpec((0, 0, 0), (44, 44, 13), 0.5),
            tools=(BALL,),
            operations=(op,),
            revision="1",
            initial_position_mm=(0, 0, 40),
        )
        with self.assertRaisesRegex(SimulationError, "degenerate triangles"):
            with Simulation(**config) as sim:
                for _ in sim.run_operations():
                    pass

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
        # On the flat top near the seed the passes are circles about it:
        # each cut move stays within the path error of its circle (a move
        # merged across a curve, checked only for height, would cut its
        # chord across the circle).
        near = 0
        for a, b in self.cuts:
            ra, rb = (math.hypot(p[0] - 6, p[1] - 6) for p in (a, b))
            if a[2] > 5.99 and b[2] > 5.99 and 1 < min(ra, rb) and max(ra, rb) < 5 and abs(ra - rb) < 0.01:
                mid = ((a[0] + b[0]) / 2 - 6, (a[1] + b[1]) / 2 - 6)
                self.assertGreaterEqual(math.hypot(*mid), min(ra, rb) - PATH_ERROR - 1e-6, (a, b))
                near += 1
        self.assertGreater(near, 50)
        self.check(plate_with_hole(), "boundary")


if __name__ == "__main__":
    unittest.main()
