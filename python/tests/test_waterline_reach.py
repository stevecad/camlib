"""Issue #161: waterline admits each feed move by the depth it actually cuts.

A cone ``z = 40 - 3r`` down to a flat at Z1 (30 x 30 mm) is taller than the
12 mm flutes of a 6 mm ball. From stock left 0.5 mm over it (a closed mesh),
cut top-down, the shank and holder always have the finished wall receding
from them: accepted, with the independent stock-contact check clear on every
move. From a box, or beside an undrafted wall where the shank would rub, the
passes really cut deeper than the flutes and are refused with a typed reach
refusal. The meshes are built here by hand.
"""

import math
import unittest

import camlib
from camlib.simulation import ReachRefused, Simulation, StockMesh, StockSpec, VerificationUpdate
from camlib.surface import SurfaceMesh
from camlib.verification import StockContactOptions, StreamChecks

CENTER = 15
SEGMENTS = 64
# The 0.5 mm normal offset of the cone's 3:1 flank, measured vertically.
LIFT = 0.5 * math.sqrt(10)
CUT = dict(feed_mm_min=1200, plunge_mm_min=300, spindle_rpm=12000, coolant="flood")


def angles():
    return [i * 2 * math.pi / SEGMENTS for i in range(SEGMENTS)]


def ring(radius, z):
    return [(CENTER + radius * math.cos(a), CENTER + radius * math.sin(a), z) for a in angles()]


def square(z):
    """The 30 x 30 square's outline at the ring's angles (corners included)."""
    points = []
    for a in angles():
        c, s = math.cos(a), math.sin(a)
        t = CENTER / max(abs(c), abs(s))
        points.append((min(max(CENTER + t * c, 0), 30), min(max(CENTER + t * s, 0), 30), z))
    return points


def turned(apex, profile, base):
    """A turned top surface: an apex, rings of (radius, z) down and out, then
    the square at the last ring's Z. With ``base``, closed by the square's
    walls down to Z0 and a bottom (normals outward): a stock solid."""
    rings = [ring(r, z) for r, z in profile] + [square(profile[-1][1])]
    if base:
        rings.append(square(0))
    top = (CENTER, CENTER, apex)
    n = SEGMENTS
    triangles = [(top, rings[0][i], rings[0][(i + 1) % n]) for i in range(n)]
    for a, b in zip(rings, rings[1:]):
        for i in range(n):
            j = (i + 1) % n
            triangles += [(a[i], b[i], b[j]), (a[i], b[j], a[j])]
    if base:
        bottom = rings[-1]
        triangles += [((CENTER, CENTER, 0), bottom[(i + 1) % n], bottom[i]) for i in range(n)]
    return triangles


def cone():
    return SurfaceMesh(turned(40, [(13, 1)], False))


def cone_stock():
    top = 40 + LIFT
    return StockSpec.from_mesh(StockMesh.from_triangles(turned(top, [((top - 1.5) / 3, 1.5)], True)), 0.5)


def ball():
    return camlib.ToolAssembly(
        "ball6",
        camlib.BallEndMill("b6", diameter=6, flute_length=12, overall_length=50, shank_diameter=6),
        25,
        60,
        (camlib.HolderSection(25, 60, 8, 8),),
    )


def waterline(surface, top, **extra):
    return camlib.Waterline(
        "wl", ball(), surface, 0, top, 1.5, 0.5, 50, tool_number=1, length_register=1, **CUT, **extra
    )


def run(stock, operations, checks=None):
    """Runs to the end; returns the motion check statuses."""
    with Simulation(
        stock,
        (ball(),),
        operations=operations,
        revision="1",
        initial_position_mm=(CENTER, CENTER, 60),
        verification=checks,
    ) as sim:
        updates = list(sim.run_operations())
        assert sim.state.finished
    return [u for u in updates if isinstance(u, VerificationUpdate) and u.check["kind"] == "motion"]


class WaterlineReachTest(unittest.TestCase):
    def assert_contact_clear(self, reports):
        self.assertGreater(len(reports), 500)
        for report in reports:
            parts = {f["finding"].get("part") for f in report.check.get("findings", ())}
            self.assertFalse(parts & {"neck", "shank", "holder"}, report.check)
        self.assertEqual({r.status for r in reports}, {"clear"})

    def test_cone_taller_than_the_flutes_is_finished_from_roughed_stock(self):
        checks = StreamChecks(stock_contact=StockContactOptions())
        self.assert_contact_clear(run(cone_stock(), (waterline(cone(), 40),), checks))

    def test_steep_and_shallow_on_the_cone_is_accepted(self):
        steep, shallow = camlib.steep_and_shallow(
            "sas",
            ball(),
            cone(),
            0,
            50,
            top_z_mm=40,
            bottom_z_mm=1.5,
            stepdown_mm=0.5,
            stepover_mm=1,
            threshold_deg=45,
            tool_number=1,
            length_register=1,
            # The shallow passes start under the cone's stock: ramped in.
            entry=camlib.RampEntry(angle_deg=10, min_length_mm=1),
            **CUT,
        )
        checks = StreamChecks(stock_contact=StockContactOptions())
        self.assert_contact_clear(run(cone_stock(), (steep, shallow), checks))

    def test_cone_from_the_box_is_refused_with_a_typed_reach_refusal(self):
        mesh = cone_stock()
        box = StockSpec(mesh.min_mm, mesh.max_mm, 0.5)
        with self.assertRaises(ReachRefused) as caught:
            run(box, (waterline(cone(), 40),))
        error = caught.exception
        self.assertIn("cuts deeper than the cutting length 12", str(error))
        r = error.refusal
        self.assertEqual((r.operation, r.tool, r.limit, r.holder_section), ("wl", "ball6", "shank", None))
        self.assertEqual(r.top_z_mm, mesh.max_mm[2])
        self.assertEqual(r.depth_available_mm, 12)
        self.assertLessEqual(r.z_mm, r.top_z_mm - 12)
        self.assertGreater(r.z_mm, r.top_z_mm - 12.5)

    def test_undrafted_wall_where_the_shank_rubs_is_refused(self):
        boss = SurfaceMesh(turned(25, [(6, 25), (6, 1)], False))
        stock = StockSpec.from_mesh(StockMesh.from_triangles(turned(25.5, [(6.5, 25.5), (6.5, 1.5)], True)), 0.5)
        with self.assertRaises(ReachRefused) as caught:
            run(stock, (waterline(boss, 25),))
        r = caught.exception.refusal
        self.assertEqual((r.limit, r.holder_section), ("shank", None))
        self.assertTrue(25 <= r.top_z_mm <= 25.5 + 1e-9, r)
        self.assertTrue(r.top_z_mm - 12.5 <= r.z_mm < r.top_z_mm - 12, r)


if __name__ == "__main__":
    unittest.main()
