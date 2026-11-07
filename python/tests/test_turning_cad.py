"""Turned profiles and bores from CadQuery solids of revolution, planned."""

import math
import unittest

import cadquery as cq

import camlib
from camlib.cadquery import turning_section
from test_turning import job, tool


def revolve(points):
    """A solid turned about Z from a closed (r, z) polygon in the XZ plane."""
    return cq.Workplane("XZ").polyline(points).close().revolve(360, (0, 0, 0), (0, 1, 0)).val()


class TurningSectionTest(unittest.TestCase):
    def test_stepped_shaft_with_a_round_and_a_blind_bore(self):
        # r 10 for 20 mm, a 2 mm round into the shoulder, r 15 to z -40; a
        # 10 mm hole 12 deep with a flat bottom.
        sketch = (
            cq.Workplane("XZ")
            .moveTo(5, 0)
            .lineTo(10, 0)
            .lineTo(10, -18)
            .threePointArc(
                (10 + 2 - 2 * math.cos(math.radians(45)), -20 + 2 - 2 * math.sin(math.radians(45))), (12, -20)
            )
            .lineTo(15, -20)
            .lineTo(15, -40)
            .lineTo(0, -40)
            .lineTo(0, -12)
            .lineTo(5, -12)
            .close()
        )
        solid = sketch.revolve(360, (0, 0, 0), (0, 1, 0)).val()
        s = turning_section(solid)
        self.assertAlmostEqual(s.z_front_mm, 0)
        self.assertAlmostEqual(s.z_back_mm, -40)
        r, z, bulge = zip(*s.profile)
        self.assertEqual(s.profile[0][:2], (0.0, 0.0))  # the face across the hole
        self.assertTrue(any(abs(b) > 0 for b in bulge))  # the round, exact
        self.assertAlmostEqual(s.profile[-1][0], 15)
        self.assertAlmostEqual(s.profile[-1][1], -40)
        self.assertEqual([tuple(round(v, 9) for v in p) for p in s.bore], [(5, 0, 0), (5, -12, 0), (0, -12, 0)])
        self.assertAlmostEqual(s.volume_mm3, s.revolved_volume_mm3, places=4)
        # Planned: face, rough and finish the outside with the extracted profile.
        plan = job(stock=camlib.TurnStockSpec(-60, 1, 17), profile=s.profile, bore=s.bore).plan()
        self.assertGreater(plan.reports[2]["cut_length_mm"], 40)

    def test_through_bore_and_arcs_keep_their_direction(self):
        # A ball end (radius 10 about z -10) on a r 10 shaft with a 4 mm through hole.
        sketch = (
            cq.Workplane("XZ")
            .moveTo(2, 0 - 10 + math.sqrt(100 - 4))
            .threePointArc((10 * math.cos(math.radians(45)), -10 + 10 * math.sin(math.radians(45))), (10, -10))
            .lineTo(10, -30)
            .lineTo(2, -30)
            .close()
        )
        s = turning_section(sketch.revolve(360, (0, 0, 0), (0, 1, 0)).val())
        # From the front (hole edge) outward over the ball toward -z: a
        # convex arc, G3 (Centroid lathe manual 9.3, rule 1): positive bulge.
        arc = [p for p in s.profile if p[2] != 0]
        self.assertEqual(len(arc), 1)
        self.assertGreater(arc[0][2], 0)
        self.assertEqual([tuple(round(v, 9) for v in p[:2]) for p in s.bore][-1], (2, -30))

    def test_face_groove_is_declared_and_planned(self):
        # A r 12 shaft with a ring groove r 4..7, 3 mm deep, in its face.
        s = turning_section(revolve([(0, 0), (4, 0), (4, -3), (7, -3), (7, 0), (12, 0), (12, -40), (0, -40)]))
        self.assertEqual([(g.inner_radius_mm, g.outer_radius_mm, g.bottom_z_mm) for g in s.face_grooves], [(4, 7, -3)])
        self.assertTrue(all(z <= 0 for _, z, _ in s.profile))
        self.assertAlmostEqual(s.volume_mm3, s.revolved_volume_mm3, places=4)
        # A blade curved for 8..14 mm grooves (its maker's range): it follows
        # this 8..14 mm groove off the centre plane (#103).
        blade = tool(
            "fg",
            camlib.FaceGrooveInsert(2, 8),
            nose_radius_mm=0.2,
            width_mm=2,
            length_mm=40,
            curved_for_diameter_mm=(8, 14),
        )
        plan = job(
            stock=camlib.TurnStockSpec(-60, 0, 13),
            profile=s.profile,
            tools=(camlib.TurnToolSetting(blade, 1, 1),),
            face_grooves=s.face_grooves,
            operations=(camlib.TurnFaceGroove("fg", "fg", 4, 7, -3, peck_mm=1, feed_mm_rev=0.03, rpm=600),),
        ).plan()
        self.assertEqual(plan.reports[0]["passes"], 2)
        self.assertTrue(plan.stock["rings"])

    def test_refusals(self):
        with self.assertRaisesRegex(ValueError, "not a solid of revolution"):
            turning_section(cq.Workplane().box(10, 10, 10).val())
        with self.assertRaisesRegex(ValueError, "undercut"):
            # A dovetail groove: its back wall leans toward the front.
            turning_section(revolve([(0, 0), (10, 0), (10, -10), (7, -10), (7, -14), (10, -12), (10, -20), (0, -20)]))
        outer = revolve([(0, 0), (10, 0), (10, -20), (0, -20)])
        with self.assertRaisesRegex(ValueError, "void"):
            turning_section(outer.cut(revolve([(3, -5), (6, -5), (6, -15), (3, -15)])))
        with self.assertRaisesRegex(ValueError, "cavity on the axis"):
            turning_section(outer.cut(revolve([(0, -5), (4, -5), (4, -15), (0, -15)])))


if __name__ == "__main__":
    unittest.main()
