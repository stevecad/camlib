"""A grooving blade's clearance per flank (#170).

ISO 1832 gives an indexable insert one normal clearance; a grooving blade's
maker states the clearance below its front edge and below its sides
separately (e.g. 10 degrees at the front, 3 at the sides), and a face
grooving blade's two sides may differ. Declared as one angle such a blade
must take its least, and camlib then refuses grooves the real blade clears.

- The issue's inside groove (a 3 mm blade, 3 mm thick, in a Ø22 bore cutting
  a Ø25 groove): below its cutting face the blade's front reaches
  √(x² + y²), so its front needs about 7 degrees. 10 degrees on every side
  plans, 3 on every side is refused (naming the front), and front 10 /
  sides 3 plans, with the same moves.
- A straight face grooving blade in a ring groove r 4..7: its outer side
  reaches out below the centre plane, and needs atan((7 - √(7² - 3²)) / 3) =
  12.7 degrees whatever the front and the inner side have.
"""

import json
import math
import unittest

import camlib
from camlib._buffers import native
from test_turning import job, tool

CONE = 8 / math.tan(math.radians(59))


def inside_groove(**clearance):
    drill = tool("drill16", camlib.DrillInsert(16, 118, 50), nose_radius_mm=0, width_mm=40, length_mm=60)
    bar = tool(
        "bar",
        camlib.TurningInsert.iso(included_angle_deg=55, approach_angle_deg=95, edge_length_mm=6, nose_radius_mm=0.4),
        nose_radius_mm=0.4,
        width_mm=10,
        length_mm=80,
        offset_mm=1,
        side="inner",
    )
    blade = tool(
        "igroove",
        camlib.GrooveInsert(3, 8),
        nose_radius_mm=0.2,
        width_mm=6,
        length_mm=80,
        offset_mm=3,
        side="inner",
        insert_thickness_mm=3,
        **clearance,
    )
    return job(
        stock=camlib.TurnStockSpec(-60, 0, 20),
        profile=((0, 0, 0), (18, 0, 0), (18, -40, 0)),
        bore=(
            (11, 0, 0),
            (11, -18, 0),
            (12.5, -18, 0),
            (12.5, -21, 0),
            (11, -21, 0),
            (11, -25, 0),
            (8, -25, 0),
            (0, -25 - CONE, 0),
        ),
        tools=(
            camlib.TurnToolSetting(drill, 1, 1),
            camlib.TurnToolSetting(bar, 2, 2),
            camlib.TurnToolSetting(blade, 3, 3),
        ),
        operations=(
            camlib.TurnDrill("drill", "drill16", -25 - CONE, peck_mm=10, feed_mm_rev=0.12, rpm=900),
            camlib.TurnBoreRough("bore_rough", "bar", -24, allowance_mm=0, feed_mm_rev=0.15, rpm=1000),
            camlib.TurnGroove("groove", "igroove", -18, -21, 12.5, feed_mm_rev=0.04, rpm=700),
        ),
    )


def per_side(front, side):
    """The blade's own angles instead of the test body's one angle."""
    return dict(insert_clearance_deg=None, insert_front_clearance_deg=front, insert_side_clearance_deg=side)


def face_groove(**clearance):
    blade = tool("fg", camlib.FaceGrooveInsert(2, 8), nose_radius_mm=0.2, width_mm=2, length_mm=40, **clearance)
    return job(
        stock=camlib.TurnStockSpec(-60, 0, 13),
        profile=((0, 0, 0), (13, 0, 0), (13, -40, 0)),
        tools=(camlib.TurnToolSetting(blade, 1, 1),),
        face_grooves=(camlib.turning.FaceGroove(4, 7, -3),),
        operations=(camlib.TurnFaceGroove("fg", "fg", 4, 7, -3, peck_mm=1, feed_mm_rev=0.03, rpm=600),),
    )


class InsideGrooveTest(unittest.TestCase):
    def test_the_issues_blade_plans_with_its_front_and_side_clearance(self):
        uniform = inside_groove(insert_clearance_deg=10).plan()
        with self.assertRaisesRegex(Exception, "groove: the insert would cut .* front flank, 3 degrees of clearance"):
            inside_groove(insert_clearance_deg=3).plan()
        declared = inside_groove(**per_side(10, 3)).plan()
        self.assertEqual(len(declared.reports), 3)
        # The body only gates the moves: the program is the same.
        self.assertEqual(declared.program.events, uniform.program.events)

    def test_a_front_too_small_is_refused_whatever_the_sides_have(self):
        with self.assertRaisesRegex(Exception, "front flank, 3 degrees of clearance"):
            inside_groove(**per_side(3, 10)).plan()


class FaceGrooveSideTest(unittest.TestCase):
    def test_the_outer_side_needs_its_clearance_at_the_outer_wall(self):
        needed = math.degrees(math.atan((7 - math.sqrt(7**2 - 3**2)) / 3))
        self.assertAlmostEqual(needed, 12.7, places=1)
        # (inner side, outer side): the outer side's own 15 degrees plans.
        self.assertEqual(len(face_groove(**per_side(0, (0, 15))).plan().reports), 1)
        # The same angles the other way round: the outer side is named.
        with self.assertRaisesRegex(Exception, "the blade's outer side flank, 0 degrees of clearance") as refused:
            face_groove(**per_side(0, (15, 0))).plan()
        self.assertNotIn("inner side", str(refused.exception))
        # A generous front does not stand in for the side.
        with self.assertRaisesRegex(Exception, "the blade's outer side flank, 12 degrees of clearance"):
            face_groove(**per_side(20, (15, 12))).plan()
        self.assertTrue(face_groove(**per_side(20, (15, 13))).plan())
        # One angle on every side is judged at the same wall.
        with self.assertRaisesRegex(Exception, "outer side flank, 12 degrees"):
            face_groove(insert_clearance_deg=12).plan()
        self.assertTrue(face_groove(insert_clearance_deg=13).plan())


class DeclarationTest(unittest.TestCase):
    def blade(self, **declared):
        return tool("g", camlib.GrooveInsert(3, 8), nose_radius_mm=0.2, width_mm=3, length_mm=40, **declared)

    def test_one_angle_serializes_as_before(self):
        body = self.blade().to_dict()["body"]["insert"]
        self.assertEqual(
            sorted(body), ["edge_deg", "inclination_deg", "insert_clearance_deg", "insert_thickness_mm", "rake_deg"]
        )
        self.assertIsNone(self.blade().insert_front_clearance_deg)
        self.assertIsNone(self.blade().insert_side_clearance_deg)

    def test_front_and_side_are_declared_together(self):
        t = self.blade(**per_side(10, 3))
        # The uniform angle is their least (the corners between them).
        self.assertEqual((t.insert_clearance_deg, t.insert_front_clearance_deg), (3.0, 10.0))
        self.assertEqual(t.insert_side_clearance_deg, (3.0, 3.0))
        body = t.to_dict()["body"]["insert"]
        self.assertEqual(body["insert_clearance_deg"], 3.0)
        self.assertEqual(body["blade_clearance"], dict(front_deg=10.0, side_deg=[3.0, 3.0]))
        t.validate()
        self.assertEqual(self.blade(**per_side(10, (3, 5))).insert_side_clearance_deg, (3.0, 5.0))
        # The least, given as well, agrees.
        self.assertEqual(
            self.blade(insert_clearance_deg=3, insert_front_clearance_deg=10, insert_side_clearance_deg=3), t
        )
        with self.assertRaisesRegex(TypeError, "insert_front_clearance_deg needs insert_side_clearance_deg too"):
            self.blade(insert_clearance_deg=3, insert_front_clearance_deg=10)
        with self.assertRaisesRegex(TypeError, "insert_side_clearance_deg needs insert_front_clearance_deg too"):
            self.blade(insert_clearance_deg=3, insert_side_clearance_deg=3)
        with self.assertRaisesRegex(TypeError, "one angle for both sides or a pair"):
            self.blade(**per_side(10, (3, 3, 3)))

    def test_a_contradicting_uniform_angle_is_refused(self):
        # The test body's 7 degrees on every side against a 3 degree side.
        with self.assertRaisesRegex(ValueError, r"insert_clearance_deg=7 contradicts .* leave it out or give 3.0"):
            self.blade(insert_front_clearance_deg=10, insert_side_clearance_deg=3)
        data = self.blade(**per_side(10, 3)).to_dict()
        data["body"]["insert"]["insert_clearance_deg"] = 10.0
        with self.assertRaisesRegex(ValueError, "contradicts its blade's front clearance .* give 3"):
            native.turning_validate_tool(json.dumps(data))
        data = self.blade(**per_side(10, 3)).to_dict()
        data["body"]["insert"]["blade_clearance"]["side_deg"] = [3.0, 80.0]
        with self.assertRaisesRegex(ValueError, "0..60 degrees"):
            native.turning_validate_tool(json.dumps(data))

    def test_other_inserts_have_one_normal_clearance(self):
        insert = camlib.TurningInsert.iso(
            included_angle_deg=80, approach_angle_deg=95, edge_length_mm=12, nose_radius_mm=0.8
        )
        with self.assertRaisesRegex(TypeError, "a grooving blade's .* insert_clearance_deg alone"):
            tool("t", insert, nose_radius_mm=0.8, width_mm=20, length_mm=100, **per_side(7, 7))
        with self.assertRaisesRegex(TypeError, "a grooving blade's"):
            tool("t", camlib.ThreadInsert(60, 2), nose_radius_mm=0.1, width_mm=20, length_mm=100, **per_side(7, 7))
        # Natively too (a tool written as JSON).
        data = tool("t", insert, nose_radius_mm=0.8, width_mm=20, length_mm=100).to_dict()
        data["body"]["insert"]["blade_clearance"] = dict(front_deg=7.0, side_deg=[7.0, 7.0])
        with self.assertRaisesRegex(ValueError, "give insert_clearance_deg only"):
            native.turning_validate_tool(json.dumps(data))


if __name__ == "__main__":
    unittest.main()
