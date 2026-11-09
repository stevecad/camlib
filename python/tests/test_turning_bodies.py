"""Turning tool bodies off the centre plane (#103).

A boring tool finishing a Ø22 bore: its insert's body below the cutting face
lies at radius √(x² + y²), so a thick insert's heel reaches the bore wall
although its outline (the section on the centre plane) clears it. The same
job is planned with different insert bodies:

- 3 mm thick, 11 degrees clearance (positive, tapering inward): clear;
- 3 mm thick, 0 degrees (negative, straight sides): the heel reaches out
  √(11² + 3²) - 11 = 0.40 mm, refused;
- the issue's 4.76 mm negative insert: refused;
- 3 mm at 11 degrees on a seat inclined -20 degrees (the edge dips toward
  the nose, swinging the heel out past the clearance): refused. (Rake turns
  about the major edge, which runs almost radially on this 95 degree tool:
  it moves the body along z, not out.)

ISO codes give the worst case their symbols allow; a drill needs no insert
body; a tool without a body is refused, naming what to declare.
"""

import math
import unittest

import camlib
from test_turning import job, tool

CONE = 8 / math.tan(math.radians(59))


def boring(**bar_body):
    drill = tool("drill16", camlib.DrillInsert(16, 118, 50), nose_radius_mm=0, width_mm=40, length_mm=60)
    bar = tool(
        "bar",
        camlib.TurningInsert.iso(included_angle_deg=55, approach_angle_deg=95, edge_length_mm=6, nose_radius_mm=0.4),
        nose_radius_mm=0.4,
        width_mm=10,
        length_mm=80,
        offset_mm=1,
        side="inner",
        **bar_body,
    )
    return job(
        stock=camlib.TurnStockSpec(-60, 0, 20),
        profile=((0, 0, 0), (18, 0, 0), (18, -40, 0)),
        bore=((11, 0, 0), (11, -25, 0), (8, -25, 0), (0, -25 - CONE, 0)),
        tools=(camlib.TurnToolSetting(drill, 1, 1), camlib.TurnToolSetting(bar, 2, 2)),
        operations=(
            camlib.TurnDrill("drill", "drill16", -25 - CONE, peck_mm=10, feed_mm_rev=0.12, rpm=900),
            camlib.TurnBoreRough("bore_rough", "bar", -24, feed_mm_rev=0.15, rpm=1000),
            camlib.TurnBoreFinish("bore_finish", "bar", -24, feed_mm_rev=0.08, rpm=1200),
        ),
    )


class BoringBodyTest(unittest.TestCase):
    def test_a_thin_tapering_insert_bores_clear(self):
        plan = boring(insert_thickness_mm=3, insert_clearance_deg=11).plan()
        self.assertEqual(len(plan.reports), 3)

    def test_a_straight_sided_insert_of_the_same_thickness_reaches_the_wall(self):
        with self.assertRaisesRegex(Exception, "insert"):
            boring(insert_thickness_mm=3, insert_clearance_deg=0).plan()

    def test_the_issues_negative_insert_reaches_the_wall(self):
        with self.assertRaisesRegex(Exception, "insert"):
            boring(insert_thickness_mm=4.76, insert_clearance_deg=0).plan()

    def test_negative_seat_inclination_swings_the_heel_out(self):
        self.assertTrue(boring(insert_thickness_mm=3, insert_clearance_deg=11, seat=camlib.TurningSeat(0, 0)).plan())
        # The inclined heel reaches r 11.69 at the 11 mm wall: refused where
        # it first meets material (here the approach rapid, at the stock).
        with self.assertRaisesRegex(Exception, "insert|a rapid would touch the stock"):
            boring(insert_thickness_mm=3, insert_clearance_deg=11, seat=camlib.TurningSeat(0, -20)).plan()


class DeclaredBodyTest(unittest.TestCase):
    def test_iso_codes_give_their_worst_case(self):
        insert = camlib.TurningInsert.iso_code("CCMT09T304", approach_angle_deg=95)
        # T3: [3, 4) mm by the symbol rule, plus class M's 0.13 mm.
        self.assertEqual((insert.thickness_mm, insert.clearance_deg, insert.edge_deg), (4.13, 7.0, 5.0))
        with self.assertRaisesRegex(ValueError, "special clearance"):
            camlib.TurningInsert.iso_code("COMT09T304", approach_angle_deg=95)

    def test_a_curved_face_grooving_blade_serves_its_range_only(self):
        blade = tool(
            "fg",
            camlib.FaceGrooveInsert(2, 8),
            nose_radius_mm=0.2,
            width_mm=2,
            length_mm=40,
            curved_for_diameter_mm=(20, 30),
        )
        with self.assertRaisesRegex(Exception, "outside the curved blade's range"):
            job(
                stock=camlib.TurnStockSpec(-60, 0, 13),
                profile=((0, 0, 0), (13, 0, 0), (13, -40, 0)),
                tools=(camlib.TurnToolSetting(blade, 1, 1),),
                face_grooves=(camlib.turning.FaceGroove(4, 7, -3),),
                operations=(camlib.TurnFaceGroove("fg", "fg", 4, 7, -3, peck_mm=1, feed_mm_rev=0.03, rpm=600),),
            ).plan()

    def test_a_tool_without_a_body_is_refused_natively(self):
        good = tool("t", camlib.GrooveInsert(3, 4), nose_radius_mm=0.2, width_mm=3, length_mm=40)
        data = good.to_dict()
        del data["body"]
        from camlib._buffers import native
        import json

        with self.assertRaisesRegex(ValueError, "needs its body"):
            native.turning_validate_tool(json.dumps(data))


if __name__ == "__main__":
    unittest.main()
