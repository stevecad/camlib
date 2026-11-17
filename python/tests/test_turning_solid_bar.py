"""Solid (one-piece) boring bars (#114).

A solid carbide boring bar has no insert: its tip is ground into the end of
a round bar, so its body below the cutting face is the bar's cylinder cut by
the ground faces. Declared as an insert body (a prism of the head's outline,
as deep as the bar reaches below the face, clearance 0) it is refused in the
small bores it is made for; declared as a ``SolidBarHead`` it bores them.

The tool is synthetic, with the dimension set of a catalogue's Ø5 bar
(KYOCERA EZ Bar catalogue, EZBR 055050ST: shank 5.0, WF 2.44, minimum bore
5.5 mm): a head 6 mm long, the cutting face on the bar's centre, a 0.1 mm
nose, 20 degrees below the side cutting edge, 8 below the end and the nose.
"""

import math
import unittest

import camlib
from camlib.turning import Arc, Line, Outline
from test_turning import job

RN = 0.1


def point(deg, r=RN):
    return (r * math.cos(math.radians(deg)), r * math.sin(math.radians(deg)))


def head_outline():
    """The head at its cutting face, drawn as an outside right-hand tool:
    the side cutting edge (2 degrees of back taper) up to z = 6, the nose,
    the end cutting edge (5 degrees) out to x = 4.8, the bar's far side and
    the back."""
    side, end = point(178), point(275)
    top = (side[0] + (6 - side[1]) / math.tan(math.radians(88)), 6.0)
    far = (4.8, end[1] + (4.8 - end[0]) * math.tan(math.radians(5)))
    outline = Outline(top, (Line(side), Arc((0, 0), RN, 178, 275), Line(far), Line((4.8, 6.0)), Line(top)))
    return camlib.InsertOutline(outline, 1, (0, 1, 2), (-RN, -RN))


def sleeve():
    outline = Outline((-5, 30), (Line((10, 30)), Line((10, 60)), Line((-5, 60)), Line((-5, 30))))
    return camlib.TurningHolder(outline, bar=camlib.TurningBar(5, 2.34, (6, 30), 0), y_mm=(-8, 8))


def head(side_clearance_deg=20, min_bore_diameter_mm=5.5, **changes):
    values = dict(
        diameter_mm=5,
        axis_x_mm=2.34,  # the edge at x -0.1, WF 2.44 from the centreline
        axis_y_mm=0,
        clearance_deg=(side_clearance_deg, min(side_clearance_deg, 8), 8, 0, 0),
        min_bore_diameter_mm=min_bore_diameter_mm,
        max_bore_depth_mm=6,
    )
    values.update(changes)
    return camlib.SolidBarHead(**values)


def solid_bar(solid=None, **body):
    return camlib.TurningTool(
        "solid",
        insert=head_outline(),
        side="inner",
        hand="right",
        holder=sleeve(),
        **(body or dict(solid_bar=solid or head())),
    )


def boring(tool, *, pilot=2.8, bore=3.25, depth=4, z_end=-3.5, clearance_mm=0.3):
    return job(
        stock=camlib.TurnStockSpec(-20, 0, 10, bore_radius_mm=pilot),
        profile=((0, 0, 0), (9, 0, 0), (9, -15, 0)),
        bore=((bore, 0, 0), (bore, -depth, 0), (pilot, -depth, 0), (pilot, -15, 0)),
        chuck_z_mm=-18,
        tools=(camlib.TurnToolSetting(tool, 7, 7),),
        operations=(
            camlib.TurnBoreRough(
                "bore_rough",
                "solid",
                z_end,
                depth_of_cut_mm=0.2,
                allowance_mm=0.05,
                feed_mm_rev=0.03,
                rpm=3000,
                retract_mm=0.1,
            ),
            camlib.TurnBoreFinish("bore_finish", "solid", z_end, feed_mm_rev=0.02, rpm=3000),
        ),
        initial_position_mm=(0.5, 20),
        clearance_mm=clearance_mm,  # 0.3: room to back off the wall inside a small bore
    )


class SolidBoringBarTest(unittest.TestCase):
    def test_it_bores_a_small_bore_its_outline_prism_is_refused_in(self):
        plan = boring(solid_bar()).plan()
        self.assertEqual(len(plan.reports), 2)
        # The same head as an insert body: a prism of its outline, 2.5 mm
        # deep (the bar's reach below the face), clearance 0.
        prism = solid_bar(seat=camlib.TurningSeat(0, 0), insert_thickness_mm=2.5, insert_clearance_deg=0, edge_deg=90)
        with self.assertRaisesRegex(Exception, "rub the stock|into the part"):
            boring(prism).plan()

    def test_a_cut_below_the_minimum_bore_is_refused(self):
        # The first roughing pass opens the Ø5.6 pilot to Ø6.0.
        with self.assertRaisesRegex(Exception, "below its declared minimum bore of 6.2 mm"):
            boring(solid_bar(head(min_bore_diameter_mm=6.2))).plan()
        # A Ø4.6 pilot to bore to Ø5.2: the first pass is at Ø5.0.
        with self.assertRaisesRegex(Exception, "below its declared minimum bore of 5.5 mm"):
            boring(solid_bar(), pilot=2.3, bore=2.6).plan()

    def test_the_geometry_is_checked_whatever_the_premises_say(self):
        # Backing 2 mm off the wall of the Ø6.5 bore puts the far side of
        # the Ø5 round into the opposite wall.
        with self.assertRaisesRegex(Exception, "bore_finish: the insert would cut"):
            boring(solid_bar(), clearance_mm=2).plan()

    def test_a_move_beyond_the_maximum_bore_depth_is_refused(self):
        with self.assertRaisesRegex(Exception, "beyond its declared maximum bore depth of 6 mm"):
            boring(solid_bar(), depth=10, z_end=-7).plan()
        self.assertEqual(len(boring(solid_bar(), depth=10, z_end=-5.5).plan().reports), 2)

    def test_too_little_clearance_for_the_bores_curvature_is_refused(self):
        # 7 degrees below the side edge clears a bore down to Ø7.47 only:
        # declared for Ø5.5 the head contradicts itself; declared honestly
        # it is refused in the Ø6.5 bore.
        with self.assertRaisesRegex(ValueError, "contradicts its head.*the least bore it enters is 7.47"):
            solid_bar(head(side_clearance_deg=7)).validate()
        fouling = solid_bar(head(side_clearance_deg=7, min_bore_diameter_mm=7.6)).validate()
        with self.assertRaisesRegex(Exception, "below its declared minimum bore of 7.6 mm"):
            boring(fouling).plan()
        # The nose with more clearance than a side it joins is refused.
        with self.assertRaisesRegex(ValueError, "rounded corner .outline segment 1: 9 degrees"):
            solid_bar(head(clearance_deg=(20, 9, 8, 0, 0))).validate()


class DeclarationTest(unittest.TestCase):
    def test_the_head_is_the_whole_body(self):
        tool = solid_bar().validate()
        body = tool.to_dict()["body"]
        self.assertNotIn("insert", body)
        self.assertEqual(
            body["solid_bar"],
            dict(
                diameter_mm=5.0,
                axis_x_mm=2.34,
                axis_y_mm=0.0,
                clearance_deg=[20.0, 8.0, 8.0, 0.0, 0.0],
                min_bore_diameter_mm=5.5,
                max_bore_depth_mm=6.0,
            ),
        )
        self.assertIn("solid_bar=SolidBarHead(", repr(tool))
        self.assertEqual(tool, solid_bar())
        self.assertNotEqual(tool, solid_bar(head(min_bore_diameter_mm=6)))
        for name, value in (
            ("seat", camlib.TurningSeat(0, 0)),
            ("insert_thickness_mm", 2.5),
            ("insert_clearance_deg", 7),
            ("edge_deg", 90),
        ):
            with self.assertRaisesRegex(TypeError, f"no insert and no seat.*leave out {name}"):
                solid_bar(solid_bar=head(), **{name: value})

    def test_other_tools_are_unchanged(self):
        insert = camlib.TurningInsert.iso_code("CCMT060204", approach_angle_deg=95)
        holder = camlib.TurningHolder.rectangle(
            insert=insert, nose_radius_mm=0.4, side="inner", width_mm=10, length_mm=80, offset_mm=1, y_mm=(-5, 5)
        )
        tool = camlib.TurningTool(
            "bar", insert=insert, side="inner", hand="right", holder=holder, seat=camlib.TurningSeat(0, 0)
        )
        self.assertIsNone(tool.solid_bar)
        self.assertEqual(sorted(tool.to_dict()["body"]), ["bar_axis_y_mm", "holder_y_mm", "insert"])
        self.assertNotIn("solid_bar", repr(tool))

    def test_contradictions_are_refused_with_the_fix(self):
        refused = (
            (dict(axis_x_mm=2.6), "outside the bar's section.*WF"),
            (dict(axis_y_mm=2.5), "axis_y_mm"),
            (dict(clearance_deg=(20, 8)), "2 angles for an outline of 5 segments"),
            (dict(min_bore_diameter_mm=5), "contradicts its head.*the least bore it enters is 5\\."),
            (dict(max_bore_depth_mm=0), "max_bore_depth_mm"),
        )
        for change, message in refused:
            with self.assertRaisesRegex(ValueError, message):
                solid_bar(head(**change)).validate()
        # A boring head only: not an outside tool, not a blade.
        outside = camlib.TurningTool(
            "solid", insert=head_outline(), side="outer", hand="right", holder=sleeve(), solid_bar=head()
        )
        with self.assertRaisesRegex(ValueError, "boring head.*threading"):
            outside.validate()
        with self.assertRaisesRegex(TypeError, "solid_bar is a SolidBarHead.*GrooveInsert"):
            camlib.TurningTool(
                "g",
                insert=camlib.GrooveInsert(1, 2),
                nose_radius_mm=0.1,
                side="inner",
                hand="right",
                holder=sleeve(),
                solid_bar=head(),
            )


if __name__ == "__main__":
    unittest.main()
