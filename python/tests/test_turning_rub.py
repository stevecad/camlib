"""Every part of a turning tool but its cutting face is held to the stock as
it is at each instant of a cutting move (#220).

Roughing a plain r 15 shaft from an r 20 bar in passes that run off the
bar's far end, so the stock a pass leaves shows nothing of what the tool met
on the way:

- an insert seated with 15 degrees of positive rake has its heel ahead of
  its major cutting edge (rake turns the insert about that edge, which runs
  almost radially on a 95 degree tool: the 3 mm body's bottom moves 0.78 mm
  along -Z, its 7 degrees of clearance give back 0.37 mm). The heel ploughs
  the shoulder the edge is about to cut. It reaches no deeper than the nose,
  which the old check compared;
- a holder head ahead of the insert, its underside below the bar's surface,
  runs into the bar before the insert has cut it;
- the same underside behind the insert follows it through what the pass has
  cut, and a flat or negative seat keeps the heel behind the edge: planned.
"""

import unittest

import camlib
from camlib import turning
from test_turning import job, tool

INSERT = camlib.TurningInsert.iso(included_angle_deg=80, approach_angle_deg=95, edge_length_mm=12, nose_radius_mm=0.8)


def rough(turning_tool):
    return job(
        stock=camlib.TurnStockSpec(-45, 0, 20),
        profile=((0, 0, 0), (15, 0, 0), (15, -40, 0)),
        tools=(camlib.TurnToolSetting(turning_tool, 1, 1),),
        operations=(camlib.TurnRough("rough", "cnmg", feed_mm_rev=0.25, rpm=1000),),
    )


def seated(rake_deg):
    return tool("cnmg", INSERT, nose_radius_mm=0.8, width_mm=20, length_mm=100, seat=camlib.TurningSeat(rake_deg, 0))


def with_head(front_z_mm, underside_mm):
    """The insert under a head from ``front_z_mm`` (from the nose center;
    negative: ahead of it, toward -Z) whose underside is ``underside_mm``
    above the nose center."""
    outline = turning.Outline(
        (underside_mm, front_z_mm),
        (
            turning.Line((underside_mm + 30, front_z_mm)),
            turning.Line((underside_mm + 30, 20)),
            turning.Line((underside_mm, 20)),
            turning.Line((underside_mm, front_z_mm)),
        ),
    )
    return camlib.TurningTool(
        "cnmg",
        insert=INSERT,
        nose_radius_mm=0.8,
        side="outer",
        hand="right",
        holder=camlib.TurningHolder(outline, y_mm=(-10, 2)),
        seat=camlib.TurningSeat(0, 0),
        insert_thickness_mm=3,
        insert_clearance_deg=7,
    )


class BodyAgainstTheStockOfTheInstantTest(unittest.TestCase):
    def test_a_flat_or_negative_seat_keeps_the_heel_behind_the_edge(self):
        for rake in (0, -6):
            self.assertEqual(len(rough(seated(rake)).plan().reports), 1)

    def test_a_heel_ahead_of_the_cutting_edge_is_refused_on_the_first_pass(self):
        # The first pass: 2 mm deep, the nose (radius 0.8) at r 18.
        with self.assertRaisesRegex(
            Exception,
            r"rough: the insert's body below its cutting face would rub the stock \(.* moving to r 18\.800, z -48\.000",
        ):
            rough(seated(15)).plan()

    def test_a_head_ahead_of_the_insert_meets_the_uncut_bar(self):
        # The underside at r 18.8 + 1 = 19.8: 0.2 mm into the r 20 bar.
        with self.assertRaisesRegex(
            Exception,
            r"rough: the holder would hit stock the cutting edge has not yet cut away \(0\.199. mm .* moving to r 18\.800, z -48\.000",
        ):
            rough(with_head(-1.5, 1)).plan()

    def test_the_same_head_behind_the_insert_follows_it(self):
        self.assertEqual(len(rough(with_head(1, 1)).plan().reports), 1)


if __name__ == "__main__":
    unittest.main()
