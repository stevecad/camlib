"""Turning tools from outlines, with no default values (issue #76)."""

import unittest

import camlib
from camlib.turning import Arc, Line, Outline
from test_turning import SHAFT, cnmg, job, tool


def head_ahead(ahead, above):
    """A holder whose head reaches ``ahead`` mm ahead of the nose (toward
    -z) at ``above`` mm above it, 30 mm long."""
    x0, x1, z0, z1 = above, above + 30, -ahead, 20
    return camlib.TurningHolder(Outline((x0, z0), (Line((x1, z0)), Line((x1, z1)), Line((x0, z1)), Line((x0, z0)))))


def rough_with(holder, chuck_z_mm=-50):
    """Roughing only, the stock faced: the shaft's two diameters."""
    insert = camlib.TurningInsert.iso(
        included_angle_deg=80, approach_angle_deg=95, edge_length_mm=12, nose_radius_mm=0.8
    )
    t = camlib.TurningTool("cnmg", insert=insert, side="outer", hand="right", holder=holder)
    return job(
        # The stock ends at z -45: no uncut ring between the passes' end and
        # the chuck for a head to meet.
        stock=camlib.TurnStockSpec(-45, 0, 20),
        chuck_z_mm=chuck_z_mm,
        tools=(camlib.TurnToolSetting(t, 1, 1),),
        operations=(camlib.TurnRough("rough", "cnmg", feed_mm_rev=0.25, rpm=1000),),
    )


class NoDefaultsTest(unittest.TestCase):
    def test_leaving_a_value_out_is_refused_by_name(self):
        insert = camlib.TurningInsert.iso(
            included_angle_deg=80, approach_angle_deg=95, edge_length_mm=12, nose_radius_mm=0.8
        )
        holder = camlib.TurningHolder.rectangle(
            insert=insert, nose_radius_mm=0.8, side="outer", width_mm=20, length_mm=100, offset_mm=0
        )
        with self.assertRaisesRegex(TypeError, "needs holder"):
            camlib.TurningTool("t", insert=insert, side="outer", hand="right")
        with self.assertRaisesRegex(TypeError, "needs side, hand"):
            camlib.TurningTool("t", insert=insert, holder=holder)
        with self.assertRaisesRegex(TypeError, "nose_radius_mm"):
            camlib.TurningTool("t", insert=camlib.GrooveInsert(2, 10), side="outer", hand="right", holder=holder)
        with self.assertRaisesRegex(TypeError, "Holder.rectangle"):
            camlib.TurningTool("t", insert=insert, side="outer", hand="right", holder_width_mm=20)
        with self.assertRaisesRegex(TypeError, "TurningInsert.iso"):
            camlib.TurningInsert(80, 95)
        with self.assertRaisesRegex(TypeError, "approach_angle_deg"):
            camlib.TurningInsert.iso(included_angle_deg=80, edge_length_mm=12, nose_radius_mm=0.8)
        with self.assertRaisesRegex(TypeError, "depth_mm"):
            camlib.ThreadInsert(60)
        with self.assertRaisesRegex(TypeError, "point_angle_deg"):
            camlib.DrillInsert(16)
        with self.assertRaisesRegex(TypeError, "depth_mm"):
            camlib.FaceGrooveInsert(2)
        with self.assertRaisesRegex(ValueError, "approach angle"):
            camlib.TurningInsert.iso_code("CNMG120408")
        with self.assertRaisesRegex(TypeError, "offset_mm"):
            camlib.TurningHolder.rectangle(insert=insert, nose_radius_mm=0.8, side="outer", width_mm=20, length_mm=100)

    def test_a_nose_radius_that_contradicts_the_outline_is_refused(self):
        insert = camlib.TurningInsert.iso_code("CNMG120408", approach_angle_deg=95)
        self.assertEqual(insert.nose_radius_mm, 0.8)
        holder = camlib.TurningHolder.rectangle(
            insert=insert, nose_radius_mm=0.8, side="outer", width_mm=25, length_mm=150, offset_mm=0
        )
        tool = camlib.TurningTool("t", insert=insert, side="outer", hand="right", holder=holder)
        self.assertEqual(tool.nose_radius_mm, 0.8)
        tool.validate()
        wrong = camlib.TurningTool("t", insert=insert, side="outer", hand="right", holder=holder, nose_radius_mm=0.4)
        with self.assertRaisesRegex(ValueError, "differs from the insert's corner radius"):
            wrong.validate()


class OutlineTest(unittest.TestCase):
    def test_iso_codes_and_holder_styles(self):
        self.assertEqual(
            camlib.TurningInsert.iso_code("CNMG120408", holder="PCLNR2525M12"),
            camlib.TurningInsert.iso_code("CNMG120408", approach_angle_deg=95),
        )
        round_insert = camlib.TurningInsert.iso_code("RCMT10T3M0", approach_angle_deg=95)
        self.assertEqual(round_insert.nose_radius_mm, 5)
        with self.assertRaisesRegex(ValueError, "give an Outline insert"):
            camlib.TurningInsert.iso_code("KNUX160405", approach_angle_deg=95)
        with self.assertRaisesRegex(ValueError, "tool style"):
            camlib.TurningInsert.iso_code("CNMG120408", holder="PCXNR2525M12")

    def test_round_and_trigon_inserts_turn_the_shaft(self):
        # A round insert of 8 mm: its 4 mm nose fits the shaft's 5 mm shoulder.
        for code in ("RCMT0803M0", "WNMG080408"):
            insert = camlib.TurningInsert.iso_code(code, approach_angle_deg=95)
            t = tool("cnmg", insert, nose_radius_mm=insert.nose_radius_mm, width_mm=20, length_mm=100)
            # The passes end the clearance short of the chuck (for the nose
            # center): a 4 mm nose needs more than the default 2 mm.
            plan = job(tools=(camlib.TurnToolSetting(t, 1, 1),), clearance_mm=insert.nose_radius_mm + 1).plan()
            self.assertEqual(len(plan.reports), 3, code)
            # (None: unbounded, no move near the part.)
            clearances = [r["min_part_clearance_mm"] for r in plan.reports if r["min_part_clearance_mm"] is not None]
            self.assertTrue(clearances, code)
            self.assertTrue(all(c >= -1e-3 for c in clearances), code)

    def test_a_drawn_round_insert_is_the_iso_one(self):
        # A 10 mm round insert drawn by hand: one full-circle arc.
        drawn = camlib.InsertOutline(
            Outline((5, 0), (Arc((0, 0), 5, 0, 360),)), corner=0, cutting=(0,), programming_point_mm=(-5, -5)
        )
        self.assertEqual(drawn, camlib.TurningInsert.iso_code("RCMT10T3M0", approach_angle_deg=95))
        t = tool("r10", drawn, nose_radius_mm=5, width_mm=20, length_mm=100)
        self.assertEqual(t.validate(), t)
        # Its nose circle must lie in it: a corner arc of the wrong radius.
        bad = camlib.InsertOutline(
            Outline((5, 0), (Arc((0, 0), 5, 0, 360),)), corner=0, cutting=(), programming_point_mm=(-5, -5)
        )
        with self.assertRaisesRegex(ValueError, "cutting segments"):
            tool("r10", bad, nose_radius_mm=5, width_mm=20, length_mm=100).validate()

    def test_a_holder_head_ahead_of_the_insert_is_caught(self):
        with self.assertRaisesRegex(Exception, "the holder would hit the part"):
            rough_with(head_ahead(1.5, 2)).plan()
        with self.assertRaisesRegex(Exception, "reach the chuck"):
            rough_with(head_ahead(6, 6)).plan()
        plan = rough_with(head_ahead(1.5, 6)).plan()
        self.assertGreater(plan.reports[0]["min_holder_clearance_mm"], 0)

    def test_the_rectangle_holder_keeps_the_old_toolpaths(self):
        # The shaft with camlib's wedge and rectangle as before #76.
        plan = job().plan()
        self.assertEqual([r["passes"] for r in plan.reports][:1], [2])
        self.assertEqual(SHAFT[1], (10, 0, 0))
        outline = cnmg().holder.outline
        self.assertEqual(outline.segments[0], Line((outline.start_mm[0] + 100, 0)))
        self.assertEqual(outline.segments[1], Line((outline.start_mm[0] + 100, 20)))


if __name__ == "__main__":
    unittest.main()
