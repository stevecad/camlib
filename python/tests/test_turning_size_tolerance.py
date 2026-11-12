"""The insert's in-plane size tolerance (#113).

An ISO 1832 code's tolerance class bounds the insert's inscribed circle d and
its corner dimension m. The tool is measured at its nose, so the nose and the
two cutting edges at it stay where they are and the rest of the insert may
reach farther: the checks use the largest insert the tolerance allows.

- The class values come from the code (ISO 1832:2017 tables 3 to 5); a code
  the standard gives no value for is refused, naming what to declare.
- The cut does not change with the tolerance: the same moves, the same stock.
- A boring insert passing a bore its nominal body just fits is refused at
  the bore's far wall when its tolerance lets it reach it.
"""

import unittest

import camlib
from test_turning import SHAFT, job, tool

CODE = "CCMT060204"


def bar(size_tolerance_mm=None, **declared):
    """A CCMT0602 boring insert on a thin bar; its size tolerance from its
    code (class M: d ±0.05, m ±0.08 mm) unless given."""
    insert = camlib.TurningInsert.iso_code(CODE, approach_angle_deg=95)
    if size_tolerance_mm is not None:
        declared["insert_size_tolerance_mm"] = size_tolerance_mm
    return tool(
        "bar",
        insert,
        nose_radius_mm=insert.nose_radius_mm,
        width_mm=2,
        length_mm=60,
        offset_mm=1,
        side="inner",
        insert_thickness_mm=1.5,
        insert_clearance_deg=11,
        **declared,
    )


def bore(radius, bar_tool):
    """Finish a bore of ``radius`` already open to 0.2 mm under it."""
    return job(
        stock=camlib.TurnStockSpec(-60, 0, 20, bore_radius_mm=radius - 0.2),
        profile=((0, 0, 0), (18, 0, 0), (18, -40, 0)),
        bore=((radius, 0, 0), (radius, -30, 0)),
        tools=(camlib.TurnToolSetting(bar_tool, 2, 2),),
        operations=(camlib.TurnBoreFinish("bore_finish", "bar", -20, feed_mm_rev=0.08, rpm=1200),),
    )


class CodeTest(unittest.TestCase):
    def test_the_class_gives_the_tolerance(self):
        for code, expected in (
            ("CNMG120408", (0.08, 0.13)),  # class M, 12.7 mm (table 4)
            ("CCMT09T304", (0.05, 0.08)),
            ("DCGT11T302", (0.025, 0.025)),  # class G (table 3)
            ("DNMG150608", (0.08, 0.15)),  # shape D (table 5)
            ("VNMG160408", (0.05, 0.16)),  # shape V (table 5)
            ("SNUN120408", (0.13, 0.2)),  # class U
            ("RCMT10T3M0", None),  # round: d is the nose
        ):
            insert = camlib.TurningInsert.iso_code(code, approach_angle_deg=62.5)
            self.assertEqual(insert.size_tolerance_mm, expected, code)

    def test_a_code_without_a_standard_value_is_refused_until_declared(self):
        # Class U of shape D, and a V insert of 4.76 mm: not in tables 4, 5.
        for code in ("DNUG150608", "VBMT080204"):
            with self.assertRaisesRegex(ValueError, r"gives no tolerance.*size_tolerance_mm"):
                camlib.TurningInsert.iso_code(code, approach_angle_deg=93)
            insert = camlib.TurningInsert.iso_code(code, approach_angle_deg=93, size_tolerance_mm=(0.1, 0.2))
            self.assertEqual(insert.size_tolerance_mm, (0.1, 0.2))
        with self.assertRaisesRegex(ValueError, r"\(d, m\)"):
            camlib.TurningInsert.iso_code("CNMG120408", approach_angle_deg=95, size_tolerance_mm=(0.1,))

    def test_the_tool_declares_it(self):
        self.assertEqual(bar().insert_size_tolerance_mm, (0.05, 0.08))
        self.assertEqual(bar().to_dict()["body"]["insert"]["size_tolerance"], dict(d_mm=0.05, m_mm=0.08))
        self.assertEqual(bar((0.02, 0.03)).to_dict()["body"]["insert"]["size_tolerance"], dict(d_mm=0.02, m_mm=0.03))
        bar().validate()
        bar((0, 0)).validate()
        # A wedge has no inscribed circle; a grooving blade's size is its width.
        wedge = camlib.TurningInsert.iso(
            included_angle_deg=80, approach_angle_deg=95, edge_length_mm=12, nose_radius_mm=0.8
        )
        self.assertIsNone(wedge.size_tolerance_mm)
        declared = tool(
            "t", wedge, nose_radius_mm=0.8, width_mm=20, length_mm=100, insert_size_tolerance_mm=(0.08, 0.13)
        )
        with self.assertRaisesRegex(ValueError, "ISO 1832 form"):
            declared.validate()
        with self.assertRaisesRegex(TypeError, "is an InsertOutline's"):
            tool(
                "g",
                camlib.GrooveInsert(3, 10),
                nose_radius_mm=0.2,
                width_mm=3,
                length_mm=40,
                insert_size_tolerance_mm=(0.08, 0.13),
            )
        with self.assertRaisesRegex(ValueError, "0..1 mm"):
            bar((2, 0)).validate()


class PlanTest(unittest.TestCase):
    def test_the_cut_is_the_same_with_the_tolerance(self):
        # Rough and finish the shaft with a CNMG: the edge along the
        # profile, the nose on it. The tolerance moves neither.
        def plan(**declared):
            insert = camlib.TurningInsert.iso_code("CNMG120408", holder="PCLNR2525M12")
            self.assertEqual(insert.size_tolerance_mm, (0.08, 0.13))
            t = tool(
                "cnmg",
                insert,
                nose_radius_mm=0.8,
                width_mm=20,
                length_mm=100,
                insert_thickness_mm=insert.thickness_mm,
                insert_clearance_deg=insert.clearance_deg,
                **declared,
            )
            return job(profile=SHAFT, tools=(camlib.TurnToolSetting(t, 1, 1),)).plan()

        sized, exact = plan(), plan(insert_size_tolerance_mm=(0, 0))
        self.assertEqual(len(sized.reports), 3)
        self.assertEqual(sized.stock, exact.stock)
        self.assertEqual(
            [(r["id"], r["moves"], r["cut_length_mm"]) for r in sized.reports],
            [(r["id"], r["moves"], r["cut_length_mm"]) for r in exact.reports],
        )

    def test_the_largest_insert_is_checked_at_the_bores_far_wall(self):
        # The insert reaches about 6.9 mm from its nose across the bore, and
        # the pass ends by pulling the nose off the wall: in a bore of about
        # 9 mm the insert's far corner then nears the far wall. Over a
        # range of bores the insert measured to be exactly nominal fits and
        # the class M insert, which may be larger, is refused.
        fits = {"exact": [], "class": []}
        refusals = []
        for step in range(30):
            radius = 4.2 + 0.02 * step
            for name, tolerance in (("exact", (0, 0)), ("class", None)):
                try:
                    bore(radius, bar(tolerance)).plan()
                    fits[name].append(step)
                except Exception as error:  # noqa: BLE001
                    if name == "class" and step in fits["exact"]:
                        refusals.append(str(error))
        self.assertTrue(fits["exact"] and fits["class"], fits)
        # Each fits every bore larger than its smallest.
        for steps in fits.values():
            self.assertEqual(steps, list(range(steps[0], 30)), fits)
        # The class insert needs a larger bore: its far side may stand up
        # to 2 x 0.08 sin 40 + 0.05 = 0.15 mm, its far corner 2 x 0.08 +
        # 0.05 mm, farther from its nose (the union of the readings of m),
        # and the wall it nears is as much farther from the nose as the
        # bore's diameter grows.
        steps = fits["class"][0] - fits["exact"][0]
        self.assertTrue(2 <= steps <= 6, fits)
        self.assertTrue(refusals)
        for message in refusals:
            self.assertRegex(message, "at its nominal size the insert clears.*inscribed circle ±0.05 mm")


if __name__ == "__main__":
    unittest.main()
