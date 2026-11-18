"""One decoded taper move swept by a continuous range of round inserts."""

from dataclasses import replace
import math
import unittest

import camlib
from camlib.post import post_program
from test_turning import job, lathe, tool


def exact_contact(start, end, radius, station, tip, side=1):
    """Intersect a horizontal ray with the capsule via point/segment distance."""
    offset = radius if tip else 0
    ax, az = start[0] + side * offset, start[2] + offset
    dx, dz = end[0] - start[0], end[2] - start[2]
    at_center = ax + (station - az) * dx / dz
    outside, inside = at_center - side * 3 * radius, at_center
    for _ in range(60):
        x = (outside + inside) / 2
        t = min(1, max(0, ((x - ax) * dx + (station - az) * dz) / (dx * dx + dz * dz)))
        if math.hypot(x - ax - t * dx, station - az - t * dz) <= radius:
            inside = x
        else:
            outside = x
    return inside


class TaperTest(unittest.TestCase):
    def test_post_once_replay_the_same_segment_for_every_radius(self):
        insert = camlib.TurningInsert.iso_code("RCMT0803M0", approach_angle_deg=95)
        cutter = tool("cnmg", insert, nose_radius_mm=4, width_mm=20, length_mm=100)
        for datum in ("tip", "nose_center"):
            with self.subTest(datum=datum):
                # Drawing radius at Z=-20 is 12 mm.
                plan = job(
                    profile=((0, 0, 0), (10, 0, 0), (14, -40, 0)),
                    tools=(camlib.TurnToolSetting(cutter, 1, 1),),
                    controlled_point=datum,
                    clearance_mm=5,
                ).plan()
                setup = plan.job.compile().setup("lathe")
                chunks = list(post_program(plan.program, machine=lathe(plan)))
                ((chunk, index, move),) = [
                    (c, i, m)
                    for c in chunks
                    for i, m in enumerate(c.motions)
                    if m.kind == "cut" and m.operation == "finish" and m.curve.start[2] > -20 > m.curve.end[2]
                ]
                args = dict(motion_index=index, profile_segment=1, nose_radius_range_mm=(3.9, 4.1), z_mm=-20)
                section = setup.taper_section(chunk, **args)
                lo, hi = section["radius_mm"]
                observed = [
                    exact_contact(move.curve.start, move.curve.end, 3.9 + i / 100, -20, datum == "tip")
                    for i in range(21)
                ]
                self.assertLessEqual(lo, min(observed))
                self.assertGreaterEqual(hi, max(observed))
                self.assertAlmostEqual(lo, min(observed), places=10)
                self.assertAlmostEqual(hi, max(observed), places=10)
                nominal = setup.taper_section(chunk, **(args | dict(nose_radius_range_mm=(4, 4))))
                nominal_lo, nominal_hi = nominal["radial_error_mm"]
                self.assertLessEqual(nominal_lo, observed[10] - 12)
                self.assertGreaterEqual(nominal_hi, observed[10] - 12)
                self.assertLess(nominal_hi - nominal_lo, 1e-10)
                self.assertLessEqual(section["radial_error_mm"][0], min(observed) - 12)
                self.assertGreaterEqual(section["radial_error_mm"][1], max(observed) - 12)
                for change in (dict(z_mm=-80), dict(nose_radius_range_mm=(4.1, 4.2)), dict(profile_segment=0)):
                    with self.assertRaises(ValueError):
                        setup.taper_section(chunk, **(args | change))
                with self.assertRaises(ValueError):
                    replace(setup, id="other").taper_section(chunk, **args)

    def test_bore_reports_error_against_the_bore_profile(self):
        insert = camlib.TurningInsert.iso_code("RCMT0803M0", approach_angle_deg=95)
        bar = tool("bar", insert, nose_radius_mm=4, width_mm=2, length_mm=60, offset_mm=1, side="inner")
        plan = job(
            profile=((0, 0, 0), (28, 0, 0), (28, -40, 0)),
            bore=((17, 0, 0), (15, -30, 0)),
            stock=camlib.TurnStockSpec(-60, 0, 30, bore_radius_mm=14.9),
            tools=(camlib.TurnToolSetting(bar, 1, 1),),
            clearance_mm=5,
            operations=(camlib.TurnBoreFinish("bf", "bar", -22, feed_mm_rev=0.08, rpm=1200),),
        ).plan()
        setup = plan.job.compile().setup("lathe")
        ((chunk, i, move),) = [
            (c, i, m)
            for c in post_program(plan.program, machine=lathe(plan))
            for i, m in enumerate(c.motions)
            if m.kind == "cut"
        ]
        args = dict(motion_index=i, profile_segment=0, nose_radius_range_mm=(3.9, 4.1), z_mm=-15)
        section = setup.taper_section(chunk, **args)
        lo, hi = section["radial_error_mm"]
        for r in (3.9, 3.95, 4.0, 4.05, 4.1):
            observed = exact_contact(move.curve.start, move.curve.end, r, -15, True, side=-1)
            self.assertLessEqual(lo, observed - 16)
            self.assertGreaterEqual(hi, observed - 16)
        self.assertLess(hi - lo, 0.02)
