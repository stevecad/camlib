"""Roughing a profile that leaves a diameter by a taper (#426): the pass at
the diameter's own level ends where the taper begins, the passes above it
step down the taper, and the setup is released on the FANUC and Centroid
lathe posts of ``examples/`` (posted, decoded again and replayed on the
stock)."""

import math
import unittest

import camlib
from camlib.post import ToolSetting
from test_turning import job
from test_turning_second_chucking import example, moves, outer

# A shaft: 20 mm diameter for 20 mm, a taper up to 30 mm over 10 mm, then
# 30 mm diameter.
TAPERED = ((0, 0, 0), (10, 0, 0), (10, -20, 0), (15, -30, 0), (15, -40, 0))
NOSE, ALLOWANCE = 0.8, 0.3
ANGLE = math.atan(0.5)
# Where the diameter's and the taper's offsets (nose plus allowance) meet:
# behind the corner at z -20 by the offset times the tangent of half the
# taper's angle.
CORNER_Z = -20 + (NOSE + ALLOWANCE) * math.tan(ANGLE / 2)


def operations(depth, finish=True, **bounds):
    turned = (
        camlib.TurnFace("face", "cnmg", 0, feed_mm_rev=0.15, rpm=1000),
        camlib.TurnRough(
            "rough", "cnmg", depth_of_cut_mm=depth, allowance_mm=ALLOWANCE, feed_mm_rev=0.25, rpm=1000, **bounds
        ),
        camlib.TurnFinish("finish", "cnmg", feed_mm_rev=0.1, rpm=1200, **bounds),
    )
    return turned if finish else turned[:2]


def taper_radius(z):
    return 10 + (-20 - z) / 2


def cells(stock, z0, z1):
    """(bin start, bin end, outer lower, outer upper) of the stock's cells
    between z0 and z1."""
    spec = stock["spec"]
    out = []
    for i, cell in enumerate(stock["cells"]):
        a = spec["z_min_mm"] + i * spec["cell_mm"]
        b = min(a + spec["cell_mm"], spec["z_max_mm"])
        if a >= z0 and b <= z1:
            out.append((a, b, cell["outer"]["lower_mm"], cell["outer"]["upper_mm"]))
    return out


def release(lathe_job, name):
    """The setup's checked NC on a lathe post of ``examples/`` (``Job.post_nc``:
    posted, decoded again and replayed on the stock). The finishing pass
    rolls the nose round the convex corner where the taper meets the 30 mm
    diameter: that arc has no radius-consistent G2/G3 words at 3 or 4
    decimals (docs/posts.md), so it is posted as checked line segments
    (``LoweringOptions(native_arcs=True)``). Those lie within the machine's
    tolerance_mm of the arc, and the replay holds them to the setup's
    tolerance_mm (0.001 mm), so the machine is declared at 4 decimals with
    that tolerance (its chords at half of it)."""
    module = example(name)
    fanuc = "fanuc" in name
    header = lathe_job.compile().setups[0].program_header()
    tools = tuple(
        ToolSetting.for_tool(t, number=i, length_register=i, position_after_mm=header.initial_position_mm)
        for i, t in enumerate(header.tools, 1)
    )
    machine = module.define_machine(
        "Lathe",
        tools,
        output_units="mm",
        decimals=4,
        tolerance_mm=0.001,
        max_feed_mm_min=5000,
        arcs=True,
        helices=False,
        max_sweep_rad=math.tau,
        max_tool_number=12,
        max_length_register=12,
        tool_changes=True,
        flood=True,
        mist=False,
        rapid_mm_min=8000,
        max_spindle_rpm=4000,
        **(dict(program_number=1001) if fanuc else {}),
    )
    return lathe_job.post_nc(
        "lathe",
        machine=machine,
        post=module.FanucLathePost if fanuc else module.CentroidLathePost,
        decoder=module.FanucLatheDecoder if fanuc else module.CentroidLatheDecoder,
        accept={"unchecked_machine"},
        lowering=camlib.LoweringOptions(native_arcs=True, chord_tolerance_mm=0.0005),
    )


class DiameterThenTaperTest(unittest.TestCase):
    def test_roughing_steps_down_the_taper_and_leaves_the_allowance(self):
        for depth in (2, 1.7):
            with self.subTest(depth_of_cut_mm=depth):
                plan = job(profile=TAPERED, operations=operations(depth, finish=False)).plan()
                report = {r["id"]: r for r in plan.reports}["rough"]
                self.assertEqual(report["material"]["gouge"]["assessment"], "within_tolerance")
                # The diameter is left at its allowance.
                on_diameter = outer(plan.stock, -18.5, -1.5)
                self.assertTrue(on_diameter)
                self.assertTrue(all(abs(lo - 10.3) < 1e-6 and abs(hi - 10.3) < 1e-6 for lo, hi in on_diameter))
                # The taper is left in steps: never less than the allowance,
                # never more than it and one depth of cut (measured along the
                # radius, where the offset path is 1.1 / cos(angle) above the
                # taper and the nose's lowest point 0.8 below that).
                on_taper = cells(plan.stock, -29, -21)
                self.assertGreater(len(on_taper), 20)
                step = (NOSE + ALLOWANCE) / math.cos(ANGLE) - NOSE + depth
                for a, b, lower, upper in on_taper:
                    self.assertGreaterEqual(lower, taper_radius(b) + ALLOWANCE - 1e-6, (a, b))
                    self.assertLessEqual(upper, taper_radius(a) + step + 1e-6, (a, b))

    def test_the_shaft_is_released_on_both_lathe_posts(self):
        for depth, bounds in ((2, {}), (1.7, {}), (2, dict(z_to_mm=-25))):
            lathe = job(profile=TAPERED, operations=operations(depth, **bounds))
            plan = lathe.plan()
            self.assertEqual([r["id"] for r in plan.reports], ["face", "rough", "finish"])
            # Finished to the drawing: the diameter, and the taper as far as
            # it was turned.
            self.assertTrue(
                all(abs(lo - 10) < 1e-6 and abs(hi - 10) < 1e-6 for lo, hi in outer(plan.stock, -18.5, -1.5))
            )
            taper_end = bounds.get("z_to_mm", -30) + 1
            on_taper = cells(plan.stock, taper_end, -21)
            self.assertGreater(len(on_taper), 10)
            for a, b, lower, upper in on_taper:
                self.assertGreaterEqual(lower, taper_radius(b) - 2e-3, (a, b))
                self.assertLessEqual(upper, taper_radius(a) + 2e-3, (a, b))
            material = {r["id"]: r for r in plan.reports}["finish"]["material"]
            self.assertEqual(material["gouge"]["assessment"], "within_tolerance")
            if not bounds:
                # Leftover only in the inside corner between the diameter
                # and the taper, which the 0.8 mm nose cannot reach.
                leftover = [f["z_mm"] for f in material["findings"] if f["residual"] == "definite_excess"]
                self.assertTrue(all(-20.5 <= a and b <= -19.5 for a, b in leftover), leftover)
            for name in ("fanuc_lathe_post", "centroid_lathe_post"):
                with self.subTest(depth_of_cut_mm=depth, bounds=bounds, post=name):
                    posted = release(lathe.job, name)
                    self.assertFalse(posted.findings)
                    self.assertEqual(posted.material["gouge"]["assessment"], "within_tolerance")
                    blocks = moves(posted.text)
                    # The roughing pass at the diameter's own level: the tool
                    # tip at X20.6 (the diameter and twice the allowance),
                    # ending where the taper begins (the tip one nose radius
                    # behind the nose centre), not further along the taper.
                    level = [(g, x, z) for g, x, z in blocks if g == 1 and abs(x - 20.6) < 1e-9 and z < -1]
                    self.assertEqual(level, [(1, 20.6, round(CORNER_Z - NOSE, 4))])


if __name__ == "__main__":
    unittest.main()
