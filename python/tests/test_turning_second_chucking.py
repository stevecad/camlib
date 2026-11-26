"""A second chucking (#418): a turned part is made in two chuckings. One end
is turned; the part is taken out, turned end for end and gripped on a
diameter the first chucking finished; the other end is turned.

Every setup here is released the way an application releases it: planned and
streamed, and posted on the FANUC and Centroid lathe posts of ``examples/``
with the posted program decoded again and replayed on the setup's stock.
"""

import importlib.util
import math
import unittest
from pathlib import Path

import camlib
from camlib.post import ToolSetting
from test_turning import job

ROOT = Path(__file__).resolve().parents[2]

# A sawn bar: 32 mm diameter, 64 mm long, 2 mm to face off each end.
BAR = camlib.TurnStockSpec(-62, 2, 16)
# A shaft 60 mm long whose 30 mm diameter runs on to its chuck-side end.
LONG = ((0, 0, 0), (10, 0, 0), (10, -20, 0), (15, -20, 0), (15, -60, 0))


def example(name):
    spec = importlib.util.spec_from_file_location(name, ROOT / f"examples/{name}.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def release(lathe_job, name):
    """The setup's checked NC on a lathe post of ``examples/`` (released by
    ``Job.post_nc``: posted, decoded again and replayed on the stock)."""
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
        decimals=3,
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
    )


def outer(stock, z0, z1):
    """The outer radius bounds of the stock's cells between z0 and z1."""
    spec = stock["spec"]
    out = []
    for i, cell in enumerate(stock["cells"]):
        a = spec["z_min_mm"] + i * spec["cell_mm"]
        b = min(a + spec["cell_mm"], spec["z_max_mm"])
        if a >= z0 and b <= z1:
            out.append((cell["outer"]["lower_mm"], cell["outer"]["upper_mm"]))
    return out


def recorded(lathe_job):
    """A recorded run of the setup: its stock before any move, its final
    stock and the material check of that (the stock against the part)."""
    with lathe_job.record("lathe", rapid_mm_min=6000) as run:
        run.wait()
        player = camlib.Player(run)
        player.seek(camlib.Cursor())
        start = player.state.stock()
        player.seek(camlib.Cursor(run.status()["available"]))
        return start, player.state.stock(), player.state.material()


def material_cells(stock):
    """The stock's cells that may hold material."""
    return [c for c in stock["cells"] if c["outer"]["upper_mm"] > c["inner"]["lower_mm"]]


def z_words(text):
    """Every Z word of a posted program."""
    return [float(w[1:]) for line in text.splitlines() for w in line.split() if w[:1] == "Z"]


def checked(material):
    """What a material check found: the gouge assessment, the cells of
    certain leftover and those behind a gripping chuck's face."""
    return material["gouge"]["assessment"], material["residual"]["definite_cells"], material["held"]


class JawsOnThePartTest(unittest.TestCase):
    """The jaws on the part itself: ``TurningSetup(grip=ChuckGrip(...))`` lets
    the chuck face sit along the part; nothing behind it is machined, and a
    cut that would reach the jaws is refused by name."""

    def test_a_chuck_face_along_the_part_needs_a_declared_grip(self):
        # The jaws hold the bar from z -62 to -44; the part runs to z -60.
        with self.assertRaisesRegex(Exception, "in front of the chuck face.*grip"):
            job(stock=BAR, profile=LONG, chuck_z_mm=-44).plan()
        held = job(stock=BAR, profile=LONG, chuck_z_mm=-44, grip=camlib.ChuckGrip(length_mm=18, radius_mm=16))
        plan = held.plan()
        # Turned to the clearance in front of the chuck face; behind the
        # face the bar is as it was.
        self.assertTrue(all(abs(lo - 15) < 1e-6 and abs(hi - 15) < 1e-6 for lo, hi in outer(plan.stock, -41, -21.5)))
        self.assertEqual(set(outer(plan.stock, -62, -44)), {(16, 16)})
        # What the jaws hold is reported as left for another setup, by the
        # operations and by the material check: leftover, not a failure.
        reports = {r["id"]: r for r in plan.reports}
        self.assertNotIn("left_for_another_setup_z_mm", reports["face"])
        for name in ("rough", "finish"):
            self.assertEqual(reports[name]["left_for_another_setup_z_mm"], [[-60, -42]])
        material = reports["finish"]["material"]
        self.assertEqual(material["held"]["z_mm"], [-62, -44])
        self.assertGreater(material["held"]["definite_cells"], 50)
        self.assertEqual(material["gouge"]["assessment"], "within_tolerance")
        # It releases on both lathe posts: the posted program, read back
        # and replayed on the stock, stays in front of the jaws too.
        for name in ("fanuc_lathe_post", "centroid_lathe_post"):
            with self.subTest(post=name):
                posted = release(held.job, name)
                self.assertGreaterEqual(min(z_words(posted.text)), -42.8 - 1e-9)
                self.assertEqual(posted.material["held"], material["held"])

    def test_a_setup_without_a_grip_reports_nothing_new(self):
        plan = job().plan()
        for report in plan.reports:
            self.assertNotIn("left_for_another_setup_z_mm", report)
            self.assertNotIn("held", report["material"])
        self.assertNotIn("grip", plan.job.compile().setups[0].config["turning"])

    def test_a_cut_that_would_reach_the_jaws_is_refused_by_name(self):
        grip = camlib.ChuckGrip(18, 16)
        behind = camlib.TurnFace("face", "cnmg", -45, feed_mm_rev=0.15, rpm=1000)
        with self.assertRaisesRegex(Exception, r"face: its z_mm \(z -45\) lies behind the chuck face.*jaws hold"):
            job(stock=BAR, profile=LONG, chuck_z_mm=-44, grip=grip, operations=(behind,)).plan()

    def test_a_grip_is_checked_against_the_stock(self):
        for grip, message in (
            (camlib.ChuckGrip(18, 15), "jaws hold a parallel diameter"),
            (camlib.ChuckGrip(30, 16), "the stock ends at z -62"),
        ):
            with self.subTest(grip=grip), self.assertRaisesRegex(Exception, message):
                job(stock=BAR, profile=LONG, chuck_z_mm=-44, grip=grip).plan()


# The LONG shaft turned around: its 30 mm diameter first, then the 20 mm
# diameter the first chucking finished.
LONG_TURNED = ((0, 0, 0), (15, 0, 0), (15, -40, 0), (10, -40, 0), (10, -60, 0))


class CarriedStockTest(unittest.TestCase):
    """The stock carried between the setups: ``TurnStockSpec.from_turned``
    makes the second setup's stock from what the first left, turned end for
    end; ``TurnStockSection`` declares a blank that is not a bar."""

    def setUp(self):
        # First chucking: the bar held by 18 mm, turned to z -42.
        self.first = job(stock=BAR, profile=LONG, chuck_z_mm=-44, grip=camlib.ChuckGrip(18, 16)).plan()

    def test_the_second_setup_starts_from_what_the_first_left(self):
        left = self.first.stock
        # Turned end for end: the part's chuck-side end (z -60) becomes the
        # front face (z 0); the faced-off front of the first setup is gone.
        stock = camlib.TurnStockSpec.from_turned(left, reversed=True, z_shift_mm=-60)
        self.assertEqual((stock.z_min_mm, stock.z_max_mm, stock.outer_radius_mm, stock.cell_mm), (-60, 2, 16, 0.25))
        second = job(stock=stock, profile=LONG_TURNED, chuck_z_mm=-42, grip=camlib.ChuckGrip(18, 10))
        # Before any move, the run's stock is the first setup's final stock,
        # cell for cell, in the other order.
        start, final, material = recorded(second.job)
        self.assertEqual(start["cells"], material_cells(left)[::-1])
        self.assertAlmostEqual(start["remaining_upper_mm3"], left["remaining_upper_mm3"], places=6)
        self.assertAlmostEqual(start["remaining_lower_mm3"], left["remaining_lower_mm3"], places=6)
        # The whole part is at size, both ends: the stock against the part,
        # independently of the planner.
        self.assertTrue(all(abs(lo - 15) < 1e-6 and abs(hi - 15) < 1e-6 for lo, hi in outer(final, -39.5, -0.5)))
        self.assertTrue(all(abs(lo - 10) < 1e-6 and abs(hi - 10) < 1e-6 for lo, hi in outer(final, -59.5, -41.5)))
        part = math.pi * (15**2 * 40 + 10**2 * 20)
        # (The certain bound loses the cell at each faced end and shoulder,
        # the possible bound holds the nose's fillets.)
        self.assertTrue(0.985 * part < final["remaining_lower_mm3"] <= part <= final["remaining_upper_mm3"])
        self.assertLess(final["remaining_upper_mm3"], part + 25)
        # Leftover only in the fillet the 0.8 mm nose leaves at the
        # shoulder; nothing behind the chuck face, where the first setup
        # finished the diameter the jaws are on.
        self.assertEqual(material["held"], dict(z_mm=[-60, -42], definite_cells=0, possible_only_cells=0))
        leftover = [f for f in material["findings"] if f["residual"] == "definite_excess"]
        self.assertTrue(leftover)
        self.assertTrue(all(-41.05 <= f["z_mm"][0] and f["z_mm"][1] <= -40 for f in leftover), leftover)
        # The second setup releases on both lathe posts: its posted
        # program, read back, replays on the carried stock.
        for name in ("fanuc_lathe_post", "centroid_lathe_post"):
            with self.subTest(post=name):
                posted = release(second.job, name)
                self.assertEqual(posted.material["held"], material["held"])
                self.assertGreaterEqual(min(z_words(posted.text)), -40.8 - 1e-9)

    def test_a_wrong_stock_model_is_refused(self):
        # The second setup on an untouched bar instead of what the first
        # left: the jaws would close on 32 mm, not on the 20 mm declared.
        with self.assertRaisesRegex(Exception, "jaws hold a parallel diameter"):
            job(
                stock=camlib.TurnStockSpec(-60, 2, 16),
                profile=LONG_TURNED,
                chuck_z_mm=-42,
                grip=camlib.ChuckGrip(18, 10),
            ).plan()
        # The chuck face set onto the shoulder of the carried stock.
        stock = camlib.TurnStockSpec.from_turned(self.first.stock, reversed=True, z_shift_mm=-60)
        with self.assertRaisesRegex(Exception, "jaws hold a parallel diameter"):
            job(stock=stock, profile=LONG_TURNED, chuck_z_mm=-39, grip=camlib.ChuckGrip(18, 10)).plan()

    def test_a_declared_stepped_blank(self):
        # A forging: 34 mm diameter for 42 mm, then 24 mm to its front end.
        blank = camlib.TurnStockSpec(
            -62,
            2,
            17,
            sections=(
                camlib.TurnStockSection((-62, -20), outer_mm=17),
                camlib.TurnStockSection((-20, 2), outer_mm=(12, 12), inner_mm=0),
            ),
        )
        plan = job(stock=blank).plan()
        self.assertEqual(set(outer(plan.stock, -18.5, -1.5)), {(10, 10)})
        # Sections that do not tile the stock are refused by name.
        gap = camlib.TurnStockSpec(-62, 2, 17, sections=(camlib.TurnStockSection((-62, -20), outer_mm=17),))
        with self.assertRaisesRegex(Exception, "sections end at z -20"):
            job(stock=gap).plan()
        # A bar's manifest carries no sections.
        self.assertNotIn("sections", camlib.TurnStockSpec(-62, 2, 17).to_dict())
        with self.assertRaisesRegex(ValueError, "outer_mm is a radius"):
            camlib.TurnStockSection((-62, 2), outer_mm="wide").to_dict()

    def test_a_stock_with_nothing_left_is_refused(self):
        empty = dict(self.first.stock)
        cell = dict(outer=dict(lower_mm=0, upper_mm=0), inner=dict(lower_mm=0, upper_mm=0))
        empty["cells"] = [cell] * len(empty["cells"])
        with self.assertRaisesRegex(ValueError, "no material left"):
            camlib.TurnStockSpec.from_turned(empty)


# A shaft 60 mm long: a 20 mm diameter, a 30 mm collar and a 24 mm diameter,
# 20 mm each. The collar's far shoulder faces away from the tool, whichever
# end is held.
A_SIDE = ((0, 0, 0), (10, 0, 0), (10, -20, 0), (15, -20, 0), (15, -40, 0), (12, -40, 0), (12, -60, 0))
# The same shaft turned around (z becomes -60 - z).
B_SIDE = ((0, 0, 0), (12, 0, 0), (12, -20, 0), (15, -20, 0), (15, -40, 0), (10, -40, 0), (10, -60, 0))


def turn_to(z_to_mm, z_from_mm=None):
    """Face, then rough and finish the profile between the two z."""
    bounds = dict(z_from_mm=z_from_mm, z_to_mm=z_to_mm)
    return (
        camlib.TurnFace("face", "cnmg", 0, feed_mm_rev=0.15, rpm=1000),
        camlib.TurnRough("rough", "cnmg", feed_mm_rev=0.25, rpm=1000, **bounds),
        camlib.TurnFinish("finish", "cnmg", feed_mm_rev=0.1, rpm=1200, **bounds),
    )


class BoundsTest(unittest.TestCase):
    """What each chucking turns: ``TurnRough`` and ``TurnFinish`` take
    ``z_from_mm`` / ``z_to_mm``; the rest of the profile is reported as
    left for another setup."""

    HELD = dict(stock=BAR, profile=A_SIDE, chuck_z_mm=-44, grip=camlib.ChuckGrip(18, 16))

    def test_roughing_and_finishing_turn_the_stretch_they_are_given(self):
        # The whole profile as the target: the finish is refused at the
        # collar's far shoulder, which the insert's trailing edge cannot
        # follow.
        with self.assertRaisesRegex(Exception, "finish: the insert would cut .* into the part"):
            job(**self.HELD).plan()
        # Told to turn as far as that shoulder, the setup plans, and
        # reports the rest as left for another setup.
        plan = job(**self.HELD, operations=turn_to(-40)).plan()
        self.assertTrue(all(abs(lo - 15) < 1e-6 and abs(hi - 15) < 1e-6 for lo, hi in outer(plan.stock, -39.5, -21.5)))
        self.assertEqual(set(outer(plan.stock, -62, -41)), {(16, 16)})
        reports = {r["id"]: r for r in plan.reports}
        for name in ("rough", "finish"):
            self.assertEqual(reports[name]["left_for_another_setup_z_mm"], [[-60, -40]])
        # The posted programs end at the bound too: the tool tip one nose
        # radius behind the nose centre.
        for name in ("fanuc_lathe_post", "centroid_lathe_post"):
            with self.subTest(post=name):
                self.assertAlmostEqual(min(z_words(release(plan.job, name).text)), -40.8, places=9)

    def test_a_front_bound_inside_the_profile(self):
        # The 20 mm end is roughed; then the 30 mm diameter behind the
        # shoulder is roughed and finished on its own.
        operations = (
            camlib.TurnFace("face", "cnmg", 0, feed_mm_rev=0.15, rpm=1000),
            camlib.TurnRough("rough", "cnmg", feed_mm_rev=0.25, rpm=1000, z_to_mm=-20),
            camlib.TurnRough("rough30", "cnmg", feed_mm_rev=0.25, rpm=1000, z_from_mm=-20),
            camlib.TurnFinish("finish", "cnmg", feed_mm_rev=0.1, rpm=1200, z_from_mm=-20),
        )
        plan = job(stock=BAR, profile=LONG, chuck_z_mm=-44, grip=camlib.ChuckGrip(18, 16), operations=operations).plan()
        self.assertTrue(all(abs(lo - 15) < 1e-6 and abs(hi - 15) < 1e-6 for lo, hi in outer(plan.stock, -41, -21)))
        # The end in front of the bound keeps its roughing allowance.
        self.assertTrue(
            all(abs(lo - 10.3) < 1e-6 and abs(hi - 10.3) < 1e-6 for lo, hi in outer(plan.stock, -17.5, -1.5))
        )
        reports = {r["id"]: r for r in plan.reports}
        self.assertEqual(reports["finish"]["left_for_another_setup_z_mm"], [[-60, -42], [-20, 0]])
        for name in ("fanuc_lathe_post", "centroid_lathe_post"):
            with self.subTest(post=name):
                self.assertTrue(release(plan.job, name).text)

    def test_bounds_are_refused_by_name(self):
        for bounds, message in (
            (dict(z_to_mm=-43), r"finish: z_to_mm -43 lies behind the clearance in front of the chuck face.*jaws hold"),
            (dict(z_from_mm=-30, z_to_mm=-20), "z_from_mm -30 must be in front of"),
            (dict(z_to_mm=0.5), "must be behind the part's front face"),
        ):
            finish = camlib.TurnFinish("finish", "cnmg", feed_mm_rev=0.1, rpm=1200, **bounds)
            with self.subTest(bounds=bounds), self.assertRaisesRegex(Exception, message):
                job(**self.HELD, operations=(finish,)).plan()

    def test_an_operation_without_bounds_is_declared_as_before(self):
        for operation in (
            camlib.TurnRough("rough", "cnmg", feed_mm_rev=0.25, rpm=1000),
            camlib.TurnFinish("finish", "cnmg", feed_mm_rev=0.1, rpm=1200),
        ):
            self.assertFalse({"z_from_mm", "z_to_mm"} & set(operation.to_dict()))
        self.assertEqual(camlib.TurnRough("rough", "cnmg", 1.5, 0.2, feed_mm_rev=0.25, rpm=1000).depth_of_cut_mm, 1.5)


class TwoChuckingsTest(unittest.TestCase):
    """The whole job: a stepped shaft in two chuckings."""

    def test_a_stepped_shaft_in_two_chuckings_releases_on_both_lathe_posts(self):
        # First chucking: the sawn bar in the jaws by 18 mm. Faced, then the
        # 20 mm end and the collar roughed and finished as far as the
        # collar's far shoulder.
        first = job(stock=BAR, profile=A_SIDE, chuck_z_mm=-44, grip=camlib.ChuckGrip(18, 16), operations=turn_to(-40))
        _, left, after_first = recorded(first.job)
        self.assertEqual(after_first["gouge"]["assessment"], "within_tolerance")
        # The other end is still the bar: leftover, behind the chuck face.
        self.assertGreater(after_first["held"]["definite_cells"], 50)
        # Second chucking: the part turned around (the old z -60 is the new
        # front face), its stock what the first left, gripped on the
        # finished 20 mm diameter by 15 mm with 5 mm of it in front of the
        # jaws. Faced to length, the 24 mm end turned up to the collar.
        stock = camlib.TurnStockSpec.from_turned(left, reversed=True, z_shift_mm=-60)
        second = job(
            stock=stock, profile=B_SIDE, chuck_z_mm=-45, grip=camlib.ChuckGrip(15, 10), operations=turn_to(-20)
        )
        start, final, material = recorded(second.job)
        self.assertEqual(start["cells"], material_cells(left)[::-1])
        # The stock is the part, end to end, by the drawing's own numbers.
        self.assertFalse(material_cells(dict(cells=final["cells"][-8:])))  # faced to length (2 mm, 8 cells)
        for z0, z1, radius in ((-18.5, -0.5, 12), (-39.5, -20.5, 15), (-59.5, -41.5, 10)):
            cells = outer(final, z0, z1)
            self.assertTrue(
                cells and all(abs(lo - radius) < 1e-6 and abs(hi - radius) < 1e-6 for lo, hi in cells), radius
            )
        part = math.pi * 20 * (12**2 + 15**2 + 10**2)
        # (The certain bound loses the cell at each faced end and shoulder,
        # the possible bound holds the nose's fillets.)
        self.assertTrue(0.985 * part < final["remaining_lower_mm3"] <= part <= final["remaining_upper_mm3"])
        self.assertLess(final["remaining_upper_mm3"], part + 100)
        # The material check agrees: no gouge, leftover only in the two
        # fillets the 0.8 mm nose leaves at the shoulders it cut toward,
        # none of it behind the chuck face.
        self.assertEqual(material["gouge"]["assessment"], "within_tolerance")
        self.assertEqual(material["held"], dict(z_mm=[-60, -45], definite_cells=0, possible_only_cells=0))
        leftover = [f["z_mm"] for f in material["findings"] if f["residual"] == "definite_excess"]
        self.assertTrue(leftover)
        self.assertTrue(all(-41.05 <= a and b <= -40 or -20 <= a and b <= -18.95 for a, b in leftover), leftover)
        # Both setups release on both lathe posts: each posted program is
        # decoded again, compared with the plan and replayed on its
        # setup's stock, and ends with the same stock as the plan.
        for name in ("fanuc_lathe_post", "centroid_lathe_post"):
            with self.subTest(post=name):
                posted = release(first.job, name)
                self.assertEqual(checked(posted.material), checked(after_first))
                self.assertAlmostEqual(min(z_words(posted.text)), -40.8, places=9)
                posted = release(second.job, name)
                self.assertFalse(posted.findings)
                self.assertEqual(checked(posted.material), checked(material))
                # Nothing is programmed behind the second setup's bound,
                # 22 mm in front of the jaws.
                self.assertAlmostEqual(min(z_words(posted.text)), -20.8, places=9)
        # A cut that would reach the jaws is refused by name: turning on
        # to the diameter they hold, and a groove under them.
        held = dict(stock=stock, profile=B_SIDE, chuck_z_mm=-45, grip=camlib.ChuckGrip(15, 10))
        with self.assertRaisesRegex(
            Exception, r"z_to_mm -44 lies behind the clearance in front of the chuck face \(z -43.*jaws hold the work"
        ):
            job(**held, operations=turn_to(-44)).plan()
        groove = camlib.TurnGroove("groove", "cnmg", -50, -53, 8, feed_mm_rev=0.05, rpm=800)
        with self.assertRaisesRegex(Exception, r"groove: .* lies behind the chuck face \(z -45\).*jaws hold"):
            job(**held, operations=(groove,)).plan()
        # And the second setup on the wrong stock (the untouched bar) is
        # refused where its jaws would close.
        with self.assertRaisesRegex(Exception, "jaws hold a parallel diameter"):
            job(**{**held, "stock": camlib.TurnStockSpec(-60, 2, 16)}, operations=turn_to(-20)).plan()


if __name__ == "__main__":
    unittest.main()
