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


if __name__ == "__main__":
    unittest.main()
