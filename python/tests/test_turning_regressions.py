"""Turning jobs that were refused although valid (#100, #101, #102).

- #100: an outside thread plans, and its posted program (FANUC and Centroid
  lathe posts) replays: the replay checks each pass against the part with
  the thread cut, as the planner does. No false "no cutting motion" warning.
- #101: a groove exactly one blade wide is one centred plunge (face and
  radial grooves), not a second pass a rounding error away.
- #102: an inside groove after the front was faced off plans: emptied bins
  in front of the bore don't close the hole the tool enters through.
"""

import importlib.util
import math
import unittest
import warnings
from pathlib import Path

import camlib
from camlib.post import ToolSetting

ROOT = Path(__file__).resolve().parents[2]


def example(name):
    spec = importlib.util.spec_from_file_location(name, ROOT / f"examples/{name}.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def holder(insert, rn, side="outer", width=4, length=60, offset=0):
    return camlib.TurningHolder.rectangle(
        insert=insert, nose_radius_mm=rn, side=side, width_mm=width, length_mm=length, offset_mm=offset
    )


CNMG_I = camlib.TurningInsert.iso_code("CNMG120408", holder="PCLNR2525M12")
CNMG = camlib.TurningTool(
    "cnmg", insert=CNMG_I, side="outer", hand="right", holder=holder(CNMG_I, 0.8, width=25, length=150)
)


def motions(setup, operation):
    out = []
    with camlib.Job("j", "1", (setup,)).regenerate() as run:
        for event in run:
            update = event.update
            if hasattr(update, "motion") and update.motion["operation"] == operation:
                out.append(update.motion)
    return out


def face_groove_setup(outer):
    insert = camlib.FaceGrooveInsert(3, 10)
    tool = camlib.TurningTool(
        "fg",
        insert=insert,
        nose_radius_mm=0.2,
        side="outer",
        hand="right",
        holder=holder(insert, 0.2, width=3, length=40),
    )
    return camlib.Setup(
        "s",
        stock=camlib.TurnStockSpec(z_min_mm=-60, z_max_mm=0, outer_radius_mm=18),
        tools=(camlib.TurnToolSetting(tool, 1, 1),),
        operations=(camlib.TurnFaceGroove("g", "fg", 13, outer, -3, peck_mm=1, feed_mm_rev=0.03, rpm=600),),
        initial_position_mm=(40, 0, 20),
        turning=camlib.TurningSetup(
            profile=((0, 0, 0), (18, 0, 0), (18, -40, 0)),
            chuck_z_mm=-50,
            clearance_mm=2,
            tolerance_mm=1e-3,
            face_grooves=(camlib.turning.FaceGroove(13, outer, -3),),
        ),
    )


def radial_groove_setup(width):
    insert = camlib.GrooveInsert(3, 4)
    tool = camlib.TurningTool(
        "g3",
        insert=insert,
        nose_radius_mm=0.2,
        side="outer",
        hand="right",
        holder=holder(insert, 0.2, width=3, length=40),
    )
    profile = (
        (0, 0, 0),
        (15, 0, 0),
        (15, -20, 0),
        (12, -20, 0),
        (12, -20 - width, 0),
        (15, -20 - width, 0),
        (15, -50, 0),
    )
    return camlib.Setup(
        "s",
        stock=camlib.TurnStockSpec(z_min_mm=-60, z_max_mm=0, outer_radius_mm=15),
        tools=(camlib.TurnToolSetting(tool, 1, 1),),
        operations=(camlib.TurnGroove("g", "g3", -20 - width, -20, 12, feed_mm_rev=0.04, rpm=700),),
        initial_position_mm=(40, 0, 20),
        turning=camlib.TurningSetup(profile=profile, chuck_z_mm=-55, clearance_mm=2, tolerance_mm=1e-3),
    )


def inside_groove_setup(z_max, face):
    cone = 8 / math.tan(math.radians(59))
    drill_insert = camlib.DrillInsert(16, 118, 50)
    drill = camlib.TurningTool(
        "drill",
        insert=drill_insert,
        nose_radius_mm=0,
        side="outer",
        hand="right",
        holder=holder(drill_insert, 0, width=40),
    )
    bar_i = camlib.TurningInsert.iso(included_angle_deg=55, approach_angle_deg=95, edge_length_mm=6, nose_radius_mm=0.4)
    bar = camlib.TurningTool(
        "bar",
        insert=bar_i,
        side="inner",
        hand="right",
        holder=holder(bar_i, 0.4, side="inner", width=10, length=80, offset=1),
    )
    gi = camlib.GrooveInsert(3, 4)
    groove = camlib.TurningTool(
        "ig",
        insert=gi,
        nose_radius_mm=0.2,
        side="inner",
        hand="right",
        holder=holder(gi, 0.2, side="inner", width=3, length=80, offset=4.5),
    )
    bore = (
        (11, 0, 0),
        (11, -18, 0),
        (12.5, -18, 0),
        (12.5, -21, 0),
        (11, -21, 0),
        (11, -25, 0),
        (8, -25, 0),
        (0, -25 - cone, 0),
    )
    operations = ((camlib.TurnFace("face", "cnmg", 0, feed_mm_rev=0.15, rpm=1000),) if face else ()) + (
        camlib.TurnDrill("drill", "drill", -25 - cone, peck_mm=10, feed_mm_rev=0.12, rpm=900),
        camlib.TurnBoreRough("rough", "bar", -24.6, allowance_mm=0, feed_mm_rev=0.15, rpm=1000),
        camlib.TurnGroove("groove", "ig", -18, -21, 12.5, feed_mm_rev=0.04, rpm=700),
    )
    return camlib.Setup(
        "s",
        stock=camlib.TurnStockSpec(z_min_mm=-60, z_max_mm=z_max, outer_radius_mm=18),
        tools=tuple(camlib.TurnToolSetting(t, i, i) for i, t in enumerate((CNMG, drill, bar, groove), 1)),
        operations=operations,
        initial_position_mm=(40, 0, 20),
        turning=camlib.TurningSetup(
            profile=((0, 0, 0), (18, 0, 0), (18, -40, 0)), bore=bore, chuck_z_mm=-50, clearance_mm=2, tolerance_mm=1e-3
        ),
    )


class GrooveWidthTest(unittest.TestCase):
    def test_face_groove_one_blade_wide_is_one_centred_plunge(self):
        exact = motions(face_groove_setup(16.0), "g")
        plunges = [m for m in exact if m["kind"] == "plunge"]
        # One radial position for every peck, between the walls.
        radii = {round(m["curve"]["end"][0], 9) for m in plunges}
        self.assertEqual(len(radii), 1)
        self.assertTrue(all(m["curve"]["start"] != m["curve"]["end"] for m in exact))
        # Half a millimetre wider: two plunge positions.
        wider = motions(face_groove_setup(16.5), "g")
        self.assertEqual(len({round(m["curve"]["end"][0], 9) for m in wider if m["kind"] == "plunge"}), 2)

    def test_radial_groove_one_blade_wide_is_one_plunge(self):
        exact = motions(radial_groove_setup(3.0), "g")
        zs = {round(m["curve"]["end"][2], 9) for m in exact if m["kind"] == "plunge"}
        self.assertEqual(len(zs), 1)
        self.assertEqual(
            len(
                {
                    round(m["curve"]["end"][2], 9)
                    for m in motions(radial_groove_setup(3.5), "g")
                    if m["kind"] == "plunge"
                }
            ),
            2,
        )


class InsideGrooveAfterFacingTest(unittest.TestCase):
    def test_faced_stock_plans_the_same_inside_groove(self):
        faced = motions(inside_groove_setup(2, True), "groove")
        at_face = motions(inside_groove_setup(0, False), "groove")
        self.assertTrue(faced)

        def cuts(ms):
            return [(m["kind"], m["curve"]["start"], m["curve"]["end"]) for m in ms if m["kind"] != "rapid"]

        # The groove's cutting moves are the same; only its rapids enter and
        # leave in front of each stock's own top.
        self.assertTrue(cuts(faced))
        self.assertEqual(cuts(faced), cuts(at_face))


class ThreadPostingTest(unittest.TestCase):
    def setUp(self):
        insert = camlib.ThreadInsert(60, 3)
        thread = camlib.TurningTool(
            "thread60",
            insert=insert,
            nose_radius_mm=0.1,
            side="outer",
            hand="right",
            holder=holder(insert, 0.1, width=4, length=60),
        )
        setup = camlib.Setup(
            "lathe",
            stock=camlib.TurnStockSpec(z_min_mm=-60, z_max_mm=2, outer_radius_mm=20),
            tools=(camlib.TurnToolSetting(CNMG, 1, 1), camlib.TurnToolSetting(thread, 2, 2)),
            operations=(
                camlib.TurnFace("face", "cnmg", z_mm=0, feed_mm_rev=0.15, rpm=1000),
                camlib.TurnRough("rough", "cnmg", feed_mm_rev=0.25, rpm=1000),
                camlib.TurnFinish("finish", "cnmg", feed_mm_rev=0.1, rpm=1200),
                camlib.TurnThread("thread", "thread60", -20, -38, 15, 2, 1.227, rpm=600),
            ),
            initial_position_mm=(30, 0, 20),
            turning=camlib.TurningSetup(
                profile=((0, 0, 0), (10, 0, 0), (10, -20, 0), (15, -20, 0), (15, -43, 0)),
                chuck_z_mm=-50,
                clearance_mm=2,
                tolerance_mm=1e-3,
            ),
        )
        self.job = camlib.Job("shaft", "1", (setup,))
        header = self.job.compile().setups[0].program_header()
        self.tools = tuple(
            ToolSetting.for_tool(t, number=i, length_register=i, position_after_mm=header.initial_position_mm)
            for i, t in enumerate(header.tools, 1)
        )
        self.declared = dict(
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
        )

    def test_thread_posts_and_replays_with_both_lathe_posts(self):
        for name, extra in (("fanuc_lathe_post", dict(program_number=1001)), ("centroid_lathe_post", {})):
            module = example(name)
            post = module.FanucLathePost if "fanuc" in name else module.CentroidLathePost
            decoder = module.FanucLatheDecoder if "fanuc" in name else module.CentroidLatheDecoder
            machine = module.define_machine("Lathe", self.tools, **self.declared, **extra)
            with self.subTest(post=name), self.job.post("lathe", machine=machine, post=post, decoder=decoder) as posted:
                posted.wait(timeout=300)
                status = posted.status()
                self.assertTrue(status["finished"])
                self.assertIsNone(status["error"])

    def test_thread_passes_count_as_cutting(self):
        with warnings.catch_warnings(record=True) as caught:
            warnings.simplefilter("always")
            with self.job.record("lathe", rapid_mm_min=6000) as run:
                run.wait()
        self.assertFalse([w for w in caught if "thread" in str(w.message)], [str(w.message) for w in caught])


if __name__ == "__main__":
    unittest.main()
