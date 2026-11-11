from camlib.postprocessor import GCodeDialect
from dataclasses import replace
import itertools
import unittest

from camlib.motion import Arc, Helix, Line, Motion, WorkFrame
from camlib.planar import Boundary, Component, PlanarRegion
from camlib.verification import MachineLimits, Options, Prism, Scene, Verifier
from test_simulation import tool


def scene():
    region = PlanarRegion("clamp", (Component(Boundary.polygon(((-0.2, -0.2), (0.2, -0.2), (0.2, 0.2), (-0.2, 0.2)))),))
    return Scene("scene", "cad-1", (Prism("clamp", region, 1, 2),))


class VerificationTest(unittest.TestCase):
    def test_python_machine_normal_acceleration_limit_and_explicit_nominal_scope(self):
        import math
        from camlib.postprocessor import Machine, ToolSetting

        machine = Machine(
            "curve-limits",
            (ToolSetting("t1", 1, 1, 60, (0, 0, 20)),),
            tip_travel_mm=((-100, -100, -100), (100, 100, 100)),
            max_normal_accel_mm_s2=40,
            dialect=GCodeDialect.linuxcnc(),
            output_units='mm',
            decimals=3,
            max_feed_mm_min=10000,
            max_spindle_rpm=30000,
            planes=('xy', 'xz', 'yz'),
            arcs=True,
            helices=True,
            max_sweep_rad=6.283185307179586,
            max_tool_number=99999,
            max_length_register=99999,
            tool_changes=True,
            flood=True,
            mist=True,
            tool_change_position_mode='table',
            rapid_mm_min=6000,
        )
        limits = machine.verification_limits()
        self.assertEqual(limits.max_normal_accel_mm_s2, 40)
        s = Scene("curve-limits", "1", machine=limits)
        arc = Motion(1, "curve", "t1", Arc((2, 0, 0), (2, 0, 0), (0, 0, 0), math.tau), 600)
        helix = replace(arc, curve=Helix((2, 0, 0), (2, 0, 2 * math.tau), (0, 0, 0), math.tau))
        with Verifier(s) as verifier:
            report = verifier.check(tool(), arc)
            self.assertEqual(report["status"], "violation")
            self.assertTrue(report["coverage"]["nominal_normal_accel_checked"])
            self.assertEqual(report["findings"][0]["kind"], "normal_acceleration_limit")
            self.assertIn("excludes junctions", report["findings"][0]["detail"])
            self.assertEqual(verifier.check(tool(), helix)["status"], "clear")
            with self.assertRaises(ValueError):
                verifier.check(tool(), replace(arc, kind="rapid", feed_mm_min=None))
            rapid = verifier.check(
                tool(), replace(arc, kind="rapid", feed_mm_min=None, curve=Line((0, 0, 0), (1, 0, 0)))
            )
            self.assertFalse(rapid["coverage"].get("nominal_normal_accel_checked", False))
        self.assertNotIn("max_normal_accel_mm_s2", replace(limits, max_normal_accel_mm_s2=None).to_dict())
        for value in (0, -1, float('nan'), float('inf')):
            with self.assertRaises(ValueError):
                replace(machine, max_normal_accel_mm_s2=value)

    def test_prepared_python_motion_stream_reports_localized_independent_findings(self):
        motions = (Motion(i, "rough", "t1", Line((-10, y, 0), (10, y, 0)), 600) for i, y in enumerate((0, 10, 0)))
        with Verifier(scene()) as verifier:
            results = list(verifier.check_motions(tool(), motions))
            self.assertEqual([r["status"] for r in results], ["violation", "clear", "violation"])
            hit = results[0]
            self.assertEqual(hit["operation"], "rough")
            self.assertEqual(hit["findings"][0]["object_id"], "clamp")
            self.assertEqual(hit["findings"][0]["witness"]["fraction"], 0.5)
            self.assertEqual(hit["findings"][0]["part"], "cutter")
            self.assertEqual(hit["scene_sha256"], results[1]["scene_sha256"])
            self.assertEqual(len(hit["tool_sha256"]), 64)
            self.assertNotEqual(hit["motion_sha256"], results[1]["motion_sha256"])
            self.assertFalse(hit["coverage"]["remaining_material"])
            self.assertFalse(hit["coverage"]["evolving_stock_contact"])

    def test_scene_geometry_digest_is_checked_even_with_same_id_and_revision(self):
        with Verifier(scene()) as verifier:
            identity = dict(verifier._identity)
            changed = replace(scene(), bodies=(replace(scene().bodies[0], z_min_mm=100, z_max_mm=101),))
            verifier.prepare(changed)
            self.assertNotEqual(identity["scene_sha256"], verifier._identity["scene_sha256"])
            m = Motion(1, "rough", "t1", Line((-10, 0, 0), (10, 0, 0)), 600)
            with self.assertRaisesRegex(ValueError, "identity/revision"):
                verifier._engine._request(
                    "verify_motion", **identity, tool=tool().to_dict(), motion=m.to_dict()["motion"]
                )
            self.assertEqual(verifier.check(tool(), m)["status"], "clear")
            with self.assertRaises(ValueError):
                verifier.prepare(replace(scene(), bodies=(replace(scene().bodies[0], z_min_mm=3, z_max_mm=2),)))
            self.assertEqual(verifier.check(tool(), m)["status"], "clear")

    def test_machine_frame_feed_and_rapid_scope_are_explicit(self):
        limits = MachineLimits((90, -10, -10), (110, 10, 100), 500)
        s = Scene("travel", "1", machine=limits, work_to_machine=WorkFrame(origin_mm=(100, 0, 0)))
        m = Motion(1, "travel", "t1", Line((-1, 0, 0), (1, 0, 0)), 600)
        with Verifier(s) as verifier:
            result = verifier.check(tool(), m)
            self.assertEqual(result["findings"][0]["kind"], "feed_limit")
            rapid = replace(m, kind="rapid", feed_mm_min=None)
            result = verifier.check(tool(), rapid)
            self.assertEqual(result["status"], "clear")
            self.assertFalse(result["coverage"]["declared_fed_rate_checked"])
            self.assertTrue(result["coverage"]["machine_tip_travel_examined"])

    def test_authored_target_floor_tolerance_is_reported_and_does_not_relax_rapid(self):
        floor = replace(scene().bodies[0], role="protected_target", z_min_mm=0, z_max_mm=3)
        target = Scene("machined-floor", "1", (floor,))
        motion = Motion(1, "floor", "t1", Line((-0.1, 0, 3), (0.1, 0, 3)), 600)
        with Verifier(target) as strict:
            self.assertEqual(strict.check(tool(), motion)["status"], "uncertain")
            strict_identity = strict._identity["scene_sha256"]
        with Verifier(target, Options(target_floor_tolerance_mm=0.01)) as verifier:
            self.assertNotEqual(verifier._identity["scene_sha256"], strict_identity)
            report = verifier.check(tool(), motion)
            self.assertEqual(report["status"], "clear")
            self.assertEqual(report["target_floor_tolerance_mm"], 0.01)
            rapid = replace(motion, kind="rapid", feed_mm_min=None)
            self.assertEqual(verifier.check(tool(), rapid)["status"], "uncertain")
            gouge = replace(motion, curve=Line((-0.1, 0, 2.98), (0.1, 0, 2.98)))
            self.assertEqual(verifier.check(tool(), gouge)["status"], "violation")

    def test_work_and_input_bounds_and_uncertainty_are_not_clear(self):
        # Any number of bodies, but an endless iterator is refused unread.
        with self.assertRaisesRegex(TypeError, "finite collection"):
            Scene("huge", "1", itertools.repeat(scene().bodies[0]))
        with Verifier(scene(), Options(max_nodes=1)) as verifier:
            m = Motion(1, "rough", "t1", Line((-10, 0, 0), (1, 0, 0)), 600)
            result = verifier.check(tool(), m)
            self.assertEqual(result["status"], "uncertain")
            self.assertTrue(result["stats"]["budget_exhausted"])
            self.assertGreater(result["stats"]["unexamined_pairs"], 0)


if __name__ == "__main__":
    unittest.main()


class TravelLimitTest(unittest.TestCase):
    """#166: the travel box is closed; a mill at home sits on its limit."""

    def facing_job(self, start, directory):
        import math
        from pathlib import Path

        from camlib import Facing, Job, Setup, StockSpec
        from camlib.export import ExportRefused
        from camlib.post import LoweringOptions, Machine, ToolSetting
        from camlib.tools import EndMill, HolderSection, ToolAssembly
        from camlib.verification import StockContactOptions, StreamChecks

        mill = ToolAssembly(
            "mill10",
            EndMill("em10", diameter=10, flute_length=30, overall_length=75),
            45,
            115,
            (HolderSection(45, 115, 20, 20),),
        )
        work = WorkFrame((200, 150, -300))
        # The gauge line travels X 0..500, Y 0..400, Z -400..0; the tip is 115 mm lower.
        travel = MachineLimits(min_tip_mm=(0, 0, -515), max_tip_mm=(500, 400, -115), max_feed_mm_min=10000)
        face = Facing(
            "face",
            mill,
            floor_z_mm=20,
            stepdown_mm=1,
            stepover_mm=4,
            clearance_z_mm=30,
            feed_mm_min=1000,
            plunge_mm_min=300,
            spindle_rpm=9000,
            coolant="off",
            tool_number=1,
            length_register=1,
        )
        setup = Setup(
            "top",
            StockSpec((-5, -5, 0), (105, 65, 21), 0.5),
            (mill,),
            (face,),
            start,
            verification=StreamChecks(
                scene=Scene("mill", "1", machine=travel, work_to_machine=work), stock_contact=StockContactOptions()
            ),
        )
        machine = Machine(
            "mill",
            (ToolSetting.for_tool(mill, number=1, length_register=1, position_after_mm=start),),
            dialect=GCodeDialect.linuxcnc(),
            work_to_machine=work,
            output_units="mm",
            decimals=3,
            max_feed_mm_min=10000,
            max_spindle_rpm=12000,
            rapid_mm_min=10000,
            planes=("xy",),
            arcs=False,
            helices=False,
            max_sweep_rad=math.tau,
            max_tool_number=1,
            max_length_register=1,
            tool_changes=True,
            flood=False,
            mist=False,
            tool_change_position_mode="preserve_tip",
        )
        with Job("travel", "1", (setup,)).post("top", machine=machine, lowering=LoweringOptions()) as posted:
            posted.wait()
            try:
                posted.export(Path(directory) / "top.nc", accept={"unchecked_machine"})
                return None
            except ExportRefused as refused:
                return refused

    def test_gauge_line_travel_checks_each_tool_against_its_own_tip_travel(self):
        # 176: the gauge line travels X 0..500, Y 0..400, Z -400..0. A 60 mm
        # tool's tip reaches machine Z -60, a 115 mm tool's only Z -115.
        from camlib.tools import EndMill, HolderSection, ToolAssembly

        def mill(id, gauge):
            return ToolAssembly(
                id,
                EndMill("em4", diameter=4, flute_length=30, overall_length=50),
                35,
                gauge,
                (HolderSection(35, gauge, 4, 4),),
            )

        short, long = mill("short", 60), mill("long", 115)
        limits = MachineLimits.gauge_line((0, 0, -400), (500, 400, 0), 10000)
        self.assertEqual(
            limits.to_dict(), dict(max_feed_mm_min=10000, min_gauge_mm=(0, 0, -400), max_gauge_mm=(500, 400, 0))
        )
        s = Scene("mill", "1", machine=limits, work_to_machine=WorkFrame((200, 150, -300)))

        def up(t, z):
            return Motion(1, "op", t.id, Line((0, 0, 100), (0, 0, z)), 600)

        with Verifier(s) as verifier:
            # Work Z 240 is machine Z -60: the short tool's Z home.
            self.assertEqual(verifier.check(short, up(short, 240))["status"], "clear")
            report = verifier.check(long, up(long, 240))
            self.assertEqual(report["status"], "violation")
            self.assertEqual(report["tool"], "long")
            finding = next(f for f in report["findings"] if f["kind"] == "machine_travel")
            self.assertIn("Tool long", finding["detail"])
            self.assertIn("machine Z", finding["detail"])
            self.assertAlmostEqual(finding["witness"]["margin_lower_mm"], 55, places=3)
            # The long tool's own home, on its limit, is inside; 0.001 above is not.
            self.assertEqual(verifier.check(long, up(long, 185))["status"], "clear")
            self.assertEqual(verifier.check(long, up(long, 185.001))["status"], "violation")
            self.assertEqual(verifier.check(short, up(short, 240.001))["status"], "violation")
        # One tool: the gauge line gives the travel its tip box did.
        tip = Scene(
            "mill",
            "1",
            machine=MachineLimits((0, 0, -515), (500, 400, -115), 10000),
            work_to_machine=WorkFrame((200, 150, -300)),
        )
        with Verifier(tip) as a, Verifier(s) as b:
            for z in (150, 185, 185.001, 240):
                x, y = a.check(long, up(long, z)), b.check(long, up(long, z))
                self.assertEqual(x["status"], y["status"], z)
                self.assertEqual(
                    [(f["kind"], f["witness"]["fraction"]) for f in x["findings"]],
                    [(f["kind"], f["witness"]["fraction"]) for f in y["findings"]],
                )
        # A tip box still serializes as it did.
        self.assertEqual(
            tip.machine.to_dict(), dict(min_tip_mm=(0, 0, -515), max_tip_mm=(500, 400, -115), max_feed_mm_min=10000)
        )

    def test_machine_travel_is_declared_once(self):
        import math

        from camlib.post import Machine, ToolSetting

        for arguments in [
            dict(min_tip_mm=None, max_tip_mm=None),
            dict(min_tip_mm=(0, 0, 0), max_tip_mm=None),
            dict(min_tip_mm=(0, 0, 0), max_tip_mm=(1, 1, 1), min_gauge_mm=(0, 0, 0), max_gauge_mm=(1, 1, 1)),
            dict(min_tip_mm=None, max_tip_mm=None, min_gauge_mm=(0, 0, 0)),
        ]:
            with self.assertRaisesRegex(ValueError, "declared once.*min_gauge_mm"):
                MachineLimits(max_feed_mm_min=1000, **arguments)
        # A work frame whose Z is not machine Z: refused, the tool axis is not modelled.
        tilted = WorkFrame((0, 0, 0), (1, 0, 0), (0, 0, 1), (0, -1, 0))
        limits = MachineLimits.gauge_line((-10, -10, -10), (10, 10, 10), 1000)
        with self.assertRaisesRegex(ValueError, "work Z axis along machine Z"):
            Verifier(Scene("s", "1", machine=limits, work_to_machine=tilted))

        def machine(**changes):
            options = dict(
                dialect=GCodeDialect.linuxcnc(),
                output_units="mm",
                decimals=3,
                max_feed_mm_min=10000,
                max_spindle_rpm=30000,
                planes=("xy", "xz", "yz"),
                arcs=True,
                helices=True,
                max_sweep_rad=math.tau,
                max_tool_number=99,
                max_length_register=99,
                tool_changes=True,
                flood=True,
                mist=True,
                tool_change_position_mode="table",
                rapid_mm_min=6000,
            )
            return Machine("mill", (ToolSetting("t1", 1, 1, 60, (0, 0, 20)),), **{**options, **changes})

        gauge = ((0, 0, -400), (500, 400, 0))
        m = machine(gauge_travel_mm=gauge)
        self.assertEqual(m.verification_limits(), MachineLimits.gauge_line(*gauge, 10000))
        self.assertEqual(m.to_dict()["gauge_travel_mm"], gauge)
        # A machine without it is recorded as before.
        self.assertNotIn("gauge_travel_mm", machine().to_dict())
        with self.assertRaisesRegex(ValueError, "declared once"):
            machine(gauge_travel_mm=gauge, tip_travel_mm=gauge)
        with self.assertRaisesRegex(ValueError, "aligned with machine Z"):
            machine(gauge_travel_mm=gauge, work_to_machine=tilted)
        with self.assertRaisesRegex(ValueError, "min below max"):
            machine(gauge_travel_mm=(gauge[1], gauge[0]))
        with self.assertRaisesRegex(ValueError, "a lathe declares tip_travel_mm"):
            machine(gauge_travel_mm=gauge, dialect=replace(GCodeDialect.linuxcnc(), x_diameter=True))
        with self.assertRaisesRegex(ValueError, "tip_travel_mm or gauge_travel_mm"):
            machine().verification_limits()

    def test_a_program_starting_at_home_on_the_limit_is_released(self):
        import tempfile

        with tempfile.TemporaryDirectory() as directory:
            # Home: machine (0, 0, 0) at the gauge line, in work coordinates.
            self.assertIsNone(self.facing_job((-200, -150, 300 - 115), directory))
            # 0.01 mm above the top limit is outside: a violation, not uncertain.
            refused = self.facing_job((-200, -150, 300 - 115 + 0.01), directory)
            self.assertIsNotNone(refused)
            self.assertIn("violation", refused.statuses)
            self.assertIn("machine_travel", str(refused))
