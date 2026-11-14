import dataclasses
import importlib.util
from pathlib import Path
import unittest

import camlib
from camlib.simulation import OperationUpdate, Simulation, SimulationError

ROOT = Path(__file__).resolve().parents[2]


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def roof():
    return load("parallel_support", ROOT / "python/tests/support/parallel.py")


def roughed_roof_stock():
    import cadquery as cq
    from camlib.cadquery import export_stock_mesh

    envelope = cq.Workplane("XZ").polyline([(0, 0), (24, 0), (24, 4.1), (12, 8.1), (0, 4.1)]).close().extrude(-16)
    return camlib.StockSpec.from_mesh(export_stock_mesh(envelope), 0.25)


def patterns(tool, surface):
    # Passes entered inside the part under the leftover a rough leaves are
    # ramped in by the declared entry (a plunge would cut into that stock).
    entry = camlib.RampEntry(angle_deg=10, min_length_mm=1)
    ops = {
        "spiral": camlib.Spiral(
            "spiral",
            tool,
            surface,
            0,
            0.75,
            12,
            (12, 8),
            feed_mm_min=600,
            plunge_mm_min=150,
            spindle_rpm=12000,
            tool_number=1,
            length_register=1,
            coolant="flood",
        ),
        "radial": camlib.Radial(
            "radial",
            tool,
            surface,
            0,
            12,
            (12, 8),
            15,
            inner_radius_mm=1,
            feed_mm_min=600,
            plunge_mm_min=150,
            spindle_rpm=12000,
            tool_number=1,
            length_register=1,
            coolant="flood",
        ),
        "scallop": camlib.Scallop(
            "scallop",
            tool,
            surface,
            0,
            0.75,
            12,
            feed_mm_min=600,
            plunge_mm_min=150,
            spindle_rpm=12000,
            tool_number=1,
            length_register=1,
            coolant="flood",
        ),
        "project": camlib.Project(
            "project",
            tool,
            surface,
            0,
            12,
            (
                camlib.TracePath((4, 8), (camlib.TraceLine((20, 8)),)),
                camlib.TracePath((17, 8), (camlib.TraceArc((12, 8), 360),)),
            ),
            feed_mm_min=600,
            plunge_mm_min=150,
            spindle_rpm=12000,
            tool_number=1,
            length_register=1,
            coolant="flood",
        ),
    }
    return {name: dataclasses.replace(op, entry=entry) for name, op in ops.items()}


class SurfacePatternTest(unittest.TestCase):
    def test_patterns_stream_with_stock_contact_checks_and_preserve_the_roof(self):
        config = roof().parallel_job()
        tool, surface = config["tools"][0], config["operations"][0].surface
        for name, op in patterns(tool, surface).items():
            with self.subTest(name):
                # Stock roughed to 0.1 mm over the roof (a planar envelope),
                # as a finishing pass meets it after roughing.
                values = dict(
                    config,
                    stock=roughed_roof_stock(),
                    operations=(op,),
                    verification=camlib.StreamChecks(stock_contact=camlib.StockContactOptions()),
                )
                report, plunges = None, []
                with Simulation(**values) as sim:
                    for update in sim.run_operations():
                        if isinstance(update, OperationUpdate) and update.stage == "end":
                            report = update.report
                        motion = getattr(update, "motion", None)
                        if motion is not None and motion["kind"] == "plunge":
                            plunges.append(list(motion["curve"]["end"]))
                    self.assertTrue(sim.state.finished)
                    self.assertEqual(report["strategy"], name)
                    self.assertEqual(report["rows_completed"], report["rows"])
                    self.assertLessEqual(report["max_certified_path_excess_mm"], 0.005)
                    self.assertGreater(sim.state.volume()["removed_lower_mm3"], 1)

                    def target(x):
                        return 4 + min(x, 24 - x) / 3

                    for y in range(sim.state.ny):
                        for x in range(sim.state.nx):
                            x0, x1 = x * sim.state.dx, (x + 1) * sim.state.dx
                            i = y * sim.state.nx + x
                            # Never below the roof (conservative stock interval).
                            self.assertGreaterEqual(sim.state.heights[2 * i + 1], min(target(x0), target(x1)))

    def test_project_dict_and_precise_refusal(self):
        config = roof().parallel_job()
        tool, surface = config["tools"][0], config["operations"][0].surface
        data = patterns(tool, surface)["project"].to_dict()
        self.assertEqual(data["kind"], "parallel")
        self.assertEqual(data["pattern"]["kind"], "project")
        self.assertNotIn("stepover_mm", data)
        self.assertEqual(data["pattern"]["paths"][1]["segments"][0]["kind"], "arc")
        bad = camlib.Radial(
            "radial",
            tool,
            surface,
            0,
            12,
            (12, 8),
            0,
            feed_mm_min=600,
            plunge_mm_min=150,
            spindle_rpm=12000,
            tool_number=1,
            length_register=1,
            coolant="flood",
        )
        with Simulation(**dict(config, operations=(bad,))) as sim:
            with self.assertRaisesRegex(SimulationError, r"angular_step_deg 0 must be within \(0, 180\]"):
                list(sim.run_operations())


class SteepAndShallowTest(unittest.TestCase):
    def test_roof_flanks_parallel_and_walls_waterline(self):
        config = roof().parallel_job()
        tool, surface = config["tools"][0], config["operations"][0].surface
        steep, shallow = camlib.steep_and_shallow(
            "finish",
            tool,
            surface,
            0,
            12,
            top_z_mm=7.5,
            bottom_z_mm=0.5,
            stepdown_mm=0.5,
            stepover_mm=0.75,
            threshold_deg=30,
            overlap_deg=5,
            feed_mm_min=600,
            plunge_mm_min=150,
            spindle_rpm=12000,
            tool_number=1,
            length_register=1,
            coolant="flood",
        )
        self.assertEqual((steep.slope_from_deg, shallow.slope_to_deg), (25, 35))
        self.assertEqual(shallow.to_dict()["slope_to_deg"], 35)
        self.assertNotIn("slope_from_deg", shallow.to_dict())
        values = dict(
            config,
            operations=(steep, shallow),
            verification=camlib.StreamChecks(stock_contact=camlib.StockContactOptions()),
        )
        cuts = {"finish-steep": [], "finish-shallow": []}
        with Simulation(**values) as sim:
            for update in sim.run_operations():
                motion = getattr(update, "motion", None)
                if motion is not None and motion["kind"] == "cut":
                    cuts[motion["operation"]].append(motion["curve"])
            self.assertTrue(sim.state.finished)
        # Roof flanks slope 18.4 degrees, the end walls (x = 0 and 24, top at
        # Z 4) are vertical. Shallow passes may roll over the wall edges only
        # while the contact angle is at most 35 degrees: the ball center stays
        # at least 2 cos 35 above the edge, so the tip at least 3.6 high; they
        # never run down the walls (apart from the floor beyond them).
        self.assertTrue(cuts["finish-steep"] and cuts["finish-shallow"])
        for curve in cuts["finish-shallow"]:
            for p in (curve["start"], curve["end"]):
                on_flank = 0 <= p[0] <= 24
                self.assertTrue(on_flank or p[2] >= 2 + 2 * 0.819 - 0.05 or p[2] < 0.01, p)
        with self.assertRaisesRegex(ValueError, "threshold_deg 95"):
            camlib.steep_and_shallow(
                "x",
                tool,
                surface,
                0,
                12,
                top_z_mm=7,
                bottom_z_mm=1,
                stepdown_mm=1,
                stepover_mm=1,
                threshold_deg=95,
                feed_mm_min=600,
                plunge_mm_min=150,
                spindle_rpm=12000,
                tool_number=1,
                length_register=1,
                coolant="flood",
            )


class RestFinishingTest(unittest.TestCase):
    def test_rest_passes_follow_the_stock_left_by_the_previous_finish(self):
        config = roof().parallel_job()
        tool, surface = config["tools"][0], config["operations"][0].surface
        # A coarse first pass (1.9 mm stepover) leaves ridges about 0.25 mm
        # high between its rows; rest finishing removes them.
        coarse = camlib.Parallel(
            "coarse",
            tool,
            surface,
            0,
            1.9,
            12,
            axis="y",
            feed_mm_min=600,
            plunge_mm_min=150,
            spindle_rpm=12000,
            tool_number=1,
            length_register=1,
            coolant="flood",
        )
        rest = camlib.Parallel(
            "rest",
            tool,
            surface,
            0,
            0.5,
            12,
            axis="y",
            rest_threshold_mm=0.1,
            feed_mm_min=600,
            plunge_mm_min=150,
            spindle_rpm=12000,
            tool_number=1,
            length_register=1,
            coolant="flood",
        )
        self.assertEqual(rest.to_dict()["rest_threshold_mm"], 0.1)
        self.assertNotIn("rest_threshold_mm", coarse.to_dict())
        values = dict(
            config,
            operations=(coarse, rest),
            verification=camlib.StreamChecks(stock_contact=camlib.StockContactOptions()),
        )
        reports = {}
        with Simulation(**values) as sim:
            for update in sim.run_operations():
                if isinstance(update, OperationUpdate) and update.stage == "end":
                    reports[update.operation] = update.report
            self.assertTrue(sim.state.finished)
        self.assertGreater(reports["rest"]["rows"], 0)
        # Fewer cuts than a full pass at the same stepover.
        full = camlib.Parallel(
            "full",
            tool,
            surface,
            0,
            0.5,
            12,
            axis="y",
            feed_mm_min=600,
            plunge_mm_min=150,
            spindle_rpm=12000,
            tool_number=1,
            length_register=1,
            coolant="flood",
        )
        with Simulation(**dict(config, operations=(coarse, full))) as sim:
            for update in sim.run_operations():
                if isinstance(update, OperationUpdate) and update.stage == "end":
                    reports[update.operation] = update.report
        self.assertLess(reports["rest"]["cut_segments"], reports["full"]["cut_segments"])


if __name__ == "__main__":
    unittest.main()
