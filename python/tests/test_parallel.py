from dataclasses import replace
import importlib.util
import itertools
import os
from pathlib import Path
import unittest
from unittest import mock

import camlib
from camlib import Engine
from camlib.cadquery import export_surface_mesh
from camlib.operations import Facing
from camlib.playback import Cursor, Player, RecordedRun
from camlib.material import Options, SurfaceTarget, Verifier
from camlib.simulation import OperationUpdate, ReachRefused, Simulation, SimulationError, VerificationUpdate
from camlib.surface import SurfaceMesh
from camlib.tools import EndMill, ToolAssembly, HolderSection
from camlib.verification import StockContactOptions, StreamChecks


def example():
    path = Path(__file__).resolve().parents[2] / "python/tests/support/parallel.py"
    spec = importlib.util.spec_from_file_location("parallel_example", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class ParallelTest(unittest.TestCase):
    def test_actual_cad_surface_streams_three_axis_removal_and_preserves_roof(self):
        module = example()
        config = module.parallel_job()
        op = config["operations"][0]
        self.assertEqual(op.surface.source["adapter"], "planar_cad_solid_facets_v1")
        self.assertAlmostEqual(module.target().val().Volume(), 2304)
        first = False
        z_moves = 0
        report = None
        with Simulation(**config) as sim:
            for update in sim.run_operations():
                if isinstance(update, OperationUpdate):
                    if update.stage == "end":
                        report = update.report
                else:
                    curve = update.motion["curve"]
                    if update.motion["kind"] == "cut":
                        z_moves += abs(curve["start"][2] - curve["end"][2]) > 0.001
                    if not first and sim.state.volume()["removed_lower_mm3"] > 0.001:
                        first = True
                        self.assertFalse(sim.state.finished)  # streamed: before the run's end
                        self.assertFalse(sim.state.finished)
            self.assertTrue(first and sim.state.finished)
            self.assertGreater(z_moves, 10)
            self.assertEqual(report["rows_completed"], report["rows"])
            self.assertLessEqual(report["max_certified_path_excess_mm"], 0.005)
            self.assertGreater(report["work"]["distance_tests"], 0)
            self.assertEqual(report["geometric_residual_status"], "unknown")
            self.assertIn("not_certified", report["surface_completion"])
            for y in range(sim.state.ny):
                for x in range(sim.state.nx):
                    x0 = x * sim.state.dx
                    x1 = (x + 1) * sim.state.dx

                    def target(x):
                        return 4 + min(x, 24 - x) / 3

                    low = min(target(x0), target(x1))
                    i = y * sim.state.nx + x
                    # A conservative stock interval can straddle the target.
                    # Native tests separately prove whole-segment roof safety
                    # and compare every cell with an exact swept-ball oracle.
                    self.assertGreaterEqual(sim.state.heights[2 * i + 1], low)
                    high = 8 if x0 <= 12 <= x1 else max(target(x0), target(x1))
                    self.assertLessEqual(sim.state.heights[2 * i + 1], high + 0.15)
            volume = sim.state.volume()
            self.assertGreater(volume["removed_lower_mm3"], 1400)
            self.assertLessEqual(volume["removed_lower_mm3"], 1536)

    def test_two_tool_job_and_partial_sloped_segment_seek_replay(self):
        config = example().parallel_job()
        face_tool = ToolAssembly(
            "face8",
            EndMill("em8", diameter=8, flute_length=20, overall_length=50),
            30,
            60,
            (HolderSection(30, 60, 5, 5),),
        )
        face = Facing(
            "face",
            face_tool,
            9,
            1,
            6,
            12,
            feed_mm_min=600,
            plunge_mm_min=150,
            spindle_rpm=12000,
            tool_number=1,
            length_register=1,
            coolant="flood",
        )
        op = replace(config["operations"][0], tool_number=2, length_register=2)
        config.update(tools=(face_tool, *config["tools"]), operations=(face, op))
        events = []
        partial_motion = None
        with Simulation(**config) as sim:
            for update in sim.run_operations():
                if isinstance(update, OperationUpdate):
                    events.extend(update.events)
                else:
                    events.append(dict(kind="motion", motion=update.motion))
                    c = update.motion["curve"]
                    if (
                        partial_motion is None
                        and update.motion["kind"] == "cut"
                        and abs(c["end"][2] - c["start"][2]) > 0.001
                        and all(4 <= p[0] <= 8 and 0 <= p[1] <= 16 for p in (c["start"], c["end"]))
                        and update.changed_count
                    ):
                        partial_motion = update.completed
            count = sim.state.completed
        program = dict(
            version=1,
            id="surface-program",
            revision="1",
            units="mm",
            tolerance_mm=1e-5,
            frame=dict(origin_mm=(0, 0, 0), x=(1, 0, 0), y=(0, 1, 0), z=(0, 0, 1)),
            initial_position_mm=(0, 0, 12),
            tools=[t.to_dict() for t in config["tools"]],
            events=events,
        )
        with Engine() as engine:
            self.assertEqual(
                engine._request("validate_program", program=program, capabilities=None)["summary"]["motions"], count
            )
        self.assertIsNotNone(partial_motion)
        with RecordedRun(**{'rapid_mm_min': 6000, **config}, checkpoint_every=16) as run:
            player = Player(run)
            player.pause()
            run.wait()
            player.seek(Cursor(partial_motion - 1))
            before = player.state.heights.tobytes()
            player.seek(Cursor(partial_motion - 1, 0.5))
            partial = player.state.heights.tobytes()
            self.assertNotEqual(before, partial)
            volume = player.state.volume()["removed_lower_mm3"]
            self.assertGreater(volume, 0.001)
            player.seek(Cursor(run.status()["available"]))
            self.assertGreater(player.state.volume()["removed_lower_mm3"], volume)
            player.seek(Cursor(partial_motion - 1, 0.5))
            self.assertEqual(player.state.heights.tobytes(), partial)

    def test_cad_approximation_bounded_input_and_invalid_before_motion(self):
        import cadquery as cq

        with self.assertRaisesRegex(ValueError, "max_facets"):
            export_surface_mesh(cq.Workplane("XY").sphere(4), max_facets=4)
        # No count cap: an endless mesh is refused by the memory budget.
        with mock.patch.dict(os.environ, {"CAMLIB_MEMORY_BUDGET_BYTES": str(64 << 20)}):
            with self.assertRaisesRegex(ValueError, "surface mesh triangles needs"):
                SurfaceMesh(itertools.repeat(((0, 0, 0), (1, 0, 0), (0, 1, 0))))
        with self.assertRaisesRegex(ValueError, "three coordinates"):
            SurfaceMesh(((itertools.repeat(1), (1, 0, 0), (0, 1, 0)),))
        config = example().parallel_job()
        config["operations"] = (replace(config["operations"][0], safety_margin_mm=0),)
        with Simulation(**config) as sim:
            with self.assertRaisesRegex(SimulationError, "parallel finishing"):
                list(sim.run_operations())
            self.assertEqual(sim.state.completed, 0)


class OneWayTest(unittest.TestCase):
    """Issue #175: every row cut in the same direction, the return a link."""

    def run_rows(self, **changes):
        config = example().parallel_job()
        op = replace(config["operations"][0], retract_z_mm=11, feed_z_mm=10.5, **changes)
        config.update(operations=(op,), verification=StreamChecks(stock_contact=StockContactOptions()))
        motions, contact, report = [], [], None
        with Simulation(**config) as sim:
            for update in sim.run_operations():
                if isinstance(update, OperationUpdate):
                    if update.stage == "end":
                        report = update.report
                elif isinstance(update, VerificationUpdate):
                    if update.kind == "motion":
                        contact.append(update.check["stock_contact"])
                else:
                    motions.append(update.motion)
            heights = list(sim.state.heights)
            material = Verifier(
                SurfaceTarget("roof", "parallel-1", op.surface, 0, source_enclosure="offset"),
                Options(max_findings=4096),
            ).check(sim.state)
        self.assertEqual(report["rows_completed"], report["rows"])
        return op, motions, contact, report, heights, material

    def test_default_is_zigzag_and_its_manifest_is_unchanged(self):
        op = example().parallel_job()["operations"][0]
        self.assertFalse(op.one_way)
        self.assertNotIn("one_way", op.to_dict())
        self.assertIs(replace(op, one_way=True).to_dict()["one_way"], True)

    def test_rows_cut_one_direction_and_return_by_checked_rapid_links(self):
        for reverse, forward in ((False, 1), (True, -1)):
            _, zigzag, _, zig_report, zig_heights, zig_material = self.run_rows(reverse=reverse)
            self.assertNotIn("one_way", zig_report)
            op, motions, contact, report, heights, material = self.run_rows(one_way=True, reverse=reverse)
            self.assertIs(report["one_way"], True)
            returns = cuts = 0
            for motion in motions:
                a, b = motion["curve"]["start"], motion["curve"]["end"]
                along = (b[0] - a[0]) * forward
                if motion["kind"] == "cut":
                    cuts += 1
                    self.assertGreater(along, 0, motion)
                    self.assertEqual(a[1], b[1])
                if motion["kind"] != "rapid":
                    self.assertGreaterEqual(along, 0, motion)
                elif along < 0 and cuts:
                    # The return (after the first row; before it the tool
                    # arrives from the start position): over the whole part
                    # at the retract height.
                    returns += 1
                    self.assertEqual((a[2], b[2]), (11, 11))
                    self.assertGreater(-along, 24)
            self.assertGreater(cuts, report["rows"])
            self.assertEqual(returns, report["rows"] - 1)
            # Every rapid and link was checked against the stock it met.
            self.assertGreater(len(contact), returns)
            self.assertEqual({c["status"] for c in contact}, {"clear"})
            self.assertTrue(all(c["fully_examined"] for c in contact))
            # The surface left: the same stock as zigzag cell by cell within
            # the operation's tolerance (either way a row lies between the
            # exact no-gouge height and safety_margin_mm + path_error_mm
            # above it); above the part; and the material verifier's verdict
            # against the part is the same.
            self.assertEqual(len(heights), len(zig_heights))
            band = op.path_error_mm + op.safety_margin_mm
            self.assertLessEqual(max(abs(a - b) for a, b in zip(heights, zig_heights)), band + 1e-9)
            # No cell's upper bound is below the roof under it (no gouge).
            for i in range(len(heights) // 2):
                x0 = (i % 96) * 0.25
                low = min(4 + min(x, 24 - x) / 3 for x in (x0, x0 + 0.25))
                self.assertGreaterEqual(heights[2 * i + 1], low)
            self.assertTrue(material["complete"])
            self.assertEqual(material["gouge"]["definite_cells"], 0)
            for side in ("gouge", "residual"):
                for key in ("assessment", "definite_cells", "possible_only_cells"):
                    self.assertEqual(material[side][key], zig_material[side][key])
            # Zigzag (the default) does cut both ways.
            signs = {m["curve"]["end"][0] > m["curve"]["start"][0] for m in zigzag if m["kind"] == "cut"}
            self.assertEqual(signs, {True, False})

    def test_other_patterns_refuse_one_way(self):
        config = example().parallel_job()
        op = config["operations"][0]
        manifest = dict(op.to_dict(), one_way=True, pattern=dict(kind="spiral", center_mm=(12, 8)))

        class Spiral:
            id, tool = op.id, op.tool

            def to_dict(self):
                return manifest

        config["operations"] = (Spiral(),)
        with Simulation(**config) as sim:
            with self.assertRaisesRegex(SimulationError, "one_way applies to the raster pattern only"):
                list(sim.run_operations())
            self.assertEqual(sim.state.completed, 0)


class ActualDepthTest(unittest.TestCase):
    """A part taller than the flutes (issue #88): parallel passes are refused
    only where they really cut deeper than the cutting length."""

    def setUp(self):
        import cadquery as cq

        from camlib.cadquery import export_surface_mesh

        # A 40 x 40 x 4 plate with a 10 x 10 mesa 16 mm tall (top Z 20).
        part = cq.Workplane("XY").box(40, 40, 4, centered=False)
        part = part.union(cq.Workplane("XY").box(10, 10, 20, centered=False).translate((15, 15, 0)))
        self.mesh = export_surface_mesh(part.val())
        holder = (camlib.HolderSection(40, 80, 10, 10),)
        self.mill = camlib.ToolAssembly(
            "em8", camlib.EndMill("em8", diameter=8, flute_length=25, overall_length=60), 40, 80, holder
        )
        self.ball = camlib.ToolAssembly(
            "b6", camlib.BallEndMill("b6", diameter=6, flute_length=18, overall_length=60), 40, 80, holder
        )
        self.cut = dict(feed_mm_min=1200, plunge_mm_min=300, spindle_rpm=9000, coolant="flood")

    def run_ops(self, operations):
        config = dict(
            stock=camlib.StockSpec((-1, -1, 0), (41, 41, 20.5), 0.5),
            tools=(self.mill, self.ball),
            operations=operations,
            revision="1",
            initial_position_mm=(0, 0, 40),
        )
        moves = 0
        with Simulation(**config) as sim:
            for update in sim.run_operations():
                moves += hasattr(update, "motion")
        return moves

    def finish(self):
        return camlib.Parallel(
            "finish",
            self.ball,
            self.mesh,
            0,
            3,
            30,
            tool_number=2,
            length_register=2,
            **self.cut,
        )

    def test_passes_after_roughing_reach_only_what_the_flutes_cut(self):
        rough = camlib.Roughing(
            "rough",
            self.mill,
            self.mesh,
            top_z_mm=20.5,
            floor_z_mm=4,
            stepdown_mm=4,
            clearance_z_mm=30,
            clearing=camlib.OffsetClearing(3),
            radial_allowance_mm=0.3,
            bottom_allowance_mm=0.2,
            tool_number=1,
            length_register=1,
            **self.cut,
        )
        # The stock top stays 20.5 over the mesa (20.5 mm above the floor,
        # past the 18 mm flutes), but no pass cuts more than the flutes.
        self.assertGreater(self.run_ops((rough, self.finish())), 100)

    def test_a_pass_really_deeper_than_the_flutes_is_refused(self):
        with self.assertRaisesRegex(ReachRefused, "cuts deeper than the cutting length 18") as caught:
            self.run_ops((self.finish(),))
        # Typed (#161): the shank, 18 mm above the tip, meets the unroughed
        # stock; the obstruction is the stock top, the tip the pass's.
        r = caught.exception.refusal
        self.assertEqual((r.operation, r.tool, r.limit, r.holder_section), ("finish", "b6", "shank", None))
        self.assertEqual((r.top_z_mm, r.depth_available_mm), (20.5, 18))
        self.assertAlmostEqual(r.depth_needed_mm, r.top_z_mm - r.z_mm)
        self.assertLess(r.z_mm, 20.5 - 18)
