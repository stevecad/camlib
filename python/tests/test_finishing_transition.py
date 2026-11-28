"""A curved floor-to-wall handoff measured with an independently decoded ball sweep."""

from dataclasses import replace
import math
import unittest

import cadquery as cq
import numpy as np

import camlib
from camlib.cadquery import export_stock_mesh, export_surface_mesh
from camlib.material import SurfaceTarget
from camlib.post import LoweringOptions, ToolSetting
from camlib.verification import MaterialCheck, StockContactOptions, StreamChecks
from test_machine import model
from test_post import fixture


def _prism(skin):
    """R6 concave blend into a 65 degree draft with a flat before the blend."""

    def arc(a):
        a = math.radians(a)
        return 6 * math.sin(a), skin + 6 * (1 - math.cos(a))

    last_x, last_z = arc(65)
    top = last_z + 2 * math.tan(math.radians(65))
    sketch = cq.Workplane("XZ")
    if skin:
        # Chords of the concave profile lie above the nominal surface.
        sketch = sketch.polyline(
            [(-9, -2), (-9, skin), *(arc(a) for a in range(0, 66, 5)), (last_x + 2, top), (last_x + 2, -2)]
        )
    else:
        sketch = (
            sketch.moveTo(-9, -2)
            .lineTo(-9, 0)
            .lineTo(0, 0)
            .threePointArc(arc(32.5), arc(65))
            .lineTo(last_x + 2, top)
            .lineTo(last_x + 2, -2)
        )
    return sketch.close().extrude(-8), top


def _tool():
    return camlib.ToolAssembly(
        "ball2",
        camlib.BallEndMill("be2", diameter=2, flute_length=15, overall_length=50),
        30,
        60,
        (camlib.HolderSection(30, 60, 4, 4),),
    )


def _operation(tool, surface, top, **changes):
    options = dict(
        top_z_mm=top,
        bottom_z_mm=0.02,
        stepdown_mm=0.65,
        stepover_mm=0.50,
        threshold_deg=45,
        axis="y",
        entry=camlib.RampEntry(angle_deg=8, min_length_mm=0.05),
        contour_error_mm=0.001,
        safety_margin_mm=0.001,
        path_error_mm=0.0003,
        feed_mm_min=500,
        plunge_mm_min=150,
        spindle_rpm=12000,
        tool_number=1,
        length_register=1,
        coolant="flood",
    )
    options.update(changes)
    return camlib.steep_and_shallow("finish", tool, surface, -2, 11, **options)


class FinishingTransitionTest(unittest.TestCase):
    def test_transition_spacing_caps_both_passes_with_ball_geometry(self):
        tool = _tool()
        baseline_steep, baseline_shallow = _operation(tool, "surface", 7)
        self.assertNotIn("slope_aware_stepover", baseline_shallow.to_dict())
        steep, shallow = _operation(tool, "surface", 7, transition_scallop_mm=0.012)
        spacing = 2 * math.sqrt(0.012 * (2 - 0.012))
        self.assertAlmostEqual(steep.stepdown_mm, spacing)
        self.assertAlmostEqual(shallow.stepover_mm, spacing)
        self.assertTrue(shallow.slope_aware_stepover)
        self.assertEqual(steep.slope_from_deg, baseline_steep.slope_from_deg)
        self.assertEqual(shallow.slope_to_deg, baseline_shallow.slope_to_deg)
        for cusp in (-0.01, 0, 1, math.nan):
            with self.assertRaisesRegex(ValueError, "transition_scallop_mm"):
                _operation(tool, "surface", 7, transition_scallop_mm=cusp)
        for unsupported_tool, pattern in (("ball2", "parallel"), (tool, "spiral")):
            with self.assertRaisesRegex(ValueError, "ball-end tool assembly and parallel"):
                _operation(unsupported_tool, "surface", 7, transition_scallop_mm=0.012, shallow=pattern)

    def test_posted_transition_matches_ridge_with_a_narrower_overlap(self):
        solid, top = _prism(0.0)
        tool = _tool()
        surface = export_surface_mesh(solid, max_error_mm=0.002)
        stock = camlib.StockSpec.from_mesh(export_stock_mesh(_prism(0.12)[0]), 0.2)
        verification = StreamChecks(
            stock_contact=StockContactOptions(),
            material=(MaterialCheck(SurfaceTarget("blend", "1", surface, -2)),),
        )
        start = (0, 0, 11)
        angles = np.radians(np.linspace(38, 52, 281))
        queries = np.column_stack([6 * np.sin(angles), np.full(len(angles), 4.0), 6 * (1 - np.cos(angles))])
        measurements = []
        cusp = 0.025
        spacing = 2 * math.sqrt(cusp * (2 - cusp))
        # Start with the same physical pitch as the candidate and the
        # original angular overlap. The control sets the two steps explicitly.
        settings = (
            dict(stepdown_mm=spacing, stepover_mm=spacing, slope_aware_stepover=True),
            dict(transition_scallop_mm=cusp, overlap_deg=1.5),
        )
        for options in settings:
            operations = _operation(tool, surface, top, **options)
            job = camlib.Job(
                "blend", "1", (camlib.Setup("top", stock, (tool,), operations, start, verification=verification),)
            )
            header = job.compile().setup("top").program_header()
            machine = replace(
                fixture()[1],
                tools=(ToolSetting.for_tool(header.tools[0], number=1, length_register=1, position_after_mm=start),),
            )
            crash = replace(model(header, machine), initial_axes_mm=(0, 0, 11 + tool.gauge_length_mm))
            distances = np.full((2, len(queries)), np.inf)
            total_feed = 0.0
            with job.post(
                "top", machine=replace(machine, crash_model=crash), lowering=LoweringOptions(chord_tolerance_mm=0.0005)
            ) as posted:
                result = posted.export()
                self.assertEqual(result["accepted"], ())
                self.assertEqual(result["findings"], ())
                self.assertEqual(result["machine"]["status"], "clear")
                for i in range(result["motions"]):
                    move = posted.posted_motion(i)["motion"]
                    if move["kind"] != "cut":
                        continue
                    op = 0 if move["operation"] == "finish-steep" else 1
                    a, b = (np.array(move["curve"][key]) + [0, 0, 1] for key in ("start", "end"))
                    d = b - a
                    total_feed += float(np.linalg.norm(d))
                    t = np.clip((queries - a) @ d / max(float(d @ d), 1e-30), 0, 1)
                    distances[op] = np.minimum(
                        distances[op], np.linalg.norm(queries - a - t[:, None] * d, axis=1) - 1.0
                    )
            ridge = distances.min(axis=0)
            twice = int(np.count_nonzero((distances <= 0.03).all(axis=0)))
            self.assertGreaterEqual(ridge.min(), -0.001)
            measurements.append((float(ridge.max()), twice, total_feed))
        baseline, matched = measurements
        print("Posted transition ridge, twice-covered samples, feed length:", baseline, matched)
        self.assertLessEqual(baseline[0], 0.03)
        self.assertLessEqual(matched[0], 0.03)
        self.assertLess(matched[1], baseline[1])
