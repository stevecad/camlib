"""Opt-in intrinsic distance on a retained, verified faceted dome."""

from dataclasses import replace
import math
import unittest

import camlib
import numpy as np
from camlib.material import SurfaceTarget
from camlib.post import ToolSetting
from camlib.verification import MaterialCheck, StockContactOptions, StreamChecks
from test_machine import model
from test_post import fixture


def dome():
    # A 4 mm square of the radius-5 dome, with 0.5 mm facet spacing.
    points = [
        (x, y, math.sqrt(25 - x * x - y * y) - 5)
        for y in (j / 2 for j in range(-4, 5))
        for x in (i / 2 for i in range(-4, 5))
    ]
    indices = [
        (k, k + 1, k + 10) if lower else (k, k + 10, k + 9)
        for k in (j * 9 + i for j in range(8) for i in range(8))
        for lower in (True, False)
    ]
    surface = camlib.SurfaceMesh(tuple(tuple(points[k] for k in tri) for tri in indices))
    count = len(points)
    vertices = [(x, y, z + 0.05) for x, y, z in points] + [(x, y, -3.0) for x, y, _ in points]
    boundary = list(range(9)) + list(range(17, 81, 9)) + list(range(79, 71, -1)) + list(range(63, 0, -9))
    caps = indices + [(c + count, b + count, a + count) for a, b, c in indices]
    sides = [
        (b, a, a + count) if top else (b, a + count, b + count)
        for a, b in zip(boundary, boundary[1:] + boundary[:1])
        for top in (True, False)
    ]
    stock = camlib.StockSpec.from_mesh(camlib.StockMesh(vertices, caps + sides), 0.05)
    return surface, stock


def dome_ridge(posted, motions):
    """Independent ball envelope over the interior of the faceted target."""
    uv = np.linspace(-1.5, 1.5, 121)
    j, i = np.divmod((uv + 2) / 0.5, 1.0)
    j = j.astype(int)
    x, y = i[None, :], i[:, None]
    u = np.linspace(-2.0, 2.0, 9)
    z = np.sqrt(25 - u[None, :] ** 2 - u[:, None] ** 2) - 5
    a, b = z[j[:, None], j], z[j[:, None], j + 1]
    c, d = z[j[:, None] + 1, j + 1], z[j[:, None] + 1, j]
    target = np.where(x >= y, a * (1 - x) + b * (x - y) + c * y, a * (1 - y) + d * (y - x) + c * x)
    xx, yy = np.meshgrid(uv, uv)
    ridge = target + 0.05
    for index in range(motions):
        motion = posted.posted_motion(index)["motion"]
        if motion["kind"] == "rapid":
            continue
        curve = motion["curve"]
        assert curve["kind"] == "line"
        ax, ay, az = curve["start"]
        bx, by, bz = curve["end"]
        dx, dy = bx - ax, by - ay
        length = math.hypot(dx, dy)
        px, py = xx - ax, yy - ay
        if length:
            along = (px * dx + py * dy) / length
            across2 = ((py * dx - px * dy) / length) ** 2
            slope = (bz - az) / length
            radius = np.sqrt(np.maximum(0.0, 0.25 - across2))
            at = np.clip(along - slope * radius / math.sqrt(1 + slope * slope), 0.0, length)
            reach = 0.25 - across2 - (along - at) ** 2
            tip = az + slope * at
        else:
            reach = 0.25 - px * px - py * py
            tip = min(az, bz)
        height = tip + 0.5 - np.sqrt(np.maximum(0.0, reach))
        ridge = np.minimum(ridge, np.where(reach >= 0, height, np.inf))
    residual = ridge - target
    return float(residual.max()), float(np.quantile(residual, 0.9) - np.quantile(residual, 0.1))


class IntrinsicScallopTest(unittest.TestCase):
    def test_posted_intrinsic_offsets_on_a_curved_region(self):
        surface, stock = dome()
        tool = camlib.ToolAssembly(
            "ball1",
            camlib.BallEndMill("d1", diameter=1, flute_length=10, overall_length=20),
            12,
            24,
            (camlib.HolderSection(12, 24, 2.0, 2.0),),
        )
        operation = camlib.Scallop(
            "dome",
            tool,
            surface,
            -3.0,
            0.341,
            4.0,
            entry=camlib.RampEntry(angle_deg=8, min_length_mm=0.3),
            safety_margin_mm=0.002,
            feed_mm_min=300.0,
            plunge_mm_min=90.0,
            spindle_rpm=1000,
            tool_number=1,
            length_register=1,
            coolant="flood",
        )
        target = SurfaceTarget("dome", "1", surface, -3.0)
        verification = StreamChecks(stock_contact=StockContactOptions(), material=(MaterialCheck(target),))
        start = (-3.0, -3.0, 4.0)
        measurements = []
        for intrinsic in (False, True):
            op = replace(operation, intrinsic_spacing=intrinsic)
            self.assertEqual(op.to_dict()["pattern"], {"kind": "scallop_intrinsic" if intrinsic else "scallop"})
            job = camlib.Job(
                "dome", "1", (camlib.Setup("top", stock, (tool,), (op,), start, verification=verification),)
            )
            header = job.compile().setup("top").program_header()
            machine = replace(
                fixture()[1],
                tools=(ToolSetting.for_tool(header.tools[0], number=1, length_register=1, position_after_mm=start),),
            )
            crash = replace(model(header, machine), initial_axes_mm=(*start[:2], start[2] + tool.gauge_length_mm))
            with job.post("top", machine=replace(machine, crash_model=crash)) as posted:
                out = posted.export()
                self.assertEqual(out["findings"], ())
                self.assertEqual(out["accepted"], ())
                self.assertEqual(out["machine"]["status"], "clear")
                material, first = None, 0
                while page := list(posted.verification_records(first, limit=256)):
                    for update in page:
                        if update.kind == "material":
                            material = update.check["report"]
                    first += len(page)
                self.assertTrue(material["complete"])
                leftover = material["residual"]["volume_mm3"]
                measurements.append((out["text"], out["motions"], leftover, dome_ridge(posted, out["motions"])))
        legacy, intrinsic = measurements
        self.assertNotEqual(legacy[0], intrinsic[0])
        self.assertLess(intrinsic[1], legacy[1])
        self.assertLess(intrinsic[-1][0], 0.049)
        self.assertLessEqual(intrinsic[-1][1], legacy[-1][1] + 0.001)
        print(
            "Curved offsets (posted moves, residual stock mm³ bounds, sampled max and p90-p10 mm):",
            legacy[1:],
            intrinsic[1:],
        )
