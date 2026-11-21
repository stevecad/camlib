"""Post and independently check a full-size adaptive layer on solid stock (#193)."""

from dataclasses import replace
import math
import unittest

from camlib.jobs import Job, Setup
from camlib.operations import AdaptiveClearing, AdaptivePocket, Roughing, StockEdgeEntry
from camlib.planar import Boundary, Component, PlanarRegion
from camlib.playback import Cursor, Player
from camlib.post import ToolSetting
from camlib.simulation import StockSpec
from camlib.surface import SurfaceMesh
from camlib.tools import EndMill, HolderSection, ToolAssembly
from camlib.verification import StreamChecks, StockContactOptions
from test_mesh_stock import box_triangles
from test_post import fixture


class LargeAdaptiveTest(unittest.TestCase):
    def check_rim(self, side, roughing):
        # Solid blank, one connected perimeter cut, 1 mm deep. Stock must
        # come off every side and the square boss must keep its original top.
        lo, hi = 2, side - 2
        tool = ToolAssembly(
            't1', EndMill('em2', diameter=2, flute_length=20, overall_length=50), 30, 60, (HolderSection(30, 60, 4, 4),)
        )
        cutting = dict(
            top_z_mm=10,
            floor_z_mm=9,
            stepdown_mm=1,
            clearance_z_mm=12,
            feed_mm_min=600,
            plunge_mm_min=150,
            spindle_rpm=12000,
            tool_number=1,
            length_register=1,
            coolant='flood',
            entry=StockEdgeEntry(helix=None),
        )
        if roughing:
            target = SurfaceMesh(box_triangles((lo, lo, 0), (hi, hi, 10)))
            op = Roughing('rough', tool, target, clearing=AdaptiveClearing(0.25), **cutting)
        else:
            outer = Boundary.polygon(((-4, -4), (side + 4, -4), (side + 4, side + 4), (-4, side + 4)))
            hole = Boundary.polygon(((lo, lo), (lo, hi), (hi, hi), (hi, lo)))
            op = AdaptivePocket(
                'pocket',
                tool,
                PlanarRegion('rim', (Component(outer, (hole,)),)),
                stepover_mm=0.25,
                max_engagement_deg=90,
                **cutting,
            )
        setup = Setup(
            'top',
            StockSpec((0, 0, 0), (side, side, 10), 1),
            (tool,),
            (op,),
            (0, 0, 12),
            verification=StreamChecks(stock_contact=StockContactOptions()),
        )
        job = Job('rim', '1', (setup,))
        _, machine = fixture()
        machine = replace(
            machine,
            tools=(
                ToolSetting.for_tool(tool, number=1, length_register=1, position_after_mm=setup.initial_position_mm),
            ),
        )
        reports = []

        def collect(update):
            if update.stage == 'end':
                reports.append(update.report)

        with job.post('top', machine=machine, on_operation=collect) as run:
            run.wait(timeout=900)
            self.assertTrue(run.posting_status()['finished'])
            player = Player(run)
            player.seek(Cursor(run.status()['available']))
            self.assertTrue(player.state.finished)
            self.assertEqual(player.verification['progress']['violation'], 0)
            self.assertEqual(player.verification['progress']['uncertain'], 0)
            heights = player.state.heights
            mid = side // 2
            for x, y in ((0, mid), (side - 1, mid), (mid, 0), (mid, side - 1)):
                cell = 2 * (y * side + x)
                self.assertGreaterEqual(heights[cell], 8.999)
                self.assertLess(heights[cell + 1], 9.001)
            self.assertGreaterEqual(min(heights[::2]), 8.999)
            # Whole cells strictly inside the boss keep their original lower bound.
            for y in range(lo + 1, hi - 1):
                row = 2 * y * side
                self.assertEqual(min(heights[row + 2 * (lo + 1) : row + 2 * (hi - 1) : 2]), 10)
            self.assertGreater(player.state.volume()['removed_lower_mm3'], side)
        clearing = reports[-1]['layers'][0]['clearing']
        self.assertAlmostEqual(clearing['engagement_grid_mm'], 0.05)
        self.assertEqual(any('tiled engagement front' in d for d in clearing['diagnostics']), side == 400)
        self.assertGreater(clearing['certified_strokes'], 0)
        self.assertEqual(clearing['engagement_proved_strokes'], clearing['certified_strokes'])
        self.assertLessEqual(clearing['proven_max_engagement_deg'], math.degrees(math.acos(0.75)) + 1e-8)
        self.assertEqual(clearing.get('confined_strokes', 0), 0)

    def test_pocket_around_a_large_boss(self):
        for side in (30, 400):
            with self.subTest(side=side):
                self.check_rim(side, False)

    def test_adaptive_roughing_around_a_large_boss(self):
        for side in (30, 400):
            with self.subTest(side=side):
                self.check_rim(side, True)
