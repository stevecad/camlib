"""Regression for contour-parallel flat finishing around a faceted round boss."""

from dataclasses import replace
import math
import unittest

import cadquery as cq

import camlib
from camlib.cadquery import export_stock_mesh, export_surface_mesh
from camlib.simulation import OperationUpdate, Simulation, SimulationError, Update, VerificationUpdate
from camlib.tools import HolderSection, ToolAssembly


def round_boss_job(*, faceted=False):
    base = cq.Workplane("XY").box(40, 30, 4, centered=False)
    plane = cq.Workplane("XY").center(21.372, 16.018).workplane(offset=4)
    if faceted:
        angles = [0, 0.00003 / 7, *(math.tau * i / 72 for i in range(1, 72))]
        boss = plane.polyline([(7 * math.cos(a), 7 * math.sin(a)) for a in angles]).close().extrude(3)
    else:
        boss = plane.circle(7).extrude(3)
    target = base.union(boss)
    # The supported polyhedral stock model circumscribes the boss and leaves
    # 0.1 mm on both flats. The circular CAD wall is faceted by the exporter.
    radius = 7 / math.cos(math.pi / 32)
    angles = [math.tau * i / 32 for i in range(32)]
    polygon = [(radius * math.cos(a), radius * math.sin(a)) for a in angles]
    blank = cq.Workplane("XY").box(40, 30, 4.1, centered=False).union(plane.polyline(polygon).close().extrude(3.1))
    tool = ToolAssembly(
        "mill6",
        camlib.EndMill("em6", diameter=6, flute_length=20, overall_length=50),
        30,
        60,
        (HolderSection(30, 60, 4, 4),),
    )
    op = camlib.Flat(
        "flats",
        tool,
        export_surface_mesh(target, max_error_mm=0.04),
        0,
        2,
        14,
        feed_mm_min=600,
        plunge_mm_min=150,
        spindle_rpm=12000,
        tool_number=1,
        length_register=1,
        coolant="flood",
        entry=camlib.RampEntry(angle_deg=3, min_length_mm=1),
    )
    return dict(
        stock=camlib.StockSpec.from_mesh(export_stock_mesh(blank), 0.4),
        tools=(tool,),
        operations=(op,),
        revision="1",
        initial_position_mm=(0, 0, 14),
        verification=camlib.StreamChecks(stock_contact=camlib.StockContactOptions()),
    )


class FlatRoundBossTest(unittest.TestCase):
    def check_finish(self, job, step, radial):
        op = replace(job["operations"][0], stepover_mm=step, radial_stock_to_leave_mm=radial)
        moves = 0
        report = None
        with Simulation(**dict(job, operations=(op,))) as sim:
            for update in sim.run_operations():
                if isinstance(update, Update):
                    moves += 1
                    self.assertLess(moves, 5000, "round-boss stream did not terminate")
                elif isinstance(update, OperationUpdate) and update.stage == "end":
                    report = update.report
                elif isinstance(update, VerificationUpdate):
                    self.assertNotEqual(update.status, "violation")
            self.assertTrue(sim.state.finished)
            self.assertGreater(sim.state.volume()["removed_lower_mm3"], 0)
        self.assertEqual(report["levels_z_mm"], [7, 4])
        self.assertEqual(report["runs_completed"], report["runs"])
        self.assertEqual(report["dropped_length_mm"], 0)

    def test_round_boss_finishes_both_levels_with_and_without_radial_stock(self):
        job = round_boss_job()
        for step in (2, 2.5, 3):
            for radial in (0, 0.2):
                with self.subTest(step=step, radial=radial):
                    self.check_finish(job, step, radial)

    def test_faceted_boss_with_a_30_nm_edge_finishes(self):
        self.check_finish(round_boss_job(faceted=True), 2.5, 0)

    def test_round_boss_under_stock_refuses_when_no_entry_is_declared(self):
        job = round_boss_job()
        op = replace(job["operations"][0], stepover_mm=3, entry=None)
        stock = camlib.StockSpec((-10, -10, 0), (50, 40, 8), 0.4)
        with Simulation(**dict(job, operations=(op,), stock=stock)) as sim:
            with self.assertRaisesRegex(SimulationError, "declares no entry"):
                for moves, _ in enumerate(sim.run_operations()):
                    self.assertLess(moves, 5000, "unplannable round-boss stream did not terminate")


def filleted_boss():
    """An 80 x 80 x 10 mm base with a 50 mm round boss 12 mm tall, its top
    edge rounded 8 mm (issue #393)."""
    base = cq.Workplane("XY").box(80, 80, 10, centered=False)
    boss = cq.Workplane("XY").workplane(offset=10).center(40, 40).circle(25).extrude(12)
    return base.union(boss.faces(">Z").edges().fillet(8))


class FlatFilletedBossTest(unittest.TestCase):
    """The fillet's tessellation leaves the boss top's tool-centre region a
    finely faceted circle with a few-micron arc at every facet corner. Its
    erosions came back from the offset kernel with touching boundaries
    (#393), and the rings along it made one swept capsule per facet."""

    def test_boss_top_and_base_finish_around_the_fillet(self):
        target = filleted_boss()
        # Roughed: 0.2 mm on both flats, the boss a 64-gon circumscribing a
        # circle 0.3 mm proud of its wall (the stock model is polyhedral).
        radius = 25.3 / math.cos(math.pi / 64)
        polygon = [(radius * math.cos(math.tau * i / 64), radius * math.sin(math.tau * i / 64)) for i in range(64)]
        blank = (
            cq.Workplane("XY")
            .box(80, 80, 10.2, centered=False)
            .union(cq.Workplane("XY").workplane(offset=10).center(40, 40).polyline(polygon).close().extrude(12.2))
        )
        tool = ToolAssembly(
            "mill6",
            camlib.EndMill("em6", diameter=6, flute_length=18, overall_length=60),
            30,
            60,
            (HolderSection(30, 60, 4, 4),),
        )
        op = camlib.Flat(
            "flats",
            tool,
            export_surface_mesh(target, max_error_mm=0.005),
            9,
            2.5,
            35,
            feed_mm_min=1200,
            plunge_mm_min=300,
            spindle_rpm=12000,
            tool_number=1,
            length_register=1,
            coolant="flood",
            entry=camlib.RampEntry(angle_deg=3, min_length_mm=1),
        )
        report = None
        with Simulation(
            stock=camlib.StockSpec.from_mesh(export_stock_mesh(blank), 0.5),
            tools=(tool,),
            operations=(op,),
            revision="1",
            initial_position_mm=(-10, -10, 35),
            verification=camlib.StreamChecks(stock_contact=camlib.StockContactOptions()),
        ) as sim:
            for update in sim.run_operations():
                if isinstance(update, OperationUpdate) and update.stage == "end":
                    report = update.report
                elif isinstance(update, VerificationUpdate):
                    self.assertNotEqual(update.status, "violation")
            s = sim.state
            self.assertTrue(s.finished)
            heights = list(s.heights)
            nx, ny, dx, dy, lo = s.nx, s.ny, s.dx, s.dy, s.min_mm
        self.assertEqual(report["levels_z_mm"], [22, 10])
        self.assertEqual(report["runs_completed"], report["runs"])
        self.assertEqual(report["dropped_length_mm"], 0)
        # Independently of the planner: every stock cell lying wholly over
        # the top flat (radius 17 about the axis) or the base (outside the
        # boss wall, inside the part) is cut down to that flat.
        checked = {22: 0, 10: 0}
        for j in range(ny):
            for i in range(nx):
                x0, y0 = lo[0] + i * dx, lo[1] + j * dy
                corners = [(x0 + a * dx - 40, y0 + b * dy - 40) for a in (0, 1) for b in (0, 1)]
                far = max(math.hypot(*c) for c in corners)
                near = math.hypot(max(abs(x0 + dx / 2 - 40) - dx / 2, 0), max(abs(y0 + dy / 2 - 40) - dy / 2, 0))
                inside = x0 >= 0 and y0 >= 0 and x0 + dx <= 80 and y0 + dy <= 80
                level = 22 if far < 16.5 else 10 if inside and near > 25.5 else None
                if level is not None:
                    top = heights[2 * (j * nx + i) + 1]
                    self.assertLessEqual(top, level + 0.005, (i, j, level))
                    checked[level] += 1
        self.assertGreater(checked[22], 600)
        self.assertGreater(checked[10], 10000)
