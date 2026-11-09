"""A close-running single-profile thread mill through the shared stock check."""

import unittest

import camlib
from camlib.simulation import Simulation, SimulationError
from camlib.tools import HolderSection, ThreadMill, ToolAssembly


class NeckClearanceTest(unittest.TestCase):
    def test_m5_coarse_grid_explains_refusal_and_fine_grid_proves_clearance(self):
        drill = ToolAssembly(
            'drill',
            camlib.Drill('drill4', diameter=4.2, flute_length=15, overall_length=30),
            20,
            30,
            (HolderSection(20, 30, 4, 4),),
        )
        thread = ToolAssembly(
            'thread',
            ThreadMill('tm3', diameter=3, tooth_height=0.5, neck_diameter=2, neck_length=10, overall_length=30),
            20,
            30,
            (HolderSection(20, 30, 4, 4),),
        )
        cutting = dict(
            feed_mm_min=200, plunge_mm_min=100, spindle_rpm=6000, tool_number=1, length_register=1, coolant='flood'
        )
        drilling = camlib.Drilling(
            'drill',
            drill,
            ((5, 5),),
            -6,
            2,
            retract_feed_mm_min=200,
            **{key: value for key, value in cutting.items() if key != 'plunge_mm_min'},
        )
        milling = camlib.ThreadMilling(
            'thread',
            thread,
            ((5, 5),),
            internal=True,
            diameter_mm=5,
            start_diameter_mm=4.2,
            pitch_mm=0.8,
            top_z_mm=-0.5,
            bottom_z_mm=-4.5,
            clearance_z_mm=2,
            **cutting,
        )
        for cell in (0.26, 0.05):
            with (
                self.subTest(cell=cell),
                Simulation(
                    camlib.StockSpec((0, 0, -8), (10, 10, 0), cell),
                    (drill, thread),
                    operations=(drilling, milling),
                    revision='1',
                    initial_position_mm=(0, 0, 2),
                ) as sim,
            ):
                if cell == 0.26:
                    with self.assertRaisesRegex(SimulationError, 'tool neck.*clearance lower bound.*cell_size_mm'):
                        list(sim.run_operations())
                else:
                    list(sim.run_operations())
                    self.assertTrue(sim.state.finished)
