"""The exact synthetic #122 manifest, through Python's streamed API."""

import json
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest

from camlib.simulation import OperationUpdate, Simulation, SimulationError, VerificationUpdate


def config():
    source = Path(__file__).resolve().parents[2] / 'fixtures/operations/roughing-layer-depth.json'
    manifest = json.loads(source.read_text())
    return {
        key: manifest[key]
        for key in (
            'stock',
            'tools',
            'initial_position_mm',
            'sweep',
            'tool_change',
            'rapid_retracts',
            'operations',
            'verification',
        )
    } | {'revision': '1'}


class DeepRoughingTest(unittest.TestCase):
    def test_warm_cache_recertifies_deep_layer_reach(self):
        states = []
        with TemporaryDirectory() as directory:
            for warm in (False, True):
                with Simulation(**config(), regeneration={'directory': directory}) as sim:
                    for update in sim.run_operations():
                        if isinstance(update, OperationUpdate) and update.stage == 'end':
                            self.assertEqual(update.report['regeneration']['cache_hit'], warm)
                        elif isinstance(update, VerificationUpdate):
                            self.assertEqual(update.status, 'clear')
                    states.append((sim.state.completed, sim.state.heights.tobytes()))
        self.assertEqual(states[0], states[1])

    def test_78_mm_depth_with_75_mm_flutes_runs_and_checks_noncutting_parts(self):
        report = None
        with Simulation(**config()) as sim:
            for update in sim.run_operations():
                if isinstance(update, VerificationUpdate):
                    self.assertEqual(update.status, 'clear')
                elif isinstance(update, OperationUpdate) and update.stage == 'end':
                    report = update.report
            self.assertTrue(sim.state.finished)
            self.assertIsNotNone(report)
            self.assertEqual(report['completed_layers'], 20)
            self.assertGreater(sim.state.volume()['removed_lower_mm3'], 400_000)

    def test_short_holder_names_the_layer_and_reach(self):
        c = config()
        c['tools'][0]['holders'][0]['z_min_mm'] = 76
        c['tools'][0]['stickout_mm'] = 76
        with Simulation(**c) as sim:
            with self.assertRaisesRegex(SimulationError, 'roughing layer .*Holder clearance.*tip Z must exceed'):
                list(sim.run_operations())
