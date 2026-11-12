"""The exact synthetic #122 manifest, through Python's streamed API."""

import json
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest

from camlib.simulation import OperationUpdate, ReachRefused, Simulation, VerificationUpdate


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
            # A typed refusal naming the section and the stock it would meet (#160).
            with self.assertRaisesRegex(
                ReachRefused, 'roughing layer .*holder section 0 .*would meet remaining stock up to Z'
            ) as refused:
                list(sim.run_operations())
            refusal = refused.exception.refusal
            self.assertEqual(
                (refusal.operation, refusal.tool, refusal.limit, refusal.holder_section), ('rough', 'em12', 'holder', 0)
            )
            self.assertEqual(refusal.depth_available_mm, 76)
            self.assertGreater(refusal.top_z_mm, refusal.z_mm)


def drafted(draft_mm):
    """The same job with the boss drafted (wider by ``draft_mm`` on every side
    at its foot than at its top) and contour-parallel passes: each layer's
    wall passes lie a little inside those of the layers above and repeat none."""
    c = config()
    target = c['operations'][0]['target']
    for vertex in target['vertices']:
        if vertex[2] != 0 and vertex[0] in (30, 50) and vertex[1] in (30, 50):
            vertex[0] += draft_mm if vertex[0] > 40 else -draft_mm
            vertex[1] += draft_mm if vertex[1] > 40 else -draft_mm
    c['operations'][0]['clearing'] = {'kind': 'offset', 'stepover_mm': 6, 'coverage_margin_mm': 0.01}
    c['operations'][0]['omit_empty_cuts'] = False
    return c


class StraightShankTest(unittest.TestCase):
    """#200: a shank as wide as the cutter, deeper than the flutes, where the
    layers above have cut but no move repeats one of theirs."""

    def test_shank_as_wide_as_the_cutter_runs_beside_a_drafted_wall(self):
        for draft in (0.01, 0.2):
            with self.subTest(draft=draft):
                c = drafted(draft)
                self.assertEqual(c['tools'][0]['cutter']['shank_diameter_mm'], c['tools'][0]['cutter']['diameter_mm'])
                report = None
                statuses = set()
                with Simulation(**c) as sim:
                    for update in sim.run_operations():
                        if isinstance(update, VerificationUpdate):
                            # Stock contact of the shank and holder, move by move.
                            statuses.add(update.status)
                        elif isinstance(update, OperationUpdate) and update.stage == 'end':
                            report = update.report
                    self.assertTrue(sim.state.finished)
                    self.assertGreater(sim.state.volume()['removed_lower_mm3'], 400_000)
                self.assertEqual(statuses, {'clear'})
                self.assertEqual(report['completed_layers'], 20)

    def test_shank_a_hair_wider_than_the_cut_above_is_refused(self):
        c = drafted(0.01)
        c['tools'][0]['cutter']['shank_diameter_mm'] = 12.1
        with Simulation(**c) as sim:
            with self.assertRaisesRegex(
                ReachRefused, 'roughing layer .*shank .*would meet remaining stock up to Z'
            ) as refused:
                list(sim.run_operations())
        refusal = refused.exception.refusal
        self.assertEqual((refusal.operation, refusal.tool, refusal.limit), ('rough', 'em12', 'shank'))
        self.assertEqual(refusal.depth_available_mm, 75)
        self.assertLess(refusal.z_mm, -75)
