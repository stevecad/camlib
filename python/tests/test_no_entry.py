"""Material a pocket operation cannot get into is refused by name (#250).

An AdaptivePocket in a slot too narrow for its helix used to plan no motion
at all, without an error. Checked from the run alone: the refusal names the
operation, the place, the level, the entry tried and the options; nothing is
cut before it; and the same slot is cut to its floor (an independent height
map of the emitted moves, not the engine's stock) with a ramp entry or after
an earlier operation has opened a start. A region the cutter fits nowhere in
is refused the same way, for Pocket and AdaptivePocket alike.
"""

from dataclasses import replace
import unittest

from camlib.operations import HelicalEntry, RampEntry
from camlib.simulation import Simulation, SimulationError

from test_ramp_entry import RADIUS, SLOT, Heights, adaptive_job, run, slot_job

# The slot of test_ramp_entry: 26 x 5 mm for a 4 mm cutter, Z10 down to Z6.
STOCK = 32 * 24 * 10
FLOOR = 6


def refused(test, job):
    with Simulation(**job) as sim:
        with test.assertRaises(SimulationError) as raised:
            list(sim.run_operations())
        test.assertEqual(sim.state.completed, 0)
        # Nothing was cut before the refusal.
        test.assertAlmostEqual(sim.state.volume()['remaining_lower_mm3'], STOCK, places=6)
    return str(raised.exception)


def slot_floor(heights):
    """The independent map over the slot between its end radii, off the walls."""
    inside = (
        (heights.x[None, :] >= SLOT[0][0] + RADIUS)
        & (heights.x[None, :] <= SLOT[1][0] - RADIUS)
        & (heights.y[:, None] >= SLOT[0][1] + 0.2)
        & (heights.y[:, None] <= SLOT[1][1] - 0.2)
    )
    return heights.top[inside]


class NoEntryTest(unittest.TestCase):
    def test_adaptive_pocket_refuses_a_slot_no_helix_fits(self):
        for entry in (None, HelicalEntry()):
            error = refused(self, adaptive_job(entry))
            for part in (
                'no entry into the enclosed material of slot at (',
                'for the level at Z8',
                'the cutter of radius 2 mm fits there',
                'no helix fits',
                'entry=RampEntry(angle_deg)',
                'a smaller cutter',
                'an earlier operation that opens a start there',
            ):
                self.assertIn(part, error)
        # The orbit strategies sweep a disk of radius 3 mm: none fits.
        for strategy in ('disk_front', 'stock_front'):
            error = refused(self, adaptive_job(None, path_strategy=strategy))
            self.assertIn('adaptive pocket slot: no orbit fits the area at (', error)
            self.assertIn('though the cutter fits it', error)
            self.assertIn('path_strategy engagement_front', error)

    def test_a_region_the_cutter_fits_nowhere_in_is_refused(self):
        # 26 x 3 mm for the 4 mm cutter: no operation can do anything there.
        narrow = ((3.0, 10.5), (29.0, 13.5))
        jobs = [(slot_job(None, region=narrow), 'pocket slot')]
        for strategy in ('engagement_front', 'disk_front', 'stock_front'):
            jobs.append((adaptive_job(None, region=narrow, path_strategy=strategy), 'adaptive pocket slot'))
        for job, name in jobs:
            error = refused(self, job)
            self.assertIn(f'{name}: the cutter fits nowhere in the region', error)
            self.assertIn('Use a smaller cutter', error)

    def test_the_same_slot_plans_with_a_ramp_or_from_an_opened_start(self):
        # With a ramp entry: cut to the floor.
        _, motions, report, _ = run(adaptive_job(RampEntry(4, min_length_mm=2)))
        heights = Heights((32, 24, 10))
        for motion in motions:
            heights.cut(motion['curve'])
        self.assertGreaterEqual(report['ramp_entries'], 2)
        self.assertTrue((slot_floor(heights) == FLOOR).all())
        # An earlier pocket (it plunges) opens the first 8 mm of the slot;
        # the adaptive pocket, with no entry given, then starts there and
        # clears the rest: no helix, and no descent into standing stock.
        opener = replace(slot_job(None, region=((3.0, 9.5), (11.0, 14.5)))['operations'][0], id='opener')
        job = adaptive_job(None)
        job['operations'] = (opener, job['operations'][0])
        _, motions, report, _ = run(job)
        heights = Heights((32, 24, 10))
        cuts = 0
        for motion in motions:
            curve = motion['curve']
            if motion['operation'] == 'slot':
                a, b = curve['start'], curve['end']
                self.assertNotEqual(curve['kind'], 'helix', motion)
                if b[2] < a[2]:
                    self.assertEqual(a[:2], b[:2], motion)
                    self.assertLessEqual(heights.under(b[0], b[1]), b[2] + 0.02, motion)
                cuts += motion['kind'] == 'cut'
            heights.cut(curve)
        self.assertGreater(cuts, 0)
        self.assertEqual(report['helical_entries'], 0)
        self.assertTrue((slot_floor(heights) == FLOOR).all())

    def test_a_slot_already_cut_is_not_an_error(self):
        # Nothing left to cut is an empty plan, not a refusal: the same
        # adaptive pocket again, after a ramped one has cleared the slot.
        first = adaptive_job(RampEntry(4, min_length_mm=2))
        again = replace(adaptive_job(None)['operations'][0], id='again')
        job = dict(first, operations=(first['operations'][0], again))
        _, motions, report, _ = run(job)
        self.assertTrue(any(m['operation'] == 'slot' for m in motions))
        for motion in motions:
            if motion['operation'] == 'again':
                self.assertNotEqual(motion['curve']['kind'], 'helix', motion)
        self.assertEqual(report['helical_entries'], 0)


if __name__ == '__main__':
    unittest.main()
