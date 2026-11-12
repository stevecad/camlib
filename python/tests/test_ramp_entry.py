"""Ramp entry (RampEntry) on Pocket, RestPocket, AdaptivePocket and Contour,
checked from the motions alone.

An independent height map (numpy, not the engine's stock) decides what is
material: straight descents must be through air, sloped ones no steeper than
the declared angle, and the floor under the ramps must end at the level depth.
The posted program is decoded back and its ramps measured again.
"""

from dataclasses import replace
import importlib.util
import math
from pathlib import Path
import unittest

import numpy as np

from camlib.motion import Program
from camlib.operations import AdaptivePocket, HelicalEntry, Pocket, RampEntry, Tabs
from camlib.planar import Boundary, Component, PlanarRegion
from camlib.post import Machine, ToolSetting, post_events
from camlib.postprocessor import GCodeDialect
from camlib.simulation import OperationUpdate, Simulation, SimulationError, StockSpec
from camlib.tools import EndMill, HolderSection, ToolAssembly

ROOT = Path(__file__).resolve().parents[2]
RADIUS = 2.0
CELL = 0.1
# A slot 26 x 5 mm for a 4 mm cutter: no room for a helix worth the name.
SLOT = ((3.0, 9.5), (29.0, 14.5))


def tool(**cutter):
    return ToolAssembly(
        't1',
        EndMill('em4', diameter=4, flute_length=20, overall_length=50, **cutter),
        30,
        60,
        (HolderSection(30, 60, 4, 4),),
    )


def slot_job(entry, region=SLOT, outline=None, t=None, **changes):
    (x0, y0), (x1, y1) = region
    outline = outline or Boundary.polygon(((x0, y0), (x1, y0), (x1, y1), (x0, y1)))
    area = PlanarRegion('slot', (Component(outline),))
    t = t or tool()
    op = Pocket(
        'slot',
        t,
        area,
        10,
        6,
        2,
        1.5,
        12,
        entry=entry,
        feed_mm_min=600,
        plunge_mm_min=150,
        spindle_rpm=12000,
        tool_number=1,
        length_register=1,
        coolant='flood',
        **changes,
    )
    return dict(
        stock=StockSpec((0, 0, 0), (32, 24, 10), 0.4),
        tools=(t,),
        operations=(op,),
        revision='ramp-1',
        initial_position_mm=(0, 0, 12),
    )


def adaptive_job(entry, region=SLOT, t=None, **changes):
    (x0, y0), (x1, y1) = region
    area = PlanarRegion('slot', (Component(Boundary.polygon(((x0, y0), (x1, y0), (x1, y1), (x0, y1)))),))
    t = t or tool()
    op = AdaptivePocket(
        'slot',
        t,
        area,
        10,
        6,
        2,
        1.5,
        12,
        entry=entry,
        feed_mm_min=600,
        plunge_mm_min=150,
        spindle_rpm=12000,
        tool_number=1,
        length_register=1,
        coolant='flood',
        **changes,
    )
    return dict(
        stock=StockSpec((0, 0, 0), (32, 24, 10), 0.4),
        tools=(t,),
        operations=(op,),
        revision='ramp-1',
        initial_position_mm=(0, 0, 12),
    )


def contour_job(entry, **changes):
    spec = importlib.util.spec_from_file_location('contour_example', ROOT / 'python/tests/support/contour.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    job = module.contour_job()
    job['operations'] = (replace(job['operations'][0], entry=entry, **changes),)
    return job


def run(job):
    events, motions, report = [], [], None
    with Simulation(**job) as sim:
        for update in sim.run_operations():
            if isinstance(update, OperationUpdate):
                events.extend(update.events)
                if update.stage == 'end':
                    report = update.report
            else:
                events.append(dict(kind='motion', motion=update.motion))
                motions.append(update.motion)
        assert sim.state.finished
        return events, motions, report, sim.state.volume()


def horizontal(curve):
    a, b = curve['start'], curve['end']
    if curve['kind'] == 'line':
        return math.hypot(a[0] - b[0], a[1] - b[1])
    c = curve['center']
    return math.hypot(a[0] - c[0], a[1] - c[1]) * abs(curve['sweep_rad'])


def samples(curve):
    """Points along a line, arc or helix, at most 0.04 mm apart."""
    a, b = curve['start'], curve['end']
    n = max(1, math.ceil(math.hypot(horizontal(curve), a[2] - b[2]) / 0.04))
    for i in range(n + 1):
        f = i / n
        z = a[2] + (b[2] - a[2]) * f
        if curve['kind'] == 'line':
            yield a[0] + (b[0] - a[0]) * f, a[1] + (b[1] - a[1]) * f, z
        else:
            c = curve['center']
            r = math.hypot(a[0] - c[0], a[1] - c[1])
            t = math.atan2(a[1] - c[1], a[0] - c[0]) + curve['sweep_rad'] * f
            yield c[0] + r * math.cos(t), c[1] + r * math.sin(t), z


class Heights:
    """The independent stock: column tops cut by a flat cutter of RADIUS."""

    def __init__(self, size):
        self.top = np.full((round(size[1] / CELL), round(size[0] / CELL)), float(size[2]))
        self.x = (np.arange(self.top.shape[1]) + 0.5) * CELL
        self.y = (np.arange(self.top.shape[0]) + 0.5) * CELL

    def disk(self, x, y, reach):
        return (self.x[None, :] - x) ** 2 + (self.y[:, None] - y) ** 2 <= reach * reach

    def under(self, x, y):
        # Well inside the cutter's edge: grid and sampling blur it by less.
        return self.top[self.disk(x, y, RADIUS - 3 * CELL)].max()

    def cut(self, curve):
        for x, y, z in samples(curve):
            i0, i1 = max(0, int((x - RADIUS) / CELL)), int((x + RADIUS) / CELL) + 1
            j0, j1 = max(0, int((y - RADIUS) / CELL)), int((y + RADIUS) / CELL) + 1
            window = self.top[j0:j1, i0:i1]
            mask = (self.x[None, i0:i1] - x) ** 2 + (self.y[j0:j1, None] - y) ** 2 <= RADIUS * RADIUS
            window[mask] = np.minimum(window[mask], z)


def check_descents(test, motions, size, angle_deg, feed, kind='plunge', heights=None, air=None):
    """Every descent: straight only through air, sloped no steeper than the angle.

    ``heights``: the map earlier moves left (default: the uncut stock).
    ``air``: no sloped descent may start more than this above the material
    under its own path.
    """
    heights = heights or Heights(size)
    limit = math.tan(math.radians(angle_deg))
    ramps = []
    for motion in motions:
        curve = motion['curve']
        a, b = curve['start'], curve['end']
        travel = horizontal(curve)
        if b[2] < a[2]:
            if travel == 0:
                test.assertLessEqual(heights.under(b[0], b[1]), b[2] + 0.02, motion)
            else:
                slope = (a[2] - b[2]) / travel
                test.assertLessEqual(slope, limit * (1 + 1e-9), motion)
                test.assertEqual(motion['kind'], kind)
                test.assertEqual(motion['feed_mm_min'], feed)
                if air is not None:
                    under = max(heights.under(x, y) for x, y, _ in samples(curve))
                    test.assertLessEqual(a[2] - under, air, motion)
                ramps.append(slope)
        elif motion['kind'] == 'rapid' and travel > 0:
            # Rapids travel above the stock (a straight rapid up out of the
            # cut is the stream's retract).
            test.assertGreaterEqual(min(a[2], b[2]), size[2])
        heights.cut(curve)
    return heights, ramps


def posted_motions(job, events):
    header = Program('posted-ramp', '1', job['tools'], job['initial_position_mm'], (), tolerance_mm=1e-5)
    setting = ToolSetting.for_tool(
        job['tools'][0], number=1, length_register=1, position_after_mm=job['initial_position_mm']
    )
    machine = Machine(
        'User machine',
        (setting,),
        dialect=GCodeDialect.linuxcnc(),
        output_units='mm',
        decimals=4,
        max_feed_mm_min=10000,
        max_spindle_rpm=30000,
        planes=('xy', 'xz', 'yz'),
        arcs=True,
        helices=True,
        max_sweep_rad=math.tau,
        max_tool_number=99999,
        max_length_register=99999,
        tool_changes=True,
        flood=True,
        mist=True,
        tool_change_position_mode='table',
        rapid_mm_min=6000,
    )
    chunks = list(post_events(header, events, machine=machine))
    return [dict(curve=m.curve.to_dict()) for chunk in chunks for m in chunk.motions], ''.join(
        chunk.text for chunk in chunks
    )


class RampEntryTest(unittest.TestCase):
    def test_pocket_ramp_zigzags_within_the_angle_and_the_floor_is_cut(self):
        for angle, feed, zigzag in ((4, None, True), (10, 90, False)):
            # A ramp that reverses needs the declared minimum span; one leg does not.
            job = slot_job(RampEntry(angle, min_length_mm=2 if zigzag else None, feed_mm_min=feed))
            events, motions, report, volume = run(job)
            heights, ramps = check_descents(self, motions, (32, 24, 10), angle, feed or 150)
            self.assertGreaterEqual(len(ramps), 2)
            self.assertGreaterEqual(report['ramp_entries'], 2)
            self.assertLessEqual(report['max_ramp_angle_deg'], angle + 1e-9)
            self.assertAlmostEqual(math.degrees(math.atan(max(ramps))), report['max_ramp_angle_deg'], places=6)
            if not zigzag:
                # One leg, as steep as declared: no longer than it has to be.
                self.assertAlmostEqual(max(ramps), math.tan(math.radians(angle)), places=6)
            # The cutter centre stays inside the slot (convex region, straight
            # moves: their ends decide), and never below the floor.
            turns = 0
            previous = 0
            for motion in motions:
                curve = motion['curve']
                if motion['kind'] == 'rapid':
                    continue
                self.assertEqual(curve['kind'], 'line')
                for x, y, z in (curve['start'], curve['end']):
                    self.assertTrue(SLOT[0][0] + RADIUS - 1e-9 <= x <= SLOT[1][0] - RADIUS + 1e-9, motion)
                    self.assertTrue(SLOT[0][1] + RADIUS - 1e-9 <= y <= SLOT[1][1] - RADIUS + 1e-9, motion)
                    self.assertGreaterEqual(z, 6)
                dx = curve['end'][0] - curve['start'][0]
                sloped = curve['end'][2] < curve['start'][2] and horizontal(curve) > 0
                turns += sloped and previous * dx < 0
                previous = dx if sloped else 0
            self.assertEqual(turns > 0, zigzag)
            # The floor under the ramps is at depth; nothing outside is cut.
            inside = (
                (heights.x[None, :] >= SLOT[0][0] + RADIUS)
                & (heights.x[None, :] <= SLOT[1][0] - RADIUS)
                & (heights.y[:, None] >= SLOT[0][1] + 0.2)
                & (heights.y[:, None] <= SLOT[1][1] - 0.2)
            )
            self.assertGreater(inside.sum(), 5000)
            self.assertTrue((heights.top[inside] == 6).all())
            outside = (
                (heights.x[None, :] < SLOT[0][0] - 0.2)
                | (heights.x[None, :] > SLOT[1][0] + 0.2)
                | (heights.y[:, None] < SLOT[0][1] - 0.2)
                | (heights.y[:, None] > SLOT[1][1] + 0.2)
            )
            self.assertTrue((heights.top[outside] == 10).all())
            ideal = 4 * (26 * 5 - (4 - math.pi) * RADIUS**2)
            self.assertLessEqual(volume['removed_lower_mm3'], ideal + 1e-6)
            self.assertGreaterEqual(volume['removed_upper_mm3'], ideal - 1e-6)
            # The posted program, decoded back: the same ramps, no steeper.
            decoded, text = posted_motions(job, events)
            self.assertTrue(text.endswith('M2\n'))
            posted = [
                (m['curve']['start'][2] - m['curve']['end'][2]) / horizontal(m['curve'])
                for m in decoded
                if m['curve']['end'][2] < m['curve']['start'][2] and horizontal(m['curve']) > 0
            ]
            self.assertEqual(len(posted), len(ramps))
            # Posted coordinates are rounded to 4 decimals.
            self.assertLessEqual(max(posted), math.tan(math.radians(angle)) + 1e-4)

    def test_pocket_without_a_ramp_is_unchanged_and_serialises_no_entry(self):
        plain = slot_job(None)
        self.assertNotIn('entry', plain['operations'][0].to_dict())
        self.assertEqual(
            slot_job(RampEntry(4))['operations'][0].to_dict()['entry'], dict(kind='ramp', angle_rad=math.radians(4))
        )
        self.assertEqual(
            RampEntry(4, min_length_mm=1, feed_mm_min=90).to_dict(),
            dict(kind='ramp', angle_rad=math.radians(4), min_length_mm=1, feed_mm_min=90),
        )
        _, motions, report, _ = run(plain)
        self.assertNotIn('ramp_entries', report)
        # The default entry is still the straight plunge.
        self.assertFalse(
            [m for m in motions if m['curve']['end'][2] < m['curve']['start'][2] and horizontal(m['curve']) > 0]
        )

    def test_pocket_refuses_a_pass_too_short_to_ramp_on_before_any_motion(self):
        small = ((10.0, 9.0), (15.5, 14.5))
        cases = (
            (
                slot_job(RampEntry(4), region=small),
                ('pocket slot', 'no ramp entry', 'has to reverse', 'min_length_mm is not set', 'HelicalEntry'),
            ),
            (
                slot_job(RampEntry(4, min_length_mm=2), region=small),
                ('pocket slot', 'shorter than min_length_mm 2.0000', 'a smaller angle does not help'),
            ),
            (slot_job(RampEntry(4, min_length_mm=0)), ('invalid ramp entry: min_length_mm',)),
            (slot_job(RampEntry(0)), ('invalid ramp entry',)),
        )
        for job, parts in cases:
            with Simulation(**job) as sim:
                with self.assertRaises(SimulationError) as raised:
                    list(sim.run_operations())
                self.assertEqual(sim.state.completed, 0)
            for part in parts:
                self.assertIn(part, str(raised.exception))
        # The author may lower the minimum leg, explicitly.
        _, motions, _, _ = run(slot_job(RampEntry(4, min_length_mm=1), region=small))
        check_descents(self, motions, (32, 24, 10), 4, 150)

    def test_pocket_ramp_follows_arcs_as_helical_moves(self):
        # The slot with semicircular ends (radius 2.5): the cutter centre
        # stays within 0.5 mm of the axis y = 12, x 5.5..26.5, and the ramp
        # into the wall pass runs over its half circles.
        ends = Boundary(((5.5, 9.5, 0), (26.5, 9.5, 1), (26.5, 14.5, 0), (5.5, 14.5, 1)))
        job = slot_job(RampEntry(4, min_length_mm=2), outline=ends)
        events, motions, report, _ = run(job)
        heights, ramps = check_descents(self, motions, (32, 24, 10), 4, 150)

        def axis(x, y):
            return math.hypot(x - min(max(x, 5.5), 26.5), y - 12)

        helical = 0
        for motion in motions:
            curve = motion['curve']
            if motion['kind'] == 'rapid':
                continue
            # The centre set is convex: a line is inside when its ends are,
            # an arc when it is one of the end circles.
            for x, y, z in (curve['start'], curve['end']):
                self.assertLessEqual(axis(x, y), 0.5 + 1e-7, motion)
                self.assertGreaterEqual(z, 6)
            if curve['kind'] != 'line':
                self.assertIn(curve['center'][0], (5.5, 26.5))
                self.assertEqual(curve['center'][1], 12)
                helical += curve['kind'] == 'helix'
        self.assertGreaterEqual(helical, 2)
        d = np.hypot(heights.x[None, :] - np.clip(heights.x[None, :], 5.5, 26.5), heights.y[:, None] - 12)
        self.assertTrue((heights.top[d <= 2.3] == 6).all())
        self.assertTrue((heights.top[d >= 2.7] == 10).all())
        decoded, _ = posted_motions(job, events)
        posted = [
            (m['curve']['start'][2] - m['curve']['end'][2]) / horizontal(m['curve'])
            for m in decoded
            if m['curve']['end'][2] < m['curve']['start'][2] and horizontal(m['curve']) > 0
        ]
        self.assertEqual(len(posted), len(ramps))
        self.assertTrue(any(m['curve']['kind'] == 'helix' for m in decoded))
        self.assertLessEqual(max(posted), math.tan(math.radians(4)) + 1e-3)

    def test_contour_ramp_with_wear_compensation_posts_one_checked_descent(self):
        # G41 goes on over the lead-in at the stock top and stays on down
        # the whole ramp: every move in between travels in XY.
        from test_cutter_compensation import job as wear_job, machine
        from test_tapping import posted_text

        j = wear_job(entry=RampEntry(3), floor_z_mm=4)
        nc = posted_text(j, machine(j))
        marks = [i for i, line in enumerate(nc) if line.split(' ')[0] in ('G41', 'G42', 'G40')][1:]
        self.assertEqual([nc[i] for i in marks], ['G41 D7', 'G40'])
        heights = []
        for line in nc[marks[0] + 1 : marks[1]]:
            words = line.split()
            z = [float(w[1:]) for w in words if w.startswith('Z')]
            if z:
                self.assertTrue(any(w[0] in 'XY' for w in words), line)
                heights.append(z[0])
        # Down from the stock top to the floor, never back up.
        self.assertEqual(heights, sorted(heights, reverse=True))
        self.assertEqual(heights[-1], 4)
        self.assertTrue(any(4 < z < 10 for z in heights))
        # Without the ramp the same contour turns compensation on per level.
        plain = wear_job(floor_z_mm=4)
        self.assertEqual(sum(line == 'G41 D7' for line in posted_text(plain, machine(plain))), 3)

    def test_contour_ramps_around_each_loop_then_cuts_it_flat(self):
        job = contour_job(RampEntry(3))
        events, motions, report, volume = run(job)
        heights, ramps = check_descents(self, motions, (40, 40, 10), 3, 150)
        laps = [0.0, 0.0]
        for motion in motions:
            curve = motion['curve']
            if motion['kind'] == 'rapid':
                continue
            points = [curve['start'], curve['end']]
            for x, y, z in points:
                r = math.hypot(x - 20, y - 20)
                # The cutter clears the kept annulus (radii 4..8) all the way.
                self.assertTrue(r - RADIUS > 8 or r + RADIUS < 4, motion)
                self.assertGreaterEqual(z, 2)
            if curve['kind'] != 'line':
                self.assertAlmostEqual(curve['center'][0], 20, places=8)
                self.assertAlmostEqual(curve['center'][1], 20, places=8)
                if curve['end'][2] < curve['start'][2]:
                    r = math.hypot(curve['start'][0] - 20, curve['start'][1] - 20)
                    laps[r < 8] += abs(curve['sweep_rad']) / math.tau
                    # No lap goes deeper than the stepdown.
                    per_lap = (curve['start'][2] - curve['end'][2]) / abs(curve['sweep_rad']) * math.tau
                    self.assertLessEqual(per_lap, 2 + 1e-9)
        # One descent per loop, 8 mm deep: the outer loop (62.9 mm) is bound
        # by the 2 mm stepdown (4 laps), the hole's (12.5 mm) by the angle
        # (8 mm at 3 degrees needs 152.6 mm: 13 laps).
        self.assertAlmostEqual(laps[0], 4)
        self.assertAlmostEqual(laps[1], 13)
        self.assertEqual(report['depth_passes'], 1)
        self.assertEqual(report['loops_completed'], 2)
        r = np.hypot(heights.x[None, :] - 20, heights.y[:, None] - 20)
        self.assertTrue((heights.top[(r < 3.7) | ((r > 8.3) & (r < 11.7))] == 2).all())
        self.assertTrue((heights.top[(r > 4.2) & (r < 7.8)] == 10).all())
        ideal = 768 * math.pi
        self.assertLessEqual(volume['removed_lower_mm3'], ideal)
        self.assertGreaterEqual(volume['removed_upper_mm3'], ideal)
        decoded, _ = posted_motions(job, events)
        posted = [
            (m['curve']['start'][2] - m['curve']['end'][2]) / horizontal(m['curve'])
            for m in decoded
            if m['curve']['end'][2] < m['curve']['start'][2] and horizontal(m['curve']) > 0
        ]
        self.assertEqual(len(posted), len(ramps))
        self.assertLessEqual(max(posted), math.tan(math.radians(3)) + 1e-4)

    def test_contour_ramp_refusals_and_default_serialisation(self):
        self.assertNotIn('entry', contour_job(None)['operations'][0].to_dict())
        cases = (
            (contour_job(RampEntry(3), tabs=Tabs(3, 1, count=2)), 'cannot be combined with holding tabs'),
            (contour_job(HelicalEntry()), 'must be a RampEntry'),
            (contour_job(RampEntry(0.005)), 'laps'),
        )
        for job, part in cases:
            with Simulation(**job) as sim:
                with self.assertRaises(SimulationError) as raised:
                    list(sim.run_operations())
                self.assertEqual(sim.state.completed, 0)
            self.assertIn(part, str(raised.exception))


class RampEntryIssue219Test(unittest.TestCase):
    """#219: ramp entry in rest pockets and adaptive clearing, and the
    decision that it does not admit a cutter that is not center cutting."""

    def test_rest_pocket_ramps_into_what_an_earlier_pocket_left(self):
        # A pocket takes the right part of the slot to Z8; the rest pocket
        # then clears the whole slot to Z6 with ramp entries: into the stock
        # left at the first level, and from that level's floor at the second.
        before = replace(slot_job(None, region=((16.0, 9.5), (29.0, 14.5)))['operations'][0], id='before', floor_z_mm=8)
        job = slot_job(RampEntry(4, min_length_mm=2), rest_tolerance_mm=0.01)
        job['operations'] = (before, job['operations'][0])
        _, motions, report, _ = run(job)
        first = next(i for i, m in enumerate(motions) if m['operation'] == 'slot')
        # The earlier pocket plunges (through uncut stock); the rest pocket
        # is checked on the map those moves left.
        heights = Heights((32, 24, 10))
        for motion in motions[:first]:
            heights.cut(motion['curve'])
        heights, ramps = check_descents(self, motions[first:], (32, 24, 10), 4, 150, heights=heights, air=0.05)
        self.assertGreaterEqual(len(ramps), 2)
        self.assertGreaterEqual(report['ramp_entries'], 2)
        self.assertLessEqual(report['max_ramp_angle_deg'], 4 + 1e-9)
        self.assertGreater(report['rest_skipped_strokes'], 0)
        inside = (
            (heights.x[None, :] >= SLOT[0][0] + RADIUS)
            & (heights.x[None, :] <= SLOT[1][0] - RADIUS)
            & (heights.y[:, None] >= SLOT[0][1] + 0.2)
            & (heights.y[:, None] <= SLOT[1][1] - 0.2)
        )
        self.assertTrue((heights.top[inside] == 6).all())
        # (Its ramps are the line moves a Pocket's are, whose posting the
        # pocket test above decodes.)

    def test_adaptive_pocket_ramps_into_a_slot_no_helix_fits(self):
        # 26 x 5 mm for a 4 mm cutter: a helix has no room. The ramp opens
        # the slot; every cutting move after it stays within the engagement
        # the stepover declares.
        job = adaptive_job(RampEntry(4, min_length_mm=2))
        events, motions, report, _ = run(job)
        heights, ramps = check_descents(self, motions, (32, 24, 10), 4, 150, kind='lead', air=0.05)
        self.assertGreaterEqual(len(ramps), 2)
        self.assertEqual(report['helical_entries'], 0)
        self.assertGreaterEqual(report['ramp_entries'], 2)
        self.assertLessEqual(report['max_ramp_angle_deg'], 4 + 1e-9)
        limit = math.degrees(math.acos(1 - 1.5 / RADIUS))
        for layer in report['layers']:
            clearing = layer['clearing']
            self.assertGreaterEqual(clearing['ramp_entries'], 1)
            self.assertGreater(clearing['ramp_slot_mm'], 0)
            self.assertLessEqual(clearing['proven_max_engagement_deg'], limit + 1e-9)
        for motion in motions:
            curve = motion['curve']
            self.assertNotEqual(curve['kind'], 'helix', motion)
            if motion['kind'] == 'rapid':
                continue
            for x, y, z in (curve['start'], curve['end']):
                self.assertTrue(SLOT[0][0] + RADIUS - 1e-6 <= x <= SLOT[1][0] - RADIUS + 1e-6, motion)
                self.assertTrue(SLOT[0][1] + RADIUS - 1e-6 <= y <= SLOT[1][1] - RADIUS + 1e-6, motion)
                self.assertGreaterEqual(z, 6)
        inside = (
            (heights.x[None, :] >= SLOT[0][0] + RADIUS)
            & (heights.x[None, :] <= SLOT[1][0] - RADIUS)
            & (heights.y[:, None] >= SLOT[0][1] + 0.2)
            & (heights.y[:, None] <= SLOT[1][1] - 0.2)
        )
        self.assertTrue((heights.top[inside] == 6).all())
        decoded, text = posted_motions(job, events)
        self.assertTrue(text.endswith('M2\n'))
        posted = [
            (m['curve']['start'][2] - m['curve']['end'][2]) / horizontal(m['curve'])
            for m in decoded
            if m['curve']['end'][2] < m['curve']['start'][2] and horizontal(m['curve']) > 0
        ]
        self.assertEqual(len(posted), len(ramps))
        self.assertLessEqual(max(posted), math.tan(math.radians(4)) + 1e-4)
        # A job that asks for no ramp is the job it was.
        self.assertNotIn('ramp_entries', run(adaptive_job(None, region=((3.0, 3.0), (29.0, 21.0))))[2])

    def test_adaptive_and_roughing_refusals_name_the_reason(self):
        cases = (
            (adaptive_job(RampEntry(4)), ('no ramp entry into the enclosed material', 'min_length_mm is not set')),
            (
                adaptive_job(RampEntry(4, min_length_mm=2), path_strategy='disk_front'),
                ('RampEntry is not supported here', 'engagement_front ramps into enclosed material'),
            ),
        )
        for job, parts in cases:
            with Simulation(**job) as sim:
                with self.assertRaises(SimulationError) as raised:
                    list(sim.run_operations())
            for part in parts:
                self.assertIn(part, str(raised.exception))

    def test_a_ramp_does_not_lift_the_need_for_a_center_cutting_cutter(self):
        # Decided in #219: every ramp turns back along the slope it has just
        # cut, and a cutter that is not center cutting must be lifted first.
        t = tool(center_cutting=False)
        jobs = (
            slot_job(RampEntry(10), t=t),
            slot_job(RampEntry(4, min_length_mm=2), t=t),
            slot_job(RampEntry(4, min_length_mm=2), t=t, rest_tolerance_mm=0.01),
            adaptive_job(RampEntry(4, min_length_mm=2), t=t),
        )
        for job in jobs:
            with Simulation(**job) as sim:
                with self.assertRaises(SimulationError) as raised:
                    list(sim.run_operations())
                self.assertEqual(sim.state.completed, 0)
            self.assertIn('a RampEntry does not lift the need for a center-cutting end mill', str(raised.exception))
            self.assertIn('must first be lifted', str(raised.exception))
        for entry in (None, HelicalEntry()):
            with Simulation(**slot_job(entry, t=t)) as sim:
                with self.assertRaises(SimulationError) as raised:
                    list(sim.run_operations())
            self.assertIn('center-cutting', str(raised.exception))


if __name__ == '__main__':
    unittest.main()
