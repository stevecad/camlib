"""Closed orbits (full circles) post at coarse controller precision (#58).

An earlier block's word choice can move the decoded start of a full circle
off its rounded source start. The circle must still close exactly: written
natively, its end word repeats the decoded start (start and end radii then
identical); lowered to lines, its last chord ends at that same start.
"""

from dataclasses import replace
import math
import unittest

from camlib.motion import Arc, Line, Motion
from camlib.post import LoweringOptions, post_program
from test_post import fixture

# An arc whose 3-decimal words exist only with a moved end: its end rounds to
# (7.146, -1.729) alone, but no radius-consistent words end there. The full orbit that follows (same circle, as adaptive
# clearing does) then starts off its own rounded source start.
CENTER = (7.5, 0.0, 3.0)
ARC_START = (5.736, 0.053, 3.0)
ORBIT_START = (7.146189078076751, -1.728965827171787, 3.0)
ROUNDED_START = (7.146, -1.729)


def orbit_program():
    p, m = fixture()
    events = list(p.events)
    i = next(k for k, e in enumerate(events) if isinstance(e, Motion))
    events[i : i + 1] = [
        Motion(1, 'pocket', 't1', Line((5, 0, 3), ARC_START), 1200),
        Motion(2, 'pocket', 't1', Arc(ARC_START, ORBIT_START, CENTER, 1.3989821295774763), 1200),
        Motion(3, 'pocket', 't1', Arc(ORBIT_START, ORBIT_START, CENTER, math.tau), 1200),
        Motion(4, 'pocket', 't1', Line(ORBIT_START, (5, 0, 3)), 1200),
    ]
    return replace(p, events=tuple(events)), replace(m, decimals=3, tolerance_mm=0.01)


def posted(program, machine, lowering):
    return [c for c in post_program(program, machine=machine, lowering=lowering)]


def motions(chunks):
    return [mv for c in chunks for mv in c.motions]


class ClosedOrbitPosting(unittest.TestCase):
    def test_native_full_circle_repeats_the_decoded_start(self):
        program, machine = orbit_program()
        moves = motions(posted(program, machine, LoweringOptions(native_arcs=True)))
        arcs = [mv.curve for mv in moves if isinstance(mv.curve, Arc)]
        self.assertEqual(len(arcs), 2, 'both curves are native blocks')
        moved = arcs[0].end[:2]
        self.assertNotEqual(moved, ROUNDED_START, 'the first arc moved its end word')
        orbit = arcs[1]
        self.assertEqual(orbit.start[:2], moved)
        self.assertEqual(orbit.end, orbit.start)
        self.assertAlmostEqual(orbit.sweep_rad, math.tau)

    def test_lowered_full_circle_closes_at_the_decoded_start(self):
        program, machine = orbit_program()
        # The machine takes the first arc natively (moved end) but not a full
        # turn, so only the orbit is lowered: its chords must end on the
        # moved start, not on the orbit's own rounded source start.
        moves = motions(posted(program, replace(machine, max_sweep_rad=math.pi), LoweringOptions(native_arcs=True)))
        kinds = [type(mv.curve) for mv in moves]
        self.assertEqual(kinds.count(Arc), 1)
        first = kinds.index(Arc)
        moved = moves[first].curve.end[:2]
        self.assertNotEqual(moved, ROUNDED_START, 'the first arc moved its end word')
        chords = [mv.curve for mv in moves[first + 1 : -1]]
        self.assertGreater(len(chords), 8)
        self.assertTrue(all(isinstance(c, Line) for c in chords))
        self.assertEqual(chords[0].start[:2], moved)
        self.assertEqual(chords[-1].end[:2], moved)

    def test_adaptive_orbits_post_at_three_decimals(self):
        """The workload that found #58: every orbit stays one native block."""
        from camlib.jobs import Job, Setup
        from camlib.post import Machine, ToolSetting
        from camlib.postprocessor import GCodeDialect
        from support.adaptive_pocket import adaptive_job

        c = adaptive_job()
        tool = c["tools"][0]
        setup = Setup("top", c["stock"], c["tools"], c["operations"], c["initial_position_mm"])
        job = Job("adaptive-orbits", c["revision"], (setup,))
        machine = Machine(
            "3-decimal mill",
            (ToolSetting.for_tool(tool, number=1, length_register=1, position_after_mm=c["initial_position_mm"]),),
            dialect=GCodeDialect.linuxcnc(),
            output_units="mm",
            decimals=3,
            max_feed_mm_min=10000,
            max_spindle_rpm=30000,
            planes=("xy", "xz", "yz"),
            arcs=True,
            helices=True,
            max_sweep_rad=math.tau,
            max_tool_number=99999,
            max_length_register=99999,
            tool_changes=True,
            flood=True,
            mist=True,
            tool_change_position_mode="table",
            rapid_mm_min=6000,
        )
        coarse = job.post_nc("top", machine=machine, lowering=LoweringOptions(native_arcs=True)).text
        fine = job.post_nc("top", machine=replace(machine, decimals=6), lowering=LoweringOptions(native_arcs=True)).text
        # Same blocks as at 6 decimals: no orbit had to be lowered to chords.
        self.assertEqual(len(coarse.splitlines()), len(fine.splitlines()))


if __name__ == "__main__":
    unittest.main()
