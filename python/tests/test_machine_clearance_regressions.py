"""Focused #116 cases; these do not declare or verify an entire machine."""

from dataclasses import replace
import unittest

from camlib.motion import Line, Motion
from camlib.planar import Boundary, Component, PlanarRegion
from camlib.tools import EndMill, HolderSection, ToolAssembly
from camlib.verification import Prism, Scene, Verifier
from test_centroid_post import CentroidDecoder, fixture, machine


def clamp(x, y, bottom, top):
    region = PlanarRegion(
        'clamp',
        (
            Component(
                Boundary.polygon(((x - 0.2, y - 0.2), (x + 0.2, y - 0.2), (x + 0.2, y + 0.2), (x - 0.2, y + 0.2)))
            ),
        ),
    )
    return Prism('tall-clamp', region, bottom, top)


def assembly(stickout=10):
    return ToolAssembly(
        'short',
        EndMill('em2', diameter=2, flute_length=5, overall_length=40),
        stickout,
        50,
        (HolderSection(stickout, 50, 6, 6),),
    )


class MachineClearanceRegressionTest(unittest.TestCase):
    def test_straight_rapid_clear_does_not_prove_a_dogleg_clear(self):
        tool = assembly(35)
        diagonal = Motion(1, 'traverse', tool.id, Line((0, 0, 0), (20, 20, 0)), kind='rapid')
        dogleg = replace(diagonal, curve=Line((0, 0, 0), (20, 0, 0)))
        with Verifier(Scene('dogleg', '1', (clamp(10, 0, -1, 2),))) as verifier:
            self.assertEqual(verifier.check(tool, diagonal)['status'], 'clear')
            report = verifier.check(tool, dogleg)
            self.assertEqual(report['status'], 'violation')
            self.assertEqual(report['findings'][0]['object_id'], 'tall-clamp')
        program, settings = fixture()
        decoder = CentroidDecoder(machine(program, settings), program)
        with self.assertRaisesRegex(ValueError, 'dogleg'):
            decoder.read_line('G0 X20 Y20')

    def test_short_holder_hits_tall_fixture_while_tip_and_long_assembly_clear(self):
        motion = Motion(7, 'near-clamp', 'short', Line((-12, 0, 0), (12, 0, 0)), 300)
        with Verifier(Scene('holder', '1', (clamp(0, 4, 12, 20),))) as verifier:
            short = verifier.check(assembly(), motion)
            self.assertEqual(short['status'], 'violation')
            self.assertEqual(short['motion_id'], 7)
            hits = [f for f in short['findings'] if f['part'] == 'holder']
            self.assertTrue(hits)
            self.assertTrue(all(f['object_id'] == 'tall-clamp' for f in hits))
            self.assertEqual(verifier.check(assembly(35), motion)['status'], 'clear')
