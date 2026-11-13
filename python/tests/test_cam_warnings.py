"""Valid-but-suspicious plans are reported, not silent."""

from dataclasses import replace
import unittest
import warnings

from camlib.jobs import Job, Setup
from camlib.operations import Pocket
from camlib.planar import Boundary, Component, PlanarRegion
from camlib.simulation import CamWarning, SimulationError, StockSpec
from camlib.tools import EndMill, HolderSection, ToolAssembly


def run(diameter, square, rest=False):
    tool = ToolAssembly(
        "m",
        EndMill("e", diameter=diameter, flute_length=20, overall_length=60),
        30,
        70,
        (HolderSection(30, 70, 10, 10),),
    )
    (x0, y0), (x1, y1) = square
    region = PlanarRegion("p", (Component(Boundary.polygon(((x0, y0), (x1, y0), (x1, y1), (x0, y1)))),))
    pocket = Pocket(
        "p",
        tool,
        region,
        top_z_mm=10,
        floor_z_mm=7,
        stepdown_mm=1.5,
        stepover_mm=2,
        clearance_z_mm=12,
        feed_mm_min=600,
        plunge_mm_min=150,
        spindle_rpm=12000,
        tool_number=1,
        length_register=1,
        coolant="flood",
    )
    # With `rest`, the pocket is cut 2 mm wider all round first, and a
    # shallower rest pocket of the region follows: nothing is left for it.
    wide = Boundary.polygon(((x0 - 2, y0 - 2), (x1 + 2, y0 - 2), (x1 + 2, y1 + 2), (x0 - 2, y1 + 2)))
    wider = replace(pocket, region=PlanarRegion("wide", (Component(wide),)))
    shallower = replace(pocket, id="rest", rest_tolerance_mm=0.01, floor_z_mm=8.5)
    operations = (wider, shallower) if rest else (pocket,)
    setup = Setup("top", StockSpec((0, 0, 0), (20, 20, 10), 0.5), (tool,), operations, (0, 0, 12))
    with warnings.catch_warnings(record=True) as caught:
        warnings.simplefilter("always")
        with Job("w", "1", (setup,)).regenerate() as job:
            for _ in job:
                pass
    return [str(w.message) for w in caught if issubclass(w.category, CamWarning)]


class CamWarningTest(unittest.TestCase):
    def test_empty_and_air_only_operations_warn_and_normal_ones_do_not(self):
        self.assertEqual(run(6, ((5, 5), (15, 15))), [])
        self.assertIn("removed no stock", " ".join(run(6, ((50, 50), (60, 60)))))
        self.assertIn("operation 'rest' produced no cutting motion", " ".join(run(6, ((5, 5), (15, 15)), rest=True)))

    def test_a_pocket_its_cutter_fits_nowhere_in_is_refused_not_warned(self):
        # A 12 mm cutter in a 10 mm square. This used to be the "no cutting
        # motion" warning. The pocket can do nothing it was asked to, which
        # is a refusal (#250), as it is for AdaptivePocket; the warning
        # stays for plans that are valid and empty (already cleared).
        with self.assertRaises(SimulationError) as raised:
            run(12, ((5, 5), (15, 15)))
        self.assertIn("pocket p: the cutter fits nowhere in the region", str(raised.exception))


if __name__ == "__main__":
    unittest.main()
