"""Roughing from a front bound behind stock that is not turned (#427): the
passes cannot be reached by their rapids, and the setup is refused before
any move, naming z_from_mm, the stock standing in front of it and the fix.
Roughed in front first, the same operation plans and is released."""

import unittest

import camlib
from test_turning import job
from test_turning_second_chucking import A_SIDE, release

# The collar shaft from a 45 mm bar (#425), faced.
COLLAR = dict(stock=camlib.TurnStockSpec(-77, 2, 22.5), profile=A_SIDE, chuck_z_mm=-67, initial_position_mm=(32.5, 20))
FACE = camlib.TurnFace("face", "cnmg", 0, feed_mm_rev=0.15, rpm=1000)
ROUGH30 = camlib.TurnRough("rough30", "cnmg", feed_mm_rev=0.25, rpm=1000, z_from_mm=-20, z_to_mm=-40)


class FrontBoundTest(unittest.TestCase):
    def test_stock_standing_in_front_of_the_bound_is_refused_by_name(self):
        lathe = job(**COLLAR, operations=(FACE, ROUGH30))
        expected = (
            r"rough30: the roughing pass at radius 21\.300 \(the nose centre\) behind z_from_mm -20 cannot be "
            r"reached: the stock in front of z_from_mm stands up to radius 22\.500 between z -20 and z 0, .*"
            r"Rough the stretch in front of z -20 first \(a TurnRough with z_to_mm -20 before this one\), or give "
            r"a z_from_mm where the stock in front is already turned down$"
        )
        with self.assertRaisesRegex(Exception, expected):
            lathe.plan()
        # Posting plans the setup itself: refused the same way, no program.
        for name in ("fanuc_lathe_post", "centroid_lathe_post"):
            with self.subTest(post=name), self.assertRaisesRegex(Exception, expected):
                release(lathe.job, name)

    def test_roughed_in_front_first_the_same_operation_is_released(self):
        rough = camlib.TurnRough("rough", "cnmg", feed_mm_rev=0.25, rpm=1000, z_to_mm=-20)
        lathe = job(**COLLAR, operations=(FACE, rough, ROUGH30))
        plan = lathe.plan()
        self.assertEqual([r["id"] for r in plan.reports], ["face", "rough", "rough30"])
        for name in ("fanuc_lathe_post", "centroid_lathe_post"):
            with self.subTest(post=name):
                posted = release(lathe.job, name)
                self.assertFalse(posted.findings)
                self.assertEqual(posted.material["gouge"]["assessment"], "within_tolerance")


if __name__ == "__main__":
    unittest.main()
