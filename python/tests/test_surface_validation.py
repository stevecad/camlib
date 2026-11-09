import math
import unittest

from camlib.surface import SurfaceMesh


class SurfaceValidationTest(unittest.TestCase):
    def test_surface_error_range_is_checked_at_construction(self):
        triangles = (((0, 0, 1), (1, 0, 1), (0, 1, 1)),)
        for error in (0, 0.1, math.nextafter(0.1, 0)):
            self.assertEqual(SurfaceMesh(triangles, error).to_dict()['surface_error_mm'], error)
        for error in (-math.ulp(0.0), math.nextafter(0.1, math.inf), 0.117, math.nan, math.inf, -math.inf):
            with self.subTest(error=error), self.assertRaisesRegex(ValueError, 'surface_error_mm .*0..=0.1 mm'):
                SurfaceMesh(triangles, error)
