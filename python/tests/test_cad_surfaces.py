"""Curved CAD import bounds, compact transport, and actual native finishing."""

from collections import Counter
import json
import math
import os
import unittest
from functools import lru_cache
from camlib.cadquery import export_surface_mesh
from camlib._cad_surface import _interpolation_error, _surface_bounds, _bezier_bounds


@lru_cache(None)
def example():
    import importlib.util
    from pathlib import Path

    spec = importlib.util.spec_from_file_location(
        'curved_solid_example', Path(__file__).resolve().parents[2] / 'python/tests/support/curved_solid.py'
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@lru_cache(None)
def curved_part():
    return example().target()


@lru_cache(None)
def imported_part():
    return export_surface_mesh(curved_part(), source={'revision': '1'})


class ParametricCadTest(unittest.TestCase):
    def test_whole_trimmed_solid_and_compact_indexed_wire(self):
        mesh = imported_part()
        kinds = Counter(f['kind'] for f in mesh.source['face_bounds'])
        self.assertTrue({'PLANE', 'CYLINDER', 'TORUS', 'EXTRUSION'} <= set(kinds))
        self.assertLessEqual(mesh.surface_error_mm, 0.005)
        self.assertEqual(sum(f['facets'] for f in mesh.source['face_bounds']), len(mesh.triangles))
        for face in mesh.source['face_bounds']:
            bound = (
                face['interpolation_error_mm']
                + face['trim_error_mm']
                + face['edge_weld_mm']
                + mesh.source['cad_topology_error_mm']
                + mesh.source['numerical_guard_mm']
            )
            self.assertLessEqual(bound, mesh.surface_error_mm)
        wire = mesh.to_dict()
        self.assertIn('indices', wire)
        restored = tuple(tuple(wire['vertices'][i] for i in tri) for tri in wire['indices'])
        self.assertEqual(restored, mesh.triangles)
        self.assertLess(len(json.dumps(wire)), len(json.dumps(dict(triangles=mesh.triangles))) * 0.65)

    def test_surface_bounds_cover_analytic_interpolation_on_transformed_cad(self):
        import cadquery as cq
        from OCP.BRepAdaptor import BRepAdaptor_Surface

        shapes = (
            cq.Solid.makeCylinder(3, 5),
            cq.Solid.makeCone(4, 2, 5),
            cq.Solid.makeTorus(5, 1),
            curved_part().val(),
        )
        for shape in shapes:
            shape = shape.rotate((0, 0, 0), (1, 2, 3), 31).translate((200, -50, 17))
            for face in shape.Faces():
                surface = BRepAdaptor_Surface(face.wrapped)
                bounds = _surface_bounds(surface)
                u0, u1 = surface.FirstUParameter(), surface.LastUParameter()
                v0, v1 = surface.FirstVParameter(), surface.LastVParameter()
                # Whole-patch Hessian proof is tested against independently
                # evaluated interior UV points, across long/skinny/oblique cells.
                for fu, fv in ((0.1, 0.8), (0.7, 0.03), (0.3, 0.4)):
                    uv = (
                        (u0, v0),
                        (u0 + (u1 - u0) * fu, v0 + (v1 - v0) * fv),
                        (u0 + (u1 - u0) * fu * 0.4, v0 + (v1 - v0) * fv * 0.9),
                    )
                    points = [tuple(surface.Value(*p).Coord()) for p in uv]
                    bound = _interpolation_error(uv, bounds)
                    for weights in ((0.2, 0.3, 0.5), (0.05, 0.9, 0.05), (0.6, 0.1, 0.3)):
                        p = tuple(sum(w * q[k] for w, q in zip(weights, uv)) for k in (0, 1))
                        actual = tuple(surface.Value(*p).Coord())
                        linear = tuple(sum(w * q[k] for w, q in zip(weights, points)) for k in range(3))
                        self.assertLessEqual(math.dist(actual, linear), bound + 1e-9)

    def test_rational_bezier_bounds(self):
        import cadquery as cq

        # A quadratic CAD curve supplies independent actual derivatives.
        from OCP.BRepAdaptor import BRepAdaptor_Curve

        edge = cq.Edge.makeBezier([cq.Vector(0, 0, 0), cq.Vector(2, 4, 1), cq.Vector(5, 0, 2)])
        curve = BRepAdaptor_Curve(edge.wrapped).Bezier()
        speed, accel = _bezier_bounds(curve)
        from OCP.gp import gp_Pnt, gp_Vec

        for i in range(33):
            p = gp_Pnt()
            d1 = gp_Vec()
            d2 = gp_Vec()
            curve.D2(i / 32, p, d1, d2)
            self.assertLessEqual(d1.Magnitude(), speed + 1e-12)
            self.assertLessEqual(d2.Magnitude(), accel + 1e-12)

    def test_positive_rational_weights_and_c0_join_use_valid_derivative_bounds(self):
        from OCP.gp import gp_Pnt, gp_Vec
        from OCP.Geom import Geom_BezierCurve, Geom_BSplineCurve
        from OCP.GeomAdaptor import GeomAdaptor_Curve
        from camlib._cad_surface import _curve_bounds

        try:
            from OCP.TColgp import TColgp_Array1OfPnt as Points
            from OCP.TColStd import TColStd_Array1OfReal as Reals, TColStd_Array1OfInteger as Integers
        except ImportError:
            from OCP.collections import Array1_gp_Pnt as Points, Array1_double as Reals, Array1_int as Integers
        points = Points(1, 3)
        weights = Reals(1, 3)
        for i, (x, y, w) in enumerate(((1, 0, 1), (1, 1, math.sqrt(0.5)), (0, 1, 1)), 1):
            points.SetValue(i, gp_Pnt(x, y, 0))
            weights.SetValue(i, w)
        curve = Geom_BezierCurve(points, weights)
        speed, accel = _bezier_bounds(curve)
        for i in range(33):
            p = gp_Pnt()
            d1 = gp_Vec()
            d2 = gp_Vec()
            curve.D2(i / 32, p, d1, d2)
            self.assertAlmostEqual(p.X() ** 2 + p.Y() ** 2, 1)
            self.assertLessEqual(d1.Magnitude(), speed)
            self.assertLessEqual(d2.Magnitude(), accel)
        for i, (x, y) in enumerate(((0, 0), (1, 2), (2, 0)), 1):
            points.SetValue(i, gp_Pnt(x, y, 0))
        knots = Reals(1, 3)
        multiplicities = Integers(1, 3)
        for i, (k, m) in enumerate(((0, 2), (0.5, 1), (1, 2)), 1):
            knots.SetValue(i, k)
            multiplicities.SetValue(i, m)
        spline = Geom_BSplineCurve(points, knots, multiplicities, 1)
        speed, accel = _curve_bounds(GeomAdaptor_Curve(spline), 3)
        self.assertIsNone(accel)
        # A piecewise line has zero second derivative on each span but a corner
        # at the join. The first-derivative bound covers that whole interval.
        deviation = math.dist(tuple(spline.Value(0.5).Coord()), (1, 0, 0))
        self.assertGreater(deviation, 1)
        self.assertLessEqual(deviation, speed / 2)

    def test_actual_sphere_seam_poles_budgets_and_cancellation(self):
        import cadquery as cq

        sphere = cq.Workplane('XY').sphere(2)
        mesh = export_surface_mesh(sphere, max_error_mm=0.01)
        self.assertLessEqual(mesh.surface_error_mm, 0.01)
        for tri in mesh.triangles:
            for p in (*tri, tuple(sum(p[k] for p in tri) / 3 for k in range(3))):
                self.assertLessEqual(abs(math.sqrt(sum(v * v for v in p)) - 2), mesh.surface_error_mm + 1e-9)
        with self.assertRaisesRegex(ValueError, 'max_facets'):
            export_surface_mesh(sphere, max_facets=4)
        with self.assertRaisesRegex(ValueError, 'cancelled'):
            export_surface_mesh(sphere, cancel=lambda: True)
        for kwargs in ({'max_error_mm': float('nan')}, {'max_error_mm': 0}, {'max_facets': 3}):
            with self.assertRaises(ValueError):
                export_surface_mesh(sphere, **kwargs)

    def test_native_parallel_stream_on_actual_curved_solid(self):
        from camlib.operations import Parallel
        from camlib.simulation import Simulation, StockSpec, Update, OperationUpdate
        from camlib.tools import BallEndMill, ToolAssembly, HolderSection

        tool = ToolAssembly(
            'ball',
            BallEndMill('bit', diameter=3, flute_length=20, overall_length=40),
            25,
            50,
            (HolderSection(25, 50, 4, 4),),
        )
        operation = Parallel(
            'finish',
            tool,
            imported_part(),
            6,
            0.5,
            11,
            feed_mm_min=600,
            plunge_mm_min=150,
            spindle_rpm=12000,
            tool_number=1,
            length_register=1,
            coolant="flood",
        )
        moves = []
        report = None
        with Simulation(
            StockSpec((0, 0, 0), (18, 14, 8.5), 0.25),
            (tool,),
            revision='1',
            initial_position_mm=(0, 0, 11),
            operations=(operation,),
            executable=os.environ.get("CAMLIB_TEST_ENGINE"),
        ) as sim:
            for update in sim.run_operations():
                if isinstance(update, Update):
                    moves.append(update.motion)
                elif isinstance(update, OperationUpdate) and update.stage == 'end':
                    report = update.report
            self.assertTrue(sim.state.finished)
            self.assertGreater(sim.state.volume()['removed_lower_mm3'], 100)
            self.assertEqual(report['rows'], report['rows_completed'])
            self.assertGreater(report['vertical_transitions'], 0)
            self.assertGreater(report['reused_segments'], 0)
            self.assertEqual(report['transition_moves'], 2 * report['vertical_transitions'])
            self.assertLessEqual(report['max_certified_path_excess_mm'], operation.path_error_mm)
            self.assertGreater(report['max_transition_height_mm'], 0.1)
        self.assertTrue(
            any(abs(m['curve']['start'][2] - m['curve']['end'][2]) > 0.01 for m in moves if m['kind'] == 'cut')
        )
        # Independent whole-segment CAD distance, including face interiors.
        import cadquery as cq
        from OCP.BRepExtrema import BRepExtrema_DistShapeShape

        part = curved_part().val()
        for motion in moves:
            curve = motion['curve']
            self.assertEqual(curve['kind'], 'line')
            a = tuple(v + (1.5 if k == 2 else 0) for k, v in enumerate(curve['start']))
            b = tuple(v + (1.5 if k == 2 else 0) for k, v in enumerate(curve['end']))
            edge = cq.Edge.makeLine(cq.Vector(*a), cq.Vector(*b))
            distance = BRepExtrema_DistShapeShape(edge.wrapped, part.wrapped)
            self.assertTrue(distance.IsDone())
            self.assertFalse(distance.InnerSolution())
            self.assertGreaterEqual(distance.Value(), 1.5 - 1e-6)

    def test_curved_job_checked_post_partial_seek_and_material_bounds(self):
        from camlib.playback import Player, Cursor
        from camlib.material import SurfaceTarget, Verifier

        job = example().build_job()
        controls = []
        with job.post(
            'top',
            machine=example().machine_for(job),
            executable=os.environ.get('CAMLIB_TEST_ENGINE'),
            on_operation=lambda u: controls.append(u),
        ) as run:
            run.wait()
            player = Player(run)
            last = run.status()['available']
            player.seek(Cursor(last))
            self.assertTrue(player.state.finished)
            self.assertGreater(run.posting_status()['checked_motions'], 100)
            self.assertEqual(player.verification['progress']['violation'], 0)
            self.assertEqual(player.verification['progress']['uncertain'], 0)
            verifier = Verifier(SurfaceTarget('curved', '1', job.setups[0].operations[0].surface, 0))
            material = verifier.check(player.state)
            self.assertTrue(material['complete'])
            self.assertEqual(material['gouge']['definite_cells'], 0)
            self.assertGreater(material['residual']['volume_mm3'][1], 0)
            self.assertGreater([u.report for u in controls if u.stage == 'end'][0]['vertical_transitions'], 0)
            player.pause()
            player.set_speed(2)
            cursor = Cursor(last // 2, 0.5)
            player.seek(cursor)
            partial = player.state.heights.tobytes()
            player.seek(Cursor(last))
            player.seek(cursor)
            self.assertEqual(player.state.heights.tobytes(), partial)
            self.assertLess(run.posting_status()['max_chunk_bytes'], 512 * 1024)
