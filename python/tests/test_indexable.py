"""Indexable cutters through the public API, native stock, reports and posting."""

from dataclasses import replace
import json
import unittest
from tempfile import TemporaryDirectory

from camlib import Engine, ToolAssembly
from camlib.jobs import Job, Setup, OperationNode, RegenerationCache
from camlib.material import Options, Target, Verifier
from camlib.motion import Program
from camlib.operations import Pocket, HelicalEntry, RampEntry, Contour, OffsetClearing
from camlib.planar import Boundary, Component, PlanarRegion
from camlib.post import LoweringOptions, Machine, ToolSetting, post_events
from camlib.postprocessor import GCodeDialect
from camlib.simulation import OperationUpdate, Simulation, SimulationError, StockSpec
from support.indexable import facing_job, roughing_job, tool


def floor_target(z=3):
    region = PlanarRegion('floor', (Component(Boundary.polygon(((-10, -8), (10, -8), (10, 8), (-10, 8)))),))
    return Target('floor', '1', region, z)


def run(config, target=None):
    events, reports = [], []
    with Simulation(**config) as sim:
        for update in sim.run_operations():
            if isinstance(update, OperationUpdate):
                events.extend(update.events)
                if update.stage == 'end':
                    reports.append(update.report)
            else:
                events.append(dict(kind='motion', motion=update.motion))
        material = Verifier(
            target or floor_target(), Options(comparison_tolerance_mm=0.002, scope_to_region=True)
        ).check(sim.state)
        return events, reports, material, sim.state.heights.tolist()


class IndexableTest(unittest.TestCase):
    def test_declared_profiles_roundtrip_and_keep_the_center_empty(self):
        with Engine() as engine:
            for rounded in (False, True):
                mill = tool(rounded)
                serialized = json.loads(json.dumps(mill.to_dict()))
                self.assertEqual(ToolAssembly.from_dict(serialized).to_dict(), serialized)
                incoming = json.loads(json.dumps(serialized))
                restored = ToolAssembly.from_dict(incoming)
                incoming['cutter']['shape']['milling']['max_depth_of_cut_mm'] = 999
                exported = restored.to_dict()
                exported['cutter']['shape']['milling']['geometry']['inserts'][0]['count'] = 99
                self.assertEqual(restored.to_dict(), serialized)
                # Public declarations retain tuples; JSON represents them as
                # arrays. Compare the same serialized representation here.
                self.assertEqual(json.loads(json.dumps(mill.to_dict())), serialized)
                response = engine.inspect_tool(mill, (0.5, 2.75))
                self.assertEqual(response['tool'], serialized)
                section = response['annular_profile'][0]['cutting']
                self.assertGreater(section['possible'][0][0], 1)
                self.assertGreater(len(section['certain']), 0)
                self.assertEqual(response['annular_profile'][1]['cutting']['possible'], [])
                self.assertEqual(response['annular_profile'][1]['noncutting']['certain'][0][0], 0)

    def test_facing_plans_simulates_and_posts_with_independent_decode(self):
        config = facing_job()
        events, reports, material, heights = run(config)
        self.assertTrue(reports)
        self.assertLessEqual(max(heights[1::2]), 3.001)
        self.assertEqual(material['gouge']['assessment'], 'within_tolerance')
        self.assertEqual(material['residual']['assessment'], 'within_tolerance')
        header = Program('face', '1', config['tools'], config['initial_position_mm'], (), tolerance_mm=1e-5)
        setting = ToolSetting.for_tool(
            config['tools'][0], number=1, length_register=1, position_after_mm=config['initial_position_mm']
        )
        machine = Machine(
            'synthetic indexable post',
            (setting,),
            dialect=GCodeDialect.linuxcnc(),
            output_units='mm',
            decimals=3,
            max_feed_mm_min=10000,
            max_spindle_rpm=30000,
            planes=('xy', 'xz', 'yz'),
            arcs=True,
            helices=True,
            max_sweep_rad=6.283185307179586,
            max_tool_number=99,
            max_length_register=99,
            tool_changes=True,
            flood=True,
            mist=True,
            tool_change_position_mode='table',
            rapid_mm_min=6000,
        )
        chunks = list(post_events(header, events, machine=machine, lowering=LoweringOptions()))
        self.assertTrue(chunks)

    def test_high_feed_roughing_keeps_layer_depth_and_reports_real_stock(self):
        config = roughing_job()
        # This strip is entirely OUTSIDE the retained boss. Its excess is
        # material under the curved insert, not the intentionally kept boss.
        strip = PlanarRegion('foot', (Component(Boundary.polygon(((-0.9, -2), (-0.1, -2), (-0.1, 2), (-0.9, 2)))),))
        events, reports, material, heights = run(config, Target('foot', '1', strip, 3))
        self.assertTrue(reports)
        cuts = [e['motion'] for e in events if e['kind'] == 'motion' and e['motion']['kind'] == 'cut']
        self.assertTrue(cuts)
        depths = sorted({m['curve']['end'][2] for m in cuts})
        self.assertTrue(all(b - a <= 0.5 + 1e-9 for a, b in zip(depths, depths[1:])))
        self.assertEqual(material['gouge']['assessment'], 'within_tolerance')
        self.assertEqual(material['residual']['assessment'], 'definite_excess')
        self.assertLess(min(heights[1::2]), 4)

    def test_insert_depth_limit_is_checked_before_planning(self):
        config = facing_job()
        config['operations'] = (replace(config['operations'][0], stepdown_mm=1.5),)
        with self.assertRaisesRegex((ValueError, SimulationError), 'max_depth_of_cut_mm'):
            run(config)

    def test_roughing_checks_engagement_in_the_previous_curved_shoulder(self):
        config = roughing_job()
        data = json.loads(json.dumps(config['tools'][0].to_dict()))
        data['cutter']['shape']['milling']['max_depth_of_cut_mm'] = 1
        mill = ToolAssembly.from_dict(data)
        config = dict(config, tools=(mill,), operations=(replace(config['operations'][0], tool=mill),))
        # Nominal 0.5 mm layers alone do not prove a 1 mm engagement limit.
        with self.assertRaisesRegex((ValueError, SimulationError), 'max_depth_of_cut_mm'):
            run(config)

    def test_pocket_helix_and_entry_refusals(self):
        config = facing_job()
        region = PlanarRegion('pocket', (Component(Boundary.polygon(((-15, -15), (15, -15), (15, 15), (-15, 15)))),))
        op = Pocket(
            'pocket',
            config['tools'][0],
            region,
            6,
            5.5,
            0.5,
            3,
            12,
            entry=HelicalEntry(angle_deg=2, radius_mm=3),
            feed_mm_min=500,
            plunge_mm_min=100,
            spindle_rpm=1000,
            tool_number=1,
            length_register=1,
            coolant='flood',
        )
        config = dict(config, operations=(op,), stock=StockSpec((-15, -15, 0), (15, 15, 6), 0.25))
        with Simulation(**config) as sim:
            motions = [u.motion for u in sim.run_operations() if not isinstance(u, OperationUpdate)]
            self.assertTrue(any(m['curve']['kind'] == 'helix' for m in motions))
            self.assertGreater(sim.state.volume()['removed_lower_mm3'], 0)
        for entry, reason in [
            (None, 'cannot plunge'),
            (RampEntry(angle_deg=5), 'max_ramp_angle_deg'),
            (HelicalEntry(angle_deg=5, radius_mm=3), 'max_ramp_angle_deg'),
            (HelicalEntry(angle_deg=2, radius_mm=0.5), 'helical_hole_diameter_mm'),
            (HelicalEntry(angle_deg=2, radius_mm=1), 'helical_hole_diameter_mm'),
        ]:
            with self.subTest(entry=entry), self.assertRaisesRegex((ValueError, SimulationError), reason):
                with Simulation(**dict(config, operations=(replace(op, entry=entry),))) as sim:
                    list(sim.run_operations())

    def test_cache_binds_the_mounted_geometry_and_warm_replay_preserves_stock(self):
        from test_jobs import collect

        config = facing_job()

        def job(mill):
            op = replace(config['operations'][0], tool=mill)
            setup = Setup(
                'main', config['stock'], (mill,), (OperationNode(op, sources=()),), config['initial_position_mm']
            )
            return Job('indexable-cache', '1', (setup,))

        with TemporaryDirectory() as directory:
            cache = RegenerationCache(directory)
            base = job(config['tools'][0])
            cold = collect(base, cache)
            warm = collect(base, cache)
            self.assertFalse(cold['statistics'][('main', 'face')]['cache_hit'])
            self.assertTrue(warm['statistics'][('main', 'face')]['cache_hit'])
            self.assertEqual(cold['states']['main'].heights.tobytes(), warm['states']['main'].heights.tobytes())
            data = json.loads(json.dumps(config['tools'][0].to_dict()))
            data['cutter']['shape']['milling']['geometry']['inserts'][0]['cutting']['regions'][0]['outer_radius_mm'][
                0
            ] = 3.5
            changed = job(ToolAssembly.from_dict(data))
            rerun = collect(changed, cache)
            self.assertFalse(rerun['statistics'][('main', 'face')]['cache_hit'])
            reference = collect(changed, None)
            self.assertEqual(rerun['states']['main'].heights.tobytes(), reference['states']['main'].heights.tobytes())

    def test_offset_roughing_and_exterior_entry_contour(self):
        config = roughing_job()
        offset = replace(config['operations'][0], clearing=OffsetClearing(3))
        region = PlanarRegion('part', (Component(Boundary.polygon(((-2, -2), (2, -2), (2, 2), (-2, 2)))),))
        face = facing_job()
        contour = Contour(
            'outline',
            face['tools'][0],
            region,
            floor_z_mm=5.5,
            stepdown_mm=0.5,
            clearance_z_mm=12,
            lead_in_mm=20,
            lead_out_mm=20,
            feed_mm_min=500,
            plunge_mm_min=100,
            spindle_rpm=1000,
            tool_number=1,
            length_register=1,
            coolant='flood',
        )
        for c in (dict(config, operations=(offset,)), dict(face, operations=(contour,))):
            with self.subTest(strategy=type(c['operations'][0]).__name__), Simulation(**c) as sim:
                list(sim.run_operations())
                self.assertGreater(sim.state.volume()['removed_lower_mm3'], 0)
