"""Real retained CAD roughing through normal Python, stock, replay and posts."""

from camlib.postprocessor import GCodeDialect

from dataclasses import replace
from functools import lru_cache
import importlib.util
import math
import os
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest
from unittest import mock

from camlib import Engine
from camlib.jobs import Job, Setup, RegenerationCache
from camlib.motion import Program
from camlib.operations import Roughing, StockEdgeEntry
from camlib.playback import Cursor, Player, RecordedRun
from camlib.post import LoweringOptions, Machine, ToolSetting, post_events
from camlib.simulation import OperationUpdate, Simulation, SimulationError


@lru_cache(None)
def example():
    path = Path(__file__).resolve().parents[2] / 'python' / 'tests' / 'support' / 'roughing.py'
    spec = importlib.util.spec_from_file_location('retained_roughing_example', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@lru_cache(None)
def config(adaptive=False):
    return example().roughing_job(adaptive)


@lru_cache(None)
def stream(adaptive=False):
    c = config(adaptive)
    events, report, helix, first = [], None, None, False
    motions = 0
    with Simulation(**c) as sim:
        for update in sim.run_operations():
            if isinstance(update, OperationUpdate):
                if not adaptive:
                    events.extend(update.events)
                if update.stage == 'end':
                    report = update.report
            else:
                m = update.motion
                if not adaptive:
                    events.append(dict(kind='motion', motion=m))
                if m['curve']['kind'] == 'helix' and helix is None:
                    # Only enclosed-opening helices contain fresh stock; outside
                    # components descend through the independently proved air disk.
                    # The motion's own index (the stock state may already hold
                    # the rest of its decoded batch).
                    helix = motions
                motions += 1
                if not first and sim.state.volume()['removed_lower_mm3'] > 0.001:
                    first = True
                    assert not sim.state.finished
        assert first and sim.state.finished
        return dict(
            events=events,
            report=report,
            helix=helix,
            count=sim.state.completed,
            heights=sim.state.heights.tobytes(),
            volume=sim.state.volume(),
        )


class RoughingTest(unittest.TestCase):
    def test_stock_front_four_layer_cad_rough_streams_bulk_removal_and_completes(self):
        c = example().roughing_job(stock_front=True, omit_empty_cuts=True)
        self.assertEqual(c['operations'][0].clearing.path_strategy, 'stock_front')
        first_removal = False
        report = None
        with Simulation(**c) as sim:
            for update in sim.run_operations():
                if isinstance(update, OperationUpdate):
                    if update.stage == 'end':
                        report = update.report
                elif not first_removal and sim.state.volume()['removed_lower_mm3'] > 1:
                    self.assertFalse(sim.state.finished)
                    first_removal = True
            self.assertTrue(first_removal and sim.state.finished)
            self.assertIsNotNone(report)
            self.assertEqual(report['completed_layers'], 4)
            self.assertEqual(report['motions'], sim.state.completed)
            self.assertLessEqual(sim.state.completed, 10000)
            self.assertGreater(sim.state.volume()['removed_lower_mm3'], 5500)
            self.assertLess(sim.state.volume()['removed_upper_mm3'], 32 * 28 * 10 - 1888)
            for layer in report['layers']:
                front = layer['clearing']['front']
                self.assertTrue(front['coverage_complete'])
                self.assertEqual(front['completed_components'], 2)
                self.assertGreater(front['bulk_contours'], 0)
                self.assertLessEqual(front['max_bounded_engagement_deg'], report['max_engagement_deg'])

    def test_step_up_layers_follow_each_stepdown_and_remove_the_wall_stair(self):
        base = example().roughing_job(offset=True)
        operation = base['operations'][0]
        self.assertNotIn('stepup_mm', operation.to_dict())
        stepped = dict(base, operations=(replace(operation, stepup_mm=0.5),))
        self.assertEqual(stepped['operations'][0].to_dict()['stepup_mm'], 0.5)

        def run(c):
            with Simulation(**c) as sim:
                report = None
                for update in sim.run_operations():
                    if isinstance(update, OperationUpdate) and update.stage == 'end':
                        report = update.report
                self.assertTrue(sim.state.finished)
                return report, sim.state.volume()['removed_lower_mm3']

        (plain, removed), (report, more) = run(base), run(stepped)
        full = [layer['tip_z_mm'] for layer in plain['layers']]
        self.assertEqual([layer['tip_z_mm'] for layer in report['layers'] if 'step_up_of_z_mm' not in layer], full)
        ups = [layer for layer in report['layers'] if 'step_up_of_z_mm' in layer]
        self.assertTrue(ups)
        self.assertTrue(all(layer['step_up_of_z_mm'] in full for layer in ups))
        self.assertTrue(all(0 < layer['tip_z_mm'] - layer['step_up_of_z_mm'] < 2 for layer in ups))
        self.assertGreater(more, removed + 1.0)

        with self.assertRaises(SimulationError):
            run(dict(base, operations=(replace(operation, stepup_mm=2.0),)))

    def test_engagement_front_clears_every_layer_and_reports_its_estimate(self):
        c = example().roughing_job(engagement_front=True)
        clearing = c['operations'][0].clearing
        self.assertEqual(clearing.to_dict()['path_strategy'], 'engagement_front')
        report = None
        with Simulation(**c) as sim:
            for update in sim.run_operations():
                if isinstance(update, OperationUpdate) and update.stage == 'end':
                    report = update.report
            self.assertTrue(sim.state.finished)
            self.assertEqual(report['completed_layers'], 4)
            self.assertEqual(report['motions'], sim.state.completed)
            self.assertLess(sim.state.completed, 20000)
            self.assertGreater(sim.state.volume()['removed_lower_mm3'], 5500)
            self.assertGreaterEqual(report['helical_entries'], 1)
            for layer in report['layers']:
                front = layer['clearing']
                self.assertEqual(front['strategy'], 'engagement_front')
                # The steering grid's own estimate; the proven bound follows.
                self.assertLessEqual(front['max_estimated_engagement_mm'], 0.25 * 1.3)
                self.assertEqual(front['geometric_residual_status'], 'estimated')
                # The engagement is proven, stroke by stroke (#167).
                limit = math.degrees(math.acos(1 - 0.25 / 1))
                self.assertAlmostEqual(front['engagement_limit_deg'], limit)
                self.assertLessEqual(front['proven_max_engagement_deg'], limit)
                self.assertLessEqual(front['proven_max_engagement_mm'], 0.25)
                self.assertGreater(front['engagement_proved_strokes'], 100)

    def test_issue_167_adaptive_roughing_around_a_block_starts_at_the_stock_edge(self):
        """#167: a 100 x 60 x 20 mm block in a 130 x 90 mm stock, one 7 mm layer.
        The old default orbited through the air beside the stock (17,792 moves,
        635,103 posted) and could not start without a helix."""
        import cadquery as cq

        from camlib import AdaptiveClearing, StockSpec
        from camlib.cadquery import export_surface_mesh
        from camlib.tools import EndMill, HolderSection, ToolAssembly

        block = cq.Workplane('XY').box(100, 60, 20, centered=False).val()
        mill = ToolAssembly(
            'mill10',
            EndMill('em10', diameter=10, flute_length=30, overall_length=75),
            45,
            115,
            (HolderSection(45, 115, 20, 20),),
        )
        self.assertEqual(AdaptiveClearing(3).path_strategy, 'engagement_front')

        def run(entry, clearing=AdaptiveClearing(3)):
            rough = Roughing(
                'rough',
                mill,
                export_surface_mesh(block),
                top_z_mm=21,
                floor_z_mm=14,
                stepdown_mm=7,
                clearance_z_mm=30,
                clearing=clearing,
                radial_allowance_mm=0.3,
                clockwise=False,
                entry=entry,
                feed_mm_min=1000,
                plunge_mm_min=300,
                spindle_rpm=9000,
                coolant='off',
                tool_number=1,
                length_register=1,
            )
            setup = Setup('top', StockSpec((-15, -15, 0), (115, 75, 21), 0.5), (mill,), (rough,), (0, 0, 60))
            kinds, report = [], None
            with Job('adaptive', '1', (setup,)).regenerate() as job:
                for item in job:
                    update = item.update
                    if isinstance(update, OperationUpdate):
                        if update.stage == 'end':
                            report = update.report
                    elif isinstance(getattr(update, 'motion', None), dict):
                        kinds.append((update.motion['kind'], update.motion['curve']['kind']))
            return kinds, report

        paths = []
        for entry in (StockEdgeEntry(helix=None), StockEdgeEntry()):
            kinds, report = run(entry)
            self.assertLess(len(kinds), 400)
            self.assertNotIn('helix', [curve for _, curve in kinds])
            front = report['layers'][0]['clearing']
            self.assertEqual(front['strategy'], 'engagement_front')
            self.assertEqual(front['helical_entries'], 0)
            self.assertLessEqual(front['passes'], 12)
            limit = math.degrees(math.acos(1 - 3 / 5))
            self.assertAlmostEqual(front['engagement_limit_deg'], limit)
            self.assertLessEqual(front['proven_max_engagement_deg'], limit)
            self.assertNotIn('confined_strokes', front)
            paths.append(kinds)
        self.assertEqual(paths[0], paths[1])
        with self.assertRaisesRegex(SimulationError, 'does not clear with path_strategy disk_front'):
            run(StockEdgeEntry(), AdaptiveClearing(3, path_strategy='disk_front'))

    def test_empty_cut_policy_is_an_explicit_strict_python_boolean(self):
        operation = config()['operations'][0]
        self.assertNotIn('omit_empty_cuts', operation.to_dict())
        self.assertIs(replace(operation, omit_empty_cuts=True).to_dict()['omit_empty_cuts'], True)
        for value in (1, 'yes', None):
            with self.assertRaisesRegex(ValueError, 'omit_empty_cuts'):
                replace(operation, omit_empty_cuts=value).to_dict()

    def test_real_cad_both_clearing_strategies_stream_actual_stock_and_typed_program(self):
        solid = example().part().val()
        self.assertTrue(solid.isValid())
        self.assertAlmostEqual(solid.Volume(), 1888)
        self.assertTrue(all(f.geomType() == 'PLANE' for f in solid.Faces()))
        for adaptive in (False, True):
            result = stream(adaptive)
            report = result['report']
            self.assertEqual(report['strategy'], 'roughing')
            self.assertEqual(report['completed_layers'], 4)
            self.assertGreaterEqual(report['stock_edge_entries'], 4)
            self.assertGreaterEqual(report['helical_entries'], 4)
            self.assertEqual(report['motions'], result['count'])
            self.assertGreater(result['volume']['removed_lower_mm3'], 5500)
            self.assertLess(result['volume']['removed_upper_mm3'], 32 * 28 * 10 - 1888)
            self.assertAlmostEqual(report['target_z_mm'], 2.2 + report['floor_geometry_guard_mm'])
            self.assertLess(report['floor_geometry_guard_mm'], 0.001)
            self.assertEqual([item['tip_z_mm'] for item in report['layers'][:3]], [8, 6, 4])
            if adaptive:
                # The contact angle of a straight pass of that width, acos(1 - w/r).
                self.assertAlmostEqual(report['max_engagement_deg'], math.degrees(math.acos(0.75)))
                self.assertEqual(report['max_radial_engagement_mm'], 0.25)
        c = config()
        program = dict(
            version=1,
            id='roughing-program',
            revision='1',
            units='mm',
            tolerance_mm=1e-5,
            frame=dict(origin_mm=(0, 0, 0), x=(1, 0, 0), y=(0, 1, 0), z=(0, 0, 1)),
            initial_position_mm=c['initial_position_mm'],
            tools=[t.to_dict() for t in c['tools']],
            events=stream()['events'],
        )
        with Engine() as engine:
            summary = engine._request('validate_program', program=program, capabilities=None)['summary']
            self.assertEqual(summary['motions'], stream()['count'])

    def test_partial_enclosed_helix_replay_and_preflight_refusal(self):
        for adaptive in (False, True):
            c = config(adaptive)
            with RecordedRun(**{'rapid_mm_min': 6000, **c}, checkpoint_every=256) as run:
                run.wait(timeout=180)
                player = Player(run)
                index = stream(adaptive)['helix']
                self.assertIsNotNone(index)
                player.seek(Cursor(index))
                before = player.state.volume()['removed_lower_mm3']
                cursor = Cursor(index, 0.371)
                player.seek(cursor)
                self.assertEqual(player.motion['curve']['kind'], 'helix')
                saved, position = player.state.heights.tobytes(), player.state.position
                self.assertGreater(player.state.volume()['removed_lower_mm3'], before + 0.001)
                player.seek(Cursor(run.status()['available']))
                self.assertEqual(player.state.heights.tobytes(), stream(adaptive)['heights'])
                player.seek(Cursor())
                player.seek(cursor)
                self.assertEqual(player.state.heights.tobytes(), saved)
                self.assertEqual(player.state.position, position)
                self.assertLessEqual(run.status()['largest_frame_bytes'], 8 * 1024 * 1024)
        c = config()
        # 40,000 layers (stepdown .0002) is refused by the engine's memory
        # guard under an explicit budget, not by a fixed layer cap.
        for op, budget in (
            (replace(c['operations'][0], entry=StockEdgeEntry(helix=None)), None),
            (replace(c['operations'][0], stepdown_mm=0.0002), 1 << 20),
            (replace(c['operations'][0], radial_allowance_mm=-1), None),
        ):
            env = {"CAMLIB_MEMORY_BUDGET_BYTES": str(budget)} if budget else {}
            with mock.patch.dict(os.environ, env), Simulation(**dict(c, operations=(op,))) as sim:
                with self.assertRaises(SimulationError):
                    list(sim.run_operations())
                self.assertEqual(sim.state.completed, 0)
        import cadquery as cq

        parameters = dict(
            top_z_mm=10, floor_z_mm=2, stepdown_mm=2, clearance_z_mm=12, clearing=c['operations'][0].clearing
        )
        supported = Roughing.from_solid(
            'cylinder',
            c['tools'][0],
            cq.Workplane('XY').circle(3).extrude(4),
            **{
                'feed_mm_min': 600,
                'plunge_mm_min': 150,
                'spindle_rpm': 12000,
                'tool_number': 1,
                'length_register': 1,
                'coolant': "flood",
                **parameters,
            },
        )
        self.assertLessEqual(supported.target.surface_error_mm, 0.005)
        self.assertTrue(any(f['kind'] == 'CYLINDER' for f in supported.target.source['face_bounds']))
        # General B-spline patches still lack an interpolation bound. A real
        # nonuniformly scaled sphere exercises unsupported-surface preflight,
        # while cylinders now use the bounded curved importer.
        ellipsoid = (
            cq.Workplane('XY')
            .sphere(3)
            .val()
            .transformGeometry(cq.Matrix([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1.5, 0], [0, 0, 0, 1]]))
        )
        self.assertTrue(ellipsoid.isValid())
        with self.assertRaisesRegex(ValueError, 'no parametric interpolation bound'):
            Roughing.from_solid(
                'unsupported',
                c['tools'][0],
                ellipsoid,
                **{
                    'feed_mm_min': 600,
                    'plunge_mm_min': 150,
                    'spindle_rpm': 12000,
                    'tool_number': 1,
                    'length_register': 1,
                    'coolant': "flood",
                    **parameters,
                },
            )

    def test_generic_python_checked_post_and_independent_setup_reuse(self):
        c = config()
        header = Program('posted-roughing', '1', c['tools'], c['initial_position_mm'], (), tolerance_mm=1e-5)
        setting = ToolSetting.for_tool(
            c['tools'][0], number=1, length_register=1, position_after_mm=c['initial_position_mm']
        )
        machine = Machine(
            'Python roughing machine',
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
            max_tool_number=99999,
            max_length_register=99999,
            tool_changes=True,
            flood=True,
            mist=True,
            tool_change_position_mode='table',
            rapid_mm_min=6000,
        )
        chunks = list(post_events(header, stream()['events'], machine=machine, lowering=LoweringOptions()))
        self.assertTrue(chunks[-1].text.endswith('M2\n'))
        self.assertTrue(all(m.curve.to_dict()['kind'] == 'line' for chunk in chunks for m in chunk.motions))
        mappings = [m for chunk in chunks for m in chunk.data.get('mappings', ())]
        helices = [
            e['motion']['id']
            for e in stream()['events']
            if e['kind'] == 'motion' and e['motion']['curve']['kind'] == 'helix'
        ]
        for id in helices:
            intervals = [m['source_fraction_interval'] for m in mappings if m['source_motion_id'] == id]
            self.assertEqual(intervals[0][0], 0)
            self.assertEqual(intervals[-1][1], 1)
            self.assertTrue(all(a[1] == b[0] for a, b in zip(intervals, intervals[1:])))

        def job(feed):
            main = replace(c['operations'][0], feed_mm_min=feed)
            other = replace(c['operations'][0], id='other-rough')
            return Job(
                'roughing-job',
                str(feed),
                tuple(
                    Setup(id, c['stock'], c['tools'], (op,), c['initial_position_mm'])
                    for id, op in [('main', main), ('other', other)]
                ),
            )

        def collect(j, cache):
            with j.regenerate(cache) as run:
                for _ in run:
                    pass
                self.assertTrue(run.finished)
                return dict(run.statistics), {k: s.heights.tobytes() for k, s in run.states.items()}

        with TemporaryDirectory() as directory:
            cache = RegenerationCache(directory)
            cold, heights = collect(job(600), cache)
            self.assertTrue(all(not v['cache_hit'] for v in cold.values()))
            warm, same = collect(job(600), cache)
            self.assertTrue(all(v['cache_hit'] for v in warm.values()))
            self.assertEqual(heights, same)
            changed, same = collect(job(700), cache)
            self.assertFalse(changed[('main', 'rough')]['cache_hit'])
            self.assertTrue(changed[('other', 'other-rough')]['cache_hit'])
            self.assertEqual(heights, same)


class MeshFinenessTest(unittest.TestCase):
    """Roughing's moves do not follow a fine mesh's facets (issue #89)."""

    def test_moves_do_not_grow_with_mesh_fineness_and_nothing_is_gouged(self):
        from camlib.material import Options, SurfaceTarget, Verifier
        from support import filleted_boss

        counts, fine = {}, None
        for error in (0.005, 0.05):
            config = filleted_boss.roughing_job(error)
            report = None
            with Simulation(**config) as sim:
                moves = 0
                for update in sim.run_operations():
                    if isinstance(update, OperationUpdate):
                        if update.stage == "end":
                            report = update.report
                    else:
                        moves += 1
                if error == 0.005:
                    fine = config["operations"][0].target
                    material = Verifier(
                        SurfaceTarget("boss", "1", fine, 0, source_enclosure="offset"), Options()
                    ).check(sim.state)
            counts[error] = moves
            # On the fine mesh every layer's shadow was simplified (a quarter
            # of the 0.3 mm radial allowance, from at least twice as many
            # vertices); a coarse one may keep its exact shadow.
            for layer in report["layers"]:
                simplified = layer["shadow"]["simplified"]
                self.assertEqual(simplified["tolerance_mm"], 0.075)
                if error == 0.005:
                    self.assertNotIn("reason", simplified)
        # 27,258 against 3,050 facets: the same number of moves, give or take.
        self.assertLess(abs(counts[0.005] - counts[0.05]), 0.15 * counts[0.05], counts)
        self.assertEqual(material["gouge"]["definite_cells"], 0)
        self.assertTrue(material["complete"])
