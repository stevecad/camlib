"""Checked recipe comparisons on small synthetic jobs, including actual NC replay."""

from dataclasses import replace
import os
from tempfile import TemporaryDirectory
import unittest
from unittest.mock import patch

from camlib.dynamics import MotionPolicy
from camlib.jobs import Job, Setup, RegenerationCache, OperationNode
from camlib.material import Target, SurfaceTarget, Options
from camlib.machine import Box, MachineBody, ToolCalibration, ToolChangeRoute
from camlib.operations import Facing, RestPocket, Parallel, RampEntry
from camlib.planar import Boundary, Component, PlanarRegion
from camlib.planning import (
    PlanningCandidate,
    ComparisonPolicy,
    OperationChoices,
    CorrectionTemplate,
    recipe_candidates,
    compare_candidates,
    correction_candidate,
    correct_residuals,
)
from camlib.post import Cancellation, ToolSetting
from camlib.simulation import StockSpec
from camlib.surface import SurfaceMesh
from camlib.tools import HolderSection
from camlib.verification import Scene, MachineLimits, StreamChecks, StockContactOptions, MaterialCheck, Prism
from test_machine import model
from test_post import fixture


def candidate(id='reference', *, feed=600, floor=0.5):
    program, machine = fixture()
    # The declared facing region includes intentional cutter overtravel.
    region = PlanarRegion('top', (Component(Boundary.polygon(((-10, -10), (18, -10), (18, 18), (-10, 18)))),))
    checks = StreamChecks(
        Scene('limits', '1', machine=MachineLimits((-100, -100, -100), (100, 100, 100), 10000)),
        material=(MaterialCheck(Target('top', '1', region, 0.5), options=Options(comparison_tolerance_mm=0.01)),),
        stock_contact=StockContactOptions(),
    )
    operation = Facing(
        'face',
        't1',
        floor,
        0.5,
        2,
        3,
        feed_mm_min=feed,
        plunge_mm_min=200,
        spindle_rpm=12000,
        tool_number=2,
        length_register=7,
        coolant='flood',
    )
    job = Job(
        'comparison',
        '1',
        (
            Setup(
                'top',
                StockSpec((0, 0, 0), (8, 8, 1), 0.5),
                program.tools,
                (operation,),
                program.initial_position_mm,
                verification=checks,
            ),
        ),
    )
    machine = replace(
        machine, crash_model=model(program, machine), motion_policy=MotionPolicy(100, 6000), schedule_post_feeds=True
    )
    return PlanningCandidate(id, job, 'top', machine)


POLICY = ComparisonPolicy((('top', 2.0),), 30)


def surface_correction(shape='ball', entry=True):
    """Real native cutter against an exact inclined plane and unchanged goal."""
    base = candidate()
    setup = base.job.setups[0]
    profile = (('kind', shape),) + ((('corner_radius_mm', 0.2),) if shape == 'bull' else ())
    tool = replace(
        setup.tools[0],
        cutter=replace(
            setup.tools[0].cutter,
            shape=profile,
            diameter_mm=1,
            shank_diameter_mm=1,
        ),
    )

    def point(x, y):
        return x, y, 0.35 + 0.05 * x

    mesh = SurfaceMesh(
        (
            (point(0, 0), point(4, 0), point(4, 4)),
            (point(0, 0), point(4, 4), point(0, 4)),
        )
    )
    rest = Parallel(
        'surface-rest',
        't1',
        mesh,
        0,
        0.25,
        3,
        rest_threshold_mm=0.005,
        safety_margin_mm=0.005,
        path_error_mm=0.002,
        feed_mm_min=600,
        plunge_mm_min=200,
        spindle_rpm=12000,
        tool_number=2,
        length_register=7,
        coolant='flood',
        # The correction's passes start under the leftover they remove: a
        # declared ramp enters them (none: such a pass is refused).
        entry=RampEntry(angle_deg=10, min_length_mm=0.25) if entry else None,
    )
    checks = replace(
        setup.verification,
        material=(
            MaterialCheck(
                SurfaceTarget('surface', '1', mesh, 0),
                options=Options(comparison_tolerance_mm=0.02, max_findings=1024),
            ),
        ),
    )
    setup = replace(
        setup,
        stock=StockSpec((0, 0, 0), (4, 4, 1), 0.25),
        tools=(tool,),
        operations=(replace(rest, id='surface-rough', stock_to_leave_mm=0.3, rest_threshold_mm=None),),
        verification=checks,
    )
    machine = replace(
        base.machine,
        tools=(
            ToolSetting.for_tool(
                tool,
                number=2,
                length_register=7,
                position_after_mm=setup.initial_position_mm,
            ),
        ),
    )
    return replace(base, job=replace(base.job, setups=(setup,)), machine=machine), rest


class PlanningTest(unittest.TestCase):
    def test_correction_construction_keeps_the_original_total_deadline(self):
        from camlib import Engine
        from camlib.planning import CandidateEvaluation, ComparisonReport, CorrectionRegion, _snapshot

        base = candidate(floor=0.9)
        target = base.job.setups[0].verification.material[0].target
        template = RestPocket(
            'touchup',
            't1',
            target.machining_region,
            1,
            0.5,
            0.5,
            2,
            3,
            feed_mm_min=600,
            plunge_mm_min=200,
            spindle_rpm=12000,
            tool_number=2,
            length_register=7,
            coolant='flood',
        )
        clock, unions = [100.0], []
        boolean_region = Engine.boolean_region

        def compare(candidates, *, reference, policy, **kwargs):
            clock[0] += 20  # Two thirds of the entire thirty-second budget.
            evaluation = CandidateEvaluation(
                reference,
                _snapshot(base, policy)[3],
                'rejected',
                residual_mm3=(('top', 0, 8),),
                release_checked=True,
                correction_regions=(
                    CorrectionRegion(
                        'top',
                        ((0, 0, 2, 2), (2, 0, 4, 2), (4, 0, 6, 2), (6, 0, 8, 2)),
                        True,
                    ),
                ),
            )
            return ComparisonReport('request', reference, (evaluation,), None, (), True)

        def slow_union(engine, other, operation, **kwargs):
            result = boolean_region(engine, other, operation, **kwargs)
            clock[0] += 6  # Indivisible native work may finish just past the deadline.
            unions.append(clock[0])
            return result

        with (
            patch('camlib.planning.time.monotonic', side_effect=lambda: clock[0]),
            patch('camlib.planning.compare_candidates', side_effect=compare) as comparison,
            patch.object(Engine, 'boolean_region', new=slow_union),
        ):
            result = correct_residuals(base, (CorrectionTemplate('top', template),), policy=POLICY, max_passes=1)
        self.assertIsNone(result.candidate)
        self.assertIn('time budget exhausted', result.reason)
        self.assertEqual(unions, [126, 132])
        self.assertEqual(comparison.call_count, 1)

    def test_eligible_correction_cannot_hide_an_increased_target_bound(self):
        from camlib.planning import CandidateEvaluation, ComparisonReport

        base = candidate()
        policy = ComparisonPolicy((('left', 6), ('right', 6)), 30)
        before = CandidateEvaluation(
            'reference', 'before', 'rejected', residual_mm3=(('left', 0, 4), ('right', 0, 8)), release_checked=True
        )
        after = replace(before, input_sha256='after', status='eligible', residual_mm3=(('right', 0, 5), ('left', 0, 5)))
        reports = (
            ComparisonReport('first', 'reference', (before,), None, (), True),
            ComparisonReport('second', 'reference', (after,), 'reference', ('reference',), True),
        )
        with (
            patch('camlib.planning.compare_candidates', side_effect=reports),
            patch('camlib.planning.correction_candidate', return_value=base),
        ):
            result = correct_residuals(base, (), policy=policy, max_passes=1)
        self.assertIsNone(result.candidate)
        self.assertEqual(result.reason, 'residual upper volume increased')
        self.assertEqual(len(result.evaluations), 2)

    def test_native_parallel_rest_correction_posts_real_surface_passes(self):
        # Ball and bull ends are proven with their real shape (#107), so
        # their corrections are release-checked like the flat end's.
        policy = ComparisonPolicy((('surface', 3.6),), 120)
        for shape in ('flat', 'ball', 'bull'):
            with self.subTest(shape=shape):
                base, rest = surface_correction(shape)
                original = base.job.compile()
                result = correct_residuals(
                    base,
                    (CorrectionTemplate('surface', rest),),
                    policy=policy,
                    max_passes=1,
                    executable=os.environ.get('CAMLIB_TEST_ENGINE'),
                )
                self.assertIsNotNone(result.candidate, result)
                self.assertEqual([e.status for e in result.evaluations], ['rejected', 'eligible'])
                first, final = result.evaluations
                self.assertGreater(first.residual_mm3[0][2], 3.6)
                self.assertLessEqual(final.residual_mm3[0][2], 3.6)
                self.assertLess(final.residual_mm3[0][2], first.residual_mm3[0][2])
                self.assertTrue(all(e.release_checked and e.posted_blocks > 0 for e in result.evaluations))
                self.assertGreater(final.posted_blocks, first.posted_blocks)
                self.assertEqual(base.job.compile(), original)
                revised = result.candidate.job.setups[0]
                self.assertEqual(revised.verification, base.job.setups[0].verification)
                self.assertEqual(revised.tools, base.job.setups[0].tools)
                self.assertIsNotNone(revised.operations[-1].limits.contain)

    def test_native_parallel_rest_correction_with_no_entry_is_refused_by_name(self):
        # The same correction with no entry declared: its passes start under
        # the leftover, so the candidate is refused, naming the entry.
        policy = ComparisonPolicy((('surface', 3.6),), 120)
        base, rest = surface_correction('ball', entry=False)
        result = correct_residuals(
            base,
            (CorrectionTemplate('surface', rest),),
            policy=policy,
            max_passes=1,
            executable=os.environ.get('CAMLIB_TEST_ENGINE'),
        )
        self.assertIsNone(result.candidate, result)
        self.assertIn('declares no entry', result.reason + ' ' + str(result.evaluations[-1].reasons))

    def test_native_surface_rest_refuses_inaccessible_keepout_and_unsafe_corrections(self):
        from camlib.limits import AvoidArea, Limits

        base, rest = surface_correction('flat')
        policy = ComparisonPolicy((('surface', 3.6),), 120)
        far = PlanarRegion(
            'inaccessible',
            (
                Component(
                    Boundary.polygon(
                        ((20, 20), (24, 20), (24, 24), (20, 24)),
                    )
                ),
            ),
        )
        protected = PlanarRegion(
            'keepout',
            (
                Component(
                    Boundary.polygon(
                        ((-1, -1), (5, -1), (5, 5), (-1, 5)),
                    )
                ),
            ),
        )
        for template, reason in (
            (OperationNode(rest, limits=Limits(far)), 'no residual correction region'),
            (OperationNode(rest, limits=Limits(avoid=(AvoidArea(protected, 2),))), 'no resolvable strokes'),
            (replace(rest, stock_to_leave_mm=0.3), 'no resolvable strokes'),
        ):
            with self.subTest(reason=reason):
                result = correct_residuals(
                    base,
                    (CorrectionTemplate('surface', template),),
                    policy=policy,
                    max_passes=1,
                    executable=os.environ.get('CAMLIB_TEST_ENGINE'),
                )
                self.assertIsNone(result.candidate, result)
                self.assertTrue(result.evaluations[0].release_checked)
                self.assertIn(reason, result.reason + ' ' + str(result.evaluations[-1].reasons))
        # A supplied recipe can describe the wrong machining surface. The
        # original independent final target must still reject its real overcut.
        too_low = SurfaceMesh(
            tuple(tuple((x, y, z - 0.15) for x, y, z in triangle) for triangle in rest.surface.triangles)
        )
        result = correct_residuals(
            base,
            (CorrectionTemplate('surface', replace(rest, surface=too_low)),),
            policy=policy,
            max_passes=1,
            executable=os.environ.get('CAMLIB_TEST_ENGINE'),
        )
        self.assertIsNone(result.candidate, result)
        self.assertEqual(len(result.evaluations), 2)
        self.assertTrue(result.evaluations[0].release_checked)
        self.assertFalse(result.evaluations[-1].release_checked)
        material = [
            finding
            for group in result.evaluations[-1].findings
            for finding in group['findings']
            if finding.get('target_id') == 'surface'
        ]
        self.assertTrue(material)
        self.assertTrue(any(report['gouge']['definite_cells'] > 0 for report in material))

    def test_cancellation_between_snapshots_stops_preparation(self):
        import camlib.planning as planning

        token = Cancellation()
        original = planning._snapshot

        def snapshot(*args):
            result = original(*args)
            token.cancel()
            return result

        with patch.object(planning, '_snapshot', side_effect=snapshot) as prepare:
            result = compare_candidates(
                (candidate(), candidate('alternative', feed=700)), reference='reference', policy=POLICY, cancel=token
            )
        self.assertEqual(prepare.call_count, 1)
        self.assertFalse(result.complete)
        self.assertIsNone(result.selected)
        self.assertEqual([r.status for r in result.evaluations], ['incomplete', 'incomplete'])
        self.assertTrue(all(r.motion_seconds is None and not r.release_checked for r in result.evaluations))

    def test_supplied_tool_proposal_is_rejected_when_its_holder_hits_a_fixture(self):
        base = candidate()
        setup = base.job.setups[0]
        wide = replace(setup.tools[0], id='wide', holders=(HolderSection(35, 60, 10, 10),))
        clamp = PlanarRegion('clamp', (Component(Boundary.polygon(((18, 2), (20, 2), (20, 6), (18, 6)))),))
        checks = replace(
            setup.verification,
            scene=replace(setup.verification.scene, bodies=(Prism('clamp', clamp, 35, 45, role='fixture'),)),
        )
        setup = replace(setup, tools=(*setup.tools, wide), verification=checks)
        machine = base.machine
        physical = machine.crash_model
        physical = replace(
            physical,
            tools=(*physical.tools, ToolCalibration('wide', 8, (), (0, 0, 60), (0, 0, 60))),
            bodies=(*physical.bodies, MachineBody('clamp', 'fixture', 'fixed', (Box((18, 2, 35), (20, 6, 45)),), {})),
            tool_change_routes={
                **physical.tool_change_routes,
                'wide': ToolChangeRoute(
                    'declared wide-holder indexing enclosure', (), (Box((-10, -10, -61), (10, 10, 1)),), ('@tool',), ()
                ),
            },
        )
        machine = replace(
            machine,
            crash_model=physical,
            tools=(
                *machine.tools,
                ToolSetting.for_tool(wide, number=3, length_register=8, position_after_mm=setup.initial_position_mm),
            ),
        )
        base = replace(base, job=replace(base.job, setups=(setup,)), machine=machine)
        operation = setup.operations[0]
        alternative = replace(operation, tool='wide', tool_number=3, length_register=8, feed_mm_min=900)
        choices = recipe_candidates(base, (OperationChoices('face', (alternative,)),), max_candidates=2)
        result = compare_candidates(choices, reference='reference', policy=POLICY)
        self.assertEqual(result.selected, 'reference', result)
        rejected = result.evaluations[1]
        self.assertEqual(rejected.status, 'rejected')
        self.assertIn('clamp', str(rejected.findings))
        self.assertEqual(rejected.assemblies, (('wide', 35, 60),))

    def test_native_rest_correction_rechecks_the_whole_program(self):
        base = candidate(floor=0.9)
        target = base.job.setups[0].verification.material[0].target
        template = RestPocket(
            'touchup',
            't1',
            target.machining_region,
            1,
            0.5,
            0.5,
            2,
            3,
            feed_mm_min=600,
            plunge_mm_min=200,
            spindle_rpm=12000,
            tool_number=2,
            length_register=7,
            coolant='flood',
        )
        result = correct_residuals(base, (CorrectionTemplate('top', template),), policy=POLICY, max_passes=2)
        self.assertIsNotNone(result.candidate, result.reason)
        self.assertEqual(result.evaluations[0].status, 'rejected')
        self.assertEqual(result.evaluations[-1].status, 'eligible')
        self.assertEqual(len(base.job.setups[0].operations), 1)
        self.assertGreater(len(result.candidate.job.setups[0].operations), 1)
        self.assertTrue(result.evaluations[-1].release_checked)
        bad = compare_candidates((candidate(floor=0.1),), reference='reference', policy=POLICY).evaluations[0]
        with self.assertRaisesRegex(ValueError, 'release check'):
            correction_candidate(base, bad, (CorrectionTemplate('top', template),), policy=POLICY)

    def test_correction_stops_on_no_progress_and_never_relaxes_the_goal(self):
        base = candidate(floor=0.9)
        target = base.job.setups[0].verification.material[0].target
        template = RestPocket(
            'touchup',
            't1',
            target.machining_region,
            1,
            0.9,
            0.5,
            2,
            3,
            feed_mm_min=600,
            plunge_mm_min=200,
            spindle_rpm=12000,
            tool_number=2,
            length_register=7,
            coolant='flood',
        )
        result = correct_residuals(base, (CorrectionTemplate('top', template),), policy=POLICY, max_passes=3)
        self.assertIsNone(result.candidate)
        self.assertIn('no progress', result.reason)
        self.assertEqual(len(result.evaluations), 2)
        first = result.evaluations[0]
        changed = replace(base, job=replace(base.job, revision='changed'))
        with self.assertRaisesRegex(ValueError, 'different inputs'):
            correction_candidate(changed, first, (CorrectionTemplate('top', template),), policy=POLICY)

    def test_supplied_recipe_combinations_retain_baseline_and_metadata(self):
        base = candidate()
        operation = base.job.setups[0].operations[0]
        alternatives = (OperationChoices('face', (operation, replace(operation, feed_mm_min=900))),)
        before = base.job.compile()
        choices = recipe_candidates(base, alternatives, max_candidates=2)
        self.assertEqual(len(choices), 2)
        self.assertIs(choices[0], base)
        self.assertEqual(base.job.compile(), before)
        report = compare_candidates(choices, reference='reference', policy=POLICY)
        self.assertEqual(report.selected, choices[1].id)
        self.assertEqual(report.evaluations[1].assemblies, (('t1', 35, 60),))
        bad = replace(operation, tool_number=20)
        with self.assertRaisesRegex(ValueError, 'register'):
            recipe_candidates(base, (OperationChoices('face', (bad,)),), max_candidates=2)
        with self.assertRaisesRegex(ValueError, 'operation ids'):
            recipe_candidates(base, (OperationChoices('missing', (operation,)),), max_candidates=2)
        with self.assertRaisesRegex(ValueError, 'exceed'):
            recipe_candidates(base, alternatives, max_candidates=1)
        self.assertEqual(
            len(recipe_candidates(base, (OperationChoices('face', (operation, operation)),), max_candidates=1)), 1
        )

    def test_reference_ties_and_faster_valid_recipe_with_posted_stock_checks(self):
        slow, fast = candidate(), candidate('fast', feed=900)
        with TemporaryDirectory() as directory:
            cache = RegenerationCache(directory)
            report = compare_candidates((fast, slow), reference='reference', policy=POLICY, cache=cache)
            self.assertTrue(report.complete)
            self.assertEqual(report.selected, 'fast', report)
            self.assertEqual([r.id for r in report.evaluations], ['reference', 'fast'])
            self.assertTrue(all(r.status == 'eligible' and r.posted_blocks > 0 for r in report.evaluations), report)
            self.assertEqual(report.evaluations[0].tool_count, 1)
            warm = compare_candidates((fast, slow), reference='reference', policy=POLICY, cache=cache)
            self.assertEqual(report, warm)
        tie = compare_candidates((candidate('other'), slow), reference='reference', policy=POLICY)
        self.assertEqual(tie.selected, 'reference')
        self.assertEqual(set(tie.best_tradeoffs), {'reference', 'other'})

    def test_faster_gouging_recipe_cannot_win_and_no_acceptance_api(self):
        result = compare_candidates(
            (candidate(), candidate('bad', feed=5000, floor=0.1)), reference='reference', policy=POLICY
        )
        self.assertEqual(result.selected, 'reference', result)
        bad = result.evaluations[1]
        self.assertEqual(bad.status, 'rejected')
        self.assertTrue(bad.findings or bad.reasons)
        with self.assertRaises(TypeError):
            compare_candidates((candidate(),), reference='reference', policy=POLICY, accept={'violation'})

    def test_residual_completion_is_separate_from_gouge_acceptance(self):
        report = compare_candidates(
            (candidate(), candidate('unfinished', feed=5000, floor=0.9)), reference='reference', policy=POLICY
        )
        self.assertEqual(report.selected, 'reference', report)
        self.assertEqual(report.evaluations[1].status, 'rejected')
        self.assertIn('residual upper volume', ' '.join(report.evaluations[1].reasons))

    def test_missing_coverage_or_changed_premises_fail_before_running(self):
        base = candidate()
        with self.assertRaisesRegex(ValueError, 'physical machine'):
            compare_candidates(
                (replace(base, machine=replace(base.machine, crash_model=None)),), reference='reference', policy=POLICY
            )
        setup = base.job.setups[0]
        unchecked = replace(base, job=replace(base.job, setups=(replace(setup, verification=None),)))
        with self.assertRaisesRegex(ValueError, 'scene, stock-contact and material'):
            compare_candidates((unchecked,), reference='reference', policy=POLICY)
        changed = replace(base, id='foreign', machine=replace(base.machine, decimals=5))
        with self.assertRaisesRegex(ValueError, 'premises'):
            compare_candidates((base, changed), reference='reference', policy=POLICY)
        with self.assertRaisesRegex(ValueError, 'final material'):
            compare_candidates((base,), reference='reference', policy=ComparisonPolicy((('missing', 1),), 30))

    def test_cancellation_selects_nothing_and_policy_changes_identity(self):
        token = Cancellation()
        token.cancel()
        stopped = compare_candidates((candidate(),), reference='reference', policy=POLICY, cancel=token)
        self.assertFalse(stopped.complete)
        self.assertIsNone(stopped.selected)
        self.assertEqual(stopped.evaluations[0].status, 'incomplete')
        changed = compare_candidates(
            (candidate(),),
            reference='reference',
            policy=replace(POLICY, residual_limits_mm3=(('top', 3),)),
            cancel=token,
        )
        self.assertNotEqual(stopped.input_sha256, changed.input_sha256)
        exhausted = compare_candidates((candidate(),), reference='reference', policy=replace(POLICY, max_seconds=1e-9))
        self.assertFalse(exhausted.complete)
        self.assertIsNone(exhausted.selected)

    def test_compiled_post_uses_snapshot_and_release_check_does_not_collect_text(self):
        base = candidate()
        compiled = base.job.compile().setup('top')
        with compiled.post(machine=base.machine) as run:
            report = run.check_release()
            self.assertNotIn('text', report)
            self.assertNotIn('path', report)
            self.assertEqual(report['accepted'], ())
            self.assertTrue(report['machine']['complete'])
            exported = run.export()
            self.assertTrue(exported['text'].startswith('%') or 'G' in exported['text'])
            for key, value in report.items():
                self.assertEqual(exported[key], value)


if __name__ == '__main__':
    unittest.main()
