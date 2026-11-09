"""Declared machine checks of retained actual decoded NC, with synthetic values."""

from dataclasses import replace
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest

from camlib.export import ExportRefused
from camlib.machine import (
    AllowedContact,
    Axis,
    Box,
    ConvexMesh,
    MachineBody,
    MachineCheckOptions,
    MachineModel,
    ToolCalibration,
    ToolChangeRoute,
)
from camlib.motion import Line, Motion, WorkFrame
from camlib.posted import PostedRun
from camlib.simulation import StockSpec
from camlib.verification import StockContactOptions
from test_post import fixture


def model(program, machine):
    t = program.tools[0]
    g = t.gauge_length_mm
    return MachineModel(
        1,
        'mm',
        'synthetic',
        '1',
        'milling',
        (Axis('head', -200, 200),) * 3,
        1,
        WorkFrame(),
        (0, 0, 0),
        (0, 0, 0),
        (5, 0, 3 + g),
        None,
        'linear',
        'synthetic exact-stop controller',
        (ToolCalibration(t.id, machine.tools[0].length_register, (), (0, 0, g), (0, 0, g)),),
        (MachineBody('head', 'spindle_head', 'head', (Box((-1, -1, 0), (1, 1, 10)),), {}),),
        {'column': 'synthetic stand', 'table': 'remote support', 'enclosure': 'open stand'},
        (AllowedContact(('head', '@tool'), 'tool mounted in spindle'),),
        {
            t.id: ToolChangeRoute(
                'synthetic complete indexing enclosure', (), (Box((-5, -5, -g - 1), (5, 5, 1)),), ('@tool',), ()
            )
        },
        (),
        MachineCheckOptions(0.00001, 0.05, 1_000_000, 128, StockContactOptions()),
    )


def run_options(program, machine):
    return dict(
        machine=replace(machine, crash_model=model(program, machine)), stock=StockSpec((100, 100, 0), (110, 110, 10), 1)
    )


class MachineTest(unittest.TestCase):
    def test_missing_model_refuses_and_acceptance_is_recorded(self):
        p, m = fixture()
        with (
            TemporaryDirectory() as folder,
            PostedRun(p, machine=m, stock=StockSpec((100, 100, 0), (110, 110, 10), 1)) as run,
        ):
            target = Path(folder) / 'part.nc'
            target.write_text('old file')
            with self.assertRaises(ExportRefused) as refused:
                run.export(target)
            self.assertEqual(refused.exception.statuses, ('unchecked_machine',))
            self.assertEqual(target.read_text(), 'old file')
            result = run.export(target, accept={'unchecked_machine'})
            self.assertEqual(result['accepted'], ('unchecked_machine',))
            self.assertIn('machine', result['unchecked'])
            self.assertFalse(result['machine']['complete'])

    def test_complete_bound_model_passes_and_actual_head_crash_names_move(self):
        p, m = fixture()
        options = run_options(p, m)
        with PostedRun(p, **options) as run:
            r = run.check_machine()
            self.assertEqual(r['status'], 'clear', r)
            self.assertTrue(r['complete'])
            self.assertEqual(r['decoded_motions'], 1)
            self.assertEqual(r['tool_changes'], 1)
            self.assertEqual(run.export()['accepted'], ())
        declared = options['machine'].crash_model
        clamp = MachineBody('clamp', 'fixture', 'fixed', (Box((7, -2, 66), (8, 2, 70)),), {})
        options['machine'] = replace(
            options['machine'], crash_model=replace(declared, bodies=(*declared.bodies, clamp))
        )
        with PostedRun(p, **options) as run:
            with self.assertRaises(ExportRefused) as refused:
                run.export()
            findings = [f for group in refused.exception.findings for f in group['findings']]
            hit = next(f for f in findings if set(f['bodies']) == {'head', 'clamp'})
            self.assertEqual(hit['motion_id'], 1)
            self.assertEqual(hit['status'], 'violation')
            self.assertGreater(hit['fraction'], 0)
            self.assertLess(hit['fraction'], 1)
            released = run.export(accept={'violation'})
            self.assertEqual(released['accepted'], ('violation',))

    def test_independent_axis_rapid_refuses_clamp_off_nominal_diagonal(self):
        p, m = fixture()
        p = replace(p, events=(*p.events[:4], Motion(99, 'rapid', 't1', Line((5, 0, 3), (25, 20, 3)), kind='rapid')))
        options = run_options(p, m)
        declared = options['machine'].crash_model
        clamp = MachineBody('clamp', 'fixture', 'fixed', (Box((14, -0.5, 3), (16, 0.5, 8)),), {})
        declared = replace(declared, bodies=(*declared.bodies, clamp))
        for rapid, expected in (('linear', 'clear'), ('axis_independent', 'uncertain')):
            with self.subTest(rapid=rapid):
                options['machine'] = replace(m, crash_model=replace(declared, rapid_mode=rapid))
                with PostedRun(p, **options) as run:
                    report = run.check_machine()
                    self.assertEqual(report['status'], expected, report)
                    if expected == 'uncertain':
                        with self.assertRaises(ExportRefused):
                            run.export()
                        self.assertTrue(
                            any(f['motion_id'] == 99 and 'clamp' in f['bodies'] for f in report['findings'])
                        )

    def test_machine_limits_use_gauge_and_declaration_is_snapshotted(self):
        p, m = fixture()
        options = run_options(p, m)
        declared = options['machine'].crash_model
        options['machine'] = replace(
            m, crash_model=replace(declared, axes=(*declared.axes[:2], Axis('head', -200, 62)))
        )
        # Head is already outside Z travel although the tip Z is only 3.
        with PostedRun(p, **options) as run:
            with self.assertRaisesRegex(ValueError, 'initial position inside'):
                run.check_machine()
        with PostedRun(p, **run_options(p, m)) as run:
            run.wait()
            self.assertEqual(run.posting_manifest()['definition']['machine']['crash_model']['id'], 'synthetic')
            self.assertEqual(run.check_machine()['status'], 'clear')

    def test_missing_components_wrong_register_and_nonconvex_mesh_refuse(self):
        p, m = fixture()
        options = run_options(p, m)
        base = options['machine'].crash_model
        invalid = [
            replace(base, absent_components={}),
            replace(base, tools=(replace(base.tools[0], length_register=8),)),
            replace(
                base,
                bodies=(
                    replace(
                        base.bodies[0],
                        solids=(
                            ConvexMesh(
                                ((0, 0, 0), (1, 0, 0), (0, 1, 0), (0, 0, 1)),
                                ((0, 1, 2),),  # open shell
                            ),
                        ),
                    ),
                ),
            ),
        ]
        for declared in invalid:
            with (
                self.subTest(model=declared.id),
                PostedRun(p, **dict(options, machine=replace(m, crash_model=declared))) as run,
            ):
                with self.assertRaises(ValueError):
                    run.check_machine()

    def test_unsupported_machine_coordinate_and_homing_blocks_refuse_decoder(self):
        # Such blocks cannot silently bypass crash checking through a custom post.
        from camlib.post import GCodePost

        p, m = fixture()
        for block in ('G53 G0 Z0', 'G28 Z0'):

            class UnsafePost(GCodePost):
                def on_finish(self):
                    self.emit(block)
                    super().on_finish()

            with self.subTest(block=block):
                with self.assertRaises(ValueError):
                    from camlib.post import post_program

                    list(post_program(p, m, post=UnsafePost))


class MachineLatheAndPersistenceTest(unittest.TestCase):
    def test_saved_program_rechecks_same_bound_machine(self):
        p, m = fixture()
        with TemporaryDirectory() as folder:
            with PostedRun(p, **run_options(p, m)) as run:
                report = run.check_machine()
                saved = run.save(Path(folder) / 'saved')
            with PostedRun.open(saved) as reopened:
                self.assertEqual(reopened.check_machine(), report)
                self.assertTrue(reopened.export()['text'].endswith('M2\n'))

    def test_later_incomplete_check_does_not_downgrade_a_definite_crash(self):
        p, m = fixture()
        options = run_options(p, m)
        base = options['machine'].crash_model
        clamp = MachineBody('clamp', 'fixture', 'fixed', (Box((7, -2, 66), (8, 2, 70)),), {})
        declared = replace(
            base,
            bodies=(*base.bodies, clamp),
            program_end_route_mm=((250, None, None),),
            options=replace(base.options, max_findings=1),
        )
        with PostedRun(p, **dict(options, machine=replace(m, crash_model=declared))) as run:
            report = run.check_machine()
            self.assertFalse(report['complete'])
            self.assertEqual(report['status'], 'violation')
            with self.assertRaises(ExportRefused) as refused:
                run.export(accept={'uncertain'})
            self.assertEqual(refused.exception.statuses, ('violation',))

    def test_lathe_turret_other_tool_tailstock_and_indexing_are_modelled(self):
        import camlib
        from test_turning import job, lathe

        plan = job(
            initial_position_mm=(100, 100), operations=(camlib.TurnFace('face', 'cnmg', 0, feed_mm_rev=0.15, rpm=1000),)
        ).plan()
        m = lathe(plan)
        # Physical turret datum is the controlled point, not the milling proxy's
        # fake gauge dimensions. Native turning bodies supply the real tool.
        base = MachineModel(
            1,
            'mm',
            'synthetic-lathe',
            '1',
            'turning',
            (Axis('head', -300, 300), Axis('fixed', 0, 0), Axis('head', -300, 300)),
            1,
            WorkFrame(),
            (0, 0, 0),
            (0, 0, 0),
            (100, 0, 100),
            None,
            'linear',
            'synthetic exact-stop lathe',
            (ToolCalibration('cnmg', 1, (), (0, 0, 0), (0, 0, 0)),),
            (
                MachineBody('chuck', 'chuck', 'fixed', (Box((-22, -22, -65), (22, 22, -51)),), {}),
                MachineBody('turret', 'turret', 'head', (Box((-5, -15, 110), (25, 15, 130)),), {}),
            ),
            {
                k: 'not fitted on synthetic single-tool stand'
                for k in ('other_tools', 'tailstock', 'quill', 'steady_rest', 'bed', 'enclosure')
            },
            (
                AllowedContact(('chuck', '@stock'), 'stock clamped in chuck'),
                AllowedContact(('turret', '@tool'), 'tool mounted on turret'),
            ),
            {
                'cnmg': ToolChangeRoute(
                    'synthetic complete turret-index enclosure',
                    (),
                    (Box((-30, -25, -30), (120, 25, 140)),),
                    ('@tool', 'turret'),
                    (),
                )
            },
            (),
            MachineCheckOptions(0.00001, 0.05, 1_000_000, 128, StockContactOptions()),
        )

        # Turret is indexed, so all its state endpoints must be in the envelope.
        def check(declared):
            with plan.job.post('lathe', machine=replace(m, crash_model=declared)) as run:
                return run.check_machine()

        report = check(base)
        self.assertEqual(report['status'], 'clear', report)
        self.assertTrue(report['complete'])
        # A long auxiliary tool on the turret hits a tailstock at the beginning.
        other = MachineBody('other', 'other_tools', 'head', (Box((-2, -2, -5), (2, 2, 5)),), {})
        tail = MachineBody('tailstock', 'tailstock', 'fixed', (Box((98, -1, 98), (102, 1, 102)),), {})
        absent = {k: v for k, v in base.absent_components.items() if k not in ('other_tools', 'tailstock')}
        route = replace(base.tool_change_routes['cnmg'], replaces=('@tool', 'turret', 'other'))
        report = check(
            replace(
                base, bodies=(*base.bodies, other, tail), absent_components=absent, tool_change_routes={'cnmg': route}
            )
        )
        self.assertEqual(report['status'], 'violation', report)
        self.assertTrue(any(set(f['bodies']) == {'other', 'tailstock'} for f in report['findings']))


class MachineJobExportTest(unittest.TestCase):
    def test_job_post_nc_uses_machine_gate_and_configured_geometry(self):
        from test_post_nc import example

        ex = example()
        job = ex.build_job()
        machine = ex.machine_for(job)
        with self.assertRaises(ExportRefused) as refused:
            job.post_nc('top', machine=machine)
        self.assertEqual(refused.exception.statuses, ('unchecked_machine',))
        header = job.compile().setup('top').program_header()
        declared = model(header, machine)
        p = header.initial_position_mm
        declared = replace(declared, initial_axes_mm=(p[0], p[1], p[2] + header.tools[0].gauge_length_mm))
        result = job.post_nc('top', machine=replace(machine, crash_model=declared))
        self.assertTrue(result.machine['complete'])
        self.assertEqual(result.machine['status'], 'clear', result.machine)
        self.assertNotIn('machine', result.unchecked)
        self.assertEqual(result.machine['decoded_motions'], result.motions)


if __name__ == '__main__':
    unittest.main()
