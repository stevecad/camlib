from camlib.postprocessor import GCodeDialect
from dataclasses import replace
import json
import math
import os
from tempfile import TemporaryDirectory
import unittest

from camlib.dynamics import MotionPolicy
from camlib.motion import Motion, Line, Arc, Helix, Stop, WorkFrame
from camlib.post import Machine, Cancellation, post_program
from test_post import fixture


class MotionPolicyTest(unittest.TestCase):
    def test_source_unchanged_feed_limited_checked_post_and_definition_identity(self):
        program, setup = fixture()
        machine = Machine(
            'Example mill',
            setup.tools,
            motion_policy=MotionPolicy(100, 6000),
            max_feed_mm_min=600,
            max_axis_feed_mm_min=(400, 500, 600),
            dialect=GCodeDialect.linuxcnc(),
            output_units='mm',
            decimals=3,
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
        before = program.to_dict()
        planned = machine.schedule(program)
        self.assertEqual(program.to_dict(), before)
        self.assertEqual(planned.report['feed_limited_motions'], 1)
        self.assertGreater(planned.motion_seconds, 5 / (400 / 60))
        chunks = list(post_program(planned.program, machine))
        decoded = [m for c in chunks for m in c.motions]
        self.assertEqual(len(decoded), 1)
        self.assertEqual(decoded[0].curve, program.events[4].curve)
        self.assertLess(decoded[0].feed_mm_min, 400)
        self.assertIn('motion_policy', chunks[0].data['definition']['machine'])
        changed = replace(machine, motion_policy=MotionPolicy(200, 6000))
        self.assertNotEqual(planned.report['identity'], changed.schedule(program).report['identity'])
        self.assertNotEqual(
            chunks[0].data['identity'], next(iter(post_program(planned.program, changed))).data['identity']
        )
        self.assertNotEqual(
            planned.report['identity'], machine.schedule(replace(program, revision='r2')).report['identity']
        )
        with self.assertRaisesRegex(ValueError, 'adjust_feeds'):
            machine.schedule(program, adjust_feeds=False)
        token = Cancellation()
        token.cancel()
        with self.assertRaisesRegex(ValueError, 'cancelled'):
            machine.schedule(program, cancel=token)
        self.assertEqual(machine.schedule(program).report, planned.report)

    def test_tangent_motion_time_corners_and_state_boundaries(self):
        p, s = fixture()
        a = p.initial_position_mm
        b = (a[0] + 10, a[1], a[2])
        c = (a[0] + 20, a[1], a[2])
        p = replace(
            p,
            events=(*p.events[:4], Motion(1, 'cut', 't1', Line(a, b), 1200), Motion(2, 'cut', 't1', Line(b, c), 1200)),
        )
        machine = Machine(
            'Example mill',
            s.tools,
            motion_policy=MotionPolicy(100, 6000, 'tangent', 0),
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
        self.assertAlmostEqual(machine.schedule(p).motion_seconds, 1.2)
        stopped = replace(machine, motion_policy=replace(machine.motion_policy, junction_mode='exact_stop'))
        self.assertAlmostEqual(stopped.schedule(p).motion_seconds, 1.4)
        interrupted = replace(p, events=(*p.events[:-1], Stop(), p.events[-1]))
        self.assertAlmostEqual(machine.schedule(interrupted).motion_seconds, 1.4)
        with self.assertRaises(ValueError):
            replace(machine, motion_policy=replace(machine.motion_policy, tangential_accel_mm_s2=0)).schedule(p)
        with self.assertRaisesRegex(ValueError, 'frame'):
            machine.schedule(replace(p, frame=WorkFrame(origin_mm=(1, 0, 0))))

    def test_circular_feed_schedule_posts_in_all_planes_units_senses_and_pitch(self):
        p, s = fixture()
        for unit in ('mm', 'inch'):
            for plane, (u, v, w) in {'xy': (0, 1, 2), 'xz': (2, 0, 1), 'yz': (1, 2, 0)}.items():
                for sign in (-1, 1):
                    for pitch in (0, 2.54):
                        start = (25.4, 0, 0)
                        center = list(start)
                        center[u] -= 2.54
                        end = list(start)
                        end[w] += pitch
                        curve = (Helix if pitch else Arc)(start, tuple(end), tuple(center), sign * math.tau, plane)
                        job = replace(
                            p,
                            initial_position_mm=start,
                            events=(
                                p.events[0],
                                replace(p.events[1], position_after_mm=start),
                                p.events[2],
                                Motion(1, 'circle', 't1', curve, 6000),
                            ),
                        )
                        machine = Machine(
                            'Example mill',
                            (replace(s.tools[0], position_after_mm=start),),
                            output_units=unit,
                            motion_policy=MotionPolicy(100, 6000),
                            max_normal_accel_mm_s2=20,
                            max_axis_feed_mm_min=(900, 1000, 1100),
                            dialect=GCodeDialect.linuxcnc(),
                            decimals=(3 if unit == 'mm' else 4),
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
                        scheduled = machine.schedule(job)
                        chunks = list(post_program(scheduled.program, machine))
                        self.assertEqual(sum(len(c.motions) for c in chunks), 1)
                        self.assertLess(scheduled.report['segments'][0]['scheduled_feed_mm_min'], 6000)

    def test_actual_cad_rest_cache_scheduled_stock_post_and_seek(self):
        from test_rest_job import example, engine
        from camlib.jobs import RegenerationCache
        from camlib.simulation import Simulation
        from camlib.posted import OperationEvents, PostedRun
        from camlib.playback import Player, Cursor

        job = example().build_rest_job()
        job.compile().setup('top')  # compiles and validates the setup
        machine = replace(
            example().rest_machine_for(job),
            motion_policy=MotionPolicy(200, 6000),
            max_feed_mm_min=300,
            max_normal_accel_mm_s2=200,
        )
        with TemporaryDirectory() as directory:
            cache = RegenerationCache(directory)
            identities = []
            saved = None
            for revision in (1, 2):
                setup = replace(job, revision=str(revision)).compile().setup('top')
                sim = Simulation(**setup.simulation_options(cache), executable=engine())
                source = OperationEvents(sim, reuse_active_tool=True)
                try:
                    scheduled = machine.schedule_events(setup.program_header(), source)
                    self.assertTrue(sim.state.finished)
                    raw = sim.state.heights.tobytes()
                finally:
                    source.close()
                if saved is None:
                    saved = raw
                else:
                    self.assertEqual(saved, raw)
                identities.append(scheduled.report['identity'])
                self.assertGreater(scheduled.report['feed_limited_motions'], 0)
                self.assertEqual(
                    {e.tool for e in scheduled.program.events if isinstance(e, Motion)}, {'mill4', 'mill2'}
                )
                # Feed changes alone must preserve every committed native stock cell.
                with Simulation(
                    setup.config['stock'],
                    setup.config['tools'],
                    initial_position_mm=setup.config['initial_position_mm'],
                    sweep=setup.config['sweep'],
                    executable=engine(),
                    revision=setup.config['revision'],
                    stream_version=2,
                ) as replay:
                    for _ in replay.run_events(scheduled.program.events):
                        pass
                    self.assertEqual(replay.state.heights.tobytes(), saved)
                if revision == 1:
                    with PostedRun(
                        scheduled.program,
                        machine=machine,
                        stock=setup.config['stock'],
                        sweep=setup.config['sweep'],
                        verification=setup.config['verification'],
                        executable=engine(),
                        checkpoint_every=16,
                    ) as posted:
                        posted.wait(timeout=120)
                        player = Player(posted)
                        last = Cursor(posted.status()['available'])
                        player.seek(last)
                        self.assertEqual(player.verification['progress']['violation'], 0)
                        self.assertEqual(player.verification['progress']['uncertain'], 0)
                        final = player.state.heights.tobytes()
                        at = Cursor(last.completed // 2, 0.5)
                        player.seek(at)
                        partial = player.state.heights.tobytes()
                        player.seek(last)
                        self.assertEqual(player.state.heights.tobytes(), final)
                        player.seek(at)
                        self.assertEqual(player.state.heights.tobytes(), partial)
            self.assertNotEqual(*identities)

    def test_event_source_cancellation_closes_without_scheduled_result(self):
        p, s = fixture()
        token = Cancellation()
        closed = []
        machine = Machine(
            'Example mill',
            s.tools,
            motion_policy=MotionPolicy(100, 6000),
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

        def source():
            try:
                yield p.events[0]
                token.cancel()
                yield p.events[1]
            finally:
                closed.append(True)

        with self.assertRaisesRegex(ValueError, 'cancelled'):
            machine.schedule_events(replace(p, events=()), source(), cancel=token)
        self.assertEqual(closed, [True])
        result = machine.schedule_events(replace(p, events=()), iter(p.events))
        self.assertEqual(result.program, p)

    def test_complete_connected_cad_passage_schedule_stock_and_checked_post(self):
        from pathlib import Path
        from camlib.motion import Program
        from camlib.post import ToolSetting
        from camlib.tools import ToolAssembly
        from camlib.posted import OperationEvents
        from camlib.simulation import Simulation
        from camlib.verification import StreamChecks, StockContactOptions

        manifest = json.loads(
            (Path(__file__).resolve().parents[2] / 'fixtures/operations/adaptive-connected.json').read_text()
        )
        manifest['operations'][0]['path_strategy'] = 'stock_front'
        tools = tuple(ToolAssembly.from_dict(t) for t in manifest['tools'])
        pose = tuple(manifest['initial_position_mm'])
        header = Program('connected-schedule', 'r1', tools, pose, (), tolerance_mm=manifest['sweep']['tolerance_mm'])
        machine = Machine(
            'Example mill',
            (ToolSetting.for_tool(tools[0], number=1, length_register=1, position_after_mm=pose),),
            motion_policy=MotionPolicy(200, 6000),
            max_feed_mm_min=300,
            dialect=GCodeDialect.linuxcnc(),
            output_units='mm',
            decimals=3,
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
        checks = StreamChecks(stock_contact=StockContactOptions())
        executable = os.environ.get('CAMLIB_TEST_ENGINE')
        with Simulation(
            manifest['stock'],
            tools,
            revision='r1',
            initial_position_mm=pose,
            sweep=manifest['sweep'],
            operations=manifest['operations'],
            verification=checks,
            executable=executable,
        ) as sim:
            scheduled = machine.schedule_events(header, OperationEvents(sim, reuse_active_tool=True))
            saved = sim.state.heights.tobytes()
            source_motion_count = sim.state.completed
            self.assertTrue(sim.state.finished)
        # Check preservation against the independently consumed stock stream.
        # A fixed planner count (3205 on Linux, 3195 on Windows) tests the host's
        # adaptive subdivision, not whether scheduling preserves every motion.
        self.assertGreater(source_motion_count, 3000)
        self.assertEqual(len(scheduled.report['segments']), source_motion_count)
        self.assertGreater(scheduled.report['feed_limited_motions'], 0)
        with Simulation(
            manifest['stock'],
            tools,
            revision='r1',
            initial_position_mm=pose,
            sweep=manifest['sweep'],
            verification=checks,
            executable=executable,
            stream_version=2,
        ) as replay:
            for _ in replay.run_events(scheduled.program.events):
                pass
            self.assertEqual(replay.state.heights.tobytes(), saved)
        chunks = list(post_program(scheduled.program, machine))
        self.assertIn('M2', chunks[-1].text)
        self.assertGreater(sum(len(c.motions) for c in chunks), 3000)
