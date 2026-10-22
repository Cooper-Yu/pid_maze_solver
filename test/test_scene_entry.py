"""Scene contract tests use files and owned fake processes, never ROS motion nodes."""

import contextlib
import copy
import io
import json
import os
from pathlib import Path
import signal
import sys
import tempfile
import threading
import unittest
from unittest.mock import patch

import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools'))
import maze_entry  # noqa: E402
import scene_route  # noqa: E402


class SceneFiles(unittest.TestCase):
    def setUp(self):
        self.real = yaml.safe_load((ROOT / 'waypoints/waypoints_real.yaml').read_text())

    def test_scene1_exact_task5_parameters(self):
        route = scene_route.load_scene(ROOT / 'waypoints/waypoints_sim.yaml', 1)
        old = yaml.safe_load((ROOT / 'config/motion_route.yaml').read_text())
        expected = old['pid_maze_solver']['ros__parameters']
        for key in ('max_speed', 'max_acceleration', 'stop_hold'):
            expected.pop(key)
        self.assertEqual(route['parameters'], expected)

    def test_scene2_complete_fixed_endpoints(self):
        route = scene_route.validate_real(self.real)
        self.assertEqual(len(route['actions']), 20)
        self.assertEqual(route['actions'][0]['points'][0], [0.0, 0.0])
        p13 = next(a for a in route['actions'] if a['name'] == 'P13')
        self.assertEqual(p13['points'][-1], [0.4258019547712687, -1.0819099009985675])
        self.assertEqual(
            route['actions'][-1]['points'][-1], [0.02899960878708014, -0.5596575672149936]
        )

    def test_guarded_prefix_and_explicit_validation(self):
        route = scene_route.validate_real(self.real)
        self.assertIs(route['validated'], False)
        self.assertEqual(route['test_until'], 'P15')
        self.assertEqual(len(scene_route.select_real(route, 'P15', True)['actions']), 20)
        route['test_until'] = 'P12'
        self.assertEqual(scene_route.select_real(route, 'P12', True)['actions'][-1]['name'], 'P12')
        with self.assertRaisesRegex(ValueError, 'not validated'):
            scene_route.select_real(route, 'P15', True)
        route['validated'] = True
        self.assertEqual(len(scene_route.select_real(route, 'P15', True)['actions']), 20)
        self.assertEqual(scene_route.select_real(route, 'P03_turn')['actions'][-1]['kind'], 'turn')

    def test_discontinuity_nonfinite_and_hidden_heading_rejected(self):
        for mutate in (
            lambda r: r['actions'][0]['points'][0].__setitem__(0, 0.01),
            lambda r: r['actions'][0]['points'][1].__setitem__(0, float('nan')),
            lambda r: r['actions'][0].__setitem__('heading_rad', 0.5),
            lambda r: r['actions'][0]['points'].__setitem__(1, [0.5, 0.0]),
            lambda r: r['actions'][1].__setitem__('name', 'P02'),
            lambda r: r.__setitem__('validated', 'true'),
        ):
            with self.subTest(mutate=mutate), self.assertRaises(ValueError):
                bad = copy.deepcopy(self.real)
                mutate(bad)
                scene_route.validate_real(bad)

    def test_yaml_duplicates_and_scene_mismatch(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'route.yaml'
            path.write_text('schema: 1\nschema: 1\n')
            with self.assertRaises(ValueError):
                scene_route.load_scene(path, 2)
        with self.assertRaises(ValueError):
            scene_route.load_scene(ROOT / 'waypoints/waypoints_real.yaml', 1)


class ReverseSceneFiles(unittest.TestCase):
    def setUp(self):
        self.path = ROOT / 'waypoints/reverse_waypoints_sim.yaml'
        self.reverse = yaml.safe_load(self.path.read_text())

    def test_reverse_file_complete_order_and_parameters(self):
        route = scene_route.load_scene(self.path, 3)
        self.assertEqual((route['scene'], route['frame']), (3, 'P15'))
        self.assertEqual(route['parameters'], self.reverse['parameters'])
        self.assertEqual(route['parameters']['route_direction'], 'reverse')
        self.assertEqual(
            list(route['parameters']['steps']),
            [f'P{i:02d}_P{i - 1:02d}' for i in range(15, 1, -1)],
        )
        self.assertFalse(route['parameters']['final_clockwise_turn'])

    def test_reverse_order_canonicalized_without_changing_steps(self):
        data = copy.deepcopy(self.reverse)
        data['parameters']['steps'] = dict(reversed(list(data['parameters']['steps'].items())))
        route = scene_route.validate_sim(data, 3)
        self.assertEqual(list(route['parameters']['steps']), list(scene_route.REVERSE_STEP_NAMES))
        self.assertEqual(route['parameters']['steps'], self.reverse['parameters']['steps'])

    def test_reverse_direction_complete_names_and_values_required(self):
        for mutate in (
            lambda p: p.pop('route_direction'),
            lambda p: p.__setitem__('route_direction', 'forward'),
            lambda p: p.__setitem__('route_mode', 'waypoints'),
            lambda p: p['steps'].pop('P02_P01'),
            lambda p: p['steps'].__setitem__('P01_P02', p['steps'].pop('P15_P14')),
            lambda p: p['steps']['P15_P14'].__setitem__('forward_m', float('nan')),
            lambda p: p.__setitem__('use_sim_time', False),
        ):
            with self.subTest(mutate=mutate), self.assertRaises(ValueError):
                data = copy.deepcopy(self.reverse)
                mutate(data['parameters'])
                scene_route.validate_sim(data, 3)

    def test_scene_and_frame_cannot_cross_boundaries(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'wrong.yaml'
            for key, value in (('frame', 'P01'), ('scene', 1), ('scene', 4)):
                data = copy.deepcopy(self.reverse)
                data[key] = value
                path.write_text(yaml.safe_dump(data))
                with self.subTest(key=key, value=value), self.assertRaises(ValueError):
                    scene_route.load_scene(path, 3)
        forward = yaml.safe_load((ROOT / 'waypoints/waypoints_sim.yaml').read_text())
        forward['parameters']['route_direction'] = 'reverse'
        with self.assertRaisesRegex(ValueError, 'Unknown scene 1'):
            scene_route.validate_sim(forward)
        with self.assertRaisesRegex(ValueError, 'Unsupported scene'):
            scene_route.load_scene(self.path, 4)


class EntryContract(unittest.TestCase):
    def quiet_main(self, args):
        with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
            return maze_entry.main(args)

    def test_dry_run_and_unvalidated_full_route_never_launch(self):
        with (
            tempfile.TemporaryDirectory() as directory,
            patch.object(maze_entry, 'run_owned') as run,
        ):
            path = Path(directory) / 'restricted.yaml'
            data = yaml.safe_load((ROOT / 'waypoints/waypoints_real.yaml').read_text())
            data['test_until'] = 'P12'
            path.write_text(yaml.safe_dump(data))
            self.assertEqual(self.quiet_main(['2', '--dry-run']), 0)
            self.assertEqual(self.quiet_main(['2', '--waypoints', str(path)]), 2)
            run.assert_not_called()

    def test_development_full_route_manifest_and_slow_turn_pass_through(self):
        def fake_backend(command):
            manifest = Path(command[command.index('--route-json') + 1])
            payload = json.loads(manifest.read_text())
            expected = yaml.safe_load((ROOT / 'waypoints/waypoints_real.yaml').read_text())
            self.assertEqual(payload['actions'], expected['actions'])
            self.assertEqual(len(payload['actions']), 20)
            self.assertEqual(payload['actions'][-1]['name'], 'P15')
            self.assertEqual(float(command[command.index('--turn-speed') + 1]), 0.20)
            self.assertIn('--laser-log-only', command)
            return 0

        with (
            patch.object(maze_entry, 'executable', return_value='/fake/backend'),
            patch.object(maze_entry, 'run_owned', side_effect=fake_backend) as run,
        ):
            self.assertEqual(self.quiet_main(['2', '--turn-speed', '0.20']), 0)
            run.assert_called_once()

    def test_scene2_custom_yaml_used_failure_propagated_and_temp_removed(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'custom.yaml'
            data = yaml.safe_load((ROOT / 'waypoints/waypoints_real.yaml').read_text())
            data['actions'][0]['points'][1][1] += 0.001
            path.write_text(yaml.safe_dump(data))
            saved = []

            def fake_backend(command):
                manifest = Path(command[command.index('--route-json') + 1])
                saved.append(manifest)
                payload = json.loads(manifest.read_text())
                self.assertEqual(
                    payload['actions'][0]['points'][1], data['actions'][0]['points'][1]
                )
                self.assertEqual(payload['actions'][-1]['name'], 'P02')
                self.assertIn('--laser-log-only', command)
                return 7

            with (
                patch.object(maze_entry, 'executable', return_value='/fake/backend'),
                patch.object(maze_entry, 'run_owned', side_effect=fake_backend),
            ):
                result = self.quiet_main(['2', '--waypoints', str(path), '--destination', 'P02'])
            self.assertEqual(result, 7)
            self.assertFalse(saved[0].exists())

    def test_scene1_loads_yaml_and_preserves_ros_override_order(self):
        captured = []

        def fake_backend(command):
            path = Path(command[command.index('--params-file') + 1])
            captured.append(yaml.safe_load(path.read_text()))
            self.assertEqual(command[-2:], ['-p', 'last_point:=2'])
            return 0

        with (
            patch.object(maze_entry, 'executable', return_value='/fake/legacy'),
            patch.object(maze_entry, 'run_owned', side_effect=fake_backend),
        ):
            result = self.quiet_main(['1', '--ros-args', '-p', 'last_point:=2'])
        self.assertEqual(result, 0)
        self.assertEqual(
            captured[0],
            {
                'pid_maze_solver': {
                    'ros__parameters': scene_route.load_scene(
                        ROOT / 'waypoints/waypoints_sim.yaml', 1
                    )['parameters']
                }
            },
        )

    def test_scene2_unchecked_ros_overrides_rejected(self):
        with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
            maze_entry.parse_args(['2', '--ros-args', '-p', 'use_sim_time:=true'])

    def test_supervised_explicit_guard_mode_and_scene_boundary(self):
        args, _ = maze_entry.parse_args(['2', '--supervised', '--destination', 'P12'])
        with patch.object(maze_entry, 'executable', return_value='/fake/runner'):
            command = maze_entry.backend_command(args, Path('/tmp/route.json'), [])
        self.assertIn('--supervised', command)
        self.assertNotIn('--laser-log-only', command)
        with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
            maze_entry.parse_args(['1', '--supervised'])

    def test_scene3_default_dry_run_and_scene4_rejected_without_launch(self):
        self.assertEqual(maze_entry.default_file(3).name, 'reverse_waypoints_sim.yaml')
        with patch.object(maze_entry, 'run_owned') as run:
            self.assertEqual(self.quiet_main(['3', '--dry-run']), 0)
            with self.assertRaises(SystemExit):
                self.quiet_main(['4', '--dry-run'])
            run.assert_not_called()
        with self.assertRaisesRegex(ValueError, 'Unsupported scene'):
            maze_entry.default_file(4)
        args, _ = maze_entry.parse_args(['3'])
        args.scene = 4
        with patch.object(maze_entry, 'executable') as executable:
            with self.assertRaisesRegex(ValueError, 'Unsupported scene'):
                maze_entry.backend_command(args, Path('/unused'), [])
            executable.assert_not_called()

    def test_scene3_custom_yaml_cpp_backend_sim_clock_and_failure(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'reverse.yaml'
            data = yaml.safe_load((ROOT / 'waypoints/reverse_waypoints_sim.yaml').read_text())
            data['parameters']['steps']['P15_P14']['max_speed'] = 0.11
            path.write_text(yaml.safe_dump(data))
            saved = []

            def fake_backend(command):
                manifest = Path(command[command.index('--params-file') + 1])
                saved.append(manifest)
                params = yaml.safe_load(manifest.read_text())['pid_maze_solver']['ros__parameters']
                self.assertEqual(params, data['parameters'])
                self.assertEqual(command[-2:], ['-p', 'use_sim_time:=true'])
                self.assertNotIn('--route-json', command)
                return 7

            with (
                patch.object(maze_entry, 'executable', return_value='/fake/cpp') as executable,
                patch.object(maze_entry, 'run_owned', side_effect=fake_backend),
            ):
                result = self.quiet_main(['3', '--route-file', str(path)])
                executable.assert_called_once_with('pid_maze_solver', 'pid_maze_solver_task5')
            self.assertEqual(result, 7)
            self.assertFalse(saved[0].exists())

    def test_scene3_rejects_unchecked_overrides_and_real_only_options(self):
        for flags in (
            ['--ros-args', '-p', 'route_direction:=forward'],
            ['--ros-args', '-p', 'use_sim_time:=false'],
            ['--supervised'],
            ['--destination', 'P12'],
            ['--speed', '0.03'],
            ['--turn-speed', '0.10'],
            ['--output', '/tmp/unused'],
        ):
            with self.subTest(flags=flags), contextlib.redirect_stderr(io.StringIO()):
                with self.assertRaises(SystemExit):
                    maze_entry.parse_args(['3', *flags])
        args, _ = maze_entry.parse_args(['3'])
        with self.assertRaisesRegex(ValueError, 'unchecked ROS'):
            maze_entry.backend_command(args, Path('/unused'), ['--ros-args'])

    def test_fake_process_exit_and_cancellation(self):
        self.assertEqual(maze_entry.run_owned([sys.executable, '-c', 'raise SystemExit(7)']), 7)
        timer = threading.Timer(0.2, os.kill, args=(os.getpid(), signal.SIGINT))
        timer.start()
        try:
            result = maze_entry.run_owned(
                [
                    sys.executable,
                    '-c',
                    'import signal,time,sys; signal.signal(signal.SIGINT,lambda *_:sys.exit(0)); time.sleep(5)',
                ]
            )
        finally:
            timer.cancel()
        self.assertEqual(result, 130)


if __name__ == '__main__':
    unittest.main()
