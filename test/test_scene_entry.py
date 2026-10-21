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
