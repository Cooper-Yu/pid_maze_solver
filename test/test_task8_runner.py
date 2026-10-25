"""No-motion scene4 contracts and owned fake-console execution tests."""

import copy
import json
import math
from pathlib import Path
import sys
import types
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools'))
from ament_index_python.packages import get_package_prefix  # noqa: E402

sys.path.insert(0, str(Path(get_package_prefix('distance_controller')) / 'lib/distance_controller'))
import maze_entry  # noqa: E402
import scene_route  # noqa: E402
import task8_route_model as model  # noqa: E402
import task8_runner  # noqa: E402
from action_plan import Pose  # noqa: E402


class Contracts(unittest.TestCase):
    def setUp(self):
        self.route = scene_route.load_scene(ROOT / 'waypoints/reverse_waypoints_real.yaml', 4)

    def test_complete_and_prefix(self):
        self.assertEqual(len(self.route['actions']), 21)
        self.assertEqual(model.select_reverse(self.route, 'P13')['actions'][-1]['name'], 'P13')
        self.assertEqual(
            model.select_reverse(self.route, 'P01')['actions'][-1]['name'], 'P01_after_turn'
        )
        self.assertEqual(len(model.validate_manifest(model.select_reverse(self.route))), 21)

    def test_invalid_geometry_and_terminal(self):
        for edit in (
            lambda r: r['actions'][0]['points'][0].__setitem__(0, 0.01),
            lambda r: r['actions'][0]['points'][1].__setitem__(0, float('nan')),
            lambda r: r['actions'][0].__setitem__('heading_rad', 0.1),
            lambda r: r['actions'].reverse(),
            lambda r: r['actions'].pop(),
            lambda r: r['actions'][-1].__setitem__('heading_rad', 0),
            lambda r: r.__setitem__('validated', 'true'),
        ):
            bad = copy.deepcopy(self.route)
            edit(bad)
            with self.subTest(edit=edit), self.assertRaises(ValueError):
                model.validate_reverse(bad)

    def test_payload_rejects_scene2_unknown_reordered_and_nonprefix(self):
        payload = model.select_reverse(self.route, 'P13')
        for edit in (
            lambda r: r.__setitem__('scene', 2),
            lambda r: r.__setitem__('frame', 'P01'),
            lambda r: r.__setitem__('extra', True),
            lambda r: r['actions'].pop(0),
            lambda r: r['actions'].reverse(),
        ):
            bad = copy.deepcopy(payload)
            edit(bad)
            with self.subTest(edit=edit), self.assertRaises(ValueError):
                model.validate_manifest(bad)

    def test_permission_and_cli_source(self):
        self.route['test_until'] = 'P13'
        with self.assertRaises(ValueError):
            model.select_reverse(self.route, 'P01', True)
        self.assertEqual(maze_entry.parse_args(['4'])[0].p15_start, 'localized')
        args, _ = maze_entry.parse_args(['4', '--source-book', '/book', '--source-bag', '/bag'])
        with patch.object(maze_entry, 'executable', return_value='/runner'):
            command = maze_entry.backend_command(args, Path('/route'), [])
        self.assertIn('--source-book', command)
        self.assertIn('--source-bag', command)
        self.assertIn('--forward-route', command)
        self.assertNotIn('--supervised', command)
        self.assertEqual(args.speed, 0.06)
        for extra in (
            ['--supervised'],
            ['--corner-placement', 'front'],
            ['--ros-args', '-p', 'x:=1'],
        ):
            with self.assertRaises(SystemExit):
                maze_entry.parse_args(
                    ['4', '--source-book', '/book', '--source-bag', '/bag', *extra]
                )

    def test_dry_run_default_prefix_no_launch(self):
        with patch.object(maze_entry, 'run_owned') as run, patch('builtins.print') as output:
            self.assertEqual(
                maze_entry.main(
                    ['4', '--source-book', '/book', '--source-bag', '/bag', '--dry-run']
                ),
                0,
            )
        run.assert_not_called()
        data = json.loads(output.call_args.args[0])
        self.assertEqual(data['route']['actions'][-1]['name'], 'P01_after_turn')

    def test_missing_initializer_fails_before_ros(self):
        base = [
            '--route-json',
            '/route',
            '--source-book',
            '/book',
            '--source-bag',
            '/bag',
            '--forward-route',
            '/forward',
        ]
        with (
            patch.object(task8_runner, 'backend_imports'),
            patch.dict(sys.modules, {'task8_p15_start': None}),
            patch('wall_runtime.WallRunner') as runner,
        ):
            self.assertEqual(task8_runner.main(base), 2)
        runner.assert_not_called()

    def test_session_preserves_origin_and_cleanup_and_prefix_status(self):
        import io

        actions = model.validate_manifest(model.select_reverse(self.route, 'P13'))
        origin = Pose(1, 2, 0.3)
        records = []
        discovery_order = []
        fake_runner = types.SimpleNamespace(
            exclusive=lambda: discovery_order.append('discovered'),
            frame='odom',
            stop_translation=lambda: None,
            stop_owned=lambda: None,
            close=lambda _: None,
            request_cancel=lambda: None,
        )
        fake_pause = types.SimpleNamespace(check=lambda: None)

        def bind_feedback(*_args):
            self.assertEqual(discovery_order[-1:], ['discovered'])
            discovery_order.append('bound')
            return fake_pause

        class Book:
            def __init__(self, path, value, frame):
                self.origin = value

        class Console:
            def __init__(self, runner, book, cancellation, audit, speed):
                self.book = book

            def record(self, name):
                records.append((name, self.book.origin))

        args = types.SimpleNamespace(
            speed=0.06,
            turn_speed=0.2,
            max_seconds=5,
            source_book=Path('/book'),
            source_bag=Path('/bag'),
            endpoint=(1, 2),
        )
        audit = io.StringIO()
        with (
            patch('wall_runtime.WallRunner', return_value=fake_runner),
            patch('route_feedback_pause.RouteFeedbackPause', side_effect=bind_feedback),
            patch('motion_console.TeachingBook', Book),
            patch('motion_console.Console', Console),
            patch.object(task8_runner, 'run_actions'),
            patch('task6_route_runner.close_runner') as close,
        ):
            task8_runner.session(args, Path('/out'), audit, actions, lambda *a: origin)
        self.assertEqual(records, [('P15', origin)])
        close.assert_called_once_with(fake_runner)
        events = [json.loads(line) for line in audit.getvalue().splitlines()]
        self.assertEqual(events[-1]['event'], 'prefix_completed')
        self.assertFalse(events[-1]['full_route_success'])
        audit = io.StringIO()
        with (
            patch('wall_runtime.WallRunner', return_value=fake_runner),
            patch('route_feedback_pause.RouteFeedbackPause', side_effect=bind_feedback),
            patch('task6_route_runner.close_runner') as close,
        ):
            with self.assertRaisesRegex(RuntimeError, 'init failure'):
                task8_runner.session(
                    args,
                    Path('/out'),
                    audit,
                    actions,
                    lambda *a: (_ for _ in ()).throw(RuntimeError('init failure')),
                )
        close.assert_called_once_with(fake_runner)

    def test_runner_bounds(self):
        base = [
            '--route-json',
            '/route',
            '--source-book',
            '/book',
            '--source-bag',
            '/bag',
            '--forward-route',
            '/forward',
        ]
        for extra in (['--speed', 'nan'], ['--turn-speed', 'inf'], ['--max-seconds', '601']):
            with self.assertRaises(SystemExit):
                task8_runner.arguments(base + extra)

    def test_actions_reconnect_first_and_after_turn_and_signed_terminal(self):
        actions = model.validate_manifest(model.select_reverse(self.route))
        moves, records, events = [], [], []
        pause = types.SimpleNamespace(check=lambda: None)
        runner = types.SimpleNamespace(
            feedback_pause=pause,
            pose=lambda: Pose(0, 0, 0),
            moving_pose=lambda: Pose(0, 0, 0),
            geometry=lambda: ([], 0),
        )
        console = types.SimpleNamespace(
            runner=runner,
            book=types.SimpleNamespace(origin=Pose(0, 0, 0)),
            command_guard=None,
            emit=lambda *a, **k: events.append((a, k)),
            move=moves.append,
            record=records.append,
        )
        cancellation = types.SimpleNamespace(check=lambda: None)
        with patch('task6_route_model.reconnect_route', side_effect=lambda m, o, p: m) as reconnect:
            task8_runner.run_actions(console, actions, cancellation)
        self.assertEqual(reconnect.call_count, 7)
        self.assertEqual(moves[-1].kind, 'turn')
        self.assertEqual(moves[-1].angle, math.pi)
        self.assertEqual(records[-1], 'P01_after_turn')
        self.assertIsNone(console.command_guard)

    def test_cancel_prevents_move(self):
        cancellation = types.SimpleNamespace(
            check=lambda: (_ for _ in ()).throw(KeyboardInterrupt())
        )
        console = types.SimpleNamespace(move=unittest.mock.Mock())
        with self.assertRaises(KeyboardInterrupt):
            task8_runner.run_actions(console, (), cancellation)
        # An empty action set is rejected by manifest; cancellation also gates any valid first action.
        actions = model.validate_manifest(model.select_reverse(self.route, 'P13'))
        with self.assertRaises(KeyboardInterrupt):
            task8_runner.run_actions(console, actions, cancellation)
        console.move.assert_not_called()


if __name__ == '__main__':
    unittest.main()
