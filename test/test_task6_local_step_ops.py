"""Pure tests: no ROS context, publisher or hardware is created."""

from copy import deepcopy
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
import task6_local_step_ops as local
from task6_ops_runtime import Operation
from task6_preflight import ODOM


class LocalStepTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.config = dict(
            entry='/verified/current_frame_step',
            distance=0.03,
            speed=0.01,
            deadline_unix=10**12,
            max_seconds=60,
            artifact_sha256={},
            destination='LOCAL_END',
        )
        self.gids = {ODOM[0]: ['a' * 48], ODOM[1]: ['b' * 48]}
        self.probe = Mock()
        self.op = local.LocalStepOperation(self.root, self.config, self.probe, self.gids)
        self.op.start_evidence = Mock()
        self.op.check_recording = Mock()
        self.args = [
            '--run-dir',
            str(self.root / 'new'),
            '--distance',
            '.03',
            '--deadline',
            '2099-01-01T14:00:00+08:00',
            '--supervised-manual',
        ]

    def events(self):
        shared = dict(historical_p01=False, task6_acceptance=False, distance_m=0.03)
        return [
            dict(
                shared,
                event='local_step_started',
                laser_policy='observation_only',
                supervised_manual=True,
                speed_cap_m_s=0.01,
                gids={topic: self.gids[topic][0] for topic in ODOM},
            ),
            dict(shared, event='local_step_completed'),
        ]

    def write_events(self, rows):
        (self.root / 'local_points.jsonl').write_text('\n'.join(map(json.dumps, rows)))

    def test_argument_bounds_and_explicit_supervision(self):
        for option, values in [
            ('--distance', ['0', '-.01', '.051', 'nan', 'inf']),
            ('--speed', ['.009', '.031', 'nan']),
            ('--max-seconds', ['29', '121']),
        ]:
            for value in values:
                with self.subTest(option=option, value=value), self.assertRaises(SystemExit):
                    local.arguments(self.args + [option, value])
        with self.assertRaises(SystemExit):
            local.arguments(self.args[:-1])
        self.assertEqual(local.arguments(self.args).distance, 0.03)

    def test_command_only_local_step_no_origin_or_force(self):
        command = self.op.build_command()
        self.assertIn('--supervised-manual', command)
        self.assertNotIn('--laser-log-only', command)
        self.assertNotIn('--source', command)
        self.assertNotIn('--at-p01', command)
        self.assertEqual(command[command.index('--raw-odom-publisher-gid') + 1], 'b' * 48)
        self.assertEqual(command[command.index('--distance') + 1], '0.03')

    def test_completion_requires_exact_one_local_terminal(self):
        self.write_events(self.events())
        self.op.verify_completion()
        variants = [
            [],
            self.events()[:1],
            list(reversed(self.events())),
            self.events() + [self.events()[-1]],
            self.events() + [dict(event='local_step_failed')],
            self.events() + [dict(event='route_completed')],
        ]
        for key, value in [
            ('gids', {}),
            ('laser_policy', 'strict_enforced'),
            ('supervised_manual', False),
            ('distance_m', 0.04),
            ('speed_cap_m_s', 0.03),
            ('historical_p01', True),
        ]:
            rows = deepcopy(self.events())
            rows[0][key] = value
            variants.append(rows)
        for rows in variants:
            self.write_events(rows)
            with self.assertRaises(RuntimeError):
                self.op.verify_completion()

    def test_recording_failure_never_spawns(self):
        self.op.check_recording.side_effect = RuntimeError('recorder failed')
        with (
            patch.object(local, 'spawn') as spawn,
            self.assertRaisesRegex(RuntimeError, 'recorder'),
        ):
            self.op.execute()
        spawn.assert_not_called()

    def test_cancel_during_record_check_prevents_spawn(self):
        self.op.check_recording.side_effect = lambda: self.op.cancel()
        with (
            patch.object(local, 'spawn') as spawn,
            self.assertRaisesRegex(RuntimeError, 'Cancelled'),
        ):
            self.op.execute()
        spawn.assert_not_called()

    def test_identity_change_before_start_prevents_spawn(self):
        self.probe.snapshot.side_effect = RuntimeError('Odometry publisher identity/count changed')
        with (
            patch.object(local, 'spawn') as spawn,
            self.assertRaisesRegex(RuntimeError, 'identity'),
        ):
            self.op.execute()
        spawn.assert_not_called()

    def test_artifact_change_prevents_spawn(self):
        self.config['artifact_sha256'] = {str(self.root / 'missing'): '0' * 64}
        with patch.object(local, 'spawn') as spawn, self.assertRaises(OSError):
            self.op.execute()
        spawn.assert_not_called()

    def test_success_launches_once_with_current_dual_identity(self):
        self.write_events(self.events())
        child = Mock(returncode=0)
        child.poll.side_effect = [None, 0]
        with patch.object(local, 'spawn', return_value=child) as spawn:
            self.op.execute()
        spawn.assert_called_once()
        self.probe.snapshot.assert_any_call(stopped=True, expected=self.gids)
        self.probe.snapshot.assert_any_call(stopped=False, expected=self.gids)

    def test_nonzero_backend_exit_is_not_completion(self):
        self.write_events(self.events())
        child = Mock(returncode=2)
        child.poll.return_value = 2
        with (
            patch.object(local, 'spawn', return_value=child),
            self.assertRaisesRegex(RuntimeError, 'exited 2'),
        ):
            self.op.execute()

    def test_no_retry_after_runtime_feedback_failure_cleanup_retained(self):
        child = Mock(returncode=None)
        child.poll.return_value = None
        self.probe.snapshot.side_effect = [dict(), RuntimeError('stale odom')]
        self.op.cleanup = Mock(return_value=dict(failures=[], zero_and_released=True))
        self.op.capture_failure = Mock(return_value=dict(error='stale odom'))
        with patch.object(local, 'spawn', return_value=child) as spawn:
            self.assertEqual(self.op.run(), 2)
        spawn.assert_called_once()
        self.op.cleanup.assert_called_once()
        result = local.read_json(self.root / 'result.json')
        self.assertFalse(result['success'])
        self.assertFalse(result['historical_p01'])
        self.assertFalse(result['route_recovery'])
        self.assertFalse(result['automatic_avoidance'])
        self.assertEqual(result['laser_policy'], 'observation_only')
        self.assertEqual(result['source_mode'], 'LOCAL_START')

    def test_cleanup_implementation_is_shared(self):
        self.assertIs(local.LocalStepOperation.cleanup, Operation.cleanup)
        self.assertIs(local.LocalStepOperation.verify_final_stop, Operation.verify_final_stop)

    def test_dirty_provenance_refuses_before_probe(self):
        args = local.arguments(self.args)
        with (
            patch.object(local, 'provenance', return_value={'x': dict(tracked_changes=' M x')}),
            patch.object(local, 'Probe') as probe,
            self.assertRaisesRegex(RuntimeError, 'Tracked'),
        ):
            local.start(args)
        probe.assert_not_called()

    def test_missing_installed_backend_refuses_before_probe(self):
        args = local.arguments(self.args)
        package = dict(tracked_changes='', installed_sha256={}, library=str(self.root))
        with (
            patch.object(local, 'provenance', return_value={'distance_controller': package}),
            patch.object(local, 'Probe') as probe,
            self.assertRaisesRegex(RuntimeError, 'installed'),
        ):
            local.start(args)
        probe.assert_not_called()


if __name__ == '__main__':
    unittest.main()
