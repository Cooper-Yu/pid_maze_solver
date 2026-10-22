"""Operations admission, live evidence, provenance and cleanup boundaries without hardware."""

from datetime import datetime, timezone
import json
from pathlib import Path
import sys
import tempfile
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
import task6_ops as ops
from task6_ops_runtime import EVIDENCE_RATE, EVIDENCE_ONCE, Operation, recorder_health, write_json
from task6_preflight import ODOM, SCAN, Probe, validate_feedback


class AdmissionTest(unittest.TestCase):
    def test_full_p15_success_requires_terminal_completion(self):
        with tempfile.TemporaryDirectory() as folder:
            operation = Operation.__new__(Operation)
            operation.directory = Path(folder)
            operation.config = dict(entry='fake', destination='P15', waypoints='fake.yaml')
            operation.start_evidence = Mock()
            operation.check_limits = Mock()
            operation.probe = Mock()
            operation.identities = {}
            child = Mock(returncode=0)
            child.poll.return_value = 0
            complete = dict(
                event='route_completed', destination='P15', final_action='P15_after_turn'
            )
            terminal = dict(event='route_action_completed', name='P15_after_turn')
            cases = (
                ([dict(event='route_completed', destination='P15')], False),
                ([complete], False),
                ([dict(terminal, name='P15'), complete], False),
                ([terminal, complete], True),
                ([dict(event='route_completed', destination='P12')], True),
            )
            for rows, valid in cases:
                operation.config['destination'] = rows[-1]['destination']
                (operation.directory / 'points.jsonl').write_text(
                    '\n'.join(json.dumps(row) for row in rows)
                )
                with self.subTest(rows=rows), patch('task6_ops_runtime.spawn', return_value=child):
                    if valid:
                        operation.execute()
                    else:
                        with self.assertRaisesRegex(RuntimeError, 'terminal evidence'):
                            operation.execute()

    def test_explicit_run_location_and_time_bounds(self):
        for argv in (
            ['run', '--run-dir', '/tmp/x'],
            ['run', '--run-dir', '/tmp/x', '--at-p01', '--speed', 'nan'],
            ['run', '--run-dir', '/tmp/x', '--at-p01', '--turn-speed', '0.61'],
            ['prepare', '--run-dir', '/tmp/x'],
            ['prepare', '--run-dir', '/tmp/x', '--deadline', '2026-10-22T14:00:00'],
            ['run', '--run-dir', '/tmp/x', '--at-p01', '--max-seconds', '1800'],
        ):
            with self.subTest(argv=argv), self.assertRaises(SystemExit):
                ops.parse_args(argv)
        self.assertEqual(
            ops.parse_args(['run', '--run-dir', '/tmp/x', '--at-p01']).destination, 'P12'
        )
        self.assertEqual(
            ops.deadline('2026-10-22T14:00:00+08:00'),
            datetime(2026, 10, 22, 6, tzinfo=timezone.utc).timestamp(),
        )

    def test_prepare_is_static_and_gate_remains_p12(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            source = Path(__file__).resolve().parents[1]
            entry = root / 'pid_maze_solver'
            entry.write_text('fake')
            entry.chmod(0o700)
            support = root / 'tools/task6_ops'
            support.mkdir(parents=True)
            for name in ('common.py', 'evidence_recorder.py', 'odom_reset_watch.py'):
                (support / name).write_text('# readonly')
            packages = {
                'pid_maze_solver': dict(source=str(source), share=str(source), library=str(root)),
                'distance_controller': dict(source=str(root)),
            }
            args = SimpleNamespace(
                workspace=root,
                run_dir=root / 'run',
                destination='P12',
                deadline=time.time() + 1000,
                max_seconds=300,
                speed=0.12,
                turn_speed=0.20,
            )
            with (
                patch.object(ops, 'provenance', return_value=packages),
                patch.object(ops, 'Probe') as probe,
                patch.object(ops, 'Operation') as operation,
            ):
                self.assertEqual(ops.prepare(args), 0)
                probe.assert_not_called()
                operation.assert_not_called()
                self.assertEqual(
                    json.loads((args.run_dir / 'route_preview.json').read_text())['actions'][-1][
                        'name'
                    ],
                    'P12',
                )
                args.run_dir = root / 'bad'
                args.destination = 'INVALID_DESTINATION'
                with self.assertRaisesRegex(ValueError, 'Unknown destination'):
                    ops.prepare(args)

    def test_used_session_and_dirty_release_cannot_run(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            args = SimpleNamespace(run_dir=root)
            config = dict(packages={'repo': dict(tracked_changes=' M controller')})
            with (
                patch.object(ops, 'verify_config', return_value=config),
                patch.object(ops, 'Probe') as probe,
            ):
                with self.assertRaisesRegex(RuntimeError, 'Commit reviewed'):
                    ops.run(args)
                (root / 'run.started.json').write_text('{}')
                with self.assertRaisesRegex(RuntimeError, 'already used'):
                    ops.run(args)
                probe.assert_not_called()

    def test_installed_python_mismatch_rejected(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            source = root / 'source'
            library = root / 'lib'
            (source / 'tools').mkdir(parents=True)
            library.mkdir()
            (source / 'tools/maze_entry.py').write_text('source')
            (library / 'pid_maze_solver').write_text('stale')
            with (
                patch.object(ops, 'package_paths', return_value=(source, library, root)),
                patch.object(ops, 'git', return_value=''),
            ):
                with self.assertRaisesRegex(RuntimeError, 'Installed Python differs'):
                    ops.provenance(root)


class FeedbackTest(unittest.TestCase):
    def samples(self):
        samples = {
            t: dict(receipt=10, stamp=100, frame='odom', child='base_link', velocity=[0, 0, 0])
            for t in ODOM
        }
        samples[SCAN] = dict(receipt=10, stamp=100, frame='laser', valid_ranges=30)
        return samples

    def test_fresh_stopped_samples_required(self):
        samples = self.samples()
        validate_feedback(samples, 100.1, 10.1, True)
        for stamp, receipt in ((101, 10), (99, 10), (100, 9)):
            bad = self.samples()
            bad[ODOM[0]].update(stamp=stamp, receipt=receipt)
            with self.assertRaises(RuntimeError):
                validate_feedback(bad, 100.1, 10.1, True)
        samples[ODOM[0]]['velocity'] = [0.02, 0, 0]
        with self.assertRaisesRegex(RuntimeError, 'not stopped'):
            validate_feedback(samples, 100.1, 10.1, True)
        validate_feedback(samples, 100.1, 10.1, False)

    def test_zero_stopped_new_identity_never_confirms_old_epoch(self):
        probe = Probe.__new__(Probe)
        probe.pump = Mock()
        probe.last_command = dict(values=[0.0] * 6)
        expected = {topic: ['original'] for topic in ODOM}
        probe.identities = lambda topic: [] if topic == '/cmd_vel' else ['replacement']
        self.assertFalse(probe.wait_zero(expected, timeout=0.1))
        probe.identities = lambda topic: [] if topic == '/cmd_vel' else ['original']
        probe.samples = self.samples()
        probe.node = Mock()
        probe.node.get_clock.return_value.now.return_value.nanoseconds = 100_100_000_000
        with patch('task6_preflight.time.monotonic', return_value=10.1):
            self.assertTrue(probe.wait_zero(expected, timeout=0.1))

    def diagnostic_probe(self):
        probe = Probe.__new__(Probe)
        probe.samples, probe.counts, probe.failure = self.samples(), {SCAN: 7}, None
        probe.last_command = None
        probe.node = Mock()
        probe.node.get_clock.return_value.now.return_value.nanoseconds = 100_100_000_000
        probe.node.get_publishers_info_by_topic.return_value = []
        probe.identities = lambda _: ['original']
        probe.buffer = Mock()
        transform = probe.buffer.lookup_transform.return_value.transform
        transform.translation = SimpleNamespace(x=0, y=0, z=0)
        transform.rotation = SimpleNamespace(x=0, y=0, z=0, w=1)
        probe.ros = Mock()
        return probe

    def test_publisher_bounds_and_exact_rejected_endpoints(self):
        probe = self.diagnostic_probe()
        endpoints = [
            SimpleNamespace(endpoint_gid=bytes([i]), node_name=f'node{i}', node_namespace='/')
            for i in (1, 2)
        ]
        with patch('task6_preflight.time.monotonic', return_value=10.1):
            for stopped, count, allowed in (
                (True, 0, True),
                (True, 1, False),
                (False, 1, True),
                (False, 2, False),
            ):
                probe.node.get_publishers_info_by_topic.return_value = endpoints[:count]
                with self.subTest(stopped=stopped, count=count):
                    if allowed:
                        self.assertEqual(probe.snapshot(stopped=stopped)['cmd_publishers'], count)
                    else:
                        with self.assertRaisesRegex(RuntimeError, f'count={count}'):
                            probe.snapshot(stopped=stopped)
                        self.assertEqual(probe.last_check['cmd_count'], count)
            # Graph may change before capture: preserve the exact rejected observation too.
            probe.node.get_publishers_info_by_topic.return_value = []
            evidence = probe.diagnostics()
            self.assertEqual(len(evidence['last_check']['cmd_endpoints']), 2)
            self.assertEqual(evidence['last_check']['cmd_endpoints'][1]['gid'], '02')
            self.assertEqual(evidence['cmd_endpoints'], [])
            self.assertAlmostEqual(evidence['topics'][SCAN]['source_age_s'], 0.1)
            self.assertAlmostEqual(evidence['topics'][SCAN]['receipt_age_s'], 0.1)

    def test_stale_source_diagnostic_keeps_failed_sample(self):
        probe = self.diagnostic_probe()
        probe.samples[SCAN]['stamp'] = 99
        with patch('task6_preflight.time.monotonic', return_value=10.1):
            with self.assertRaisesRegex(RuntimeError, 'source timestamp'):
                probe.snapshot(stopped=False)
            self.assertAlmostEqual(probe.diagnostics()['topics'][SCAN]['source_age_s'], 1.1)

    def test_recorder_must_grow_and_remain_fresh(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            session = root / 'session'
            session.mkdir()
            (root / 'latest_session.txt').write_text(str(session))
            counts = {t: 10 for t in (*EVIDENCE_RATE, *EVIDENCE_ONCE)}
            write_json(session / 'health.json', dict(unix_time=time.time(), message_counts=counts))
            recorder_health(root, {t: 9 for t in counts})
            with self.assertRaisesRegex(RuntimeError, 'did not grow'):
                recorder_health(root, counts)
            write_json(
                session / 'health.json', dict(unix_time=time.time() - 20, message_counts=counts)
            )
            with self.assertRaisesRegex(RuntimeError, 'stale'):
                recorder_health(root)
            (root / 'latest_session.txt').write_text('/tmp/another-run')
            with self.assertRaisesRegex(RuntimeError, 'escaped'):
                recorder_health(root)


class LifetimeTest(unittest.TestCase):
    def config(self):
        return dict(max_seconds=30, deadline_unix=time.time() + 300, destination='P12')

    def test_deadline_and_cancellation_are_failures(self):
        operation = Operation(Path('/tmp'), self.config(), Mock(), {})
        operation.check_limits()
        operation.cancel()
        with self.assertRaisesRegex(RuntimeError, 'Cancelled'):
            operation.check_limits()
        operation.cancelled = False
        operation.started -= 31
        with self.assertRaisesRegex(RuntimeError, 'Maximum'):
            operation.check_limits()
        operation.started = time.monotonic()
        operation.config['deadline_unix'] = time.time() + 29
        with self.assertRaisesRegex(RuntimeError, 'deadline'):
            operation.check_limits()

    def test_stop_zero_then_recorder_close_and_no_false_success(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            session = root / 'evidence/session'
            (session / 'bag').mkdir(parents=True)
            (root / 'evidence/latest_session.txt').write_text(str(session))
            (session / 'bag/metadata.yaml').write_text('closed')
            write_json(session / 'result.json', dict(closed_metadata=True, exception=None))
            order = []
            probe = Mock()
            probe.wait_zero.side_effect = lambda *_: order.append('zero') or True
            operation = Operation(root, self.config(), probe, {})
            operation.runner = 'motion'
            operation.recorder = 'recorder'
            operation.watch = 'watch'
            with patch(
                'task6_ops_runtime.stop_child',
                side_effect=lambda child, _probe: order.append(child),
            ):
                result = operation.cleanup()
                self.assertEqual(order, ['motion', 'zero', 'recorder', 'watch'])
                self.assertFalse(result['failures'])
                operation.execute = Mock(side_effect=RuntimeError('Cancelled'))
                self.assertEqual(operation.run(), 2)
                self.assertFalse(json.loads((root / 'result.json').read_text())['success'])

    def test_failure_snapshot_precedes_cleanup_and_keeps_first_error(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            probe = Mock()
            probe.diagnostics.return_value = {'cmd_endpoints': ['controller', 'stopper']}
            operation = Operation(root, self.config(), probe, {})
            operation.phase = 'official_runner'
            operation.execute = Mock(side_effect=RuntimeError('original fault'))

            def cleanup():
                probe.diagnostics.assert_called_once()
                self.assertFalse((root / 'failure_snapshot.json').exists())
                probe.diagnostics.return_value = {'cmd_endpoints': []}
                return dict(failures=[], zero_and_released=True)

            operation.cleanup = cleanup
            self.assertEqual(operation.run(), 2)
            saved = json.loads((root / 'failure_snapshot.json').read_text())
            self.assertEqual(saved['error'], 'original fault')
            self.assertEqual(saved['phase'], 'official_runner')
            self.assertEqual(len(saved['probe']['cmd_endpoints']), 2)
            self.assertEqual(
                json.loads((root / 'result.json').read_text())['error'], 'original fault'
            )

    def test_diagnostic_capture_failure_never_skips_cleanup(self):
        with tempfile.TemporaryDirectory() as folder:
            operation = Operation(Path(folder), self.config(), Mock(), {})
            operation.execute = Mock(side_effect=RuntimeError('primary'))
            operation.capture_failure = Mock(side_effect=OSError('disk failure'))
            operation.cleanup = Mock(return_value=dict(failures=[], zero_and_released=True))
            self.assertEqual(operation.run(), 2)
            operation.cleanup.assert_called_once()
            result = json.loads((Path(folder) / 'result.json').read_text())
            self.assertEqual(result['error'], 'primary')
            self.assertEqual(result['failure_snapshot_error'], 'disk failure')

    def test_diagnostic_write_failure_preserves_primary_result(self):
        with tempfile.TemporaryDirectory() as folder:
            operation = Operation(Path(folder), self.config(), Mock(), {})
            operation.execute = Mock(side_effect=RuntimeError('primary'))
            operation.probe.diagnostics.return_value = {}
            operation.cleanup = Mock(return_value=dict(failures=[], zero_and_released=True))

            def write(path, value):
                operation.cleanup.assert_called_once()
                if path.name == 'failure_snapshot.json':
                    raise OSError('snapshot disk failure')
                write_json(path, value)

            with patch('task6_ops_runtime.write_json', side_effect=write):
                self.assertEqual(operation.run(), 2)
            result = json.loads((Path(folder) / 'result.json').read_text())
            self.assertEqual(result['error'], 'primary')
            self.assertEqual(result['failure_snapshot_error'], 'snapshot disk failure')

    def test_missing_final_zero_or_metadata_fails_cleanup(self):
        with tempfile.TemporaryDirectory() as folder:
            operation = Operation(Path(folder), self.config(), Mock(), {})
            operation.probe.wait_zero.return_value = False
            operation.runner = Mock()
            operation.runner.poll.return_value = 0
            result = operation.cleanup()
            self.assertFalse(result['zero_and_released'])
            self.assertTrue(result['failures'])


if __name__ == '__main__':
    unittest.main()
