"""Stopped evidence startup recovery; real Probe validation with synthetic clocks."""

from pathlib import Path
import sys
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
import task6_ops_runtime as runtime
from task6_preflight import ODOM, SCAN
import test_task6_ops as existing


class StartupTest(unittest.TestCase):
    def setUp(self):
        self.now = 10.1
        self.probe = existing.FeedbackTest().diagnostic_probe()
        self.gids = {topic: ['original'] for topic in ODOM}
        self.operation = runtime.Operation(
            Path('/tmp/unused-startup'),
            dict(support='/tmp', max_seconds=30, deadline_unix=1000),
            self.probe,
            self.gids,
        )
        self.operation.started = self.now
        self.wait = runtime.StartupFeedbackWait()

    def observe(self, stamp, fresh, elapsed=0):
        return self.wait.ready(
            dict(feedback_fresh=fresh, samples={t: dict(stamp=stamp) for t in (*ODOM, SCAN)}),
            elapsed,
        )

    def test_three_new_fresh_stamps_after_outage(self):
        self.assertFalse(self.observe(100, False))
        self.assertFalse(self.observe(101, True, 0.1))
        self.assertFalse(self.observe(102, True, 0.2))
        self.assertFalse(self.observe(103, True, 0.3))
        self.assertTrue(self.observe(104, True, 0.5))

    def test_duplicates_never_complete_recovery(self):
        self.assertFalse(self.observe(100, False))
        for elapsed in (0.1, 0.4, 1.0, 1.9):
            self.assertFalse(self.observe(101, True, elapsed))
        with self.assertRaisesRegex(RuntimeError, '5 seconds'):
            self.observe(102, True, 5.0)

    def test_bad_frames_reset_count_not_deadline(self):
        self.observe(100, False)
        self.observe(101, True, 0.1)
        self.observe(102, True, 0.2)
        self.assertFalse(self.observe(103, False, 1.8))
        self.assertFalse(self.observe(104, True, 1.9))
        with self.assertRaisesRegex(RuntimeError, '5 seconds'):
            self.observe(105, False, 5.0)

    def test_real_probe_only_scan_age_is_waitable(self):
        for problem in ('gid', 'publisher', 'odom', 'motion', 'tf', 'latched', 'future', 'ranges'):
            with self.subTest(problem=problem):
                self.setUp()
                self.probe.samples[SCAN]['stamp'] = 99
                if problem == 'gid':
                    self.probe.identities = lambda _: ['replacement']
                elif problem == 'publisher':
                    self.probe.endpoints = lambda _: [dict(gid='foreign')]
                elif problem == 'odom':
                    self.probe.samples[ODOM[0]]['stamp'] = 99
                elif problem == 'motion':
                    self.probe.samples[ODOM[1]]['velocity'][0] = 0.02
                elif problem == 'tf':
                    self.probe.buffer.lookup_transform.side_effect = RuntimeError('missing TF')
                elif problem == 'latched':
                    self.probe.failure = 'Timestamp moved backward'
                elif problem == 'future':
                    self.probe.samples[SCAN]['stamp'] = 101
                else:
                    self.probe.samples[SCAN]['valid_ranges'] = 0
                with patch('task6_preflight.time.monotonic', return_value=self.now):
                    with self.assertRaises(RuntimeError):
                        self.probe.snapshot(expected=self.gids, stopped_scan_wait=True)

    def test_active_and_final_admission_remain_strict(self):
        self.probe.samples[SCAN]['stamp'] = 99
        with patch('task6_preflight.time.monotonic', return_value=self.now):
            self.assertFalse(
                self.probe.snapshot(expected=self.gids, stopped_scan_wait=True)['scan_fresh']
            )
            for stopped in (True, False):
                with self.assertRaisesRegex(RuntimeError, 'source timestamp'):
                    self.probe.snapshot(stopped=stopped, expected=self.gids)

    def test_stale_odom_waits_only_in_stopped_startup_scope(self):
        self.probe.samples[ODOM[0]]['stamp'] = 99
        with patch('task6_preflight.time.monotonic', return_value=self.now):
            current = self.probe.snapshot(expected=self.gids, stopped_feedback_wait=True)
            self.assertFalse(current['feedback_fresh'])
            with self.assertRaisesRegex(RuntimeError, 'source timestamp'):
                self.probe.snapshot(expected=self.gids)
            with self.assertRaises(ValueError):
                self.probe.snapshot(stopped=False, stopped_feedback_wait=True)

    def test_startup_wait_still_rejects_identity_motion_future_frame_and_controller(self):
        for problem in ('gid', 'publisher', 'motion', 'future', 'frame', 'latched'):
            with self.subTest(problem=problem):
                self.setUp()
                self.probe.samples[ODOM[0]]['stamp'] = 99
                if problem == 'gid':
                    self.probe.identities = lambda _: ['replacement']
                elif problem == 'publisher':
                    self.probe.endpoints = lambda _: [dict(gid='foreign')]
                elif problem == 'motion':
                    self.probe.samples[ODOM[1]]['velocity'][0] = 0.02
                elif problem == 'future':
                    self.probe.samples[ODOM[1]]['stamp'] = 101
                elif problem == 'frame':
                    self.probe.samples[ODOM[1]]['frame'] = 'new_odom'
                else:
                    self.probe.failure = 'Timestamp moved backward'
                with patch('task6_preflight.time.monotonic', return_value=self.now):
                    with self.assertRaises(RuntimeError):
                        self.probe.snapshot(expected=self.gids, stopped_feedback_wait=True)

    def run_startup(self, callback, health=True):
        def pump():
            self.now += 0.1
            callback()

        self.probe.pump = pump
        self.probe.snapshot = Mock(
            side_effect=lambda **kw: dict(
                samples={t: dict(stamp=self.now) for t in (*ODOM, SCAN)},
                feedback_fresh=self.now >= 10.4,
            )
        )
        child = Mock()
        child.poll.return_value = None
        with (
            patch('task6_ops_runtime.time.monotonic', side_effect=lambda: self.now),
            patch('task6_ops_runtime.time.time', return_value=100),
            patch('task6_ops_runtime.spawn', return_value=child),
            patch('task6_ops_runtime.recorder_health') as recorder,
        ):
            if health:
                recorder.side_effect = lambda *_: (
                    Path('/tmp/session'),
                    dict(unix_time=self.now, message_counts={}),
                )
            else:
                recorder.side_effect = FileNotFoundError()
            self.operation.start_evidence()

    def test_startup_waits_before_growth_admission_and_strict_rechecks(self):
        self.run_startup(lambda: None)
        self.assertGreaterEqual(self.now, 10.6)
        self.assertEqual(
            self.probe.snapshot.call_args.kwargs,
            dict(stopped=True, expected=self.gids, stopped_feedback_wait=True),
        )
        self.assertIsNone(self.operation.runner)

    def test_cancel_during_wait_is_immediate(self):
        with self.assertRaisesRegex(RuntimeError, 'Cancelled'):
            self.run_startup(lambda: self.operation.cancel())
        self.assertLess(self.now, 10.3)

    def test_non_scan_startup_faults_propagate_without_wait(self):
        for reason in (
            'Odometry identity changed',
            'Unexpected publisher',
            'Odom stale',
            'Missing TF',
        ):
            with self.subTest(reason=reason):
                self.setUp()
                with self.assertRaisesRegex(RuntimeError, reason):
                    self.run_startup(
                        lambda: setattr(self.probe.snapshot, 'side_effect', RuntimeError(reason))
                    )
                self.assertLess(self.now, 10.3)

    def test_original_eighteen_second_startup_bound(self):
        with self.assertRaisesRegex(RuntimeError, 'Recorder did not prove'):
            self.run_startup(lambda: None, health=False)
        self.assertLess(self.now, 28.3)

    def test_final_pre_spawn_check_prevents_dispatch(self):
        self.operation.start_evidence = Mock()
        self.operation.check_limits = Mock()
        self.operation.require_evidence_alive = Mock()
        self.probe.pump = Mock()
        self.operation.config.update(entry='/unused', destination='P12', waypoints='/unused')
        self.probe.snapshot = Mock(side_effect=RuntimeError('source stale before spawn'))
        with patch('task6_ops_runtime.write_json'), patch('task6_ops_runtime.spawn') as spawn:
            with self.assertRaisesRegex(RuntimeError, 'before spawn'):
                self.operation.execute()
            spawn.assert_not_called()


if __name__ == '__main__':
    unittest.main()
