"""Bounded ready-callback processing without waiting out stale feedback."""

from collections import deque
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from task6_preflight import PUMP_DRAIN_MAX_CALLS, Probe, SCAN, validate_feedback, ODOM


class PumpTest(unittest.TestCase):
    def setUp(self):
        self.now = 10.0
        self.calls = []
        self.pending = deque()
        self.probe = Probe.__new__(Probe)
        self.probe.counts = {}
        self.probe.command_count = 0
        self.probe.samples = {}
        self.probe.failure = None
        self.probe.node = object()
        self.cost = 0.0001
        self.probe.ros = SimpleNamespace(spin_once=self.spin)

    def spin(self, _node, timeout_sec):
        self.calls.append(timeout_sec)
        self.now += self.cost
        if self.pending:
            self.pending.popleft()()

    def pump(self):
        with patch('task6_preflight.time.monotonic', side_effect=lambda: self.now):
            self.probe.pump()

    def test_ready_scan_after_odom_tf_or_cmd_is_processed(self):
        for first in ('odom', 'tf', 'cmd'):
            with self.subTest(first=first):
                self.setUp()
                seen = []
                self.pending.extend((lambda: seen.append(first), lambda: seen.append('scan')))
                self.pump()
                self.assertEqual(seen, [first, 'scan'])
                self.assertEqual(self.calls[0], 0.05)
                self.assertTrue(all(value == 0 for value in self.calls[1:]))

    def test_continuous_ready_work_stops_at_time_budget(self):
        self.cost = 0.0006
        self.pending.extend([lambda: None] * 100)
        self.pump()
        self.assertEqual(self.probe.last_pump['drain_spin_calls'], 4)
        self.assertLess(self.probe.last_pump['drain_elapsed_s'], 0.0027)
        self.assertGreater(len(self.pending), 0)

    def test_fast_callbacks_stop_at_count_limit(self):
        self.cost = 0
        self.pending.extend([lambda: None] * 100)
        self.pump()
        self.assertEqual(len(self.calls), 1 + PUMP_DRAIN_MAX_CALLS)

    def test_slow_callback_not_preempted_but_no_following_call(self):
        self.cost = 0.01
        self.pending.extend([lambda: None] * 100)
        self.pump()
        self.assertEqual(len(self.calls), 2)
        self.assertAlmostEqual(self.probe.last_pump['drain_elapsed_s'], 0.01)

    def test_no_new_scan_remains_stale_after_pump(self):
        samples = {
            t: dict(receipt=10, stamp=100, frame='odom', child='base_link', velocity=[0, 0, 0])
            for t in ODOM
        }
        samples[SCAN] = dict(receipt=10, stamp=99, frame='laser', valid_ranges=30)
        self.pump()
        with self.assertRaisesRegex(RuntimeError, 'source timestamp'):
            validate_feedback(samples, 100.1, self.now, False)
        self.assertLess(self.now - 10, 0.003)

    def test_monitor_count_and_latched_fault_survive_drain(self):
        self.pending.extend(
            (
                lambda: self.probe.accept(SCAN, dict(stamp=5)),
                lambda: self.probe.accept(SCAN, dict(stamp=4)),
                lambda: self.probe.accept(SCAN, dict(stamp=6)),
            )
        )
        self.pump()
        self.assertEqual(self.probe.last_pump['monitored_callbacks'], 3)
        self.assertIn('backward', self.probe.failure)


if __name__ == '__main__':
    unittest.main()
