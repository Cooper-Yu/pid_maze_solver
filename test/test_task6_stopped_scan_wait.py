"""Stopped-only scan grace and no-command cleanup evidence; no ROS or hardware."""

from dataclasses import asdict, dataclass
from io import StringIO
from pathlib import Path
import sys
import tempfile
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from task6_preflight import ODOM, SCAN, Probe, validate_feedback
from task6_prefix_return_ops import PrefixReturnOperation


@dataclass
class Pose:
    x: float = 0
    y: float = 0
    yaw: float = 0


def feedback():
    samples = {
        topic: dict(
            stamp=100,
            receipt=10,
            frame='odom',
            child='base_link',
            velocity=[0, 0, 0],
            pose=[0, 0, 0],
        )
        for topic in ODOM
    }
    samples[SCAN] = dict(stamp=100, receipt=10, frame='laser', valid_ranges=30)
    return samples


def probe():
    value = Probe.__new__(Probe)
    value.samples, value.counts = feedback(), {topic: 10 for topic in (*ODOM, SCAN)}
    value.failure = None
    value.last_command = None
    value.command_count = 0
    value.node = Mock()
    value.node.get_clock.return_value.now.return_value.nanoseconds = 100_100_000_000
    value.node.get_publishers_info_by_topic.return_value = []
    value.identities = lambda _: ['epoch']
    value.buffer = Mock()
    transform = value.buffer.lookup_transform.return_value.transform
    transform.translation = SimpleNamespace(x=0, y=0, z=0)
    transform.rotation = SimpleNamespace(x=0, y=0, z=0, w=1)
    value.ros = Mock()
    return value


def operation(root):
    source = SimpleNamespace(
        session_id='session',
        source_hash='hash',
        origin=Pose(),
        actions=[
            SimpleNamespace(name='P02', kind='path'),
            SimpleNamespace(name='P03', kind='path'),
        ],
    )
    value = PrefixReturnOperation(
        root,
        dict(max_seconds=600, deadline_unix=time.time() + 1000),
        probe(),
        {topic: ['epoch'] for topic in ODOM},
        source,
        root / 'points.json',
        root,
    )
    value.runner = Mock()
    value.runner.poll.return_value = None
    value.runner.stdin = StringIO()
    rows = [
        dict(
            event='return_session_started',
            source_session='session',
            source_hash='hash',
            source_origin=asdict(source.origin),
            publisher_gid='epoch',
        )
    ]
    return value, rows


class StoppedWaitTest(unittest.TestCase):
    def test_only_stopped_scan_positive_age_has_grace(self):
        samples = feedback()
        samples[SCAN]['stamp'] = 99
        self.assertFalse(validate_feedback(samples, 100.1, 10.1, True, stopped_scan_wait=True))
        with self.assertRaisesRegex(RuntimeError, 'source timestamp'):
            validate_feedback(samples, 100.1, 10.1, True)
        with self.assertRaisesRegex(RuntimeError, 'source timestamp'):
            validate_feedback(samples, 100.1, 10.1, False)
        with self.assertRaises(ValueError):
            validate_feedback(samples, 100.1, 10.1, False, stopped_scan_wait=True)
        samples[SCAN].update(stamp=100, receipt=9)
        self.assertFalse(validate_feedback(samples, 100.1, 10.1, True, stopped_scan_wait=True))
        with self.assertRaisesRegex(RuntimeError, 'receipt'):
            validate_feedback(samples, 100.1, 10.1, False)

    def test_invalid_future_and_odom_failures_have_no_grace(self):
        for topic, fields in [
            (SCAN, dict(stamp=101)),
            (SCAN, dict(stamp=0)),
            (SCAN, dict(frame='')),
            (SCAN, dict(valid_ranges=0)),
            (ODOM[0], dict(stamp=99)),
            (ODOM[1], dict(receipt=9)),
            (ODOM[0], dict(velocity=[0.02, 0, 0])),
            (ODOM[1], dict(frame='map')),
        ]:
            with self.subTest(topic=topic, fields=fields):
                samples = feedback()
                samples[topic].update(fields)
                with self.assertRaises(RuntimeError):
                    validate_feedback(samples, 100.1, 10.1, True, stopped_scan_wait=True)

    def test_wait_snapshot_never_ignores_epoch_publisher_or_latched_failure(self):
        p = probe()
        p.samples[SCAN]['stamp'] = 99
        with patch('time.monotonic', return_value=10.1):
            self.assertFalse(
                p.snapshot(expected={t: ['epoch'] for t in ODOM}, stopped_scan_wait=True)[
                    'scan_fresh'
                ]
            )
            with self.assertRaisesRegex(RuntimeError, 'identity'):
                p.snapshot(expected={t: ['other'] for t in ODOM}, stopped_scan_wait=True)
            p.node.get_publishers_info_by_topic.return_value = [
                SimpleNamespace(endpoint_gid=b'1', node_name='other', node_namespace='/')
            ]
            with self.assertRaisesRegex(RuntimeError, 'publisher'):
                p.snapshot(stopped_scan_wait=True)
            p.failure = 'Timestamp moved backward'
            with self.assertRaisesRegex(RuntimeError, 'backward'):
                p.snapshot(stopped_scan_wait=True)

    def test_three_new_stamps_required_no_automatic_next(self):
        with tempfile.TemporaryDirectory() as folder, patch('time.monotonic', return_value=10.1):
            op, rows = operation(Path(folder))
            op.probe.samples[SCAN]['stamp'] = 99
            op.observe_feedback(rows)
            op.save_state(rows)
            self.assertTrue(op.state['waiting_for_fresh_scan'])
            self.assertFalse(op.state['ready'])
            op.request(dict(id='blocked', command='next', wall_s=time.time()), rows)
            self.assertEqual(op.runner.stdin.getvalue(), '')
            for index, stamp in enumerate([100, 100, 100.01, 100.02]):
                op.probe.samples[SCAN]['stamp'] = stamp
                op.observe_feedback(rows)
                op.save_state(rows)
                self.assertEqual(op.state['ready'], index == 3)
            self.assertEqual(op.runner.stdin.getvalue(), '')
            op.request(dict(id='go', command='next', wall_s=time.time()), rows)
            self.assertEqual(op.runner.stdin.getvalue(), 'next\n')
            self.assertTrue(op.prefix_audit.requested)
            self.assertTrue(op.next_dispatched)

    def test_scan_gap_at_dispatch_blocks_before_write(self):
        with tempfile.TemporaryDirectory() as folder, patch('time.monotonic', return_value=10.1):
            op, rows = operation(Path(folder))
            op.observe_feedback(rows)
            op.save_state(rows)
            op.probe.samples[SCAN]['stamp'] = 99
            op.request(dict(id='raced', command='next', wall_s=time.time()), rows)
            self.assertFalse(op.state['last_request']['accepted'])
            self.assertFalse(op.next_dispatched)
            self.assertEqual(op.runner.stdin.getvalue(), '')

    def test_active_and_pre_attach_scan_loss_still_fails_immediately(self):
        for attached in (True, False):
            with (
                self.subTest(attached=attached),
                tempfile.TemporaryDirectory() as folder,
                patch('time.monotonic', return_value=10.1),
            ):
                op, rows = operation(Path(folder))
                if attached:
                    op.observe_feedback(rows)
                    op.save_state(rows)
                    op.request(dict(id='go', command='next', wall_s=time.time()), rows)
                op.probe.samples[SCAN]['stamp'] = 99
                with self.assertRaisesRegex(RuntimeError, 'source timestamp'):
                    op.observe_feedback(rows if attached else [])
                self.assertIsNone(op.scan_wait_since)

    def test_completion_is_required_before_wait_after_motion(self):
        with tempfile.TemporaryDirectory() as folder, patch('time.monotonic', return_value=10.1):
            op, rows = operation(Path(folder))
            op.observe_feedback(rows)
            op.save_state(rows)
            op.request(dict(id='go', command='next', wall_s=time.time()), rows)
            rows.append(dict(event='return_action_started', index=1, name='P03', kind='path'))
            op.observe_feedback(rows)
            rows.append(dict(event='return_action_completed', cursor=1, name='P03'))
            op.probe.samples[SCAN]['stamp'] = 99
            op.observe_feedback(rows)
            op.save_state(rows)
            self.assertTrue(op.state['waiting_for_fresh_scan'])
            self.assertFalse(op.busy)

    def test_two_second_deadline_does_not_refresh_and_fault_latches(self):
        with tempfile.TemporaryDirectory() as folder:
            with patch('time.monotonic', return_value=10):
                op, rows = operation(Path(folder))
                op.probe.samples[SCAN]['stamp'] = 99
                op.observe_feedback(rows)
            for now, fresh, stamp in [(10.5, True, 100), (11, False, 100.1), (11.9, True, 101)]:
                with patch('time.monotonic', return_value=now):
                    op.update_scan_wait(dict(scan_fresh=fresh, samples={SCAN: dict(stamp=stamp)}))
                    self.assertEqual(op.scan_wait_since, 10)
            with patch('time.monotonic', return_value=12):
                with self.assertRaisesRegex(RuntimeError, '2 seconds'):
                    op.update_scan_wait(dict(scan_fresh=True, samples={SCAN: dict(stamp=102)}))
            with patch('time.monotonic', return_value=12.1):
                with self.assertRaisesRegex(RuntimeError, '2 seconds'):
                    op.observe_feedback(rows)

    def test_cancellation_failure_and_global_deadline_cannot_resume(self):
        with tempfile.TemporaryDirectory() as folder, patch('time.monotonic', return_value=10.1):
            op, rows = operation(Path(folder))
            op.observe_feedback(rows)
            op.save_state(rows)
            op.cancelled = True
            op.request(dict(id='cancelled', command='next', wall_s=time.time()), rows)
            self.assertEqual(op.runner.stdin.getvalue(), '')
            op.cancelled = False
            op.started -= 601
            with self.assertRaisesRegex(RuntimeError, 'Maximum'):
                op.next_dispatch_reason()
            rows.append(dict(event='return_failed'))
            with self.assertRaisesRegex(RuntimeError, 'failed'):
                op.observe_feedback(rows)
            with self.assertRaises(RuntimeError):
                op.observe_feedback(rows[:1])

    def test_no_command_stop_does_not_claim_observed_zero(self):
        with tempfile.TemporaryDirectory() as folder:
            op, rows = operation(Path(folder))
            op.prefix_audit.consume(rows)
            op.runner.poll.return_value = 2
            op.probe.wait_no_command_stop = Mock(return_value=dict(gids=op.identities))
            result = op.verify_final_stop()
            self.assertFalse(result['zero_and_released'])
            self.assertTrue(result['stopped_and_released'])
            self.assertFalse(result['stop_evidence']['zero_command_observed'])
            op.probe.wait_zero = Mock(return_value=False)
            op.next_dispatched = True
            self.assertFalse(op.verify_final_stop()['stopped_and_released'])
            op.probe.wait_zero.assert_called_once()

    def test_no_command_cleanup_rejects_unexited_child_and_changed_epoch(self):
        with tempfile.TemporaryDirectory() as folder:
            op, rows = operation(Path(folder))
            op.prefix_audit.consume(rows)
            with self.assertRaisesRegex(RuntimeError, 'child exit'):
                op.verify_final_stop()
        p = probe()
        p.pump = Mock()
        with patch('time.monotonic', return_value=10.1):
            with self.assertRaisesRegex(RuntimeError, 'identity'):
                p.wait_no_command_stop({t: ['new'] for t in ODOM})
        p.command_count = 1
        self.assertIsNone(p.wait_no_command_stop({t: ['epoch'] for t in ODOM}))

    def test_no_command_stop_requires_new_dual_samples_and_hold(self):
        p = probe()
        clock = [10.0]
        p.samples[SCAN]['stamp'] = 99

        def pump():
            clock[0] += 0.05
            for topic in ODOM:
                p.counts[topic] += 1
                p.samples[topic]['receipt'] = clock[0]

        p.pump = pump
        with patch('time.monotonic', side_effect=lambda: clock[0]):
            result = p.wait_no_command_stop({t: ['epoch'] for t in ODOM})
        self.assertIsNotNone(result)
        self.assertGreaterEqual(clock[0], 10.35)
        self.assertFalse(result['scan_fresh'])


if __name__ == '__main__':
    unittest.main()
