"""Owner hold acknowledgement, strict active freshness, and bounded recovery evidence."""

import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from task6_feedback_pause import PauseEvents, SupervisorPause, validate_paused_feedback
from task6_preflight import ODOM, SCAN, validate_feedback


def samples(age=0.7):
    data = {
        t: dict(
            stamp=100 - age,
            receipt=10 - age,
            frame='odom',
            child='base_link',
            pose=[1, 2, 0],
            velocity=[0, 0, 0],
        )
        for t in ODOM
    }
    data[SCAN] = dict(stamp=100 - age, receipt=10 - age, frame='laser', valid_ranges=30)
    return data


class PauseValidationTest(unittest.TestCase):
    def test_held_cache_never_passes_active_admission(self):
        data = samples()
        validate_paused_feedback(data, 100, 10, 1.0)
        with self.assertRaises(RuntimeError):
            validate_feedback(data, 100, 10, False, scan_timeout=1.0)

    def test_insufficient_scan_returns_only_waits_in_acknowledged_hold(self):
        data = samples(age=0.1)
        data[SCAN]['valid_ranges'] = 0
        self.assertFalse(validate_paused_feedback(data, 100, 10, 1.0))
        with self.assertRaisesRegex(RuntimeError, 'Insufficient finite scan returns'):
            validate_feedback(data, 100, 10, False, scan_timeout=1.0)

    def test_invalid_or_absent_cached_data_stays_fatal(self):
        for topic in (*ODOM, SCAN):
            for change in (
                dict(stamp=100.2),
                dict(stamp=float('nan')),
                dict(stamp=0),
                dict(receipt=4.4),
                dict(frame=''),
            ):
                data = samples()
                data[topic].update(change)
                with self.subTest(topic=topic, change=change), self.assertRaises(RuntimeError):
                    validate_paused_feedback(data, 100, 10, 1.0)
            data = samples()
            del data[topic]
            with self.assertRaises(RuntimeError):
                validate_paused_feedback(data, 100, 10, 1.0)
        for change in (dict(pose=[float('inf'), 0, 0]), dict(child='changed')):
            data = samples()
            data[ODOM[0]].update(change)
            with self.assertRaises(RuntimeError):
                validate_paused_feedback(data, 100, 10, 1.0)


class OwnerHoldTest(unittest.TestCase):
    def setUp(self):
        self.folder = tempfile.TemporaryDirectory()
        self.addCleanup(self.folder.cleanup)
        self.root = Path(self.folder.name)
        self.monitor = SupervisorPause(self.root)
        self.probe = SimpleNamespace(
            snapshot=Mock(return_value={'strict': True}),
            paused_snapshot=Mock(return_value={'cmd_endpoints': [{'gid': 'owner'}]}),
            last_command=dict(values=[0.0] * 6, receipt=10),
        )
        self.expected = {t: ['epoch'] for t in ODOM}

    def event(self, name='feedback_pause_started', **extra):
        row = dict(event=name, pause_id=1, budget_s=5, owns_publisher=True, **extra)
        with (self.root / 'points.jsonl').open('a') as stream:
            stream.write(json.dumps(row) + '\n')

    def observe(self, now=10):
        with patch('task6_feedback_pause.time.monotonic', return_value=now):
            return self.monitor.observe(self.probe, self.expected, 1.0)

    def test_zero_history_without_owner_ack_stays_strict(self):
        self.observe()
        self.probe.snapshot.assert_called_once_with(
            stopped=False, expected=self.expected, scan_timeout=1.0
        )
        self.probe.paused_snapshot.assert_not_called()

    def test_acknowledged_hold_only_and_complete_restores_strict_check(self):
        self.event()
        self.observe()
        self.probe.paused_snapshot.assert_called_once_with(expected=self.expected, scan_timeout=1.0)
        self.event('feedback_pause_completed')
        self.observe()
        self.probe.snapshot.assert_called_once()

    def test_zero_ack_race_gets_no_stale_data_grace(self):
        self.event()
        self.probe.last_command['values'][0] = 0.1
        self.probe.snapshot.side_effect = RuntimeError('stale')
        with self.assertRaisesRegex(RuntimeError, 'stale'):
            self.observe()
        self.probe.paused_snapshot.assert_not_called()

    def test_nonzero_or_stale_zero_after_ack_aborts(self):
        self.event()
        self.observe()
        self.probe.last_command['values'][0] = 0.001
        with self.assertRaisesRegex(RuntimeError, 'lost its zero'):
            self.observe()
        self.probe.last_command['values'][0] = 0
        with self.assertRaisesRegex(RuntimeError, 'lost its zero'):
            self.observe(10.21)

    def test_missing_owner_competitor_or_replaced_owner_aborts(self):
        for endpoints in ([], [{'gid': 'owner'}, {'gid': 'other'}]):
            self.event()
            self.probe.paused_snapshot.return_value = dict(cmd_endpoints=endpoints)
            self.monitor = SupervisorPause(self.root)
            if not endpoints:
                self.observe()
                self.probe.snapshot.assert_called()
                self.probe.last_command['receipt'] = 10.3
                with self.assertRaisesRegex(RuntimeError, 'publisher missing'):
                    self.observe(10.3)
                self.probe.last_command['receipt'] = 10
            else:
                with self.assertRaisesRegex(RuntimeError, 'publisher missing'):
                    self.observe()
        self.monitor = SupervisorPause(self.root)
        self.probe.paused_snapshot.return_value = dict(cmd_endpoints=[{'gid': 'owner'}])
        self.observe()
        self.probe.paused_snapshot.return_value = dict(cmd_endpoints=[{'gid': 'replacement'}])
        with self.assertRaisesRegex(RuntimeError, 'owner changed'):
            self.observe()

    def test_feedback_or_identity_failure_is_not_swallowed(self):
        self.event()
        self.probe.paused_snapshot.side_effect = RuntimeError('identity changed')
        with self.assertRaisesRegex(RuntimeError, 'identity changed'):
            self.observe()

    def test_owner_hold_has_fixed_five_second_deadline(self):
        self.event()
        self.observe()
        self.probe.last_command['receipt'] = 15.01
        with self.assertRaisesRegex(RuntimeError, 'exceeded 5 seconds'):
            self.observe(15.01)

    def test_initialization_marker_has_same_owned_zero_contract(self):
        (self.root / 'motion.log').write_text('FEEDBACK_RECOVERING: zero commands; 5 s budget\n')
        self.observe()
        self.probe.paused_snapshot.assert_called_once()
        with (self.root / 'motion.log').open('a') as stream:
            stream.write('FEEDBACK_RECOVERED: verified\n')
        self.observe()
        self.probe.snapshot.assert_called_once()

    def test_partial_line_and_completion_for_other_id_cannot_release_hold(self):
        path = self.root / 'points.jsonl'
        row = json.dumps(dict(event='feedback_pause_started', pause_id=1, budget_s=5))
        path.write_text(row)
        events = PauseEvents(self.root)
        self.assertIsNone(events.poll())
        with path.open('a') as stream:
            stream.write(
                '\n' + json.dumps(dict(event='feedback_pause_completed', pause_id=2)) + '\n'
            )
        self.assertEqual(events.poll()['key'], ('route', 1))
        self.assertEqual(events.poll()['key'], ('route', 1))

    def test_released_boundary_requires_no_current_publisher(self):
        self.event(owns_publisher_unused=False)
        # Replace the row rather than silently changing the owned-hold default.
        (self.root / 'points.jsonl').write_text(
            json.dumps(
                dict(event='feedback_pause_started', pause_id=1, budget_s=5, owns_publisher=False)
            )
            + '\n'
        )
        self.probe.paused_snapshot.return_value = dict(cmd_endpoints=[])
        self.probe.last_command['receipt'] = 1
        self.observe()
        self.probe.paused_snapshot.return_value = dict(cmd_endpoints=[{'gid': 'other'}])
        with self.assertRaisesRegex(RuntimeError, 'Unexpected publisher'):
            self.observe()


if __name__ == '__main__':
    unittest.main()
