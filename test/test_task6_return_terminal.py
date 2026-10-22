"""An audited terminal event changes supervision phase, never proves process success alone."""

from copy import deepcopy
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock, patch

from test_task6_return_auto import action_rows, context_observation, fixture, probe
from task6_preflight import ODOM
from task6_return_ops import ReturnOperation


def operation(root, complete=True):
    gate, rows = fixture()
    gate.mark_requested()
    if complete:
        rows += action_rows() + [dict(event='return_completed')]
        gate.consume(rows)
    p = probe(10)
    p.snapshot.return_value = {}
    op = ReturnOperation(
        root,
        dict(auto_next=True),
        p,
        {ODOM[0]: ['gid']},
        gate.source,
        root / 'points.json',
        root,
    )
    op.auto = gate
    op.check_limits = Mock()
    op.check_recording = Mock()
    op.runner = Mock(returncode=0)
    op.runner.poll.return_value = 0
    return op, rows


@patch('task6_return_ops.time.monotonic', return_value=10)
class TerminalTest(unittest.TestCase):
    def finish(self, op, rows, digest='hash'):
        with (
            patch('task6_return_ops.audit_rows', return_value=rows),
            patch('task6_return_ops.source_digest', return_value=digest),
        ):
            op.finish_wait(op.directory / 'return_points.jsonl')

    def test_complete_exit_zero_checks_stop_without_rosout_or_dispatch(self, _clock):
        with tempfile.TemporaryDirectory() as folder:
            op, rows = operation(Path(folder))
            op.probe.return_context.side_effect = AssertionError('Do not query closing rosout')
            op.receive = op.advance = Mock()
            self.finish(op, rows)
            op.probe.snapshot.assert_called_once_with(stopped=True, expected=op.identities)
            op.check_limits.assert_called_once()
            op.check_recording.assert_called_once()
            op.probe.return_context.assert_not_called()
            op.receive.assert_not_called()
            self.assertFalse(op.state['ready'])
            self.assertEqual(op.phase, 'return_exit_wait')

    def test_terminal_event_with_nonzero_child_exit_fails(self, _clock):
        for code in (2, -2, -9):
            with tempfile.TemporaryDirectory() as folder, self.subTest(code=code):
                op, rows = operation(Path(folder))
                op.runner.poll.return_value = code
                with self.assertRaisesRegex(RuntimeError, f'exited {code}'):
                    self.finish(op, rows)

    def test_alive_child_has_two_second_exit_deadline(self, clock):
        with tempfile.TemporaryDirectory() as folder:
            op, rows = operation(Path(folder))
            op.runner.poll.return_value = None
            op.probe.pump.side_effect = lambda: setattr(
                clock, 'return_value', clock.return_value + 0.6
            )
            with self.assertRaisesRegex(RuntimeError, 'within 2 seconds'):
                self.finish(op, rows)
            self.assertEqual(op.probe.snapshot.call_count, 4)
            self.assertEqual(op.check_recording.call_count, 4)

    def test_wait_checks_each_iteration_until_normal_exit(self, _clock):
        with tempfile.TemporaryDirectory() as folder:
            op, rows = operation(Path(folder))
            op.runner.poll.side_effect = [None, None, 0]
            self.finish(op, rows)
            self.assertEqual(op.probe.snapshot.call_count, 3)
            self.assertEqual(op.check_recording.call_count, 3)

    def test_missing_or_premature_completion_cannot_enter_exit_wait(self, _clock):
        with tempfile.TemporaryDirectory() as folder:
            op, rows = operation(Path(folder), complete=False)
            with self.assertRaisesRegex(RuntimeError, 'Premature'):
                op.auto.consume(rows + [dict(event='return_completed')])
            with self.assertRaisesRegex(RuntimeError, 'audited complete'):
                self.finish(op, rows)
            op.probe.snapshot.assert_not_called()
        with tempfile.TemporaryDirectory() as folder:
            op, rows = operation(Path(folder), complete=False)
            rows += action_rows()
            op.auto.consume(rows)
            with self.assertRaisesRegex(RuntimeError, 'audited complete'):
                self.finish(op, rows)

    def test_external_publisher_stale_feedback_epoch_motion_and_recording_fail(self, _clock):
        for reason in (
            'cmd publisher',
            'stale feedback',
            'GID changed',
            'not stopped',
            'recording failed',
        ):
            with tempfile.TemporaryDirectory() as folder, self.subTest(reason=reason):
                op, rows = operation(Path(folder))
                target = op.check_recording if reason == 'recording failed' else op.probe.snapshot
                target.side_effect = RuntimeError(reason)
                with self.assertRaisesRegex(RuntimeError, reason):
                    self.finish(op, rows)
                op.probe.snapshot.assert_called_with(stopped=True, expected=op.identities)
                op.runner.poll.assert_not_called()

    def test_final_zero_must_follow_last_requested_action(self, _clock):
        for command in (
            None,
            dict(receipt=9, values=[0] * 6),
            dict(receipt=10, values=[0, 0, 0, 0, 0, 0.1]),
            dict(receipt=10, values=[float('nan')] * 6),
        ):
            with tempfile.TemporaryDirectory() as folder, self.subTest(command=command):
                op, rows = operation(Path(folder))
                op.probe.last_command = command
                with self.assertRaisesRegex(RuntimeError, 'final zero'):
                    self.finish(op, rows)

    def test_source_or_audit_change_after_terminal_fails(self, _clock):
        for change in ('source', 'truncated', 'failed', 'extra'):
            with tempfile.TemporaryDirectory() as folder, self.subTest(change=change):
                op, rows = operation(Path(folder))
                rows = deepcopy(rows)
                digest = 'hash'
                if change == 'source':
                    digest = 'changed'
                elif change == 'truncated':
                    rows.pop()
                else:
                    rows.append(dict(event='return_failed' if change == 'failed' else 'progress'))
                with self.assertRaises(RuntimeError):
                    self.finish(op, rows, digest)

    def test_cancellation_and_booking_limits_still_abort_exit_wait(self, _clock):
        for reason in ('Cancelled', 'Maximum duration', 'Booking deadline'):
            with tempfile.TemporaryDirectory() as folder, self.subTest(reason=reason):
                op, rows = operation(Path(folder))
                op.check_limits.side_effect = RuntimeError(reason)
                with self.assertRaisesRegex(RuntimeError, reason):
                    self.finish(op, rows)
                op.runner.poll.assert_not_called()

    def test_advance_after_terminal_never_queries_or_dispatches(self, _clock):
        with tempfile.TemporaryDirectory() as folder:
            op, rows = operation(Path(folder))
            op.auto.ready = Mock(side_effect=AssertionError('No active admission after terminal'))
            op.request = Mock()
            op.advance(rows)
            op.auto.ready.assert_not_called()
            op.request.assert_not_called()

    def test_execute_terminal_before_child_exit_skips_disappearing_rosout(self, _clock):
        with tempfile.TemporaryDirectory() as folder:
            op, rows = operation(Path(folder), complete=False)
            rows += action_rows() + [dict(event='return_completed')]
            op.auto.before_child = (
                Mock()
            )  # Already admitted by the fixture; isolate terminal order.
            op.start_evidence = Mock()
            op.receive = op.advance = Mock()
            op.probe.return_context.side_effect = [
                context_observation([]),
                AssertionError('rosout unregistered'),
            ]
            child = Mock(returncode=0)
            child.poll.side_effect = [None, 0]
            try:
                with (
                    patch('subprocess.Popen', return_value=child),
                    patch('task6_return_ops.audit_rows', return_value=rows),
                    patch('task6_return_ops.source_digest', return_value='hash'),
                ):
                    op.execute()
                self.assertTrue(op.auto.complete)
                self.assertEqual(op.probe.return_context.call_count, 1)
                self.assertEqual(
                    op.probe.snapshot.call_args_list[-1].kwargs,
                    dict(stopped=True, expected=op.identities),
                )
                op.advance.assert_not_called()
                op.receive.assert_not_called()
            finally:
                if op.fifo is not None:
                    os.close(op.fifo)


if __name__ == '__main__':
    unittest.main()
