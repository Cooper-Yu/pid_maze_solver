"""Pure state/protocol tests; no ROS node or child process is launched."""

import json
from pathlib import Path
import sys
import tempfile
import time
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
import task6_local_session_ops as session
from task6_preflight import ODOM


class SessionTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.path = Path(self.temp.name)
        self.gids = {t: [str(n) * 48] for n, t in enumerate(ODOM)}
        self.config = dict(
            distance=0.03,
            speed=0.01,
            entry='/installed/current_frame_session',
            destination='LOCAL_SESSION_CLOSED',
            max_seconds=600,
            deadline_unix=time.time() + 1000,
            artifact_sha256={},
        )
        self.anchor = dict(x=1.0, y=2.0, yaw=0.3)
        self.start = dict(
            event='session_started',
            session_id='new-local',
            anchor=self.anchor,
            gids={t: v[0] for t, v in self.gids.items()},
            step_distance_m=0.03,
            speed_cap_m_s=0.01,
            laser_policy='observation_only',
            supervised_manual=True,
            historical_p01=False,
            task6_acceptance=False,
        )
        self.ready = dict(
            event='session_ready',
            session_id='new-local',
            anchor=self.anchor,
            next_index=1,
            ready_token='one',
        )
        self.probe = Mock()
        self.op = session.LocalSessionOperation(self.path, self.config, self.probe, self.gids)
        self.op.runner = Mock()
        self.op.check_recording = Mock()
        self.op.audit.consume([self.start, self.ready])

    def request(self, name='step', **extra):
        return dict(
            command=name,
            id=extra.pop('id', 'request-one'),
            wall_s=time.time(),
            ready_token='one',
            session_id='new-local',
            next_index=1,
            **extra,
        )

    def finished_rows(self):
        return [
            self.start,
            self.ready,
            dict(event='step_started', session_id='new-local', index=1, anchor=self.anchor),
            dict(event='step_completed', session_id='new-local', index=1, anchor=self.anchor),
        ]

    def test_one_request_consumes_token_without_auto_continuation(self):
        self.op.request(self.request())
        self.op.runner.stdin.write.assert_called_once_with('step one\n')
        self.assertTrue(self.op.audit.requested)
        self.op.audit.consume(self.finished_rows())
        self.assertFalse(self.op.audit.ready)
        self.op.audit.consume(
            self.finished_rows() + [dict(self.ready, next_index=2, ready_token='two')]
        )
        self.assertTrue(self.op.audit.ready)
        self.op.runner.stdin.write.assert_called_once()
        self.assertEqual(self.op.audit.completed, 1)

    def test_busy_request_is_dropped_not_queued(self):
        self.op.request(self.request())
        self.op.request(self.request(id='second'))
        self.assertFalse(self.op.last_request['accepted'])
        self.op.audit.consume(
            self.finished_rows() + [dict(self.ready, next_index=2, ready_token='two')]
        )
        self.op.runner.stdin.write.assert_called_once()

    def test_previous_token_cannot_dispatch_next_ready(self):
        self.op.request(self.request())
        self.op.audit.consume(
            self.finished_rows() + [dict(self.ready, next_index=2, ready_token='two')]
        )
        self.op.request(self.request(id='old-token'))
        self.assertFalse(self.op.last_request['accepted'])
        self.op.runner.stdin.write.assert_called_once()

    def test_stale_duplicate_and_wrong_identity_refused(self):
        variants = []
        for key, val in [
            ('wall_s', time.time() - 6),
            ('wall_s', float('nan')),
            ('ready_token', 'bad'),
            ('session_id', 'old-p01'),
            ('next_index', 2),
            ('command', 'next'),
            ('command', 'resume'),
        ]:
            message = self.request()
            message[key] = val
            variants.append(message)
        for index, message in enumerate(variants):
            message['id'] = str(index)
            self.op.request(message)
            self.assertFalse(self.op.last_request['accepted'])
        self.op.runner.stdin.write.assert_not_called()
        self.op.request(self.request(id='valid'))
        self.op.request(self.request(id='valid'))
        self.assertFalse(self.op.last_request['accepted'])
        self.op.runner.stdin.write.assert_called_once()

    def test_cancel_during_recording_check_cannot_dispatch(self):
        self.op.check_recording.side_effect = lambda: self.op.cancel()
        with self.assertRaisesRegex(RuntimeError, 'Cancelled'):
            self.op.request(self.request())
        self.op.runner.stdin.write.assert_not_called()

    def test_ready_feedback_identity_failure_stops_dispatch(self):
        self.probe.snapshot.side_effect = RuntimeError('identity changed')
        with self.assertRaisesRegex(RuntimeError, 'identity'):
            self.op.request(self.request())
        self.op.runner.stdin.write.assert_not_called()

    def test_recording_failure_stops_dispatch(self):
        self.op.check_recording.side_effect = RuntimeError('recorder ended')
        with self.assertRaisesRegex(RuntimeError, 'recorder'):
            self.op.request(self.request())
        self.op.runner.stdin.write.assert_not_called()

    def test_quit_once_and_latch_closing(self):
        self.op.request(self.request('quit'))
        self.assertTrue(self.op.closing)
        self.op.request(self.request(id='later'))
        self.assertFalse(self.op.last_request['accepted'])
        self.op.runner.stdin.write.assert_called_once_with('quit\n')

    def test_fixed_anchor_and_identity_cannot_change(self):
        for key, value in [('anchor', dict(x=0, y=0, yaw=0)), ('session_id', 'newer')]:
            row = dict(self.ready, ready_token='two', **{key: value})
            with self.assertRaises(RuntimeError):
                self.op.audit.accept(row)

    def test_audit_append_only_and_ordered(self):
        with self.assertRaises(RuntimeError):
            self.op.audit.consume([self.start])
        with self.assertRaises(RuntimeError):
            self.op.audit.consume([dict(self.start, session_id='bad'), self.ready])
        for event in ('step_started', 'step_completed'):
            with self.assertRaises(RuntimeError):
                self.op.audit.accept(
                    dict(event=event, session_id='new-local', index=1, anchor=self.anchor)
                )

    def test_fresh_token_index_and_close_count_required(self):
        self.op.request(self.request())
        self.op.audit.consume(self.finished_rows())
        for row in [
            dict(self.ready, next_index=2),
            dict(self.ready, ready_token='two'),
            dict(event='session_closed', session_id='new-local', completed_steps=2),
        ]:
            with self.assertRaises(RuntimeError):
                self.op.audit.accept(row)
        self.op.audit.accept(
            dict(event='session_closed', session_id='new-local', completed_steps=1)
        )
        self.assertTrue(self.op.audit.closed)

    def test_failed_and_invalid_start_never_ready(self):
        with self.assertRaisesRegex(RuntimeError, 'Session fault'):
            self.op.audit.accept(dict(event='session_failed', reason='stale scan'))
        for key, value in [
            ('gids', {}),
            ('historical_p01', True),
            ('supervised_manual', False),
            ('task6_acceptance', True),
            ('step_distance_m', 0.1),
            ('speed_cap_m_s', 0.1),
            ('laser_policy', 'enforced'),
            ('anchor', dict(x=float('nan'), y=0, yaw=0)),
        ]:
            audit = session.SessionAudit(self.config, self.gids)
            with self.assertRaises(RuntimeError):
                audit.consume([dict(self.start, **{key: value})])
            self.assertFalse(audit.ready)

    def test_no_command_close_does_not_claim_observed_zero(self):
        self.op.runner.poll.return_value = 0
        self.probe.command_count = 0
        self.probe.last_command = None
        self.probe.wait_no_command_stop.return_value = {}
        result = self.op.verify_final_stop()
        self.assertFalse(result['zero_and_released'])
        self.assertTrue(result['stopped_and_released'])
        self.assertFalse(result['stop_evidence']['zero_command_observed'])

    def test_cli_parameters_and_mode(self):
        args = ['start', '--session-dir', str(self.path / 'new')]
        with patch.object(session, 'provenance') as provenance:
            self.assertEqual(session.main(args), 2)
        provenance.assert_not_called()
        for option, value in [
            ('--speed', 'nan'),
            ('--speed', '.031'),
            ('--step-distance', '.051'),
            ('--step-distance', '0'),
            ('--max-seconds', '1801'),
        ]:
            with self.assertRaises(SystemExit):
                session.main(args + [option, value])

    def test_command_and_saved_state_preserve_local_only(self):
        argv = self.op.build_command()
        self.assertIn('--supervised-manual', argv)
        self.assertIn('--step-distance', argv)
        self.assertNotIn('--distance', argv)
        self.assertNotIn('--source', argv)
        self.assertNotIn('--auto-next', argv)
        self.op.save_state()
        state = json.loads((self.path / 'control.json').read_text())
        self.assertEqual(state['anchor'], self.anchor)
        self.assertEqual(state['ready_token'], 'one')
        self.assertFalse(state['task6_acceptance'])

    def test_send_rejects_stale_closed_or_busy_before_fifo(self):
        for changed in [
            dict(closed=True),
            dict(wall_s=time.time() - 3),
            dict(busy=True),
            dict(ready=False),
        ]:
            state = dict(closed=False, ready=True, busy=False, wall_s=time.time())
            state.update(changed)
            with (
                patch.object(session, 'read_json', return_value=state),
                patch.object(session.os, 'open') as opened,
                self.assertRaises(RuntimeError),
            ):
                session.send(self.path, 'step')
            opened.assert_not_called()


if __name__ == '__main__':
    unittest.main()
