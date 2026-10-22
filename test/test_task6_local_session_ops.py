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

    def waiting_row(self, **changes):
        now = time.monotonic()
        row = dict(
            event='session_waiting',
            session_id='new-local',
            anchor=self.anchor,
            next_index=1,
            reason_code='SCAN_ODOM_BRACKET_TOO_WIDE',
            reason='Odom bracket too wide for scan: 0.165s',
            scan_stamp_ns=100,
            waiting_started_mono=now,
            waiting_deadline_mono=now + 2,
            revoked_ready_token='one',
        )
        row.update(changes)
        return row

    def test_wait_revokes_ready_and_recovery_requires_new_manual_command(self):
        self.op.audit.accept(self.waiting_row())
        self.op.save_state()
        self.assertFalse(self.op.audit.ready)
        self.assertIsNone(self.op.audit.token)
        self.op.request(self.request())
        self.assertFalse(self.op.last_request['accepted'])
        self.op.runner.stdin.write.assert_not_called()
        self.op.audit.accept(
            dict(self.ready, ready_token='two', recovered_scan_stamps=[101, 102, 103])
        )
        self.assertTrue(self.op.audit.ready)
        self.op.runner.stdin.write.assert_not_called()
        self.op.request(self.request(id='stale-after-recovery'))
        self.assertFalse(self.op.last_request['accepted'])
        self.op.runner.stdin.write.assert_not_called()

    def test_wait_revokes_sent_but_not_started_request(self):
        self.op.request(self.request())
        self.op.audit.accept(self.waiting_row())
        self.op.save_state()
        self.assertFalse(self.op.audit.requested)
        self.assertFalse(self.op.last_request['accepted'])
        self.op.audit.accept(
            dict(self.ready, ready_token='two', recovered_scan_stamps=[101, 102, 103])
        )
        self.op.runner.stdin.write.assert_called_once_with('step one\n')
        self.assertFalse(self.op.audit.active)

    def test_wait_prohibited_during_active(self):
        self.op.request(self.request())
        self.op.audit.accept(self.finished_rows()[2])
        with self.assertRaisesRegex(RuntimeError, 'Invalid stopped'):
            self.op.audit.accept(self.waiting_row())

    def test_wait_only_whitelist_same_boundary_and_bounded_deadline(self):
        for changes in [
            dict(reason_code='TF_MISSING'),
            dict(next_index=2),
            dict(revoked_ready_token='foreign'),
            dict(scan_stamp_ns=0),
            dict(waiting_started_mono=0, waiting_deadline_mono=3),
            dict(waiting_started_mono=float('nan')),
        ]:
            with self.assertRaises(RuntimeError):
                self.op.audit.accept(self.waiting_row(**changes))
        self.op.audit.accept(self.waiting_row())
        with self.assertRaises(RuntimeError):
            self.op.audit.accept(self.waiting_row())

    def test_recovery_requires_three_distinct_new_scan_stamps(self):
        self.op.audit.accept(self.waiting_row())
        for stamps in ([], [101, 102], [101, 101, 102], [99, 101, 102], [103, 102, 101], None):
            with self.assertRaisesRegex(RuntimeError, 'three new'):
                self.op.audit.accept(
                    dict(self.ready, ready_token='two', recovered_scan_stamps=stamps)
                )
        self.assertFalse(self.op.audit.ready)

    def test_wait_timeout_cannot_dispatch_or_restart_deadline(self):
        self.op.audit.accept(self.waiting_row())
        deadline = self.op.audit.waiting['deadline']
        with (
            patch.object(session.time, 'monotonic', return_value=deadline),
            self.assertRaisesRegex(RuntimeError, '2 seconds'),
        ):
            self.op.audit.check_wait_deadline()
        self.op.runner.stdin.write.assert_not_called()
        self.assertEqual(self.op.audit.waiting['deadline'], deadline)

    def test_initial_wait_without_ready_token_is_supported(self):
        audit = session.SessionAudit(self.config, self.gids)
        audit.consume([self.start, self.waiting_row(revoked_ready_token=None)])
        self.assertFalse(audit.ready)
        audit.accept(dict(self.ready, ready_token='initial', recovered_scan_stamps=[101, 102, 103]))
        self.assertTrue(audit.ready)

    def test_wait_status_exposes_no_valid_permit(self):
        self.op.audit.accept(self.waiting_row())
        self.op.save_state()
        state = json.loads((self.path / 'control.json').read_text())
        self.assertTrue(state['waiting_for_scan_pair'])
        self.assertFalse(state['ready'])
        self.assertIsNone(state['ready_token'])

    def wait_state(self):
        message = dict(id='observed', session_id='new-local', next_index=1, ready_token='one')
        state = dict(
            session_id='new-local',
            wall_s=time.time(),
            closed=False,
            closing=False,
            last_request=dict(message, accepted=True),
        )
        rows = [
            dict(event=e, session_id='new-local', index=1)
            for e in ('step_started', 'step_completed')
        ]
        return message, state, rows

    def test_wait_observes_exact_completed_request_without_sending(self):
        message, state, rows = self.wait_state()
        with (
            patch.object(session, 'read_json', return_value=state),
            patch.object(session, 'audit_rows', return_value=rows),
            patch.object(session, 'send') as send,
            patch.object(session.os, 'open') as opened,
        ):
            session.wait_step(self.path, message)
        send.assert_not_called()
        opened.assert_not_called()

    def test_wait_explicit_rejection_including_pairing_revocation(self):
        message, state, rows = self.wait_state()
        state['last_request'].update(
            accepted=False, reason='Pending step revoked by scan-pairing wait'
        )
        with (
            patch.object(session, 'read_json', return_value=state),
            patch.object(session, 'audit_rows', return_value=rows),
            self.assertRaisesRegex(RuntimeError, 'request rejected'),
        ):
            session.wait_step(self.path, message)

    def test_wait_closed_stale_fault_or_wrong_identity_refuses(self):
        message, state, rows = self.wait_state()
        for changed in (
            dict(closed=True),
            dict(closing=True),
            dict(wall_s=time.time() - 3),
            dict(session_id='other'),
        ):
            with (
                patch.object(session, 'read_json', return_value=dict(state, **changed)),
                self.assertRaises(RuntimeError),
            ):
                session.wait_step(self.path, message)
        with (
            patch.object(session, 'read_json', return_value=state),
            patch.object(
                session,
                'audit_rows',
                return_value=rows + [dict(event='session_failed', session_id='new-local')],
            ),
            self.assertRaisesRegex(RuntimeError, 'session fault'),
        ):
            session.wait_step(self.path, message)

    def test_wait_requires_request_binding_and_matching_completed_index(self):
        message, state, rows = self.wait_state()
        state['last_request']['next_index'] = 2
        with (
            patch.object(session, 'read_json', return_value=state),
            self.assertRaisesRegex(RuntimeError, 'binding mismatch'),
        ):
            session.wait_step(self.path, message)
        state['last_request']['next_index'] = 1
        with (
            patch.object(session, 'read_json', return_value=state),
            patch.object(session, 'audit_rows', return_value=[dict(r, index=2) for r in rows]),
            patch.object(session.time, 'monotonic', side_effect=[0, 0, 21]),
            patch.object(session.time, 'sleep'),
            self.assertRaisesRegex(RuntimeError, 'completion unconfirmed'),
        ):
            session.wait_step(self.path, message)

    def test_wait_timeout_never_resends_and_pending_is_not_completion(self):
        message, state, rows = self.wait_state()
        state['last_request'] = None
        with (
            patch.object(session, 'read_json', return_value=state),
            patch.object(session, 'audit_rows', return_value=rows),
            patch.object(session.time, 'monotonic', side_effect=[0, 0, 21]),
            patch.object(session.time, 'sleep'),
            patch.object(session, 'send') as send,
            self.assertRaisesRegex(RuntimeError, '20s.*do not resend'),
        ):
            session.wait_step(self.path, message)
        send.assert_not_called()

    def test_wait_cli_is_step_only_and_sends_exactly_once(self):
        message, _, _ = self.wait_state()
        with (
            patch.object(session, 'send', return_value=message) as send,
            patch.object(session, 'wait_step', side_effect=RuntimeError('unconfirmed')) as wait,
        ):
            self.assertEqual(session.main(['step', '--session-dir', str(self.path), '--wait']), 2)
        send.assert_called_once_with(self.path, 'step')
        wait.assert_called_once_with(self.path, message)
        with self.assertRaises(SystemExit):
            session.main(['status', '--session-dir', str(self.path), '--wait'])


if __name__ == '__main__':
    unittest.main()
