"""Exact endpoint observations tolerate a bounded name-cache gap without permitting next."""

from copy import deepcopy
import os
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import test_task6_ops as ops_tests
from test_task6_return_auto import context_observation, fixture, probe
from task6_preflight import ODOM
from task6_return_ops import ReturnOperation


GID = '010f525733f822720000000000001a030000000000000000'
KNOWN = dict(gid=GID, node='supervised_return', namespace='/')
UNKNOWN = dict(gid=GID, node='_NODE_NAME_UNKNOWN_', namespace='_NODE_NAMESPACE_UNKNOWN_')


def observation(endpoints=None, stamp=10):
    return dict(
        cmd_endpoints=deepcopy([KNOWN] if endpoints is None else endpoints), cmd_observed_at=stamp
    )


@patch('task6_return_auto.time.monotonic', return_value=10)
class EndpointBindingTest(unittest.TestCase):
    def test_known_then_unknown_never_dispatches_at_boundary(self, clock):
        gate, _ = fixture(participant=False)
        gate.observe_publishers(observation())
        p = probe(10)
        p.endpoints.return_value = [UNKNOWN]
        clock.return_value = 10.49
        self.assertFalse(gate.ready(p))
        self.assertIsNone(gate.hold_since)
        # Unknown observations do not renew the known-identity lease.
        gate.observe_publishers(observation([UNKNOWN], stamp=10.49))
        self.assertEqual(gate.known_command['observed_at'], 10)
        clock.return_value = 10.501
        with self.assertRaisesRegex(RuntimeError, 'Unexpected velocity'):
            gate.ready(p)

    def test_known_then_unknown_is_allowed_only_during_current_action(self, _clock):
        gate, _ = fixture(participant=False)
        gate.before_child(context_observation([]))
        gate.observe_context(context_observation(), child_alive=True)
        gate.mark_requested()
        gate.started = True
        gate.observe_publishers(observation())
        p = probe(10)
        p.endpoints.return_value = [UNKNOWN]
        self.assertFalse(gate.ready(p))
        self.assertIsNone(gate.failure)
        self.assertTrue(gate.requested)

    def test_first_unknown_or_new_entity_same_prefix_is_rejected(self, _clock):
        for prior in (False, True):
            gate, _ = fixture(participant=False)
            if prior:
                gate.observe_publishers(observation())
            endpoint = dict(UNKNOWN)
            if prior:
                endpoint['gid'] = GID[:24] + '00001b03' + GID[32:]
            p = probe(10)
            p.endpoints.return_value = [endpoint]
            with self.subTest(prior=prior), self.assertRaisesRegex(RuntimeError, 'Unexpected'):
                gate.ready(p)

    def test_explicit_alias_namespace_partial_unknown_and_two_are_rejected(self, _clock):
        for endpoints in (
            [dict(KNOWN, node='teleop')],
            [dict(KNOWN, namespace='/other')],
            [dict(UNKNOWN, namespace='/')],
            [dict(KNOWN, namespace='_NODE_NAMESPACE_UNKNOWN_')],
            [KNOWN, UNKNOWN],
        ):
            gate, _ = fixture(participant=False)
            gate.observe_publishers(observation())
            p = probe(10)
            p.endpoints.return_value = endpoints
            with (
                self.subTest(endpoints=endpoints),
                self.assertRaisesRegex(RuntimeError, 'Unexpected'),
            ):
                gate.ready(p)

    def test_empty_observation_or_current_query_clears_binding(self, _clock):
        for via_snapshot in (False, True):
            gate, _ = fixture(participant=False)
            gate.observe_publishers(observation())
            p = probe(10)
            if via_snapshot:
                gate.observe_publishers(observation([]))
            else:
                gate.ready(p)
            self.assertIsNone(gate.known_command)
            p.endpoints.return_value = [UNKNOWN]
            with self.subTest(via_snapshot=via_snapshot), self.assertRaises(RuntimeError):
                gate.ready(p)

    def test_new_known_endpoint_replaces_old_and_new_operation_has_no_binding(self, _clock):
        gate, _ = fixture(participant=False)
        gate.observe_publishers(observation())
        gate.observe_publishers(observation([dict(KNOWN, gid='another-opaque-gid')]))
        p = probe(10)
        p.endpoints.return_value = [UNKNOWN]
        with self.assertRaises(RuntimeError):
            gate.ready(p)
        fresh, _ = fixture(participant=False)
        self.assertIsNone(fresh.known_command)
        with self.assertRaises(RuntimeError):
            fresh.ready(p)

    def test_stale_future_nonfinite_observation_cannot_bind(self, _clock):
        for stamp in (9.499, 10.001, float('nan'), float('inf')):
            gate, _ = fixture(participant=False)
            with self.subTest(stamp=stamp), self.assertRaisesRegex(RuntimeError, 'Stale'):
                gate.observe_publishers(observation(stamp=stamp))
            self.assertIsNone(gate.known_command)

    def test_ready_known_query_alone_does_not_create_binding(self, _clock):
        gate, _ = fixture(participant=False)
        p = probe(10)
        p.endpoints.return_value = [KNOWN]
        self.assertFalse(gate.ready(p))
        self.assertIsNone(gate.known_command)
        p.endpoints.return_value = [UNKNOWN]
        with self.assertRaises(RuntimeError):
            gate.ready(p)

    def test_probe_snapshot_endpoint_copy_is_independent(self, _clock):
        p = ops_tests.FeedbackTest().diagnostic_probe()
        p.node.get_publishers_info_by_topic.return_value = [
            SimpleNamespace(
                endpoint_gid=bytes.fromhex(GID), node_name='supervised_return', node_namespace='/'
            )
        ]
        snapshot = p.snapshot(stopped=False)
        self.assertEqual(snapshot['cmd_endpoints'], [KNOWN])
        self.assertEqual(snapshot['cmd_observed_at'], 10)
        p.last_check['cmd_endpoints'][0]['node'] = 'changed'
        self.assertEqual(snapshot['cmd_endpoints'], [KNOWN])
        snapshot['cmd_endpoints'][0]['gid'] = 'changed'
        self.assertEqual(p.last_check['cmd_endpoints'][0]['gid'], GID)

    def test_existing_rosout_prevents_child_launch(self, _clock):
        gate, _ = fixture(participant=False)
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            p = probe(10)
            p.return_context.return_value = context_observation()
            op = ReturnOperation(
                root,
                dict(auto_next=True),
                p,
                {ODOM[0]: ['gid']},
                gate.source,
                root / 'points.json',
                root,
            )
            op.start_evidence = Mock()
            try:
                with patch('subprocess.Popen') as launch:
                    with self.assertRaisesRegex(RuntimeError, 'Existing supervised_return'):
                        op.execute()
                    launch.assert_not_called()
            finally:
                if op.fifo is not None:
                    os.close(op.fifo)

    def test_execute_consumes_successful_snapshot_before_current_query(self, _clock):
        gate, _ = fixture(participant=False)
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            p = probe(10)
            p.snapshot.return_value = observation()
            p.return_context.side_effect = [context_observation([]), context_observation()]
            p.endpoints.return_value = [UNKNOWN]
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
            op.start_evidence = op.check_limits = op.check_recording = Mock()
            op.save_state = op.receive = Mock()
            op.finish_wait = Mock()  # This test isolates the active-loop observation ordering.
            calls = []

            def advance(rows):
                calls.append(deepcopy(gate.known_command))
                self.assertFalse(gate.ready(p))

            op.advance = advance
            child = Mock(returncode=0)
            child.poll.side_effect = [None, None, 0]
            try:
                with (
                    patch('subprocess.Popen', return_value=child) as launch,
                    patch(
                        'task6_return_ops.audit_rows', return_value=[dict(event='return_completed')]
                    ),
                    patch('task6_return_ops.source_digest', return_value='hash'),
                ):
                    op.execute()
                    self.assertEqual(
                        launch.call_args.kwargs['env']['RMW_IMPLEMENTATION'], 'rmw_fastrtps_cpp'
                    )
            finally:
                if op.fifo is not None:
                    os.close(op.fifo)
            self.assertEqual(calls, [dict(gid=GID, observed_at=10)])


if __name__ == '__main__':
    unittest.main()
