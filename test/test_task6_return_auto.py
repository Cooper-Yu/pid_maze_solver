"""Automatic progression rejects reordered evidence and unobserved stop boundaries."""

from dataclasses import dataclass
import json
from pathlib import Path
import sys
import tempfile
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from task6_preflight import ODOM, SCAN
from scene_route import ACTION_NAMES
from task6_ops import digest
from task6_return_auto import AutoAdvance, source_digest, validate_full_route


@dataclass
class Pose:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0


ROSOUT = dict(
    gid='010f525733f8227200000000000011030000000000000000',
    node='supervised_return',
    namespace='/',
)


def context_observation(endpoints=None, stamp=None):
    return dict(
        rmw_identifier='rmw_fastrtps_cpp',
        rosout_endpoints=[dict(ROSOUT)] if endpoints is None else endpoints,
        rosout_observed_at=time.monotonic() if stamp is None else stamp,
    )


def fixture(participant=True):
    source = SimpleNamespace(
        session_id='uuid',
        source_hash='hash',
        origin=Pose(),
        actions=[SimpleNamespace(name='P02', kind='path')],
    )
    gate = AutoAdvance(source, 'gid')
    rows = [
        dict(
            event='return_session_started',
            source_session='uuid',
            source_hash='hash',
            source_origin=dict(x=0.0, y=0.0, yaw=0.0),
            publisher_gid='gid',
        )
    ]
    gate.consume(rows)
    if participant:
        gate.before_child(context_observation([]))
        gate.observe_context(context_observation(), child_alive=True)
    return gate, rows


def action_rows():
    return [
        dict(event='return_action_started', index=0, name='P02', kind='path'),
        dict(event='return_action_completed', cursor=0, name='P02'),
    ]


def probe(now):
    p = Mock()
    p.endpoints.return_value = []
    p.node.get_clock().now().nanoseconds = int(now * 1e9)
    p.samples = {
        t: dict(stamp=now, receipt=now, frame='odom', child='base_link', velocity=(0, 0, 0))
        for t in ODOM
    }
    p.samples[SCAN] = dict(stamp=now, receipt=now, frame='laser', valid_ranges=100)
    p.counts = {t: 10 for t in ODOM}
    p.last_command = dict(receipt=now, values=[0] * 6)
    return p


class AutoTest(unittest.TestCase):
    def test_complete_requires_exact_start_completion_then_terminal(self):
        gate, rows = fixture()
        gate.mark_requested()
        rows += action_rows()
        gate.consume(rows)
        self.assertEqual(gate.cursor, 0)
        gate.consume(rows + [dict(event='return_completed')])
        self.assertTrue(gate.complete)
        with self.assertRaises(RuntimeError):
            gate.mark_requested()

    def test_audit_failures_latch(self):
        for bad in (
            dict(event='return_action_completed', cursor=0, name='P02'),
            dict(event='return_action_started', index=1, name='P02', kind='path'),
            dict(event='return_completed'),
            dict(event='return_failed'),
            dict(event='return_session_failed'),
            dict(event='return_closed'),
        ):
            gate, rows = fixture()
            gate.mark_requested()
            with self.subTest(bad=bad), self.assertRaises(RuntimeError):
                gate.consume(rows + [bad])
            with self.assertRaises(RuntimeError):
                gate.consume(rows)

    def test_binding_and_rewrite_rejected(self):
        for key in ('source_session', 'source_hash', 'source_origin', 'publisher_gid'):
            gate, rows = fixture()
            other = AutoAdvance(gate.source, 'gid')
            rows[0][key] = 'wrong'
            with self.subTest(key=key), self.assertRaises(RuntimeError):
                other.consume(rows)
        gate, _rows = fixture()
        with self.assertRaisesRegex(RuntimeError, 'rewritten'):
            gate.consume([])

    def test_no_duplicate_or_skipped_completion(self):
        gate, rows = fixture()
        gate.mark_requested()
        rows += action_rows()
        gate.consume(rows)
        with self.assertRaises(RuntimeError):
            gate.consume(rows + [rows[-1]])

    @patch('task6_return_auto.time.monotonic')
    def test_stationary_hold_requires_new_both_odometry_samples(self, clock):
        clock.return_value = 10
        gate, _rows = fixture()
        p = probe(10)
        self.assertFalse(gate.ready(p))
        clock.return_value = 10.4
        p = probe(10.4)
        self.assertFalse(gate.ready(p))
        p.counts[ODOM[0]] = 14
        self.assertFalse(gate.ready(p))
        p.counts[ODOM[1]] = 14
        self.assertTrue(gate.ready(p))

    @patch('task6_return_auto.time.monotonic', return_value=10)
    def test_competitor_moving_stale_and_timeout_fail(self, _clock):
        for fault in ('competing', 'moving', 'stale', 'timeout'):
            gate, _rows = fixture()
            p = probe(10)
            if fault == 'competing':
                p.endpoints.return_value = [dict(node='teleop')]
            elif fault == 'moving':
                p.samples[ODOM[1]]['velocity'] = (0.02, 0, 0)
            elif fault == 'stale':
                p.samples[ODOM[0]]['receipt'] = 9
            else:
                gate.boundary_since = 1
            with self.subTest(fault=fault), self.assertRaises(RuntimeError):
                gate.ready(p)

    @patch('task6_return_auto.time.monotonic', return_value=10)
    def test_completed_segment_requires_its_observed_zero(self, _clock):
        for command in (
            None,
            dict(receipt=8, values=[0] * 6),
            dict(receipt=10, values=[0, 0, 0, 0, 0, 0.1]),
            dict(receipt=10, values=[float('nan')] * 6),
        ):
            gate, _rows = fixture()
            gate.source.actions.append(SimpleNamespace(name='later', kind='path'))
            gate.requested_at = 9
            p = probe(10)
            p.last_command = command
            with self.subTest(command=command), self.assertRaises(RuntimeError):
                gate.ready(p)

    def test_full_route_refuses_prefix_and_changed_hash(self):
        gate, rows = fixture()
        with self.assertRaisesRegex(ValueError, 'complete successful'):
            validate_full_route(dict(destination='P12'), gate.source, Path('absent'), rows)
        with tempfile.TemporaryDirectory() as folder:
            p = Path(folder) / 'points.json'
            p.write_text(json.dumps({'a': 1}))
            p.with_suffix('.jsonl').write_text('{}\n')
            before = source_digest(p)
            p.with_suffix('.jsonl').write_text('{}\n{}\n')
            self.assertNotEqual(before, source_digest(p))

    def test_full_fixed_manifest_and_hash_admission(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            route = root / 'route.yaml'
            actions, x = [], 0.0
            for name in ACTION_NAMES:
                a = dict(
                    name=name,
                    kind='turn' if name.endswith('_after_turn') else 'path',
                    heading_rad=0.0,
                )
                if a['kind'] == 'path':
                    a['points'] = [[x, 0.0], [x + 0.02, 0.0]]
                    x += 0.02
                actions.append(a)
            route.write_text(
                json.dumps(
                    dict(
                        schema=1,
                        scene=2,
                        frame='P01',
                        validated=False,
                        test_until='P15',
                        actions=actions,
                    )
                )
            )
            path = root / 'points.json'
            path.write_text('{}')
            rows = [dict(event='route_started', actions=actions)]
            path.with_suffix('.jsonl').write_text(json.dumps(rows[0]) + '\n')
            source = SimpleNamespace(
                actions=[SimpleNamespace(name=a['name']) for a in actions],
                source_hash=source_digest(path),
            )
            config = dict(destination='P15', waypoints=str(route), route_sha256=digest(route))
            validate_full_route(config, source, path, rows)
            with self.assertRaisesRegex(ValueError, 'manifest'):
                validate_full_route(config, source, path, [])
            path.write_text('{"modified":true}')
            with self.assertRaisesRegex(ValueError, 'hash'):
                validate_full_route(config, source, path, rows)

    def test_wrapper_auto_rejects_external_next_and_honors_quit(self):
        from io import StringIO
        from task6_return_ops import ReturnOperation

        gate, rows = fixture()
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            op = ReturnOperation(
                root,
                dict(auto_next=True),
                Mock(),
                {ODOM[0]: ['gid']},
                gate.source,
                root / 'points.json',
                root,
            )
            op.runner = SimpleNamespace(stdin=StringIO())
            op.save_state(rows)
            op.request(dict(id='external', command='next', wall_s=time.time()), rows)
            self.assertEqual(op.runner.stdin.getvalue(), '')
            self.assertFalse(op.state['last_request']['accepted'])
            op.request(dict(id='quit', command='quit', wall_s=time.time()), rows)
            op.advance(rows)
            self.assertEqual(op.runner.stdin.getvalue(), 'quit\n')
            self.assertTrue(op.closing)

    def test_dispatch_health_deadline_epoch_and_hash_failures_never_queue(self):
        from task6_return_ops import ReturnOperation

        for failure in ('hash', 'limits', 'epoch', 'recording', 'cancelled'):
            gate, rows = fixture()
            with tempfile.TemporaryDirectory() as folder:
                root = Path(folder)
                op = ReturnOperation(
                    root,
                    dict(auto_next=True),
                    Mock(),
                    {ODOM[0]: ['gid']},
                    gate.source,
                    root / 'points.json',
                    root,
                )
                op.auto.ready = Mock(return_value=True)
                op.check_limits = Mock()
                op.check_recording = Mock()
                op.request = Mock()
                if failure == 'limits':
                    op.check_limits.side_effect = RuntimeError('deadline')
                elif failure == 'epoch':
                    op.probe.snapshot.side_effect = RuntimeError('GID changed')
                elif failure == 'recording':
                    op.check_recording.side_effect = RuntimeError('recorder exited')
                elif failure == 'cancelled':
                    op.cancelled = True
                with patch(
                    'task6_return_ops.source_digest',
                    return_value='wrong' if failure == 'hash' else 'hash',
                ):
                    if failure == 'cancelled':
                        op.advance(rows)
                    else:
                        with self.subTest(failure=failure), self.assertRaises(RuntimeError):
                            op.advance(rows)
                op.request.assert_not_called()

    def test_cancel_during_pre_dispatch_checks_never_queues(self):
        from task6_return_ops import ReturnOperation

        for stage in ('snapshot', 'recording'):
            gate, rows = fixture()
            with tempfile.TemporaryDirectory() as folder, self.subTest(stage=stage):
                root = Path(folder)
                op = ReturnOperation(
                    root,
                    dict(auto_next=True),
                    Mock(),
                    {ODOM[0]: ['gid']},
                    gate.source,
                    root / 'points.json',
                    root,
                )
                op.auto.ready = Mock(return_value=True)
                op.check_limits = Mock()
                op.check_recording = Mock()
                op.request = Mock()
                target = op.probe.snapshot if stage == 'snapshot' else op.check_recording
                target.side_effect = lambda **kwargs: op.cancel()
                with patch('task6_return_ops.source_digest', return_value='hash'):
                    op.advance(rows)
                self.assertTrue(op.cancelled)
                op.request.assert_not_called()

    def test_request_next_defends_cancel_and_closing_at_dispatch(self):
        from io import StringIO
        from task6_return_ops import ReturnOperation

        for auto in (False, True):
            for state in ('cancelled', 'closing', 'cancel_during_limits'):
                gate, rows = fixture()
                with tempfile.TemporaryDirectory() as folder, self.subTest(auto=auto, state=state):
                    root = Path(folder)
                    op = ReturnOperation(
                        root,
                        dict(auto_next=auto),
                        Mock(),
                        {ODOM[0]: ['gid']},
                        gate.source,
                        root / 'points.json',
                        root,
                    )
                    op.runner = SimpleNamespace(stdin=StringIO())
                    op.save_state(rows)
                    op.check_limits = Mock()
                    if op.auto:
                        op.auto.mark_requested = Mock()
                    if state == 'cancel_during_limits':
                        op.check_limits.side_effect = op.cancel
                    else:
                        setattr(op, state, True)
                    op.request(
                        dict(id='next', command='next', wall_s=time.time()), rows, automatic=auto
                    )
                    self.assertEqual(op.runner.stdin.getvalue(), '')
                    self.assertFalse(op.busy)
                    self.assertFalse(op.state['last_request']['accepted'])
                    if op.auto:
                        op.auto.mark_requested.assert_not_called()

    def test_rejected_endpoint_snapshot_survives_discovery_changes(self):
        for endpoints in (
            [dict(node='unknown', gid='new')],
            [dict(node='supervised_return', gid='old'), dict(node='supervised_return', gid='new')],
        ):
            gate, _rows = fixture()
            p = probe(time.monotonic())
            p.endpoints.return_value = endpoints
            with self.subTest(endpoints=endpoints), self.assertRaises(RuntimeError) as caught:
                gate.ready(p)
            preserved = json.loads(str(caught.exception).split(': ', 1)[1])
            self.assertEqual(preserved['endpoints'], endpoints)
            self.assertEqual(preserved['cursor'], 1)
            self.assertFalse(preserved['requested'])
            self.assertFalse(preserved['started'])
            endpoints.clear()
            self.assertEqual(gate.failure_context, preserved)
            with self.assertRaises(RuntimeError):
                gate.consume([])


if __name__ == '__main__':
    unittest.main()
