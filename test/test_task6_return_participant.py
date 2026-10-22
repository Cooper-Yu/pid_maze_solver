"""Fast DDS participant correlation is bounded graph evidence, never authentication."""

from copy import deepcopy
import unittest
from unittest.mock import patch

from test_task6_return_auto import ROSOUT, context_observation, fixture, probe
from test_task6_return_endpoint_binding import GID, KNOWN, UNKNOWN, observation
from task6_return_auto import AutoAdvance


@patch('task6_return_auto.time.monotonic', return_value=10)
class ParticipantTest(unittest.TestCase):
    def test_first_unknown_and_new_entity_match_bound_context_but_never_queue(self, _clock):
        for gid in (GID, GID[:24] + '0000ff03' + GID[32:]):
            gate, _ = fixture()
            p = probe(10)
            p.endpoints.return_value = [dict(UNKNOWN, gid=gid)]
            gate.observe_publishers(observation(p.endpoints.return_value))
            self.assertIsNone(gate.known_command)
            self.assertFalse(gate.ready(p))
            self.assertIsNone(gate.hold_since)
            gate.mark_requested()
            self.assertFalse(gate.ready(p))
            self.assertIsNone(gate.failure)

    def test_wrong_rmw_rejected_before_child_or_after_binding(self, _clock):
        for after in (False, True):
            for rmw in ('rmw_cyclonedds_cpp', 'rmw_fastrtps_dynamic_cpp', None):
                gate, _ = fixture(participant=after)
                item = context_observation([])
                item['rmw_identifier'] = rmw
                with (
                    self.subTest(after=after, rmw=rmw),
                    self.assertRaisesRegex(RuntimeError, 'actual rmw'),
                ):
                    if after:
                        gate.observe_context(item, child_alive=True)
                    else:
                        gate.before_child(item)

    def test_old_same_name_any_namespace_and_restart_rejected(self, _clock):
        for namespace in ('/', '/another'):
            gate, _ = fixture(participant=False)
            with (
                self.subTest(namespace=namespace),
                self.assertRaisesRegex(RuntimeError, 'Existing'),
            ):
                gate.before_child(context_observation([dict(ROSOUT, namespace=namespace)]))
        gate, _ = fixture()
        with self.assertRaisesRegex(RuntimeError, 'cannot restart'):
            gate.before_child(context_observation([]))

    def test_baseline_unknown_cannot_be_later_adopted(self, _clock):
        gate, _ = fixture(participant=False)
        gate.before_child(context_observation([dict(ROSOUT, node='_NODE_NAME_UNKNOWN_')]))
        with self.assertRaisesRegex(RuntimeError, 'new rosout endpoint'):
            gate.observe_context(context_observation(), child_alive=True)

    def test_attach_must_precede_binding_and_binding_must_precede_next(self, _clock):
        existing, rows = fixture(participant=False)
        gate = AutoAdvance(existing.source, 'gid')
        gate.before_child(context_observation([]))
        gate.observe_context(context_observation(), child_alive=True)
        self.assertIsNone(gate.context_binding)
        gate.consume(rows)
        self.assertFalse(gate.ready(probe(10)))
        with self.assertRaisesRegex(RuntimeError, 'fresh bound'):
            gate.mark_requested()
        other, _ = fixture(participant=False)
        other.before_child(context_observation([]))
        other.requested_at = 9
        with self.assertRaisesRegex(RuntimeError, 'before first next'):
            other.observe_context(context_observation(), child_alive=True)

    def test_missing_initial_rosout_waits_only_within_existing_deadline(self, clock):
        gate, _ = fixture(participant=False)
        gate.before_child(context_observation([]))
        gate.observe_context(context_observation([]), child_alive=True)
        self.assertFalse(gate.ready(probe(10)))
        clock.return_value = 18.01
        with self.assertRaisesRegex(RuntimeError, '8 seconds'):
            gate.ready(probe(18.01))

    def test_duplicate_or_wrong_namespace_cannot_bind(self, _clock):
        for endpoints in ([ROSOUT, ROSOUT], [dict(ROSOUT, namespace='/other')]):
            gate, _ = fixture(participant=False)
            gate.before_child(context_observation([]))
            with (
                self.subTest(endpoints=endpoints),
                self.assertRaisesRegex(RuntimeError, 'ambiguous'),
            ):
                gate.observe_context(context_observation(endpoints), child_alive=True)

    def test_missing_changed_unknown_rosout_or_dead_child_cannot_rebind(self, _clock):
        for endpoints, alive in (
            ([], True),
            ([dict(ROSOUT, gid=GID)], True),
            ([dict(ROSOUT, node='_NODE_NAME_UNKNOWN_')], True),
            ([ROSOUT], False),
            ([ROSOUT, ROSOUT], True),
        ):
            gate, _ = fixture()
            before = deepcopy(gate.context_binding)
            with self.subTest(endpoints=endpoints, alive=alive), self.assertRaises(RuntimeError):
                gate.observe_context(context_observation(endpoints), child_alive=alive)
            self.assertEqual(gate.context_binding, before)

    def test_stale_future_and_invalid_context_observation_rejected(self, _clock):
        for stamp in (9.499, 10.001, float('nan'), float('inf')):
            gate, _ = fixture()
            with self.subTest(stamp=stamp), self.assertRaisesRegex(RuntimeError, 'Stale rosout'):
                gate.observe_context(context_observation(stamp=stamp), child_alive=True)

    def test_unknown_does_not_refresh_rosout_and_expired_binding_refuses_next(self, clock):
        gate, _ = fixture()
        p = probe(10)
        p.endpoints.return_value = [UNKNOWN]
        clock.return_value = 10.5
        self.assertFalse(gate.ready(p))
        self.assertEqual(gate.context_seen_at, 10)
        clock.return_value = 10.501
        with self.assertRaises(RuntimeError):
            gate.ready(p)
        with self.assertRaisesRegex(RuntimeError, 'fresh bound'):
            gate.mark_requested()

    def test_unknown_wrong_prefix_explicit_alias_namespace_multiple_rejected(self, _clock):
        for endpoints in (
            [dict(UNKNOWN, gid='ff' + GID[2:])],
            [dict(KNOWN, gid='ff' + GID[2:])],
            [dict(KNOWN, node='teleop')],
            [dict(KNOWN, namespace='/other')],
            [dict(UNKNOWN, namespace='/')],
            [UNKNOWN, UNKNOWN],
        ):
            gate, _ = fixture()
            p = probe(10)
            p.endpoints.return_value = endpoints
            with (
                self.subTest(endpoints=endpoints),
                self.assertRaisesRegex(RuntimeError, 'Unexpected velocity'),
            ):
                gate.ready(p)

    def test_gid_layout_rejects_bad_length_hex_zero_prefix_and_nonzero_padding(self, _clock):
        for gid in (
            '',
            GID[:32],
            GID + '00',
            'zz' + GID[2:],
            '00' * 12 + GID[24:],
            GID[:-2] + '01',
            None,
        ):
            self.assertIsNone(AutoAdvance.fastdds_prefix(gid))
            gate, _ = fixture(participant=False)
            gate.before_child(context_observation([]))
            with self.subTest(gid=gid), self.assertRaisesRegex(RuntimeError, 'Invalid Fast DDS'):
                gate.observe_context(context_observation([dict(ROSOUT, gid=gid)]), child_alive=True)

    def test_other_rosout_publishers_allowed_and_empty_cmd_keeps_same_context(self, _clock):
        gate, _ = fixture()
        gate.observe_context(
            context_observation(
                [dict(ROSOUT), dict(node='recorder', namespace='/', gid='unrelated')]
            ),
            child_alive=True,
        )
        before = deepcopy(gate.context_binding)
        gate.observe_publishers(observation([]))
        self.assertIsNone(gate.known_command)
        self.assertEqual(gate.context_binding, before)
        p = probe(10)
        p.endpoints.return_value = [UNKNOWN]
        self.assertFalse(gate.ready(p))

    def test_context_snapshot_is_read_only_and_independent(self, _clock):
        from task6_preflight import Probe
        from unittest.mock import Mock

        p = Probe.__new__(Probe)
        p.rmw_identifier = 'rmw_fastrtps_cpp'
        endpoints = [dict(ROSOUT)]
        p.endpoints = Mock(return_value=endpoints)
        item = p.return_context()
        self.assertEqual(item, context_observation())
        p.endpoints.assert_called_once_with('/rosout')
        endpoints[0]['node'] = 'changed'
        self.assertEqual(item['rosout_endpoints'], [ROSOUT])


if __name__ == '__main__':
    unittest.main()
