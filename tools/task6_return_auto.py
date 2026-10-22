"""Default-off return sequencing admission; never publishes robot commands."""

from copy import deepcopy
from dataclasses import asdict
import hashlib
import json
import math
import time

from scene_route import ACTION_NAMES, load_scene, select_real
from task6_ops import digest
from task6_preflight import ODOM, validate_feedback


def source_digest(path):
    return hashlib.sha256(
        path.read_bytes() + b'\n' + path.with_suffix('.jsonl').read_bytes()
    ).hexdigest()


def validate_full_route(config, source, path, rows):
    """Automatic return requires the unchanged, complete selected P01-to-P15 route."""
    if config.get('destination') != 'P15' or tuple(a.name for a in source.actions) != ACTION_NAMES:
        raise ValueError('Automatic return requires the complete successful P01-to-P15 route')
    if (
        digest(config['waypoints']) != config['route_sha256']
        or source_digest(path) != source.source_hash
    ):
        raise ValueError('Automatic return source/route hash changed')
    selected = select_real(load_scene(config['waypoints'], 2), 'P15', execute=True)
    manifests = [r['actions'] for r in rows if r.get('event') == 'route_started']
    normalized = [
        [{k: v for k, v in a.items() if not (k == 'points' and not v)} for a in actions]
        for actions in manifests
    ]
    if normalized != [selected['actions']]:
        raise ValueError('Completed action manifest differs from the fixed full route')


class AutoAdvance:
    """Consume an append-only audit and permit exactly one outstanding next."""

    def __init__(self, source, gid):
        self.source, self.gid = source, gid
        self.cursor = len(source.actions)
        self.rows = []
        self.attached = self.requested = self.started = self.complete = False
        self.failure = None
        self.failure_context = None
        self.boundary_since = self.hold_since = None
        self.first_counts = None
        self.requested_at = None
        self.known_command = None
        self.rosout_baseline = None
        self.context_binding = None
        self.context_seen_at = None
        self.context_alive = False

    def fail(self, reason):
        self.failure = reason
        raise RuntimeError(reason)

    def consume(self, rows):
        if self.failure:
            self.fail(self.failure)
        if rows[: len(self.rows)] != self.rows or len(rows) < len(self.rows):
            self.fail('Return audit was rewritten or truncated')
        for row in rows[len(self.rows) :]:
            self.event(row)
        self.rows = deepcopy(rows)

    def event(self, row):
        kind = row.get('event')
        if kind in ('return_failed', 'return_session_failed', 'return_closed'):
            self.fail('Return failed or was closed; automatic retry is forbidden')
        if kind == 'return_session_started':
            if (
                self.attached
                or row.get('source_session') != self.source.session_id
                or row.get('source_hash') != self.source.source_hash
                or row.get('source_origin') != asdict(self.source.origin)
                or row.get('publisher_gid') != self.gid
            ):
                self.fail('Return attachment does not bind the exact source and epoch')
            self.attached = True
            self.boundary_since = time.monotonic()
        elif kind == 'return_action_started':
            action = self.source.actions[self.cursor - 1] if self.cursor else None
            if (
                not self.attached
                or not self.requested
                or self.started
                or action is None
                or row.get('index') != self.cursor - 1
                or row.get('name') != action.name
                or row.get('kind') != action.kind
            ):
                self.fail('Unexpected return action start/index')
            self.started = True
        elif kind == 'return_action_completed':
            if (
                not self.started
                or row.get('cursor') != self.cursor - 1
                or row.get('name') != self.source.actions[self.cursor - 1].name
            ):
                self.fail('Unexpected return completion/cursor')
            self.cursor -= 1
            self.requested = self.started = False
            self.boundary_since = time.monotonic()
            self.hold_since = self.first_counts = None
        elif kind == 'return_completed':
            if not self.attached or self.cursor or self.requested or self.complete:
                self.fail('Premature or duplicate full return completion')
            self.complete = True

    @staticmethod
    def fastdds_prefix(gid):
        """Decode only the verified Humble Fast DDS 16-byte GUID in 24-byte RMW storage."""
        if not isinstance(gid, str) or len(gid) != 48:
            return None
        try:
            raw = bytes.fromhex(gid)
        except ValueError:
            return None
        if len(raw) != 24 or not any(raw[:12]) or any(raw[16:]):
            return None
        return raw[:12].hex()

    def context_observation(self, snapshot):
        if snapshot.get('rmw_identifier') != 'rmw_fastrtps_cpp':
            self.fail('Automatic return participant binding requires actual rmw_fastrtps_cpp')
        stamp = snapshot['rosout_observed_at']
        if not 0 <= time.monotonic() - stamp <= 0.5:
            self.fail('Stale rosout participant observation')
        return snapshot['rosout_endpoints'], stamp

    def before_child(self, snapshot):
        endpoints, _stamp = self.context_observation(snapshot)
        if self.rosout_baseline is not None:
            self.fail('Return child participant admission cannot restart')
        if any(e.get('node') == 'supervised_return' for e in endpoints):
            self.fail('Existing supervised_return rosout publisher before child startup')
        self.rosout_baseline = {e['gid'] for e in endpoints}

    def observe_context(self, snapshot, child_alive):
        endpoints, stamp = self.context_observation(snapshot)
        self.context_alive = child_alive
        if not child_alive or self.rosout_baseline is None:
            self.fail('Return participant has no admitted live child')
        if not self.attached:
            return
        candidates = [e for e in endpoints if e.get('node') == 'supervised_return']
        if not candidates and self.context_binding is None:
            return  # Existing eight-second boundary deadline bounds discovery before first next.
        if len(candidates) != 1 or candidates[0].get('namespace') != '/':
            self.fail('Return rosout participant identity missing or ambiguous')
        gid = candidates[0].get('gid')
        prefix = self.fastdds_prefix(gid)
        if prefix is None:
            self.fail('Invalid Fast DDS rosout GID')
        if self.context_binding is None:
            if self.requested_at is not None or gid in self.rosout_baseline:
                self.fail('Return participant must bind a new rosout endpoint before first next')
            self.context_binding = dict(gid=gid, prefix=prefix)
        elif gid != self.context_binding['gid']:
            self.fail('Return rosout participant identity changed; rebinding forbidden')
        self.context_seen_at = stamp

    def context_fresh(self, now):
        return (
            self.context_binding is not None
            and self.context_alive
            and self.context_seen_at is not None
            and 0 <= now - self.context_seen_at <= 0.5
        )

    def same_participant(self, gid, now):
        return (
            self.context_fresh(now) and self.fastdds_prefix(gid) == self.context_binding['prefix']
        )

    def mark_requested(self):
        if not self.context_fresh(time.monotonic()):
            self.fail('Automatic next requires a fresh bound return participant')
        if not self.attached or self.requested or not self.cursor or self.complete or self.failure:
            self.fail('Cannot queue another automatic action')
        self.requested = True
        self.requested_at = time.monotonic()
        self.hold_since = self.first_counts = None

    def observe_publishers(self, snapshot):
        """Bind only a fresh, successful Probe observation in this operation instance."""
        now = time.monotonic()
        observed_at = snapshot['cmd_observed_at']
        if not 0 <= now - observed_at <= 0.5:
            self.fail('Stale command endpoint observation')
        self.check_publishers(snapshot['cmd_endpoints'], now, observed_at)

    def check_publishers(self, endpoints, now, observed_at=None):
        """Admit graph metadata gaps only through exact GID or the live bound participant."""
        if not endpoints:
            self.known_command = None
            return
        endpoint = endpoints[0]
        gid = endpoint.get('gid')
        if self.known_command and gid != self.known_command['gid']:
            self.known_command = None
        known = (
            bool(gid)
            and endpoint.get('node') == 'supervised_return'
            and endpoint.get('namespace') == '/'
        )
        unknown = (
            endpoint.get('node') == '_NODE_NAME_UNKNOWN_'
            and endpoint.get('namespace') == '_NODE_NAMESPACE_UNKNOWN_'
            and (
                (
                    self.known_command is not None
                    and gid == self.known_command['gid']
                    and 0 <= now - self.known_command['observed_at'] <= 0.5
                )
                or self.same_participant(gid, now)
            )
        )
        context_mismatch = self.context_binding is not None and not self.same_participant(gid, now)
        if len(endpoints) != 1 or not (known or unknown) or context_mismatch:
            self.failure_context = dict(
                endpoints=deepcopy(endpoints),
                known_command=deepcopy(self.known_command),
                participant=deepcopy(self.context_binding),
                participant_observed_at=self.context_seen_at,
                cursor=self.cursor,
                requested=self.requested,
                started=self.started,
                monotonic=now,
            )
            self.fail(
                'Unexpected velocity publisher during automatic return: '
                + json.dumps(self.failure_context, sort_keys=True)
            )
        if known and observed_at is not None:
            self.known_command = dict(gid=gid, observed_at=observed_at)

    def ready(self, probe):
        """Released publisher plus fresh dual-odom stationary hold; no invented zero source."""
        endpoints = probe.endpoints('/cmd_vel')
        now = time.monotonic()
        self.check_publishers(endpoints, now)
        command = probe.last_command
        if command is not None and not all(math.isfinite(v) for v in command['values']):
            self.fail('Nonfinite observed velocity command')
        if self.requested and not self.started and now - self.requested_at > 8:
            self.fail('Automatic next did not start within 8 seconds')
        if not self.attached or self.requested or not self.cursor or self.complete:
            return False
        if now - self.boundary_since > 8:
            self.fail('Return boundary did not become stopped and released within 8 seconds')
        if endpoints or not self.context_fresh(now):
            self.hold_since = self.first_counts = None
            return False
        validate_feedback(probe.samples, probe.node.get_clock().now().nanoseconds / 1e9, now, True)
        if self.cursor < len(self.source.actions):
            command = probe.last_command
            if (
                command is None
                or command['receipt'] < self.requested_at
                or not all(math.isfinite(v) and abs(v) < 1e-9 for v in command['values'])
            ):
                self.fail('Completed return action has no observed final zero command')
        if any(probe.samples[t]['receipt'] < self.boundary_since for t in ODOM):
            return False
        if self.hold_since is None:
            self.hold_since, self.first_counts = now, dict(probe.counts)
        return now - self.hold_since >= 0.3 and all(
            probe.counts[t] >= self.first_counts[t] + 3 for t in ODOM
        )
