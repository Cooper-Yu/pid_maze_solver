"""Read-only supervisor acknowledgement of a bounded hold owned by the backend.

This module never publishes velocity and never asks an active controller to keep
using expired feedback. Only an explicit backend zero-hold can outlive freshness.
"""

import json
import math
from pathlib import Path
import time

from task6_preflight import ODOM, SCAN


def validate_paused_feedback(samples, now_ros, now_mono, scan_timeout):
    """Validate cached data during HOLD; this result must never authorize movement."""
    for topic in (*ODOM, SCAN):
        item = samples.get(topic)
        if item is None:
            raise RuntimeError(f'Feedback missing during owner hold: {topic}')
        age, receipt = now_ros - item['stamp'], now_mono - item['receipt']
        if (
            not math.isfinite(age)
            or item['stamp'] <= 0
            or not -0.1 <= age <= 5.5
            or not math.isfinite(receipt)
            or not 0 <= receipt <= 5.5
        ):
            raise RuntimeError(f'Invalid/expired feedback during owner hold: {topic}')
        if topic in ODOM:
            if item['frame'] != 'odom' or item['child'] != 'base_link':
                raise RuntimeError(f'Unexpected odometry frames: {topic}')
            if not all(math.isfinite(v) for v in (*item['pose'], *item['velocity'])):
                raise RuntimeError('Nonfinite odometry during owner hold')
        elif not item['frame']:
            raise RuntimeError('Insufficient finite scan returns')
    scan = samples[SCAN]
    return (
        scan['valid_ranges'] >= 20
        and now_ros - scan['stamp'] <= scan_timeout
        and now_mono - scan['receipt'] <= scan_timeout
    )


class PauseEvents:
    """Tail only this run's owner logs; partial lines cannot create a hold."""

    def __init__(self, directory):
        self.directory = Path(directory)
        self.offsets = {}
        self.active = None
        self.init_serial = 0

    def lines(self, name):
        path = self.directory / name
        if not path.exists():
            return
        with path.open() as stream:
            stream.seek(self.offsets.get(name, 0))
            while line := stream.readline():
                if not line.endswith('\n'):
                    break
                self.offsets[name] = stream.tell()
                yield line

    def poll(self):
        for line in self.lines('motion.log'):
            if 'FEEDBACK_RECOVERING: zero commands; 5 s budget' in line:
                self.init_serial += 1
                self.active = dict(
                    key=('initialization', self.init_serial), owns_publisher=True, budget_s=5
                )
            elif 'FEEDBACK_RECOVERED:' in line:
                self.active = None
        for line in self.lines('points.jsonl'):
            row = json.loads(line)
            event = row.get('event')
            if event == 'feedback_pause_started':
                budget = row.get('budget_s')
                if (
                    isinstance(budget, bool)
                    or not isinstance(budget, (int, float))
                    or not math.isfinite(budget)
                    or not 0 < budget <= 5
                    or type(row.get('pause_id')) is not int
                    or row['pause_id'] <= 0
                    or type(row.get('owns_publisher', True)) is not bool
                ):
                    raise RuntimeError('Invalid owner feedback hold contract')
                self.active = dict(
                    key=('route', row['pause_id']),
                    owns_publisher=row.get('owns_publisher', True),
                    budget_s=budget,
                )
            elif event == 'feedback_pause_completed':
                key = ('route', row.get('pause_id'))
                if self.active and self.active['key'] == key:
                    self.active = None
        return self.active


class SupervisorPause:
    """Permit stale samples only while the actual command owner proves a zero hold."""

    def __init__(self, directory):
        self.events = PauseEvents(directory)
        self.directory = Path(directory)
        self.key, self.started, self.command_gids = None, None, None
        self.acknowledged = False

    def report(self, event, **data):
        with (self.directory / 'feedback_wait.jsonl').open('a') as stream:
            stream.write(json.dumps(dict(time=time.time(), event=event, **data)) + '\n')

    def observe(self, probe, expected, scan_timeout):
        active = self.events.poll()
        if active is None:
            self.key, self.started, self.command_gids = None, None, None
            self.acknowledged = False
            return probe.snapshot(stopped=False, expected=expected, scan_timeout=scan_timeout)
        now = time.monotonic()
        if self.key != active['key']:
            self.key, self.started, self.command_gids = active['key'], now, None
            self.acknowledged = False
            self.report('owner_hold_seen', key=self.key)
        if now - self.started >= active['budget_s']:
            raise RuntimeError('Owner feedback hold exceeded 5 seconds')
        last = probe.last_command
        acknowledged = last is not None and all(
            math.isfinite(v) and abs(v) < 1e-9 for v in last['values']
        )
        if active['owns_publisher']:
            acknowledged = acknowledged and 0 <= now - last['receipt'] <= 0.2
        if not acknowledged and (self.acknowledged or now - self.started >= 0.2):
            raise RuntimeError('Owner feedback hold lost its zero acknowledgement')
        if not acknowledged:
            # DDS may deliver the log before its zero: no grace before acknowledgement.
            return probe.snapshot(stopped=False, expected=expected, scan_timeout=scan_timeout)
        self.acknowledged = True
        current = probe.paused_snapshot(expected=expected, scan_timeout=scan_timeout)
        gids = tuple(row['gid'] for row in current['cmd_endpoints'])
        if not active['owns_publisher'] and gids:
            raise RuntimeError('Unexpected publisher during released owner hold')
        if active['owns_publisher'] and len(gids) != 1:
            if not gids and now - self.started < 0.2 and self.command_gids is None:
                return probe.snapshot(stopped=False, expected=expected, scan_timeout=scan_timeout)
            raise RuntimeError('Owner zero publisher missing during feedback hold')
        if self.command_gids is not None and gids != self.command_gids:
            raise RuntimeError('Command owner changed during feedback hold')
        self.command_gids = gids
        return current
