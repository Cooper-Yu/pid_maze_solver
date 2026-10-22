#!/usr/bin/env python3
"""One explicitly requested step at a time in a fixed, new-epoch manual session."""

import argparse
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time
import uuid

from task6_local_step_ops import LocalStepOperation, unchanged
from task6_ops import deadline, digest, lock, provenance
from task6_ops_runtime import read_json, write_json
from task6_preflight import ODOM, Probe
from task6_return_ops import audit_rows


class SessionAudit:
    """Accept only ordered session evidence; never turn a completed step into automatic next."""

    def __init__(self, config, identities):
        self.config, self.identities = config, identities
        self.rows = []
        self.session_id = self.anchor = self.token = None
        self.completed = 0
        self.requested = self.active = self.closed = False
        self.used_tokens = set()

    @property
    def ready(self):
        return bool(self.token) and not (self.requested or self.active or self.closed)

    def consume(self, rows):
        if rows[: len(self.rows)] != self.rows:
            raise RuntimeError('Session audit changed or truncated')
        for row in rows[len(self.rows) :]:
            self.accept(row)
        self.rows = rows

    def accept(self, row):
        event = row.get('event')
        if event == 'session_failed':
            raise RuntimeError('Session fault: ' + str(row.get('reason')))
        if event == 'session_started':
            self.start_session(row)
        elif event in ('session_ready', 'step_started', 'step_completed', 'session_closed'):
            if not self.session_id or row.get('session_id') != self.session_id or self.closed:
                raise RuntimeError('Session identity/terminal mismatch')
            if event != 'session_closed' and row.get('anchor') != self.anchor:
                raise RuntimeError('Fixed session anchor changed')
            if event == 'session_ready':
                self.accept_ready(row)
            elif event == 'step_started':
                if not self.requested or self.active or row.get('index') != self.completed + 1:
                    raise RuntimeError('Unrequested or out-of-order step started')
                self.active = True
            elif event == 'step_completed':
                if not self.requested or not self.active or row.get('index') != self.completed + 1:
                    raise RuntimeError('Unstarted or out-of-order step completed')
                self.completed += 1
                self.requested = self.active = False
            else:
                if self.requested or self.active or row.get('completed_steps') != self.completed:
                    raise RuntimeError('Session closed with pending/inconsistent step')
                self.closed, self.token = True, None

    def accept_ready(self, row):
        token = row.get('ready_token')
        if (
            self.requested
            or self.active
            or self.token
            or row.get('next_index') != self.completed + 1
            or not isinstance(token, str)
            or not token
            or token in self.used_tokens
        ):
            raise RuntimeError('Unexpected READY token/index/state')
        self.token = token
        self.used_tokens.add(token)

    def start_session(self, row):
        if (
            self.session_id
            or not row.get('session_id')
            or row.get('gids') != {t: self.identities[t][0] for t in ODOM}
            or row.get('step_distance_m') != self.config['distance']
            or row.get('speed_cap_m_s') != self.config['speed']
            or row.get('laser_policy') != 'observation_only'
            or row.get('supervised_manual') is not True
            or row.get('historical_p01') is not False
            or row.get('task6_acceptance') is not False
        ):
            raise RuntimeError('Session start contract mismatch')
        anchor = row.get('anchor')
        if (
            not isinstance(anchor, dict)
            or set(anchor) != {'x', 'y', 'yaw'}
            or not all(isinstance(v, (int, float)) and math.isfinite(v) for v in anchor.values())
        ):
            raise RuntimeError('Invalid fixed session anchor')
        self.session_id, self.anchor = row['session_id'], anchor

    def mark_requested(self):
        if not self.ready:
            raise RuntimeError('No fresh READY for one manual step')
        token, self.token, self.requested = self.token, None, True
        return token


class LocalSessionOperation(LocalStepOperation):
    """Same recorder/stop lifecycle; FIFO requests cannot schedule future READY states."""

    def __init__(self, *args):
        super().__init__(*args)
        self.audit = SessionAudit(self.config, self.identities)
        self.fifo = None
        self.buffer = b''
        self.seen = set()
        self.closing = False
        self.close_started = None
        self.last_request = None

    def build_command(self):
        command = super().build_command()
        command[command.index('--distance')] = '--step-distance'
        command[command.index('--output') + 1] = str(self.directory / 'local_session.json')
        return command + ['--max-seconds', str(self.config['max_seconds'])]

    def save_state(self):
        write_json(
            self.directory / 'control.json',
            dict(
                ready=self.audit.ready and not self.closing and not self.cancelled,
                busy=self.audit.requested or self.audit.active,
                closed=self.audit.closed,
                closing=self.closing,
                session_id=self.audit.session_id,
                anchor=self.audit.anchor,
                ready_token=self.audit.token,
                next_index=self.audit.completed + 1,
                completed_steps=self.audit.completed,
                last_request=self.last_request,
                wall_s=time.time(),
                pid=os.getpid(),
                manual_supervision=True,
                task6_acceptance=False,
            ),
        )

    def request(self, message):
        name, ident = message.get('command'), message.get('id')
        stamp = message.get('wall_s')
        reason = None
        if (
            not isinstance(ident, str)
            or not ident
            or ident in self.seen
            or not isinstance(stamp, (int, float))
            or not math.isfinite(stamp)
            or abs(time.time() - stamp) > 5
        ):
            reason = 'Duplicate/invalid/stale request'
        elif name not in ('step', 'quit'):
            reason = 'Unknown command'
        elif self.cancelled or self.closing or self.audit.closed:
            reason = 'Session cancelled or closing'
        elif name == 'step' and (
            not self.audit.ready
            or message.get('ready_token') != self.audit.token
            or message.get('session_id') != self.audit.session_id
            or message.get('next_index') != self.audit.completed + 1
        ):
            reason = 'Not READY or token/session/index changed; request discarded'
        if isinstance(ident, str):
            self.seen.add(ident)
        if reason is None:
            self.check_limits()
            self.check_recording()
            if name == 'step':
                self.probe.snapshot(stopped=True, expected=self.identities)
                unchanged(self.config)
            self.check_limits()
            if self.closing or self.cancelled:
                raise RuntimeError('Session cancelled before dispatch')
            if name == 'step':
                command = 'step ' + self.audit.mark_requested()
            else:
                self.closing, self.close_started = True, time.monotonic()
                command = 'quit'
            self.runner.stdin.write(command + '\n')
            self.runner.stdin.flush()
        self.last_request = dict(id=ident, command=name, accepted=reason is None, reason=reason)
        self.save_state()

    def receive(self):
        try:
            self.buffer += os.read(self.fifo, 4096)
        except BlockingIOError:
            return
        if len(self.buffer) > 8192:
            raise RuntimeError('Manual request buffer limit exceeded')
        while b'\n' in self.buffer:
            line, self.buffer = self.buffer.split(b'\n', 1)
            try:
                message = json.loads(line)
                if not isinstance(message, dict):
                    raise ValueError('Request must be an object')
            except (ValueError, UnicodeError):
                raise RuntimeError('Malformed manual request; session stopped') from None
            self.request(message)

    def execute(self):
        self.start_evidence()
        self.check_recording()
        unchanged(self.config)
        self.probe.snapshot(stopped=True, expected=self.identities)
        command = self.build_command()
        write_json(self.directory / 'motion_command.json', command)
        os.mkfifo(self.directory / 'control.fifo', 0o600)
        self.fifo = os.open(self.directory / 'control.fifo', os.O_RDWR | os.O_NONBLOCK)
        self.check_limits()
        with (self.directory / 'motion.log').open('x') as output:
            self.runner = subprocess.Popen(
                command,
                stdin=subprocess.PIPE,
                stdout=output,
                stderr=subprocess.STDOUT,
                text=True,
                start_new_session=True,
            )
        self.phase = 'current_frame_session'
        audit_path = self.directory / 'local_session.jsonl'
        while self.runner.poll() is None:
            self.probe.pump()
            self.check_limits()
            self.audit.consume(audit_rows(audit_path))
            self.probe.snapshot(
                stopped=not (self.audit.requested or self.audit.active), expected=self.identities
            )
            self.check_recording()
            self.save_state()
            if self.close_started is not None and time.monotonic() - self.close_started > 8:
                raise RuntimeError('Session did not close within bounded quit wait')
            if self.runner.poll() is None:
                self.receive()
        self.audit.consume(audit_rows(audit_path))
        if (
            self.runner.returncode != 0
            or not self.closing
            or not self.audit.closed
            or not self.audit.rows
            or self.audit.rows[-1].get('event') != 'session_closed'
        ):
            raise RuntimeError('Manual session did not close normally; no resume')

    def verify_final_stop(self):
        if (
            self.runner is not None
            and self.runner.poll() is not None
            and self.probe.command_count == 0
            and self.probe.last_command is None
        ):
            observed = self.probe.wait_no_command_stop(self.identities)
            return dict(
                zero_and_released=False,
                stopped_and_released=observed is not None,
                stop_evidence=dict(kind='no_command_stopped_session', zero_command_observed=False),
            )
        return super().verify_final_stop()

    def run(self):
        try:
            result = super().run()
            data = read_json(self.directory / 'result.json')
            data.update(
                fixed_new_epoch_anchor=True,
                completed_steps=self.audit.completed,
                anchor=self.audit.anchor,
                session_id=self.audit.session_id,
            )
            write_json(self.directory / 'result.json', data)
            return result
        finally:
            if self.fifo is not None:
                os.close(self.fifo)
            self.audit.closed = self.closing = True
            self.audit.token = None
            self.save_state()


def send(directory, name):
    state = read_json(directory / 'control.json')
    if state.get('closed') or not -0.1 <= time.time() - state['wall_s'] <= 2:
        raise RuntimeError('Session is closed or stale')
    if name == 'step' and (not state.get('ready') or state.get('busy')):
        raise RuntimeError('Session is not READY for one manual step')
    message = dict(
        command=name,
        id=str(uuid.uuid4()),
        wall_s=time.time(),
        ready_token=state.get('ready_token'),
        session_id=state.get('session_id'),
        next_index=state.get('next_index'),
    )
    payload = (json.dumps(message) + '\n').encode()
    fd = os.open(directory / 'control.fifo', os.O_WRONLY | os.O_NONBLOCK)
    try:
        if os.write(fd, payload) != len(payload):
            raise RuntimeError('Incomplete request; inspect status and do not retry automatically')
    finally:
        os.close(fd)
    print('QUEUED', message['id'], name, '(inspect status; no automatic next)')


def start(args):
    if args.deadline is None or args.deadline - time.time() < 120 or not args.supervised_manual:
        raise ValueError('Require supervised-manual and booking deadline >=120 seconds away')
    packages = provenance(args.workspace.resolve())
    if any(p['tracked_changes'] for p in packages.values()):
        raise RuntimeError('Tracked changes require review and deployment')
    backend = packages['distance_controller']
    library = Path(backend['library'])
    if 'current_frame_session' not in backend['installed_sha256']:
        raise RuntimeError('Fixed-reference session backend is not installed and verified')
    support = Path(backend['source']) / 'tools/task6_ops'
    artifacts = {str(library / name): sha for name, sha in backend['installed_sha256'].items()}
    for path in (
        Path(__file__),
        *(
            Path(__file__).with_name(name)
            for name in ('task6_local_step_ops.py', 'task6_ops_runtime.py', 'task6_preflight.py')
        ),
        *(support / name for name in ('common.py', 'evidence_recorder.py', 'odom_reset_watch.py')),
    ):
        artifacts[str(path.resolve())] = digest(path)
    options = dict(
        entry=str(library / 'current_frame_session'),
        support=str(support),
        distance=args.step_distance,
        speed=args.speed,
        max_seconds=args.max_seconds,
        deadline_unix=args.deadline,
        destination='LOCAL_SESSION_CLOSED',
        artifact_sha256=artifacts,
    )
    domain = int(os.environ.get('ROS_DOMAIN_ID', '0'))
    if not 0 <= domain <= 232:
        raise ValueError('ROS_DOMAIN_ID outside 0..232')
    with lock(f'/tmp/task6_operations_{os.getuid()}_{domain}.lock'):
        with lock('/tmp/distance_controller_survey_return.lock'):
            pass
        args.session_dir.mkdir(parents=True, exist_ok=False)
        probe = Probe()
        try:
            snapshot = probe.ready()
            write_json(
                args.session_dir / 'manifest.json',
                dict(
                    schema='task6_manual_fixed_local_session_v1',
                    packages=packages,
                    options=options,
                    preflight=snapshot,
                    source_mode='LOCAL_START',
                    fixed_new_epoch_anchor=True,
                    supervised_manual=True,
                    automatic_avoidance=False,
                    laser_policy='observation_only',
                    historical_p01=False,
                    task6_acceptance=False,
                ),
            )
            return LocalSessionOperation(args.session_dir, options, probe, snapshot['gids']).run()
        finally:
            probe.close()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=('start', 'step', 'status', 'quit'))
    parser.add_argument('--session-dir', type=Path, required=True)
    parser.add_argument('--workspace', type=Path, default=Path.home() / 'ros2_ws')
    parser.add_argument('--supervised-manual', action='store_true')
    parser.add_argument('--step-distance', type=float, default=0.03)
    parser.add_argument('--speed', type=float, default=0.01)
    parser.add_argument('--deadline', type=deadline)
    parser.add_argument('--max-seconds', type=int, default=600)
    args = parser.parse_args(argv)
    if not math.isfinite(args.step_distance) or not 0 < args.step_distance <= 0.05:
        parser.error('step-distance must be positive and <=.05 m backward')
    if not math.isfinite(args.speed) or not 0.01 <= args.speed <= 0.03:
        parser.error('speed must be .01..03 m/s')
    if not 30 <= args.max_seconds <= 1680:
        parser.error('max-seconds must be 30..1680')
    try:
        if args.action == 'start':
            return start(args)
        if args.action == 'status':
            print(json.dumps(read_json(args.session_dir / 'control.json'), indent=2))
        else:
            send(args.session_dir, args.action)
        return 0
    except (OSError, ValueError, RuntimeError, ImportError, KeyError) as error:
        print('LOCAL SESSION REFUSED:', error, file=sys.stderr, flush=True)
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
