#!/usr/bin/env python3
"""Attach one audited same-epoch return; manual next by default, optional audited automatic next."""

import argparse
from dataclasses import asdict
import json
import math
import os
from pathlib import Path
import sys
import time
import uuid

from task6_ops import deadline, digest, git, lock
from task6_ops_runtime import Operation, read_json, write_json
from task6_preflight import ODOM, Probe
from task6_return_auto import AutoAdvance, source_digest, validate_full_route


def load_run(directory):
    """Require a completed owned outward run, closed evidence and unchanged control artifacts."""
    config, result = read_json(directory / 'config.json'), read_json(directory / 'result.json')
    cleanup = result.get('cleanup', {})
    if (
        not result.get('success')
        or not cleanup.get('zero_and_released')
        or not cleanup.get('evidence', {}).get('closed_metadata')
    ):
        raise ValueError('Outward run did not complete with verified stop and closed recording')
    recording = Path((directory / 'evidence/latest_session.txt').read_text().strip()).resolve()
    if (
        not recording.is_relative_to((directory / 'evidence').resolve())
        or not (recording / 'bag/metadata.yaml').is_file()
    ):
        raise ValueError('Outward recording metadata missing or not owned by this run')
    evidence = read_json(recording / 'result.json')
    if not evidence.get('closed_metadata') or evidence.get('exception'):
        raise ValueError('Outward recording close failed')
    started, checked = (
        read_json(directory / 'run.started.json'),
        read_json(directory / 'preflight.json'),
    )
    if started['gids'] != checked['gids'] or any(
        len(started['gids'].get(t, [])) != 1 for t in ODOM
    ):
        raise ValueError('Outward publisher evidence is inconsistent')
    for package in ('distance_controller', 'turn_controller'):
        artifact = config['packages'][package]
        for relative, sha in artifact['installed_sha256'].items():
            if digest(Path(artifact['library']) / relative) != sha:
                raise ValueError(
                    'Installed return/control artifact changed since outward run: ' + relative
                )
    library = Path(config['packages']['distance_controller']['library'])
    sys.path.insert(0, str(library))
    from route_return_plan import load_source

    source_path = directory / 'points.json'
    source = load_source(source_path)
    if source_path.with_suffix('.return_claim.json').exists():
        raise ValueError('Source already has a return claim; no restart or resume')
    return config, source, started['gids'], library


def audit_rows(path):
    if not path.exists():
        return []
    lines = path.read_text().splitlines(keepends=True)
    return [json.loads(line) for line in lines if line.endswith('\n')]


class ReturnOperation(Operation):
    def __init__(self, directory, config, probe, identities, source, source_path, library):
        super().__init__(directory, config, probe, identities)
        self.source, self.source_path, self.library = source, source_path, library
        self.fifo = None
        self.buffer = b''
        self.seen = set()
        self.busy, self.closing = False, False
        self.completed_count = 0
        self.auto = AutoAdvance(source, identities[ODOM[0]][0]) if config.get('auto_next') else None
        self.state = dict(
            ready=False, busy=False, closed=False, cursor=len(source.actions), last_request=None
        )

    def save_state(self, rows):
        if self.auto:
            self.auto.consume(rows)
        completed = [r for r in rows if r.get('event') == 'return_action_completed']
        if len(completed) > self.completed_count:
            self.busy = False
        self.completed_count = len(completed)
        attached = any(r.get('event') == 'return_session_started' for r in rows)
        self.state.update(
            ready=attached
            and not self.busy
            and not self.closing
            and not (self.auto and self.auto.complete),
            auto_next=self.auto is not None,
            participant_binding=self.auto.context_binding if self.auto else None,
            participant_observed_at=self.auto.context_seen_at if self.auto else None,
            busy=self.busy,
            pid=os.getpid(),
            completed=self.completed_count,
            cursor=completed[-1]['cursor'] if completed else len(self.source.actions),
            last_completed=completed[-1] if completed else None,
            wall_s=time.time(),
        )
        write_json(self.directory / 'control.json', self.state)

    def next_dispatch_reason(self):
        if self.cancelled or self.closing:
            return 'Return is cancelled or closing'
        self.check_limits()
        if self.cancelled or self.closing:
            return 'Return is cancelled or closing'
        return None

    def request(self, request, rows, *, automatic=False):
        name, ident = request.get('command'), request.get('id')
        reason = None
        if (
            not isinstance(ident, str)
            or ident in self.seen
            or abs(time.time() - request.get('wall_s', 0)) > 5
        ):
            reason = 'Duplicate/stale request'
        elif name not in ('next', 'plan', 'status', 'quit'):
            reason = 'Unknown command'
        elif name == 'next' and self.auto and not automatic:
            reason = 'Automatic mode owns next; use quit to stop'
        elif name == 'next' and (not self.state['ready'] or self.busy or self.state['cursor'] == 0):
            reason = 'Not ready or an action is already pending'
        elif name in ('plan', 'status') and self.busy:
            reason = 'Wait for the current action to stop'
        if isinstance(ident, str):
            self.seen.add(ident)
        if reason is None and name == 'next':
            reason = self.next_dispatch_reason()
        if reason is None:
            self.before_dispatch(name)
            if name == 'next':
                if self.auto:
                    self.auto.mark_requested()
                self.busy = True
            if name == 'quit':
                self.closing = True
            self.runner.stdin.write(name + '\n')
            self.runner.stdin.flush()
        self.state['last_request'] = dict(
            id=ident, command=name, accepted=reason is None, reason=reason
        )
        self.save_state(rows)

    def before_dispatch(self, name):
        """Subclass hook after admission and before writing any child command."""

    def observe_feedback(self, rows):
        return self.probe.snapshot(stopped=False, expected=self.identities)

    def receive(self, rows):
        try:
            data = os.read(self.fifo, 4096)
        except BlockingIOError:
            return
        self.buffer += data
        if len(self.buffer) > 8192:
            raise RuntimeError('Control FIFO message overflow')
        while b'\n' in self.buffer:
            line, self.buffer = self.buffer.split(b'\n', 1)
            self.request(json.loads(line), rows)

    def advance(self, rows):
        if not self.auto or self.auto.complete or self.closing or self.cancelled:
            return
        if self.auto.ready(self.probe):
            if source_digest(self.source_path) != self.source.source_hash:
                raise RuntimeError('Source book/audit changed before automatic next')
            self.check_limits()
            self.probe.snapshot(stopped=True, expected=self.identities)
            self.check_recording()
            self.check_limits()
            if self.cancelled or self.closing:
                return
            self.request(
                dict(id=str(uuid.uuid4()), command='next', wall_s=time.time()), rows, automatic=True
            )

    def finish_wait(self, audit):
        """After audited completion, verify stopped evidence and bound normal child exit."""
        if self.auto is None or not self.auto.complete:
            raise RuntimeError('Return exit wait requires audited complete return')
        self.phase = 'return_exit_wait'
        deadline = time.monotonic() + 2.0
        while True:
            self.probe.pump()
            self.check_limits()
            self.probe.snapshot(stopped=True, expected=self.identities)
            self.check_recording()
            rows = audit_rows(audit)
            self.save_state(rows)
            if not rows or rows[-1].get('event') != 'return_completed':
                raise RuntimeError('Completed return audit changed during child exit')
            if source_digest(self.source_path) != self.source.source_hash:
                raise RuntimeError('Source book/audit changed during return exit')
            command = self.probe.last_command
            if (
                command is None
                or self.auto.requested_at is None
                or command['receipt'] < self.auto.requested_at
                or not all(math.isfinite(v) and abs(v) < 1e-9 for v in command['values'])
            ):
                raise RuntimeError('Completed return has no observed final zero command')
            if time.monotonic() >= deadline:
                raise RuntimeError('Return child did not exit normally within 2 seconds')
            code = self.runner.poll()
            if code is not None:
                if code != 0:
                    raise RuntimeError(f'Completed return child exited {code}')
                return

    def build_command(self):
        return [
            str(self.library / 'route_return_runner'),
            '--source',
            str(self.source_path),
            '--confirm-same-odom',
            self.source.session_id,
            '--odom-publisher-gid',
            self.identities[ODOM[0]][0],
            '--output',
            str(self.directory / 'return_points.json'),
            '--laser-log-only',
        ]

    def execute(self):
        import subprocess

        self.start_evidence()
        command = self.build_command()
        write_json(self.directory / 'motion_command.json', command)
        fifo_path = self.directory / 'control.fifo'
        os.mkfifo(fifo_path, 0o600)
        self.fifo = os.open(fifo_path, os.O_RDWR | os.O_NONBLOCK)
        if self.auto:
            self.auto.before_child(self.probe.return_context())
        with (self.directory / 'motion.log').open('x') as output:
            self.runner = subprocess.Popen(
                command,
                stdin=subprocess.PIPE,
                stdout=output,
                stderr=subprocess.STDOUT,
                text=True,
                start_new_session=True,
                env={**os.environ, 'RMW_IMPLEMENTATION': 'rmw_fastrtps_cpp'} if self.auto else None,
            )
        audit = self.directory / 'return_points.jsonl'
        while self.runner.poll() is None:
            self.probe.pump()
            self.check_limits()
            rows = audit_rows(audit)
            observed = self.observe_feedback(rows)
            self.check_recording()
            self.save_state(rows)
            if self.auto and self.auto.complete:
                break
            if self.runner.poll() is not None:
                break
            if self.auto:
                self.auto.observe_context(self.probe.return_context(), child_alive=True)
                self.auto.observe_publishers(observed)
            self.receive(rows)
            self.advance(rows)
        rows = audit_rows(audit)
        self.save_state(rows)
        if self.auto:
            self.finish_wait(audit)
        if self.runner.returncode != 0 or not rows or rows[-1].get('event') != 'return_completed':
            raise RuntimeError('Return closed without complete original-P01 return; no resume')

    def run(self):
        try:
            return super().run()
        finally:
            if self.fifo is not None:
                os.close(self.fifo)
            self.state.update(ready=False, busy=False, closed=True, wall_s=time.time())
            write_json(self.directory / 'control.json', self.state)


def send(directory, command):
    state = read_json(directory / 'control.json')
    if state.get('closed') or time.time() - state['wall_s'] > 2:
        raise RuntimeError('Return wrapper is closed or stale')
    if command == 'next' and (not state.get('ready') or state.get('busy')):
        raise RuntimeError('Return is not ready for exactly one next')
    message = dict(command=command, id=str(uuid.uuid4()), wall_s=time.time())
    encoded = (json.dumps(message) + '\n').encode()
    fd = os.open(directory / 'control.fifo', os.O_WRONLY | os.O_NONBLOCK)
    try:
        if os.write(fd, encoded) != len(encoded):
            raise RuntimeError('Incomplete FIFO write')
    finally:
        os.close(fd)
    print('QUEUED', message['id'], command, '(check status for acceptance/completion)')


def start(args):
    config, source, gids, library = load_run(args.source_run_dir)
    if getattr(args, 'auto_next', False):
        validate_full_route(
            config,
            source,
            args.source_run_dir / 'points.json',
            audit_rows(args.source_run_dir / 'points.jsonl'),
        )
    if (
        args.deadline is None
        or args.deadline - time.time() < 120
        or not 30 <= args.max_seconds <= 1680
    ):
        raise ValueError('Require deadline with >=120s remaining and max-seconds 30..1680')
    domain = int(os.environ.get('ROS_DOMAIN_ID', '0'))
    if not 0 <= domain <= 232:
        raise ValueError('ROS_DOMAIN_ID outside 0..232')
    with lock(f'/tmp/task6_operations_{os.getuid()}_{domain}.lock'):
        with lock('/tmp/distance_controller_survey_return.lock'):
            pass
        probe = Probe()
        try:
            snapshot = probe.ready()
            if snapshot['gids'] != gids:
                raise RuntimeError(
                    'Current odometry GID differs from the completed source; no rebase'
                )
            args.return_dir.mkdir(parents=True, exist_ok=False)
            support = Path(config['support'])
            options = dict(
                support=str(support),
                deadline_unix=args.deadline,
                max_seconds=args.max_seconds,
                destination='P01',
                auto_next=getattr(args, 'auto_next', False),
            )
            manifest = dict(
                options=options,
                source_run=str(args.source_run_dir.resolve()),
                source_session=source.session_id,
                source_hash=source.source_hash,
                origin=asdict(source.origin),
                expected_gids=gids,
                preflight=snapshot,
                return_entry_sha256=digest(library / 'route_return_runner'),
                wrapper_sha256=digest(__file__),
                auto_helper_sha256=digest(Path(__file__).with_name('task6_return_auto.py')),
                wrapper_git=git(Path(__file__).resolve().parents[1], 'rev-parse', 'HEAD'),
                support_sha256={
                    n: digest(support / n)
                    for n in ('common.py', 'evidence_recorder.py', 'odom_reset_watch.py')
                },
            )
            write_json(args.return_dir / 'manifest.json', manifest)
            return ReturnOperation(
                args.return_dir,
                options,
                probe,
                gids,
                source,
                args.source_run_dir / 'points.json',
                library,
            ).run()
        finally:
            probe.close()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=('plan', 'start', 'next', 'status', 'quit'))
    parser.add_argument('--source-run-dir', type=Path)
    parser.add_argument('--return-dir', type=Path)
    parser.add_argument('--deadline', type=deadline)
    parser.add_argument('--max-seconds', type=int, default=900)
    parser.add_argument(
        '--auto-next',
        action='store_true',
        help='Advance only a completed full route; default is manual next',
    )
    args = parser.parse_args(argv)
    try:
        if args.auto_next and args.action != 'start':
            raise ValueError('--auto-next is valid only with start')
        if args.action in ('plan', 'start') and args.source_run_dir is None:
            raise ValueError('--source-run-dir required')
        if args.action != 'plan' and args.return_dir is None:
            raise ValueError('--return-dir required')
        if args.action == 'plan':
            _config, source, gids, _library = load_run(args.source_run_dir)
            print(
                json.dumps(
                    dict(
                        source_session=source.session_id,
                        actions=len(source.actions),
                        next=asdict(source.actions[-1]),
                        origin=asdict(source.origin),
                        gids=gids,
                    ),
                    indent=2,
                )
            )
            return 0
        if args.action == 'start':
            return start(args)
        if args.action == 'status':
            print(json.dumps(read_json(args.return_dir / 'control.json'), indent=2))
        else:
            send(args.return_dir, args.action)
        return 0
    except (OSError, ValueError, RuntimeError) as error:
        print('RETURN OPS REFUSED:', error, file=sys.stderr, flush=True)
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
