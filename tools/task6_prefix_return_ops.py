#!/usr/bin/env python3
"""Manual return from an independently audited completed, stopped route prefix."""

import argparse
from dataclasses import asdict
import importlib
import json
import os
from pathlib import Path
import sys
import time

from task6_ops import deadline, digest, git, lock
from task6_ops_runtime import read_json, write_json
from task6_preflight import ODOM, SCAN, Probe
from task6_return_ops import ReturnOperation, send
from task6_return_auto import AutoAdvance


def load_prefix(directory):
    """Use the separate prefix validator; never rewrite or promote the failed run."""
    directory = directory.resolve()
    config = read_json(directory / 'config.json')
    for package in ('distance_controller', 'turn_controller'):
        artifact = config['packages'][package]
        for relative, sha in artifact['installed_sha256'].items():
            if digest(Path(artifact['library']) / relative) != sha:
                raise ValueError(
                    'Installed control artifact changed since outward run: ' + relative
                )
    library = Path(config['packages']['distance_controller']['library']).resolve()
    required = (
        'route_prefix_return_runner',
        'route_prefix_return_plan.py',
        'route_prefix_return_runtime.py',
        'route_return_runner.py',
    )
    if any(not (library / name).is_file() for name in required):
        raise ValueError('Separate prefix return backend is not installed')
    if digest(library / 'route_return_runner.py') != digest(library / 'route_return_runner'):
        raise ValueError('Shared runner import differs from original return executable')
    sys.path.insert(0, str(library))
    module = importlib.import_module('route_prefix_return_plan')
    if Path(module.__file__).resolve() != (library / 'route_prefix_return_plan.py').resolve():
        raise ValueError('Prefix validator does not match configured installed library')
    source, metadata = module.load_prefix_run(directory)
    gids = metadata['gids']
    if set(gids) != set(ODOM) or any(len(gids[topic]) != 1 for topic in ODOM):
        raise ValueError('Prefix requires one recorded publisher for each odometry topic')
    if metadata['source_hash'] != source.source_hash:
        raise ValueError('Prefix metadata source hash mismatch')
    if (directory / 'points.return_claim.json').exists():
        raise ValueError('Source already claimed; no restart or resume')
    return config, source, metadata, library


class PrefixReturnOperation(ReturnOperation):
    """Keep bounded recording, manual FIFO, cancellation and final-stop verification."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        # Reuse only the append-only event parser, never its automatic dispatch policy.
        self.prefix_audit = AutoAdvance(self.source, self.identities[ODOM[0]][0])
        self.next_dispatched = False
        self.scan_wait_since = None
        self.scan_wait_stamp = None
        self.scan_wait_count = 0
        self.scan_wait_fault = None

    def stopped_boundary(self):
        audit = self.prefix_audit
        return (
            audit.attached
            and not audit.requested
            and not audit.started
            and not audit.complete
            and not audit.failure
            and not self.scan_wait_fault
            and not self.cancelled
            and not self.closing
        )

    def update_scan_wait(self, observed):
        if self.scan_wait_fault:
            raise RuntimeError(self.scan_wait_fault)
        now = time.monotonic()
        stamp = observed['samples'][SCAN]['stamp']
        if not observed['scan_fresh'] and self.scan_wait_since is None:
            self.scan_wait_since, self.scan_wait_stamp = now, stamp
            self.scan_wait_count = 0
        if self.scan_wait_since is None:
            return
        if now - self.scan_wait_since >= 2.0:
            self.scan_wait_fault = 'Stopped scan wait exceeded 2 seconds; no retry'
            raise RuntimeError(self.scan_wait_fault)
        if not observed['scan_fresh']:
            self.scan_wait_count = 0
            self.scan_wait_stamp = stamp
        elif stamp > self.scan_wait_stamp:
            self.scan_wait_stamp = stamp
            self.scan_wait_count += 1
            if self.scan_wait_count >= 3:
                self.scan_wait_since = None

    def observe_feedback(self, rows):
        if self.scan_wait_fault:
            raise RuntimeError(self.scan_wait_fault)
        self.prefix_audit.consume(rows)
        if not self.stopped_boundary():
            return super().observe_feedback(rows)
        observed = self.probe.snapshot(
            stopped=True, expected=self.identities, stopped_scan_wait=True
        )
        self.update_scan_wait(observed)
        return observed

    def next_dispatch_reason(self):
        reason = super().next_dispatch_reason()
        if reason is not None:
            return reason
        if not self.stopped_boundary():
            return 'No audited stopped boundary for next'
        observed = self.probe.snapshot(
            stopped=True, expected=self.identities, stopped_scan_wait=True
        )
        self.update_scan_wait(observed)
        if self.scan_wait_since is not None:
            return 'Waiting for three fresh scan timestamps; next refused'
        # Recheck all original motion-admission conditions immediately before dispatch.
        self.probe.snapshot(stopped=True, expected=self.identities)
        return super().next_dispatch_reason()

    def before_dispatch(self, name):
        if name == 'next':
            if not self.stopped_boundary() or self.scan_wait_since is not None:
                raise RuntimeError('Cannot dispatch next outside a ready stopped boundary')
            self.next_dispatched = True
            self.prefix_audit.requested = True
            self.prefix_audit.requested_at = time.monotonic()

    def save_state(self, rows):
        self.prefix_audit.consume(rows)
        super().save_state(rows)
        self.state.update(
            ready=self.state['ready'] and self.scan_wait_since is None and self.stopped_boundary(),
            waiting_for_fresh_scan=self.scan_wait_since is not None,
            scan_wait_started_monotonic=self.scan_wait_since,
            scan_wait_new_frames=self.scan_wait_count,
            next_dispatched=self.next_dispatched,
        )
        write_json(self.directory / 'control.json', self.state)

    def verify_final_stop(self):
        if (
            self.runner is not None
            and not self.next_dispatched
            and self.prefix_audit.attached
            and not self.prefix_audit.failure
            and not self.prefix_audit.started
            and self.probe.command_count == 0
            and self.probe.last_command is None
        ):
            if self.runner.poll() is None:
                raise RuntimeError('No-command stop requires owned child exit')
            observed = self.probe.wait_no_command_stop(self.identities)
            return dict(
                zero_and_released=False,
                stopped_and_released=observed is not None,
                stop_evidence=dict(
                    kind='stopped_without_motion_command',
                    zero_command_observed=False,
                    next_dispatched=False,
                    observation=observed,
                ),
            )
        return super().verify_final_stop()

    def build_command(self):
        return [
            str(self.library / 'route_prefix_return_runner'),
            '--source-run-dir',
            str(self.source_path.parent),
            '--confirm-same-odom',
            self.source.session_id,
            '--odom-publisher-gid',
            self.identities[ODOM[0]][0],
            '--raw-odom-publisher-gid',
            self.identities[ODOM[1]][0],
            '--output',
            str(self.directory / 'return_points.json'),
            '--speed',
            '0.03',
            '--turn-speed',
            '0.10',
            '--laser-log-only',
        ]


def start(args):
    config, source, metadata, library = load_prefix(args.source_run_dir)
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
            if snapshot['gids'] != metadata['gids']:
                raise RuntimeError('Current odometry GID differs from stopped prefix; no rebase')
            args.return_dir.mkdir(parents=True, exist_ok=False)
            support = Path(config['support'])
            options = dict(
                support=str(support),
                deadline_unix=args.deadline,
                max_seconds=args.max_seconds,
                destination='P01',
                auto_next=False,
                source_mode='completed_stopped_prefix',
            )
            manifest = dict(
                schema='task6_completed_stopped_prefix_return_v1',
                options=options,
                source_run=str(args.source_run_dir.resolve()),
                source_session=source.session_id,
                source_hash=source.source_hash,
                prefix_evidence=metadata,
                origin=asdict(source.origin),
                expected_gids=metadata['gids'],
                preflight=snapshot,
                backend_sha256={
                    name: digest(library / name)
                    for name in (
                        'route_prefix_return_runner',
                        'route_prefix_return_plan.py',
                        'route_prefix_return_runtime.py',
                        'route_return_runner.py',
                    )
                },
                wrapper_sha256=digest(__file__),
                inherited_wrapper_sha256=digest(Path(__file__).with_name('task6_return_ops.py')),
                wrapper_git=git(Path(__file__).resolve().parents[1], 'rev-parse', 'HEAD'),
                support_sha256={
                    name: digest(support / name)
                    for name in ('common.py', 'evidence_recorder.py', 'odom_reset_watch.py')
                },
                laser_policy='observation_only',
                task6_acceptance=False,
            )
            write_json(args.return_dir / 'manifest.json', manifest)
            return PrefixReturnOperation(
                args.return_dir,
                options,
                probe,
                metadata['gids'],
                source,
                args.source_run_dir.resolve() / 'points.json',
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
    parser.add_argument('--max-seconds', type=int, default=600)
    args = parser.parse_args(argv)
    try:
        if args.action in ('plan', 'start') and args.source_run_dir is None:
            raise ValueError('--source-run-dir required')
        if args.action != 'plan' and args.return_dir is None:
            raise ValueError('--return-dir required')
        if args.action == 'plan':
            _config, source, metadata, _library = load_prefix(args.source_run_dir)
            print(
                json.dumps(
                    dict(
                        source_mode='completed_stopped_prefix',
                        source_session=source.session_id,
                        source_hash=source.source_hash,
                        actions=len(source.actions),
                        prefix_names=metadata['prefix_names'],
                        next=asdict(source.actions[-1]),
                        origin=asdict(source.origin),
                        gids=metadata['gids'],
                        motion_started=False,
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
    except (OSError, ValueError, RuntimeError, ImportError, KeyError) as error:
        print('PREFIX RETURN REFUSED:', error, file=sys.stderr, flush=True)
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
