#!/usr/bin/env python3
"""Record and supervise one supervised manual current-frame backward step (no route recovery)."""

import argparse
import json
import math
import os
from pathlib import Path
import sys
import time

from task6_ops import deadline, digest, lock, provenance
from task6_ops_runtime import Operation, read_json, spawn, write_json
from task6_preflight import ODOM, Probe


def arguments(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--supervised-manual',
        action='store_true',
        required=True,
        help='Operator watches camera and can stop; NO automatic avoidance',
    )
    parser.add_argument('--run-dir', type=Path, required=True)
    parser.add_argument('--workspace', type=Path, default=Path.home() / 'ros2_ws')
    parser.add_argument('--distance', type=float, required=True, help='Positive BACKWARD meters')
    parser.add_argument('--speed', type=float, default=0.01)
    parser.add_argument('--deadline', type=deadline, required=True)
    parser.add_argument('--max-seconds', type=int, default=60)
    args = parser.parse_args(argv)
    if not math.isfinite(args.distance) or not 0 < args.distance <= 0.05:
        parser.error('distance must be finite, positive and <= .05 m backward')
    if not math.isfinite(args.speed) or not 0.01 <= args.speed <= 0.03:
        parser.error('speed must be finite .01..03 m/s')
    if not 30 <= args.max_seconds <= 120:
        parser.error('max-seconds must be 30..120; backend motion limit remains 10 seconds')
    return args


def unchanged(options):
    for path, expected in options['artifact_sha256'].items():
        if digest(path) != expected:
            raise RuntimeError('Step artifact changed after preflight: ' + path)


class LocalStepOperation(Operation):
    """Reuse evidence, cancellation, deadlines and stop cleanup, without route/source state."""

    def build_command(self):
        return [
            self.config['entry'],
            '--supervised-manual',
            '--distance',
            str(self.config['distance']),
            '--speed',
            str(self.config['speed']),
            '--odom-publisher-gid',
            self.identities[ODOM[0]][0],
            '--raw-odom-publisher-gid',
            self.identities[ODOM[1]][0],
            '--output',
            str(self.directory / 'local_points.json'),
        ]

    def verify_completion(self):
        rows = [
            json.loads(line)
            for line in (self.directory / 'local_points.jsonl').read_text().splitlines()
        ]
        starts = [row for row in rows if row.get('event') == 'local_step_started']
        ends = [row for row in rows if row.get('event') == 'local_step_completed']
        expected = {topic: self.identities[topic][0] for topic in ODOM}
        if (
            len(starts) != 1
            or len(ends) != 1
            or rows[-1] != ends[0]
            or rows.index(starts[0]) >= rows.index(ends[0])
            or any(row.get('event') in ('local_step_failed', 'route_completed') for row in rows)
            or starts[0].get('gids') != expected
            or starts[0].get('laser_policy') != 'observation_only'
            or starts[0].get('supervised_manual') is not True
            or any(row.get('historical_p01') is not False for row in (*starts, *ends))
            or any(row.get('task6_acceptance') is not False for row in (*starts, *ends))
            or any(row.get('distance_m') != self.config['distance'] for row in (*starts, *ends))
            or starts[0].get('speed_cap_m_s') != self.config['speed']
        ):
            raise RuntimeError('No matching supervised local_step_completed evidence')

    def execute(self):
        self.phase = 'local_step_evidence'
        self.start_evidence()
        self.check_recording()
        unchanged(self.config)
        self.probe.snapshot(stopped=True, expected=self.identities)
        command = self.build_command()
        write_json(self.directory / 'motion_command.json', command)
        self.check_limits()
        self.phase = 'current_frame_step'
        self.runner = spawn(command, self.directory / 'motion.log')
        while self.runner.poll() is None:
            self.probe.pump()
            self.check_limits()
            self.probe.snapshot(stopped=False, expected=self.identities)
            self.check_recording()
        self.check_limits()
        self.check_recording()
        if self.runner.returncode != 0:
            raise RuntimeError(f'Current-frame step exited {self.runner.returncode}')
        self.verify_completion()

    def run(self):
        result = super().run()
        record = read_json(self.directory / 'result.json')
        record.update(
            laser_policy='observation_only',
            source_mode='LOCAL_START',
            historical_p01=False,
            route_recovery=False,
            supervised_manual=True,
            automatic_avoidance=False,
        )
        write_json(self.directory / 'result.json', record)
        return result


def start(args):
    if args.deadline - time.time() < 120:
        raise ValueError('Deadline requires at least 120 seconds remaining')
    packages = provenance(args.workspace.resolve())
    if any(package['tracked_changes'] for package in packages.values()):
        raise RuntimeError('Tracked package changes require review and deployment first')
    backend = packages['distance_controller']
    library = Path(backend['library'])
    required = ('current_frame_step', 'current_frame_step_runtime.py')
    if any(name not in backend['installed_sha256'] for name in required):
        raise RuntimeError('Supervised current-frame backend is not installed and verified')
    support = Path(backend['source']) / 'tools/task6_ops'
    artifacts = {str(library / name): sha for name, sha in backend['installed_sha256'].items()}
    for path in (
        Path(__file__),
        Path(__file__).with_name('task6_ops_runtime.py'),
        Path(__file__).with_name('task6_preflight.py'),
        *(support / name for name in ('common.py', 'evidence_recorder.py', 'odom_reset_watch.py')),
    ):
        artifacts[str(path.resolve())] = digest(path)
    options = dict(
        entry=str(library / 'current_frame_step'),
        support=str(support),
        distance=args.distance,
        speed=args.speed,
        deadline_unix=args.deadline,
        max_seconds=args.max_seconds,
        destination='LOCAL_END',
        artifact_sha256=artifacts,
    )
    domain = int(os.environ.get('ROS_DOMAIN_ID', '0'))
    if not 0 <= domain <= 232:
        raise ValueError('ROS_DOMAIN_ID outside 0..232')
    with lock(f'/tmp/task6_operations_{os.getuid()}_{domain}.lock'):
        with lock('/tmp/distance_controller_survey_return.lock'):
            pass
        args.run_dir.mkdir(parents=True, exist_ok=False)
        probe = Probe()
        try:
            snapshot = probe.ready()
            write_json(
                args.run_dir / 'manifest.json',
                dict(
                    schema='task6_local_step_v1',
                    packages=packages,
                    options=options,
                    preflight=snapshot,
                    source_mode='LOCAL_START',
                    historical_p01=False,
                    supervised_manual=True,
                    automatic_avoidance=False,
                    laser_policy='observation_only',
                    task6_acceptance=False,
                ),
            )
            return LocalStepOperation(args.run_dir, options, probe, snapshot['gids']).run()
        finally:
            probe.close()


def main(argv=None):
    args = arguments(argv)
    try:
        return start(args)
    except (OSError, ValueError, RuntimeError, ImportError, KeyError) as error:
        print('LOCAL STEP REFUSED:', error, file=sys.stderr, flush=True)
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
