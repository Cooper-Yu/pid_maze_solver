#!/usr/bin/env python3
"""Prepare/check one fresh run; only explicit run can start official scene2 motion."""

import argparse
from contextlib import contextmanager
from datetime import datetime
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time

from scene_route import load_scene, select_real
from task6_ops_runtime import Operation, read_json, write_json
from task6_preflight import Probe


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def git(repository, *args):
    return subprocess.check_output(
        ['git', '-C', str(repository), *args], text=True, timeout=10
    ).strip()


def package_paths(workspace, name):
    from ament_index_python.packages import get_package_prefix, get_package_share_directory

    source = workspace / 'src' / name
    if not (source / '.git').exists():
        raise ValueError(f'Expected a Git checkout: {source}')
    return (
        source,
        Path(get_package_prefix(name)) / 'lib' / name,
        Path(get_package_share_directory(name)),
    )


def provenance(workspace):
    packages = {}
    for name in ('pid_maze_solver', 'distance_controller', 'turn_controller'):
        source, library, share = package_paths(workspace, name)
        tracked = git(source, 'ls-files').splitlines()
        sources = {p: digest(source / p) for p in tracked if (source / p).is_file()}
        installed = {
            str(p.relative_to(library)): digest(p)
            for p in library.rglob('*')
            if p.is_file() and '__pycache__' not in p.parts and p.suffix != '.pyc'
        }
        aliases = (
            {'pid_maze_solver': 'maze_entry.py', 'task6_run': 'task6_ops.py'}
            if name == 'pid_maze_solver'
            else {}
        )
        for relative, sha in installed.items():
            candidate = source / 'tools' / aliases.get(relative, relative)
            if not candidate.is_file() and (source / 'tools' / (relative + '.py')).is_file():
                candidate = source / 'tools' / (relative + '.py')
            if relative.endswith('.py') or relative in aliases or candidate.is_file():
                if not candidate.is_file() or digest(candidate) != sha:
                    raise RuntimeError(f'Installed Python differs from checkout: {name}/{relative}')
        packages[name] = dict(
            source=str(source),
            library=str(library),
            share=str(share),
            commit=git(source, 'rev-parse', 'HEAD'),
            tracked_changes=git(source, 'status', '--porcelain', '--untracked-files=no'),
            source_sha256=sources,
            installed_sha256=installed,
        )
    return packages


@contextmanager
def lock(path):
    with open(path, 'a') as stream:
        fcntl.flock(stream, fcntl.LOCK_EX | fcntl.LOCK_NB)
        yield


def deadline(value):
    parsed = datetime.fromisoformat(value)
    if parsed.tzinfo is None:
        raise argparse.ArgumentTypeError(
            'Deadline must include timezone, e.g. 2026-10-22T14:00:00+08:00'
        )
    return parsed.timestamp()


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=('prepare', 'preflight', 'run'))
    parser.add_argument('--run-dir', type=Path, required=True)
    parser.add_argument('--workspace', type=Path, default=Path.home() / 'ros2_ws')
    parser.add_argument('--deadline', type=deadline)
    parser.add_argument('--max-seconds', type=int, default=1200)
    parser.add_argument('--destination', default='P12')
    parser.add_argument('--speed', type=float, default=0.04)
    parser.add_argument('--turn-speed', type=float, default=0.10)
    parser.add_argument('--start-position-tolerance', type=float, default=0.005)
    parser.add_argument(
        '--at-p01',
        action='store_true',
        help='Explicitly confirm robot is currently near P01, never a previous route endpoint',
    )
    args = parser.parse_args(argv)
    if not math.isfinite(args.speed) or not 0.01 <= args.speed <= 0.12:
        parser.error('speed must be finite .01..12 m/s')
    if not math.isfinite(args.turn_speed) or not 0.02 <= args.turn_speed <= 0.60:
        parser.error('turn-speed must be finite .02..60 rad/s')
    if not math.isfinite(args.start_position_tolerance) or not (
        0.005 <= args.start_position_tolerance <= 0.010
    ):
        parser.error('start-position-tolerance must be finite .005..010 m')
    if args.action == 'prepare' and args.deadline is None:
        parser.error('prepare requires an explicit timezone-aware --deadline')
    if not 30 <= args.max_seconds <= 1680:
        parser.error('max-seconds must be 30..1680 (recorder reserve included separately)')
    if args.action == 'run' and not args.at_p01:
        parser.error('run requires --at-p01; no old session may establish the start location')
    return args


def prepare(args):
    packages = provenance(args.workspace.resolve())
    frontend = packages['pid_maze_solver']
    source_route = Path(frontend['source']) / 'waypoints/waypoints_real.yaml'
    route_file = Path(frontend['share']) / 'waypoints/waypoints_real.yaml'
    if digest(source_route) != digest(route_file):
        raise RuntimeError('Installed route YAML differs from checkout')
    route = load_scene(route_file, 2)
    selected = select_real(route, args.destination, execute=True)
    support = Path(packages['distance_controller']['source']) / 'tools/task6_ops'
    support_files = {
        name: digest(support / name)
        for name in ('common.py', 'evidence_recorder.py', 'odom_reset_watch.py')
    }
    if args.deadline - time.time() < 120:
        raise ValueError('Booking deadline has less than 120 seconds remaining')
    entry = Path(frontend['library']) / 'pid_maze_solver'
    if not os.access(entry, os.X_OK):
        raise ValueError('Official installed entry is not executable')
    config = dict(
        schema=1,
        created_unix=time.time(),
        workspace=str(args.workspace.resolve()),
        deadline_unix=args.deadline,
        max_seconds=args.max_seconds,
        speed=args.speed,
        turn_speed=args.turn_speed,
        start_position_tolerance=getattr(args, 'start_position_tolerance', 0.005),
        destination=(
            'P15'
            if selected['actions'][-1]['name'] == 'P15_after_turn'
            else selected['actions'][-1]['name']
        ),
        waypoints=str(route_file),
        route_sha256=digest(route_file),
        entry=str(entry),
        support=str(support),
        support_sha256=support_files,
        packages=packages,
        laser_policy='observation_only',
        task6_acceptance=False,
    )
    args.run_dir.mkdir(parents=True, exist_ok=False)
    write_json(args.run_dir / 'config.json', config)
    write_json(args.run_dir / 'route_preview.json', selected)
    print('PREPARED; no motion:', args.run_dir)
    return 0


def verify_config(directory):
    config = read_json(directory / 'config.json')
    if config.get('schema') != 1 or time.time() >= config['deadline_unix'] - 90:
        raise RuntimeError('Invalid config or booking deadline reached')
    if not math.isfinite(config['speed']) or not 0.01 <= config['speed'] <= 0.12:
        raise RuntimeError('Invalid speed')
    if not math.isfinite(config['turn_speed']) or not 0.02 <= config['turn_speed'] <= 0.60:
        raise RuntimeError('Invalid turn speed')
    tolerance = config.setdefault('start_position_tolerance', 0.005)
    if (
        type(tolerance) not in (int, float)
        or not math.isfinite(tolerance)
        or not (0.005 <= tolerance <= 0.010)
    ):
        raise RuntimeError('Invalid start position tolerance')
    if not 30 <= config['max_seconds'] <= 1680:
        raise RuntimeError('Invalid maximum duration')
    if provenance(Path(config['workspace'])) != config['packages']:
        raise RuntimeError('Source, Git, or installed artifacts changed; prepare a new run')
    if digest(config['waypoints']) != config['route_sha256']:
        raise RuntimeError('Route YAML changed; prepare again')
    for name, sha in config['support_sha256'].items():
        if digest(Path(config['support']) / name) != sha:
            raise RuntimeError('Evidence support changed; prepare again')
    select_real(load_scene(config['waypoints'], 2), config['destination'], execute=True)
    return config


def checked_preflight(directory, config):
    with lock('/tmp/distance_controller_survey_return.lock'):
        pass
    probe = Probe()
    try:
        snapshot = probe.ready()
        write_json(directory / 'preflight.json', dict(wall_s=time.time(), **snapshot))
        return probe, snapshot
    except BaseException:
        probe.close()
        raise


def run(args):
    config = verify_config(args.run_dir)
    if (args.run_dir / 'run.started.json').exists():
        raise RuntimeError(
            'Run directory already used; no resume/rebase; prepare a new run near P01'
        )
    if any(p['tracked_changes'] for p in config['packages'].values()):
        raise RuntimeError(
            'Commit reviewed tracked changes before running; manifest is not a release'
        )
    domain = os.environ.get('ROS_DOMAIN_ID', '0')
    if not domain.isdigit() or not 0 <= int(domain) <= 232:
        raise ValueError('ROS_DOMAIN_ID must be numeric 0..232')
    domain = str(int(domain))
    with lock(f'/tmp/task6_operations_{os.getuid()}_{domain}.lock'):
        probe, snapshot = checked_preflight(args.run_dir, config)
        try:
            with (args.run_dir / 'run.started.json').open('x') as stream:
                json.dump(
                    dict(
                        wall_s=time.time(),
                        at_p01_confirmed=True,
                        gids=snapshot['gids'],
                        policy='fresh_precise_initialization_no_resume',
                    ),
                    stream,
                )
            return Operation(args.run_dir, config, probe, snapshot['gids']).run()
        finally:
            probe.close()


def main(argv=None):
    args = parse_args(argv)
    try:
        if args.action == 'prepare':
            return prepare(args)
        if args.action == 'run':
            return run(args)
        config = verify_config(args.run_dir)
        probe, _snapshot = checked_preflight(args.run_dir, config)
        probe.close()
        print('PREFLIGHT READY; no motion; physical P01 still requires explicit run confirmation')
        return 0
    except (OSError, ValueError, RuntimeError, subprocess.SubprocessError) as error:
        print('OPS REFUSED:', str(error), file=sys.stderr, flush=True)
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
