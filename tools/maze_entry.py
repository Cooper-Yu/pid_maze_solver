#!/usr/bin/env python3
"""Scene-aware official entry; validate first, then run one owned backend."""

import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time

import yaml
from scene_route import load_scene, select_real


def package_directory(package, kind):
    from ament_index_python.packages import get_package_prefix, get_package_share_directory

    if kind == 'share':
        return Path(get_package_share_directory(package))
    return Path(get_package_prefix(package)) / 'lib' / package


def default_file(scene):
    files = {
        1: 'waypoints_sim.yaml',
        2: 'waypoints_real.yaml',
        3: 'reverse_waypoints_sim.yaml',
        4: 'reverse_waypoints_real.yaml',
    }
    if scene not in files:
        raise ValueError('Unsupported scene')
    source = Path(__file__).resolve().parent.parent / 'waypoints'
    base = (
        source if source.is_dir() else package_directory('pid_maze_solver', 'share') / 'waypoints'
    )
    return base / files[scene]


def executable(package, name):
    path = package_directory(package, 'lib') / name
    if not path.is_file() or not os.access(path, os.X_OK):
        raise FileNotFoundError(f'Missing executable: {path}')
    return str(path)


def bounded_value(low, high):
    def parse(value):
        parsed = float(value)
        if not low <= parsed <= high:
            raise argparse.ArgumentTypeError(f'Expected {low}..{high}')
        return parsed

    return parse


def parse_args(argv):
    values = list(argv)
    ros_args = []
    if '--ros-args' in values:
        split = values.index('--ros-args')
        values, ros_args = values[:split], values[split:]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('scene', nargs='?', type=int, choices=(1, 2, 3, 4), default=1)
    parser.add_argument('--waypoints', '--route-file', dest='waypoints', type=Path)
    parser.add_argument('--dry-run', action='store_true')
    parser.add_argument('--destination')
    parser.add_argument(
        '--supervised',
        action='store_true',
        help='Experimental scene2 single actions; enforced laser guard',
    )
    parser.add_argument('--speed', type=bounded_value(0.01, 0.12))
    parser.add_argument('--turn-speed', type=bounded_value(0.02, 0.60))
    parser.add_argument('--start-position-tolerance', type=bounded_value(0.005, 0.010))
    parser.add_argument('--corner-placement', choices=('off', 'front', 'lidar'))
    parser.add_argument('--output', type=Path)
    parser.add_argument(
        '--p15-observation',
        choices=('two_wall_tls', 'front_heading_fixed_reference_side'),
        default='front_heading_fixed_reference_side',
    )
    parser.add_argument('--p15-start', choices=('precise', 'localized'), default='localized')
    parser.add_argument('--source-book', type=Path)
    parser.add_argument('--source-bag', type=Path)
    parser.add_argument('--forward-route', type=Path)
    parser.add_argument('--max-seconds', type=bounded_value(1, 600), default=600)
    args = parser.parse_args(values)
    if args.scene in (2, 3, 4) and ros_args:
        parser.error(f'Scene {args.scene} does not accept unchecked ROS overrides')
    if args.scene in (1, 3) and any(
        (
            args.destination,
            args.speed,
            args.turn_speed,
            args.output,
            args.supervised,
            args.start_position_tolerance is not None,
            args.corner_placement is not None,
        )
    ):
        parser.error(
            'Destination/speed/output/supervised/start-position-tolerance/corner-placement '
            'are scene 2 options'
        )
    if args.scene in (2, 4):
        # Match the reviewed hardware profile when using the course's bare scene-2 command.
        if args.speed is None:
            args.speed = 0.06
        if args.turn_speed is None:
            args.turn_speed = 0.20
        if args.start_position_tolerance is None:
            args.start_position_tolerance = 0.010
    if args.scene == 4:
        if args.supervised or args.corner_placement not in (None, 'off'):
            parser.error('Scene4 does not accept supervised/corner-placement overrides')
        if bool(args.source_book) != bool(args.source_bag):
            parser.error('Scene4 source book and bag must be supplied together')
    elif args.source_book or args.source_bag or args.forward_route:
        parser.error('P15 source options require scene4')
    if args.scene == 2 and args.corner_placement is None:
        args.corner_placement = 'off'
    return args, ros_args


def send_owned(process, signum):
    try:
        os.killpg(process.pid, signum)
    except ProcessLookupError:
        pass


def run_owned(command):
    """Forward cancellation to this process group; never report cancellation as success."""
    cancelled = []
    previous = {}

    def cancel(signum, _frame):
        if not cancelled:
            cancelled.append((signum, time.monotonic()))

    for signum in (signal.SIGINT, signal.SIGTERM):
        previous[signum] = signal.signal(signum, cancel)
    process, sent = None, set()
    try:
        process = subprocess.Popen(command, start_new_session=True)
        while True:
            if cancelled:
                signum, started = cancelled[0]
                elapsed = time.monotonic() - started
                requested = (
                    signal.SIGKILL if elapsed >= 15 else signal.SIGTERM if elapsed >= 10 else signum
                )
                if requested not in sent:
                    send_owned(process, requested)
                    sent.add(requested)
            try:
                result = process.wait(timeout=0.2)
                return (
                    128 + cancelled[0][0] if cancelled else result if result >= 0 else 128 - result
                )
            except subprocess.TimeoutExpired:
                continue
    finally:
        if process is not None and process.poll() is None:
            send_owned(process, signal.SIGKILL)
            process.wait()
        for signum, handler in previous.items():
            signal.signal(signum, handler)


def backend_command(args, route_file, ros_args):
    if args.scene not in (1, 2, 3, 4):
        raise ValueError('Unsupported scene')
    if args.scene in (2, 3, 4) and ros_args:
        raise ValueError(f'Scene {args.scene} does not accept unchecked ROS overrides')
    if args.scene in (1, 3):
        return [
            executable('pid_maze_solver', 'pid_maze_solver_task5'),
            '--ros-args',
            '--params-file',
            str(route_file),
            *(ros_args[1:] if args.scene == 1 else ['-p', 'use_sim_time:=true']),
        ]
    if args.scene == 4:
        command = [
            executable('pid_maze_solver', 'task8_runner'),
            '--route-json',
            str(route_file),
            '--p15-start',
            args.p15_start,
            '--p15-observation',
            args.p15_observation,
            '--source-book',
            str(args.source_book),
            '--source-bag',
            str(args.source_bag),
            '--forward-route',
            str(args.forward_route or default_file(2)),
            '--speed',
            str(args.speed),
            '--turn-speed',
            str(args.turn_speed),
            '--max-seconds',
            str(args.max_seconds),
        ]
        if args.output is not None:
            command.extend(('--output', str(args.output)))
        return command
    command = [
        executable('distance_controller', 'task6_route_runner'),
        '--scene',
        '2',
        '--route-json',
        str(route_file),
        '--supervised' if getattr(args, 'supervised', False) else '--laser-log-only',
    ]
    for name, value in (
        ('--speed', args.speed),
        ('--turn-speed', args.turn_speed),
        ('--output', args.output),
        ('--start-position-tolerance', getattr(args, 'start_position_tolerance', 0.005)),
        ('--corner-placement', getattr(args, 'corner_placement', 'off')),
    ):
        if value is not None:
            command.extend((name, str(value)))
    return command


def main(argv=None):
    args, ros_args = parse_args(sys.argv[1:] if argv is None else argv)
    try:
        path = args.waypoints or default_file(args.scene)
        route = load_scene(path, args.scene)
        if args.scene == 4:
            from task8_route_model import select_reverse

            if not args.dry_run and args.source_book is None:
                from task8_reference import resolve

                args.source_book, args.source_bag = resolve(args.forward_route or default_file(2))
            payload = select_reverse(route, args.destination or 'P01', execute=not args.dry_run)
        else:
            payload = (
                select_real(route, args.destination or 'P15', execute=not args.dry_run)
                if args.scene == 2
                else route
            )
        if args.dry_run:
            print(
                json.dumps(
                    dict(
                        dry_run=True,
                        source=str(path),
                        route=payload,
                        full_route_validated=route.get('validated'),
                        ros_args=ros_args,
                    ),
                    indent=2,
                    allow_nan=False,
                )
            )
            return 0
        with tempfile.TemporaryDirectory(prefix='pid_maze_scene_') as temporary:
            resolved = Path(temporary) / (
                'route.json' if args.scene in (2, 4) else 'parameters.yaml'
            )
            if args.scene in (2, 4):
                resolved.write_text(json.dumps(payload, allow_nan=False))
            else:
                resolved.write_text(
                    yaml.safe_dump({'pid_maze_solver': {'ros__parameters': route['parameters']}})
                )
            command = backend_command(args, resolved, ros_args)
            print(f'SCENE={args.scene} source={path}; backend owns motion', flush=True)
            return run_owned(command)
    except (OSError, ValueError, yaml.YAMLError) as error:
        print(f'SCENE_REJECTED: {error}', file=sys.stderr, flush=True)
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
