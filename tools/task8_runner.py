#!/usr/bin/env python3
"""Scene4 hardware runner; requires explicit lidar-localized P15 handoff."""

import argparse
from contextlib import contextmanager
from dataclasses import asdict
import fcntl
import json
import math
from pathlib import Path
import signal
import sys
import time


def backend_imports():
    from ament_index_python.packages import get_package_prefix

    backend = Path(get_package_prefix('distance_controller')) / 'lib/distance_controller'
    sys.path.insert(0, str(backend))


def emit(audit, event, **data):
    audit.write(json.dumps(dict(time=time.time(), event=event, **data), allow_nan=False) + '\n')
    audit.flush()


@contextmanager
def terminal_guard(console, action):
    if action.name != 'P01_after_turn':
        yield
        return
    previous = console.command_guard

    def checked(command):
        console.runner.moving_pose()
        console.runner.geometry()
        return previous(command) if previous else command

    checked((0.0, 0.0, 0.0))
    console.emit(
        'terminal_turn_policy',
        name=action.name,
        angle_rad=math.pi,
        swept_envelope_gate=False,
        laser_policy='observation_only',
    )
    console.command_guard = checked
    try:
        yield
    finally:
        console.command_guard = previous


def localized_first_edge(motion, origin, actual):
    """Connect a measured near-P15 pose; keep every later waypoint and heading."""
    from dataclasses import replace
    from motion_model import local_delta

    delta = local_delta(origin, actual)
    position = (delta['x_m'], delta['y_m'])
    heading_error = math.remainder(math.radians(delta['yaw_deg']) - motion.angle, 2 * math.pi)
    if math.dist(position, motion.points[0]) > 0.05 or abs(heading_error) > math.radians(8):
        raise RuntimeError('Localized first edge outside 5cm/8deg handoff region')
    target = motion.points[1]
    distance = math.dist(position, target)
    if distance <= 1e-9:
        raise RuntimeError('Localized first edge duplicates target')
    count = math.ceil(distance / 0.05)
    prefix = tuple(
        (
            position[0] + (target[0] - position[0]) * i / count,
            position[1] + (target[1] - position[1]) * i / count,
        )
        for i in range(count)
    )
    return replace(motion, points=(*prefix, *motion.points[1:]))


def run_actions(console, actions, cancellation, localized_start=False):
    from motion_model import Motion
    from task6_route_model import reconnect_route
    from teaching_path import turn_to_heading

    after_turn = True  # First path must reconnect to the fixed nominal P15 origin too.
    for index, action in enumerate(actions):
        cancellation.check()
        console.runner.feedback_pause.check()
        console.emit('route_action_started', index=index, name=action.name)
        if action.kind == 'turn':
            motion = (
                Motion('turn', angle=math.pi)
                if action.name == 'P01_after_turn'
                else turn_to_heading(console.book.origin, console.runner.pose(), action.heading_rad)
            )
        else:
            motion = action.path_motion()
            if after_turn:
                connector = (
                    localized_first_edge if index == 0 and localized_start else reconnect_route
                )
                motion = connector(motion, console.book.origin, console.runner.pose())
                console.emit('post_turn_reconnect', adjusted=asdict(motion))
        with terminal_guard(console, action):
            console.move(motion)
        cancellation.check()
        console.record(action.name)
        console.emit('route_action_completed', index=index, name=action.name)
        after_turn = action.kind == 'turn'
    cancellation.check()
    console.runner.pose()


def session(args, path, audit, actions, initialize):
    from motion_console import Console, TeachingBook
    from route_feedback_pause import RouteFeedbackPause
    from task6_route_runner import Cancellation, close_runner
    from wall_runtime import WallRunner

    runner = WallRunner(args.speed, alignment_wall='right')

    class BoundedCancellation(Cancellation):
        def __init__(self, owned):
            super().__init__(owned)
            self.deadline = time.monotonic() + args.max_seconds

        def check(self):
            super().check()
            if time.monotonic() >= self.deadline:
                raise RuntimeError('Task8 execution deadline exceeded')

    cancellation = BoundedCancellation(runner)
    handlers = {
        sig: signal.signal(sig, cancellation.request) for sig in (signal.SIGINT, signal.SIGTERM)
    }

    def emitter(event, **data):
        emit(audit, event, **data)

    try:
        try:
            runner.task8_emit = emitter
            runner.exclusive()  # Allow DDS discovery before immutable publisher binding.
            runner.feedback_pause = RouteFeedbackPause(runner, emitter, cancellation.check)
            cancellation.check()
            origin = initialize(
                runner, args.source_book, args.source_bag, args.endpoint, cancellation
            )
            if not all(math.isfinite(v) for v in (origin.x, origin.y, origin.yaw)):
                raise ValueError('P15 initialization returned nonfinite origin')
            runner.feedback_pause.check()
            cancellation.check()
            runner.observation_scan_timeout = 1.0
            book = TeachingBook(path, origin, runner.frame)
            console = Console(runner, book, cancellation, audit, args.turn_speed)
            console.record('P15')  # P01 would overwrite the fixed origin in TeachingBook.
            emitter('preparation_handoff', origin=asdict(origin), frame='P15')
            run_actions(
                console,
                actions,
                cancellation,
                localized_start=getattr(args, 'p15_start', 'precise') == 'localized',
            )
        finally:
            close_runner(runner)
        cancellation.check()
        complete = actions[-1].name == 'P01_after_turn'
        emitter(
            'route_completed' if complete else 'prefix_completed',
            scene=4,
            destination='P01' if complete else actions[-1].name,
            final_action=actions[-1].name,
            full_route_success=complete,
            exit_code=0,
        )
    finally:
        for sig, handler in handlers.items():
            signal.signal(sig, handler)


def arguments(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--p15-observation',
        choices=('two_wall_tls', 'front_heading_fixed_reference_side'),
        default='two_wall_tls',
    )
    parser.add_argument('--p15-start', choices=('precise', 'localized'), default='precise')
    parser.add_argument('--route-json', type=Path, required=True)
    parser.add_argument('--source-book', type=Path, required=True)
    parser.add_argument('--source-bag', type=Path, required=True)
    parser.add_argument('--forward-route', type=Path, required=True)
    parser.add_argument('--speed', type=float, default=0.06)
    parser.add_argument('--turn-speed', type=float, default=0.20)
    parser.add_argument('--max-seconds', type=float, default=600)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args(argv)
    for value, low, high in (
        (args.speed, 0.01, 0.12),
        (args.turn_speed, 0.02, 0.60),
        (args.max_seconds, 1, 600),
    ):
        if not math.isfinite(value) or not low <= value <= high:
            parser.error('speed/turn-speed/max-seconds outside bounded limits')
    return args


def main(argv=None):
    args = arguments(argv)
    try:
        backend_imports()
        # Import must fail closed before any ROS runner/publisher is created.
        from task8_p15_start import initialize
        from scene_route import load_scene
        from task8_route_model import validate_manifest

        actions = validate_manifest(json.loads(args.route_json.read_text()))
        forward = load_scene(args.forward_route, 2)
        args.endpoint = tuple(
            next(a for a in forward['actions'] if a['name'] == 'P15')['points'][-1]
        )
        if not args.source_book.is_file() or not args.source_bag.exists():
            raise ValueError('Explicit source book and bag must exist')
        path = (args.output or Path(f'task8_route_{time.time_ns()}.json')).resolve()
        if path.suffix != '.json' or not path.parent.is_dir():
            raise ValueError('Output must end .json in an existing directory')
        if path.exists() or path.with_suffix('.jsonl').exists():
            raise ValueError('Output already exists; choose a new session file')
        with open('/tmp/distance_controller_survey_return.lock', 'a') as lock:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            with path.with_suffix('.jsonl').open('x') as audit:
                emit(
                    audit,
                    'route_started',
                    scene=4,
                    frame='P15',
                    laser_policy='observation_only',
                    actions=[asdict(a) for a in actions],
                )
                try:
                    session(
                        args,
                        path,
                        audit,
                        actions,
                        lambda *values: initialize(
                            *values,
                            observation_policy=args.p15_observation,
                            start_mode=args.p15_start,
                        ),
                    )
                except (Exception, KeyboardInterrupt) as error:
                    emit(audit, 'route_failed', reason=str(error) or 'Cancelled', exit_code=2)
                    raise
        print('Stopped:', actions[-1].name, flush=True)
        return 0
    except (Exception, KeyboardInterrupt) as error:
        print('STOPPED:', str(error) or 'Cancelled', flush=True)
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
