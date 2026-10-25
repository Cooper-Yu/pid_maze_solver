#!/usr/bin/env python3
"""Thirty-second P15 wall feedback correction; never starts the reverse route."""

from collections import deque
from dataclasses import asdict
import fcntl
import json
import math
from pathlib import Path
import signal
from statistics import median
import sys
import time
from task8_wall_servo import velocity
from task8_p15_start import current_sample


def maximum_drift(runner, start, anchors):
    actual = runner.moving_pose()
    drift = math.hypot(actual.x - start.x, actual.y - start.y)
    for topic, source in runner.feedback_pause.sources.items():
        before, current = anchors[topic], source.sample
        drift = max(drift, math.hypot(before['x'] - current['x'], before['y'] - current['y']))
    return drift


def servo(console, reference, cancellation, clock=time.monotonic):  # noqa: PLR0915
    # One bounded publisher lifecycle remains inside a single auditable try/finally.
    r = console.runner
    start = r.pose()
    anchors = {k: dict(s.sample) for k, s in r.feedback_pause.sources.items()}
    started = clock()
    held = None
    seen = None
    samples = deque(maxlen=3)
    travel_angle = 0.0
    previous = started
    r.assert_exclusive()
    r.velocity_pub = r.node.create_publisher(r.twist, '/cmd_vel', 10)
    try:
        while clock() - started <= 30:
            r.pump()
            cancellation.check()
            now = clock()
            drift = maximum_drift(r, start, anchors)
            if drift > 0.04:
                raise RuntimeError('Wall servo center drift exceeds 4cm')
            try:
                sample = current_sample(r, reference)
            except (ValueError, RuntimeError) as error:
                r.send((0.0, 0.0, 0.0))
                samples.clear()
                held = None
                console.emit('wall_servo_observation_wait', reason=str(error))
                continue
            if sample['scan_stamp'] != seen:
                if samples and sample['time'] - samples[-1]['time'] > 0.35:
                    samples.clear()
                samples.append(sample)
                seen = sample['scan_stamp']
            if len(samples) < 3:
                r.send((0.0, 0.0, 0.0))
                continue
            estimate = {k: median(s['estimate'][k] for s in samples) for k in ('x', 'y', 'yaw')}
            source_age = (r.node.get_clock().now().nanoseconds - seen) / 1e9
            receipt_age = time.monotonic() - r.observer.received
            if not -0.1 <= source_age <= 0.30 or not 0 <= receipt_age <= 0.30:
                r.send((0.0, 0.0, 0.0))
                samples.clear()
                held = None
                console.emit(
                    'wall_servo_freshness_wait', source_age_s=source_age, receipt_age_s=receipt_age
                )
                continue
            command, ready = velocity(
                estimate, drift, now - started, source_age, time.monotonic() - r.observer.received
            )
            # Never integrate a delayed callback into an unbounded command budget.
            travel_angle += abs(command[2]) * max(0.0, now - previous)
            previous = now
            if travel_angle > math.radians(8):
                raise RuntimeError('Wall servo requested angular budget exceeds 8deg')
            r.send(command)
            console.emit(
                'wall_servo_feedback',
                estimate=estimate,
                command=command,
                center_drift_m=drift,
                source_age_s=source_age,
                ready=ready,
            )
            if ready:
                held = held if held is not None else now
                if now - held >= 0.5:
                    return
            else:
                held = None
        raise RuntimeError('Wall servo 30s deadline')
    finally:
        console.stop_motion()


def main():
    from task8_runner import backend_imports

    backend_imports()
    from motion_console import Console, TeachingBook
    from route_feedback_pause import RouteFeedbackPause
    from task6_route_runner import Cancellation, close_runner
    from task8_p15_start import validate_source, load_reference, initialize
    from wall_runtime import WallRunner

    out = Path(sys.argv[1])
    recording = Path((out / 'evidence/latest_session.txt').read_text().strip())
    inputs = json.loads(
        Path('/home/user/task6_runs/task8_preflight_20261026_0233/inputs.json').read_text()
    )
    policy = 'front_heading_fixed_reference_side'
    deadline = time.monotonic() + 65
    with open('/tmp/distance_controller_survey_return.lock', 'a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        runner = WallRunner(0.01)

        class Bounded(Cancellation):
            def check(self):
                super().check()
                h = json.loads((recording / 'health.json').read_text())
                if (
                    time.time() - h['unix_time'] > 10
                    or time.monotonic() > deadline
                    or time.time() >= 1792961400
                ):
                    raise RuntimeError('Wall servo recorder/deadline fault')

        cancellation = Bounded(runner)
        handlers = {
            s: signal.signal(s, cancellation.request) for s in (signal.SIGINT, signal.SIGTERM)
        }
        try:
            runner.exclusive()
            reference = load_reference(
                validate_source(inputs['book'], inputs['bag'], inputs['endpoint']),
                inputs['bag'],
                inputs['endpoint'],
                policy,
            )
            with (out / 'servo.jsonl').open('x') as audit:
                console = Console(
                    runner,
                    TeachingBook(out / 'servo.json', runner.pose(), runner.frame),
                    cancellation,
                    audit,
                    0.02,
                )
                runner.feedback_pause = RouteFeedbackPause(runner, console.emit, cancellation.check)
                runner.task8_emit = console.emit
                console.record('P15_wall_servo_start')
                servo(console, reference, cancellation)
                console.record('P15_wall_servo_end')
                origin = initialize(
                    runner, inputs['book'], inputs['bag'], inputs['endpoint'], cancellation, policy
                )
                (out / 'verified_origin.json').write_text(json.dumps(asdict(origin)))
                print('P15_STRICT_VERIFIED; no route started', flush=True)
        finally:
            try:
                close_runner(runner)
            finally:
                for s, handler in handlers.items():
                    signal.signal(s, handler)


if __name__ == '__main__':
    main()
