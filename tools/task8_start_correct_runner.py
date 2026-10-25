#!/usr/bin/env python3
"""Bounded, recorded P15 wall corrections; original start acceptance remains final."""

from collections import deque
from dataclasses import asdict
import argparse
import fcntl
import json
import math
from pathlib import Path
import signal
import time

from task8_p15_start import load_reference, mounting_values, require_same_mount, validate_source
from task8_start_correction import coarse_estimate, plan


def collect(runner, reference, cancellation, seconds=20):
    from rclpy.time import Time
    from wall_geometry import fit_wall

    samples = deque(maxlen=12)
    seen = None
    reason = 'awaiting stopped samples'
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        cancellation.check()
        pause = runner.feedback_pause
        pause.original_pump()
        pause.check_faults()
        if runner.node.count_publishers('/cmd_vel'):
            raise RuntimeError('Correction sampling requires publisher count zero')
        if not pause.ready() or not all(
            s.sample and s.sample['stopped'] for s in pause.sources.values()
        ):
            samples.clear()
            continue
        try:
            runner.pose()
            points, counts = runner.geometry()
            scan = runner.observer.scan
            if scan.header.frame_id != reference[3]:
                raise ValueError('Laser frame changed')
            mount = runner.observer.buffer.lookup_transform(
                'base_link', reference[3], Time.from_msg(scan.header.stamp)
            ).transform
            require_same_mount(reference[2], mounting_values(mount))
            estimate = coarse_estimate(
                reference[0],
                {side: asdict(fit_wall(points, counts, side)) for side in ('front', 'right')},
                reference[1],
            )
            if runner.cached_stamp == seen:
                continue
            seen = runner.cached_stamp
            samples.append(
                dict(
                    time=time.monotonic(),
                    scan_stamp=seen,
                    odom_stamp=runner.stamp,
                    odom=asdict(runner.latest[0]),
                    estimate={k: estimate[k] for k in ('x', 'y', 'yaw')},
                    heading_disagreement=estimate['heading_disagreement'],
                )
            )
            proposal = plan(list(samples))
        except (ValueError, RuntimeError) as error:
            reason = str(error)
            continue
        cancellation.check()
        pause.check_faults()
        if pause.ready():
            return proposal
    raise RuntimeError('Stopped correction sampling expired: ' + reason)


def correct(console, reference, cancellation, final_initialize, max_steps=12):
    from motion_model import Motion

    distance = angle = 0.0
    for index in range(max_steps + 1):
        cancellation.check()
        proposal = collect(console.runner, reference, cancellation)
        console.emit('p15_correction_proposal', index=index, **proposal)
        print('P15_CORRECTION', index, json.dumps(proposal), flush=True)
        if proposal['action'] == 'wait':
            origin = final_initialize()
            console.emit('p15_correction_verified', origin=asdict(origin), route_started=False)
            return origin
        if index == max_steps:
            raise RuntimeError('P15 correction step budget exhausted')
        distance += math.hypot(proposal['forward_m'], proposal['left_m'])
        angle += abs(proposal['angle_rad'])
        if distance > 0.15 + 1e-9 or angle > math.radians(12) + 1e-9:
            raise RuntimeError('P15 cumulative correction budget exhausted')
        motion = (
            Motion('turn', angle=proposal['angle_rad'])
            if proposal['action'] == 'turn'
            else Motion('move', x=proposal['forward_m'], y=proposal['left_m'])
        )
        console.move(motion)
        console.record('P15_correction_' + str(index))
    raise RuntimeError('P15 correction did not finish')


def main():
    from task8_runner import backend_imports

    backend_imports()
    from motion_console import Console, TeachingBook
    from route_feedback_pause import RouteFeedbackPause
    from task6_route_runner import Cancellation, close_runner
    from task8_p15_start import initialize
    from wall_runtime import WallRunner

    parser = argparse.ArgumentParser()
    parser.add_argument('--inputs', required=True)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    inputs = json.loads(Path(args.inputs).read_text())
    out = Path(args.output)
    recording = Path((out / 'evidence/latest_session.txt').read_text().strip())
    deadline = time.monotonic() + 180

    def health():
        data = json.loads((recording / 'health.json').read_text())
        if not -1 <= time.time() - data['unix_time'] <= 10:
            raise RuntimeError('Recorder health stale')
        required = (
            '/scan_filtered',
            '/odometry/filtered',
            '/rosbot_xl_base_controller/odom',
            '/image_raw/compressed',
        )
        if any(data['message_counts'].get(k, 0) <= 0 for k in required):
            raise RuntimeError('Recorder required topics absent')

    with open('/tmp/distance_controller_survey_return.lock', 'a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        runner = WallRunner(0.02)

        class Bounded(Cancellation):
            def check(self):
                super().check()
                health()
                if time.monotonic() >= deadline or time.time() >= 1792961400:
                    raise RuntimeError('P15 correction deadline')

        cancellation = Bounded(runner)
        handlers = {
            s: signal.signal(s, cancellation.request) for s in (signal.SIGINT, signal.SIGTERM)
        }
        try:
            runner.exclusive()
            health()
            book = validate_source(inputs['book'], inputs['bag'], inputs['endpoint'])
            reference = load_reference(book, inputs['bag'], inputs['endpoint'])
            with (out / 'correction.jsonl').open('x') as audit:
                console = Console(
                    runner,
                    TeachingBook(out / 'correction.json', runner.pose(), runner.frame),
                    cancellation,
                    audit,
                    0.08,
                )
                runner.feedback_pause = RouteFeedbackPause(runner, console.emit, cancellation.check)
                runner.observation_scan_timeout = 1.0
                runner.task8_emit = console.emit
                origin = correct(
                    console,
                    reference,
                    cancellation,
                    lambda: initialize(
                        runner, inputs['book'], inputs['bag'], inputs['endpoint'], cancellation
                    ),
                )
                (out / 'verified_origin.json').write_text(json.dumps(asdict(origin)))
                print('P15_STRICT_VERIFIED; no route started', flush=True)
        finally:
            close_runner(runner)
            for sig, handler in handlers.items():
                signal.signal(sig, handler)


if __name__ == '__main__':
    main()
