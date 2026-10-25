"""Bounded heading-only preparation, followed by the unchanged P15 localization."""

from collections import deque
from dataclasses import asdict
import math
import time

from task8_coarse_heading import observe, qualify
from task8_p15_start import mounting_values, require_same_mount


def collect(runner, reference, cancellation, check_stage, seconds=20):
    from rclpy.time import Time

    samples = deque(maxlen=12)
    deadline = time.monotonic() + seconds
    seen = None
    reason = 'awaiting reliable stopped side walls'
    while time.monotonic() < deadline:
        cancellation.check()
        check_stage()
        pause = runner.feedback_pause
        pause.original_pump()
        pause.check_faults()
        if runner.node.count_publishers('/cmd_vel'):
            raise RuntimeError('Coarse acquisition requires publisher count zero')
        if not pause.ready() or not all(
            s.sample and s.sample['stopped'] for s in pause.sources.values()
        ):
            samples.clear()
            continue
        try:
            pose = runner.pose()
            points, counts = runner.geometry()
            scan = runner.observer.scan
            if scan.header.frame_id != reference[3]:
                raise ValueError('Coarse acquisition laser frame changed')
            mount = runner.observer.buffer.lookup_transform(
                'base_link', reference[3], Time.from_msg(scan.header.stamp)
            ).transform
            require_same_mount(reference[2], mounting_values(mount))
            row = observe(points, counts, reference[0])
        except (RuntimeError, ValueError) as error:
            samples.clear()
            reason = str(error)
            continue
        now = time.monotonic()
        if runner.cached_stamp == seen or (samples and now - samples[-1]['time'] < 0.10):
            continue
        seen = runner.cached_stamp
        samples.append(
            dict(time=now, scan_stamp=seen, odom_stamp=runner.stamp, odom=asdict(pose), **row)
        )
        try:
            correction = qualify(list(samples))
        except ValueError as error:
            reason = str(error)
            continue
        cancellation.check()
        check_stage()
        return dict(correction=correction, samples=len(samples), walls=row['walls'])
    raise RuntimeError('Coarse heading acquisition expired: ' + reason)


def align(runner, reference, cancellation, audit, path, emitter):
    from motion_console import Console, TeachingBook
    from motion_model import Motion

    anchor = None
    deadline = time.monotonic() + 90

    def check_stage():
        if time.monotonic() >= deadline:
            raise RuntimeError('P15 coarse heading 90s deadline')
        if anchor is None:
            return
        pose = runner.moving_pose()
        if math.hypot(pose.x - anchor.x, pose.y - anchor.y) > 0.04:
            raise RuntimeError('P15 coarse heading overall drift exceeds 4cm')

    def guard(command):
        check_stage()
        runner.geometry()
        return command

    console = None
    exposure = 0.0
    for index in range(3):
        proposal = collect(runner, reference, cancellation, check_stage)
        emitter('p15_coarse_heading_observed', index=index, **proposal)
        if anchor is None:
            anchor = runner.pose()
        correction = proposal['correction']
        if abs(correction) <= math.radians(2):
            emitter('p15_coarse_heading_ready', aligned_start=False, position_verified=False)
            return
        if index == 2:
            raise RuntimeError('P15 coarse heading two-turn budget exhausted')
        turn = math.copysign(min(abs(correction), math.radians(15)), correction)
        exposure += abs(turn)
        if exposure > math.radians(30) + 1e-9:
            raise RuntimeError('P15 coarse heading total angle budget exhausted')
        if console is None:
            prep_path = path.with_name(path.stem + '_heading.json')
            console = Console(
                runner,
                TeachingBook(prep_path, anchor, runner.frame),
                cancellation,
                audit,
                0.12,
                guard,
            )
        console.move(Motion('turn', angle=turn))
        check_stage()
        emitter('p15_coarse_heading_turn_completed', index=index, requested_rad=turn)
        if abs(correction) <= math.radians(15):
            emitter('p15_coarse_heading_handoff', position_verified=False)
            return
    raise RuntimeError('P15 coarse heading did not converge')
