"""Small stopped-measurement translations; never replace the final start gate."""

from collections import deque
import math
import time

from task8_p15_start import current_sample
from task8_p15_localization import _qualify_stopped


def plan(estimate):
    x, y, a = (estimate[k] for k in ('x', 'y', 'yaw'))
    if (
        not all(math.isfinite(v) for v in (x, y, a))
        or math.hypot(x, y) > 0.15
        or abs(a) > math.radians(8)
    ):
        raise ValueError('Position acquisition outside 15cm/8deg region')
    distance = math.hypot(x, y)
    if distance <= 0.035:
        return None
    length = min(0.03, distance - 0.025)
    return dict(
        x=-(math.cos(a) * x + math.sin(a) * y) * length / distance,
        y=(math.sin(a) * x - math.cos(a) * y) * length / distance,
    )


def collect(runner, reference, cancellation, check_stage):
    samples = deque(maxlen=12)
    deadline = time.monotonic() + 20
    seen = None
    reason = 'awaiting stopped localized position'
    while time.monotonic() < deadline:
        cancellation.check()
        check_stage()
        pause = runner.feedback_pause
        pause.original_pump()
        pause.check_faults()
        if runner.node.count_publishers('/cmd_vel'):
            raise RuntimeError('Position sampling requires publisher count zero')
        if not pause.ready() or not all(
            s.sample and s.sample['stopped'] for s in pause.sources.values()
        ):
            samples.clear()
            continue
        try:
            runner.pose()
            row = current_sample(runner, reference)
            if row['scan_stamp'] == seen or (samples and row['time'] - samples[-1]['time'] < 0.10):
                continue
            seen = row['scan_stamp']
            samples.append(row)
            estimate = _qualify_stopped(
                list(samples), 0.15, math.radians(8), '15cm/8deg acquisition only'
            )
        except (RuntimeError, ValueError) as error:
            reason = str(error)
            continue
        cancellation.check()
        check_stage()
        return estimate
    raise RuntimeError('Position sampling expired: ' + reason)


def adjust(runner, reference, cancellation, audit, path, emitter):
    from motion_console import Console, TeachingBook
    from motion_model import Motion

    anchor = None
    deadline = time.monotonic() + 90

    def check_stage():
        if time.monotonic() >= deadline:
            raise RuntimeError('P15 position 90s deadline')
        if anchor is not None:
            pose = runner.moving_pose()
            if math.hypot(pose.x - anchor.x, pose.y - anchor.y) > 0.07:
                raise RuntimeError('P15 position translation exceeds 7cm episode displacement')

    def guard(command):
        check_stage()
        runner.geometry()
        return command

    console = None
    for index in range(3):
        estimate = collect(runner, reference, cancellation, check_stage)
        emitter('p15_position_acquired', index=index, estimate=estimate)
        step = plan(estimate)
        if step is None:
            return
        if index == 2:
            raise RuntimeError('P15 two-position-step budget exhausted')
        if anchor is None:
            anchor = runner.pose()
            console = Console(
                runner,
                TeachingBook(path.with_name(path.stem + '_position.json'), anchor, runner.frame),
                cancellation,
                audit,
                0.12,
                guard,
            )
        emitter('p15_position_step', index=index, body_step=step)
        console.move(Motion('move', **step))
        check_stage()
