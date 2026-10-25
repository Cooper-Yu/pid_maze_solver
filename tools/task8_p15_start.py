"""Stopped P15 initialization from recorded walls and current-epoch feedback.

No controller or velocity publisher is created here. Historical odom is used only
to express the recorded physical scan in nominal P15, never as a live odom origin.
"""

from collections import deque
from dataclasses import asdict
import json
import math
from pathlib import Path
import sqlite3
import time

from task8_p15_localization import nominal_estimate, origin_from_current, qualify_samples


def validate_source(book_path, bag_path, endpoint):
    book_path, bag_path = Path(book_path), Path(bag_path)
    book = json.loads(book_path.read_text())
    events = [json.loads(x) for x in book_path.with_suffix('.jsonl').read_text().splitlines()]
    final = events[-1] if events else {}
    if (
        final.get('event'),
        final.get('destination'),
        final.get('final_action'),
        final.get('exit_code'),
    ) != ('route_completed', 'P15', 'P15_after_turn', 0):
        raise ValueError('Source route did not complete P15 half-turn')
    starts = [e for e in events if e.get('event') == 'route_started']
    if len(starts) != 1 or starts[0].get('scene') != 2 or book.get('frame') != 'odom':
        raise ValueError('Source route/frame identity invalid')
    actions = starts[0]['actions']
    if (
        len(actions) != 21
        or actions[-1]['name'] != 'P15_after_turn'
        or actions[-2]['name'] != 'P15'
    ):
        raise ValueError('Source full route evidence missing')
    if [e['name'] for e in events if e.get('event') == 'route_action_completed'] != [
        a['name'] for a in actions
    ]:
        raise ValueError('Source action completion evidence incomplete')
    if any(abs(a - b) > 1e-9 for a, b in zip(endpoint, actions[-2]['points'][-1])):
        raise ValueError('P15 nominal endpoint differs from source')
    if abs(math.remainder(actions[-1]['heading_rad'], 2 * math.pi)) > 1e-8:
        raise ValueError('Source P15 final heading invalid')
    session = bag_path.parent.parent
    closed = json.loads((session / 'result.json').read_text())
    if closed.get('closed_metadata') is not True or closed.get('exception') is not None:
        raise ValueError('Source bag was not closed cleanly')
    if not (bag_path.parent / 'metadata.yaml').is_file():
        raise ValueError('Source bag metadata missing')
    return book


def mounting_values(transform):
    t, q = transform.translation, transform.rotation
    return [t.x, t.y, t.z], [q.x, q.y, q.z, q.w]


def require_same_mount(reference, actual):
    t, q = reference
    u, v = actual
    if (len(t), len(q), len(u), len(v)) != (3, 4, 3, 4):
        raise ValueError('Incomplete P15 mounting transform')
    if not all(math.isfinite(x) for x in (*t, *q, *u, *v)):
        raise ValueError('Invalid P15 mounting transform')
    if any(abs(a - b) > 1e-6 for a, b in zip(t, u)):
        raise ValueError('P15 laser mounting translation changed')
    if any(abs(sum(x * x for x in quat) - 1) > 1e-6 for quat in (q, v)):
        raise ValueError('P15 laser mounting quaternion invalid')
    if abs(abs(sum(a * b for a, b in zip(q, v))) - 1) > 1e-6:
        raise ValueError('P15 laser mounting rotation changed')


def load_reference(book, bag_path, endpoint, observation_policy='two_wall_tls'):
    from rclpy.serialization import deserialize_message
    from rclpy.time import Time
    from tf2_msgs.msg import TFMessage
    from tf2_ros import Buffer
    from task8_p15_probe import reference_scan, source_pose
    from wall_geometry import scan_points, fit_wall

    scan, reference = reference_scan(book, bag_path)
    buffer = Buffer()
    with sqlite3.connect('file:' + str(Path(bag_path).resolve()) + '?mode=ro', uri=True) as con:
        row = con.execute('SELECT id FROM topics WHERE name=?', ('/tf_static',)).fetchone()
        if row is None:
            raise ValueError('Source static mounting TF missing')
        for (data,) in con.execute('SELECT data FROM messages WHERE topic_id=?', (row[0],)):
            for transform in deserialize_message(data, TFMessage).transforms:
                buffer.set_transform_static(transform, 'recorded_p15')
    mount = buffer.lookup_transform('base_link', scan.header.frame_id, Time()).transform
    expected = (reference['sensor_translation_m'], reference['sensor_rotation_xyzw'])
    require_same_mount(expected, mounting_values(mount))
    points, counts = scan_points(scan, mount)
    if observation_policy == 'front_heading_fixed_reference_side':
        from task8_wall_observation import observe

        walls = observe(points, counts)
    elif observation_policy == 'two_wall_tls':
        walls = {s: asdict(fit_wall(points, counts, s)) for s in ('front', 'right')}
    else:
        raise ValueError('Unknown P15 observation policy')
    return walls, source_pose(book, endpoint), expected, scan.header.frame_id


def current_sample(runner, reference):
    from rclpy.time import Time
    from wall_geometry import fit_wall

    walls, source, mount, scan_frame = reference
    points, counts = runner.geometry()
    scan = runner.observer.scan
    if scan.header.frame_id != scan_frame:
        raise ValueError('P15 laser frame changed')
    actual_mount = runner.observer.buffer.lookup_transform(
        'base_link', scan_frame, Time.from_msg(scan.header.stamp)
    ).transform
    require_same_mount(mount, mounting_values(actual_mount))
    if walls.get('_policy') == 'front_heading_fixed_reference_side':
        from task8_wall_observation import observe, require_patch
        from task8_front_side import estimate
        from task8_p15_localization import register_walls

        current = observe(points, counts, walls)
        observed = estimate(walls, current, source)
        relative = register_walls(walls, current)
        require_patch(walls, current, relative)
    else:
        current = {s: asdict(fit_wall(points, counts, s)) for s in ('front', 'right')}
        observed = nominal_estimate(walls, current, source)
    return dict(
        time=time.monotonic(),
        scan_stamp=runner.cached_stamp,
        odom_stamp=runner.stamp,
        odom=asdict(runner.latest[0]),
        estimate=observed,
        observation=current,
    )


def initialize(
    runner, source_book, source_bag, endpoint, cancellation, observation_policy='two_wall_tls'
):
    """Wait up to 20s for qualified fresh stopped batches; never publish velocity."""
    from action_plan import Pose

    cancellation.check()
    book = validate_source(source_book, source_bag, endpoint)
    reference = load_reference(book, source_bag, endpoint, observation_policy)
    pause = runner.feedback_pause
    samples = deque(maxlen=12)
    seen = None
    last_reason = 'Awaiting samples'
    deadline = time.monotonic() + 20
    emitter = getattr(runner, 'task8_emit', lambda *_args, **_kwargs: None)
    while time.monotonic() < deadline:
        cancellation.check()
        pause.original_pump()
        pause.check_faults()
        if runner.node.count_publishers('/cmd_vel'):
            raise RuntimeError('P15 initialization requires no velocity publishers')
        if not pause.ready() or not all(
            s.sample and s.sample['stopped'] for s in pause.sources.values()
        ):
            samples.clear()
            last_reason = 'Waiting for fresh stopped dual odometry and scan'
            continue
        if runner.latest is None:
            continue
        _, receipt, speed, wz = runner.latest
        age = (runner.node.get_clock().now().nanoseconds - runner.stamp) / 1e9
        if (
            time.monotonic() - receipt > 0.35
            or not -0.1 <= age <= 0.35
            or speed >= 0.01
            or wz >= 0.02
            or runner.frame != 'odom'
            or runner.twist_frame != 'base_link'
        ):
            samples.clear()
            last_reason = 'Waiting for fresh stopped follower odometry'
            continue
        try:
            sample = current_sample(runner, reference)
            if sample['scan_stamp'] == seen or (
                samples and sample['time'] - samples[-1]['time'] < 0.10
            ):
                continue
            seen = sample['scan_stamp']
            samples.append(sample)
            estimate = qualify_samples(list(samples))
        except (ValueError, RuntimeError) as error:
            last_reason = str(error)
            if len(samples) == 12:
                samples.popleft()
            continue
        cancellation.check()
        pause.check_faults()
        if not pause.ready():
            samples.clear()
            continue
        origin = origin_from_current(sample['odom'], estimate)
        emitter(
            'p15_start_verified',
            sample_count=len(samples),
            observation=sample.get('observation'),
            estimate=estimate,
            origin=origin,
            source_book=str(source_book),
            method=observation_policy + '_current_epoch',
            independent_heading_sources=1 if observation_policy != 'two_wall_tls' else 2,
            motion_published=False,
        )
        return Pose(**origin)
    raise RuntimeError('P15 initialization wait expired: ' + last_reason)
