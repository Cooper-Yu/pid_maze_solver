#!/usr/bin/env python3
"""Offline P01 scan statistics from a closed bag; no ROS node or command publisher.

Geometry/statistics follow distance_controller db60999. Bag receipt ordering is
only a proxy for controller callbacks/timer selection, never an exact trace.
"""

import argparse
from bisect import bisect_right
import json
import math
from pathlib import Path
import re
import sqlite3
import statistics

ODOM = '/odometry/filtered'
SCAN = '/scan_filtered'


def upper_median(values):
    return sorted(values)[len(values) // 2]


def dominant_wall(points, baseline=0.18):
    best = []
    if len(points) < 8:
        return best
    stride = (len(points) + 31) // 32
    for i in range(0, len(points), stride):
        for j in range(i + stride, len(points), stride):
            dx, dy = points[j][0] - points[i][0], points[j][1] - points[i][1]
            length = math.hypot(dx, dy)
            if length < baseline:
                continue
            selected = [
                p
                for p in points
                if abs(-dy * (p[0] - points[i][0]) + dx * (p[1] - points[i][1])) / length <= 0.012
            ]
            if len(selected) > len(best):
                best = selected
    return best if len(best) * 5 >= len(points) * 4 else []


def side(values, count):
    if len(values) < 6 or not count or len(values) * 2 < count:
        return dict(valid=False, median=None, mad=None, finite=len(values), rays=count)
    median = upper_median(values)
    mad = upper_median([abs(v - median) for v in values])
    return dict(
        valid=0.18 <= median <= 1.5 and mad <= 0.03,
        median=median,
        mad=mad,
        finite=len(values),
        rays=count,
    )


def heading(raw, count):
    points = dominant_wall(raw)
    result = dict(
        valid=False,
        angle=None,
        rms=None,
        span=None,
        finite=len(raw),
        inliers=len(points),
        rays=count,
    )
    if len(points) < 12 or len(points) * 2 < count:
        return result
    mx, my = sum(x for x, _ in points) / len(points), sum(y for _, y in points) / len(points)
    centered = [(x - mx, y - my) for x, y in points]
    xx = sum(x * x for x, _ in centered)
    xy = sum(x * y for x, y in centered)
    yy = sum(y * y for _, y in centered)
    angle = 0.5 * math.atan2(2 * xy, xx - yy)
    result['angle'] = angle
    if abs(angle) > 0.5235987755982988:
        return result
    c, s = math.cos(angle), math.sin(angle)
    along = [c * x + s * y for x, y in centered]
    rms = math.sqrt(sum((-s * x + c * y) ** 2 for x, y in centered) / len(points))
    span = max(along) - min(along)
    result.update(rms=rms, span=span, valid=span >= 0.18 and rms <= 0.012)
    return result


def rotation(transform):
    q, t = transform.rotation, transform.translation
    x, y, z, w = q.x, q.y, q.z, q.w
    if (
        not all(math.isfinite(v) for v in (x, y, z, w, t.x, t.y))
        or abs(x * x + y * y + z * z + w * w - 1) > 0.01
        or 1 - 2 * (x * x + y * y) < 0.99
    ):
        raise ValueError('Nonunit/nonplanar/nonfinite static mounting')
    return (
        (1 - 2 * (y * y + z * z), 2 * (x * y - z * w)),
        (2 * (x * y + z * w), 1 - 2 * (x * x + z * z)),
    )


def measure(scan, transform):
    matrix = rotation(transform)
    t = transform.translation
    right, rear, wall = [], [], []
    counts = [0, 0, 0]
    for i, radius in enumerate(scan.ranges):
        a = scan.angle_min + i * scan.angle_increment
        c, s = math.cos(a), math.sin(a)
        dx, dy = matrix[0][0] * c + matrix[0][1] * s, matrix[1][0] * c + matrix[1][1] * s
        angle = math.atan2(dy, dx)
        valid = math.isfinite(radius) and scan.range_min <= radius <= scan.range_max
        x, y = (t.x + radius * dx, t.y + radius * dy) if valid else (0, 0)
        if abs(angle + math.pi / 2) <= 0.5235987755982988:
            counts[2] += 1
            if valid and 0.18 <= -y <= 1.5:
                wall.append((x, y))
        if abs(abs(angle) - math.pi) <= 0.17453292519943295:
            counts[1] += 1
            if valid and -x > 0:
                rear.append(-x)
        if abs(abs(angle) - math.pi / 2) <= 0.17453292519943295 and angle < 0:
            counts[0] += 1
            if valid and y < 0:
                right.append(-y)
    r, b, h = side(right, counts[0]), side(rear, counts[1]), heading(wall, counts[2])
    if b['median'] is not None:
        b['valid'] = b['valid'] and b['median'] >= 0.22
    return dict(
        right=r,
        rear=b,
        heading=h,
        sample=[r['median'], b['median'], h['angle'] if h['valid'] else None],
        wall_windows_valid=r['valid'] and b['valid'],
    )


def summarize(samples):
    result = dict(retained=0, stable=False, median=[0.0, 0.0, 0.0], spread=[0.0, 0.0, 0.0])
    finite = [p for p in samples if all(v is not None and math.isfinite(v) for v in p)]
    if len(samples) != 15 or len(finite) < 12:
        return result
    anchor = [statistics.median(v) for v in zip(*finite)]
    kept = [
        p
        for p in finite
        if all(abs(v - a) <= limit for v, a, limit in zip(p, anchor, (0.02, 0.02, 0.034906585)))
    ]
    result['retained'] = len(kept)
    if len(kept) < 12:
        return result
    axes = list(zip(*kept))
    result.update(
        median=[statistics.median(v) for v in axes], spread=[max(v) - min(v) for v in axes]
    )
    result['stable'] = all(
        v <= limit for v, limit in zip(result['spread'], (0.01, 0.01, 0.034906585))
    )
    return result


def window(frames):
    result = summarize([f['sample'] for f in frames])
    result.update(
        first=frames[0]['index'],
        last=frames[-1]['index'],
        start=frames[0]['receipt'],
        end=frames[-1]['receipt'],
    )
    result['within_target'] = (
        result['stable']
        and abs(result['median'][0] - 0.26) <= 0.005
        and abs(result['median'][1] - 0.28) <= 0.005
        and abs(result['median'][2]) <= 0.008726646
    )
    return result


def comparisons(frames):
    groups = {}
    for frame in frames:
        if frame['eligible_proxy']:
            groups.setdefault(frame['segment'], []).append(frame)
    nonoverlap, sliding = [], []
    for group in groups.values():
        nonoverlap += [window(group[i : i + 15]) for i in range(0, len(group) - 14, 15)]
        sliding += [window(group[i : i + 15]) for i in range(len(group) - 14)]
    return dict(nonoverlap=nonoverlap, sliding=sliding)


def parse_cycles(text):
    cycles, current = [], None
    for line in text.splitlines():
        match = re.search(r'\[(\d+\.\d+)\]', line)
        if not match:
            continue
        stamp = float(match[1])
        if 'START_VERIFY: stopped hold' in line:
            current = dict(start=stamp, end=stamp + 5.1, logged_batches=[])
            cycles.append(current)
        elif current and 'START_VERIFY: retained=' in line:
            fields = dict(
                re.findall(
                    r'\b(retained|stable|right|rear|angle|spread_right|spread_rear|spread_angle)=([^ ]+)',
                    line,
                )
            )
            current['logged_batches'].append(
                dict(
                    time=stamp,
                    retained=int(fields['retained'].split('/')[0]),
                    stable=fields['stable'] == 'true',
                    median=[float(fields[k]) for k in ('right', 'rear', 'angle')],
                    spread=[
                        float(fields[k]) for k in ('spread_right', 'spread_rear', 'spread_angle')
                    ],
                )
            )
        elif current and any(
            s in line for s in ('Initial preparation fault:', 'START_REFINE:', 'START_VERIFIED:')
        ):
            current.update(end=stamp, terminal=line.strip())
            current = None
    if not cycles:
        raise ValueError('No timestamped START_VERIFY hold entry in motion log')
    return cycles


def stamp(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9


def odom_row(receipt, msg):
    p, q, v = msg.pose.pose.position, msg.pose.pose.orientation, msg.twist.twist
    return dict(
        receipt=receipt,
        stamp=stamp(msg),
        frame=msg.header.frame_id,
        child=msg.child_frame_id,
        x=p.x,
        y=p.y,
        yaw=math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)),
        velocity=[v.linear.x, v.linear.y, v.angular.z],
    )


def read_bag(bag, low, high):
    import yaml
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    from tf2_ros import Buffer

    metadata = yaml.safe_load((bag / 'metadata.yaml').read_text())['rosbag2_bagfile_information']
    buffer, records, statics = Buffer(), [], {}
    for rel in metadata['relative_file_paths']:
        path = (bag / rel).resolve(strict=True)
        if not path.is_relative_to(bag.resolve()) or path.suffix != '.db3':
            raise ValueError('Expected sqlite database inside closed bag')
        with sqlite3.connect(path.as_uri() + '?mode=ro', uri=True) as db:
            for ident, name, kind in db.execute('SELECT id,name,type FROM topics'):
                if name not in (SCAN, ODOM, '/tf_static'):
                    continue
                sql = 'SELECT timestamp,data FROM messages WHERE topic_id=?'
                args = [ident]
                if name != '/tf_static':
                    sql += ' AND timestamp BETWEEN ? AND ?'
                    args += [int(low * 1e9), int(high * 1e9)]
                for timestamp, data in db.execute(sql + ' ORDER BY timestamp', args):
                    msg = deserialize_message(data, get_message(kind))
                    if name == '/tf_static':
                        for tf in msg.transforms:
                            key = (tf.header.frame_id, tf.child_frame_id)
                            value = str(tf.transform)
                            if key in statics and statics[key] != value:
                                raise ValueError('Static mounting changed in recorded bag')
                            statics[key] = value
                            buffer.set_transform_static(tf, 'closed_bag')
                    else:
                        records.append((timestamp / 1e9, name, msg))
    return buffer, sorted(records, key=lambda item: item[0])


def annotate(frames, odom, start):
    times = [o['receipt'] for o in odom]
    anchor, hold, segment, last_stamp = None, None, 0, -math.inf
    for f in frames:
        i = bisect_right(times, f['receipt']) - 1
        recent = odom[i] if i >= 0 else None
        f['odom'] = recent
        blockers = []
        if (
            recent is None
            or f['receipt'] - recent['receipt'] > 0.5
            or not -0.1 <= f['receipt'] - recent['stamp'] <= 0.5
        ):
            blockers.append('odom_missing_stale')
        elif recent['frame'] != 'odom' or recent['child'] != 'base_link':
            blockers.append('odom_frame')
        elif math.hypot(*recent['velocity'][:2]) >= 0.003 or abs(recent['velocity'][2]) >= 0.01:
            blockers.append('odom_velocity')
        if anchor is not None and recent is not None:
            delta = math.atan2(
                math.sin(recent['yaw'] - anchor['yaw']), math.cos(recent['yaw'] - anchor['yaw'])
            )
            if (
                math.hypot(recent['x'] - anchor['x'], recent['y'] - anchor['y']) > 0.003
                or abs(delta) > 0.005
            ):
                blockers.append('odom_anchor_motion')
        if blockers:
            anchor, hold = None, None
            segment += 1
        elif anchor is None:
            anchor, hold = recent, f['receipt']
        if hold is None or f['receipt'] - hold < 0.5 or f['stamp'] < start + 0.5:
            blockers.append('hold_proxy')
        if not -0.1 <= f['receipt'] - f['stamp'] <= 0.5 or f['stamp'] <= last_stamp:
            blockers.append('scan_age_or_order')
        last_stamp = max(last_stamp, f['stamp'])
        if not f['wall_windows_valid']:
            blockers.append('wall_window')
        if any(b in blockers for b in ('scan_age_or_order', 'wall_window')):
            segment += 1
        f.update(eligible_proxy=not blockers, blockers=blockers, segment=segment)


def cycle_report(cycle, buffer, records):
    from rclpy.time import Time

    scans = [(t, m) for t, n, m in records if n == SCAN and cycle['start'] <= t <= cycle['end']]
    odom = [
        odom_row(t, m)
        for t, n, m in records
        if n == ODOM and cycle['start'] - 0.5 <= t <= cycle['end']
    ]
    frames = []
    mounting = {}
    for t, scan in scans:
        if (
            not scan.header.frame_id
            or not all(
                math.isfinite(v)
                for v in (scan.angle_min, scan.angle_increment, scan.range_min, scan.range_max)
            )
            or scan.angle_increment <= 0
            or scan.range_min < 0
            or scan.range_max <= scan.range_min
        ):
            raise ValueError('Invalid scan metadata')
        tf = buffer.lookup_transform('base_link', scan.header.frame_id, Time()).transform
        mounting[scan.header.frame_id] = str(tf)
        frames.append(dict(index=len(frames), receipt=t, stamp=stamp(scan), **measure(scan, tf)))
    annotate(frames, odom, cycle['start'])
    report = dict(**cycle, frames=frames, odom=odom, mounting=mounting, **comparisons(frames))
    candidates = [window(frames[i : i + 15]) for i in range(len(frames) - 14)]
    for logged in report['logged_batches']:
        eligible = [
            dict(w)
            for w in candidates
            if w['end'] <= logged['time'] + 0.15 and logged['time'] - w['end'] <= 1
        ]
        for w in eligible:
            w['logged_max_abs_error'] = max(
                abs(a - b) for key in ('median', 'spread') for a, b in zip(w[key], logged[key])
            )
        logged['closest_contiguous_window'] = (
            min(eligible, key=lambda w: w['logged_max_abs_error']) if eligible else None
        )
    report['signals'] = signal_summary(frames, report['sliding'])
    moving = [o for o in odom if cycle['start'] <= o['receipt'] <= cycle['end']]
    report['odom_stop_summary'] = dict(
        samples=len(moving),
        linear_threshold_exceeded=sum(math.hypot(*o['velocity'][:2]) >= 0.003 for o in moving),
        angular_threshold_exceeded=sum(abs(o['velocity'][2]) >= 0.01 for o in moving),
        max_linear=max((math.hypot(*o['velocity'][:2]) for o in moving), default=None),
        max_angular=max((abs(o['velocity'][2]) for o in moving), default=None),
    )
    return report


def signal_summary(frames, windows):
    output = {}
    for i, (name, target, tolerance) in enumerate(
        zip(('right', 'rear', 'angle'), (0.26, 0.28, 0.0), (0.005, 0.005, 0.008726646))
    ):
        values = [f['sample'][i] for f in frames if f['sample'][i] is not None]
        stable = [w['median'][i] for w in windows if w['stable']]
        output[name] = dict(
            finite_count=len(values),
            per_frame_min=min(values, default=None),
            per_frame_max=max(values, default=None),
            per_frame_median=statistics.median(values) if values else None,
            stable_median_min=min(stable, default=None),
            stable_median_max=max(stable, default=None),
            stable_windows_outside_target=sum(abs(v - target) > tolerance for v in stable),
            stable_windows_total=len(stable),
            target=target,
            tolerance=tolerance,
        )
    return output


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-dir', type=Path)
    parser.add_argument('--bag', type=Path)
    parser.add_argument('--log', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    bag, log = args.bag, args.log
    if args.run_dir:
        session = Path((args.run_dir / 'evidence/latest_session.txt').read_text().strip())
        result = json.loads((session / 'result.json').read_text())
        if result.get('closed_metadata') is not True:
            raise ValueError('Recorder did not confirm closed metadata')
        bag, log = bag or session / 'bag', log or args.run_dir / 'motion.log'
    if bag is None or log is None:
        parser.error('Use --run-dir or both --bag and --log')
    cycles = parse_cycles(log.read_text())
    buffer, records = read_bag(
        bag, min(c['start'] for c in cycles) - 0.5, max(c['end'] for c in cycles)
    )
    reports = [cycle_report(c, buffer, records) for c in cycles]
    report = dict(
        schema=1,
        scope='offline_same_statistics_not_controller_callback_replay',
        reference='distance_controller db60999 precise_start/right-wall defaults',
        parameters=dict(
            right_target=0.26,
            rear_target=0.28,
            distance_tolerance=0.005,
            heading_tolerance=0.008726646,
            batch_size=15,
            retained_min=12,
            spread_limits=[0.01, 0.01, 0.034906585],
            side_half_angle=0.17453292519943295,
            rear_half_angle=0.17453292519943295,
            heading_half_angle=0.5235987755982988,
            side_min=0.18,
            side_max=1.5,
            rear_min=0.22,
            side_max_mad=0.03,
            heading_min_span=0.18,
            heading_max_rms=0.012,
            robust_wall_heading=True,
        ),
        limitations=[
            'Recorder receipt differs from controller callback receipt.',
            'Stop hold is sampled at scan receipts; timer/odom resets between scans are not exact.',
            'Window comparison does not authorize movement or alter acceptance thresholds.',
        ],
        bag=str(bag.resolve()),
        log=str(log.resolve()),
        cycles=reports,
    )
    args.output.write_text(json.dumps(report, indent=2, allow_nan=False) + '\n')
    for i, c in enumerate(reports[-3:]):
        print(
            'C%d F%d E%d' % (i + 1, len(c['frames']), sum(f['eligible_proxy'] for f in c['frames']))
        )
        print(
            'N stable/total %d/%d'
            % (sum(w['stable'] for w in c['nonoverlap']), len(c['nonoverlap']))
        )
        print(
            'S stable/target %d/%d'
            % (
                sum(w['stable'] for w in c['sliding']),
                sum(w['within_target'] for w in c['sliding']),
            )
        )
    print('JSON ' + str(args.output))


if __name__ == '__main__':
    main()
