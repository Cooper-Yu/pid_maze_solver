#!/usr/bin/env python3
"""Read-only P15 registration experiment; never starts a controller or publisher."""

import argparse
from dataclasses import asdict
import json
import math
from pathlib import Path
import sqlite3
import sys
import time
from types import SimpleNamespace as N

from task8_p15_localization import nominal_estimate, origin_from_current


def reference_scan(book, database):
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import LaserScan

    last = book['points']['P15_after_turn']
    reference = last['laser']
    stamp = reference['stamp_ns']
    with sqlite3.connect('file:' + str(Path(database).resolve()) + '?mode=ro', uri=True) as con:
        topic = con.execute('SELECT id FROM topics WHERE name=?', ('/scan_filtered',)).fetchone()
        if topic is None:
            raise ValueError('Reference scan topic missing')
        rows = con.execute(
            'SELECT data FROM messages WHERE topic_id=? AND timestamp BETWEEN ? AND ?',
            (topic[0], int((last['recorded'] - 2) * 1e9), int((last['recorded'] + 2) * 1e9)),
        )
        for (data,) in rows:
            scan = deserialize_message(data, LaserScan)
            actual = scan.header.stamp.sec * 1_000_000_000 + scan.header.stamp.nanosec
            if actual == stamp:
                if scan.header.frame_id != reference['scan_frame']:
                    raise ValueError('Reference scan frame mismatch')
                return scan, reference
    raise ValueError('Exact recorded P15 scan absent from closed bag')


def source_pose(book, endpoint):
    first = book['points']['P01']['pose']
    last = book['points']['P15_after_turn']['pose']
    a = first['yaw']
    dx = last['x'] - first['x']
    dy = last['y'] - first['y']
    return dict(
        x=math.cos(a) * dx + math.sin(a) * dy - endpoint[0],
        y=-math.sin(a) * dx + math.cos(a) * dy - endpoint[1],
        yaw=math.remainder(last['yaw'] - a, 2 * math.pi),
    )


def run(args):
    from wall_geometry import scan_points, fit_wall
    from wall_runtime import WallRunner

    book = json.loads(args.book.read_text())
    scan, reference = reference_scan(book, args.bag)
    t = reference['sensor_translation_m']
    q = reference['sensor_rotation_xyzw']
    mount = N(translation=N(x=t[0], y=t[1], z=t[2]), rotation=N(x=q[0], y=q[1], z=q[2], w=q[3]))
    points, counts = scan_points(scan, mount)
    walls = {s: asdict(fit_wall(points, counts, s)) for s in ('front', 'right')}
    source = source_pose(book, args.endpoint)
    runner = WallRunner()
    samples = []
    errors = []
    start = time.monotonic()
    seen = set()
    try:
        while time.monotonic() - start < 12 and len(samples) < 12:
            runner.pump()
            try:
                pose = runner.pose()
                if runner.node.count_publishers('/cmd_vel'):
                    raise ValueError('Velocity publisher present during read-only registration')
                points, counts = runner.geometry()
                stamp = runner.cached_stamp
                if stamp in seen:
                    continue
                seen.add(stamp)
                current = {s: asdict(fit_wall(points, counts, s)) for s in ('front', 'right')}
                estimate = nominal_estimate(walls, current, source)
                odom = asdict(pose)
                samples.append(
                    dict(
                        scan_stamp=stamp,
                        odom_stamp=runner.stamp,
                        odom=odom,
                        estimate=estimate,
                        origin=origin_from_current(odom, estimate),
                        elapsed_s=time.monotonic() - start,
                    )
                )
            except (RuntimeError, ValueError) as error:
                errors.append(str(error))
        return dict(
            scope='read_only_registration_experiment_not_motion_authorization',
            source_in_nominal=source,
            reference_walls=walls,
            samples=samples,
            errors=errors[-12:],
            elapsed_s=time.monotonic() - start,
            stable_start_verified=False,
            success=len(samples) >= 5,
        )
    finally:
        runner.close(False)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--book', type=Path, required=True)
    parser.add_argument('--bag', type=Path, required=True)
    parser.add_argument('--endpoint', type=float, nargs=2, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    with args.output.open('x') as stream:
        try:
            result = run(args)
        except Exception as error:
            result = dict(success=False, error=str(error), stable_start_verified=False)
        json.dump(result, stream, indent=2, allow_nan=False)
        print(json.dumps(result, allow_nan=False), flush=True)
    return 0 if result['success'] else 2


if __name__ == '__main__':
    # Existing installed distance tools only; no dependency installation.
    from ament_index_python.packages import get_package_prefix

    sys.path.insert(
        0, str(Path(get_package_prefix('distance_controller')) / 'lib/distance_controller')
    )
    raise SystemExit(main())
