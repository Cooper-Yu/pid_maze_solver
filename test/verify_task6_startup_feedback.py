"""Localhost-only live DDS regression for the pre-dispatch no-publisher boundary."""

import json
import os
from pathlib import Path
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from task6_ops_runtime import Operation
from task6_preflight import ODOM, SCAN, Probe


def main():  # noqa: PLR0915 - one bounded ROS fixture lifetime with unconditional cleanup
    if os.environ.get('ROS_LOCALHOST_ONLY') != '1' or os.environ.get('ROS_DOMAIN_ID') != '195':
        raise RuntimeError('Fixture requires localhost domain 195')
    from geometry_msgs.msg import TransformStamped
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import LaserScan
    from tf2_ros import StaticTransformBroadcaster

    mode = sys.argv[1]
    probe = Probe()
    source = probe.ros.create_node('startup_feedback_fixture')
    odom = [source.create_publisher(Odometry, t, 10) for t in ODOM]
    scan = source.create_publisher(LaserScan, SCAN, 10)
    broadcaster = StaticTransformBroadcaster(source)
    mount = TransformStamped()
    mount.header.frame_id, mount.child_frame_id = 'base_link', 'laser'
    mount.transform.rotation.w = 1.0
    broadcaster.sendTransform(mount)
    real_pump = probe.pump
    started = None

    def pump():
        now = source.get_clock().now().nanoseconds
        stale = started is not None and (mode != 'recover' or time.monotonic() - started < 1.8)
        for publisher in odom:
            message = Odometry()
            message.header.stamp = probe.ros.time.Time(
                nanoseconds=now - (800_000_000 if stale else 0)
            ).to_msg()
            message.header.frame_id, message.child_frame_id = 'odom', 'base_link'
            message.pose.pose.orientation.w = 1.0
            if started is None or time.monotonic() - started >= 0.85:
                publisher.publish(message)
        message = LaserScan()
        message.header.stamp = probe.ros.time.Time(nanoseconds=now).to_msg()
        message.header.frame_id = 'laser'
        message.range_min, message.range_max = 0.05, 10.0
        message.ranges = [1.0] * 60
        scan.publish(message)
        real_pump()
        if mode == 'cancel' and started is not None and time.monotonic() - started > 1.3:
            operation.cancel()

    probe.pump = pump
    try:
        initial = probe.ready()
        operation = Operation(
            Path('/tmp/startup-feedback-no-motion'),
            dict(max_seconds=20, deadline_unix=time.time() + 200),
            probe,
            initial['gids'],
        )
        started = time.monotonic()
        while time.monotonic() - started < 0.9:
            pump()
        assert all(
            probe.samples[t]['stamp'] < probe.node.get_clock().now().nanoseconds / 1e9 - 0.5
            for t in ODOM
        )
        error = None
        try:
            operation.wait_before_dispatch()
        except RuntimeError as exc:
            error = str(exc)
        elapsed = time.monotonic() - started
        assert probe.command_count == 0 and not probe.identities('/cmd_vel')
        if mode == 'recover':
            assert error is None and elapsed >= 2.1, (error, elapsed)
            assert probe.snapshot(expected=initial['gids'])['feedback_fresh']
        elif mode == 'timeout':
            assert error and '5 seconds' in error and 5 <= elapsed < 7, (error, elapsed)
        else:
            assert error == 'Cancelled' and elapsed < 1.8, (error, elapsed)
        output = dict(
            mode=mode,
            success=True,
            elapsed_s=elapsed,
            error=error,
            command_count=probe.command_count,
            cmd_publishers=probe.identities('/cmd_vel'),
        )
        directory = Path('/tmp/task6_startup_feedback') / f'{time.time_ns()}_{mode}'
        directory.mkdir(parents=True)
        (directory / 'result.json').write_text(json.dumps(output, indent=2) + '\n')
        print('PASS', directory, json.dumps(output))
    finally:
        source.destroy_node()
        probe.close()


if __name__ == '__main__':
    main()
