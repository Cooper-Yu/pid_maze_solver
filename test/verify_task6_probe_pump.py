#!/usr/bin/env python3
"""Localhost-only real Humble executor queue checks; no hardware or motion controller."""

import json
import os
from pathlib import Path
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from task6_preflight import ODOM, SCAN, Probe, validate_feedback


def main():  # noqa: PLR0915 - one bounded node lifetime with unconditional cleanup
    if os.environ.get('ROS_LOCALHOST_ONLY') != '1' or os.environ.get('ROS_DOMAIN_ID') != '194':
        raise RuntimeError('Require isolated localhost domain 194')
    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import LaserScan

    root = Path('/tmp/task6_probe_pump') / str(time.time_ns())
    root.mkdir(parents=True)
    probe = Probe()
    source = rclpy.create_node('probe_pump_synthetic_source')
    odom = [source.create_publisher(Odometry, topic, 10) for topic in ODOM]
    laser = source.create_publisher(LaserScan, SCAN, 10)
    command = source.create_publisher(Twist, '/cmd_vel', 10)
    report = {}
    real_spin = rclpy.spin_once
    try:
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline and laser.get_subscription_count() < 1:
            probe.pump()
        assert laser.get_subscription_count() == 1
        old = source.get_clock().now().nanoseconds - 700_000_000

        def publish(stamp):
            msg = Odometry()
            msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(stamp, 1_000_000_000)
            msg.header.frame_id, msg.child_frame_id = 'odom', 'base_link'
            msg.pose.pose.orientation.w = 1.0
            for publisher in odom:
                publisher.publish(msg)
            scan = LaserScan()
            scan.header = msg.header
            scan.header.frame_id = 'laser'
            scan.range_min, scan.range_max = 0.05, 10.0
            scan.ranges = [1.0] * 60
            command.publish(Twist())
            laser.publish(scan)

        publish(old)
        deadline = time.monotonic() + 3
        while not all(probe.counts.get(t, 0) for t in (*ODOM, SCAN)):
            assert time.monotonic() < deadline
            probe.pump()
        for _ in range(20):
            real_spin(probe.node, timeout_sec=0.0)
        publish(source.get_clock().now().nanoseconds)
        time.sleep(0.03)  # DDS delivery before observation; never inside production pump.
        callbacks = []

        def observe(node, timeout_sec):
            before = dict(probe.counts)
            real_spin(node, timeout_sec=timeout_sec)
            callbacks.append(
                dict(
                    timeout=timeout_sec,
                    changed=[t for t in probe.counts if probe.counts[t] != before.get(t)],
                )
            )

        rclpy.spin_once = observe
        probe.pump()
        rclpy.spin_once = real_spin
        assert SCAN not in callbacks[0]['changed'], callbacks
        assert any(SCAN in row['changed'] for row in callbacks[1:]), callbacks
        validate_feedback(
            probe.samples, probe.node.get_clock().now().nanoseconds / 1e9, time.monotonic(), False
        )
        report['queued_scan'] = dict(callbacks=callbacks, pump=probe.last_pump)

        # No new scan for >.5s; keep odometry fresh, then test the actual strict validator.
        time.sleep(0.55)
        msg = Odometry()
        msg.header.stamp = source.get_clock().now().to_msg()
        msg.header.frame_id, msg.child_frame_id = 'odom', 'base_link'
        msg.pose.pose.orientation.w = 1.0
        for publisher in odom:
            publisher.publish(msg)
        time.sleep(0.02)
        probe.pump()
        try:
            validate_feedback(
                probe.samples,
                probe.node.get_clock().now().nanoseconds / 1e9,
                time.monotonic(),
                False,
            )
        except RuntimeError as error:
            assert SCAN in str(error), str(error)
            report['stale_scan'] = dict(error=str(error), pump=probe.last_pump)
        else:
            raise AssertionError('Persistent stale scan was accepted')

        # Same delivered scan is accepted only by the explicit observation policy.
        validate_feedback(
            probe.samples,
            probe.node.get_clock().now().nanoseconds / 1e9,
            time.monotonic(),
            False,
            scan_timeout=1.0,
        )
        report['observation_short_gap'] = dict(accepted=True, limit_s=1.0)
        time.sleep(0.5)
        msg.header.stamp = source.get_clock().now().to_msg()
        for publisher in odom:
            publisher.publish(msg)
        time.sleep(0.02)
        probe.pump()
        try:
            validate_feedback(
                probe.samples,
                probe.node.get_clock().now().nanoseconds / 1e9,
                time.monotonic(),
                False,
                scan_timeout=1.0,
            )
        except RuntimeError as error:
            assert SCAN in str(error), str(error)
            report['observation_long_gap'] = dict(error=str(error), limit_s=1.0)
        else:
            raise AssertionError('Scan outage beyond one second was accepted')

        busy_count = 0

        def busy():
            nonlocal busy_count
            busy_count += 1
            time.sleep(0.0007)
            guard.trigger()

        guard = probe.node.create_guard_condition(busy)
        guard.trigger()
        probe.pump()
        probe.node.destroy_guard_condition(guard)
        assert 1 <= probe.last_pump['drain_spin_calls'] <= 16
        assert busy_count >= 2
        report['busy_queue'] = dict(callbacks=busy_count, pump=probe.last_pump)
    finally:
        rclpy.spin_once = real_spin
        source.destroy_node()
        probe.close()
    (root / 'report.json').write_text(json.dumps(report, indent=2) + '\n')
    print('FIVE_CASES_PASS', root, json.dumps(report), flush=True)


if __name__ == '__main__':
    main()
