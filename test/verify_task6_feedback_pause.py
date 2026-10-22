#!/usr/bin/env python3
"""Isolated real ROS supervisor hold protocol; synthetic zero commands only."""

import json
import os
from pathlib import Path
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from task6_feedback_pause import SupervisorPause
from task6_preflight import ODOM, Probe


def main():  # noqa: PLR0912,PLR0915 - one bounded synthetic ROS lifetime and unconditional cleanup
    if os.environ.get('ROS_LOCALHOST_ONLY') != '1' or os.environ.get('ROS_DOMAIN_ID') != '195':
        raise RuntimeError('Require isolated localhost domain 195')
    import rclpy
    from geometry_msgs.msg import TransformStamped, Twist
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import LaserScan
    from tf2_ros import StaticTransformBroadcaster

    root = Path('/tmp/task6_feedback_pause') / str(time.time_ns())
    root.mkdir(parents=True)
    probe = Probe()
    source = rclpy.create_node('synthetic_feedback_hold_owner')
    odom = [source.create_publisher(Odometry, topic, 10) for topic in ODOM]
    laser = source.create_publisher(LaserScan, '/scan_filtered', 10)
    command = source.create_publisher(Twist, '/cmd_vel', 10)
    broadcaster = StaticTransformBroadcaster(source)
    transform = TransformStamped()
    transform.header.frame_id, transform.child_frame_id = 'base_link', 'laser'
    transform.transform.rotation.w = 1.0
    broadcaster.sendTransform(transform)
    monitor = SupervisorPause(root)
    report = {}

    def publish(feedback=True):
        command.publish(Twist())
        if not feedback:
            return
        msg = Odometry()
        msg.header.stamp = source.get_clock().now().to_msg()
        msg.header.frame_id, msg.child_frame_id = 'odom', 'base_link'
        msg.pose.pose.orientation.w = 1.0
        for publisher in odom:
            publisher.publish(msg)
        scan = LaserScan()
        scan.header.stamp = msg.header.stamp
        scan.header.frame_id = 'laser'
        scan.range_min, scan.range_max = 0.05, 10.0
        scan.ranges = [1.0] * 60
        laser.publish(scan)

    def event(name, pause_id=1):
        with (root / 'points.jsonl').open('a') as stream:
            stream.write(
                json.dumps(dict(event=name, pause_id=pause_id, budget_s=5, owns_publisher=True))
                + '\n'
            )

    try:
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            publish()
            probe.pump()
            if all(probe.counts.get(t, 0) >= 3 for t in ODOM):
                try:
                    probe.snapshot(stopped=False, scan_timeout=1.0)
                    break
                except Exception:
                    pass
        else:
            raise AssertionError('Synthetic feedback/TF discovery failed')
        expected = {t: probe.identities(t) for t in ODOM}
        event('feedback_pause_started')
        until = time.monotonic() + 0.75
        iterations = 0
        while time.monotonic() < until:
            publish(feedback=False)
            probe.pump()
            monitor.observe(probe, expected, 1.0)
            iterations += 1
        try:
            probe.snapshot(stopped=False, expected=expected, scan_timeout=1.0)
        except RuntimeError as error:
            assert 'stale' in str(error), str(error)
            report['active_stale_rejected'] = str(error)
        else:
            raise AssertionError('Active stale samples accepted')
        report['held_outage_accepted'] = iterations
        for _ in range(6):
            publish()
            probe.pump()
        event('feedback_pause_completed')
        monitor.observe(probe, expected, 1.0)
        report['completion_restored_strict'] = True
        event('feedback_pause_started', 2)
        publish(feedback=False)
        probe.pump()
        monitor.observe(probe, expected, 1.0)
        competitor = rclpy.create_node('synthetic_unexpected_odom_owner')
        rival = competitor.create_publisher(Odometry, ODOM[1], 10)
        deadline = time.monotonic() + 2
        while len(probe.identities(ODOM[1])) < 2 and time.monotonic() < deadline:
            publish()
            probe.pump()
        try:
            monitor.observe(probe, expected, 1.0)
        except RuntimeError as error:
            assert 'identity/count changed' in str(error), str(error)
            report['identity_change_during_hold_rejected'] = str(error)
        else:
            raise AssertionError('Changed odom identity accepted during hold')
        competitor.destroy_publisher(rival)
        competitor.destroy_node()
    finally:
        command.publish(Twist())
        source.destroy_node()
        probe.close()
    (root / 'report.json').write_text(json.dumps(report, indent=2) + '\n')
    print('SUPERVISOR_ROS_PASS', root, json.dumps(report), flush=True)


if __name__ == '__main__':
    main()
