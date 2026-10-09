#!/usr/bin/env python3
"""Verify diagnostic gate labels with synthetic feedback on an isolated ROS domain."""

import math
from pathlib import Path
import subprocess
import tempfile
import time

from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from sensor_msgs.msg import LaserScan


def publish_feedback(node, odom_pub, scan_pub, elapsed):
    """Publish the deliberately staged gate inputs; this is not a physics model."""
    yaw = math.pi / 4 * min(1.0, max(0.0, elapsed - 2.0))
    stamp = node.get_clock().now().to_msg()
    odom = Odometry()
    odom.header.stamp = stamp
    odom.header.frame_id = 'odom'
    odom.child_frame_id = 'base_link'
    odom.pose.pose.orientation.z = math.sin(yaw / 2)
    odom.pose.pose.orientation.w = math.cos(yaw / 2)
    odom.twist.twist.angular.z = 0.05 if 3 <= elapsed < 5 else 0.0
    odom.twist.twist.linear.x = 0.02 if 5 <= elapsed < 7 else 0.0
    odom_pub.publish(odom)
    scan = LaserScan()
    scan.header.stamp = stamp
    scan.header.frame_id = 'base_link'
    scan.angle_min = -math.pi
    scan.angle_increment = 2 * math.pi / 360
    scan.range_min = 0.01
    scan.range_max = 10.0
    scan.ranges = [2.0] * 360
    scan_pub.publish(scan)


def main():
    """Feed angle, angular-speed, linear-speed and stopped-hold phases; verify logs and exit."""
    rclpy.init()
    node = rclpy.create_node('turn_diagnostics_fixture')
    odom_pub = node.create_publisher(Odometry, '/odometry/filtered', 10)
    scan_pub = node.create_publisher(LaserScan, '/scan', 10)
    commands = []
    node.create_subscription(Twist, '/cmd_vel', lambda m: commands.append(m), 10)
    exe = Path(get_package_prefix('pid_maze_solver')) / 'lib/pid_maze_solver/pid_maze_solver'
    params = [
        'use_sim_time:=false',
        'last_point:=2',
        'stop_hold:=0.8',
        'steps.P01_P02.turn_deg:=45.0',
        'steps.P01_P02.forward_m:=0.0',
    ]
    with tempfile.TemporaryFile(mode='w+') as output:
        proc = subprocess.Popen(
            [str(exe), '--ros-args'] + [arg for value in params for arg in ['-p', value]],
            stdout=output,
            stderr=subprocess.STDOUT,
        )
        start = time.monotonic()
        try:
            while proc.poll() is None and time.monotonic() - start < 13:
                elapsed = time.monotonic() - start
                publish_feedback(node, odom_pub, scan_pub, elapsed)
                rclpy.spin_once(node, timeout_sec=0.02)
            assert proc.poll() == 0, 'Controller must complete partial trial'
            output.seek(0)
            log = output.read()
            for reason in ['angle', 'angular_speed', 'linear_speed', 'hold']:
                assert f'reason={reason}' in log, reason
            assert 'Final clockwise turn:' not in log
            assert commands and commands[-1].linear.x == 0 and commands[-1].angular.z == 0
            print(log)
            print('Diagnostic gates and partial-route termination: PASS')
        finally:
            if proc.poll() is None:
                proc.terminate()
                proc.wait(timeout=3)
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
