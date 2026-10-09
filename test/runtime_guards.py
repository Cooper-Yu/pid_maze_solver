#!/usr/bin/env python3
"""Isolated fake-feedback checks; never connect this harness to a robot domain."""

import math
import subprocess
import time
import rclpy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from ament_index_python.packages import get_package_prefix
from pathlib import Path

rclpy.init()
n = rclpy.create_node('maze_guard_fixture')
o = n.create_publisher(Odometry, '/odometry/filtered', 10)
s = n.create_publisher(LaserScan, '/scan', 10)
commands = []
n.create_subscription(
    Twist, '/cmd_vel', lambda m: commands.append((m.linear.x, m.linear.y, m.angular.z)), 10
)
exe = Path(get_package_prefix('pid_maze_solver')) / 'lib/pid_maze_solver/pid_maze_solver'
for mode, expected, limit in [
    ('stale', 'FEEDBACK_TIMEOUT', 6),
    ('obstacle', 'OBSTACLE_BLOCKED', 10),
    ('invalid', 'INITIAL_FEEDBACK_TIMEOUT', 18),
]:
    commands.clear()
    proc = subprocess.Popen(
        [str(exe), '--ros-args', '-p', 'use_sim_time:=false'],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )
    started = time.monotonic()
    try:
        while proc.poll() is None and time.monotonic() - started < limit:
            elapsed = time.monotonic() - started
            if mode != 'stale' or elapsed < 2:
                stamp = n.get_clock().now().to_msg()
                od = Odometry()
                od.header.stamp = stamp
                od.header.frame_id = 'odom'
                od.child_frame_id = 'base_link'
                od.pose.pose.orientation.w = 0.0 if mode == 'invalid' else 1.0
                o.publish(od)
                sc = LaserScan()
                sc.header.stamp = stamp
                sc.header.frame_id = 'base_link'
                sc.angle_min = -math.pi
                sc.angle_increment = 2 * math.pi / 360
                sc.range_min = 0.01
                sc.range_max = 10.0
                sc.ranges = [0.205 if mode == 'obstacle' else 2.0] * 360
                s.publish(sc)
            rclpy.spin_once(n, timeout_sec=0.02)
        assert proc.poll() is not None, (mode, 'did not terminate')
        log = proc.communicate()[0]
        assert proc.returncode == 2 and expected in log, (mode, proc.returncode, log)
        assert commands and all(abs(x) < 1e-9 for x in commands[-1]), (mode, 'missing final zero')
        if mode != 'stale':
            assert not any(any(abs(x) > 1e-9 for x in c) for c in commands), (
                mode,
                'unexpected movement',
            )
        print(mode, 'PASS', flush=True)
    finally:
        if proc.poll() is None:
            proc.terminate()
            proc.wait(timeout=3)
n.destroy_node()
rclpy.shutdown()
