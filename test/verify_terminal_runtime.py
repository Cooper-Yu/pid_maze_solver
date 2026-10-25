"""Synthetic ROS plant: terminal state transitions only, not official course acceptance."""

import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import yaml


# Keep the bounded fixture lifecycle and its single finally cleanup visible together.
def main():  # noqa: PLR0915
    parser = argparse.ArgumentParser()
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    params = {
        'use_sim_time': False,
        'route_direction': 'reverse',
        'final_clockwise_turn': False,
        'final_counterclockwise_turn': True,
        'side_centering': False,
        'stop_hold': 0.1,
    }
    for point in range(15, 1, -1):
        for name, value in (('turn_deg', 0.0), ('forward_m', 0.0), ('left_m', 0.0)):
            params[f'steps.P{point:02d}_P{point - 1:02d}.{name}'] = value
    config = args.output / 'params.yaml'
    config.write_text(yaml.safe_dump({'/**': {'ros__parameters': params}}))
    rclpy.init()
    node = rclpy.create_node('terminal_synthetic_plant')
    assert not node.get_publishers_info_by_topic('/cmd_vel'), 'Existing speed publisher'
    odom_pub = node.create_publisher(Odometry, '/odometry/filtered', 10)
    scan_pub = node.create_publisher(LaserScan, '/scan', 10)
    command = [0.0, 0.0, 0.0]
    commands = []

    def receive(msg):
        command[:] = [msg.linear.x, msg.linear.y, msg.angular.z]
        commands.append(list(command))

    sub = node.create_subscription(Twist, '/cmd_vel', receive, 10)
    log = (args.output / 'controller.log').open('w')
    binary = (
        '/home/cooper/task8_dev/install/pid_maze_solver/lib/pid_maze_solver/pid_maze_solver_task5'
    )
    process = subprocess.Popen(
        [binary, '--ros-args', '--params-file', str(config)],
        stdout=log,
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )
    x = y = yaw = 0.0
    max_radius = 0.0
    minimum_gap = float('inf')
    start = previous = time.monotonic()
    try:
        while process.poll() is None and time.monotonic() - start < 35:
            now = time.monotonic()
            dt = min(now - previous, 0.03)
            previous = now
            c, s = math.cos(yaw), math.sin(yaw)
            x += (c * command[0] - s * command[1]) * dt
            y += (s * command[0] + c * command[1]) * dt
            yaw += command[2] * dt
            max_radius = max(max_radius, math.hypot(x, y))
            stamp = node.get_clock().now().to_msg()
            odom = Odometry()
            odom.header.stamp = stamp
            odom.header.frame_id = 'odom'
            odom.child_frame_id = 'base_link'
            odom.pose.pose.position.x, odom.pose.pose.position.y = x, y
            odom.pose.pose.orientation.z = math.sin(yaw / 2)
            odom.pose.pose.orientation.w = math.cos(yaw / 2)
            odom.twist.twist.linear.x, odom.twist.twist.linear.y = command[:2]
            odom.twist.twist.angular.z = command[2]
            odom_pub.publish(odom)
            scan = LaserScan()
            scan.header.stamp = stamp
            scan.header.frame_id = 'base_link'
            scan.angle_min, scan.angle_increment = -math.pi, math.pi / 360
            scan.angle_max = scan.angle_min + scan.angle_increment * 719
            scan.range_min, scan.range_max = 0.01, 8.0
            ranges = []
            for i in range(720):
                a = scan.angle_min + i * scan.angle_increment
                world_direction = math.cos(yaw + a)
                d = (0.230 - x) / world_direction if world_direction > 1e-9 else 5.0
                d = min(d, 5.0)
                ranges.append(d)
                bx, by = d * math.cos(a), d * math.sin(a)

                def box(hx, hy):
                    dx, dy = abs(bx) - hx, abs(by) - hy
                    return math.hypot(max(dx, 0), max(dy, 0)) if dx > 0 or dy > 0 else max(dx, dy)

                minimum_gap = min(minimum_gap, box(0.170, 0.135), box(0.135, 0.160))
            scan.ranges = ranges
            scan_pub.publish(scan)
            rclpy.spin_once(node, timeout_sec=0.01)
            time.sleep(0.005)
        assert process.poll() is not None, 'Runtime timeout'
        log.flush()
        text = (args.output / 'controller.log').read_text()
        result = {
            'scope': 'synthetic ROS state transitions, not Gazebo/official acceptance',
            'exit_code': process.returncode,
            'placement': 'TERMINAL_PLACEMENT' in text,
            'restore': 'TERMINAL_RESTORE' in text,
            'completed': 'Final counterclockwise turn completed' in text,
            'maximum_displacement_m': max_radius,
            'final_position_error_m': math.hypot(x, y),
            'signed_yaw_rad': yaw,
            'minimum_model_clearance_m': minimum_gap,
            'last_command': commands[-1] if commands else None,
        }
        (args.output / 'result.json').write_text(json.dumps(result, indent=2) + '\n')
        print(json.dumps(result))
        assert (
            result['exit_code'] == 0
            and result['placement']
            and result['restore']
            and result['completed']
        )
        assert 0.005 < max_radius <= 0.03 and math.hypot(x, y) < 0.0035
        assert abs(yaw - math.pi) < 0.015 and minimum_gap >= 0.015
        assert commands[-1] == [0.0, 0.0, 0.0]
    finally:
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=3)
        log.close()
        node.destroy_subscription(sub)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
