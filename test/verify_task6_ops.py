#!/usr/bin/env python3
"""Isolated synthetic feedback + real recorder/watch + mock official runner lifecycle."""

import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

CONTROLLER = """#!/usr/bin/env python3
import argparse,json,signal,time
from pathlib import Path
import rclpy
from geometry_msgs.msg import Twist
from rclpy.signals import SignalHandlerOptions
p=argparse.ArgumentParser();p.add_argument('scene');p.add_argument('--destination');p.add_argument('--waypoints');p.add_argument('--output');p.add_argument('--speed');p.add_argument('--turn-speed');a=p.parse_args()
rclpy.init(signal_handler_options=SignalHandlerOptions.NO);n=rclpy.create_node('fake_official_controller');v=n.create_publisher(Twist,'/cmd_vel',10)
stop=[]
for s in (signal.SIGINT,signal.SIGTERM):signal.signal(s,lambda *_:stop.append(True))
t=time.monotonic()
while not stop and time.monotonic()-t<3:
 m=Twist();m.linear.x=.01;v.publish(m);rclpy.spin_once(n,timeout_sec=.05)
t=time.monotonic()
while time.monotonic()-t<.5:v.publish(Twist());rclpy.spin_once(n,timeout_sec=.05)
n.destroy_node();rclpy.shutdown()
if not stop:Path(a.output).with_suffix('.jsonl').write_text(json.dumps(dict(event='route_completed',destination=a.destination))+'\\n')
raise SystemExit(2 if stop else 0)
"""


def child(directory, support, installed):
    sys.path.insert(0, installed)
    from task6_ops_runtime import Operation
    from task6_preflight import Probe

    probe = Probe()
    try:
        initial = probe.ready()
        config = dict(
            support=support,
            max_seconds=30,
            speed=0.12,
            turn_speed=0.20,
            deadline_unix=time.time() + 300,
            entry=str(directory / 'controller'),
            destination='P12',
            waypoints='fixture.yaml',
        )
        raise SystemExit(Operation(directory, config, probe, initial['gids']).run())
    finally:
        probe.close()


def run_case(root, mode, support, installed):  # noqa: PLR0915 - keep synthetic process cleanup with fixture
    import rclpy
    from geometry_msgs.msg import TransformStamped, Twist
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import CompressedImage, LaserScan
    from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

    directory = root / mode
    directory.mkdir()
    controller = directory / 'controller'
    controller.write_text(CONTROLLER)
    controller.chmod(0o700)
    node = rclpy.create_node('ops_fixture_sensors')
    filtered = node.create_publisher(Odometry, '/odometry/filtered', 10)
    base = node.create_publisher(Odometry, '/rosbot_xl_base_controller/odom', 10)
    scan = node.create_publisher(LaserScan, '/scan_filtered', 10)
    camera = node.create_publisher(CompressedImage, '/image_raw/compressed', 10)
    tf = StaticTransformBroadcaster(node)
    transform = TransformStamped()
    transform.header.frame_id = 'base_link'
    transform.child_frame_id = 'laser'
    transform.transform.rotation.w = 1.0
    tf.sendTransform(transform)
    commands = []
    node.create_subscription(
        Twist,
        '/cmd_vel',
        lambda msg: commands.append((msg.linear.x, msg.linear.y, msg.angular.z)),
        10,
    )
    log = (directory / 'wrapper.log').open('w')
    process = subprocess.Popen(
        [sys.executable, __file__, 'child', str(directory), support, installed],
        stdout=log,
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )
    deadline = time.monotonic() + 35
    injected = False
    fresh = True
    try:
        while process.poll() is None and time.monotonic() < deadline:
            msg = Odometry()
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.header.frame_id = 'odom'
            msg.child_frame_id = 'base_link'
            msg.pose.pose.orientation.w = 1.0
            if fresh:
                filtered.publish(msg)
                base.publish(msg)
            laser = LaserScan()
            laser.header.stamp = msg.header.stamp
            laser.header.frame_id = 'laser'
            laser.angle_min = -math.pi
            laser.angle_increment = 2 * math.pi / 360
            laser.range_min = 0.05
            laser.range_max = 10.0
            laser.ranges = [1.0] * 360
            scan.publish(laser)
            image = CompressedImage()
            image.header.stamp = msg.header.stamp
            image.header.frame_id = 'synthetic_camera'
            image.format = 'fixture_bytes'
            image.data = bytes([1, 2, 3])
            camera.publish(image)
            rclpy.spin_once(node, timeout_sec=0.03)
            if any(abs(v[0]) > 0.001 for v in commands) and not injected:
                if mode == 'cancel':
                    os.killpg(process.pid, signal.SIGINT)
                elif mode == 'gid_change':
                    node.destroy_publisher(filtered)
                    filtered = node.create_publisher(Odometry, '/odometry/filtered', 10)
                elif mode == 'feedback_loss':
                    fresh = False
                injected = mode != 'success'
        process.wait(timeout=3)
        result = json.loads((directory / 'result.json').read_text())
        assert result['success'] == (mode == 'success'), result
        command = json.loads((directory / 'motion_command.json').read_text())
        assert command[command.index('--speed') + 1] == '0.12'
        assert command[command.index('--turn-speed') + 1] == '0.2'
        assert result['cleanup']['evidence']['closed_metadata'], result
        assert commands and commands[-1] == (0.0, 0.0, 0.0), commands[-5:]
        until = time.monotonic() + 2
        while node.get_publishers_info_by_topic('/cmd_vel') and time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=0.05)
        assert not node.get_publishers_info_by_topic('/cmd_vel')
        if mode == 'gid_change':
            assert not result['cleanup']['zero_and_released'], result
        print(mode, 'PASS', directory, flush=True)
    finally:
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            process.wait(timeout=15)
        log.close()
        node.destroy_node()


def main():
    if len(sys.argv) > 1 and sys.argv[1] == 'child':
        return child(Path(sys.argv[2]), sys.argv[3], sys.argv[4])
    import rclpy

    root = Path('/tmp/task6_ops_ros') / str(time.time_ns())
    root.mkdir(parents=True)
    support = str(Path.home() / 'ros2_ws/src/distance_controller/tools/task6_ops')
    installed = str(Path.home() / 'ros2_ws/install/pid_maze_solver/lib/pid_maze_solver')
    rclpy.init()
    try:
        for mode in os.environ.get(
            'TASK6_OPS_CASES', 'success,cancel,gid_change,feedback_loss'
        ).split(','):
            run_case(root, mode, support, installed)
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
