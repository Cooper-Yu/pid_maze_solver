#!/usr/bin/env python3
"""Localhost full 20-action synthetic source: real runner/recorder automatic return and quit."""

import hashlib
import json
import math
import os
from pathlib import Path
import signal
import sys
import time
import threading

from verify_task6_return_ops import ReturnOpsFixture, rclpy

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from scene_route import ACTION_NAMES

os.environ['ROS_DOMAIN_ID'] = '198'
os.environ['ROS_LOCALHOST_ONLY'] = '1'


class AutoFixture(ReturnOpsFixture):
    def __init__(self, directory, cancel):
        self.cancel_case = cancel
        self.fixture_seconds = 420
        super().__init__(directory)
        from geometry_msgs.msg import Twist
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import qos_profile_sensor_data

        self.node.destroy_subscription(self.sub)
        self.command_node = rclpy.create_node('task6_fake_base_command')
        self.command_sub = self.command_node.create_subscription(
            Twist, '/cmd_vel', self.receive, qos_profile_sensor_data
        )
        self.command_executor = SingleThreadedExecutor()
        self.command_executor.add_node(self.command_node)
        self.command_thread = threading.Thread(target=self.command_executor.spin, daemon=True)
        self.command_thread.start()

    def close(self):
        try:
            super().close()
        finally:
            self.command_executor.shutdown(timeout_sec=2)
            self.command_thread.join(timeout=2)
            self.command_node.destroy_node()

    def tick(self):
        now = time.monotonic(), time.time()
        previous = getattr(self, 'tick_clock', now)
        if now[0] - previous[0] > 0.1 or abs((now[1] - previous[1]) - (now[0] - previous[0])) > 0.1:
            with (self.directory / 'tick_gaps.jsonl').open('a') as stream:
                stream.write(
                    json.dumps(
                        dict(
                            mono_gap=now[0] - previous[0], wall_gap=now[1] - previous[1], at=now[1]
                        )
                    )
                    + '\n'
                )
        self.tick_clock = now
        super().tick()

    def inject(self):
        """No synthetic fault injection: do not repeatedly parse a growing audit in sensor tick."""
        return

    def launch(self, source):
        actions, position, heading = [], [0.0, 0.0], 0.0
        for name in ACTION_NAMES:
            if name.endswith('_after_turn'):
                heading += 0.12
                actions.append(dict(name=name, kind='turn', heading_rad=heading))
            else:
                end = [
                    position[0] + 0.018 * math.cos(heading),
                    position[1] + 0.018 * math.sin(heading),
                ]
                actions.append(
                    dict(name=name, kind='path', heading_rad=heading, points=[position, end])
                )
                position = end
        data = dict(schema=1, scene=2, frame='P01', actions=actions)
        source.write_text(json.dumps(data))
        self.waypoints = self.directory / 'fixture_waypoints.yaml'
        self.waypoints.write_text(json.dumps(dict(data, validated=False, test_until='P15')))
        return super().launch(source)

    def return_options(self, config):
        config.update(
            destination='P15',
            waypoints=str(self.waypoints),
            route_sha256=hashlib.sha256(self.waypoints.read_bytes()).hexdigest(),
        )
        return ['--auto-next']

    def cancel_moving_action(self):
        """Cancel only after the audit and observed command prove motion was requested."""

        def started_and_moving():
            audit = self.return_dir / 'return_points.jsonl'
            rows = audit.read_text().splitlines(keepends=True) if audit.exists() else []
            started = any(
                row.endswith('\n')
                and json.loads(row).get('event') == 'return_action_started'
                and json.loads(row).get('index') == 19
                for row in rows
            )
            return started and any(
                all(math.isfinite(value) for value in command)
                and any(abs(value) > 1e-9 for value in command)
                for command in self.commands
            )

        self.wait(lambda: started_and_moving() or self.child.poll() is not None, 30)
        assert self.child.poll() is None and started_and_moving(), self.log.read_text()[-4000:]
        (self.directory / 'cancel_trigger.json').write_text(
            json.dumps(
                dict(
                    synthetic_fixture=True,
                    unix_time=time.time(),
                    action_started_index=19,
                    received_command_count=len(self.commands),
                    latest_command=self.commands[-1],
                    observed_nonzero=True,
                ),
                indent=2,
            )
        )
        self.command('quit')

    def verify(self):
        self.prepare_return()
        self.wait(lambda: self.state().get('busy') or self.child.poll() is not None, 30)
        assert self.child.poll() is None, self.log.read_text()[-4000:]
        if self.cancel_case:
            self.cancel_moving_action()
        self.wait(lambda: self.child.poll() is not None, 420)
        result = json.loads((self.return_dir / 'result.json').read_text())
        rows = [
            json.loads(s)
            for s in (self.return_dir / 'return_points.jsonl').read_text().splitlines()
        ]
        started = [r for r in rows if r['event'] == 'return_action_started']
        completed = [r for r in rows if r['event'] == 'return_action_completed']
        if self.cancel_case:
            assert self.child.returncode == 2 and not result['success'], result
            assert len(started) <= 1 and not any(r['event'] == 'return_completed' for r in rows)
        else:
            assert self.child.returncode == 0 and result['success'], (
                result,
                self.log.read_text()[-4000:],
            )
            assert [r['index'] for r in started] == list(range(19, -1, -1))
            assert [r['cursor'] for r in completed] == list(range(19, -1, -1))
            assert rows[-1]['event'] == 'return_completed'
        assert result['cleanup']['zero_and_released'], result
        assert result['cleanup']['evidence']['closed_metadata'], result
        assert self.commands[-1] == (0.0, 0.0, 0.0)
        assert (self.directory / 'points.return_claim.json').exists()
        print(
            'AUTO_QUIT_PASS' if self.cancel_case else 'AUTO_FULL_PASS', self.directory, flush=True
        )


def main():
    root = Path('/tmp/task6_return_auto_ros') / str(time.time_ns())
    root.mkdir(parents=True)
    rclpy.init()
    try:
        for name, cancel in [('full', False), ('quit', True)]:
            fixture = None
            try:
                fixture = AutoFixture(root / name, cancel)
                fixture.verify()
            finally:
                if fixture:
                    if fixture.recorder.poll() is None:
                        os.killpg(fixture.recorder.pid, signal.SIGINT)
                        fixture.recorder.wait(timeout=8)
                    fixture.close()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
