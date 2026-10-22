#!/usr/bin/env python3
"""Real formal/return runners plus recorder; explicit one-next wrapper in isolated ROS."""

from datetime import datetime, timedelta, timezone
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

DISTANCE = Path.home() / 'ros2_ws/src/distance_controller'
sys.path.insert(0, str(DISTANCE / 'test'))
from verify_task6_route_runner import Fixture, rclpy  # noqa: E402 - explicit existing fixture path
from nav_msgs.msg import Odometry  # noqa: E402
from sensor_msgs.msg import CompressedImage  # noqa: E402

os.environ['ROS_DOMAIN_ID'] = '198'
os.environ['ROS_LOCALHOST_ONLY'] = '1'
SCRIPT = Path(__file__).resolve().parents[1] / 'tools/task6_return_ops.py'


class ReturnOpsFixture(Fixture):
    def __init__(self, directory):
        super().__init__(directory, 'success')
        self.raw = self.node.create_publisher(Odometry, '/rosbot_xl_base_controller/odom', 10)
        self.node.create_subscription(Odometry, '/odometry/filtered', self.raw.publish, 10)
        self.image = self.node.create_publisher(CompressedImage, '/image_raw/compressed', 10)
        self.image_timer = self.node.create_timer(0.1, self.camera)
        self.record_log = (directory / 'outward_record.log').open('w')
        self.recorder = subprocess.Popen(
            [
                sys.executable,
                str(DISTANCE / 'tools/task6_ops/evidence_recorder.py'),
                '--root',
                str(directory / 'evidence'),
                '--seconds',
                str(getattr(self, 'fixture_seconds', 180)),
                '--image-hz',
                '2',
                '--max-mib',
                str(getattr(self, 'fixture_seconds', 180)),
            ],
            stdin=subprocess.DEVNULL,
            stdout=self.record_log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )

    def camera(self):
        msg = CompressedImage()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = 'synthetic'
        msg.format = 'fixture_bytes'
        msg.data = bytes([1, 2, 3])
        self.image.publish(msg)

    def return_options(self, config):
        """The manual fixture remains the default; automatic fixture supplies full-route evidence."""
        return []

    def prepare_return(self):
        self.wait(lambda: self.child.poll() is not None, getattr(self, 'fixture_seconds', 100))
        assert self.child.returncode == 0, self.log.read_text()[-2000:]
        self.wait(self.released, 10)
        self.wait(lambda: not self.node.get_publishers_info_by_topic('/cmd_vel'), 5)
        os.killpg(self.recorder.pid, signal.SIGINT)
        self.wait(lambda: self.recorder.poll() is not None, 8)
        self.record_log.close()
        self.output.close()
        session = Path((self.directory / 'evidence/latest_session.txt').read_text().strip())
        evidence = json.loads((session / 'result.json').read_text())
        assert evidence['closed_metadata']
        config = dict(support=str(DISTANCE / 'tools/task6_ops'), packages={})
        for package in ('distance_controller', 'turn_controller'):
            library = Path.home() / 'ros2_ws/install' / package / 'lib' / package
            config['packages'][package] = dict(
                library=str(library),
                installed_sha256={
                    str(p.relative_to(library)): hashlib.sha256(p.read_bytes()).hexdigest()
                    for p in library.rglob('*')
                    if p.is_file() and '__pycache__' not in p.parts and p.suffix != '.pyc'
                },
            )
        gids = {
            t: [bytes(i.endpoint_gid).hex() for i in self.node.get_publishers_info_by_topic(t)]
            for t in ('/odometry/filtered', '/rosbot_xl_base_controller/odom')
        }
        for name, value in (
            ('config', config),
            ('result', dict(success=True, cleanup=dict(zero_and_released=True, evidence=evidence))),
            ('preflight', dict(gids=gids)),
            ('run.started', dict(gids=gids)),
        ):
            (self.directory / (name + '.json')).write_text(json.dumps(value))
        extra = self.return_options(config)
        (self.directory / 'config.json').write_text(json.dumps(config))
        self.return_dir = self.directory / 'return'
        self.log = self.directory / 'wrapper.log'
        self.output = self.log.open('w')
        self.commands = []
        self.child = subprocess.Popen(
            [
                sys.executable,
                str(SCRIPT),
                'start',
                '--source-run-dir',
                str(self.directory),
                '--return-dir',
                str(self.return_dir),
                '--deadline',
                (
                    datetime.now(timezone.utc)
                    + timedelta(seconds=getattr(self, 'fixture_seconds', 180) + 180)
                ).isoformat(),
                '--max-seconds',
                str(getattr(self, 'fixture_seconds', 180)),
                *extra,
            ],
            stdin=subprocess.DEVNULL,
            stdout=self.output,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )

    def state(self):
        path = self.return_dir / 'control.json'
        return json.loads(path.read_text()) if path.exists() else {}

    def command(self, value):
        result = subprocess.run(
            [sys.executable, str(SCRIPT), value, '--return-dir', str(self.return_dir)],
            capture_output=True,
            text=True,
            timeout=3,
        )
        assert result.returncode == 0, result.stderr

    def verify(self):
        self.prepare_return()
        self.wait(lambda: self.state().get('ready') or self.child.poll() is not None, 25)
        assert self.child.poll() is None, self.log.read_text()[-4000:]
        until = time.monotonic() + 1
        self.wait(lambda: time.monotonic() > until, 3)
        assert not any(any(abs(v) > 1e-8 for v in row) for row in self.commands), (
            'Automatic motion before next'
        )
        self.command('next')
        self.wait(
            lambda: self.state().get('completed', 0) == 1 or self.child.poll() is not None, 25
        )
        assert self.state().get('completed') == 1, self.log.read_text()[-4000:]
        until = time.monotonic() + 1
        self.wait(lambda: time.monotonic() > until, 3)
        assert self.state()['completed'] == 1 and not self.state()['busy']
        self.command('quit')
        self.wait(lambda: self.child.poll() is not None, 20)
        result = json.loads((self.return_dir / 'result.json').read_text())
        assert self.child.returncode == 2 and not result['success'], result
        assert (
            result['cleanup']['zero_and_released']
            and result['cleanup']['evidence']['closed_metadata']
        ), result
        assert (self.directory / 'points.return_claim.json').exists()
        assert self.commands[-1] == (0.0, 0.0, 0.0)
        print('RETURN_OPS_ONE_NEXT_PASS', self.directory, flush=True)


def main():
    root = Path('/tmp/task6_return_ops_ros') / str(time.time_ns())
    root.parent.mkdir(exist_ok=True)
    rclpy.init()
    fixture = None
    try:
        fixture = ReturnOpsFixture(root)
        fixture.verify()
    finally:
        if fixture:
            if fixture.recorder.poll() is None:
                os.killpg(fixture.recorder.pid, signal.SIGINT)
                fixture.recorder.wait(timeout=8)
            fixture.close()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
