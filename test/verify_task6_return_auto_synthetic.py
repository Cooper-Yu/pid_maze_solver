#!/usr/bin/env python3
"""Synthetic source admission + actual ROS return; never evidence of a real outward run."""

import argparse
from dataclasses import asdict
from datetime import datetime, timedelta, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import uuid

from verify_task6_return_auto import AutoFixture, ACTION_NAMES, rclpy
from verify_task6_return_ops import DISTANCE, SCRIPT

sys.path.insert(0, str(DISTANCE / 'tools'))
from action_plan import Pose
from motion_model import Motion, target_pose
from task6_route_model import RouteAction
from route_return_plan import load_source


class SyntheticSourceFixture(AutoFixture):
    def launch(self, source):
        """No outward process: source generation is explicitly test scaffolding."""
        self.waypoints = self.directory / 'fixture_waypoints.yaml'
        self.receive_count = 0
        self.source_definition = source
        return None

    def receive(self, msg):
        super().receive(msg)
        self.receive_count += 1

    def diagnostic(self):
        return dict(
            synthetic_fixture=True,
            pose=self.pose,
            velocity=self.velocity,
            callback_count=self.receive_count,
            last_receive_age=time.monotonic() - self.received,
            recorded_command_count=len(self.commands),
            filtered_subscriptions=self.odom.get_subscription_count(),
            command_publishers=[
                dict(name=i.node_name, gid=bytes(i.endpoint_gid).hex())
                for i in self.node.get_publishers_info_by_topic('/cmd_vel')
            ],
        )

    def tick(self):
        super().tick()
        if time.monotonic() - getattr(self, 'last_diagnostic', 0) > 1:
            self.last_diagnostic = time.monotonic()
            with (self.directory / 'plant_diagnostic.jsonl').open('a') as stream:
                stream.write(json.dumps(self.diagnostic()) + '\n')

    def generate_source(self):
        origin, previous = Pose(0, 0, 0), Pose(0, 0, 0)
        position, heading, actions = [0.0, 0.0], 0.0, []
        for name in ACTION_NAMES:
            if name.endswith('_after_turn'):
                heading += 0.12
                actions.append(RouteAction(name, 'turn', heading))
            else:
                end = [
                    position[0] + 0.018 * math.cos(heading),
                    position[1] + 0.018 * math.sin(heading),
                ]
                actions.append(RouteAction(name, 'path', heading, (tuple(position), tuple(end))))
                position = end
        raw = [asdict(a) for a in actions]
        for item in raw:
            if item['kind'] == 'turn':
                item.pop('points')
        manifest = dict(schema=1, scene=2, frame='P01', actions=raw)
        self.source_definition.write_text(json.dumps(manifest))
        self.waypoints.write_text(json.dumps(dict(manifest, validated=False, test_until='P15')))
        stamp = self.node.get_clock().now().nanoseconds - 100_000_000
        book = dict(
            schema=1,
            frame='odom',
            session_id='synthetic-' + str(uuid.uuid4()),
            synthetic_fixture=True,
            origin=asdict(origin),
            points={},
        )
        book['points']['P01'] = dict(
            pose=asdict(origin), odom_stamp_ns=stamp, synthetic_fixture=True
        )
        rows = [
            dict(event='route_started', scene=2, frame='P01', actions=raw),
            dict(event='record', name='P01', point=book['points']['P01']),
        ]
        for index, action in enumerate(actions):
            motion = (
                action.path_motion()
                if action.kind == 'path'
                else Motion('turn', angle=action.heading_rad - previous.yaw)
            )
            anchor = origin if action.kind == 'path' else previous
            end = target_pose(anchor, motion)
            point = dict(
                pose=asdict(end),
                odom_stamp_ns=stamp + (index + 1) * 1_000_000,
                synthetic_fixture=True,
            )
            book['points'][action.name] = point
            rows.extend(
                [
                    dict(event='route_action_started', index=index, name=action.name),
                    dict(
                        event='started',
                        start=asdict(anchor),
                        motion=asdict(motion),
                        target=asdict(end),
                    ),
                    dict(event='final_check', ready=True),
                    dict(event='completed', actual=asdict(end)),
                    dict(event='record', name=action.name, point=point),
                ]
            )
            previous = end
        rows.append(dict(event='route_completed', destination='P15', exit_code=0))
        for index, row in enumerate(rows):
            row.update(synthetic_fixture=True, time=time.time() - 0.1 + index * 0.0001)
        self.path.write_text(json.dumps(book))
        self.path.with_suffix('.jsonl').write_text(''.join(json.dumps(r) + '\n' for r in rows))
        self.pose = [previous.x, previous.y, previous.yaw]
        source = load_source(self.path)
        (self.directory / 'SYNTHETIC_SOURCE.json').write_text(
            json.dumps(
                dict(
                    synthetic_fixture=True,
                    real_outward_executed=False,
                    source_hash=source.source_hash,
                    generator=__file__,
                    note='All source completion rows are constructed fixture inputs.',
                )
            )
        )

    def prepare_return(self):
        self.generate_source()
        until = time.monotonic() + 2
        self.wait(lambda: time.monotonic() > until, 4)
        assert not self.node.get_publishers_info_by_topic('/cmd_vel')
        os.killpg(self.recorder.pid, signal.SIGINT)
        self.wait(lambda: self.recorder.poll() is not None, 8)
        self.record_log.close()
        self.output.close()
        session = Path((self.directory / 'evidence/latest_session.txt').read_text().strip())
        evidence = json.loads((session / 'result.json').read_text())
        assert evidence['closed_metadata'] and not evidence.get('exception'), evidence
        config = dict(
            synthetic_fixture=True, support=str(DISTANCE / 'tools/task6_ops'), packages={}
        )
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
        extra = self.return_options(config)
        for name, value in (
            ('config', config),
            (
                'result',
                dict(
                    synthetic_fixture=True,
                    real_outward_executed=False,
                    success=True,
                    cleanup=dict(zero_and_released=True, evidence=evidence),
                ),
            ),
            ('preflight', dict(synthetic_fixture=True, gids=gids)),
            ('run.started', dict(synthetic_fixture=True, gids=gids)),
        ):
            (self.directory / (name + '.json')).write_text(json.dumps(value))
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
                (datetime.now(timezone.utc) + timedelta(seconds=450)).isoformat(),
                '--max-seconds',
                '420',
                *extra,
            ],
            stdin=subprocess.DEVNULL,
            stdout=self.output,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--case', choices=('all', 'full', 'quit'), default='all')
    args = parser.parse_args()
    root = Path('/tmp/task6_return_auto_synthetic') / str(time.time_ns())
    root.mkdir(parents=True)
    print('ARTIFACT', root, flush=True)
    rclpy.init()
    try:
        for name, cancel in [('full', False), ('quit', True)]:
            if args.case != 'all' and args.case != name:
                continue
            fixture = SyntheticSourceFixture(root / name, cancel)
            try:
                fixture.verify()
            except Exception as exc:
                (root / 'failure.json').write_text(
                    json.dumps(
                        dict(
                            synthetic_fixture=True,
                            case=name,
                            error=repr(exc),
                            plant=fixture.diagnostic(),
                        ),
                        indent=2,
                    )
                )
                raise
            finally:
                if fixture.recorder.poll() is None:
                    os.killpg(fixture.recorder.pid, signal.SIGINT)
                    fixture.recorder.wait(timeout=8)
                if fixture.child is None:
                    fixture.child = subprocess.Popen(['true'])
                    fixture.child.wait()
                fixture.close()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
