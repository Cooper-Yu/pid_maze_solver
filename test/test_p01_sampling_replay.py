#!/usr/bin/env python3
"""Offline geometry/statistics parity and closed-bag replay; never start a ROS node."""

import json
import math
from pathlib import Path
import sqlite3
import subprocess
import sys
import tempfile
from types import SimpleNamespace
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
import p01_sampling_replay as replay


def mount():
    return SimpleNamespace(
        translation=SimpleNamespace(x=0.02, y=0.0, z=0.17345),
        rotation=SimpleNamespace(x=0.0, y=0.0, z=1.0, w=0.0),
    )


def rectangle_scan(right=0.26, rear=0.28):
    ranges = []
    for i in range(720):
        angle = -math.pi + i * 2 * math.pi / 720
        dx, dy = -math.cos(angle), -math.sin(angle)
        distances = []
        if dx < -1e-9:
            distances.append((-rear - 0.02) / dx)
        elif dx > 1e-9:
            distances.append((3 - 0.02) / dx)
        if dy < -1e-9:
            distances.append(-right / dy)
        elif dy > 1e-9:
            distances.append(2 / dy)
        ranges.append(min(distances))
    return SimpleNamespace(
        ranges=ranges,
        angle_min=-math.pi,
        angle_increment=2 * math.pi / 720,
        range_min=0.05,
        range_max=10.0,
    )


class ReplayTest(unittest.TestCase):
    def test_geometry_uses_body_tf_upper_median_and_tls(self):
        data = replay.measure(rectangle_scan(), mount())
        self.assertTrue(data['wall_windows_valid'])
        self.assertTrue(data['heading']['valid'])
        for value, expected in zip(data['sample'], [0.26, 0.28, 0]):
            self.assertAlmostEqual(value, expected, places=12)
        self.assertEqual(replay.upper_median([1, 2, 3, 4]), 3)
        self.assertFalse(replay.side([0.26] * 5, 20)['valid'])
        self.assertFalse(replay.heading([(i * 0.02, -0.26) for i in range(20)], 50)['valid'])

    def test_joint_rejection_and_stability_limits(self):
        base = [[0.26, 0.28, 0.0]] * 15
        self.assertTrue(replay.summarize(base)['stable'])
        self.assertEqual(replay.summarize(base[:12] + [[1.0, 1.0, 1.0]] * 3)['retained'], 12)
        self.assertFalse(replay.summarize(base[:11] + [[1.0, 1.0, 1.0]] * 4)['stable'])
        changed = base[:14] + [[0.271, 0.28, 0]]
        self.assertEqual(replay.summarize(changed)['retained'], 15)
        self.assertFalse(replay.summarize(changed)['stable'])
        bad = [*base[:11], *[[None, 0.28, 0]] * 4]
        self.assertFalse(replay.summarize(bad)['stable'])

    def test_sliding_window_same_threshold_can_differ(self):
        frames = [
            dict(
                index=i,
                receipt=i * 0.1,
                sample=[0.271 if i == 0 else 0.26, 0.28, 0],
                eligible_proxy=True,
                segment=0,
            )
            for i in range(16)
        ]
        result = replay.comparisons(frames)
        self.assertFalse(result['nonoverlap'][0]['stable'])
        self.assertTrue(result['sliding'][1]['stable'])
        self.assertTrue(result['sliding'][1]['within_target'])

    def test_log_keeps_medians_separate_from_spreads(self):
        text = (
            '[1000.0] START_VERIFY: stopped hold then 15 fresh scans; 5 s budget\n'
            '[1002.0] START_VERIFY: retained=15/15 stable=false right=0.261149 rear=0.288974 '
            'angle=-0.003948 spread_right=0.011674 spread_rear=0.011264 spread_angle=0.026300\n'
            '[1005.05] Initial preparation fault: reason=START_VERIFY_TIMEOUT\n'
        )
        cycle = replay.parse_cycles(text)[0]
        self.assertEqual(cycle['logged_batches'][0]['median'], [0.261149, 0.288974, -0.003948])
        self.assertEqual(cycle['logged_batches'][0]['spread'], [0.011674, 0.011264, 0.026300])
        self.assertEqual(cycle['end'], 1005.05)

    def test_cpp_header_parity(self):
        include = Path(__file__).resolve().parents[2] / 'distance_controller/include'
        source = r"""#include <iomanip>
#include <iostream>
#include "distance_controller/robust_wall.hpp"
#include "distance_controller/start_verification.hpp"
int main(){
 std::vector<distance_controller::StartSample> s(15);
 for(auto &p:s)for(double &v:p)std::cin>>v;
 auto r=distance_controller::summarize_start(s);
 std::cout<<std::setprecision(17)<<r.retained<<" "<<r.stable;
 for(double v:r.median)std::cout<<" "<<v;
 for(double v:r.spread)std::cout<<" "<<v;
 int n;std::cin>>n;std::vector<std::pair<double,double>> p(n);
 for(auto &v:p)std::cin>>v.first>>v.second;
 auto kept=distance_controller::dominant_wall(p,.18);
 std::cout<<" "<<kept.size();for(auto v:kept)std::cout<<" "<<v.first<<" "<<v.second;
}"""
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            (root / 'oracle.cpp').write_text(source)
            subprocess.run(
                [
                    'g++',
                    '-std=c++17',
                    '-I' + str(include),
                    str(root / 'oracle.cpp'),
                    '-o',
                    str(root / 'oracle'),
                ],
                check=True,
            )
            for outliers in (0, 3, 4):
                samples = [
                    [0.26 + i * 0.0002, 0.28 - i * 0.0001, 0.001 * i] for i in range(15 - outliers)
                ] + [[1, 1, 1]] * outliers
                points = [(i * 0.02, -0.26 + (i % 3) * 0.0005) for i in range(20)] + [(0.12, -0.5)]
                payload = (
                    ' '.join(str(v) for p in samples for v in p)
                    + ' '
                    + str(len(points))
                    + ' '
                    + ' '.join(str(v) for p in points for v in p)
                )
                answer = list(
                    map(
                        float,
                        subprocess.check_output(
                            [str(root / 'oracle')], input=payload, text=True
                        ).split(),
                    )
                )
                expected = replay.summarize(samples)
                self.assertEqual(answer[:2], [expected['retained'], float(expected['stable'])])
                for a, b in zip(answer[2:8], expected['median'] + expected['spread']):
                    self.assertAlmostEqual(a, b, places=15)
                kept = replay.dominant_wall(points)
                self.assertEqual(answer[8], len(kept))
                self.assertEqual(answer[9:], [v for p in kept for v in p])

    def test_closed_sqlite_fixture(self):
        from geometry_msgs.msg import TransformStamped
        from nav_msgs.msg import Odometry
        from rclpy.serialization import serialize_message
        from sensor_msgs.msg import LaserScan
        from tf2_msgs.msg import TFMessage
        from rclpy.time import Time
        import yaml

        with tempfile.TemporaryDirectory() as folder:
            bag = Path(folder)
            db = sqlite3.connect(bag / 'fixture.db3')
            db.execute('CREATE TABLE topics(id INTEGER,name TEXT,type TEXT)')
            db.execute('CREATE TABLE messages(topic_id INTEGER,timestamp INTEGER,data BLOB)')
            for i, name, kind in [
                (1, replay.SCAN, 'sensor_msgs/msg/LaserScan'),
                (2, replay.ODOM, 'nav_msgs/msg/Odometry'),
                (3, '/tf_static', 'tf2_msgs/msg/TFMessage'),
            ]:
                db.execute('INSERT INTO topics VALUES(?,?,?)', (i, name, kind))
            tf = TransformStamped()
            tf.header.frame_id, tf.child_frame_id = 'base_link', 'laser'
            tf.transform.translation.x = 0.02
            tf.transform.rotation.z = 1.0
            tf.transform.rotation.w = 0.0
            db.execute(
                'INSERT INTO messages VALUES(?,?,?)',
                (3, 999_000_000_000, serialize_message(TFMessage(transforms=[tf]))),
            )
            for i in range(52):
                tick = 1000_000_000_000 + i * 100_000_000
                scan, odom = LaserScan(), Odometry()
                for msg in (scan, odom):
                    msg.header.stamp = Time(nanoseconds=tick).to_msg()
                scan.header.frame_id = 'laser'
                scan.angle_min, scan.angle_increment = -math.pi, 2 * math.pi / 720
                scan.range_min, scan.range_max = 0.05, 10.0
                scan.ranges = rectangle_scan().ranges
                odom.header.frame_id, odom.child_frame_id = 'odom', 'base_link'
                odom.pose.pose.orientation.w = 1.0
                for ident, msg in ((2, odom), (1, scan)):
                    db.execute(
                        'INSERT INTO messages VALUES(?,?,?)',
                        (ident, tick + (0 if ident == 2 else 1000), serialize_message(msg)),
                    )
            db.commit()
            db.close()
            (bag / 'metadata.yaml').write_text(
                yaml.safe_dump(
                    {'rosbag2_bagfile_information': {'relative_file_paths': ['fixture.db3']}}
                )
            )
            buffer, records = replay.read_bag(bag, 999.5, 1005.05)
            cycle = dict(start=1000.0, end=1005.05, logged_batches=[])
            result = replay.cycle_report(cycle, buffer, records)
            self.assertEqual(len(result['frames']), 51)
            self.assertTrue(result['nonoverlap'], str(result['frames'][:8]))
            self.assertTrue(all(w['within_target'] for w in result['sliding']))
            self.assertEqual(result['frames'][0]['odom']['velocity'], [0.0, 0.0, 0.0])
            json.dumps(result, allow_nan=False)
            log = bag / 'motion.log'
            log.write_text(
                '[1000.0] START_VERIFY: stopped hold then 15 fresh scans; 5 s budget\n'
                '[1005.05] Initial preparation fault: reason=START_VERIFY_TIMEOUT\n'
            )
            output = bag / 'report.json'
            cli = subprocess.check_output(
                [
                    sys.executable,
                    replay.__file__,
                    '--bag',
                    str(bag),
                    '--log',
                    str(log),
                    '--output',
                    str(output),
                ],
                text=True,
            )
            self.assertLessEqual(len(cli.splitlines()), 10)
            self.assertEqual(
                json.loads(output.read_text())['cycles'][0]['odom_stop_summary'][
                    'linear_threshold_exceeded'
                ],
                0,
            )


if __name__ == '__main__':
    unittest.main()
