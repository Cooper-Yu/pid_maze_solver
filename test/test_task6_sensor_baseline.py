"""Pure subprocess mocks; never create ROS nodes or call the real recorder."""

import json
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import unittest

SCRIPT = Path(__file__).resolve().parents[1] / 'tools/task6_sensor_baseline.bash'

ROS = """#!/usr/bin/env python3
import json,os,sys
from pathlib import Path
a=sys.argv[1:]
with open(os.environ['CALLS'],'a') as f: f.write(json.dumps(a)+'\\n')
if a[:2]==['node','list']: print('/robot_state_publisher\\n/unrelated_last_node')
elif a[:2]==['topic','info']:
 n={'/scan':'real_lidar','/scan_filtered':'real_filter'}.get(a[2],'other')
 print('Node name: '+n+'\\nNode namespace: /robot\\nEndpoint type: PUBLISHER')
elif a[:2]==['param','dump']:
 if os.environ.get('MISSING') and a[2]=='/robot/real_filter': sys.exit(1)
 print(a[2]+':\\n  ros__parameters:\\n    readonly_fixture: true')
elif a[:3]==['run','tf2_ros','tf2_echo']: print('- Translation: [0.020, 0.000, 0.173]')
else: raise SystemExit('FORBIDDEN MOCK CALL '+repr(a))
"""

RECORDER = """#!/bin/bash
exec python3 - "$@" <<'PY'
import json,os,sys,time
from pathlib import Path
a=sys.argv[1:]
assert a[0]=='record'
root=Path(a[a.index('--root')+1]); session=root/'fixture'; (session/'bag').mkdir(parents=True)
(root/'latest_session.txt').write_text(str(session))
Path(os.environ['RECORDER_PID']).write_text(str(os.getpid()))
if os.environ.get('SLOW'): time.sleep(30)
(session/'bag/metadata.yaml').write_text('mock: true\\n')
topics=['/scan','/scan_filtered','/tf','/tf_static','/odometry/filtered','/rosbot_xl_base_controller/odom']
result=dict(closed_metadata=True,exception=None,stop_reason='duration_limit',message_counts={t:1 for t in topics})
if os.environ.get('NO_SCAN'): result['message_counts']['/scan']=0
(session/'result.json').write_text(json.dumps(result))
PY
"""


class BaselineMockTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix='task6_baseline_mock_')
        self.root = Path(self.temp.name)
        self.workspace = self.root / 'ws'
        tools = self.workspace / 'src/distance_controller/tools/task6_ops'
        tools.mkdir(parents=True)
        (tools / 'run.bash').write_text(RECORDER)
        bindir = self.root / 'bin'
        bindir.mkdir()
        (bindir / 'ros2').write_text(ROS)
        (bindir / 'ros2').chmod(0o755)
        overlay = self.root / 'setup.bash'
        overlay.write_text(f'export PATH="{bindir}:$PATH"\n')
        self.env = dict(
            os.environ,
            TASK6_WORKSPACE=str(self.workspace),
            TASK6_OVERLAY=str(overlay),
            CALLS=str(self.root / 'calls.jsonl'),
            RECORDER_PID=str(self.root / 'recorder.pid'),
        )
        self.output = self.root / 'baseline'

    def tearDown(self):
        self.temp.cleanup()

    def run_helper(self, **extra):
        return subprocess.run(
            ['bash', str(SCRIPT), str(self.output), '10'],
            env=dict(self.env, **extra),
            text=True,
            capture_output=True,
            timeout=25,
        )

    def test_complete_actual_discovered_names_and_read_only_calls(self):
        done = self.run_helper()
        self.assertEqual(done.returncode, 0, done.stderr)
        self.assertEqual(done.stdout.splitlines(), [str(self.output), 'COMPLETE'])
        result = json.loads((self.output / 'result.json').read_text())
        self.assertEqual(result['parameter_nodes']['driver'], ['/robot/real_lidar'])
        self.assertEqual(result['parameter_nodes']['filter'], ['/robot/real_filter'])
        self.assertFalse(result['motion_authorized'])
        calls = [json.loads(s) for s in (self.root / 'calls.jsonl').read_text().splitlines()]
        self.assertTrue(all(a[:2] != ['topic', 'pub'] and a[:2] != ['param', 'set'] for a in calls))
        refused = self.run_helper()
        self.assertNotEqual(refused.returncode, 0)

    def test_missing_parameters_remain_explicit_and_recording_is_preserved(self):
        done = self.run_helper(MISSING='1')
        self.assertEqual(done.returncode, 2, done.stderr)
        result = json.loads((self.output / 'result.json').read_text())
        self.assertIn('filter:missing_or_ambiguous_parameter_dump', result['issues'])
        self.assertTrue(result['recording']['closed_metadata'])

    def test_closed_bag_without_raw_scan_is_incomplete(self):
        done = self.run_helper(NO_SCAN='1')
        self.assertEqual(done.returncode, 2, done.stderr)
        self.assertIn(
            '/scan:no_recorded_messages',
            json.loads((self.output / 'result.json').read_text())['issues'],
        )

    def test_interrupt_cleans_only_owned_mock_recorder(self):
        proc = subprocess.Popen(
            ['bash', str(SCRIPT), str(self.output), '10'],
            env=dict(self.env, SLOW='1'),
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            start_new_session=True,
        )
        try:
            deadline = time.monotonic() + 15
            pidfile = self.root / 'recorder.pid'
            while not pidfile.exists() and proc.poll() is None and time.monotonic() < deadline:
                time.sleep(0.05)
            self.assertTrue(pidfile.exists())
            owned = int(pidfile.read_text())
            proc.send_signal(signal.SIGINT)
            proc.communicate(timeout=10)
            self.assertNotEqual(proc.returncode, 0)
            self.assertEqual((self.output / 'status.txt').read_text().strip(), 'INTERRUPTED')
            with self.assertRaises(ProcessLookupError):
                os.kill(owned, 0)
        finally:
            if proc.poll() is None:
                proc.kill()
                proc.communicate()


if __name__ == '__main__':
    unittest.main()
