#!/usr/bin/env python3
"""Observe the known WSL real-clock fault without the test clock shim; never robot-side."""

import json
import os
from pathlib import Path
import signal
import time

from verify_task6_return_auto import AutoFixture, rclpy


def main():
    if os.environ.get('TASK6_TEST_REALTIME_OFFSET_NS') or os.environ.get('LD_PRELOAD'):
        raise RuntimeError('Host-clock fault observation must not use a preload shim')
    root = Path('/tmp/task6_host_clock_fault') / str(time.time_ns())
    root.parent.mkdir(exist_ok=True)
    rclpy.init()
    fixture = None
    try:
        fixture = AutoFixture(root, False)
        fixture.wait(lambda: fixture.child.poll() is not None, 180)
        assert fixture.child.returncode == 2, fixture.log.read_text()[-2000:]
        assert 'Odom stale' in fixture.log.read_text()
        fixture.wait(lambda: not fixture.node.get_publishers_info_by_topic('/cmd_vel'), 8)
        fixture.wait(fixture.released, 8)
        assert fixture.commands and fixture.commands[-1] == (0.0, 0.0, 0.0)
        os.killpg(fixture.recorder.pid, signal.SIGINT)
        fixture.wait(lambda: fixture.recorder.poll() is not None, 8)
        session = Path((root / 'evidence/latest_session.txt').read_text().strip())
        recorded = json.loads((session / 'result.json').read_text())
        assert recorded['closed_metadata']
        gaps = [json.loads(s) for s in (root / 'tick_gaps.jsonl').read_text().splitlines()]
        assert any(g['wall_gap'] - g['mono_gap'] >= 0.4 for g in gaps)
        result = dict(
            exit_code=2,
            last_command=list(fixture.commands[-1]),
            cmd_publishers=0,
            motion_lock_released=True,
            closed_metadata=True,
            clock_gaps=gaps,
            return_wrapper_started=False,
            production_limits_unchanged=True,
        )
        (root / 'fault_result.json').write_text(json.dumps(result, indent=2))
        print('HOST_CLOCK_REJECTION_PASS', root, flush=True)
    finally:
        if fixture:
            if fixture.recorder.poll() is None:
                os.killpg(fixture.recorder.pid, signal.SIGINT)
                fixture.recorder.wait(timeout=8)
            fixture.close()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
