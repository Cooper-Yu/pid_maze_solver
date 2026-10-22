#!/usr/bin/env python3
"""Audit recorded cloud simulation results; no ROS publishers or subprocesses."""

import json
import math
from pathlib import Path
import re
import sys

import yaml


def odom(path):
    documents = [d for d in yaml.safe_load_all(path.read_text()) if d is not None]
    if len(documents) != 1:
        raise ValueError('Expected one odom sample')
    pose = documents[0]['pose']['pose']
    q = pose['orientation']
    yaw = math.atan2(2 * (q['w'] * q['z'] + q['x'] * q['y']), 1 - 2 * (q['y'] ** 2 + q['z'] ** 2))
    return pose['position']['x'], pose['position']['y'], yaw


def audit(root):
    results = {}
    for scene in (1, 3):
        log = (root / f'scene{scene}.log').read_text()
        points = [int(p) for p in re.findall(r'Reached P(\d+):', log)]
        expected = list(range(2, 16)) if scene == 1 else list(range(14, 0, -1))
        code = int((root / f'scene{scene}_exit.txt').read_text())
        assert code == 0 and points == expected and 'Route completed; stopped.' in log
        assert ('Final clockwise turn completed:' in log) == (scene == 1)
        assert 'Publisher count: 0' in (root / f'scene{scene}_publishers_after.txt').read_text()
        results[str(scene)] = {'exit': code, 'points': points}
    commands = [d for d in yaml.safe_load_all((root / 'commands.yaml').read_text()) if d]
    last = commands[-1]
    assert all(last[k][axis] == 0 for k in ('linear', 'angular') for axis in ('x', 'y', 'z'))
    start = odom(root / 'scene1_start_odom.yaml')
    end = odom(root / 'scene3_end_odom.yaml')
    position = math.dist(start[:2], end[:2])
    angle = math.atan2(math.sin(end[2] - start[2] - math.pi), math.cos(end[2] - start[2] - math.pi))
    result = {
        'phases': results,
        'return_odom_difference_m': position,
        'opposite_heading_error_rad': angle,
        'terminal_command_zero': True,
        'source_head': (root / 'source_head.txt').read_text().strip(),
        'wall_contact_measurement': 'Unavailable; no contact classifier in this cloud harness',
        'official_score': 'Not evaluated',
        'engineering_pass': position <= 0.04 and abs(angle) <= 0.03,
    }
    (root / 'summary.json').write_text(json.dumps(result, indent=2) + '\n')
    print(f'RETURN_ODOM_M={position:.6f}')
    print(f'HEADING_ERROR_RAD={angle:.6f}')
    print('TERMINAL_ZERO=true')
    assert result['engineering_pass'], result
    print('CLOUD_FUNCTIONAL_PASS=true')
    return result


if __name__ == '__main__':
    audit(Path(sys.argv[1]))
