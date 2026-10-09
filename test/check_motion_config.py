#!/usr/bin/env python3
"""Compare the shipped motion YAML with compiled C++ defaults (requires g++/PyYAML)."""

import json
import math
from pathlib import Path
import subprocess
import tempfile

import yaml


def main():
    """Read actual MotionStep defaults and reject missing, extra or different YAML fields."""
    root = Path(__file__).resolve().parents[1]
    config = yaml.safe_load((root / 'config/motion_route.yaml').read_text())
    steps = config['pid_maze_solver']['ros__parameters']['steps']
    source = r"""
#include <iomanip>
#include <iostream>
#include "pid_maze_solver/motion.hpp"
int main() {
  std::cout << std::setprecision(17);
  for (const auto &s : maze::default_steps)
    std::cout << s.name << ' ' << s.turn * 180 / M_PI << ' ' << s.forward
              << ' ' << s.left << ' ' << s.side_centering << ' ' << s.max_speed << '\n';
}
"""
    with tempfile.TemporaryDirectory(prefix='motion-config-') as folder:
        source_path = Path(folder) / 'defaults.cpp'
        binary = Path(folder) / 'defaults'
        source_path.write_text(source)
        subprocess.run(
            ['g++', '-std=c++17', '-I', str(root / 'include'), str(source_path), '-o', str(binary)],
            check=True,
        )
        lines = subprocess.check_output([str(binary)], text=True).splitlines()
    defaults = {}
    for line in lines:
        name, turn, forward, left, centering, speed = line.split()
        defaults[name] = {
            'turn_deg': float(turn),
            'forward_m': float(forward),
            'left_m': float(left),
            'side_centering': centering == '1',
            'max_speed': float(speed),
        }
    assert set(defaults) == set(steps), 'Named segments differ'
    for name, expected in defaults.items():
        actual = {'side_centering': True, **steps[name]}
        assert set(expected) == set(actual), f'{name}: fields differ'
        for field, value in expected.items():
            assert math.isclose(value, actual[field], abs_tol=1e-12), (
                f'{name}.{field}: C++={value}, YAML={actual[field]}'
            )
    print(json.dumps({'result': 'PASS', 'matching_segments': len(defaults)}))


if __name__ == '__main__':
    main()
