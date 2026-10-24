#!/bin/bash
set -e
source /opt/ros/humble/setup.bash
source /home/cooper/ros2_ws/install/setup.bash
cd /home/cooper/task8_dev/pid_maze_solver
python3 -m pytest -q test/test_task8_runner.py test/test_task8_reverse_geometry.py test/test_scene_entry.py
