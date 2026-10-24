#!/bin/bash
set -e
source /opt/ros/humble/setup.bash
source /home/cooper/ros2_ws/install/setup.bash
cd /home/cooper/task8_dev
colcon build --base-paths pid_maze_solver --packages-select pid_maze_solver --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
ros2 run pid_maze_solver pid_maze_solver 4 --source-book /not-used-book --source-bag /not-used-bag --dry-run > /tmp/task8_scene4_dry_run.json
python3 -m pytest -q /home/cooper/task8_dev/pid_maze_solver/test/test_task8_runner.py /home/cooper/task8_dev/pid_maze_solver/test/test_task8_reverse_geometry.py /home/cooper/task8_dev/pid_maze_solver/test/test_scene_entry.py
