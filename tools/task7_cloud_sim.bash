#!/usr/bin/env bash
# Dedicated Gazebo transport partition prevents discovery of an older shared world.
source /opt/ros/humble/setup.bash
source /home/user/ros2_ws/install/setup.bash
set -eo pipefail
export ROS_DOMAIN_ID=147 ROS_LOCALHOST_ONLY=1 ROS2CLI_NO_DAEMON=1
export IGN_PARTITION=task7_cloud_verification_147 GZ_PARTITION=task7_cloud_verification_147
export IGN_IP=127.0.0.1 GZ_IP=127.0.0.1
exec timeout --signal=INT --kill-after=15s 1800s ros2 launch rosbot_xl_gazebo simulation.launch.py mecanum:=True
