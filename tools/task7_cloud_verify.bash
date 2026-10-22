#!/usr/bin/env bash
# Run only against the already started local simulation in dedicated domain 147.
source /opt/ros/humble/setup.bash
source /home/user/ros2_ws/install/setup.bash
set -eo pipefail
export ROS_DOMAIN_ID=147 ROS_LOCALHOST_ONLY=1 ROS2CLI_NO_DAEMON=1
export IGN_PARTITION=task7_cloud_verification_147 GZ_PARTITION=task7_cloud_verification_147
out="$HOME/task7_cloud_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$out"
printf '%s\n' "$out" > "$HOME/task7_cloud_latest.txt"
controller_pid='' recorder_pid=''
stop_group() {
  pid=$1
  [ -n "$pid" ] || return 0
  kill -INT -- -"$pid" 2>/dev/null || true
  for attempt in $(seq 1 30); do
    kill -0 "$pid" 2>/dev/null || break
    sleep .1
  done
  kill -TERM -- -"$pid" 2>/dev/null || true
  sleep .2
  kill -KILL -- -"$pid" 2>/dev/null || true
  wait "$pid" 2>/dev/null || true
}
cleanup() {
  result=$?
  trap - EXIT INT TERM
  stop_group "$controller_pid"
  stop_group "$recorder_pid"
  printf '%s\n' "$result" > "$out/harness_exit_code.txt"
  printf 'TASK7_HARNESS_EXIT=%s\nEVIDENCE=%s\n' "$result" "$out"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
repo=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
pgrep -af 'gzserver|ign gazebo|ignition-gazebo|gz sim' > "$out/simulator_processes.txt"
git -C "$repo" rev-parse HEAD > "$out/source_head.txt"
# Controller-service discovery is not a motion gate; require live feedback below.
for topic in clock odometry/filtered scan; do
  name=${topic//\//_}
  timeout --signal=INT 10s ros2 topic echo "/$topic" --once > "$out/preflight_$name.yaml"
done
ros2 topic info /cmd_vel > "$out/publishers_before.txt"
grep -q '^Publisher count: 0$' "$out/publishers_before.txt"
setsid ros2 topic echo /cmd_vel geometry_msgs/msg/Twist > "$out/commands.yaml" 2> "$out/commands.log" < /dev/null &
recorder_pid=$!
sleep 3
kill -0 "$recorder_pid"
for scene in 1 3; do
  timeout --signal=INT 10s ros2 topic echo /odometry/filtered --once > "$out/scene${scene}_start_odom.yaml"
  date -u +%s > "$out/scene${scene}_wall_start.txt"
  setsid timeout --signal=INT --kill-after=20s 1200s ros2 run pid_maze_solver pid_maze_solver "$scene" > "$out/scene${scene}.log" 2>&1 &
  controller_pid=$!
  set +e
  wait "$controller_pid"
  result=$?
  set -e
  date -u +%s > "$out/scene${scene}_wall_end.txt"
  controller_pid=''
  printf '%s\n' "$result" > "$out/scene${scene}_exit.txt"
  if [ "$result" != 0 ]; then tail -n 5 "$out/scene${scene}.log"; exit "$result"; fi
  sleep 2
  ros2 topic info /cmd_vel > "$out/scene${scene}_publishers_after.txt"
  grep -q '^Publisher count: 0$' "$out/scene${scene}_publishers_after.txt"
  timeout --signal=INT 10s ros2 topic echo /odometry/filtered --once > "$out/scene${scene}_end_odom.yaml"
  printf 'SCENE%s_EXIT=0\n' "$scene"
done
stop_group "$recorder_pid"
recorder_pid=''
python3 "$repo/tools/task7_cloud_report.py" "$out"
