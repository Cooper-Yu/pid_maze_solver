#!/usr/bin/env bash
# Foreground-only cooperative run. task6_run owns signals, zeroing and evidence cleanup.
set -eo pipefail
source /opt/ros/humble/setup.bash
cd "$HOME/ros2_ws"
expected_distance=${1:?usage: task6_cooperative_run.bash DISTANCE_COMMIT PID_COMMIT}
expected_pid=${2:?usage: task6_cooperative_run.bash DISTANCE_COMMIT PID_COMMIT}
[[ "$expected_distance" =~ ^[0-9a-f]{7,40}$ && "$expected_pid" =~ ^[0-9a-f]{7,40}$ ]]
[[ $(git -C src/distance_controller rev-parse HEAD) == "$expected_distance"* ]]
[[ $(git -C src/pid_maze_solver rev-parse HEAD) == "$expected_pid"* ]]
run_dir=""
finish() {
  rc=$?
  trap - EXIT
  printf '\nTASK6_EXIT=%s\nRUN=%s\n' "$rc" "$run_dir"
  if [[ -n "$run_dir" ]]; then
    python3 - "$run_dir" <<'PY'
from pathlib import Path
import sys
run = Path(sys.argv[1])
p = run / 'result.json'
if p.exists():
    print(p.read_text())
p = run / 'motion.log'
if p.exists():
    lines = p.read_text().splitlines()
    errors = [line for line in lines if '[ERROR]' in line]
    print('FIRST_CONTROLLER_ERROR:', errors[0] if errors else 'none')
    print('\n'.join(lines[-20:]))
PY
  fi
  exit "$rc"
}
trap finish EXIT
colcon build --packages-select distance_controller pid_maze_solver
source install/setup.bash
run_dir="$HOME/task6_runs/feedback_pause_$(date +%Y%m%d_%H%M%S)"
printf '%s\n' "$run_dir" > "$HOME/task6_coop_latest.txt"
cutoff=$(python3 -c 'from datetime import datetime,timedelta,timezone; print((datetime.now(timezone.utc)+timedelta(minutes=20)).isoformat())')
ros2 run pid_maze_solver task6_run prepare \
  --run-dir "$run_dir" --deadline "$cutoff" \
  --max-seconds 600 --destination P15 --speed .12 --turn-speed .20
ros2 run pid_maze_solver task6_run preflight --run-dir "$run_dir"
# User invocation means physical P01 is ready and teleop has been released.
ros2 run pid_maze_solver task6_run run --run-dir "$run_dir" --at-p01