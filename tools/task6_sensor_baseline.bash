#!/usr/bin/env bash
# Read-only baseline: no velocity publisher, parameter changes, or hardware restart.
set -eo pipefail
workspace="${TASK6_WORKSPACE:-$HOME/ros2_ws}"
source /opt/ros/humble/setup.bash >/dev/null
source "${TASK6_OVERLAY:-$workspace/install/setup.bash}" >/dev/null
set -u
if [[ "${1:-}" != --bounded ]]; then
  exec timeout --signal=INT --kill-after=5s 120s bash "${BASH_SOURCE[0]}" --bounded "$@"
fi
shift
if [[ $# -lt 1 || $# -gt 2 || "$1" != /* || ! "${2:-12}" =~ ^(10|11|12|13|14|15)$ ]]; then
  printf '%s\n' 'Usage: bash task6_sensor_baseline.bash /absolute/NEW_DIRECTORY [10..15]' >&2
  exit 2
fi
out="$1"
seconds="${2:-12}"
recorder="$workspace/src/distance_controller/tools/task6_ops/run.bash"
[[ -f "$recorder" ]] || { printf '%s\n' 'Recorder source missing' >&2; exit 2; }
mkdir -m 700 -- "$out"  # Refuse every existing path, including a symlink.
out="$(cd -- "$out" && pwd -P)"
active=''
cleanup() {
  if [[ -n "$active" ]]; then
    kill -INT -- "-$active" 2>/dev/null || true
    sleep 1
    kill -TERM -- "-$active" 2>/dev/null || true
    sleep 1
    kill -KILL -- "-$active" 2>/dev/null || true
    wait "$active" 2>/dev/null || true
  fi
}
trap cleanup EXIT
trap 'printf "%s\n" INTERRUPTED >"$out/status.txt"; exit 130' INT TERM
capture() {
  local limit="$1" name="$2" code=0
  shift 2
  setsid timeout --foreground --signal=INT --kill-after=3s "$limit" "$@" >"$out/$name" 2>&1 &
  active=$!
  wait "$active" || code=$?
  active=''
  printf '%s\t%s\n' "$name" "$code" >>"$out/commands.tsv"
}
printf '%s\n' COLLECTING >"$out/status.txt"
printf '%s\n' "$out"
capture 5 nodes.txt ros2 node list
for entry in scan scan_filtered odometry/filtered rosbot_xl_base_controller/odom cmd_vel; do
  capture 5 "topic_${entry//\//_}.txt" ros2 topic info "/$entry" -v
done
# Discover driver/filter from the actual publisher records; never guess a node name.
for role in driver filter; do
  topic=scan
  [[ "$role" == driver ]] || topic=scan_filtered
  awk -v role="$role" '
    $1=="Node" && $2=="name:" {name=$3}
    $1=="Node" && $2=="namespace:" {ns=$3}
    $1=="Endpoint" && $2=="type:" && $3=="PUBLISHER" {
      sub(/\/$/,"",ns); print role "\t" ns "/" name
    }' "$out/topic_$topic.txt" >>"$out/parameter_nodes.tsv"
done
while IFS= read -r node; do
  if [[ "$node" =~ (^|/)robot_state_publisher[^/]*$ ]]; then
    printf 'robot_state_publisher\t%s\n' "$node" >>"$out/parameter_nodes.tsv"
  fi
done <"$out/nodes.txt"
sort -u "$out/parameter_nodes.tsv" -o "$out/parameter_nodes.tsv"
index=0
while IFS=$'\t' read -r role node; do
  [[ "$node" == /* && "$node" != *UNKNOWN* ]] || continue
  index=$((index + 1))
  # Limit ambiguous/duplicate graphs; retain discovery but do not spend an unbounded time.
  [[ "$index" -le 6 ]] || break
  printf '%s\t%s\tparams_%s_%s.yaml\n' "$role" "$node" "$role" "$index" >>"$out/parameter_files.tsv"
  capture 5 "params_${role}_${index}.yaml" ros2 param dump "$node"
done <"$out/parameter_nodes.tsv"
capture 6 tf_base_laser.txt ros2 run tf2_ros tf2_echo base_link laser
export TASK6_OVERLAY="${TASK6_OVERLAY:-$workspace/install/setup.bash}"
capture "$((seconds + 12))" recorder.log bash "$recorder" record --root "$out/evidence" --seconds "$seconds" --image-hz 2 --max-mib 30
# Read only recorder JSON and closed metadata, never query a live SQLite database.
python3 - "$out" <<'PY'
import json, sys
from pathlib import Path
root = Path(sys.argv[1])
issues = []
commands = dict(line.split('\t') for line in (root/'commands.tsv').read_text().splitlines())
for name, code in commands.items():
    if code != '0' and not (name == 'tf_base_laser.txt' and code == '124'):
        issues.append(name + ':exit=' + code)
roles = {}
files = root/'parameter_files.tsv'
for line in files.read_text().splitlines() if files.exists() else []:
    role, node, name = line.split('\t')
    if commands.get(name) == '0' and 'ros__parameters:' in (root/name).read_text():
        roles.setdefault(role, []).append(node)
for role in ('driver', 'filter', 'robot_state_publisher'):
    if len(roles.get(role, [])) != 1:
        issues.append(role + ':missing_or_ambiguous_parameter_dump')
if 'Translation:' not in (root/'tf_base_laser.txt').read_text():
    issues.append('base_link_to_laser_transform_missing')
result = {}
try:
    session = Path((root/'evidence/latest_session.txt').read_text().strip()).resolve(strict=True)
    if not session.is_relative_to((root/'evidence').resolve()):
        raise ValueError('Recording session escaped output directory')
    result = json.loads((session/'result.json').read_text())
    if not result.get('closed_metadata') or not (session/'bag/metadata.yaml').is_file():
        issues.append('closed_metadata_missing')
    if result.get('exception') or result.get('stop_reason') != 'duration_limit':
        issues.append('recorder_did_not_reach_duration_limit')
    for topic in ('/scan', '/scan_filtered', '/tf', '/tf_static', '/odometry/filtered', '/rosbot_xl_base_controller/odom'):
        if result.get('message_counts', {}).get(topic, 0) <= 0:
            issues.append(topic + ':no_recorded_messages')
except (OSError, ValueError) as error:
    issues.append('recording:' + str(error))
status = 'INCOMPLETE' if issues else 'COMPLETE'
(root/'result.json').write_text(json.dumps(dict(status=status, issues=issues, parameter_nodes=roles, recording=result, motion_authorized=False), indent=2)+'\n')
(root/'status.txt').write_text(status+'\n')
print(status)
sys.exit(2 if issues else 0)
PY