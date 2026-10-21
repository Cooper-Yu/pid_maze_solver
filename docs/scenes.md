# Scene contracts

## Entry and data ownership

`tools/maze_entry.py` is installed as the public `pid_maze_solver` executable.
The unchanged C++ target is installed internally as `pid_maze_solver_task5`.
`tools/scene_route.py` loads YAML with duplicate-key rejection and validates the
selected scene before starting any backend. `--dry-run` never resolves or starts
a motion executable and prints the resolved file-derived data.

Scene 1 reads `waypoints/waypoints_sim.yaml`: the complete Task5 named relative
steps and policy. The frontend writes these values as a temporary ROS parameter
file and passes it to the original controller. Subsequent user `--ros-args`
remain after that file and retain ROS override precedence. Mode-dependent global
speed, acceleration and hold are intentionally left to the unchanged C++ defaults,
so an existing fixed_points override does not inherit motion_steps-only defaults.
The provided relative-step values match `config/motion_route.yaml`.

Scene 2 reads `waypoints/waypoints_real.yaml`: all 20 path/turn actions through P15,
with points in the fixed P01 frame and headings relative to P01's yaw. Explicit
turn actions separate heading changes. The current coordinates reproduce the
reviewed d2ec797 teaching route; P13, P14, and P15 have not been shifted by this work.

Validation checks schema/scene/frame, complete label order, finite numbers,
boolean validation status, exact path continuity from P01=(0,0), explicit heading
changes, nonzero edges no longer than 5 cm, and declared test scope. A path does
not rebase on its last measured position. The backend separately validates its
manifest and retains its bounded post-turn reconnect rule.

## Run and test scope

```bash
# Read only: inspect either scene before any backend exists.
ros2 run pid_maze_solver pid_maze_solver 1 --dry-run
ros2 run pid_maze_solver pid_maze_solver 2 --dry-run --destination P03_turn

# Motion: only during an authorized robot test with appropriate space/feedback.
ros2 run pid_maze_solver pid_maze_solver 2 --destination P02 --output /tmp/scene2_trial.json
```

`--waypoints PATH` selects a reviewed alternative scene file. Scene 2's optional
`--speed` (0.01–0.12 m/s) and `--turn-speed` (0.02–0.60 rad/s) pass through to the shared
backend. When omitted, backend defaults apply. `--output` is the backend session
JSON/audit destination. Unchecked ROS parameter overrides are rejected for scene 2.

The supplied scene 2 file has `validated: false` and `test_until: P12`. Full scene 2
execution is therefore rejected. `--destination` selects a prefix and cannot bypass
that gate. P03_turn aliases P03_after_turn. After actual route and protection
acceptance, an explicitly reviewed validation-state change can enable the standard
`ros2 run pid_maze_solver pid_maze_solver 2` full-route entry. Changing this field
is not itself evidence of acceptance. No final half-turn is added by this frontend.

## Backend and exit contract

Scene 2 starts exactly one child process:

```
task6_route_runner --scene 2 --route-json RESOLVED.json --laser-log-only
```

The manifest contains only schema=1, scene=2, frame=P01 and the selected actions.
It is generated from YAML, exists for the child's lifetime, and is removed after
completion or failure. No stdin commands or terminal-output success heuristics
are used. The backend owns precise P01 preparation, command ownership, movement,
stop/cleanup and per-point audit. This frontend never creates ROS publishers.

A backend nonzero exit is propagated. Local validation/file errors return 2.
SIGINT/SIGTERM are forwarded to the owned child process group; cancellation returns
128+signal even if the child cleans up with exit 0. If cancellation is ignored,
only that owned group receives TERM after 10 seconds and KILL after 15 seconds.
Other controllers and ROS processes are never stopped by this frontend.

There is no midroute restart/resume contract. Every new backend execution must
perform its normal P01 preparation. Do not launch scene 1 and scene 2 together.

## Verification boundary

The entry tests cover real file consumption, duplicate/malformed data, changed
coordinates, route gates, backend exit propagation, cancellation and temporary-file
cleanup using fake owned processes. Existing C++ tests protect the preserved Task5
math. Installed dry-runs and rejection of the unvalidated full route do not move
a robot. Backend synthetic-feedback tests and real-robot verification are separate;
a wrapper build or a successful prefix does not complete Task6.