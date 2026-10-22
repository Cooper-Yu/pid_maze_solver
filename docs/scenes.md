# Scene contracts

## Entry and data ownership

`tools/maze_entry.py` is installed as the public `pid_maze_solver` executable.
The shared C++ target is installed internally as `pid_maze_solver_task5`.
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

The supplied scene 2 file has `validated: false` and `test_until: P15`. The standard
`ros2 run pid_maze_solver pid_maze_solver 2` entry therefore permits the full route
for authorized development testing. P13's prior hardware failure remains unresolved;
the expanded scope is not evidence of route, clearance or obstacle-avoidance acceptance.
All route points, headings, gains and the backend 4 cm turn-drift limit are unchanged.
`--destination` selects a prefix and cannot bypass a file's declared test scope.
P03_turn aliases P03_after_turn. A restricted file with `test_until: P12` still rejects
P15 execution while unvalidated. No final half-turn is added by this frontend.

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
math. Dry-runs and rejection of a route beyond its declared test scope do not move
a robot. Backend synthetic-feedback tests and real-robot verification are separate;
a wrapper build or a successful prefix does not complete Task6.
## Experimental same-process supervised mode

`ros2 run pid_maze_solver pid_maze_solver 2 --destination P12 --supervised`
prepares once and then waits for `next` before every action. This explicit mode
uses an enforced laser guard and caps linear/angular speed at 0.03 m/s / 0.10 rad/s.
The manifest is restricted to the selected destination; in this example `next`
cannot continue beyond P12, even though the supplied file permits development through P15.
`back_plan` then `back TOKEN` may undo only the last completed translation while
retaining every bend and original P01. Turns and partial motions cannot be undone.
`quit`/EOF closes the session; exit 0 is orderly closure, not route completion.
No restart/resume or old-session loading is provided. The normal noninteractive
entry is unchanged. This mode is local engineering support, not validated physical
P12-to-P01 recovery; strict missing-ray stops remain. See the backend package's
`docs/route_session.md` for the contract and evidence boundary.

## Scene 3 / simulation reverse contract

`reverse_waypoints_sim.yaml` requires scene=3, frame=P15 and
`parameters.route_direction=reverse`. All fourteen step namespaces P15_P14 through
P02_P01 are required and normalized into traversal order. The same C++ controller
selects reverse defaults/labels; all PID, TF/scan, stale-feedback, collision and
settling code is shared. This entry forces `use_sim_time=true` and rejects unchecked
ROS overrides; change a complete validated YAML through `--route-file` instead.

Start at a stopped Task5 P15 after its final clockwise half-turn, without reset or
repositioning. The first accepted stopped pose anchors the relative route. Startup
is not localization: an arbitrary pose is not evidence that P15 has been reached.
Scene 3 retraces nominal Task5 geometry with headings opposite each outbound leg.
Its final P01 heading differs by pi from the original start. The shipped route
has no terminal half-turn. Scene 4 is rejected before a backend is started; no
hardware return or odometry-reset recovery is implied by this simulation route.