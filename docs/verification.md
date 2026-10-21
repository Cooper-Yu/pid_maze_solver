# Local verification — 2026-10-09

Environment: Ubuntu-22.04, ROS2 Humble, Gazebo Fortress; isolated domain 191,
maze scene from `husarion_office_gz cp18_local.launch.py`, standard spawn.
No real hardware was driven. Task1–4 repositories/tags were not modified.

## Observed results

- `colcon build --packages-select pid_maze_solver`: successful.
- `colcon test --packages-select pid_maze_solver`: four GTests passed.
- `ROS_DOMAIN_ID=198 python3 test/runtime_guards.py`: stale odom/scan,
  blocked movement and invalid quaternion tests passed; each stopped and exited 2.
- Full `ros2 run pid_maze_solver pid_maze_solver`: P02 through P15 reached,
  `Route completed; stopped.`, exit 0. Maximum reported stopped position error
  0.013545 m; maximum reported heading error 0.000678 rad. These are odom-based
  arrival errors, not independent ground-truth accuracy or peak transit errors.
- Supplied destination yaw is retained, including lateral movement at P07/P09/P12.
- P05 needed bounded clearance adjustment before completing its turn.
- Final telemetry command is zero. No independent Gazebo contact sensor was
  recorded for this run, so no physical wall-contact certification is claimed.
- clang-format-14, bundled clang-tidy 23 with system clang-14 resource directory,
  and Doxygen checked; no project diagnostics from the final checks.

Evidence: [controller log](evidence/controller.log), [exit code](evidence/exit_code.txt),
[start feedback](evidence/start_odom.yaml). Authoritative full telemetry/log directory:
`training_notes/checkpoint18/robot_control_rosbot_xl/code_lab/runtime_logs/task5_local_20261009_214344`.
The later AngularPid-to-AxisPid rename/doc cleanup changed no control arithmetic;
the renamed source rebuilt and passed the unit and runtime guard tests.

## Repairs observed during development

Raw scan chassis returns caused an initial stop: the known chassis self-filter
box is now applied. A full rectangular envelope falsely filled empty wheel/body
corners: the guard now uses the union of model-derived boxes. A tight P05 turn
needs bounded clearance translation. At P06, full repulsion cancelled attraction
outside the arrival tolerance: tapering correction near the goal removed the
stall in the complete run. Hard obstacle checks still apply.

## Remaining acceptance boundary

Cloud maze run, visual/contact observation and official marking remain pending.
No Task5 tag or independent learner PASS is created. The route origin must match
the demonstrated start, and sensor/model geometry must match this environment.
This sampled local guard is not a global path planner or collision guarantee.

## Follow-up laser calibration

The subsequent nominal-route survey stopped at P06 (OBSTACLE_BLOCKED, exit 2).
A separate XY-only configuration adjusts P04/P06 without changing yaw. The
contact-instrumented maze run completed all 14 legs, exit 0, zero classified
wall contact points, with 37 transient obstacle holds (P06: 11, P08: 8, P14: 18)
and no clearance translation. This is local evidence, not a cloud acceptance.
See [full survey](laser_survey.md); six GTests now include override validation.


## Relative-action default

The default now generates poses from named turn/forward/left actions and uses gated side-wall correction. See [motion verification](motion_steps.md) and [raw summary](evidence/motion_steps/summary.json). Nine GTests and three runtime guards passed. The first action trial completed with 924 obstacle-hold log entries; shortening the P06 approach by 30 mm and compensating on P07_P08 was retested without weakening the guard.


## Staged optimization (CP18-C028)

Only the P10 approach offset and per-segment centering switch are retained. Full runs completed in 108.03 and 106.91 seconds with zero classified wall contacts. Duplicate-stop reuse and selected-segment acceleration failed at P14 and were reverted. Full comparison and failure evidence: [optimization report](optimization.md). Existing single-run completion evidence does not certify P14 robustness under timing changes.

## P14 follow-up verification

The route-data repair passed two full contact-instrumented runs at 0.12 and 0.11 m/s, 14 legs and exit 0 each, zero wall contact points. See [P14 report](p14_clearance.md) for commands, limits, point-cloud evidence and 10 passing unit tests. Cloud acceptance remains pending.


## CP18-C030 final performance verification

Two complete local runs: 88.104/87.585 seconds, 14 legs, exit 0, zero obstacle-hold logs and zero classified wall contacts. Fourteen GoogleTests and all three runtime guard fixtures pass. See [staged evidence and limits](performance90.md). Cloud acceptance remains pending; no Task5 tag.

The final YAML matches the C++ route defaults; the parity check passes and rejects the historical 0.558/0.528 m drift. Earlier YAML trial snapshots preserve that difference.


## CP18-C031 terminal clockwise turn

Full default route plus terminal action completed in 94.825 s, exit 0, wall contact points 0. Terminal turn 6.640 s, error -0.006940 rad, all linear commands zero. Earlier P10 had four short obstacle holds; terminal action had none. See [full evidence](final_turn.md). Fifteen unit tests and three runtime guard fixtures passed; cloud acceptance pending.

## 2026-10-22 Scene entry and file contract

Ubuntu-22.04 / ROS2 Humble, base HEAD b2e47ab; task5 still dereferences to
75df8bb488dd97eadc513d16fb827c593e488a51. The C++ control source was not edited.
The public scene frontend, strict YAML loader, two installed scene files and
internal legacy executable name were added. Scene 2 preserves the d2ec797 route
and blocks the unresolved full route with validated=false / test_until=P12.

Commands run locally:

- `python3 test/test_scene_entry.py`: 10 tests passed, including fake child exit 7,
  SIGINT cancellation returning 130, actual custom-YAML payload and temp cleanup.
- `ruff check tools/maze_entry.py tools/scene_route.py test/test_scene_entry.py`:
  passed; Ruff formatting applied only to those new Python files.
- `bash /tmp/verify_pid_scenes.bash`: Humble colcon build and CTest passed
  (15 existing GoogleTests plus 10 Python contract tests); installed scene 1/2
  dry-runs succeeded; installed scene 2 full execution returned 2 before a backend
  because the route remains unvalidated. `git diff --check` passed.

The first verification shell re-sourced the overlay with nounset enabled and
stopped at COLCON_TRACE before tests; the shell was corrected to disable nounset
around sourcing. A scene-1 YAML edit was caught by parser tests and regenerated
structurally before verification. Neither issue changed the C++ controller.

No robot, ROS motion node, deployment or push was performed in this frontend
verification. The separate distance_controller backend is being integrated and
must pass its own synthetic-feedback/cleanup checks before this can be treated as
an operational end-to-end route. Laser observation only is explicit; no wall
avoidance, P13 repair, repeated full route, final-turn or course PASS is claimed.
## 2026-10-22 Installed frontend/backend integration

`bash /tmp/pid_scene_integration.bash` ran the actual installed
`ros2 run pid_maze_solver pid_maze_solver 2 --waypoints SCENE.yaml --destination P04`
entry, its generated manifest, and the installed distance_controller backend.
ROS_LOCALHOST_ONLY=1 and domain 191 isolated a synthetic odom/raycast-scan fixture;
no hardware or external ROS network was used. The temporary complete scene YAML
had short P02/P03 legs, a P03 turn of .15 rad, and a bent P04 path. Production
waypoints_real.yaml and its validated=false/test_until=P12 state were unchanged.

Evidence root: `/tmp/pid_scene_integration/1792613955278106284/`.
Each case contains runtime.log, points.jsonl, and the temporary input scene.yaml.

| Case | Observed process result | Verified terminal behavior |
| --- | --- | --- |
| success | 0 | precise P01 handoff, P01/P02/P03/P03_after_turn/P04 records, route_completed |
| stale | 2 | route_failed: Odom stale during wall action; no completion |
| drift | 2 | route_failed: Turn center drift exceeds 4 cm; no completion |
| cancel | 130 | SIGINT forwarded, route_failed cancellation; no completion |
| prepare_cancel | -15 from the outer ros2 launcher | SIGTERM during preparation, route_failed cancellation; no completion |

All five cases checked the final command was zero when commands existed, no
remaining /cmd_vel publisher, and release of the owned controller lock. A signal
result of -15 is subprocess's representation of launcher termination by SIGTERM,
not successful route completion. Success P04 was x=.0791804440, y=.0250763229 m,
yaw=8.4887537 deg against (.08,.03,.15 rad), within the existing 5 mm/.01 rad limits.

The backend model source/installed SHA256 for this run was
`fad35ab2c4572837d139cd17afd581153f530dc95682016f8ad82bf7c141aacf`.
Its reviewed reconnect repair subdivides only the changed first edge when needed;
all original points[1:] remain a suffix. This preserves the 5 cm representation
bound without straightening a supplied bend or relaxing the 2 cm/1 deg reconnect
and 4 cm turn-center guards. The frontend source required no change for integration.

This establishes the installed execution/exit/cleanup chain under synthetic
feedback. It does not validate P13 geometry, active laser obstacle avoidance,
physical repeatability, or full Task6 acceptance.
## 2026-10-22 supervised single-segment local verification

Environment: WSL Ubuntu-22.04 / Humble, localhost-only domain 192, synthetic
scan/odom/TF; no cloud or physical robot. The installed official frontend was
used with a complete temporary YAML and `--supervised`, not a fake stdin success
parser. `bash /tmp/route_session_ros.bash` builds both packages and executes
`distance_controller/test/verify_route_session.py` with a bounded timeout.

Evidence `/tmp/route_session_ros/1792617367284358424/`:

- success: outward translation, token-confirmed return to original P01, exit 0.
- return_stale: odometry removed during return, exit 2, no returned event.
- return_scan_stale: scans withheld during return, exit 2, no returned event.
- return_cancel: SIGINT during return, frontend exit 130, no returned event.
- turn_boundary: completed turn then back_plan rejected without moving, quit exit 0.

Every case verified final zero, no remaining cmd_vel publisher, released process
lock, enforced guard audit, and absence of `route_completed`. Exit 0 in this mode
means orderly session closure, not full-route acceptance. The final optional
polyline-length deadline change was unit-tested (0.8 m return gets 110 s, default
unchanged at 30 s) and the rebuilt installed roundtrip rechecked at
`/tmp/route_session_ros/1792617536181642942/success/runtime.log`.

14 new state/runtime tests passed and are registered as CTest test_route_session;
13 existing backend and 10 console tests passed. Frontend tests now total 11.
Humble build, Ruff formatting/check and git diff --check passed. The five-case
fixture's 13-branch dispatch has a narrow PLR0912 exemption because it describes
explicit fault-injection states; production code passes without size exemptions.

The guard's strict missing-ray restriction is unchanged. No turn undo, partial
recovery, old-session adoption, P12-to-P01 full return or physical route safety
is claimed. No production waypoint, PID, drift threshold, task tag or validation
gate changed. No commit or push was performed by this implementation agent.
