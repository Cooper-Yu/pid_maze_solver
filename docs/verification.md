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
