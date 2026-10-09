# Task5 runtime and obstacle-hold reduction

The user target is a full local route below 90 seconds with fewer actual
obstacle holds. This is not an official course scoring claim. Each policy change
was followed by a complete instrumented maze run before the next change.

## Current policy

- P07/P08/P09 shift 30 mm right: P06_P07 right travel 0.37 m, P08_P09 right
  travel 0.615 m. P09_P10 left travel 0.06 m compensates at P10. P07_P08 disables centering so the planned margin
  is not cancelled. P10 and later nominal destinations and all headings stay
  fixed. The previous P14 corner repair remains.
- complete_stage() qualifies every destination and real turn using unchanged
  position/yaw/velocity conditions, held continuously for stop_hold=0.20 ROS s.
  Startup still requires 0.40 s stopped feedback. An unchanged planned heading
  can reuse the preceding qualified stop only with actual heading error <0.01 rad,
  planar speed <=0.01 m/s, angular speed <=0.02 rad/s and fresh odom/scan <=0.5 s.
  No actual turn, destination qualification or final stop is skipped.
- General cruise and global max_speed are 0.24 m/s. P02_P03 and P13_P14 stay
  at 0.12 m/s; P06_P07, P07_P08, P08_P09 and P11_P12 use 0.20 m/s. Named
  `steps.<name>.max_speed` values must be finite and positive; global max_speed
  can only reduce a segment's limit. Translation acceleration is 0.40 m/s^2.
- translation_limit() samples up to 0.12 m along the current body-frame target
  direction. Predicted footprint clearance below 40 mm caps cruise at 0.12 m/s.
  The arrival cap is 0.12 + 0.6 * max(0, d - 0.12), with remaining distance d in m.
  Axis PID reads this cap, and corrected body-velocity norm is capped again.
- Maximum yaw rate/acceleration remain 0.6 rad/s and 0.6 rad/s^2. PID gains,
  15 mm hard clearance, 0.48-second commanded/measured-motion prediction,
  position/yaw tolerances and feedback watchdogs remain unchanged. The preview
  is anticipatory slowing, not a substitute for safe() or a collision guarantee.

fixed_points compatibility retains defaults 0.12 m/s, 0.25 m/s^2 and 0.40 s hold.
For a conservative motion_steps comparison, override max_speed:=0.12,
max_acceleration:=0.25 and stop_hold:=0.4. Geometry and qualified stop reuse remain.

## Staged results

| Trial | Wall elapsed (s) | Legs reached | Exit | Obstacle-hold logs | Wall contact points |
| --- | --- | --- | --- | --- | --- |
| [p14_default](evidence/optimization/p14_default/summary.json) | 107.404 | 14 | 0 | 2 | 0 |
| [p08_space](evidence/optimization/p08_space/summary.json) | 106.445 | 14 | 0 | 0 | 0 |
| [reuse_after_clearance](evidence/optimization/reuse_after_clearance/summary.json) | 102.946 | 14 | 0 | 0 | 0 |
| [speed_nearest_wall](evidence/optimization/speed_nearest_wall/summary.json) | 100.786 | 14 | 0 | 0 | 0 |
| [speed_preview40](evidence/optimization/speed_preview40/summary.json) | 96.406 | 14 | 0 | 0 | 0 |
| [acceleration04](evidence/optimization/acceleration04/summary.json) | 94.125 | 14 | 0 | 0 | 0 |
| [hold020](evidence/optimization/hold020/summary.json) | 89.724 | 14 | 0 | 0 | 0 |
| [hold020_yaml_repeat](evidence/optimization/hold020_yaml_repeat/summary.json) | 90.726 | 14 | 0 | 0 | 0 |
| [yaw_acceleration10](evidence/optimization/yaw_acceleration10/summary.json) | 91.187 | 14 | 0 | 0 | 0 |
| [cruise024](evidence/optimization/cruise024/summary.json) | 87.906 | 14 | 0 | 0 | 0 |
| [cruise024_yaml_repeat](evidence/optimization/cruise024_yaml_repeat/summary.json) | 92.444 | 14 | 0 | 10 | 0 |
| [p10_entry](evidence/optimization/p10_entry/summary.json) | 88.104 | 14 | 0 | 0 | 0 |
| [p10_entry_yaml_repeat](evidence/optimization/p10_entry_yaml_repeat/summary.json) | 87.845 | 14 | 0 | 0 | 0 |
| [p10_entry_yaml_matched](evidence/optimization/p10_entry_yaml_matched/summary.json) | 87.585 | 14 | 0 | 0 | 0 |

All listed completed runs had nonzero floor contacts as a positive control for
the contact observer. Elapsed time runs from the controller's first log to route
completion, excluding simulator startup/build time. Command lines, configuration
snapshots, source patches and controller logs accompany each summary. Patches
are relative to f592b6e, so include the previously retained P10/P14 changes.

The omnidirectional 80 mm slowdown policy was too conservative for parallel
walls. The first 0.20 m/s cruise / 0.20 s hold pair took 89.724 and 90.726 s, so
was not accepted as repeatably sub-90. Increasing angular acceleration to 1.0
rad/s^2 took 91.187 s and was reverted. No warning was disabled, filtered or
throttled differently to improve these counts. The 0.25-second hold proposal
was not executed; its value must not be presented as tested.

Fourteen GoogleTests cover route generation/compensation, footprint/preview
geometry, invalid segment caps and immediate PID cap reductions. Stale feedback,
obstacle and invalid-quaternion runtime fixtures also stop and exit as expected.
Full runs supplement those checks; cloud timing, changed placement, sensor/model
variation and real hardware remain unverified. No Task5 tag is created.


## Final repeated configuration

With the P09 entry shifted 30 mm right and compensated at P10, the default run
completed in **88.104 s**, and the same configuration through YAML completed in
**87.585 s**. Both reached all 14 legs, exited 0, recorded zero OBSTACLE_HOLD
entries and zero classified wall contacts. Maximum arrival errors were
0.013186 m / 0.000861 rad and 0.013202 m / 0.000817 rad respectively.

The preceding 0.24 m/s trial repeated at 92.444 s with 10 P10 obstacle-hold entries,
all from measured-motion prediction. P09's right scan distance was about 0.271 m.
Increasing the P09 entry margin, with compensation at P10, addressed this local
corner condition; increasing speed alone did not. Earlier results are retained.
The final pair is about 18% faster than the 107.404-second preceding default.
Two local runs support this result, not a universal/cloud timing guarantee.

Commands in the already running maze (source the workspace first):

```bash
ros2 run pid_maze_solver pid_maze_solver
ros2 run pid_maze_solver pid_maze_solver --ros-args --params-file \
  "$(ros2 pkg prefix pid_maze_solver)/share/pid_maze_solver/config/motion_route.yaml"
```

The bounded verification runner also starts the isolated contact-instrumented
simulator and observers; its startup/cleanup time is excluded from the metric.


## Configuration parity correction

The historical YAML runs through p10_entry_yaml_repeat used P12_P13=0.558 m,
while the bare default was 0.528 m. They are configuration variants, not strictly
identical repeats. The 87.845-second trial is retained under its original label.
The shipped YAML now uses 0.528 m. The final comparison above uses p10_entry
and p10_entry_yaml_matched (88.104 / 87.585 s).

`python3 test/check_motion_config.py` compiles a small reader of MotionStep
defaults and compares every field of all 14 named segments with YAML. The current
configuration passes; a temporary copy with the historical 0.558 m discrepancy
was rejected at P12_P13.forward_m. This check requires g++ and PyYAML.
