# Turn settling diagnostics

TURN_SETTLING logs every 0.5 steady seconds during TURN and FINAL_TURN, including
the previously silent stopped-hold branch. These logs do not change control,
PID state, tolerances, watchdogs, commands or the 0.20 ROS-second hold.

| Field | Meaning |
| --- | --- |
| reason=angle | Heading error has not entered 0.01 rad tolerance |
| reason=angular_speed | Angle is acceptable; measured odom yaw rate exceeds turn_stop_yaw_rate (default 0.05 rad/s) |
| reason=linear_speed | Angle/yaw rate acceptable; planar speed exceeds 0.01 m/s |
| reason=hold | All instantaneous gates pass; continuous hold is not complete |
| reason=ready | All gates and the hold pass at this sampled tick |
| angle_ok/angular_ok/linear_ok | Individual flags expose simultaneous failures |
| hold_ros | Actual continuously qualified hold / required duration, ROS seconds |
| stage_ros/stage_wall | Time since this stage started, in ROS and steady seconds |

The first failing gate determines reason; all flags remain visible. Sampling can
miss short transitions. Hold is zero after a failed gate, so occasional resets
are distinguishable from a fixed dwell. A stage_wall value much greater than
stage_ros supports slow simulation, but these logs alone do not identify CPU,
network or estimator causes. measured_wz is feedback, not the sent command.

## Verification and limits

15 GoogleTests and the isolated synthetic gate fixture passed. The fixture
intentionally provides inconsistent pose/velocity phases to exercise diagnostics;
it is not a physical robot model. A real local maze run through P04 also completed
(exit 0), covering both initial 45-degree turns. The cloud long wait was not
reproduced locally. No gain/tolerance change is justified by this instrumentation
alone. See [timings](evidence/turn_diagnostics/summary.json),
[fixture](evidence/turn_diagnostics/fixture.log),
[local maze log](evidence/turn_diagnostics/maze_first_turns.log).

The submitted earlier cloud log completes the route and final half-turn, exit 0,
but lacks measured speeds in the silent hold interval. New cloud diagnostics are
needed to separate convergence, feedback-speed decay and simulation slowdown.

## Configurable turn stop threshold

The trial default is 0.05 rad/s for TURN and FINAL_TURN only. Startup and MOVE
retain 0.02 rad/s. Heading tolerance remains 0.01 rad and the default motion-step
hold remains 0.20 ROS seconds. Override with --ros-args -p turn_stop_yaw_rate:=0.02
to compare the previous behavior. This tuning does not fix or prove estimator lag.


Local trial: 14 translations plus final clockwise half-turn completed, exit 0; 94.481 steady seconds from P02 target. Final turn error -0.008275 rad. Cloud timing/accuracy remain pending. See [trial evidence](evidence/turn_stop_005/summary.json). The full trial used an initial build with startup also at 0.05; startup was subsequently restored to 0.02 and build/fixture checks rerun. Route logic was identical.
