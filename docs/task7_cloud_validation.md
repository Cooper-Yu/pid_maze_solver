# Task7 final cloud validation — 2026-10-22

Functional acceptance passed for core commit
`7bbc4ccb187c75e6e812b5e8fb383d5f2722e77e` after the user-required P01
counterclockwise half-turn was added. A fresh world ran scene1 then scene3 without
resetting or teleporting between directions. Both processes returned zero.

| Phase | Ordered reached points | Final action | Wall seconds | Exit |
| --- | --- | --- | ---: | ---: |
| scene1 | P02 through P15, all 14 | Clockwise rotation -3.132492 rad, error -0.009100 rad | 228.603 | 0 |
| scene3 | P14 through P01, all 14 | Counterclockwise rotation +3.134459 rad, error +0.007134 rad | 216.583 | 0 |

Both phases logged `Route completed; stopped.`. The read-only summary reported
`FINAL_PASS=True` with the exact source commit above. The program retains PID,
feedback, laser, and stopped-pose guards. See `task7_requirements.md` for the
requirement-to-evidence mapping. The annotated `task7` tag marks this tested core
plus acceptance documentation; no post-test behavior changes are included.

Cloud evidence:

- `/home/user/task7_final_forward.log` and `.exit`
- `/home/user/task7_final_reverse.log` and `.exit`
- `/home/user/task7_final_deploy.log`
- `/home/user/task7_final_summary.json`
- `/home/user/task7_final_evidence.tar.gz`

Log SHA256:

- forward: `b2a1137871c3aeeba76d60ede8075ff3d21afaebc4dd6ad498ad1fb103f16007`
- reverse: `8f9e5888eeef441460dde6279897d3c31c8881978b0bcb29e24c3ce1bef619bb`

The same version also completed a full local Humble/Fortress round trip:
95.940/90.520 wall seconds, 30.500 mm filtered-odom return difference,
-0.001963 rad original-heading difference, zero final Twist, zero remaining
publishers, and zero classified wall contacts in both phases. Valid live contact
packets/floor contacts were present. This is separate local evidence; cloud wall
contacts/video were not recorded and are not claimed. These functional tests do
not constitute an official score or independent learner assessment.

Validation accompanying the core change: 17 GTest, 20 entry tests, 7 independent
geometry tests, and 15 existing Task6 operations tests passed; colcon build,
clang-format, Ruff, clang-tidy23 and Git diff checks passed. Illegal forward+CCW
and simultaneous CW+CCW configurations were rejected before startup.

## Earlier run before the final half-turn requirement

The following records remain historical evidence, not the final acceptance run.
# Task7 cloud simulation validation — 2026-10-22

The same cloud maze ran scene 1 followed immediately by scene 3, without resetting
or re-anchoring the simulated robot between directions. This is functional
verification of the delegated implementation, not an official score or an
independent learner assessment.

- Core deployed from 42f8c39; runtime helper checkout 3142c8b. Later 74e3075 only
  supplies explicit message types to the optional verification helper.
- ROS domain 147, localhost only, dedicated Gazebo partition
  `task7_cloud_verification_147`; no real robot connection.
- Forward reached P02 through P15 in order, completed its final clockwise
  half-turn, and logged `Route completed; stopped.`. Log wall duration: 257.743 s.
- Reverse reached P14 through P01 in order and logged `Route completed; stopped.`.
  Log wall duration: 234.602 s. Last target error: 0.012000 m / 0.002069 rad.
- Forward final half-turn error: -0.009284 rad.
- Cloud logs: `/home/user/task7_forward_long.log`,
  `/home/user/task7_reverse_long.log`, `/home/user/task7_direct_pair_summary.json`.
- Forward SHA256:
  `255eaaaa65deddc236f291d7f9da9a01560a8b41ec398bef231771a6bae9b748`.
- Reverse SHA256:
  `aa18bf6e55abb468ccf34082dd48e71684c63de2c0da02cb31afaac6986486d7`.

The earlier 120 wall-second run reached P07 and began P08 before SIGINT. It did
not demonstrate a route failure. Cloud output showed stage_ros=3.758 s versus
stage_wall=8.419 s. Each direction now has a 1200 wall-second outer budget; the
simulation helper has a 3600 wall-second budget. Existing controller feedback,
progress, and stop protections remain unchanged.

A controller service-list query timed out even though direct route execution
worked. It is no longer a blocking gate in the helper. An additional helper
preflight failed to infer a topic type; explicit ROS message types address that
CLI lookup dependency. These are harness issues, separate from route behavior.

Cloud wall contacts were not instrumented and no cloud video was saved. Do not
infer a zero-contact cloud result from completion logs. Local same-world
forward/reverse validation separately measured zero wall contacts and a
29.005 mm odometry return offset; see `task7_local_validation.md`.