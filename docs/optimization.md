# Task5 staged performance verification

This page preserves its historical trial state. See [current performance verification](performance90.md) for later speed, acceleration and stop-reuse changes.
Each category is built and run over the complete local contact-instrumented maze before the next category is changed. The reference is f592b6e. PID gains, arrival tolerances and hard clearance thresholds remain unchanged.

1. P10 turn space: the stopped scan predicts only 8 mm minimum clearance across the next 90-degree rotation. A 25 mm body-left displacement predicts 33 mm clearance. Add 30 mm left to P09_P10 and shorten P10_P11 by 30 mm so later nominal targets are preserved.
2. Reuse an already-qualified stop only when the next planned heading is unchanged, the current heading is still within tolerance, measured velocity is low and feedback is fresh. Keep startup, real-turn and final-stop qualification.
3. Introduce named segment speed limits, conservatively testing 0.15 m/s only on selected long forward segments with near-goal slowdown. Keep narrow/strafe segments at their existing limit. Do not change PID gains during this experiment.

Each run records elapsed wall time from controller logs, per-stage time, final position/heading errors, obstacle-hold log entries, and the independent contact monitor. Log entries are not a count of distinct collisions. One local pass is not proof of cloud acceptance or universal repeatability.
## Acceptance decisions

The first P10-offset trial completed in 115.61 s but centering cancelled the intended offset. The corrected per-segment centering setting completed in 108.03 s, with P11 TURN reduced from 12.46 to 4.86 s. Duplicate-hold reuse and segment speed trials both stopped at P14 with OBSTACLE_BLOCKED and exit 2; neither change is retained. Their patches and logs remain under evidence/optimization. These failures do not isolate causality, and expose unresolved P14 clearance sensitivity. Final repeat verification of the P10-only version follows.

## Measured comparison

| Trial | Complete time (s) | Reached legs | Exit | Wall contacts | Hold log entries |
|---|---:|---:|---:|---:|---:|
| baseline | 115.21 | 14 | 0 | 0 | 66 |
| p10_space | 115.61 | 14 | 0 | 0 | 57 |
| p10_space_fixed | 108.03 | 14 | 0 | 0 | 23 |
| reuse_stop_failed | incomplete | 12 | 2 | 0 | 162 |
| segment_speed_failed | incomplete | 12 | 2 | 0 | 392 |
| final_repeat | 106.91 | 14 | 0 | 0 | 12 |

The retained P10-only version passed twice (108.03 and 106.91 seconds). Final maximum stopped errors were 0.014203 m and 0.001695 rad, below the unchanged 0.015 m / 0.01 rad thresholds. This is local functional evidence only. The two rejected trials expose P14 sensitivity; neither reduced waiting nor higher speed is enabled. Do not infer either change was independently proven to cause the blockage.

Final code: 9 GTests; stale/obstacle/invalid feedback fixtures passed on the same retained control behavior. The rejected speed variant separately passed 11 GTests but failed the maze run. clang-format, clang-tidy, Doxygen and YAML checks passed. No Task5 tag or cloud acceptance.

## Subsequent P14 repair

[P14 corner clearance](p14_clearance.md) now passes two complete runs at 0.12 and 0.11 m/s. The rejected waiting/speed features remain disabled; their historical failed evidence above is unchanged.
