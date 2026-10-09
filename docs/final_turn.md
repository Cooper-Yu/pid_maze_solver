# Final clockwise half-turn

P15 arrival -> stopped qualification -> clockwise 180-degree rotation -> stopped
qualification -> zero command and successful exit. The target uses the actual
P15 arrival heading, while route waypoints remain unchanged.

`begin_final_turn()` clears PID state and accumulated rotation once. `odom()`
adds the shortest signed increment between adjacent accepted yaw samples.
`tick()` uses `-pi - final_rotation_`, without wrapping that remaining angle.
This avoids choosing the opposite half-circle after initial positive noise.
Existing odom discontinuity checks reject jumps before accumulation.

The terminal action never enters MOVE and cannot request clearance translation.
It retains feedback, time, obstacle and stopped-velocity checks. A partial
`last_point` run ends at that point. `final_clockwise_turn:=false` reproduces the
previous full-route finish. No additional positional initialization is performed.

## Local verification

Ubuntu-22.04 / Humble, isolated contact-instrumented maze, default command:
14 travel legs plus terminal turn completed, exit 0, **94.825 wall seconds**
(controller first log through completion; simulator startup excluded).
Terminal turn took **6.640 s**, signed rotation **-3.134652 rad**, final error
**-0.006940 rad (about -0.398 degrees)**. All terminal command samples had zero
linear x/y; final command was zero. No terminal obstacle hold or wall contact
was observed. The earlier P10 approach produced **4 obstacle-hold entries**;
these remain visible and demonstrate that prior zero-hold runs do not guarantee
zero holds on every repeat. Contact observer: 97763 packets, 811245 floor points
as a positive control, zero wall points.

15 GoogleTests, route/YAML parity, stale/obstacle/invalid-feedback fixtures,
clang-format/tidy, YAML and Doxygen checks passed. Source and runtime evidence:
[summary](evidence/optimization/final_clockwise/summary.json),
[terminal metrics](evidence/optimization/final_clockwise/terminal.json),
[log](evidence/optimization/final_clockwise/controller.log).
Cloud acceptance remains pending; no Task5 tag is created.
