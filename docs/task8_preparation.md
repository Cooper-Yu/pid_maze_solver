# Task8 reverse hardware preparation

This branch generates geometry only. Scene 4 remains rejected by the executable;
no hardware motion, deployment or Task8 acceptance is implied.

The input is the complete Task6 real interpolation including the P15 left half-turn.
The candidate has 14 translations, six intermediate turns and one final turn:
P15 -> P14 -> ... -> P01 -> **counterclockwise +180 degrees** -> stopped exit.
Every intermediate interpolation point is traversed in reverse order. Translation
headings equal the outbound headings plus pi; each intermediate turn returns to
the preceding outbound segment's opposite heading. Lateral displacement must be
computed in that heading, rather than blindly negating a body-axis command.

`P01_after_turn` is a signed +pi operation. Its unwrapped target is 2*pi, restoring
the original outbound P01 heading. A future backend must explicitly preserve the
positive direction across angle wrapping and qualify the stopped final pose before
reporting completion. Reaching P01 alone must not report full success.

The P15 frame here is a **nominal geometry frame**, parallel to the original P01
axes with its origin at the configured P15 coordinate. It is not localization and
must not be blindly anchored to the measured current pose after a drifting turn.
The runtime handoff must preserve the verified forward P01-to-odom reference and
check actual P15 arrival, terminal turn, stopped state, sensor epoch and exclusive
control. On odometry reset, stop; do not load an old odom reference. Independent
reverse startup needs a separately verified P15 localization contract.

## Next implementation gates

1. Scene 4 parsing and a handoff contract retaining the forward geometric anchor.
2. Reuse the real path follower, explicit signed terminal turn, feedback and 4cm
   turn drift protection. Preserve the fixed Task6 release during development.
3. Run deterministic backend normal/cancellation/feedback-reset cases locally.
4. Deploy through local verification -> GitHub -> cloud. Test P15->P14->P13 first,
   then extend to P01 plus the final left half-turn and stopped cleanup.
5. Record obstacle policy and actual contact evidence separately. Neither a
   generated route nor simulation passes certify hardware clearance or Task8.

## Offline command

From this checkout with Python3/PyYAML:

```bash
python3 tools/task8_reverse_geometry.py waypoints/waypoints_real.yaml /tmp/task8_candidate.yaml
python3 -m pytest -q test/test_task8_reverse_geometry.py
```

The output must not already exist. The checked-in candidate is deliberately
`validated: false`. The generator has no ROS imports or control publishers.

## 2026-10-25 current baseline and P15 initialization

Task6 official scene2 completed under cb3021f and task6 tag is fixed there. This branch merges that baseline and regenerates the reverse candidate from its latest P12/P13/P14/P15 geometry. Earlier reverse YAML was historical and must not be deployed as current. Scene4 remains disabled until start and cancellation/feedback contracts are implemented and verified.

P15 needs its own stopped/fresh localization and heading verification. Do not reuse P01 right26cm/rear28cm targets. For a same-epoch forward-to-reverse handoff, retain the measured P01-to-odom transform, compare actual P15 against the nominal current route endpoint and current fresh wall observations, and reject clock/frame/epoch discontinuity. A moved robot or reset epoch needs independent P15 localization. Merely setting the current pose as the reverse origin hides forward terminal-turn drift.

The reversed interpolation is an initial geometric candidate, not proof of reverse clearance. Reverse approach headings change the swept footprint and wall side. Keep Task8-specific point adjustments separate from the task6 tag; validate P15-P14-P13 first with odom/laser/camera evidence before the full P01 route. Retain P01 final counterclockwise180 and original stop/feedback/4cm guards.
