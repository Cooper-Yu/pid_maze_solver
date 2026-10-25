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

## P15 localization experiment, 2026-10-25

Historical TeachingBook records do not contain a continuous odom publisher/reset
lease. Pose and scan similarity are consistency evidence, not proof of an unchanged
odom epoch. Do not use the unintegrated handoff draft to authorize motion.

`task8_p15_localization.py` estimates a small relative pose from nonparallel front
and right wall lines, using the existing chassis/wheel support geometry. It retains
the recorded terminal offset from nominal P15 and constructs a new transform using
current odometry. It rejects bad fits, inconsistent rotations and estimates beyond
5cm/5deg. This is bounded local registration near visually confirmed P15, not global
localization or a complete initialized-motion contract.

`task8_p15_probe.py` is a read-only experiment: use the exact terminal scan stamp
from a closed bag, collect current stopped scans, fit both walls and preserve raw
estimates. It never creates a velocity publisher or starts a controller. Its result
always reports `stable_start_verified: false`; scene4 remains disabled. Multi-frame
stability, mounting provenance, localization confidence and motion handoff still
need verification before the estimate may authorize an inverse route.

Local checks: 24 reverse-geometry/localization tests passed; Ruff format/check and
Git whitespace checks passed in Ubuntu-22.04. This does not certify hardware motion.
## Scene4 stopped-wall initialization and prefix runner

The new initializer qualifies twelve distinct laser scans over at least one second,
with at least ten odometry timestamps, fresh stopped dual sources, unchanged live
publisher identities and mounting TF checked against the closed recording. Every
laser pose estimate must remain inside 2cm/1deg of nominal P15. Two half-window
medians must agree within 5mm/.5deg; actual odometry must stay stationary within
5mm/.01rad. The current-epoch origin is derived only after these checks. The old
unobserved odom gap is not treated as continuous. No startup publisher is created.
Missing or unreliable data waits up to twenty seconds, then refuses motion.

User authorized choosing the most reliable walls: the front/right pair is used
because rear fits at 20/12/10/8deg failed the actual recorded-data coverage or
consensus checks. Front/right matching is bounded local registration near confirmed
P15, not global localization. Current odometry carries the calibrated frame during
motion; it does not establish the initial physical position by itself.

Scene4 requires explicit --source-book and --source-bag, and defaults to a P13
prefix. It records P15 without overwriting the nominal origin, reconnects the first
path and every post-turn path, and uses the existing controller protections. Prefix
completion is not full success; only P01 plus signed CCW+pi can finish the full route.
The Task6 tag and forward geometry remain unchanged. Hardware validation is pending.

## 2026-10-26 06:00 forward candidate

Operator request: P10->P11 forward +3 cm; P11->P12 forward +3 cm. P12->P13 is straight from the new P12, with P13 shifted longitudinally with P11 and no previous lateral tail. P14 starts at new P13 and rejoins its unchanged endpoint; P15, all headings, terminal turn and safety limits are unchanged. Reverse YAML remains the separately calibrated 754adcd candidate; do not regenerate it from this forward edit. Physical clearance remains to be tested.


## 2026-10-26 P13 next trial after forward protection stop

4fa916d reached P13 but its turn exceeded the unchanged4cm drift guard; publisher0 and recorder closure verified, reverse never started. Operator now requests P13 body-right3cm and body-forward2cm. At heading-pi this means P01-frame delta(-.02,+.03)m. Approach interpolates from unchanged P12 to new P13, superseding the earlier straight-only request. P14 starts continuously and rejoins unchanged endpoint; other actions and reverse YAML are unchanged. No claim of clearance improvement before hardware verification.
