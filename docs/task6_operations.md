# Task6 bounded operations

`ros2 run pid_maze_solver task6_run` prepares and checks a fresh run and owns its
recording lifetime. It does not change the controller, route, gains, turn drift
limit or laser policy. The default destination is **P12**. Explicit longer development prefixes remain
limited by the current scene2 YAML `test_until` gate with `validated: false`. This is an observation-only engineering test;
it cannot satisfy the Task6 laser-avoidance or repeated-full-route acceptance.

## Preparation and explicit motion

After the reviewed Git update, build and source the workspace as usual. Both
packages must be in `~/ros2_ws/src`, with their Git history, plus the existing
`turn_controller` checkout. The source support scripts live at
`distance_controller/tools/task6_ops/`; they are versioned Git files and do not
require a separate cloud paste or installation.

For the 2026-10-22 11:00–14:00 Beijing booking:

```bash
ros2 run pid_maze_solver task6_run prepare --run-dir ~/task6_runs/am01 --deadline 2026-10-22T14:00:00+08:00
ros2 run pid_maze_solver task6_run preflight --run-dir ~/task6_runs/am01
```

For the 15:00–17:00 booking use a new directory and
`--deadline 2026-10-22T17:00:00+08:00`. The deadline is mandatory and timezone
aware. These commands do not initialize or move the robot. `prepare` records a
route preview, Git revisions, tracked source hashes, installed artifact hashes,
and exact equality of installed Python against source and the installed YAML
against source. Binary hashes identify the actual executable artifacts; they
are not proof of reproducible compilation from a Git commit. `preflight` only
subscribes and verifies fresh stopped odometry, finite laser returns, laser TF,
single stable odometry publishers, no velocity publisher, and an available
controller lock.

Only after confirming the robot is physically near P01, release teleop and run:

```bash
ros2 run pid_maze_solver task6_run run --run-dir ~/task6_runs/am01 --at-p01
```

`--at-p01` is a physical-location assertion, not permission to adopt the current
pose as P01. The official scene2 runner still performs its precise right-wall
26 cm preparation and captures a fresh P01. No old session, return file or old
odometry origin is accepted. A used run directory cannot be resumed. A stopped
P12 robot must first return through a separately validated procedure; it must
not start this command with a false P01 assertion.

During prepare, optional `--speed` (0.01–0.12 m/s) and `--turn-speed` (0.02–0.60 rad/s)
are finite-checked, stored in the manifest, and passed to the official entry.
Defaults remain 0.04 m/s and 0.10 rad/s. A 600-second development run can use
`--max-seconds 600 --speed 0.12 --turn-speed 0.20` with its actual booking cutoff.

The wrapper rechecks installed/source identity and fresh feedback on every run.
Reviewed tracked changes must be committed before `run`; read-only preparation
can inspect a dirty checkout. Source/version changes invalidate the prepared
manifest. There is no bypass flag. A larger destination remains subject to the
existing YAML validation gate. Scene1/Task5 and the existing official entry
retain their behavior.

## Recording and stop ownership

The wrapper starts the repository recorder and reset monitor with no TTY stdin,
separate logs, and owned process groups. Motion is admitted only after two
recorder health samples demonstrate increasing filtered/base odometry and scan
and image counts, plus at least one static TF message. The monitor records command/odometry/identity evidence; it does not
control the robot. Camera images are retained at at most 2 Hz, with an explicit
180 MiB recorder cap. This is not full-rate video or proof of camera viewpoint.

Before the official runner exists, evidence startup may wait up to two monotonic
seconds for an already observed scan whose source/receipt age exceeds .5 seconds.
This requires fresh stopped dual odometry in the original GIDs, no command
publisher, valid scan structure and a resolvable laser TF on every check. Three
strictly increasing fresh scan timestamps recover the wait; duplicate timestamps
do not count and another bad frame clears the count without extending the deadline.
The original 18-second evidence startup deadline and recorder-growth requirements
remain. Any odometry, identity, command-publisher, invalid scan, backward timestamp
or TF fault still fails immediately. Strict feedback is rechecked before spawning
the runner. After spawn, the original .5-second source/receipt protection applies
without this waiting allowance. Startup waiting grants no motion permission.

A per-user/domain operations lock prevents duplicate wrappers; the official
backend retains its controller lock and publisher exclusivity. During motion,
more than one velocity publisher, stale odometry/scan, changed odometry GID,
backward odometry timestamps, stopped evidence processes or stale recorder
counts cancel the official runner. Unknown laser sectors are not converted into
free space; this wrapper does not provide active geometric laser avoidance.

Ctrl-C, maximum duration or the booking cutoff send SIGINT to the owned official
frontend, which forwards it to the backend. Cleanup checks final zero command,
no remaining velocity publisher, and fresh stopped feedback in the original
odometry epoch before closing the recorder/monitor and checking metadata. A GID
change prevents claiming stopped-pose verification even if new feedback says
zero. Failed cleanup is recorded as failure; it is not disguised as success.

`--max-seconds` is set during `prepare`, defaults to 1200 and accepts 30–1680.
The recorder/watch allowance is this value plus 90 seconds (at most 1770).
The wrapper begins cleanup 90 seconds before the absolute booking deadline.
The nominal sequential cleanup bound is 25 + 8 + 25 + 25 = 83 seconds, assuming
OS scheduling and APIs return. A hung process is reported for manual stop; the
wrapper does not blindly kill a forwarding parent and orphan its controller.
A wrapper cannot guarantee a physical stop if its host/process itself crashes.

Each run contains `config.json`, `route_preview.json`, `preflight.json`, the
one-use `run.started.json`, `motion.log`, fresh `points.json/jsonl`, recorder and
monitor logs/evidence, and final `result.json`. Exit0 requires matching
`route_completed` at the chosen prefix, zero/release verification and closed bag
metadata. It does not represent Task6 completion.

## Verification

Ubuntu-22.04 / Humble, no cloud or robot. Unit tests cover admission/route gates,
source-vs-install mismatch, fresh stopped feedback, required recorder growth,
deadlines/cancellation, one-use sessions, original-GID stopped evidence, and
controller-zero-recorder cleanup ordering. `test/verify_task6_ops.py` is a
localhost-only harness using synthetic feedback and a mock controller with the
actual installed preflight/runtime and actual repository recorder/watch. It
checks success, cancellation, publisher replacement and feedback loss. This is
operations integration evidence; it does not replace the existing official
controller fixtures or real route verification.

Verified on 2026-10-22: 10 unit cases; registered installed-package CTests;
`/tmp/task6_ops_ros/1792620747361110514/` contains all four operations cases
with the required image/static-TF evidence gate. The final speed-forwarding
success rerun is `/tmp/task6_ops_ros/1792620809510505303/success/`; its saved
motion command explicitly contains `--speed 0.12 --turn-speed 0.2`.
All cases closed bag metadata and released the mock velocity publisher after
zero. GID-change and feedback-loss cases rejected trustworthy stopped-feedback
confirmation as intended. The installed `task6_run prepare` command also ran
successfully without starting ROS control. Ruff and `git diff --check` passed.

Failure diagnostics (2026-10-22): `failure_snapshot.json` captures cached feedback
and discovered endpoint identities before cleanup, then writes the captured data
after cleanup so filesystem I/O cannot postpone the stop signal. It includes
wrapper phase, wall/ROS/monotonic times, source/receipt ages, feedback counts,
last command and node/namespace/GID descriptors. `last_check` preserves the exact
publisher-count rejection even if graph discovery changes during cancellation.
Correlate its timestamp with `motion.log` and `points.jsonl`; an ops publisher
error during controller cleanup is not proof of an external teleop or the first
controller fault. Missing diagnostics are reported by `failure_snapshot_error`
without replacing the primary error or bypassing cleanup. Limits remain zero
publishers before motion, at most one while the official runner is active, and
0.5 seconds for feedback freshness. Fifteen unit cases cover these diagnostics,
including unchanged admission bounds and capture/write failures. These are local
unit results, not additional hardware verification.

2026-10-22 stopped evidence startup repair: 10 targeted startup tests plus 15
existing operations tests passed under Ubuntu-22.04/Humble; both registered
CTests passed. The existing real ROS recorder/watch fixture ran only
`TASK6_OPS_CASES=startup_recover,startup_timeout,active_scan_loss` in localhost
domain 194, evidence `/tmp/task6_ops_ros/1792670976021367023/`. A one-second scan
publication pause caused an observed stale interval, then at least three new
fresh scan stamps before successful controller admission. A sustained startup
pause failed after the fixed two-second wait with no controller/commands. Active
scan loss still failed immediately at the original .5-second receipt bound;
its final observed command was zero and graph publisher count zero, but the
strict cleanup stopped-feedback flag remained false because scan was still
missing. All three recorder metadata files closed. These are synthetic ROS
lifecycle checks, not real robot motion or obstacle-avoidance acceptance.

Probe callback processing retains its original single `spin_once(timeout_sec=.05)`
then makes at most 16 additional `timeout_sec=0` calls. A two-millisecond
monotonic drain budget prevents starting another call once exhausted. A callback
already executing cannot be preempted; neither this nor the original .05-second
spin is a hard real-time duration guarantee. No drain call waits for future data.
Source/receipt ages are still evaluated at the actual subsequent check time,
with the unchanged .5-second limit and all latched faults preserved.
`failure_snapshot.probe.last_pump` records total/drain elapsed seconds, additional
spin attempts and monitored callback count. Attempts are not necessarily executed
callbacks; the monitored count includes odom/scan/cmd but excludes TF/executor work.

This addresses a possible local ready-queue delay only. In coop122450, the cached
scan source stamp matched the preceding recorded filtered frame and two newer raw
frames lacked recorded filtered counterparts before the fault. The queue change
does not establish that this incident was a false positive or repair the observed
filtered gap. It does not switch to raw scans or automatically resume motion.

Local verification: `bash /tmp/check_probe_pump.bash` under Ubuntu-22.04/Humble
ran 43 pure tests and `ROS_DOMAIN_ID=194 ROS_LOCALHOST_ONLY=1 timeout 20s python3
test/verify_task6_probe_pump.py`. Report:
`/tmp/task6_probe_pump/1792672459081074346/report.json`. Real executor first
processed filtered odom, then raw odom and the already queued scan during the
nonblocking drain. Persistent scan loss still failed the original receipt check.
A continuously retriggered guard callback exited the drain after 3 calls; measured
drain times across the cases were 2.026/1.873/2.103 ms. These small overruns include
completion of an already started callback, not an added waiting allowance.
Synthetic localhost checks do not prove cloud scheduling or filtered availability.

## Observation-only scan tolerance (2026-10-22)

The user explicitly selected a 1.0-second scan source/receipt limit for the
observation-only official run. Only the running `Operation.execute` monitor uses
this exception. Default preflight, evidence startup and final stopped checks keep
0.5 seconds; both odometry topics always keep 0.5 seconds. Invalid/future stamps,
missing scan, invalid frames/ranges, identity changes and prolonged loss still
reject motion. Diagnostics record both effective scan and odometry limits.
The backend enables matching 1.0-second geometry freshness only after precise
P01 preparation, including the observation-only terminal half-turn. Enforced
supervised guards and preparation remain strict. This is not obstacle avoidance.
Full success requires canonical destination P15 and explicit completed
P15_after_turn evidence; preparation accepts either destination spelling.

## P10 forward-offset candidate (2026-10-22)

`scan_grace_20261022_153333` reached P10, then stopped while turning right at
about -47 degrees when filtered-odom center drift crossed 4cm. The operator
could not confirm contact; a photograph shows the slanted robot near a wall end,
which does not prove collision or its cause. Cleanup and evidence closure passed.
The candidate advances P10 by 0.020m along its existing -pi/2 heading: fixed-P01
XY changes from (1.3549013479064715,-1.8855925989094273) to
(1.3549013479064715,-1.9055925989094273). P09 and P11 endpoints, all other targets
and turn headings are unchanged. P09->P10 and P10->P11 are reinterpolated with
spacing <=5cm (8/10 points). This aims to clear the rear of the wall corner before
turning; it is a hardware-unverified candidate, not a confirmed collision repair.
All speed, scan policy and 4cm drift limits remain. Restart near verified P01;
no partial-turn resume or old odom reference. Task8 geometry must be regenerated
from the final accepted forward route before it can be integrated for execution.
## Owner-side feedback pause (2026-10-23)

After strict preparation, the observation-only route now binds both filtered and
raw odometry identities. It proactively holds zero at 0.35s odometry or 0.8s scan
age, before the existing outer 0.5s/1s movement admission checks. A pause lasts at
most 5s, with at most 10s cumulative per route, and never extends action deadlines.
The backend alone controls zero and release. Recovery requires both odometry
streams to provide at least three advancing, fresh, stopped samples spanning
0.3s, fresh scan/TF, unchanged identities/frames, and <=4cm pause-anchor drift.
The same fixed target/path and original turn center are retained; PID, limiter
and settling history reset. Invalid data, clock/pose jumps, competing publishers,
cancellation and exhausted budgets stop the run. This is not obstacle avoidance.

C++ precise initialization separately uses the same 5s bounded zero-hold for
scan/filtered-odom interruptions. It requires fresh stopped feedback before
reacquiring wall alignment/position, discards interrupted micro-targets, and
preserves final 5mm/0.5deg, travel, total60s and stage deadlines. The supervisor
still checks both odometry identities; raw-only initialization outages without
an owner hold remain strict failures. Preflight/final-stop admission is unchanged.

The read-only supervisor accepts old cached samples only after an explicit
`feedback_pause_started` event or initialization `FEEDBACK_RECOVERING` marker,
plus actual zero-command acknowledgement and unchanged command ownership.
`feedback_wait.jsonl` records observed holds. No hold event means strict previous
freshness, even with a long zero-command history. Recording health remains
required; feedback count growth may stall only during an acknowledged hold.
Initialization recovery logs and route pause events are retained in motion.log
and points.jsonl. Full-run completion still requires all21 actions including
P15_after_turn, successful cleanup and closed recording metadata.

Local isolated ROS fixtures verify temporary outage, zero hold, recovery,
identity changes, timeout and cancellation; joint owner/supervisor fixtures also
exercise the acknowledgement. Hardware validation remains pending, including
same-version three consecutive complete runs. The P10 forward2cm candidate is
unchanged. Task7 tag and Task8 candidate are unaffected.
Cooperative deployment shortcut: after pulling and reviewing both expected
commits, run `bash src/pid_maze_solver/tools/task6_cooperative_run.bash DISTANCE_SHA PID_SHA`.
It verifies the pinned versions, builds both packages, creates a new run, performs
preflight, then uses the already-confirmed physical P01. It has a 600s operation
limit and an explicit UTC deadline twenty minutes from invocation; the existing
wrapper reserves cleanup time. Invoke only after teleop is released. The EXIT
trap prints the run path, result and final controller lines; it creates no extra
velocity publisher. This shortcut does not return the robot or resume old odom.
### Stopped feedback wait before controller dispatch

`feedback_pause_20261022_165902` failed with filtered odom source age before any
recorded velocity command or controller log. The evidence-startup guard previously
allowed only scan age to wait; either odom aging past 0.5 s aborted immediately.
The exact failed phase/ages require that run's failure_snapshot for confirmation.

During evidence startup and the final pre-dispatch boundary, the supervisor now
keeps **zero velocity publishers** while a timestamp-age outage waits at most five
seconds. Cached samples are never motion-ready. Recovery requires both odometry
streams and scan to be fresh under the original 0.5 s limits, with at least three
distinct source stamps per stream over a 0.3 s observation interval. Each renewed
age outage resets this recovery evidence, not its five-second outage deadline.
The existing total startup limit of 18 s and run/cancellation/deadline checks remain.
Publisher identity/count, stopped velocities, TF, frames, invalid/future/backward
stamps and latched data faults remain mandatory throughout. Initial preflight and
normal active/final validation retain their existing strict rules; this is an
explicit stopped startup scope only. Recorder process health remains mandatory;
required count growth must be proved after feedback readiness before dispatch.

Regression: `test_task6_evidence_startup.py` plus `test_task6_ops.py`; live localhost
DDS fixture `ROS_DOMAIN_ID=195 ROS_LOCALHOST_ONLY=1 python3
 test/verify_task6_startup_feedback.py recover` (also `timeout` and `cancel`).
The live fixture asserts no `/cmd_vel` messages or publishers in all cases.
### P10 corner-clearance candidate after run 171305

Run `feedback_pause_20261022_171305` reached P10 within about 2.6 mm of the
configured endpoint, then stopped on the original 4 cm turn-center drift guard
at about 35 degrees of clockwise rotation. The operator suspected wall contact;
a single camera frame does not establish its cause. A closed-bag scan 81 ms before
the P10 pose record fitted the front wall at 129.1 mm body clearance. Side and rear
single-wall fits had no dominant consensus; this is unavailable geometry, not
proof of free space.

A read-only comparison of those observed points against the chassis/wheel rectangle
union during a clockwise quarter-turn found: front6/right6 cm gave only 1.1 mm
minimum model gap; front6/right8 gave 20.7 mm; front6/right10 gave 20.1 mm.
Front3/right8 and front3/right10 gave only 9.3 and 11.1 mm respectively. The
unshifted rotation gave -40.8 mm; the unshifted static model gap was already
-3.6 mm. These are counterfactual observed-point diagnostics, not certified physical
clearances: unseen space, model/mount errors and actual center drift remain unknown.
No swept-envelope gate or obstacle-policy change is introduced.

Select front6/right8 cm relative to P10's existing -pi/2 heading. In fixed P01
axes the new P10 is (1.2749013479064715, -1.9655925989094274), a delta of
(-0.08,-0.06) m from the previous candidate. Preserve every existing P10 approach
point, append three <=3.34 cm diagonal intervals, then turn. This keeps the earlier
P09 approach instead of spreading the extra right shift through its narrow section.
Reinterpolate P10->P11 at <=5 cm spacing (eight points); P09, P11's endpoint, every
other action and all headings remain unchanged. Path tracking can round the local
bend; the preserved approach points are nominal geometry, not an exact-stop claim.
The route remains validated:false and full-route hardware repeatability is pending.
Retain all speeds, feedback pauses, final tolerances, cancellation and 4 cm guard.
Regenerate Task8 from the final accepted forward geometry later.
### Superseding staged P10 approach after e10bede contact

The operator confirmed that e10bede contacted the wall while translating, before
its clockwise turn. Retaining the old P10 approach and adding displacement only
at its end was a planning error. The endpoint scan's model gap did not validate
the entire approach. Do not repeat that failed candidate.

The earlier closed recording 171305 was sampled along P09->P10. Model gaps after
right shifts of 0/2/4/6/8 cm were: at 0 cm forward, 2.82/0.82/-1.18/-3.18/-5.18 cm;
at 18 cm, 0.57/0.62/-1.38/-3.38/-5.38 cm; at 24.3 cm,
0.03/1.52/-0.48/-2.42/-2.60 cm; at 26.7 cm,
-0.62/0.93/-0.43/-0.57/-2.57 cm; at 30.1 cm,
-0.41/1.58/2.51/0.67/-0.35 cm. This supports a gradual small right offset through
the narrowing approach, followed by a larger offset as the right side opens.
These isolated observed-point comparisons do not certify the continuous corridor.

Replace only P10's interior points. Relative to the original near-straight approach,
reviewed nominal forward/right anchors in meters are (0.12,0), (0.18,0.010),
(0.24,0.014), (0.30,0.016), (0.34,0.038), then the existing new P10 endpoint.
Subdivide all edges to <=5 cm: P10 now has 13 points. P09, P10's final target,
P10's heading, P11 and every other action remain unchanged from e10bede.
The removed original near-wall P10 anchor is not a required waypoint or stop.

The existing path follower has speed-scaled lookahead (8 cm at 0.12 m/s), so it
rounds these anchors. A local ideal-response run of the actual PathTracker at
0.12 m/s produced extra right offsets about 0.93/1.43/1.94/3.24 cm at forward
18.0/24.3/26.7/30.1 cm. A 0.08 m/s comparison produced 0.91/1.32/1.58/2.59 cm.
This checks planning and tracker interaction, not drivetrain dynamics or measured
clearance. No follower/speed/feedback/4 cm protection code is changed. The
candidate remains validated:false and must start from freshly initialized P01.