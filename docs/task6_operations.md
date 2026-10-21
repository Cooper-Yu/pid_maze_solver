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
