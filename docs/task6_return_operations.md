# Same-epoch return operations

`tools/task6_return_ops.py` runs from the reviewed source checkout and launches
the installed `distance_controller/route_return_runner`. Use it after a successful
outward `task6_run` session with a verified stop and closed evidence bag. It
reverses completed route actions, not recorded velocity commands, and retains
the original P01, controller tolerances, 4 cm turn-drift limit and source claim.

**Manual one-action requests are the default. Automatic next is opt-in and full
automatic-return acceptance is pending.** Both modes use `--laser-log-only`;
neither provides Task6 laser-avoidance or physical-clearance acceptance.

## Source entry and manual use

Source ROS 2 Humble and the workspace overlay in the operating terminal. Use
the wrapper from the Git checkout, not an installed copy or a temporary fixture.
It records that checkout's revision and file hashes, then checks that installed
controller artifacts still match the outward session. Do not rebuild or replace
those controller artifacts between the outward run and its return.

Set `OUTWARD` to the completed operations directory, `RETURN` to a new directory,
and `DEADLINE` to the actual authorized booking cutoff in ISO 8601 format with an
explicit timezone. The cutoff must be at least 120 seconds away; cleanup begins
90 seconds before it. `--max-seconds` accepts 30–1680 seconds, defaults to 900,
and must fit the remaining booking and evidence capacity.

```bash
R=~/ros2_ws/src/pid_maze_solver/tools/task6_return_ops.py
OUTWARD=~/task6_runs/full01
RETURN=~/task6_runs/full01_return
# Set DEADLINE to the current booking cutoff before start.
python3 "$R" plan --source-run-dir "$OUTWARD"
python3 "$R" start --source-run-dir "$OUTWARD" --return-dir "$RETURN" \
  --deadline "$DEADLINE" --max-seconds 900 </dev/null >"$RETURN.start.log" 2>&1 &
python3 "$R" status --return-dir "$RETURN"
```

`plan` validates saved evidence without creating a ROS node. `start` creates
fresh monitoring/recording, attaches the return runner and waits; manual mode
never sends `next` automatically. Inspect the plan and a fresh ready status,
then request exactly one action:

```bash
python3 "$R" next --return-dir "$RETURN"
python3 "$R" status --return-dir "$RETURN"
```

Status includes readiness, busy state, cursor, last completed action and request
acceptance. `QUEUED` means the FIFO accepted a request, not that the action
completed. Wait for completion and a fresh ready status before another `next`.
The mode-0600 FIFO rejects duplicate IDs, requests older than five seconds and
`next` while busy. Do not reuse a return directory.

## Admission, stop and failure gates

Both modes require:

- A successful outward result, verified stop, closed bag metadata and complete,
  matching source JSON/JSONL; original session UUID/hash, P01 and recorded GIDs.
- Byte-identical installed `distance_controller`/`turn_controller` artifacts
  compared with the outward manifest, plus recorded wrapper/support hashes.
- Fresh stopped filtered/base odometry, original publisher identities, no current
  velocity publisher, available control locks and healthy independent recording.
- The core runner's original source, pose, heading, feedback and atomic-claim
  checks. Changed odometry identity is refused; this wrapper does not rebase or
  automatically calibrate a new epoch.

Close or cancel with:

```bash
python3 "$R" quit --return-dir "$RETURN"
```

An active action is cancelled through the owned runner. SIGINT/TERM, duration or
booking cutoff, failed recording and failed feedback/identity checks also end
the attempt. Cleanup verifies zero/publisher release and stopped feedback in
the original epoch, then closes recorder/watch and checks metadata. An incomplete
return reports failure. After first attempted motion, the durable source claim
forbids restarting that source; a new directory does not bypass the claim.
There is no automatic retry or recovery from a partial turn.

The new return directory contains `manifest.json`, `control.json`, `motion.log`,
`return_points.json/jsonl`, independent bag/monitor output and `result.json`.
Only the core's `return_completed` plus successful cleanup can produce success;
intermediate quit is not completion. Preserve these files for review.

## Optional automatic next

After review of the entire plan, `--auto-next` may be added to `start` for an
explicitly authorized test. It requires a successful **complete P01-to-P15**
outward session, the exact 20 ordered action names, unchanged waypoint hash and
an outward manifest equal to the selected fixed route. A prefix or failed source
is rejected. The default return speeds remain 0.03 m/s and 0.10 rad/s.

```bash
python3 "$R" start --source-run-dir "$OUTWARD" --return-dir "$RETURN" \
  --deadline "$DEADLINE" --max-seconds 1200 --auto-next
```

Only one `next` may be outstanding. The wrapper checks the matching action
start/index/name/kind, completion/decremented cursor and unchanged source hash.
Before another action it requires **zero `/cmd_vel` publishers**, fresh stopped
filtered and base odometry for at least 0.3 seconds with three additional messages
from each, and the executed action's observed final zero command. The first
boundary uses stopped attachment and the dual-odometry hold. Each boundary and
requested-action startup has an eight-second limit. Rewritten/truncated audit,
out-of-order events or failed gates terminate the attempt without retry.

External `next` is rejected in auto mode. `quit` takes priority. Cancellation
and limits are rechecked after feedback/recording checks and again at dispatch,
before marking the action pending or writing to the runner.
After the strict audit accepts all action completions and `return_completed`,
auto mode stops active participant/name checks and dispatch. Rosout may unregister
before the child process exits. A separate, at-most-two-second normal-exit wait
continues feedback pumping, booking/duration/cancel checks, fresh stopped
original-epoch dual feedback, zero command publishers, recording health,
unchanged source/audit and the observed final zero command. It still requires
child exit 0 and the usual recording/cleanup verification. Missing or premature
terminal events, late/nonzero child exit, an outside publisher or any failed
gate remain failures; the terminal audit event alone is never success.

### Publisher association

Auto mode requires the actual identifier returned by
`rclpy.utilities.get_rmw_implementation_identifier()` to be `rmw_fastrtps_cpp`;
it explicitly selects that implementation for its child. Before child startup,
any existing `supervised_return` rosout publisher is refused. After the audited
source/epoch attachment, while the child is alive and before first-next, exactly
one new rosout endpoint named `supervised_return` in `/` binds once. A rosout GID
already present before startup cannot later be adopted. Missing initial discovery
may wait within the existing boundary deadline; no action is dispatched meanwhile.

During execution there may be at most one command publisher. A known endpoint
must have the expected name/namespace and bound participant prefix. For both
ROS unknown-name sentinels, admission uses a recently confirmed exact command
GID or the frozen participant association. The bound rosout endpoint must retain
its complete GID and explicit name/namespace, with an observation no older than
0.5 seconds and a live child. Unknown command observations do not renew evidence.
Seeing no command publisher clears the exact-command cache but retains the
single child participant association for subsequent action writers.

The supported Humble Fast DDS layout is checked: 24-byte RMW GID storage with a
nonzero 12-byte participant prefix, four-byte entity ID and eight zero padding
bytes. Different prefixes, explicit aliases/namespaces, multiple command writers,
invalid GIDs, wrong RMW, duplicate/missing/changed bound rosout publishers and
stale observations are refused. There is no rebinding. **An admitted endpoint
still present always prevents the next action.** `control.json` records the
participant binding and observation time; rejected endpoint details are preserved.

This is graph/context association, not cryptographic process authentication.
Multiple ROS nodes can share a context. Its operational boundary is the reviewed
single-context child, prelaunch graph baseline, child lifetime, source-bound
attachment and existing exclusive control. UNKNOWN means the graph lacks a node
association; it does not by itself prove a competing publisher.

Implementation references: [ROS graph-cache name association](https://github.com/ros2/rmw_dds_common/blob/humble/rmw_dds_common/src/graph_cache.cpp#L490),
[Fast DDS writer GUID construction](https://github.com/eProsima/Fast-DDS/blob/v2.6.11/src/cpp/rtps/participant/RTPSParticipantImpl.cpp#L662),
[rmw_fastrtps GID byte layout](https://github.com/ros2/rmw_fastrtps/blob/6.2.10/rmw_fastrtps_shared_cpp/include/rmw_fastrtps_shared_cpp/guid_utils.hpp#L49).

## Evidence budget and verification scope

Recording uses compressed images at 2 Hz and a 180 MiB/session limit. Duration
must account for measured recording growth, path length / 0.03, reverse angles /
0.10, settling and boundary holds. Removing manual round trips does not guarantee
a fixed return time. A capacity stop aborts motion rather than permitting
unrecorded continuation. Camera viewpoint and physical clearance need separate
real-robot verification.

Current local evidence is from Ubuntu-22.04/Humble:

- Manual integration: the actual installed controllers and recorder/watch with
  synthetic sensors verified waiting, one explicit reverse translation, waiting
  again and quit. Incomplete return correctly exited 2 with claim retained,
  final zero/publisher release and closed bag. This was not a physical route.
- Current automatic admission: 71 related pure tests passed, including event and
  hash binding, participant/exact-GID checks, first unknown, stale/changed identity,
  invalid layouts, no dispatch while a publisher remains, cancellation races,
  FIFO requests, terminal/child-exit ordering and cleanup. Ruff/format/diff checks passed.
- Full automatic ROS integration passed all 20 ordered actions and normal exit
  in 209.1 seconds, with final zero, publisher release and closed recording.
  A separate quit case waited for an actual nonzero command before cancelling;
  it verified final zero, zero remaining command publishers, released control
  lock and closed recording. Both used isolated synthetic sensors/source evidence.
  No real-robot auto-return or laser-avoidance acceptance is claimed.

Run the pure suite from the source repository:

```bash
python3 -m pytest -q test/test_task6_return_terminal.py test/test_task6_return_participant.py test/test_task6_return_endpoint_binding.py test/test_task6_return_auto.py test/test_task6_return_ops.py test/test_task6_ops.py
```

The repository's integration fixtures construct synthetic sensors/source evidence;
they are test infrastructure, not deployment entry points. The isolated test-only
clock shim must never be loaded into a cloud or real-robot process. Detailed
commands, failure histories and runtime evidence belong in the authoritative
WSL `~/ros2_ws/training_notes/checkpoint18/robot_control_rosbot_xl/code_lab/verification.md`,
not in this usage guide. Record the current runtime result there before changing
these acceptance statements.
