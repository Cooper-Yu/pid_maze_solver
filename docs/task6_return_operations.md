# Same-epoch explicit return operations

This source-run wrapper adds evidence and command plumbing around the existing
`distance_controller/route_return_runner`. It does not change return planning,
turn drift limits, endpoint acceptance, controller gains or source claims.
Use it only after a successful `task6_run` outward session whose closed bag and
stopped result are present. It is observation-only, not Task6 avoidance evidence.

The script is deliberately source-run, so deploying this wrapper does not
replace an already-running installed controller. After reviewed Git deployment:

```bash
R=~/ros2_ws/src/pid_maze_solver/tools/task6_return_ops.py
python3 "$R" plan --source-run-dir ~/task6_runs/full01
python3 "$R" start --source-run-dir ~/task6_runs/full01 --return-dir ~/task6_runs/full01_return --deadline "$DEADLINE" --max-seconds 900 </dev/null >~/task6_return_start.log 2>&1 &
python3 "$R" status --return-dir ~/task6_runs/full01_return
```

`DEADLINE` is the actual current booking cutoff, with explicit timezone. The
wrapper begins cleanup 90 seconds earlier. `start` never sends `next`; it checks
fresh stopped feedback, original filtered/base publisher identities, no velocity
publisher, control lock availability, independent recording health, and the
return runner's own source/pose/claim checks. It then waits. If the source GID
changed, it refuses instead of adopting or rebasing the new odometry epoch.

Only after inspecting `status` and the next action, submit exactly one action:

```bash
python3 "$R" next --return-dir ~/task6_runs/full01_return
python3 "$R" status --return-dir ~/task6_runs/full01_return
```

The status contains readiness, busy state, cursor, latest completed action and
the last request's acceptance. `QUEUED` is not action completion. Wait for the
completed action and a fresh ready status before the next explicit `next`.
A mode-0600 FIFO carries atomic requests. Duplicate IDs, requests older than
five seconds, and `next` while busy are rejected. No complete-route or automatic
advance command exists.

To close or cancel:

```bash
python3 "$R" quit --return-dir ~/task6_runs/full01_return
```

During an action this asks the original runner to cancel. SIGINT/TERM to the
wrapper or reaching its duration/cutoff also cancels the owned runner. Cleanup
uses the existing operations sequence: zero/publisher release/original-epoch
stopped feedback, then recorder/watch shutdown and metadata verification.
A partial return is reported as incomplete with nonzero status. After any first
motion, the source runner's durable claim forbids restarting that source; a
new wrapper directory is not a claim bypass.

`plan` reads and validates source evidence without creating a ROS node. It
requires an outward success result, verified stop, closed bag metadata, complete
matching source JSON/JSONL, consistent recorded GIDs, and byte-identical installed
`distance_controller`/`turn_controller` artifacts compared with the outward
manifest. The new wrapper's own revision/hash and support hashes are recorded
separately. The wrapper does not use a previous P01 as a new current origin.

Evidence appears under the new return directory: `manifest.json`, `control.json`,
`motion.log`, `return_points.json/jsonl`, independent bag/monitor output and
`result.json`. Only the core's `return_completed` plus successful cleanup can
produce success. Intermediate quit is not completion. Full physical return and
collision-free operation remain separate real-robot checks.

Local verification uses Ubuntu-22.04/Humble only. Unit tests cover no automatic
motion, one-next admission, busy/duplicate/stale rejection, FIFO permissions,
partial audit writes, failed/unclosed outward evidence, and changed-GID refusal
before creating a return directory. The integration fixture uses the actual
installed formal/return controllers, actual recorder/watch and synthetic sensors;
it constructs the outward operations envelope from the verified formal output
and real closed bag, then checks waiting, one explicit action, waiting again,
quit, durable source claim, zero and closed return bag. It does not certify a
physical route or real camera viewpoint.

Verified 2026-10-22: six unit cases and Ruff/diff checks passed.
`/tmp/task6_return_ops_ros/1792621295065404939/` contains the real-controller
one-next fixture: no motion before request, exactly one completed reverse
translation, no automatic following action, then explicit quit. It correctly
returned exit2 for incomplete return, retained the durable source claim,
verified final zero/publisher release and closed the independent return bag.
No cloud or physical robot was used for these tests.
