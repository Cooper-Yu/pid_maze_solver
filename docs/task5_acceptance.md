# Task5 functional acceptance

Accepted by the user on 2026-10-10; no collision confirmed by the user.
The task5 tag freezes the tested control code from 3bec06e plus this acceptance documentation.

Command: `ros2 run pid_maze_solver pid_maze_solver`

- P02 through P15 each report arrival; final clockwise half-turn completes.
- Turn stop rate: 0.05 rad/s; heading tolerance: 0.01 rad; motion-step hold: 0.20 ROS seconds.
- Final rotation: -3.132904 rad; final error: -0.008689 rad (about -0.50 degrees).
- Source logs overlap and are preserved separately. Summary deduplicates exact log lines.
- The last cloud log line is truncated; the user subsequently supplied [shell exit_code=0](evidence/task5_acceptance/exit_code.txt), confirming successful process exit.
- This is user-approved functional acceptance, not official course scoring or independent learner implementation acceptance.
- Filtered angular-rate decay still adds waiting. Its upstream cause is unverified; the user accepts the current duration.

See [summary](evidence/task5_acceptance/summary.json), [start](evidence/task5_acceptance/cloud_start.log), and [finish](evidence/task5_acceptance/cloud_finish.log).
Local build, 15 GoogleTests, diagnostic fixture and maze evidence remain in [trial diagnostics](turn_diagnostics.md).
No controller tuning or Task1-4 tag changes are part of this closeout.

Exit-code evidence was added after tagging. The frozen task5 tag remains at 75df8bb; this documentation follow-up does not change controller code.
