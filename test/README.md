# Tests

Build and run `colcon test --packages-select pid_maze_solver`, then inspect
`colcon test-result --verbose`. Fourteen GoogleTests cover supplied headings, angle wrapping, footprint and cruise-preview
geometry, finite origin-preserving route overrides, destination-preserving offsets,
segment speed validation and immediate PID speed-cap reductions.

With the ROS environment sourced, use an unused ROS domain (no robot):
`ROS_DOMAIN_ID=198 ROS_LOCALHOST_ONLY=1 python3 test/runtime_guards.py`.
This fixture publishes synthetic odom/scan and launches the installed executable;
it verifies final zero commands and failure status for stale feedback,
obstacle blocking and invalid orientation. It is not a hardware test.

See `docs/verification.md` for actual maze evidence and remaining acceptance.

Run `python3 test/check_motion_config.py` from the package root (g++ and PyYAML
required) to compare all 14 compiled MotionStep defaults with the shipped YAML.
This prevents the bare command and parameter-file route from drifting apart.
