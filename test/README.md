# Tests

Build and run `colcon test --packages-select pid_maze_solver`, then inspect
`colcon test-result --verbose`. Six GTests cover supplied lateral headings,
angle wrapping model footprint regressions and complete, finite, origin-preserving XY overrides.

With the ROS environment sourced, use an unused ROS domain (no robot):
`ROS_DOMAIN_ID=198 ROS_LOCALHOST_ONLY=1 python3 test/runtime_guards.py`.
This fixture publishes synthetic odom/scan and launches the installed executable;
it verifies final zero commands and failure status for stale feedback,
obstacle blocking and invalid orientation. It is not a hardware test.

See `docs/verification.md` for actual maze evidence and remaining acceptance.
