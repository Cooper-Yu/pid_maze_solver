# Task7 local functional validation — 2026-10-22

Environment: Ubuntu-22.04 WSL, ROS2 Humble, Gazebo Fortress, localhost-only
ROS domain 194. This is local functional evidence, not official cloud grading or
learner-independent implementation acceptance. No real robot was connected.

The existing local maze adapter ran scene 1 to completion including its clockwise
half-turn, then scene 3 in the same simulator with no reset or teleport. The
controller executable/YAML SHA256 manifest and logs are retained in the Checkpoint18
WSL training notes under `code_lab/runtime_logs/task7_reverse_20261022_165937/`.
See [machine-readable result](task7_local_validation.json).

- Scene 1: all fourteen destinations, final half-turn and exit 0; zero wall contacts.
- Scene 3: P14 through P01, exit 0, 83.78 s; zero wall contacts from 86,921 live
  contact packets (719,530 floor-contact points), not missing-data inference.
- Scene 3 maximum stopped error against its active target: 12.780 mm / 0.000791 rad.
- Return to original scene-1 origin: filtered-odom position difference 29.005 mm;
  heading differs from the expected opposite original heading by 0.009452 rad.
  Target error and whole-trip origin difference are distinct measurements; wall
  centering may adjust the active target. These are not absolute physical metrology.
- Both phases ended with command (0,0,0) and zero remaining velocity publishers.
- Local engineering round-trip limits were 40 mm / 0.03 rad; no claim of an
  official course tolerance. Course cloud calibration/grading remains pending.

Checks: Humble colcon build; CTest route/entry/reverse tests (16 GTest, 19 entry
unittest, 7 independent geometry unittest); original fourteen-segment config
comparison; Ruff, clang-format, yamllint and LLVM 23.1.0 naming/function-size review.
LLVM needs explicit C++17 when reading this GCC compile database. No PID or safety
threshold was changed. Existing Task5/6 scene policies and historical tags remain.

The first harness run completed scene 1 but its reporter rejected ros2 echo's
trailing `---`; it cleaned up before scene 3. The reporter now accepts exactly one
nonempty YAML document, rejects multiple real documents, and was regression-tested
against the captured output. The subsequent whole-trip run passed.

Local replay: source the normal WSL ROS/simulation overlays, then run the retained
`code_lab/task7_reverse_verify.bash` with a unique output directory and an outer
650-second timeout. The script is local-environment support, not a cloud launcher.
The cloud user command after deployment and correct P15 placement is:

```bash
ros2 run pid_maze_solver pid_maze_solver 3
```

Do not use scene 3 on hardware or as recovery after an odometry reset. Scene 4 /
Task8 and Task6's three consecutive hardware runs are separate unfinished gates.
