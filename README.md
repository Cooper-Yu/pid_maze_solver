# PID Maze Solver — Task5

A simulation-first holonomic ROSBot XL controller. The default route is a
sequence of relative turns and forward/left distances in `motion.hpp`, configurable
by named ROS parameters. The first stopped odometry pose anchors P01, and all
nominal waypoint poses are generated from that origin. Placement must match the
maze start; startup does not reposition the robot automatically.

Each destination uses TURN -> MOVE -> stopped hold. Strafes keep the planned
heading. Reliable paired side walls can correct a forward segment's cross-track
target within 6 cm; openings do not trigger blind recentering. See
[relative actions and wall correction](docs/motion_steps.md).

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select pid_maze_solver
source install/setup.bash
ros2 run pid_maze_solver pid_maze_solver
```

Run in the maze simulator, with /odometry/filtered, /scan, /clock and TF from the
scan frame to base_link. use_sim_time defaults true. No hardware execution was
validated. Stop other velocity publishers before starting this controller.
For a bounded first-leg trial use `--ros-args -p last_point:=2`.

## Control and laser behavior

The planar PID uses default P/I/D = 1.5/0/0 with speed and acceleration limits.
Angular PID uses 1.8/.03/.35 with measured-rate damping, based on turn_controller.
Both controllers expose all three gains. Positive body y is left, positive yaw
is counterclockwise; heading errors use the shortest signed angle.

Laser returns are transformed with TF. The known Task2 self-filter box
x=[-.195,.165], y=[-.145,.145] m excludes chassis returns. Near-wall points add
bounded body-frame repulsion during translation, tapered over the final 0.10 m
above the arrival tolerance to avoid cancelling target attraction. Odom, not ray length alone,
defines waypoint arrival (15 mm position, .01 rad heading, stopped for .4 s).
The laser guard predicts chassis clearance over .48 s using both commanded and
measured velocity. It checks the union of the model-derived body box (half extents .170/.135 m)
and wheel envelope (.135/.160 m), plus .015 m clearance, stops on an obstruction, and faults after a 5 s blocked hold. This
is local sampled protection, not a proof of collision-free motion or a planner.
Unknown blind regions and differing robot geometry require separate validation.

Both sensor receipt watchdogs use steady time (.5 s). PID and hold timing use
ROS time; duplicate time skips integration without resetting output. Startup
is bounded to 15 s; stages to 60 s. Missing/invalid feedback, clock discontinuity,
blocked motion or a stage timeout stop the robot and exit with failure.

## Reading order

- `include/pid_maze_solver/motion.hpp`: relative actions, waypoint generation and wall fitting.
- `include/pid_maze_solver/route.hpp`: compatibility poses and footprint geometry.
- `include/pid_maze_solver/pid.hpp`: reused PID math (from Task4 a9beeac).
- `src/pid_maze_solver.cpp`: PIDMazeSolver, interfaces, state transitions and guards.
- `test/test_route.cpp`: heading preservation and footprint regressions.
- `docs/verification.md`: actual evidence and remaining limits.

Task1–4 repositories/tags remain unchanged. Task5 acceptance and tagging require
cloud maze verification; generated documentation is not an official score.

## Turning clearance

A blocked TURN may first perform a low-speed (0.02 m/s) translation away from the
nearest wall, bounded to 0.06 m from its TURN entry pose. This is a distinct
clearance adjustment, not a new route origin or a change to supplied headings.
It waits for angular velocity to drop and checks commanded/measured swept
footprints. If already inside the soft clearance margin, only non-worsening
clearance is allowed and an absolute 5 mm guard remains. Failure to find space
stops the task. This local rule can fail in concave spaces; it is not a planner.

The geometry derives from local body_colision.stl bounds x=[-.166736,.161000],
y=[-.134700,.134700], plus wheel center x=+/-.085, y=+/-.135,
radius .05 and thickness .05. Revalidate these assumptions for another model.

## Laser-measured route trial

The original 15-point table remains available in `fixed_points` mode. An optional `waypoint_xy`
parameter supplies exactly 15 XY pairs in the P01 route frame; all yaw values
remain the original values. Invalid length, nonfinite values or a moved P01
origin are rejected before starting. `config/measured_route.yaml` contains a
separate calibration trial, preserving `docs/reference_points.csv`.

```bash
ros2 run pid_maze_solver pid_maze_solver --ros-args --params-file \
  "$(ros2 pkg prefix pid_maze_solver)/share/pid_maze_solver/config/measured_route.yaml"
```

Read [motion sequence](docs/motion_plan.md) and [laser survey](docs/laser_survey.md)
for measured distances, tested coordinates and the acceptance boundary.
