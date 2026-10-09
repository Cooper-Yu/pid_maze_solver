# Route from relative motion and measured side walls

The default `motion_steps` mode starts from the first accepted stopped pose.
For each named action, it adds the relative turn to the previous planned heading,
then rotates the requested forward/left displacement into the route frame:

```
yaw_next = yaw_previous + turn
x_next = x_previous + cos(yaw_next)*forward - sin(yaw_next)*left
y_next = y_previous + sin(yaw_next)*forward + cos(yaw_next)*left
```

Every nominal target is generated once from the same origin. Small stopping
errors do not accumulate into all subsequent points. The reference table remains
available in `fixed_points` compatibility mode but is not used to generate the
normal route. Positive turn is left, positive left_m is left translation;
negative left_m is right translation. The 30 mm lateral terms at tight approaches
are explicit motions from the measured route, not hidden target coordinates.

## Configuration

`config/motion_route.yaml` lists named actions and distances. For example:

```yaml
steps:
  P01_P02:
    turn_deg: 0.0
    forward_m: 0.35
    left_m: 0.0
  P02_P03:
    turn_deg: -45.0
    forward_m: 0.21
    left_m: 0.0
```

Change one action with a ROS parameter, e.g. `-p steps.P01_P02.forward_m:=0.40`.
Later generated nominal points move accordingly. Default actions are in motion.hpp;
adding a new named action there also makes its three parameters available.

## Side-wall correction

Forward-dominant moves can adjust their cross-track destination toward a corridor
center. Both walls need at least 12 side returns, 0.12 m observed span, residual
RMS <= 8 mm, tangent angle <= 10 degrees, mutual angle difference < 5 degrees,
and total width 0.45..0.65 m. Three consecutive fresh scans are required.
A missing wall, wider opening, corner, or unsupported fit disables new correction;
the last accepted track offset is retained for that segment. Pure strafes use
odom heading/position control and the independent laser clearance guard instead.

The accepted target shift is bounded to +/-0.06 m from the generated nominal
cross-track coordinate and changes by at most 0.004 m per fresh scan. Future
nominal targets remain tied to the common origin. `side_centering:=false` disables
this correction. These corridor/geometry limits belong to the tested maze, not
all robots or environments.

The controller still needs odometry: wall range constrains relative clearance,
not the complete 2D pose or progress through an opening. Current arrival remains
odom-to-generated-target plus stopped qualification; it does not declare arrival
just because one front ray is short. Front and side laser points independently
limit unsafe commands throughout movement and turning.

## Compatibility

Use `route_mode:=fixed_points` for the original reference table, or load
`config/measured_route.yaml` for the previous XY-calibrated version. Supplying
waypoint_xy in motion_steps mode is rejected, avoiding two competing route sources.


P05_P06 now advances 0.50 m, leaving 30 mm more front-wall clearance than the first motion-step trial. P07_P08 advances 0.55 m to restore the later nominal route coordinates. The original 0.53/0.52 m trial completed but produced 852 P06 obstacle-hold log entries; its training logs are retained.

## Local verification — 2026-10-09

Default command completed 14 legs and exited 0. Maximum stopped odometry error: 0.013385 m / 0.000977 rad. The contact observer received 118466 packets with 981929 floor points and zero above-floor robot contact points. This supports this local run, not untested cloud geometry. Side centering produced 16 logged updates; 66 transient obstacle holds remain a performance limitation. Nine unit tests and three runtime guard fixtures passed. Cloud acceptance is pending; no Task5 tag.

[Run summary](evidence/motion_steps/summary.json) · [Controller log](evidence/motion_steps/controller.log)
