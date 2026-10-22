# Task7 acceptance boundary

Task7 is the simulation reverse task in Checkpoint18 Part4. Task8 real-robot
reverse and unfinished Task6 repetition are separate tasks.

| Requirement | Implementation and evidence |
| --- | --- |
| Select reverse simulation without editing code | `ros2 run pid_maze_solver pid_maze_solver 3`; frontend and scene contract tests. |
| Load reversed simulation waypoints | `waypoints/reverse_waypoints_sim.yaml`, strict ordered P15_P14 through P02_P01 schema; independent SE(2) geometry tests. |
| Keep previous distance/turn PID | Shared C++ backend: distance 1.5/0/0 and turn 1.8/0.03/0.35; scene1 default behavior preserved. |
| Use measured position and heading errors | Odometry subscriptions, fixed stopped origin, body/world conversion, signed terminal yaw accumulation. |
| Turn, move, repeat through the route | Existing state machine, named reverse targets, ordered reached-point logs. |
| Correct the course while travelling | XY PID and heading feedback; existing side-wall correction retained. |
| Avoid walls near the target | Existing transformed scan/footprint clearance guard and bounded target/turn-clearance handling retained; feedback timeouts still stop. Local full same-world run measures zero wall contacts. |
| Stop and terminate normally | Zero command plus qualified stop, `Route completed; stopped.`, normal exit; runtime evidence must confirm both directions. |
| User-required final action | Scene3 appends counterclockwise +pi at stopped P01 with the same PID/guard/settling logic, restoring the original heading. Scene1 retains clockwise -pi at P15. |
| Mark progress | Create and push annotated `task7` only after the final cloud forward/reverse/CCW run passes. |

The learner explicitly delegated implementation and testing. Functional acceptance
and a Git tag do not claim an official grade or independent learner reconstruction.
See `task7_cloud_validation.md` and the local validation reports for actual results.
The schema/geometry tests cover route semantics; a successful run is evidence for
that version and environment, not a universal collision-free guarantee.