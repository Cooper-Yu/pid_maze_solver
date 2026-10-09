# Tooling

Use colcon and GoogleTest for compilation and geometry regressions, clang-format
for C++, Doxygen for interfaces, ROS topic/log and CSV capture for runtime evidence.
The local verification runner has isolated DDS/Gazebo domains and bounded cleanup.
Canvas points to real files; Mermaid describes the state machine.
GDB/rqt_graph, rosbag2/PlotJuggler and coverage remain deferred until a diagnostic
requires them. No UI Testing, launch_testing or coverage result is claimed.
