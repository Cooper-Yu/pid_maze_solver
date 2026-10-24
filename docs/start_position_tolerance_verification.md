# 起点位置容差独立验证

2026-10-25，WSL Ubuntu-22.04。仅测试/文档由验证者写入；未改production，未stage/commit/push。

命令：`python3 -m pytest -q test/test_start_position_tolerance.py test/test_task6_ops.py test/test_scene_entry.py`，最终 **57 passed in0.86s**（新增17项，既有40项）。新增文件ruff format/check均通过。

验证默认.005；.010持久config.json，经Operation.build_command→maze_entry.parse_args scene2→backend_command传同名CLI值0.01；非法NaN/inf/界外拒绝；scene1/3显式新flag拒绝，未给它们新增参数或ROS覆盖；scene3既有unchecked ROS overrides拒绝仍保持。旧manifest缺项只在内存归一.005，不回写原文件；manifest bool/string/null/nonfinite/界外拒绝。

链路测试使用真实生产命令构造和解析函数，provenance/包定位使用本地隔离fixture，不启动命令、不代表真实机器人执行。实际后端C++构建与隔离运行见distance_controller/docs/start_position_tolerance_verification.md。

已实际执行WSL `source /opt/ros/humble/setup.bash && cd /home/cooper/ros2_ws && colcon build --packages-select distance_controller pid_maze_solver --event-handlers console_direct+`，2包成功，6.49s。最终maze_entry.py与安装entry pid_maze_solver摘要一致；task6_ops_runtime.py与安装副本摘要一致，无需为这两文件重建。

源快照SHA256：task6_ops.py a173c8fa14df48e08de778ebc28662cb560bf7e4b57952bc2f07d66f5f91f5be；task6_ops_runtime.py 53c3d070046cb9a7918e9fac89304936be28feef4a6dd11cecc05b7ae79055ba；maze_entry.py e7cb5c4ee1926c97adf4b2aa142506f3b2e4d240d679dbcd9a5e82818d0e8f37。

唯一测试初次失败来自验证者将命令argv错误切成[2:]，已修为[1:]，未要求production改动。课程PASS/Task6验收不变，未云端或机器人运行。
