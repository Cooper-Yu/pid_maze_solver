# Task6 只读传感器基线采集

在机器人已经停止时运行；脚本不发布速度、不改参数、不连接或重启硬件。它不认证当前位置、无擦碰或可开始运动。

最短云端命令（输出目录必须全新且父目录已存在）：

```bash
bash ~/ros2_ws/src/pid_maze_solver/tools/task6_sensor_baseline.bash "$HOME/task6_baseline_$(date +%Y%m%d_%H%M%S)"
```

默认采集12秒，可在目录后追加10至15的整数。默认source `/opt/ros/humble/setup.bash`及`~/ros2_ws/install/setup.bash`，均早于`set -u`。非默认工作区可设置`TASK6_WORKSPACE`，overlay可用既有`TASK6_OVERLAY`选择。

输出仅路径及`COMPLETE`/`INCOMPLETE`。详细内容在新目录：

- `nodes.txt`、原始/过滤激光、双odom、cmd_vel的`topic_*.txt`。
- 从`/scan`和`/scan_filtered`实际publisher名称/namespace发现driver及filter，保存对应`params_*.yaml`；从节点列表发现`robot_state_publisher`名称。缺失、超时、失败、未知名称或多个匹配明确记入结果，不推断采集成功。自定义且不含该名称的robot-state节点会显示缺失，需后续明确处理。
- `tf_base_laser.txt`为有界`tf2_echo base_link laser`；参数和TF读取在录制之前，录制本身仍包含TF及静态TF。
- 通过同工作区`distance_controller/tools/task6_ops/run.bash record`复用现有单实例录制器，10个topic不变、压缩图2Hz、30MiB上限、原300MiB磁盘floor不变。
- 录制正常退出后仅检查result JSON、metadata文件及各topic计数；不查询活动SQLite。raw/filtered/双odom/TF/static任一零消息、非duration结束或参数/TF不完整均报`INCOMPLETE`、exit2，并保留证据。`cmd_vel`无消息在静止采集时允许，但不等于已经证明停止。
- `commands.tsv`保留逐项exit；`result.json`保留缺失原因、发现节点及原录制结果。总120秒加5秒强制截止，每条读取有独立截止；INT/TERM清理仅自身进程组，写`INTERRUPTED`，不会停止其他录制器或硬件节点。

本地仅纯mock验证，不创建ROS节点或调用真实录制器：

```bash
bash -n tools/task6_sensor_baseline.bash
python3 -m unittest discover -s test -p test_task6_sensor_baseline.py -v
ruff check test/test_task6_sensor_baseline.py
ruff format --check test/test_task6_sensor_baseline.py
```

Ubuntu-22.04：4项mock通过，覆盖完整采集/既有目录拒绝、缺参数仍保留录制、缺原始扫描不冒充完成、INT后自有录制进程退出。`bash -n`、Ruff及diff空白检查通过；本环境未发现shellcheck，因此未声称通过shellcheck。真实云端行为待验，脚本从已审阅源码checkout运行，无新增安装入口。