# Completed stopped prefix return（实验性、手动分段）

`tools/task6_prefix_return_ops.py` 是独立 source wrapper，仅用于经过闭包验证、停在已完成动作边界的失败去程。它不把原 `result.success=false` 改为成功，不放宽完整返程或 `--auto-next` 的门槛，不进行初始化/rebase，不倒放速度。当前固定平移上限 0.03m/s、转速上限 0.10rad/s，`observation_only`，不作为 Task6 激光避墙验收。

## 使用

先 source Humble 与当前 workspace overlay。`plan` 只验证原闭包及列出反向动作，不启动 ROS 节点或运动：

```bash
python3 ~/ros2_ws/src/pid_maze_solver/tools/task6_prefix_return_ops.py plan \
  --source-run-dir ~/task6_runs/am02_20261022
```

明确确认原会话两个 odom publisher、当前位置和姿态仍一致后，在专用终端启动。deadline 必须替换成真实预约内的 UTC 时间，并留出至少 90 秒停车/关闭证据预算。输出目录必须不存在。

```bash
python3 ~/ros2_ws/src/pid_maze_solver/tools/task6_prefix_return_ops.py start \
  --source-run-dir ~/task6_runs/am02_20261022 \
  --return-dir ~/task6_runs/am02_prefix_return \
  --deadline 2026-10-22T05:55:00Z --max-seconds 600
```

上面的日期时间只是格式示例，不能照抄成现场截止时间。启动仅附着并等待；没有 `--auto-next`。另一个终端先 `status` 检查 ready/cursor，再发一次 `next`，观察该动作完成与停车，再决定下一次。`next` 请求被队列接收不等于运动成功。

```bash
python3 ~/ros2_ws/src/pid_maze_solver/tools/task6_prefix_return_ops.py status --return-dir ~/task6_runs/am02_prefix_return
python3 ~/ros2_ws/src/pid_maze_solver/tools/task6_prefix_return_ops.py next --return-dir ~/task6_runs/am02_prefix_return
python3 ~/ros2_ws/src/pid_maze_solver/tools/task6_prefix_return_ops.py quit --return-dir ~/task6_runs/am02_prefix_return
```

## 强制边界

- 独立 distance `route_prefix_return_plan.load_prefix_run` 审计原文件/闭包/完成动作与停止边界；原完整 `load_run` 仍拒绝失败去程。
- 原控制安装产物 SHA 必须与去程一致；新 prefix runner/validator 必须安装并记录 SHA。原源只读，不复制伪造成功会话；claim 沿用原 points 文件，已 claim 不可重启。
- 新附着前和运行期间检查双 odom GID、反馈新鲜度、无外部速度发布者；后端检查原点/当前边界姿态与首实际命令。新 epoch、部分动作或漂移不被此入口恢复。
- 使用原 ReturnOperation 的独占锁、有限时长录制/监测、FIFO 去重/busy/取消、停止核验与证据关闭；原 500ms 源/接收新鲜度和 4cm 转向漂移保护保持。
- 一旦故障立即停止并关闭，不自动重试，不重新 claim；早退不标完整 P01 返回。未知覆盖仍未知，不能用这个入口认证路径无障碍。

## 本地验证范围

新增纯测试验证失败源只能进入独立 validator、原成功门槛不变、安装 SHA/claim/模块来源/双 GID/hash 拒绝、固定低速/手动单 next/取消，以及 deadline/禁止 auto。实际 source wrapper 到新后端的隔离 ROS 联合验证和真机结果须由当次验证记录另行确认，不能由这些纯测试推出。