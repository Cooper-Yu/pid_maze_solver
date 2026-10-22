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
## 已停稳边界的短暂扫描等待

此策略仅属于独立 prefix wrapper。附着审计必须匹配原会话/hash/原点/filtered GID，动作事件必须追加且按请求顺序完成。尚未确认附着、已派发 next、动作执行中、退出/取消/故障时，不提供扫描等待。正式去程与原完整返程仍使用原严格规则。

在后端等待输入、没有待执行动作的明确边界，wrapper要求两个 odom 持续新鲜、同 GID、速度已停、cmd publisher 为零。仅 scan 的正向源年龄或接收年龄超过 .5s 可进入最长2秒等待；未来时间、倒退/锁存错误、无效scan、odom失鲜/变化或出现发布者仍立即失败。原 max-seconds 和预约 deadline 不暂停、不重置。间歇扫描不能刷新这2秒期限；超时为锁存故障，不因之后恢复扫描而复活。

等待期间 control.json 的 waiting_for_fresh_scan=true/ready=false；需要3个新递增、满足原[-.1,.5]源时间及.5s接收限制的扫描才解除等待，不把重复stamp计入、不声称空间覆盖或无障碍。解除后仍不会自动 next。每次手动next在写入子进程前再次严格检查所有反馈和停稳条件；如果间隔内扫描失鲜则拒绝本次请求。下一动作派发后立即恢复原500ms运动监护。

若从未派发next、从未观察到任何cmd，且已附着的子进程退出，cleanup可额外观察至少.3s、每路至少3个新odom样本，验证原双GID/新鲜停稳/无发布者。它记录stopped_and_released=true和stop_evidence.kind=stopped_without_motion_command；zero_and_released仍为false、zero_command_observed=false，明确没有伪造零命令证据。这不等于返程完成，也不授权旧epoch继续。已经派发next或观察到cmd则仍走原零命令验证。

本地验证：新增12项纯边界测试，与prefix/原return/auto/ops合计56 PASS。覆盖运动中仍拒绝、分段完成顺序、派发前scan竞态、3新stamp/重复排除、固定2秒/锁存、取消/总时限、双GID/外来publisher、无命令停稳和保留zero=false。此等待改动尚无隔离ROS/真机运行证据。