# 固定当前基准的人工监视后退会话

`tools/task6_local_session_ops.py` 是源码包装入口。它在新 odom epoch 中只建立一次
LOCAL_START，固定初始朝向和横向参考线。纵向目标为当前实际投影减本次步长，避免为补偿
上一步纵向不足而把单步移动扩大；不承诺累计纵向距离等于步数乘步长。这样避免每步把横向
偏差重新当成新基准。它不导入旧 P01、旧路径或失败源，不进行坐标重定位。

## 人工控制边界

- `start` 只建立会话和录制；没有自动第一步。
- 操作者确认当前俯视画面、后方路径与线缆后，每次显式发送 `step`。
- 步长 `(0, .05] m`，速度 `.01–.03 m/s`，默认 `.03 m/.01 m/s`；后端每步运动预算
  保持 10 秒。步长是目标长度，不是对真实物理位移误差的保证。
- 人工激光观测模式**没有自动避障**。未知覆盖仍为未知；命令/odom/scan/TF保护不会关闭。
- READY 时无速度发布者，持续检查双 odom 身份、新鲜度、停稳及 scan。失效即结束并锁存，
  不自动等待恢复或重试。ACTIVE 时只允许当前一个速度发布者。
  唯一例外是 READY/pub0/双 odom 与 scan 均仍新鲜、固定基准停稳时，明确的
  `SCAN_ODOM_BRACKET_TOO_WIDE` 扫描配对间隔过大：最多等待 2 秒，撤销当前 token，
  `control.json.waiting_for_scan_pair=true`；不能把 TF 缺失、frame/epoch 改变或 age 超限吞入等待。
  至少三个严格递增的新 scan timestamp 已成功配对后，后端发同会话/同索引的新 READY token。
  原请求（包括已写入但尚未 started 的请求）被撤销，操作者必须重新确认并发送 step；没有自动补发。
  重复等待事件不能重置期限，ACTIVE 配对失败仍直接停车。
- 正常一步完成后，后端再次确认停稳和发布者释放，才产生新的 READY。
- `step` 请求绑定 session_id、next_index、一次性 ready_token；重复、过期、旧 token、
  BUSY 请求立即丢弃，不排队到后续 READY。没有 `--auto-next`。
- `quit` 在 READY 关闭会话；运动中 quit 或 Ctrl+C 取消当前步骤并停车，作为失败退出。
- 全新目录只用一次；退出后无 resume。重新启动必须重新人工确认环境并建立新基准，
  不得把新基准称为旧 P01 或继续旧步数。

## 命令模板

先经 Git 审阅和部署，安装匹配的 `distance_controller/current_frame_session`。
在已 source ROS 和工作区 overlay 的终端执行；预约时间和目录必须换成本轮实际值。

```bash
python3 ~/ros2_ws/src/pid_maze_solver/tools/task6_local_session_ops.py start \
  --session-dir ~/task6_local_sessions/manual_01 --supervised-manual \
  --step-distance .03 --speed .01 --max-seconds 600 \
  --deadline 2026-10-22T17:00:00+08:00
```

在另一终端检查状态，再确认画面后只发送一步：

```bash
python3 ~/ros2_ws/src/pid_maze_solver/tools/task6_local_session_ops.py status \
  --session-dir ~/task6_local_sessions/manual_01
python3 ~/ros2_ws/src/pid_maze_solver/tools/task6_local_session_ops.py step \
  --session-dir ~/task6_local_sessions/manual_01
```

`QUEUED` 仅表示请求进入 FIFO；查看 `control.json.last_request.accepted` 和
`completed_steps` 才能判断接收、完成。不要把多个 step 写入循环或批量粘贴。
将 `step` 换为 `quit` 可停止/关闭；运行终端 Ctrl+C 也会取消。不要同时运行 teleop。

## 证据与清理

复用既有 Operation 的 recorder、odom watcher、独占锁、截止检查和自有进程清理。
录制必须先证明 required topics 增长；健康失效即停车。图像仍为 2 Hz / 每次 180 MiB，
容量停止不代表运动可以继续。预约结束前保留 90 秒收尾。

- `manifest.json`：源码/安装散列、Git 状态、预检、双 GID、人工模式。
- `local_session.json/.jsonl`：后端固定基准、每步及 READY/终态；审计必须只追加，索引单调。
- `control.json`：实时 READY/BUSY、会话/token、完成步数、最后请求是否接收。
- `result.json`：会话是否正常关闭及清理，不表示抵达 P01；`task6_acceptance=false`、
  `route_recovery=false`、`historical_p01=false`、`automatic_avoidance=false`。
- 一旦执行过运动，收尾要求末零速、发布者释放、反馈停稳和录制 metadata 闭合。
  从未发布命令就关闭时，单独记录 stopped/released，明确 `zero_command_observed=false`。

本片段 24 个纯状态/协议测试通过；与已有单步包装器及 ops 合计 52 PASS。
覆盖 token/index/session/anchor、重复/过期/BUSY丢弃、取消与反馈失效阻止调度、
无自动下一步、失败终态及无命令关闭证据。Ruff/format 通过。
2026-10-22 已完成 WSL Ubuntu-22.04/Humble 隔离 ROS 包装器集成：两次显式步进后 quit 为 exit 0，运动中 quit 为 exit 2/Cancelled；两例均末零速、pub0/maxpub1、录制闭合和锁释放。正常例原测试误将累计纵向位移期待为两倍步长，已保留该断言错误并按每步 actual−distance 合同离线复核；未因此重跑或改动生产逻辑。尚未完成此会话的真机验收，合成反馈不能证明现场纠偏收敛或避障。
`step --wait` 是可选的一次性结果观察：命令只写一次 FIFO，随后最多 20 秒读取
该请求 UUID、session、index 的接受记录和 `step_started` / `step_completed`。
拒绝、WAIT 撤销、关闭、失去新鲜状态或超时均退出 2；超时表示尚未证实完成，
不能据此重发。完成输出 `STEP_COMPLETED <request-id> index=N`，退出 0。
观察本身不会发第二步，也不会解除故障。普通 `step` 保持原来的非阻塞行为。

恢复时刻的 2 秒截止由已校验 SHA 的后端在 paired(now) 与 READY 前检查负责。
包装器审计验证配对新戳/身份/token，同时对仍处于 WAIT 的运行状态检查截止；
不以稍后读取审计文件的时刻代替后端恢复时刻。
