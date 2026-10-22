# 单次当前坐标系人工监视后退

`tools/task6_local_step_ops.py` 是源码工具入口。它仅执行一次当前车身方向后退，
不恢复旧 P01、不变换历史坐标、不接续失败路线，也不自动发送下一步。
操作者须在每次调用前检查当前俯视画面、后方路径和线缆，并持续监视、可随时停止。

- 必须显式传 `--supervised-manual`。激光是观测模式，**没有自动避障**；缺射线仍是未知，
  不能由本工具宣称后方无障碍。严格覆盖诊断保留在后端，不以它认证此人工遥控模式。
- 正数 `--distance` 表示后退米数，范围 `(0, .05]`；`--speed` 范围 `.01–.03 m/s`，
  默认 `.01`。后端运动含停车收敛最多 10 秒；包装器整体默认 60 秒，允许 30–120 秒。
- 调用需要全新 `--run-dir` 及含时区的预约 `--deadline`，至少留 120 秒；保留最后
  90 秒用于停车和证据闭合。旧目录不可重启，也无自动重试。
- 先核对源码/安装 SHA 与 Git 状态、独占 operations 锁、控制锁空闲、双 odom 停稳及
  GID、扫描和 TF 新鲜，再启动既有 recorder/watch。录制健康和必需话题增长通过后才启动一步。
- 运行中继续检查双 odom 身份、反馈新鲜、发布者数量、录制健康、取消及截止时间；
  后端负责每帧实际唯一控制权与有界运动。任何失败结束本次调用。
- 收尾复用原 Operation：停止自有子进程、核对末零命令、无速度发布者、双 odom 停稳、
  recorder metadata 闭合。不得以未观察到零命令冒充已验证停车。

## 使用条件与命令

仅在审阅、Git 部署并安装对应 `distance_controller/current_frame_step` 后可用。
包装器从经 Git 核对的 pid 源码运行；无需为此新增安装入口。以下是命令模板，
时间和目录必须按当前预约更新；不代表已经执行或现场准入。

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
python3 ~/ros2_ws/src/pid_maze_solver/tools/task6_local_step_ops.py \
  --supervised-manual --distance 0.03 --speed 0.01 \
  --run-dir ~/task6_local_steps/step_01 \
  --deadline 2026-10-22T14:00:00+08:00 --max-seconds 60
```

Ctrl+C 取消；不在另一个终端启动 teleop 或其他速度发布者。下一步必须由操作者重新确认
画面后显式使用新目录调用，工具不会链式执行。

## 证据与验证边界

`manifest.json` 保存实际源码/安装散列、包版本、预检、双 GID 和明确人工监视模式。
`motion_command.json` 保存确切参数；后端 `local_points.json` 和 `.jsonl` 只使用
LOCAL_START/LOCAL_END。成功须 exit 0，加唯一 `local_step_started` → `local_step_completed`
（最后事件），匹配距离、速度、双 GID、人工监视标志；拒绝 route_completed 或 failed 混入。
`result.json` 保持 `task6_acceptance=false`、`historical_p01=false`、
`route_recovery=false`、`automatic_avoidance=false`；`failure_snapshot.json` 记录失败现场。
录制和监测目录沿用既有工具，运行期间不读 SQLite。

本片段完成 13 个纯 mock 用例（参数、显式监督、SHA改变、双 GID 变化、录制失败、
取消窗口、唯一启动、后端异常、精确成功事件、共享清理及无旧路线接口）及 Ruff。
这不替代后端隔离 ROS 或真机安全验收，未运行真机，未认证能够从 P03 返回 P01。