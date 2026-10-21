# P01 停车采样离线重建

本工具是只读诊断，不创建 ROS node、不发布速度、不修改控制器或准入标准。
复用来源为 `distance_controller` 的 `db60999`：`initial_centering.cpp`
窗口投影与上中位数/MAD、`right_wall_heading.cpp` 主墙 TLS、
`robust_wall.hpp` 主墙点选择、`start_verification.hpp` 15 帧统计。
既有 `scan_compare.py` 做配准，过滤和统计不同，因此不用于此重建。

在 source ROS Humble 的终端直接运行，无需构建或安装新入口：

```bash
python3 ~/ros2_ws/src/pid_maze_solver/tools/p01_sampling_replay.py \
  --run-dir ~/task6_runs/full03 --output ~/full03_p01_sampling.json
```

复制到别的位置的离线包可使用 `--bag CLOSED_BAG_DIR --log MOTION_LOG` 代替
`--run-dir`。前者检查 metadata/SQLite；后者还检查录制结果 `closed_metadata`。
工具不读取 live graph，不启动录制。数据库用 SQLite `mode=ro`。

固定统计标准：15 帧；至少 12 帧有限且共同通过 2 cm / 2 deg 离群筛选；
保留帧的 right/rear 全距不超过 1 cm，angle 全距不超过 2 deg。
稳定窗口仍需 right 0.26 m / rear 0.28 m 各 ±5 mm、angle ±0.5 deg。
输出 JSON 明列重建使用的参数。若实际运行覆盖这些参数，不得把此报告当一致性证据。

JSON 内容：

- `cycles[].frames`：每帧 receipt/header、right/rear median/MAD、主墙角度/RMS/span、
  关联的前一条 odom、近似入选条件及被排除原因。
- `cycles[].odom`：窗口内所有 filtered odom，包含每帧之间的观测；
  `odom_stop_summary` 汇总 3 mm/s、0.01 rad/s 阈值越界。
- `nonoverlap` / `sliding`：对同一近似可用序列做相同的 15 帧统计，前者步长 15、后者步长 1。
  `within_target` 仅表示离线窗口满足上述统计/目标条件，不表示真实初始化或路线通过。
- `logged_batches[].closest_contiguous_window`：在日志批次末尾附近寻找统计最接近的连续
  15 帧，保存误差、首末索引。匹配只支持时序诊断，不唯一证明实际被控制器接受的序列。
- `signals`：逐帧 min/max/median，以及所有稳定滑动窗口中值范围、偏离目标的数量，
  用于区分波动过大与稳定但持续偏移。

必须保留的边界：录包 receipt 不是控制器 callback receipt；控制器 timer 可能跳过扫描，
其两次 scan 之间的 odom/hold/reset 时序不能由工具精确认证。近似停止保持以 scan receipt
采样，0.5 秒保持期间不入选；完整 odom 仍保存供交叉检查。无效距离窗口/过期扫描会分开
离线比较段，不宣称复现完整 scan-recovery 状态机。只支持静态 laser→base_link 链；
缺失或变动的静态 TF 会失败，不猜测变换。没有放宽阈值或建议采纳滑动窗口控制策略。

本地 Ubuntu-22.04/Humble 验证：

```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_ws/src/pid_maze_solver
python3 -m unittest discover -s test -p test_p01_sampling_replay.py -v
ruff check tools/p01_sampling_replay.py test/test_p01_sampling_replay.py
ruff format --check tools/p01_sampling_replay.py test/test_p01_sampling_replay.py
```

6 项测试涵盖已知矩形/真实安装旋转、上中位数、15 帧离群/稳定边界、滑窗与非重叠差异、
日志 right 与 spread_right 分离、C++ 原 header 逐值对照、真实 ROS 消息序列化的只读
closed SQLite + TF + 51 帧重建和 CLI JSON。测试不创建 ROS node，不构成真机验收。
