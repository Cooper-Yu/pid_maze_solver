# Task6 / Task8 功能验收 — 2026-10-26

## 版本与验收范围

用户明确确认正向和逆向成功，并要求最新版本标记为 `task6`、`task8`。
本次真机运行代码提交为 `649f6ad8116108f1349bfa2d4a4b9cecfbc45b2d`，
分支 `codex/task8-reverse`。云端源码 tracked clean；实际安装的官方入口、
Task8 runner、自动录制模块以及正/反向 YAML 均与该源码 SHA256 一致。
发布提交只补充本文档，运行代码与此测试提交完全相同。

这是官方命令的单轮功能验收，不等同于 Notebook Grading 得分、
独立学习能力 PASS、Task6 同版本连续三次成功，亦不新增全程零擦碰断言。

## 官方命令与证据

环境：The Construct rosject 1017013，ROS 2 Humble，Python3.10；
PID 包实际安装于 `/home/user/task8_build_cd60193/install/pid_maze_solver`。
后端使用独立 `distance_controller` 的 `codex/settle-final-check` 修复版本
`202b5a5`（微调预算届满先停车复核，不放宽最终门）。
加载 ROS、PID 与该后端 overlay 后执行：

```bash
ros2 run pid_maze_solver pid_maze_solver 2
# 正向成功闭包并出现 P15_REFERENCE_REGISTERED_NO_MOTION 后，在 P15 退出遥控：
ros2 run pid_maze_solver pid_maze_solver 4
```

| 项目 | 核对结果 |
| --- | --- |
| Task6 正向 | 21 动作完成；终态 P15_after_turn；exit_code=0 |
| 正向日志跨度 | 360.03988456726074 s |
| 正向录包 | closed_metadata=true，exception=null |
| P15 参考登记 | 官方正向入口自动登记成功 |
| Task8 逆向 | 21 动作完成；终态 P01_after_turn；full_route_success=true；exit_code=0 |
| 逆向日志跨度 | 341.9422960281372 s |
| 收尾 | /cmd_vel Publisher count=0、Subscription count=1；无 owned runner/recorder 残留 |

正向审计：
`/home/user/.local/share/pid_maze_solver/runs/forward_7q_nhe0u/route.jsonl`

正向闭合录包：
`/home/user/.local/share/pid_maze_solver/runs/forward_7q_nhe0u/evidence/20261026_110739_31628/bag/bag_0.db3`

逆向审计：
`/home/user/ros2_ws/src/pid_maze_solver/task8_route_1793013230215572172.jsonl`

上述云端路径为本场证据位置，不随 Git 一起传输，不应在当前 P15 登记仍引用时删除。
本次逆向独立 rosbag 未提供；已核对其完整动作审计与终态，未冒认逆向录包闭合。

## 本版最终几何和保护

- P13 保留原总右移3cm以及后来增加的前移3cm，P12到P13渐近；
  P13名义点 `(0.3158019547712686, -1.0315868730786444)`。
- P15 在到达时车头方向、末半圈之前逐步右移3cm；
  新点 `(0.02899960878708014, -0.5496575672149936)`。
- 逆向坐标随新P15原点仅作 y-=.03 的重表达，保留独立调好的实际航点；
  本地审计核对189个原有采样点；首边补点保持5cm网格。
- 正向全程无显式 `--output` 时自动管理录制与登记；失败/取消不覆盖旧参考。
  外层已管理录制时须显式 `--output`，不可重复启动录制器。
- 速度/反馈失鲜/停车/4cm转向中心漂移等既有保护不变。

## 本地检查与历史边界

此前针对实际测试提交的116项 Python 检查、Ruff、Humble构建通过；
自动录制模块另有两项CTest、localhost-only/domain187真实ROS录制和闭包验证，
期间 /cmd_vel 消息0、owned测试进程退出。

上午 P13 漂移及 P01 初始化失败记录保留，不用本轮成功反推其唯一根因。
曾在 P01 运行 scene4 的错误已确认起点不符；另有 P15 墙面匹配失败，
本轮成功不等于所有放置偏差或环境条件均已覆盖。

## 标签策略

- 按用户最新明确要求，将 `task6` 更新到本次发布提交，新增同提交的 `task8`。
- 原 `task6` 的 peeled commit 为 `cb3021f28a4f331a0c65808fd4b55583fcaa474f`；
  原 annotated tag object 为 `7f0265fb209e1403c76d1c39909b2510c4079aab`。
- `task7` 保持其历史仿真验收提交 `08847817377d886c2482c64c2fdbd8c7bfbb26bb`，不移动。
- 不删除开发分支，不自动合并 `master`。保留独立试验记录及既有未跟踪草稿。