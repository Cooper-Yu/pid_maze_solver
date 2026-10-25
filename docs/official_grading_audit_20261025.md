# 2026-10-25 官方 Notebook 逐项审核

本记录针对用户要求的 Task1/3/5/7；不自授分数，不把本地回归等同官方环境验收。
来源：本地课程 Notebook 导出 Checkpoint 18 - Robot Control (ROSBot XL).md 的各任务 Grading Guide。

## 官方入口与当前环境差异

| Task | 世界启动 | 控制器命令 | Notebook checkout |
| --- | --- | --- | --- |
| 1 | ros2 launch rosbot_xl_gazebo empty_simulation.launch.py | ros2 run distance_controller distance_controller | task1 |
| 3 | ros2 launch rosbot_xl_gazebo simulation.launch.py | ros2 run turn_controller turn_controller | task3 |
| 5 | ros2 launch rosbot_xl_gazebo simulation.launch.py | ros2 run pid_maze_solver pid_maze_solver | task5 |
| 7 | ros2 launch rosbot_xl_gazebo simulation.launch.py | ros2 run pid_maze_solver pid_maze_solver 3 | task7 |

当前 WSL 已安装的 rosbot_xl_gazebo 来自另一版本：没有 empty_simulation.launch.py，simulation.launch.py 默认世界/驱动也不同于课程。课程启动文件快照存在 checkpoint18_assets/cloud_snapshot，但不将自制包装器改名冒充官方启动。已执行的本地 empty/maze 适配仅为诊断回归。当前源码也不能冒称是在旧任务标签上执行。

浏览器按用户要求已经释放，未自动恢复。完整官方命令、目标修复 revision、标签 checkout 与 Notebook 审核的联合复验尚待课程环境可用；此门通过前不发布“全部官方验收通过”的修复标签。

## Task1 / Task3 每项审核

| 条目 | 分值/任务 | 当前证据与结论 |
| --- | --- | --- |
| Kp/Ki/Kd 明确定义 | .375 | 源码已核对；Task1 1.5/0/0，Task3 1.8/.03/.35。零增益不等于缺少参数，是否符合课程预期不自行评分 |
| 高速且一致的速度控制 | .75 | CSV/速度边界可审核；本地观测不代替官方环境动画行为比较 |
| 每点停止几秒 | .5 | 实际默认 Task1 dwell1s、Task3 dwell.5s，与文字“a few seconds”有差异，列为待核对，不直接PASS |
| 最终完全停止 | .25 | 本地末 cmd 和测量速度记录已留存；官方环境待验 |
| 精确到达，无过冲/不足 | .375 | 需逐点轨迹及末误差，不能只看最终点或exit0 |
| 程序结束 | .25 | 本地两项exit0；官方环境待验 |

## Task5 / Task7 每项审核

| 条目 | 分值/任务 | 审核证据与门 |
| --- | --- | --- |
| 距离与角度三增益定义 | .5 | 源码参数与PID类；待最终revision固定后记录 |
| 当前位置/朝向计算误差 | 1 | odom回调、坐标变换、误差计算源码及运行日志 |
| 遍历点并按PID输出速度 | 1 | 14段顺序、PID调用、cmd时间序列 |
| turn/move/repeat模式 | 1 | TURN/MOVE状态日志；同向段复用停稳状态须说明 |
| 运动中修正方向 | .5 | heading误差、angular命令和轨迹，不能只靠代码存在 |
| 终点避墙 | .5 | 完整激光保护、contact有效包且墙接触0；原Task7半圈被保护中断，修复待回归 |
| 最终停止并退出 | .5 | 全部14段、约定末半圈、末零指令、pub0及exit0联合检查 |

终点新增几何让位：保留全部已接受扫描点和15mm硬保护，搜索20mm圆内位置，要求平移路径>=15mm及整半圈>=25mm；运行中相对原停点30mm界限，最终回原停点。采样模型不是真实无碰撞证明；不得从8项单测通过推论全程通过。

## 当前验收界限

- Task6固定标签与主工作区未修改；Task8全程真机未验。
- 不以本次修复自动改写学习者PASS或历史任务标签。
- 保留初次Task7失败日志；完成修复后先审查完整回归，再等待官方命令环境复验，官方验收标签仅在该门通过后创建。本地修复可使用明确带local的独立标签，不能冒称官方验收。