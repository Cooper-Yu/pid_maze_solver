# Task7 末半圈本地修复证据

## 触发与范围

首次本地Task5→Task7同一世界，逆向14段完成后P01末半圈在剩约0.857rad时被15mm保护拦住，exit2。odom XY变化不足1mm，不能归因为控制平移漂移；不宣称已证明真实根因。原日志保留于WSL CP18 code_lab/runtime_logs/sim_suite_20261025_0800/task57。

## 实现

仅修改仿真C++后端收尾，不修改Task6主工作区或既有标签。对已接受激光点完整采样带符号半圈；若净空不足25mm，搜索20mm圆内最短5mm网格位置，平移每2.5mm采样保留15mm硬门。无解或无效扫描安全退出。让位/复位速度不超过.03m/s，目标容差3mm；收尾实际odom相对原停点超过30mm则停车。半圈后必须返回原停点再完成。实时safe()及反馈保护保持。

## 实际验证

- Ubuntu22.04/Humble隔离colcon构建成功；9个CTest入口全通过；colcon test-result汇总34 tests/0 failures。Python入口另含各自unittest数量，不将该汇总冒称独立用例总数。
- 新增8个GTest覆盖无须移动、单墙让位、窄通道无解、中间平移障碍、已有硬门违反、20mm界限、旋转方向和非法输入。
- 同一Gazebo本地迷宫Task5→Task7：各14段、各末半圈、两exit0、两cmdpub0、墙接触0，接触包及地面接触均有效增长；往返filtered odom位置差26.365mm、朝向差-.005403rad。该轮无须让位，不能用于证明新增状态分支。
- 独立合成ROS闭环实际执行安装的C++二进制，正前方.230m直墙、零距离测试路线：触发PLACEMENT→FINAL_TURN→RESTORE，exit0；最大位移12.119mm，复位误差2.856mm，signed yaw3.138989rad，模型最小净距25.037mm，末cmd全零。合成plant不是Gazebo/实机证明。
- Task1本地10段exit0，最大停点误差8.561mm，末cmd零；Task3本地4转exit0，最大停角误差.008193rad，平移cmd零。代码包未在此次修复改变。
- 所有证据根：WSL CP18 code_lab/runtime_logs/terminal_placement_20261025_0812、terminal_runtime_20261025_0820；Task1/3原始路径见本地regression_summary.json。

## 明确未验

本地世界使用既有适配启动器，控制器命令采用课程入口。完整官方启动命令、旧task标签checkout、Grading逐项环境验收尚待；见official_grading_audit_20261025.md。不标官方PASS/得分，不改学习者状态。本地修复标签名称必须带local。Task8全程真机及Task6同版连续3次不因本次回归而完成。浏览器保持释放，未部署云端或控制真机。
风格审查：合成ROS测试main保留90语句的局部PLR0915例外，原因是让有界fixture、子进程及唯一finally清理在同一生命周期中可审计；不推广到生产实现。clang-format、Ruff其余检查及git diff --check均执行。
