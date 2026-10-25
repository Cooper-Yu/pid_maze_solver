# P15显式观测策略：front_heading_fixed_reference_side

2026-10-26真机P15当前墙面几何中，独立front/right TLS推算朝向相差约3deg，旧two_wall_tls拒绝，未进入路线。旧策略保持默认；本策略需scene4显式--p15-observation front_heading_fixed_reference_side。

新模型只把跨度>=30cm、RMS<=8mm的front当朝向来源。source right原TLS法向固定在源几何中；当前right按front测得相对旋转转换这个法向，使用已有side_distance在相同±20deg窗口投影测距（60%覆盖、80%一致、12mm RMS、8cm跨度保持）。不强制右墙与前墙垂直。source right独立TLS须质量合格；current right独立TLS仅留作诊断（不可用时保留原因，不代替受约束距离80%门），但不是第二个朝向传感器；不能把构造后的两角一致当作两份独立朝向证据。

source/current右墙投影patch变换后须75%重叠，沿墙中心差<=5cm；此为局部对应闸，不能证明任意重复场景的全局定位。仅用于用户已视觉确认的P15附近，不是全局重定位。

相对配准5cm/5deg、最终每帧2cm/1deg、12独立扫描/1秒、双停稳批次、当前epoch及TF安装一致、唯一速度源、失鲜停车/恢复和4cm转向保护保持。实际精度仍是观测估计，不能由合成测试宣称真值保证。通过初始化不等于Task8路线通过。

原默认两TLS策略和其独立1.5deg一致门未删除；新模型明确为单朝向源，避免错误描述“保留了独立双墙朝向一致性”。完整Notebook评分及正式Task8标签仍待。
