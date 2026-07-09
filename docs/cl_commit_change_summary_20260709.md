# cl 分支本次提交改动说明

对比基线：`45707ec Update CL control and docs`

生成时间：2026-07-09

## 1. ESP32-P4 固件控制与标定

- 双电机负反馈由旧的固定几何/硬编码零点模型，切换为自动标定阶段生成的 LUT 映射。
- `JointCalibrationResult` 增加双电机 LUT 字段，用于保存主/副电机在关节角度区间内的 abs 采样表。
- 自动标定双电机 `DUAL_EXCHANGE` 阶段新增采样、插值和 LUT 生成逻辑，完成后打印 `DUAL_LUT` 日志。
- 双电机负反馈目标计算改为基于 LUT 插值生成 `motorRef/motorFbModel`，再进入 PID 修正。
- 保留之前已经验证过的保护逻辑：两圈过度松绳限制、方向错误诊断、J20 不再特殊化、双电机最终副电机负载阈值 230。
- 自动标定生成 ABS limit 时仍保存 valid/min/max，但默认不再自动 lock，负反馈默认解锁。
- ABS limit max 继续使用“标定最大 counts + 500 counts”的规则。
- Rope Init 后增加 READY recheck/reapply 相关逻辑，避免刚回工作空间后 UI/下位机状态仍停在 IDLE。
- 舵机掉电再上电的多圈恢复阈值放宽到 256 counts。
- MultiTurn NVS 存储/恢复逻辑有补强，提升部分舵机掉电、重新上电后的恢复可靠性。
- 删除旧的 `ESP32-P4/libraries/ServoManager`，避免旧 `EMERGENCY_LOAD=5000`、旧跨圈逻辑和旧 FindLimitPosition 继续误导。

## 2. 上位机协议与 Calib Load

- `Calib Load Joint` 协议扩展：旧 54 字节 payload 继续兼容，新 payload 追加双电机 LUT。
- 下位机 `UpperCommTask` 支持发送和接收带 LUT 的关节标定快照。
- 上位机 `protocol.py` 支持解析和构造带 LUT 的标定数据。
- Calib snapshot 升级为 `schema_version: 2`。
- 双电机关节 `J00/J04/J08/J12/J16/J20` 在保存和加载 Calib snapshot 时必须具备有效 LUT。
- 旧 snapshot 如果缺少双电机 LUT，会拒绝加载对应双电机关节，避免进入新负反馈链路后映射缺失。
- Calib Load 下发 ABS limits 时，先校验 joint/LUT 完整性，再下发 limits 和 joint calib，避免坏快照先改写下位机状态。

## 3. 上位机 UI 行为

- Feedback ABS Limit 默认解锁，界面按钮默认显示 `LOCK`。
- 只有用户点击 `LOCK` 后，负反馈 ABS 越界监控/弹窗才会生效。
- 断开或未连接的关节行不再把无效哨兵值显示成 `current=0` 或 `raw=0`，降低 UI 闪烁和误判。
- Calibrate 前增加 side adjust 提示流程；侧摆选择集合的旧 side adjust 入口逻辑已清理。
- Rope Init 完成后不再只尝试一次 active scope reapply，而是等待下位机退出 `RUN_DIRECT` 且控制 owner 清空后再 reapply 并触发 READY recheck。
- Calib Load、New Device、Clear Fault 等关键操作事件继续走上位机状态/日志链路。

## 4. 数据保存与记录

- Save Data 从单 CSV 方案重构为多 topic、多文件结构。
- 新增 `desktop/core/data_recording/`，包含 recorder、CSV exporter、HDF5 recorder、topic schema、event recorder。
- CSV 默认按 topic 拆分，提升 Excel 查看时的可读性。
- HDF5 保存结构支持 topic group、dataset append、压缩和 meta/config/calibration snapshot。
- 新增 ROS2 bag 记录入口 `desktop/recording/rosbag_recorder.py`，后续可将同一套 topic 写入 ROS2 bag。
- teleop input/output schema 已预留，为后续遥操作复现和训练数据闭环做准备。

## 5. ROS2 / MANUS / 遥操作链路

- 保留 `/mmhand/joint_targets` 作为最终控制输入。
- 新增/整理 `/raw_ik_target` 链路，用于 MANUS/IK 外部节点输出的 21 关节角直接进入上位机 backend。
- 删除或弱化 UI 内部 MANUS route via ROS2 的混淆入口，MANUS 解析默认交给外部节点。
- 上位机协议和数据记录中保留 ROS2 输入事件记录能力。

## 6. 文档与审计

- 更新 bug audit 文档，重点转向当前有效链路：S3、P4、上位机、ROS2。
- 删除旧的 `docs/magnetic_encoder_range_analysis.md` 和旧 MANUS ROS2 文档，避免与当前 `/raw_ik_target` 设计冲突。
- 新增/保留用于问题追踪的审计文档 `docs/bug_audit_20260709.md`。
- 本次提交说明文件即本文档。

## 7. 本次不上传/继续屏蔽的内容

保持与上次提交相同的屏蔽思路，本次不上传：

- `ESP32-P4Test/`
- `ESP32-P4Test2/`
- `ESP32-P4Test3/`
- `run_data/`
- `desktop/logs/`
- Python 缓存与 pytest 缓存
- 当前仍未跟踪的 `ESP32-P4/CanWaveform/`、`ESP32-P4/I2cWaveform/`、`ESP32-P4/SpiWaveform/`、`ESP32-P4/UartLoopback/`

## 8. 注意事项

- 旧 Calib Load 文件没有双电机 LUT 时，不能安全恢复新双电机负反馈控制；需要重新自动标定或使用 schema_version 2 快照。
- 负反馈默认解锁后，不会自动弹 ABS limit 越界保护；需要用户点击 `LOCK` 才启用该保护。
- 本地 Python 文件已做语法检查；P4 C++ 固件未在本机完整编译验证，因为本机没有 PlatformIO 环境。
