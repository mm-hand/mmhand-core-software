# MMHand 当前文档索引

新增交接入口（2026-09-05）：[项目接手总览](交接/00_项目接手总览.md)、[逐文件索引覆盖说明](交接/07_索引覆盖说明.md)、[现场交接清单](交接/08_现场交接与验证清单.md)。下方原索引含历史路径与参数，发生冲突时先按当前源码和新交接总览核对。

最后核对：2026-08-01

本目录同时保留当前设计、专项说明和历史故障复盘。总览文档用于建立现行架构，专项文档保留实现细节和操作步骤，历史文档保留当时的推理与测量方法。旧文档即使部分结论过期，也应在文首增加“当前状态对照”并修正关键冲突，不应整篇删除。

## 阅读顺序

1. [上手代码必读](上手代码必读.md)：仓库入口、事实源、启动前检查和阅读路径。
2. [系统状态机与RTOS任务](系统状态机与RTOS任务.md)：状态、事件、owner、任务优先级、队列和急停链路。
3. [控制算法与标定](控制算法与标定.md)：单电机/对抗双电机控制、碰撞保护、自动标定和 RopeInit。
4. [通信协议与数据校验](通信协议与数据校验.md)：上位机串口、S3-P4 CAN、舵机总线、CRC/奇偶校验和时延估算。
5. [持久化与掉电恢复](持久化与掉电恢复.md)：桌面 JSON、Manual 标定、多圈坐标、舵机单独掉电和 Reset Zero。
6. [硬件映射与连线](硬件映射与连线.md)：21 关节/27 电机映射、总线、引脚、SPI 模式和手指顺序。
7. [ROS2与遥操链路](ROS2与遥操链路.md)：外部发布者、ROS2 topic、URDF 契约、输出授权与 Replay。
8. [实机验证与故障排查](实机验证与故障排查.md)：ENC 灯、磁编无读数、RopeInit 黄灯、通信与标定排查顺序。

## P4 架构专题

- [P4 固件架构重构说明](../ESP32-P4/ServoBoardMain/docs/P4_固件架构重构说明_20260731.md)：模块边界、依赖方向和 STM32 迁移路线。
- [P0 机器人模型单一数据源与产品指纹](../ESP32-P4/ServoBoardMain/docs/P0_机器人模型单一数据源与产品指纹_20260731.md)：URDF、映射、生成物与产品指纹。
- [P1 统一舵机安全决策与协议 Schema](../ESP32-P4/ServoBoardMain/docs/P1_统一舵机安全决策与协议Schema_20260731.md)：执行前安全判定与协议单一事实源。

## 补充教程

- [UART/I2C/SPI/CAN 通信协议入门教程](UART_I2C_SPI_CAN通信协议入门教程.md)：通用原理教程，不是本项目参数的事实源。

## 当前专项说明

- 控制与状态：[当前单/双电机 PID](pid_control_mode_current.md)、[当前系统与碰撞检测状态机](当前系统与碰撞检测状态机.md)、[状态机优化说明](状态机优化说明.md)、[对抗双电机标定与 RopeInit](对抗双电机标定与RopeInit说明.md)。
- 标定与恢复：[CalibLoad 标定快照恢复](CalibLoad标定快照恢复说明.md)、[多圈与标定 JSON 恢复](多圈与标定_JSON_恢复.md)、[舵机单独掉电恢复](舵机单独掉电_多圈坐标与动态行程恢复说明.md)。
- 硬件与传感：[当前硬件映射](current_mapping_readme.md)、[SPI 模式与磁编/触觉](SPI模式与磁编触觉连线说明.md)、[舵机/磁编/触觉连线总结](舵机磁编触觉连线总结.md)、[触觉与磁编改进详解](触觉与磁编改进详解.md)、[电路接口清单](电路.md)。
- 通信：[详细通信说明](通信.md)、[通信帧与面试指南](通信帧详解与面试指南.md)、[校验算法入门](校验算法入门与代码说明.md)。
- 舵机实时反馈：[飞特舵机四 BUS 10 ms 并行同步读改造](飞特舵机四BUS_10ms并行同步读改造说明.md)：SDK 两阶段读、四 UART 并行接收、时序预算与 Task Runtime 实测。
- 外部控制：[Wuji URDF 与 ROS2](wuji_URDF_适配与ROS2链路说明.md)、[ROS2 输入与 Replay](ROS2与遥操链路.md)。
- 工具与流程：[Arduino IDE 快速开始](start.md)、[算法开发路线](1.0算法.md)、[V1/V2 兼容审计](V1_V2兼容与禁用逻辑清理审计_20260718.md)。

## 历史故障与审计

以下文档保留故障现场、推理过程和测试方案，文首“当前状态对照”优先于正文中的旧行号和旧待办：

- [三模块 BUG 审计 20260709](bug_audit_20260709.md)
- [历史 BUG 审计报告](bug_audit_report.md)
- [危机 BUG 清除计划 20260706](critical_bug_clearance_plan_20260706.md)
- [P4 上位通信状态丢失复盘](P4上位通信状态丢失问题修复说明.md)
- [ControlTask 抢占标定队列复盘](标定卡顿_ControlTask队列抢占根因说明_20260712.md)

## 事实源优先级

发生冲突时按以下顺序处理：

1. 当前源码、生成器及 JSON manifest/schema；
2. 本索引列出的当前文档；
3. 日志与实机测量结果；
4. 专项文档文首的当前状态对照；
5. 历史正文与 Git 历史。

关键单一事实源：

| 内容 | 权威位置 |
| --- | --- |
| 任务优先级/栈 | `ESP32-P4/ServoBoardMain/src/config/TaskConfig.h` |
| 安全阈值 | `ESP32-P4/ServoBoardMain/src/config/SafetyConfig.h` |
| 控制参数 | `ESP32-P4/ServoBoardMain/src/config/ControlConfig.h` |
| 标定参数 | `ESP32-P4/ServoBoardMain/src/config/CalibrationConfig.h` |
| 电机/关节映射 | `ESP32-P4/ServoBoardMain/src/hardware/HardwareMap.cpp` |
| 上位机协议 | `desktop/resources/config/upper_protocol_schema.json` |
| 机器人模型 | `desktop/resources/config/robot_model_manifest.json` 及其引用文件 |
| Wuji URDF 契约 | `wuji_hand_ws/src/wuji_mmhand_teleop/wuji_mmhand_teleop/assets/hku_hand_v2_urdf/urdf_contract.json` |

带日期的 P0/P1/P4 文件名是阶段代号，不代表内容停止更新；文首的“最后核对”日期才表示其与源码同步时间。

## 删除规则

只有满足以下全部条件才删除文档：内容与另一篇高度重复、没有独立实验/故障/使用价值、有效内容已经逐段迁移、索引和引用已经更新。仅有一处参数或接口过期时，直接修改该段并保留文档。
