# ServoBoardMain 固件说明

本文档描述 `ESP32-P4/ServoBoardMain` 当前固件架构。当前版本已经按任务层级重构：上位机通信、状态机、实时控制、CAN 通信、舵机通信彼此解耦，通过 `TaskSharedData_t` 中的队列、快照和状态字段交互。

旧的 `AngleSolver`、`ServoBusManager` 公共模块已经移除：

- 控制求解逻辑并入 `ControlTask` 内部模块。
- FTServo/SMS_STS 舵机总线访问只允许出现在 `ServoCommunicationTask.cpp`。
- 如果 IDE 仍显示 `AngleSolver.*` 或 `ServoBusManager.*` 标签页，那只是旧文件缓存，应关闭标签页并以当前文件树为准。

## 目录定位

`ServoBoardMain.ino` 只负责 Arduino 入口：

- `setup()` 调用 `System_Init()`。
- `loop()` 不做业务逻辑，只周期性 `delay()`。

真正的业务初始化、任务创建和共享资源分配都在 `src/system/SystemTask.cpp` 中完成。源码按 Arduino 兼容的 `src/` 目录分层组织，sketch 根目录只保留 `ServoBoardMain.ino` 和本文档。

## 分层架构

| 层级 | 任务/模块 | 责任 |
| --- | --- | --- |
| 上层通信层 | `upperCommunicationTask` | 解析上位机串口协议，投递系统事件，写入控制命令快照，回传传感器/舵机/故障遥测 |
| 系统管理层 | `stateMachineTask` | 统一处理 START/STOP/RESET/CALIBRATE，维护系统状态、故障位、控制使能和舵机目标所有权 |
| 实时控制层 | `controlTask` | 读取命令快照和反馈，执行关节 PID、direct motor 输出、安全保护，生成舵机目标 batch |
| 底层 CAN 通信层 | `canCommunicationTask` | 接收编码器、触觉和错误码 CAN 帧，写入共享队列 |
| 底层舵机通信层 | `servoCommunicationTask` | 独占 UART 舵机总线，执行同步写、同步读、多圈缓存和舵机遥测发布 |
| 校准服务模块 | `CalibrationTask` | 提供标定参数、单关节标定算法和校准目标构建；生命周期由状态机驱动 |

## 当前文件职责

| 文件 | 说明 |
| --- | --- |
| `ServoBoardMain.ino` | Arduino 入口 |
| `src/shared/TaskSharedData.h` | 跨层共享类型、常量、队列句柄、状态字段 |
| `src/system/SystemTask.*` | 创建队列、互斥锁、任务和系统初始状态 |
| `src/system/StateMachineTask.*` | 状态机和故障管理 |
| `src/communication/upper/*` | 上位机串口协议、命令路由和遥测发送 |
| `src/communication/can/CanCommTask.*` | TWAI/CAN 收发、编码器/触觉/错误帧解析 |
| `src/communication/servo/*` | 舵机通信任务和飞特协议常量，唯一访问 FTServo/SMS_STS 和舵机 UART 的位置 |
| `src/control/*` | 实时控制任务、PID 求解、安全保护、输出 batch 构建和 C PID 实现 |
| `src/calibration/*` | 校准服务模块、手工标定注入、单关节标定算法和校准 profile |
| `src/hardware/HardwareMap.*` | 关节到舵机、舵机索引到总线 ID 的硬件映射 |

## 启动流程

1. `ServoBoardMain.ino::setup()` 调用 `System_Init()`。
2. `src/system/SystemTask.cpp` 初始化 `TaskSharedData_t`、硬件映射、PID/校准 profile、队列和互斥锁。
3. 创建任务：
   - `upperCommunicationTask`
   - `stateMachineTask`
   - `canCommunicationTask`
   - `servoCommunicationTask`
   - `controlTask`
4. 系统初始状态为 `SYSTEM_STATE_IDLE`，舵机目标所有权为 `SERVO_TARGET_OWNER_NONE`。

## 共享数据边界

跨任务通信集中在 `TaskSharedData_t`：

- `stateEventQueue`：上位机命令转为系统事件，由状态机消费。
- `servoTargetQueue`：控制层或校准层生成舵机目标 batch，由舵机通信层消费。
- `servoFeedbackQueue` / `servoTelemetryQueue` / `servoTelemetrySnapshotQueue`：舵机通信层发布反馈。
- `canRxQueue` / `tactileQueue`：CAN 层发布编码器、触觉和错误状态。
- `mappedAngleQueue` / `jointDebugQueue`：控制层发布内部映射角度和调试信息；上位机 sensor 包直接来自 CAN 原始编码器 raw。
- `commandStateMutex`：保护控制命令相关共享字段。
- `targetAnglesMutex`：保留兼容旧目标访问，新的控制命令优先走 `commandStateMutex`。

控制任务每个周期只读取一次 `ControlCommandSnapshot_t`，避免读到上位机连续下发时的半更新数据。

## 状态机与舵机目标所有权

状态机是系统状态和控制使能的唯一权威。

| 事件 | 状态变化 | 舵机目标 owner | 控制输出 |
| --- | --- | --- | --- |
| START | 进入 `SYSTEM_STATE_RUNNING` | `SERVO_TARGET_OWNER_CONTROL` | 允许 |
| STOP | 进入 `SYSTEM_STATE_STOPPED` | `SERVO_TARGET_OWNER_NONE` | 停止 |
| RESET | 回到 `SYSTEM_STATE_IDLE` | `SERVO_TARGET_OWNER_NONE` | 停止并清空目标/故障 |
| CALIBRATE | 进入 `SYSTEM_STATE_CALIBRATION_IDLE` | `SERVO_TARGET_OWNER_CALIBRATION` | 普通控制暂停 |
| CALIBRATION_DONE | 进入 `SYSTEM_STATE_CALIBRATION_SUCCESS` | `SERVO_TARGET_OWNER_CALIBRATION` | 普通控制暂停 |
| CALIBRATION_FAILED | 进入 `SYSTEM_STATE_CALIBRATION_FAILED` | `SERVO_TARGET_OWNER_CALIBRATION` | 普通控制暂停 |

`ServoTargetBatch_t` 带有 `source` 字段：

- `SERVO_TARGET_OWNER_CONTROL`：来自 `ControlTask`。
- `SERVO_TARGET_OWNER_CALIBRATION`：来自 `CalibrationTask`。

`ServoCommunicationTask` 只执行与当前 `servo_target_owner` 匹配的 batch。状态切换时会 `xQueueReset(servoTargetQueue)`，避免旧目标在模式切换后继续执行。

## 控制模式

### Joint 模式

上位机下发 `CMD_ANGLE_CTRL` 后进入 `CONTROL_MODE_JOINT`。

数据流：

1. 上位机发送 21 个 float 目标角。
2. `UpperCommCommandRouter` 在 `commandStateMutex` 下更新 `targetAngles` 和 `joint_command_token`。
3. `ControlTask` 周期性读取命令快照、CAN 编码器反馈和舵机反馈。
4. `ControlSolver` 执行 PID 求解。
5. `ControlOutputBuilder` 生成舵机目标 batch。
6. `ServoCommunicationTask` 执行同步写。

### Direct Motor 模式

Direct motor 模式用于上位机滑条、绝对位置或扫动命令直接控制 22 个舵机目标。

相关命令：

- `CMD_MOTOR_POS`
- `CMD_MOTOR_POS_SWEEP`
- `CMD_MOTOR_POS_ABS`

进入 direct motor 模式后，`ControlTask` 不走关节 PID，而是直接根据 motor raw 目标生成 batch。为了支持没有完整 CAN 编码器或部分舵机未在线时调试滑条，direct motor 模式下 `CAN_OFFLINE` 和 `SERVO_OFFLINE` 仍会上报故障，但不会单独把系统强制拉入 `FAULT_HOLD`。过载、释放保护等硬安全仍会阻断输出。

## 安全保护

当前安全逻辑集中在 `ControlSafety` 和 `StateMachineTask`：

- `SYSTEM_FAULT_CAN_OFFLINE`：CAN 编码器反馈超时。
- `SYSTEM_FAULT_SERVO_OFFLINE`：舵机反馈超时或部分舵机未上报。
- `SYSTEM_FAULT_JOINT16_DUAL`：joint16 双舵机同步保护。
- `SYSTEM_FAULT_OVERLOAD`：负载异常保护。
- `SYSTEM_FAULT_RELEASE_GUARD`：反绕释放保护或肌腱保护触发。

`FAULT_HOLD` 下普通 PID 不继续生成新的运动目标，只保持当前位置或进入安全保持逻辑。

## 上位机串口协议

帧格式：

```text
[0xFE][LEN][TYPE/CMD][PAYLOAD...][0xFF]
```

其中 `LEN` 表示 `TYPE/CMD + PAYLOAD` 的长度。

### 上位机命令

| 命令 | ID | 说明 |
| --- | --- | --- |
| `CMD_CALIBRATE` | `0xCA` | 进入校准状态 |
| `CMD_ANGLE_CTRL` | `0xCB` | 下发 21 个关节角目标 |
| `CMD_START` | `0xCC` | 进入运行状态 |
| `CMD_STOP` | `0xCD` | 停止控制输出 |
| `CMD_RESET` | `0xCE` | 清空状态并回到 IDLE |
| `CMD_CALIB_DATA` | `0xCF` | 写入校准零点数据 |
| `CMD_MOTOR_POS` | `0xD0` | 直接下发 22 个舵机 raw 目标 |
| `CMD_SENSOR_STREAM_MODE` | `0xD1` | 切换传感器流格式 |
| `CMD_MOTOR_POS_SWEEP` | `0xD2` | 下发滑条 sweep 目标 |
| `CMD_MOTOR_POS_ABS` | `0xD3` | 下发绝对舵机位置 |
| `CMD_TENDON_GUARD` | `0xD4` | 配置肌腱保护 |

### 固件上报

| 包类型 | ID | 说明 |
| --- | --- | --- |
| `PKT_SENSOR` | `0x01` | 21 路原始磁编码器 raw 值 |
| `PKT_CALIB_ACK` | `0x02` | 校准 ACK |
| `PKT_SERVO_ANGLE` | `0x03` | 舵机角度 |
| `PKT_JOINT1_DEBUG` | `0x04` | joint debug |
| `PKT_SERVO_TELEM` | `0x05` | 舵机遥测 |
| `PKT_PROTO_ACK` | `0x06` | 协议 ACK |
| `PKT_FAULT_STATUS` | `0x07` | 故障状态 |
| `PKT_RELEASE_FAULT` | `0x08` | 释放保护故障 |
| `PKT_SERVO_RAW` | `0x09` | 舵机 raw 位置 |
| `PKT_TACTILE` | `0x0A` | 触觉数据 |

## CAN 协议概览

CAN 使用 ESP32 TWAI，默认引脚：

- TX：GPIO 47
- RX：GPIO 48

主要帧：

- 编码器帧：`0x100` 起，每帧 4 个通道。
- 错误详情帧：`0x1F0` 到 `0x1F2`，用于重组错误码表。
- 触觉汇总帧：`0x240` 到 `0x246`，用于重组触觉组数据。

编码器总数为 `ENCODER_TOTAL_NUM = 21`。触觉数据为 5 组，每组 3 个传感器，每个传感器 3 轴。

## 舵机通信

舵机总线由 `ServoCommunicationTask.cpp` 内部私有 `ServoBusDriver` 管理。其他文件不能直接访问 `SMS_STS`、`HardwareSerial` 或同步读写 API。

默认总线配置：

| Bus | Serial | RX | TX | Baud |
| --- | --- | --- | --- | --- |
| 0 | `Serial1` | GPIO 21 | GPIO 20 | 1 Mbps |
| 1 | `Serial2` | GPIO 23 | GPIO 22 | 1 Mbps |
| 2 | `Serial3` | GPIO 27 | GPIO 26 | 1 Mbps |
| 3 | `Serial4` | GPIO 33 | GPIO 32 | 1 Mbps |

舵机通信层职责：

- 接收 `servoTargetQueue` 中 owner 匹配的目标 batch。
- 执行同步写位置、速度、加速度。
- 分阶段同步读舵机位置、负载等反馈。
- 维护多圈位置缓存。
- 发布 `servoFeedbackQueue`、`servoAngleQueue`、`servoRawQueue`、`servoTelemetryQueue` 和 `servoTelemetrySnapshotQueue`。

## 硬件映射

硬件映射集中在 `HardwareMap.cpp`。

关节数量：

- `JOINT_COUNT = 21`
- `SERVO_TOTAL_NUM = 22`

其中 joint16 使用双舵机：

- 主舵机：bus 3, id 17
- 副舵机：bus 3, id 18

22 个舵机索引由 `motorMap` 映射到具体 bus/id。21 个关节由 `jointMap` 映射到主舵机。不要在控制、校准或通信任务里散落硬编码映射。

## 校准状态

校准不再是常驻 FreeRTOS 任务，而是由 `StateMachineTask` 管理生命周期。`CalibrationTask` 仅作为服务模块保留标定参数、单关节标定算法、反馈读取 helper 和校准目标构建能力。

手工标定采用原始磁编码器限位表：

- `minraw`：关节初始/机械下限位置的 CAN 原始磁编码器读数。
- `maxraw`：关节机械上限位置的 CAN 原始磁编码器读数。
- `g_encoderDirection[]`：安装方向，固件会自动把 `minraw/maxraw` 转成方向归一后的 raw，不需要手动计算 oriented raw。
- `angleScope`：由 `minraw -> maxraw` 的 14-bit 环形正向差值推导后换算为 degrees，不再作为设计活动角度手工给定。
- `bottomReserved/topReserved/angleReserved`：单位均为 degrees，实际可控范围为 `angleScope - bottomReserved - topReserved`。

当前策略：

- `CMD_CALIBRATE` 只进入校准生命周期，不自动跑完整机械标定。
- `StateMachineTask` 设置 `g_calibrationUIStatus`，由 `UpperCommTask` 一次性上报给上位机。
- 校准期间 `servo_target_owner = SERVO_TARGET_OWNER_CALIBRATION`。
- 普通控制目标不会覆盖校准目标。
- 退出校准需要通过 START、STOP 或 RESET。
- `runSingleJointCalibration()` 仍保留为内部能力，可后续接入明确的单关节校准命令。

## 开发边界

新增代码时应遵守以下边界：

- 只有 `ServoCommunicationTask.cpp` 可以 include 或调用 FTServo/SMS_STS、`HardwareSerial` 舵机总线读写、同步读写 API。
- `ControlTask` 和 `CalibrationTask` 只能通过队列、快照和共享状态与舵机通信层交互。
- `UpperCommTask` 不直接写 `control_enabled`，START/STOP/RESET/CALIBRATE 只能投递 `SystemEvent_t`。
- 控制命令字段必须在 `commandStateMutex` 保护下读写。
- 状态切换时必须清空旧 `servoTargetQueue`，避免历史目标跨状态执行。
- 新增硬件映射应修改 `HardwareMap.cpp`，不要在任务文件中复制 bus/id 表。

## 编译和验证建议

静态检查：

```text
搜索 AngleSolver / ServoBusManager，应无公共引用。
除 ServoCommunicationTask.cpp 外，不应出现 SMS_STS、HardwareSerial 舵机读写、syncRead/write、setTarget 等舵机总线调用。
UpperCommTask 不应直接写 control_enabled。
ControlTask 不应绕过 commandStateMutex 读取命令数组。
```

功能检查：

1. START 后进入 `RUNNING`，direct motor 滑条可以下发舵机目标。
2. STOP 后 owner 变为 `NONE`，旧目标不会继续执行。
3. RESET 后清空目标、tokens、故障位并回到 `IDLE`。
4. CALIBRATE 后进入 `CALIBRATION_IDLE`，普通控制不再写 `servoTargetQueue`。
5. START 可以从校准状态恢复到控制 owner。
6. joint16 双舵机异常会进入保护。
7. 上位机连续下发 joint/direct/sweep 命令时，控制任务读到的是完整快照。

## 阅读顺序

建议按以下顺序理解代码：

1. `src/shared/TaskSharedData.h`
2. `src/system/SystemTask.cpp`
3. `src/system/StateMachineTask.cpp`
4. `src/communication/upper/UpperCommProtocol.h`
5. `src/communication/upper/UpperCommTask.cpp` 和 `UpperCommCommandRouter.cpp`
6. `src/control/ControlTask.cpp`
7. `src/control/ControlSolver.cpp`、`ControlSafety.cpp`、`ControlOutputBuilder.cpp`
8. `src/communication/servo/ServoCommunicationTask.cpp`
9. `src/communication/can/CanCommTask.cpp`
10. `src/hardware/HardwareMap.cpp`
11. `src/calibration/CalibrationTask.cpp`

## 当前限制

- 自动全关节校准暂未启用，避免误触发硬件运动。
- 校准协议仍保留向后兼容空间，后续可增加单关节校准命令。
- direct motor 模式为了方便滑条调试，对 CAN/SERVO offline 故障采用“上报但不单独阻断”的策略；真实闭环运行仍建议保证全部反馈在线。
