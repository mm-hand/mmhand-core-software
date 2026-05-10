# ESP32-P4 ServoBoardMain 架构说明

本文档用于帮助使用者快速理解 `ESP32-P4/ServoBoardMain` 程序的功能、任务架构、数据流和通信协议。该程序运行在 ESP32-P4 舵机主控板上，是整只手的控制中心。

## 1. 系统定位

`ServoBoardMain` 负责连接三类对象：

- 上位机：通过 USB/串口通信，接收控制命令，回传传感器、舵机和调试状态。
- ESP32-S3 掌端采集板：通过 TWAI/CAN 接收 21 路磁编码器数据、编码器错误状态和触觉摘要数据。
- 飞特舵机总线：通过 4 路 UART 总线控制 22 个舵机，并读取舵机位置、速度、负载、电压、温度和在线状态。

整体上，ESP32-P4 的职责是：

- 接收 S3 的关节传感器数据。
- 接收上位机目标角度或舵机直控命令。
- 将编码器反馈和舵机反馈融合为闭环控制输入。
- 运行双环 PID 和安全保护逻辑。
- 将目标位置批量下发到多路舵机总线。
- 将实时状态回传给上位机。

## 2. 目录与模块分层

```text
ServoBoardMain/
├── ServoBoardMain.ino          # Arduino 入口，仅调用 System_Init/System_Loop
├── SystemTask.*                # 系统初始化、任务创建、全局映射表
├── TaskSharedData.h            # 跨任务共享数据结构、队列、控制状态
├── UpperCommTask.*             # 上位机串口协议解析与状态上报
├── CanCommTask.*               # TWAI/CAN 收发、S3 传感数据重组
├── AngleSolver.*               # 角度映射、双环 PID、控制模式、安全保护
├── ServoBusManager.*           # 飞特舵机总线读写、多圈位置跟踪
├── CalibrationTask.*           # 标定配置、标定结果、自动标定接口
├── JointCalibrationProfile.*   # 默认关节标定参数表
└── pid.*                       # PID 基础算法
```

### 2.1 入口层

`ServoBoardMain.ino` 很薄：

- `setup()` 调用 `System_Init()`。
- `loop()` 调用 `System_Loop()`。

主业务不放在 Arduino 主循环中，而是交给 FreeRTOS 任务并行执行。

### 2.2 系统层

`SystemTask` 负责系统级资源初始化：

- 初始化 `Serial`，波特率为 `921600`。
- 创建跨任务队列和互斥锁。
- 初始化目标角缓存、控制模式、故障位图和保护状态。
- 初始化 4 路舵机总线。
- 初始化 `AngleSolver` 和双环 PID 参数。
- 加载当前测试阶段使用的手动标定结果。
- 创建 `UpperComm`、`CanComm`、`Solver` 三个核心任务。

### 2.3 共享数据层

`TaskSharedData.h` 定义所有任务共享的数据中心 `TaskSharedData_t`，主要包含：

- 上位机命令队列和状态上报队列。
- CAN 接收/发送队列。
- 舵机角度、raw 位置、遥测数据队列。
- 映射后的关节角队列。
- 触觉摘要队列。
- 21 路目标关节角缓存。
- 22 路电机直控目标缓存。
- 控制使能、控制模式、故障位图、热插拔与保护相关状态。

关键数量：

- `ENCODER_TOTAL_NUM = 21`：关节/磁编码器通道数。
- `SERVO_TOTAL_NUM = 22`：舵机通道数，joint16 使用主副双舵机。
- `TACTILE_GROUP_NUM = 5`：触觉组数量。

## 3. 启动流程

系统启动顺序由 `System_Init()` 统一组织：

1. 初始化上位机串口 `Serial.begin(921600)`。
2. 创建 FreeRTOS 队列：
   - `cmdQueue`
   - `statusQueue`
   - `canRxQueue`
   - `canTxQueue`
   - `servoAngleQueue`
   - `servoRawQueue`
   - `servoTelemetryQueue`
   - `mappedAngleQueue`
   - `jointDebugQueue`
   - `tactileQueue`
3. 创建 `targetAnglesMutex`，保护目标关节角数组。
4. 清空目标、控制状态和故障状态。
5. 初始化 4 路舵机总线：
   - bus0：RX 21，TX 20，1 Mbps
   - bus1：RX 23，TX 22，1 Mbps
   - bus2：RX 27，TX 26，1 Mbps
   - bus3：RX 33，TX 32，1 Mbps
6. 初始化 `AngleSolver` 的零点、传动比、方向和 PID 参数。
7. 调用 `initManualCalibrationForTest()` 加载手动标定测试数据。
8. 创建三个核心任务：
   - `UpperComm`
   - `CanComm`
   - `Solver`
9. `System_Loop()` 仅执行 `vTaskDelay(10ms)`，保持轻量调度入口。

## 4. FreeRTOS 任务架构

### 4.1 UpperComm 任务

文件：`UpperCommTask.cpp`

职责：

- 解析上位机串口下行命令。
- 更新共享目标、控制模式和故障复位状态。
- 从各类上报队列读取数据并打包发给上位机。
- 周期性上报故障位图。

运行节奏：

- 主循环末尾 `vTaskDelay(5ms)`。
- 任务优先级 `TASK_UPPER_COMM_PRIORITY = 1`。

主要处理内容：

- 关节角目标。
- 电机直控目标。
- START/STOP/RESET。
- 传感器流模式切换。
- 腱绳保护配置。
- 标定 UI 状态回传。
- 舵机角度、raw 位置、遥测、触觉、调试数据上报。

### 4.2 CanComm 任务

文件：`CanCommTask.cpp`

职责：

- 初始化 TWAI/CAN，波特率 1 Mbps。
- 接收 S3 发来的编码器帧。
- 重组编码器错误详情帧。
- 重组触觉摘要帧。
- 将最新传感器快照写入 `canRxQueue`。
- 从 `canTxQueue` 取出命令并发送到 CAN 总线。

运行节奏：

- 主循环末尾 `vTaskDelay(5ms)`。
- 任务优先级 `TASK_CAN_COMM_PRIORITY = 3`。

CAN 引脚：

- TX：`TWAI_TX_PIN = 47`
- RX：`TWAI_RX_PIN = 48`

### 4.3 Solver 任务

文件：`AngleSolver.cpp`

职责：

- 同步读取 4 路舵机总线反馈。
- 维护舵机多圈绝对位置。
- 从 `canRxQueue` 获取 S3 编码器数据。
- 将原始编码器值转换为关节侧角度。
- 执行目标限幅和双环 PID 求解。
- 根据控制模式生成舵机目标。
- 执行安全保护、故障判断和保持策略。
- 批量下发舵机目标。
- 将舵机状态、映射角度和调试信息写入上报队列。

运行节奏：

- `solverPeriodTicks = 10ms`，即约 100 Hz。
- 任务优先级 `TASK_SOLVER_PRIORITY = 4`。

## 5. 数据流

### 5.1 S3 传感器到 P4 控制器

```text
ESP32-S3
  -> CAN 编码器/错误/触觉帧
  -> CanCommTask
  -> canRxQueue / tactileQueue
  -> AngleSolver / UpperCommTask
```

用途：

- `AngleSolver` 使用编码器数据做闭环反馈。
- `UpperCommTask` 将映射后的传感器数据和触觉数据回传给上位机。

### 5.2 上位机到舵机

```text
上位机
  -> Serial 协议帧
  -> UpperCommTask
  -> sharedData 目标缓存/控制状态
  -> AngleSolver
  -> ServoBusManager
  -> 飞特舵机
```

支持两类控制模式：

- 关节控制模式：上位机下发 21 路目标关节角，P4 根据编码器反馈和 PID 生成舵机目标。
- 电机直控模式：上位机直接下发 22 路电机位置目标，P4 负责限幅、热插拔保持和总线下发。

### 5.3 舵机反馈到上位机

```text
飞特舵机
  -> ServoBusManager 同步读
  -> AngleSolver
  -> servoAngleQueue / servoRawQueue / servoTelemetryQueue
  -> UpperCommTask
  -> Serial
  -> 上位机
```

上报内容包括：

- 舵机多圈绝对位置。
- 舵机单圈 raw 位置。
- 速度、负载、电压、温度。
- 在线状态。
- 关节调试数据。

## 6. 通信协议

### 6.1 上位机串口协议

串口帧格式：

```text
0xFE LEN TYPE_OR_CMD PAYLOAD 0xFF
```

说明：

- `0xFE`：帧头。
- `LEN`：从 `TYPE_OR_CMD` 到帧尾 `0xFF` 的长度。
- `TYPE_OR_CMD`：上行包类型或下行命令。
- `PAYLOAD`：负载。
- `0xFF`：帧尾。

兼容历史单字节命令：

- `'c'`：设置标定 UI 状态为空闲。
- `'b'`：清空目标关节角。

### 6.2 上位机下行命令

| 命令 | 名称 | 功能 |
| --- | --- | --- |
| `0xCA` | `CMD_CALIBRATE` | 标定命令，目前设置 UI 标定状态 |
| `0xCB` | `CMD_ANGLE_CTRL` | 下发 21 路关节目标角，float 小端 |
| `0xCC` | `CMD_START` | 使能控制，并触发故障复位 token |
| `0xCD` | `CMD_STOP` | 关闭控制输出 |
| `0xCE` | `CMD_RESET` | 清空目标，回到关节控制模式，关闭控制 |
| `0xCF` | `CMD_CALIB_DATA` | 下发 21 路标定零位 raw 缓存 |
| `0xD0` | `CMD_MOTOR_POS` | 22 路单圈电机直控目标 |
| `0xD1` | `CMD_SENSOR_STREAM_MODE` | 切换传感器上报编码模式 |
| `0xD2` | `CMD_MOTOR_POS_SWEEP` | 滑条扫动直控目标 |
| `0xD3` | `CMD_MOTOR_POS_ABS` | 22 路多圈绝对电机直控目标 |
| `0xD4` | `CMD_TENDON_GUARD` | 腱绳保护配置 |

### 6.3 P4 上行包类型

| 类型 | 名称 | 内容 |
| --- | --- | --- |
| `0x01` | `PACKET_TYPE_SENSOR` | 21 路映射后的关节传感器值 |
| `0x02` | `PACKET_TYPE_CALIB_ACK` | 标定 UI 状态 |
| `0x03` | `PACKET_TYPE_SERVO_ANGLE` | 22 路舵机多圈绝对位置和在线状态 |
| `0x04` | `PACKET_TYPE_JOINT1_DEBUG` | 指定关节的目标、反馈、PID 输出和命令位置 |
| `0x05` | `PACKET_TYPE_SERVO_TELEM` | 舵机速度、负载、电压、温度和在线状态 |
| `0x06` | `PACKET_TYPE_PROTO_ACK` | 协议命令 ACK |
| `0x07` | `PACKET_TYPE_FAULT_STATUS` | 过载故障位图 |
| `0x08` | `PACKET_TYPE_RELEASE_FAULT` | 反绕释放保护故障位图 |
| `0x09` | `PACKET_TYPE_SERVO_RAW` | 22 路舵机单圈 raw 位置和在线状态 |
| `0x0A` | `PACKET_TYPE_TACTILE` | 触觉摘要数据 |

### 6.4 CAN 输入协议

P4 主要接收 S3 发来的 CAN 数据。

| CAN ID | 内容 | 说明 |
| --- | --- | --- |
| `0x100~0x105` | 编码器角度 | 21 路编码器，每帧最多 4 个 `uint16_t` |
| `0x1F0~0x1F2` | 编码器错误详情 | 每帧 `seq + 7` 路错误码，共覆盖 21 路 |
| `0x240~0x246` | 触觉摘要 | 每帧 `seq + 7` 字节负载，重组后写入 `tactileQueue` |

错误详情重组规则：

- 3 帧序号 `seq` 必须一致。
- 超过 `CAN_ERROR_REASSEMBLY_TIMEOUT_MS = 100ms` 会丢弃本批次。
- 若 `300ms` 内没有新的完整错误批次，会清空错误表。

触觉摘要重组规则：

- 7 帧序号 `seq` 必须一致。
- 超过 `CAN_TAC_REASSEMBLY_TIMEOUT_MS = 80ms` 会丢弃本批次。

## 7. 舵机总线与映射

系统使用 4 个 `ServoBusManager` 实例：

| 总线 | 串口 | RX | TX | 波特率 |
| --- | --- | --- | --- | --- |
| bus0 | `Serial1` | 21 | 20 | 1 Mbps |
| bus1 | `Serial2` | 23 | 22 | 1 Mbps |
| bus2 | `Serial3` | 27 | 26 | 1 Mbps |
| bus3 | `Serial4` | 33 | 32 | 1 Mbps |

`jointMap` 定义 21 个关节到主舵机的映射。

`motorMap` 定义 22 个电机通道到实际总线和舵机 ID 的映射。

joint16 使用双舵机拮抗结构：

- 主舵机：`bus3-id17`
- 副舵机：`bus3-id18`
- 副舵机目标由主舵机目标反向映射得到：

```text
secondaryTarget = -primaryTarget + secondaryOffset + tensionBias
```

## 8. 控制模式

### 8.1 关节控制模式

控制模式值：`CONTROL_MODE_JOINT`

输入：

- 上位机下发的 21 路目标关节角。
- S3 通过 CAN 发来的 21 路编码器反馈。
- 舵机总线读回的 22 路电机反馈。

处理：

- 统一编码器方向。
- 使用标定结果或手动 offset 将 raw 编码器值映射到关节角。
- 对目标角做标定范围限幅。
- 外环 PID：目标关节角 vs 磁编码器实际角。
- 内环 PID：目标舵机位置 vs 舵机绝对位置。
- 输出舵机目标位置。

### 8.2 电机直控模式

控制模式值：`CONTROL_MODE_DIRECT_MOTOR`

输入：

- `CMD_MOTOR_POS`：单圈 raw 目标，会展开到离当前多圈位置最近的等效目标。
- `CMD_MOTOR_POS_SWEEP`：滑条扫动目标，根据单圈变化量连续累加。
- `CMD_MOTOR_POS_ABS`：多圈绝对位置目标，范围限制为 `-30719~30719`。

处理：

- 对 22 路电机逐通道生成目标。
- 在线热插拔后先保持当前位置，避免新上线舵机突然跳到旧目标。
- joint16 主副舵机发生双反馈故障时保持当前位置。

## 9. 安全与保护逻辑

### 9.1 控制使能

系统上电后默认不输出控制。收到 `CMD_START` 后：

- `control_enabled = 1`
- 清除 joint16 双反馈故障状态。
- 自增过载和反绕故障 reset token。

收到 `CMD_STOP` 后：

- `control_enabled = 0`
- 停止继续下发主动控制目标。

收到 `CMD_RESET` 后：

- 清空关节目标和电机直控目标。
- 回到关节控制模式。
- 关闭控制输出。
- 刷新故障复位 token。

### 9.2 CAN 和编码器异常

以下情况会触发对应关节保持当前位置：

- CAN 数据超过 `300ms` 未更新。
- `RemoteSensorData_t` 无效。
- 编码器值为 `0xFFFF`。
- 对应关节错误码非 0。
- 映射后的关节数据无效。

### 9.3 joint16 双舵机保护

joint16 使用主副双舵机反馈融合：

- 将副舵机位置投影到主舵机坐标。
- 主副差异超过阈值并持续多个周期后触发故障。
- 故障时 joint16 主副舵机均保持当前位置。

### 9.4 反绕释放保护

`AngleSolver` 维护每个关节的释放保护状态，监测释放方向上的异常：

- 释放窗口内舵机累计运动过大。
- 实际角度没有按预期回退。
- 出现持续的反向异常趋势。

触发后会设置 `reverse_release_fault_bitmap`，由 `UpperCommTask` 周期性上报给上位机。

### 9.5 腱绳保护

上位机可通过 `CMD_TENDON_GUARD` 配置每个关节：

- 是否启用保护。
- 保护方向。
- 保护阈值位置 `x1Abs`。

当目标命令是释放方向，且当前位置已到达保护边界时，对应关节会保持当前位置，不继续释放。

### 9.6 热插拔保持

当舵机从离线变为在线时，系统会标记该电机进入热插拔保持状态，优先保持当前反馈位置，避免突然执行旧目标造成跳动。

## 10. 标定体系

标定相关文件：

- `CalibrationTask.h/.cpp`
- `JointCalibrationProfile.h/.cpp`

当前系统使用测试阶段的手动标定结果：

- `initManualCalibrationForTest()` 加载默认标定参数。
- 使用 `kManualEncoderMax[]` 推导每个关节的 offset 和 angleMax。
- `encoderMax == 0` 表示该关节没有有效手动标定数据。

自动标定能力保留了接口：

- `runSingleJointCalibration()`
- `taskServoCalibration()`

但当前 `taskServoCalibration()` 是占位实现，启动后会设置状态并删除自身，不会自动驱动舵机执行完整标定流程。

## 11. 已实现功能点

- 21 路关节编码器数据接收、重组、方向统一和角度映射。
- 21 路编码器错误详情接收和错误位图维护。
- 5 组触觉摘要 CAN 数据重组和串口上报。
- 22 路飞特舵机管理。
- 4 路舵机 UART 总线同步写和同步读。
- 舵机多圈绝对位置跟踪，范围限制为 `-30719~30719`。
- 关节控制模式和电机直控模式。
- 双环 PID 控制。
- joint16 主副双舵机拮抗控制。
- CAN 离线、编码器断连和通道错误保护。
- joint16 双反馈差异故障保护。
- 反绕释放保护。
- 腱绳保护配置。
- 热插拔保持。
- 上位机串口协议收发。
- 舵机角度、raw、遥测、触觉、调试和故障状态上报。

## 12. 当前限制与待完善点

- 自动标定任务当前处于占位关闭状态，系统依赖手动标定测试数据。
- `CMD_CALIBRATE` 当前主要更新 UI 状态，并未直接启动完整自动标定流程。
- 过载保护协议字段和上报位图已保留，但当前求解任务中 `overload_fault_bitmap` 被置为 0，实际过载保护逻辑未启用。
- 触觉侧在 P4 端支持摘要重组和上报，完整触觉 dump 不在本程序中实现。
- 源码中部分中文注释存在编码乱码，阅读代码时建议以本架构文档和实际 C/C++ 逻辑为准。

## 13. 快速阅读建议

如果只想快速理解系统，建议按以下顺序阅读源码：

1. `ServoBoardMain.ino`：了解入口。
2. `SystemTask.cpp`：了解启动流程、任务创建和舵机映射。
3. `TaskSharedData.h`：了解任务之间交换的数据。
4. `CanCommTask.cpp`：了解 S3 传感数据如何进入 P4。
5. `UpperCommTask.cpp`：了解上位机协议。
6. `AngleSolver.cpp`：了解控制模式、PID 和保护逻辑。
7. `ServoBusManager.cpp`：了解舵机总线读写和多圈位置跟踪。
