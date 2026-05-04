# ESP32-S3 System 架构说明

本文档用于帮助使用者快速理解 `ESP32-S3/System` 目录在 ESP32-S3 上承担的功能、模块分层、任务调度、数据流和 CAN 通信约定。内容以当前源码实现为准，并明确标注已实现功能与仍处于占位状态的部分。

## 1. 系统定位

`System` 是运行在 ESP32-S3 上的传感采集与 CAN 通信节点，主要负责：

- 采集 21 路 AS5047P 磁编码器角度数据。
- 维护编码器链路状态、帧校验错误和 AS5047P 诊断错误。
- 通过 ESP-IDF TWAI 驱动向 CAN 总线发送编码器数据和错误状态。
- 预留触觉传感器数据结构、HAL 和任务入口。
- 接收来自 ESP32-P4/上位机侧的校准命令，并通过系统管理任务执行校准流程占位逻辑。
- 配置 FreeRTOS 多任务、任务看门狗和双核任务绑定。

整体上，ESP32-S3 扮演底层实时采集节点，ESP32-P4/桌面端可通过 CAN 接收数据、下发命令。

## 2. 目录结构

```text
ESP32-S3/System
├── System.ino                  # Arduino 主入口，完成启动初始化和主循环让出 CPU
├── architecture.md             # 当前架构说明文档
└── src
    ├── config
    │   └── Config.h            # 全局参数、数据结构、引脚、任务和硬件兼容配置
    ├── hal
    │   ├── HalEncoders.*       # 编码器 SPI/MUX 访问与 AS5047P 数据解码
    │   ├── HalTactile.*        # 触觉传感器数据缓存与占位更新逻辑
    │   └── HalTWAI.*           # TWAI/CAN 初始化、发送、接收和总线维护
    ├── system
    │   └── SystemTasks.*       # FreeRTOS 队列创建和任务启动编排
    └── tasks
        ├── EncodersTask.*      # 编码器采集任务
        ├── CanTask.*           # CAN 通信任务
        ├── TactileTask.*       # 触觉任务入口
        └── SysMgrTask.*        # 系统管理/校准任务
```

## 3. 启动流程

启动入口位于 `System.ino`。

1. `Serial.begin(115200)` 初始化串口，并延时等待串口稳定。
2. 配置 ESP-IDF v5 风格任务看门狗：
   - `timeout_ms = 1000`
   - 监控 Core 0 和 Core 1 的 Idle Task
   - 超时触发 panic 重启
3. 配置 ESP 日志等级。
4. 初始化硬件抽象层：
   - `encoders.begin()` 初始化编码器 SPI、CS 和 MUX 控制引脚。
   - `tactile.begin()` 初始化触觉传感器片选引脚，目前 SPI 读取逻辑仍是占位。
   - `twaiBus.begin()` 安装并启动 TWAI/CAN 驱动。
5. 调用 `startSystemTasks()` 创建队列并启动所有 FreeRTOS 任务。
6. `loop()` 中当前仅周期性 `vTaskDelay(pdMS_TO_TICKS(100))`，把 CPU 时间让给 FreeRTOS 任务和 Idle Task。

## 4. 分层架构

### 主入口层

- 文件：`System.ino`
- 职责：系统启动、看门狗配置、HAL 初始化、任务启动。
- 当前主循环不承载核心业务逻辑，实时功能由 FreeRTOS 任务完成。

### 配置与数据结构层

- 文件：`src/config/Config.h`
- 职责：
  - 定义编码器数量：`ENCODER_TOTAL_NUM = 21`
  - 定义触觉组数：`TACTILE_GROUP_NUM = 5`
  - 定义编码器、触觉、CAN、MUX 引脚。
  - 定义 `EncoderData`、`TactileData`、`RemoteCommand` 等跨模块数据结构。
  - 定义 AS5047P 错误码、编码器 FSM 状态、硬件兼容模式和任务参数。

### HAL 层

- `HalEncoders`：负责 AS5047P 编码器的 SPI 事务、MUX 切换、帧解码、奇偶校验、链路丢失判断和错误码锁存。
- `HalTWAI`：负责 TWAI/CAN 驱动初始化、CAN 帧发送、校准命令接收、错误状态打包和总线自恢复。
- `HalTactile`：负责触觉数据缓存和互斥访问。目前只实现数据结构、片选初始化和模拟更新入口。

### 任务层

- `Task_Encoders`：周期采集编码器数据并写入队列。
- `Task_CanBus`：从队列获取最新编码器数据，通过 CAN 发送，同时接收校准命令。
- `Task_Tactile`：触觉任务入口，目前实际 `tactile.update()` 被注释。
- `Task_SysMgr`：等待通知后执行系统级校准流程占位逻辑。

### 系统编排层

- 文件：`src/system/SystemTasks.cpp`
- 职责：
  - 创建 `xQueueEncoderData`，队列长度为 1，只保留最新一帧 `EncoderData`。
  - 创建并绑定四个任务到指定 CPU 核心。

## 5. FreeRTOS 任务架构

| 任务 | 周期 | Core | 优先级 | 栈大小 | 当前职责 |
| --- | ---: | ---: | ---: | ---: | --- |
| `Task_Encoders` | 5 ms / 200 Hz | 1 | 10 | 4096 | 读取 21 路编码器，处理错误标记，覆盖写入编码器队列 |
| `Task_CanBus` | 10 ms / 100 Hz | 0 | 5 | 4096 | 维护 TWAI 总线，发送编码器数据/错误状态，接收校准命令 |
| `Task_Tactile` | 100 ms / 10 Hz | 0 | 5 | 8192 | 触觉任务入口，目前更新逻辑被注释 |
| `Task_SysMgr` | 事件触发 | 1 | 1 | 6144 | 等待校准通知，挂起/恢复任务，发送校准 ACK |

任务间主要通过两种方式通信：

- 编码器数据：`Task_Encoders` 使用 `xQueueOverwrite()` 写入 `xQueueEncoderData`，`Task_CanBus` 使用 `xQueueReceive()` 读取。
- 校准通知：`Task_CanBus` 收到指定 CAN 命令后，使用 `xTaskNotifyGive(xSysMgrTask)` 通知 `Task_SysMgr`。

## 6. 数据流

编码器高频数据流：

```text
AS5047P 编码器
    ↓ SPI + MUX
HalEncoders::getData()
    ↓
Task_Encoders
    ↓ xQueueEncoderData，长度 1，只保留最新数据
Task_CanBus
    ↓
HalTWAI::sendEncoderData()
    ↓
TWAI/CAN 总线
    ↓
ESP32-P4 / 上位机
```

错误状态数据流：

```text
HalEncoders::decodeFrame()
    ↓ 奇偶校验 / error bit / ERRFL / DIAAGC / link lost
EncoderData.errorFlags + EncoderData.latchedErrors
    ↓
Task_CanBus 每 5 个 CAN 周期检查一次
    ↓
HalTWAI::sendErrorStatus()
    ↓
CAN ID 0x1F0 ~ 0x1F2
```

校准命令流：

```text
CAN RX: ID 0x200, data[0] == 0xCA
    ↓
Task_CanBus
    ↓ xTaskNotifyGive()
Task_SysMgr
    ↓
读取当前编码器数据，挂起任务，执行占位校准流程，恢复任务
    ↓
HalTWAI::sendCalibrationAck(true)
    ↓
CAN TX: ID 0x300
```

## 7. 编码器子系统

### 硬件与数量

- 编码器总数：21 路。
- 分组数量：5 组。
- 当前组大小：`{4, 4, 4, 4, 5}`。
- 编码器型号按代码实现面向 AS5047P。
- 编码器 SPI 使用 `SPI2_HOST`。
- 编码器读数范围：14 bit，`0 ~ 16383`。

### 引脚

| 信号 | 引脚 |
| --- | ---: |
| `PIN_ENC_MISO` | 47 |
| `PIN_ENC_MOSI` | 38 |
| `PIN_ENC_SCLK` | 48 |
| `PIN_ENC_CS` | 7 |
| `PIN_MUX_A` | 2 |
| `PIN_MUX_B` | 4 |
| `PIN_MUX_C` | 5 |

### 硬件兼容模式

代码支持两种编码器硬件拓扑：

- `ENC_HW_MODE_138_CS_DEMUX`：CS demux 拓扑。
- `ENC_HW_MODE_151_MISO_MUX`：MISO mux 拓扑，当前默认模式。

151 模式下的组通道映射：

```cpp
{4, 3, 5, 6, 7}
```

151 模式还支持诊断模式：

- `ENC_151_DIAG_MODE_SAFE`：按需读取 ERRFL。
- `ENC_151_DIAG_MODE_FULL`：保持 `ANGLE -> ERRFL -> DIAAGC` 诊断流程，当前默认模式。

### 错误处理

`HalEncoders` 会在解码阶段处理：

- SPI 传输失败。
- 返回帧为 `0x0000` 或 `0xFFFF`，判定为链路丢失。
- AS5047P 帧奇偶校验失败。
- AS5047P error bit 置位。
- `ERRFL` 寄存器错误：
  - `FRERR`
  - `INVCOMM`
  - `PARERR`
- `DIAAGC` 诊断错误：
  - 磁场过低 `MAG_LOW`
  - 磁场过高 `MAG_HIGH`
  - 其他诊断位

错误码会锁存在 `EncoderData.latchedErrors` 中，发送端再映射为 CAN 错误详情码。

## 8. 触觉子系统

触觉相关数据结构已定义：

- `TACTILE_GROUP_NUM = 5`
- 每组 `TacGroup` 包含：
  - 1 个 `TacIndent2015 sensor_A`
  - 2 个 `TacIndent1610 sensor_B/sensor_C`
- 每个触觉传感器包含局部 force 数组和 `global[3]` 摘要数据。

当前实现状态：

- `HalTactile::begin()` 初始化 `PIN_TAC_CS_A` 和 `PIN_TAC_CS_B`。
- `HalTactile::update()` 目前只包含模拟数据递增逻辑。
- `Task_Tactile` 中 `tactile.update()` 目前被注释，实际不会周期更新触觉数据。
- `HalTWAI::sendTactileSummary()` 已实现触觉摘要数据打包发送。
- `HalTWAI::sendTactileFullDump()` 仍是占位函数。

触觉引脚：

| 信号 | 引脚 |
| --- | ---: |
| `PIN_TAC_MISO` | 13 |
| `PIN_TAC_MOSI` | 11 |
| `PIN_TAC_SCLK` | 12 |
| `PIN_TAC_CS_A` | 7 |
| `PIN_TAC_CS_B` | 6 |

注意：`PIN_TAC_CS_A` 与 `PIN_ENC_CS` 当前都定义为 7。使用时需要结合实际硬件确认是否存在片选冲突或是否由不同总线/外部电路隔离。

## 9. CAN/TWAI 通信

CAN 使用 ESP-IDF TWAI 驱动：

- TX：`PIN_TWAI_TX = 9`
- RX：`PIN_TWAI_RX = 8`
- 波特率：`TWAI_TIMING_CONFIG_1MBITS()`
- 模式：`TWAI_MODE_NORMAL`
- 发送队列长度：50
- 接收队列长度：10
- 过滤器：接收全部帧

### 编码器角度数据

- 起始 ID：`0x100`
- 帧范围：`0x100 ~ 0x105`
- 总帧数：6 帧
- 数据格式：
  - 前 5 帧每帧发送 4 个编码器值。
  - 最后一帧发送第 21 路编码器值，剩余字节填 0。
  - 每个编码器值为 big-endian `uint16_t`。

### 编码器错误详情

- 起始 ID：`0x1F0`
- 帧范围：`0x1F0 ~ 0x1F2`
- 总帧数：3 帧
- 每帧格式：
  - `byte[0]`：序号 `seq`
  - `byte[1..7]`：最多 7 路编码器错误码
- 只有检测到至少一路 `errorFlags[i] != 0` 时才发送。
- `Task_CanBus` 每 5 个 CAN 周期检查一次错误状态。

常见错误码映射：

| CAN 错误码 | 含义 |
| ---: | --- |
| `0x00` | OK |
| `0x01` | LINK_LOST |
| `0x10` | ERRFL_FRERR |
| `0x11` | ERRFL_INVCOMM |
| `0x12` | ERRFL_PARERR |
| `0x20` | DIA_MAG_LOW |
| `0x21` | DIA_MAG_HIGH |
| `0x23` | DIA_OTHER_ONLY |
| `0xF0` | UNKNOWN_RAW |

### 触觉摘要数据

- 起始 ID：`0x200`
- 数据来源：每组触觉传感器的 `global[3]`。
- `sendTactileSummary()` 会按 8 字节一帧连续打包发送。
- 当前触觉任务未实际调用该发送流程。

### 校准命令接收

`Task_CanBus` 直接调用 `twai_receive()` 读取 CAN 帧。

触发条件：

- CAN ID：`0x200`
- DLC 大于 0
- `data[0] == 0xCA`

触发后通知 `Task_SysMgr` 执行校准流程。

注意：`0x200` 同时也是触觉摘要发送的起始 ID。当前代码里一个用于 TX 摘要，一个用于 RX 校准命令，实际系统集成时需要确认总线方向和协议设计不会冲突。

### 校准 ACK

- CAN ID：`0x300`
- DLC：1
- `data[0] = 0x01` 表示成功。
- `data[0] = 0x00` 表示失败。

当前 `Task_SysMgr` 固定发送 `sendCalibrationAck(true)`。

### 总线维护

`HalTWAI::maintain()` 在 CAN 任务中每 10 ms 调用：

- 如果总线进入 `TWAI_STATE_BUS_OFF`，调用 `twai_initiate_recovery()`。
- 如果总线进入 `TWAI_STATE_STOPPED`，尝试 `twai_start()`。
- 如果 TX/RX error counter 超过 100，通过串口打印警告。
- 总线异常时，`Task_CanBus` 会额外延时 50 ms，给驱动恢复时间。

## 10. 具体功能点清单

已实现：

- ESP32-S3 启动初始化。
- ESP-IDF 任务看门狗配置。
- 编码器 SPI 总线初始化。
- 编码器 MUX 通道选择。
- 21 路 AS5047P 编码器采集。
- 编码器分组读取和组内索引重映射。
- AS5047P 命令帧生成和奇偶校验。
- 编码器链路丢失检测。
- 编码器错误位、ERRFL 和 DIAAGC 诊断状态机。
- 编码器错误码锁存。
- FreeRTOS 队列传递最新编码器数据。
- TWAI/CAN 驱动初始化。
- 编码器角度 CAN 打包发送。
- 编码器错误详情 CAN 打包发送。
- TWAI bus-off/stopped 状态恢复。
- CAN 校准命令接收和任务通知。
- 校准 ACK 帧发送。
- 系统管理任务挂起/恢复其他任务的流程框架。

占位或待完善：

- 触觉传感器 SPI 初始化和真实读取尚未完成。
- `Task_Tactile` 当前没有调用 `tactile.update()`。
- 触觉完整数据 dump 函数 `sendTactileFullDump()` 仍为空实现。
- 校准流程目前没有真正计算、保存或应用校准参数。
- `RemoteCommand.value` 在当前 CAN 接收逻辑中未解析。
- `EncoderData.finalAngles` 当前未见实际计算流程。
- 部分源码中文注释存在编码乱码，阅读和维护时需要注意。

## 11. 快速阅读建议

如果想快速理解运行链路，建议按以下顺序阅读：

1. `System.ino`：看启动入口和初始化顺序。
2. `src/config/Config.h`：看数量、引脚、数据结构和协议常量。
3. `src/system/SystemTasks.cpp`：看任务创建和队列关系。
4. `src/tasks/EncodersTask.cpp`：看编码器采集周期和队列写入。
5. `src/hal/HalEncoders.cpp`：看 AS5047P 读取、诊断和错误处理。
6. `src/tasks/CanTask.cpp`：看 CAN 发送、错误上报和校准命令接收。
7. `src/hal/HalTWAI.cpp`：看 CAN 帧打包格式和总线维护逻辑。
