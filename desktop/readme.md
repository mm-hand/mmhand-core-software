# desktop 上位机程序说明

本文档用于帮助使用者快速理解 `desktop` 目录的功能定位、运行方式、模块分层、数据流、串口协议和配置文件。内容以当前源码实现为准；该目录已经从原先 ESP32-P4 工程下的子目录上移到仓库根目录，使上位机程序与 ESP32-P4、ESP32-S3 固件目录并列。

## 1. 系统定位

`desktop` 是运行在 PC 上的灵巧手上位机程序，主要负责：

- 提供图形界面，显示关节、舵机、触觉、故障和调试状态。
- 通过串口连接 ESP32-P4 主控板，发送控制命令并接收实时状态。
- 打包和解析自定义二进制串口协议帧。
- 维护上位机侧的 `HandModel` 状态快照。
- 读取和保存手部几何、编码器零点、腱保护等配置。
- 提供监控、遥操作、算法接入和开发调试模式。
- 记录运行数据，并提供运行曲线查看工具。

整体关系如下：

```text
desktop 上位机
  -> USB CDC / UART 串口
  -> ESP32-P4 ServoBoardMain
  -> TWAI/CAN
  -> ESP32-S3 System
```

上位机不直接控制 ESP32-S3；它主要通过 ESP32-P4 间接看到 S3 上传的关节编码器、错误和触觉摘要数据。

## 2. 目录结构

```text
desktop/
├── main.py                    # 上位机入口，解析命令行参数并选择运行模式
├── modes.py                   # monitor / teleop / algorithm 三种 GUI 流程入口
├── ui_main.py                 # 主 GUI，串口选择、控制面板、状态显示、触觉和曲线视图
├── core_logic.py              # HandController，维护连接生命周期、状态更新和控制命令
├── comm_layer.py              # 串口收发线程、TX/RX 队列、上行包处理
├── protocol.py                # 协议常量、帧解析、下行命令打包和上行 payload 解析
├── data_models.py             # HandModel、编码器、舵机、触觉模型和触觉形状工具
├── calibration.py             # 标定入口，触发下位机标定并保存零点
├── hand_geometry.py           # 手部几何、零点配置读写和 2D 线段计算
├── hand_visualizer_ui.py      # 手部可视化组件
├── development_mode.py        # 终端开发调试模式
├── example_teleop.py          # teleop 模式示例入口
├── example_algorithm.py       # algorithm 模式示例入口
├── run_data_logger.py         # 运行数据 CSV 记录器
├── plot_hand.py               # 手模型演示可视化
├── plot_hand_from_urdf.py     # 基于 URDF 的手部骨架/姿态可视化
├── plot_joint_result.py       # 运行数据曲线查看工具
├── requirements.txt           # Python 依赖
├── config/
│   ├── calib_deg.json         # 21 路编码器零点 raw 值
│   ├── hand_geo.json          # 手部几何、手指和角度索引配置
│   └── tendon_manual_calib.json # 腱保护手动配置
└── hku_hand_v2_urdf/          # 手部 URDF、mesh 和相关分析资源
```

## 3. 运行方式

建议在 `desktop` 目录下运行：

```powershell
cd desktop
pip install -r requirements.txt
python main.py
```

默认运行等价于：

```powershell
python main.py --app-mode control --mode monitor
```

常用启动命令：

| 命令 | 说明 |
| --- | --- |
| `python main.py` | 启动默认 control + monitor GUI |
| `python main.py --app-mode control --mode monitor` | 启动监控/手动控制 GUI |
| `python main.py --app-mode control --mode teleop` | 启动遥操作示例 GUI 流程 |
| `python main.py --app-mode control --mode algorithm` | 启动算法示例线程和 GUI |
| `python main.py --app-mode development` | 启动终端开发调试模式 |
| `python main.py --port COMx` | 启动时直接连接指定串口 |
| `python main.py --app-mode development --no-plot` | 开发模式下关闭调试曲线窗口 |

依赖来自 `requirements.txt`：

- `pyserial`
- `numpy`
- `matplotlib`
- `opencv-python`
- `Pillow`

## 4. 运行模式

### 4.1 control / monitor

默认模式。`main.py` 创建 `HandController`，根据 `--mode monitor` 关闭 PID 控制标志，然后通过 `modes.py` 打开 `HandGUI`。

该模式主要用于：

- 选择并连接串口。
- 查看 21 路关节/编码器状态。
- 查看 22 路舵机角度、raw 位置、遥测和在线状态。
- 查看触觉摘要、故障位图和 joint debug 数据。
- 发送 START、STOP、RESET、关节角目标、电机位置目标和腱保护配置。
- 记录运行数据到仓库根目录 `run_data`。

### 4.2 control / teleop

`modes.py` 调用 `example_teleop.py` 中的 `run(controller, port=None)`。当前文件是示例入口，适合作为遥操作逻辑的接入点。

### 4.3 control / algorithm

`modes.py` 调用 `example_algorithm.py` 中的 `run(controller, stop_event=None)`，并同时打开 GUI。算法线程通过 `stop_event` 接收退出信号。

### 4.4 development

`development_mode.py` 提供终端调试模式：

- 交互式选择串口。
- 周期打印传感器、舵机和标定状态。
- 可选显示关节调试曲线。
- 支持 `--no-plot` 关闭曲线窗口。

## 5. 模块分层

### 5.1 入口层

- `main.py` 负责解析 `--app-mode`、`--mode`、`--port`、`--no-plot`。
- `modes.py` 负责把 monitor、teleop、algorithm 分发到对应 GUI 或示例逻辑。

### 5.2 控制核心层

`core_logic.py` 中的 `HandController` 是上位机核心对象，负责：

- 创建和关闭 `LowerComputerComm` 串口通信对象。
- 启动状态更新线程，从 RX 队列取最新 `HandModel`。
- 维护暂停、启动、PID 标志和控制模式。
- 下发关节角、电机 raw 位置、电机绝对位置、腱保护、START、STOP、RESET 等命令。
- 连接成功后读取 `config/calib_deg.json`，并向下位机发送 21 路零点数据。

### 5.3 通信层

`comm_layer.py` 中的 `LowerComputerComm` 负责串口连接和线程化收发：

- RX 线程从串口读取字节流，调用 `parse_frame()` 拆帧。
- TX 线程从发送队列取命令，调用 `protocol.py` 中的打包函数发送。
- RX 队列默认保留上行模型快照，GUI 和控制层消费最新状态。
- 连接后默认请求 `SENSOR_STREAM_MODE_SIGNED_I16` 传感器流模式。

### 5.4 协议层

`protocol.py` 定义协议常量、帧格式、上行解析和下行命令构造。关键数量：

| 常量 | 值 | 含义 |
| --- | ---: | --- |
| `ENCODER_COUNT` | 21 | 关节/磁编码器通道数 |
| `MOTOR_COUNT` | 22 | 舵机/电机通道数 |
| `TACTILE_GROUP_COUNT` | 5 | 触觉组数 |
| `TACTILE_SENSOR_PER_GROUP` | 3 | 每组触觉传感器数 |
| `TACTILE_AXIS_COUNT` | 3 | 每个触觉传感器摘要轴数 |
| `BAUDRATE` | 921600 | 上位机和 ESP32-P4 串口波特率 |

### 5.5 数据模型层

`data_models.py` 定义上位机内部状态：

- `HandModel`：整只手的状态快照。
- `JointEncoder`：单路关节/编码器数据。
- `MotorSlot`：单路电机状态。
- `FingerTactile` 和 `TactileSensor`：触觉摘要和可视化用数据。
- 触觉形状、拼接、传感器轮廓和绘制辅助函数。

### 5.6 配置和可视化层

- `hand_geometry.py` 读取 `hand_geo.json` 和 `calib_deg.json`，并计算 2D 手部线段。
- `hand_visualizer_ui.py` 提供手部可视化组件。
- `plot_hand.py` 和 `plot_hand_from_urdf.py` 用于离线姿态和 URDF 可视化。
- `plot_joint_result.py` 读取 `run_data` 中的 CSV 并绘制运行曲线。
- `run_data_logger.py` 负责运行数据 CSV 写入。

## 6. 数据流

### 6.1 上行状态流

```text
ESP32-P4
  -> 串口帧 [0xFE][LEN][TYPE][PAYLOAD][0xFF]
  -> LowerComputerComm._rx_loop()
  -> protocol.parse_frame()
  -> comm_layer._process_packets()
  -> HandModel
  -> HandController._update_loop()
  -> GUI callback / UI 状态刷新
```

上行状态包括：

- 21 路传感器/编码器数据。
- 22 路舵机多圈角度和在线状态。
- 22 路舵机 raw 位置和在线状态。
- 舵机速度、负载、电压、温度和在线状态。
- 5 组触觉摘要数据。
- 协议 ACK、标定状态、故障位图、反缠绕释放保护位图。
- joint debug 数据。

### 6.2 下行控制流

```text
GUI / 示例算法 / 开发模式
  -> HandController
  -> LowerComputerComm.tx_queue
  -> protocol.build_*_cmd()
  -> 串口写入
  -> ESP32-P4 UpperCommTask
```

下行控制包括：

- 标定命令和零点数据。
- START、STOP、RESET。
- 21 路关节角目标。
- 22 路电机 raw 位置、滑条扫动位置、多圈绝对位置。
- 传感器流模式切换。
- 21 路腱保护配置。

### 6.3 运行数据记录

GUI 中的记录功能会通过 `run_data_logger.py` 写入 CSV。默认目录为仓库根目录：

```text
run_data/
```

`plot_joint_result.py` 默认从该目录读取最新 CSV 文件。

## 7. 串口协议

上位机和 ESP32-P4 之间使用自定义二进制帧：

```text
0xFE LEN TYPE_OR_CMD PAYLOAD 0xFF
```

说明：

- `0xFE`：帧头。
- `LEN`：从 `TYPE_OR_CMD` 到帧尾 `0xFF` 的长度。
- `TYPE_OR_CMD`：上行包类型或下行命令。
- `PAYLOAD`：负载。
- `0xFF`：帧尾。

### 7.1 下行命令

| 命令 | 名称 | 功能 |
| ---: | --- | --- |
| `0xCA` | `CMD_CALIBRATE` | 触发标定流程 |
| `0xCB` | `CMD_ANGLE_CTRL` | 下发 21 路关节目标角，float 小端 |
| `0xCC` | `CMD_START` | 使能控制输出 |
| `0xCD` | `CMD_STOP` | 停止控制输出 |
| `0xCE` | `CMD_RESET` | 清空目标并回到默认状态 |
| `0xCF` | `CMD_CALIB_DATA` | 下发 21 路编码器零点 raw 值 |
| `0xD0` | `CMD_MOTOR_POS` | 下发 22 路单圈电机 raw 目标 |
| `0xD1` | `CMD_SENSOR_STREAM_MODE` | 切换传感器上报编码模式 |
| `0xD2` | `CMD_MOTOR_POS_SWEEP` | 下发 22 路滑条扫动电机目标 |
| `0xD3` | `CMD_MOTOR_POS_ABS` | 下发 22 路多圈绝对电机目标 |
| `0xD4` | `CMD_TENDON_GUARD` | 下发 21 路腱保护配置 |

### 7.2 上行包类型

| 类型 | 名称 | 内容 |
| ---: | --- | --- |
| `0x01` | `PACKET_TYPE_SENSOR` | 21 路关节/编码器数据 |
| `0x02` | `PACKET_TYPE_CALIB_ACK` | 标定状态 |
| `0x03` | `PACKET_TYPE_SERVO_ANGLE` | 舵机多圈角度和在线状态 |
| `0x04` | `PACKET_TYPE_JOINT_DEBUG` | 关节目标、反馈、PID 和命令位置调试数据 |
| `0x05` | `PACKET_TYPE_SERVO_TELEM` | 舵机速度、负载、电压、温度和在线状态 |
| `0x06` | `PACKET_TYPE_PROTO_ACK` | 协议命令 ACK |
| `0x07` | `PACKET_TYPE_FAULT_STATUS` | 过载故障位图 |
| `0x08` | `PACKET_TYPE_RELEASE_FAULT` | 反缠绕释放保护故障位图 |
| `0x09` | `PACKET_TYPE_SERVO_RAW` | 舵机单圈 raw 位置和在线状态 |
| `0x0A` | `PACKET_TYPE_TACTILE` | 触觉摘要数据 |

### 7.3 触觉摘要

当前上位机按以下结构解析触觉摘要：

```text
5 groups * 3 sensors/group * 3 axes/sensor = 45 bytes
```

每个触觉传感器摘要包含：

- `fx`：int8
- `fy`：int8
- `fz`：uint8

如果 payload 带有额外的 1 字节序号，上位机会同时接受并忽略到数据模型外。

## 8. 配置文件

### 8.1 `config/calib_deg.json`

保存 21 路编码器零点 raw 值。`HandController.initialize()` 连接成功后会读取该文件，并通过 `CMD_CALIB_DATA` 下发给 ESP32-P4。

### 8.2 `config/hand_geo.json`

保存手部几何、手指结构和角度索引映射。主要用于：

- 2D 手部线段计算。
- GUI 手部可视化。
- URDF 可视化角度映射。

### 8.3 `config/tendon_manual_calib.json`

保存腱保护相关的手动配置。GUI 会读取并可下发为 `CMD_TENDON_GUARD`。

## 9. 已实现功能和注意事项

已实现：

- GUI 启动、串口枚举和连接。
- 上位机和 ESP32-P4 的二进制协议收发。
- 21 路关节目标角下发。
- 22 路电机 raw / sweep / absolute 位置下发。
- START、STOP、RESET、标定和零点下发。
- 舵机角度、raw、遥测、在线状态解析。
- 编码器、触觉、故障和 joint debug 数据解析。
- 运行数据 CSV 记录和曲线查看。
- 基于手部几何和 URDF 的可视化工具。

注意事项：

- `desktop` 现在位于仓库根目录，文档和脚本应使用 `desktop/...` 路径。
- `ESP32-P4/client.py` 是旧脚本，不在 `desktop` 目录下。
- 部分源码中文注释在当前文件中显示为乱码，维护时需要注意源文件编码历史。
- `teleop` 和 `algorithm` 当前是示例接入点，实际行为取决于对应示例文件中的实现。
- 真实硬件连接前，应确认串口号、ESP32-P4 固件协议版本和电机安全状态。

## 10. 快速阅读建议

如果想快速理解上位机运行链路，建议按以下顺序阅读：

1. `main.py`：看命令行参数和应用模式。
2. `modes.py`：看 monitor、teleop、algorithm 如何进入 GUI。
3. `core_logic.py`：看 `HandController` 如何连接、更新状态和发送命令。
4. `comm_layer.py`：看串口 RX/TX 线程和 `HandModel` 更新。
5. `protocol.py`：看协议帧格式、命令和上行包解析。
6. `data_models.py`：看上位机状态结构和触觉模型。
7. `ui_main.py`：看主 GUI、控制按钮、数据展示和运行数据记录。
8. `hand_geometry.py` 与 `config/`：看手部几何、零点和腱保护配置。
