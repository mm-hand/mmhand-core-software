# MANUS Bridge Test

当前阶段目标：打通只读实时链路。

```text
MANUS 手套
-> Windows MANUS Core
-> MANUS C++ SDK
-> manus_bridge_test
-> TCP JSON
-> mmhand/desktop Teleop / Glove 入口
-> 上位机显示 MANUS ergonomics 数据
```

当前只接收、打印、记录 CSV、通过 TCP 发送 JSON、显示数据。不要控制灵巧手，不要发送到 ESP32 / STM32，不要做 IK / retargeting，也不要把 MANUS 数据接入正式电机控制链路。

## 前置条件

- Windows
- Visual Studio 2022
- CMake
- MANUS Core
- MANUS C++ SDK
- MANUS 手套已连接并完成校准
- MANUS Core 里的虚拟手能跟随真实手运动

## SDK 放置方式

推荐把 SDK 放到：

```text
third_party/ManusSDK/
├── include/
│   ├── ManusSDK.h
│   ├── ManusSDKTypes.h
│   └── ManusSDKTypeInitializers.h
└── lib/
    ├── ManusSDK.lib
    └── ManusSDK.dll
```

也可以编译时显式指定：

```powershell
.\scripts\build_windows.ps1 -ManusSdkRoot C:\path\to\ManusSDK
```

## 配置

默认配置在：

```text
config/manus_bridge_config.json
```

关键字段：

```json
{
  "connection": "local",
  "print_hz": 10.0,
  "host_search_seconds": 1,
  "raw_skeleton_nodes_to_print": 5,
  "default_csv_output": "manus_log.csv",
  "tcp": {
    "enabled": true,
    "target_ip": "127.0.0.1",
    "target_port": 9001,
    "reconnect_s": 1.0,
    "max_hz": 120.0
  },
  "debug": {
    "print_hz": 10.0
  }
}
```

`tcp.max_hz` 用来限制实时显示流的发送频率，默认 120Hz。bridge 只向上位机发送最新帧，避免堆积旧帧。

## 编译

在 `manus_bridge_test` 目录下运行：

```powershell
cd D:\hand\new\mmhand\manus\manus_bridge_test
.\scripts\build_windows.ps1
```

## 运行

### Step 1：启动 MANUS Core

在 Windows 上打开 MANUS Core，确认：

```text
手套已连接
手套已校准
Core 里虚拟手能动
```

### Step 2：启动 mmhand 上位机

```powershell
cd D:\hand\new\mmhand\desktop
python main.py
```

切到：

```text
Teleop -> Mode 2: MANUS / Rokoko Glove
```

该入口是 receive-only，只监听 TCP、显示数据和状态，不调用任何电机控制发送函数。

### Step 3：启动 manus_bridge_test

```powershell
cd D:\hand\new\mmhand\manus\manus_bridge_test
.\scripts\run_windows.ps1 -Mode debug
```

bridge 控制台应显示：

```text
TCP JSON enabled target=127.0.0.1:9001 max_hz=120
[TCP] sent_total=... sent_hz=... tcp=connected
```

### Step 4：观察上位机

上位机 MANUS 面板应显示：

```text
MANUS connected
LIVE
Listen: 127.0.0.1:9001
TCP Connections: ...
seq=...
recv_hz=...
glove_id=...
side=...
ergonomics values...
```

停止 bridge 后，上位机应在约 2 秒后显示 timeout / disconnected。

## TCP JSON 协议

bridge 通过 TCP 发送 newline-delimited JSON，每行一帧：

```json
{"type":"manus_frame","version":1,"seq":1234,"timestamp":1710000000.123,"dt":0.0083,"glove_id":0,"side":"right","ergonomics_names":["ergonomics_0"],"ergonomics_values":[0.12]}
```

当前不要发送：

```text
robot_q_target
motor_target
joint_target
servo_target
```

## 成功标准

```text
1. manus_bridge_test 仍然能正常连接 MANUS Core；
2. 原来的 debug 数据仍然正常；
3. bridge 能通过 TCP 发送 JSON；
4. desktop 的 Teleop / Glove 入口能收到 JSON；
5. 上位机能显示 seq、glove_id、side、ergonomics_values；
6. 动手指时 ergonomics_values 跟着变化；
7. 停止 bridge 后，上位机显示 timeout / disconnected；
8. 整个过程没有控制电机。
```

## 代码组织

- `src/ManusClient.h/.cpp`：MANUS SDK 初始化、连接、callback、数据转换
- `src/DataPrinter.h/.cpp`：终端打印和打印频率控制
- `src/CsvRecorder.h/.cpp`：CSV 创建、header、逐帧写入、安全关闭
- `src/TcpSender.h/.cpp`：TCP JSON 发送
- `src/main.cpp`：参数解析、配置读取、主循环、Ctrl+C 退出
