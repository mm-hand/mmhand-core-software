# MANUS 遥操链路简明说明

本文是当前 MANUS 遥操的唯一主说明。其它 MANUS 目录 README 只做入口跳转，避免多份文档内容重复。

## 1. 数据链路

当前支持两种 MANUS 输入源：

```text
TCP Bridge:
MANUS 手套 -> MANUS Core -> manus_bridge_test.exe
-> TCP JSON 127.0.0.1:9001 -> desktop 上位机 -> P4 -> 灵巧手

Direct SDK:
MANUS 手套 -> MANUS Core -> MANUS C++ SDK
-> mmhand_manus_sdk.dll -> desktop 上位机 -> P4 -> 灵巧手
```

两种输入源进入上位机后共用同一套逻辑：

```text
MANUS ergonomics
-> Open/Close 校准
-> 归一化
-> 映射到 J00-J20
-> 按遥操勾选关节发送目标
```

## 2. 关键文件

- `desktop/ui/ui_teleop_view.py`：遥操入口，选择 `Mode 2: MANUS / Rokoko Glove`。
- `desktop/ui/ui_manus_view.py`：MANUS 页面、输入源切换、校准、预览、发送门控。
- `desktop/teleop/manus_receiver.py`：TCP Bridge 接收器。
- `desktop/teleop/manus_sdk_receiver.py`：Direct SDK 接收器。
- `desktop/teleop/manus_calibration.py`：Open/Close 校准、归一化、J00-J20 映射。
- `desktop/config/manus_calibration.json`：MANUS 校准数据。
- `manus/manus_bridge_test/`：旧版 TCP Bridge 源码。
- `manus/mmhand_manus_sdk/`：Direct SDK 的 C ABI 包装层。

## 3. 使用步骤

1. 启动 MANUS Core，并确认手套已连接。
2. 启动 mmhand desktop 上位机。
3. 在遥操区域选择 `Mode 2: MANUS / Rokoko Glove`。
4. 在 `Source` 选择输入源：
   - `TCP Bridge 127.0.0.1:9001`
   - `Direct SDK`
5. 等待页面显示 `LIVE`，并确认 Typed Ergonomics 表格有数据。
6. 点击 `Load Calibration`，或重新执行 `Record Open`、`Record Close`。
7. 在遥操关节选择区勾选需要跟随 MANUS 的手指或关节。
8. 确认灵巧手已标定且负反馈可用，再勾选 `Enable Joint Send`。

未勾选的关节不会跟随 MANUS，会用当前反馈角保持。

## 4. 发送安全门控

MANUS 数据不会自动直接控制灵巧手。发送前必须同时满足：

- 输入源为 `LIVE`。
- 已加载或记录 Open/Close 校准。
- 用户手动勾选 `Enable Joint Send`。
- 上位机成功切到 `ControlMode.JOINT_ANGLE` 并开启 PID 负反馈。
- 至少选中一个遥操关节。
- 未选中关节有反馈角可用于保持。

如果条件不满足，上位机会暂停发送并显示原因。

## 5. TCP Bridge 模式

运行 Bridge：

```powershell
cd manus/manus_bridge_test
.\scripts\build_windows.ps1
.\scripts\run_windows.ps1
```

然后在上位机选择 `TCP Bridge 127.0.0.1:9001`。

正常状态应看到：

- `Client Connected: yes`
- `Frames` 持续增加
- `recv_hz` 有数值
- `LIVE`

## 6. Direct SDK 模式

SDK 文件位置：

```text
manus/third_party/ManusSDK/
  include/ManusSDK.h
  include/ManusSDKTypes.h
  include/ManusSDKTypeInitializers.h
  lib/ManusSDK.lib
  lib/ManusSDK.dll
```

当前本机已从旧工程补入 `ManusSDK.dll`，并根据 DLL 导出表生成了本地 `ManusSDK.lib`。这些 `.dll/.lib` 属于本机 SDK/构建产物，已被 `manus/.gitignore` 忽略，不建议提交。

编译包装层：

```powershell
cd manus/mmhand_manus_sdk
.\scripts\build_windows.ps1
```

上位机会自动尝试加载：

```text
manus/mmhand_manus_sdk/build/windows/Release/mmhand_manus_sdk.dll
manus/mmhand_manus_sdk/build/windows/Debug/mmhand_manus_sdk.dll
manus/mmhand_manus_sdk/bin/mmhand_manus_sdk.dll
manus/mmhand_manus_sdk/mmhand_manus_sdk.dll
```

当前本机状态：

- `mmhand_manus_sdk.dll` 已成功编译。
- Python `ManusSdkReceiver` 已能加载该 DLL。
- 还需要启动 MANUS Core 并连接真实手套后，实测 Direct SDK 是否进入 `LIVE`。

## 7. 当前验证结论

- Python 语法检查通过。
- TCP 接收器本地 JSON 收帧自测通过。
- Direct SDK 包装层可编译、可被 Python 加载。
- TCP Bridge 和 Direct SDK 共用同一套 `Enable Joint Send` 安全门控。
- MANUS 相关二进制产物不应提交到仓库。

## 8. 常见排查

- `MANUS receiver failed`：
  - TCP 模式检查 `manus_bridge_test.exe` 是否运行。
  - Direct SDK 模式检查 `mmhand_manus_sdk.dll` 是否存在。
- 有数据但不发送：
  - 检查 Open/Close 校准。
  - 检查是否勾选遥操关节。
  - 检查输入源是否 `LIVE`。
  - 检查灵巧手是否已标定并处于可控制状态。
- 映射方向不对：
  - 优先重新采集 Open/Close。
  - 再检查 `desktop/teleop/manus_calibration.py` 中的映射关系。
