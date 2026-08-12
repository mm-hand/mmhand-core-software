# ServoBoardMain

`ServoBoardMain` 是 MMHand 的 ESP32-P4 主控固件，负责：

- 21 路磁编码器和触觉 CAN 数据接收。
- 27 台 ST3215 舵机的四总线通信。
- 单腱/双腱反馈控制、碰撞保持和动态工作区约束。
- Manual/Auto 自动标定、Tendon LUT 和 Rope Init。
- 舵机热插拔、多圈恢复、Reset Zero 与 NVS 持久化。
- 上位机串口协议、状态、故障和遥测。

## 入口

`ServoBoardMain.ino` 只保留 Arduino `setup()/loop()`：

- `setup()` 调用 `appBootstrapSetup()`。
- `loop()` 调用 `appBootstrapLoop()`。

所有业务源码位于 `src/`。

## 关键边界

- 飞特 SDK 只允许出现在
  `src/hal/servo/FeetechServoDriver.*`。
- 所有舵机目标只允许通过
  `src/modules/actuator/ActuatorCommandGateway.*`。
- 状态和控制 owner 由
  `src/tasks/system_state/SystemStateCoordinator.*` 统一仲裁；`SystemStateTask` 仅为薄入口。
- 运行期 Flash 写入只允许由
  `src/tasks/nvs_persistence/NvsPersistenceService.*` 执行。
- 固定参数集中在 `src/config/`。
- 协议编号和 NVS 格式保持向当前桌面端兼容。

当前分层目录、命令/反馈链路、状态机、安全规则和双格式框图统一见：

[P4_嵌入式软件架构与安全规则_20260807.md](docs/P4_嵌入式软件架构与安全规则_20260807.md)

## 编译

Arduino CLI 调试编译：

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass `
  -File tools\p4_debug\build_debug.ps1 -ReuseCache
```

架构静态检查：

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass `
  -File tools\p4_architecture\check_p4_architecture.ps1
```

URDF 上限表检查：

```powershell
python tools\generate_robot_model.py --check
```

## 调试

Arduino 兼容的 ESP-IDF/OpenOCD 断点环境见：

[ESP32-P4_Arduino断点调试_README.md](ESP32-P4_Arduino断点调试_README.md)
