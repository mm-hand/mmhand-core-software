# CAN 零点标定旧链路说明

当前系统采用方案 A：

- S3 只作为原始磁编码器和触觉采集板。
- S3 上传 raw encoder values，不在 S3 内部做零点 offset 变换。
- P4 负责关节自动标定、`g_jointCalibResult`、舵机 ABS 限位和上位机 `Calib Load` 恢复。

## 废弃的旧链路

旧 S3 代码曾保留一个采集板零点标定入口：

```text
CAN ID: 0x200
Data[0]: 0xCA
```

收到该帧后，旧逻辑会通知 `SysMgrTask` 并发送 `0x300` ACK。

这条链路目前不再作为有效标定链路使用，原因是：

- P4 当前自动标定不依赖 S3 保存零点。
- P4 未使用 S3 `0x300` ACK 作为自动标定完成条件。
- 如果 S3 也做零点变换，会和 P4 标定结果叠加，增加排查难度。

## 当前代码行为

`ESP32-S3/System/src/tasks/CanTask.cpp` 现在会忽略 `0x200/0xCA`，并只打印一次提示：

```text
[Task_CanBus] Ignored legacy CAN calibration command 0x200/0xCA; P4 owns encoder calibration.
```

这不会影响 P4 自动标定。P4 自动标定仍然读取 S3 上传的 raw 编码器值，并在 P4 端完成关节标定闭环。
