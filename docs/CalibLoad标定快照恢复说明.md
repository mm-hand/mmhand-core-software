# Calib Load 标定快照恢复说明

> 当前专项文档，最后核对：2026-08-01。CalibLoad 恢复的是 Auto/LUT/ABS-limit 软件快照，不等于恢复飞特舵机的物理多圈坐标；后者见[持久化与掉电恢复](持久化与掉电恢复.md)。恢复前仍须匹配 Manual 条目 CRC、URDF 配置 CRC、mapping version 和 active scope。

本文说明上位机 `Calib Load` 按钮的使用场景、用户操作方法，以及后续开发者维护该功能时需要理解的数据流和协议。

## 1. 功能目的

自动标定完成后，下位机会得到两类关键运行时数据：

- 关节标定结果：`g_jointCalibResult[]`
- 舵机 ABS 限位：`motor_abs_limit_min/max`、`motor_abs_limit_valid_bitmap`、`motor_abs_limit_locked_bitmap`

这些数据让系统能够判断当前 active scope 是否已经标定、是否在工作空间内，并允许进入 `READY` 和负反馈控制。

问题是：这些标定结果目前主要保存在下位机运行时内存里。下位机断电重启后，运行时标定结果会丢失。如果机械结构、绳长、舵机安装和磁编码器安装都没有变化，重新完整自动标定会浪费时间。

`Calib Load` 的作用就是：

1. 自动标定成功后，上位机保存一份标定快照 JSON。
2. 下位机重启但机械未变化时，用户点击 `Calib Load`。
3. 上位机把上次快照恢复给下位机，让系统快速回到已标定状态。

## 2. 用户使用方法

### 第一次标定或机械变化后

1. 打开上位机并连接下位机。
2. 选择需要控制/标定的 `Control Set`。
3. 点击 `Calibrate`。
4. 等待标定完成，顶部状态显示 `CALIB SUCCESS`。
5. 等待约 1-2 秒，上位机会自动保存快照。
6. 状态栏出现类似信息：

```text
Calib snapshot saved to calib_load_snapshot.json
```

快照文件位置：

```text
desktop/resources/config/calib_load_snapshot.json
```

### 下位机重启后恢复标定

适用条件：

- 机械结构没有改变。
- 绳长没有改变。
- 舵机和绕线没有重新装配。
- 磁编码器位置没有改变。
- 使用的是同一套代码映射和同一只手。

操作：

1. 上电下位机。
2. 打开上位机并连接串口。
3. 确认当前没有运行直控或负反馈。
4. 点击 `Calib Load`。
5. 弹出确认框后，确认机械没有变化。
6. 加载成功后，状态栏会显示：

```text
Calib Load success: loaded N joints
```

随后系统会重新基于恢复后的标定结果判断 `READY`。

## 3. 两种断电场景

### 上位机重启，下位机不断电

下位机内存中的标定结果还在：

- `g_jointCalibResult[]` 仍有效。
- `motor_abs_limit_*` 仍有效。

上位机重连后，会重新收到下位机周期上报的 ABS limit 和关节标定快照，并再次保存/刷新 JSON。

这种情况通常不需要点击 `Calib Load`。

### 下位机重启，上位机不断电或重新打开

下位机运行时标定结果会清空：

- `g_jointCalibResult[]` 失效。
- `motor_abs_limit_valid_bitmap` 清零。
- ABS limit min/max 清空。

这时需要点击 `Calib Load`，让上位机从 JSON 恢复标定结果到下位机。

## 4. 不能使用 Calib Load 的情况

以下情况必须重新 `Calibrate`，不要使用 `Calib Load`：

- 换了绳或重新绕线。
- 调整了绳长、预紧位置或机械限位。
- 更换或重装舵机。
- 更换或重装磁编码器。
- 修改了舵机映射、关节映射、磁编 SPI 映射。
- 换了手型或代码中的 `HAND_PROFILE`。
- 标定时 active scope 和当前准备恢复的控制范围不一致，且快照中缺少对应关节。

简单判断：只要“当前机械状态”和上次成功标定时不是同一个状态，就重新标定。

## 5. 上位机保存逻辑

相关代码：

```text
desktop/ui/ui_control_panels.py
```

保存入口：

```text
_maybe_save_calibration_snapshot()
```

保存条件：

- 当前 `calib_status == SUCCESS`。
- 当前 active joint mask 不为空。
- 当前 active motor 的 ABS limit 都有效。
- 至少收到一个成功的关节标定快照。
- active scope 中要求的关节都已有成功标定快照。

保存文件：

```text
desktop/resources/config/calib_load_snapshot.json
```

主要字段：

```json
{
  "schema_version": 1,
  "hand_profile": "v2",
  "active_joint_mask": "0x0000000F",
  "active_motor_mask": "0x0000001F",
  "motor_abs_limit_valid_bitmap": "0x0000001F",
  "motor_abs_limit_locked_bitmap": "0x0000001F",
  "motor_abs_limit_min": [],
  "motor_abs_limit_max": [],
  "joint_calib_results": []
}
```

`joint_calib_results` 中保存每个关节的：

- `success`
- `encoderMin`
- `encoderMax`
- `offset`
- `angleMax`
- `startEncoder`
- `tightEncoder`
- `zeroEncoder`
- `encoderSpan`
- `startServoAbs`
- `tightServoAbs`
- `zeroServoAbs`
- `startDeg`
- `tightDeg`
- `totalDeg`
- `scaled`
- `encoderDirection`

## 6. 上位机加载逻辑

相关代码：

```text
desktop/ui/ui_control_panels.py
desktop/core/core_logic.py
desktop/core/comm_layer.py
desktop/core/protocol.py
```

UI 入口：

```text
load_calibration_snapshot()
```

控制器入口：

```text
HandController.load_calibration_snapshot()
```

加载顺序：

1. 读取 `desktop/resources/config/calib_load_snapshot.json`。
2. 校验 `hand_profile`。
3. 弹出中文确认框，要求用户确认机械未变化。
4. 下发 `CMD_CALIB_LOAD_LIMITS`，恢复舵机 ABS limit。
5. 逐个下发 `CMD_CALIB_LOAD_JOINT`，恢复每个成功关节的标定结果。
6. 每帧等待下位机 ACK。
7. 任一帧失败则显示失败原因。

## 7. 下位机恢复逻辑

相关代码：

```text
ESP32-P4/ServoBoardMain/src/communication/upper/UpperCommProtocol.h
ESP32-P4/ServoBoardMain/src/communication/upper/UpperCommTask.cpp
ESP32-P4/ServoBoardMain/src/calibration/CalibrationTask.h
```

新增上行包：

```text
PACKET_TYPE_JOINT_CALIB_SNAPSHOT = 0x0E
```

作用：下位机在 `CALIB SUCCESS` 后周期性上报已成功关节的 `JointCalibrationResult`。

新增下行命令：

```text
CMD_CALIB_LOAD_JOINT  = 0xDF
CMD_CALIB_LOAD_LIMITS = 0xE0
```

`CMD_CALIB_LOAD_LIMITS` 恢复：

- `motor_abs_limit_min[]`
- `motor_abs_limit_max[]`
- `motor_abs_limit_valid_bitmap`
- `motor_abs_limit_locked_bitmap`

`CMD_CALIB_LOAD_JOINT` 恢复：

- `g_jointCalibResult[jointIndex]`
- `g_encoderOffsetManual[jointIndex]`
- `g_encoderDirection[jointIndex]`

加载成功后，下位机会设置：

```text
g_calibrationStatus = CALIB_STATUS_SUCCESS
```

并清除 calibration fault。随后状态机会重新按恢复后的标定结果判断能否进入 `READY`。

## 8. 为什么要分帧

当前串口协议帧格式：

```text
[0xFE][LEN][CMD/PACKET][PAYLOAD][0xFF]
```

`LEN` 是 1 字节，所以单帧 payload 不能太大。完整保存 21 个关节的 `JointCalibrationResult` 会超过单帧容量，因此采用：

- ABS limit 一帧恢复。
- 关节标定结果一关节一帧恢复。

这样不需要重构底层串口帧格式，风险更低。

## 9. 后续开发注意事项

- 修改关节/舵机/磁编映射后，应重新 `Calibrate`，不要复用旧快照。
- 如果未来支持多套手型，应把更明确的 mapping version 写入 JSON 并参与校验。
- 如果未来把标定结果写入下位机 NVS，`Calib Load` 可以作为备用恢复路径保留。
- 如果要扩展快照字段，必须同步修改：
  - 上位机 `parse_joint_calib_snapshot_packet()`
  - 上位机 `build_calib_load_joint_cmd()`
  - 下位机 `sendJointCalibSnapshotPacket()`
  - 下位机 `CMD_CALIB_LOAD_JOINT` 解析逻辑
- 不建议在机械状态不确定时自动加载快照。当前设计保留中文确认框，是为了避免误恢复导致机械冲突。

## 10. 推荐测试流程

1. 单关节标定。
2. 等待上位机保存 `calib_load_snapshot.json`。
3. 关闭下位机再重启。
4. 打开上位机或保持上位机连接。
5. 点击 `Calib Load`。
6. 确认状态栏显示加载成功。
7. 检查对应关节能进入 `READY`。
8. 执行小幅度 `Open Pose` 或单关节目标，确认反馈正常。

