# 下位机多圈圈数 NVS 保存与烧录注意事项

本文档说明当前 ESP32-P4 下位机如何保存飞特舵机多圈圈数、为什么需要逻辑坐标/硬件坐标分离、换舵机时如何关闭 NVS，以及 Arduino 烧录时需要注意的位置。

## 背景

飞特 ST-3215 多圈模式掉电后，舵机内部多圈圈数不会保存。再次上电时，舵机只能可靠提供当前单圈 raw 位置。
如果系统仍把上一次的多圈 ABS 直接写给舵机，舵机会按本次上电后的硬件坐标运动，可能多跑一整圈。

因此当前方案是：

1. 下位机在 NVS 中保存每个电机的 `turn + raw` 快照。
2. 下位机重新上电后，先读取当前舵机 raw。
3. 只有当前 raw 与 NVS 快照 raw 足够接近时，才恢复该电机的逻辑多圈 ABS。
4. 控制时对外继续使用逻辑 ABS，但真正写给舵机前转换成本次上电的硬件坐标。

## 坐标定义

当前代码里有两套坐标：

- 逻辑多圈 ABS：上位机、NVS、状态机、PID 控制层看到的位置。
- 硬件坐标：飞特舵机本次上电后自己认为的位置。

恢复成功后使用：

```text
logical_abs = hardware_abs + logical_offset
hardware_target = logical_target - logical_offset
```

例如：

```text
NVS 恢复逻辑位置 = 4309
舵机上电后当前 raw/hardware = 213
logical_offset = 4096

上位机发送 rel=200：
logical_target = 4509
真正写给舵机的 hardware_target = 4509 - 4096 = 413
```

这样物理上只会从 `213` 走到 `413`，也就是 `+200`，不会多跨一圈。

核心代码位置：

- `ESP32-P4/ServoBoardMain/src/communication/servo/ServoCommunicationTask.cpp`
  - `ServoFeedbackInternal::logicalOffset`
  - `restoreAbsolutePosition()`
  - `logicalToHardwareTarget()`
  - `setTarget()`

## NVS 保存内容

NVS 保存的是下位机侧的多圈安全快照：

```cpp
struct MultiTurnNvsSnapshot {
    uint32_t magic;
    uint16_t version;
    uint16_t motorCount;
    uint32_t sequence;
    uint32_t validMask;
    int16_t turns[SERVO_TOTAL_NUM];
    int16_t raw[SERVO_TOTAL_NUM];
    uint32_t crc32;
};
```

保存 `turn + raw` 的原因：

- `turn` 用于恢复多圈。
- `raw` 用于重新上电时和当前物理单圈位置做校验。
- CRC 和 A/B slot 用于避免写入过程中掉电导致坏快照被误用。

核心代码位置：

- `ESP32-P4/ServoBoardMain/src/persistence/MultiTurnNvsStore.h`
- `ESP32-P4/ServoBoardMain/src/persistence/MultiTurnNvsStore.cpp`

## 什么时候保存

下位机当前会在这些时机尝试保存 NVS 快照：

- 运行开始
- 运行结束
- 上位机心跳丢失，判断上位机离线
- 上位机恢复多圈成功后
- 自动标定完成后
- 上位机离线期间检测到运动
- 上位机离线期间接近跨圈边界

保存前会过滤不可信通道：

- `multi_turn_unknown_bitmap` 中仍为 unknown 的电机不会保存。
- 离线或无反馈的电机不会覆盖当前值。
- 本次未采样到的电机沿用旧快照，避免局部 active scope 把其它电机数据清掉。

相关代码：

- `ServoCommunicationTask.cpp`
  - `saveCurrentMultiTurnSnapshot()`
  - `processMultiTurnNvsSave()`
  - `collectTrustedMultiTurnSnapshot()`
- `StateMachineTask.cpp`
  - `requestMultiTurnNvsSave()`

## 上位机心跳与离线保存

上位机在线时每 500 ms 发送一次 heartbeat。
下位机 1000 ms 内没有收到合法上位机帧，就认为上位机离线，并触发一次 NVS 保存。

相关代码：

- 上位机：
  - `desktop/core/protocol.py`
  - `desktop/core/comm_layer.py`
- 下位机：
  - `ESP32-P4/ServoBoardMain/src/communication/upper/UpperCommTask.cpp`
  - `ESP32-P4/ServoBoardMain/src/communication/servo/ServoCommunicationTask.cpp`

## NVS ON / OFF 按钮

上位机 Device 区有 `Flash ON / Flash OFF` 按钮，用于控制多圈缓存是否写入下位机 flash，并同步启用或禁用上位机缓存恢复。

### NVS ON

- 允许下位机开机读取 NVS 快照。
- 允许后续运行中继续保存多圈快照。
- 不会主动恢复已经被清除的旧快照。

### NVS OFF

用于更换舵机、重新装配、或者不希望继承旧多圈数据的维护场景。

点击后下位机会：

1. 将 `enabled=0` 持久写入 NVS。
2. 清空旧的 `slot0/slot1` 多圈快照。
3. 禁止后续保存。
4. 下次上电跳过 NVS 恢复。
5. 在线舵机使用当前单圈 raw 作为新的 0 圈起点。

推荐换舵机流程：

1. 保持下位机和上位机在线。
2. 点击 `NVS OFF`。
3. 确认按钮显示 `NVS OFF`。
4. 断电并更换舵机。
5. 下位机重新上电，此时不会恢复旧圈数。
6. 确认单圈/多圈方向和控制正常后，再点击 `NVS ON`。
7. 后续重新运行或校准后生成新的 NVS 快照。

相关代码：

- 协议命令：`CMD_MULTITURN_NVS_CONTROL = 0xDD`
- 上位机：
  - `desktop/ui/ui_control_panels.py`
  - `desktop/core/core_logic.py`
  - `desktop/core/comm_layer.py`
  - `desktop/core/protocol.py`
- 下位机：
  - `UpperCommTask.cpp`
  - `MultiTurnNvsStore.cpp`

## 直控模式与负反馈模式是否都正确

当前逻辑坐标到硬件坐标的转换在舵机通信层统一完成：

```cpp
ServoBusDriver::setTarget()
```

直控模式路径：

```text
上位机 motor absolute/direct command
-> UpperCommCommandRouter::upperApplyMotorTargets()
-> ControlTask 组装 ServoTargetBatch
-> servoTargetQueue
-> ServoCommunicationTask::applyServoTargetBatch()
-> ServoBusDriver::setTarget()
-> logicalToHardwareTarget()
-> 飞特舵机
```

负反馈模式路径：

```text
上位机 joint target / run feedback
-> ControlTask
-> ControlSolver PID 输出 outPulses
-> appendServoTarget()
-> servoTargetQueue
-> ServoCommunicationTask::applyServoTargetBatch()
-> ServoBusDriver::setTarget()
-> logicalToHardwareTarget()
-> 飞特舵机
```

所以只要目标最终通过 `ServoTargetBatch` 进入 `ServoCommunicationTask::applyServoTargetBatch()`，直控和负反馈都会走同一个转换点。
目前代码是这样走的，因此这次修正不是只对直控生效，负反馈模式也会生效。

但实际硬件上仍建议再做一次负反馈验证：

1. 让某个电机处在 `turn=1` 附近并保存 NVS。
2. 先关上位机，再关下位机。
3. 重新上电并确认恢复成功，串口应出现类似：

```text
[SERVO RESTORE OFFSET] id=1 logical=4309 hardware=213 offset=4096 raw=213
```

4. 在 Joint Channel 进入负反馈模式。
5. 给对应 joint 一个小目标，例如产生约 100 到 200 pulse 的 PID 输出。
6. 观察 Motor Channel：
   - 逻辑 ABS 应连续变化。
   - raw 只变化对应的小步数。
   - 不应额外跨一整圈。

## Arduino 烧录注意事项

工程目录：

```text
ESP32-P4/ServoBoardMain
```

注意事项：

1. 使用 Arduino IDE 打开该工程目录下的主 `.ino` 工程。
2. 确认选择的是 ESP32-P4 对应开发板。
3. 你的板子是 32MB Flash，Arduino IDE 里 `Tools -> Flash Size` 要选择 32MB。
4. 当前仓库没有使用自定义 `partitions.csv`。不要为了 NVS 手动添加旧的 `partitions.csv`，否则可能导致烧录分区和实际板卡配置不一致。
5. 之前 `Preferences.h` 编译失败的问题已经处理，现在下位机 NVS 使用 ESP-IDF 原生接口：

```cpp
#include <nvs.h>
#include <nvs_flash.h>
```

6. 如果改过分区表或怀疑 NVS 里有旧脏数据，可以选择：
   - 优先使用上位机 `NVS OFF` 清空多圈快照。
   - 或者 Arduino IDE 开启 `Erase All Flash Before Sketch Upload` 后重新烧录。
7. 如果只是普通更新代码，不建议每次都擦全 Flash，因为会清掉 NVS 中的开关和快照。
8. 烧录后建议先看串口日志：

```text
[NVS MULTITURN] loaded seq=...
[NVS MULTITURN] no valid boot snapshot
[NVS MULTITURN] disabled by user, boot restore skipped
[SERVO RESTORE OFFSET] ...
```

这些日志可以判断当前到底是恢复了快照、没有快照，还是 NVS 被用户关闭。

## 风险边界

NVS 只能保存“下位机断电前已经知道的位置”。以下情况仍然不安全：

- 舵机断电后被外力转动。
- 更换舵机但没有先 `NVS OFF` 清快照。
- 当前 raw 与保存 raw 差距过大，此时恢复会被拒绝并保持 unknown。
- NVS 写入过程中电源反复抖动，可能只保留上一个有效 slot。

因此硬件更换、拆装、手动转动后，建议先 `NVS OFF`，重新确认机械位置和控制方向，再 `NVS ON` 生成新的快照。
