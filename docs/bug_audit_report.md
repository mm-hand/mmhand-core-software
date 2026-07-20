# 历史 BUG 审计报告（参考恢复版）

日期：2026-07-06  
状态：参考文档，不作为当前主链路 BUG 结论直接使用。

> 说明：本文件曾被删除，后续发现仍有参考价值，因此恢复一版。由于原文件已被本地删除且未在当前工作区找到副本，本文不是原文逐字恢复，而是基于此前审计条目、终端检索结果和后续逐项核验重新整理的“参考恢复版”。
>
> 使用原则：所有条目必须重新对照当前主链路代码确认。当前 P4 主固件以 `ESP32-P4/ServoBoardMain` 为准；旧 `ServoManager` 目录已删除，不应再作为现行控制链路依据。

---

## 0. 当前主链路入口

当前 P4 主固件入口：

- `ESP32-P4/ServoBoardMain/ServoBoardMain.ino`
- `ESP32-P4/ServoBoardMain/src/system/SystemTask.cpp`
- `ESP32-P4/ServoBoardMain/src/communication/servo/ServoCommunicationTask.cpp`
- `ESP32-P4/ServoBoardMain/src/control/ControlTask.cpp`
- `ESP32-P4/ServoBoardMain/src/calibration/AutoCalibrationTask.cpp`

当前 S3 主固件入口：

- `ESP32-S3/System/System.ino`
- `ESP32-S3/System/src/tasks/EncodersTask.cpp`
- `ESP32-S3/System/src/tasks/TactileTask.cpp`
- `ESP32-S3/System/src/tasks/CanTask.cpp`

当前上位机入口：

- `desktop/__main__.py`
- `desktop/app/entry.py`
- `desktop/app/ui_runner.py`
- `desktop/app/cli_monitor.py`

当前 ROS2 21 角链路：

- `desktop/ros2/joint_targets.py`
- `desktop/ros2/raw_ik_target.py`

---

## 1. 旧审计条目的当前状态

| 编号 | 旧结论 | 当前状态 | 备注 |
|---|---|---|---|
| C01 | `EMERGENCY_LOAD = 5000`，过载保护永远不触发 | 当前主链路不成立 | 旧 `ServoManager` 已删除；当前 `ControlTask.cpp` 和 `AutoCalibrationTask.cpp` 使用 800 |
| C02 | 多圈跨圈阈值 2048 应改 3072 | 当前主链路不成立 | 当前 `ServoCommunicationTask.cpp` 中 2048 是 near-anchor unwrap 半圈判定，不是旧裸 `turns++/--` |
| C03 | PID 积分不重置导致暂停恢复跳变 | 原描述不成立，但有相邻隐患 | 当前 Ki=0，积分每帧清零；但状态切换时 D 项历史/Output 仍建议显式 reset state |
| H11 | 双电机副舵机热插拔后 PID 微分使用陈旧状态 | 需要复核 | 当前 `DualTendonControl` 有 commandToken reset，但副电机热插拔是否触发 reset 仍值得测 |
| M31 | CAN bus-off 恢复后旧队列/旧帧问题 | 需要复核 | 当前 S3/P4 CAN 都有 CRC/超时机制，但仍应看 bus recovery 后是否清理 pending 状态 |

---

## 2. 已确认不应按旧结论修改的条目

### C01：`EMERGENCY_LOAD = 5000` 死阈值

旧描述：

```cpp
#define EMERGENCY_LOAD 5000
if (abs(raw_load) > EMERGENCY_LOAD) {
    EmergencyStop();
}
```

旧问题确实存在于历史 `ServoManager` 代码里，但该目录已经删除，且当前 `ServoBoardMain` 没有走这套链路。

当前主链路：

- `ESP32-P4/ServoBoardMain/src/control/ControlTask.cpp`
  - `kControlOverloadLoadThreshold = 800`
  - `kJointControlOverloadLoadThreshold = 800`
  - 连续 `20` 个周期高负载后 latch fault。

- `ESP32-P4/ServoBoardMain/src/calibration/AutoCalibrationTask.cpp`
  - `kAutoCalibrationOverloadLoadThreshold = 800`

当前判断：

- 当前主链路不是死阈值。
- 不需要再按 C01 修改当前主固件。
- 仍应通过实测负载曲线确认 800 是否合适。

---

### C02：多圈跨圈阈值 2048 应改 3072

旧描述：

```cpp
if (diff < -2048) {
    turns++;
} else if (diff > 2048) {
    turns--;
}
absolutePos = turns * 4096 + newRawPos;
```

旧结论的问题：

- 该逻辑来自历史 `ServoManager`，当前已删除。
- 当前主链路不再用该裸 `diff -> turns++/--` 的实现。
- “4095 正常移动到 0 不应跨圈”这个说法本身不严谨。对于连续多圈位置展开，4095 到 0 本来就是一次跨过单圈边界。

当前主链路：

`ServoCommunicationTask.cpp` 使用 near-anchor 展开：

```cpp
while ((candidate - anchor) > 2048) {
    candidate -= 4096;
}
while ((anchor - candidate) > 2048) {
    candidate += 4096;
}
```

当前判断：

- 这里的 2048 是半圈距离，用于选择离当前 anchor 最近的等价位置。
- 不建议把当前主链路里的 2048 改成 3072。
- 如果要增强鲁棒性，应从反馈频率、速度上限、NVS 恢复容差、掉电边界保存这几个方向处理。

---

### C03：PID 积分不重置

旧描述：

`ControlSolver::compute()` 中 PID 积分项在 `PAUSE/IDLE -> RUN_FEEDBACK` 时不重置，恢复第一帧可能带旧积分。

当前主链路事实：

`ControlSolver.cpp` 默认 PID 参数：

```cpp
{5.7f, 0.0f, 0.0f, 0.0f, 100.0f, 600.0f}
```

当前单环 PID 的 `Ki` 是 `0.0f`。`pid.c` 中当 `ki == 0` 时会强制：

```cpp
Pid->Integral = 0;
```

当前判断：

- “积分项不重置导致跳变”按当前参数不成立。
- 但状态切换时没有显式重置 PID 状态，D 项历史误差、上一帧 Output 仍可能影响第一帧。

建议后续修改：

- 新增 `ControlSolver::resetState()`，只清运行态：
  - `Err`
  - `Integral`
  - `Pout/Iout/Dout`
  - `Output`
- 不要用当前 `resetAll()` 直接做状态切换 reset，因为它会 `memset(_pids, 0, sizeof(_pids))`，把 PID 参数也清掉。
- 在以下边界调用：
  - 进入 `RUN_FEEDBACK`
  - `control_enabled` 从 0 到 1
  - active joint mask 变化
  - 标定完成后第一次进入闭环

---

## 3. 仍然值得复核的高风险条目

### H01：上位机到 P4 命令帧缺少 CRC/seq

当前风险：

- 当前上位机下行帧主要靠 header、length、tail。
- `ANGLE_CTRL`、`MOTOR_POS_ABS`、`CALIB_LOAD_JOINT`、`RESTORE_MULTITURN` 都是关键命令。
- 串口错帧、丢字节、payload 位翻转时，诊断能力不足。

建议：

- 新增协议 V2：
  - version
  - command
  - seq
  - payload length
  - payload
  - CRC16/CRC32
- 保留 V1 兼容期。
- 高风险命令必须 ACK 返回 seq 和 reject reason。

---

### H02：ROS2 21 角输入缺少速率/范围/超时保护

当前风险：

- `/mmhand/joint_targets` 和 `/raw_ik_target` 主要检查 21 个 float 和 finite。
- 高频发布或角度突变会直接进入 `backend.send_joint_targets(... live=True)`。
- 没有 staleness watchdog 时，外部节点停止发布后，系统可能保持最后目标。

建议：

- 增加 range limit。
- 增加 slew rate limit。
- 增加 topic hz 统计。
- 增加 stale timeout。
- 日志记录被 clamp 的 joint index。

---

### H03：双电机防松绳 clamp 仍未完全来自标定 anchor

当前事实：

- `DualTendonControl.cpp` 已有 `450 counts` release clamp。
- J20 已单独放宽到 `600 counts`。
- 但 `motorZero = 2557 / 358` 仍是静态值，且所有双电机关节共用同一几何模型。

风险：

- 当前参数可能在实测中有效，但并不是严格的“每关节自动标定闭环参数”。
- 如果不同双关节机械绕绳差异明显，静态值可能留下边界风险。

建议：

- 自动标定输出 per-joint/per-motor preload anchor。
- 双电机 clamp 使用每个关节自己的 anchor。
- 标定快照协议升级到 int32 ABS，避免多圈值被 int16 截断。

---

### H04：双电机热插拔/掉线恢复后的控制状态重置

风险点：

- 双电机控制器内部保存：
  - integral
  - prevFbModel
  - lastToken
  - prevValid
- 如果副电机掉线、恢复，但 command token 未变，控制器是否会完全重置需要实测确认。

建议：

- 当任一双电机关节 actuator offline -> online，调用 `resetDualTendonController(jointIndex)`。
- 记录热插拔事件：
  - joint index
  - primary/secondary motor
  - previous online
  - current online
  - reset reason

---

### H05：P4 舵机反馈年龄未显式上报

当前风险：

- ControlTask 10 ms。
- ServoCommunicationTask 分总线刷新，所有总线完整反馈可能约 20 ms。
- 如果某个 bus 延迟或掉线，控制层只看到最新缓存，不容易判断反馈年龄。

建议：

- 每个 motor 增加 `feedback_age_ms`。
- UI 显示过期反馈。
- 负反馈控制使用过期反馈时进入 hold 或降级。

---

### H06：S3 编码器/触觉/CAN 实时性竞争

当前风险：

- S3 编码器任务约 2 ms。
- CAN 任务约 4 ms。
- 触觉任务约 33 ms。
- 触觉和 CAN 同 core0、同优先级。
- 编码器 debug dump 曾默认开启，会影响实时性。

建议：

- 默认关闭高频 debug。
- 增加任务耗时统计。
- Kingst 测：
  - encoder SPI 周期。
  - tactile SPI 一轮扫描耗时。
  - CAN 0x100-0x106 周期。
  - 触觉发送是否影响编码器 CAN。

---

### H07：S3 引脚配置命名/模式容易误读

风险点：

- 配置中曾出现 encoder/tactile 相关 GPIO 命名容易混淆。
- 某些引脚在不同硬件模式下含义不同。

建议：

- 按硬件模式拆分配置。
- 增加编译期冲突检查。
- 用 Kingst 实测关键 CS/MUX 引脚。

---

## 4. 中低优先级参考条目

| 编号 | 问题 | 当前建议 |
|---|---|---|
| M01 | 文档引用历史路径导致审计误导 | 删除旧文档或在开头标注“历史参考” |
| M02 | `desktop/resources/config` 历史快照太多 | 当前快照和归档快照分目录管理 |
| M03 | `__pycache__` / build 产物污染搜索 | 保持 `.gitignore`，不要提交 Python 缓存 |
| M04 | Kingst 波形 demo sketch 与主固件并列 | 移到 tools 或保留 README 说明 |
| M05 | 控制参数硬编码较多 | 建立参数表和版本化配置 |
| M06 | 文档中存在旧版/新版比较但没有日期 | 所有文档加日期和适用分支 |

---

## 5. Kingst 实测建议

优先测 P4 舵机总线：

- 采样率：20 MSa/s 到 50 MSa/s。
- 解码：UART 1 Mbps，8N1。
- 观察：
  - TX 是否有长空窗。
  - TX 后 RX 是否返回。
  - 卡顿时 abs 是否不变。
  - 半双工方向控制是否正常。

再测 S3-P4 CAN：

- 采样率：10 MSa/s 到 20 MSa/s。
- 解码：CAN 1 Mbps。
- 观察：
  - `0x100-0x106` 是否约 4 ms 成组出现。
  - `0x106` CRC 帧是否稳定。
  - `0x240-0x246` 触觉摘要是否约 33 ms 出现。
  - 是否有 error frame / bus-off。

再测 S3 编码器 SPI：

- 采样率：50 MSa/s 到 100 MSa/s。
- 观察：
  - SPI 周期是否稳定。
  - MUX 切换到 SPI 开始是否有足够 settle。
  - MISO 是否毛刺、全 0、全 1。

最后测上位机串口/ROS2：

- 采样率：20 MSa/s 到 50 MSa/s。
- 观察：
  - ROS2 发布频率和串口真实发送频率是否一致。
  - 是否出现突发发送。
  - P4 是否回包。

---

## 6. 后续使用方式

这份文档只作为候选问题池。

每次引用其中某条 BUG 前，必须补充三项：

1. 当前文件路径。
2. 当前代码证据。
3. 当前实测或日志证据。

如果某条只存在于已删除旧代码，不能再作为当前 BUG 结论。
