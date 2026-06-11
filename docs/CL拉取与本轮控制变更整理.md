# CL 拉取逻辑与本地当前逻辑差异说明

本文重点说明：从 CL 侧拉取后带入了哪些行为，本地调试中发现了哪些不适配点，以及当前本地代码最终保留/修改成了什么逻辑。

结论先放前面：本地没有完全照搬 CL 的保护策略。CL 侧更偏向“保护条件命中即 hold/阻断”；本地当前更偏向“逐 motor 限幅 + 持续异常才 fault”，避免双绳拮抗关节因为一侧到边界而把另一侧也停掉。

## 1. Release / Tendon Guard

| 项目 | CL 拉取逻辑 | 本地当前逻辑 |
| --- | --- | --- |
| 目标角小于当前角 | 视为 release 方向，进入 release guard 判断 | 仍识别 release 方向，但默认不直接 emergency |
| tendon guard 到边界 | 可参与 hold / emergency | 只作为 warning，不直接阻断目标 |
| 上位机 tendon guard | 可能改写目标角，阻止继续 release | 不再改写目标角，只提示 boundary warning |
| release fault | 舵机 release 一段距离或 tendon guard 命中后可能 hold | 只有持续 release 后关节角没有按预期回退，才认为松绳风险 |

本地这样改的原因：

- 双绳拮抗关节回退时，本来就需要一侧放绳、另一侧收绳。
- 如果“目标角变小”立刻按 release 风险处理，会把正常回放/减小角度误判成松绳。
- 真正危险的不是“正在放绳”，而是“电机在放绳但关节角不回退”。

相关代码：

- `ESP32-P4/ServoBoardMain/src/control/ControlSafety.cpp`
- `ESP32-P4/ServoBoardMain/src/control/ControlTask.cpp`
- `desktop/ui/ui_tendon_guard.py`

## 2. 双绳 ABS Limit

| 项目 | CL/旧逻辑 | 本地当前逻辑 |
| --- | --- | --- |
| 双绳 limit 来源 | 沿用 tight 点和固定窗口 | 双绳专用 release limit |
| 旧窗口 | `tightAbs` 到 `tightAbs - 4096*5` | 不再用于双绳关节 |
| primary motor | 旧窗口 clamp | `primaryReleaseAbs .. 30719` |
| secondary motor | 旧窗口 clamp | `secondaryReleaseAbs - 1850 .. 30719` |
| 写入时机 | 标定流程内写入 | 标定成功结束后写入 |
| 再次标定 | 可能受旧 limit 影响 | 标定开始先清除该关节旧 limit |

以 J00 为例：

```text
M00: 标定完成时 M00 ABS .. 30719
M01: 标定完成时 M01 ABS - 1850 .. 30719
```

本地这样改的原因：

- 用户期望 M01 可以继续放绳一段空间，而不是标定完成后马上被当前位置卡死。
- 双绳关节只需要限制过度放绳方向；收绳方向由协议上界和 overload 保护约束。
- limit 不能在标定过程中生效，否则二次标定会被上次 limit 卡住。

相关代码：

- `ESP32-P4/ServoBoardMain/src/calibration/AutoCalibrationTask.cpp`
- `ESP32-P4/ServoBoardMain/src/control/ControlOutputBuilder.cpp`

## 3. ABS Limit 的运行语义

| 项目 | 容易误解的行为 | 本地当前行为 |
| --- | --- | --- |
| 某一侧 motor 超出 limit | 整个 RUN_FEEDBACK 被阻断或弹窗打断 | 只钳该 motor 的 target |
| 另一侧 active motor | 可能被一起影响 | 继续正常输出 |
| 上位机反馈越界提示 | 原来弹出 blocking message box | 改为状态栏提示 |

当前固件输出路径都是逐 motor clamp：

```text
targetAbs -> clampServoPosByMotorAbsLimit(motorChannel) -> append target
```

因此双绳中 M01 到放绳下界时，只会钳 M01；M00 仍可继续按控制器输出。

上位机提示改为：

```text
Motor ABS limited: ...
Feedback ABS clamped by firmware: ...
```

相关代码：

- `ESP32-P4/ServoBoardMain/src/control/ControlTask.cpp`
- `ESP32-P4/ServoBoardMain/src/control/ControlOutputBuilder.cpp`
- `desktop/ui/ui_control_panels.py`

## 4. 双绳 PID

| 项目 | CL/旧认知 | 本地当前逻辑 |
| --- | --- | --- |
| 控制域 | 容易被理解成关节角 PID | 电机域 PID |
| PID 输入误差 | 不是直接 `qRef - qFb` | `motorErr = motorRef - motorFbModel` |
| Kp | `1.0` | `1.25` |
| Ki | `0.0` | `0.0` |
| Kd | `0.03` | `0.03` |
| correction 输出限幅 | 曾加过 `+/-400 count` | 当前暂时取消 |

当前链路：

```text
qRef/qFb
-> tendon length model
-> motorRef / motorFbModel
-> motorErr = motorRef - motorFbModel
-> correction = Kp * motorErr + Ki * integral - Kd * dFb
-> targetAbs = motorCurrent + correction
```

注意：取消的是 PID correction 的额外输出限幅。最终命令仍会经过：

- 舵机协议范围 `-30719 .. 30719`
- 每个 motor 自己的 ABS limit

相关代码：

- `ESP32-P4/ServoBoardMain/src/control/DualTendonControl.cpp`

## 5. Overload

| 项目 | 之前运行阈值 | 本地当前阈值 |
| --- | ---: | ---: |
| RUN_DIRECT | `700` | `600` |
| RUN_FEEDBACK | `800` | `600` |
| latch 周期 | 20 个控制周期 | 20 个控制周期 |
| 控制周期 | 10ms | 10ms |

当前含义：

```text
load >= 600 持续约 200ms -> latch overload
```

相关代码：

- `ESP32-P4/ServoBoardMain/src/control/ControlTask.cpp`

## 6. RUN_FEEDBACK 退出条件

| 项目 | CL/旧行为 | 本地当前行为 |
| --- | --- | --- |
| 停止发送手套/测试目标 | 可能继续停留在 RUN_FEEDBACK | 500ms 无新 joint command 自动退出 |
| 本地 sine test | 可能被误退出 | sine test active 时不按输入超时退出 |

当前规则：

```text
进入 RUN_FEEDBACK 后给 500ms 宽限
joint_command_token 更新 -> 刷新输入时间
500ms 无更新且 sine test 未运行 -> post SYSTEM_EVENT_RUN_FEEDBACK_DONE
```

相关代码：

- `ESP32-P4/ServoBoardMain/src/control/ControlTask.cpp`

## 7. 急停/断电后的队列与多圈状态

| 项目 | 旧问题 | 本地当前逻辑 |
| --- | --- | --- |
| STOP/RESET/fault 后旧 target | 可能残留在队列 | 清运行队列和旧 target |
| START/RUN 时清队列 | 曾经清得过多导致不能 enable | 只清命令/输出队列，不清反馈快照 |
| 舵机掉电重连 | 可能复用上一电源会话 logicalOffset | 重连时清旧 offset/reliable/initialized |
| online mask 变化 | 旧 target 可能继续发送 | online 变化时清 servoTargetQueue |

本地区分两类清理：

```text
STOP / RESET / fault hold / CALIBRATE / calibration failed:
  清运行时队列和旧 target

START / RUN_DIRECT / RUN_FEEDBACK:
  只清命令和输出队列，不清 feedback/mapped 快照
```

相关代码：

- `ESP32-P4/ServoBoardMain/src/system/StateMachineTask.cpp`
- `ESP32-P4/ServoBoardMain/src/communication/servo/ServoCommunicationTask.cpp`

## 8. 上位机 UI 行为差异

| 项目 | 之前 | 本地当前 |
| --- | --- | --- |
| feedback ABS 越界 | 弹 blocking warning | 状态栏提示 |
| direct motor target 超 limit | 弹 blocking warning | 状态栏提示 |
| tendon guard release 边界 | 改写/阻断 target | 不改写，只提示 |

目的：

- UI 不再因为某一路 motor 到边界而打断整个调试过程。
- 真正的限制由固件逐 motor clamp 执行。
- UI 只负责提示当前哪一路被 clamp 或接近边界。

相关代码：

- `desktop/ui/ui_control_panels.py`
- `desktop/ui/ui_tendon_guard.py`

## 9. 需要烧录和重新标定的内容

需要重新烧录固件：

- 双绳 ABS limit 规则和 secondary release count `1850`
- 双绳 PID `Kp=1.25`
- 取消双绳 PID correction `+/-400 count` 输出限幅
- overload 阈值 `600`
- RUN_FEEDBACK 输入超时退出
- 急停/断电后的队列和多圈状态清理

需要重新标定才会更新运行时数据：

- 双绳 motor ABS limit 的 min/max

## 10. 当前代码取舍总结

本地当前取舍可以概括为：

```text
CL:    更偏硬保护，边界/guard 命中容易进入 hold
本地:  边界先 warning，输出逐 motor clamp，持续异常才 fault
```

这样更适合双绳拮抗关节调试：一侧达到放绳边界时，不会把另一侧的主动收绳/回退控制一起停掉。
