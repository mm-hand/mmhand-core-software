# 安全规则精简与统一用户提示：实际变更清单（2026-10-27）

本次代码中的有效 P4 安全规则由 103 条减至 **73 条**：删除 28 条已停用规则，合并 2 条重复报告。编号不重新排序，已删除编号永久保留。上位机为全部 73 条有效规则提供英文标题、影响、最多三步操作建议、影响范围、严重程度、状态和原始 Details。

## 删除的 28 条规则

下表条目从固件 `.def`、生成枚举、上位机有效规则表及有效调用索引中移除。它们不再参与当前规则闭环；历史解释单独保存在 `desktop/resources/config/safety_rules.archive.json`，不把历史故障当成当前故障。

| 编号 | 原含义 |
| --- | --- |
| ACT-1011 | Target exceeds the logical range |
| SYS-3005 | Current state rejected automatic calibration |
| FBK-3205 | Feedback control is missing valid ABS or LUT limits |
| FLT-3504 | Automatic calibration failure |
| CAL-4001 | Calibration active scope is empty |
| CAL-4005 | Encoder error during calibration |
| CAL-4006 | Servo offline during calibration |
| CAL-4008 | Calibration load is too high |
| CAL-4013 | Invalid encoder wrap during calibration |
| CAL-4014 | Calibration encoder wrap span is abnormal |
| CAL-4015 | Calibration ABS limit is invalid |
| CAL-4016 | Calibration target publish failed |
| CAL-4017 | Feedback lost during calibration |
| CAL-4018 | Automatic calibration task creation failed |
| CAL-4020 | Calibration lost output ownership |
| CAL-4021 | Failed to hold non-active joints |
| CAL-4022 | Calibration joint list is invalid |
| CAL-4023 | Calibration joint list contains duplicates |
| CAL-4024 | Dual-tendon workspace preparation failed |
| CAL-4025 | Joint calibration step failed |
| CAL-4026 | Timed out waiting for servo recovery during calibration |
| CAL-4027 | Single tendon released more than two turns |
| CAL-4028 | Calibration motor mapping is invalid |
| CAL-4029 | Calibration result travel is invalid |
| CAL-4030 | Calibration result build failed |
| CAL-4031 | Calibration result commit failed |
| CAL-4032 | Calibration multi-turn trust commit failed |
| CAL-4034 | Joint stalled during dual-tendon exchange |

24 条 CAL 编号属于已从 P4 移除的旧自动标定流程。**现在运行在上位机的自动标定检查和测试仍保留**，包括编码器/舵机离线、过载、两圈放绳限制、行程、LUT 拟合和写回确认。

删除专门断言旧 CAL 规则仍驻留有效表/被特殊判退役的两个测试：`test_retired_calibration_ids_keep_history_without_runtime_bindings`、`test_closure_rejects_resurrected_retired_runtime_mapping`。保留通用历史解码与禁止复用编号的约束测试，不再测试这些旧规则的运行行为。

## 合并的 2 条规则

| 删除的重复报告 | 保留报告 | 实际代码改动 |
| --- | --- | --- |
| SYS-3003 | DIR-3101 | 删除 `SystemEventDispatcher.cpp` 同一拒绝分支中的通用报告；专用拒绝及其证据继续保留。 |
| SYS-3004 | FBK-3202 | 删除 `SystemEventDispatcher.cpp` 同一拒绝分支中的通用报告；专用拒绝及其证据继续保留。 |

这两处是同一失败分支重复上报两条事件，并非删除第二道独立安全检查。

## 保留规则的行为与解释调整

| 规则 | 实际变化 |
| --- | --- |
| ACT-1004 | 说明改为当前配置的 BUS 容量，不再写死错误的数量；容量判据不变。 |
| ACT-1005 | 当前停止锁存时，F1 电机位置重设零在收包入口明确拒绝，返回 ACK status=9 `EMERGENCY_STOP_LATCHED`，不新建事务、不改请求令牌；执行前门禁仍保留。提示先检查机构，再 Clear Faults，再重试原操作。 |
| CTL-2006 | 校准边界、加载边界与运行控制使用同一组已应用的 release/tight 余量，默认 500/1000。说明不再把单腱或固定余量写死。 |
| RDY-3301 | 修正解释：此分支实际检查“是否选择关节反馈模式”，不是 CAN 线故障。 |
| RDY-3303 | 空关节选择先报告空范围，避免被较早的 Manual 门禁笼统归为缺少标定；仍拒绝进入控制。 |
| RDY-3308 / ROP-3406 | 共用 Manual/LUT 版本、URDF CRC、Manual 条目 CRC 一致性判据；保留各自的调用时机和报告。 |
| RDY-3309 / RDY-3310 / FLT-3501 / FLT-3503 | 同一次 READY 评估复用同一份健康快照，不再分别重复取 CAN 邮箱并计算系统故障；1000ms CAN、500ms 舵机反馈门槛不变。 |
| CAW-4101 / CAW-4102 / CAW-4103 | 装配提示信任 P4 警告位；移除上位机 motor_quality 对 350/300 的再次判定。原始测量仍供 Details 查看。Manual 录入时自身的行程校验保留。 |
| 全部 73 条 | 增加统一的通俗英文解释；只凭历史事件不能推断故障仍存在或已经恢复。 |

`ServoRejectReason::LogicalRange` 已删除，数值 11 留空；`HardwareRange=12`、`TurnOverflow=13` 不前移。删除三个无使用常量 `CALIB_LOAD_THRESHOLD`、`ERROR_VAL_FLAG`、`kMotorAbsWindowCounts`。

## 900 和 1000 到底变了什么

它们是电机 ABS **位置计数**的收紧侧外延余量，不是负载阈值。以前上位机标定/P4 加载边界用了 900，运行 PID 默认用了 1000，导致同一 LUT 对应两套边界。现在全部从已应用的控制参数读取，默认 release=500、tight=1000；已有自定义值按实际成功下发的值使用。

1000 比 900 多允许 100 counts 的收紧侧外延，约为电机轴 8.79°（4096 counts/圈），不是关节角 8.79°。没有改变 ±30719 的舵机硬件边界、±30715 越圈触发线或过载保护。

具体实现：

- 校准开始时冻结已确认应用的余量；参数尚未应用时阻止开始校准/加载，校准准备及执行期间拒绝修改/恢复参数。
- 单腱和双腱标定都使用该冻结值。
- P4 加载 LUT、加载 ABS 限位、修改控制参数时，按 LUT 和当前参数重建对应电机范围。旧文件里 900 的派生限位不会覆盖新的范围。
- 双腱的两个电机先完整计算后一起发布，计算失败不部分发布；无效重建撤销旧限位有效位。
- 参数与边界均采用已有命令，没有新增命令或 telemetry 类型。ACK 仅增加状态值 9，包结构不变。
- Python BOUNDS/RAMP_BOUNDS 改由共享 schema 生成，字段顺序不变。渐进换向负载上界统一为 1000，上位机直接指出 1001–1023 不合法，不等 P4 拒收。

## 为什么仍保留看起来相似的检查

上位机的输入检查、P4 收包检查、状态机授权和舵机实际发送检查发生在不同时间，之间可能出现 STOP、失联、切换所有者或坐标变化。它们继续保留。总线是否可控、系统是否应进入故障、是否满足 READY 的业务含义也不同；本次只共用计算，不混用这些动作。

未修改 80ms 舵机陈旧门限、同步读时序、UART 重启策略或自动重扫行为。本次安全整理不能证明已经修复最初的 P4 舵机通信故障。

## 上位机现在如何展示

- 每个页面上方都有共用安全提示区；Diagnostics 展示同一组 UserIssue，失败弹窗和校准弹窗使用同一份解释。
- **Current issue** 只来自当前新鲜状态/明确的本机停止门禁；**Last action failed** 表示某次操作结果；**History** 表示发生过的设备事件。
- 相同原因、相同状态的问题合并展示影响范围和次数，Details 保留所有原始事件、编号和数值；不同恢复方法的问题分开。
- 超时未确认、部分完成、明确拒绝和用户取消分别说明，不自动追加“Please try again”，不自动重设零或恢复运动。
- 原始失败原因仍保存在操作历史和会话日志，新 `[USER_ISSUE]` 条目记录用户提示；不恢复独立校准日志目录。

源文件：`desktop/core/user_issues.py`、`desktop/resources/config/safety_guidance.json`、`desktop/ui/web/safety_status.py`。规则编译目录只保留有效规则；生成器检查所有有效规则均有提示，历史编号不得复用。固件源码指纹也纳入 `.def` 源文件。

## 73 条有效规则逐条清单

下列所有条目均接入统一提示；除上文明确列出的变化外，原有判据和保护动作保留。

| 规则 | 用户看到的标题 | 下一步操作 |
| --- | --- | --- |
| ACT-1001 | The motor command could not be used | 1. Stop using motion controls and open Details. 2. When the hand is safe, restart the controller and reconnect. 3. If it happens again, save the session log for support. |
| ACT-1002 | Motor locations are not known yet | 1. Check motor power and the affected bus connections. 2. Wait for the affected motors to show Online, then retry the original action. |
| ACT-1003 | Two motors are using the same ID | 1. Open Renumber Motor and inspect the reported bus and ID. 2. Assign unique IDs using the existing maintenance procedure, then scan again. |
| ACT-1004 | Too many motors are connected to one bus | 1. Open Details to check the affected bus and allowed count. 2. Move extra motors to another bus, then scan again. |
| ACT-1005 | Stop is still active | 1. Check that the hand is clear of obstacles. 2. Click Clear Faults. 3. Retry the original action. |
| ACT-1006 | Another task controls the motors | 1. Wait for the current action or mode change to finish. 2. If the app remains busy, click Stop, then Clear Faults before enabling again. |
| ACT-1007 | This action is not available in the current mode | 1. Wait for the current action or mode change to finish. 2. If the app remains busy, click Stop, then Clear Faults before enabling again. |
| ACT-1008 | Motor feedback is unavailable | 1. Check motor power and the affected bus connections. 2. Wait for the affected motors to show Online, then retry the original action. |
| ACT-1009 | Motor positions need to be recovered | 1. Wait for position recovery to finish. 2. If recovery fails, open Motor Calibration and calibrate the affected joints. |
| ACT-1010 | The motor command arrived too late | 1. Wait for the current action or mode change to finish. 2. If the app remains busy, click Stop, then Clear Faults before enabling again. |
| ACT-1012 | The requested motor position is outside the hardware limit | 1. Stop motion and check the affected motor position in Details. 2. Check the saved motor position and calibration before sending another target. |
| ACT-1013 | A motor has reached its position safety limit | 1. Check that the affected joint and tendons can move freely. 2. If Stop is active, click Clear Faults after checking the hand. 3. Open Motor Calibration, select the affected joints, and calibrate them again. |
| CTL-2001 | Joint feedback is unavailable | 1. Check the S3 sensor board and its power and CAN connections. 2. Wait for current joint readings to return before enabling control. |
| CTL-2002 | The controller could not identify a joint | 1. Stop using motion controls and open Details. 2. When the hand is safe, restart the controller and reconnect. 3. If it happens again, save the session log for support. |
| CTL-2003 | A joint position sensor is not working reliably | 1. Check the S3 sensor board and its power and CAN connections. 2. Wait for current joint readings to return before enabling control. |
| CTL-2004 | A joint angle could not be calculated | 1. Open Joint Calibration and check the selected joints' Manual Min and Max. 2. Open Motor Calibration and calibrate the affected joints. |
| CTL-2005 | Joint movement has been paused | 1. Check whether the hand is touching an object or a joint is blocked. 2. When it is safe, send a target in the reverse direction to release the existing hold. |
| CTL-2006 | A motor target has reached its configured limit | 1. Check the requested target angle and the affected joint. 2. If ordinary targets keep reaching this limit, check calibration before continuing. |
| CTL-2007 | A requested angle was limited to the allowed range | 1. Check the requested target angle and the affected joint. 2. If ordinary targets keep reaching this limit, check calibration before continuing. |
| CTL-2008 | A joint moved beyond its calibrated warning boundary | 1. Check the requested target angle and the affected joint. 2. If ordinary targets keep reaching this limit, check calibration before continuing. |
| CTL-2009 | A reconnected motor is being held in position | 1. Check that position recovery completed and the hand is clear. 2. Send a new target using the existing control. |
| SYS-3001 | The controller lost contact with the app | 1. Check the app and USB connection, then reconnect. 2. Check the hand and clear any remaining fault before enabling again. |
| SYS-3002 | The controller cannot start this action yet | 1. Wait for the current action or mode change to finish. 2. If the app remains busy, click Stop, then Clear Faults before enabling again. |
| DIR-3101 | Direct Control cannot start in the current state | 1. Wait for the current action or mode change to finish. 2. If the app remains busy, click Stop, then Clear Faults before enabling again. |
| DIR-3102 | Selected motors are not ready | 1. Check motor power and the affected bus connections. 2. Wait for the affected motors to show Online, then retry the original action. |
| DIR-3103 | Selected motor positions need to be recovered | 1. Wait for position recovery to finish. 2. If recovery fails, open Motor Calibration and calibrate the affected joints. |
| DIR-3104 | The controller is busy handling earlier requests | 1. Wait for the current action or mode change to finish. 2. If the app remains busy, click Stop, then Clear Faults before enabling again. |
| DIR-3105 | This Direct Control request is no longer current | 1. Wait for the current action or mode change to finish. 2. If the app remains busy, click Stop, then Clear Faults before enabling again. |
| DIR-3106 | A device fault is blocking Direct Control | 1. Open Diagnostics and resolve the current fault shown for the affected device. 2. After correcting the cause, click Clear Faults before enabling control again. |
| FBK-3201 | Selected joints need Manual Min and Max values | 1. Open Joint Calibration and check the selected joints' Manual Min and Max. 2. Open Motor Calibration and calibrate the affected joints. |
| FBK-3202 | Feedback Control cannot start in the current state | 1. Wait for the current action or mode change to finish. 2. If the app remains busy, click Stop, then Clear Faults before enabling again. |
| FBK-3203 | Selected joints are outside the starting range | 1. Check that the hand can move freely. 2. Use the existing Enable flow to return the selected joints to the starting range. |
| FBK-3204 | Selected motor positions need to be recovered | 1. Wait for position recovery to finish. 2. If recovery fails, open Motor Calibration and calibrate the affected joints. |
| FBK-3206 | The controller is still changing control mode | 1. Wait for the current action or mode change to finish. 2. If the app remains busy, click Stop, then Clear Faults before enabling again. |
| FBK-3207 | The controller is busy handling earlier requests | 1. Wait for the current action or mode change to finish. 2. If the app remains busy, click Stop, then Clear Faults before enabling again. |
| RDY-3301 | Joint feedback mode is not selected | 1. Select Feedback Control when you want to control joint angles. |
| RDY-3302 | Motor positions are not ready for joint control | 1. Wait for position recovery to finish. 2. If recovery fails, open Motor Calibration and calibrate the affected joints. |
| RDY-3303 | No joints are selected | 1. Select the joints you want to use, then retry the action. |
| RDY-3304 | Current joint angles are unavailable | 1. Check the S3 sensor board and its power and CAN connections. 2. Wait for current joint readings to return before enabling control. |
| RDY-3305 | Selected joints need calibration data | 1. Open Joint Calibration and check the selected joints' Manual Min and Max. 2. Open Motor Calibration and calibrate the affected joints. |
| RDY-3306 | A current joint angle is invalid | 1. Check the S3 sensor board and its power and CAN connections. 2. Wait for current joint readings to return before enabling control. |
| RDY-3307 | A joint is below its starting range | 1. Check that the hand can move freely. 2. Use the existing Enable flow to return the selected joints to the starting range. |
| RDY-3308 | Joint calibration no longer matches its Manual endpoints | 1. Open Joint Calibration and check the selected joints' Manual Min and Max. 2. Open Motor Calibration and calibrate the affected joints. |
| RDY-3309 | Joint position feedback is unavailable | 1. Check the S3 sensor board and its power and CAN connections. 2. Wait for current joint readings to return before enabling control. |
| RDY-3310 | A device fault is preventing the Ready state | 1. Open Diagnostics and resolve the current fault shown for the affected device. 2. After correcting the cause, click Clear Faults before enabling control again. |
| RDY-3311 | A motor position safety limit is preventing control | 1. Check that the affected joint and tendons can move freely. 2. If Stop is active, click Clear Faults after checking the hand. 3. Open Motor Calibration, select the affected joints, and calibrate them again. |
| ROP-3401 | Return to the starting range took too long | 1. Check for an obstruction or a joint that cannot move freely. 2. Check that live motor and joint position readings update. 3. Correct the observed problem before trying Enable again. |
| ROP-3402 | Position updates stopped during the return movement | 1. Check the S3 sensor board and its power and CAN connections. 2. Wait for current joint readings to return before enabling control. |
| ROP-3403 | A motor stopped responding during the return movement | 1. Check motor power and the affected bus connections. 2. Wait for the affected motors to show Online, then retry the original action. |
| ROP-3404 | Motor positions became uncertain during the return movement | 1. Wait for position recovery to finish. 2. If recovery fails, open Motor Calibration and calibrate the affected joints. |
| ROP-3405 | The return movement needs Manual endpoints | 1. Open Joint Calibration and check the selected joints' Manual Min and Max. 2. Open Motor Calibration and calibrate the affected joints. |
| ROP-3406 | The return movement needs motor calibration | 1. Open Joint Calibration and check the selected joints' Manual Min and Max. 2. Open Motor Calibration and calibrate the affected joints. |
| ROP-3407 | The controller could not send the return target | 1. Stop using motion controls and open Details. 2. When the hand is safe, restart the controller and reconnect. 3. If it happens again, save the session log for support. |
| ROP-3408 | A device fault interrupted the return movement | 1. Open Diagnostics and resolve the current fault shown for the affected device. 2. After correcting the cause, click Clear Faults before enabling control again. |
| ROP-3409 | The return movement was cancelled | 1. Check the selected mode and start the intended action again when ready. |
| FLT-3501 | The joint sensor board is not responding reliably | 1. Check the S3 sensor board and its power and CAN connections. 2. Wait for current joint readings to return before enabling control. |
| FLT-3502 | A motor load safety limit was reached | 1. Check for an obstruction, a tight tendon, or a joint that cannot move freely. 2. After correcting the cause, click Clear Faults and retry the original action. |
| FLT-3503 | Motor feedback is unavailable | 1. Check motor power and the affected bus connections. 2. Wait for the affected motors to show Online, then retry the original action. |
| FLT-3505 | Motor positions are not trusted | 1. Wait for position recovery to finish. 2. If recovery fails, open Motor Calibration and calibrate the affected joints. |
| CAW-4101 | A joint did not return far enough during calibration | 1. Check tendon routing, preload, and the affected joint's ability to move freely. 2. Correct the observed issue, then calibrate that joint again. |
| CAW-4102 | A joint did not reach far enough during calibration | 1. Check tendon routing, preload, and the affected joint's ability to move freely. 2. Correct the observed issue, then calibrate that joint again. |
| CAW-4103 | Measured joint travel differs from the model | 1. Check tendon routing, preload, and the affected joint's ability to move freely. 2. Correct the observed issue, then calibrate that joint again. |
| COM-6001 | The controller could not start the sensor connection | 1. Check the affected connection and confirm that the app and firmware versions match. 2. If the problem continues, open Details and save the session log. |
| COM-6002 | A sensor message could not be read | 1. Check the affected connection and confirm that the app and firmware versions match. 2. If the problem continues, open Details and save the session log. |
| COM-6003 | A complete joint position update was not received | 1. Check the affected connection and confirm that the app and firmware versions match. 2. If the problem continues, open Details and save the session log. |
| COM-6004 | A complete touch sensor update was not received | 1. Check the affected connection and confirm that the app and firmware versions match. 2. If the problem continues, open Details and save the session log. |
| COM-6005 | The controller could not read an app command | 1. Check the affected connection and confirm that the app and firmware versions match. 2. If the problem continues, open Details and save the session log. |
| RUN-8001 | Some older diagnostic events were overwritten | 1. Save the current session log. 2. Check Details for a frequently repeated issue. |
| RUN-8002 | The controller could not create an internal message channel | 1. Stop using motion controls and open Details. 2. When the hand is safe, restart the controller and reconnect. 3. If it happens again, save the session log for support. |
| RUN-8003 | The controller could not create an internal lock | 1. Stop using motion controls and open Details. 2. When the hand is safe, restart the controller and reconnect. 3. If it happens again, save the session log for support. |
| RUN-8004 | An essential controller task could not start | 1. Stop using motion controls and open Details. 2. When the hand is safe, restart the controller and reconnect. 3. If it happens again, save the session log for support. |
| RUN-8005 | A controller task is repeatedly running late | 1. If this repeats during movement, click Stop. 2. Save the session log and Task Runtime details for support. |
| RUN-8006 | A controller task is running low on working memory | 1. If this repeats during movement, click Stop. 2. Save the session log and Task Runtime details for support. |

## 验证记录

- 全量 pytest：**1511 passed, 1 skipped**（28.71 秒）。跳过项是仅适用于 Windows 的 GDB 启动器进程树清理测试；Linux 平台按原有测试条件跳过。测试使用仓库要求的 pytest 8.4.2 / NiceGUI 3.16.0，运行数据和日志重定向到 `/tmp`。
- P4 原生 C++ 测试验证停止锁存下拒绝 F1 且事务状态无改写、双向 LUT 余量、旧限位重建、健康快照单次取样及原有超时边界。
- 协议、机器人模型/指纹、安全规则、控制参数四个生成检查通过；安全规则闭环 **73/73** 通过；Desktop 架构 **86 文件**通过；P4 架构 **211 个 C/C++ 文件**通过（使用临时目录中的 PowerShell 7.4.6 运行原检查脚本）。
- P4 Arduino 编译通过（ESP32 Arduino core 3.3.10，`-Og -g3`）。程序 503444 bytes（38%），全局内存 159962 bytes（48%），固件源指纹 **B29824D4**。产物单独放入 `.debug/p4-safety-cleanup/`，没有覆盖此前已刷写的 `.debug/p4/` ELF/BIN。
- 本次没有刷写、重设零或驱动真实电机；硬件复测仍需在新固件刷写后进行。
