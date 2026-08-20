# 动作 CSV Replay

Replay 直接读取 `Save Data` 生成的动作指令或实际关节轨迹 CSV，并按记录中的
上位机时间戳重放整段21关节动作。它不再保存单帧手势 JSON，也不再提供手势
序列编辑。

## 1. 生成轨迹

1. 在任意控制页面连接 P4。
2. 完成标定与 Rope Init，确认系统进入 `READY`。
3. `Save Data` 默认已开启；连接成功后自动保存基础 CSV。若此前手动关闭，重新勾选即可恢复记录。
4. 使用普通负反馈 UI，或启用 `ROS2 INPUT` 后由外部 ROS2 节点下发目标。
5. 动作结束后如不再需要记录，可取消 `Save Data`。

若先启用 `ROS2 INPUT` 再开启 `Save Data`，记录器会从开启后的下一帧开始保存，
不会补录此前已经发送的动作。外部节点必须继续向 ROS2 topic 发布新帧，CSV 才会增长。

新版本会在本次数据目录生成：

```text
02_关节总表.csv
07_遥操作总表.csv
```

`07_遥操作总表.csv` 保存真正进入 backend 的动作指令；`02_关节总表.csv`
保存标定后的实际 `Current` 轨迹。前者适合复现“当时发了什么”，后者适合复现
“手实际上怎么运动”。

动作总表每行同时保存：

- `host_time`：用于 Replay 的上位机动作时间轴；
- `p4_time_ms`：用于上下位机日志关联，不用于播放调度；
- 原始输入、URDF输入和经过工作空间保护后的实际输出；
- `active_joint_mask` 和 `command_joint_mask`；
- 是否被拦截、是否钳位、是否真正送入 backend。
- 来源类型、连续/离散类型、连续流编号和命令序号。

Replay 只使用 `sent_to_backend=1` 且 `replayable=1` 的
`output_joint_targets_json`，因此不会重放被拒绝的目标、Replay自身命令或系统
Hold。普通按钮的单次 Send 是离散帧，只发送一次；连续 ROS2 和滑条目标最多
按20 Hz落盘。

## 2. 启动

```powershell
cd C:\浏览器下周\mmhand-cl\mmhand-cl
python -m desktop ui --mode replay --port COM3
```

Linux 示例：

```bash
cd /path/to/mmhand-cl
python -m desktop ui --mode replay --port /dev/ttyUSB0
```

## 3. 页面操作

页面只有两个操作按钮：

1. `导入 CSV`：选择 `07_遥操作总表.csv` 或 `02_关节总表.csv`。
2. `Send`：按 CSV 原始时间轴播放整个动作。

播放期间 `Send` 会变成 `Stop`。顶部 `Pause`、`STOP`、`E-STOP` 仍然可以终止 Replay。

导入后应核对：

- 帧数与动作时长；
- 平均频率和最大帧间隔；
- Active mask；
- Mapping 与 URDF CRC；
- 表格中的21个实时 `Current`、当前 `Replay Target` 和误差。

## 4. 时间语义

第一帧的 `host_time` 作为零点：

```text
t_rel = 当前帧host_time - 第一帧host_time
```

Replay 使用单调时钟按 `t_rel` 调度。连续轨迹以20 Hz关键帧保存，播放时在相邻
关键帧之间线性插值为100 Hz；超过200 ms的空白区保留为停顿，不跨区插值。
离散 UI 命令不插值且只发送一次。若上位机调度落后超过250 ms，Replay会把
剩余时间轴整体后移：当前帧正常发送，后续帧不突发补发，也不会仅因一次系统
调度抖动而终止整段动作。

记录器为动作轨迹使用独立的不丢帧队列。普通高频传感器仍使用有界队列，磁盘
短时繁忙时不会挤掉已经成功下发的动作目标。

历史 `16_遥操作输入_teleop_input.csv` 可以只读导入，但旧文件没有 `sent_to_backend` 或 active scope 时会显示警告。新保存逻辑不会再生成旧字段。

## 5. 安全条件

开始播放必须满足：

- P4处于 `READY` 或 `RUN_FEEDBACK`；
- CSV与当前 active scope至少有一个共同关节；
- 共同关节中至少有一个已经完成 Manual/Auto 标定；
- `mapping_id` 与 J00-J20关节顺序签名一致。

Replay允许降级播放：

- 带 `Jxx_target_deg` 或 `Jxx_current_deg` 名称的CSV可以只包含部分关节；
- CSV关节范围与当前active scope取交集，缺失或离线关节只跳过；
- 未完成Manual/Auto标定的关节只跳过，其余关节继续；
- 历史角度超出当前工作空间时逐帧钳到最近边界；
- URDF范围CRC变化只告警并钳位，不再拒绝整段轨迹；
- 旧CSV若既缺少关节顺序签名又发生mapping版本变化，仍会拒绝，避免Jxx含义错位。

页面会显示源关节数、实际播放关节数、跳过关节、钳位关键帧数量和时间轴
重定位次数。被跳过的关节显示 `SKIP`，发生钳位的目标使用橙色提示。

运行期间出现以下任一情况会立即终止：

- P4离开 `READY/RUN_FEEDBACK`；
- active scope变化；
- 普通UI或ROS2命令抢占Replay控制权；
- Pause、STOP、E-STOP、串口断开或发送失败。

碰撞保持不会暂停遥操或Replay。P4会冻结触发关节的舵机ABS，同时继续接收目标；
当后续目标相对碰撞目标反向回撤至少1度时解除保持。因此动作CSV中的反向帧必须
继续发送，不能由上位机提前终止。

Replay启动时会撤销 RAW IK 输出授权，避免手套和Replay同时写入目标。
