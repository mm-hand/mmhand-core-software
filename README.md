# MMHand 上位机与 HORA ROS2 消息接口

本分支保存当前 `/home/mmhand/hand_ws/mh` 工作树的代码快照，并说明本次为
HORA 增加的双向 ROS2 通信接口。P4 下位机协议未因本次适配而修改。

## 数据链路

```text
P4 实测映射角
  -> MMHand 上位机
  -> /mmhand/joint_states
  -> HORA 策略节点
  -> /mmhand/joint_targets
  -> 上位机安全门、作用域和工作空间检查
  -> P4
```

上位机只创建命令行所选输入源的一份订阅：

- `--ros2-input joint_target`：订阅 `/mmhand/joint_targets`，供 HORA 使用。
- `--ros2-input raw_ik`：订阅 `/raw_ik_target`，供外部遥操/IK 使用。
- `--no-ros2`：完全关闭 ROS2 桥。

## 关节反馈 `/mmhand/joint_states`

| 属性 | 内容 |
| --- | --- |
| ROS2 类型 | `sensor_msgs/msg/JointState` |
| QoS | depth 1、best-effort、volatile |
| 检查频率 | 100 Hz；只有新的有效实测帧才发布 |
| `name` | `J00` 到 `J20`，固定顺序，共 21 项 |
| `position` | P4 实测映射关节角，URDF 弧度，固定 21 项 |
| `velocity` | 空数组；当前策略不使用该字段 |
| `effort` | 空数组；当前策略不使用该字段 |

`JointState` 是 ROS2 标准消息，`velocity` 和 `effort` 是其固定字段；没有数据时
保留为空数组，不代表错误。

### `header.stamp` 的含义

`header.stamp` 是**该帧 P4 映射关节角到达上位机时，上位机记录的系统时间**，
由 `sec` 和 `nanosec` 两部分组成：

```text
stamp_seconds = sec + nanosec / 1e9
```

它不是：

- P4 真正采样关节角的时间；
- P4 `millis()` 时间；
- 上位机调用 ROS2 `publish()` 的时间；
- HORA 收到消息的时间。

因此 HORA 计算的

```text
feedback_age = HORA 当前系统时间 - header.stamp
```

覆盖“上位机收到映射角到 HORA 检查该帧”的延迟，但不包含此前 P4 采样和 USB
传输的完整延迟。上位机与 HORA 位于同一台机器时二者直接使用同一系统时钟。
若跨机器部署，必须先同步系统时钟。

每个关节角状态还独立保存 P4 来源时间、上位机单调接收时间和帧序号用于内部
诊断与新鲜度判断；其他遥测包和 UI 刷新不会更新 `header.stamp`。同一映射角包
携带的 P4 开机后 `millis()` 写入 `header.frame_id`：

```text
mmhand:p4_time_ms=<uint32>
```

该值约 49.7 天回绕，只用于上下位机日志关联。它没有映射到 ROS 时钟，不能与
`header.stamp` 直接相减；这里的 `frame_id` 是诊断元数据，不表示 TF 坐标系。

以下反馈不会发布：

- 长度不是 21、包含非有限数或编码器角度错误；
- 不是一帧新的映射角；
- 在上位机中已超过 200 ms；
- `armed` 模式下尚未获得 `ROS2 INPUT` 授权。

## HORA 目标 `/mmhand/joint_targets`

| 属性 | 内容 |
| --- | --- |
| ROS2 类型 | `std_msgs/msg/Float32MultiArray` |
| QoS | depth 1、reliable、volatile |
| `data` | `J00` 到 `J20` 的 21 个绝对 URDF 目标角 |
| 默认单位 | degree (`urdf_deg`) |
| `layout.dim[0].label` | HORA 发布 `mmhand:J00-J20:urdf_deg` |

`Float32MultiArray` 本身没有时间戳。当前上位机根据启动参数确定目标坐标空间，
并执行长度、有限值、关节映射、工作空间、活动作用域、普通控制器状态和用户授权
检查。等待、关闭或未授权期间收到的目标直接丢弃，不缓存、不补发。

## 反馈模式与 `ROS2 INPUT`

`--ros2-feedback-mode` 支持：

- `armed`（默认）：`ROS2 INPUT` 同时控制目标放行和反馈发布。
- `always`：持续发布有效新反馈，仅用于诊断；关闭按钮不会让 HORA 超时重置。
- `off`：不发布反馈，只保留所选目标订阅。

在 `armed` 模式下，关闭、暂停、急停或断连会立即撤销目标授权并停止反馈。
重新开启前至少停发 300 ms，丢弃冷却期间的反馈与目标；冷却结束后必须收到
一帧新的有效实测角才进入闭环。该窗口用于触发 HORA 默认 200 ms 超时并清空
策略历史。

启动程序不要求手已经抓握，但授权 HORA 前必须由操作员确认物体已经稳定抓住。
普通 `ENABLE`、全手作用域、串口在线和有效实测角仍是放行条件。

## 启动上位机

```bash
cd /home/mmhand/hand_ws/mh
conda activate mmhand
source /opt/ros/jazzy/setup.bash

python -m desktop ui --mode monitor --port /dev/ttyACM0 \
  --ros2-input joint_target \
  --ros2-feedback-mode armed \
  --ros2-joint-state-topic /mmhand/joint_states \
  --ros2-joint-target-topic /mmhand/joint_targets
```

将 `/dev/ttyACM0` 替换为实际 P4 串口。启动后依次确认标定、全手作用域和普通
`ENABLE`，稳定抓握后再点击 `ROS2 INPUT`。

## ROS2 诊断

```bash
ros2 topic info -v /mmhand/joint_states
ros2 topic echo /mmhand/joint_states sensor_msgs/msg/JointState --qos-profile sensor_data
ros2 topic hz /mmhand/joint_states

ros2 topic info -v /mmhand/joint_targets
ros2 topic echo /mmhand/joint_targets std_msgs/msg/Float32MultiArray
ros2 topic hz /mmhand/joint_targets
```

`armed` 模式且 `ROS2 INPUT` 关闭时，能够看到 publisher 端点但收不到
`/mmhand/joint_states` 数据是预期行为。

更完整的联调步骤见 `docs/HORA_ROS2_联调.md`。

## 切换到 mmhand_teleop 并持续发布实测关节角

停止 HORA 节点并关闭原上位机进程后，以 `raw_ik` 输入重新启动。`always` 模式
会持续发布 `/mmhand/joint_states`；遥操作目标仍需在界面中点击 `ROS2 INPUT`
才会下发。

```bash
cd /home/mmhand/hand_ws/mh
source /opt/ros/jazzy/setup.bash

export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

.venv/bin/python -m desktop ui \
  --mode monitor \
  --port /dev/ttyUSB0 \
  --ros2-input raw_ik \
  --ros2-raw-ik-topic /raw_ik_target \
  --ros2-raw-ik-space urdf_deg \
  --ros2-feedback-mode always \
  --ros2-joint-state-topic /mmhand/joint_states
```
