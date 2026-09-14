# MMHand 上位机

## 1. 环境配置

推荐使用 Python 3.12，与 ROS2 Jazzy 的 Python 版本保持一致。

```bash
cd /home/mmhand/hand_ws/mh
conda create -n mmhand python=3.12 -y
conda activate mmhand
python -m pip install -r desktop/requirements.txt
```

每次启动上位机前激活 Conda 环境并加载 ROS2：

```bash
cd /home/mmhand/hand_ws/mh
conda activate mmhand
source /opt/ros/jazzy/setup.bash
```

检查环境：

```bash
python -c "import rclpy, serial, numpy, PySide6; print('environment OK')"
```

## 2. 上位机 CLI 启动及参数

上位机统一使用 `python -m desktop` 启动。图形上位机使用 `ui` 子命令：

```bash
python -m desktop ui -p /dev/ttyUSB0
```

`ui` 默认进入 `monitor` 模式。终端监视与控制界面使用 `cli` 子命令：

```bash
python -m desktop cli -p /dev/ttyUSB0
```

不指定 `-p` 时，终端 CLI 会交互选择 P4 串口；图形 UI 会在未连接状态下启动，
可在界面中选择串口。

### 公共参数

| 参数 | 可选值 | 默认值 | 含义 |
|---|---|---|---|
| `--port`、`-p` | 串口名称 | 未指定 | P4 主串口，例如 `/dev/ttyUSB0` 或 `COM7` |
| `--tactile-port` | 串口名称 | 未指定 | 可选的 S3 触觉串口；其他数据和控制仍使用 P4 主串口 |
| `--ros2` | 无 | 开启 | 启用选定的 ROS2 订阅和发布；通常无需显式填写 |
| `--no-ros2` | 无 | 关闭 | 完全关闭 ROS2 订阅和发布 |
| `--ros2-input` | `off`、`joint_target`、`raw_ik` | `joint_target` | 选择不订阅、订阅 `/mmhand/joint_targets` 或订阅 `/raw_ik_target` |
| `--ros2-output` | `off`、`joint_states`、`joint_cmd`、`all` | `all` | 选择不发布、只发布实测状态、只发布收到的命令或同时发布两者 |
| `--ros2-feedback-mode` | `armed`、`always` | `armed` | `armed` 仅在 ROS2 INPUT 授权后发布状态；`always` 持续发布状态用于诊断 |
| `--auto-start` | 无 | 关闭 | 连接后自动启动普通控制器；不会自动授权 ROS2 输入 |
| `--help`、`-h` | 无 | — | 显示帮助 |

ROS2 输入话题和单位固定：

| `--ros2-input` | 订阅话题 | 消息类型 | 内容 |
|---|---|---|---|
| `off` | 不订阅 | — | 不接收 ROS2 关节命令 |
| `joint_target` | `/mmhand/joint_targets` | `std_msgs/msg/Float32MultiArray` | J00–J20 共 21 个 URDF 目标角，单位 degree |
| `raw_ik` | `/raw_ik_target` | `std_msgs/msg/Float32MultiArray` | J00–J20 共 21 个 raw IK 目标角，单位 degree；发布端负责 radian 到 degree 的转换 |

### UI 专用参数

| 参数 | 可选值 | 默认值 | 含义 |
|---|---|---|---|
| `--mode`、`-m` | `monitor`、`replay` | `monitor` | 选择实时监视或轨迹回放界面 |

示例：

```bash
python -m desktop ui -p /dev/ttyUSB0 --ros2-input raw_ik
```

### 终端 CLI 专用参数

| 参数 | 默认值 | 含义 |
|---|---|---|
| `--no-plot` | 关闭 | 禁用关节调试曲线窗口 |

终端 CLI 控制键：

| 按键 | 功能 |
|---|---|
| `s` | 启动普通控制器 |
| `i` | 授权 ROS2 输入 |
| `o` | 撤销 ROS2 输入授权 |
| `x` | 停止控制器 |
| `r` | 重置控制器 |
| `c` | 请求固件标定 |
| `q` | 退出终端 CLI |

## 3. 发布的两个 ROS2 话题

发布内容由 `--ros2-output` 选择。默认值 `all` 会同时发布以下两个固定话题。

### `/mmhand/joint_states`

- 消息类型：`sensor_msgs/msg/JointState`
- 内容：P4 上报并由上位机映射得到的 21 个实测 URDF 关节角
- `header.stamp`：该实测关节角帧到达上位机的系统时间
- `header.frame_id`：`mmhand:p4_time_ms=<uint32>`，记录同一数据包中的 P4 开机毫秒计数
- `name`：`J00` 至 `J20`
- `position`：21 个实测关节角，单位 radian
- `velocity`：空
- `effort`：空

在默认的 `--ros2-feedback-mode armed` 下，只有操作员在 UI 点击 `ROS2 INPUT`
或在终端 CLI 按 `i` 后才会发布。使用 `always` 时持续发布，主要用于诊断。

### `/mmhand/joint_cmd`

- 消息类型：`sensor_msgs/msg/JointState`
- 内容：上位机从当前选定输入话题实际收到的有效关节角命令
- `header.stamp`：命令到达上位机的系统时间
- `header.frame_id`：记录输入话题和输入空间，例如 `mmhand:input=/raw_ik_target;space=urdf_deg`
- `name`：`J00` 至 `J20`
- `position`：收到的原始 21 个目标角，单位 degree
- `velocity`：空
- `effort`：空

`joint_cmd.position` 不使用经过限幅或其他处理后的硬件下发值，而是保留上位机
收到并通过长度及有限数校验的原始命令。因此，即使命令因未授权、控制器未启动或
串口离线而没有发送给 P4，有效输入仍会发布到 `/mmhand/joint_cmd`。

查看发布内容：

```bash
ros2 topic echo /mmhand/joint_states --once
ros2 topic echo /mmhand/joint_cmd --once
```
