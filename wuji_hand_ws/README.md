# Wuji 手套到 MMHand 遥操

本工作区直接使用 Wuji SDK、Wuji 原生界面和当前 MMHand `structure_urdf_v2` 模型，发布 21 个关节的 **URDF 度数**到 `/raw_ik_target`。工作区不依赖 mmhand 源码，运行时只通过 ROS2 topic 与 Linux 上位机连接。

## 数据契约

- Topic：`/raw_ik_target`
- 类型：`std_msgs/msg/Float32MultiArray`
- 顺序：J00 到 J20，共 21 项
- 单位：度
- 坐标：直接 URDF 角度，不对 MCP/PIP/DIP 额外取反
- 模型：`structure_urdf_v2`，mapping v3，CRC `73FD45FA`
- URDF 来源提交：`372a647e2f2a48ac48707c717790482a7819500a`

模型、mesh、关节顺序和角度范围都记录在 `urdf_contract.json`。发送节点启动时会校验 URDF SHA256，UI 会校验消息契约。

## 环境与编译

```bash
cd /path/to/mmhand-cl/wuji_hand_ws
source /opt/ros/jazzy/setup.bash
rm -rf build install log
colcon build --symlink-install
source install/setup.bash
```

官方 `wuji-sdk` Linux runtime 位于包内 `_vendor`。Windows 可以阅读和测试纯 Python 映射代码，但 Wuji SDK 的 `.so` 不能在 Windows 原生进程中运行；实际手套采集节点应在 Linux 启动。

## 启动前准备

1. MMHand 上位机已连接 P4。
2. Device Check 中本次使用的关节已有 Manual 数据。
3. 已完成 Auto Calibration，选中关节进入 READY。
4. 遥操前确认急停可用，并先低速、小范围测试。

## 启动 Wuji UI

```bash
source /opt/ros/jazzy/setup.bash
source /path/to/mmhand-cl/wuji_hand_ws/install/setup.bash
ros2 run wuji_mmhand_teleop wuji_raw_ik_teleop_ui
```

在界面中选择 `Side`，可填写 `Device SN`；留空时由 SDK 自动扫描。点击 `Send` 启动手套采集和 IK，点击 `Stop` 停止。

更换本次 URDF 后，旧 Wuji 姿态标定不会静默复用。依次采集：

1. `Record Align`
2. `Record Open AA`
3. `Record Fist Flex`
4. `Record Thumb->PinkyRoot`

## 仅启动发送节点

```bash
ros2 run wuji_mmhand_teleop wuji_raw_ik_target_sender --ros-args \
  -p side:=left \
  -p device_sn:="" \
  -p confidence_threshold:=0.4 \
  -p output_topic:=/raw_ik_target
```

## 启动 MMHand 接收链路

在 mmhand 仓库根目录执行：

```bash
source /opt/ros/jazzy/setup.bash
python -m desktop ui --mode teleop --port /dev/ttyUSB0 \
  --ros2 \
  --ros2-input raw_ik \
  --ros2-raw-ik-topic /raw_ik_target \
  --ros2-raw-ik-space urdf_deg
```

该命令复用 mmhand 通用 `raw_ik` 接收器，不再需要专用 Wuji ROS2 节点。

## 连接诊断

```bash
ros2 node list | grep wuji
ros2 topic list | grep -E 'raw_ik_target|wuji_calibration'
ros2 topic info /raw_ik_target -v
ros2 topic hz /raw_ik_target
ros2 topic echo /raw_ik_target --once
ros2 topic echo /wuji_calibration_status
```

正常情况应看到：

- 节点 `/wuji_raw_ik_target_sender`；
- `/raw_ik_target` 约按配置周期持续发布；
- 每帧 `data` 长度为 21；
- `layout.dim[0].label` 包含 `urdf_deg` 和 `structure_urdf_v2`；
- 状态话题显示 `valid 21/21`，而不是 `Waiting for live skeleton` 或 `input error`。

## 自动测试

```bash
cd /path/to/mmhand-cl/wuji_hand_ws
python3 -m pytest tests -q
```

测试会检查 URDF 哈希、21 关节顺序、mesh 完整性、角度限位和直接 URDF 角度往返换算。
