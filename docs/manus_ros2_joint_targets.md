# MANUS 21 角度 ROS2 链路

## 数据链路

```text
MANUS Core (Windows)
-> manus_bridge_test TCP JSON
-> desktop MANUS calibration / retargeting (Linux)
-> Float32MultiArray /mmhand/joint_targets
-> desktop ROS2 subscriber
-> HandBackend -> P4 -> hand
```

消息约定：

- type: `std_msgs/msg/Float32MultiArray`
- data: 正好 21 个有限数值
- unit: degree
- order: desktop `J00` 到 `J20`
- layout label: `mmhand_joints_deg`

## 启动

Linux 上启动 ROS2 和 Teleop 上位机：

```bash
cd ~/Desktop/mmhand-cl
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0
python3 -m desktop ui --mode teleop --port /dev/ttyACM0 --ros2
```

Windows 上 `manus/manus_bridge_test/config/manus_bridge_config.json` 需要把
`tcp.target_ip` 改成 Linux 上位机的局域网 IP，例如：

```json
"tcp": {
  "enabled": true,
  "target_ip": "192.168.1.23",
  "target_port": 9001,
  "reconnect_s": 1.0,
  "max_hz": 120.0
}
```

Linux 上位机监听 `0.0.0.0:9001`，所以同机 `127.0.0.1` 和跨机器 Linux IP
都可以接收。跨机器时还要放行 Linux 防火墙的 TCP 9001 端口。

仅验证数据、不连接灵巧手时去掉 `--port`：

```bash
python3 -m desktop ui --mode teleop --ros2
```

在 UI 中：

1. 选择 `Mode 2: MANUS / Rokoko Glove`。
2. Source 选择 `TCP Bridge :9001`。
3. 完成 MANUS 标定并确认输入为 `LIVE`。
4. 选择需要遥控的关节；无硬件反馈时必须选择全部 21 个关节。
5. 保持 `Route via ROS2` 勾选。使用 `--ros2` 启动时默认勾选。
6. 点击 `Send`。

勾选 ROS2 路由后，MANUS 页面不会再直接调用硬件发送；消息只发布到
ROS2，由现有 desktop subscriber 接收后下发，避免同一帧双重控制。

## 验证

另开一个已 source ROS2 的终端：

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0
ros2 node list | grep mmhand
ros2 topic type /mmhand/joint_targets
ros2 topic info -v /mmhand/joint_targets
ros2 topic hz /mmhand/joint_targets
ros2 topic echo /mmhand/joint_targets --once
```

应同时看到节点：

```text
/mmhand_manus_joint_target_publisher
/mmhand_desktop_joint_target_subscriber
```

验收条件：

- topic type 为 `std_msgs/msg/Float32MultiArray`
- publisher count 和 subscription count 都至少为 1
- `data` 长度为 21，且没有 `NaN`/`Inf`
- 活动手套时数据持续变化
- UI 的 `sent` 和 `last_sent_seq` 持续增加
- 上位机日志中的消息来源为 `ROS2:/mmhand/joint_targets`

可以用下面的命令独立校验一帧长度和数值：

```bash
source /opt/ros/jazzy/setup.bash
python3 - <<'PY'
import math
import rclpy
from std_msgs.msg import Float32MultiArray

rclpy.init()
node = rclpy.create_node("mmhand_joint_target_validator")

def callback(msg):
    values = list(msg.data)
    assert len(values) == 21, len(values)
    assert all(math.isfinite(value) for value in values)
    print("PASS", values)
    rclpy.shutdown()

node.create_subscription(Float32MultiArray, "/mmhand/joint_targets", callback, 10)
rclpy.spin(node)
PY
```

真实硬件验证时不要使用 `--auto-start`。先确认反馈、关节方向和限位，
从一根手指开始逐步扩大选中范围。
