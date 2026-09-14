# HORA ROS2 联调

本适配基于独立仓库 `/home/mmhand/hand_ws/mh`，来源
`https://github.com/mm-hand/mmhand.git` 的 `cl` 分支，适配前基线提交为
`cf29ba6`（`merge: sync cl handover docs`）。

## 启动

上位机和 RL 节点运行在同一台机器并使用同一系统时钟。先启动上位机：

```bash
cd /home/mmhand/hand_ws/mh
source /opt/ros/jazzy/setup.bash
.venv/bin/python -m desktop ui --mode monitor --port /dev/ttyUSB0
```

另一个终端提前启动 HORA：

```bash
cd /home/mmhand/hand_ws/rl
source /opt/ros/jazzy/setup.bash
rl/hora/vendor/IsaacLab/env_isaaclab/bin/python -m rl.hora.deploy_ros \
  /path/to/exported_stage2.ckpt --device cuda:0
```

## 操作顺序

1. 保持 `ROS2 INPUT` 关闭，完成标定、选择全手作用域并按普通 `ENABLE`。
2. 手动形成稳定抓握；两个程序可一直保持运行。
3. 点击 `ROS2 INPUT`。系统完成至少 300 ms 停发窗口后，等待窗口结束后的
   新 P4 映射角，向 HORA 发布第一帧反馈，然后才放行策略目标。
4. 关闭按钮会立即阻止新目标并停止反馈；快速重新开启会显示“等待策略重置”。
   冷却期间的反馈和目标都不会补发。

正常调度下，300 ms 停发会触发 HORA 默认 200 ms 超时并清空历史。它不是
显式重置确认协议；如果 RL 进程卡死、DDS 严重积压或时钟异常，应停止并重启
HORA。急停、断连后必须由操作员重新授权。

## 诊断

```bash
ros2 topic info -v /mmhand/joint_states
ros2 topic hz /mmhand/joint_states
ros2 topic echo /mmhand/joint_states --once
ros2 topic info -v /mmhand/joint_targets
ros2 topic echo /mmhand/joint_cmd --once
```

`armed` 模式且按钮关闭时，看不到 `/mmhand/joint_states` 数据是预期行为。
`always` 仅用于诊断，因为它会让 HORA 持续推理，按钮关闭也不会触发历史重置。

`JointState.header.stamp` 是映射角包到达上位机的系统时间，供 HORA 判断
200 ms 新鲜度。`JointState.header.frame_id` 为
`mmhand:p4_time_ms=<uint32>`，其中数值是同一映射角包携带的 P4 开机后
`millis()`；该值会约 49.7 天回绕，只用于诊断和上下位机日志关联。
当前没有做 P4 到 ROS 时钟映射，禁止把两个时间戳直接相减。
