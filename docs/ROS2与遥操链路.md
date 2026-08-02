# ROS2 与遥操链路

最后核对：2026-08-01

## 1. ROS2 在本项目中的职责

ROS2 不是直接驱动 P4 的另一套协议，而是手套/IK 算法与 desktop 之间的数据总线。它提供节点解耦、topic、QoS、参数、状态和可替换发布者；最终安全授权、工作空间保护、串口编码、状态准入和舵机执行仍走现有 desktop→P4 链路。

```text
Wuji 设备 → retarget/四姿态标定 → ROS2 publisher
                                      │ /raw_ik_target
                                      ▼
desktop ROS2 subscriber → 坐标适配 → workspace guard → 用户授权/scope 绑定
                                      │ USB UART
                                      ▼
P4 状态机 → ControlSolver → ServoSafetyPolicy → ServoComm
```

这样做的好处是更换手套、IK 或可视化节点时不改下位机协议，同时所有外部运动源都必须通过同一安全门，避免 ROS2 回调直接写串口造成 E-STOP 后自动复活。

## 2. Wuji 发布端

包路径为 `wuji_hand_ws/src/wuji_mmhand_teleop`，入口点 `wuji_raw_ik_target_sender`，节点名 `wuji_raw_ik_target_sender`。主要参数：

| 参数 | 当前默认 |
| --- | --- |
| `side` / `device_sn` | 选择手与设备 |
| `confidence_threshold` | 0.4 |
| `output_topic` | `/raw_ik_target` |
| `timer_period_s` | 0.01 s（100 Hz） |

消息类型为 `std_msgs/msg/Float32MultiArray`，QoS depth 10，严格输出 J00~J20 共 21 个角度。只有 IK 结果 21/21 全部有效时才发布，避免局部 NaN 或缺关节被 desktop 当作完整命令。layout label 为：

```text
mmhand:J00-J20:urdf_deg:<model_id>
```

四姿态标定为 `align`、`open_aa`、`fist_flex`、`thumb_pinky_root`，并把结果绑定到当前 model id。Wuji 还发布 `wuji_calibration_status`、订阅 `wuji_calibration_command`，便于 UI/脚本触发标定并观察阶段状态。

## 3. URDF 契约与为什么要绑定模型

Wuji 启动时读取随包发布的 `urdf_contract.json`，校验 URDF/adapter 文件 SHA256、21 路顺序、输入空间和关节限位。当前 contract 为 schema 2、mapping version 3，输入空间 `urdf_deg`，并记录控制/可视模型 CRC。

绑定模型的原因是：同一个 21 元数组在不同关节顺序、正负方向或限位版本下会代表不同姿态。没有 model id/CRC 时，ROS2 topic 名相同也不能证明发布者和执行端理解一致。

目前 Wuji 发布端会验证 layout contract；desktop 的通用 `JointTargetCommandHandler` 只读取 `msg.data`，执行 21 路长度、finite 值与坐标适配，并没有再次核验 `Float32MultiArray.layout.label`。因此严格端到端契约仍有改进空间：desktop 应把 label/model id 纳入拒绝条件，或改用自定义带 version/model_id/sequence/timestamp 的 ROS2 message。

## 4. desktop 订阅与坐标适配

desktop 提供两条 ROS2 输入：

| 输入 | 默认 topic | 默认坐标空间 | 用途 |
| --- | --- | --- | --- |
| final joint target | `/mmhand/joint_targets` | `urdf_deg` | 已经是当前 URDF 21 关节目标 |
| raw IK | `/raw_ik_target` | 兼容默认 `legacy_calibrated_deg` | 外部 MANUS/IK 或 Wuji 输出 |

Wuji 当前发布 `urdf_deg`，启动 desktop raw-IK bridge 时必须选择 `--ros2-raw-ik-space urdf_deg`；保留 legacy 默认只是为了旧发布者兼容。坐标空间设错可能出现数值长度正常但方向/零点错误，这是比 ROS2 是否连通更关键的检查。

订阅节点分别为 `mmhand_desktop_joint_target_subscriber` 和 `mmhand_desktop_raw_ik_target_subscriber`，均使用 `Float32MultiArray`、QoS depth 10，并在后台线程 spin。`adapt_joint_targets()` 负责长度、finite 和坐标变换；随后统一调用 `HandBackend.send_ros2_joint_targets()`，不能直接调用普通 send。

## 5. 输出授权门

用户打开 RAW IK/ROS2 输出前，普通 controller 必须已经 ENABLE。授权时记录当前 active scope；每一帧发送前在同一把 `_ros2_output_lock` 内检查：

1. 输出仍 armed；
2. P4 串口在线；
3. controller 已启动；
4. 当前 active scope 与授权时 scope 相同。

通过后才设置 `JOINT_ANGLE` 模式，并把授权时 joint mask 写入本帧命令。pause、E-STOP、reset、active scope change、ROS2 bridge stop 和 replay start 都会撤销授权。E-STOP 使用同一把锁完成“撤权→STOP”，因此高频 ROS2 回调不能插到两步之间重新发送 RUN_FEEDBACK。

未授权时帧仍可进入 preview cache，用于观察/保存手势，但不会下发；preview 超过 0.3 s 被视为过期。相同丢帧原因的日志每秒最多一次，防止 100 Hz 输入刷屏。

## 6. MANUS 与 Wuji 的区别

- Wuji 当前提供完整原生 ROS2 publisher、URDF contract 和四姿态标定链路。
- MANUS 可通过 desktop 本地 SDK/TCP/IK 兼容路径产生 21 路目标；是否经过 ROS2 取决于启动方式，不能笼统写成“MANUS 使用 ROS2”。
- 两者最终都应转换为明确的输入坐标空间，并汇合到 desktop backend 的授权门。
- MANUS 逻辑保留 `MANUS_JOINT_SEND_GATED`，默认强调 preview 与“Enable Joint Send”显式授权；不得把手套连接成功等同于舵机输出已启用。

## 7. ROS2 能力验证

建议分层验证：

1. `ros2 topic hz /raw_ik_target`：确认接近 100 Hz；
2. `ros2 topic echo --once /raw_ik_target`：确认 21 路、finite、layout label/model id；
3. 只开 desktop preview，验证 URDF 姿态方向和范围，不授权舵机；
4. 选择单关节 active scope，ENABLE 后再单独打开 RAW IK 输出；
5. 测试 pause/E-STOP/scope change 后持续发布仍不能恢复下发；
6. 断开串口、停止 bridge、切换 URDF contract，确认均产生明确拒绝；
7. 最后记录 publisher timestamp、desktop receive/send 和 P4 apply，统计端到端延时及抖动。

## 8. 可继续改进

- 自定义 ROS2 message，加入 schema version、model id、sequence、source timestamp、valid mask；
- desktop 严格验证 layout label 和 model fingerprint；
- 加入 source freshness watchdog，发布者停更时主动撤权/hold，而不仅依赖最后一次回调；
- 用 ROS2 diagnostics 上报 IK valid ratio、drop reason、scope、armed 状态和端到端 latency；
- 多个遥操源需要显式 arbiter/lease，不能让 topic 到达顺序决定谁控制。
