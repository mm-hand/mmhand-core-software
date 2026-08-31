# Wuji 手套适配当前 MMHand URDF 与 ROS2 链路说明

> 当前专项文档，最后核对：2026-08-27。Wuji 发布端使用 `urdf_deg` 和带 model id 的 layout label；desktop raw-IK bridge 默认并只接受 URDF 坐标。desktop 当前尚未严格校验 layout label，详见[ROS2与遥操链路](ROS2与遥操链路.md)。

## 1. 目的与对比范围

本文对比以下两个 Wuji ROS2 工作区：

| 角色 | 目录 |
|---|---|
| 参考版本 | `D:\11111\wuji_hand_ws` |
| 当前 MMHand 集成版本 | `wuji_hand_ws` |

目标不是把手套的原始输出直接转发给灵巧手，而是让 Wuji 经过重定向后输出的 21 个角度，严格使用当前 MMHand 的 `structure_urdf_v2` 坐标、关节顺序和角度单位。

当前工作区随提交 `dd2dde9` 进入 `cl` 分支，可用以下命令追溯本次完整改动：

```bash
git show --stat dd2dde9
git show dd2dde9 -- wuji_hand_ws
git diff dd2dde9^ dd2dde9 -- wuji_hand_ws
```

## 2. 适配结论

与 `D:\11111\wuji_hand_ws` 相比，当前版本完成了四层适配：

1. 替换了 Wuji 包内的 `hand.urdf` 和 27 个 mesh。实测两边各有 28 个 URDF/mesh 资产，SHA256 哈希集合交集为 `0`；其中部分 mesh 文件名也随当前结构仓库而变化，因此不能只按同名文件判断模型是否一致。
2. 固定了 J00-J20 的语义顺序、URDF 限位、角度单位和模型 CRC，避免发布端、UI 与标定文件各自理解不同。
3. 取消旧版本对 MCP/PIP/DIP 屈曲关节的隐藏符号翻转，所有 21 个输出统一为当前 URDF 的直接角度。
4. 让手套标定数据绑定当前模型 ID；替换 URDF 后旧标定不会被静默复用。

本次适配只影响 ROS2/Wuji 到上位机的输入语义，不修改 P4 的关节控制、自动标定或舵机映射。

## 3. 当前 21 关节契约

唯一配置源是：

- `wuji_hand_ws/src/wuji_mmhand_teleop/wuji_mmhand_teleop/assets/hku_hand_v2_urdf/urdf_contract.json`
- `wuji_hand_ws/src/wuji_mmhand_teleop/wuji_mmhand_teleop/urdf_contract.py`

当前契约：

| 项目 | 值 |
|---|---|
| 模型 | `structure_urdf_v2` |
| Mapping version | `3` |
| URDF CRC | `73FD45FA` |
| 话题 | `/raw_ik_target` |
| 消息类型 | `std_msgs/msg/Float32MultiArray` |
| 元素数量 | 21 |
| 顺序 | `J00` 至 `J20` |
| 单位 | 度 |
| 坐标含义 | 直接 URDF 角度，`urdf_deg` |

| 关节 | URDF 范围（度） |
|---|---:|
| J00 | `[0, 55]` |
| J01 | `[-15, 90]` |
| J02 | `[-9, 88]` |
| J03 | `[-12, 90]` |
| J04 | `[0, 63]` |
| J05 | `[-15, 90]` |
| J06 | `[-9, 88]` |
| J07 | `[-12, 90]` |
| J08 | `[0, 60]` |
| J09 | `[-15, 90]` |
| J10 | `[-9, 88]` |
| J11 | `[-12, 90]` |
| J12 | `[0, 65]` |
| J13 | `[-15, 90]` |
| J14 | `[-9, 88]` |
| J15 | `[-12, 90]` |
| J16 | `[0, 56]` |
| J17 | `[-15, 90]` |
| J18 | `[-9, 88]` |
| J19 | `[-12, 90]` |
| J20 | `[0, 77]` |

顺序按手指分组为：J00-J03 小拇指、J04-J07 无名指、J08-J11 中指、J12-J15 食指、J16-J20 拇指。该编号是逻辑关节编号，不等于 P4 中的电机编号 `Mxx`；双电机关节由 P4 后续展开为实际电机控制。

## 4. 相比参考工作区的代码修改

### 4.1 URDF 与 mesh

| 位置 | 当前修改 | 作用 |
|---|---|---|
| `assets/hku_hand_v2_urdf/urdf/hand.urdf` | 替换为当前活动 MMHand URDF | IK 使用的关节零点、轴向、层级和限位与灵巧手一致 |
| `assets/hku_hand_v2_urdf/meshes/` | 替换 27 个 mesh | Wuji UI/URDF Viewer 显示的模型与实际 URDF 一致 |
| `urdf_contract.json` | 新增 | 保存 21 关节顺序、限位、话题、SHA256、CRC 和来源提交 |
| `urdf_contract.py` | 新增 | 启动时验证 URDF SHA256、关节名与限位，并提供消息校验函数 |

发布节点 `sender.py` 在启动时会执行 `validate_bundled_urdf()`。如果 URDF 被手动替换但没有同步更新契约，节点会明确拒绝启动，不会带着错误模型下发角度。

### 4.2 关节角符号与限位

参考版本在 `robot_joint_map.py` 中对 MCP/PIP/DIP 隐式执行了转换：旧 URDF 的屈曲方向为负时，对外目标改为正的“屈曲幅值”。这样虽然方便旧模型使用，但“ROS2 输出角度”不再等于“URDF 角度”。

当前版本改为：

```text
target[Jxx] = 当前 URDF 对应 joint 的直接角度（度）
```

不再对屈曲关节执行 `target = -urdf_angle`。因此：

- `build_encoder_target_limits_deg()` 直接读取 URDF `lower/upper`；
- `/raw_ik_target` 的 21 个值可直接用于 URDF Viewer、ROS2 诊断和 MMHand 上位机；
- 上位机默认按 `urdf_deg` 解释；若发布端输出弧度，使用 `--ros2-raw-ik-space urdf_rad`。

### 4.3 手套姿态标定

Wuji 重定向不是“手套角度 = 机器人角度”的直接比例关系。它使用四组姿态建立人体手势与机器人关节目标之间的标定参考：

1. `align`
2. `open_aa`
3. `fist_flex`
4. `thumb_pinky_root`

`wuji_calibration.py` 额外把 `robot_model_id` 写入标定 JSON。读取旧标定文件时，如果模型 ID 与当前 `structure_urdf_v2:v3:73FD45FA` 不一致，会清空四个姿态结果并要求重新采集，而不是静默沿用旧模型数据。

### 4.4 发布节点与 Wuji UI

`nodes/sender.py` 的变化：

- 默认输出话题由契约提供：`/raw_ik_target`；
- 启动时校验 URDF；
- 只在 21 个关节全部有有效 IK 解时发布；
- `Float32MultiArray.data` 写入 21 个 `urdf_deg`；
- `layout.dim[0].label` 写入模型契约字符串，包含 J00-J20、单位、模型 ID 和 CRC；
- 日志输出当前手套左右手、置信度阈值和契约。

`ui/teleop_ui.py` 的变化：

- 显示当前模型契约；
- 支持选择 `left/right`、指定设备 SN、设置置信度阈值；
- 启动发送时把这些参数传给 ROS2 节点；
- 本地订阅预览时校验数组长度、有限数值和 `layout` 契约。

## 5. ROS2 信息流与基础知识

```mermaid
flowchart LR
  A["Wuji Glove"] --> B["Wuji SDK / WujiReceiver"]
  B --> C["手部骨架与指尖接触点"]
  C --> D["FullRawSkeletonRetargeter"]
  D --> E["21 个直接 URDF 角度\nJ00-J20 / urdf_deg"]
  E --> F["ROS2 Publisher\n/wuji_raw_ik_target_sender"]
  F --> G["/raw_ik_target\nFloat32MultiArray"]
  G --> H["MMHand RawIkTargetBridge"]
  H --> I["HandBackend.send_joint_targets()"]
  I --> J["P4 负反馈与舵机控制"]
```

### ROS2 中几个关键概念

| 概念 | 在本项目中的含义 |
|---|---|
| Workspace | `wuji_hand_ws`。源码放在 `src/`，由 `colcon build` 构建。 |
| Package | `wuji_mmhand_teleop`，通过 `package.xml` 声明 `rclpy`、`std_msgs` 等依赖。 |
| Node | 进程内的 ROS2 功能单元，例如 `wuji_raw_ik_target_sender`。 |
| Topic | 节点之间的异步数据通道；本链路是 `/raw_ik_target`。 |
| Publisher | Wuji 节点持续发布 21 个 IK 目标角。 |
| Subscriber | MMHand Linux 上位机订阅该话题并转交给 `HandBackend`。 |
| Message | `Float32MultiArray`，实际数值在 `data`，元数据在 `layout`。 |
| `source install/setup.bash` | 使当前终端能找到刚由 `colcon` 构建的 ROS2 包。每开一个新终端都要执行。 |

`Float32MultiArray` 本身不能强制约束“第 0 个元素一定是 J00”。因此本项目用 `urdf_contract.json` 和 `layout.dim[0].label` 作为附加约定。发布端和 Wuji UI 会校验该约定；MMHand 通用接收器当前以长度、数值合法性和启动参数 `urdf_deg` 为主进行验证，后续可增加对 `layout` 标签的强制校验以实现端到端硬门禁。

## 6. Linux 构建、运行与诊断

### 6.1 构建

```bash
cd /path/to/mmhand-cl/wuji_hand_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

`build/`、`install/`、`log/` 是本机生成目录，不提交 Git。它们不包含源代码版本事实，也不能跨不同 ROS2 发行版、Python 版本和机器环境稳定复用。Wuji 的源代码、URDF、mesh 与包内 Linux SDK 则必须提交。

### 6.2 运行顺序

1. MMHand 已连接 P4；目标关节已完成 Manual/Auto 标定并处于 READY。
2. 在第一个 Linux 终端启动 Wuji UI：

   ```bash
   source /opt/ros/jazzy/setup.bash
   source /path/to/mmhand-cl/wuji_hand_ws/install/setup.bash
   ros2 run wuji_mmhand_teleop wuji_raw_ik_teleop_ui
   ```

3. 在第二个 Linux 终端启动 MMHand 上位机 ROS2 接收器：

   ```bash
   source /opt/ros/jazzy/setup.bash
   cd /path/to/mmhand-cl
   python -m desktop ui --mode monitor --port /dev/ttyUSB0 \
     --ros2 \
     --ros2-input raw_ik \
     --ros2-raw-ik-topic /raw_ik_target \
     --ros2-raw-ik-space urdf_deg
   ```

4. 在 Wuji UI 内选择手套侧别、SN 和置信度，先完成四组姿态标定，再点击 `Send`。

### 6.3 常用诊断命令

```bash
ros2 node list | grep wuji
ros2 topic list | grep -E 'raw_ik_target|wuji_calibration'
ros2 topic info /raw_ik_target -v
ros2 topic hz /raw_ik_target
ros2 topic echo /raw_ik_target --once
ros2 topic echo /wuji_calibration_status
```

正常时应看到：

- 节点 `/wuji_raw_ik_target_sender`；
- `/raw_ik_target` 的 `data` 长度为 21；
- `layout.dim[0].label` 含 `urdf_deg` 与 `structure_urdf_v2`；
- 状态话题出现 `Wuji raw_ik_target valid 21/21`；
- MMHand 上位机已以 `urdf_deg` 模式启动。

## 7. 变更 URDF 时的固定流程

以后替换 URDF，不能只复制 `hand.urdf`。必须按以下顺序更新：

1. 替换 `hand.urdf` 与引用的 mesh。
2. 更新 `urdf_contract.json`：21 个关节名称、角度范围、URDF SHA256、来源版本和 CRC。
3. 执行或更新 `urdf_contract.py` 的校验测试。
4. 检查 `robot_joint_map.py` 是否仍可将全部语义关节映射到 J00-J20。
5. 检查并更新四组姿态标定参考值。
6. 强制重新采集 Wuji 四组姿态标定，不能复用旧模型数据。
7. 在 Linux 重建工作区，运行 `python3 -m pytest tests -q`。
8. 用 `ros2 topic echo /raw_ik_target --once` 验证 21 个值、单位和 `layout` 契约。

这样，URDF、Wuji IK、ROS2 消息和 MMHand 上位机才能继续使用同一套角度语义。
