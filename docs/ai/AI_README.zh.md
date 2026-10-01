# MMHand AI 代码导览

> mmhand `65e8e63f8b570c6b099c8b65dd37d796dcb66bb9` · mmhand_teleop `45eb698ba97c2559cad93db159a15877f93e8851` · branch `main`.

## 一、项目概览与本次变更

桌面上位机是操作入口：解析 `ui` 参数，启动 Qt 界面，连接 P4 主串口；也可按参数启动 ROS2 输入桥和 S3 触觉串口。它把人的选择和外部目标转换成控制端能消费的命令。 操作者通过上位机界面选择设备、控制模式和动作。点击只会启动上位机中的处理流程；连接状态、当前模式和安全检查仍可能阻止命令继续。 [425](#ref-0425) [329](#ref-0329)

对比基线：`65e8e63f8b570c6b099c8b65dd37d796dcb66bb9`。

本次改动涉及 **0** 个跟踪文件，其中 **0** 个计入代码行数。


覆盖检查：ai_explained_commands=15, ai_explained_edges=407, ai_explained_nodes=455, commands=25, p4_events=16, p4_explicit_transitions=10, p4_states=9, p4_tasks=7, packets=21, s3_tasks=3, safety_layers=7, safety_rules=103, ui_binding_files=11, ui_bindings=48, unverified_nodes=103。

[完整覆盖报告](coverage_report.json)。未核实规则的触发条件须沿图中的源码调用点人工核对。

生成器只读检查：

- `upper_protocol --check`: **通过**. Upper protocol generated artifacts are current.
- `robot_model --check`: **需修复**. generated robot-model file is stale: C:\Users\73547\mmhand\desktop\resources\config\product_fingerprint.json generated robot-model file is stale: C:\Users\73547\mmhand\ESP32-P4\ServoBoardMain\src\generated\ProductFingerprint.generated.h
- `p4_safety_catalog --check`: **通过**.

## 二、安装、运行、操作与维护

### 安装与启动 [371](#ref-0371) [329](#ref-0329) [431](#ref-0431)

在目标 Python 环境安装源码列出的桌面端依赖：

```powershell
python -m pip install -r desktop/requirements.txt
```

```powershell
python -m desktop ui --mode monitor --no-ros2
python -m desktop ui --mode replay --no-ros2
python -m desktop cli --help
```

监视界面的 Manual 页包含 Device Check、Joint Channels、Motor Channels 和 Test 子页；顶部 Mode 菜单还可进入 Replay。先连接 P4，再在 Device Check 检查或录入 Manual 端点；Joint Channels 使用磁编负反馈，Motor Channels 发送舵机 logical ABS 直控。ENABLE、Pause、Stop、Reset 与 Calibrate 的具体门禁及状态效果请打开第三节的交互图逐节点核对。[431](#ref-0431)

### ROS2 遥操 [746](#ref-0746) [747](#ref-0747) [745](#ref-0745) [409](#ref-0409)

遥操源码仓库：[mmhand_teleop](https://github.com/mm-hand/mmhand_teleop)。在其根目录执行 `python track.py --ros`：`RosOutput` 将 21 个 J00–J20 角度（degree）作为 `std_msgs/msg/Float32MultiArray` 发布到 `/raw_ik_target`。上位机需显式选择 raw IK 输入：

```powershell
python -m desktop ui --mode monitor --ros2 --ros2-input raw_ik --ros2-raw-ik-topic /raw_ik_target
ros2 topic echo --once /raw_ik_target
```

在 Joint Channels 选择活动范围并完成 ENABLE 后，开启 ROS2 INPUT 授权；输入仍经过目标格式、工作区和 P4 状态门禁。Pause、Stop、范围改变和 Replay 会撤销授权；具体判断式见行为图。[356](#ref-0356)

### 更新 URDF 与生成物 [731](#ref-0731) [719](#ref-0719) [720](#ref-0720) [717](#ref-0717)

结构仓库更新入口为 `tools/update_structure_urdf.ps1 -StructureRepo <路径> -NoPull -SkipP4Build`；脚本同步适配、校验 URDF、生成桌面/P4 模型契约并检查陈旧。直接修改模型清单或活动 URDF 后，运行 `python tools/generate_robot_model.py`，再运行同一命令加 `--check`。修改 `desktop/resources/config/upper_protocol_schema.json` 后运行 `python tools/generate_upper_protocol.py` 及其 `--check`；修改 P4 安全规则目录后运行 `python tools/generate_p4_safety_catalog.py` 及其 `--check`。生成文件应改上游输入后重生成，不要手工改生成物。

### 配置位置

下列条目从已提交配置源码及字段提取；改变值前请查看读取方与对应生成器：

- `ESP32-P4/ServoBoardMain/src/config/BoardConfig.h` — MMHAND_BOARD_CONFIG_H, TWAI_TX_PIN, TWAI_RX_PIN, kHostUartBaud [28](#ref-0028)
- `ESP32-P4/ServoBoardMain/src/config/CalibrationConfig.h` — MMHAND_CALIBRATION_CONFIG_H, kFeedbackReadyServoTimeoutMs, kEncoderTrustedSampleMaxAgeMs, kInitialFeedbackWaitMs [29](#ref-0029)
- `ESP32-P4/ServoBoardMain/src/config/Config.h` — MMHAND_CONFIG_H [30](#ref-0030)
- `ESP32-P4/ServoBoardMain/src/config/ControlConfig.h` — MMHAND_CONTROL_CONFIG_H, kPeriodSec, kEncoderModulo, kEncoderDisconnectRaw [31](#ref-0031)
- `ESP32-P4/ServoBoardMain/src/config/DiagnosticsConfig.h` — MMHAND_DIAGNOSTICS_CONFIG_H, kServoLoopGapLogThresholdMs, kServoApplyStallLogThresholdMs, kServoQueueDropLogIntervalMs [32](#ref-0032)
- `ESP32-P4/ServoBoardMain/src/config/DimensionsConfig.h` — MMHAND_DIMENSIONS_CONFIG_H, ENCODER_TOTAL_NUM, SERVO_TOTAL_NUM, JOINT_COUNT [33](#ref-0033)
- `ESP32-P4/ServoBoardMain/src/config/SafetyConfig.h` — MMHAND_SAFETY_CONFIG_H, kServoHardwareCommandMin, kServoHardwareCommandMax, kServoResetZeroTriggerMin [34](#ref-0034)
- `ESP32-P4/ServoBoardMain/src/config/ServoConfig.h` — MMHAND_SERVO_CONFIG_H, SERVO_SYNC_READ_TIMEOUT_MS_DEFAULT, SERVO_TARGET_SPEED_DEFAULT, SERVO_TARGET_ACC_DEFAULT [35](#ref-0035)
- `ESP32-P4/ServoBoardMain/src/config/TaskConfig.h` — MMHAND_TASK_CONFIG_H, TASK_HOST_COMM_PRIORITY, TASK_SYSTEM_STATE_PRIORITY, TASK_S3_CAN_RX_PRIORITY [36](#ref-0036)
- `ESP32-P4/ServoBoardMain/src/config/protocol/S3SensorCanProtocol.h` — P4_CONFIG_PROTOCOL_SENSOR_CAN_PROTOCOL_H, CAN_ID_ENC_BASE, CAN_ID_ENC_LAST, CAN_ID_ENC_CRC [37](#ref-0037)
- `ESP32-P4/ServoBoardMain/src/config/safety/SafetyRuleCatalog.def` — 需查看字段 [38](#ref-0038)
- `ESP32-P4/ServoBoardMain/src/config/safety/ServoPositionRange.h` — P4_CONFIG_SAFETY_SERVO_POSITION_RANGE_H, minValue, maxValue, hardwareAbs [39](#ref-0039)
- `ESP32-P4/ServoBoardMain/src/config/topology/HardwareMap.cpp` — ACT_NONE, ACT_PRIMARY, ACT_SECONDARY_OPPOSE, GROUP1 [40](#ref-0040)
- `ESP32-P4/ServoBoardMain/src/config/topology/HardwareMap.h` — HARDWARE_MAP_H, kThumbBaseJointIndex [41](#ref-0041)
- `ESP32-S3/System/src/config/Config.h` — CONFIG_H, ENCODER_TOTAL_NUM, TACTILE_GROUP_NUM, ENABLE_TACTILE_TASK [303](#ref-0303)
- `desktop/resources/config/p4_safety_rules.json` — schema_version, description, rules, rule_id [373](#ref-0373)
- `desktop/resources/config/product_fingerprint.json` — schema_version, model_id, mapping_id, feedback_mapping_version [374](#ref-0374)
- `desktop/resources/config/robot_model_manifest.json` — schema_version, model_id, mapping_id, feedback_mapping_version [375](#ref-0375)
- `desktop/resources/config/upper_protocol_schema.json` — schema_version, description, maintenance_note, frame [376](#ref-0376)
- `desktop/resources/config/urdf_joint_mapping.json` — schema_version, model_id, mapping_id, feedback_mapping_version [377](#ref-0377)
- `mmhand_teleop/config.py` — URDF_PATH, WILOR_ASSET_DIR, INPUT_SOURCE, STANDARD_PALM_SIZE, NORMALIZE_INPUT_HAND, CAMERA_DEVICE [745](#ref-0745)


### 安全策略位置与集中审计 [38](#ref-0038) [717](#ref-0717) [360](#ref-0360) [724](#ref-0724) [58](#ref-0058)

`SafetyRuleCatalog.def` 是规则编号、层级、动作和恢复建议的维护源。运行 `python tools/generate_p4_safety_catalog.py` 会同步生成上位机展示目录 `desktop/resources/config/p4_safety_rules.json`、固件枚举头 `SafetyRuleCatalog.generated.h`，以及供离线审计的 `SafetyRuleIndex.generated.json`。后者逐条给出 Policy 判断入口、业务调用、事件上报和恢复入口；上位机通过 `desktop/core/safety_rules.py` 按 rule_id 读取展示目录。生成文件应从目录重生成，不应手工修改。[38](#ref-0038) [717](#ref-0717) [360](#ref-0360) [58](#ref-0058)

`python tools/generate_p4_safety_catalog.py --check` 只核对三份生成物是否与目录一致；`python tools/p4_architecture/check_safety_rule_closure.py --json` 把注册、Policy、调用、映射、事件和恢复的闭环检查输出为机器可读 JSON，便于集中查看或重定向保存。闭环入口存在不等于每条规则的触发谓词已被证明；行为图保留“未核实”标记并链接具体源码。[717](#ref-0717) [724](#ref-0724)


### 逐条命令使用说明

#### `python -m pip install -r desktop/requirements.txt` [371](#ref-0371)

执行目录：`mmhand`。

**用途与用法：** 从 mmhand 仓库根目录安装上位机 Python 依赖。`-r` 让 pip 逐行读取 `desktop/requirements.txt`；其中列出串口、数值计算、绘图和 Qt 界面等运行库。

**前提与参数：** 在仓库根目录执行，并使用能运行项目 Python 的环境。安装失败通常意味着网络、Python 环境或某个包版本约束未满足；静态源码不能证明特定电脑上的安装结果。

**源码中的结果：** pip 读取依赖清单并安装/升级满足约束的包；这一步不启动上位机、不连接 P4，也不写入固件。

**失败或拒绝：** 命令返回非零时，后续 `python -m desktop ...` 可能因模块缺失而无法启动。逐项检查 pip 输出和 Python 环境；具体网络或系统依赖问题未核实。

**与相近命令的区别：** 这是运行前的依赖准备；它不同于启动 `ui` 或 `cli`，不会建立设备连接。

**处理方式：** 激活正确的 Python 环境后重试；若失败来自环境或网络，先修复原因，再重跑安装。

**未核实：** 源码仅列出包名和最低版本，未证明所有操作系统/硬件的安装兼容性。

**本次改动：** 本次比较基线未标记此命令所依赖内容发生变化。

#### `python -m desktop ui --mode monitor --no-ros2` [329](#ref-0329) [431](#ref-0431)

执行目录：`mmhand`。

**用途与用法：** 从仓库根目录启动 Qt 上位机的 monitor 模式，并明确关闭 ROS2 输入桥。`--mode monitor` 选择实时监控/控制 UI；`--no-ros2` 覆盖入口默认开启 ROS2 的设置。

**前提与参数：** 需要已安装 `desktop/requirements.txt` 中依赖。可用 `--port` 指定 P4 串口、`--tactile-port` 指定可选 S3 触觉串口；本命令未显式传入端口，实际自动选择或连接结果未核实。

**源码中的结果：** `entry.py` 将参数交给 `run_ui(mode="monitor", ros2_enabled=False, ...)`。这启动 UI 路径而非回放文件路径；不会因 `--no-ros2` 自动建立或验证硬件连接。

**失败或拒绝：** 端口打不开、设备未连接或 UI 依赖缺失时的具体提示由运行环境决定；源码静态分析不能证明该机器上的连接成功。

**与相近命令的区别：** 与 replay 模式相比，它进入实时设备工作流；与 ROS2 命令相比，它明确禁用 ROS2 bridge。

**处理方式：** 先确认 P4/S3 端口和依赖，再按需加 `--port COM...` 或 `--tactile-port COM...` 重启。若要使用 ROS 输入，去掉 `--no-ros2` 并配置匹配的输入参数。

**未核实：** 未传串口参数时设备选择规则以及实机状态未核实。

**本次改动：** 本次比较基线未标记入口参数路径变化。

#### `python -m desktop ui --mode replay --no-ros2` [329](#ref-0329) [367](#ref-0367)

执行目录：`mmhand`。

**用途与用法：** 启动 Qt 上位机的 replay 模式。这里的 `--no-ros2` 避免把实时 ROS2 输入桥带入回放流程；回放控制器维护帧索引、时间、目标值及关节掩码等状态。

**前提与参数：** 在仓库根目录且依赖已安装时运行。回放数据文件的选择方式不由此命令行参数表达；请在 UI/回放配置中提供有效记录，具体文件位置未由此证据确定。

**源码中的结果：** `entry.py` 将 `mode="replay"` 交给 `run_ui`；与 monitor 不同，后端采用 replay 状态/帧流程，而不是把命令解释为实时串口监控。

**失败或拒绝：** 无可读回放记录、数据格式错误或 UI 初始化失败时的表现取决于对应控制器和环境；此任务包没有证明某条具体错误消息。

**与相近命令的区别：** 与 monitor 的区别是回放时间轴和录制目标驱动显示/控制；此命令同样显式关闭 ROS2 bridge。

**处理方式：** 先在 UI 选择有效记录并确认状态；若要实时设备监控，改用 `--mode monitor`。

**未核实：** 支持的记录文件格式、当前默认文件及硬件回放安全性未核实。

**本次改动：** 本次比较基线未标记入口参数路径变化。

#### `python -m desktop cli --help` [329](#ref-0329)

执行目录：`mmhand`。

**用途与用法：** 运行终端版 monitor/control CLI 的帮助入口。`cli` 是 `desktop` 子命令；`--help` 由 argparse 显示该子命令可用参数，正常使用时可去掉 `--help` 并按需传串口、ROS2 等选项。

**前提与参数：** 从仓库根目录执行。帮助调用不要求先连接设备；实际 CLI 运行则需要对应 Python 依赖及所选设备/输入环境。

**源码中的结果：** 入口解析到 `command == "cli"` 后调用 `run_cli_monitor`；不是 Qt UI。`--no-plot` 是 CLI 的独有可选项。

**失败或拒绝：** 若误把参数放到子命令之前或参数名不合法，argparse 会显示错误/帮助；运行期设备错误由 CLI/backend 处理。

**与相近命令的区别：** 与 `ui` 相比这是终端界面，可选关闭绘图；两者共用串口和 ROS2 参数解析。

**处理方式：** 先用 `python -m desktop cli --help` 查看参数；再运行 `python -m desktop cli`，使用 `--port` 等选项指定设备。

**未核实：** 帮助文本不证明特定设备连接、控制权限或动作执行成功。

**本次改动：** 本次比较基线未标记 CLI 分发路径变化。

#### `python track.py --ros` [747](#ref-0747) [746](#ref-0746) [745](#ref-0745)

执行目录：`mmhand_teleop`。

**用途与用法：** 在 `mmhand_teleop` 仓库运行手部跟踪程序并开启 ROS 发布路径。`--ros` 是布尔开关；源码把发布器类型设为 `Float32MultiArray`，默认机器人目标话题为 `/raw_ik_target`。

**前提与参数：** 先进入遥操仓库并具备其 ROS2 Python 依赖、ROS 环境及输入设备。若未加 `--ros`，这个参数路径不会要求 ROS 发布。

**源码中的结果：** 跟踪程序启动 ROS 发布器，把该仓库生成的目标数组发布到 `ROBOT_TOPIC`。上位机必须同时订阅同一 topic，并选择兼容的 `raw_ik` 输入模式。

**失败或拒绝：** 节点启动、摄像头/输入设备可用性、数组长度和实时发布频率不能仅由这三处源码引用保证；启动失败或消息无效时的实机表现未核实。

**与相近命令的区别：** 与上位机的 `--ros2-input raw_ik` 配套；它负责生产消息，上位机命令负责订阅并在用户授权后接收控制输入。

**处理方式：** 先检查 ROS 环境与 topic，再用 `ros2 topic echo --once /raw_ik_target` 确认是否出现消息；无消息时检查节点与 ROS domain 配置。

**未核实：** 此静态证据没有证明摄像头、IK 算法精度、实际消息值或硬件运动结果。

**本次改动：** 本次比较基线未标记该 teleop 参数/发布定义变化。

#### `python -m desktop ui --mode monitor --ros2 --ros2-input raw_ik --ros2-raw-ik-topic /raw_ik_target` [329](#ref-0329) [409](#ref-0409) [356](#ref-0356)

执行目录：`mmhand`。

**用途与用法：** 启动上位机实时 UI，订阅原始 IK 目标话题 `/raw_ik_target`。`--ros2` 开启桥接；`--ros2-input raw_ik` 选择原始 IK 路径；`--ros2-raw-ik-topic` 指定订阅 topic。

**前提与参数：** 先运行遥操发布节点并保证 ROS2 节点互相可发现。上位机入口默认也启用 ROS2，但命令显式写出该意图；另需连接 P4。原始 IK 输出需由上位机授权门控，不能把“收到 topic”误认为“已驱动电机”。

**源码中的结果：** 上位机创建 `Float32MultiArray` 订阅并把消息交给 raw-IK handler。授权函数在串口未连通或活动关节范围为空时拒绝 armed；通过检查后记录 scope 并等待有效 P4 状态。

**失败或拒绝：** topic/类型不匹配会导致没有可用输入；P4 未连接或 scope 为空时授权失败。数据校验、丢弃策略和实机执行效果需看各自处理函数/日志，不能仅凭启动命令断言成功。

**与相近命令的区别：** 与 `--no-ros2` monitor 命令相比，它添加 ROS订阅和 raw-IK 路径；与 `joint_target` 输入相比，选择的是外部原始 IK 角度路径。

**处理方式：** 确认遥操发布端话题与类型一致，连接 P4，在 UI 执行明确的 ROS2 输入授权；被拒绝时按返回原因检查串口状态和活动关节范围。

**未核实：** 命令未显式指定 `--ros2-raw-ik-space`，因此使用入口默认 `urdf_deg`；不同来源数据是否符合该坐标定义未核实。

**本次改动：** 本次比较基线未标记这些入口和授权路径变化。

#### `ros2 topic echo --once /raw_ik_target` [745](#ref-0745) [746](#ref-0746)

执行目录：`mmhand_teleop`。

**用途与用法：** 在遥操仓库的 ROS 环境中读取 `/raw_ik_target` 上一条消息后退出。`--once` 适合快速确认订阅端是否看见发布数据；它只是观察命令，不会向电机发命令。

**前提与参数：** 先启动 `python track.py --ros`，并让 CLI 与发布节点使用可互相发现的 ROS domain。topic 名必须与 `ROBOT_TOPIC` 一致。

**源码中的结果：** ROS2 CLI 创建一次性订阅并打印收到的 `Float32MultiArray`。源码指出发布器使用 `/raw_ik_target`，类型为数组。

**失败或拒绝：** 命令等待/无输出通常说明没有匹配发布者、节点发现不通或尚无消息；确切超时行为由 ROS CLI 版本决定，未核实。

**与相近命令的区别：** 用于诊断 publisher 到 topic 的链路；不验证上位机授权、P4 状态、关节映射或机械臂运动。

**处理方式：** 检查发布进程、ROS domain、topic 名和消息类型；再运行一次 echo。

**未核实：** 静态证据不确定消息频率和数组每一索引的关节含义。

**本次改动：** 本次比较基线未标记遥操 topic 定义变化。

#### `tools/update_structure_urdf.ps1 -StructureRepo <path> -NoPull -SkipP4Build` [731](#ref-0731)

执行目录：`mmhand`。

**用途与用法：** 从结构描述仓库同步 URDF 到当前项目，并用开关控制同步副作用。`-StructureRepo <path>` 指定结构仓库；`-NoPull` 跳过 git pull；`-SkipP4Build` 跳过 P4 固件构建。

**前提与参数：** 在 mmhand 根目录运行，并把 `<path>` 替换为含 `.git` 的结构仓库路径。脚本期望其中存在 `mmhand_description` 和 `desktop_urdf_adapter`。去掉 `-NoPull` 会尝试快进拉取和 LFS 拉取；因此执行前确认本地分支状态。

**源码中的结果：** 脚本同步结构源与 adapter，随后调用机器人模型生成器 `--check` 校验生成物；默认还会执行架构检查及 P4 构建，`-SkipP4Build` 只跳过构建阶段。

**失败或拒绝：** 仓库目录缺失、同步命令失败、模型校验或架构检查失败都会中止流程。跳过构建不等于验证了固件编译。

**与相近命令的区别：** 与直接运行 `generate_robot_model.py` 相比，此脚本处理上游 URDF 仓库同步及后续集成检查；`-NoPull` 控制是否更新上游，`-SkipP4Build` 控制是否编译。

**处理方式：** 核对结构仓库路径与 Git/LFS 状态，再重跑；若跳过构建，仅把结果当作同步/静态检查，之后仍需在适当环境编译 P4。

**未核实：** 脚本的完整文件复制清单和此次本机同步效果未核实。

**本次改动：** 本次比较基线未标记脚本参数变化。

#### `python tools/generate_robot_model.py` [719](#ref-0719)

执行目录：`mmhand`。

**用途与用法：** 从机器人模型源文件重新生成项目使用的模型文件。它是生成动作：如果生成器计算出的内容与当前文件不同，会更新输出文件。

**前提与参数：** 从仓库根目录运行，并确保生成器所需输入文件存在。具体哪些文件会变更应检查 `tools/generate_robot_model.py` 中的输出清单及 Git diff。

**源码中的结果：** `main()` 不带 `--check` 时执行生成逻辑，写入机器人模型生成物；不等同于从上游下载 URDF，也不构建固件。

**失败或拒绝：** 缺少输入、解析失败或输出不可写会返回错误；源代码片段未列出具体机器相关依赖。

**与相近命令的区别：** 与 `--check` 相比会写文件；与 URDF 同步脚本相比只负责本仓库模型生成，不负责拉取结构仓库或构建 P4。

**处理方式：** 先修复输入/格式问题，重跑生成，再审核 Git diff，确认生成输出可由预期源数据解释。

**未核实：** 命令本身不证明生成模型在实机或仿真中的正确性。

**本次改动：** 本次比较基线未标记生成器入口变化。

#### `python tools/generate_robot_model.py --check` [719](#ref-0719)

执行目录：`mmhand`。

**用途与用法：** 只检查机器人模型生成物是否过期，不执行更新。`--check` 让生成器比较当前文件与按源数据计算出的期望内容，并以退出状态供开发者/CI 判断。

**前提与参数：** 与生成命令相同的仓库根目录和输入文件条件。适合提交前检查；若检查失败，先确认是否因为源模型更新后忘记生成。

**源码中的结果：** 脚本计算期望输出并报告是否陈旧；按参数帮助说明，它只检查，不写生成文件。

**失败或拒绝：** 检查失败表示至少有生成物与预期不一致，不代表源文件一定错误；也可能只是尚未运行普通生成命令。

**与相近命令的区别：** 与普通生成命令的核心区别是只读校验，不修改文件；它不做固件构建。

**处理方式：** 先运行 `python tools/generate_robot_model.py` 更新，再查看 diff 并重跑 `--check`。

**未核实：** 退出码细节之外的本机运行结果未核实。

**本次改动：** 本次比较基线未标记检查逻辑变化。

#### `python tools/generate_upper_protocol.py` [720](#ref-0720)

执行目录：`mmhand`。

**用途与用法：** 从上位机通信协议定义重新生成协议相关代码或清单。普通模式是写入模式，适用于协议定义变更后更新衍生文件。

**前提与参数：** 在仓库根目录执行。运行后审阅生成文件的 Git diff，确认命令 ID、字段布局和对应代码变化都来自协议定义。

**源码中的结果：** `main()` 不带 `--check` 执行生成逻辑；具体输出由脚本中的生成目标确定。此命令不自动证明 P4 与桌面端版本已经同时部署。

**失败或拒绝：** 协议定义错误、生成器校验失败或写文件失败会返回错误；不能仅凭成功退出推断固件和上位机互通。

**与相近命令的区别：** 与 `--check` 相比普通模式会更新生成物；与安全目录生成器相比它处理通信协议产物。

**处理方式：** 修正定义或生成错误后重跑，并运行 `--check`；变更通信帧后还需审查两端解析/编码路径。

**未核实：** 实际设备兼容性和双方部署版本未核实。

**本次改动：** 本次比较基线未标记此入口变化。

#### `python tools/generate_upper_protocol.py --check` [720](#ref-0720)

执行目录：`mmhand`。

**用途与用法：** 只检查通信协议生成物是否与协议定义一致/陈旧，不重写生成文件。用于确认提交前没有漏掉协议生成步骤。

**前提与参数：** 在仓库根目录运行，确保协议源定义可读取。

**源码中的结果：** 生成器 `--check` 分支仅报告文件是否最新，并通过退出码表达检查结果。

**失败或拒绝：** 若检查失败，生成物可能陈旧或缺失；不能直接判断协议定义本身有错。

**与相近命令的区别：** 与普通协议生成相比只读、不写文件；不测试串口链路或真实设备协议兼容性。

**处理方式：** 先运行 `python tools/generate_upper_protocol.py`，审查 Git diff，再执行本命令确认。

**未核实：** 此检查的本机结果未核实。

**本次改动：** 本次比较基线未标记此检查逻辑变化。

#### `python tools/generate_p4_safety_catalog.py` [717](#ref-0717)

执行目录：`mmhand`。

**用途与用法：** 根据唯一维护源安全规则目录生成桌面端和 P4 固件使用的规则文件及集中索引。安全策略源变更后运行普通模式同步衍生文件。

**前提与参数：** 从仓库根目录运行。生成后检查 `safety/policies`、固件 generated 文件和 SafetyRuleIndex 的 diff；不要手改生成文件，修改维护源再重生成。

**源码中的结果：** 普通模式生成/写入规则目录、固件头文件和集中审计索引；这是代码生成，不是安全谓词的正确性证明。

**失败或拒绝：** 解析或一致性校验失败应阻止可信发布；具体错误由脚本打印。即使生成成功，也需在调用点独立核实触发条件。

**与相近命令的区别：** 与 `--check` 相比会写产物；与闭环审计相比负责生成文件，不负责完整性审计本身。

**处理方式：** 修正 `SafetyRuleCatalog.def` 等维护源后重跑，再运行 `--check` 和安全闭环 JSON 审计。

**未核实：** 索引闭环完整不代表每条运行时保护条件都正确或覆盖所有故障。

**本次改动：** 本次比较基线未标记该生成入口变化。

#### `python tools/generate_p4_safety_catalog.py --check` [717](#ref-0717)

执行目录：`mmhand`。

**用途与用法：** 只检查安全规则生成文件是否与维护源同步，不修改文件。适合确认规则目录、固件头文件及索引没有落后于 `SafetyRuleCatalog.def`。

**前提与参数：** 从仓库根目录执行；适用于提交前检查。

**源码中的结果：** 脚本在 `--check` 模式比较期望内容与生成文件并报告陈旧项。

**失败或拒绝：** 发现不一致说明需要审查并重新生成，不等价于安全规则运行行为检查。

**与相近命令的区别：** 只读文件一致性检查，不做规则调用闭环审计，也不验证谓词在运行中会否触发。

**处理方式：** 运行 `python tools/generate_p4_safety_catalog.py`，查看受影响生成文件的 diff，再复查 `--check`。

**未核实：** 不包含实机故障注入或覆盖率证据。

**本次改动：** 本次比较基线未标记检查逻辑变化。

#### `python tools/p4_architecture/check_safety_rule_closure.py --json` [724](#ref-0724) [58](#ref-0058)

执行目录：`mmhand`。

**用途与用法：** 导出安全规则闭环审计 JSON。脚本从生成索引核对每条规则是否能关联策略实现、业务调用方、事件上报及恢复入口；`--json` 选择机器可读输出。

**前提与参数：** 在仓库根目录运行。需先有最新安全索引；若源规则刚变更，先运行安全目录生成器。

**源码中的结果：** `audit()` 读取集中索引并检查各引用目标/符号，输出 JSON；当前静态检查曾记录 103/103 条闭环入口齐全，但该数字须与本次 commit 的输出对应，不能沿用为永恒事实。

**失败或拒绝：** 不完整映射会输出失败条目并以非零状态结束；通过只说明索引链路齐全，不证明触发谓词、调用覆盖和恢复动作在运行时正确。

**与相近命令的区别：** 与安全生成器相比这是只读审计/导出；与 `--check` 相比它关注跨文件闭环关系，而非仅比对文件新旧。

**处理方式：** 查看 JSON 中失败规则，补齐策略、调用、事件或恢复引用；重生成索引后重新导出并复核。

**未核实：** 闭环审计不进行硬件试验，也不证明规则没有误报或漏报。

**本次改动：** 本次比较基线未标记审计入口变化。


## 三、操作、状态与保护交互图

[操作、状态与保护交互图](behavior.zh.html)

## 四、系统架构与数据流图

[系统架构与数据流图](architecture.zh.html)

## 五、控制公式

以下式子按当前源码的计算顺序展开；输入异常、死区、限幅及模式准入仍以对应分支为准。

### 单腱离散 PID [212](#ref-0212)

$$
\begin{aligned}
e_k &= r_k-y_k \\
v_k &= v_{k-1}+\frac{\Delta t_k}{\max(0,\tau)+\Delta t_k}
       \left(\frac{y_k-y_{k-1}}{\Delta t_k}-v_{k-1}\right) \\
I_k &= \begin{cases}
\operatorname{clip}(I_{k-1}+e_k\Delta t_k,-I_{\max},I_{\max}),& |e_k|\geq|b|,\ K_i\neq0 \\
0,& |e_k|\geq|b|,\ K_i=0 \\
I_{k-1},& |e_k|<|b|
\end{cases} \\
u_k &= \begin{cases}
K_p e_k+K_i I_k+\operatorname{clip}(-K_dv_k,-D_{\max},D_{\max}),& |e_k|\geq|b| \\
u_{k-1},& |e_k|<|b|\text{ or }\Delta t_k\leq0
\end{cases}
\end{aligned}
$$

`dtSeconds` 无效时保持上次输出；非有限输入会清零并报错；死区内保持输出。测量速度用于 D 项，首次样本没有 D 项。

### 双腱拮抗控制 [200](#ref-0200)

$$
\begin{aligned}
e_{i,k} &= R_{i,k}-F_{i,k} \\
\alpha_k &= \frac{\Delta t_k}{\tau+\Delta t_k} \\
V_{i,k} &= V_{i,k-1}+\alpha_k\left(\frac{\Delta R_{i,k}-\Delta F_{i,k}}{\Delta t_k}-V_{i,k-1}\right) \\
I_{i,k} &= \operatorname{clip}(I_{i,k-1}+e_{i,k}\Delta t_k,-I_{\max},I_{\max}) \\
C_{i,k} &= K_p e_{i,k}+K_i I_{i,k}+\operatorname{clip}(K_dV_{i,k},-D_{\max},D_{\max}) \\
A_{i,k} &= \operatorname{clip}_{[L_i^-,L_i^+]}\left(A^{current}_{i,k}+\operatorname{round}(C_{i,k})\right),\quad i\in\{0,1\}
\end{aligned}
$$

每侧电机先映射到标定电机空间，再计算误差；两台电机任一无效或离线时该求解失败。结果随后按 LUT 裕量边界截取。

## 六、通信指令表

命令编号、payload 长度及说明直接取协议 schema；实际准入仍由 P4 处理代码决定。 [376](#ref-0376)

| Direction | Command | ID | Payload | Description |
| --- | --- | ---: | ---: | --- |
| Desktop→P4 | `CMD_CALIBRATE` | 202 | 0 B | 按当前 active scope 启动自动标定 [376](#ref-0376) |
| Desktop→P4 | `CMD_ANGLE_CTRL` | 203 | 84 B | 下发完整 21 路关节目标角，单位 degree [376](#ref-0376) |
| Desktop→P4 | `CMD_START` | 204 | 0 B | 历史启动命令，进入控制申请流程 [376](#ref-0376) |
| Desktop→P4 | `CMD_STOP` | 205 | 0 B | 停止当前运行并撤销待执行目标 [376](#ref-0376) |
| Desktop→P4 | `CMD_RESET` | 206 | 0 B | 清除允许恢复的系统故障锁存 [376](#ref-0376) |
| Desktop→P4 | `CMD_MOTOR_POS_ABS` | 211 | 108 B | 27 路 logical ABS 直控目标，每路 int32 counts [376](#ref-0376) |
| Desktop→P4 | `CMD_ACTIVE_SCOPE` | 215 | 9 B | 同步 active joint/motor mask 与 control enable [376](#ref-0376) |
| Desktop→P4 | `CMD_RUN_FEEDBACK` | 216 | 0 B | 申请状态机进入负反馈运行 [376](#ref-0376) |
| Desktop→P4 | `CMD_PAUSE` | 217 | 0 B | 暂停当前负反馈或直控运行 [376](#ref-0376) |
| Desktop→P4 | `CMD_RESTORE_MULTITURN` | 218 | 112 B | 按位图恢复 27 路 logical ABS 多圈快照 [376](#ref-0376) |
| Desktop→P4 | `CMD_HOST_HEARTBEAT` | 220 | 0 B | 刷新上位机在线计时器，不携带业务数据 [376](#ref-0376) |
| Desktop→P4 | `CMD_MOTOR_POS_ABS_ONE` | 222 | 5 B | 单台 Mxx logical ABS 直控，Mxx 加 int32 counts [376](#ref-0376) |
| Desktop→P4 | `CMD_CALIB_LOAD_JOINT` | 223 | 189 B | 恢复单关节唯一 Auto/LUT v4 标定快照 [376](#ref-0376) |
| Desktop→P4 | `CMD_CALIB_LOAD_LIMITS` | 224 | 224 B | 批量恢复 27 路 ABS 标定范围与有效位图 [376](#ref-0376) |
| Desktop→P4 | `CMD_CLEAR_CALIBRATION` | 229 | 0 B | 清除全部 Auto/LUT 标定结果，保留 Manual [376](#ref-0376) |
| Desktop→P4 | `CMD_MANUAL_ENCODER_REFERENCE` | 230 | 85 B | 写入或清除 21 组 Manual Min/Max 原始磁编值 [376](#ref-0376) |
| Desktop→P4 | `CMD_MANUAL_ENCODER_QUERY` | 231 | 0 B | 请求 P4 立即回传 Manual 标定表 [376](#ref-0376) |
| Desktop→P4 | `CMD_MANUAL_ENCODER_SINGLE` | 232 | 5 B | 写入单个 Jxx 的 Manual Min/Max [376](#ref-0376) |
| Desktop→P4 | `CMD_ROPE_INIT` | 233 | 4 B | 按 21 位关节掩码启动 P4 Rope Init [376](#ref-0376) |
| Desktop→P4 | `CMD_ANGLE_CTRL_MASKED` | 234 | 88 B | 按关节掩码下发稀疏关节目标角 [376](#ref-0376) |
| Desktop→P4 | `CMD_CLEAR_CALIBRATION_SCOPE` | 235 | 4 B | 按电机位图清除关联关节 Auto/LUT [376](#ref-0376) |
| Desktop→P4 | `CMD_SERVO_RESET_ZERO` | 236 | 4 B | 标定前归零所选舵机硬件圈数并清除对应多圈锚点 [376](#ref-0376) |
| Desktop→P4 | `CMD_STATE_SYNC_REQUEST` | 237 | 4 B | 按位请求静态状态、标定快照、路由和产品指纹 [376](#ref-0376) |
| Desktop→P4 | `CMD_TASK_DIAG_SUBSCRIBE` | 238 | 1 B | 打开或关闭 RTOS 任务诊断按需上报 [376](#ref-0376) |
| Desktop→P4 | `CMD_CONTROL_TRACE_SUBSCRIBE` | 239 | 1 B | 打开或关闭统一下位机时间的控制跟踪流 [376](#ref-0376) |

| Direction | Packet | ID | Description |
| --- | --- | ---: | --- |
| P4→Desktop | `PACKET_TYPE_SENSOR` | 1 | 21 关节映射角 [376](#ref-0376) |
| P4→Desktop | `PACKET_TYPE_SERVO_ANGLE` | 3 | 27 路 logical ABS [376](#ref-0376) |
| P4→Desktop | `PACKET_TYPE_SERVO_TELEM` | 5 | 舵机遥测 [376](#ref-0376) |
| P4→Desktop | `PACKET_TYPE_PROTO_ACK` | 6 | 命令 ACK [376](#ref-0376) |
| P4→Desktop | `PACKET_TYPE_FAULT_STATUS` | 7 | 故障与在线位图 [376](#ref-0376) |
| P4→Desktop | `PACKET_TYPE_SERVO_RAW` | 9 | 27 路单圈 raw [376](#ref-0376) |
| P4→Desktop | `PACKET_TYPE_TACTILE` | 10 | 五指触觉摘要 [376](#ref-0376) |
| P4→Desktop | `PACKET_TYPE_ENCODER_RAW` | 11 | 21 路磁编 raw [376](#ref-0376) |
| P4→Desktop | `PACKET_TYPE_SYSTEM_STATUS` | 12 | 状态机与 owner [376](#ref-0376) |
| P4→Desktop | `PACKET_TYPE_MOTOR_ABS_LIMIT` | 13 | 标定 ABS 边界 [376](#ref-0376) |
| P4→Desktop | `PACKET_TYPE_JOINT_CALIB_SNAPSHOT` | 14 | Auto/LUT 快照 [376](#ref-0376) |
| P4→Desktop | `PACKET_TYPE_TACTILE_FULL` | 15 | 全量触觉阵列 [376](#ref-0376) |
| P4→Desktop | `PACKET_TYPE_CALIB_DIAG` | 16 | 结构化标定诊断 [376](#ref-0376) |
| P4→Desktop | `PACKET_TYPE_TASK_DIAG` | 17 | RTOS 任务诊断 [376](#ref-0376) |
| P4→Desktop | `PACKET_TYPE_MANUAL_ENCODER_REFERENCE` | 18 | 21 路 Manual Min/Max [376](#ref-0376) |
| P4→Desktop | `PACKET_TYPE_SERVO_LOGICAL_RANGE` | 19 | 27 路会话 logical range [376](#ref-0376) |
| P4→Desktop | `PACKET_TYPE_PRODUCT_FINGERPRINT` | 20 | 产品配置指纹 [376](#ref-0376) |
| P4→Desktop | `PACKET_TYPE_SERVO_ROUTE_STATUS` | 21 | 动态舵机 BUS 路由、冲突、容量和扫描状态 [376](#ref-0376) |
| P4→Desktop | `PACKET_TYPE_SAFETY_EVENT` | 22 | 去重后的结构化安全规则事件 [376](#ref-0376) |
| P4→Desktop | `PACKET_TYPE_MULTITURN_RESTORE_RESULT` | 23 | 桌面 JSON 多圈恢复异步最终结果 [376](#ref-0376) |
| P4→Desktop | `PACKET_TYPE_CONTROL_TRACE` | 24 | 同一控制周期的目标角、反馈角、舵机CMD和下位机时间 [376](#ref-0376) |

## 七、逐文件简介与代码行数

项目源码、脚本及配置共 **102387** 物理行；其中生成代码 **4196** 行，第三方代码 **6419** 行，IDE 工具配置 **295** 行。空行和注释计入。

### 目录汇总

- `.` — 102387
- `.vscode` — 226
- `ESP32-P4` — 35662
- `ESP32-P4/.idea` — 24
- `ESP32-P4/.idea/inspectionProfiles` — 6
- `ESP32-P4/.vscode` — 45
- `ESP32-P4/ServoBoardMain` — 32385
- `ESP32-P4/ServoBoardMain/src` — 32369
- `ESP32-P4/ServoBoardMain/src/app` — 299
- `ESP32-P4/ServoBoardMain/src/config` — 929
- `ESP32-P4/ServoBoardMain/src/config/protocol` — 58
- `ESP32-P4/ServoBoardMain/src/config/safety` — 179
- `ESP32-P4/ServoBoardMain/src/config/topology` — 224
- `ESP32-P4/ServoBoardMain/src/data` — 622
- `ESP32-P4/ServoBoardMain/src/generated` — 2627
- `ESP32-P4/ServoBoardMain/src/hal` — 1520
- `ESP32-P4/ServoBoardMain/src/hal/can` — 178
- `ESP32-P4/ServoBoardMain/src/hal/serial` — 89
- `ESP32-P4/ServoBoardMain/src/hal/servo` — 1253
- `ESP32-P4/ServoBoardMain/src/modules` — 1507
- `ESP32-P4/ServoBoardMain/src/modules/actuator` — 158
- `ESP32-P4/ServoBoardMain/src/modules/calibration` — 1188
- `ESP32-P4/ServoBoardMain/src/modules/integrity` — 94
- `ESP32-P4/ServoBoardMain/src/modules/protocol` — 14
- `ESP32-P4/ServoBoardMain/src/modules/runtime` — 53
- `ESP32-P4/ServoBoardMain/src/platform` — 571
- `ESP32-P4/ServoBoardMain/src/platform/log` — 138
- `ESP32-P4/ServoBoardMain/src/platform/rtos` — 402
- `ESP32-P4/ServoBoardMain/src/platform/time` — 31
- `ESP32-P4/ServoBoardMain/src/runtime` — 670
- `ESP32-P4/ServoBoardMain/src/runtime/adapters` — 160
- `ESP32-P4/ServoBoardMain/src/runtime/channels` — 40
- `ESP32-P4/ServoBoardMain/src/runtime/metrics` — 209
- `ESP32-P4/ServoBoardMain/src/runtime/ports` — 57
- `ESP32-P4/ServoBoardMain/src/runtime/state` — 178
- `ESP32-P4/ServoBoardMain/src/safety` — 3212
- `ESP32-P4/ServoBoardMain/src/safety/policies` — 2598
- `ESP32-P4/ServoBoardMain/src/tasks` — 20412
- `ESP32-P4/ServoBoardMain/src/tasks/auto_calibration` — 6471
- `ESP32-P4/ServoBoardMain/src/tasks/auto_calibration/dual_motor` — 2442
- `ESP32-P4/ServoBoardMain/src/tasks/auto_calibration/single_motor` — 1292
- `ESP32-P4/ServoBoardMain/src/tasks/host_comm` — 3868
- `ESP32-P4/ServoBoardMain/src/tasks/motion_control` — 3326
- `ESP32-P4/ServoBoardMain/src/tasks/runtime_monitor` — 604
- `ESP32-P4/ServoBoardMain/src/tasks/s3_can_rx` — 974
- `ESP32-P4/ServoBoardMain/src/tasks/servo_bus` — 2558
- `ESP32-P4/ServoBoardMain/src/tasks/system_state` — 2611
- `ESP32-P4/libraries` — 3059
- `ESP32-P4/libraries/FTServo_Arduino` — 3059
- `ESP32-P4/libraries/FTServo_Arduino/examples` — 1323
- `ESP32-P4/libraries/FTServo_Arduino/examples/HLSCL` — 503
- `ESP32-P4/libraries/FTServo_Arduino/examples/HLSCL/Broadcast` — 22
- `ESP32-P4/libraries/FTServo_Arduino/examples/HLSCL/CalibrationOfs` — 28
- `ESP32-P4/libraries/FTServo_Arduino/examples/HLSCL/FeedBack` — 148
- `ESP32-P4/libraries/FTServo_Arduino/examples/HLSCL/Ping` — 34
- `ESP32-P4/libraries/FTServo_Arduino/examples/HLSCL/ProgramEprom` — 27
- `ESP32-P4/libraries/FTServo_Arduino/examples/HLSCL/RegWritePos` — 26
- `ESP32-P4/libraries/FTServo_Arduino/examples/HLSCL/SyncRead` — 45
- `ESP32-P4/libraries/FTServo_Arduino/examples/HLSCL/SyncWritePos` — 39
- `ESP32-P4/libraries/FTServo_Arduino/examples/HLSCL/SyncWriteSpe` — 50
- `ESP32-P4/libraries/FTServo_Arduino/examples/HLSCL/WriteEle` — 31
- `ESP32-P4/libraries/FTServo_Arduino/examples/HLSCL/WritePos` — 22
- `ESP32-P4/libraries/FTServo_Arduino/examples/HLSCL/WriteSpe` — 31
- `ESP32-P4/libraries/FTServo_Arduino/examples/SCSCL` — 358
- `ESP32-P4/libraries/FTServo_Arduino/examples/SCSCL/Broadcast` — 25
- `ESP32-P4/libraries/FTServo_Arduino/examples/SCSCL/FeedBack` — 147
- `ESP32-P4/libraries/FTServo_Arduino/examples/SCSCL/Ping` — 33
- `ESP32-P4/libraries/FTServo_Arduino/examples/SCSCL/ProgramEprom` — 28
- `ESP32-P4/libraries/FTServo_Arduino/examples/SCSCL/RegWritePos` — 29
- `ESP32-P4/libraries/FTServo_Arduino/examples/SCSCL/SyncWritePos` — 37
- `ESP32-P4/libraries/FTServo_Arduino/examples/SCSCL/WritePWM` — 34
- `ESP32-P4/libraries/FTServo_Arduino/examples/SCSCL/WritePos` — 25
- `ESP32-P4/libraries/FTServo_Arduino/examples/SMS_STS` — 462
- `ESP32-P4/libraries/FTServo_Arduino/examples/SMS_STS/Broadcast` — 22
- `ESP32-P4/libraries/FTServo_Arduino/examples/SMS_STS/CalibrationOfs` — 25
- `ESP32-P4/libraries/FTServo_Arduino/examples/SMS_STS/FeedBack` — 148
- `ESP32-P4/libraries/FTServo_Arduino/examples/SMS_STS/Ping` — 33
- `ESP32-P4/libraries/FTServo_Arduino/examples/SMS_STS/ProgramEprom` — 27
- `ESP32-P4/libraries/FTServo_Arduino/examples/SMS_STS/RegWritePos` — 26
- `ESP32-P4/libraries/FTServo_Arduino/examples/SMS_STS/SyncRead` — 45
- `ESP32-P4/libraries/FTServo_Arduino/examples/SMS_STS/SyncWritePos` — 36
- `ESP32-P4/libraries/FTServo_Arduino/examples/SMS_STS/SyncWriteSpe` — 47
- `ESP32-P4/libraries/FTServo_Arduino/examples/SMS_STS/WritePos` — 22
- `ESP32-P4/libraries/FTServo_Arduino/examples/SMS_STS/WriteSpe` — 31
- `ESP32-P4/libraries/FTServo_Arduino/src` — 1736
- `ESP32-P4/tools` — 149
- `ESP32-S3` — 2582
- `ESP32-S3/System` — 2582
- `ESP32-S3/System/.theia` — 8
- `ESP32-S3/System/src` — 2509
- `ESP32-S3/System/src/config` — 153
- `ESP32-S3/System/src/hal` — 1698
- `ESP32-S3/System/src/system` — 107
- `ESP32-S3/System/src/tasks` — 551
- `desktop` — 36894
- `desktop/app` — 798
- `desktop/core` — 12906
- `desktop/core/communication` — 1264
- `desktop/core/data_recording` — 870
- `desktop/examples` — 64
- `desktop/replay` — 1297
- `desktop/resources` — 4424
- `desktop/resources/config` — 3545
- `desktop/resources/hku_hand_v2_urdf` — 879
- `desktop/resources/hku_hand_v2_urdf/urdf` — 850
- `desktop/ros2` — 341
- `desktop/tests` — 651
- `desktop/ui` — 12942
- `desktop/visualize` — 3407
- `manus` — 12963
- `manus/SDKClient_Windows` — 5444
- `manus/SDKClient_Windows/PlatformSpecific` — 1174
- `manus/SDKClient_Windows/PlatformSpecific/Linux` — 607
- `manus/SDKClient_Windows/PlatformSpecific/Windows` — 485
- `manus/SDKMinimalClient_Windows` — 464
- `manus/manus_bridge_test` — 1993
- `manus/manus_bridge_test/config` — 17
- `manus/manus_bridge_test/scripts` — 82
- `manus/manus_bridge_test/src` — 1833
- `manus/mmhand_manus_sdk` — 1702
- `manus/mmhand_manus_sdk/build` — 1247
- `manus/mmhand_manus_sdk/build/windows` — 1247
- `manus/mmhand_manus_sdk/build/windows/CMakeFiles` — 1191
- `manus/mmhand_manus_sdk/build/windows/CMakeFiles/3.31.6-msvc6` — 1041
- `manus/mmhand_manus_sdk/build/windows/CMakeFiles/3.31.6-msvc6/CompilerIdCXX` — 919
- `manus/mmhand_manus_sdk/include` — 32
- `manus/mmhand_manus_sdk/scripts` — 49
- `manus/mmhand_manus_sdk/src` — 342
- `manus/third_party` — 3360
- `manus/third_party/ManusSDK` — 3360
- `manus/third_party/ManusSDK/include` — 3360
- `playground` — 2256
- `playground/Standalone` — 292
- `playground/Standalone/paxini_all_read_newboard` — 292
- `playground/paxini_all_read_2.12` — 497
- `playground/paxini_dist_scan` — 251
- `playground/paxini_manual_cs_scan` — 246
- `playground/paxini_spi1_p1_test` — 193
- `playground/paxini_spi_scan` — 286
- `playground/tactile_sim_visualizer` — 199
- `tests` — 7976
- `tests/desktop` — 7684
- `tests/desktop/algorithm` — 653
- `tests/desktop/app` — 371
- `tests/desktop/core` — 4000
- `tests/desktop/ros2` — 163
- `tests/desktop/ui` — 2372
- `tests/desktop/visualize` — 125
- `tests/tools` — 286
- `tools` — 3788
- `tools/desktop_architecture` — 274
- `tools/p4_architecture` — 624
- `tools/p4_debug` — 194

### 逐文件

- `.vscode/c_cpp_properties.json` — 16 行 · tooling · c_cpp_properties：在 `.vscode` 中定义或处理 configurations, version。 [735](#ref-0735)
- `.vscode/launch.json` — 59 行 · tooling · launch：在 `.vscode` 中定义或处理 version, configurations。 [736](#ref-0736)
- `.vscode/settings.json` — 64 行 · tooling · settings：在 `.vscode` 中定义或处理 cmake.ignoreCMakeListsMissing, python-envs.defaultEnvManager, python-envs.defaultPackageManager, C_Cpp_Runner.cCompilerPath。 [737](#ref-0737)
- `.vscode/tasks.json` — 87 行 · tooling · tasks：在 `.vscode` 中定义或处理 version, tasks, inputs。 [738](#ref-0738)
- `ESP32-P4/.idea/inspectionProfiles/profiles_settings.xml` — 6 行 · tooling · profiles_settings：位于 `ESP32-P4/.idea/inspectionProfiles`，具体职责需沿源码核对。 [13](#ref-0013)
- `ESP32-P4/.idea/misc.xml` — 4 行 · tooling · misc：位于 `ESP32-P4/.idea`，具体职责需沿源码核对。 [14](#ref-0014)
- `ESP32-P4/.idea/modules.xml` — 8 行 · tooling · modules：位于 `ESP32-P4/.idea`，具体职责需沿源码核对。 [15](#ref-0015)
- `ESP32-P4/.idea/vcs.xml` — 6 行 · tooling · vcs：位于 `ESP32-P4/.idea`，具体职责需沿源码核对。 [16](#ref-0016)
- `ESP32-P4/.vscode/settings.json` — 8 行 · tooling · settings：在 `ESP32-P4/.vscode` 中定义或处理 idf.currentSetup, files.encoding, files.autoGuessEncoding, task.allowAutomaticTasks。 [739](#ref-0739)
- `ESP32-P4/.vscode/tasks.json` — 37 行 · tooling · tasks：在 `ESP32-P4/.vscode` 中定义或处理 version, tasks。 [740](#ref-0740)
- `ESP32-P4/ServoBoardMain/ServoBoardMain.ino` — 16 行 · source · ServoBoardMain：在 `ESP32-P4/ServoBoardMain` 中定义或处理 setup, loop。 [20](#ref-0020)
- `ESP32-P4/ServoBoardMain/src/app/AppBootstrap.cpp` — 40 行 · source · AppBootstrap：在 `ESP32-P4/ServoBoardMain/src/app` 中定义或处理 appBootstrapSetup, appBootstrapLoop。 [24](#ref-0024)
- `ESP32-P4/ServoBoardMain/src/app/AppBootstrap.h` — 19 行 · source · AppBootstrap：在 `ESP32-P4/ServoBoardMain/src/app` 中定义或处理 appBootstrapSetup, appBootstrapLoop。 [25](#ref-0025)
- `ESP32-P4/ServoBoardMain/src/app/SystemAssembly.cpp` — 228 行 · source · SystemAssembly：在 `ESP32-P4/ServoBoardMain/src/app` 中定义或处理 recordAssemblyFailure, assembleAndStartSystem。 [26](#ref-0026)
- `ESP32-P4/ServoBoardMain/src/app/SystemAssembly.h` — 12 行 · source · SystemAssembly：在 `ESP32-P4/ServoBoardMain/src/app` 中定义或处理 assembleAndStartSystem。 [27](#ref-0027)
- `ESP32-P4/ServoBoardMain/src/config/BoardConfig.h` — 33 行 · source · BoardConfig：位于 `ESP32-P4/ServoBoardMain/src/config`，具体职责需沿源码核对。 [28](#ref-0028)
- `ESP32-P4/ServoBoardMain/src/config/CalibrationConfig.h` — 57 行 · source · CalibrationConfig：位于 `ESP32-P4/ServoBoardMain/src/config`，具体职责需沿源码核对。 [29](#ref-0029)
- `ESP32-P4/ServoBoardMain/src/config/Config.h` — 18 行 · source · Config：位于 `ESP32-P4/ServoBoardMain/src/config`，具体职责需沿源码核对。 [30](#ref-0030)
- `ESP32-P4/ServoBoardMain/src/config/ControlConfig.h` — 148 行 · source · ControlConfig：位于 `ESP32-P4/ServoBoardMain/src/config`，具体职责需沿源码核对。 [31](#ref-0031)
- `ESP32-P4/ServoBoardMain/src/config/DiagnosticsConfig.h` — 22 行 · source · DiagnosticsConfig：位于 `ESP32-P4/ServoBoardMain/src/config`，具体职责需沿源码核对。 [32](#ref-0032)
- `ESP32-P4/ServoBoardMain/src/config/DimensionsConfig.h` — 40 行 · source · DimensionsConfig：位于 `ESP32-P4/ServoBoardMain/src/config`，具体职责需沿源码核对。 [33](#ref-0033)
- `ESP32-P4/ServoBoardMain/src/config/SafetyConfig.h` — 49 行 · source · SafetyConfig：位于 `ESP32-P4/ServoBoardMain/src/config`，具体职责需沿源码核对。 [34](#ref-0034)
- `ESP32-P4/ServoBoardMain/src/config/ServoConfig.h` — 72 行 · source · ServoConfig：位于 `ESP32-P4/ServoBoardMain/src/config`，具体职责需沿源码核对。 [35](#ref-0035)
- `ESP32-P4/ServoBoardMain/src/config/TaskConfig.h` — 29 行 · source · TaskConfig：位于 `ESP32-P4/ServoBoardMain/src/config`，具体职责需沿源码核对。 [36](#ref-0036)
- `ESP32-P4/ServoBoardMain/src/config/protocol/S3SensorCanProtocol.h` — 58 行 · source · S3SensorCanProtocol：位于 `ESP32-P4/ServoBoardMain/src/config/protocol`，具体职责需沿源码核对。 [37](#ref-0037)
- `ESP32-P4/ServoBoardMain/src/config/safety/SafetyRuleCatalog.def` — 116 行 · source · SafetyRuleCatalog：位于 `ESP32-P4/ServoBoardMain/src/config/safety`，具体职责需沿源码核对。 [38](#ref-0038)
- `ESP32-P4/ServoBoardMain/src/config/safety/ServoPositionRange.h` — 63 行 · source · ServoPositionRange：在 `ESP32-P4/ServoBoardMain/src/config/safety` 中定义或处理 servoLogicalToHardwareAbs。 [39](#ref-0039)
- `ESP32-P4/ServoBoardMain/src/config/topology/HardwareMap.cpp` — 190 行 · source · HardwareMap：在 `ESP32-P4/ServoBoardMain/src/config/topology` 中定义或处理 isMotorMapItemValid, isMotorChannelValid, validMotorChannelMask, getJointActuatorCount。 [40](#ref-0040)
- `ESP32-P4/ServoBoardMain/src/config/topology/HardwareMap.h` — 34 行 · source · HardwareMap：在 `ESP32-P4/ServoBoardMain/src/config/topology` 中定义或处理 isMotorMapItemValid, isMotorChannelValid, validMotorChannelMask, getJointActuatorCount。 [41](#ref-0041)
- `ESP32-P4/ServoBoardMain/src/data/ActuatorTypes.h` — 63 行 · source · ActuatorTypes：位于 `ESP32-P4/ServoBoardMain/src/data`，具体职责需沿源码核对。 [42](#ref-0042)
- `ESP32-P4/ServoBoardMain/src/data/CalibrationTypes.h` — 79 行 · source · CalibrationTypes：位于 `ESP32-P4/ServoBoardMain/src/data`，具体职责需沿源码核对。 [43](#ref-0043)
- `ESP32-P4/ServoBoardMain/src/data/ControlTraceTypes.h` — 34 行 · source · ControlTraceTypes：位于 `ESP32-P4/ServoBoardMain/src/data`，具体职责需沿源码核对。 [44](#ref-0044)
- `ESP32-P4/ServoBoardMain/src/data/HardwareTypes.h` — 34 行 · source · HardwareTypes：位于 `ESP32-P4/ServoBoardMain/src/data`，具体职责需沿源码核对。 [45](#ref-0045)
- `ESP32-P4/ServoBoardMain/src/data/JointTypes.h` — 27 行 · source · JointTypes：位于 `ESP32-P4/ServoBoardMain/src/data`，具体职责需沿源码核对。 [46](#ref-0046)
- `ESP32-P4/ServoBoardMain/src/data/RecoveryTypes.h` — 16 行 · source · RecoveryTypes：位于 `ESP32-P4/ServoBoardMain/src/data`，具体职责需沿源码核对。 [47](#ref-0047)
- `ESP32-P4/ServoBoardMain/src/data/RopeInitTypes.h` — 38 行 · source · RopeInitTypes：位于 `ESP32-P4/ServoBoardMain/src/data`，具体职责需沿源码核对。 [48](#ref-0048)
- `ESP32-P4/ServoBoardMain/src/data/SafetyRuleTypes.h` — 30 行 · source · SafetyRuleTypes：位于 `ESP32-P4/ServoBoardMain/src/data`，具体职责需沿源码核对。 [49](#ref-0049)
- `ESP32-P4/ServoBoardMain/src/data/SafetyTypes.h` — 47 行 · source · SafetyTypes：位于 `ESP32-P4/ServoBoardMain/src/data`，具体职责需沿源码核对。 [50](#ref-0050)
- `ESP32-P4/ServoBoardMain/src/data/ServiceSnapshots.h` — 61 行 · source · ServiceSnapshots：位于 `ESP32-P4/ServoBoardMain/src/data`，具体职责需沿源码核对。 [51](#ref-0051)
- `ESP32-P4/ServoBoardMain/src/data/ServoRouteTypes.h` — 44 行 · source · ServoRouteTypes：位于 `ESP32-P4/ServoBoardMain/src/data`，具体职责需沿源码核对。 [52](#ref-0052)
- `ESP32-P4/ServoBoardMain/src/data/ServoTypes.h` — 45 行 · source · ServoTypes：位于 `ESP32-P4/ServoBoardMain/src/data`，具体职责需沿源码核对。 [53](#ref-0053)
- `ESP32-P4/ServoBoardMain/src/data/SystemTypes.h` — 81 行 · source · SystemTypes：位于 `ESP32-P4/ServoBoardMain/src/data`，具体职责需沿源码核对。 [54](#ref-0054)
- `ESP32-P4/ServoBoardMain/src/data/TelemetryTypes.h` — 23 行 · source · TelemetryTypes：位于 `ESP32-P4/ServoBoardMain/src/data`，具体职责需沿源码核对。 [55](#ref-0055)
- `ESP32-P4/ServoBoardMain/src/generated/ProductFingerprint.generated.h` — 20 行 · generated · ProductFingerprint.generated：位于 `ESP32-P4/ServoBoardMain/src/generated`，具体职责需沿源码核对。 [56](#ref-0056)
- `ESP32-P4/ServoBoardMain/src/generated/SafetyRuleCatalog.generated.h` — 112 行 · generated · SafetyRuleCatalog.generated：位于 `ESP32-P4/ServoBoardMain/src/generated`，具体职责需沿源码核对。 [57](#ref-0057)
- `ESP32-P4/ServoBoardMain/src/generated/SafetyRuleIndex.generated.json` — 2300 行 · generated · SafetyRuleIndex.generated：在 `ESP32-P4/ServoBoardMain/src/generated` 中定义或处理 schema_version, description, _format_help, rules。 [58](#ref-0058)
- `ESP32-P4/ServoBoardMain/src/generated/UpperProtocol.generated.h` — 145 行 · generated · UpperProtocol.generated：位于 `ESP32-P4/ServoBoardMain/src/generated`，具体职责需沿源码核对。 [59](#ref-0059)
- `ESP32-P4/ServoBoardMain/src/generated/UrdfJointLimits.generated.h` — 50 行 · generated · UrdfJointLimits.generated：位于 `ESP32-P4/ServoBoardMain/src/generated`，具体职责需沿源码核对。 [60](#ref-0060)
- `ESP32-P4/ServoBoardMain/src/hal/can/TwaiCanDriver.cpp` — 116 行 · source · TwaiCanDriver：位于 `ESP32-P4/ServoBoardMain/src/hal/can`，具体职责需沿源码核对。 [61](#ref-0061)
- `ESP32-P4/ServoBoardMain/src/hal/can/TwaiCanDriver.h` — 62 行 · source · TwaiCanDriver：在 `ESP32-P4/ServoBoardMain/src/hal/can` 中定义或处理 begin, tryReceive, requestRecovery, restart。 [62](#ref-0062)
- `ESP32-P4/ServoBoardMain/src/hal/serial/HostSerialPort.cpp` — 61 行 · source · HostSerialPort：在 `ESP32-P4/ServoBoardMain/src/hal/serial` 中定义或处理 notifyHostRx, notifyHostRxError。 [63](#ref-0063)
- `ESP32-P4/ServoBoardMain/src/hal/serial/HostSerialPort.h` — 28 行 · source · HostSerialPort：在 `ESP32-P4/ServoBoardMain/src/hal/serial` 中定义或处理 begin, setRxWakeHandler, takeRxErrorFlags, available。 [64](#ref-0064)
- `ESP32-P4/ServoBoardMain/src/hal/servo/FeetechServoDriver.cpp` — 11 行 · source · FeetechServoDriver：位于 `ESP32-P4/ServoBoardMain/src/hal/servo`，具体职责需沿源码核对。 [65](#ref-0065)
- `ESP32-P4/ServoBoardMain/src/hal/servo/FeetechServoDriver.h` — 1126 行 · source · FeetechServoDriver：在 `ESP32-P4/ServoBoardMain/src/hal/servo` 中定义或处理 singleTurnDistanceCounts, begin, restartTransport, configureMultiTurnMode。 [66](#ref-0066)
- `ESP32-P4/ServoBoardMain/src/hal/servo/ServoBusRegistry.cpp` — 32 行 · source · ServoBusRegistry：位于 `ESP32-P4/ServoBoardMain/src/hal/servo`，具体职责需沿源码核对。 [67](#ref-0067)
- `ESP32-P4/ServoBoardMain/src/hal/servo/ServoBusRegistry.h` — 26 行 · source · ServoBusRegistry：在 `ESP32-P4/ServoBoardMain/src/hal/servo` 中定义或处理 begin。 [68](#ref-0068)
- `ESP32-P4/ServoBoardMain/src/hal/servo/ServoValue.h` — 58 行 · source · ServoValue：位于 `ESP32-P4/ServoBoardMain/src/hal/servo`，具体职责需沿源码核对。 [69](#ref-0069)
- `ESP32-P4/ServoBoardMain/src/modules/actuator/ActuatorCommandArbiter.cpp` — 24 行 · source · ActuatorCommandArbiter：在 `ESP32-P4/ServoBoardMain/src/modules/actuator` 中定义或处理 producerOwnsOutput。 [70](#ref-0070)
- `ESP32-P4/ServoBoardMain/src/modules/actuator/ActuatorCommandArbiter.h` — 24 行 · source · ActuatorCommandArbiter：在 `ESP32-P4/ServoBoardMain/src/modules/actuator` 中定义或处理 producerOwnsOutput。 [71](#ref-0071)
- `ESP32-P4/ServoBoardMain/src/modules/actuator/ActuatorCommandGateway.cpp` — 77 行 · source · ActuatorCommandGateway：在 `ESP32-P4/ServoBoardMain/src/modules/actuator` 中定义或处理 publishLatest, clearPending, receiveLatest。 [72](#ref-0072)
- `ESP32-P4/ServoBoardMain/src/modules/actuator/ActuatorCommandGateway.h` — 33 行 · source · ActuatorCommandGateway：在 `ESP32-P4/ServoBoardMain/src/modules/actuator` 中定义或处理 publishLatest, clearPending, receiveLatest。 [73](#ref-0073)
- `ESP32-P4/ServoBoardMain/src/modules/calibration/CalibrationMath.cpp` — 277 行 · source · CalibrationMath：在 `ESP32-P4/ServoBoardMain/src/modules/calibration` 中定义或处理 calibrationEncoderCountToDeg, applyManualScaledUrdfMappingToCalibration, mapManualScaledEncoderToUrdf。 [74](#ref-0074)
- `ESP32-P4/ServoBoardMain/src/modules/calibration/CalibrationMath.h` — 69 行 · source · CalibrationMath：在 `ESP32-P4/ServoBoardMain/src/modules/calibration` 中定义或处理 calibrationEncoderCountToDeg, applyManualScaledUrdfMappingToCalibration, mapManualScaledEncoderToUrdf。 [75](#ref-0075)
- `ESP32-P4/ServoBoardMain/src/modules/calibration/CalibrationRuntimeStore.cpp` — 285 行 · source · CalibrationRuntimeStore：在 `ESP32-P4/ServoBoardMain/src/modules/calibration` 中定义或处理 readJointCalibrationRuntimeSnapshot, readAllJointCalibrationRuntimeSnapshots, publishJointCalibrationResult, clearJointCalibrationResult。 [76](#ref-0076)
- `ESP32-P4/ServoBoardMain/src/modules/calibration/CalibrationRuntimeStore.h` — 128 行 · source · CalibrationRuntimeStore：在 `ESP32-P4/ServoBoardMain/src/modules/calibration` 中定义或处理 setRuntimeEncoderDirection, readJointCalibrationRuntimeSnapshot, readAllJointCalibrationRuntimeSnapshots, publishJointCalibrationResult。 [77](#ref-0077)
- `ESP32-P4/ServoBoardMain/src/modules/calibration/ManualEncoderReferenceStore.cpp` — 123 行 · source · ManualEncoderReferenceStore：在 `ESP32-P4/ServoBoardMain/src/modules/calibration` 中定义或处理 manualEncoderReferencesValidate, manualEncoderReferencesCoverMask, manualEncoderReferencesCrc, manualEncoderReferenceEntryCrc。 [78](#ref-0078)
- `ESP32-P4/ServoBoardMain/src/modules/calibration/ManualEncoderReferenceStore.h` — 40 行 · source · ManualEncoderReferenceStore：在 `ESP32-P4/ServoBoardMain/src/modules/calibration` 中定义或处理 manualEncoderReferencesValidate, manualEncoderReferencesCoverMask, manualEncoderReferencesCrc, manualEncoderReferenceEntryCrc。 [79](#ref-0079)
- `ESP32-P4/ServoBoardMain/src/modules/calibration/TendonCalibrationLut.cpp` — 211 行 · source · TendonCalibrationLut：在 `ESP32-P4/ServoBoardMain/src/modules/calibration` 中定义或处理 calibrationHasUsableTendonLut, tendonLutSampleAngleDeg, tendonCalibrationContainsAngle, tendonCalibrationMotorAbsRange。 [80](#ref-0080)
- `ESP32-P4/ServoBoardMain/src/modules/calibration/TendonCalibrationLut.h` — 55 行 · source · TendonCalibrationLut：在 `ESP32-P4/ServoBoardMain/src/modules/calibration` 中定义或处理 interpolateTendonCalibrationMotorAbs, tendonCalibrationContainsAngle, tendonCalibrationMotorAbsRange, tendonCalibrationMotorAbsRangeWithMargins。 [81](#ref-0081)
- `ESP32-P4/ServoBoardMain/src/modules/integrity/Crc16Ccitt.cpp` — 26 行 · source · Crc16Ccitt：在 `ESP32-P4/ServoBoardMain/src/modules/integrity` 中定义或处理 crc16CcittFalse。 [82](#ref-0082)
- `ESP32-P4/ServoBoardMain/src/modules/integrity/Crc16Ccitt.h` — 19 行 · source · Crc16Ccitt：在 `ESP32-P4/ServoBoardMain/src/modules/integrity` 中定义或处理 crc16CcittFalse。 [83](#ref-0083)
- `ESP32-P4/ServoBoardMain/src/modules/integrity/Crc32Ieee.cpp` — 28 行 · source · Crc32Ieee：在 `ESP32-P4/ServoBoardMain/src/modules/integrity` 中定义或处理 crc32IeeeUpdate。 [84](#ref-0084)
- `ESP32-P4/ServoBoardMain/src/modules/integrity/Crc32Ieee.h` — 21 行 · source · Crc32Ieee：在 `ESP32-P4/ServoBoardMain/src/modules/integrity` 中定义或处理 crc32IeeeUpdate。 [85](#ref-0085)
- `ESP32-P4/ServoBoardMain/src/modules/protocol/HostProtocol.h` — 14 行 · source · HostProtocol：位于 `ESP32-P4/ServoBoardMain/src/modules/protocol`，具体职责需沿源码核对。 [86](#ref-0086)
- `ESP32-P4/ServoBoardMain/src/modules/runtime/ServiceCyclePolicy.cpp` — 37 行 · source · ServiceCyclePolicy：在 `ESP32-P4/ServoBoardMain/src/modules/runtime` 中定义或处理 controlSnapshotHasActiveScope, servoSnapshotIsConsistent, calibrationSnapshotHasSelection, stateSnapshotIsConsistent。 [87](#ref-0087)
- `ESP32-P4/ServoBoardMain/src/modules/runtime/ServiceCyclePolicy.h` — 16 行 · source · ServiceCyclePolicy：在 `ESP32-P4/ServoBoardMain/src/modules/runtime` 中定义或处理 controlSnapshotHasActiveScope, servoSnapshotIsConsistent, calibrationSnapshotHasSelection, stateSnapshotIsConsistent。 [88](#ref-0088)
- `ESP32-P4/ServoBoardMain/src/platform/log/FirmwareLog.cpp` — 21 行 · source · FirmwareLog：位于 `ESP32-P4/ServoBoardMain/src/platform/log`，具体职责需沿源码核对。 [741](#ref-0741)
- `ESP32-P4/ServoBoardMain/src/platform/log/FirmwareLog.h` — 117 行 · source · FirmwareLog：在 `ESP32-P4/ServoBoardMain/src/platform/log` 中定义或处理 fwSerialTake, fwSerialGive, fwLogPrintln, fwLogPrintf。 [742](#ref-0742)
- `ESP32-P4/ServoBoardMain/src/platform/rtos/AtomicValue.h` — 50 行 · source · AtomicValue：在 `ESP32-P4/ServoBoardMain/src/platform/rtos` 中定义或处理 atomicStore。 [91](#ref-0091)
- `ESP32-P4/ServoBoardMain/src/platform/rtos/FreeRtosKernel.h` — 19 行 · source · FreeRtosKernel：位于 `ESP32-P4/ServoBoardMain/src/platform/rtos`，具体职责需沿源码核对。 [92](#ref-0092)
- `ESP32-P4/ServoBoardMain/src/platform/rtos/MessageChannel.h` — 75 行 · source · MessageChannel：在 `ESP32-P4/ServoBoardMain/src/platform/rtos` 中定义或处理 create, valid, send, receive。 [93](#ref-0093)
- `ESP32-P4/ServoBoardMain/src/platform/rtos/PeriodicDeadline.h` — 55 行 · source · PeriodicDeadline：在 `ESP32-P4/ServoBoardMain/src/platform/rtos` 中定义或处理 due, advancePast。 [94](#ref-0094)
- `ESP32-P4/ServoBoardMain/src/platform/rtos/SharedSafetyState.cpp` — 18 行 · source · SharedSafetyState：在 `ESP32-P4/ServoBoardMain/src/platform/rtos` 中定义或处理 sharedSafetyLock, sharedSafetyUnlock。 [95](#ref-0095)
- `ESP32-P4/ServoBoardMain/src/platform/rtos/SharedSafetyState.h` — 59 行 · source · SharedSafetyState：在 `ESP32-P4/ServoBoardMain/src/platform/rtos` 中定义或处理 sharedSafetyLock, sharedSafetyUnlock, sharedAtomicStore。 [96](#ref-0096)
- `ESP32-P4/ServoBoardMain/src/platform/rtos/TaskFactory.cpp` — 32 行 · source · TaskFactory：在 `ESP32-P4/ServoBoardMain/src/platform/rtos` 中定义或处理 createTask。 [97](#ref-0097)
- `ESP32-P4/ServoBoardMain/src/platform/rtos/TaskFactory.h` — 36 行 · source · TaskFactory：在 `ESP32-P4/ServoBoardMain/src/platform/rtos` 中定义或处理 createTask。 [98](#ref-0098)
- `ESP32-P4/ServoBoardMain/src/platform/rtos/TaskSignal.h` — 58 行 · source · TaskSignal：在 `ESP32-P4/ServoBoardMain/src/platform/rtos` 中定义或处理 bind, notify, wait。 [99](#ref-0099)
- `ESP32-P4/ServoBoardMain/src/platform/time/Clock.h` — 31 行 · source · Clock：在 `ESP32-P4/ServoBoardMain/src/platform/time` 中定义或处理 monotonicMillis, monotonicMicros。 [100](#ref-0100)
- `ESP32-P4/ServoBoardMain/src/runtime/SystemRuntimeContext.h` — 26 行 · source · SystemRuntimeContext：位于 `ESP32-P4/ServoBoardMain/src/runtime`，具体职责需沿源码核对。 [101](#ref-0101)
- `ESP32-P4/ServoBoardMain/src/runtime/adapters/ServiceContextAdapter.cpp` — 136 行 · source · ServiceContextAdapter：在 `ESP32-P4/ServoBoardMain/src/runtime/adapters` 中定义或处理 postEventAdapter。 [102](#ref-0102)
- `ESP32-P4/ServoBoardMain/src/runtime/adapters/ServiceContextAdapter.h` — 24 行 · source · ServiceContextAdapter：位于 `ESP32-P4/ServoBoardMain/src/runtime/adapters`，具体职责需沿源码核对。 [103](#ref-0103)
- `ESP32-P4/ServoBoardMain/src/runtime/channels/InterTaskChannels.h` — 40 行 · source · InterTaskChannels：位于 `ESP32-P4/ServoBoardMain/src/runtime/channels`，具体职责需沿源码核对。 [104](#ref-0104)
- `ESP32-P4/ServoBoardMain/src/runtime/metrics/RuntimeMonitorConfig.h` — 22 行 · source · RuntimeMonitorConfig：位于 `ESP32-P4/ServoBoardMain/src/runtime/metrics`，具体职责需沿源码核对。 [105](#ref-0105)
- `ESP32-P4/ServoBoardMain/src/runtime/metrics/TaskRuntimeMetrics.h` — 91 行 · source · TaskRuntimeMetrics：在 `ESP32-P4/ServoBoardMain/src/runtime/metrics` 中定义或处理 taskRuntimeRegister, taskRuntimeUnregister, taskRuntimeMarkLoopStart, taskRuntimeMarkLoopEnd。 [106](#ref-0106)
- `ESP32-P4/ServoBoardMain/src/runtime/metrics/TaskRuntimeWindow.cpp` — 65 行 · source · TaskRuntimeWindow：在 `ESP32-P4/ServoBoardMain/src/runtime/metrics` 中定义或处理 bucketForExecUs, bucketUpperUs, taskRuntimeWindowReset, taskRuntimeWindowAddExec。 [107](#ref-0107)
- `ESP32-P4/ServoBoardMain/src/runtime/metrics/TaskRuntimeWindow.h` — 31 行 · source · TaskRuntimeWindow：在 `ESP32-P4/ServoBoardMain/src/runtime/metrics` 中定义或处理 taskRuntimeWindowReset, taskRuntimeWindowAddExec, taskRuntimeWindowQuantileUs。 [108](#ref-0108)
- `ESP32-P4/ServoBoardMain/src/runtime/ports/ServiceOutputPort.h` — 28 行 · source · ServiceOutputPort：在 `ESP32-P4/ServoBoardMain/src/runtime/ports` 中定义或处理 servicePostEvent。 [109](#ref-0109)
- `ESP32-P4/ServoBoardMain/src/runtime/ports/SystemEventPort.h` — 29 行 · source · SystemEventPort：在 `ESP32-P4/ServoBoardMain/src/runtime/ports` 中定义或处理 postSystemEvent, postSystemEventForToken, requestEmergencyStop。 [110](#ref-0110)
- `ESP32-P4/ServoBoardMain/src/runtime/state/CalibrationRuntimeState.h` — 42 行 · source · CalibrationRuntimeState：位于 `ESP32-P4/ServoBoardMain/src/runtime/state`，具体职责需沿源码核对。 [111](#ref-0111)
- `ESP32-P4/ServoBoardMain/src/runtime/state/CommandRuntimeState.h` — 33 行 · source · CommandRuntimeState：位于 `ESP32-P4/ServoBoardMain/src/runtime/state`，具体职责需沿源码核对。 [112](#ref-0112)
- `ESP32-P4/ServoBoardMain/src/runtime/state/ControlRuntimeState.h` — 56 行 · source · ControlRuntimeState：位于 `ESP32-P4/ServoBoardMain/src/runtime/state`，具体职责需沿源码核对。 [113](#ref-0113)
- `ESP32-P4/ServoBoardMain/src/runtime/state/ServoRuntimeState.h` — 47 行 · source · ServoRuntimeState：位于 `ESP32-P4/ServoBoardMain/src/runtime/state`，具体职责需沿源码核对。 [114](#ref-0114)
- `ESP32-P4/ServoBoardMain/src/safety/SafetyEventReporter.cpp` — 168 行 · source · SafetyEventReporter：在 `ESP32-P4/ServoBoardMain/src/safety` 中定义或处理 eventScopeKey, reportWithContext, reportSafetyRuleEvent。 [115](#ref-0115)
- `ESP32-P4/ServoBoardMain/src/safety/SafetyEventReporter.h` — 85 行 · source · SafetyEventReporter：在 `ESP32-P4/ServoBoardMain/src/safety` 中定义或处理 reportSafetyRuleEvent, reportServoSafetyDecision。 [116](#ref-0116)
- `ESP32-P4/ServoBoardMain/src/safety/SafetyEventStore.cpp` — 117 行 · source · SafetyEventStore：在 `ESP32-P4/ServoBoardMain/src/safety` 中定义或处理 appendEventLocked, recordSafetyEvent, recordSafetyRuleEvent, safetyEventOverflowCount。 [117](#ref-0117)
- `ESP32-P4/ServoBoardMain/src/safety/SafetyEventStore.h` — 46 行 · source · SafetyEventStore：在 `ESP32-P4/ServoBoardMain/src/safety` 中定义或处理 recordSafetyEvent, recordSafetyRuleEvent, safetyEventOverflowCount。 [118](#ref-0118)
- `ESP32-P4/ServoBoardMain/src/safety/SafetyRuleRegistry.cpp` — 154 行 · source · SafetyRuleRegistry：位于 `ESP32-P4/ServoBoardMain/src/safety`，具体职责需沿源码核对。 [119](#ref-0119)
- `ESP32-P4/ServoBoardMain/src/safety/SafetyRuleRegistry.h` — 44 行 · source · SafetyRuleRegistry：位于 `ESP32-P4/ServoBoardMain/src/safety`，具体职责需沿源码核对。 [120](#ref-0120)
- `ESP32-P4/ServoBoardMain/src/safety/policies/ActuatorSafetyPolicy.cpp` — 310 行 · source · ActuatorSafetyPolicy：在 `ESP32-P4/ServoBoardMain/src/safety/policies` 中定义或处理 ownerMatchesSystemState, computeTargetAgeMs。 [121](#ref-0121)
- `ESP32-P4/ServoBoardMain/src/safety/policies/ActuatorSafetyPolicy.h` — 69 行 · source · ActuatorSafetyPolicy：在 `ESP32-P4/ServoBoardMain/src/safety/policies` 中定义或处理 computeTargetAgeMs。 [122](#ref-0122)
- `ESP32-P4/ServoBoardMain/src/safety/policies/CalibrationSafetyPolicy.cpp` — 68 行 · source · CalibrationSafetyPolicy：在 `ESP32-P4/ServoBoardMain/src/safety/policies` 中定义或处理 calibrationReleaseExceeded, calibrationCommandInsideRange, calibrationLoadOverloaded。 [123](#ref-0123)
- `ESP32-P4/ServoBoardMain/src/safety/policies/CalibrationSafetyPolicy.h` — 90 行 · source · CalibrationSafetyPolicy：在 `ESP32-P4/ServoBoardMain/src/safety/policies` 中定义或处理 calibrationAssemblyWarningReasonForMetrics, calibrationReleaseExceeded, calibrationCommandInsideRange, calibrationLoadOverloaded。 [124](#ref-0124)
- `ESP32-P4/ServoBoardMain/src/safety/policies/CollisionSafetyPolicy.cpp` — 374 行 · source · CollisionSafetyPolicy：在 `ESP32-P4/ServoBoardMain/src/safety/policies` 中定义或处理 directionOf, normalizeMotionMode, progressWindowForMotionMode, minProgressForMotionMode。 [125](#ref-0125)
- `ESP32-P4/ServoBoardMain/src/safety/policies/CollisionSafetyPolicy.h` — 138 行 · source · CollisionSafetyPolicy：在 `ESP32-P4/ServoBoardMain/src/safety/policies` 中定义或处理 resetAll, isArmed, isHoldActive, holdAngleDeg。 [126](#ref-0126)
- `ESP32-P4/ServoBoardMain/src/safety/policies/CommunicationSafetyPolicy.cpp` — 70 行 · source · CommunicationSafetyPolicy：在 `ESP32-P4/ServoBoardMain/src/safety/policies` 中定义或处理 communicationLengthValid, communicationReassemblyComplete, communicationFrameCountComplete, communicationIndexValid。 [127](#ref-0127)
- `ESP32-P4/ServoBoardMain/src/safety/policies/CommunicationSafetyPolicy.h` — 41 行 · source · CommunicationSafetyPolicy：在 `ESP32-P4/ServoBoardMain/src/safety/policies` 中定义或处理 communicationLengthValid, communicationReassemblyComplete, communicationFrameCountComplete, communicationIndexValid。 [128](#ref-0128)
- `ESP32-P4/ServoBoardMain/src/safety/policies/ControlBoundarySafetyPolicy.cpp` — 90 行 · source · ControlBoundarySafetyPolicy：在 `ESP32-P4/ServoBoardMain/src/safety/policies` 中定义或处理 shouldEnterHotplugHold, shouldReportHotplugHold。 [129](#ref-0129)
- `ESP32-P4/ServoBoardMain/src/safety/policies/ControlBoundarySafetyPolicy.h` — 67 行 · source · ControlBoundarySafetyPolicy：在 `ESP32-P4/ServoBoardMain/src/safety/policies` 中定义或处理 shouldEnterHotplugHold, shouldReportHotplugHold。 [130](#ref-0130)
- `ESP32-P4/ServoBoardMain/src/safety/policies/ControlInputSafetyPolicy.cpp` — 64 行 · source · ControlInputSafetyPolicy：位于 `ESP32-P4/ServoBoardMain/src/safety/policies`，具体职责需沿源码核对。 [131](#ref-0131)
- `ESP32-P4/ServoBoardMain/src/safety/policies/ControlInputSafetyPolicy.h` — 40 行 · source · ControlInputSafetyPolicy：位于 `ESP32-P4/ServoBoardMain/src/safety/policies`，具体职责需沿源码核对。 [132](#ref-0132)
- `ESP32-P4/ServoBoardMain/src/safety/policies/EncoderHealthSafetyPolicy.cpp` — 69 行 · source · EncoderHealthSafetyPolicy：在 `ESP32-P4/ServoBoardMain/src/safety/policies` 中定义或处理 acceptEncoderSample, rejectEncoderSample, applyEncoderFeedbackTimeout, resetEncoderHealthState。 [133](#ref-0133)
- `ESP32-P4/ServoBoardMain/src/safety/policies/EncoderHealthSafetyPolicy.h` — 32 行 · source · EncoderHealthSafetyPolicy：在 `ESP32-P4/ServoBoardMain/src/safety/policies` 中定义或处理 acceptEncoderSample, rejectEncoderSample, applyEncoderFeedbackTimeout, resetEncoderHealthState。 [134](#ref-0134)
- `ESP32-P4/ServoBoardMain/src/safety/policies/HostLeaseSafetyPolicy.cpp` — 33 行 · source · HostLeaseSafetyPolicy：在 `ESP32-P4/ServoBoardMain/src/safety/policies` 中定义或处理 requiresHostControlLease, isHostControlLeaseFresh。 [135](#ref-0135)
- `ESP32-P4/ServoBoardMain/src/safety/policies/HostLeaseSafetyPolicy.h` — 21 行 · source · HostLeaseSafetyPolicy：在 `ESP32-P4/ServoBoardMain/src/safety/policies` 中定义或处理 requiresHostControlLease, isHostControlLeaseFresh。 [136](#ref-0136)
- `ESP32-P4/ServoBoardMain/src/safety/policies/OverloadSafetyPolicy.cpp` — 79 行 · source · OverloadSafetyPolicy：在 `ESP32-P4/ServoBoardMain/src/safety/policies` 中定义或处理 overloadActiveMotorMask, jointHasOverloadFault。 [137](#ref-0137)
- `ESP32-P4/ServoBoardMain/src/safety/policies/OverloadSafetyPolicy.h` — 38 行 · source · OverloadSafetyPolicy：在 `ESP32-P4/ServoBoardMain/src/safety/policies` 中定义或处理 overloadActiveMotorMask, jointHasOverloadFault。 [138](#ref-0138)
- `ESP32-P4/ServoBoardMain/src/safety/policies/RopeInitSafetyPolicy.cpp` — 96 行 · source · RopeInitSafetyPolicy：在 `ESP32-P4/ServoBoardMain/src/safety/policies` 中定义或处理 ropeInitCalibrationMatchesManual, evaluateRopeInitActiveScope, readRopeInitFeedback。 [139](#ref-0139)
- `ESP32-P4/ServoBoardMain/src/safety/policies/RopeInitSafetyPolicy.h` — 32 行 · source · RopeInitSafetyPolicy：在 `ESP32-P4/ServoBoardMain/src/safety/policies` 中定义或处理 evaluateRopeInitActiveScope, readRopeInitFeedback, ropeInitCalibrationMatchesManual。 [140](#ref-0140)
- `ESP32-P4/ServoBoardMain/src/safety/policies/RuntimeSafetyPolicy.cpp` — 58 行 · source · RuntimeSafetyPolicy：在 `ESP32-P4/ServoBoardMain/src/safety/policies` 中定义或处理 runtimeEventCacheWillOverflow, runtimeStackWarningThreshold。 [141](#ref-0141)
- `ESP32-P4/ServoBoardMain/src/safety/policies/RuntimeSafetyPolicy.h` — 46 行 · source · RuntimeSafetyPolicy：在 `ESP32-P4/ServoBoardMain/src/safety/policies` 中定义或处理 runtimeEventCacheWillOverflow, runtimeStackWarningThreshold。 [142](#ref-0142)
- `ESP32-P4/ServoBoardMain/src/safety/policies/StateAdmissionPolicy.cpp` — 43 行 · source · StateAdmissionPolicy：在 `ESP32-P4/ServoBoardMain/src/safety/policies` 中定义或处理 canAcceptStart, canAcceptRunDirect, canAcceptRunFeedback, canAcceptCalibration。 [143](#ref-0143)
- `ESP32-P4/ServoBoardMain/src/safety/policies/StateAdmissionPolicy.h` — 19 行 · source · StateAdmissionPolicy：在 `ESP32-P4/ServoBoardMain/src/safety/policies` 中定义或处理 canAcceptStart, canAcceptRunDirect, canAcceptRunFeedback, canAcceptCalibration。 [144](#ref-0144)
- `ESP32-P4/ServoBoardMain/src/safety/policies/SystemFaultPolicy.cpp` — 108 行 · source · SystemFaultPolicy：在 `ESP32-P4/ServoBoardMain/src/safety/policies` 中定义或处理 collectSystemFaults, blockingFaultsForState。 [145](#ref-0145)
- `ESP32-P4/ServoBoardMain/src/safety/policies/SystemFaultPolicy.h` — 27 行 · source · SystemFaultPolicy：在 `ESP32-P4/ServoBoardMain/src/safety/policies` 中定义或处理 collectSystemFaults, blockingFaultsForState。 [146](#ref-0146)
- `ESP32-P4/ServoBoardMain/src/safety/policies/SystemReadinessPolicy.cpp` — 406 行 · source · SystemReadinessPolicy：在 `ESP32-P4/ServoBoardMain/src/safety/policies` 中定义或处理 routesControllable, requiredMotorMask, requiresCan, requiredServosOnline。 [147](#ref-0147)
- `ESP32-P4/ServoBoardMain/src/safety/policies/SystemReadinessPolicy.h` — 70 行 · source · SystemReadinessPolicy：在 `ESP32-P4/ServoBoardMain/src/safety/policies` 中定义或处理 requiredMotorMask, requiresCan, requiredServosOnline, directMotorMaskReady。 [148](#ref-0148)
- `ESP32-P4/ServoBoardMain/src/tasks/auto_calibration/AutoCalibrationCoordinator.cpp` — 202 行 · source · AutoCalibrationCoordinator：在 `ESP32-P4/ServoBoardMain/src/tasks/auto_calibration` 中定义或处理 runAutoCalibrationSequence。 [149](#ref-0149)
- `ESP32-P4/ServoBoardMain/src/tasks/auto_calibration/AutoCalibrationCoordinator.h` — 30 行 · source · AutoCalibrationCoordinator：在 `ESP32-P4/ServoBoardMain/src/tasks/auto_calibration` 中定义或处理 runAutoCalibrationSequence。 [150](#ref-0150)
- `ESP32-P4/ServoBoardMain/src/tasks/auto_calibration/AutoCalibrationTask.cpp` — 44 行 · source · AutoCalibrationTask：在 `ESP32-P4/ServoBoardMain/src/tasks/auto_calibration` 中定义或处理 calibrationTaskNowMs, calibrationTaskWaitMs, autoCalibrationTask。 [151](#ref-0151)
- `ESP32-P4/ServoBoardMain/src/tasks/auto_calibration/AutoCalibrationTask.h` — 12 行 · source · AutoCalibrationTask：在 `ESP32-P4/ServoBoardMain/src/tasks/auto_calibration` 中定义或处理 autoCalibrationTask。 [152](#ref-0152)
- `ESP32-P4/ServoBoardMain/src/tasks/auto_calibration/AutoCalibrationTaskRuntime.cpp` — 48 行 · source · AutoCalibrationTaskRuntime：在 `ESP32-P4/ServoBoardMain/src/tasks/auto_calibration` 中定义或处理 startAutoCalibrationTaskRuntime, finishAutoCalibrationTaskRuntime。 [153](#ref-0153)
- `ESP32-P4/ServoBoardMain/src/tasks/auto_calibration/AutoCalibrationTaskRuntime.h` — 17 行 · source · AutoCalibrationTaskRuntime：在 `ESP32-P4/ServoBoardMain/src/tasks/auto_calibration` 中定义或处理 startAutoCalibrationTaskRuntime, finishAutoCalibrationTaskRuntime。 [154](#ref-0154)
- `ESP32-P4/ServoBoardMain/src/tasks/auto_calibration/CalibrationCoordinatorPort.h` — 27 行 · source · CalibrationCoordinatorPort：在 `ESP32-P4/ServoBoardMain/src/tasks/auto_calibration` 中定义或处理 waitForCalibrationEncoderRecovery, waitForCalibrationServoRecovery, isWorkspaceDualJoint, buildDynamicWorkspaceMotorMask。 [155](#ref-0155)
- `ESP32-P4/ServoBoardMain/src/tasks/auto_calibration/CalibrationDiagnostics.cpp` — 130 行 · source · CalibrationDiagnostics：在 `ESP32-P4/ServoBoardMain/src/tasks/auto_calibration` 中定义或处理 setCalibrationDiag, clearCalibrationDiag, setCalibrationDiagIfEmpty, hasCalibrationDiag。 [156](#ref-0156)
- `ESP32-P4/ServoBoardMain/src/tasks/auto_calibration/CalibrationDiagnostics.h` — 56 行 · source · CalibrationDiagnostics：在 `ESP32-P4/ServoBoardMain/src/tasks/auto_calibration` 中定义或处理 setCalibrationDiag, setCalibrationDiagIfEmpty, clearCalibrationDiag, hasCalibrationDiag。 [157](#ref-0157)
- `ESP32-P4/ServoBoardMain/src/tasks/auto_calibration/CalibrationFeedbackReader.cpp` — 177 行 · source · CalibrationFeedbackReader：在 `ESP32-P4/ServoBoardMain/src/tasks/auto_calibration` 中定义或处理 encoderOnlineAndHealthy, readLatestEncoderSample, readLatestEncoderOriented, readLatestMotorServoAbs。 [158](#ref-0158)
- `ESP32-P4/ServoBoardMain/src/tasks/auto_calibration/CalibrationFeedbackReader.h` — 53 行 · source · CalibrationFeedbackReader：在 `ESP32-P4/ServoBoardMain/src/tasks/auto_calibration` 中定义或处理 readLatestEncoderSample, readLatestEncoderOriented, readLatestMotorServoAbs, readLatestMotorServoLoad。 [159](#ref-0159)
- `ESP32-P4/ServoBoardMain/src/tasks/auto_calibration/CalibrationLutBuilder.cpp` — 407 行 · source · CalibrationLutBuilder：在 `ESP32-P4/ServoBoardMain/src/tasks/auto_calibration` 中定义或处理 resetTendonRawFitBuffer, appendTendonRawFitSample, maybeAppendTendonTravelFitSample, tendonFitSampleAngleDeg。 [160](#ref-0160)
- `ESP32-P4/ServoBoardMain/src/tasks/auto_calibration/CalibrationLutBuilder.h` — 85 行 · source · CalibrationLutBuilder：在 `ESP32-P4/ServoBoardMain/src/tasks/auto_calibration` 中定义或处理 resetTendonRawFitBuffer, appendTendonRawFitSample, maybeAppendTendonTravelFitSample, buildTendonLutFromRawSamples。 [161](#ref-0161)
- `ESP32-P4/ServoBoardMain/src/tasks/auto_calibration/CalibrationMotionSupport.cpp` — 929 行 · source · CalibrationMotionSupport：在 `ESP32-P4/ServoBoardMain/src/tasks/auto_calibration` 中定义或处理 setCalibratedMotorAbsLimit, setSingleTendonLutAbsLimit, clearCalibratedMotorAbsLimitsForJoint, autoCalibrationStillOwnsControl。 [162](#ref-0162)
- `ESP32-P4/ServoBoardMain/src/tasks/auto_calibration/CalibrationMotionSupport.h` — 81 行 · source · CalibrationMotionSupport：在 `ESP32-P4/ServoBoardMain/src/tasks/auto_calibration` 中定义或处理 setCalibratedMotorAbsLimit, setSingleTendonLutAbsLimit, clearCalibratedMotorAbsLimitsForJoint, autoCalibrationStillOwnsControl。 [163](#ref-0163)
- `ESP32-P4/ServoBoardMain/src/tasks/auto_calibration/CalibrationResultCommitter.cpp` — 65 行 · source · CalibrationResultCommitter：在 `ESP32-P4/ServoBoardMain/src/tasks/auto_calibration` 中定义或处理 syncJointTarget, commitCalibrationResult。 [164](#ref-0164)
- `ESP32-P4/ServoBoardMain/src/tasks/auto_calibration/CalibrationResultCommitter.h` — 23 行 · source · CalibrationResultCommitter：在 `ESP32-P4/ServoBoardMain/src/tasks/auto_calibration` 中定义或处理 commitCalibrationResult。 [165](#ref-0165)
- `ESP32-P4/ServoBoardMain/src/tasks/auto_calibration/CalibrationRuntimePort.cpp` — 31 行 · source · CalibrationRuntimePort：在 `ESP32-P4/ServoBoardMain/src/tasks/auto_calibration` 中定义或处理 configureCalibrationRuntimePort, calibrationNowMs, calibrationWaitMs。 [166](#ref-0166)
- `ESP32-P4/ServoBoardMain/src/tasks/auto_calibration/CalibrationRuntimePort.h` — 26 行 · source · CalibrationRuntimePort：在 `ESP32-P4/ServoBoardMain/src/tasks/auto_calibration` 中定义或处理 configureCalibrationRuntimePort, calibrationNowMs, calibrationWaitMs。 [167](#ref-0167)
- `ESP32-P4/ServoBoardMain/src/tasks/auto_calibration/CalibrationSequencePlanner.cpp` — 94 行 · source · CalibrationSequencePlanner：在 `ESP32-P4/ServoBoardMain/src/tasks/auto_calibration` 中定义或处理 autoCalibrationSequenceRank, calibrationJointMotorMask, buildCalibrationJointMask, buildCalibrationMotorMask。 [168](#ref-0168)
- `ESP32-P4/ServoBoardMain/src/tasks/auto_calibration/CalibrationSequencePlanner.h` — 21 行 · source · CalibrationSequencePlanner：在 `ESP32-P4/ServoBoardMain/src/tasks/auto_calibration` 中定义或处理 autoCalibrationSequenceRank, calibrationJointMotorMask, buildCalibrationJointMask, buildCalibrationMotorMask。 [169](#ref-0169)
- `ESP32-P4/ServoBoardMain/src/tasks/auto_calibration/CalibrationSessionController.cpp` — 158 行 · source · CalibrationSessionController：在 `ESP32-P4/ServoBoardMain/src/tasks/auto_calibration` 中定义或处理 logActiveScope, requiresWorkspaceHelpers, runAutoCalibrationSession。 [170](#ref-0170)
- `ESP32-P4/ServoBoardMain/src/tasks/auto_calibration/CalibrationSessionController.h` — 21 行 · source · CalibrationSessionController：在 `ESP32-P4/ServoBoardMain/src/tasks/auto_calibration` 中定义或处理 runAutoCalibrationSession。 [171](#ref-0171)
- `ESP32-P4/ServoBoardMain/src/tasks/auto_calibration/dual_motor/DualMotorCalibrationMachine.cpp` — 2414 行 · source · DualMotorCalibrationMachine：在 `ESP32-P4/ServoBoardMain/src/tasks/auto_calibration/dual_motor` 中定义或处理 setDualFeedbackLostDiag, setDualPublishFailedDiag, setDualControlLostDiag, waitMotorTargetReached。 [172](#ref-0172)
- `ESP32-P4/ServoBoardMain/src/tasks/auto_calibration/dual_motor/DualMotorCalibrationMachine.h` — 28 行 · source · DualMotorCalibrationMachine：在 `ESP32-P4/ServoBoardMain/src/tasks/auto_calibration/dual_motor` 中定义或处理 resetDualWorkspaceCalibrationCache。 [173](#ref-0173)
- `ESP32-P4/ServoBoardMain/src/tasks/auto_calibration/single_motor/SingleMotorCalibrationMachine.cpp` — 1265 行 · source · SingleMotorCalibrationMachine：在 `ESP32-P4/ServoBoardMain/src/tasks/auto_calibration/single_motor` 中定义或处理 failAutoCalibContext, failAutoCalibContextWithDiag, failSingleFeedbackLost, initTightenStateFromFeedback。 [174](#ref-0174)
- `ESP32-P4/ServoBoardMain/src/tasks/auto_calibration/single_motor/SingleMotorCalibrationMachine.h` — 27 行 · source · SingleMotorCalibrationMachine：位于 `ESP32-P4/ServoBoardMain/src/tasks/auto_calibration/single_motor`，具体职责需沿源码核对。 [175](#ref-0175)
- `ESP32-P4/ServoBoardMain/src/tasks/host_comm/HostAckPublisher.cpp` — 80 行 · source · HostAckPublisher：在 `ESP32-P4/ServoBoardMain/src/tasks/host_comm` 中定义或处理 writeProtoAckPacket, enqueueProtoAck, sendProtoAckPacket, retryPendingProtoAckPacket。 [176](#ref-0176)
- `ESP32-P4/ServoBoardMain/src/tasks/host_comm/HostAckPublisher.h` — 20 行 · source · HostAckPublisher：在 `ESP32-P4/ServoBoardMain/src/tasks/host_comm` 中定义或处理 sendProtoAckPacket, retryPendingProtoAckPacket。 [177](#ref-0177)
- `ESP32-P4/ServoBoardMain/src/tasks/host_comm/HostCalibrationCommandHandler.cpp` — 239 行 · source · HostCalibrationCommandHandler：在 `ESP32-P4/ServoBoardMain/src/tasks/host_comm` 中定义或处理 autoCalibrationValidMask, commandRequiresManualReference, manualUpdateMotionActive, changedManualReferenceMask。 [178](#ref-0178)
- `ESP32-P4/ServoBoardMain/src/tasks/host_comm/HostCalibrationCommandHandler.h` — 44 行 · source · HostCalibrationCommandHandler：在 `ESP32-P4/ServoBoardMain/src/tasks/host_comm` 中定义或处理 commandRequiresManualReference, manualUpdateMotionActive, jointMaskForMotorMask, invalidateAutomaticCalibration。 [179](#ref-0179)
- `ESP32-P4/ServoBoardMain/src/tasks/host_comm/HostCommService.cpp` — 596 行 · source · HostCommService：在 `ESP32-P4/ServoBoardMain/src/tasks/host_comm` 中定义或处理 appendFnv1a, motorAbsLimitStateHash, servoLogicalRangeStateHash, requestHostStateSync。 [180](#ref-0180)
- `ESP32-P4/ServoBoardMain/src/tasks/host_comm/HostCommService.h` — 44 行 · source · HostCommService：在 `ESP32-P4/ServoBoardMain/src/tasks/host_comm` 中定义或处理 initHostCommService, stepHostCommService, requestHostStateSync, setHostTaskDiagSubscribed。 [181](#ref-0181)
- `ESP32-P4/ServoBoardMain/src/tasks/host_comm/HostCommTask.cpp` — 77 行 · source · HostCommTask：在 `ESP32-P4/ServoBoardMain/src/tasks/host_comm` 中定义或处理 wakeHostComm, hostCommTask。 [182](#ref-0182)
- `ESP32-P4/ServoBoardMain/src/tasks/host_comm/HostCommTask.h` — 13 行 · source · HostCommTask：在 `ESP32-P4/ServoBoardMain/src/tasks/host_comm` 中定义或处理 hostCommTask。 [183](#ref-0183)
- `ESP32-P4/ServoBoardMain/src/tasks/host_comm/HostCommandDispatcher.cpp` — 920 行 · source · HostCommandDispatcher：在 `ESP32-P4/ServoBoardMain/src/tasks/host_comm` 中定义或处理 refreshLoadedSingleTendonAbsLimit, dispatchHostCommand。 [184](#ref-0184)
- `ESP32-P4/ServoBoardMain/src/tasks/host_comm/HostCommandDispatcher.h` — 23 行 · source · HostCommandDispatcher：在 `ESP32-P4/ServoBoardMain/src/tasks/host_comm` 中定义或处理 dispatchHostCommand。 [185](#ref-0185)
- `ESP32-P4/ServoBoardMain/src/tasks/host_comm/HostCommandRouter.cpp` — 284 行 · source · HostCommandRouter：在 `ESP32-P4/ServoBoardMain/src/tasks/host_comm` 中定义或处理 cancelStagedDirectCommand, upperApplyTargetAnglesMasked, upperApplyTargetAngles, upperApplyMotorTargets。 [186](#ref-0186)
- `ESP32-P4/ServoBoardMain/src/tasks/host_comm/HostCommandRouter.h` — 39 行 · source · HostCommandRouter：在 `ESP32-P4/ServoBoardMain/src/tasks/host_comm` 中定义或处理 upperApplyTargetAngles, upperApplyTargetAnglesMasked, upperApplyMotorTargets, upperApplyActiveScope。 [187](#ref-0187)
- `ESP32-P4/ServoBoardMain/src/tasks/host_comm/HostFrameReader.cpp` — 124 行 · source · HostFrameReader：位于 `ESP32-P4/ServoBoardMain/src/tasks/host_comm`，具体职责需沿源码核对。 [188](#ref-0188)
- `ESP32-P4/ServoBoardMain/src/tasks/host_comm/HostFrameReader.h` — 62 行 · source · HostFrameReader：在 `ESP32-P4/ServoBoardMain/src/tasks/host_comm` 中定义或处理 push, pop, reset, takeRejectReason。 [189](#ref-0189)
- `ESP32-P4/ServoBoardMain/src/tasks/host_comm/HostMultiTurnPublisher.cpp` — 67 行 · source · HostMultiTurnPublisher：在 `ESP32-P4/ServoBoardMain/src/tasks/host_comm` 中定义或处理 initHostMultiTurnPublisher, stepHostMultiTurnPublisher。 [190](#ref-0190)
- `ESP32-P4/ServoBoardMain/src/tasks/host_comm/HostMultiTurnPublisher.h` — 23 行 · source · HostMultiTurnPublisher：在 `ESP32-P4/ServoBoardMain/src/tasks/host_comm` 中定义或处理 initHostMultiTurnPublisher, stepHostMultiTurnPublisher。 [191](#ref-0191)
- `ESP32-P4/ServoBoardMain/src/tasks/host_comm/HostProtocolCodec.cpp` — 148 行 · source · HostProtocolCodec：在 `ESP32-P4/ServoBoardMain/src/tasks/host_comm` 中定义或处理 appendFloatBigEndian, appendI32BigEndian, appendI16BigEndian, appendU16BigEndian。 [192](#ref-0192)
- `ESP32-P4/ServoBoardMain/src/tasks/host_comm/HostProtocolCodec.h` — 46 行 · source · HostProtocolCodec：在 `ESP32-P4/ServoBoardMain/src/tasks/host_comm` 中定义或处理 appendFloatBigEndian, appendI32BigEndian, appendI16BigEndian, appendU16BigEndian。 [193](#ref-0193)
- `ESP32-P4/ServoBoardMain/src/tasks/host_comm/HostTelemetryPublisher.cpp` — 967 行 · source · HostTelemetryPublisher：在 `ESP32-P4/ServoBoardMain/src/tasks/host_comm` 中定义或处理 calibrationTendonLutMatchesJoint, activeManualReferencesReady, encodeRawEncoderForWire, sendSystemStatusPacket。 [194](#ref-0194)
- `ESP32-P4/ServoBoardMain/src/tasks/host_comm/HostTelemetryPublisher.h` — 52 行 · source · HostTelemetryPublisher：在 `ESP32-P4/ServoBoardMain/src/tasks/host_comm` 中定义或处理 calibrationTendonLutMatchesJoint, activeManualReferencesReady, sendSystemStatusPacket, sendManualEncoderReferencePacket。 [195](#ref-0195)
- `ESP32-P4/ServoBoardMain/src/tasks/motion_control/ControlCommandSnapshotReader.cpp` — 44 行 · source · ControlCommandSnapshotReader：在 `ESP32-P4/ServoBoardMain/src/tasks/motion_control` 中定义或处理 readCommandSnapshot。 [196](#ref-0196)
- `ESP32-P4/ServoBoardMain/src/tasks/motion_control/ControlCommandSnapshotReader.h` — 23 行 · source · ControlCommandSnapshotReader：在 `ESP32-P4/ServoBoardMain/src/tasks/motion_control` 中定义或处理 readCommandSnapshot。 [197](#ref-0197)
- `ESP32-P4/ServoBoardMain/src/tasks/motion_control/ControlSolver.cpp` — 92 行 · source · ControlSolver：位于 `ESP32-P4/ServoBoardMain/src/tasks/motion_control`，具体职责需沿源码核对。 [198](#ref-0198)
- `ESP32-P4/ServoBoardMain/src/tasks/motion_control/ControlSolver.h` — 44 行 · source · ControlSolver：在 `ESP32-P4/ServoBoardMain/src/tasks/motion_control` 中定义或处理 begin, compute, getPidOutput, resetJoint。 [199](#ref-0199)
- `ESP32-P4/ServoBoardMain/src/tasks/motion_control/DualTendonControl.cpp` — 349 行 · source · DualTendonControl：在 `ESP32-P4/ServoBoardMain/src/tasks/motion_control` 中定义或处理 getDualTendonConfig, constrainFloat, getDualTendonActuators, computeDualTendonModelAbsFromCalibration。 [200](#ref-0200)
- `ESP32-P4/ServoBoardMain/src/tasks/motion_control/DualTendonControl.h` — 47 行 · source · DualTendonControl：在 `ESP32-P4/ServoBoardMain/src/tasks/motion_control` 中定义或处理 isDualTendonJoint, resetDualTendonController, computeDualTendonMotorTargets。 [201](#ref-0201)
- `ESP32-P4/ServoBoardMain/src/tasks/motion_control/JointAngleMapper.cpp` — 43 行 · source · JointAngleMapper：在 `ESP32-P4/ServoBoardMain/src/tasks/motion_control` 中定义或处理 clampJointTargetDeg。 [202](#ref-0202)
- `ESP32-P4/ServoBoardMain/src/tasks/motion_control/JointAngleMapper.h` — 24 行 · source · JointAngleMapper：在 `ESP32-P4/ServoBoardMain/src/tasks/motion_control` 中定义或处理 clampJointTargetDeg。 [203](#ref-0203)
- `ESP32-P4/ServoBoardMain/src/tasks/motion_control/MotionCompletionEvaluator.cpp` — 84 行 · source · MotionCompletionEvaluator：在 `ESP32-P4/ServoBoardMain/src/tasks/motion_control` 中定义或处理 directTargetsReached, feedbackTargetsReached。 [204](#ref-0204)
- `ESP32-P4/ServoBoardMain/src/tasks/motion_control/MotionCompletionEvaluator.h` — 26 行 · source · MotionCompletionEvaluator：在 `ESP32-P4/ServoBoardMain/src/tasks/motion_control` 中定义或处理 directTargetsReached, feedbackTargetsReached。 [205](#ref-0205)
- `ESP32-P4/ServoBoardMain/src/tasks/motion_control/MotionControlService.cpp` — 2076 行 · source · MotionControlService：在 `ESP32-P4/ServoBoardMain/src/tasks/motion_control` 中定义或处理 resetSingleTendonDirectionalTakeup, resetSingleTendonVelocityFeedforward, computeSingleTendonVelocityFeedforward, updateSingleTendonDirectionalTakeupTarget。 [206](#ref-0206)
- `ESP32-P4/ServoBoardMain/src/tasks/motion_control/MotionControlService.h` — 28 行 · source · MotionControlService：在 `ESP32-P4/ServoBoardMain/src/tasks/motion_control` 中定义或处理 initMotionControlService, stepMotionControlService。 [207](#ref-0207)
- `ESP32-P4/ServoBoardMain/src/tasks/motion_control/MotionControlTask.cpp` — 49 行 · source · MotionControlTask：在 `ESP32-P4/ServoBoardMain/src/tasks/motion_control` 中定义或处理 motionControlTask。 [208](#ref-0208)
- `ESP32-P4/ServoBoardMain/src/tasks/motion_control/MotionControlTask.h` — 16 行 · source · MotionControlTask：在 `ESP32-P4/ServoBoardMain/src/tasks/motion_control` 中定义或处理 motionControlTask。 [209](#ref-0209)
- `ESP32-P4/ServoBoardMain/src/tasks/motion_control/ServoTargetBuilder.cpp` — 135 行 · source · ServoTargetBuilder：在 `ESP32-P4/ServoBoardMain/src/tasks/motion_control` 中定义或处理 appendServoTarget, appendJointActuatorTargets, appendJointActuatorRuntimeTargets。 [210](#ref-0210)
- `ESP32-P4/ServoBoardMain/src/tasks/motion_control/ServoTargetBuilder.h` — 47 行 · source · ServoTargetBuilder：在 `ESP32-P4/ServoBoardMain/src/tasks/motion_control` 中定义或处理 appendServoTarget, appendJointActuatorTargets, appendJointActuatorRuntimeTargets。 [211](#ref-0211)
- `ESP32-P4/ServoBoardMain/src/tasks/motion_control/pid.c` — 134 行 · source · pid：在 `ESP32-P4/ServoBoardMain/src/tasks/motion_control` 中定义或处理 limitFloat, parametersAreFinite, PID_Calc_Clear, f_PID_Calculate。 [212](#ref-0212)
- `ESP32-P4/ServoBoardMain/src/tasks/motion_control/pid.h` — 65 行 · source · pid：在 `ESP32-P4/ServoBoardMain/src/tasks/motion_control` 中定义或处理 PID_Calc_Clear, f_PID_Calculate。 [213](#ref-0213)
- `ESP32-P4/ServoBoardMain/src/tasks/runtime_monitor/RuntimeMonitorService.cpp` — 547 行 · source · RuntimeMonitorService：在 `ESP32-P4/ServoBoardMain/src/tasks/runtime_monitor` 中定义或处理 clampUs, sampleCurrentTaskStackFreeBytes, sampleTaskStacks, sampleCpuUsage。 [214](#ref-0214)
- `ESP32-P4/ServoBoardMain/src/tasks/runtime_monitor/RuntimeMonitorService.h` — 14 行 · source · RuntimeMonitorService：在 `ESP32-P4/ServoBoardMain/src/tasks/runtime_monitor` 中定义或处理 stepRuntimeMonitorService。 [215](#ref-0215)
- `ESP32-P4/ServoBoardMain/src/tasks/runtime_monitor/RuntimeMonitorTask.cpp` — 34 行 · source · RuntimeMonitorTask：在 `ESP32-P4/ServoBoardMain/src/tasks/runtime_monitor` 中定义或处理 runtimeMonitorTask。 [216](#ref-0216)
- `ESP32-P4/ServoBoardMain/src/tasks/runtime_monitor/RuntimeMonitorTask.h` — 9 行 · source · RuntimeMonitorTask：在 `ESP32-P4/ServoBoardMain/src/tasks/runtime_monitor` 中定义或处理 runtimeMonitorTask。 [217](#ref-0217)
- `ESP32-P4/ServoBoardMain/src/tasks/s3_can_rx/EncoderHealthMonitor.cpp` — 107 行 · source · EncoderHealthMonitor：在 `ESP32-P4/ServoBoardMain/src/tasks/s3_can_rx` 中定义或处理 encoderChannelMask, publishEncoderHealth, updateEncoderHealthFromSnapshot, markEncoderSnapshotBad。 [218](#ref-0218)
- `ESP32-P4/ServoBoardMain/src/tasks/s3_can_rx/EncoderHealthMonitor.h` — 34 行 · source · EncoderHealthMonitor：在 `ESP32-P4/ServoBoardMain/src/tasks/s3_can_rx` 中定义或处理 updateEncoderHealthFromSnapshot, markEncoderSnapshotBad, applyEncoderHealthTimeout, clearEncoderHealth。 [219](#ref-0219)
- `ESP32-P4/ServoBoardMain/src/tasks/s3_can_rx/S3CanRxService.cpp` — 766 行 · source · S3CanRxService：在 `ESP32-P4/ServoBoardMain/src/tasks/s3_can_rx` 中定义或处理 resetErrorDetailReassembly, resetEncoderReassembly, resetTactileReassembly, resetTactileFullReassembly。 [220](#ref-0220)
- `ESP32-P4/ServoBoardMain/src/tasks/s3_can_rx/S3CanRxService.h` — 17 行 · source · S3CanRxService：在 `ESP32-P4/ServoBoardMain/src/tasks/s3_can_rx` 中定义或处理 initS3CanRxService, stepS3CanRxService。 [221](#ref-0221)
- `ESP32-P4/ServoBoardMain/src/tasks/s3_can_rx/S3CanRxTask.cpp` — 37 行 · source · S3CanRxTask：在 `ESP32-P4/ServoBoardMain/src/tasks/s3_can_rx` 中定义或处理 s3CanRxTask。 [222](#ref-0222)
- `ESP32-P4/ServoBoardMain/src/tasks/s3_can_rx/S3CanRxTask.h` — 13 行 · source · S3CanRxTask：在 `ESP32-P4/ServoBoardMain/src/tasks/s3_can_rx` 中定义或处理 s3CanRxTask。 [223](#ref-0223)
- `ESP32-P4/ServoBoardMain/src/tasks/servo_bus/ParallelServoFeedbackCycle.cpp` — 130 行 · source · ParallelServoFeedbackCycle：位于 `ESP32-P4/ServoBoardMain/src/tasks/servo_bus`，具体职责需沿源码核对。 [224](#ref-0224)
- `ESP32-P4/ServoBoardMain/src/tasks/servo_bus/ParallelServoFeedbackCycle.h` — 45 行 · source · ParallelServoFeedbackCycle：在 `ESP32-P4/ServoBoardMain/src/tasks/servo_bus` 中定义或处理 begin, collect, abort, active。 [225](#ref-0225)
- `ESP32-P4/ServoBoardMain/src/tasks/servo_bus/ServoBusRouting.cpp` — 155 行 · source · ServoBusRouting：在 `ESP32-P4/ServoBoardMain/src/tasks/servo_bus` 中定义或处理 publishServoRouteSnapshot, addServoReadTarget, restoreServoMultiTurnFromHost, beginServoBuses。 [226](#ref-0226)
- `ESP32-P4/ServoBoardMain/src/tasks/servo_bus/ServoBusRouting.h` — 33 行 · source · ServoBusRouting：在 `ESP32-P4/ServoBoardMain/src/tasks/servo_bus` 中定义或处理 publishServoRouteSnapshot, addServoReadTarget, restoreServoMultiTurnFromHost, beginServoBuses。 [227](#ref-0227)
- `ESP32-P4/ServoBoardMain/src/tasks/servo_bus/ServoBusService.cpp` — 1220 行 · source · ServoBusService：在 `ESP32-P4/ServoBoardMain/src/tasks/servo_bus` 中定义或处理 logServoSafetyReject, batchMotorMask, reportCalibrationTargetResult, reportHostControlHoldResult。 [228](#ref-0228)
- `ESP32-P4/ServoBoardMain/src/tasks/servo_bus/ServoBusService.h` — 36 行 · source · ServoBusService：在 `ESP32-P4/ServoBoardMain/src/tasks/servo_bus` 中定义或处理 initServoBusService, stepServoBusService, servoBusFeedbackCollectDue, servoBusFeedbackWaitMs。 [229](#ref-0229)
- `ESP32-P4/ServoBoardMain/src/tasks/servo_bus/ServoBusTask.cpp` — 86 行 · source · ServoBusTask：在 `ESP32-P4/ServoBoardMain/src/tasks/servo_bus` 中定义或处理 servoBusTask。 [230](#ref-0230)
- `ESP32-P4/ServoBoardMain/src/tasks/servo_bus/ServoBusTask.h` — 13 行 · source · ServoBusTask：在 `ESP32-P4/ServoBoardMain/src/tasks/servo_bus` 中定义或处理 servoBusTask。 [231](#ref-0231)
- `ESP32-P4/ServoBoardMain/src/tasks/servo_bus/ServoDiscoveryManager.cpp` — 236 行 · source · ServoDiscoveryManager：位于 `ESP32-P4/ServoBoardMain/src/tasks/servo_bus`，具体职责需沿源码核对。 [232](#ref-0232)
- `ESP32-P4/ServoBoardMain/src/tasks/servo_bus/ServoDiscoveryManager.h` — 52 行 · source · ServoDiscoveryManager：在 `ESP32-P4/ServoBoardMain/src/tasks/servo_bus` 中定义或处理 begin, requestMotorScan, step, active。 [233](#ref-0233)
- `ESP32-P4/ServoBoardMain/src/tasks/servo_bus/ServoFeedbackPublisher.cpp` — 267 行 · source · ServoFeedbackPublisher：在 `ESP32-P4/ServoBoardMain/src/tasks/servo_bus` 中定义或处理 collectTurnOverflowMask, collectOnlineMotorMask, publishRecoveryState, publishTurnOverflowState。 [234](#ref-0234)
- `ESP32-P4/ServoBoardMain/src/tasks/servo_bus/ServoFeedbackPublisher.h` — 38 行 · source · ServoFeedbackPublisher：在 `ESP32-P4/ServoBoardMain/src/tasks/servo_bus` 中定义或处理 collectOnlineMotorMask, publishRecoveryState, publishTurnOverflowState, publishFeedback。 [235](#ref-0235)
- `ESP32-P4/ServoBoardMain/src/tasks/servo_bus/ServoRouteTable.cpp` — 211 行 · source · ServoRouteTable：在 `ESP32-P4/ServoBoardMain/src/tasks/servo_bus` 中定义或处理 popcount27。 [236](#ref-0236)
- `ESP32-P4/ServoBoardMain/src/tasks/servo_bus/ServoRouteTable.h` — 36 行 · source · ServoRouteTable：在 `ESP32-P4/ServoBoardMain/src/tasks/servo_bus` 中定义或处理 begin, beginFullScan, beginMotorScan, recordProbe。 [237](#ref-0237)
- `ESP32-P4/ServoBoardMain/src/tasks/system_state/DirectCommandCoordinator.cpp` — 257 行 · source · DirectCommandCoordinator：在 `ESP32-P4/ServoBoardMain/src/tasks/system_state` 中定义或处理 resetCommandState, setCommandModeJoint, setCommandModeDirect, clearPendingDirectCommand。 [238](#ref-0238)
- `ESP32-P4/ServoBoardMain/src/tasks/system_state/DirectCommandCoordinator.h` — 42 行 · source · DirectCommandCoordinator：在 `ESP32-P4/ServoBoardMain/src/tasks/system_state` 中定义或处理 resetCommandState, setCommandModeJoint, setCommandModeDirect, clearPendingDirectCommand。 [239](#ref-0239)
- `ESP32-P4/ServoBoardMain/src/tasks/system_state/FeedbackReadyDiagnostics.cpp` — 106 行 · source · FeedbackReadyDiagnostics：在 `ESP32-P4/ServoBoardMain/src/tasks/system_state` 中定义或处理 logFeedbackReadyReject。 [240](#ref-0240)
- `ESP32-P4/ServoBoardMain/src/tasks/system_state/FeedbackReadyDiagnostics.h` — 25 行 · source · FeedbackReadyDiagnostics：在 `ESP32-P4/ServoBoardMain/src/tasks/system_state` 中定义或处理 logFeedbackReadyReject。 [241](#ref-0241)
- `ESP32-P4/ServoBoardMain/src/tasks/system_state/HostLeaseSupervisor.cpp` — 320 行 · source · HostLeaseSupervisor：在 `ESP32-P4/ServoBoardMain/src/tasks/system_state` 中定义或处理 resetHostControlLeaseRuntime, prepareHostControlCommandAdmission, beginHostControlHold, cancelInternalMotionForHostLeaseLoss。 [242](#ref-0242)
- `ESP32-P4/ServoBoardMain/src/tasks/system_state/HostLeaseSupervisor.h` — 41 行 · source · HostLeaseSupervisor：在 `ESP32-P4/ServoBoardMain/src/tasks/system_state` 中定义或处理 resetHostControlLeaseRuntime, prepareHostControlCommandAdmission, updateHostControlLease。 [243](#ref-0243)
- `ESP32-P4/ServoBoardMain/src/tasks/system_state/RopeInitController.cpp` — 328 行 · source · RopeInitController：在 `ESP32-P4/ServoBoardMain/src/tasks/system_state` 中定义或处理 appendMotorTarget, completedDualJointMask, publishNextDualZeroTarget, finish。 [244](#ref-0244)
- `ESP32-P4/ServoBoardMain/src/tasks/system_state/RopeInitController.h` — 29 行 · source · RopeInitController：在 `ESP32-P4/ServoBoardMain/src/tasks/system_state` 中定义或处理 startRopeInitController, stepRopeInitController, cancelRopeInitController, ropeInitActiveScopeRejectReason。 [245](#ref-0245)
- `ESP32-P4/ServoBoardMain/src/tasks/system_state/SystemEventDispatcher.cpp` — 1215 行 · source · SystemEventDispatcher：在 `ESP32-P4/ServoBoardMain/src/tasks/system_state` 中定义或处理 activeJointsInControlWorkspace, idleOrReadyState, logManualReferenceReject。 [246](#ref-0246)
- `ESP32-P4/ServoBoardMain/src/tasks/system_state/SystemEventDispatcher.h` — 33 行 · source · SystemEventDispatcher：在 `ESP32-P4/ServoBoardMain/src/tasks/system_state` 中定义或处理 handleSystemEvent, setSystemState, setCalibrationStatus, enterFaultHold。 [247](#ref-0247)
- `ESP32-P4/ServoBoardMain/src/tasks/system_state/SystemStateCoordinator.cpp` — 109 行 · source · SystemStateCoordinator：在 `ESP32-P4/ServoBoardMain/src/tasks/system_state` 中定义或处理 initSystemStateCoordinator, stepSystemStateCoordinator。 [248](#ref-0248)
- `ESP32-P4/ServoBoardMain/src/tasks/system_state/SystemStateCoordinator.h` — 31 行 · source · SystemStateCoordinator：在 `ESP32-P4/ServoBoardMain/src/tasks/system_state` 中定义或处理 initSystemStateCoordinator, stepSystemStateCoordinator。 [249](#ref-0249)
- `ESP32-P4/ServoBoardMain/src/tasks/system_state/SystemStateTask.cpp` — 58 行 · source · SystemStateTask：在 `ESP32-P4/ServoBoardMain/src/tasks/system_state` 中定义或处理 systemStateTask。 [250](#ref-0250)
- `ESP32-P4/ServoBoardMain/src/tasks/system_state/SystemStateTask.h` — 17 行 · source · SystemStateTask：在 `ESP32-P4/ServoBoardMain/src/tasks/system_state` 中定义或处理 systemStateTask。 [251](#ref-0251)
- `ESP32-P4/libraries/FTServo_Arduino/examples/HLSCL/Broadcast/Broadcast.ino` — 22 行 · third_party · Broadcast：在 `ESP32-P4/libraries/FTServo_Arduino/examples/HLSCL/Broadcast` 中定义或处理 setup, loop。 [254](#ref-0254)
- `ESP32-P4/libraries/FTServo_Arduino/examples/HLSCL/CalibrationOfs/CalibrationOfs.ino` — 28 行 · third_party · CalibrationOfs：在 `ESP32-P4/libraries/FTServo_Arduino/examples/HLSCL/CalibrationOfs` 中定义或处理 setup, loop。 [255](#ref-0255)
- `ESP32-P4/libraries/FTServo_Arduino/examples/HLSCL/FeedBack/FeedBack.ino` — 148 行 · third_party · FeedBack：在 `ESP32-P4/libraries/FTServo_Arduino/examples/HLSCL/FeedBack` 中定义或处理 setup, loop。 [256](#ref-0256)
- `ESP32-P4/libraries/FTServo_Arduino/examples/HLSCL/Ping/Ping.ino` — 34 行 · third_party · Ping：在 `ESP32-P4/libraries/FTServo_Arduino/examples/HLSCL/Ping` 中定义或处理 setup, loop。 [257](#ref-0257)
- `ESP32-P4/libraries/FTServo_Arduino/examples/HLSCL/ProgramEprom/ProgramEprom.ino` — 27 行 · third_party · ProgramEprom：在 `ESP32-P4/libraries/FTServo_Arduino/examples/HLSCL/ProgramEprom` 中定义或处理 setup, loop。 [258](#ref-0258)
- `ESP32-P4/libraries/FTServo_Arduino/examples/HLSCL/RegWritePos/RegWritePos.ino` — 26 行 · third_party · RegWritePos：在 `ESP32-P4/libraries/FTServo_Arduino/examples/HLSCL/RegWritePos` 中定义或处理 setup, loop。 [259](#ref-0259)
- `ESP32-P4/libraries/FTServo_Arduino/examples/HLSCL/SyncRead/SyncRead.ino` — 45 行 · third_party · SyncRead：在 `ESP32-P4/libraries/FTServo_Arduino/examples/HLSCL/SyncRead` 中定义或处理 setup, loop。 [260](#ref-0260)
- `ESP32-P4/libraries/FTServo_Arduino/examples/HLSCL/SyncWritePos/SyncWritePos.ino` — 39 行 · third_party · SyncWritePos：在 `ESP32-P4/libraries/FTServo_Arduino/examples/HLSCL/SyncWritePos` 中定义或处理 setup, loop。 [261](#ref-0261)
- `ESP32-P4/libraries/FTServo_Arduino/examples/HLSCL/SyncWriteSpe/SyncWriteSpe.ino` — 50 行 · third_party · SyncWriteSpe：在 `ESP32-P4/libraries/FTServo_Arduino/examples/HLSCL/SyncWriteSpe` 中定义或处理 setup, loop。 [262](#ref-0262)
- `ESP32-P4/libraries/FTServo_Arduino/examples/HLSCL/WriteEle/WriteEle.ino` — 31 行 · third_party · WriteEle：在 `ESP32-P4/libraries/FTServo_Arduino/examples/HLSCL/WriteEle` 中定义或处理 setup, loop。 [263](#ref-0263)
- `ESP32-P4/libraries/FTServo_Arduino/examples/HLSCL/WritePos/WritePos.ino` — 22 行 · third_party · WritePos：在 `ESP32-P4/libraries/FTServo_Arduino/examples/HLSCL/WritePos` 中定义或处理 setup, loop。 [264](#ref-0264)
- `ESP32-P4/libraries/FTServo_Arduino/examples/HLSCL/WriteSpe/WriteSpe.ino` — 31 行 · third_party · WriteSpe：在 `ESP32-P4/libraries/FTServo_Arduino/examples/HLSCL/WriteSpe` 中定义或处理 setup, loop。 [265](#ref-0265)
- `ESP32-P4/libraries/FTServo_Arduino/examples/SCSCL/Broadcast/Broadcast.ino` — 25 行 · third_party · Broadcast：在 `ESP32-P4/libraries/FTServo_Arduino/examples/SCSCL/Broadcast` 中定义或处理 setup, loop。 [266](#ref-0266)
- `ESP32-P4/libraries/FTServo_Arduino/examples/SCSCL/FeedBack/FeedBack.ino` — 147 行 · third_party · FeedBack：在 `ESP32-P4/libraries/FTServo_Arduino/examples/SCSCL/FeedBack` 中定义或处理 setup, loop。 [267](#ref-0267)
- `ESP32-P4/libraries/FTServo_Arduino/examples/SCSCL/Ping/Ping.ino` — 33 行 · third_party · Ping：在 `ESP32-P4/libraries/FTServo_Arduino/examples/SCSCL/Ping` 中定义或处理 setup, loop。 [268](#ref-0268)
- `ESP32-P4/libraries/FTServo_Arduino/examples/SCSCL/ProgramEprom/ProgramEprom.ino` — 28 行 · third_party · ProgramEprom：在 `ESP32-P4/libraries/FTServo_Arduino/examples/SCSCL/ProgramEprom` 中定义或处理 setup, loop。 [269](#ref-0269)
- `ESP32-P4/libraries/FTServo_Arduino/examples/SCSCL/RegWritePos/RegWritePos.ino` — 29 行 · third_party · RegWritePos：在 `ESP32-P4/libraries/FTServo_Arduino/examples/SCSCL/RegWritePos` 中定义或处理 setup, loop。 [270](#ref-0270)
- `ESP32-P4/libraries/FTServo_Arduino/examples/SCSCL/SyncWritePos/SyncWritePos.ino` — 37 行 · third_party · SyncWritePos：在 `ESP32-P4/libraries/FTServo_Arduino/examples/SCSCL/SyncWritePos` 中定义或处理 setup, loop。 [271](#ref-0271)
- `ESP32-P4/libraries/FTServo_Arduino/examples/SCSCL/WritePWM/WritePWM.ino` — 34 行 · third_party · WritePWM：在 `ESP32-P4/libraries/FTServo_Arduino/examples/SCSCL/WritePWM` 中定义或处理 setup, loop。 [272](#ref-0272)
- `ESP32-P4/libraries/FTServo_Arduino/examples/SCSCL/WritePos/WritePos.ino` — 25 行 · third_party · WritePos：在 `ESP32-P4/libraries/FTServo_Arduino/examples/SCSCL/WritePos` 中定义或处理 setup, loop。 [273](#ref-0273)
- `ESP32-P4/libraries/FTServo_Arduino/examples/SMS_STS/Broadcast/Broadcast.ino` — 22 行 · third_party · Broadcast：在 `ESP32-P4/libraries/FTServo_Arduino/examples/SMS_STS/Broadcast` 中定义或处理 setup, loop。 [274](#ref-0274)
- `ESP32-P4/libraries/FTServo_Arduino/examples/SMS_STS/CalibrationOfs/CalibrationOfs.ino` — 25 行 · third_party · CalibrationOfs：在 `ESP32-P4/libraries/FTServo_Arduino/examples/SMS_STS/CalibrationOfs` 中定义或处理 setup, loop。 [275](#ref-0275)
- `ESP32-P4/libraries/FTServo_Arduino/examples/SMS_STS/FeedBack/FeedBack.ino` — 148 行 · third_party · FeedBack：在 `ESP32-P4/libraries/FTServo_Arduino/examples/SMS_STS/FeedBack` 中定义或处理 setup, loop。 [276](#ref-0276)
- `ESP32-P4/libraries/FTServo_Arduino/examples/SMS_STS/Ping/Ping.ino` — 33 行 · third_party · Ping：在 `ESP32-P4/libraries/FTServo_Arduino/examples/SMS_STS/Ping` 中定义或处理 setup, loop。 [277](#ref-0277)
- `ESP32-P4/libraries/FTServo_Arduino/examples/SMS_STS/ProgramEprom/ProgramEprom.ino` — 27 行 · third_party · ProgramEprom：在 `ESP32-P4/libraries/FTServo_Arduino/examples/SMS_STS/ProgramEprom` 中定义或处理 setup, loop。 [278](#ref-0278)
- `ESP32-P4/libraries/FTServo_Arduino/examples/SMS_STS/RegWritePos/RegWritePos.ino` — 26 行 · third_party · RegWritePos：在 `ESP32-P4/libraries/FTServo_Arduino/examples/SMS_STS/RegWritePos` 中定义或处理 setup, loop。 [279](#ref-0279)
- `ESP32-P4/libraries/FTServo_Arduino/examples/SMS_STS/SyncRead/SyncRead.ino` — 45 行 · third_party · SyncRead：在 `ESP32-P4/libraries/FTServo_Arduino/examples/SMS_STS/SyncRead` 中定义或处理 setup, loop。 [280](#ref-0280)
- `ESP32-P4/libraries/FTServo_Arduino/examples/SMS_STS/SyncWritePos/SyncWritePos.ino` — 36 行 · third_party · SyncWritePos：在 `ESP32-P4/libraries/FTServo_Arduino/examples/SMS_STS/SyncWritePos` 中定义或处理 setup, loop。 [281](#ref-0281)
- `ESP32-P4/libraries/FTServo_Arduino/examples/SMS_STS/SyncWriteSpe/SyncWriteSpe.ino` — 47 行 · third_party · SyncWriteSpe：在 `ESP32-P4/libraries/FTServo_Arduino/examples/SMS_STS/SyncWriteSpe` 中定义或处理 setup, loop。 [282](#ref-0282)
- `ESP32-P4/libraries/FTServo_Arduino/examples/SMS_STS/WritePos/WritePos.ino` — 22 行 · third_party · WritePos：在 `ESP32-P4/libraries/FTServo_Arduino/examples/SMS_STS/WritePos` 中定义或处理 setup, loop。 [283](#ref-0283)
- `ESP32-P4/libraries/FTServo_Arduino/examples/SMS_STS/WriteSpe/WriteSpe.ino` — 31 行 · third_party · WriteSpe：在 `ESP32-P4/libraries/FTServo_Arduino/examples/SMS_STS/WriteSpe` 中定义或处理 setup, loop。 [284](#ref-0284)
- `ESP32-P4/libraries/FTServo_Arduino/src/HLSCL.cpp` — 260 行 · third_party · HLSCL：位于 `ESP32-P4/libraries/FTServo_Arduino/src`，具体职责需沿源码核对。 [286](#ref-0286)
- `ESP32-P4/libraries/FTServo_Arduino/src/HLSCL.h` — 89 行 · third_party · HLSCL：在 `ESP32-P4/libraries/FTServo_Arduino/src` 中定义或处理 WritePosEx, RegWritePosEx, SyncWritePosEx, SyncWriteSpe。 [287](#ref-0287)
- `ESP32-P4/libraries/FTServo_Arduino/src/INST.h` — 51 行 · third_party · INST：位于 `ESP32-P4/libraries/FTServo_Arduino/src`，具体职责需沿源码核对。 [288](#ref-0288)
- `ESP32-P4/libraries/FTServo_Arduino/src/SCS.cpp` — 480 行 · third_party · SCS：位于 `ESP32-P4/libraries/FTServo_Arduino/src`，具体职责需沿源码核对。 [289](#ref-0289)
- `ESP32-P4/libraries/FTServo_Arduino/src/SCS.h` — 72 行 · third_party · SCS：在 `ESP32-P4/libraries/FTServo_Arduino/src` 中定义或处理 genWrite, regWrite, RegWriteAction, syncWrite。 [290](#ref-0290)
- `ESP32-P4/libraries/FTServo_Arduino/src/SCSCL.cpp` — 203 行 · third_party · SCSCL：位于 `ESP32-P4/libraries/FTServo_Arduino/src`，具体职责需沿源码核对。 [291](#ref-0291)
- `ESP32-P4/libraries/FTServo_Arduino/src/SCSCL.h` — 77 行 · third_party · SCSCL：在 `ESP32-P4/libraries/FTServo_Arduino/src` 中定义或处理 WritePos, RegWritePos, SyncWritePos, PWMMode。 [292](#ref-0292)
- `ESP32-P4/libraries/FTServo_Arduino/src/SCSerial.cpp` — 115 行 · third_party · SCSerial：位于 `ESP32-P4/libraries/FTServo_Arduino/src`，具体职责需沿源码核对。 [293](#ref-0293)
- `ESP32-P4/libraries/FTServo_Arduino/src/SCSerial.h` — 39 行 · third_party · SCSerial：在 `ESP32-P4/libraries/FTServo_Arduino/src` 中定义或处理 writeSCS, readSCS, readSCSAvailable。 [294](#ref-0294)
- `ESP32-P4/libraries/FTServo_Arduino/src/SCServo.h` — 15 行 · third_party · SCServo：位于 `ESP32-P4/libraries/FTServo_Arduino/src`，具体职责需沿源码核对。 [295](#ref-0295)
- `ESP32-P4/libraries/FTServo_Arduino/src/SMS_STS.cpp` — 249 行 · third_party · SMS_STS：位于 `ESP32-P4/libraries/FTServo_Arduino/src`，具体职责需沿源码核对。 [296](#ref-0296)
- `ESP32-P4/libraries/FTServo_Arduino/src/SMS_STS.h` — 86 行 · third_party · SMS_STS：在 `ESP32-P4/libraries/FTServo_Arduino/src` 中定义或处理 WritePosEx, RegWritePosEx, SyncWritePosEx, SyncWriteSpe。 [297](#ref-0297)
- `ESP32-P4/tools/strip_utf8_bom.py` — 68 行 · source · strip_utf8_bom：在 `ESP32-P4/tools` 中定义或处理 iter_source_files, strip_bom, main。 [298](#ref-0298)
- `ESP32-P4/tools/watch_strip_utf8_bom.py` — 81 行 · source · watch_strip_utf8_bom：在 `ESP32-P4/tools` 中定义或处理 iter_source_files, strip_bom, scan_once, main。 [299](#ref-0299)
- `ESP32-S3/System/.theia/launch.json` — 8 行 · source · launch：位于 `ESP32-S3/System/.theia`，具体职责需沿源码核对。 [301](#ref-0301)
- `ESP32-S3/System/System.ino` — 65 行 · source · System：在 `ESP32-S3/System` 中定义或处理 setup, loop。 [302](#ref-0302)
- `ESP32-S3/System/src/config/Config.h` — 153 行 · source · Config：位于 `ESP32-S3/System/src/config`，具体职责需沿源码核对。 [303](#ref-0303)
- `ESP32-S3/System/src/hal/HalEncoders.cpp` — 349 行 · source · HalEncoders：位于 `ESP32-S3/System/src/hal`，具体职责需沿源码核对。 [304](#ref-0304)
- `ESP32-S3/System/src/hal/HalEncoders.h` — 86 行 · source · HalEncoders：在 `ESP32-S3/System/src/hal` 中定义或处理 begin, getData, setupSPI, setDemuxChannel。 [305](#ref-0305)
- `ESP32-S3/System/src/hal/HalSensorSpi.cpp` — 119 行 · source · HalSensorSpi：在 `ESP32-S3/System/src/hal` 中定义或处理 beginSpiDevice, beginTactileSensorSpi, beginEncoderSensorSpi。 [306](#ref-0306)
- `ESP32-S3/System/src/hal/HalSensorSpi.h` — 17 行 · source · HalSensorSpi：在 `ESP32-S3/System/src/hal` 中定义或处理 beginTactileSensorSpi, beginEncoderSensorSpi。 [307](#ref-0307)
- `ESP32-S3/System/src/hal/HalTWAI.cpp` — 435 行 · source · HalTWAI：在 `ESP32-S3/System/src/hal` 中定义或处理 tactileAxisMagnitude, writeFilteredTactileSummary, crc16Ccitt。 [308](#ref-0308)
- `ESP32-S3/System/src/hal/HalTWAI.h` — 65 行 · source · HalTWAI：在 `ESP32-S3/System/src/hal` 中定义或处理 begin, sendEncoderData, sendTactileSummary, sendTactileFullDump。 [309](#ref-0309)
- `ESP32-S3/System/src/hal/HalTactile.cpp` — 533 行 · source · HalTactile：在 `ESP32-S3/System/src/hal` 中定义或处理 makeSelectorPattern, configureSelectPinHighWithPullup, sensorForceLen, deriveGlobalFromForces。 [310](#ref-0310)
- `ESP32-S3/System/src/hal/HalTactile.h` — 94 行 · source · HalTactile：在 `ESP32-S3/System/src/hal` 中定义或处理 prepareBootSelectPins, begin, update, setupSPI。 [311](#ref-0311)
- `ESP32-S3/System/src/system/FirmwareLog.h` — 44 行 · source · FirmwareLog：位于 `ESP32-S3/System/src/system`，具体职责需沿源码核对。 [312](#ref-0312)
- `ESP32-S3/System/src/system/SystemTasks.cpp` — 48 行 · source · SystemTasks：在 `ESP32-S3/System/src/system` 中定义或处理 startSystemTasks。 [313](#ref-0313)
- `ESP32-S3/System/src/system/SystemTasks.h` — 15 行 · source · SystemTasks：在 `ESP32-S3/System/src/system` 中定义或处理 startSystemTasks。 [314](#ref-0314)
- `ESP32-S3/System/src/tasks/CanTask.cpp` — 94 行 · source · CanTask：在 `ESP32-S3/System/src/tasks` 中定义或处理 Task_CanBus。 [315](#ref-0315)
- `ESP32-S3/System/src/tasks/CanTask.h` — 14 行 · source · CanTask：在 `ESP32-S3/System/src/tasks` 中定义或处理 Task_CanBus。 [316](#ref-0316)
- `ESP32-S3/System/src/tasks/EncodersTask.cpp` — 78 行 · source · EncodersTask：在 `ESP32-S3/System/src/tasks` 中定义或处理 Task_Encoders。 [317](#ref-0317)
- `ESP32-S3/System/src/tasks/EncodersTask.h` — 14 行 · source · EncodersTask：在 `ESP32-S3/System/src/tasks` 中定义或处理 Task_Encoders。 [318](#ref-0318)
- `ESP32-S3/System/src/tasks/TactileTask.cpp` — 337 行 · source · TactileTask：在 `ESP32-S3/System/src/tasks` 中定义或处理 axisMagnitude, printForceZStats, writeFilteredSummaryAxes, crc16Ccitt。 [319](#ref-0319)
- `ESP32-S3/System/src/tasks/TactileTask.h` — 14 行 · source · TactileTask：在 `ESP32-S3/System/src/tasks` 中定义或处理 Task_Tactile。 [320](#ref-0320)
- `desktop/__init__.py` — 10 行 · source · __init__：位于 `desktop`，具体职责需沿源码核对。 [325](#ref-0325)
- `desktop/__main__.py` — 15 行 · source · __main__：位于 `desktop`，具体职责需沿源码核对。 [326](#ref-0326)
- `desktop/app/__init__.py` — 10 行 · source · __init__：位于 `desktop/app`，具体职责需沿源码核对。 [327](#ref-0327)
- `desktop/app/cli_monitor.py` — 423 行 · source · cli_monitor：在 `desktop/app` 中定义或处理 _KeyReader, _fmt_sensor_cell, _fmt_servo_cell, _fmt_telem_cell。 [328](#ref-0328)
- `desktop/app/entry.py` — 138 行 · source · entry：在 `desktop/app` 中定义或处理 _add_common_args, build_parser, _resolve_ros_topic, main。 [329](#ref-0329)
- `desktop/app/ui_runner.py` — 227 行 · source · ui_runner：在 `desktop/app` 中定义或处理 _schedule_ros2_warning, _configure_gui_diagnostics, Mode, _run_monitor_gui。 [330](#ref-0330)
- `desktop/core/__init__.py` — 10 行 · source · __init__：位于 `desktop/core`，具体职责需沿源码核对。 [331](#ref-0331)
- `desktop/core/angle_mapping.py` — 507 行 · source · angle_mapping：在 `desktop/core` 中定义或处理 _urdf_mapping_metadata, urdf_feedback_mapping_metadata, urdf_manual_scaled_mapping_metadata, JointAngleSpec。 [332](#ref-0332)
- `desktop/core/auto_calib_logger.py` — 105 行 · source · auto_calib_logger：在 `desktop/core` 中定义或处理 AutoCalibLogger。 [333](#ref-0333)
- `desktop/core/calibration_diagnostics.py` — 405 行 · source · calibration_diagnostics：在 `desktop/core` 中定义或处理 CalibrationDiagnosticSpec, _hex, _fields_formatter, _format_from_spec。 [334](#ref-0334)
- `desktop/core/calibration_snapshot.py` — 816 行 · source · calibration_snapshot：在 `desktop/core` 中定义或处理 CalibrationSnapshotError, CalibrationSnapshotStore, parse_mask, joint_calib_result_has_valid_tendon_lut。 [335](#ref-0335)
- `desktop/core/calibration_workflow.py` — 893 行 · source · calibration_workflow：在 `desktop/core` 中定义或处理 CalibrationWorkflowEvent, joint_calib_result_signature, _format_joints, _format_motors。 [336](#ref-0336)
- `desktop/core/comm_layer.py` — 1737 行 · source · comm_layer：在 `desktop/core` 中定义或处理 _parse_tactile_diagnostic_packet, _synthetic_contact_grid, _build_tactile_fingers, _u8_to_i8。 [337](#ref-0337)
- `desktop/core/communication/__init__.py` — 26 行 · source · __init__：位于 `desktop/core/communication`，具体职责需沿源码核对。 [338](#ref-0338)
- `desktop/core/communication/ack_tracker.py` — 105 行 · source · ack_tracker：在 `desktop/core/communication` 中定义或处理 AckTracker。 [339](#ref-0339)
- `desktop/core/communication/event_channel.py` — 43 行 · source · event_channel：在 `desktop/core/communication` 中定义或处理 EventChannel。 [340](#ref-0340)
- `desktop/core/communication/frame_decoder.py` — 84 行 · source · frame_decoder：在 `desktop/core/communication` 中定义或处理 FrameDecoder。 [341](#ref-0341)
- `desktop/core/communication/packet_router.py` — 51 行 · source · packet_router：在 `desktop/core/communication` 中定义或处理 RoutedPacket, P4PacketRouter。 [342](#ref-0342)
- `desktop/core/communication/serial_transport.py` — 101 行 · source · serial_transport：在 `desktop/core/communication` 中定义或处理 SerialTransport。 [343](#ref-0343)
- `desktop/core/communication/state_reducer.py` — 854 行 · source · state_reducer：在 `desktop/core/communication` 中定义或处理 StateReducer。 [344](#ref-0344)
- `desktop/core/core_logic.py` — 2164 行 · source · core_logic：在 `desktop/core` 中定义或处理 ManualPrepareResult, HandController。 [345](#ref-0345)
- `desktop/core/data_models.py` — 1059 行 · source · data_models：在 `desktop/core` 中定义或处理 tactile_band_shape, tactile_segment_shape, tactile_upsample_tip_grid_to_band0, tactile_upsample_pad_grid_to_band。 [346](#ref-0346)
- `desktop/core/data_recording/__init__.py` — 13 行 · source · __init__：位于 `desktop/core/data_recording`，具体职责需沿源码核对。 [347](#ref-0347)
- `desktop/core/data_recording/csv_exporter.py` — 109 行 · source · csv_exporter：在 `desktop/core/data_recording` 中定义或处理 CsvTopicWriter, CsvSessionExporter。 [348](#ref-0348)
- `desktop/core/data_recording/event_recorder.py` — 44 行 · source · event_recorder：在 `desktop/core/data_recording` 中定义或处理 EventRecorder。 [349](#ref-0349)
- `desktop/core/data_recording/motion_sampler.py` — 125 行 · source · motion_sampler：在 `desktop/core/data_recording` 中定义或处理 classify_motion_source, MotionCommandSampler。 [350](#ref-0350)
- `desktop/core/data_recording/packet_recorder.py` — 67 行 · source · packet_recorder：在 `desktop/core/data_recording` 中定义或处理 PacketRecorder。 [351](#ref-0351)
- `desktop/core/data_recording/recorder.py` — 285 行 · source · recorder：在 `desktop/core/data_recording` 中定义或处理 DataSessionRecorder, _host_time_text, _to_float_or_none。 [352](#ref-0352)
- `desktop/core/data_recording/topic_schema.py` — 227 行 · source · topic_schema：在 `desktop/core/data_recording` 中定义或处理 _wide_joint_fields, _wide_servo_fields, _wide_tactile_summary_fields。 [353](#ref-0353)
- `desktop/core/device_state_store.py` — 237 行 · source · device_state_store：在 `desktop/core` 中定义或处理 DeviceStateStoreError, circular_raw_distance, nearest_logical_abs, _fixed_int_list。 [354](#ref-0354)
- `desktop/core/generated_upper_protocol.py` — 140 行 · generated · generated_upper_protocol：位于 `desktop/core`，具体职责需沿源码核对。 [355](#ref-0355)
- `desktop/core/hand_backend.py` — 880 行 · source · hand_backend：在 `desktop/core` 中定义或处理 list_ports, merge_tactile_state, JointCommandResult, HandBackend。 [356](#ref-0356)
- `desktop/core/joint_target_adapter.py` — 188 行 · source · joint_target_adapter：在 `desktop/core` 中定义或处理 UrdfMappingIdentity, JointTargetAdaptation, NamedJointTargetAdaptation, active_mapping_identity。 [357](#ref-0357)
- `desktop/core/motor_mapping.py` — 48 行 · source · motor_mapping：在 `desktop/core` 中定义或处理 clamp_motor_abs, motor_tendon_shorten_abs_sign, motor_abs_after_tendon_action, motor_target_from_servo_feedback。 [358](#ref-0358)
- `desktop/core/protocol.py` — 1390 行 · source · protocol：在 `desktop/core` 中定义或处理 ControlMode, tendon_lut_field, directed_encoder_delta, _build_command_frame。 [359](#ref-0359)
- `desktop/core/safety_rules.py` — 99 行 · source · safety_rules：在 `desktop/core` 中定义或处理 safety_rule_table, explain_safety_rule, explain_direct_reject, explain_feedback_reject。 [360](#ref-0360)
- `desktop/core/session_log.py` — 94 行 · source · session_log：在 `desktop/core` 中定义或处理 SessionLog, log_session, close_session_log。 [361](#ref-0361)
- `desktop/examples/__init__.py` — 2 行 · source · __init__：位于 `desktop/examples`，具体职责需沿源码核对。 [362](#ref-0362)
- `desktop/examples/ros2_joint_target_sender.py` — 62 行 · source · ros2_joint_target_sender：在 `desktop/examples` 中定义或处理 main。 [363](#ref-0363)
- `desktop/paths.py` — 30 行 · source · paths：位于 `desktop`，具体职责需沿源码核对。 [364](#ref-0364)
- `desktop/replay/__init__.py` — 24 行 · source · __init__：位于 `desktop/replay`，具体职责需沿源码核对。 [366](#ref-0366)
- `desktop/replay/replay_controller.py` — 675 行 · source · replay_controller：在 `desktop/replay` 中定义或处理 replay_start_state_allowed, ReplayStatus, ReplayOutputSample, ReplayController。 [367](#ref-0367)
- `desktop/replay/replay_csv_loader.py` — 400 行 · source · replay_csv_loader：在 `desktop/replay` 中定义或处理 _is_true, _parse_int, _parse_host_time, _parse_joint_targets。 [368](#ref-0368)
- `desktop/replay/replay_models.py` — 121 行 · source · replay_models：在 `desktop/replay` 中定义或处理 _joint_values, ReplayFrame, ReplayPreparation, ReplayTrace。 [369](#ref-0369)
- `desktop/replay/replay_service.py` — 77 行 · source · replay_service：在 `desktop/replay` 中定义或处理 ReplayService。 [370](#ref-0370)
- `desktop/requirements.txt` — 9 行 · source · requirements：位于 `desktop`，具体职责需沿源码核对。 [371](#ref-0371)
- `desktop/resources/config/calibration_bundle.mmcal.json` — 1964 行 · source · calibration_bundle.mmcal：在 `desktop/resources/config` 中定义或处理 joint_calib_results, motor_abs_limit_min, motor_abs_limit_max, motor_abs_limit_valid_bitmap。 [372](#ref-0372)
- `desktop/resources/config/p4_safety_rules.json` — 933 行 · generated · p4_safety_rules：在 `desktop/resources/config` 中定义或处理 schema_version, description, rules。 [373](#ref-0373)
- `desktop/resources/config/product_fingerprint.json` — 14 行 · generated · product_fingerprint：在 `desktop/resources/config` 中定义或处理 schema_version, model_id, mapping_id, feedback_mapping_version。 [374](#ref-0374)
- `desktop/resources/config/robot_model_manifest.json` — 31 行 · source · robot_model_manifest：在 `desktop/resources/config` 中定义或处理 schema_version, model_id, mapping_id, feedback_mapping_version。 [375](#ref-0375)
- `desktop/resources/config/upper_protocol_schema.json` — 121 行 · source · upper_protocol_schema：在 `desktop/resources/config` 中定义或处理 schema_version, description, maintenance_note, frame。 [376](#ref-0376)
- `desktop/resources/config/urdf_joint_mapping.json` — 482 行 · generated · urdf_joint_mapping：在 `desktop/resources/config` 中定义或处理 schema_version, model_id, mapping_id, feedback_mapping_version。 [377](#ref-0377)
- `desktop/resources/hku_hand_v2_urdf/structure_urdf_adapter.json` — 29 行 · source · structure_urdf_adapter：在 `desktop/resources/hku_hand_v2_urdf` 中定义或处理 source_root, source_urdf, source_commit, adapter_urdf。 [405](#ref-0405)
- `desktop/resources/hku_hand_v2_urdf/urdf/hand.urdf` — 850 行 · source · hand：位于 `desktop/resources/hku_hand_v2_urdf/urdf`，具体职责需沿源码核对。 [406](#ref-0406)
- `desktop/ros2/__init__.py` — 10 行 · source · __init__：位于 `desktop/ros2`，具体职责需沿源码核对。 [407](#ref-0407)
- `desktop/ros2/joint_targets.py` — 191 行 · source · joint_targets：在 `desktop/ros2` 中定义或处理 Ros2JointTargetConfig, JointTargetCommandHandler, Ros2JointTargetBridge。 [408](#ref-0408)
- `desktop/ros2/raw_ik_target.py` — 140 行 · source · raw_ik_target：在 `desktop/ros2` 中定义或处理 Ros2RawIkTargetConfig, RawIkTargetBridge。 [409](#ref-0409)
- `desktop/tests/test_calibration_snapshot_v4_protocol.py` — 71 行 · source · test_calibration_snapshot_v4_protocol：在 `desktop/tests` 中定义或处理 _minimal_v4_result, test_calib_load_joint_uses_only_v4_payload, test_snapshot_parser_rejects_removed_v2_lengths, test_manual_entry_crc_matches_p4_contract_vectors。 [410](#ref-0410)
- `desktop/tests/test_control_trace_protocol.py` — 122 行 · source · test_control_trace_protocol：在 `desktop/tests` 中定义或处理 _business_payload, test_control_trace_subscription_has_explicit_on_off_payload, test_control_trace_parser_uses_fixed_payload_and_centi_degrees, test_control_trace_reducer_keeps_dedicated_p4_timestamps。 [411](#ref-0411)
- `desktop/tests/test_manual_reference_ack_scope.py` — 46 行 · source · test_manual_reference_ack_scope：在 `desktop/tests` 中定义或处理 _Owner, test_manual_single_ack_preserves_unrelated_calibration。 [412](#ref-0412)
- `desktop/tests/test_multiturn_runtime_identity.py` — 70 行 · source · test_multiturn_runtime_identity：在 `desktop/tests` 中定义或处理 _identity, test_multiturn_anchor_survives_firmware_and_calibration_update, test_multiturn_anchor_rejects_different_hardware_map。 [413](#ref-0413)
- `desktop/tests/test_p4_safety_rule_closure.py` — 74 行 · source · test_p4_safety_rule_closure：在 `desktop/tests` 中定义或处理 _load_checker, test_every_registered_rule_has_a_closed_safety_path, test_collision_rule_uses_dedicated_state_machine_contract, test_generated_index_contains_every_rule。 [414](#ref-0414)
- `desktop/tests/test_rope_init_compatibility.py` — 23 行 · source · test_rope_init_compatibility：在 `desktop/tests` 中定义或处理 test_legacy_rope_init_command_remains_available, test_legacy_rope_init_rejects_empty_scope。 [415](#ref-0415)
- `desktop/tests/test_safety_event_protocol.py` — 50 行 · source · test_safety_event_protocol：在 `desktop/tests` 中定义或处理 test_safety_event_packet_round_trip_contract, test_safety_event_parser_rejects_wrong_length。 [416](#ref-0416)
- `desktop/tests/test_safety_rules.py` — 33 行 · source · test_safety_rules：在 `desktop/tests` 中定义或处理 test_safety_rule_ids_are_unique_and_stable, test_reject_reasons_use_central_explanations。 [417](#ref-0417)
- `desktop/tests/test_state_sync_protocol.py` — 39 行 · source · test_state_sync_protocol：在 `desktop/tests` 中定义或处理 test_state_sync_request_encodes_big_endian_mask, test_state_sync_request_rejects_unknown_bits, test_task_diag_subscription_has_explicit_on_off_payload。 [418](#ref-0418)
- `desktop/tests/test_task_runtime_protocol.py` — 44 行 · source · test_task_runtime_protocol：在 `desktop/tests` 中定义或处理 _build_v4_payload, test_task_diag_v4_decodes_window_metrics, test_task_diag_rejects_legacy_or_truncated_payload。 [419](#ref-0419)
- `desktop/tests/test_task_runtime_window_reducer.py` — 61 行 · source · test_task_runtime_window_reducer：在 `desktop/tests` 中定义或处理 _Owner, _reducer, _packets, test_task_runtime_is_published_only_after_full_window。 [420](#ref-0420)
- `desktop/tests/test_ui_status_labels.py` — 18 行 · source · test_ui_status_labels：在 `desktop/tests` 中定义或处理 test_status_mask_summary_keeps_all_objects_in_detail, test_status_labels_use_full_english_words。 [421](#ref-0421)
- `desktop/ui/__init__.py` — 10 行 · source · __init__：位于 `desktop/ui`，具体职责需沿源码核对。 [422](#ref-0422)
- `desktop/ui/official_tactile_layout.py` — 178 行 · source · official_tactile_layout：在 `desktop/ui` 中定义或处理 OfficialTactilePoint, OfficialTactileLayout, _numbers, _candidate_bundle_paths。 [423](#ref-0423)
- `desktop/ui/ui_app.py` — 2407 行 · source · ui_app：在 `desktop/ui` 中定义或处理 HandGUI。 [425](#ref-0425)
- `desktop/ui/ui_context.py` — 108 行 · source · ui_context：在 `desktop/ui` 中定义或处理 HandGuiContext。 [426](#ref-0426)
- `desktop/ui/ui_control_calibration.py` — 1012 行 · source · ui_control_calibration：在 `desktop/ui` 中定义或处理 CalibrationWorkflowController。 [427](#ref-0427)
- `desktop/ui/ui_control_commands.py` — 306 行 · source · ui_control_commands：在 `desktop/ui` 中定义或处理 ControlCommandController。 [428](#ref-0428)
- `desktop/ui/ui_control_common.py` — 209 行 · source · ui_control_common：在 `desktop/ui` 中定义或处理 _ModeBody。 [429](#ref-0429)
- `desktop/ui/ui_control_device.py` — 101 行 · source · ui_control_device：在 `desktop/ui` 中定义或处理 DeviceControlBar。 [430](#ref-0430)
- `desktop/ui/ui_control_manual.py` — 662 行 · source · ui_control_manual：在 `desktop/ui` 中定义或处理 ControlPanelManualMixin。 [431](#ref-0431)
- `desktop/ui/ui_control_modes.py` — 110 行 · source · ui_control_modes：在 `desktop/ui` 中定义或处理 ManualControlView。 [432](#ref-0432)
- `desktop/ui/ui_control_panels.py` — 255 行 · source · ui_control_panels：在 `desktop/ui` 中定义或处理 ControlPanel。 [433](#ref-0433)
- `desktop/ui/ui_control_scope.py` — 280 行 · source · ui_control_scope：在 `desktop/ui` 中定义或处理 _motor_channel_valid, ControlScopeModel。 [434](#ref-0434)
- `desktop/ui/ui_control_state.py` — 1794 行 · source · ui_control_state：在 `desktop/ui` 中定义或处理 ControlPanelStateMixin。 [435](#ref-0435)
- `desktop/ui/ui_control_tables.py` — 426 行 · source · ui_control_tables：在 `desktop/ui` 中定义或处理 JointChannelTable, MotorChannelTable。 [436](#ref-0436)
- `desktop/ui/ui_control_target_model.py` — 80 行 · source · ui_control_target_model：在 `desktop/ui` 中定义或处理 JointTargetModel。 [437](#ref-0437)
- `desktop/ui/ui_control_test_panel.py` — 179 行 · source · ui_control_test_panel：在 `desktop/ui` 中定义或处理 ManualTestPanel。 [438](#ref-0438)
- `desktop/ui/ui_control_tests.py` — 426 行 · source · ui_control_tests：在 `desktop/ui` 中定义或处理 ManualTestPanelMixin。 [439](#ref-0439)
- `desktop/ui/ui_control_view.py` — 342 行 · source · ui_control_view：在 `desktop/ui` 中定义或处理 ControlPanelViewMixin。 [440](#ref-0440)
- `desktop/ui/ui_device_check_view.py` — 567 行 · source · ui_device_check_view：在 `desktop/ui` 中定义或处理 DeviceCheckView。 [441](#ref-0441)
- `desktop/ui/ui_dialogs.py` — 140 行 · source · ui_dialogs：在 `desktop/ui` 中定义或处理 _standard_pixmap, ScrollableMessageDialog, show_scrollable_message。 [442](#ref-0442)
- `desktop/ui/ui_joint_plot_view.py` — 178 行 · source · ui_joint_plot_view：在 `desktop/ui` 中定义或处理 JointRealtimePlotView。 [443](#ref-0443)
- `desktop/ui/ui_pose_view.py` — 247 行 · source · ui_pose_view：在 `desktop/ui` 中定义或处理 PoseSkeletonCanvas, Pose3DView。 [444](#ref-0444)
- `desktop/ui/ui_qt_compat.py` — 77 行 · source · ui_qt_compat：在 `desktop/ui` 中定义或处理 _try_pyside6, _try_pyqt5, configure_opengl_before_app, ensure_app。 [445](#ref-0445)
- `desktop/ui/ui_replay_view.py` — 327 行 · source · ui_replay_view：在 `desktop/ui` 中定义或处理 ReplayView。 [446](#ref-0446)
- `desktop/ui/ui_services.py` — 31 行 · source · ui_services：在 `desktop/ui` 中定义或处理 LatestStateBuffer。 [447](#ref-0447)
- `desktop/ui/ui_style.py` — 31 行 · source · ui_style：在 `desktop/ui` 中定义或处理 configure_qt_application, apply_main_window_defaults。 [448](#ref-0448)
- `desktop/ui/ui_tactile_view.py` — 1722 行 · source · ui_tactile_view：在 `desktop/ui` 中定义或处理 _TactileRenderJob, _TactileRenderResult, _TaxelPeak, _TaxelCenter。 [449](#ref-0449)
- `desktop/ui/ui_task_runtime.py` — 478 行 · source · ui_task_runtime：在 `desktop/ui` 中定义或处理 TaskRuntimeWindow。 [450](#ref-0450)
- `desktop/ui/ui_windows.py` — 259 行 · source · ui_windows：在 `desktop/ui` 中定义或处理 _DisplayWindow, DisplayWindowManager。 [451](#ref-0451)
- `desktop/visualize/__init__.py` — 1 行 · source · __init__：位于 `desktop/visualize`，具体职责需沿源码核对。 [452](#ref-0452)
- `desktop/visualize/analyze_single_motor_pid.py` — 723 行 · source · analyze_single_motor_pid：在 `desktop/visualize` 中定义或处理 JointTrace, ServoTrace, safe_float, safe_int。 [453](#ref-0453)
- `desktop/visualize/plot_hand_from_urdf.py` — 785 行 · source · plot_hand_from_urdf：在 `desktop/visualize` 中定义或处理 _default_joint_positions, _apply_overrides, apply_pose_view_background, fill_3d_axes_to_figure。 [454](#ref-0454)
- `desktop/visualize/plot_sine_response.py` — 821 行 · source · plot_sine_response：在 `desktop/visualize` 中定义或处理 SineMetrics, parse_joint, parse_motor, safe_float。 [455](#ref-0455)
- `desktop/visualize/plot_step_response.py` — 560 行 · source · plot_step_response：在 `desktop/visualize` 中定义或处理 StepMetrics, _format_ms, detect_step_indices, _first_crossing。 [456](#ref-0456)
- `desktop/visualize/plot_tendon_lut.py` — 358 行 · source · plot_tendon_lut：在 `desktop/visualize` 中定义或处理 ControlGains, parse_joint, _read_cpp_float, load_control_gains。 [457](#ref-0457)
- `desktop/visualize/urdf_kinematics.py` — 159 行 · source · urdf_kinematics：在 `desktop/visualize` 中定义或处理 _make_display_transform, _parse_floats, _rpy_to_rotation, _homogeneous_from_origin。 [458](#ref-0458)
- `manus/SDKClient_Windows/Main.cpp` — 42 行 · source · Main：在 `manus/SDKClient_Windows` 中定义或处理 main。 [529](#ref-0529)
- `manus/SDKClient_Windows/PlatformSpecific/ClientPlatformSpecific.hpp` — 82 行 · source · ClientPlatformSpecific：在 `manus/SDKClient_Windows/PlatformSpecific` 中定义或处理 PlatformSpecificInitialization, PlatformSpecificShutdown, UpdateInput, CopyString。 [530](#ref-0530)
- `manus/SDKClient_Windows/PlatformSpecific/Linux/ClientPlatformSpecific.cpp` — 559 行 · source · ClientPlatformSpecific：在 `manus/SDKClient_Windows/PlatformSpecific/Linux` 中定义或处理 HandleTerminationSignal, HandleInterruptSignal, HandleQuitSignal, HandleHangupSignal。 [531](#ref-0531)
- `manus/SDKClient_Windows/PlatformSpecific/Linux/ClientPlatformSpecificTypes.hpp` — 48 行 · source · ClientPlatformSpecificTypes：位于 `manus/SDKClient_Windows/PlatformSpecific/Linux`，具体职责需沿源码核对。 [532](#ref-0532)
- `manus/SDKClient_Windows/PlatformSpecific/Windows/ClientPlatformSpecific.cpp` — 447 行 · source · ClientPlatformSpecific：在 `manus/SDKClient_Windows/PlatformSpecific/Windows` 中定义或处理 DoesWindowHaveFocus, EnableVirtualTerminalProcessing。 [533](#ref-0533)
- `manus/SDKClient_Windows/PlatformSpecific/Windows/ClientPlatformSpecificTypes.hpp` — 38 行 · source · ClientPlatformSpecificTypes：位于 `manus/SDKClient_Windows/PlatformSpecific/Windows`，具体职责需沿源码核对。 [534](#ref-0534)
- `manus/SDKClient_Windows/SDKClient.cpp` — 3861 行 · source · SDKClient：位于 `manus/SDKClient_Windows`，具体职责需沿源码核对。 [536](#ref-0536)
- `manus/SDKClient_Windows/SDKClient.hpp` — 367 行 · source · SDKClient：在 `manus/SDKClient_Windows` 中定义或处理 OnConnectedCallback, OnDisconnectedCallback, OnLogCallback, OnSkeletonStreamCallback。 [537](#ref-0537)
- `manus/SDKMinimalClient_Windows/SDKMinimalClient.cpp` — 382 行 · source · SDKMinimalClient：在 `manus/SDKMinimalClient_Windows` 中定义或处理 main。 [542](#ref-0542)
- `manus/SDKMinimalClient_Windows/SDKMinimalClient.hpp` — 82 行 · source · SDKMinimalClient：在 `manus/SDKMinimalClient_Windows` 中定义或处理 Run, PrintRawSkeletonNodeInfo, OnRawSkeletonStreamCallback。 [543](#ref-0543)
- `manus/manus_bridge_test/CMakeLists.txt` — 61 行 · source · CMakeLists：位于 `manus/manus_bridge_test`，具体职责需沿源码核对。 [548](#ref-0548)
- `manus/manus_bridge_test/config/manus_bridge_config.json` — 17 行 · source · manus_bridge_config：在 `manus/manus_bridge_test/config` 中定义或处理 connection, print_hz, host_search_seconds, raw_skeleton_nodes_to_print。 [550](#ref-0550)
- `manus/manus_bridge_test/scripts/build_windows.ps1` — 52 行 · source · build_windows：位于 `manus/manus_bridge_test/scripts`，具体职责需沿源码核对。 [552](#ref-0552)
- `manus/manus_bridge_test/scripts/run_windows.ps1` — 30 行 · source · run_windows：位于 `manus/manus_bridge_test/scripts`，具体职责需沿源码核对。 [553](#ref-0553)
- `manus/manus_bridge_test/src/CsvRecorder.cpp` — 125 行 · source · CsvRecorder：位于 `manus/manus_bridge_test/src`，具体职责需沿源码核对。 [554](#ref-0554)
- `manus/manus_bridge_test/src/CsvRecorder.h` — 22 行 · source · CsvRecorder：在 `manus/manus_bridge_test/src` 中定义或处理 open, writeFrames, close, isOpen。 [555](#ref-0555)
- `manus/manus_bridge_test/src/DataPrinter.cpp` — 71 行 · source · DataPrinter：位于 `manus/manus_bridge_test/src`，具体职责需沿源码核对。 [556](#ref-0556)
- `manus/manus_bridge_test/src/DataPrinter.h` — 18 行 · source · DataPrinter：在 `manus/manus_bridge_test/src` 中定义或处理 shouldPrint, print。 [557](#ref-0557)
- `manus/manus_bridge_test/src/ManusClient.cpp` — 520 行 · source · ManusClient：位于 `manus/manus_bridge_test/src`，具体职责需沿源码核对。 [558](#ref-0558)
- `manus/manus_bridge_test/src/ManusClient.h` — 116 行 · source · ManusClient：在 `manus/manus_bridge_test/src` 中定义或处理 initialize, connect, shutdown, isConnected。 [559](#ref-0559)
- `manus/manus_bridge_test/src/TcpSender.cpp` — 314 行 · source · TcpSender：在 `manus/manus_bridge_test/src` 中定义或处理 writeNodeJson, writeRawSkeletonNodesJson, writeSkeletonDictJson。 [560](#ref-0560)
- `manus/manus_bridge_test/src/TcpSender.h` — 42 行 · source · TcpSender：在 `manus/manus_bridge_test/src` 中定义或处理 open, close, isEnabled, isConnected。 [561](#ref-0561)
- `manus/manus_bridge_test/src/UdpSender.cpp` — 170 行 · source · UdpSender：位于 `manus/manus_bridge_test/src`，具体职责需沿源码核对。 [562](#ref-0562)
- `manus/manus_bridge_test/src/UdpSender.h` — 36 行 · source · UdpSender：在 `manus/manus_bridge_test/src` 中定义或处理 open, close, isEnabled, sendFrame。 [563](#ref-0563)
- `manus/manus_bridge_test/src/main.cpp` — 399 行 · source · main：在 `manus/manus_bridge_test/src` 中定义或处理 handleSignal, printUsage, jsonUintValue, jsonBoolValue。 [564](#ref-0564)
- `manus/mmhand_manus_sdk/CMakeLists.txt` — 32 行 · source · CMakeLists：位于 `manus/mmhand_manus_sdk`，具体职责需沿源码核对。 [566](#ref-0566)
- `manus/mmhand_manus_sdk/build/windows/CMakeFiles/3.31.6-msvc6/CMakeCXXCompiler.cmake` — 101 行 · source · CMakeCXXCompiler：位于 `manus/mmhand_manus_sdk/build/windows/CMakeFiles/3.31.6-msvc6`，具体职责需沿源码核对。 [748](#ref-0748)
- `manus/mmhand_manus_sdk/build/windows/CMakeFiles/3.31.6-msvc6/CMakeRCCompiler.cmake` — 6 行 · source · CMakeRCCompiler：位于 `manus/mmhand_manus_sdk/build/windows/CMakeFiles/3.31.6-msvc6`，具体职责需沿源码核对。 [749](#ref-0749)
- `manus/mmhand_manus_sdk/build/windows/CMakeFiles/3.31.6-msvc6/CMakeSystem.cmake` — 15 行 · source · CMakeSystem：位于 `manus/mmhand_manus_sdk/build/windows/CMakeFiles/3.31.6-msvc6`，具体职责需沿源码核对。 [750](#ref-0750)
- `manus/mmhand_manus_sdk/build/windows/CMakeFiles/3.31.6-msvc6/CompilerIdCXX/CMakeCXXCompilerId.cpp` — 919 行 · source · CMakeCXXCompilerId：在 `manus/mmhand_manus_sdk/build/windows/CMakeFiles/3.31.6-msvc6/CompilerIdCXX` 中定义或处理 main。 [743](#ref-0743)
- `manus/mmhand_manus_sdk/build/windows/CMakeFiles/CMakeConfigureLog.yaml` — 150 行 · source · CMakeConfigureLog：位于 `manus/mmhand_manus_sdk/build/windows/CMakeFiles`，具体职责需沿源码核对。 [744](#ref-0744)
- `manus/mmhand_manus_sdk/build/windows/cmake_install.cmake` — 56 行 · source · cmake_install：位于 `manus/mmhand_manus_sdk/build/windows`，具体职责需沿源码核对。 [751](#ref-0751)
- `manus/mmhand_manus_sdk/include/mmhand_manus_sdk.h` — 32 行 · source · mmhand_manus_sdk：在 `manus/mmhand_manus_sdk/include` 中定义或处理 mmhand_manus_start, mmhand_manus_stop, mmhand_manus_is_connected, mmhand_manus_get_latest_frame_json。 [635](#ref-0635)
- `manus/mmhand_manus_sdk/scripts/build_windows.ps1` — 49 行 · source · build_windows：位于 `manus/mmhand_manus_sdk/scripts`，具体职责需沿源码核对。 [636](#ref-0636)
- `manus/mmhand_manus_sdk/src/mmhand_manus_sdk.cpp` — 342 行 · source · mmhand_manus_sdk：在 `manus/mmhand_manus_sdk/src` 中定义或处理 setError, writeNodeJson, writeRawSkeletonNodesJson, writeSkeletonDictJson。 [637](#ref-0637)
- `manus/third_party/ManusSDK/include/ManusSDK.h` — 1200 行 · third_party · ManusSDK：位于 `manus/third_party/ManusSDK/include`，具体职责需沿源码核对。 [638](#ref-0638)
- `manus/third_party/ManusSDK/include/ManusSDKTypeInitializers.h` — 396 行 · third_party · ManusSDKTypeInitializers：在 `manus/third_party/ManusSDK/include` 中定义或处理 ManusVec3_Init, ManusVec2_Init, ManusQuaternion_Init, ManusTransform_Init。 [639](#ref-0639)
- `manus/third_party/ManusSDK/include/ManusSDKTypes.h` — 1764 行 · third_party · ManusSDKTypes：位于 `manus/third_party/ManusSDK/include`，具体职责需沿源码核对。 [640](#ref-0640)
- `playground/Standalone/paxini_all_read_newboard/paxini_all_read_newboard.ino` — 292 行 · source · paxini_all_read_newboard：在 `playground/Standalone/paxini_all_read_newboard` 中定义或处理 calculateCRC8, useSelectFromLineLevel, setGroupEnable, setGroupCode。 [641](#ref-0641)
- `playground/paxini_all_read_2.12/paxini_all_read_2.12.ino` — 497 行 · source · paxini_all_read_2.12：在 `playground/paxini_all_read_2.12` 中定义或处理 setMux, calculateCRC8, spiReadRegisters, printForceData。 [642](#ref-0642)
- `playground/paxini_all_read_newboard.ino` — 292 行 · source · paxini_all_read_newboard：在 `playground` 中定义或处理 calculateCRC8, useSelectFromLineLevel, setGroupEnable, setGroupCode。 [643](#ref-0643)
- `playground/paxini_dist_scan/paxini_dist_scan.ino` — 251 行 · source · paxini_dist_scan：在 `playground/paxini_dist_scan` 中定义或处理 crc8, allSelectorsHigh, setSelectorPattern, readPattern。 [644](#ref-0644)
- `playground/paxini_manual_cs_scan/paxini_manual_cs_scan.ino` — 246 行 · source · paxini_manual_cs_scan：在 `playground/paxini_manual_cs_scan` 中定义或处理 configureSelectPinHighWithPullup, crc8, allSelectorsHigh, setSelectorPattern。 [645](#ref-0645)
- `playground/paxini_spi1_p1_test/paxini_spi1_p1_test.ino` — 193 行 · source · paxini_spi1_p1_test：在 `playground/paxini_spi1_p1_test` 中定义或处理 crc8, idleSelectorsHigh, setP1Group, readOne。 [646](#ref-0646)
- `playground/paxini_spi_scan/paxini_spi_scan.ino` — 286 行 · source · paxini_spi_scan：在 `playground/paxini_spi_scan` 中定义或处理 crc8, setEnable, setGroupCode, readOne。 [647](#ref-0647)
- `playground/tactile_sim_visualizer/simulate_tactile_dot_matrix.py` — 199 行 · source · simulate_tactile_dot_matrix：在 `playground/tactile_sim_visualizer` 中定义或处理 FrameData, TactileSignalGenerator, DotMatrixVisualizer, parse_args。 [649](#ref-0649)
- `pytest.ini` — 10 行 · source · pytest：位于 `.`，具体职责需沿源码核对。 [650](#ref-0650)
- `start_mmhand.cmd` — 30 行 · source · start_mmhand：位于 `.`，具体职责需沿源码核对。 [651](#ref-0651)
- `tests/conftest.py` — 6 行 · source · conftest：位于 `tests`，具体职责需沿源码核对。 [652](#ref-0652)
- `tests/desktop/algorithm/test_replay_controller.py` — 315 行 · source · test_replay_controller：在 `tests/desktop/algorithm` 中定义或处理 _trace, _ready_state, _Backend, test_replay_start_state_allows_ready_and_run_feedback。 [653](#ref-0653)
- `tests/desktop/algorithm/test_replay_csv_loader.py` — 338 行 · source · test_replay_csv_loader：在 `tests/desktop/algorithm` 中定义或处理 _targets, _write_csv, test_loader_restores_full_timeline_and_actual_output, test_loader_rejects_timestamp_rollback。 [654](#ref-0654)
- `tests/desktop/app/test_cli_monitor.py` — 130 行 · source · test_cli_monitor：在 `tests/desktop/app` 中定义或处理 FakeBackend, _RenderBackend, FakeKeyReader, test_cli_monitor_drives_backend_commands_only。 [655](#ref-0655)
- `tests/desktop/app/test_entry.py` — 30 行 · source · test_entry：在 `tests/desktop/app` 中定义或处理 test_cli_args_parse_ros2_auto_start, test_ui_args_parse_mode_and_ros2, test_ros2_is_enabled_by_default_and_can_be_disabled。 [656](#ref-0656)
- `tests/desktop/app/test_ui_runner_ros2.py` — 89 行 · source · test_ui_runner_ros2：在 `tests/desktop/app` 中定义或处理 test_replay_runner_forwards_ros2_configuration, test_ros2_startup_and_manual_fallback, test_ros2_warning_waits_for_gui_event_loop, test_gui_failure_still_stops_ros2_bridge。 [657](#ref-0657)
- `tests/desktop/app/test_windows_launcher.py` — 122 行 · source · test_windows_launcher：在 `tests/desktop/app` 中定义或处理 launcher, run_cmd, test_launcher_paths_setup_and_argument_forwarding, test_double_click_without_virtualenv_or_configuration。 [658](#ref-0658)
- `tests/desktop/core/test_angle_mapping.py` — 142 行 · source · test_angle_mapping：在 `tests/desktop/core` 中定义或处理 _active_urdf_limits_deg, test_all_21_mapping_limits_match_active_urdf, test_feedback_mapping_declares_current_v4_manual_contract, test_generated_p4_urdf_limits_are_current_and_crc_matches。 [659](#ref-0659)
- `tests/desktop/core/test_calib_snapshot_current.py` — 290 行 · source · test_calib_snapshot_current：在 `tests/desktop/core` 中定义或处理 test_motor_limit_load_is_scoped_to_update_bitmap, test_joint_calibration_current_protocol_round_trip, test_joint_calibration_preserves_large_logical_abs, test_legacy_joint_payload_lengths_are_rejected。 [660](#ref-0660)
- `tests/desktop/core/test_calibration_diag_protocol.py` — 25 行 · source · test_calibration_diag_protocol：在 `tests/desktop/core` 中定义或处理 test_calibration_diag_packet_carries_exact_reason_and_result_masks, test_calibration_diag_rejects_obsolete_short_packet。 [661](#ref-0661)
- `tests/desktop/core/test_calibration_diagnostics.py` — 48 行 · source · test_calibration_diagnostics：在 `tests/desktop/core` 中定义或处理 test_all_reserved_legacy_reasons_remain_readable, test_calibration_reason_titles_are_unique_and_english。 [662](#ref-0662)
- `tests/desktop/core/test_calibration_snapshot.py` — 212 行 · source · test_calibration_snapshot：在 `tests/desktop/core` 中定义或处理 _result, _current_payload, test_empty_auto_snapshot_is_not_loadable, test_partial_recalibration_preserves_unaffected_joint_and_limits。 [663](#ref-0663)
- `tests/desktop/core/test_calibration_workflow.py` — 465 行 · source · test_calibration_workflow：在 `tests/desktop/core` 中定义或处理 _Controller, _motor_for_joint, _motors_for_joint, _successful_state。 [664](#ref-0664)
- `tests/desktop/core/test_comm_layer_mapped_angles.py` — 193 行 · source · test_comm_layer_mapped_angles：在 `tests/desktop/core` 中定义或处理 test_split_state_reducer_can_resolve_protocol_parsers, _sensor_payload, _calibrated_model, test_servo_telemetry_packet_includes_signed_current_without_online_array。 [665](#ref-0665)
- `tests/desktop/core/test_communication_components.py` — 224 行 · source · test_communication_components：在 `tests/desktop/core` 中定义或处理 _FakeSerial, test_serial_transport_owns_open_write_and_close, test_tactile_transport_keeps_usb_cdc_control_lines_active, test_frame_decoder_handles_log_bytes_fragmentation_and_sticky_frames。 [666](#ref-0666)
- `tests/desktop/core/test_core_logic.py` — 811 行 · source · test_core_logic：在 `tests/desktop/core` 中定义或处理 _trusted_multiturn_state, test_calibrated_anchor_is_saved_only_after_explicit_capture_request, test_calibrated_anchors_are_saved_per_eligible_motor, test_startup_multiturn_recovery_waits_for_first_complete_route_scan。 [667](#ref-0667)
- `tests/desktop/core/test_device_state_store.py` — 83 行 · source · test_device_state_store：在 `tests/desktop/core` 中定义或处理 _identity, test_runtime_store_round_trip_and_backup, test_runtime_store_rejects_identity_or_integrity_mismatch, test_circular_raw_distance_handles_wrap。 [668](#ref-0668)
- `tests/desktop/core/test_direct_hold_and_release.py` — 89 行 · source · test_direct_hold_and_release：在 `tests/desktop/core` 中定义或处理 controller_with_motor, test_direct_enable_holds_exact_abs_and_ignores_unselected_unknown, test_direct_enable_rejects_untrusted_or_clamped_hold, test_release_only_connected_motor。 [669](#ref-0669)
- `tests/desktop/core/test_feedback_command_order.py` — 64 行 · source · test_feedback_command_order：在 `tests/desktop/core` 中定义或处理 test_latest_live_joint_target_stays_before_pending_feedback_start, test_joint_target_and_feedback_start_are_one_queue_item, test_latest_paired_joint_command_replaces_stale_target_and_run_request, test_latest_masked_joint_command_replaces_stale_full_hand_command。 [670](#ref-0670)
- `tests/desktop/core/test_joint_target_adapter.py` — 63 行 · source · test_joint_target_adapter：在 `tests/desktop/core` 中定义或处理 test_active_urdf_degrees_are_identity, test_active_urdf_radians_convert_to_degrees, test_legacy_target_space_is_rejected, test_semantic_thumb_names_use_canonical_j16_to_j20_order。 [671](#ref-0671)
- `tests/desktop/core/test_joint_workspace_guard.py` — 394 行 · source · test_joint_workspace_guard：在 `tests/desktop/core` 中定义或处理 _ready_state, test_workspace_uses_static_urdf_min_and_max, test_workspace_rejects_dynamic_min_from_removed_mapping, test_workspace_rejects_manual_crc_mismatch。 [672](#ref-0672)
- `tests/desktop/core/test_manual_reference_crc.py` — 21 行 · source · test_manual_reference_crc：在 `tests/desktop/core` 中定义或处理 test_manual_entry_crc_matches_p4_ieee_crc32_layout, test_manual_entry_crc_changes_only_with_bound_joint_data。 [673](#ref-0673)
- `tests/desktop/core/test_manual_reference_protocol.py` — 216 行 · source · test_manual_reference_protocol：在 `tests/desktop/core` 中定义或处理 test_manual_reference_command_contains_all_21_big_endian_pairs, test_manual_reference_command_rejects_partial_or_out_of_range_data, test_clear_manual_reference_uses_same_fixed_wire_length, test_system_status_parses_manual_nvs_extension_and_old_payload。 [674](#ref-0674)
- `tests/desktop/core/test_manual_scaled_mapping_v3.py` — 46 行 · source · test_manual_scaled_mapping_v3：在 `tests/desktop/core` 中定义或处理 directed_delta, build_range, test_directed_delta_handles_both_wrap_directions, test_manual_scaled_range_uses_requested_formula。 [675](#ref-0675)
- `tests/desktop/core/test_motion_sampler.py` — 47 行 · source · test_motion_sampler：在 `tests/desktop/core` 中定义或处理 _row, test_continuous_stream_preserves_every_input, test_discrete_ui_command_is_written_immediately, test_external_ros2_is_a_replayable_continuous_source。 [676](#ref-0676)
- `tests/desktop/core/test_packet_recording.py` — 86 行 · source · test_packet_recording：在 `tests/desktop/core` 中定义或处理 frames, test_five_frames_in_one_batch_have_five_original_timestamps, test_old_firmware_missing_detail_gap_and_retry, test_stop_drains_batch_and_keeps_s3_payload。 [677](#ref-0677)
- `tests/desktop/core/test_pose_urdf_mapping.py` — 31 行 · source · test_pose_urdf_mapping：在 `tests/desktop/core` 中定义或处理 test_pose_mapping_uses_all_active_urdf_joint_names, test_pose_mapping_rejects_incomplete_joint_vector。 [678](#ref-0678)
- `tests/desktop/core/test_product_fingerprint_protocol.py` — 37 行 · source · test_product_fingerprint_protocol：在 `tests/desktop/core` 中定义或处理 test_parse_product_fingerprint_packet, test_reject_truncated_product_fingerprint_packet。 [679](#ref-0679)
- `tests/desktop/core/test_recording_timestamps.py` — 220 行 · source · test_recording_timestamps：在 `tests/desktop/core` 中定义或处理 test_split_p4_timestamp_from_current_packet, test_current_timestamped_packet_rejects_missing_trailer, test_protocol_ack_exposes_exact_p4_processing_time, test_csv_exposes_only_host_and_p4_time_axes。 [680](#ref-0680)
- `tests/desktop/core/test_scoped_multiturn_protocol.py` — 38 行 · source · test_scoped_multiturn_protocol：在 `tests/desktop/core` 中定义或处理 test_scoped_calibration_clear_encodes_motor_mask, test_scoped_calibration_clear_rejects_empty_mask, test_servo_logical_ranges_are_independent_per_motor。 [681](#ref-0681)
- `tests/desktop/core/test_tactile_summary_protocol.py` — 155 行 · source · test_tactile_summary_protocol：在 `tests/desktop/core` 中定义或处理 _summary_payload, _diag_payload, test_tactile_summary_parser_requires_and_returns_module_valid_mask, test_s3_direct_summary_uses_mask_instead_of_nonzero_force_for_online_state。 [682](#ref-0682)
- `tests/desktop/ros2/test_joint_targets.py` — 97 行 · source · test_joint_targets：在 `tests/desktop/ros2` 中定义或处理 FakeBackend, test_ros2_handler_sends_valid_21_degree_targets, test_ros2_handler_rejects_wrong_length_and_non_finite_values, test_ros2_handler_auto_start_only_once。 [683](#ref-0683)
- `tests/desktop/ros2/test_raw_ik_target.py` — 66 行 · source · test_raw_ik_target：在 `tests/desktop/ros2` 中定义或处理 FakeBackend, test_raw_ik_bridge_defaults_to_raw_ik_topic, test_raw_ik_bridge_handler_sends_directly_to_backend, test_raw_ik_bridge_can_accept_active_urdf_degrees_explicitly。 [684](#ref-0684)
- `tests/desktop/ui/test_calibration_failure_popup.py` — 157 行 · source · test_calibration_failure_popup：在 `tests/desktop/ui` 中定义或处理 test_popup_explains_result_span_failure_instead_of_generic_step_failure, test_every_firmware_calibration_reason_has_a_dedicated_popup_explanation, test_joint_step_fallback_uses_result_masks_not_payload_values, test_dual_exchange_stall_has_dedicated_payload_names。 [685](#ref-0685)
- `tests/desktop/ui/test_calibration_warning_popup.py` — 155 行 · source · test_calibration_warning_popup：在 `tests/desktop/ui` 中定义或处理 _WarningHarness, _state, test_calibration_warning_popup_explains_all_reasons_once, test_calibration_warning_popup_episode_resets_after_warning_clears。 [686](#ref-0686)
- `tests/desktop/ui/test_control_panel_modules.py` — 209 行 · source · test_control_panel_modules：在 `tests/desktop/ui` 中定义或处理 test_control_scope_model_orders_thumb_workflow_and_masks_motors, test_joint_target_model_uses_calibrated_min_and_clamps_invalid_values, test_device_control_bar_exposes_expected_buttons, test_joint_table_displays_ros2_targets_next_to_current。 [687](#ref-0687)
- `tests/desktop/ui/test_device_check_view.py` — 49 行 · source · test_device_check_view：在 `tests/desktop/ui` 中定义或处理 test_encoder_direction_display_uses_clear_arrows, test_device_check_replaces_visible_refresh_with_manual_prepare。 [688](#ref-0688)
- `tests/desktop/ui/test_enable_scope.py` — 44 行 · source · test_enable_scope：在 `tests/desktop/ui` 中定义或处理 test_scan_and_same_session_pause。 [689](#ref-0689)
- `tests/desktop/ui/test_encoder_common_bus_guard.py` — 83 行 · source · test_encoder_common_bus_guard：在 `tests/desktop/ui` 中定义或处理 _state, _GuardHarness, test_all_zero_all_online_is_confirmed_and_overlays_red_health, test_zero_snapshot_is_not_fault_when_firmware_does_not_report_all_online。 [690](#ref-0690)
- `tests/desktop/ui/test_feedback_collision_status.py` — 43 行 · source · test_feedback_collision_status：在 `tests/desktop/ui` 中定义或处理 _format_status, test_dual_tendon_j00_hold_is_visible_in_top_status, test_dual_tendon_j20_armed_is_visible_in_top_status。 [691](#ref-0691)
- `tests/desktop/ui/test_joint_target_edit_persistence.py` — 415 行 · source · test_joint_target_edit_persistence：在 `tests/desktop/ui` 中定义或处理 _Backend, _Context, _panel, test_manual_workflow_buttons_keep_signals_after_module_split。 [692](#ref-0692)
- `tests/desktop/ui/test_partial_calibration_result.py` — 261 行 · source · test_partial_calibration_result：在 `tests/desktop/ui` 中定义或处理 _FirmwareEpochHarness, _valid_tendon_lut_result, _ReadyStateHarness, _TextItem。 [693](#ref-0693)
- `tests/desktop/ui/test_pose_urdf_reload.py` — 61 行 · source · test_pose_urdf_reload：在 `tests/desktop/ui` 中定义或处理 _reload_harness, test_reload_urdf_clears_old_scene_and_resets_angles, test_reload_urdf_reports_load_failure。 [694](#ref-0694)
- `tests/desktop/ui/test_replay_view.py` — 65 行 · source · test_replay_view：在 `tests/desktop/ui` 中定义或处理 _ReplayService, _ControlPanel, _Context, test_replay_send_button_can_restart_from_run_feedback_state。 [695](#ref-0695)
- `tests/desktop/ui/test_rope_init_progress.py` — 13 行 · source · test_rope_init_progress：在 `tests/desktop/ui` 中定义或处理 test_manual_events_are_not_misclassified_as_calibration。 [696](#ref-0696)
- `tests/desktop/ui/test_scrollable_message_dialog.py` — 46 行 · source · test_scrollable_message_dialog：在 `tests/desktop/ui` 中定义或处理 test_long_message_uses_scrollable_read_only_body_and_visible_buttons, test_dialog_initial_size_stays_inside_available_screen。 [697](#ref-0697)
- `tests/desktop/ui/test_stop_paths.py` — 556 行 · source · test_stop_paths：在 `tests/desktop/ui` 中定义或处理 _FakeSignal, _FakeCheckBox, test_startup_fault_does_not_latch_estop_before_motion_starts, test_internal_p4_motion_fault_does_not_send_competing_host_estop。 [698](#ref-0698)
- `tests/desktop/ui/test_tactile_view.py` — 159 行 · source · test_tactile_view：在 `tests/desktop/ui` 中定义或处理 _Context, _tactile_state, test_tactile_view_removes_module_and_3d_controls_and_shows_summary_vectors, test_tactile_view_marks_full_taxel_data_without_changing_resultant_vectors。 [699](#ref-0699)
- `tests/desktop/ui/test_view_menu.py` — 56 行 · source · test_view_menu：在 `tests/desktop/ui` 中定义或处理 _ViewMenuOwner, test_view_menu_opens_task_runtime_outside_display_window_manager, test_release_gui_no_longer_defines_device_menu_factory。 [700](#ref-0700)
- `tests/desktop/visualize/test_analyze_single_motor_pid.py` — 83 行 · source · test_analyze_single_motor_pid：在 `tests/desktop/visualize` 中定义或处理 _write_csv, test_classifies_step_and_wave_targets, test_step_metrics_detect_rise_overshoot_and_settling, test_loads_debug_data_and_infers_single_motor。 [701](#ref-0701)
- `tests/desktop/visualize/test_plot_step_response.py` — 42 行 · source · test_plot_step_response：在 `tests/desktop/visualize` 中定义或处理 test_detect_step_indices_ignores_platform_samples, test_analyze_steps_handles_positive_and_negative_steps。 [702](#ref-0702)
- `tests/tools/test_desktop_architecture.py` — 9 行 · source · test_desktop_architecture：在 `tests/tools` 中定义或处理 test_desktop_architecture_contract。 [703](#ref-0703)
- `tests/tools/test_generate_robot_model.py` — 72 行 · source · test_generate_robot_model：在 `tests/tools` 中定义或处理 _args, test_generator_outputs_desktop_and_p4_contracts, test_urdf_limit_change_updates_every_control_contract, test_manifest_rejects_duplicate_motor_binding。 [704](#ref-0704)
- `tests/tools/test_generate_upper_protocol.py` — 87 行 · source · test_generate_upper_protocol：在 `tests/tools` 中定义或处理 _load_generated_python, test_schema_generates_matching_cpp_and_python_contracts, test_every_packet_and_command_value_is_unique, test_duplicate_command_value_is_rejected。 [705](#ref-0705)
- `tests/tools/test_p4_host_control_lease_policy.py` — 82 行 · source · test_p4_host_control_lease_policy：在 `tests/tools` 中定义或处理 test_host_control_lease_policy_covers_every_servo_motion_state。 [706](#ref-0706)
- `tests/tools/test_sync_structure_urdf_adapter.py` — 36 行 · source · test_sync_structure_urdf_adapter：在 `tests/tools` 中定义或处理 test_prefers_named_mmhand_urdf, test_accepts_only_urdf_as_explicit_fallback, test_rejects_ambiguous_urdf_fallback。 [707](#ref-0707)
- `tools/build_protocol_docx.py` — 329 行 · source · build_protocol_docx：在 `tools` 中定义或处理 set_run_font, set_paragraph_format, set_cell_shading, set_cell_margins。 [708](#ref-0708)
- `tools/capture_dist_scan.ps1` — 53 行 · source · capture_dist_scan：位于 `tools`，具体职责需沿源码核对。 [709](#ref-0709)
- `tools/capture_manual_cs_scan.ps1` — 59 行 · source · capture_manual_cs_scan：位于 `tools`，具体职责需沿源码核对。 [710](#ref-0710)
- `tools/capture_tactile_map.ps1` — 69 行 · source · capture_tactile_map：位于 `tools`，具体职责需沿源码核对。 [711](#ref-0711)
- `tools/check_desktop_architecture.py` — 189 行 · source · check_desktop_architecture：在 `tools` 中定义或处理 DependencyRule, iter_target_files, _module_name_for, _resolve_import。 [712](#ref-0712)
- `tools/clone_structure_repo.ps1` — 81 行 · source · clone_structure_repo：位于 `tools`，具体职责需沿源码核对。 [713](#ref-0713)
- `tools/create_desktop_shortcut.ps1` — 27 行 · source · create_desktop_shortcut：位于 `tools`，具体职责需沿源码核对。 [714](#ref-0714)
- `tools/desktop_architecture/apply_desktop_headers.py` — 274 行 · source · apply_desktop_headers：在 `tools/desktop_architecture` 中定义或处理 iter_target_files, _profile, _replace_module_docstring, main。 [715](#ref-0715)
- `tools/generate_handover_index.py` — 304 行 · source · generate_handover_index：在 `tools` 中定义或处理 git, inventory, category, inspect。 [716](#ref-0716)
- `tools/generate_p4_safety_catalog.py` — 462 行 · source · generate_p4_safety_catalog：在 `tools` 中定义或处理 Probe, RuleBinding, binding, _unquote。 [717](#ref-0717)
- `tools/generate_p4_urdf_limits.py` — 16 行 · source · generate_p4_urdf_limits：位于 `tools`，具体职责需沿源码核对。 [718](#ref-0718)
- `tools/generate_robot_model.py` — 380 行 · source · generate_robot_model：在 `tools` 中定义或处理 ModelGenerationError, _json_bytes, _crc32, _hex32。 [719](#ref-0719)
- `tools/generate_upper_protocol.py` — 270 行 · source · generate_upper_protocol：在 `tools` 中定义或处理 _load_schema, _render_cpp, _render_python, _write_atomic。 [720](#ref-0720)
- `tools/install_git_hooks.ps1` — 24 行 · source · install_git_hooks：位于 `tools`，具体职责需沿源码核对。 [721](#ref-0721)
- `tools/link_urdf.ps1` — 52 行 · source · link_urdf：位于 `tools`，具体职责需沿源码核对。 [722](#ref-0722)
- `tools/p4_architecture/check_p4_architecture.ps1` — 421 行 · source · check_p4_architecture：位于 `tools/p4_architecture`，具体职责需沿源码核对。 [723](#ref-0723)
- `tools/p4_architecture/check_safety_rule_closure.py` — 203 行 · source · check_safety_rule_closure：在 `tools/p4_architecture` 中定义或处理 read, probe_ok, generated_entry_ok, audit。 [724](#ref-0724)
- `tools/p4_debug/build_debug.ps1` — 43 行 · source · build_debug：位于 `tools/p4_debug`，具体职责需沿源码核对。 [725](#ref-0725)
- `tools/p4_debug/p4_debug_env.ps1` — 60 行 · source · p4_debug_env：位于 `tools/p4_debug`，具体职责需沿源码核对。 [726](#ref-0726)
- `tools/p4_debug/start_openocd.ps1` — 19 行 · source · start_openocd：位于 `tools/p4_debug`，具体职责需沿源码核对。 [727](#ref-0727)
- `tools/p4_debug/upload_debug.ps1` — 27 行 · source · upload_debug：位于 `tools/p4_debug`，具体职责需沿源码核对。 [728](#ref-0728)
- `tools/p4_debug/verify_debug_env.ps1` — 45 行 · source · verify_debug_env：位于 `tools/p4_debug`，具体职责需沿源码核对。 [729](#ref-0729)
- `tools/sync_structure_urdf_adapter.py` — 183 行 · source · sync_structure_urdf_adapter：在 `tools` 中定义或处理 _load_joint_aliases, _atomic_write_text, _repo_commit, _resolve_source_urdf。 [730](#ref-0730)
- `tools/update_structure_urdf.ps1` — 77 行 · source · update_structure_urdf：位于 `tools`，具体职责需沿源码核对。 [731](#ref-0731)
- `tools/validate_urdf.py` — 121 行 · source · validate_urdf：在 `tools` 中定义或处理 _required_joint_names, _resolve_mesh_path, validate, main。 [732](#ref-0732)

## 八、代码引用

<a id="ref-0001"></a>

- **[1]** `mmhand:.githooks/pre-commit` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/.githooks/pre-commit#L1-L1) · [local](../../.githooks/pre-commit)
<a id="ref-0002"></a>

- **[2]** `mmhand:.gitignore` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/.gitignore#L1-L1) · [local](../../.gitignore)
<a id="ref-0007"></a>

- **[7]** `mmhand:ESP32-P4/.editorconfig` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/.editorconfig#L1-L1) · [local](../../ESP32-P4/.editorconfig)
<a id="ref-0008"></a>

- **[8]** `mmhand:ESP32-P4/.githooks/pre-commit` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/.githooks/pre-commit#L1-L1) · [local](../../ESP32-P4/.githooks/pre-commit)
<a id="ref-0009"></a>

- **[9]** `mmhand:ESP32-P4/.gitignore` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/.gitignore#L1-L1) · [local](../../ESP32-P4/.gitignore)
<a id="ref-0010"></a>

- **[10]** `mmhand:ESP32-P4/.gitmodules` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/.gitmodules#L1-L1) · [local](../../ESP32-P4/.gitmodules)
<a id="ref-0011"></a>

- **[11]** `mmhand:ESP32-P4/.idea/.gitignore` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/.idea/.gitignore#L1-L1) · [local](../../ESP32-P4/.idea/.gitignore)
<a id="ref-0012"></a>

- **[12]** `mmhand:ESP32-P4/.idea/HandControl.iml` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/.idea/HandControl.iml#L1-L1) · [local](../../ESP32-P4/.idea/HandControl.iml)
<a id="ref-0013"></a>

- **[13]** `mmhand:ESP32-P4/.idea/inspectionProfiles/profiles_settings.xml` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/.idea/inspectionProfiles/profiles_settings.xml#L1-L1) · [local](../../ESP32-P4/.idea/inspectionProfiles/profiles_settings.xml)
<a id="ref-0014"></a>

- **[14]** `mmhand:ESP32-P4/.idea/misc.xml` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/.idea/misc.xml#L1-L1) · [local](../../ESP32-P4/.idea/misc.xml)
<a id="ref-0015"></a>

- **[15]** `mmhand:ESP32-P4/.idea/modules.xml` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/.idea/modules.xml#L1-L1) · [local](../../ESP32-P4/.idea/modules.xml)
<a id="ref-0016"></a>

- **[16]** `mmhand:ESP32-P4/.idea/vcs.xml` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/.idea/vcs.xml#L1-L1) · [local](../../ESP32-P4/.idea/vcs.xml)
<a id="ref-0019"></a>

- **[19]** `mmhand:ESP32-P4/ServoBoardMain/ESP32-P4_Arduino断点调试_README.md` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/ESP32-P4_Arduino%E6%96%AD%E7%82%B9%E8%B0%83%E8%AF%95_README.md#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/ESP32-P4_Arduino%E6%96%AD%E7%82%B9%E8%B0%83%E8%AF%95_README.md)
<a id="ref-0020"></a>

- **[20]** `mmhand:ESP32-P4/ServoBoardMain/ServoBoardMain.ino` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/ServoBoardMain.ino#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/ServoBoardMain.ino)
<a id="ref-0021"></a>

- **[21]** `mmhand:ESP32-P4/ServoBoardMain/docs/P4_嵌入式软件架构与安全规则_20260807.md` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/docs/P4_%E5%B5%8C%E5%85%A5%E5%BC%8F%E8%BD%AF%E4%BB%B6%E6%9E%B6%E6%9E%84%E4%B8%8E%E5%AE%89%E5%85%A8%E8%A7%84%E5%88%99_20260807.md#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/docs/P4_%E5%B5%8C%E5%85%A5%E5%BC%8F%E8%BD%AF%E4%BB%B6%E6%9E%B6%E6%9E%84%E4%B8%8E%E5%AE%89%E5%85%A8%E8%A7%84%E5%88%99_20260807.md)
<a id="ref-0022"></a>

- **[22]** `mmhand:ESP32-P4/ServoBoardMain/docs/P4_架构迁移实施记录_20260810.md` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/docs/P4_%E6%9E%B6%E6%9E%84%E8%BF%81%E7%A7%BB%E5%AE%9E%E6%96%BD%E8%AE%B0%E5%BD%95_20260810.md#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/docs/P4_%E6%9E%B6%E6%9E%84%E8%BF%81%E7%A7%BB%E5%AE%9E%E6%96%BD%E8%AE%B0%E5%BD%95_20260810.md)
<a id="ref-0023"></a>

- **[23]** `mmhand:ESP32-P4/ServoBoardMain/readme.md` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/readme.md#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/readme.md)
<a id="ref-0024"></a>

- **[24]** `mmhand:ESP32-P4/ServoBoardMain/src/app/AppBootstrap.cpp` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/app/AppBootstrap.cpp#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/app/AppBootstrap.cpp)
<a id="ref-0025"></a>

- **[25]** `mmhand:ESP32-P4/ServoBoardMain/src/app/AppBootstrap.h` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/app/AppBootstrap.h#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/app/AppBootstrap.h)
<a id="ref-0026"></a>

- **[26]** `mmhand:ESP32-P4/ServoBoardMain/src/app/SystemAssembly.cpp` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/app/SystemAssembly.cpp#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/app/SystemAssembly.cpp)
  - 相关源码行：[L43–L43 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/app/SystemAssembly.cpp#L43-L43) · [local](../../ESP32-P4/ServoBoardMain/src/app/SystemAssembly.cpp), [L68–L68 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/app/SystemAssembly.cpp#L68-L68) · [local](../../ESP32-P4/ServoBoardMain/src/app/SystemAssembly.cpp), [L79–L79 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/app/SystemAssembly.cpp#L79-L79) · [local](../../ESP32-P4/ServoBoardMain/src/app/SystemAssembly.cpp), [L88–L88 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/app/SystemAssembly.cpp#L88-L88) · [local](../../ESP32-P4/ServoBoardMain/src/app/SystemAssembly.cpp), [L90–L90 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/app/SystemAssembly.cpp#L90-L90) · [local](../../ESP32-P4/ServoBoardMain/src/app/SystemAssembly.cpp), [L144–L144 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/app/SystemAssembly.cpp#L144-L144) · [local](../../ESP32-P4/ServoBoardMain/src/app/SystemAssembly.cpp), [L145–L145 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/app/SystemAssembly.cpp#L145-L145) · [local](../../ESP32-P4/ServoBoardMain/src/app/SystemAssembly.cpp), [L146–L146 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/app/SystemAssembly.cpp#L146-L146) · [local](../../ESP32-P4/ServoBoardMain/src/app/SystemAssembly.cpp), [L154–L154 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/app/SystemAssembly.cpp#L154-L154) · [local](../../ESP32-P4/ServoBoardMain/src/app/SystemAssembly.cpp), [L163–L163 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/app/SystemAssembly.cpp#L163-L163) · [local](../../ESP32-P4/ServoBoardMain/src/app/SystemAssembly.cpp), [L172–L172 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/app/SystemAssembly.cpp#L172-L172) · [local](../../ESP32-P4/ServoBoardMain/src/app/SystemAssembly.cpp), [L181–L181 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/app/SystemAssembly.cpp#L181-L181) · [local](../../ESP32-P4/ServoBoardMain/src/app/SystemAssembly.cpp), [L190–L190 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/app/SystemAssembly.cpp#L190-L190) · [local](../../ESP32-P4/ServoBoardMain/src/app/SystemAssembly.cpp), [L206–L206 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/app/SystemAssembly.cpp#L206-L206) · [local](../../ESP32-P4/ServoBoardMain/src/app/SystemAssembly.cpp)
<a id="ref-0027"></a>

- **[27]** `mmhand:ESP32-P4/ServoBoardMain/src/app/SystemAssembly.h` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/app/SystemAssembly.h#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/app/SystemAssembly.h)
<a id="ref-0028"></a>

- **[28]** `mmhand:ESP32-P4/ServoBoardMain/src/config/BoardConfig.h` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/config/BoardConfig.h#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/config/BoardConfig.h)
<a id="ref-0029"></a>

- **[29]** `mmhand:ESP32-P4/ServoBoardMain/src/config/CalibrationConfig.h` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/config/CalibrationConfig.h#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/config/CalibrationConfig.h)
<a id="ref-0030"></a>

- **[30]** `mmhand:ESP32-P4/ServoBoardMain/src/config/Config.h` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/config/Config.h#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/config/Config.h)
<a id="ref-0031"></a>

- **[31]** `mmhand:ESP32-P4/ServoBoardMain/src/config/ControlConfig.h` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/config/ControlConfig.h#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/config/ControlConfig.h)
<a id="ref-0032"></a>

- **[32]** `mmhand:ESP32-P4/ServoBoardMain/src/config/DiagnosticsConfig.h` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/config/DiagnosticsConfig.h#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/config/DiagnosticsConfig.h)
<a id="ref-0033"></a>

- **[33]** `mmhand:ESP32-P4/ServoBoardMain/src/config/DimensionsConfig.h` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/config/DimensionsConfig.h#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/config/DimensionsConfig.h)
<a id="ref-0034"></a>

- **[34]** `mmhand:ESP32-P4/ServoBoardMain/src/config/SafetyConfig.h` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/config/SafetyConfig.h#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/config/SafetyConfig.h)
<a id="ref-0035"></a>

- **[35]** `mmhand:ESP32-P4/ServoBoardMain/src/config/ServoConfig.h` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/config/ServoConfig.h#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/config/ServoConfig.h)
<a id="ref-0036"></a>

- **[36]** `mmhand:ESP32-P4/ServoBoardMain/src/config/TaskConfig.h` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/config/TaskConfig.h#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/config/TaskConfig.h)
  - 相关源码行：[L11–L11 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/config/TaskConfig.h#L11-L11) · [local](../../ESP32-P4/ServoBoardMain/src/config/TaskConfig.h), [L12–L12 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/config/TaskConfig.h#L12-L12) · [local](../../ESP32-P4/ServoBoardMain/src/config/TaskConfig.h), [L14–L14 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/config/TaskConfig.h#L14-L14) · [local](../../ESP32-P4/ServoBoardMain/src/config/TaskConfig.h), [L15–L15 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/config/TaskConfig.h#L15-L15) · [local](../../ESP32-P4/ServoBoardMain/src/config/TaskConfig.h), [L17–L17 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/config/TaskConfig.h#L17-L17) · [local](../../ESP32-P4/ServoBoardMain/src/config/TaskConfig.h), [L21–L21 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/config/TaskConfig.h#L21-L21) · [local](../../ESP32-P4/ServoBoardMain/src/config/TaskConfig.h), [L23–L23 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/config/TaskConfig.h#L23-L23) · [local](../../ESP32-P4/ServoBoardMain/src/config/TaskConfig.h), [L24–L24 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/config/TaskConfig.h#L24-L24) · [local](../../ESP32-P4/ServoBoardMain/src/config/TaskConfig.h), [L25–L25 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/config/TaskConfig.h#L25-L25) · [local](../../ESP32-P4/ServoBoardMain/src/config/TaskConfig.h), [L27–L27 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/config/TaskConfig.h#L27-L27) · [local](../../ESP32-P4/ServoBoardMain/src/config/TaskConfig.h)
<a id="ref-0037"></a>

- **[37]** `mmhand:ESP32-P4/ServoBoardMain/src/config/protocol/S3SensorCanProtocol.h` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/config/protocol/S3SensorCanProtocol.h#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/config/protocol/S3SensorCanProtocol.h)
  - 相关源码行：[L1–L1 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/config/protocol/S3SensorCanProtocol.h#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/config/protocol/S3SensorCanProtocol.h), [L16–L16 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/config/protocol/S3SensorCanProtocol.h#L16-L16) · [local](../../ESP32-P4/ServoBoardMain/src/config/protocol/S3SensorCanProtocol.h), [L17–L17 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/config/protocol/S3SensorCanProtocol.h#L17-L17) · [local](../../ESP32-P4/ServoBoardMain/src/config/protocol/S3SensorCanProtocol.h), [L18–L18 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/config/protocol/S3SensorCanProtocol.h#L18-L18) · [local](../../ESP32-P4/ServoBoardMain/src/config/protocol/S3SensorCanProtocol.h), [L25–L25 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/config/protocol/S3SensorCanProtocol.h#L25-L25) · [local](../../ESP32-P4/ServoBoardMain/src/config/protocol/S3SensorCanProtocol.h), [L26–L26 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/config/protocol/S3SensorCanProtocol.h#L26-L26) · [local](../../ESP32-P4/ServoBoardMain/src/config/protocol/S3SensorCanProtocol.h), [L34–L34 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/config/protocol/S3SensorCanProtocol.h#L34-L34) · [local](../../ESP32-P4/ServoBoardMain/src/config/protocol/S3SensorCanProtocol.h), [L35–L35 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/config/protocol/S3SensorCanProtocol.h#L35-L35) · [local](../../ESP32-P4/ServoBoardMain/src/config/protocol/S3SensorCanProtocol.h), [L46–L46 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/config/protocol/S3SensorCanProtocol.h#L46-L46) · [local](../../ESP32-P4/ServoBoardMain/src/config/protocol/S3SensorCanProtocol.h), [L49–L49 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/config/protocol/S3SensorCanProtocol.h#L49-L49) · [local](../../ESP32-P4/ServoBoardMain/src/config/protocol/S3SensorCanProtocol.h), [L54–L54 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/config/protocol/S3SensorCanProtocol.h#L54-L54) · [local](../../ESP32-P4/ServoBoardMain/src/config/protocol/S3SensorCanProtocol.h)
<a id="ref-0038"></a>

- **[38]** `mmhand:ESP32-P4/ServoBoardMain/src/config/safety/SafetyRuleCatalog.def` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/config/safety/SafetyRuleCatalog.def#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/config/safety/SafetyRuleCatalog.def)
  - 相关源码行：[L4–L4 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/config/safety/SafetyRuleCatalog.def#L4-L4) · [local](../../ESP32-P4/ServoBoardMain/src/config/safety/SafetyRuleCatalog.def), [L5–L5 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/config/safety/SafetyRuleCatalog.def#L5-L5) · [local](../../ESP32-P4/ServoBoardMain/src/config/safety/SafetyRuleCatalog.def), [L6–L6 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/config/safety/SafetyRuleCatalog.def#L6-L6) · [local](../../ESP32-P4/ServoBoardMain/src/config/safety/SafetyRuleCatalog.def), [L7–L7 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/config/safety/SafetyRuleCatalog.def#L7-L7) · [local](../../ESP32-P4/ServoBoardMain/src/config/safety/SafetyRuleCatalog.def), [L8–L8 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/config/safety/SafetyRuleCatalog.def#L8-L8) · [local](../../ESP32-P4/ServoBoardMain/src/config/safety/SafetyRuleCatalog.def), [L9–L9 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/config/safety/SafetyRuleCatalog.def#L9-L9) · [local](../../ESP32-P4/ServoBoardMain/src/config/safety/SafetyRuleCatalog.def), [L10–L10 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/config/safety/SafetyRuleCatalog.def#L10-L10) · [local](../../ESP32-P4/ServoBoardMain/src/config/safety/SafetyRuleCatalog.def), [L11–L11 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/config/safety/SafetyRuleCatalog.def#L11-L11) · [local](../../ESP32-P4/ServoBoardMain/src/config/safety/SafetyRuleCatalog.def), [L12–L12 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/config/safety/SafetyRuleCatalog.def#L12-L12) · [local](../../ESP32-P4/ServoBoardMain/src/config/safety/SafetyRuleCatalog.def), [L13–L13 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/config/safety/SafetyRuleCatalog.def#L13-L13) · [local](../../ESP32-P4/ServoBoardMain/src/config/safety/SafetyRuleCatalog.def), [L14–L14 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/config/safety/SafetyRuleCatalog.def#L14-L14) · [local](../../ESP32-P4/ServoBoardMain/src/config/safety/SafetyRuleCatalog.def), [L15–L15 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/config/safety/SafetyRuleCatalog.def#L15-L15) · [local](../../ESP32-P4/ServoBoardMain/src/config/safety/SafetyRuleCatalog.def), [L16–L16 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/config/safety/SafetyRuleCatalog.def#L16-L16) · [local](../../ESP32-P4/ServoBoardMain/src/config/safety/SafetyRuleCatalog.def), [L18–L18 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/config/safety/SafetyRuleCatalog.def#L18-L18) · [local](../../ESP32-P4/ServoBoardMain/src/config/safety/SafetyRuleCatalog.def), [L19–L19 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/config/safety/SafetyRuleCatalog.def#L19-L19) · [local](../../ESP32-P4/ServoBoardMain/src/config/safety/SafetyRuleCatalog.def), [L20–L20 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/config/safety/SafetyRuleCatalog.def#L20-L20) · [local](../../ESP32-P4/ServoBoardMain/src/config/safety/SafetyRuleCatalog.def)
<a id="ref-0039"></a>

- **[39]** `mmhand:ESP32-P4/ServoBoardMain/src/config/safety/ServoPositionRange.h` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/config/safety/ServoPositionRange.h#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/config/safety/ServoPositionRange.h)
<a id="ref-0040"></a>

- **[40]** `mmhand:ESP32-P4/ServoBoardMain/src/config/topology/HardwareMap.cpp` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/config/topology/HardwareMap.cpp#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/config/topology/HardwareMap.cpp)
<a id="ref-0041"></a>

- **[41]** `mmhand:ESP32-P4/ServoBoardMain/src/config/topology/HardwareMap.h` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/config/topology/HardwareMap.h#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/config/topology/HardwareMap.h)
<a id="ref-0042"></a>

- **[42]** `mmhand:ESP32-P4/ServoBoardMain/src/data/ActuatorTypes.h` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/data/ActuatorTypes.h#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/data/ActuatorTypes.h)
<a id="ref-0043"></a>

- **[43]** `mmhand:ESP32-P4/ServoBoardMain/src/data/CalibrationTypes.h` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/data/CalibrationTypes.h#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/data/CalibrationTypes.h)
<a id="ref-0044"></a>

- **[44]** `mmhand:ESP32-P4/ServoBoardMain/src/data/ControlTraceTypes.h` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/data/ControlTraceTypes.h#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/data/ControlTraceTypes.h)
<a id="ref-0045"></a>

- **[45]** `mmhand:ESP32-P4/ServoBoardMain/src/data/HardwareTypes.h` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/data/HardwareTypes.h#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/data/HardwareTypes.h)
<a id="ref-0046"></a>

- **[46]** `mmhand:ESP32-P4/ServoBoardMain/src/data/JointTypes.h` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/data/JointTypes.h#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/data/JointTypes.h)
<a id="ref-0047"></a>

- **[47]** `mmhand:ESP32-P4/ServoBoardMain/src/data/RecoveryTypes.h` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/data/RecoveryTypes.h#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/data/RecoveryTypes.h)
<a id="ref-0048"></a>

- **[48]** `mmhand:ESP32-P4/ServoBoardMain/src/data/RopeInitTypes.h` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/data/RopeInitTypes.h#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/data/RopeInitTypes.h)
<a id="ref-0049"></a>

- **[49]** `mmhand:ESP32-P4/ServoBoardMain/src/data/SafetyRuleTypes.h` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/data/SafetyRuleTypes.h#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/data/SafetyRuleTypes.h)
<a id="ref-0050"></a>

- **[50]** `mmhand:ESP32-P4/ServoBoardMain/src/data/SafetyTypes.h` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/data/SafetyTypes.h#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/data/SafetyTypes.h)
<a id="ref-0051"></a>

- **[51]** `mmhand:ESP32-P4/ServoBoardMain/src/data/ServiceSnapshots.h` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/data/ServiceSnapshots.h#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/data/ServiceSnapshots.h)
<a id="ref-0052"></a>

- **[52]** `mmhand:ESP32-P4/ServoBoardMain/src/data/ServoRouteTypes.h` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/data/ServoRouteTypes.h#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/data/ServoRouteTypes.h)
<a id="ref-0053"></a>

- **[53]** `mmhand:ESP32-P4/ServoBoardMain/src/data/ServoTypes.h` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/data/ServoTypes.h#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/data/ServoTypes.h)
<a id="ref-0054"></a>

- **[54]** `mmhand:ESP32-P4/ServoBoardMain/src/data/SystemTypes.h` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/data/SystemTypes.h#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/data/SystemTypes.h)
  - 相关源码行：[L39–L39 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/data/SystemTypes.h#L39-L39) · [local](../../ESP32-P4/ServoBoardMain/src/data/SystemTypes.h), [L40–L40 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/data/SystemTypes.h#L40-L40) · [local](../../ESP32-P4/ServoBoardMain/src/data/SystemTypes.h), [L41–L41 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/data/SystemTypes.h#L41-L41) · [local](../../ESP32-P4/ServoBoardMain/src/data/SystemTypes.h), [L42–L42 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/data/SystemTypes.h#L42-L42) · [local](../../ESP32-P4/ServoBoardMain/src/data/SystemTypes.h), [L43–L43 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/data/SystemTypes.h#L43-L43) · [local](../../ESP32-P4/ServoBoardMain/src/data/SystemTypes.h), [L44–L44 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/data/SystemTypes.h#L44-L44) · [local](../../ESP32-P4/ServoBoardMain/src/data/SystemTypes.h), [L45–L45 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/data/SystemTypes.h#L45-L45) · [local](../../ESP32-P4/ServoBoardMain/src/data/SystemTypes.h), [L46–L46 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/data/SystemTypes.h#L46-L46) · [local](../../ESP32-P4/ServoBoardMain/src/data/SystemTypes.h), [L47–L47 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/data/SystemTypes.h#L47-L47) · [local](../../ESP32-P4/ServoBoardMain/src/data/SystemTypes.h)
<a id="ref-0055"></a>

- **[55]** `mmhand:ESP32-P4/ServoBoardMain/src/data/TelemetryTypes.h` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/data/TelemetryTypes.h#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/data/TelemetryTypes.h)
<a id="ref-0056"></a>

- **[56]** `mmhand:ESP32-P4/ServoBoardMain/src/generated/ProductFingerprint.generated.h` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/generated/ProductFingerprint.generated.h#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/generated/ProductFingerprint.generated.h)
<a id="ref-0057"></a>

- **[57]** `mmhand:ESP32-P4/ServoBoardMain/src/generated/SafetyRuleCatalog.generated.h` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/generated/SafetyRuleCatalog.generated.h#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/generated/SafetyRuleCatalog.generated.h)
<a id="ref-0058"></a>

- **[58]** `mmhand:ESP32-P4/ServoBoardMain/src/generated/SafetyRuleIndex.generated.json` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/generated/SafetyRuleIndex.generated.json#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/generated/SafetyRuleIndex.generated.json)
<a id="ref-0059"></a>

- **[59]** `mmhand:ESP32-P4/ServoBoardMain/src/generated/UpperProtocol.generated.h` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/generated/UpperProtocol.generated.h#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/generated/UpperProtocol.generated.h)
<a id="ref-0060"></a>

- **[60]** `mmhand:ESP32-P4/ServoBoardMain/src/generated/UrdfJointLimits.generated.h` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/generated/UrdfJointLimits.generated.h#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/generated/UrdfJointLimits.generated.h)
<a id="ref-0061"></a>

- **[61]** `mmhand:ESP32-P4/ServoBoardMain/src/hal/can/TwaiCanDriver.cpp` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/hal/can/TwaiCanDriver.cpp#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/hal/can/TwaiCanDriver.cpp)
<a id="ref-0062"></a>

- **[62]** `mmhand:ESP32-P4/ServoBoardMain/src/hal/can/TwaiCanDriver.h` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/hal/can/TwaiCanDriver.h#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/hal/can/TwaiCanDriver.h)
<a id="ref-0063"></a>

- **[63]** `mmhand:ESP32-P4/ServoBoardMain/src/hal/serial/HostSerialPort.cpp` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/hal/serial/HostSerialPort.cpp#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/hal/serial/HostSerialPort.cpp)
<a id="ref-0064"></a>

- **[64]** `mmhand:ESP32-P4/ServoBoardMain/src/hal/serial/HostSerialPort.h` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/hal/serial/HostSerialPort.h#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/hal/serial/HostSerialPort.h)
<a id="ref-0065"></a>

- **[65]** `mmhand:ESP32-P4/ServoBoardMain/src/hal/servo/FeetechServoDriver.cpp` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/hal/servo/FeetechServoDriver.cpp#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/hal/servo/FeetechServoDriver.cpp)
<a id="ref-0066"></a>

- **[66]** `mmhand:ESP32-P4/ServoBoardMain/src/hal/servo/FeetechServoDriver.h` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/hal/servo/FeetechServoDriver.h#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/hal/servo/FeetechServoDriver.h)
<a id="ref-0067"></a>

- **[67]** `mmhand:ESP32-P4/ServoBoardMain/src/hal/servo/ServoBusRegistry.cpp` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/hal/servo/ServoBusRegistry.cpp#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/hal/servo/ServoBusRegistry.cpp)
<a id="ref-0068"></a>

- **[68]** `mmhand:ESP32-P4/ServoBoardMain/src/hal/servo/ServoBusRegistry.h` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/hal/servo/ServoBusRegistry.h#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/hal/servo/ServoBusRegistry.h)
<a id="ref-0069"></a>

- **[69]** `mmhand:ESP32-P4/ServoBoardMain/src/hal/servo/ServoValue.h` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/hal/servo/ServoValue.h#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/hal/servo/ServoValue.h)
<a id="ref-0070"></a>

- **[70]** `mmhand:ESP32-P4/ServoBoardMain/src/modules/actuator/ActuatorCommandArbiter.cpp` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/modules/actuator/ActuatorCommandArbiter.cpp#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/modules/actuator/ActuatorCommandArbiter.cpp)
<a id="ref-0071"></a>

- **[71]** `mmhand:ESP32-P4/ServoBoardMain/src/modules/actuator/ActuatorCommandArbiter.h` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/modules/actuator/ActuatorCommandArbiter.h#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/modules/actuator/ActuatorCommandArbiter.h)
<a id="ref-0072"></a>

- **[72]** `mmhand:ESP32-P4/ServoBoardMain/src/modules/actuator/ActuatorCommandGateway.cpp` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/modules/actuator/ActuatorCommandGateway.cpp#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/modules/actuator/ActuatorCommandGateway.cpp)
<a id="ref-0073"></a>

- **[73]** `mmhand:ESP32-P4/ServoBoardMain/src/modules/actuator/ActuatorCommandGateway.h` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/modules/actuator/ActuatorCommandGateway.h#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/modules/actuator/ActuatorCommandGateway.h)
<a id="ref-0074"></a>

- **[74]** `mmhand:ESP32-P4/ServoBoardMain/src/modules/calibration/CalibrationMath.cpp` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/modules/calibration/CalibrationMath.cpp#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/modules/calibration/CalibrationMath.cpp)
<a id="ref-0075"></a>

- **[75]** `mmhand:ESP32-P4/ServoBoardMain/src/modules/calibration/CalibrationMath.h` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/modules/calibration/CalibrationMath.h#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/modules/calibration/CalibrationMath.h)
<a id="ref-0076"></a>

- **[76]** `mmhand:ESP32-P4/ServoBoardMain/src/modules/calibration/CalibrationRuntimeStore.cpp` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/modules/calibration/CalibrationRuntimeStore.cpp#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/modules/calibration/CalibrationRuntimeStore.cpp)
  - 相关源码行：[L156–L156 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/modules/calibration/CalibrationRuntimeStore.cpp#L156-L156) · [local](../../ESP32-P4/ServoBoardMain/src/modules/calibration/CalibrationRuntimeStore.cpp), [L192–L192 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/modules/calibration/CalibrationRuntimeStore.cpp#L192-L192) · [local](../../ESP32-P4/ServoBoardMain/src/modules/calibration/CalibrationRuntimeStore.cpp), [L238–L238 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/modules/calibration/CalibrationRuntimeStore.cpp#L238-L238) · [local](../../ESP32-P4/ServoBoardMain/src/modules/calibration/CalibrationRuntimeStore.cpp)
<a id="ref-0077"></a>

- **[77]** `mmhand:ESP32-P4/ServoBoardMain/src/modules/calibration/CalibrationRuntimeStore.h` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/modules/calibration/CalibrationRuntimeStore.h#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/modules/calibration/CalibrationRuntimeStore.h)
<a id="ref-0078"></a>

- **[78]** `mmhand:ESP32-P4/ServoBoardMain/src/modules/calibration/ManualEncoderReferenceStore.cpp` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/modules/calibration/ManualEncoderReferenceStore.cpp#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/modules/calibration/ManualEncoderReferenceStore.cpp)
<a id="ref-0079"></a>

- **[79]** `mmhand:ESP32-P4/ServoBoardMain/src/modules/calibration/ManualEncoderReferenceStore.h` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/modules/calibration/ManualEncoderReferenceStore.h#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/modules/calibration/ManualEncoderReferenceStore.h)
<a id="ref-0080"></a>

- **[80]** `mmhand:ESP32-P4/ServoBoardMain/src/modules/calibration/TendonCalibrationLut.cpp` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/modules/calibration/TendonCalibrationLut.cpp#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/modules/calibration/TendonCalibrationLut.cpp)
<a id="ref-0081"></a>

- **[81]** `mmhand:ESP32-P4/ServoBoardMain/src/modules/calibration/TendonCalibrationLut.h` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/modules/calibration/TendonCalibrationLut.h#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/modules/calibration/TendonCalibrationLut.h)
<a id="ref-0082"></a>

- **[82]** `mmhand:ESP32-P4/ServoBoardMain/src/modules/integrity/Crc16Ccitt.cpp` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/modules/integrity/Crc16Ccitt.cpp#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/modules/integrity/Crc16Ccitt.cpp)
<a id="ref-0083"></a>

- **[83]** `mmhand:ESP32-P4/ServoBoardMain/src/modules/integrity/Crc16Ccitt.h` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/modules/integrity/Crc16Ccitt.h#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/modules/integrity/Crc16Ccitt.h)
<a id="ref-0084"></a>

- **[84]** `mmhand:ESP32-P4/ServoBoardMain/src/modules/integrity/Crc32Ieee.cpp` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/modules/integrity/Crc32Ieee.cpp#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/modules/integrity/Crc32Ieee.cpp)
<a id="ref-0085"></a>

- **[85]** `mmhand:ESP32-P4/ServoBoardMain/src/modules/integrity/Crc32Ieee.h` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/modules/integrity/Crc32Ieee.h#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/modules/integrity/Crc32Ieee.h)
<a id="ref-0086"></a>

- **[86]** `mmhand:ESP32-P4/ServoBoardMain/src/modules/protocol/HostProtocol.h` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/modules/protocol/HostProtocol.h#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/modules/protocol/HostProtocol.h)
<a id="ref-0087"></a>

- **[87]** `mmhand:ESP32-P4/ServoBoardMain/src/modules/runtime/ServiceCyclePolicy.cpp` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/modules/runtime/ServiceCyclePolicy.cpp#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/modules/runtime/ServiceCyclePolicy.cpp)
<a id="ref-0088"></a>

- **[88]** `mmhand:ESP32-P4/ServoBoardMain/src/modules/runtime/ServiceCyclePolicy.h` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/modules/runtime/ServiceCyclePolicy.h#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/modules/runtime/ServiceCyclePolicy.h)
<a id="ref-0091"></a>

- **[91]** `mmhand:ESP32-P4/ServoBoardMain/src/platform/rtos/AtomicValue.h` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/platform/rtos/AtomicValue.h#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/platform/rtos/AtomicValue.h)
<a id="ref-0092"></a>

- **[92]** `mmhand:ESP32-P4/ServoBoardMain/src/platform/rtos/FreeRtosKernel.h` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/platform/rtos/FreeRtosKernel.h#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/platform/rtos/FreeRtosKernel.h)
<a id="ref-0093"></a>

- **[93]** `mmhand:ESP32-P4/ServoBoardMain/src/platform/rtos/MessageChannel.h` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/platform/rtos/MessageChannel.h#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/platform/rtos/MessageChannel.h)
<a id="ref-0094"></a>

- **[94]** `mmhand:ESP32-P4/ServoBoardMain/src/platform/rtos/PeriodicDeadline.h` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/platform/rtos/PeriodicDeadline.h#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/platform/rtos/PeriodicDeadline.h)
<a id="ref-0095"></a>

- **[95]** `mmhand:ESP32-P4/ServoBoardMain/src/platform/rtos/SharedSafetyState.cpp` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/platform/rtos/SharedSafetyState.cpp#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/platform/rtos/SharedSafetyState.cpp)
<a id="ref-0096"></a>

- **[96]** `mmhand:ESP32-P4/ServoBoardMain/src/platform/rtos/SharedSafetyState.h` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/platform/rtos/SharedSafetyState.h#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/platform/rtos/SharedSafetyState.h)
<a id="ref-0097"></a>

- **[97]** `mmhand:ESP32-P4/ServoBoardMain/src/platform/rtos/TaskFactory.cpp` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/platform/rtos/TaskFactory.cpp#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/platform/rtos/TaskFactory.cpp)
<a id="ref-0098"></a>

- **[98]** `mmhand:ESP32-P4/ServoBoardMain/src/platform/rtos/TaskFactory.h` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/platform/rtos/TaskFactory.h#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/platform/rtos/TaskFactory.h)
<a id="ref-0099"></a>

- **[99]** `mmhand:ESP32-P4/ServoBoardMain/src/platform/rtos/TaskSignal.h` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/platform/rtos/TaskSignal.h#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/platform/rtos/TaskSignal.h)
<a id="ref-0100"></a>

- **[100]** `mmhand:ESP32-P4/ServoBoardMain/src/platform/time/Clock.h` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/platform/time/Clock.h#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/platform/time/Clock.h)
<a id="ref-0101"></a>

- **[101]** `mmhand:ESP32-P4/ServoBoardMain/src/runtime/SystemRuntimeContext.h` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/runtime/SystemRuntimeContext.h#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/runtime/SystemRuntimeContext.h)
<a id="ref-0102"></a>

- **[102]** `mmhand:ESP32-P4/ServoBoardMain/src/runtime/adapters/ServiceContextAdapter.cpp` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/runtime/adapters/ServiceContextAdapter.cpp#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/runtime/adapters/ServiceContextAdapter.cpp)
<a id="ref-0103"></a>

- **[103]** `mmhand:ESP32-P4/ServoBoardMain/src/runtime/adapters/ServiceContextAdapter.h` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/runtime/adapters/ServiceContextAdapter.h#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/runtime/adapters/ServiceContextAdapter.h)
<a id="ref-0104"></a>

- **[104]** `mmhand:ESP32-P4/ServoBoardMain/src/runtime/channels/InterTaskChannels.h` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/runtime/channels/InterTaskChannels.h#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/runtime/channels/InterTaskChannels.h)
<a id="ref-0105"></a>

- **[105]** `mmhand:ESP32-P4/ServoBoardMain/src/runtime/metrics/RuntimeMonitorConfig.h` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/runtime/metrics/RuntimeMonitorConfig.h#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/runtime/metrics/RuntimeMonitorConfig.h)
<a id="ref-0106"></a>

- **[106]** `mmhand:ESP32-P4/ServoBoardMain/src/runtime/metrics/TaskRuntimeMetrics.h` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/runtime/metrics/TaskRuntimeMetrics.h#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/runtime/metrics/TaskRuntimeMetrics.h)
<a id="ref-0107"></a>

- **[107]** `mmhand:ESP32-P4/ServoBoardMain/src/runtime/metrics/TaskRuntimeWindow.cpp` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/runtime/metrics/TaskRuntimeWindow.cpp#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/runtime/metrics/TaskRuntimeWindow.cpp)
<a id="ref-0108"></a>

- **[108]** `mmhand:ESP32-P4/ServoBoardMain/src/runtime/metrics/TaskRuntimeWindow.h` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/runtime/metrics/TaskRuntimeWindow.h#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/runtime/metrics/TaskRuntimeWindow.h)
<a id="ref-0109"></a>

- **[109]** `mmhand:ESP32-P4/ServoBoardMain/src/runtime/ports/ServiceOutputPort.h` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/runtime/ports/ServiceOutputPort.h#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/runtime/ports/ServiceOutputPort.h)
<a id="ref-0110"></a>

- **[110]** `mmhand:ESP32-P4/ServoBoardMain/src/runtime/ports/SystemEventPort.h` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/runtime/ports/SystemEventPort.h#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/runtime/ports/SystemEventPort.h)
<a id="ref-0111"></a>

- **[111]** `mmhand:ESP32-P4/ServoBoardMain/src/runtime/state/CalibrationRuntimeState.h` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/runtime/state/CalibrationRuntimeState.h#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/runtime/state/CalibrationRuntimeState.h)
<a id="ref-0112"></a>

- **[112]** `mmhand:ESP32-P4/ServoBoardMain/src/runtime/state/CommandRuntimeState.h` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/runtime/state/CommandRuntimeState.h#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/runtime/state/CommandRuntimeState.h)
<a id="ref-0113"></a>

- **[113]** `mmhand:ESP32-P4/ServoBoardMain/src/runtime/state/ControlRuntimeState.h` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/runtime/state/ControlRuntimeState.h#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/runtime/state/ControlRuntimeState.h)
<a id="ref-0114"></a>

- **[114]** `mmhand:ESP32-P4/ServoBoardMain/src/runtime/state/ServoRuntimeState.h` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/runtime/state/ServoRuntimeState.h#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/runtime/state/ServoRuntimeState.h)
<a id="ref-0115"></a>

- **[115]** `mmhand:ESP32-P4/ServoBoardMain/src/safety/SafetyEventReporter.cpp` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/safety/SafetyEventReporter.cpp#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/safety/SafetyEventReporter.cpp)
  - 相关源码行：[L141–L141 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/safety/SafetyEventReporter.cpp#L141-L141) · [local](../../ESP32-P4/ServoBoardMain/src/safety/SafetyEventReporter.cpp)
<a id="ref-0116"></a>

- **[116]** `mmhand:ESP32-P4/ServoBoardMain/src/safety/SafetyEventReporter.h` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/safety/SafetyEventReporter.h#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/safety/SafetyEventReporter.h)
<a id="ref-0117"></a>

- **[117]** `mmhand:ESP32-P4/ServoBoardMain/src/safety/SafetyEventStore.cpp` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/safety/SafetyEventStore.cpp#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/safety/SafetyEventStore.cpp)
  - 相关源码行：[L19–L19 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/safety/SafetyEventStore.cpp#L19-L19) · [local](../../ESP32-P4/ServoBoardMain/src/safety/SafetyEventStore.cpp), [L43–L43 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/safety/SafetyEventStore.cpp#L43-L43) · [local](../../ESP32-P4/ServoBoardMain/src/safety/SafetyEventStore.cpp), [L51–L51 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/safety/SafetyEventStore.cpp#L51-L51) · [local](../../ESP32-P4/ServoBoardMain/src/safety/SafetyEventStore.cpp)
<a id="ref-0118"></a>

- **[118]** `mmhand:ESP32-P4/ServoBoardMain/src/safety/SafetyEventStore.h` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/safety/SafetyEventStore.h#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/safety/SafetyEventStore.h)
<a id="ref-0119"></a>

- **[119]** `mmhand:ESP32-P4/ServoBoardMain/src/safety/SafetyRuleRegistry.cpp` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/safety/SafetyRuleRegistry.cpp#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/safety/SafetyRuleRegistry.cpp)
  - 相关源码行：[L15–L15 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/safety/SafetyRuleRegistry.cpp#L15-L15) · [local](../../ESP32-P4/ServoBoardMain/src/safety/SafetyRuleRegistry.cpp), [L16–L16 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/safety/SafetyRuleRegistry.cpp#L16-L16) · [local](../../ESP32-P4/ServoBoardMain/src/safety/SafetyRuleRegistry.cpp), [L17–L17 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/safety/SafetyRuleRegistry.cpp#L17-L17) · [local](../../ESP32-P4/ServoBoardMain/src/safety/SafetyRuleRegistry.cpp), [L18–L18 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/safety/SafetyRuleRegistry.cpp#L18-L18) · [local](../../ESP32-P4/ServoBoardMain/src/safety/SafetyRuleRegistry.cpp), [L19–L19 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/safety/SafetyRuleRegistry.cpp#L19-L19) · [local](../../ESP32-P4/ServoBoardMain/src/safety/SafetyRuleRegistry.cpp), [L20–L20 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/safety/SafetyRuleRegistry.cpp#L20-L20) · [local](../../ESP32-P4/ServoBoardMain/src/safety/SafetyRuleRegistry.cpp), [L21–L21 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/safety/SafetyRuleRegistry.cpp#L21-L21) · [local](../../ESP32-P4/ServoBoardMain/src/safety/SafetyRuleRegistry.cpp), [L22–L22 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/safety/SafetyRuleRegistry.cpp#L22-L22) · [local](../../ESP32-P4/ServoBoardMain/src/safety/SafetyRuleRegistry.cpp), [L23–L23 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/safety/SafetyRuleRegistry.cpp#L23-L23) · [local](../../ESP32-P4/ServoBoardMain/src/safety/SafetyRuleRegistry.cpp), [L24–L24 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/safety/SafetyRuleRegistry.cpp#L24-L24) · [local](../../ESP32-P4/ServoBoardMain/src/safety/SafetyRuleRegistry.cpp), [L25–L25 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/safety/SafetyRuleRegistry.cpp#L25-L25) · [local](../../ESP32-P4/ServoBoardMain/src/safety/SafetyRuleRegistry.cpp), [L26–L26 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/safety/SafetyRuleRegistry.cpp#L26-L26) · [local](../../ESP32-P4/ServoBoardMain/src/safety/SafetyRuleRegistry.cpp), [L27–L27 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/safety/SafetyRuleRegistry.cpp#L27-L27) · [local](../../ESP32-P4/ServoBoardMain/src/safety/SafetyRuleRegistry.cpp), [L36–L36 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/safety/SafetyRuleRegistry.cpp#L36-L36) · [local](../../ESP32-P4/ServoBoardMain/src/safety/SafetyRuleRegistry.cpp), [L37–L37 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/safety/SafetyRuleRegistry.cpp#L37-L37) · [local](../../ESP32-P4/ServoBoardMain/src/safety/SafetyRuleRegistry.cpp), [L38–L38 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/safety/SafetyRuleRegistry.cpp#L38-L38) · [local](../../ESP32-P4/ServoBoardMain/src/safety/SafetyRuleRegistry.cpp)
<a id="ref-0120"></a>

- **[120]** `mmhand:ESP32-P4/ServoBoardMain/src/safety/SafetyRuleRegistry.h` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/safety/SafetyRuleRegistry.h#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/safety/SafetyRuleRegistry.h)
<a id="ref-0121"></a>

- **[121]** `mmhand:ESP32-P4/ServoBoardMain/src/safety/policies/ActuatorSafetyPolicy.cpp` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/safety/policies/ActuatorSafetyPolicy.cpp#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/safety/policies/ActuatorSafetyPolicy.cpp)
  - 相关源码行：[L138–L138 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/safety/policies/ActuatorSafetyPolicy.cpp#L138-L138) · [local](../../ESP32-P4/ServoBoardMain/src/safety/policies/ActuatorSafetyPolicy.cpp), [L153–L153 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/safety/policies/ActuatorSafetyPolicy.cpp#L153-L153) · [local](../../ESP32-P4/ServoBoardMain/src/safety/policies/ActuatorSafetyPolicy.cpp)
<a id="ref-0122"></a>

- **[122]** `mmhand:ESP32-P4/ServoBoardMain/src/safety/policies/ActuatorSafetyPolicy.h` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/safety/policies/ActuatorSafetyPolicy.h#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/safety/policies/ActuatorSafetyPolicy.h)
<a id="ref-0123"></a>

- **[123]** `mmhand:ESP32-P4/ServoBoardMain/src/safety/policies/CalibrationSafetyPolicy.cpp` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/safety/policies/CalibrationSafetyPolicy.cpp#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/safety/policies/CalibrationSafetyPolicy.cpp)
  - 相关源码行：[L9–L9 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/safety/policies/CalibrationSafetyPolicy.cpp#L9-L9) · [local](../../ESP32-P4/ServoBoardMain/src/safety/policies/CalibrationSafetyPolicy.cpp)
<a id="ref-0124"></a>

- **[124]** `mmhand:ESP32-P4/ServoBoardMain/src/safety/policies/CalibrationSafetyPolicy.h` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/safety/policies/CalibrationSafetyPolicy.h#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/safety/policies/CalibrationSafetyPolicy.h)
  - 相关源码行：[L23–L23 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/safety/policies/CalibrationSafetyPolicy.h#L23-L23) · [local](../../ESP32-P4/ServoBoardMain/src/safety/policies/CalibrationSafetyPolicy.h)
<a id="ref-0125"></a>

- **[125]** `mmhand:ESP32-P4/ServoBoardMain/src/safety/policies/CollisionSafetyPolicy.cpp` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/safety/policies/CollisionSafetyPolicy.cpp#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/safety/policies/CollisionSafetyPolicy.cpp)
  - 相关源码行：[L194–L194 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/safety/policies/CollisionSafetyPolicy.cpp#L194-L194) · [local](../../ESP32-P4/ServoBoardMain/src/safety/policies/CollisionSafetyPolicy.cpp), [L204–L204 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/safety/policies/CollisionSafetyPolicy.cpp#L204-L204) · [local](../../ESP32-P4/ServoBoardMain/src/safety/policies/CollisionSafetyPolicy.cpp), [L236–L236 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/safety/policies/CollisionSafetyPolicy.cpp#L236-L236) · [local](../../ESP32-P4/ServoBoardMain/src/safety/policies/CollisionSafetyPolicy.cpp), [L246–L246 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/safety/policies/CollisionSafetyPolicy.cpp#L246-L246) · [local](../../ESP32-P4/ServoBoardMain/src/safety/policies/CollisionSafetyPolicy.cpp), [L284–L284 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/safety/policies/CollisionSafetyPolicy.cpp#L284-L284) · [local](../../ESP32-P4/ServoBoardMain/src/safety/policies/CollisionSafetyPolicy.cpp), [L329–L329 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/safety/policies/CollisionSafetyPolicy.cpp#L329-L329) · [local](../../ESP32-P4/ServoBoardMain/src/safety/policies/CollisionSafetyPolicy.cpp), [L351–L351 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/safety/policies/CollisionSafetyPolicy.cpp#L351-L351) · [local](../../ESP32-P4/ServoBoardMain/src/safety/policies/CollisionSafetyPolicy.cpp)
<a id="ref-0126"></a>

- **[126]** `mmhand:ESP32-P4/ServoBoardMain/src/safety/policies/CollisionSafetyPolicy.h` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/safety/policies/CollisionSafetyPolicy.h#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/safety/policies/CollisionSafetyPolicy.h)
<a id="ref-0127"></a>

- **[127]** `mmhand:ESP32-P4/ServoBoardMain/src/safety/policies/CommunicationSafetyPolicy.cpp` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/safety/policies/CommunicationSafetyPolicy.cpp#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/safety/policies/CommunicationSafetyPolicy.cpp)
  - 相关源码行：[L9–L9 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/safety/policies/CommunicationSafetyPolicy.cpp#L9-L9) · [local](../../ESP32-P4/ServoBoardMain/src/safety/policies/CommunicationSafetyPolicy.cpp), [L13–L13 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/safety/policies/CommunicationSafetyPolicy.cpp#L13-L13) · [local](../../ESP32-P4/ServoBoardMain/src/safety/policies/CommunicationSafetyPolicy.cpp), [L16–L16 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/safety/policies/CommunicationSafetyPolicy.cpp#L16-L16) · [local](../../ESP32-P4/ServoBoardMain/src/safety/policies/CommunicationSafetyPolicy.cpp), [L62–L62 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/safety/policies/CommunicationSafetyPolicy.cpp#L62-L62) · [local](../../ESP32-P4/ServoBoardMain/src/safety/policies/CommunicationSafetyPolicy.cpp)
<a id="ref-0128"></a>

- **[128]** `mmhand:ESP32-P4/ServoBoardMain/src/safety/policies/CommunicationSafetyPolicy.h` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/safety/policies/CommunicationSafetyPolicy.h#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/safety/policies/CommunicationSafetyPolicy.h)
<a id="ref-0129"></a>

- **[129]** `mmhand:ESP32-P4/ServoBoardMain/src/safety/policies/ControlBoundarySafetyPolicy.cpp` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/safety/policies/ControlBoundarySafetyPolicy.cpp#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/safety/policies/ControlBoundarySafetyPolicy.cpp)
  - 相关源码行：[L11–L11 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/safety/policies/ControlBoundarySafetyPolicy.cpp#L11-L11) · [local](../../ESP32-P4/ServoBoardMain/src/safety/policies/ControlBoundarySafetyPolicy.cpp), [L33–L33 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/safety/policies/ControlBoundarySafetyPolicy.cpp#L33-L33) · [local](../../ESP32-P4/ServoBoardMain/src/safety/policies/ControlBoundarySafetyPolicy.cpp), [L52–L52 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/safety/policies/ControlBoundarySafetyPolicy.cpp#L52-L52) · [local](../../ESP32-P4/ServoBoardMain/src/safety/policies/ControlBoundarySafetyPolicy.cpp), [L78–L78 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/safety/policies/ControlBoundarySafetyPolicy.cpp#L78-L78) · [local](../../ESP32-P4/ServoBoardMain/src/safety/policies/ControlBoundarySafetyPolicy.cpp)
<a id="ref-0130"></a>

- **[130]** `mmhand:ESP32-P4/ServoBoardMain/src/safety/policies/ControlBoundarySafetyPolicy.h` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/safety/policies/ControlBoundarySafetyPolicy.h#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/safety/policies/ControlBoundarySafetyPolicy.h)
<a id="ref-0131"></a>

- **[131]** `mmhand:ESP32-P4/ServoBoardMain/src/safety/policies/ControlInputSafetyPolicy.cpp` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/safety/policies/ControlInputSafetyPolicy.cpp#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/safety/policies/ControlInputSafetyPolicy.cpp)
  - 相关源码行：[L25–L25 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/safety/policies/ControlInputSafetyPolicy.cpp#L25-L25) · [local](../../ESP32-P4/ServoBoardMain/src/safety/policies/ControlInputSafetyPolicy.cpp), [L37–L37 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/safety/policies/ControlInputSafetyPolicy.cpp#L37-L37) · [local](../../ESP32-P4/ServoBoardMain/src/safety/policies/ControlInputSafetyPolicy.cpp), [L44–L44 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/safety/policies/ControlInputSafetyPolicy.cpp#L44-L44) · [local](../../ESP32-P4/ServoBoardMain/src/safety/policies/ControlInputSafetyPolicy.cpp), [L51–L51 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/safety/policies/ControlInputSafetyPolicy.cpp#L51-L51) · [local](../../ESP32-P4/ServoBoardMain/src/safety/policies/ControlInputSafetyPolicy.cpp), [L56–L56 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/safety/policies/ControlInputSafetyPolicy.cpp#L56-L56) · [local](../../ESP32-P4/ServoBoardMain/src/safety/policies/ControlInputSafetyPolicy.cpp)
<a id="ref-0132"></a>

- **[132]** `mmhand:ESP32-P4/ServoBoardMain/src/safety/policies/ControlInputSafetyPolicy.h` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/safety/policies/ControlInputSafetyPolicy.h#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/safety/policies/ControlInputSafetyPolicy.h)
<a id="ref-0133"></a>

- **[133]** `mmhand:ESP32-P4/ServoBoardMain/src/safety/policies/EncoderHealthSafetyPolicy.cpp` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/safety/policies/EncoderHealthSafetyPolicy.cpp#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/safety/policies/EncoderHealthSafetyPolicy.cpp)
<a id="ref-0134"></a>

- **[134]** `mmhand:ESP32-P4/ServoBoardMain/src/safety/policies/EncoderHealthSafetyPolicy.h` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/safety/policies/EncoderHealthSafetyPolicy.h#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/safety/policies/EncoderHealthSafetyPolicy.h)
<a id="ref-0135"></a>

- **[135]** `mmhand:ESP32-P4/ServoBoardMain/src/safety/policies/HostLeaseSafetyPolicy.cpp` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/safety/policies/HostLeaseSafetyPolicy.cpp#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/safety/policies/HostLeaseSafetyPolicy.cpp)
  - 相关源码行：[L26–L26 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/safety/policies/HostLeaseSafetyPolicy.cpp#L26-L26) · [local](../../ESP32-P4/ServoBoardMain/src/safety/policies/HostLeaseSafetyPolicy.cpp)
<a id="ref-0136"></a>

- **[136]** `mmhand:ESP32-P4/ServoBoardMain/src/safety/policies/HostLeaseSafetyPolicy.h` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/safety/policies/HostLeaseSafetyPolicy.h#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/safety/policies/HostLeaseSafetyPolicy.h)
<a id="ref-0137"></a>

- **[137]** `mmhand:ESP32-P4/ServoBoardMain/src/safety/policies/OverloadSafetyPolicy.cpp` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/safety/policies/OverloadSafetyPolicy.cpp#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/safety/policies/OverloadSafetyPolicy.cpp)
<a id="ref-0138"></a>

- **[138]** `mmhand:ESP32-P4/ServoBoardMain/src/safety/policies/OverloadSafetyPolicy.h` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/safety/policies/OverloadSafetyPolicy.h#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/safety/policies/OverloadSafetyPolicy.h)
<a id="ref-0139"></a>

- **[139]** `mmhand:ESP32-P4/ServoBoardMain/src/safety/policies/RopeInitSafetyPolicy.cpp` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/safety/policies/RopeInitSafetyPolicy.cpp#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/safety/policies/RopeInitSafetyPolicy.cpp)
  - 相关源码行：[L56–L56 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/safety/policies/RopeInitSafetyPolicy.cpp#L56-L56) · [local](../../ESP32-P4/ServoBoardMain/src/safety/policies/RopeInitSafetyPolicy.cpp)
<a id="ref-0140"></a>

- **[140]** `mmhand:ESP32-P4/ServoBoardMain/src/safety/policies/RopeInitSafetyPolicy.h` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/safety/policies/RopeInitSafetyPolicy.h#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/safety/policies/RopeInitSafetyPolicy.h)
<a id="ref-0141"></a>

- **[141]** `mmhand:ESP32-P4/ServoBoardMain/src/safety/policies/RuntimeSafetyPolicy.cpp` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/safety/policies/RuntimeSafetyPolicy.cpp#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/safety/policies/RuntimeSafetyPolicy.cpp)
  - 相关源码行：[L9–L9 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/safety/policies/RuntimeSafetyPolicy.cpp#L9-L9) · [local](../../ESP32-P4/ServoBoardMain/src/safety/policies/RuntimeSafetyPolicy.cpp), [L15–L15 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/safety/policies/RuntimeSafetyPolicy.cpp#L15-L15) · [local](../../ESP32-P4/ServoBoardMain/src/safety/policies/RuntimeSafetyPolicy.cpp), [L17–L17 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/safety/policies/RuntimeSafetyPolicy.cpp#L17-L17) · [local](../../ESP32-P4/ServoBoardMain/src/safety/policies/RuntimeSafetyPolicy.cpp), [L19–L19 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/safety/policies/RuntimeSafetyPolicy.cpp#L19-L19) · [local](../../ESP32-P4/ServoBoardMain/src/safety/policies/RuntimeSafetyPolicy.cpp), [L21–L21 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/safety/policies/RuntimeSafetyPolicy.cpp#L21-L21) · [local](../../ESP32-P4/ServoBoardMain/src/safety/policies/RuntimeSafetyPolicy.cpp), [L25–L25 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/safety/policies/RuntimeSafetyPolicy.cpp#L25-L25) · [local](../../ESP32-P4/ServoBoardMain/src/safety/policies/RuntimeSafetyPolicy.cpp), [L40–L40 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/safety/policies/RuntimeSafetyPolicy.cpp#L40-L40) · [local](../../ESP32-P4/ServoBoardMain/src/safety/policies/RuntimeSafetyPolicy.cpp), [L49–L49 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/safety/policies/RuntimeSafetyPolicy.cpp#L49-L49) · [local](../../ESP32-P4/ServoBoardMain/src/safety/policies/RuntimeSafetyPolicy.cpp)
<a id="ref-0142"></a>

- **[142]** `mmhand:ESP32-P4/ServoBoardMain/src/safety/policies/RuntimeSafetyPolicy.h` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/safety/policies/RuntimeSafetyPolicy.h#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/safety/policies/RuntimeSafetyPolicy.h)
<a id="ref-0143"></a>

- **[143]** `mmhand:ESP32-P4/ServoBoardMain/src/safety/policies/StateAdmissionPolicy.cpp` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/safety/policies/StateAdmissionPolicy.cpp#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/safety/policies/StateAdmissionPolicy.cpp)
  - 相关源码行：[L11–L11 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/safety/policies/StateAdmissionPolicy.cpp#L11-L11) · [local](../../ESP32-P4/ServoBoardMain/src/safety/policies/StateAdmissionPolicy.cpp)
<a id="ref-0144"></a>

- **[144]** `mmhand:ESP32-P4/ServoBoardMain/src/safety/policies/StateAdmissionPolicy.h` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/safety/policies/StateAdmissionPolicy.h#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/safety/policies/StateAdmissionPolicy.h)
<a id="ref-0145"></a>

- **[145]** `mmhand:ESP32-P4/ServoBoardMain/src/safety/policies/SystemFaultPolicy.cpp` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/safety/policies/SystemFaultPolicy.cpp#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/safety/policies/SystemFaultPolicy.cpp)
  - 相关源码行：[L22–L22 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/safety/policies/SystemFaultPolicy.cpp#L22-L22) · [local](../../ESP32-P4/ServoBoardMain/src/safety/policies/SystemFaultPolicy.cpp)
<a id="ref-0146"></a>

- **[146]** `mmhand:ESP32-P4/ServoBoardMain/src/safety/policies/SystemFaultPolicy.h` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/safety/policies/SystemFaultPolicy.h#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/safety/policies/SystemFaultPolicy.h)
<a id="ref-0147"></a>

- **[147]** `mmhand:ESP32-P4/ServoBoardMain/src/safety/policies/SystemReadinessPolicy.cpp` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/safety/policies/SystemReadinessPolicy.cpp#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/safety/policies/SystemReadinessPolicy.cpp)
  - 相关源码行：[L105–L105 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/safety/policies/SystemReadinessPolicy.cpp#L105-L105) · [local](../../ESP32-P4/ServoBoardMain/src/safety/policies/SystemReadinessPolicy.cpp), [L182–L182 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/safety/policies/SystemReadinessPolicy.cpp#L182-L182) · [local](../../ESP32-P4/ServoBoardMain/src/safety/policies/SystemReadinessPolicy.cpp)
<a id="ref-0148"></a>

- **[148]** `mmhand:ESP32-P4/ServoBoardMain/src/safety/policies/SystemReadinessPolicy.h` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/safety/policies/SystemReadinessPolicy.h#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/safety/policies/SystemReadinessPolicy.h)
<a id="ref-0149"></a>

- **[149]** `mmhand:ESP32-P4/ServoBoardMain/src/tasks/auto_calibration/AutoCalibrationCoordinator.cpp` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/auto_calibration/AutoCalibrationCoordinator.cpp#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/auto_calibration/AutoCalibrationCoordinator.cpp)
<a id="ref-0150"></a>

- **[150]** `mmhand:ESP32-P4/ServoBoardMain/src/tasks/auto_calibration/AutoCalibrationCoordinator.h` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/auto_calibration/AutoCalibrationCoordinator.h#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/auto_calibration/AutoCalibrationCoordinator.h)
<a id="ref-0151"></a>

- **[151]** `mmhand:ESP32-P4/ServoBoardMain/src/tasks/auto_calibration/AutoCalibrationTask.cpp` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/auto_calibration/AutoCalibrationTask.cpp#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/auto_calibration/AutoCalibrationTask.cpp)
  - 相关源码行：[L27–L27 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/auto_calibration/AutoCalibrationTask.cpp#L27-L27) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/auto_calibration/AutoCalibrationTask.cpp)
<a id="ref-0152"></a>

- **[152]** `mmhand:ESP32-P4/ServoBoardMain/src/tasks/auto_calibration/AutoCalibrationTask.h` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/auto_calibration/AutoCalibrationTask.h#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/auto_calibration/AutoCalibrationTask.h)
<a id="ref-0153"></a>

- **[153]** `mmhand:ESP32-P4/ServoBoardMain/src/tasks/auto_calibration/AutoCalibrationTaskRuntime.cpp` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/auto_calibration/AutoCalibrationTaskRuntime.cpp#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/auto_calibration/AutoCalibrationTaskRuntime.cpp)
<a id="ref-0154"></a>

- **[154]** `mmhand:ESP32-P4/ServoBoardMain/src/tasks/auto_calibration/AutoCalibrationTaskRuntime.h` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/auto_calibration/AutoCalibrationTaskRuntime.h#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/auto_calibration/AutoCalibrationTaskRuntime.h)
<a id="ref-0155"></a>

- **[155]** `mmhand:ESP32-P4/ServoBoardMain/src/tasks/auto_calibration/CalibrationCoordinatorPort.h` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/auto_calibration/CalibrationCoordinatorPort.h#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/auto_calibration/CalibrationCoordinatorPort.h)
<a id="ref-0156"></a>

- **[156]** `mmhand:ESP32-P4/ServoBoardMain/src/tasks/auto_calibration/CalibrationDiagnostics.cpp` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/auto_calibration/CalibrationDiagnostics.cpp#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/auto_calibration/CalibrationDiagnostics.cpp)
  - 相关源码行：[L18–L18 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/auto_calibration/CalibrationDiagnostics.cpp#L18-L18) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/auto_calibration/CalibrationDiagnostics.cpp), [L28–L28 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/auto_calibration/CalibrationDiagnostics.cpp#L28-L28) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/auto_calibration/CalibrationDiagnostics.cpp), [L55–L55 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/auto_calibration/CalibrationDiagnostics.cpp#L55-L55) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/auto_calibration/CalibrationDiagnostics.cpp)
<a id="ref-0157"></a>

- **[157]** `mmhand:ESP32-P4/ServoBoardMain/src/tasks/auto_calibration/CalibrationDiagnostics.h` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/auto_calibration/CalibrationDiagnostics.h#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/auto_calibration/CalibrationDiagnostics.h)
<a id="ref-0158"></a>

- **[158]** `mmhand:ESP32-P4/ServoBoardMain/src/tasks/auto_calibration/CalibrationFeedbackReader.cpp` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/auto_calibration/CalibrationFeedbackReader.cpp#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/auto_calibration/CalibrationFeedbackReader.cpp)
<a id="ref-0159"></a>

- **[159]** `mmhand:ESP32-P4/ServoBoardMain/src/tasks/auto_calibration/CalibrationFeedbackReader.h` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/auto_calibration/CalibrationFeedbackReader.h#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/auto_calibration/CalibrationFeedbackReader.h)
<a id="ref-0160"></a>

- **[160]** `mmhand:ESP32-P4/ServoBoardMain/src/tasks/auto_calibration/CalibrationLutBuilder.cpp` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/auto_calibration/CalibrationLutBuilder.cpp#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/auto_calibration/CalibrationLutBuilder.cpp)
<a id="ref-0161"></a>

- **[161]** `mmhand:ESP32-P4/ServoBoardMain/src/tasks/auto_calibration/CalibrationLutBuilder.h` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/auto_calibration/CalibrationLutBuilder.h#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/auto_calibration/CalibrationLutBuilder.h)
<a id="ref-0162"></a>

- **[162]** `mmhand:ESP32-P4/ServoBoardMain/src/tasks/auto_calibration/CalibrationMotionSupport.cpp` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/auto_calibration/CalibrationMotionSupport.cpp#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/auto_calibration/CalibrationMotionSupport.cpp)
<a id="ref-0163"></a>

- **[163]** `mmhand:ESP32-P4/ServoBoardMain/src/tasks/auto_calibration/CalibrationMotionSupport.h` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/auto_calibration/CalibrationMotionSupport.h#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/auto_calibration/CalibrationMotionSupport.h)
<a id="ref-0164"></a>

- **[164]** `mmhand:ESP32-P4/ServoBoardMain/src/tasks/auto_calibration/CalibrationResultCommitter.cpp` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/auto_calibration/CalibrationResultCommitter.cpp#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/auto_calibration/CalibrationResultCommitter.cpp)
<a id="ref-0165"></a>

- **[165]** `mmhand:ESP32-P4/ServoBoardMain/src/tasks/auto_calibration/CalibrationResultCommitter.h` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/auto_calibration/CalibrationResultCommitter.h#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/auto_calibration/CalibrationResultCommitter.h)
<a id="ref-0166"></a>

- **[166]** `mmhand:ESP32-P4/ServoBoardMain/src/tasks/auto_calibration/CalibrationRuntimePort.cpp` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/auto_calibration/CalibrationRuntimePort.cpp#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/auto_calibration/CalibrationRuntimePort.cpp)
<a id="ref-0167"></a>

- **[167]** `mmhand:ESP32-P4/ServoBoardMain/src/tasks/auto_calibration/CalibrationRuntimePort.h` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/auto_calibration/CalibrationRuntimePort.h#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/auto_calibration/CalibrationRuntimePort.h)
<a id="ref-0168"></a>

- **[168]** `mmhand:ESP32-P4/ServoBoardMain/src/tasks/auto_calibration/CalibrationSequencePlanner.cpp` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/auto_calibration/CalibrationSequencePlanner.cpp#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/auto_calibration/CalibrationSequencePlanner.cpp)
<a id="ref-0169"></a>

- **[169]** `mmhand:ESP32-P4/ServoBoardMain/src/tasks/auto_calibration/CalibrationSequencePlanner.h` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/auto_calibration/CalibrationSequencePlanner.h#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/auto_calibration/CalibrationSequencePlanner.h)
<a id="ref-0170"></a>

- **[170]** `mmhand:ESP32-P4/ServoBoardMain/src/tasks/auto_calibration/CalibrationSessionController.cpp` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/auto_calibration/CalibrationSessionController.cpp#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/auto_calibration/CalibrationSessionController.cpp)
<a id="ref-0171"></a>

- **[171]** `mmhand:ESP32-P4/ServoBoardMain/src/tasks/auto_calibration/CalibrationSessionController.h` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/auto_calibration/CalibrationSessionController.h#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/auto_calibration/CalibrationSessionController.h)
<a id="ref-0172"></a>

- **[172]** `mmhand:ESP32-P4/ServoBoardMain/src/tasks/auto_calibration/dual_motor/DualMotorCalibrationMachine.cpp` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/auto_calibration/dual_motor/DualMotorCalibrationMachine.cpp#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/auto_calibration/dual_motor/DualMotorCalibrationMachine.cpp)
<a id="ref-0173"></a>

- **[173]** `mmhand:ESP32-P4/ServoBoardMain/src/tasks/auto_calibration/dual_motor/DualMotorCalibrationMachine.h` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/auto_calibration/dual_motor/DualMotorCalibrationMachine.h#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/auto_calibration/dual_motor/DualMotorCalibrationMachine.h)
<a id="ref-0174"></a>

- **[174]** `mmhand:ESP32-P4/ServoBoardMain/src/tasks/auto_calibration/single_motor/SingleMotorCalibrationMachine.cpp` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/auto_calibration/single_motor/SingleMotorCalibrationMachine.cpp#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/auto_calibration/single_motor/SingleMotorCalibrationMachine.cpp)
<a id="ref-0175"></a>

- **[175]** `mmhand:ESP32-P4/ServoBoardMain/src/tasks/auto_calibration/single_motor/SingleMotorCalibrationMachine.h` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/auto_calibration/single_motor/SingleMotorCalibrationMachine.h#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/auto_calibration/single_motor/SingleMotorCalibrationMachine.h)
<a id="ref-0176"></a>

- **[176]** `mmhand:ESP32-P4/ServoBoardMain/src/tasks/host_comm/HostAckPublisher.cpp` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/host_comm/HostAckPublisher.cpp#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/host_comm/HostAckPublisher.cpp)
<a id="ref-0177"></a>

- **[177]** `mmhand:ESP32-P4/ServoBoardMain/src/tasks/host_comm/HostAckPublisher.h` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/host_comm/HostAckPublisher.h#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/host_comm/HostAckPublisher.h)
<a id="ref-0178"></a>

- **[178]** `mmhand:ESP32-P4/ServoBoardMain/src/tasks/host_comm/HostCalibrationCommandHandler.cpp` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/host_comm/HostCalibrationCommandHandler.cpp#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/host_comm/HostCalibrationCommandHandler.cpp)
<a id="ref-0179"></a>

- **[179]** `mmhand:ESP32-P4/ServoBoardMain/src/tasks/host_comm/HostCalibrationCommandHandler.h` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/host_comm/HostCalibrationCommandHandler.h#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/host_comm/HostCalibrationCommandHandler.h)
<a id="ref-0180"></a>

- **[180]** `mmhand:ESP32-P4/ServoBoardMain/src/tasks/host_comm/HostCommService.cpp` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/host_comm/HostCommService.cpp#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/host_comm/HostCommService.cpp)
  - 相关源码行：[L223–L223 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/host_comm/HostCommService.cpp#L223-L223) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/host_comm/HostCommService.cpp), [L228–L228 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/host_comm/HostCommService.cpp#L228-L228) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/host_comm/HostCommService.cpp), [L230–L230 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/host_comm/HostCommService.cpp#L230-L230) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/host_comm/HostCommService.cpp), [L266–L266 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/host_comm/HostCommService.cpp#L266-L266) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/host_comm/HostCommService.cpp)
<a id="ref-0181"></a>

- **[181]** `mmhand:ESP32-P4/ServoBoardMain/src/tasks/host_comm/HostCommService.h` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/host_comm/HostCommService.h#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/host_comm/HostCommService.h)
<a id="ref-0182"></a>

- **[182]** `mmhand:ESP32-P4/ServoBoardMain/src/tasks/host_comm/HostCommTask.cpp` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/host_comm/HostCommTask.cpp#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/host_comm/HostCommTask.cpp)
  - 相关源码行：[L1–L1 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/host_comm/HostCommTask.cpp#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/host_comm/HostCommTask.cpp), [L25–L25 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/host_comm/HostCommTask.cpp#L25-L25) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/host_comm/HostCommTask.cpp), [L39–L39 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/host_comm/HostCommTask.cpp#L39-L39) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/host_comm/HostCommTask.cpp), [L41–L41 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/host_comm/HostCommTask.cpp#L41-L41) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/host_comm/HostCommTask.cpp), [L48–L48 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/host_comm/HostCommTask.cpp#L48-L48) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/host_comm/HostCommTask.cpp), [L72–L72 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/host_comm/HostCommTask.cpp#L72-L72) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/host_comm/HostCommTask.cpp)
<a id="ref-0183"></a>

- **[183]** `mmhand:ESP32-P4/ServoBoardMain/src/tasks/host_comm/HostCommTask.h` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/host_comm/HostCommTask.h#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/host_comm/HostCommTask.h)
<a id="ref-0184"></a>

- **[184]** `mmhand:ESP32-P4/ServoBoardMain/src/tasks/host_comm/HostCommandDispatcher.cpp` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/host_comm/HostCommandDispatcher.cpp#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/host_comm/HostCommandDispatcher.cpp)
  - 相关源码行：[L211–L211 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/host_comm/HostCommandDispatcher.cpp#L211-L211) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/host_comm/HostCommandDispatcher.cpp), [L219–L219 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/host_comm/HostCommandDispatcher.cpp#L219-L219) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/host_comm/HostCommandDispatcher.cpp), [L227–L227 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/host_comm/HostCommandDispatcher.cpp#L227-L227) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/host_comm/HostCommandDispatcher.cpp), [L229–L229 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/host_comm/HostCommandDispatcher.cpp#L229-L229) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/host_comm/HostCommandDispatcher.cpp), [L267–L267 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/host_comm/HostCommandDispatcher.cpp#L267-L267) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/host_comm/HostCommandDispatcher.cpp), [L269–L269 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/host_comm/HostCommandDispatcher.cpp#L269-L269) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/host_comm/HostCommandDispatcher.cpp), [L274–L274 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/host_comm/HostCommandDispatcher.cpp#L274-L274) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/host_comm/HostCommandDispatcher.cpp), [L277–L277 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/host_comm/HostCommandDispatcher.cpp#L277-L277) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/host_comm/HostCommandDispatcher.cpp), [L285–L285 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/host_comm/HostCommandDispatcher.cpp#L285-L285) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/host_comm/HostCommandDispatcher.cpp), [L287–L287 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/host_comm/HostCommandDispatcher.cpp#L287-L287) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/host_comm/HostCommandDispatcher.cpp)
<a id="ref-0185"></a>

- **[185]** `mmhand:ESP32-P4/ServoBoardMain/src/tasks/host_comm/HostCommandDispatcher.h` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/host_comm/HostCommandDispatcher.h#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/host_comm/HostCommandDispatcher.h)
<a id="ref-0186"></a>

- **[186]** `mmhand:ESP32-P4/ServoBoardMain/src/tasks/host_comm/HostCommandRouter.cpp` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/host_comm/HostCommandRouter.cpp#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/host_comm/HostCommandRouter.cpp)
  - 相关源码行：[L85–L85 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/host_comm/HostCommandRouter.cpp#L85-L85) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/host_comm/HostCommandRouter.cpp)
<a id="ref-0187"></a>

- **[187]** `mmhand:ESP32-P4/ServoBoardMain/src/tasks/host_comm/HostCommandRouter.h` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/host_comm/HostCommandRouter.h#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/host_comm/HostCommandRouter.h)
<a id="ref-0188"></a>

- **[188]** `mmhand:ESP32-P4/ServoBoardMain/src/tasks/host_comm/HostFrameReader.cpp` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/host_comm/HostFrameReader.cpp#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/host_comm/HostFrameReader.cpp)
  - 相关源码行：[L40–L40 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/host_comm/HostFrameReader.cpp#L40-L40) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/host_comm/HostFrameReader.cpp)
<a id="ref-0189"></a>

- **[189]** `mmhand:ESP32-P4/ServoBoardMain/src/tasks/host_comm/HostFrameReader.h` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/host_comm/HostFrameReader.h#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/host_comm/HostFrameReader.h)
<a id="ref-0190"></a>

- **[190]** `mmhand:ESP32-P4/ServoBoardMain/src/tasks/host_comm/HostMultiTurnPublisher.cpp` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/host_comm/HostMultiTurnPublisher.cpp#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/host_comm/HostMultiTurnPublisher.cpp)
<a id="ref-0191"></a>

- **[191]** `mmhand:ESP32-P4/ServoBoardMain/src/tasks/host_comm/HostMultiTurnPublisher.h` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/host_comm/HostMultiTurnPublisher.h#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/host_comm/HostMultiTurnPublisher.h)
<a id="ref-0192"></a>

- **[192]** `mmhand:ESP32-P4/ServoBoardMain/src/tasks/host_comm/HostProtocolCodec.cpp` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/host_comm/HostProtocolCodec.cpp#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/host_comm/HostProtocolCodec.cpp)
<a id="ref-0193"></a>

- **[193]** `mmhand:ESP32-P4/ServoBoardMain/src/tasks/host_comm/HostProtocolCodec.h` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/host_comm/HostProtocolCodec.h#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/host_comm/HostProtocolCodec.h)
<a id="ref-0194"></a>

- **[194]** `mmhand:ESP32-P4/ServoBoardMain/src/tasks/host_comm/HostTelemetryPublisher.cpp` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/host_comm/HostTelemetryPublisher.cpp#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/host_comm/HostTelemetryPublisher.cpp)
<a id="ref-0195"></a>

- **[195]** `mmhand:ESP32-P4/ServoBoardMain/src/tasks/host_comm/HostTelemetryPublisher.h` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/host_comm/HostTelemetryPublisher.h#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/host_comm/HostTelemetryPublisher.h)
<a id="ref-0196"></a>

- **[196]** `mmhand:ESP32-P4/ServoBoardMain/src/tasks/motion_control/ControlCommandSnapshotReader.cpp` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/motion_control/ControlCommandSnapshotReader.cpp#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/motion_control/ControlCommandSnapshotReader.cpp)
<a id="ref-0197"></a>

- **[197]** `mmhand:ESP32-P4/ServoBoardMain/src/tasks/motion_control/ControlCommandSnapshotReader.h` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/motion_control/ControlCommandSnapshotReader.h#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/motion_control/ControlCommandSnapshotReader.h)
<a id="ref-0198"></a>

- **[198]** `mmhand:ESP32-P4/ServoBoardMain/src/tasks/motion_control/ControlSolver.cpp` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/motion_control/ControlSolver.cpp#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/motion_control/ControlSolver.cpp)
<a id="ref-0199"></a>

- **[199]** `mmhand:ESP32-P4/ServoBoardMain/src/tasks/motion_control/ControlSolver.h` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/motion_control/ControlSolver.h#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/motion_control/ControlSolver.h)
<a id="ref-0200"></a>

- **[200]** `mmhand:ESP32-P4/ServoBoardMain/src/tasks/motion_control/DualTendonControl.cpp` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/motion_control/DualTendonControl.cpp#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/motion_control/DualTendonControl.cpp)
  - 相关源码行：[L290–L290 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/motion_control/DualTendonControl.cpp#L290-L290) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/motion_control/DualTendonControl.cpp), [L310–L310 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/motion_control/DualTendonControl.cpp#L310-L310) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/motion_control/DualTendonControl.cpp)
<a id="ref-0201"></a>

- **[201]** `mmhand:ESP32-P4/ServoBoardMain/src/tasks/motion_control/DualTendonControl.h` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/motion_control/DualTendonControl.h#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/motion_control/DualTendonControl.h)
<a id="ref-0202"></a>

- **[202]** `mmhand:ESP32-P4/ServoBoardMain/src/tasks/motion_control/JointAngleMapper.cpp` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/motion_control/JointAngleMapper.cpp#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/motion_control/JointAngleMapper.cpp)
<a id="ref-0203"></a>

- **[203]** `mmhand:ESP32-P4/ServoBoardMain/src/tasks/motion_control/JointAngleMapper.h` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/motion_control/JointAngleMapper.h#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/motion_control/JointAngleMapper.h)
<a id="ref-0204"></a>

- **[204]** `mmhand:ESP32-P4/ServoBoardMain/src/tasks/motion_control/MotionCompletionEvaluator.cpp` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/motion_control/MotionCompletionEvaluator.cpp#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/motion_control/MotionCompletionEvaluator.cpp)
<a id="ref-0205"></a>

- **[205]** `mmhand:ESP32-P4/ServoBoardMain/src/tasks/motion_control/MotionCompletionEvaluator.h` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/motion_control/MotionCompletionEvaluator.h#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/motion_control/MotionCompletionEvaluator.h)
<a id="ref-0206"></a>

- **[206]** `mmhand:ESP32-P4/ServoBoardMain/src/tasks/motion_control/MotionControlService.cpp` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/motion_control/MotionControlService.cpp#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/motion_control/MotionControlService.cpp)
  - 相关源码行：[L113–L113 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/motion_control/MotionControlService.cpp#L113-L113) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/motion_control/MotionControlService.cpp), [L115–L115 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/motion_control/MotionControlService.cpp#L115-L115) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/motion_control/MotionControlService.cpp), [L117–L117 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/motion_control/MotionControlService.cpp#L117-L117) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/motion_control/MotionControlService.cpp), [L124–L124 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/motion_control/MotionControlService.cpp#L124-L124) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/motion_control/MotionControlService.cpp), [L437–L437 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/motion_control/MotionControlService.cpp#L437-L437) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/motion_control/MotionControlService.cpp), [L448–L448 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/motion_control/MotionControlService.cpp#L448-L448) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/motion_control/MotionControlService.cpp), [L478–L478 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/motion_control/MotionControlService.cpp#L478-L478) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/motion_control/MotionControlService.cpp), [L943–L943 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/motion_control/MotionControlService.cpp#L943-L943) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/motion_control/MotionControlService.cpp), [L954–L954 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/motion_control/MotionControlService.cpp#L954-L954) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/motion_control/MotionControlService.cpp), [L1066–L1066 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/motion_control/MotionControlService.cpp#L1066-L1066) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/motion_control/MotionControlService.cpp), [L1080–L1080 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/motion_control/MotionControlService.cpp#L1080-L1080) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/motion_control/MotionControlService.cpp), [L1191–L1191 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/motion_control/MotionControlService.cpp#L1191-L1191) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/motion_control/MotionControlService.cpp), [L1210–L1210 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/motion_control/MotionControlService.cpp#L1210-L1210) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/motion_control/MotionControlService.cpp), [L1220–L1220 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/motion_control/MotionControlService.cpp#L1220-L1220) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/motion_control/MotionControlService.cpp), [L1225–L1225 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/motion_control/MotionControlService.cpp#L1225-L1225) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/motion_control/MotionControlService.cpp), [L1318–L1318 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/motion_control/MotionControlService.cpp#L1318-L1318) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/motion_control/MotionControlService.cpp)
<a id="ref-0207"></a>

- **[207]** `mmhand:ESP32-P4/ServoBoardMain/src/tasks/motion_control/MotionControlService.h` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/motion_control/MotionControlService.h#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/motion_control/MotionControlService.h)
<a id="ref-0208"></a>

- **[208]** `mmhand:ESP32-P4/ServoBoardMain/src/tasks/motion_control/MotionControlTask.cpp` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/motion_control/MotionControlTask.cpp#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/motion_control/MotionControlTask.cpp)
  - 相关源码行：[L1–L1 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/motion_control/MotionControlTask.cpp#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/motion_control/MotionControlTask.cpp), [L26–L26 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/motion_control/MotionControlTask.cpp#L26-L26) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/motion_control/MotionControlTask.cpp), [L27–L27 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/motion_control/MotionControlTask.cpp#L27-L27) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/motion_control/MotionControlTask.cpp), [L43–L43 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/motion_control/MotionControlTask.cpp#L43-L43) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/motion_control/MotionControlTask.cpp), [L45–L45 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/motion_control/MotionControlTask.cpp#L45-L45) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/motion_control/MotionControlTask.cpp)
<a id="ref-0209"></a>

- **[209]** `mmhand:ESP32-P4/ServoBoardMain/src/tasks/motion_control/MotionControlTask.h` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/motion_control/MotionControlTask.h#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/motion_control/MotionControlTask.h)
<a id="ref-0210"></a>

- **[210]** `mmhand:ESP32-P4/ServoBoardMain/src/tasks/motion_control/ServoTargetBuilder.cpp` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/motion_control/ServoTargetBuilder.cpp#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/motion_control/ServoTargetBuilder.cpp)
<a id="ref-0211"></a>

- **[211]** `mmhand:ESP32-P4/ServoBoardMain/src/tasks/motion_control/ServoTargetBuilder.h` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/motion_control/ServoTargetBuilder.h#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/motion_control/ServoTargetBuilder.h)
<a id="ref-0212"></a>

- **[212]** `mmhand:ESP32-P4/ServoBoardMain/src/tasks/motion_control/pid.c` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/motion_control/pid.c#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/motion_control/pid.c)
  - 相关源码行：[L70–L70 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/motion_control/pid.c#L70-L70) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/motion_control/pid.c)
<a id="ref-0213"></a>

- **[213]** `mmhand:ESP32-P4/ServoBoardMain/src/tasks/motion_control/pid.h` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/motion_control/pid.h#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/motion_control/pid.h)
<a id="ref-0214"></a>

- **[214]** `mmhand:ESP32-P4/ServoBoardMain/src/tasks/runtime_monitor/RuntimeMonitorService.cpp` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/runtime_monitor/RuntimeMonitorService.cpp#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/runtime_monitor/RuntimeMonitorService.cpp)
  - 相关源码行：[L42–L42 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/runtime_monitor/RuntimeMonitorService.cpp#L42-L42) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/runtime_monitor/RuntimeMonitorService.cpp), [L43–L43 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/runtime_monitor/RuntimeMonitorService.cpp#L43-L43) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/runtime_monitor/RuntimeMonitorService.cpp), [L122–L122 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/runtime_monitor/RuntimeMonitorService.cpp#L122-L122) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/runtime_monitor/RuntimeMonitorService.cpp), [L132–L132 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/runtime_monitor/RuntimeMonitorService.cpp#L132-L132) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/runtime_monitor/RuntimeMonitorService.cpp), [L133–L133 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/runtime_monitor/RuntimeMonitorService.cpp#L133-L133) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/runtime_monitor/RuntimeMonitorService.cpp), [L465–L465 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/runtime_monitor/RuntimeMonitorService.cpp#L465-L465) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/runtime_monitor/RuntimeMonitorService.cpp), [L478–L478 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/runtime_monitor/RuntimeMonitorService.cpp#L478-L478) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/runtime_monitor/RuntimeMonitorService.cpp)
<a id="ref-0215"></a>

- **[215]** `mmhand:ESP32-P4/ServoBoardMain/src/tasks/runtime_monitor/RuntimeMonitorService.h` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/runtime_monitor/RuntimeMonitorService.h#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/runtime_monitor/RuntimeMonitorService.h)
<a id="ref-0216"></a>

- **[216]** `mmhand:ESP32-P4/ServoBoardMain/src/tasks/runtime_monitor/RuntimeMonitorTask.cpp` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/runtime_monitor/RuntimeMonitorTask.cpp#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/runtime_monitor/RuntimeMonitorTask.cpp)
  - 相关源码行：[L1–L1 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/runtime_monitor/RuntimeMonitorTask.cpp#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/runtime_monitor/RuntimeMonitorTask.cpp), [L19–L19 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/runtime_monitor/RuntimeMonitorTask.cpp#L19-L19) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/runtime_monitor/RuntimeMonitorTask.cpp), [L28–L28 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/runtime_monitor/RuntimeMonitorTask.cpp#L28-L28) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/runtime_monitor/RuntimeMonitorTask.cpp), [L30–L30 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/runtime_monitor/RuntimeMonitorTask.cpp#L30-L30) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/runtime_monitor/RuntimeMonitorTask.cpp)
<a id="ref-0217"></a>

- **[217]** `mmhand:ESP32-P4/ServoBoardMain/src/tasks/runtime_monitor/RuntimeMonitorTask.h` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/runtime_monitor/RuntimeMonitorTask.h#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/runtime_monitor/RuntimeMonitorTask.h)
<a id="ref-0218"></a>

- **[218]** `mmhand:ESP32-P4/ServoBoardMain/src/tasks/s3_can_rx/EncoderHealthMonitor.cpp` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/s3_can_rx/EncoderHealthMonitor.cpp#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/s3_can_rx/EncoderHealthMonitor.cpp)
  - 相关源码行：[L48–L48 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/s3_can_rx/EncoderHealthMonitor.cpp#L48-L48) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/s3_can_rx/EncoderHealthMonitor.cpp)
<a id="ref-0219"></a>

- **[219]** `mmhand:ESP32-P4/ServoBoardMain/src/tasks/s3_can_rx/EncoderHealthMonitor.h` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/s3_can_rx/EncoderHealthMonitor.h#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/s3_can_rx/EncoderHealthMonitor.h)
<a id="ref-0220"></a>

- **[220]** `mmhand:ESP32-P4/ServoBoardMain/src/tasks/s3_can_rx/S3CanRxService.cpp` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/s3_can_rx/S3CanRxService.cpp#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/s3_can_rx/S3CanRxService.cpp)
  - 相关源码行：[L216–L216 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/s3_can_rx/S3CanRxService.cpp#L216-L216) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/s3_can_rx/S3CanRxService.cpp), [L228–L228 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/s3_can_rx/S3CanRxService.cpp#L228-L228) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/s3_can_rx/S3CanRxService.cpp), [L230–L230 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/s3_can_rx/S3CanRxService.cpp#L230-L230) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/s3_can_rx/S3CanRxService.cpp), [L289–L289 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/s3_can_rx/S3CanRxService.cpp#L289-L289) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/s3_can_rx/S3CanRxService.cpp), [L293–L293 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/s3_can_rx/S3CanRxService.cpp#L293-L293) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/s3_can_rx/S3CanRxService.cpp), [L328–L328 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/s3_can_rx/S3CanRxService.cpp#L328-L328) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/s3_can_rx/S3CanRxService.cpp), [L344–L344 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/s3_can_rx/S3CanRxService.cpp#L344-L344) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/s3_can_rx/S3CanRxService.cpp), [L384–L384 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/s3_can_rx/S3CanRxService.cpp#L384-L384) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/s3_can_rx/S3CanRxService.cpp), [L389–L389 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/s3_can_rx/S3CanRxService.cpp#L389-L389) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/s3_can_rx/S3CanRxService.cpp), [L453–L453 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/s3_can_rx/S3CanRxService.cpp#L453-L453) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/s3_can_rx/S3CanRxService.cpp), [L508–L508 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/s3_can_rx/S3CanRxService.cpp#L508-L508) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/s3_can_rx/S3CanRxService.cpp), [L695–L695 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/s3_can_rx/S3CanRxService.cpp#L695-L695) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/s3_can_rx/S3CanRxService.cpp), [L709–L709 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/s3_can_rx/S3CanRxService.cpp#L709-L709) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/s3_can_rx/S3CanRxService.cpp)
<a id="ref-0221"></a>

- **[221]** `mmhand:ESP32-P4/ServoBoardMain/src/tasks/s3_can_rx/S3CanRxService.h` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/s3_can_rx/S3CanRxService.h#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/s3_can_rx/S3CanRxService.h)
<a id="ref-0222"></a>

- **[222]** `mmhand:ESP32-P4/ServoBoardMain/src/tasks/s3_can_rx/S3CanRxTask.cpp` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/s3_can_rx/S3CanRxTask.cpp#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/s3_can_rx/S3CanRxTask.cpp)
  - 相关源码行：[L1–L1 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/s3_can_rx/S3CanRxTask.cpp#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/s3_can_rx/S3CanRxTask.cpp), [L32–L32 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/s3_can_rx/S3CanRxTask.cpp#L32-L32) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/s3_can_rx/S3CanRxTask.cpp), [L34–L34 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/s3_can_rx/S3CanRxTask.cpp#L34-L34) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/s3_can_rx/S3CanRxTask.cpp)
<a id="ref-0223"></a>

- **[223]** `mmhand:ESP32-P4/ServoBoardMain/src/tasks/s3_can_rx/S3CanRxTask.h` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/s3_can_rx/S3CanRxTask.h#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/s3_can_rx/S3CanRxTask.h)
<a id="ref-0224"></a>

- **[224]** `mmhand:ESP32-P4/ServoBoardMain/src/tasks/servo_bus/ParallelServoFeedbackCycle.cpp` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/servo_bus/ParallelServoFeedbackCycle.cpp#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/servo_bus/ParallelServoFeedbackCycle.cpp)
<a id="ref-0225"></a>

- **[225]** `mmhand:ESP32-P4/ServoBoardMain/src/tasks/servo_bus/ParallelServoFeedbackCycle.h` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/servo_bus/ParallelServoFeedbackCycle.h#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/servo_bus/ParallelServoFeedbackCycle.h)
<a id="ref-0226"></a>

- **[226]** `mmhand:ESP32-P4/ServoBoardMain/src/tasks/servo_bus/ServoBusRouting.cpp` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/servo_bus/ServoBusRouting.cpp#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/servo_bus/ServoBusRouting.cpp)
<a id="ref-0227"></a>

- **[227]** `mmhand:ESP32-P4/ServoBoardMain/src/tasks/servo_bus/ServoBusRouting.h` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/servo_bus/ServoBusRouting.h#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/servo_bus/ServoBusRouting.h)
<a id="ref-0228"></a>

- **[228]** `mmhand:ESP32-P4/ServoBoardMain/src/tasks/servo_bus/ServoBusService.cpp` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/servo_bus/ServoBusService.cpp#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/servo_bus/ServoBusService.cpp)
  - 相关源码行：[L207–L207 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/servo_bus/ServoBusService.cpp#L207-L207) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/servo_bus/ServoBusService.cpp), [L231–L231 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/servo_bus/ServoBusService.cpp#L231-L231) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/servo_bus/ServoBusService.cpp), [L258–L258 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/servo_bus/ServoBusService.cpp#L258-L258) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/servo_bus/ServoBusService.cpp)
<a id="ref-0229"></a>

- **[229]** `mmhand:ESP32-P4/ServoBoardMain/src/tasks/servo_bus/ServoBusService.h` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/servo_bus/ServoBusService.h#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/servo_bus/ServoBusService.h)
<a id="ref-0230"></a>

- **[230]** `mmhand:ESP32-P4/ServoBoardMain/src/tasks/servo_bus/ServoBusTask.cpp` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/servo_bus/ServoBusTask.cpp#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/servo_bus/ServoBusTask.cpp)
  - 相关源码行：[L1–L1 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/servo_bus/ServoBusTask.cpp#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/servo_bus/ServoBusTask.cpp), [L29–L29 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/servo_bus/ServoBusTask.cpp#L29-L29) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/servo_bus/ServoBusTask.cpp), [L32–L32 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/servo_bus/ServoBusTask.cpp#L32-L32) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/servo_bus/ServoBusTask.cpp), [L47–L47 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/servo_bus/ServoBusTask.cpp#L47-L47) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/servo_bus/ServoBusTask.cpp), [L52–L52 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/servo_bus/ServoBusTask.cpp#L52-L52) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/servo_bus/ServoBusTask.cpp), [L81–L81 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/servo_bus/ServoBusTask.cpp#L81-L81) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/servo_bus/ServoBusTask.cpp)
<a id="ref-0231"></a>

- **[231]** `mmhand:ESP32-P4/ServoBoardMain/src/tasks/servo_bus/ServoBusTask.h` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/servo_bus/ServoBusTask.h#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/servo_bus/ServoBusTask.h)
<a id="ref-0232"></a>

- **[232]** `mmhand:ESP32-P4/ServoBoardMain/src/tasks/servo_bus/ServoDiscoveryManager.cpp` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/servo_bus/ServoDiscoveryManager.cpp#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/servo_bus/ServoDiscoveryManager.cpp)
<a id="ref-0233"></a>

- **[233]** `mmhand:ESP32-P4/ServoBoardMain/src/tasks/servo_bus/ServoDiscoveryManager.h` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/servo_bus/ServoDiscoveryManager.h#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/servo_bus/ServoDiscoveryManager.h)
<a id="ref-0234"></a>

- **[234]** `mmhand:ESP32-P4/ServoBoardMain/src/tasks/servo_bus/ServoFeedbackPublisher.cpp` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/servo_bus/ServoFeedbackPublisher.cpp#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/servo_bus/ServoFeedbackPublisher.cpp)
<a id="ref-0235"></a>

- **[235]** `mmhand:ESP32-P4/ServoBoardMain/src/tasks/servo_bus/ServoFeedbackPublisher.h` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/servo_bus/ServoFeedbackPublisher.h#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/servo_bus/ServoFeedbackPublisher.h)
<a id="ref-0236"></a>

- **[236]** `mmhand:ESP32-P4/ServoBoardMain/src/tasks/servo_bus/ServoRouteTable.cpp` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/servo_bus/ServoRouteTable.cpp#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/servo_bus/ServoRouteTable.cpp)
<a id="ref-0237"></a>

- **[237]** `mmhand:ESP32-P4/ServoBoardMain/src/tasks/servo_bus/ServoRouteTable.h` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/servo_bus/ServoRouteTable.h#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/servo_bus/ServoRouteTable.h)
<a id="ref-0238"></a>

- **[238]** `mmhand:ESP32-P4/ServoBoardMain/src/tasks/system_state/DirectCommandCoordinator.cpp` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/system_state/DirectCommandCoordinator.cpp#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/system_state/DirectCommandCoordinator.cpp)
  - 相关源码行：[L55–L55 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/system_state/DirectCommandCoordinator.cpp#L55-L55) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/system_state/DirectCommandCoordinator.cpp), [L198–L198 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/system_state/DirectCommandCoordinator.cpp#L198-L198) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/system_state/DirectCommandCoordinator.cpp)
<a id="ref-0239"></a>

- **[239]** `mmhand:ESP32-P4/ServoBoardMain/src/tasks/system_state/DirectCommandCoordinator.h` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/system_state/DirectCommandCoordinator.h#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/system_state/DirectCommandCoordinator.h)
<a id="ref-0240"></a>

- **[240]** `mmhand:ESP32-P4/ServoBoardMain/src/tasks/system_state/FeedbackReadyDiagnostics.cpp` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/system_state/FeedbackReadyDiagnostics.cpp#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/system_state/FeedbackReadyDiagnostics.cpp)
  - 相关源码行：[L46–L46 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/system_state/FeedbackReadyDiagnostics.cpp#L46-L46) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/system_state/FeedbackReadyDiagnostics.cpp)
<a id="ref-0241"></a>

- **[241]** `mmhand:ESP32-P4/ServoBoardMain/src/tasks/system_state/FeedbackReadyDiagnostics.h` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/system_state/FeedbackReadyDiagnostics.h#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/system_state/FeedbackReadyDiagnostics.h)
<a id="ref-0242"></a>

- **[242]** `mmhand:ESP32-P4/ServoBoardMain/src/tasks/system_state/HostLeaseSupervisor.cpp` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/system_state/HostLeaseSupervisor.cpp#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/system_state/HostLeaseSupervisor.cpp)
  - 相关源码行：[L30–L30 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/system_state/HostLeaseSupervisor.cpp#L30-L30) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/system_state/HostLeaseSupervisor.cpp), [L55–L55 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/system_state/HostLeaseSupervisor.cpp#L55-L55) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/system_state/HostLeaseSupervisor.cpp), [L58–L58 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/system_state/HostLeaseSupervisor.cpp#L58-L58) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/system_state/HostLeaseSupervisor.cpp), [L60–L60 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/system_state/HostLeaseSupervisor.cpp#L60-L60) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/system_state/HostLeaseSupervisor.cpp), [L310–L310 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/system_state/HostLeaseSupervisor.cpp#L310-L310) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/system_state/HostLeaseSupervisor.cpp)
<a id="ref-0243"></a>

- **[243]** `mmhand:ESP32-P4/ServoBoardMain/src/tasks/system_state/HostLeaseSupervisor.h` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/system_state/HostLeaseSupervisor.h#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/system_state/HostLeaseSupervisor.h)
<a id="ref-0244"></a>

- **[244]** `mmhand:ESP32-P4/ServoBoardMain/src/tasks/system_state/RopeInitController.cpp` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/system_state/RopeInitController.cpp#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/system_state/RopeInitController.cpp)
  - 相关源码行：[L123–L123 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/system_state/RopeInitController.cpp#L123-L123) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/system_state/RopeInitController.cpp), [L146–L146 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/system_state/RopeInitController.cpp#L146-L146) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/system_state/RopeInitController.cpp), [L192–L192 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/system_state/RopeInitController.cpp#L192-L192) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/system_state/RopeInitController.cpp)
<a id="ref-0245"></a>

- **[245]** `mmhand:ESP32-P4/ServoBoardMain/src/tasks/system_state/RopeInitController.h` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/system_state/RopeInitController.h#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/system_state/RopeInitController.h)
<a id="ref-0246"></a>

- **[246]** `mmhand:ESP32-P4/ServoBoardMain/src/tasks/system_state/SystemEventDispatcher.cpp` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/system_state/SystemEventDispatcher.cpp#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/system_state/SystemEventDispatcher.cpp)
  - 相关源码行：[L99–L99 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/system_state/SystemEventDispatcher.cpp#L99-L99) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/system_state/SystemEventDispatcher.cpp), [L100–L100 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/system_state/SystemEventDispatcher.cpp#L100-L100) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/system_state/SystemEventDispatcher.cpp), [L162–L162 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/system_state/SystemEventDispatcher.cpp#L162-L162) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/system_state/SystemEventDispatcher.cpp), [L269–L269 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/system_state/SystemEventDispatcher.cpp#L269-L269) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/system_state/SystemEventDispatcher.cpp), [L449–L449 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/system_state/SystemEventDispatcher.cpp#L449-L449) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/system_state/SystemEventDispatcher.cpp), [L450–L450 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/system_state/SystemEventDispatcher.cpp#L450-L450) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/system_state/SystemEventDispatcher.cpp), [L453–L453 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/system_state/SystemEventDispatcher.cpp#L453-L453) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/system_state/SystemEventDispatcher.cpp), [L461–L461 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/system_state/SystemEventDispatcher.cpp#L461-L461) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/system_state/SystemEventDispatcher.cpp), [L468–L468 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/system_state/SystemEventDispatcher.cpp#L468-L468) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/system_state/SystemEventDispatcher.cpp), [L477–L477 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/system_state/SystemEventDispatcher.cpp#L477-L477) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/system_state/SystemEventDispatcher.cpp), [L496–L496 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/system_state/SystemEventDispatcher.cpp#L496-L496) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/system_state/SystemEventDispatcher.cpp), [L509–L509 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/system_state/SystemEventDispatcher.cpp#L509-L509) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/system_state/SystemEventDispatcher.cpp), [L513–L514 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/system_state/SystemEventDispatcher.cpp#L513-L514) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/system_state/SystemEventDispatcher.cpp), [L521–L521 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/system_state/SystemEventDispatcher.cpp#L521-L521) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/system_state/SystemEventDispatcher.cpp), [L524–L524 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/system_state/SystemEventDispatcher.cpp#L524-L524) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/system_state/SystemEventDispatcher.cpp), [L527–L527 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/system_state/SystemEventDispatcher.cpp#L527-L527) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/system_state/SystemEventDispatcher.cpp)
<a id="ref-0247"></a>

- **[247]** `mmhand:ESP32-P4/ServoBoardMain/src/tasks/system_state/SystemEventDispatcher.h` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/system_state/SystemEventDispatcher.h#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/system_state/SystemEventDispatcher.h)
<a id="ref-0248"></a>

- **[248]** `mmhand:ESP32-P4/ServoBoardMain/src/tasks/system_state/SystemStateCoordinator.cpp` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/system_state/SystemStateCoordinator.cpp#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/system_state/SystemStateCoordinator.cpp)
  - 相关源码行：[L94–L94 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/system_state/SystemStateCoordinator.cpp#L94-L94) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/system_state/SystemStateCoordinator.cpp)
<a id="ref-0249"></a>

- **[249]** `mmhand:ESP32-P4/ServoBoardMain/src/tasks/system_state/SystemStateCoordinator.h` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/system_state/SystemStateCoordinator.h#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/system_state/SystemStateCoordinator.h)
<a id="ref-0250"></a>

- **[250]** `mmhand:ESP32-P4/ServoBoardMain/src/tasks/system_state/SystemStateTask.cpp` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/system_state/SystemStateTask.cpp#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/system_state/SystemStateTask.cpp)
  - 相关源码行：[L1–L1 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/system_state/SystemStateTask.cpp#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/system_state/SystemStateTask.cpp), [L25–L25 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/system_state/SystemStateTask.cpp#L25-L25) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/system_state/SystemStateTask.cpp), [L26–L26 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/system_state/SystemStateTask.cpp#L26-L26) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/system_state/SystemStateTask.cpp), [L29–L29 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/system_state/SystemStateTask.cpp#L29-L29) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/system_state/SystemStateTask.cpp), [L30–L30 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/system_state/SystemStateTask.cpp#L30-L30) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/system_state/SystemStateTask.cpp), [L31–L31 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/system_state/SystemStateTask.cpp#L31-L31) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/system_state/SystemStateTask.cpp), [L40–L40 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/system_state/SystemStateTask.cpp#L40-L40) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/system_state/SystemStateTask.cpp), [L41–L41 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/system_state/SystemStateTask.cpp#L41-L41) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/system_state/SystemStateTask.cpp), [L42–L42 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/system_state/SystemStateTask.cpp#L42-L42) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/system_state/SystemStateTask.cpp), [L43–L43 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/system_state/SystemStateTask.cpp#L43-L43) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/system_state/SystemStateTask.cpp), [L52–L52 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/system_state/SystemStateTask.cpp#L52-L52) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/system_state/SystemStateTask.cpp), [L54–L54 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/system_state/SystemStateTask.cpp#L54-L54) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/system_state/SystemStateTask.cpp)
<a id="ref-0251"></a>

- **[251]** `mmhand:ESP32-P4/ServoBoardMain/src/tasks/system_state/SystemStateTask.h` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/tasks/system_state/SystemStateTask.h#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/tasks/system_state/SystemStateTask.h)
<a id="ref-0252"></a>

- **[252]** `mmhand:ESP32-P4/libraries/FTServo_Arduino/LICENSE` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/libraries/FTServo_Arduino/LICENSE#L1-L1) · [local](../../ESP32-P4/libraries/FTServo_Arduino/LICENSE)
<a id="ref-0253"></a>

- **[253]** `mmhand:ESP32-P4/libraries/FTServo_Arduino/README.md` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/libraries/FTServo_Arduino/README.md#L1-L1) · [local](../../ESP32-P4/libraries/FTServo_Arduino/README.md)
<a id="ref-0254"></a>

- **[254]** `mmhand:ESP32-P4/libraries/FTServo_Arduino/examples/HLSCL/Broadcast/Broadcast.ino` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/libraries/FTServo_Arduino/examples/HLSCL/Broadcast/Broadcast.ino#L1-L1) · [local](../../ESP32-P4/libraries/FTServo_Arduino/examples/HLSCL/Broadcast/Broadcast.ino)
<a id="ref-0255"></a>

- **[255]** `mmhand:ESP32-P4/libraries/FTServo_Arduino/examples/HLSCL/CalibrationOfs/CalibrationOfs.ino` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/libraries/FTServo_Arduino/examples/HLSCL/CalibrationOfs/CalibrationOfs.ino#L1-L1) · [local](../../ESP32-P4/libraries/FTServo_Arduino/examples/HLSCL/CalibrationOfs/CalibrationOfs.ino)
<a id="ref-0256"></a>

- **[256]** `mmhand:ESP32-P4/libraries/FTServo_Arduino/examples/HLSCL/FeedBack/FeedBack.ino` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/libraries/FTServo_Arduino/examples/HLSCL/FeedBack/FeedBack.ino#L1-L1) · [local](../../ESP32-P4/libraries/FTServo_Arduino/examples/HLSCL/FeedBack/FeedBack.ino)
<a id="ref-0257"></a>

- **[257]** `mmhand:ESP32-P4/libraries/FTServo_Arduino/examples/HLSCL/Ping/Ping.ino` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/libraries/FTServo_Arduino/examples/HLSCL/Ping/Ping.ino#L1-L1) · [local](../../ESP32-P4/libraries/FTServo_Arduino/examples/HLSCL/Ping/Ping.ino)
<a id="ref-0258"></a>

- **[258]** `mmhand:ESP32-P4/libraries/FTServo_Arduino/examples/HLSCL/ProgramEprom/ProgramEprom.ino` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/libraries/FTServo_Arduino/examples/HLSCL/ProgramEprom/ProgramEprom.ino#L1-L1) · [local](../../ESP32-P4/libraries/FTServo_Arduino/examples/HLSCL/ProgramEprom/ProgramEprom.ino)
<a id="ref-0259"></a>

- **[259]** `mmhand:ESP32-P4/libraries/FTServo_Arduino/examples/HLSCL/RegWritePos/RegWritePos.ino` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/libraries/FTServo_Arduino/examples/HLSCL/RegWritePos/RegWritePos.ino#L1-L1) · [local](../../ESP32-P4/libraries/FTServo_Arduino/examples/HLSCL/RegWritePos/RegWritePos.ino)
<a id="ref-0260"></a>

- **[260]** `mmhand:ESP32-P4/libraries/FTServo_Arduino/examples/HLSCL/SyncRead/SyncRead.ino` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/libraries/FTServo_Arduino/examples/HLSCL/SyncRead/SyncRead.ino#L1-L1) · [local](../../ESP32-P4/libraries/FTServo_Arduino/examples/HLSCL/SyncRead/SyncRead.ino)
<a id="ref-0261"></a>

- **[261]** `mmhand:ESP32-P4/libraries/FTServo_Arduino/examples/HLSCL/SyncWritePos/SyncWritePos.ino` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/libraries/FTServo_Arduino/examples/HLSCL/SyncWritePos/SyncWritePos.ino#L1-L1) · [local](../../ESP32-P4/libraries/FTServo_Arduino/examples/HLSCL/SyncWritePos/SyncWritePos.ino)
<a id="ref-0262"></a>

- **[262]** `mmhand:ESP32-P4/libraries/FTServo_Arduino/examples/HLSCL/SyncWriteSpe/SyncWriteSpe.ino` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/libraries/FTServo_Arduino/examples/HLSCL/SyncWriteSpe/SyncWriteSpe.ino#L1-L1) · [local](../../ESP32-P4/libraries/FTServo_Arduino/examples/HLSCL/SyncWriteSpe/SyncWriteSpe.ino)
<a id="ref-0263"></a>

- **[263]** `mmhand:ESP32-P4/libraries/FTServo_Arduino/examples/HLSCL/WriteEle/WriteEle.ino` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/libraries/FTServo_Arduino/examples/HLSCL/WriteEle/WriteEle.ino#L1-L1) · [local](../../ESP32-P4/libraries/FTServo_Arduino/examples/HLSCL/WriteEle/WriteEle.ino)
<a id="ref-0264"></a>

- **[264]** `mmhand:ESP32-P4/libraries/FTServo_Arduino/examples/HLSCL/WritePos/WritePos.ino` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/libraries/FTServo_Arduino/examples/HLSCL/WritePos/WritePos.ino#L1-L1) · [local](../../ESP32-P4/libraries/FTServo_Arduino/examples/HLSCL/WritePos/WritePos.ino)
<a id="ref-0265"></a>

- **[265]** `mmhand:ESP32-P4/libraries/FTServo_Arduino/examples/HLSCL/WriteSpe/WriteSpe.ino` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/libraries/FTServo_Arduino/examples/HLSCL/WriteSpe/WriteSpe.ino#L1-L1) · [local](../../ESP32-P4/libraries/FTServo_Arduino/examples/HLSCL/WriteSpe/WriteSpe.ino)
<a id="ref-0266"></a>

- **[266]** `mmhand:ESP32-P4/libraries/FTServo_Arduino/examples/SCSCL/Broadcast/Broadcast.ino` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/libraries/FTServo_Arduino/examples/SCSCL/Broadcast/Broadcast.ino#L1-L1) · [local](../../ESP32-P4/libraries/FTServo_Arduino/examples/SCSCL/Broadcast/Broadcast.ino)
<a id="ref-0267"></a>

- **[267]** `mmhand:ESP32-P4/libraries/FTServo_Arduino/examples/SCSCL/FeedBack/FeedBack.ino` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/libraries/FTServo_Arduino/examples/SCSCL/FeedBack/FeedBack.ino#L1-L1) · [local](../../ESP32-P4/libraries/FTServo_Arduino/examples/SCSCL/FeedBack/FeedBack.ino)
<a id="ref-0268"></a>

- **[268]** `mmhand:ESP32-P4/libraries/FTServo_Arduino/examples/SCSCL/Ping/Ping.ino` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/libraries/FTServo_Arduino/examples/SCSCL/Ping/Ping.ino#L1-L1) · [local](../../ESP32-P4/libraries/FTServo_Arduino/examples/SCSCL/Ping/Ping.ino)
<a id="ref-0269"></a>

- **[269]** `mmhand:ESP32-P4/libraries/FTServo_Arduino/examples/SCSCL/ProgramEprom/ProgramEprom.ino` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/libraries/FTServo_Arduino/examples/SCSCL/ProgramEprom/ProgramEprom.ino#L1-L1) · [local](../../ESP32-P4/libraries/FTServo_Arduino/examples/SCSCL/ProgramEprom/ProgramEprom.ino)
<a id="ref-0270"></a>

- **[270]** `mmhand:ESP32-P4/libraries/FTServo_Arduino/examples/SCSCL/RegWritePos/RegWritePos.ino` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/libraries/FTServo_Arduino/examples/SCSCL/RegWritePos/RegWritePos.ino#L1-L1) · [local](../../ESP32-P4/libraries/FTServo_Arduino/examples/SCSCL/RegWritePos/RegWritePos.ino)
<a id="ref-0271"></a>

- **[271]** `mmhand:ESP32-P4/libraries/FTServo_Arduino/examples/SCSCL/SyncWritePos/SyncWritePos.ino` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/libraries/FTServo_Arduino/examples/SCSCL/SyncWritePos/SyncWritePos.ino#L1-L1) · [local](../../ESP32-P4/libraries/FTServo_Arduino/examples/SCSCL/SyncWritePos/SyncWritePos.ino)
<a id="ref-0272"></a>

- **[272]** `mmhand:ESP32-P4/libraries/FTServo_Arduino/examples/SCSCL/WritePWM/WritePWM.ino` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/libraries/FTServo_Arduino/examples/SCSCL/WritePWM/WritePWM.ino#L1-L1) · [local](../../ESP32-P4/libraries/FTServo_Arduino/examples/SCSCL/WritePWM/WritePWM.ino)
<a id="ref-0273"></a>

- **[273]** `mmhand:ESP32-P4/libraries/FTServo_Arduino/examples/SCSCL/WritePos/WritePos.ino` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/libraries/FTServo_Arduino/examples/SCSCL/WritePos/WritePos.ino#L1-L1) · [local](../../ESP32-P4/libraries/FTServo_Arduino/examples/SCSCL/WritePos/WritePos.ino)
<a id="ref-0274"></a>

- **[274]** `mmhand:ESP32-P4/libraries/FTServo_Arduino/examples/SMS_STS/Broadcast/Broadcast.ino` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/libraries/FTServo_Arduino/examples/SMS_STS/Broadcast/Broadcast.ino#L1-L1) · [local](../../ESP32-P4/libraries/FTServo_Arduino/examples/SMS_STS/Broadcast/Broadcast.ino)
<a id="ref-0275"></a>

- **[275]** `mmhand:ESP32-P4/libraries/FTServo_Arduino/examples/SMS_STS/CalibrationOfs/CalibrationOfs.ino` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/libraries/FTServo_Arduino/examples/SMS_STS/CalibrationOfs/CalibrationOfs.ino#L1-L1) · [local](../../ESP32-P4/libraries/FTServo_Arduino/examples/SMS_STS/CalibrationOfs/CalibrationOfs.ino)
<a id="ref-0276"></a>

- **[276]** `mmhand:ESP32-P4/libraries/FTServo_Arduino/examples/SMS_STS/FeedBack/FeedBack.ino` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/libraries/FTServo_Arduino/examples/SMS_STS/FeedBack/FeedBack.ino#L1-L1) · [local](../../ESP32-P4/libraries/FTServo_Arduino/examples/SMS_STS/FeedBack/FeedBack.ino)
<a id="ref-0277"></a>

- **[277]** `mmhand:ESP32-P4/libraries/FTServo_Arduino/examples/SMS_STS/Ping/Ping.ino` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/libraries/FTServo_Arduino/examples/SMS_STS/Ping/Ping.ino#L1-L1) · [local](../../ESP32-P4/libraries/FTServo_Arduino/examples/SMS_STS/Ping/Ping.ino)
<a id="ref-0278"></a>

- **[278]** `mmhand:ESP32-P4/libraries/FTServo_Arduino/examples/SMS_STS/ProgramEprom/ProgramEprom.ino` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/libraries/FTServo_Arduino/examples/SMS_STS/ProgramEprom/ProgramEprom.ino#L1-L1) · [local](../../ESP32-P4/libraries/FTServo_Arduino/examples/SMS_STS/ProgramEprom/ProgramEprom.ino)
<a id="ref-0279"></a>

- **[279]** `mmhand:ESP32-P4/libraries/FTServo_Arduino/examples/SMS_STS/RegWritePos/RegWritePos.ino` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/libraries/FTServo_Arduino/examples/SMS_STS/RegWritePos/RegWritePos.ino#L1-L1) · [local](../../ESP32-P4/libraries/FTServo_Arduino/examples/SMS_STS/RegWritePos/RegWritePos.ino)
<a id="ref-0280"></a>

- **[280]** `mmhand:ESP32-P4/libraries/FTServo_Arduino/examples/SMS_STS/SyncRead/SyncRead.ino` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/libraries/FTServo_Arduino/examples/SMS_STS/SyncRead/SyncRead.ino#L1-L1) · [local](../../ESP32-P4/libraries/FTServo_Arduino/examples/SMS_STS/SyncRead/SyncRead.ino)
<a id="ref-0281"></a>

- **[281]** `mmhand:ESP32-P4/libraries/FTServo_Arduino/examples/SMS_STS/SyncWritePos/SyncWritePos.ino` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/libraries/FTServo_Arduino/examples/SMS_STS/SyncWritePos/SyncWritePos.ino#L1-L1) · [local](../../ESP32-P4/libraries/FTServo_Arduino/examples/SMS_STS/SyncWritePos/SyncWritePos.ino)
<a id="ref-0282"></a>

- **[282]** `mmhand:ESP32-P4/libraries/FTServo_Arduino/examples/SMS_STS/SyncWriteSpe/SyncWriteSpe.ino` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/libraries/FTServo_Arduino/examples/SMS_STS/SyncWriteSpe/SyncWriteSpe.ino#L1-L1) · [local](../../ESP32-P4/libraries/FTServo_Arduino/examples/SMS_STS/SyncWriteSpe/SyncWriteSpe.ino)
<a id="ref-0283"></a>

- **[283]** `mmhand:ESP32-P4/libraries/FTServo_Arduino/examples/SMS_STS/WritePos/WritePos.ino` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/libraries/FTServo_Arduino/examples/SMS_STS/WritePos/WritePos.ino#L1-L1) · [local](../../ESP32-P4/libraries/FTServo_Arduino/examples/SMS_STS/WritePos/WritePos.ino)
<a id="ref-0284"></a>

- **[284]** `mmhand:ESP32-P4/libraries/FTServo_Arduino/examples/SMS_STS/WriteSpe/WriteSpe.ino` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/libraries/FTServo_Arduino/examples/SMS_STS/WriteSpe/WriteSpe.ino#L1-L1) · [local](../../ESP32-P4/libraries/FTServo_Arduino/examples/SMS_STS/WriteSpe/WriteSpe.ino)
<a id="ref-0285"></a>

- **[285]** `mmhand:ESP32-P4/libraries/FTServo_Arduino/library.properties` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/libraries/FTServo_Arduino/library.properties#L1-L1) · [local](../../ESP32-P4/libraries/FTServo_Arduino/library.properties)
<a id="ref-0286"></a>

- **[286]** `mmhand:ESP32-P4/libraries/FTServo_Arduino/src/HLSCL.cpp` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/libraries/FTServo_Arduino/src/HLSCL.cpp#L1-L1) · [local](../../ESP32-P4/libraries/FTServo_Arduino/src/HLSCL.cpp)
<a id="ref-0287"></a>

- **[287]** `mmhand:ESP32-P4/libraries/FTServo_Arduino/src/HLSCL.h` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/libraries/FTServo_Arduino/src/HLSCL.h#L1-L1) · [local](../../ESP32-P4/libraries/FTServo_Arduino/src/HLSCL.h)
<a id="ref-0288"></a>

- **[288]** `mmhand:ESP32-P4/libraries/FTServo_Arduino/src/INST.h` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/libraries/FTServo_Arduino/src/INST.h#L1-L1) · [local](../../ESP32-P4/libraries/FTServo_Arduino/src/INST.h)
<a id="ref-0289"></a>

- **[289]** `mmhand:ESP32-P4/libraries/FTServo_Arduino/src/SCS.cpp` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/libraries/FTServo_Arduino/src/SCS.cpp#L1-L1) · [local](../../ESP32-P4/libraries/FTServo_Arduino/src/SCS.cpp)
<a id="ref-0290"></a>

- **[290]** `mmhand:ESP32-P4/libraries/FTServo_Arduino/src/SCS.h` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/libraries/FTServo_Arduino/src/SCS.h#L1-L1) · [local](../../ESP32-P4/libraries/FTServo_Arduino/src/SCS.h)
<a id="ref-0291"></a>

- **[291]** `mmhand:ESP32-P4/libraries/FTServo_Arduino/src/SCSCL.cpp` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/libraries/FTServo_Arduino/src/SCSCL.cpp#L1-L1) · [local](../../ESP32-P4/libraries/FTServo_Arduino/src/SCSCL.cpp)
<a id="ref-0292"></a>

- **[292]** `mmhand:ESP32-P4/libraries/FTServo_Arduino/src/SCSCL.h` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/libraries/FTServo_Arduino/src/SCSCL.h#L1-L1) · [local](../../ESP32-P4/libraries/FTServo_Arduino/src/SCSCL.h)
<a id="ref-0293"></a>

- **[293]** `mmhand:ESP32-P4/libraries/FTServo_Arduino/src/SCSerial.cpp` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/libraries/FTServo_Arduino/src/SCSerial.cpp#L1-L1) · [local](../../ESP32-P4/libraries/FTServo_Arduino/src/SCSerial.cpp)
<a id="ref-0294"></a>

- **[294]** `mmhand:ESP32-P4/libraries/FTServo_Arduino/src/SCSerial.h` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/libraries/FTServo_Arduino/src/SCSerial.h#L1-L1) · [local](../../ESP32-P4/libraries/FTServo_Arduino/src/SCSerial.h)
<a id="ref-0295"></a>

- **[295]** `mmhand:ESP32-P4/libraries/FTServo_Arduino/src/SCServo.h` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/libraries/FTServo_Arduino/src/SCServo.h#L1-L1) · [local](../../ESP32-P4/libraries/FTServo_Arduino/src/SCServo.h)
<a id="ref-0296"></a>

- **[296]** `mmhand:ESP32-P4/libraries/FTServo_Arduino/src/SMS_STS.cpp` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/libraries/FTServo_Arduino/src/SMS_STS.cpp#L1-L1) · [local](../../ESP32-P4/libraries/FTServo_Arduino/src/SMS_STS.cpp)
<a id="ref-0297"></a>

- **[297]** `mmhand:ESP32-P4/libraries/FTServo_Arduino/src/SMS_STS.h` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/libraries/FTServo_Arduino/src/SMS_STS.h#L1-L1) · [local](../../ESP32-P4/libraries/FTServo_Arduino/src/SMS_STS.h)
<a id="ref-0298"></a>

- **[298]** `mmhand:ESP32-P4/tools/strip_utf8_bom.py` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/tools/strip_utf8_bom.py#L1-L1) · [local](../../ESP32-P4/tools/strip_utf8_bom.py)
<a id="ref-0299"></a>

- **[299]** `mmhand:ESP32-P4/tools/watch_strip_utf8_bom.py` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/tools/watch_strip_utf8_bom.py#L1-L1) · [local](../../ESP32-P4/tools/watch_strip_utf8_bom.py)
<a id="ref-0300"></a>

- **[300]** `mmhand:ESP32-S3/ESP32-S3架构与状态机说明.md` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-S3/ESP32-S3%E6%9E%B6%E6%9E%84%E4%B8%8E%E7%8A%B6%E6%80%81%E6%9C%BA%E8%AF%B4%E6%98%8E.md#L1-L1) · [local](../../ESP32-S3/ESP32-S3%E6%9E%B6%E6%9E%84%E4%B8%8E%E7%8A%B6%E6%80%81%E6%9C%BA%E8%AF%B4%E6%98%8E.md)
<a id="ref-0301"></a>

- **[301]** `mmhand:ESP32-S3/System/.theia/launch.json` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-S3/System/.theia/launch.json#L1-L1) · [local](../../ESP32-S3/System/.theia/launch.json)
<a id="ref-0302"></a>

- **[302]** `mmhand:ESP32-S3/System/System.ino` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-S3/System/System.ino#L1-L1) · [local](../../ESP32-S3/System/System.ino)
<a id="ref-0303"></a>

- **[303]** `mmhand:ESP32-S3/System/src/config/Config.h` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-S3/System/src/config/Config.h#L1-L1) · [local](../../ESP32-S3/System/src/config/Config.h)
  - 相关源码行：[L146–L146 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-S3/System/src/config/Config.h#L146-L146) · [local](../../ESP32-S3/System/src/config/Config.h), [L147–L147 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-S3/System/src/config/Config.h#L147-L147) · [local](../../ESP32-S3/System/src/config/Config.h), [L148–L148 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-S3/System/src/config/Config.h#L148-L148) · [local](../../ESP32-S3/System/src/config/Config.h), [L149–L149 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-S3/System/src/config/Config.h#L149-L149) · [local](../../ESP32-S3/System/src/config/Config.h), [L150–L150 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-S3/System/src/config/Config.h#L150-L150) · [local](../../ESP32-S3/System/src/config/Config.h), [L151–L151 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-S3/System/src/config/Config.h#L151-L151) · [local](../../ESP32-S3/System/src/config/Config.h)
<a id="ref-0304"></a>

- **[304]** `mmhand:ESP32-S3/System/src/hal/HalEncoders.cpp` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-S3/System/src/hal/HalEncoders.cpp#L1-L1) · [local](../../ESP32-S3/System/src/hal/HalEncoders.cpp)
<a id="ref-0305"></a>

- **[305]** `mmhand:ESP32-S3/System/src/hal/HalEncoders.h` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-S3/System/src/hal/HalEncoders.h#L1-L1) · [local](../../ESP32-S3/System/src/hal/HalEncoders.h)
<a id="ref-0306"></a>

- **[306]** `mmhand:ESP32-S3/System/src/hal/HalSensorSpi.cpp` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-S3/System/src/hal/HalSensorSpi.cpp#L1-L1) · [local](../../ESP32-S3/System/src/hal/HalSensorSpi.cpp)
<a id="ref-0307"></a>

- **[307]** `mmhand:ESP32-S3/System/src/hal/HalSensorSpi.h` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-S3/System/src/hal/HalSensorSpi.h#L1-L1) · [local](../../ESP32-S3/System/src/hal/HalSensorSpi.h)
<a id="ref-0308"></a>

- **[308]** `mmhand:ESP32-S3/System/src/hal/HalTWAI.cpp` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-S3/System/src/hal/HalTWAI.cpp#L1-L1) · [local](../../ESP32-S3/System/src/hal/HalTWAI.cpp)
<a id="ref-0309"></a>

- **[309]** `mmhand:ESP32-S3/System/src/hal/HalTWAI.h` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-S3/System/src/hal/HalTWAI.h#L1-L1) · [local](../../ESP32-S3/System/src/hal/HalTWAI.h)
<a id="ref-0310"></a>

- **[310]** `mmhand:ESP32-S3/System/src/hal/HalTactile.cpp` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-S3/System/src/hal/HalTactile.cpp#L1-L1) · [local](../../ESP32-S3/System/src/hal/HalTactile.cpp)
<a id="ref-0311"></a>

- **[311]** `mmhand:ESP32-S3/System/src/hal/HalTactile.h` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-S3/System/src/hal/HalTactile.h#L1-L1) · [local](../../ESP32-S3/System/src/hal/HalTactile.h)
<a id="ref-0312"></a>

- **[312]** `mmhand:ESP32-S3/System/src/system/FirmwareLog.h` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-S3/System/src/system/FirmwareLog.h#L1-L1) · [local](../../ESP32-S3/System/src/system/FirmwareLog.h)
<a id="ref-0313"></a>

- **[313]** `mmhand:ESP32-S3/System/src/system/SystemTasks.cpp` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-S3/System/src/system/SystemTasks.cpp#L1-L1) · [local](../../ESP32-S3/System/src/system/SystemTasks.cpp)
  - 相关源码行：[L15–L15 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-S3/System/src/system/SystemTasks.cpp#L15-L15) · [local](../../ESP32-S3/System/src/system/SystemTasks.cpp), [L17–L17 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-S3/System/src/system/SystemTasks.cpp#L17-L17) · [local](../../ESP32-S3/System/src/system/SystemTasks.cpp), [L29–L29 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-S3/System/src/system/SystemTasks.cpp#L29-L29) · [local](../../ESP32-S3/System/src/system/SystemTasks.cpp), [L35–L35 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-S3/System/src/system/SystemTasks.cpp#L35-L35) · [local](../../ESP32-S3/System/src/system/SystemTasks.cpp), [L44–L44 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-S3/System/src/system/SystemTasks.cpp#L44-L44) · [local](../../ESP32-S3/System/src/system/SystemTasks.cpp)
<a id="ref-0314"></a>

- **[314]** `mmhand:ESP32-S3/System/src/system/SystemTasks.h` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-S3/System/src/system/SystemTasks.h#L1-L1) · [local](../../ESP32-S3/System/src/system/SystemTasks.h)
<a id="ref-0315"></a>

- **[315]** `mmhand:ESP32-S3/System/src/tasks/CanTask.cpp` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-S3/System/src/tasks/CanTask.cpp#L1-L1) · [local](../../ESP32-S3/System/src/tasks/CanTask.cpp)
  - 相关源码行：[L1–L1 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-S3/System/src/tasks/CanTask.cpp#L1-L1) · [local](../../ESP32-S3/System/src/tasks/CanTask.cpp), [L33–L33 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-S3/System/src/tasks/CanTask.cpp#L33-L33) · [local](../../ESP32-S3/System/src/tasks/CanTask.cpp), [L42–L42 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-S3/System/src/tasks/CanTask.cpp#L42-L42) · [local](../../ESP32-S3/System/src/tasks/CanTask.cpp), [L72–L72 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-S3/System/src/tasks/CanTask.cpp#L72-L72) · [local](../../ESP32-S3/System/src/tasks/CanTask.cpp), [L92–L92 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-S3/System/src/tasks/CanTask.cpp#L92-L92) · [local](../../ESP32-S3/System/src/tasks/CanTask.cpp)
<a id="ref-0316"></a>

- **[316]** `mmhand:ESP32-S3/System/src/tasks/CanTask.h` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-S3/System/src/tasks/CanTask.h#L1-L1) · [local](../../ESP32-S3/System/src/tasks/CanTask.h)
<a id="ref-0317"></a>

- **[317]** `mmhand:ESP32-S3/System/src/tasks/EncodersTask.cpp` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-S3/System/src/tasks/EncodersTask.cpp#L1-L1) · [local](../../ESP32-S3/System/src/tasks/EncodersTask.cpp)
  - 相关源码行：[L1–L1 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-S3/System/src/tasks/EncodersTask.cpp#L1-L1) · [local](../../ESP32-S3/System/src/tasks/EncodersTask.cpp), [L29–L29 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-S3/System/src/tasks/EncodersTask.cpp#L29-L29) · [local](../../ESP32-S3/System/src/tasks/EncodersTask.cpp), [L76–L76 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-S3/System/src/tasks/EncodersTask.cpp#L76-L76) · [local](../../ESP32-S3/System/src/tasks/EncodersTask.cpp)
<a id="ref-0318"></a>

- **[318]** `mmhand:ESP32-S3/System/src/tasks/EncodersTask.h` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-S3/System/src/tasks/EncodersTask.h#L1-L1) · [local](../../ESP32-S3/System/src/tasks/EncodersTask.h)
<a id="ref-0319"></a>

- **[319]** `mmhand:ESP32-S3/System/src/tasks/TactileTask.cpp` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-S3/System/src/tasks/TactileTask.cpp#L1-L1) · [local](../../ESP32-S3/System/src/tasks/TactileTask.cpp)
  - 相关源码行：[L1–L1 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-S3/System/src/tasks/TactileTask.cpp#L1-L1) · [local](../../ESP32-S3/System/src/tasks/TactileTask.cpp), [L257–L257 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-S3/System/src/tasks/TactileTask.cpp#L257-L257) · [local](../../ESP32-S3/System/src/tasks/TactileTask.cpp), [L270–L270 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-S3/System/src/tasks/TactileTask.cpp#L270-L270) · [local](../../ESP32-S3/System/src/tasks/TactileTask.cpp), [L277–L277 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-S3/System/src/tasks/TactileTask.cpp#L277-L277) · [local](../../ESP32-S3/System/src/tasks/TactileTask.cpp), [L335–L335 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-S3/System/src/tasks/TactileTask.cpp#L335-L335) · [local](../../ESP32-S3/System/src/tasks/TactileTask.cpp)
<a id="ref-0320"></a>

- **[320]** `mmhand:ESP32-S3/System/src/tasks/TactileTask.h` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-S3/System/src/tasks/TactileTask.h#L1-L1) · [local](../../ESP32-S3/System/src/tasks/TactileTask.h)
<a id="ref-0321"></a>

- **[321]** `mmhand:ESP32-S3/readme.md` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-S3/readme.md#L1-L1) · [local](../../ESP32-S3/readme.md)
<a id="ref-0322"></a>

- **[322]** `mmhand:ESP32-S3/待确认无用逻辑清单.md` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-S3/%E5%BE%85%E7%A1%AE%E8%AE%A4%E6%97%A0%E7%94%A8%E9%80%BB%E8%BE%91%E6%B8%85%E5%8D%95.md#L1-L1) · [local](../../ESP32-S3/%E5%BE%85%E7%A1%AE%E8%AE%A4%E6%97%A0%E7%94%A8%E9%80%BB%E8%BE%91%E6%B8%85%E5%8D%95.md)
<a id="ref-0323"></a>

- **[323]** `mmhand:README.md` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/README.md#L1-L1) · [local](../../README.md)
<a id="ref-0324"></a>

- **[324]** `mmhand:desktop/README.md` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/README.md#L1-L1) · [local](../../desktop/README.md)
<a id="ref-0325"></a>

- **[325]** `mmhand:desktop/__init__.py` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/__init__.py#L1-L1) · [local](../../desktop/__init__.py)
<a id="ref-0326"></a>

- **[326]** `mmhand:desktop/__main__.py` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/__main__.py#L1-L1) · [local](../../desktop/__main__.py)
<a id="ref-0327"></a>

- **[327]** `mmhand:desktop/app/__init__.py` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/app/__init__.py#L1-L1) · [local](../../desktop/app/__init__.py)
<a id="ref-0328"></a>

- **[328]** `mmhand:desktop/app/cli_monitor.py` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/app/cli_monitor.py#L1-L1) · [local](../../desktop/app/cli_monitor.py)
<a id="ref-0329"></a>

- **[329]** `mmhand:desktop/app/entry.py` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/app/entry.py#L1-L1) · [local](../../desktop/app/entry.py)
  - 相关源码行：[L24–L24 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/app/entry.py#L24-L24) · [local](../../desktop/app/entry.py), [L105–L105 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/app/entry.py#L105-L105) · [local](../../desktop/app/entry.py)
<a id="ref-0330"></a>

- **[330]** `mmhand:desktop/app/ui_runner.py` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/app/ui_runner.py#L1-L1) · [local](../../desktop/app/ui_runner.py)
<a id="ref-0331"></a>

- **[331]** `mmhand:desktop/core/__init__.py` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/core/__init__.py#L1-L1) · [local](../../desktop/core/__init__.py)
<a id="ref-0332"></a>

- **[332]** `mmhand:desktop/core/angle_mapping.py` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/core/angle_mapping.py#L1-L1) · [local](../../desktop/core/angle_mapping.py)
<a id="ref-0333"></a>

- **[333]** `mmhand:desktop/core/auto_calib_logger.py` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/core/auto_calib_logger.py#L1-L1) · [local](../../desktop/core/auto_calib_logger.py)
<a id="ref-0334"></a>

- **[334]** `mmhand:desktop/core/calibration_diagnostics.py` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/core/calibration_diagnostics.py#L1-L1) · [local](../../desktop/core/calibration_diagnostics.py)
<a id="ref-0335"></a>

- **[335]** `mmhand:desktop/core/calibration_snapshot.py` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/core/calibration_snapshot.py#L1-L1) · [local](../../desktop/core/calibration_snapshot.py)
<a id="ref-0336"></a>

- **[336]** `mmhand:desktop/core/calibration_workflow.py` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/core/calibration_workflow.py#L1-L1) · [local](../../desktop/core/calibration_workflow.py)
<a id="ref-0337"></a>

- **[337]** `mmhand:desktop/core/comm_layer.py` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/core/comm_layer.py#L1-L1) · [local](../../desktop/core/comm_layer.py)
<a id="ref-0338"></a>

- **[338]** `mmhand:desktop/core/communication/__init__.py` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/core/communication/__init__.py#L1-L1) · [local](../../desktop/core/communication/__init__.py)
<a id="ref-0339"></a>

- **[339]** `mmhand:desktop/core/communication/ack_tracker.py` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/core/communication/ack_tracker.py#L1-L1) · [local](../../desktop/core/communication/ack_tracker.py)
<a id="ref-0340"></a>

- **[340]** `mmhand:desktop/core/communication/event_channel.py` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/core/communication/event_channel.py#L1-L1) · [local](../../desktop/core/communication/event_channel.py)
<a id="ref-0341"></a>

- **[341]** `mmhand:desktop/core/communication/frame_decoder.py` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/core/communication/frame_decoder.py#L1-L1) · [local](../../desktop/core/communication/frame_decoder.py)
<a id="ref-0342"></a>

- **[342]** `mmhand:desktop/core/communication/packet_router.py` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/core/communication/packet_router.py#L1-L1) · [local](../../desktop/core/communication/packet_router.py)
<a id="ref-0343"></a>

- **[343]** `mmhand:desktop/core/communication/serial_transport.py` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/core/communication/serial_transport.py#L1-L1) · [local](../../desktop/core/communication/serial_transport.py)
<a id="ref-0344"></a>

- **[344]** `mmhand:desktop/core/communication/state_reducer.py` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/core/communication/state_reducer.py#L1-L1) · [local](../../desktop/core/communication/state_reducer.py)
<a id="ref-0345"></a>

- **[345]** `mmhand:desktop/core/core_logic.py` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/core/core_logic.py#L1-L1) · [local](../../desktop/core/core_logic.py)
  - 相关源码行：[L249–L249 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/core/core_logic.py#L249-L249) · [local](../../desktop/core/core_logic.py), [L319–L319 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/core/core_logic.py#L319-L319) · [local](../../desktop/core/core_logic.py), [L486–L486 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/core/core_logic.py#L486-L486) · [local](../../desktop/core/core_logic.py), [L1216–L1216 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/core/core_logic.py#L1216-L1216) · [local](../../desktop/core/core_logic.py), [L2152–L2152 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/core/core_logic.py#L2152-L2152) · [local](../../desktop/core/core_logic.py)
<a id="ref-0346"></a>

- **[346]** `mmhand:desktop/core/data_models.py` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/core/data_models.py#L1-L1) · [local](../../desktop/core/data_models.py)
<a id="ref-0347"></a>

- **[347]** `mmhand:desktop/core/data_recording/__init__.py` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/core/data_recording/__init__.py#L1-L1) · [local](../../desktop/core/data_recording/__init__.py)
<a id="ref-0348"></a>

- **[348]** `mmhand:desktop/core/data_recording/csv_exporter.py` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/core/data_recording/csv_exporter.py#L1-L1) · [local](../../desktop/core/data_recording/csv_exporter.py)
<a id="ref-0349"></a>

- **[349]** `mmhand:desktop/core/data_recording/event_recorder.py` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/core/data_recording/event_recorder.py#L1-L1) · [local](../../desktop/core/data_recording/event_recorder.py)
<a id="ref-0350"></a>

- **[350]** `mmhand:desktop/core/data_recording/motion_sampler.py` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/core/data_recording/motion_sampler.py#L1-L1) · [local](../../desktop/core/data_recording/motion_sampler.py)
<a id="ref-0351"></a>

- **[351]** `mmhand:desktop/core/data_recording/packet_recorder.py` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/core/data_recording/packet_recorder.py#L1-L1) · [local](../../desktop/core/data_recording/packet_recorder.py)
<a id="ref-0352"></a>

- **[352]** `mmhand:desktop/core/data_recording/recorder.py` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/core/data_recording/recorder.py#L1-L1) · [local](../../desktop/core/data_recording/recorder.py)
<a id="ref-0353"></a>

- **[353]** `mmhand:desktop/core/data_recording/topic_schema.py` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/core/data_recording/topic_schema.py#L1-L1) · [local](../../desktop/core/data_recording/topic_schema.py)
<a id="ref-0354"></a>

- **[354]** `mmhand:desktop/core/device_state_store.py` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/core/device_state_store.py#L1-L1) · [local](../../desktop/core/device_state_store.py)
<a id="ref-0355"></a>

- **[355]** `mmhand:desktop/core/generated_upper_protocol.py` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/core/generated_upper_protocol.py#L1-L1) · [local](../../desktop/core/generated_upper_protocol.py)
<a id="ref-0356"></a>

- **[356]** `mmhand:desktop/core/hand_backend.py` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/core/hand_backend.py#L1-L1) · [local](../../desktop/core/hand_backend.py)
  - 相关源码行：[L93–L93 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/core/hand_backend.py#L93-L93) · [local](../../desktop/core/hand_backend.py), [L549–L549 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/core/hand_backend.py#L549-L549) · [local](../../desktop/core/hand_backend.py), [L551–L551 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/core/hand_backend.py#L551-L551) · [local](../../desktop/core/hand_backend.py), [L555–L555 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/core/hand_backend.py#L555-L555) · [local](../../desktop/core/hand_backend.py), [L559–L559 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/core/hand_backend.py#L559-L559) · [local](../../desktop/core/hand_backend.py), [L564–L564 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/core/hand_backend.py#L564-L564) · [local](../../desktop/core/hand_backend.py), [L626–L626 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/core/hand_backend.py#L626-L626) · [local](../../desktop/core/hand_backend.py), [L642–L642 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/core/hand_backend.py#L642-L642) · [local](../../desktop/core/hand_backend.py), [L644–L644 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/core/hand_backend.py#L644-L644) · [local](../../desktop/core/hand_backend.py), [L646–L646 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/core/hand_backend.py#L646-L646) · [local](../../desktop/core/hand_backend.py), [L650–L650 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/core/hand_backend.py#L650-L650) · [local](../../desktop/core/hand_backend.py), [L674–L674 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/core/hand_backend.py#L674-L674) · [local](../../desktop/core/hand_backend.py), [L676–L676 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/core/hand_backend.py#L676-L676) · [local](../../desktop/core/hand_backend.py), [L678–L678 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/core/hand_backend.py#L678-L678) · [local](../../desktop/core/hand_backend.py), [L795–L795 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/core/hand_backend.py#L795-L795) · [local](../../desktop/core/hand_backend.py), [L854–L854 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/core/hand_backend.py#L854-L854) · [local](../../desktop/core/hand_backend.py)
<a id="ref-0357"></a>

- **[357]** `mmhand:desktop/core/joint_target_adapter.py` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/core/joint_target_adapter.py#L1-L1) · [local](../../desktop/core/joint_target_adapter.py)
<a id="ref-0358"></a>

- **[358]** `mmhand:desktop/core/motor_mapping.py` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/core/motor_mapping.py#L1-L1) · [local](../../desktop/core/motor_mapping.py)
<a id="ref-0359"></a>

- **[359]** `mmhand:desktop/core/protocol.py` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/core/protocol.py#L1-L1) · [local](../../desktop/core/protocol.py)
  - 相关源码行：[L216–L216 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/core/protocol.py#L216-L216) · [local](../../desktop/core/protocol.py)
<a id="ref-0360"></a>

- **[360]** `mmhand:desktop/core/safety_rules.py` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/core/safety_rules.py#L1-L1) · [local](../../desktop/core/safety_rules.py)
<a id="ref-0361"></a>

- **[361]** `mmhand:desktop/core/session_log.py` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/core/session_log.py#L1-L1) · [local](../../desktop/core/session_log.py)
<a id="ref-0362"></a>

- **[362]** `mmhand:desktop/examples/__init__.py` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/examples/__init__.py#L1-L1) · [local](../../desktop/examples/__init__.py)
<a id="ref-0363"></a>

- **[363]** `mmhand:desktop/examples/ros2_joint_target_sender.py` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/examples/ros2_joint_target_sender.py#L1-L1) · [local](../../desktop/examples/ros2_joint_target_sender.py)
<a id="ref-0364"></a>

- **[364]** `mmhand:desktop/paths.py` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/paths.py#L1-L1) · [local](../../desktop/paths.py)
<a id="ref-0365"></a>

- **[365]** `mmhand:desktop/replay/README.md` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/replay/README.md#L1-L1) · [local](../../desktop/replay/README.md)
<a id="ref-0366"></a>

- **[366]** `mmhand:desktop/replay/__init__.py` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/replay/__init__.py#L1-L1) · [local](../../desktop/replay/__init__.py)
<a id="ref-0367"></a>

- **[367]** `mmhand:desktop/replay/replay_controller.py` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/replay/replay_controller.py#L1-L1) · [local](../../desktop/replay/replay_controller.py)
  - 相关源码行：[L53–L53 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/replay/replay_controller.py#L53-L53) · [local](../../desktop/replay/replay_controller.py)
<a id="ref-0368"></a>

- **[368]** `mmhand:desktop/replay/replay_csv_loader.py` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/replay/replay_csv_loader.py#L1-L1) · [local](../../desktop/replay/replay_csv_loader.py)
<a id="ref-0369"></a>

- **[369]** `mmhand:desktop/replay/replay_models.py` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/replay/replay_models.py#L1-L1) · [local](../../desktop/replay/replay_models.py)
<a id="ref-0370"></a>

- **[370]** `mmhand:desktop/replay/replay_service.py` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/replay/replay_service.py#L1-L1) · [local](../../desktop/replay/replay_service.py)
<a id="ref-0371"></a>

- **[371]** `mmhand:desktop/requirements.txt` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/requirements.txt#L1-L1) · [local](../../desktop/requirements.txt)
<a id="ref-0372"></a>

- **[372]** `mmhand:desktop/resources/config/calibration_bundle.mmcal.json` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/resources/config/calibration_bundle.mmcal.json#L1-L1) · [local](../../desktop/resources/config/calibration_bundle.mmcal.json)
<a id="ref-0373"></a>

- **[373]** `mmhand:desktop/resources/config/p4_safety_rules.json` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/resources/config/p4_safety_rules.json#L1-L1) · [local](../../desktop/resources/config/p4_safety_rules.json)
<a id="ref-0374"></a>

- **[374]** `mmhand:desktop/resources/config/product_fingerprint.json` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/resources/config/product_fingerprint.json#L1-L1) · [local](../../desktop/resources/config/product_fingerprint.json)
<a id="ref-0375"></a>

- **[375]** `mmhand:desktop/resources/config/robot_model_manifest.json` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/resources/config/robot_model_manifest.json#L1-L1) · [local](../../desktop/resources/config/robot_model_manifest.json)
<a id="ref-0376"></a>

- **[376]** `mmhand:desktop/resources/config/upper_protocol_schema.json` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/resources/config/upper_protocol_schema.json#L1-L1) · [local](../../desktop/resources/config/upper_protocol_schema.json)
  - 相关源码行：[L17–L17 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/resources/config/upper_protocol_schema.json#L17-L17) · [local](../../desktop/resources/config/upper_protocol_schema.json), [L18–L18 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/resources/config/upper_protocol_schema.json#L18-L18) · [local](../../desktop/resources/config/upper_protocol_schema.json), [L19–L19 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/resources/config/upper_protocol_schema.json#L19-L19) · [local](../../desktop/resources/config/upper_protocol_schema.json), [L20–L20 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/resources/config/upper_protocol_schema.json#L20-L20) · [local](../../desktop/resources/config/upper_protocol_schema.json), [L21–L21 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/resources/config/upper_protocol_schema.json#L21-L21) · [local](../../desktop/resources/config/upper_protocol_schema.json), [L22–L22 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/resources/config/upper_protocol_schema.json#L22-L22) · [local](../../desktop/resources/config/upper_protocol_schema.json), [L23–L23 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/resources/config/upper_protocol_schema.json#L23-L23) · [local](../../desktop/resources/config/upper_protocol_schema.json), [L24–L24 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/resources/config/upper_protocol_schema.json#L24-L24) · [local](../../desktop/resources/config/upper_protocol_schema.json), [L25–L25 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/resources/config/upper_protocol_schema.json#L25-L25) · [local](../../desktop/resources/config/upper_protocol_schema.json), [L26–L26 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/resources/config/upper_protocol_schema.json#L26-L26) · [local](../../desktop/resources/config/upper_protocol_schema.json), [L27–L27 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/resources/config/upper_protocol_schema.json#L27-L27) · [local](../../desktop/resources/config/upper_protocol_schema.json), [L28–L28 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/resources/config/upper_protocol_schema.json#L28-L28) · [local](../../desktop/resources/config/upper_protocol_schema.json), [L29–L29 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/resources/config/upper_protocol_schema.json#L29-L29) · [local](../../desktop/resources/config/upper_protocol_schema.json), [L30–L30 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/resources/config/upper_protocol_schema.json#L30-L30) · [local](../../desktop/resources/config/upper_protocol_schema.json), [L31–L31 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/resources/config/upper_protocol_schema.json#L31-L31) · [local](../../desktop/resources/config/upper_protocol_schema.json), [L32–L32 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/resources/config/upper_protocol_schema.json#L32-L32) · [local](../../desktop/resources/config/upper_protocol_schema.json)
<a id="ref-0377"></a>

- **[377]** `mmhand:desktop/resources/config/urdf_joint_mapping.json` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/resources/config/urdf_joint_mapping.json#L1-L1) · [local](../../desktop/resources/config/urdf_joint_mapping.json)
<a id="ref-0378"></a>

- **[378]** `mmhand:desktop/resources/hku_hand_v2_urdf/meshes/base_link.STL` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/resources/hku_hand_v2_urdf/meshes/base_link.STL#L1-L1) · [local](../../desktop/resources/hku_hand_v2_urdf/meshes/base_link.STL)
<a id="ref-0379"></a>

- **[379]** `mmhand:desktop/resources/hku_hand_v2_urdf/meshes/finger_1_distal_phalanx_1.stl` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/resources/hku_hand_v2_urdf/meshes/finger_1_distal_phalanx_1.stl#L1-L1) · [local](../../desktop/resources/hku_hand_v2_urdf/meshes/finger_1_distal_phalanx_1.stl)
<a id="ref-0380"></a>

- **[380]** `mmhand:desktop/resources/hku_hand_v2_urdf/meshes/finger_1_fingertip_1.stl` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/resources/hku_hand_v2_urdf/meshes/finger_1_fingertip_1.stl#L1-L1) · [local](../../desktop/resources/hku_hand_v2_urdf/meshes/finger_1_fingertip_1.stl)
<a id="ref-0381"></a>

- **[381]** `mmhand:desktop/resources/hku_hand_v2_urdf/meshes/finger_1_proximal_phalanx_1.stl` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/resources/hku_hand_v2_urdf/meshes/finger_1_proximal_phalanx_1.stl#L1-L1) · [local](../../desktop/resources/hku_hand_v2_urdf/meshes/finger_1_proximal_phalanx_1.stl)
<a id="ref-0382"></a>

- **[382]** `mmhand:desktop/resources/hku_hand_v2_urdf/meshes/finger_2_distal_phalanx_1.stl` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/resources/hku_hand_v2_urdf/meshes/finger_2_distal_phalanx_1.stl#L1-L1) · [local](../../desktop/resources/hku_hand_v2_urdf/meshes/finger_2_distal_phalanx_1.stl)
<a id="ref-0383"></a>

- **[383]** `mmhand:desktop/resources/hku_hand_v2_urdf/meshes/finger_2_fingertip_1.stl` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/resources/hku_hand_v2_urdf/meshes/finger_2_fingertip_1.stl#L1-L1) · [local](../../desktop/resources/hku_hand_v2_urdf/meshes/finger_2_fingertip_1.stl)
<a id="ref-0384"></a>

- **[384]** `mmhand:desktop/resources/hku_hand_v2_urdf/meshes/finger_2_proximal_phalanx_1.stl` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/resources/hku_hand_v2_urdf/meshes/finger_2_proximal_phalanx_1.stl#L1-L1) · [local](../../desktop/resources/hku_hand_v2_urdf/meshes/finger_2_proximal_phalanx_1.stl)
<a id="ref-0385"></a>

- **[385]** `mmhand:desktop/resources/hku_hand_v2_urdf/meshes/finger_3_distal_phalanx_1.stl` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/resources/hku_hand_v2_urdf/meshes/finger_3_distal_phalanx_1.stl#L1-L1) · [local](../../desktop/resources/hku_hand_v2_urdf/meshes/finger_3_distal_phalanx_1.stl)
<a id="ref-0386"></a>

- **[386]** `mmhand:desktop/resources/hku_hand_v2_urdf/meshes/finger_3_fingertip_1.stl` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/resources/hku_hand_v2_urdf/meshes/finger_3_fingertip_1.stl#L1-L1) · [local](../../desktop/resources/hku_hand_v2_urdf/meshes/finger_3_fingertip_1.stl)
<a id="ref-0387"></a>

- **[387]** `mmhand:desktop/resources/hku_hand_v2_urdf/meshes/finger_3_proximal_phalanx_1.stl` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/resources/hku_hand_v2_urdf/meshes/finger_3_proximal_phalanx_1.stl#L1-L1) · [local](../../desktop/resources/hku_hand_v2_urdf/meshes/finger_3_proximal_phalanx_1.stl)
<a id="ref-0388"></a>

- **[388]** `mmhand:desktop/resources/hku_hand_v2_urdf/meshes/finger_4_distal_phalanx_1.stl` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/resources/hku_hand_v2_urdf/meshes/finger_4_distal_phalanx_1.stl#L1-L1) · [local](../../desktop/resources/hku_hand_v2_urdf/meshes/finger_4_distal_phalanx_1.stl)
<a id="ref-0389"></a>

- **[389]** `mmhand:desktop/resources/hku_hand_v2_urdf/meshes/finger_4_fingertip_1.stl` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/resources/hku_hand_v2_urdf/meshes/finger_4_fingertip_1.stl#L1-L1) · [local](../../desktop/resources/hku_hand_v2_urdf/meshes/finger_4_fingertip_1.stl)
<a id="ref-0390"></a>

- **[390]** `mmhand:desktop/resources/hku_hand_v2_urdf/meshes/finger_4_proximal_phalanx_1.stl` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/resources/hku_hand_v2_urdf/meshes/finger_4_proximal_phalanx_1.stl#L1-L1) · [local](../../desktop/resources/hku_hand_v2_urdf/meshes/finger_4_proximal_phalanx_1.stl)
<a id="ref-0391"></a>

- **[391]** `mmhand:desktop/resources/hku_hand_v2_urdf/meshes/mmhand_thumb_1_abduction_adduction_link_without_fin_1.stl` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/resources/hku_hand_v2_urdf/meshes/mmhand_thumb_1_abduction_adduction_link_without_fin_1.stl#L1-L1) · [local](../../desktop/resources/hku_hand_v2_urdf/meshes/mmhand_thumb_1_abduction_adduction_link_without_fin_1.stl)
<a id="ref-0392"></a>

- **[392]** `mmhand:desktop/resources/hku_hand_v2_urdf/meshes/mmhand_thumb_1_finger_7_distal_phalanx_1.stl` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/resources/hku_hand_v2_urdf/meshes/mmhand_thumb_1_finger_7_distal_phalanx_1.stl#L1-L1) · [local](../../desktop/resources/hku_hand_v2_urdf/meshes/mmhand_thumb_1_finger_7_distal_phalanx_1.stl)
<a id="ref-0393"></a>

- **[393]** `mmhand:desktop/resources/hku_hand_v2_urdf/meshes/mmhand_thumb_1_finger_7_fingertip_1.stl` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/resources/hku_hand_v2_urdf/meshes/mmhand_thumb_1_finger_7_fingertip_1.stl#L1-L1) · [local](../../desktop/resources/hku_hand_v2_urdf/meshes/mmhand_thumb_1_finger_7_fingertip_1.stl)
<a id="ref-0394"></a>

- **[394]** `mmhand:desktop/resources/hku_hand_v2_urdf/meshes/mmhand_thumb_1_finger_7_proximal_phalanx_1.stl` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/resources/hku_hand_v2_urdf/meshes/mmhand_thumb_1_finger_7_proximal_phalanx_1.stl#L1-L1) · [local](../../desktop/resources/hku_hand_v2_urdf/meshes/mmhand_thumb_1_finger_7_proximal_phalanx_1.stl)
<a id="ref-0395"></a>

- **[395]** `mmhand:desktop/resources/hku_hand_v2_urdf/meshes/mmhand_thumb_1_thumb_base_1.stl` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/resources/hku_hand_v2_urdf/meshes/mmhand_thumb_1_thumb_base_1.stl#L1-L1) · [local](../../desktop/resources/hku_hand_v2_urdf/meshes/mmhand_thumb_1_thumb_base_1.stl)
<a id="ref-0396"></a>

- **[396]** `mmhand:desktop/resources/hku_hand_v2_urdf/meshes/palm_1.stl` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/resources/hku_hand_v2_urdf/meshes/palm_1.stl#L1-L1) · [local](../../desktop/resources/hku_hand_v2_urdf/meshes/palm_1.stl)
<a id="ref-0397"></a>

- **[397]** `mmhand:desktop/resources/hku_hand_v2_urdf/meshes/palm_1_abduction_adduction_link_1.stl` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/resources/hku_hand_v2_urdf/meshes/palm_1_abduction_adduction_link_1.stl#L1-L1) · [local](../../desktop/resources/hku_hand_v2_urdf/meshes/palm_1_abduction_adduction_link_1.stl)
<a id="ref-0398"></a>

- **[398]** `mmhand:desktop/resources/hku_hand_v2_urdf/meshes/palm_1_abduction_adduction_link_2.stl` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/resources/hku_hand_v2_urdf/meshes/palm_1_abduction_adduction_link_2.stl#L1-L1) · [local](../../desktop/resources/hku_hand_v2_urdf/meshes/palm_1_abduction_adduction_link_2.stl)
<a id="ref-0399"></a>

- **[399]** `mmhand:desktop/resources/hku_hand_v2_urdf/meshes/palm_1_abduction_adduction_link_3.stl` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/resources/hku_hand_v2_urdf/meshes/palm_1_abduction_adduction_link_3.stl#L1-L1) · [local](../../desktop/resources/hku_hand_v2_urdf/meshes/palm_1_abduction_adduction_link_3.stl)
<a id="ref-0400"></a>

- **[400]** `mmhand:desktop/resources/hku_hand_v2_urdf/meshes/palm_1_abduction_adduction_link_without_fin_2.stl` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/resources/hku_hand_v2_urdf/meshes/palm_1_abduction_adduction_link_without_fin_2.stl#L1-L1) · [local](../../desktop/resources/hku_hand_v2_urdf/meshes/palm_1_abduction_adduction_link_without_fin_2.stl)
<a id="ref-0401"></a>

- **[401]** `mmhand:desktop/resources/hku_hand_v2_urdf/meshes/palm_1_finger_base_1.stl` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/resources/hku_hand_v2_urdf/meshes/palm_1_finger_base_1.stl#L1-L1) · [local](../../desktop/resources/hku_hand_v2_urdf/meshes/palm_1_finger_base_1.stl)
<a id="ref-0402"></a>

- **[402]** `mmhand:desktop/resources/hku_hand_v2_urdf/meshes/palm_1_finger_base_2.stl` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/resources/hku_hand_v2_urdf/meshes/palm_1_finger_base_2.stl#L1-L1) · [local](../../desktop/resources/hku_hand_v2_urdf/meshes/palm_1_finger_base_2.stl)
<a id="ref-0403"></a>

- **[403]** `mmhand:desktop/resources/hku_hand_v2_urdf/meshes/palm_1_finger_base_3.stl` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/resources/hku_hand_v2_urdf/meshes/palm_1_finger_base_3.stl#L1-L1) · [local](../../desktop/resources/hku_hand_v2_urdf/meshes/palm_1_finger_base_3.stl)
<a id="ref-0404"></a>

- **[404]** `mmhand:desktop/resources/hku_hand_v2_urdf/meshes/palm_1_finger_base_5.stl` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/resources/hku_hand_v2_urdf/meshes/palm_1_finger_base_5.stl#L1-L1) · [local](../../desktop/resources/hku_hand_v2_urdf/meshes/palm_1_finger_base_5.stl)
<a id="ref-0405"></a>

- **[405]** `mmhand:desktop/resources/hku_hand_v2_urdf/structure_urdf_adapter.json` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/resources/hku_hand_v2_urdf/structure_urdf_adapter.json#L1-L1) · [local](../../desktop/resources/hku_hand_v2_urdf/structure_urdf_adapter.json)
<a id="ref-0406"></a>

- **[406]** `mmhand:desktop/resources/hku_hand_v2_urdf/urdf/hand.urdf` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/resources/hku_hand_v2_urdf/urdf/hand.urdf#L1-L1) · [local](../../desktop/resources/hku_hand_v2_urdf/urdf/hand.urdf)
<a id="ref-0407"></a>

- **[407]** `mmhand:desktop/ros2/__init__.py` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/ros2/__init__.py#L1-L1) · [local](../../desktop/ros2/__init__.py)
<a id="ref-0408"></a>

- **[408]** `mmhand:desktop/ros2/joint_targets.py` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/ros2/joint_targets.py#L1-L1) · [local](../../desktop/ros2/joint_targets.py)
  - 相关源码行：[L136–L136 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/ros2/joint_targets.py#L136-L136) · [local](../../desktop/ros2/joint_targets.py)
<a id="ref-0409"></a>

- **[409]** `mmhand:desktop/ros2/raw_ik_target.py` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/ros2/raw_ik_target.py#L1-L1) · [local](../../desktop/ros2/raw_ik_target.py)
  - 相关源码行：[L61–L61 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/ros2/raw_ik_target.py#L61-L61) · [local](../../desktop/ros2/raw_ik_target.py), [L62–L62 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/ros2/raw_ik_target.py#L62-L62) · [local](../../desktop/ros2/raw_ik_target.py), [L73–L73 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/ros2/raw_ik_target.py#L73-L73) · [local](../../desktop/ros2/raw_ik_target.py), [L82–L82 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/ros2/raw_ik_target.py#L82-L82) · [local](../../desktop/ros2/raw_ik_target.py)
<a id="ref-0410"></a>

- **[410]** `mmhand:desktop/tests/test_calibration_snapshot_v4_protocol.py` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/tests/test_calibration_snapshot_v4_protocol.py#L1-L1) · [local](../../desktop/tests/test_calibration_snapshot_v4_protocol.py)
<a id="ref-0411"></a>

- **[411]** `mmhand:desktop/tests/test_control_trace_protocol.py` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/tests/test_control_trace_protocol.py#L1-L1) · [local](../../desktop/tests/test_control_trace_protocol.py)
<a id="ref-0412"></a>

- **[412]** `mmhand:desktop/tests/test_manual_reference_ack_scope.py` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/tests/test_manual_reference_ack_scope.py#L1-L1) · [local](../../desktop/tests/test_manual_reference_ack_scope.py)
<a id="ref-0413"></a>

- **[413]** `mmhand:desktop/tests/test_multiturn_runtime_identity.py` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/tests/test_multiturn_runtime_identity.py#L1-L1) · [local](../../desktop/tests/test_multiturn_runtime_identity.py)
<a id="ref-0414"></a>

- **[414]** `mmhand:desktop/tests/test_p4_safety_rule_closure.py` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/tests/test_p4_safety_rule_closure.py#L1-L1) · [local](../../desktop/tests/test_p4_safety_rule_closure.py)
<a id="ref-0415"></a>

- **[415]** `mmhand:desktop/tests/test_rope_init_compatibility.py` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/tests/test_rope_init_compatibility.py#L1-L1) · [local](../../desktop/tests/test_rope_init_compatibility.py)
<a id="ref-0416"></a>

- **[416]** `mmhand:desktop/tests/test_safety_event_protocol.py` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/tests/test_safety_event_protocol.py#L1-L1) · [local](../../desktop/tests/test_safety_event_protocol.py)
<a id="ref-0417"></a>

- **[417]** `mmhand:desktop/tests/test_safety_rules.py` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/tests/test_safety_rules.py#L1-L1) · [local](../../desktop/tests/test_safety_rules.py)
<a id="ref-0418"></a>

- **[418]** `mmhand:desktop/tests/test_state_sync_protocol.py` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/tests/test_state_sync_protocol.py#L1-L1) · [local](../../desktop/tests/test_state_sync_protocol.py)
<a id="ref-0419"></a>

- **[419]** `mmhand:desktop/tests/test_task_runtime_protocol.py` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/tests/test_task_runtime_protocol.py#L1-L1) · [local](../../desktop/tests/test_task_runtime_protocol.py)
<a id="ref-0420"></a>

- **[420]** `mmhand:desktop/tests/test_task_runtime_window_reducer.py` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/tests/test_task_runtime_window_reducer.py#L1-L1) · [local](../../desktop/tests/test_task_runtime_window_reducer.py)
<a id="ref-0421"></a>

- **[421]** `mmhand:desktop/tests/test_ui_status_labels.py` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/tests/test_ui_status_labels.py#L1-L1) · [local](../../desktop/tests/test_ui_status_labels.py)
<a id="ref-0422"></a>

- **[422]** `mmhand:desktop/ui/__init__.py` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/ui/__init__.py#L1-L1) · [local](../../desktop/ui/__init__.py)
<a id="ref-0423"></a>

- **[423]** `mmhand:desktop/ui/official_tactile_layout.py` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/ui/official_tactile_layout.py#L1-L1) · [local](../../desktop/ui/official_tactile_layout.py)
<a id="ref-0424"></a>

- **[424]** `mmhand:desktop/ui/readme.md` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/ui/readme.md#L1-L1) · [local](../../desktop/ui/readme.md)
<a id="ref-0425"></a>

- **[425]** `mmhand:desktop/ui/ui_app.py` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/ui/ui_app.py#L1-L1) · [local](../../desktop/ui/ui_app.py)
  - 相关源码行：[L1–L1 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/ui/ui_app.py#L1-L1) · [local](../../desktop/ui/ui_app.py), [L83–L83 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/ui/ui_app.py#L83-L83) · [local](../../desktop/ui/ui_app.py), [L202–L202 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/ui/ui_app.py#L202-L202) · [local](../../desktop/ui/ui_app.py), [L207–L207 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/ui/ui_app.py#L207-L207) · [local](../../desktop/ui/ui_app.py), [L260–L260 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/ui/ui_app.py#L260-L260) · [local](../../desktop/ui/ui_app.py), [L320–L320 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/ui/ui_app.py#L320-L320) · [local](../../desktop/ui/ui_app.py), [L431–L431 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/ui/ui_app.py#L431-L431) · [local](../../desktop/ui/ui_app.py), [L433–L433 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/ui/ui_app.py#L433-L433) · [local](../../desktop/ui/ui_app.py), [L437–L437 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/ui/ui_app.py#L437-L437) · [local](../../desktop/ui/ui_app.py), [L446–L446 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/ui/ui_app.py#L446-L446) · [local](../../desktop/ui/ui_app.py)
<a id="ref-0426"></a>

- **[426]** `mmhand:desktop/ui/ui_context.py` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/ui/ui_context.py#L1-L1) · [local](../../desktop/ui/ui_context.py)
<a id="ref-0427"></a>

- **[427]** `mmhand:desktop/ui/ui_control_calibration.py` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/ui/ui_control_calibration.py#L1-L1) · [local](../../desktop/ui/ui_control_calibration.py)
  - 相关源码行：[L55–L55 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/ui/ui_control_calibration.py#L55-L55) · [local](../../desktop/ui/ui_control_calibration.py), [L57–L57 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/ui/ui_control_calibration.py#L57-L57) · [local](../../desktop/ui/ui_control_calibration.py), [L60–L60 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/ui/ui_control_calibration.py#L60-L60) · [local](../../desktop/ui/ui_control_calibration.py), [L69–L69 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/ui/ui_control_calibration.py#L69-L69) · [local](../../desktop/ui/ui_control_calibration.py), [L73–L73 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/ui/ui_control_calibration.py#L73-L73) · [local](../../desktop/ui/ui_control_calibration.py), [L81–L81 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/ui/ui_control_calibration.py#L81-L81) · [local](../../desktop/ui/ui_control_calibration.py), [L86–L86 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/ui/ui_control_calibration.py#L86-L86) · [local](../../desktop/ui/ui_control_calibration.py), [L88–L88 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/ui/ui_control_calibration.py#L88-L88) · [local](../../desktop/ui/ui_control_calibration.py), [L91–L91 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/ui/ui_control_calibration.py#L91-L91) · [local](../../desktop/ui/ui_control_calibration.py), [L102–L107 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/ui/ui_control_calibration.py#L102-L107) · [local](../../desktop/ui/ui_control_calibration.py), [L140–L140 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/ui/ui_control_calibration.py#L140-L140) · [local](../../desktop/ui/ui_control_calibration.py), [L189–L189 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/ui/ui_control_calibration.py#L189-L189) · [local](../../desktop/ui/ui_control_calibration.py), [L194–L194 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/ui/ui_control_calibration.py#L194-L194) · [local](../../desktop/ui/ui_control_calibration.py), [L452–L452 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/ui/ui_control_calibration.py#L452-L452) · [local](../../desktop/ui/ui_control_calibration.py), [L454–L454 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/ui/ui_control_calibration.py#L454-L454) · [local](../../desktop/ui/ui_control_calibration.py), [L456–L456 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/ui/ui_control_calibration.py#L456-L456) · [local](../../desktop/ui/ui_control_calibration.py)
<a id="ref-0428"></a>

- **[428]** `mmhand:desktop/ui/ui_control_commands.py` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/ui/ui_control_commands.py#L1-L1) · [local](../../desktop/ui/ui_control_commands.py)
  - 相关源码行：[L23–L23 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/ui/ui_control_commands.py#L23-L23) · [local](../../desktop/ui/ui_control_commands.py), [L25–L25 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/ui/ui_control_commands.py#L25-L25) · [local](../../desktop/ui/ui_control_commands.py), [L28–L28 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/ui/ui_control_commands.py#L28-L28) · [local](../../desktop/ui/ui_control_commands.py), [L31–L31 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/ui/ui_control_commands.py#L31-L31) · [local](../../desktop/ui/ui_control_commands.py), [L32–L32 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/ui/ui_control_commands.py#L32-L32) · [local](../../desktop/ui/ui_control_commands.py), [L37–L37 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/ui/ui_control_commands.py#L37-L37) · [local](../../desktop/ui/ui_control_commands.py), [L41–L41 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/ui/ui_control_commands.py#L41-L41) · [local](../../desktop/ui/ui_control_commands.py), [L49–L49 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/ui/ui_control_commands.py#L49-L49) · [local](../../desktop/ui/ui_control_commands.py), [L58–L58 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/ui/ui_control_commands.py#L58-L58) · [local](../../desktop/ui/ui_control_commands.py), [L60–L60 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/ui/ui_control_commands.py#L60-L60) · [local](../../desktop/ui/ui_control_commands.py), [L70–L70 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/ui/ui_control_commands.py#L70-L70) · [local](../../desktop/ui/ui_control_commands.py), [L82–L83 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/ui/ui_control_commands.py#L82-L83) · [local](../../desktop/ui/ui_control_commands.py), [L86–L86 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/ui/ui_control_commands.py#L86-L86) · [local](../../desktop/ui/ui_control_commands.py), [L89–L89 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/ui/ui_control_commands.py#L89-L89) · [local](../../desktop/ui/ui_control_commands.py), [L96–L96 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/ui/ui_control_commands.py#L96-L96) · [local](../../desktop/ui/ui_control_commands.py), [L108–L108 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/ui/ui_control_commands.py#L108-L108) · [local](../../desktop/ui/ui_control_commands.py)
<a id="ref-0429"></a>

- **[429]** `mmhand:desktop/ui/ui_control_common.py` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/ui/ui_control_common.py#L1-L1) · [local](../../desktop/ui/ui_control_common.py)
<a id="ref-0430"></a>

- **[430]** `mmhand:desktop/ui/ui_control_device.py` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/ui/ui_control_device.py#L1-L1) · [local](../../desktop/ui/ui_control_device.py)
  - 相关源码行：[L1–L1 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/ui/ui_control_device.py#L1-L1) · [local](../../desktop/ui/ui_control_device.py), [L76–L76 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/ui/ui_control_device.py#L76-L76) · [local](../../desktop/ui/ui_control_device.py), [L77–L77 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/ui/ui_control_device.py#L77-L77) · [local](../../desktop/ui/ui_control_device.py), [L78–L78 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/ui/ui_control_device.py#L78-L78) · [local](../../desktop/ui/ui_control_device.py), [L79–L79 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/ui/ui_control_device.py#L79-L79) · [local](../../desktop/ui/ui_control_device.py), [L80–L80 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/ui/ui_control_device.py#L80-L80) · [local](../../desktop/ui/ui_control_device.py), [L81–L81 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/ui/ui_control_device.py#L81-L81) · [local](../../desktop/ui/ui_control_device.py)
<a id="ref-0431"></a>

- **[431]** `mmhand:desktop/ui/ui_control_manual.py` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/ui/ui_control_manual.py#L1-L1) · [local](../../desktop/ui/ui_control_manual.py)
  - 相关源码行：[L386–L386 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/ui/ui_control_manual.py#L386-L386) · [local](../../desktop/ui/ui_control_manual.py), [L388–L388 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/ui/ui_control_manual.py#L388-L388) · [local](../../desktop/ui/ui_control_manual.py), [L391–L391 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/ui/ui_control_manual.py#L391-L391) · [local](../../desktop/ui/ui_control_manual.py), [L403–L403 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/ui/ui_control_manual.py#L403-L403) · [local](../../desktop/ui/ui_control_manual.py), [L405–L405 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/ui/ui_control_manual.py#L405-L405) · [local](../../desktop/ui/ui_control_manual.py), [L410–L410 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/ui/ui_control_manual.py#L410-L410) · [local](../../desktop/ui/ui_control_manual.py), [L413–L413 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/ui/ui_control_manual.py#L413-L413) · [local](../../desktop/ui/ui_control_manual.py)
<a id="ref-0432"></a>

- **[432]** `mmhand:desktop/ui/ui_control_modes.py` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/ui/ui_control_modes.py#L1-L1) · [local](../../desktop/ui/ui_control_modes.py)
  - 相关源码行：[L1–L1 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/ui/ui_control_modes.py#L1-L1) · [local](../../desktop/ui/ui_control_modes.py), [L60–L60 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/ui/ui_control_modes.py#L60-L60) · [local](../../desktop/ui/ui_control_modes.py), [L63–L63 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/ui/ui_control_modes.py#L63-L63) · [local](../../desktop/ui/ui_control_modes.py), [L64–L64 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/ui/ui_control_modes.py#L64-L64) · [local](../../desktop/ui/ui_control_modes.py), [L104–L104 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/ui/ui_control_modes.py#L104-L104) · [local](../../desktop/ui/ui_control_modes.py)
<a id="ref-0433"></a>

- **[433]** `mmhand:desktop/ui/ui_control_panels.py` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/ui/ui_control_panels.py#L1-L1) · [local](../../desktop/ui/ui_control_panels.py)
<a id="ref-0434"></a>

- **[434]** `mmhand:desktop/ui/ui_control_scope.py` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/ui/ui_control_scope.py#L1-L1) · [local](../../desktop/ui/ui_control_scope.py)
<a id="ref-0435"></a>

- **[435]** `mmhand:desktop/ui/ui_control_state.py` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/ui/ui_control_state.py#L1-L1) · [local](../../desktop/ui/ui_control_state.py)
<a id="ref-0436"></a>

- **[436]** `mmhand:desktop/ui/ui_control_tables.py` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/ui/ui_control_tables.py#L1-L1) · [local](../../desktop/ui/ui_control_tables.py)
  - 相关源码行：[L1–L1 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/ui/ui_control_tables.py#L1-L1) · [local](../../desktop/ui/ui_control_tables.py), [L71–L71 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/ui/ui_control_tables.py#L71-L71) · [local](../../desktop/ui/ui_control_tables.py), [L116–L116 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/ui/ui_control_tables.py#L116-L116) · [local](../../desktop/ui/ui_control_tables.py), [L157–L157 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/ui/ui_control_tables.py#L157-L157) · [local](../../desktop/ui/ui_control_tables.py), [L252–L252 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/ui/ui_control_tables.py#L252-L252) · [local](../../desktop/ui/ui_control_tables.py), [L305–L305 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/ui/ui_control_tables.py#L305-L305) · [local](../../desktop/ui/ui_control_tables.py), [L325–L325 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/ui/ui_control_tables.py#L325-L325) · [local](../../desktop/ui/ui_control_tables.py), [L379–L379 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/ui/ui_control_tables.py#L379-L379) · [local](../../desktop/ui/ui_control_tables.py), [L385–L385 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/ui/ui_control_tables.py#L385-L385) · [local](../../desktop/ui/ui_control_tables.py), [L399–L399 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/ui/ui_control_tables.py#L399-L399) · [local](../../desktop/ui/ui_control_tables.py)
<a id="ref-0437"></a>

- **[437]** `mmhand:desktop/ui/ui_control_target_model.py` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/ui/ui_control_target_model.py#L1-L1) · [local](../../desktop/ui/ui_control_target_model.py)
<a id="ref-0438"></a>

- **[438]** `mmhand:desktop/ui/ui_control_test_panel.py` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/ui/ui_control_test_panel.py#L1-L1) · [local](../../desktop/ui/ui_control_test_panel.py)
  - 相关源码行：[L1–L1 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/ui/ui_control_test_panel.py#L1-L1) · [local](../../desktop/ui/ui_control_test_panel.py), [L48–L48 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/ui/ui_control_test_panel.py#L48-L48) · [local](../../desktop/ui/ui_control_test_panel.py), [L49–L49 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/ui/ui_control_test_panel.py#L49-L49) · [local](../../desktop/ui/ui_control_test_panel.py), [L92–L92 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/ui/ui_control_test_panel.py#L92-L92) · [local](../../desktop/ui/ui_control_test_panel.py), [L93–L93 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/ui/ui_control_test_panel.py#L93-L93) · [local](../../desktop/ui/ui_control_test_panel.py), [L116–L116 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/ui/ui_control_test_panel.py#L116-L116) · [local](../../desktop/ui/ui_control_test_panel.py), [L117–L117 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/ui/ui_control_test_panel.py#L117-L117) · [local](../../desktop/ui/ui_control_test_panel.py), [L147–L147 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/ui/ui_control_test_panel.py#L147-L147) · [local](../../desktop/ui/ui_control_test_panel.py), [L150–L150 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/ui/ui_control_test_panel.py#L150-L150) · [local](../../desktop/ui/ui_control_test_panel.py)
<a id="ref-0439"></a>

- **[439]** `mmhand:desktop/ui/ui_control_tests.py` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/ui/ui_control_tests.py#L1-L1) · [local](../../desktop/ui/ui_control_tests.py)
<a id="ref-0440"></a>

- **[440]** `mmhand:desktop/ui/ui_control_view.py` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/ui/ui_control_view.py#L1-L1) · [local](../../desktop/ui/ui_control_view.py)
<a id="ref-0441"></a>

- **[441]** `mmhand:desktop/ui/ui_device_check_view.py` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/ui/ui_device_check_view.py#L1-L1) · [local](../../desktop/ui/ui_device_check_view.py)
  - 相关源码行：[L1–L1 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/ui/ui_device_check_view.py#L1-L1) · [local](../../desktop/ui/ui_device_check_view.py), [L211–L211 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/ui/ui_device_check_view.py#L211-L211) · [local](../../desktop/ui/ui_device_check_view.py), [L214–L214 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/ui/ui_device_check_view.py#L214-L214) · [local](../../desktop/ui/ui_device_check_view.py), [L215–L215 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/ui/ui_device_check_view.py#L215-L215) · [local](../../desktop/ui/ui_device_check_view.py), [L248–L248 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/ui/ui_device_check_view.py#L248-L248) · [local](../../desktop/ui/ui_device_check_view.py), [L257–L257 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/ui/ui_device_check_view.py#L257-L257) · [local](../../desktop/ui/ui_device_check_view.py), [L259–L259 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/ui/ui_device_check_view.py#L259-L259) · [local](../../desktop/ui/ui_device_check_view.py), [L285–L285 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/ui/ui_device_check_view.py#L285-L285) · [local](../../desktop/ui/ui_device_check_view.py), [L287–L287 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/ui/ui_device_check_view.py#L287-L287) · [local](../../desktop/ui/ui_device_check_view.py), [L289–L289 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/ui/ui_device_check_view.py#L289-L289) · [local](../../desktop/ui/ui_device_check_view.py), [L306–L306 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/ui/ui_device_check_view.py#L306-L306) · [local](../../desktop/ui/ui_device_check_view.py), [L333–L333 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/ui/ui_device_check_view.py#L333-L333) · [local](../../desktop/ui/ui_device_check_view.py), [L335–L335 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/ui/ui_device_check_view.py#L335-L335) · [local](../../desktop/ui/ui_device_check_view.py), [L350–L350 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/ui/ui_device_check_view.py#L350-L350) · [local](../../desktop/ui/ui_device_check_view.py), [L369–L369 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/ui/ui_device_check_view.py#L369-L369) · [local](../../desktop/ui/ui_device_check_view.py), [L377–L377 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/ui/ui_device_check_view.py#L377-L377) · [local](../../desktop/ui/ui_device_check_view.py)
<a id="ref-0442"></a>

- **[442]** `mmhand:desktop/ui/ui_dialogs.py` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/ui/ui_dialogs.py#L1-L1) · [local](../../desktop/ui/ui_dialogs.py)
<a id="ref-0443"></a>

- **[443]** `mmhand:desktop/ui/ui_joint_plot_view.py` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/ui/ui_joint_plot_view.py#L1-L1) · [local](../../desktop/ui/ui_joint_plot_view.py)
  - 相关源码行：[L1–L1 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/ui/ui_joint_plot_view.py#L1-L1) · [local](../../desktop/ui/ui_joint_plot_view.py), [L55–L55 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/ui/ui_joint_plot_view.py#L55-L55) · [local](../../desktop/ui/ui_joint_plot_view.py)
<a id="ref-0444"></a>

- **[444]** `mmhand:desktop/ui/ui_pose_view.py` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/ui/ui_pose_view.py#L1-L1) · [local](../../desktop/ui/ui_pose_view.py)
  - 相关源码行：[L1–L1 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/ui/ui_pose_view.py#L1-L1) · [local](../../desktop/ui/ui_pose_view.py), [L138–L138 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/ui/ui_pose_view.py#L138-L138) · [local](../../desktop/ui/ui_pose_view.py)
<a id="ref-0445"></a>

- **[445]** `mmhand:desktop/ui/ui_qt_compat.py` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/ui/ui_qt_compat.py#L1-L1) · [local](../../desktop/ui/ui_qt_compat.py)
<a id="ref-0446"></a>

- **[446]** `mmhand:desktop/ui/ui_replay_view.py` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/ui/ui_replay_view.py#L1-L1) · [local](../../desktop/ui/ui_replay_view.py)
  - 相关源码行：[L1–L1 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/ui/ui_replay_view.py#L1-L1) · [local](../../desktop/ui/ui_replay_view.py), [L48–L48 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/ui/ui_replay_view.py#L48-L48) · [local](../../desktop/ui/ui_replay_view.py), [L103–L103 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/ui/ui_replay_view.py#L103-L103) · [local](../../desktop/ui/ui_replay_view.py), [L125–L125 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/ui/ui_replay_view.py#L125-L125) · [local](../../desktop/ui/ui_replay_view.py), [L126–L126 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/ui/ui_replay_view.py#L126-L126) · [local](../../desktop/ui/ui_replay_view.py), [L130–L130 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/ui/ui_replay_view.py#L130-L130) · [local](../../desktop/ui/ui_replay_view.py)
<a id="ref-0447"></a>

- **[447]** `mmhand:desktop/ui/ui_services.py` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/ui/ui_services.py#L1-L1) · [local](../../desktop/ui/ui_services.py)
<a id="ref-0448"></a>

- **[448]** `mmhand:desktop/ui/ui_style.py` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/ui/ui_style.py#L1-L1) · [local](../../desktop/ui/ui_style.py)
<a id="ref-0449"></a>

- **[449]** `mmhand:desktop/ui/ui_tactile_view.py` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/ui/ui_tactile_view.py#L1-L1) · [local](../../desktop/ui/ui_tactile_view.py)
  - 相关源码行：[L1–L1 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/ui/ui_tactile_view.py#L1-L1) · [local](../../desktop/ui/ui_tactile_view.py), [L384–L384 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/ui/ui_tactile_view.py#L384-L384) · [local](../../desktop/ui/ui_tactile_view.py), [L385–L385 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/ui/ui_tactile_view.py#L385-L385) · [local](../../desktop/ui/ui_tactile_view.py), [L386–L386 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/ui/ui_tactile_view.py#L386-L386) · [local](../../desktop/ui/ui_tactile_view.py), [L463–L463 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/ui/ui_tactile_view.py#L463-L463) · [local](../../desktop/ui/ui_tactile_view.py), [L465–L465 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/ui/ui_tactile_view.py#L465-L465) · [local](../../desktop/ui/ui_tactile_view.py), [L466–L466 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/ui/ui_tactile_view.py#L466-L466) · [local](../../desktop/ui/ui_tactile_view.py)
<a id="ref-0450"></a>

- **[450]** `mmhand:desktop/ui/ui_task_runtime.py` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/ui/ui_task_runtime.py#L1-L1) · [local](../../desktop/ui/ui_task_runtime.py)
<a id="ref-0451"></a>

- **[451]** `mmhand:desktop/ui/ui_windows.py` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/ui/ui_windows.py#L1-L1) · [local](../../desktop/ui/ui_windows.py)
  - 相关源码行：[L1–L1 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/ui/ui_windows.py#L1-L1) · [local](../../desktop/ui/ui_windows.py), [L63–L63 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/ui/ui_windows.py#L63-L63) · [local](../../desktop/ui/ui_windows.py), [L69–L69 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/ui/ui_windows.py#L69-L69) · [local](../../desktop/ui/ui_windows.py), [L77–L77 GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/ui/ui_windows.py#L77-L77) · [local](../../desktop/ui/ui_windows.py)
<a id="ref-0452"></a>

- **[452]** `mmhand:desktop/visualize/__init__.py` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/visualize/__init__.py#L1-L1) · [local](../../desktop/visualize/__init__.py)
<a id="ref-0453"></a>

- **[453]** `mmhand:desktop/visualize/analyze_single_motor_pid.py` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/visualize/analyze_single_motor_pid.py#L1-L1) · [local](../../desktop/visualize/analyze_single_motor_pid.py)
<a id="ref-0454"></a>

- **[454]** `mmhand:desktop/visualize/plot_hand_from_urdf.py` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/visualize/plot_hand_from_urdf.py#L1-L1) · [local](../../desktop/visualize/plot_hand_from_urdf.py)
<a id="ref-0455"></a>

- **[455]** `mmhand:desktop/visualize/plot_sine_response.py` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/visualize/plot_sine_response.py#L1-L1) · [local](../../desktop/visualize/plot_sine_response.py)
<a id="ref-0456"></a>

- **[456]** `mmhand:desktop/visualize/plot_step_response.py` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/visualize/plot_step_response.py#L1-L1) · [local](../../desktop/visualize/plot_step_response.py)
<a id="ref-0457"></a>

- **[457]** `mmhand:desktop/visualize/plot_tendon_lut.py` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/visualize/plot_tendon_lut.py#L1-L1) · [local](../../desktop/visualize/plot_tendon_lut.py)
<a id="ref-0458"></a>

- **[458]** `mmhand:desktop/visualize/urdf_kinematics.py` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/desktop/visualize/urdf_kinematics.py#L1-L1) · [local](../../desktop/visualize/urdf_kinematics.py)
<a id="ref-0459"></a>

- **[459]** `mmhand:docs/1.0算法.md` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/docs/1.0%E7%AE%97%E6%B3%95.md#L1-L1) · [local](../1.0%E7%AE%97%E6%B3%95.md)
<a id="ref-0460"></a>

- **[460]** `mmhand:docs/CalibLoad标定快照恢复说明.md` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/docs/CalibLoad%E6%A0%87%E5%AE%9A%E5%BF%AB%E7%85%A7%E6%81%A2%E5%A4%8D%E8%AF%B4%E6%98%8E.md#L1-L1) · [local](../CalibLoad%E6%A0%87%E5%AE%9A%E5%BF%AB%E7%85%A7%E6%81%A2%E5%A4%8D%E8%AF%B4%E6%98%8E.md)
<a id="ref-0461"></a>

- **[461]** `mmhand:docs/P4上位通信状态丢失问题修复说明.md` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/docs/P4%E4%B8%8A%E4%BD%8D%E9%80%9A%E4%BF%A1%E7%8A%B6%E6%80%81%E4%B8%A2%E5%A4%B1%E9%97%AE%E9%A2%98%E4%BF%AE%E5%A4%8D%E8%AF%B4%E6%98%8E.md#L1-L1) · [local](../P4%E4%B8%8A%E4%BD%8D%E9%80%9A%E4%BF%A1%E7%8A%B6%E6%80%81%E4%B8%A2%E5%A4%B1%E9%97%AE%E9%A2%98%E4%BF%AE%E5%A4%8D%E8%AF%B4%E6%98%8E.md)
<a id="ref-0462"></a>

- **[462]** `mmhand:docs/P4固件架构与函数调用关系全景解析_20260810.docx` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/docs/P4%E5%9B%BA%E4%BB%B6%E6%9E%B6%E6%9E%84%E4%B8%8E%E5%87%BD%E6%95%B0%E8%B0%83%E7%94%A8%E5%85%B3%E7%B3%BB%E5%85%A8%E6%99%AF%E8%A7%A3%E6%9E%90_20260810.docx#L1-L1) · [local](../P4%E5%9B%BA%E4%BB%B6%E6%9E%B6%E6%9E%84%E4%B8%8E%E5%87%BD%E6%95%B0%E8%B0%83%E7%94%A8%E5%85%B3%E7%B3%BB%E5%85%A8%E6%99%AF%E8%A7%A3%E6%9E%90_20260810.docx)
<a id="ref-0463"></a>

- **[463]** `mmhand:docs/README.md` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/docs/README.md#L1-L1) · [local](../README.md)
<a id="ref-0464"></a>

- **[464]** `mmhand:docs/ROS2与遥操链路.md` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/docs/ROS2%E4%B8%8E%E9%81%A5%E6%93%8D%E9%93%BE%E8%B7%AF.md#L1-L1) · [local](../ROS2%E4%B8%8E%E9%81%A5%E6%93%8D%E9%93%BE%E8%B7%AF.md)
<a id="ref-0465"></a>

- **[465]** `mmhand:docs/SPI模式与磁编触觉连线说明.md` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/docs/SPI%E6%A8%A1%E5%BC%8F%E4%B8%8E%E7%A3%81%E7%BC%96%E8%A7%A6%E8%A7%89%E8%BF%9E%E7%BA%BF%E8%AF%B4%E6%98%8E.md#L1-L1) · [local](../SPI%E6%A8%A1%E5%BC%8F%E4%B8%8E%E7%A3%81%E7%BC%96%E8%A7%A6%E8%A7%89%E8%BF%9E%E7%BA%BF%E8%AF%B4%E6%98%8E.md)
<a id="ref-0466"></a>

- **[466]** `mmhand:docs/UART_I2C_SPI_CAN通信协议入门教程.docx` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/docs/UART_I2C_SPI_CAN%E9%80%9A%E4%BF%A1%E5%8D%8F%E8%AE%AE%E5%85%A5%E9%97%A8%E6%95%99%E7%A8%8B.docx#L1-L1) · [local](../UART_I2C_SPI_CAN%E9%80%9A%E4%BF%A1%E5%8D%8F%E8%AE%AE%E5%85%A5%E9%97%A8%E6%95%99%E7%A8%8B.docx)
<a id="ref-0467"></a>

- **[467]** `mmhand:docs/UART_I2C_SPI_CAN通信协议入门教程.md` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/docs/UART_I2C_SPI_CAN%E9%80%9A%E4%BF%A1%E5%8D%8F%E8%AE%AE%E5%85%A5%E9%97%A8%E6%95%99%E7%A8%8B.md#L1-L1) · [local](../UART_I2C_SPI_CAN%E9%80%9A%E4%BF%A1%E5%8D%8F%E8%AE%AE%E5%85%A5%E9%97%A8%E6%95%99%E7%A8%8B.md)
<a id="ref-0468"></a>

- **[468]** `mmhand:docs/URDF自主更新指南.md` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/docs/URDF%E8%87%AA%E4%B8%BB%E6%9B%B4%E6%96%B0%E6%8C%87%E5%8D%97.md#L1-L1) · [local](../URDF%E8%87%AA%E4%B8%BB%E6%9B%B4%E6%96%B0%E6%8C%87%E5%8D%97.md)
<a id="ref-0469"></a>

- **[469]** `mmhand:docs/current_mapping_readme.md` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/docs/current_mapping_readme.md#L1-L1) · [local](../current_mapping_readme.md)
<a id="ref-0470"></a>

- **[470]** `mmhand:docs/images/ESP32Nano.jpg` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/docs/images/ESP32Nano.jpg#L1-L1) · [local](../images/ESP32Nano.jpg)
<a id="ref-0471"></a>

- **[471]** `mmhand:docs/images/ESP32P4Dev.jpg` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/docs/images/ESP32P4Dev.jpg#L1-L1) · [local](../images/ESP32P4Dev.jpg)
<a id="ref-0472"></a>

- **[472]** `mmhand:docs/images/PackageSelection.jpg` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/docs/images/PackageSelection.jpg#L1-L1) · [local](../images/PackageSelection.jpg)
<a id="ref-0473"></a>

- **[473]** `mmhand:docs/images/SketchbookLocation.jpg` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/docs/images/SketchbookLocation.jpg#L1-L1) · [local](../images/SketchbookLocation.jpg)
<a id="ref-0474"></a>

- **[474]** `mmhand:docs/images/protocol_waveforms/can_expected_txd_canh_canl.png` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/docs/images/protocol_waveforms/can_expected_txd_canh_canl.png#L1-L1) · [local](../images/protocol_waveforms/can_expected_txd_canh_canl.png)
<a id="ref-0475"></a>

- **[475]** `mmhand:docs/images/protocol_waveforms/i2c_actual_nack_kingst.png` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/docs/images/protocol_waveforms/i2c_actual_nack_kingst.png#L1-L1) · [local](../images/protocol_waveforms/i2c_actual_nack_kingst.png)
<a id="ref-0476"></a>

- **[476]** `mmhand:docs/images/protocol_waveforms/i2c_expected_ack.png` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/docs/images/protocol_waveforms/i2c_expected_ack.png#L1-L1) · [local](../images/protocol_waveforms/i2c_expected_ack.png)
<a id="ref-0477"></a>

- **[477]** `mmhand:docs/images/protocol_waveforms/spi_actual_mosi_kingst.png` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/docs/images/protocol_waveforms/spi_actual_mosi_kingst.png#L1-L1) · [local](../images/protocol_waveforms/spi_actual_mosi_kingst.png)
<a id="ref-0478"></a>

- **[478]** `mmhand:docs/images/protocol_waveforms/spi_expected_miso_loopback.png` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/docs/images/protocol_waveforms/spi_expected_miso_loopback.png#L1-L1) · [local](../images/protocol_waveforms/spi_expected_miso_loopback.png)
<a id="ref-0479"></a>

- **[479]** `mmhand:docs/images/protocol_waveforms/uart_actual_kingst.png` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/docs/images/protocol_waveforms/uart_actual_kingst.png#L1-L1) · [local](../images/protocol_waveforms/uart_actual_kingst.png)
<a id="ref-0480"></a>

- **[480]** `mmhand:docs/images/upload.jpg` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/docs/images/upload.jpg#L1-L1) · [local](../images/upload.jpg)
<a id="ref-0481"></a>

- **[481]** `mmhand:docs/pid_control_mode_current.md` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/docs/pid_control_mode_current.md#L1-L1) · [local](../pid_control_mode_current.md)
<a id="ref-0482"></a>

- **[482]** `mmhand:docs/start.md` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/docs/start.md#L1-L1) · [local](../start.md)
<a id="ref-0483"></a>

- **[483]** `mmhand:docs/wuji_URDF_适配与ROS2链路说明.md` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/docs/wuji_URDF_%E9%80%82%E9%85%8D%E4%B8%8EROS2%E9%93%BE%E8%B7%AF%E8%AF%B4%E6%98%8E.md#L1-L1) · [local](../wuji_URDF_%E9%80%82%E9%85%8D%E4%B8%8EROS2%E9%93%BE%E8%B7%AF%E8%AF%B4%E6%98%8E.md)
<a id="ref-0484"></a>

- **[484]** `mmhand:docs/上位机使用说明.md` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/docs/%E4%B8%8A%E4%BD%8D%E6%9C%BA%E4%BD%BF%E7%94%A8%E8%AF%B4%E6%98%8E.md#L1-L1) · [local](../%E4%B8%8A%E4%BD%8D%E6%9C%BA%E4%BD%BF%E7%94%A8%E8%AF%B4%E6%98%8E.md)
<a id="ref-0485"></a>

- **[485]** `mmhand:docs/上手代码必读.md` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/docs/%E4%B8%8A%E6%89%8B%E4%BB%A3%E7%A0%81%E5%BF%85%E8%AF%BB.md#L1-L1) · [local](../%E4%B8%8A%E6%89%8B%E4%BB%A3%E7%A0%81%E5%BF%85%E8%AF%BB.md)
<a id="ref-0486"></a>

- **[486]** `mmhand:docs/交接/00_项目接手总览.md` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/docs/%E4%BA%A4%E6%8E%A5/00_%E9%A1%B9%E7%9B%AE%E6%8E%A5%E6%89%8B%E6%80%BB%E8%A7%88.md#L1-L1) · [local](../%E4%BA%A4%E6%8E%A5/00_%E9%A1%B9%E7%9B%AE%E6%8E%A5%E6%89%8B%E6%80%BB%E8%A7%88.md)
<a id="ref-0487"></a>

- **[487]** `mmhand:docs/交接/01_上位机逐文件.md` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/docs/%E4%BA%A4%E6%8E%A5/01_%E4%B8%8A%E4%BD%8D%E6%9C%BA%E9%80%90%E6%96%87%E4%BB%B6.md#L1-L1) · [local](../%E4%BA%A4%E6%8E%A5/01_%E4%B8%8A%E4%BD%8D%E6%9C%BA%E9%80%90%E6%96%87%E4%BB%B6.md)
<a id="ref-0488"></a>

- **[488]** `mmhand:docs/交接/02_P4逐文件.md` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/docs/%E4%BA%A4%E6%8E%A5/02_P4%E9%80%90%E6%96%87%E4%BB%B6.md#L1-L1) · [local](../%E4%BA%A4%E6%8E%A5/02_P4%E9%80%90%E6%96%87%E4%BB%B6.md)
<a id="ref-0489"></a>

- **[489]** `mmhand:docs/交接/03_S3逐文件.md` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/docs/%E4%BA%A4%E6%8E%A5/03_S3%E9%80%90%E6%96%87%E4%BB%B6.md#L1-L1) · [local](../%E4%BA%A4%E6%8E%A5/03_S3%E9%80%90%E6%96%87%E4%BB%B6.md)
<a id="ref-0490"></a>

- **[490]** `mmhand:docs/交接/04_工具与遥操逐文件.md` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/docs/%E4%BA%A4%E6%8E%A5/04_%E5%B7%A5%E5%85%B7%E4%B8%8E%E9%81%A5%E6%93%8D%E9%80%90%E6%96%87%E4%BB%B6.md#L1-L1) · [local](../%E4%BA%A4%E6%8E%A5/04_%E5%B7%A5%E5%85%B7%E4%B8%8E%E9%81%A5%E6%93%8D%E9%80%90%E6%96%87%E4%BB%B6.md)
<a id="ref-0491"></a>

- **[491]** `mmhand:docs/交接/05_资源与第三方文件.md` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/docs/%E4%BA%A4%E6%8E%A5/05_%E8%B5%84%E6%BA%90%E4%B8%8E%E7%AC%AC%E4%B8%89%E6%96%B9%E6%96%87%E4%BB%B6.md#L1-L1) · [local](../%E4%BA%A4%E6%8E%A5/05_%E8%B5%84%E6%BA%90%E4%B8%8E%E7%AC%AC%E4%B8%89%E6%96%B9%E6%96%87%E4%BB%B6.md)
<a id="ref-0492"></a>

- **[492]** `mmhand:docs/交接/06_测试文件.md` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/docs/%E4%BA%A4%E6%8E%A5/06_%E6%B5%8B%E8%AF%95%E6%96%87%E4%BB%B6.md#L1-L1) · [local](../%E4%BA%A4%E6%8E%A5/06_%E6%B5%8B%E8%AF%95%E6%96%87%E4%BB%B6.md)
<a id="ref-0493"></a>

- **[493]** `mmhand:docs/交接/07_索引覆盖说明.md` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/docs/%E4%BA%A4%E6%8E%A5/07_%E7%B4%A2%E5%BC%95%E8%A6%86%E7%9B%96%E8%AF%B4%E6%98%8E.md#L1-L1) · [local](../%E4%BA%A4%E6%8E%A5/07_%E7%B4%A2%E5%BC%95%E8%A6%86%E7%9B%96%E8%AF%B4%E6%98%8E.md)
<a id="ref-0494"></a>

- **[494]** `mmhand:docs/交接/08_现场交接与验证清单.md` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/docs/%E4%BA%A4%E6%8E%A5/08_%E7%8E%B0%E5%9C%BA%E4%BA%A4%E6%8E%A5%E4%B8%8E%E9%AA%8C%E8%AF%81%E6%B8%85%E5%8D%95.md#L1-L1) · [local](../%E4%BA%A4%E6%8E%A5/08_%E7%8E%B0%E5%9C%BA%E4%BA%A4%E6%8E%A5%E4%B8%8E%E9%AA%8C%E8%AF%81%E6%B8%85%E5%8D%95.md)
<a id="ref-0495"></a>

- **[495]** `mmhand:docs/单腱速度前馈与响应优化说明.md` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/docs/%E5%8D%95%E8%85%B1%E9%80%9F%E5%BA%A6%E5%89%8D%E9%A6%88%E4%B8%8E%E5%93%8D%E5%BA%94%E4%BC%98%E5%8C%96%E8%AF%B4%E6%98%8E.md#L1-L1) · [local](../%E5%8D%95%E8%85%B1%E9%80%9F%E5%BA%A6%E5%89%8D%E9%A6%88%E4%B8%8E%E5%93%8D%E5%BA%94%E4%BC%98%E5%8C%96%E8%AF%B4%E6%98%8E.md)
<a id="ref-0496"></a>

- **[496]** `mmhand:docs/双腱PID与LUT配对恢复说明.md` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/docs/%E5%8F%8C%E8%85%B1PID%E4%B8%8ELUT%E9%85%8D%E5%AF%B9%E6%81%A2%E5%A4%8D%E8%AF%B4%E6%98%8E.md#L1-L1) · [local](../%E5%8F%8C%E8%85%B1PID%E4%B8%8ELUT%E9%85%8D%E5%AF%B9%E6%81%A2%E5%A4%8D%E8%AF%B4%E6%98%8E.md)
<a id="ref-0497"></a>

- **[497]** `mmhand:docs/多圈与标定_JSON_恢复.md` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/docs/%E5%A4%9A%E5%9C%88%E4%B8%8E%E6%A0%87%E5%AE%9A_JSON_%E6%81%A2%E5%A4%8D.md#L1-L1) · [local](../%E5%A4%9A%E5%9C%88%E4%B8%8E%E6%A0%87%E5%AE%9A_JSON_%E6%81%A2%E5%A4%8D.md)
<a id="ref-0498"></a>

- **[498]** `mmhand:docs/实机验证与故障排查.md` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/docs/%E5%AE%9E%E6%9C%BA%E9%AA%8C%E8%AF%81%E4%B8%8E%E6%95%85%E9%9A%9C%E6%8E%92%E6%9F%A5.md#L1-L1) · [local](../%E5%AE%9E%E6%9C%BA%E9%AA%8C%E8%AF%81%E4%B8%8E%E6%95%85%E9%9A%9C%E6%8E%92%E6%9F%A5.md)
<a id="ref-0499"></a>

- **[499]** `mmhand:docs/对抗双电机标定与RopeInit说明.md` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/docs/%E5%AF%B9%E6%8A%97%E5%8F%8C%E7%94%B5%E6%9C%BA%E6%A0%87%E5%AE%9A%E4%B8%8ERopeInit%E8%AF%B4%E6%98%8E.md#L1-L1) · [local](../%E5%AF%B9%E6%8A%97%E5%8F%8C%E7%94%B5%E6%9C%BA%E6%A0%87%E5%AE%9A%E4%B8%8ERopeInit%E8%AF%B4%E6%98%8E.md)
<a id="ref-0500"></a>

- **[500]** `mmhand:docs/当前系统与碰撞检测状态机.md` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/docs/%E5%BD%93%E5%89%8D%E7%B3%BB%E7%BB%9F%E4%B8%8E%E7%A2%B0%E6%92%9E%E6%A3%80%E6%B5%8B%E7%8A%B6%E6%80%81%E6%9C%BA.md#L1-L1) · [local](../%E5%BD%93%E5%89%8D%E7%B3%BB%E7%BB%9F%E4%B8%8E%E7%A2%B0%E6%92%9E%E6%A3%80%E6%B5%8B%E7%8A%B6%E6%80%81%E6%9C%BA.md)
<a id="ref-0501"></a>

- **[501]** `mmhand:docs/持久化与掉电恢复.md` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/docs/%E6%8C%81%E4%B9%85%E5%8C%96%E4%B8%8E%E6%8E%89%E7%94%B5%E6%81%A2%E5%A4%8D.md#L1-L1) · [local](../%E6%8C%81%E4%B9%85%E5%8C%96%E4%B8%8E%E6%8E%89%E7%94%B5%E6%81%A2%E5%A4%8D.md)
<a id="ref-0502"></a>

- **[502]** `mmhand:docs/控制算法与标定.md` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/docs/%E6%8E%A7%E5%88%B6%E7%AE%97%E6%B3%95%E4%B8%8E%E6%A0%87%E5%AE%9A.md#L1-L1) · [local](../%E6%8E%A7%E5%88%B6%E7%AE%97%E6%B3%95%E4%B8%8E%E6%A0%87%E5%AE%9A.md)
<a id="ref-0503"></a>

- **[503]** `mmhand:docs/标定卡顿_ControlTask队列抢占根因说明_20260712.md` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/docs/%E6%A0%87%E5%AE%9A%E5%8D%A1%E9%A1%BF_ControlTask%E9%98%9F%E5%88%97%E6%8A%A2%E5%8D%A0%E6%A0%B9%E5%9B%A0%E8%AF%B4%E6%98%8E_20260712.md#L1-L1) · [local](../%E6%A0%87%E5%AE%9A%E5%8D%A1%E9%A1%BF_ControlTask%E9%98%9F%E5%88%97%E6%8A%A2%E5%8D%A0%E6%A0%B9%E5%9B%A0%E8%AF%B4%E6%98%8E_20260712.md)
<a id="ref-0504"></a>

- **[504]** `mmhand:docs/校验算法入门与代码说明.md` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/docs/%E6%A0%A1%E9%AA%8C%E7%AE%97%E6%B3%95%E5%85%A5%E9%97%A8%E4%B8%8E%E4%BB%A3%E7%A0%81%E8%AF%B4%E6%98%8E.md#L1-L1) · [local](../%E6%A0%A1%E9%AA%8C%E7%AE%97%E6%B3%95%E5%85%A5%E9%97%A8%E4%B8%8E%E4%BB%A3%E7%A0%81%E8%AF%B4%E6%98%8E.md)
<a id="ref-0505"></a>

- **[505]** `mmhand:docs/状态机优化说明.md` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/docs/%E7%8A%B6%E6%80%81%E6%9C%BA%E4%BC%98%E5%8C%96%E8%AF%B4%E6%98%8E.md#L1-L1) · [local](../%E7%8A%B6%E6%80%81%E6%9C%BA%E4%BC%98%E5%8C%96%E8%AF%B4%E6%98%8E.md)
<a id="ref-0506"></a>

- **[506]** `mmhand:docs/硬件映射与连线.md` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/docs/%E7%A1%AC%E4%BB%B6%E6%98%A0%E5%B0%84%E4%B8%8E%E8%BF%9E%E7%BA%BF.md#L1-L1) · [local](../%E7%A1%AC%E4%BB%B6%E6%98%A0%E5%B0%84%E4%B8%8E%E8%BF%9E%E7%BA%BF.md)
<a id="ref-0507"></a>

- **[507]** `mmhand:docs/系统状态机与RTOS任务.md` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/docs/%E7%B3%BB%E7%BB%9F%E7%8A%B6%E6%80%81%E6%9C%BA%E4%B8%8ERTOS%E4%BB%BB%E5%8A%A1.md#L1-L1) · [local](../%E7%B3%BB%E7%BB%9F%E7%8A%B6%E6%80%81%E6%9C%BA%E4%B8%8ERTOS%E4%BB%BB%E5%8A%A1.md)
<a id="ref-0508"></a>

- **[508]** `mmhand:docs/舵机单独掉电_多圈坐标与动态行程恢复说明.md` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/docs/%E8%88%B5%E6%9C%BA%E5%8D%95%E7%8B%AC%E6%8E%89%E7%94%B5_%E5%A4%9A%E5%9C%88%E5%9D%90%E6%A0%87%E4%B8%8E%E5%8A%A8%E6%80%81%E8%A1%8C%E7%A8%8B%E6%81%A2%E5%A4%8D%E8%AF%B4%E6%98%8E.md#L1-L1) · [local](../%E8%88%B5%E6%9C%BA%E5%8D%95%E7%8B%AC%E6%8E%89%E7%94%B5_%E5%A4%9A%E5%9C%88%E5%9D%90%E6%A0%87%E4%B8%8E%E5%8A%A8%E6%80%81%E8%A1%8C%E7%A8%8B%E6%81%A2%E5%A4%8D%E8%AF%B4%E6%98%8E.md)
<a id="ref-0509"></a>

- **[509]** `mmhand:docs/舵机磁编触觉连线总结.md` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/docs/%E8%88%B5%E6%9C%BA%E7%A3%81%E7%BC%96%E8%A7%A6%E8%A7%89%E8%BF%9E%E7%BA%BF%E6%80%BB%E7%BB%93.md#L1-L1) · [local](../%E8%88%B5%E6%9C%BA%E7%A3%81%E7%BC%96%E8%A7%A6%E8%A7%89%E8%BF%9E%E7%BA%BF%E6%80%BB%E7%BB%93.md)
<a id="ref-0510"></a>

- **[510]** `mmhand:docs/触觉与磁编改进详解.md` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/docs/%E8%A7%A6%E8%A7%89%E4%B8%8E%E7%A3%81%E7%BC%96%E6%94%B9%E8%BF%9B%E8%AF%A6%E8%A7%A3.md#L1-L1) · [local](../%E8%A7%A6%E8%A7%89%E4%B8%8E%E7%A3%81%E7%BC%96%E6%94%B9%E8%BF%9B%E8%AF%A6%E8%A7%A3.md)
<a id="ref-0511"></a>

- **[511]** `mmhand:docs/通信.md` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/docs/%E9%80%9A%E4%BF%A1.md#L1-L1) · [local](../%E9%80%9A%E4%BF%A1.md)
<a id="ref-0512"></a>

- **[512]** `mmhand:docs/通信协议与数据校验.md` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/docs/%E9%80%9A%E4%BF%A1%E5%8D%8F%E8%AE%AE%E4%B8%8E%E6%95%B0%E6%8D%AE%E6%A0%A1%E9%AA%8C.md#L1-L1) · [local](../%E9%80%9A%E4%BF%A1%E5%8D%8F%E8%AE%AE%E4%B8%8E%E6%95%B0%E6%8D%AE%E6%A0%A1%E9%AA%8C.md)
<a id="ref-0513"></a>

- **[513]** `mmhand:docs/通信帧详解与面试指南.md` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/docs/%E9%80%9A%E4%BF%A1%E5%B8%A7%E8%AF%A6%E8%A7%A3%E4%B8%8E%E9%9D%A2%E8%AF%95%E6%8C%87%E5%8D%97.md#L1-L1) · [local](../%E9%80%9A%E4%BF%A1%E5%B8%A7%E8%AF%A6%E8%A7%A3%E4%B8%8E%E9%9D%A2%E8%AF%95%E6%8C%87%E5%8D%97.md)
<a id="ref-0514"></a>

- **[514]** `mmhand:docs/飞书导入包/00_项目总览与接手指南.md` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/docs/%E9%A3%9E%E4%B9%A6%E5%AF%BC%E5%85%A5%E5%8C%85/00_%E9%A1%B9%E7%9B%AE%E6%80%BB%E8%A7%88%E4%B8%8E%E6%8E%A5%E6%89%8B%E6%8C%87%E5%8D%97.md#L1-L1) · [local](../%E9%A3%9E%E4%B9%A6%E5%AF%BC%E5%85%A5%E5%8C%85/00_%E9%A1%B9%E7%9B%AE%E6%80%BB%E8%A7%88%E4%B8%8E%E6%8E%A5%E6%89%8B%E6%8C%87%E5%8D%97.md)
<a id="ref-0515"></a>

- **[515]** `mmhand:docs/飞书导入包/01_上位机架构与使用流程.md` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/docs/%E9%A3%9E%E4%B9%A6%E5%AF%BC%E5%85%A5%E5%8C%85/01_%E4%B8%8A%E4%BD%8D%E6%9C%BA%E6%9E%B6%E6%9E%84%E4%B8%8E%E4%BD%BF%E7%94%A8%E6%B5%81%E7%A8%8B.md#L1-L1) · [local](../%E9%A3%9E%E4%B9%A6%E5%AF%BC%E5%85%A5%E5%8C%85/01_%E4%B8%8A%E4%BD%8D%E6%9C%BA%E6%9E%B6%E6%9E%84%E4%B8%8E%E4%BD%BF%E7%94%A8%E6%B5%81%E7%A8%8B.md)
<a id="ref-0516"></a>

- **[516]** `mmhand:docs/飞书导入包/02_P4固件架构与状态机.md` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/docs/%E9%A3%9E%E4%B9%A6%E5%AF%BC%E5%85%A5%E5%8C%85/02_P4%E5%9B%BA%E4%BB%B6%E6%9E%B6%E6%9E%84%E4%B8%8E%E7%8A%B6%E6%80%81%E6%9C%BA.md#L1-L1) · [local](../%E9%A3%9E%E4%B9%A6%E5%AF%BC%E5%85%A5%E5%8C%85/02_P4%E5%9B%BA%E4%BB%B6%E6%9E%B6%E6%9E%84%E4%B8%8E%E7%8A%B6%E6%80%81%E6%9C%BA.md)
<a id="ref-0517"></a>

- **[517]** `mmhand:docs/飞书导入包/03_控制算法与标定恢复.md` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/docs/%E9%A3%9E%E4%B9%A6%E5%AF%BC%E5%85%A5%E5%8C%85/03_%E6%8E%A7%E5%88%B6%E7%AE%97%E6%B3%95%E4%B8%8E%E6%A0%87%E5%AE%9A%E6%81%A2%E5%A4%8D.md#L1-L1) · [local](../%E9%A3%9E%E4%B9%A6%E5%AF%BC%E5%85%A5%E5%8C%85/03_%E6%8E%A7%E5%88%B6%E7%AE%97%E6%B3%95%E4%B8%8E%E6%A0%87%E5%AE%9A%E6%81%A2%E5%A4%8D.md)
<a id="ref-0518"></a>

- **[518]** `mmhand:docs/飞书导入包/04_通信硬件映射与触觉数据.md` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/docs/%E9%A3%9E%E4%B9%A6%E5%AF%BC%E5%85%A5%E5%8C%85/04_%E9%80%9A%E4%BF%A1%E7%A1%AC%E4%BB%B6%E6%98%A0%E5%B0%84%E4%B8%8E%E8%A7%A6%E8%A7%89%E6%95%B0%E6%8D%AE.md#L1-L1) · [local](../%E9%A3%9E%E4%B9%A6%E5%AF%BC%E5%85%A5%E5%8C%85/04_%E9%80%9A%E4%BF%A1%E7%A1%AC%E4%BB%B6%E6%98%A0%E5%B0%84%E4%B8%8E%E8%A7%A6%E8%A7%89%E6%95%B0%E6%8D%AE.md)
<a id="ref-0519"></a>

- **[519]** `mmhand:docs/飞书导入包/05_数据记录测试与故障排查.md` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/docs/%E9%A3%9E%E4%B9%A6%E5%AF%BC%E5%85%A5%E5%8C%85/05_%E6%95%B0%E6%8D%AE%E8%AE%B0%E5%BD%95%E6%B5%8B%E8%AF%95%E4%B8%8E%E6%95%85%E9%9A%9C%E6%8E%92%E6%9F%A5.md#L1-L1) · [local](../%E9%A3%9E%E4%B9%A6%E5%AF%BC%E5%85%A5%E5%8C%85/05_%E6%95%B0%E6%8D%AE%E8%AE%B0%E5%BD%95%E6%B5%8B%E8%AF%95%E4%B8%8E%E6%95%85%E9%9A%9C%E6%8E%92%E6%9F%A5.md)
<a id="ref-0520"></a>

- **[520]** `mmhand:docs/飞书导入包/06_文件职责与维护规则.md` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/docs/%E9%A3%9E%E4%B9%A6%E5%AF%BC%E5%85%A5%E5%8C%85/06_%E6%96%87%E4%BB%B6%E8%81%8C%E8%B4%A3%E4%B8%8E%E7%BB%B4%E6%8A%A4%E8%A7%84%E5%88%99.md#L1-L1) · [local](../%E9%A3%9E%E4%B9%A6%E5%AF%BC%E5%85%A5%E5%8C%85/06_%E6%96%87%E4%BB%B6%E8%81%8C%E8%B4%A3%E4%B8%8E%E7%BB%B4%E6%8A%A4%E8%A7%84%E5%88%99.md)
<a id="ref-0521"></a>

- **[521]** `mmhand:docs/飞书导入包/07_历史资料附录.md` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/docs/%E9%A3%9E%E4%B9%A6%E5%AF%BC%E5%85%A5%E5%8C%85/07_%E5%8E%86%E5%8F%B2%E8%B5%84%E6%96%99%E9%99%84%E5%BD%95.md#L1-L1) · [local](../%E9%A3%9E%E4%B9%A6%E5%AF%BC%E5%85%A5%E5%8C%85/07_%E5%8E%86%E5%8F%B2%E8%B5%84%E6%96%99%E9%99%84%E5%BD%95.md)
<a id="ref-0522"></a>

- **[522]** `mmhand:docs/飞书导入包/README.md` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/docs/%E9%A3%9E%E4%B9%A6%E5%AF%BC%E5%85%A5%E5%8C%85/README.md#L1-L1) · [local](../%E9%A3%9E%E4%B9%A6%E5%AF%BC%E5%85%A5%E5%8C%85/README.md)
<a id="ref-0523"></a>

- **[523]** `mmhand:docs/飞特舵机四BUS_10ms并行同步读改造说明.md` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/docs/%E9%A3%9E%E7%89%B9%E8%88%B5%E6%9C%BA%E5%9B%9BBUS_10ms%E5%B9%B6%E8%A1%8C%E5%90%8C%E6%AD%A5%E8%AF%BB%E6%94%B9%E9%80%A0%E8%AF%B4%E6%98%8E.md#L1-L1) · [local](../%E9%A3%9E%E7%89%B9%E8%88%B5%E6%9C%BA%E5%9B%9BBUS_10ms%E5%B9%B6%E8%A1%8C%E5%90%8C%E6%AD%A5%E8%AF%BB%E6%94%B9%E9%80%A0%E8%AF%B4%E6%98%8E.md)
<a id="ref-0524"></a>

- **[524]** `mmhand:manus/.gitignore` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/manus/.gitignore#L1-L1) · [local](../../manus/.gitignore)
<a id="ref-0525"></a>

- **[525]** `mmhand:manus/README.md` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/manus/README.md#L1-L1) · [local](../../manus/README.md)
<a id="ref-0526"></a>

- **[526]** `mmhand:manus/SDKClient.sln` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/manus/SDKClient.sln#L1-L1) · [local](../../manus/SDKClient.sln)
<a id="ref-0527"></a>

- **[527]** `mmhand:manus/SDKClient_Windows/.editorconfig` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/manus/SDKClient_Windows/.editorconfig#L1-L1) · [local](../../manus/SDKClient_Windows/.editorconfig)
<a id="ref-0528"></a>

- **[528]** `mmhand:manus/SDKClient_Windows/.gitignore` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/manus/SDKClient_Windows/.gitignore#L1-L1) · [local](../../manus/SDKClient_Windows/.gitignore)
<a id="ref-0529"></a>

- **[529]** `mmhand:manus/SDKClient_Windows/Main.cpp` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/manus/SDKClient_Windows/Main.cpp#L1-L1) · [local](../../manus/SDKClient_Windows/Main.cpp)
<a id="ref-0530"></a>

- **[530]** `mmhand:manus/SDKClient_Windows/PlatformSpecific/ClientPlatformSpecific.hpp` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/manus/SDKClient_Windows/PlatformSpecific/ClientPlatformSpecific.hpp#L1-L1) · [local](../../manus/SDKClient_Windows/PlatformSpecific/ClientPlatformSpecific.hpp)
<a id="ref-0531"></a>

- **[531]** `mmhand:manus/SDKClient_Windows/PlatformSpecific/Linux/ClientPlatformSpecific.cpp` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/manus/SDKClient_Windows/PlatformSpecific/Linux/ClientPlatformSpecific.cpp#L1-L1) · [local](../../manus/SDKClient_Windows/PlatformSpecific/Linux/ClientPlatformSpecific.cpp)
<a id="ref-0532"></a>

- **[532]** `mmhand:manus/SDKClient_Windows/PlatformSpecific/Linux/ClientPlatformSpecificTypes.hpp` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/manus/SDKClient_Windows/PlatformSpecific/Linux/ClientPlatformSpecificTypes.hpp#L1-L1) · [local](../../manus/SDKClient_Windows/PlatformSpecific/Linux/ClientPlatformSpecificTypes.hpp)
<a id="ref-0533"></a>

- **[533]** `mmhand:manus/SDKClient_Windows/PlatformSpecific/Windows/ClientPlatformSpecific.cpp` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/manus/SDKClient_Windows/PlatformSpecific/Windows/ClientPlatformSpecific.cpp#L1-L1) · [local](../../manus/SDKClient_Windows/PlatformSpecific/Windows/ClientPlatformSpecific.cpp)
<a id="ref-0534"></a>

- **[534]** `mmhand:manus/SDKClient_Windows/PlatformSpecific/Windows/ClientPlatformSpecificTypes.hpp` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/manus/SDKClient_Windows/PlatformSpecific/Windows/ClientPlatformSpecificTypes.hpp#L1-L1) · [local](../../manus/SDKClient_Windows/PlatformSpecific/Windows/ClientPlatformSpecificTypes.hpp)
<a id="ref-0535"></a>

- **[535]** `mmhand:manus/SDKClient_Windows/Readme.md` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/manus/SDKClient_Windows/Readme.md#L1-L1) · [local](../../manus/SDKClient_Windows/Readme.md)
<a id="ref-0536"></a>

- **[536]** `mmhand:manus/SDKClient_Windows/SDKClient.cpp` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/manus/SDKClient_Windows/SDKClient.cpp#L1-L1) · [local](../../manus/SDKClient_Windows/SDKClient.cpp)
<a id="ref-0537"></a>

- **[537]** `mmhand:manus/SDKClient_Windows/SDKClient.hpp` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/manus/SDKClient_Windows/SDKClient.hpp#L1-L1) · [local](../../manus/SDKClient_Windows/SDKClient.hpp)
<a id="ref-0538"></a>

- **[538]** `mmhand:manus/SDKClient_Windows/SDKClient.vcxproj` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/manus/SDKClient_Windows/SDKClient.vcxproj#L1-L1) · [local](../../manus/SDKClient_Windows/SDKClient.vcxproj)
<a id="ref-0539"></a>

- **[539]** `mmhand:manus/SDKClient_Windows/SDKClient.vcxproj.filters` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/manus/SDKClient_Windows/SDKClient.vcxproj.filters#L1-L1) · [local](../../manus/SDKClient_Windows/SDKClient.vcxproj.filters)
<a id="ref-0540"></a>

- **[540]** `mmhand:manus/SDKMinimalClient_Windows/.gitignore` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/manus/SDKMinimalClient_Windows/.gitignore#L1-L1) · [local](../../manus/SDKMinimalClient_Windows/.gitignore)
<a id="ref-0541"></a>

- **[541]** `mmhand:manus/SDKMinimalClient_Windows/Readme.md` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/manus/SDKMinimalClient_Windows/Readme.md#L1-L1) · [local](../../manus/SDKMinimalClient_Windows/Readme.md)
<a id="ref-0542"></a>

- **[542]** `mmhand:manus/SDKMinimalClient_Windows/SDKMinimalClient.cpp` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/manus/SDKMinimalClient_Windows/SDKMinimalClient.cpp#L1-L1) · [local](../../manus/SDKMinimalClient_Windows/SDKMinimalClient.cpp)
<a id="ref-0543"></a>

- **[543]** `mmhand:manus/SDKMinimalClient_Windows/SDKMinimalClient.hpp` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/manus/SDKMinimalClient_Windows/SDKMinimalClient.hpp#L1-L1) · [local](../../manus/SDKMinimalClient_Windows/SDKMinimalClient.hpp)
<a id="ref-0544"></a>

- **[544]** `mmhand:manus/SDKMinimalClient_Windows/SDKMinimalClient.vcxproj` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/manus/SDKMinimalClient_Windows/SDKMinimalClient.vcxproj#L1-L1) · [local](../../manus/SDKMinimalClient_Windows/SDKMinimalClient.vcxproj)
<a id="ref-0545"></a>

- **[545]** `mmhand:manus/SDKMinimalClient_Windows/SDKMinimalClient.vcxproj.filters` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/manus/SDKMinimalClient_Windows/SDKMinimalClient.vcxproj.filters#L1-L1) · [local](../../manus/SDKMinimalClient_Windows/SDKMinimalClient.vcxproj.filters)
<a id="ref-0546"></a>

- **[546]** `mmhand:manus/license/license.txt` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/manus/license/license.txt#L1-L1) · [local](../../manus/license/license.txt)
<a id="ref-0547"></a>

- **[547]** `mmhand:manus/manus_bridge_test/.gitignore` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/manus/manus_bridge_test/.gitignore#L1-L1) · [local](../../manus/manus_bridge_test/.gitignore)
<a id="ref-0548"></a>

- **[548]** `mmhand:manus/manus_bridge_test/CMakeLists.txt` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/manus/manus_bridge_test/CMakeLists.txt#L1-L1) · [local](../../manus/manus_bridge_test/CMakeLists.txt)
<a id="ref-0549"></a>

- **[549]** `mmhand:manus/manus_bridge_test/README.md` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/manus/manus_bridge_test/README.md#L1-L1) · [local](../../manus/manus_bridge_test/README.md)
<a id="ref-0550"></a>

- **[550]** `mmhand:manus/manus_bridge_test/config/manus_bridge_config.json` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/manus/manus_bridge_test/config/manus_bridge_config.json#L1-L1) · [local](../../manus/manus_bridge_test/config/manus_bridge_config.json)
<a id="ref-0551"></a>

- **[551]** `mmhand:manus/manus_bridge_test/include/README_include_manus_sdk_here.txt` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/manus/manus_bridge_test/include/README_include_manus_sdk_here.txt#L1-L1) · [local](../../manus/manus_bridge_test/include/README_include_manus_sdk_here.txt)
<a id="ref-0552"></a>

- **[552]** `mmhand:manus/manus_bridge_test/scripts/build_windows.ps1` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/manus/manus_bridge_test/scripts/build_windows.ps1#L1-L1) · [local](../../manus/manus_bridge_test/scripts/build_windows.ps1)
<a id="ref-0553"></a>

- **[553]** `mmhand:manus/manus_bridge_test/scripts/run_windows.ps1` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/manus/manus_bridge_test/scripts/run_windows.ps1#L1-L1) · [local](../../manus/manus_bridge_test/scripts/run_windows.ps1)
<a id="ref-0554"></a>

- **[554]** `mmhand:manus/manus_bridge_test/src/CsvRecorder.cpp` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/manus/manus_bridge_test/src/CsvRecorder.cpp#L1-L1) · [local](../../manus/manus_bridge_test/src/CsvRecorder.cpp)
<a id="ref-0555"></a>

- **[555]** `mmhand:manus/manus_bridge_test/src/CsvRecorder.h` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/manus/manus_bridge_test/src/CsvRecorder.h#L1-L1) · [local](../../manus/manus_bridge_test/src/CsvRecorder.h)
<a id="ref-0556"></a>

- **[556]** `mmhand:manus/manus_bridge_test/src/DataPrinter.cpp` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/manus/manus_bridge_test/src/DataPrinter.cpp#L1-L1) · [local](../../manus/manus_bridge_test/src/DataPrinter.cpp)
<a id="ref-0557"></a>

- **[557]** `mmhand:manus/manus_bridge_test/src/DataPrinter.h` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/manus/manus_bridge_test/src/DataPrinter.h#L1-L1) · [local](../../manus/manus_bridge_test/src/DataPrinter.h)
<a id="ref-0558"></a>

- **[558]** `mmhand:manus/manus_bridge_test/src/ManusClient.cpp` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/manus/manus_bridge_test/src/ManusClient.cpp#L1-L1) · [local](../../manus/manus_bridge_test/src/ManusClient.cpp)
<a id="ref-0559"></a>

- **[559]** `mmhand:manus/manus_bridge_test/src/ManusClient.h` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/manus/manus_bridge_test/src/ManusClient.h#L1-L1) · [local](../../manus/manus_bridge_test/src/ManusClient.h)
<a id="ref-0560"></a>

- **[560]** `mmhand:manus/manus_bridge_test/src/TcpSender.cpp` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/manus/manus_bridge_test/src/TcpSender.cpp#L1-L1) · [local](../../manus/manus_bridge_test/src/TcpSender.cpp)
<a id="ref-0561"></a>

- **[561]** `mmhand:manus/manus_bridge_test/src/TcpSender.h` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/manus/manus_bridge_test/src/TcpSender.h#L1-L1) · [local](../../manus/manus_bridge_test/src/TcpSender.h)
<a id="ref-0562"></a>

- **[562]** `mmhand:manus/manus_bridge_test/src/UdpSender.cpp` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/manus/manus_bridge_test/src/UdpSender.cpp#L1-L1) · [local](../../manus/manus_bridge_test/src/UdpSender.cpp)
<a id="ref-0563"></a>

- **[563]** `mmhand:manus/manus_bridge_test/src/UdpSender.h` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/manus/manus_bridge_test/src/UdpSender.h#L1-L1) · [local](../../manus/manus_bridge_test/src/UdpSender.h)
<a id="ref-0564"></a>

- **[564]** `mmhand:manus/manus_bridge_test/src/main.cpp` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/manus/manus_bridge_test/src/main.cpp#L1-L1) · [local](../../manus/manus_bridge_test/src/main.cpp)
<a id="ref-0565"></a>

- **[565]** `mmhand:manus/manus_bridge_test/third_party/ManusSDK/README_put_manus_sdk_here.txt` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/manus/manus_bridge_test/third_party/ManusSDK/README_put_manus_sdk_here.txt#L1-L1) · [local](../../manus/manus_bridge_test/third_party/ManusSDK/README_put_manus_sdk_here.txt)
<a id="ref-0566"></a>

- **[566]** `mmhand:manus/mmhand_manus_sdk/CMakeLists.txt` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/manus/mmhand_manus_sdk/CMakeLists.txt#L1-L1) · [local](../../manus/mmhand_manus_sdk/CMakeLists.txt)
<a id="ref-0567"></a>

- **[567]** `mmhand:manus/mmhand_manus_sdk/README.md` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/manus/mmhand_manus_sdk/README.md#L1-L1) · [local](../../manus/mmhand_manus_sdk/README.md)
<a id="ref-0635"></a>

- **[635]** `mmhand:manus/mmhand_manus_sdk/include/mmhand_manus_sdk.h` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/manus/mmhand_manus_sdk/include/mmhand_manus_sdk.h#L1-L1) · [local](../../manus/mmhand_manus_sdk/include/mmhand_manus_sdk.h)
<a id="ref-0636"></a>

- **[636]** `mmhand:manus/mmhand_manus_sdk/scripts/build_windows.ps1` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/manus/mmhand_manus_sdk/scripts/build_windows.ps1#L1-L1) · [local](../../manus/mmhand_manus_sdk/scripts/build_windows.ps1)
<a id="ref-0637"></a>

- **[637]** `mmhand:manus/mmhand_manus_sdk/src/mmhand_manus_sdk.cpp` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/manus/mmhand_manus_sdk/src/mmhand_manus_sdk.cpp#L1-L1) · [local](../../manus/mmhand_manus_sdk/src/mmhand_manus_sdk.cpp)
<a id="ref-0638"></a>

- **[638]** `mmhand:manus/third_party/ManusSDK/include/ManusSDK.h` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/manus/third_party/ManusSDK/include/ManusSDK.h#L1-L1) · [local](../../manus/third_party/ManusSDK/include/ManusSDK.h)
<a id="ref-0639"></a>

- **[639]** `mmhand:manus/third_party/ManusSDK/include/ManusSDKTypeInitializers.h` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/manus/third_party/ManusSDK/include/ManusSDKTypeInitializers.h#L1-L1) · [local](../../manus/third_party/ManusSDK/include/ManusSDKTypeInitializers.h)
<a id="ref-0640"></a>

- **[640]** `mmhand:manus/third_party/ManusSDK/include/ManusSDKTypes.h` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/manus/third_party/ManusSDK/include/ManusSDKTypes.h#L1-L1) · [local](../../manus/third_party/ManusSDK/include/ManusSDKTypes.h)
<a id="ref-0641"></a>

- **[641]** `mmhand:playground/Standalone/paxini_all_read_newboard/paxini_all_read_newboard.ino` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/playground/Standalone/paxini_all_read_newboard/paxini_all_read_newboard.ino#L1-L1) · [local](../../playground/Standalone/paxini_all_read_newboard/paxini_all_read_newboard.ino)
<a id="ref-0642"></a>

- **[642]** `mmhand:playground/paxini_all_read_2.12/paxini_all_read_2.12.ino` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/playground/paxini_all_read_2.12/paxini_all_read_2.12.ino#L1-L1) · [local](../../playground/paxini_all_read_2.12/paxini_all_read_2.12.ino)
<a id="ref-0643"></a>

- **[643]** `mmhand:playground/paxini_all_read_newboard.ino` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/playground/paxini_all_read_newboard.ino#L1-L1) · [local](../../playground/paxini_all_read_newboard.ino)
<a id="ref-0644"></a>

- **[644]** `mmhand:playground/paxini_dist_scan/paxini_dist_scan.ino` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/playground/paxini_dist_scan/paxini_dist_scan.ino#L1-L1) · [local](../../playground/paxini_dist_scan/paxini_dist_scan.ino)
<a id="ref-0645"></a>

- **[645]** `mmhand:playground/paxini_manual_cs_scan/paxini_manual_cs_scan.ino` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/playground/paxini_manual_cs_scan/paxini_manual_cs_scan.ino#L1-L1) · [local](../../playground/paxini_manual_cs_scan/paxini_manual_cs_scan.ino)
<a id="ref-0646"></a>

- **[646]** `mmhand:playground/paxini_spi1_p1_test/paxini_spi1_p1_test.ino` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/playground/paxini_spi1_p1_test/paxini_spi1_p1_test.ino#L1-L1) · [local](../../playground/paxini_spi1_p1_test/paxini_spi1_p1_test.ino)
<a id="ref-0647"></a>

- **[647]** `mmhand:playground/paxini_spi_scan/paxini_spi_scan.ino` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/playground/paxini_spi_scan/paxini_spi_scan.ino#L1-L1) · [local](../../playground/paxini_spi_scan/paxini_spi_scan.ino)
<a id="ref-0648"></a>

- **[648]** `mmhand:playground/tactile_sim_visualizer/README.md` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/playground/tactile_sim_visualizer/README.md#L1-L1) · [local](../../playground/tactile_sim_visualizer/README.md)
<a id="ref-0649"></a>

- **[649]** `mmhand:playground/tactile_sim_visualizer/simulate_tactile_dot_matrix.py` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/playground/tactile_sim_visualizer/simulate_tactile_dot_matrix.py#L1-L1) · [local](../../playground/tactile_sim_visualizer/simulate_tactile_dot_matrix.py)
<a id="ref-0650"></a>

- **[650]** `mmhand:pytest.ini` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/pytest.ini#L1-L1) · [local](../../pytest.ini)
<a id="ref-0651"></a>

- **[651]** `mmhand:start_mmhand.cmd` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/start_mmhand.cmd#L1-L1) · [local](../../start_mmhand.cmd)
<a id="ref-0652"></a>

- **[652]** `mmhand:tests/conftest.py` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/tests/conftest.py#L1-L1) · [local](../../tests/conftest.py)
<a id="ref-0653"></a>

- **[653]** `mmhand:tests/desktop/algorithm/test_replay_controller.py` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/tests/desktop/algorithm/test_replay_controller.py#L1-L1) · [local](../../tests/desktop/algorithm/test_replay_controller.py)
<a id="ref-0654"></a>

- **[654]** `mmhand:tests/desktop/algorithm/test_replay_csv_loader.py` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/tests/desktop/algorithm/test_replay_csv_loader.py#L1-L1) · [local](../../tests/desktop/algorithm/test_replay_csv_loader.py)
<a id="ref-0655"></a>

- **[655]** `mmhand:tests/desktop/app/test_cli_monitor.py` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/tests/desktop/app/test_cli_monitor.py#L1-L1) · [local](../../tests/desktop/app/test_cli_monitor.py)
<a id="ref-0656"></a>

- **[656]** `mmhand:tests/desktop/app/test_entry.py` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/tests/desktop/app/test_entry.py#L1-L1) · [local](../../tests/desktop/app/test_entry.py)
<a id="ref-0657"></a>

- **[657]** `mmhand:tests/desktop/app/test_ui_runner_ros2.py` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/tests/desktop/app/test_ui_runner_ros2.py#L1-L1) · [local](../../tests/desktop/app/test_ui_runner_ros2.py)
<a id="ref-0658"></a>

- **[658]** `mmhand:tests/desktop/app/test_windows_launcher.py` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/tests/desktop/app/test_windows_launcher.py#L1-L1) · [local](../../tests/desktop/app/test_windows_launcher.py)
<a id="ref-0659"></a>

- **[659]** `mmhand:tests/desktop/core/test_angle_mapping.py` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/tests/desktop/core/test_angle_mapping.py#L1-L1) · [local](../../tests/desktop/core/test_angle_mapping.py)
<a id="ref-0660"></a>

- **[660]** `mmhand:tests/desktop/core/test_calib_snapshot_current.py` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/tests/desktop/core/test_calib_snapshot_current.py#L1-L1) · [local](../../tests/desktop/core/test_calib_snapshot_current.py)
<a id="ref-0661"></a>

- **[661]** `mmhand:tests/desktop/core/test_calibration_diag_protocol.py` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/tests/desktop/core/test_calibration_diag_protocol.py#L1-L1) · [local](../../tests/desktop/core/test_calibration_diag_protocol.py)
<a id="ref-0662"></a>

- **[662]** `mmhand:tests/desktop/core/test_calibration_diagnostics.py` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/tests/desktop/core/test_calibration_diagnostics.py#L1-L1) · [local](../../tests/desktop/core/test_calibration_diagnostics.py)
<a id="ref-0663"></a>

- **[663]** `mmhand:tests/desktop/core/test_calibration_snapshot.py` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/tests/desktop/core/test_calibration_snapshot.py#L1-L1) · [local](../../tests/desktop/core/test_calibration_snapshot.py)
<a id="ref-0664"></a>

- **[664]** `mmhand:tests/desktop/core/test_calibration_workflow.py` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/tests/desktop/core/test_calibration_workflow.py#L1-L1) · [local](../../tests/desktop/core/test_calibration_workflow.py)
<a id="ref-0665"></a>

- **[665]** `mmhand:tests/desktop/core/test_comm_layer_mapped_angles.py` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/tests/desktop/core/test_comm_layer_mapped_angles.py#L1-L1) · [local](../../tests/desktop/core/test_comm_layer_mapped_angles.py)
<a id="ref-0666"></a>

- **[666]** `mmhand:tests/desktop/core/test_communication_components.py` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/tests/desktop/core/test_communication_components.py#L1-L1) · [local](../../tests/desktop/core/test_communication_components.py)
<a id="ref-0667"></a>

- **[667]** `mmhand:tests/desktop/core/test_core_logic.py` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/tests/desktop/core/test_core_logic.py#L1-L1) · [local](../../tests/desktop/core/test_core_logic.py)
<a id="ref-0668"></a>

- **[668]** `mmhand:tests/desktop/core/test_device_state_store.py` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/tests/desktop/core/test_device_state_store.py#L1-L1) · [local](../../tests/desktop/core/test_device_state_store.py)
<a id="ref-0669"></a>

- **[669]** `mmhand:tests/desktop/core/test_direct_hold_and_release.py` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/tests/desktop/core/test_direct_hold_and_release.py#L1-L1) · [local](../../tests/desktop/core/test_direct_hold_and_release.py)
<a id="ref-0670"></a>

- **[670]** `mmhand:tests/desktop/core/test_feedback_command_order.py` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/tests/desktop/core/test_feedback_command_order.py#L1-L1) · [local](../../tests/desktop/core/test_feedback_command_order.py)
<a id="ref-0671"></a>

- **[671]** `mmhand:tests/desktop/core/test_joint_target_adapter.py` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/tests/desktop/core/test_joint_target_adapter.py#L1-L1) · [local](../../tests/desktop/core/test_joint_target_adapter.py)
<a id="ref-0672"></a>

- **[672]** `mmhand:tests/desktop/core/test_joint_workspace_guard.py` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/tests/desktop/core/test_joint_workspace_guard.py#L1-L1) · [local](../../tests/desktop/core/test_joint_workspace_guard.py)
<a id="ref-0673"></a>

- **[673]** `mmhand:tests/desktop/core/test_manual_reference_crc.py` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/tests/desktop/core/test_manual_reference_crc.py#L1-L1) · [local](../../tests/desktop/core/test_manual_reference_crc.py)
<a id="ref-0674"></a>

- **[674]** `mmhand:tests/desktop/core/test_manual_reference_protocol.py` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/tests/desktop/core/test_manual_reference_protocol.py#L1-L1) · [local](../../tests/desktop/core/test_manual_reference_protocol.py)
<a id="ref-0675"></a>

- **[675]** `mmhand:tests/desktop/core/test_manual_scaled_mapping_v3.py` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/tests/desktop/core/test_manual_scaled_mapping_v3.py#L1-L1) · [local](../../tests/desktop/core/test_manual_scaled_mapping_v3.py)
<a id="ref-0676"></a>

- **[676]** `mmhand:tests/desktop/core/test_motion_sampler.py` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/tests/desktop/core/test_motion_sampler.py#L1-L1) · [local](../../tests/desktop/core/test_motion_sampler.py)
<a id="ref-0677"></a>

- **[677]** `mmhand:tests/desktop/core/test_packet_recording.py` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/tests/desktop/core/test_packet_recording.py#L1-L1) · [local](../../tests/desktop/core/test_packet_recording.py)
<a id="ref-0678"></a>

- **[678]** `mmhand:tests/desktop/core/test_pose_urdf_mapping.py` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/tests/desktop/core/test_pose_urdf_mapping.py#L1-L1) · [local](../../tests/desktop/core/test_pose_urdf_mapping.py)
<a id="ref-0679"></a>

- **[679]** `mmhand:tests/desktop/core/test_product_fingerprint_protocol.py` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/tests/desktop/core/test_product_fingerprint_protocol.py#L1-L1) · [local](../../tests/desktop/core/test_product_fingerprint_protocol.py)
<a id="ref-0680"></a>

- **[680]** `mmhand:tests/desktop/core/test_recording_timestamps.py` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/tests/desktop/core/test_recording_timestamps.py#L1-L1) · [local](../../tests/desktop/core/test_recording_timestamps.py)
<a id="ref-0681"></a>

- **[681]** `mmhand:tests/desktop/core/test_scoped_multiturn_protocol.py` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/tests/desktop/core/test_scoped_multiturn_protocol.py#L1-L1) · [local](../../tests/desktop/core/test_scoped_multiturn_protocol.py)
<a id="ref-0682"></a>

- **[682]** `mmhand:tests/desktop/core/test_tactile_summary_protocol.py` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/tests/desktop/core/test_tactile_summary_protocol.py#L1-L1) · [local](../../tests/desktop/core/test_tactile_summary_protocol.py)
<a id="ref-0683"></a>

- **[683]** `mmhand:tests/desktop/ros2/test_joint_targets.py` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/tests/desktop/ros2/test_joint_targets.py#L1-L1) · [local](../../tests/desktop/ros2/test_joint_targets.py)
<a id="ref-0684"></a>

- **[684]** `mmhand:tests/desktop/ros2/test_raw_ik_target.py` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/tests/desktop/ros2/test_raw_ik_target.py#L1-L1) · [local](../../tests/desktop/ros2/test_raw_ik_target.py)
<a id="ref-0685"></a>

- **[685]** `mmhand:tests/desktop/ui/test_calibration_failure_popup.py` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/tests/desktop/ui/test_calibration_failure_popup.py#L1-L1) · [local](../../tests/desktop/ui/test_calibration_failure_popup.py)
<a id="ref-0686"></a>

- **[686]** `mmhand:tests/desktop/ui/test_calibration_warning_popup.py` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/tests/desktop/ui/test_calibration_warning_popup.py#L1-L1) · [local](../../tests/desktop/ui/test_calibration_warning_popup.py)
<a id="ref-0687"></a>

- **[687]** `mmhand:tests/desktop/ui/test_control_panel_modules.py` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/tests/desktop/ui/test_control_panel_modules.py#L1-L1) · [local](../../tests/desktop/ui/test_control_panel_modules.py)
<a id="ref-0688"></a>

- **[688]** `mmhand:tests/desktop/ui/test_device_check_view.py` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/tests/desktop/ui/test_device_check_view.py#L1-L1) · [local](../../tests/desktop/ui/test_device_check_view.py)
<a id="ref-0689"></a>

- **[689]** `mmhand:tests/desktop/ui/test_enable_scope.py` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/tests/desktop/ui/test_enable_scope.py#L1-L1) · [local](../../tests/desktop/ui/test_enable_scope.py)
<a id="ref-0690"></a>

- **[690]** `mmhand:tests/desktop/ui/test_encoder_common_bus_guard.py` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/tests/desktop/ui/test_encoder_common_bus_guard.py#L1-L1) · [local](../../tests/desktop/ui/test_encoder_common_bus_guard.py)
<a id="ref-0691"></a>

- **[691]** `mmhand:tests/desktop/ui/test_feedback_collision_status.py` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/tests/desktop/ui/test_feedback_collision_status.py#L1-L1) · [local](../../tests/desktop/ui/test_feedback_collision_status.py)
<a id="ref-0692"></a>

- **[692]** `mmhand:tests/desktop/ui/test_joint_target_edit_persistence.py` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/tests/desktop/ui/test_joint_target_edit_persistence.py#L1-L1) · [local](../../tests/desktop/ui/test_joint_target_edit_persistence.py)
<a id="ref-0693"></a>

- **[693]** `mmhand:tests/desktop/ui/test_partial_calibration_result.py` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/tests/desktop/ui/test_partial_calibration_result.py#L1-L1) · [local](../../tests/desktop/ui/test_partial_calibration_result.py)
<a id="ref-0694"></a>

- **[694]** `mmhand:tests/desktop/ui/test_pose_urdf_reload.py` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/tests/desktop/ui/test_pose_urdf_reload.py#L1-L1) · [local](../../tests/desktop/ui/test_pose_urdf_reload.py)
<a id="ref-0695"></a>

- **[695]** `mmhand:tests/desktop/ui/test_replay_view.py` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/tests/desktop/ui/test_replay_view.py#L1-L1) · [local](../../tests/desktop/ui/test_replay_view.py)
<a id="ref-0696"></a>

- **[696]** `mmhand:tests/desktop/ui/test_rope_init_progress.py` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/tests/desktop/ui/test_rope_init_progress.py#L1-L1) · [local](../../tests/desktop/ui/test_rope_init_progress.py)
<a id="ref-0697"></a>

- **[697]** `mmhand:tests/desktop/ui/test_scrollable_message_dialog.py` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/tests/desktop/ui/test_scrollable_message_dialog.py#L1-L1) · [local](../../tests/desktop/ui/test_scrollable_message_dialog.py)
<a id="ref-0698"></a>

- **[698]** `mmhand:tests/desktop/ui/test_stop_paths.py` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/tests/desktop/ui/test_stop_paths.py#L1-L1) · [local](../../tests/desktop/ui/test_stop_paths.py)
<a id="ref-0699"></a>

- **[699]** `mmhand:tests/desktop/ui/test_tactile_view.py` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/tests/desktop/ui/test_tactile_view.py#L1-L1) · [local](../../tests/desktop/ui/test_tactile_view.py)
<a id="ref-0700"></a>

- **[700]** `mmhand:tests/desktop/ui/test_view_menu.py` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/tests/desktop/ui/test_view_menu.py#L1-L1) · [local](../../tests/desktop/ui/test_view_menu.py)
<a id="ref-0701"></a>

- **[701]** `mmhand:tests/desktop/visualize/test_analyze_single_motor_pid.py` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/tests/desktop/visualize/test_analyze_single_motor_pid.py#L1-L1) · [local](../../tests/desktop/visualize/test_analyze_single_motor_pid.py)
<a id="ref-0702"></a>

- **[702]** `mmhand:tests/desktop/visualize/test_plot_step_response.py` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/tests/desktop/visualize/test_plot_step_response.py#L1-L1) · [local](../../tests/desktop/visualize/test_plot_step_response.py)
<a id="ref-0703"></a>

- **[703]** `mmhand:tests/tools/test_desktop_architecture.py` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/tests/tools/test_desktop_architecture.py#L1-L1) · [local](../../tests/tools/test_desktop_architecture.py)
<a id="ref-0704"></a>

- **[704]** `mmhand:tests/tools/test_generate_robot_model.py` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/tests/tools/test_generate_robot_model.py#L1-L1) · [local](../../tests/tools/test_generate_robot_model.py)
<a id="ref-0705"></a>

- **[705]** `mmhand:tests/tools/test_generate_upper_protocol.py` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/tests/tools/test_generate_upper_protocol.py#L1-L1) · [local](../../tests/tools/test_generate_upper_protocol.py)
<a id="ref-0706"></a>

- **[706]** `mmhand:tests/tools/test_p4_host_control_lease_policy.py` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/tests/tools/test_p4_host_control_lease_policy.py#L1-L1) · [local](../../tests/tools/test_p4_host_control_lease_policy.py)
<a id="ref-0707"></a>

- **[707]** `mmhand:tests/tools/test_sync_structure_urdf_adapter.py` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/tests/tools/test_sync_structure_urdf_adapter.py#L1-L1) · [local](../../tests/tools/test_sync_structure_urdf_adapter.py)
<a id="ref-0708"></a>

- **[708]** `mmhand:tools/build_protocol_docx.py` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/tools/build_protocol_docx.py#L1-L1) · [local](../../tools/build_protocol_docx.py)
<a id="ref-0709"></a>

- **[709]** `mmhand:tools/capture_dist_scan.ps1` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/tools/capture_dist_scan.ps1#L1-L1) · [local](../../tools/capture_dist_scan.ps1)
<a id="ref-0710"></a>

- **[710]** `mmhand:tools/capture_manual_cs_scan.ps1` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/tools/capture_manual_cs_scan.ps1#L1-L1) · [local](../../tools/capture_manual_cs_scan.ps1)
<a id="ref-0711"></a>

- **[711]** `mmhand:tools/capture_tactile_map.ps1` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/tools/capture_tactile_map.ps1#L1-L1) · [local](../../tools/capture_tactile_map.ps1)
<a id="ref-0712"></a>

- **[712]** `mmhand:tools/check_desktop_architecture.py` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/tools/check_desktop_architecture.py#L1-L1) · [local](../../tools/check_desktop_architecture.py)
<a id="ref-0713"></a>

- **[713]** `mmhand:tools/clone_structure_repo.ps1` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/tools/clone_structure_repo.ps1#L1-L1) · [local](../../tools/clone_structure_repo.ps1)
<a id="ref-0714"></a>

- **[714]** `mmhand:tools/create_desktop_shortcut.ps1` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/tools/create_desktop_shortcut.ps1#L1-L1) · [local](../../tools/create_desktop_shortcut.ps1)
<a id="ref-0715"></a>

- **[715]** `mmhand:tools/desktop_architecture/apply_desktop_headers.py` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/tools/desktop_architecture/apply_desktop_headers.py#L1-L1) · [local](../../tools/desktop_architecture/apply_desktop_headers.py)
<a id="ref-0716"></a>

- **[716]** `mmhand:tools/generate_handover_index.py` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/tools/generate_handover_index.py#L1-L1) · [local](../../tools/generate_handover_index.py)
<a id="ref-0717"></a>

- **[717]** `mmhand:tools/generate_p4_safety_catalog.py` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/tools/generate_p4_safety_catalog.py#L1-L1) · [local](../../tools/generate_p4_safety_catalog.py)
<a id="ref-0718"></a>

- **[718]** `mmhand:tools/generate_p4_urdf_limits.py` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/tools/generate_p4_urdf_limits.py#L1-L1) · [local](../../tools/generate_p4_urdf_limits.py)
<a id="ref-0719"></a>

- **[719]** `mmhand:tools/generate_robot_model.py` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/tools/generate_robot_model.py#L1-L1) · [local](../../tools/generate_robot_model.py)
<a id="ref-0720"></a>

- **[720]** `mmhand:tools/generate_upper_protocol.py` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/tools/generate_upper_protocol.py#L1-L1) · [local](../../tools/generate_upper_protocol.py)
<a id="ref-0721"></a>

- **[721]** `mmhand:tools/install_git_hooks.ps1` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/tools/install_git_hooks.ps1#L1-L1) · [local](../../tools/install_git_hooks.ps1)
<a id="ref-0722"></a>

- **[722]** `mmhand:tools/link_urdf.ps1` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/tools/link_urdf.ps1#L1-L1) · [local](../../tools/link_urdf.ps1)
<a id="ref-0723"></a>

- **[723]** `mmhand:tools/p4_architecture/check_p4_architecture.ps1` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/tools/p4_architecture/check_p4_architecture.ps1#L1-L1) · [local](../../tools/p4_architecture/check_p4_architecture.ps1)
<a id="ref-0724"></a>

- **[724]** `mmhand:tools/p4_architecture/check_safety_rule_closure.py` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/tools/p4_architecture/check_safety_rule_closure.py#L1-L1) · [local](../../tools/p4_architecture/check_safety_rule_closure.py)
<a id="ref-0725"></a>

- **[725]** `mmhand:tools/p4_debug/build_debug.ps1` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/tools/p4_debug/build_debug.ps1#L1-L1) · [local](../../tools/p4_debug/build_debug.ps1)
<a id="ref-0726"></a>

- **[726]** `mmhand:tools/p4_debug/p4_debug_env.ps1` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/tools/p4_debug/p4_debug_env.ps1#L1-L1) · [local](../../tools/p4_debug/p4_debug_env.ps1)
<a id="ref-0727"></a>

- **[727]** `mmhand:tools/p4_debug/start_openocd.ps1` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/tools/p4_debug/start_openocd.ps1#L1-L1) · [local](../../tools/p4_debug/start_openocd.ps1)
<a id="ref-0728"></a>

- **[728]** `mmhand:tools/p4_debug/upload_debug.ps1` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/tools/p4_debug/upload_debug.ps1#L1-L1) · [local](../../tools/p4_debug/upload_debug.ps1)
<a id="ref-0729"></a>

- **[729]** `mmhand:tools/p4_debug/verify_debug_env.ps1` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/tools/p4_debug/verify_debug_env.ps1#L1-L1) · [local](../../tools/p4_debug/verify_debug_env.ps1)
<a id="ref-0730"></a>

- **[730]** `mmhand:tools/sync_structure_urdf_adapter.py` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/tools/sync_structure_urdf_adapter.py#L1-L1) · [local](../../tools/sync_structure_urdf_adapter.py)
<a id="ref-0731"></a>

- **[731]** `mmhand:tools/update_structure_urdf.ps1` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/tools/update_structure_urdf.ps1#L1-L1) · [local](../../tools/update_structure_urdf.ps1)
<a id="ref-0732"></a>

- **[732]** `mmhand:tools/validate_urdf.py` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/tools/validate_urdf.py#L1-L1) · [local](../../tools/validate_urdf.py)
<a id="ref-0733"></a>

- **[733]** `mmhand:wuji_hand_ws/.gitignore` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/wuji_hand_ws/.gitignore#L1-L1) · [local](../../wuji_hand_ws/.gitignore)
<a id="ref-0734"></a>

- **[734]** `mmhand:帕西尼 PX-6AX GEN3 系列多维触觉传感器用户手册_V1.0.5.pdf` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/%E5%B8%95%E8%A5%BF%E5%B0%BC%20PX-6AX%20GEN3%20%E7%B3%BB%E5%88%97%E5%A4%9A%E7%BB%B4%E8%A7%A6%E8%A7%89%E4%BC%A0%E6%84%9F%E5%99%A8%E7%94%A8%E6%88%B7%E6%89%8B%E5%86%8C_V1.0.5.pdf#L1-L1) · [local](../../%E5%B8%95%E8%A5%BF%E5%B0%BC%20PX-6AX%20GEN3%20%E7%B3%BB%E5%88%97%E5%A4%9A%E7%BB%B4%E8%A7%A6%E8%A7%89%E4%BC%A0%E6%84%9F%E5%99%A8%E7%94%A8%E6%88%B7%E6%89%8B%E5%86%8C_V1.0.5.pdf)
<a id="ref-0735"></a>

- **[735]** `mmhand:.vscode/c_cpp_properties.json` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/.vscode/c_cpp_properties.json#L1-L1) · [local](../../.vscode/c_cpp_properties.json)
<a id="ref-0736"></a>

- **[736]** `mmhand:.vscode/launch.json` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/.vscode/launch.json#L1-L1) · [local](../../.vscode/launch.json)
<a id="ref-0737"></a>

- **[737]** `mmhand:.vscode/settings.json` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/.vscode/settings.json#L1-L1) · [local](../../.vscode/settings.json)
<a id="ref-0738"></a>

- **[738]** `mmhand:.vscode/tasks.json` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/.vscode/tasks.json#L1-L1) · [local](../../.vscode/tasks.json)
<a id="ref-0739"></a>

- **[739]** `mmhand:ESP32-P4/.vscode/settings.json` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/.vscode/settings.json#L1-L1) · [local](../../ESP32-P4/.vscode/settings.json)
<a id="ref-0740"></a>

- **[740]** `mmhand:ESP32-P4/.vscode/tasks.json` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/.vscode/tasks.json#L1-L1) · [local](../../ESP32-P4/.vscode/tasks.json)
<a id="ref-0741"></a>

- **[741]** `mmhand:ESP32-P4/ServoBoardMain/src/platform/log/FirmwareLog.cpp` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/platform/log/FirmwareLog.cpp#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/platform/log/FirmwareLog.cpp)
<a id="ref-0742"></a>

- **[742]** `mmhand:ESP32-P4/ServoBoardMain/src/platform/log/FirmwareLog.h` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/ESP32-P4/ServoBoardMain/src/platform/log/FirmwareLog.h#L1-L1) · [local](../../ESP32-P4/ServoBoardMain/src/platform/log/FirmwareLog.h)
<a id="ref-0743"></a>

- **[743]** `mmhand:manus/mmhand_manus_sdk/build/windows/CMakeFiles/3.31.6-msvc6/CompilerIdCXX/CMakeCXXCompilerId.cpp` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/manus/mmhand_manus_sdk/build/windows/CMakeFiles/3.31.6-msvc6/CompilerIdCXX/CMakeCXXCompilerId.cpp#L1-L1) · [local](../../manus/mmhand_manus_sdk/build/windows/CMakeFiles/3.31.6-msvc6/CompilerIdCXX/CMakeCXXCompilerId.cpp)
<a id="ref-0744"></a>

- **[744]** `mmhand:manus/mmhand_manus_sdk/build/windows/CMakeFiles/CMakeConfigureLog.yaml` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/manus/mmhand_manus_sdk/build/windows/CMakeFiles/CMakeConfigureLog.yaml#L1-L1) · [local](../../manus/mmhand_manus_sdk/build/windows/CMakeFiles/CMakeConfigureLog.yaml)
<a id="ref-0745"></a>

- **[745]** `teleop:config.py` · [GitHub](https://github.com/mm-hand/mmhand_teleop/blob/45eb698ba97c2559cad93db159a15877f93e8851/config.py#L1-L1) · [local](../../../mmhand_teleop/config.py)
  - 相关源码行：[L119–L119 GitHub](https://github.com/mm-hand/mmhand_teleop/blob/45eb698ba97c2559cad93db159a15877f93e8851/config.py#L119-L119) · [local](../../../mmhand_teleop/config.py)
<a id="ref-0746"></a>

- **[746]** `teleop:ros.py` · [GitHub](https://github.com/mm-hand/mmhand_teleop/blob/45eb698ba97c2559cad93db159a15877f93e8851/ros.py#L1-L1) · [local](../../../mmhand_teleop/ros.py)
  - 相关源码行：[L20–L20 GitHub](https://github.com/mm-hand/mmhand_teleop/blob/45eb698ba97c2559cad93db159a15877f93e8851/ros.py#L20-L20) · [local](../../../mmhand_teleop/ros.py), [L25–L25 GitHub](https://github.com/mm-hand/mmhand_teleop/blob/45eb698ba97c2559cad93db159a15877f93e8851/ros.py#L25-L25) · [local](../../../mmhand_teleop/ros.py)
<a id="ref-0747"></a>

- **[747]** `teleop:track.py` · [GitHub](https://github.com/mm-hand/mmhand_teleop/blob/45eb698ba97c2559cad93db159a15877f93e8851/track.py#L1-L1) · [local](../../../mmhand_teleop/track.py)
  - 相关源码行：[L15–L15 GitHub](https://github.com/mm-hand/mmhand_teleop/blob/45eb698ba97c2559cad93db159a15877f93e8851/track.py#L15-L15) · [local](../../../mmhand_teleop/track.py)
<a id="ref-0748"></a>

- **[748]** `mmhand:manus/mmhand_manus_sdk/build/windows/CMakeFiles/3.31.6-msvc6/CMakeCXXCompiler.cmake` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/manus/mmhand_manus_sdk/build/windows/CMakeFiles/3.31.6-msvc6/CMakeCXXCompiler.cmake#L1-L1) · [local](../../manus/mmhand_manus_sdk/build/windows/CMakeFiles/3.31.6-msvc6/CMakeCXXCompiler.cmake)
<a id="ref-0749"></a>

- **[749]** `mmhand:manus/mmhand_manus_sdk/build/windows/CMakeFiles/3.31.6-msvc6/CMakeRCCompiler.cmake` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/manus/mmhand_manus_sdk/build/windows/CMakeFiles/3.31.6-msvc6/CMakeRCCompiler.cmake#L1-L1) · [local](../../manus/mmhand_manus_sdk/build/windows/CMakeFiles/3.31.6-msvc6/CMakeRCCompiler.cmake)
<a id="ref-0750"></a>

- **[750]** `mmhand:manus/mmhand_manus_sdk/build/windows/CMakeFiles/3.31.6-msvc6/CMakeSystem.cmake` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/manus/mmhand_manus_sdk/build/windows/CMakeFiles/3.31.6-msvc6/CMakeSystem.cmake#L1-L1) · [local](../../manus/mmhand_manus_sdk/build/windows/CMakeFiles/3.31.6-msvc6/CMakeSystem.cmake)
<a id="ref-0751"></a>

- **[751]** `mmhand:manus/mmhand_manus_sdk/build/windows/cmake_install.cmake` · [GitHub](https://github.com/mm-hand/mmhand/blob/65e8e63f8b570c6b099c8b65dd37d796dcb66bb9/manus/mmhand_manus_sdk/build/windows/cmake_install.cmake#L1-L1) · [local](../../manus/mmhand_manus_sdk/build/windows/cmake_install.cmake)
