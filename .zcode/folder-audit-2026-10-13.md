# mmhand 仓库全量审计（2026-10-13，分支 mmhand_robin）

行数 = `git ls-files` 跟踪文件的实际行数（wc -l）。共 510 个跟踪文件，约 10.7 万行。
【生成】= 由 tools/ 下生成器产出，禁止手改；【AI】= docs/ai 注释流水线产物。

## 0. 顶层总览

```
mmhand/                        107,643 行   510 文件
├── .githooks/                     5 行    1   pre-commit → 门禁入口（git hooksPath 指向这里）
├── .github/workflows/check.yml   27 行    1   CI：tools/check.py + 仅 P4 编译
├── docs/                      28,302 行   72
│   ├── ai/                    28,110 行   71   AI 代码指南生成流水线（含 6 个发布产物）
│   └── debug/                    192 行    1   esp32-jtag-debugging.md（JTAG 调试全指南）
├── ESP32-P4/                  27,681 行  227   P4 主控固件（Arduino sketch，扁平化布局）
│   ├── src/                   27,461 行  219   七层架构 config/data/generated/hal/safety/system/tasks
│   ├── tools/                    149 行    2   BOM 清理脚本
│   ├── .vscode/                   45 行    2   Arduino 编译任务
│   ├── .githooks/pre-commit        5 行    1   ⚠ 死钩子（见 §4）
│   └── ESP32-P4.ino 等            21 行    3   sketch 入口/编辑器配置
├── ESP32-S3/                   2,553 行   19   S3 手背传感固件
│   ├── src/                   2,488 行   18   hal/tasks/system/config 四层
│   └── ESP32-S3.ino              65 行    1   sketch 入口
├── desktop/                   23,832 行   81   NiceGUI 桌面上位机
│   ├── core/                 17,923 行   43   业务核心（协议/通信/标定/状态/录制）
│   ├── resources/             3,130 行    7   配置 JSON + URDF（生成器输入/输出）
│   ├── replay/                1,425 行    6   CSV 回放子系统
│   ├── ui/                      741 行   13   NiceGUI 页面
│   ├── ros2/                    369 行    3   ROS2 订阅节点
│   ├── app/                     120 行    3   入口与 ui_runner
│   └── examples/                 64 行    2   ROS2 发送示例
├── tests/                     16,107 行   80   pytest 套件（1264 用例）
│   ├── desktop/               12,768 行   57   core 47 / ui 3 / app 3 / algorithm 2 / ros2 2
│   ├── tools/                  2,695 行   11   生成器/架构/门禁自身 + 5 个 P4 宿主机 C++ 测试
│   ├── firmware/                 638 行   11   协议契约测试（Python 侧模型）
│   └── conftest.py                6 行    1   注入仓库根到 sys.path
├── tools/                      3,276 行   21   生成器/门禁/架构检查/JTAG 调试
│   ├── p4_architecture/          729 行    2   P4 七层架构门禁 + 安全规则闭环探针
│   ├── debug/                    531 行    6   OpenOCD+GDB 调试工具（-Target p4|s3）
│   └── 根                        2,016 行   13   三个生成器、precommit_checks 等
└── 根文件                      6,801 行    8   AI_README.zh/en(3305×2)、AGENTS.md、pytest.ini、
                                            .gitignore、.gitattributes、.zcodeignore、requirements-test.txt
```

未跟踪（gitignore，不进门禁）：`.debug/`（调试构建）、`run_data/`（运行数据）、`.pytest_cache/`、`.zcode/`。

---

## 1. 根目录与基础设施

| 文件 | 行数 | 用途 |
|---|---|---|
| AI_README.zh.md / AI_README.en.md | 3305×2 | 【生成】docs/ai 发布后复制到根目录的 AI 代码指南（⚠ 仍引用已删文件，未再生） |
| AGENTS.md | 39 | 代理工作区说明：docs/ai 规程指引 + ESP32 JTAG 调试速查与五条铁律 |
| pytest.ini | 10 | testpaths=tests（⚠ docs/ai/build 的测试因此不被收集）；禁用 launch/ros 插件 |
| requirements-test.txt | 2 | pytest==8.4.2（门禁锁版本） |
| .gitignore | 55 | 缓存/调试构建/运行数据/历史实验目录 |
| .gitattributes | 5 | py/sh/githooks 强制 LF；URDF 特例 |
| .zcodeignore | 80 | ZCode 会话忽略表（含历史残留模式） |
| .githooks/pre-commit | 5 | `exec python tools/precommit_checks.py --staged`，只读、不代为暂存 |
| .github/workflows/check.yml | 27 | CI 双 job：① tools/check.py（门禁去掉 --staged）② arduino-cli 只编译 P4 |

## 2. docs/ai —— AI 代码指南流水线（71 文件，28,110 行）

### build/ 生成器（12 文件，3,485 行，纯确定性 Python，不调用任何模型）
| 文件 | 行数 | 用途 |
|---|---|---|
| make.py | 356 | 主入口：--prepare（产出图任务包）/ --validate / --publish / --estimate；覆盖率硬校验、生成器 --check、发布原子替换 |
| extract.py | 637 | 从已提交源码静态抽图：AST 解析 UI 入口函数（if 条件+调用）、正则抽 UI 信号绑定、P4 状态机 case/setSystemState、安全规则宏与调用点、任务周期/通道、协议 schema、CAN ID（⚠ 硬编码旧路径 ServoBoardMain） |
| source.py | 186 | Sources：按 commit 读 git 文件、证据（repo,path,start,end,text）、diff 范围、证据注册表 |
| source_scope.py | 376 | source_scope.json 的加载/校验：configured_path、候选发现、extensions/exclusions 处置 |
| explain.py | 188 | fact cards：每节点/边/命令的 evidence 上下文、变量定义定位、changed+diff；15 条 README 命令清单 |
| graph_explain.py | 344 | 任务包打包（bundle_hash 指纹、required_ids、响应 schema）+ 注释校验全规则（长度/模板腔/原始条件/true-false 双分支/标识符防幻觉） |
| graph_context.py | 182 | build_bundle：图内源码片段+清单的打包与哈希 |
| render.py | 480 | 渲染 AI_README.{zh,en}.md 与 architecture/behavior.{zh,en}.html（cytoscape 图）（⚠ 含旧路径） |
| test_pipeline.py / test_guide.py / test_explain.py / test_source_scope.py | 77/73/210/196 | 流水线自身的单元测试（⚠ 不在 pytest testpaths 内，仅手工跑，共 21 用例） |

### annotations/（42 个 JSON，约 1.39 万行）【AI 手写、工具校验】
每个 `<graph_id>.json` 对应一张图的完整双语解释（schema_version/instruction_version/bundle_hash/overview/glossary/items）。分组：
- 行为图 12：behavior_overview、connect、control、device_check、joint_channels、motor_channels、calibration、replay、ros2、recovery、collision、p4_state
- UI 图 12：ui_controls + ui_ui_app、ui_ui_windows、ui_ui_control_modes、ui_ui_control_tables、ui_ui_control_test_panel、ui_ui_control_device、ui_ui_tactile_view、ui_ui_device_check_view、ui_ui_replay_view、ui_ui_pose_view、ui_ui_joint_plot_view（⚠ 对应旧 Qt 界面文件，现 UI 已是 NiceGUI web/）
- 安全图 8：safety_rules + safety_{actuator,calibration,communication,control,recovery,runtime,system}
- 架构图 10：architecture_overview、arch_p4、arch_s3、arch_desktop、arch_can、arch_ros、arch_serial、arch_serial_commands、arch_serial_packets、readme_commands

### 其余
| 文件 | 用途 |
|---|---|
| GENERATE.md（227） | 流水线规程：prepare→annotate→validate→publish + 发布后复制 README 到根目录 |
| source_scope.json（586） | 【配置】文档覆盖范围的唯一配置：paths 路径映射/entrypoints/p4_tasks/s3_tasks/command_sources/extensions/exclusions/audit（⚠ 全部指向旧布局） |
| source_refs.json（681） | 【生成】路径→编号注册表 |
| snapshot.json（147） | 【生成】上次发布的 commit 指纹与覆盖率（下次的默认 baseline） |
| coverage_report.json（166） / annotation_report.json（13） | 【生成】覆盖率与注释校验报告 |
| AI_README.zh/en.md（3305×2） | 【生成】发布产物原件 |
| architecture.{zh,en}.html、behavior.{zh,en}.html（各 2 行） | 【生成】交互式图页面（cytoscape.js 渲染） |
| assets/：cytoscape.min.js（31，压缩）、guide.js（72）、guide.css（13）、CYTOSCAPE_LICENSE.txt（18） | 页面静态资源 |
| annotations/.gitkeep | 占位 |

## 3. docs/debug/（1 文件）
esp32-jtag-debugging.md（192）：JTAG 调试完整指南（工具链定位、端到端流程、人工分步教程、铁律与恢复）。

## 4. ESP32-P4/（227 文件，27,681 行）

### 顶层
ESP32-P4.ino（16）sketch 入口 → system/AppBootstrap；.editorconfig（4）；.gitignore（1）；.vscode/tasks.json（37，Arduino 编译任务）+ settings.json（8）；tools/strip_utf8_bom.py（68）+ watch_strip_utf8_bom.py（81）BOM 清理（唯一调用方是下述死钩子）；**.githooks/pre-commit（5）⚠ 死钩子**：core.hooksPath 已指向根 .githooks，此文件永不执行，且其内容引用旧布局路径（`main`、`src/src/...`）。

### src/config/（第 1 层：编译期配置，18 文件 ≈880 行）
Config.h(18) 总装配头；BoardConfig.h(33) 板级引脚；TaskConfig.h(28) 任务周期/栈/优先级；ControlConfig.h(148) 控制环参数；ServoConfig.h(72) 舵机总线；SafetyConfig.h(49) 安全阈值；DimensionsConfig.h(40) 机械尺寸/电机布局；CalibrationConfig.h(17)；RuntimeMonitorConfig.h(22)；DiagnosticsConfig.h(22)；topology/HardwareMap.cpp/.h(122/34) 电机↔舵机 ID 拓扑；safety/SafetyRuleCatalog.def(116) **安全规则唯一手写源**（P4_SAFETY_RULE 宏表，103 条）；safety/ServoPositionRange.h(54) 位置软限位；protocol/S3SensorCanProtocol.h(58) S3→P4 CAN ID；protocol/HostProtocol.h(14)+Crc32Ieee.cpp/.h(28/21)+Crc16Ccitt.cpp/.h(26/19) 帧头与 CRC。

### src/data/（第 2 层：纯值类型，18 文件 ≈770 行，R2：禁依赖 Arduino/RTOS）
SystemTypes.h(79) 系统状态/事件枚举（p4_state 图的数据源）；ActuatorTypes.h(68)；ServoTypes.h(45)；ServoRuntimeState.h(52)；ServoRouteTypes.h(44)；ControlRuntimeState.h(55)；CommandRuntimeState.h(51)；ControlTraceTypes.h(34)；JointTypes.h(27)；TelemetryTypes.h(23)；SafetyTypes.h(47)；SafetyRuleTypes.h(30)；CalibrationTypes.h(20)；CalibrationRuntimeState.h(27)；RecoveryTypes.h(16)；RopeInitTypes.h(35)；HardwareTypes.h(33)；ServiceSnapshots.h(51) 服务间快照聚合。

### src/generated/（6 文件 ≈2,490 行）【生成，禁止手改】
UpperProtocol.generated.h(183)←upper_protocol_schema.json；SafetyRuleCatalog.generated.h(112)+SafetyRuleIndex.generated.json(2092)←SafetyRuleCatalog.def；UrdfJointLimits.generated.h(50)+ProductFingerprint.generated.h(20)+HardwareBindings.generated.h(34)←URDF+全树 CRC 指纹。

### src/hal/（第 3 层：外设封装，R3）
servo/FTServo_Arduino/（~20 文件 ≈1,900 行）：内嵌飞特第三方 SDK（SCS/SMS_STS/HLSCL/SCSCL/SCSerial/INST + 头 + LICENSE/README/library.properties），整体豁免架构规则；servo/FeetechServoDriver.h(1077)/.cpp(5) P4 侧唯一飞特入口封装；servo/ServoBusRegistry.cpp/.h(32/26) 总线注册表；servo/ServoValue.h(58) 值类型；can/TwaiCanDriver.cpp/.h(116/62) TWAI 收发；serial/HostSerialPort.cpp/.h(61/28) 上位机串口。

### src/safety/（第 4 层：安全策略，~35 文件 ≈3,050 行）
SafetyRuleRegistry.cpp/.h(121/41) 规则注册表；SafetyEventStore.cpp/.h(117/46) 事件存储；SafetyEventReporter.cpp/.h(168/85) 事件上报；ActuatorCommandArbiter.cpp/.h(24/24) 命令仲裁。
policies/ 15 策略：SystemReadinessPolicy(405/70) 系统就绪门禁；CollisionSafetyPolicy(374/138) 反馈碰撞保护；ActuatorSafetyPolicy(292/68) 执行器（过流/超温类）；ControlBoundarySafetyPolicy(90/67) 控制边界；ControlInputSafetyPolicy(64/40) 输入合法性；CommunicationSafetyPolicy(70/41) 通信超时/校验；SystemFaultPolicy(94/27) 系统故障；OverloadSafetyPolicy(79/38) 过载；RopeInitSafetyPolicy(96/32) 绳驱初始化；EncoderHealthSafetyPolicy(69/32) 编码器健康；RuntimeSafetyPolicy(58/46) 运行时；HostLeaseSafetyPolicy(32/21) 上位机控制权租约；StateAdmissionPolicy(34/18) 状态准入；CalibrationSafetyPolicy.h(51) 标定；ProtocolContractPolicy.h(16) 协议契约。

### src/system/（第 5 层：组装根+基础设施，~30 文件 ≈2,000 行）
SystemAssembly.cpp/.h(228/12) 组合根：任务装配表；AppBootstrap.cpp/.h(36/16) 启动；SystemRuntimeContext.h(26) 共享上下文聚合（R7）；ActuatorCommandGateway.cpp/.h(77/33) 命令网关；ServiceCyclePolicy.cpp/.h(29/14) 服务周期策略；TaskRuntimeWindow.cpp/.h(65/31) 运行窗口统计；TaskRuntimeMetrics.h(108) 任务运行指标；adapters/ServiceContextAdapter.cpp/.h(113/22) 上下文适配；channels/InterTaskChannels.h(39) 任务间通道声明；identity/DeviceIdentity.h(19) 设备标识；log/FirmwareLog.cpp/.h(117/21) 日志；time/Clock.h(31)；ports/：DirectTargetResult.h(42)、SystemEventPort.h(29)、ServiceOutputPort.h(28) 跨层稳定端口；rtos/：MessageChannel.h(75)、SharedSafetyState.cpp/.h(18/59)、TaskSignal.h(58)、PeriodicDeadline.h(55)、AtomicValue.h(50)、TaskFactory.cpp/.h(32/36)、FreeRtosKernel.h(19) RTOS 薄封装；calibration/ 标定四件套：CalibrationRuntimeStore.cpp/.h(279/127)、CalibrationMath.cpp/.h(277/69)、TendonCalibrationLut.cpp/.h(211/55)、ManualEncoderReferenceStore.cpp/.h(123/40)。

### src/tasks/（第 6 层：FreeRTOS 薄入口 + 服务，~45 文件 ≈12,600 行）
**motion_control/**：MotionControlTask.cpp/.h(49/16) 薄入口；MotionControlService.cpp/.h(2089/28) 主控制服务（目标插补/模式/完成判定）；DualTendonControl.cpp/.h(349/47) 双腱驱动控制；pid.c/.h(134/65) PID 实现；ControlSolver.cpp/.h(92/44)；ServoTargetBuilder.cpp/.h(135/47) 目标→舵机位置；JointAngleMapper.cpp/.h(43/24) 关节↔电机角；MotionCompletionEvaluator.cpp/.h(84/26) 动作完成评估；ControlCommandSnapshotReader.cpp/.h(48/23) 快照读取。
**servo_bus/**：ServoBusTask.cpp/.h(60/13)；ServoBusService.cpp/.h(1386/36) 舵机总线服务；ServoFeedbackPublisher.cpp/.h(267/38) 反馈回传；ServoDiscoveryManager.cpp/.h(236/52) 在线扫描；ServoBusRouting.cpp/.h(229/34) 路由；ServoRouteTable.cpp/.h(211/36)；ParallelServoFeedbackCycle.cpp/.h(130/45) 并行反馈轮询。
**host_comm/**：HostCommTask.cpp/.h(55/13)；HostCommService.cpp/.h(567/44) 上位机通信服务；HostCommandDispatcher.cpp/.h(1060/23) 命令分发（系统事件/控制权）；HostCommandRouter.cpp/.h(329/45) 路由；HostCalibrationCommandHandler.cpp/.h(231/44) 标定命令；HostProtocolCodec.cpp/.h(178/53) 帧编解码（有宿主机 C++ 测试）；HostFrameReader.cpp/.h(124/62) 帧读取；HostTelemetryPublisher.cpp/.h(959/60) 遥测发布；HostMultiTurnPublisher.cpp/.h(133/23) 多圈数据；HostAckPublisher.cpp/.h(80/20) ACK。
**system_state/**：SystemStateTask.cpp/.h(58/17)；SystemEventDispatcher.cpp/.h(1077/30) 状态机事件分发（p4_state 图核心）；SystemStateCoordinator.cpp/.h(107/31)；DirectCommandCoordinator.cpp/.h(293/44) 直控协调；HostLeaseSupervisor.cpp/.h(280/40) 控制权租约监督；RopeInitController.cpp/.h(210/29) 绳驱初始化；FeedbackReadyDiagnostics.cpp/.h(106/25)。
**s3_can_rx/**：S3CanRxTask.cpp/.h(37/13)；S3CanRxService.cpp/.h(766/17) CAN 接收服务；EncoderHealthMonitor.cpp/.h(107/34)。
**runtime_monitor/**：RuntimeMonitorTask.cpp/.h(34/9)；RuntimeMonitorService.cpp/.h(547/14) 运行监控/看门狗指标。

## 5. ESP32-S3/（19 文件，2,553 行）
ESP32-S3.ino(65) sketch 入口；src/config/Config.h(150) 引脚/波特率/任务参数；src/system/SystemTasks.cpp/.h(48/15) startSystemTasks 任务装配；src/system/FirmwareLog.h(27)；src/hal/HalEncoders.cpp/.h(349/86) 磁编码器采集；src/hal/HalTactile.cpp/.h(533/94) 触觉传感器驱动；src/hal/HalTWAI.cpp/.h(435/64) CAN 发送；src/hal/HalSensorSpi.cpp/.h(119/17) SPI 底层；src/tasks/EncodersTask.cpp/.h(78/14) 编码器采集任务；src/tasks/TactileTask.cpp/.h(337/14) 触觉任务（CAN+独立串口双路摘要）；src/tasks/CanTask.cpp/.h(94/14) 编码器帧发往 P4。

## 6. desktop/（81 文件，23,832 行，NiceGUI 上位机）
每个源文件头部带统一契约注释（职责/线程/输入/输出/状态所有权/禁止依赖），受 desktop 架构门禁约束。

### app/（3） entry.py(22) CLI 参数；ui_runner.py(88) 装配后端+启动网页；__init__(10)
### __main__.py(15) `python -m desktop`；__init__.py(10)；paths.py(32) 数据路径；requirements.txt(3)
### core/（43 文件，17,923 行）
core_logic.py(2960) 核心业务状态机（连接/使能/模式/安全闸门，UI 快照来源）；comm_layer.py(1768) 串口通信层（连接管理/收发线程/重连/握手）；protocol.py(1485) 主串口协议编解码（FE LEN cmd payload FF）；application_actions.py(1150) UI 动作编排；hand_backend.py(1034) HandBackend 门面（UI↔core 装配）；data_models.py(567) 共享数据类型；angle_mapping.py(474) 关节↔电机角映射；calibration_snapshot.py(830) 标定快照模型与持久化；calibration_diagnostics.py(416) 标定诊断；calibration_workflow.py(336) 标定向导；safety_rules.py(99) 103 条安全规则英文解释表（与 .def 同步，有测试）；motor_mapping.py(47)；device_state_store.py(247) 每设备状态持久化；joint_target_adapter.py(188) ROS2 目标适配；console_snapshot.py(175) CLI 快照；generated_upper_protocol.py(173)【生成】协议常量；session_log.py(88)；auto_calib_logger.py(86)。
**host_calibration/**（9）：runtime.py(1003) 标定运行时状态机；dual_motor.py(774)/single_motor.py(545) 双/单电机流程；math.py(329) 拟合数学；flow.py(238) 步骤编排；time_alignment.py(204) 时间对齐；config.py(114)；finish.py(110)；__init__(9)。
**communication/**（8）：state_reducer.py(1086) 回包/遥测归约为设备快照；feedback_history.py(113) 反馈历史；ack_tracker.py(105) 命令-ACK 关联；serial_transport.py(101) pyserial 抽象；frame_decoder.py(84) 拆帧；packet_router.py(51)；event_channel.py(43)；__init__(26)。
**data_recording/**（7）：recorder.py(262) 录制会话；topic_schema.py(227) schema；motion_sampler.py(125)；csv_exporter.py(109)；packet_recorder.py(75)；event_recorder.py(44)；__init__(13)。
### ui/（13，741 行）browser_view.py(185) 原生控制台主页面；web/common.py(103) 展示工具；web/direct_control.py(77) 直控页；web/feedback_control.py(56)；web/tactile.py(51) 触觉视图；web/joint_calibration.py(49)；web/plots.py(47) 曲线；web/motor_calibration.py(46)；web/replay.py(44)；web/diagnostics.py(41)；web/__init__(9)；__init__(10)；readme.md(23)
### replay/（6，1,425 行）replay_controller.py(676) 回放控制；replay_csv_loader.py(400) CSV 加载；replay_models.py(121)；replay_service.py(79)；__init__(24)；README.md(125)
### ros2/（3）joint_targets.py(205) 关节目标订阅；raw_ik_target.py(154) 原始 IK 订阅；__init__(10)
### examples/（2）ros2_joint_target_sender.py(62) 测试用发布者；__init__(2)
### resources/（7，3,130 行）
p4_safety_rules.json(933)【生成】安全目录导出；hku_hand_v2_urdf/urdf/hand.urdf(953) URDF 模型（生成器输入，手工维护）；urdf_joint_mapping.json(653)【生成】关节映射；robot_model_manifest.json(438)【生成】模型清单；upper_protocol_schema.json(131) 协议 schema（**手写输入**）；product_fingerprint.json(14)【生成】全 P4 源码树 CRC；robot_model_source.json(8) 生成器输入声明。

## 7. tools/（21 文件，3,276 行）
generate_robot_model.py(510) 机器人模型生成器：URDF→manifest/映射/指纹/三个 generated 头，--check 防漂移；generate_p4_safety_catalog.py(494) 安全目录生成器：.def→JSON+generated 头+索引，内置规则调用链绑定表；generate_upper_protocol.py(291) 协议生成器：schema→generated .h/.py；precommit_checks.py(103) **门禁本体**（依赖锁/编译器注入/原子暂存/生成器 check/双架构检查/全量 pytest 0 失败 0 跳过）；check.py(8) 同一门的别名（CI 用）；check_desktop_architecture.py(187) desktop 分层 import 规则（AST）；robot_model_source.py(100) 模型来源声明加载；validate_urdf.py(102) ⚠ 无调用方、无测试（孤儿工具）；install_git_hooks.ps1(24) `git config core.hooksPath .githooks`（每个 clone 跑一次）；capture_tactile_map.ps1(69)/capture_manual_cs_scan.ps1(59)/capture_dist_scan.ps1(53) 帕西尼触觉硬件抓包脚本（手工）。
p4_architecture/：check_p4_architecture.ps1(475) P4 七层架构门禁 R0–R9（R0 顶层白名单、R1 头部契约注释、R2 data 纯值、R3 HAL 无业务依赖、R4 基础设施白名单禁反向、R5 safety/system 无 task、R6/6.1/6.2 任务薄入口与服务 init/step、R7 共享上下文聚合、R8 外设唯一入口、R9 生成物无漂移；第 457 行内部调用闭环探针）；check_safety_rule_closure.py(254) 安全规则 policy→caller→event→recovery 调用链闭环探针（也有 pytest 覆盖）。
debug/：gdb_batch.py(255) agent 一键 GDB 批处理（自动起停 OpenOCD、断点/读变量、finally 清断点、ELF 同源校验）；debug_env.ps1(119) 工具链定位与 -Target p4|s3 参数；build_debug.ps1(45) 调试构建（--optimize-for-debug + ELF SHA256）；upload_debug.ps1(35) 烧录 + flashed 标记；start_openocd.ps1(25)（含 gdb_memory_map disable）；verify_debug_env.ps1(52) 环境自检。

## 8. tests/（80 文件，16,107 行，1264 用例）
### tests/desktop/core/（47 文件，≈11.3k 行）
覆盖：core_logic 快照/状态（test_core_logic 940、test_state_copy、test_product_fingerprint_protocol…）、协议与握手（test_protocol_handshake、test_feedback_command_order、test_connection_identity…）、标定全家桶（test_host_calibration_runtime 930、dual/single/math/flow/finish/transport/time_alignment/handoff、test_calibration_snapshot 226、test_calib_snapshot_current、test_multiturn_restore_retry 446、test_servo_coordinate_retry 388、test_manual_reference_protocol/crc/scaled_mapping_v3、test_calibration_completion_alert、test_calibration_diag_protocol、test_calibration_device_identity…）、录制（test_recording_timestamps、test_packet_recording、test_motion_sampler…）、安全与映射（test_joint_workspace_guard 558、test_angle_mapping、test_joint_target_adapter、test_servo_rebase_zero_protocol 622、test_scoped_multiturn_protocol、test_direct_hold_and_release…）、console 与 UI 支撑（test_console_operations 364、test_console_snapshot/connections/control_status）。
### tests/desktop/ui/（3）test_native_pages.py(262) NiceGUI 页面；test_browser_view.py(16)；test_calibration_completion_popup.py(120)
### tests/desktop/app/（3）test_entry.py(22)；test_ui_runner_port.py(59)；test_ui_runner_ros2.py(93)
### tests/desktop/algorithm/（2）test_replay_csv_loader.py(338)；test_replay_controller.py(315)
### tests/desktop/ros2/（2）test_joint_targets.py(106)；test_raw_ik_target.py(66)
### tests/firmware/（11，638 行，协议契约，Python 侧模型）
test_safety_rules.py(33) 103 条规则表一致性；test_p4_safety_rule_closure.py(77) 规则调用链闭环；test_safety_event_protocol.py(50)；test_state_sync_protocol.py(39)；test_task_runtime_protocol.py(44)；test_task_runtime_window_reducer.py(61)；test_control_trace_protocol.py(122)；test_calibration_snapshot_v4_protocol.py(71)；test_manual_reference_ack_scope.py(46)；test_multiturn_runtime_identity.py(72)；test_rope_init_compatibility.py(23)。
### tests/tools/（11，2,695 行）
五个 **宿主机 C++ 编译测试**（把生产 .cpp 编译成原生可执行跑断言，无编译器时 pytest.skip——门禁因强制编译器+禁 skip 而保证真正运行）：test_p4_servo_position_codec.py(1068)、test_p4_servo_custom_position.py(1068)、test_p4_direct_abs_zero.py(391)、test_p4_servo_hotplug_recovery.py(592)、test_p4_host_control_lease_policy.py(101)。
工具自身：test_generate_robot_model.py(125)、test_generate_upper_protocol.py(100)、test_generate_p4_safety_catalog.py(82)、test_robot_model_source.py(91)、test_desktop_architecture.py(9)、test_precommit_checks.py(51)（门禁自测）。

## 9. 调整文件夹时值得注意的死配置/过期面

1. **ESP32-P4/.githooks/pre-commit**：永不执行（core.hooksPath 指根 .githooks）且引用旧布局路径 `main`、`src/src/...`；其唯一依赖 ESP32-P4/tools/strip_utf8_bom.py(+watch) 因此也成孤儿 → 可整体删除。
2. **tools/validate_urdf.py**：无任何调用方、无测试 → 删或纳入门禁。
3. **check_desktop_architecture.py** 的 desktop/teleop、desktop/recording 依赖规则指向不存在的目录（死规则，无害）。
4. **docs/ai 全线停摆**：source_scope.json 所有路径仍指 `ESP32-P4/ServoBoardMain/src/...`、`ESP32-S3/System/src/...`；extract.py:222、:248 与 render.py 硬编码 ServoBoardMain；42 个注释里的 UI 图对应已不存在的旧 Qt 界面（ui_app/ui_windows/…）；extract.py:120 的 UI 信号探针是 Qt 的 `.clicked.connect`，现 NiceGUI 写法抽不出任何绑定；根目录与 docs/ai 的 AI_README 互相引用已删文件。恢复需：改两处硬编码 → 重写 source_scope.json → UI 探针改 NiceGUI 事件 → commit 后 make.py --prepare/--validate/--publish（需 ../mmhand_teleop）→ 重写全部 42 图注释 → 复制 AI_README 到根。
5. .gitignore/.zcodeignore 历史模式（ESP32-P4Test*、CanWaveform、ESP32-S3/System/build、desktop/teleop/Posture 等）指向早已不存在的目录，可清理。
6. 未纳入门禁的测试：docs/ai/build 4 个测试文件（21 用例，pytest testpaths 不含）；S3 固件编译（门禁与 CI 均无）；docs/ai --validate（手工）。
