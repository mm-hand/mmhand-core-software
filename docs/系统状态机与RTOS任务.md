# 系统状态机与 RTOS 任务

最后核对：2026-08-01

## 1. 状态机解决什么问题

P4 同时存在上位机命令、10 ms 控制循环、自动标定、RopeInit、舵机反馈恢复、CAN 故障和 NVS 保存。如果每个任务根据局部布尔量自行决定是否输出，会出现三类危险竞态：

- 控制与标定同时写目标；
- 上一个命令的 DONE/TIMEOUT 晚到，错误释放当前命令；
- STOP 后串口或 ROS2 的下一帧又把系统拉回运行态。

因此 `StateMachineTask` 是 `systemState`、`activeOwner`、`controlEnabled` 的唯一写者。状态描述生命周期，owner 描述当前唯一执行器所有者，generation/token 描述事件属于哪一代命令。三者分开后，旧事件可以被丢弃，任务也不能凭优先级越权。

## 2. 当前状态和准入

状态数值是稳定协议的一部分，不能为了补齐空号而重排。

| 值 | 状态 | 含义 |
| ---: | --- | --- |
| 0 | `UNCONTROLLABLE` | 当前反馈/配置不足，禁止运动 |
| 1 | `IDLE` | 可通信但不满足 READY 条件 |
| 2 | `RUN_FEEDBACK` | 关节角闭环运行 |
| 4 | `CALIBRATION_RUNNING` | 自动标定独占执行器 |
| 5 | `READY` | 已具备闭环运行条件 |
| 6 | `RUN_DIRECT` | 直接电机位置命令运行 |
| 7 | `PAUSED` | 已暂停输出，可按策略恢复 |
| 8 | `ROPE_INIT_RUNNING` | 使用人工端点把关节带回工作区 |
| 9 | `RECOVERY_REQUIRED` | 关节在工作区外但允许 RopeInit 恢复 |

主要命令准入：

| 命令 | 允许的起始状态 | 设计原因 |
| --- | --- | --- |
| START | `UNCONTROLLABLE/IDLE/READY/RECOVERY_REQUIRED/PAUSED` | 只开启普通控制准备，不等于绕过 READY |
| RUN_DIRECT | `IDLE/READY/RECOVERY_REQUIRED/RUN_DIRECT` | 可用于受限调试与恢复，但仍受安全策略约束 |
| RUN_FEEDBACK | 仅 `READY` | 闭环前必须确认标定、反馈和工作区 |
| CALIBRATION | `UNCONTROLLABLE/IDLE/READY/RECOVERY_REQUIRED` | 标定能建立新的工作区，但必须独占执行器 |
| ROPE_INIT | 满足人工标定、映射、反馈、在线和无故障前置条件 | 只负责回到工作区，不重新标定 |
| STOP | 任意状态 | 走独立急停 token，不依赖普通事件队列是否拥塞 |

稳定态不是固定返回 `IDLE`：反馈不可控时为 `UNCONTROLLABLE`；关节都在工作区时为 `READY`；低于工作区但可恢复时为 `RECOVERY_REQUIRED`；其余情况为 `IDLE`。

READY 使用迟滞和连续样本，防止边界抖动：进入容差 24 encoder counts，退出容差 48 counts，并要求 3 个新鲜映射样本。

## 3. 所有权与命令代次

执行器 owner 为 `NONE`、`CONTROL` 或 `CALIBRATION`。RopeInit 虽然有独立状态，但复用 `CALIBRATION` owner，因为二者都属于排他性维护运动。所有目标批次必须携带 owner，经 `ActuatorCommandGateway` 投递；安全策略会再次确认批次 owner 与当前 owner 相同。

每次长流程启动都会分配新 token/generation。完成、失败、超时事件也携带该 token；状态机只处理与当前命令一致的事件。因此“标定 A 超时事件晚到，误终止已经开始的标定 B”不会发生。

## 4. P4 任务优先级与周期

| 任务 | 优先级 | 典型周期/触发 | 为什么这样排 |
| --- | ---: | --- | --- |
| StateMachine | 5 | 10 ms，`vTaskDelayUntil` | 最先处理急停、故障、状态和 owner；避免安全决策被业务任务拖延 |
| Control | 4 | 10 ms，`vTaskDelayUntil` | 需要稳定采样周期；实测 dt 异常时回退 10 ms，避免 PID 数值突变 |
| CAN | 3 | 1 ms 轮询 | 快速接收 S3 分帧并维持反馈新鲜度 |
| ServoComm | 3 | 10 ms，交替读取两组总线 | 同时承担反馈刷新和目标执行；四总线分相减少单周期阻塞 |
| UpperComm | 2 | 5 ms | 串口解析/遥测可能有突发流量，不应抢占控制和安全任务 |
| AutoCalibration | 2 | phase 参数决定 | 标定是长流程，不靠高优先级保证独占，靠 owner 保证 |
| Monitor | 1 | 1000 ms | 日志和诊断不进入实时关键路径 |

这种分配的核心不是“数值越大越重要”，而是把安全判定和确定性闭环置于串口、日志、标定编排及 Flash 之前。优先级相同的 CAN/Servo 仍可能互相影响，所以实际时序要结合 runtime monitor 和实机测量，而不能把周期之和当作硬实时保证。

## 5. 队列、互斥锁和 latest-value 语义

`SystemTask.cpp` 创建的主要队列：

| 队列 | 深度 | 语义 |
| --- | ---: | --- |
| state event | 8 | 保留离散状态事件，由状态机顺序消费 |
| CAN/servo feedback、angle/raw/telemetry、mapped angle | 1 | `xQueueOverwrite`，消费者只需要最新快照，不积压旧采样 |
| servo target | 1 | 只允许 Gateway 写入；新的目标覆盖旧目标，避免控制命令排队后延迟执行 |
| joint debug | 8 | 允许短时诊断突发，不参与控制决策 |
| tactile | 3 | 容纳少量显示侧抖动，仍避免无限积压 |

`commandStateMutex` 保护跨任务共享的命令状态。互斥锁解决数据一致性，owner 解决业务授权，队列解决生产者/消费者解耦，三者职责不同。

## 6. 急停为何不用普通队列

普通 state event 队列深度有限，也可能暂时被任务占用。STOP 同时递增单调 `emergency_stop_request_token` 和舵机侧 `servo_emergency_stop_token`：

1. 状态机每个 10 ms 周期先检查急停 token，再处理普通事件；
2. ServoComm 独立看到舵机急停 token 后直接关闭 torque；
3. desktop 的 ROS2 输出门使用同一把锁完成“撤权 → STOP”，下一帧无法在两步之间插入；
4. STOP 后控制 generation 失效，旧目标与旧完成事件都不能恢复运动。

因此急停不依赖某一个普通消息被及时出队，状态层和执行层形成两条互补路径。

## 7. 故障与安全分层

状态机汇总的主要 fault bit：CAN 离线 bit0、过载 bit2、舵机离线 bit4、标定故障 bit5、多圈未知 bit6。当前安全链路分四层：

1. 状态机决定命令是否准入、哪个 owner 有权输出；
2. 控制/标定生成 owner-tagged target batch；
3. `ServoSafetyPolicy` 检查批次结构、owner、在线、多圈可信、圈数溢出、会话 logical range 和硬件 ABS 范围；
4. `FeetechServoDriver` 保留 `[-30719, 30719]` 的独立硬边界，防止上层遗漏。

`EmergencyStop/InvalidState/StaleTarget` 已在结构化拒绝原因中预留，但目前 E-STOP、状态准入和部分新鲜度仍分别由状态机/协调器处理，不能在文档里声称已经全部迁入 `ServoSafetyPolicy`。

## 8. 维护检查

- 新增能写舵机的流程时，必须先定义 owner、状态准入、token 失效条件和 STOP 行为。
- latest snapshot 不应改成 FIFO；离散事件不应无理由改成 overwrite。
- 不允许绕过 `ActuatorCommandGateway` 直接访问 `servoTargetQueue`。
- 调整优先级或周期后，检查栈水位、任务耗时、deadline miss、STOP latency 和串口/CAN 丢包。
- 状态值、事件值、fault bit 需要保持协议兼容；确需变更时同时修改 schema、生成物、desktop 和测试。
