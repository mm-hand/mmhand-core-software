# MM-Hand 系统重构实施计划

状态：重构实施中；固件最终构建、烧录和实机验收结果以验证记录为准。本文是后续实施的完整依据；数值中标为“试验起点”的项须经过基线记录和实机整定才能成为正式默认值。

## 1. 已确认的目标与当前基线

主导航固定为 **Calibration → Control → Replay → Plot → Settings**。Calibration 含 Joint Calibration、Motor Calibration；Control 含 Feedback Control、Direct Control；Plot 含曲线/录制与 Tactile；Settings 含 Renumber Motor、Renumber Tactile、Control Parameters、Diagnostics。打开软件时显示 Calibration / Joint Calibration。任何标定或控制页面、连接或重连后，都不预选关节或电机；P4 启动活动范围为空。全局 STOP 和明确的 Clear Faults 保留；上位机与 P4 的 Pause、恢复旧目标、Exit Control 全部删除。

目前 P4 的 `MotionControlService.cpp` 同时承担调度、换向、碰撞和控制分量，`ControlSolver.cpp` 与 `DualTendonControl.cpp` 有两条不同计算路径；`ControlConfig.h` 集中写死调参常数。桌面端 `application_actions.py`、`hand_backend.py`、`core_logic.py` 和 `comm_layer.py` 承担多种职责。工作树内已有部分未提交的 USB、控制释放、电机维护改动，实施时应在现状上增量整理，不覆盖用户已有修改。报告 `C:\Users\73547\MM-Hand 1.5技术报告\main.tex` 的传动控制式、两份 Downloads 设计任务书以及仓库内协议 schema 为对照依据。

### 1.1 不可混淆的运动含义

- P4 外环约 100 Hz，每个有效周期根据**本周期实测电机 logical ABS** 计算新的**绝对舵机目标**。它不是把前一个指令目标累加成增量式 PID，也不是每收到一次上位机目标就只给舵机发一次指令。上位机目标被保持后，P4 会在每个控制周期重新计算并向所选电机发位置指令，实际总线写次数取决于控制持续时间、总线调度和故障状态。
- `ACK/QUEUED` 只表示接收或入队；`TARGET_RESULT/APPLIED` 表示第一批目标已提交到 ServoBus，不表示机械已经到位。`ENABLED` 必须以带 token 的最终执行结果、随后新鲜状态包中的 owner/活动范围，以及无拒绝位共同确认。
- LUT 提供局部斜率及双腱配对几何，不把 `g(q_target)` 直接当舵机绝对指令。型号/方向、Manual、Auto/LUT、ABS 限位和多圈锚点在连接与改号后必须各自验证。

## 2. 新控制链路与公式

### 2.1 数据流、单位和时间

ROS2 原始目标 → 上位机有效性/关节映射 → 21 路各自的静止感知 One Euro → 已授权目标 → P4。GUI 手工目标直接走既有范围与安全检查，不重复经过 ROS2 滤波；Replay 播放录制的已执行目标，不二次滤波。P4 磁编 raw（14 bit）→ 有效性、断线/跨圈处理 → 快/慢反馈估计 → 外环、换向、碰撞 → 统一限位/速率约束 → ServoBus。用传感器和控制循环的**实际单调时间戳**求 `dt`；不以接收包时间代替设备时间。关节角内部统一 degree，One Euro 内部 degree，电机位置/修正统一 logical ABS counts，串口关节目标继续用 degree。

每个关节 `j`、电机 `i`、周期 `k`：

```text
e_j[k]       = q_des,j[k] - q_fast,j[k]                         [degree]
s_i[k]       = d f_i(q) / d q | q=q_fast,j[k]                    [counts/degree, signed]
P_i[k]       = Kp_group × s_i[k] × e_j[k]                       [counts]
u_base,i[k]  = m_measured,i[k] + P_i[k] + I_i[k]                 [counts]
u_raw,i[k]   = u_base,i[k] + C_reverse,i[k] + R_pair,i[k]       [counts]
u_cmd[k]     = jointwise_safety_and_rate_limit(u_raw, m_measured,
                 last_command, ABS_limits, LUT_margins, active_scope)
```

单腱 `R_pair=0`。双腱两个电机共用同一个关节误差，各自取**自己的有符号局部 LUT 斜率**；斜率包含齿轮/腱方向，不另乘经验正负号。`Kp_single` 和 `Kp_dual` 是无量纲共享倍率，分别适用于全部单腱和全部双腱关节；不支持逐关节覆盖。局部斜率不存在、为零、非有限或校准不可信时拒绝 ENABLE/停止该关节输出并报明缺失 LUT，绝不回退为旧固定 `counts/degree` 增益。P4 只使用测得的新鲜 `m_measured`，禁止用上周期命令替代。最后按电机 ABS 机械范围、LUT 两端收绳/放绳裕量、每周期位置变化和真实时间的速率界限裁剪；成对电机共同评估，不能只允许其中一台越过配对约束。

本轮生产控制器为 **PI + 换向补偿 + 双腱配对恢复**；`D=0`，没有单腱目标速度前馈。删除旧的“误差≤0.8° 用 Kp=20”、0.9/0.41 局部/平均斜率倍率、2.5°/3.0° 混合区及相关 host/P4 参数与日志；删除旧单腱目标速度预测/lead、目标速度滤波及任何前馈发送字段。旧 `velocity_filter_tau=0.050 s` 服务于 D 速度估计，不能与新两路反馈时间常数混为一谈；D 关闭后移出活跃配置，诊断仍可报告速度。

### 2.2 P4 磁编滑动窗口与时序

在原始 raw 合法、跨圈解开、物理突变检查通过后，为每个关节保存最近有效的 `(设备时间戳, 连续角度)`，并按**时间**而非固定帧数维护两段共享样本的窗口。默认 `W_fast=20 ms`、`W_slow=500 ms`，均由上位机 JSON 整组下发。输出就是窗口内样本的算术平均：

```text
q_fast(t) = mean{q_valid(t_i) | t-W_fast < t_i ≤ t}
q_slow(t) = mean{q_valid(t_i) | t-W_slow < t_i ≤ t}
```

一个环形缓冲同时服务两窗口；新有效样本到来才更新，窗口为空不使用旧值。控制启用前要求快窗口有新鲜样本、慢窗口已覆盖完整 500 ms；首次有效样本只初始化，不从零爬升。P 用 `q_fast`；积分持续误差参考 `q_slow`；换向由过滤后的目标净位移和 `q_fast` 响应确认；**碰撞速度直接由 `q_slow` 算**。无新样本时不重复积分、不延长碰撞证据。时间戳不增、断线、物理跳变或反馈过期时清空窗口并进入安全状态，恢复后重新积满慢窗口才能重新碰撞武装。500 ms 慢平均本身有约 250 ms 的低频延迟，再叠加两段碰撞窗口；这是明确接受的设计代价，实机必须测其最坏触发时间和漏检风险，硬过载继续独立生效。

### 2.3 条件式慢积分

积分状态直接保存为**电机 counts 修正**，每个参与电机一份：

```text
e_I = sign(e) × max(0, |q_des - q_slow| - e_noise)   [degree]
I_i[k+1] = clamp(I_i[k] + Ki_group × s_i[k] × e_I × dt,
                 -I_limit_group, +I_limit_group)      [counts]
```

`Ki_group` 单位 `1/s`，单腱和双腱各一组；`I_limit_group` 单位 counts。仅在目标保持稳定、误差超过磁编噪声门槛持续确认时间、非换向/稳定等待、非碰撞/维护、反馈与电机均新鲜、输出未沿该积分方向饱和时更新。电机不随动或被物体阻挡、发生碰撞或已在限位时冻结积分；重获许可后延续原状态，不因静止而清零。**不设置积分泄漏、泄漏开关或泄漏时间常数。** STOP、owner 失去、关节重标定、坐标重基准、参数修订应用时清零；普通小幅目标变化只冻结，不突然抹去姿态保持偏置。初始化 `Ki=0`，基线 P 稳定后再逐步增加；`I_limit=100 counts` 是旧代码量级的保守试验起点，仍须用实际安全行程确认。

### 2.4 双腱正交配对恢复

在 `q_fast` 处按报告定义 `f_1(q), f_2(q)`，令 `m=(m_1,m_2)^T`、`s=(s_1,s_2)^T`：

```text
z       = [ f_1(q_fast)-m_1, f_2(q_fast)-m_2 ]ᵀ
z_parallel = s × (sᵀ z) / (sᵀ s)
z_perp  = z - z_parallel
R_pair  = k_pair × z_perp
```

`k_pair` 一个无量纲可调增益，0.05 可作旧系统对照起点。若 `sᵀs` 近零、LUT/电机不新鲜或两个电机不是完整活动配对，则整项为零并拒绝不完整 ENABLE。删除旧的 10 counts 死区、3 counts 单周期上限、200 counts/s 局部速率上限。仍经**共同最终输出**机械范围和变化速率限制；若安全约束不能共同容纳修正，取消两台的本周期配对恢复并记录原因。正交只表示局部传动模型的一阶性质，不声称测量或恒定控制腱张力。

### 2.5 快速换向

P4 为每关节维护 `TRACKING → REVERSE_PENDING → TAKE_UP → SETTLING → TRACKING`；方向由已授权目标的**净位移趋势**与有符号 LUT 判定，单帧速度符号翻转不触发。候选反向累计达 1°，并满足持续确认时间、静止噪声滞回和冷却后，仅触发一次事件。沿用已有效的单腱试验起点：初始 60 counts、增长 5000 counts/s、上限 140 counts、关节在新方向响应 0.25° 退出；双腱补偿方向按各自局部斜率，需在单关节台架重新验证。

关键是**以换向开始时的绝对位置作锚点**，不能每周期向 `m_measured+P+I` 重复追加同一 60–140 counts：

```text
B(t)       = min(B_max, B_0 + v_B × t)
m_ref,i(t) = m_anchor,i + d_motor,i × B(t)
C_reverse,i = d_motor,i × max(0,
                d_motor,i × (m_ref,i(t) - u_base,i))
```

`d_motor` 为该事件的真实电机方向 `±1`；`m_anchor` 只在进入 TAKE_UP 时采集一次。`C_reverse` 是达到事件绝对参考位置尚欠的余量；普通 PI 已经推进到参考位置就为零。正常输出安全限制照常作用。新方向实测关节净位移达到 0.25° 后进入 SETTLING，期间冻结 I；无响应到试验上限 500 ms/机械边界时结束补偿、保持受影响关节并报告 `REVERSAL_NO_RESPONSE`，不在每圈继续加绳，也不把未武装的静止卡住伪报为碰撞。20 ms 候选确认、50 ms 稳定期、100 ms 冷却仅为试验初值，最终按静止噪声和慢速真实动作回放确定。STOP、失联、目标源切换、标定变化即丢弃事件锚点。

### 2.6 简化的碰撞保护

P4 唯一编译期宏 `COLLISION_WINDOW_MS=500`，**武装和武装后的最终检测分别完整使用这个 500 ms 窗口**。对屈曲、伸展各有独立 `v_arm` 和 `v_stall`；AA/CMC 等非屈伸关节用第三组参数。为避免把正常接近目标误判，当前误差 `|e|` 必须超过可配置 `collision_min_error_deg`（旧值 4° 为试验起点）。

```text
v_dir(W) = direction × (q_slow(t_end)-q_slow(t_start)) /
           (t_end-t_start) [degree/s]
武装：连续完整 500 ms 窗口，v_dir > v_arm，P4 确实输出推进，数据新鲜。
触发：此后另一个完整 500 ms 窗口，v_dir < v_stall，且最终限幅后的
      舵机指令仍沿目标方向推进、没有被 ABS/速率/故障限住。
```

这里“角速度突变小于某一值”按用户澄清解释为**当前方向速度低于停滞阈值**，并非两个窗口速度差或加速度。`q_slow` 是 500 ms 滑动均值；武装前要有完整慢窗口，武装和最终判断又各需完整 500 ms，因此从首次有效采样到最早触发至少约 1.5 s，接触发生时的额外反应延迟也可能受慢均值拖尾影响。初始试验阈值 `v_arm=8°/s`、屈曲 `v_stall=1.0°/s`、伸展和其他 `0.8°/s`；必须用新 `q_slow` 信号重新整定，不能沿用原始速度阈值而声称等效。目标每次跳变超过 1° 或方向反转时清除旧窗口并从新方向重新武装；1° 比较的是**相邻已授权目标**，不是累计小动作。反转时始终清空旧武装，不把旧方向样本带过去。探测到停滞则只对该关节进入 HOLD，冻结 P/I 与换向，保持当下舵机位置，发带 joint ID 的安全事件；用户明确反向退让至少 1° 后解除该关节 HOLD，再重新经历武装。没有负载或触觉量测参与此碰撞判据；现有硬过载、主机租约、编码器故障仍独立保护。页面仅对受影响关节追加红色 `COLLISION DETECTED`。

### 2.7 ROS2 输入静止感知 One Euro

在桌面 ROS2 通用接收入口，对 `/raw_ik_target` 和 `/mmhand/joint_targets` 先适配 21 个 URDF 关节角，再用同一实现、21 份独立状态处理；同一运行会话只授权一个目标源，切换源时重置历史。消息 `Float32MultiArray` 没有源时间戳，用主机单调接收时间；日志标出无法测量传输延迟的限制。过滤器直接处理适配后的 **degree**，与现有 `mmhand_teleop` 末端关节角滤波参数单位一致：

```text
dt=t[k]-t[k-1]
a(f,dt)=2πfdt/(1+2πfdt)
d_raw=(x_raw[k]-y_prev)/dt
d_hat=d_hat_prev+a(f_deriv,dt)×(d_raw-d_hat_prev)
f_cut=min_cutoff+β×|d_hat|
y_1e=y_prev+a(f_cut,dt)×(x_raw-y_prev)
```

`min_cutoff=1 Hz`、`f_deriv=1 Hz`、`β=0.02` 是外部 `mmhand_teleop` 现有 degree 空间的试验起点。明确处理坏 `dt`、NaN、越物理上限跳变、角度跨圈、丢帧与初始化；坏帧返回上一有效输出但 `valid=false`，不能授权新目标或触发换向。

静止感知只保留**一个固定锚点和两个滞回阈值**，不做 150 ms 统计窗口、在线噪声估计或趋势斜率计算：`MOVING` 时若 One Euro 输出距候选锚点始终小于 `δ_enter` 达 200 ms，就以当时输出锁定 `anchor` 并进入 `STATIONARY`；若中途越界便把候选锚点重设为当前输出并重新计时。`STATIONARY` 输出固定 `anchor`；One Euro 内部仍连续更新，若其输出距锚点超过更大的 `δ_exit` 达 20 ms，则回到 `MOVING`，首帧从锚点平滑接到 One Euro 值。初始 `δ_enter=0.08°`、`δ_exit=0.25°` 仅供录制回放试调；其中小于 `δ_exit` 的真实慢动作会暂时被保持，这是简化的明确代价。两个阈值和两个时长是整手共用 JSON 参数，21 路只保留各自的滤波、锚点和计时状态，无逐关节覆盖。P4 换向仍由过滤目标的反向净位移独立确认，不能把 One Euro 的导数当实际关节速度。用户稍后从外部 `mmhand_teleop` 删除其末端关节角 One Euro；本仓库实施不改外部项目的关键点/指腹方向滤波。源断开或模式退出重置 One Euro/静止锚点，重连第一帧直接初始化，不自动 ENABLE。

## 3. 参数归属、配置和报告差异

桌面资源目录 `desktop/resources/config/` 新增 `control_parameters.default.json`（发布的默认模板）、`control_parameters.json`（当前期望值）、`control_parameters.schema.json`。JSON 原子写入；文件损坏拒绝控制并提示恢复默认，首次缺失才由模板生成。默认恢复按钮直接把模板复制为当前期望配置并提交等待应用，无二次确认弹窗。UI 同时显示 **期望 revision/CRC**、P4 **实际 revision/CRC**、同步状态。运行中修改先保存期望值，等 owner 释放、系统停止后整组提交；不能强行 STOP，不能在一周期中混用两套系数。连接 P4 时整组下发、读回实际哈希；未同步禁止 ENABLE。P4 将参数只保存在 RAM，复位后必须重下发，不能把同一 CRC 当成固件编译指纹。

| 上位机 JSON 调参组 | 内容与单位 | 原则/试验起点 |
|---|---|---|
| 单腱与双腱 | 各自 `Kp` 无量纲、`Ki` 1/s、`I_limit` counts、启用 I | Kp 单腱 0.8、双腱 0.8 仅低增益起点；Ki=0，逐组整定 |
| 共同反馈 | `window_fast_ms`, `window_slow_ms`；噪声门槛 degree、积分确认 ms | 默认 20/500 ms；窗口按有效磁编时间戳计算，不配置 IIR 时间常数、积分泄漏或相关开关 |
| 输出约束 | 最大每周期 counts、最大 counts/s、LUT 收/放边界裕量 counts | 不得高于 P4 编译期硬上限；以安全行程和总线能力实测。先保留当前已验证裕量 |
| 配对恢复 | `k_pair` 无量纲、开关 | 初始 0.05；无独立死区/局部限速 |
| 快速换向 | 方向确认位移 degree、持续/稳定/冷却 ms、初值/增长/上限 counts | 先沿用 1°、60、5000/s、140、0.25°，时间阈值台架整定 |
| 碰撞 | 屈曲/伸展/其他各自武装、停滞速度 degree/s；最小误差、目标跳变重建、退让距离 degree | 500 ms **窗口长度只在 P4 宏**；试验速度阈值见上 |
| ROS2 主机专用 | One Euro 的三个参数、固定锚点进入/退出阈值与计时、源失效与重置 | 随 JSON 恢复默认，但绝不发送给 P4；无统计窗口/斜率参数 |
| 诊断 | 控制细节采样关节、分频、启用状态 | 不进入运动参数 CRC；运行期按订阅设置 |

P4 `config/` 仍保留板级 GPIO、四条总线及产品关节↔舵机映射、控制周期、帧上限、状态机安全硬限、编码器断线码与硬件过载阈值、心跳租约、`COLLISION_WINDOW_MS=500`、参数合法范围与**最后兜底上限**。这些值变更需重编译。P4 RAM 参数结构的 schema/版本与上位机生成；只有经版本、范围、CRC 验证的整组数据可成为 active。配置页面显示每项单位、说明、允许范围、当前 P4 值、应用状态，并有保存/恢复默认；无每关节覆盖表。

与 [MM-Hand 1.5 技术报告](<C:/Users/73547/MM-Hand 1.5技术报告/main.tex>) 相同：实测 `m_i` 作基底、有符号 LUT 局部斜率决定增益、双腱采用 `z⊥`、换向事件补绳、磁编进度碰撞而非伪张力测量。不同：报告的积分为裁剪误差累积并在换向清零，本计划改为持续条件积分、换向冻结且无泄漏；报告的配对项在 10 counts 死区和局部幅度/速率限制后生效，本计划只留 `k_pair` 和统一最终输出限制；报告的武装与检测窗口是当前实现设置，本计划都固定为同一编译期 500 ms，速度从 500 ms 的 `q_slow` 滑动均值计算；换向补偿把报告的增长量落实成绝对事件锚点，以适配每圈 `m_measured` 基底；增加主机 One Euro 加固定锚点滞回与参数整组同步。上述是**拟改的控制方案**，不是声称报告实验已经验证这些改动。

### 3.1 旧功能、旧格式与旧参数的完整清理表

用户已确定**不保留历史格式兼容**。实施时先把正在使用的调用迁到新接口，再删除旧代码和生成物；协议版本握手拒绝旧 P4，旧 CSV/旧固件不能静默按新格式解释。当前 v4 标定 JSON、当前多圈运行记录、Manual/Auto/LUT 的实际功能仍需读取，它们不是“旧版兼容”。

| 当前旧功能或格式 | 在 v4 中的处理 |
|---|---|
| `CA` 固件自动标定空实现；`CB/EA` 旧角度目标；`CC` START；`D8` RUN_FEEDBACK；`DE` 单电机直控；`D9` PAUSE；`EF` 旧 trace；`F5/F7` 可变 hold 开关 | 删除上下位机 schema/生成常量/构包/分发/状态解释；标定的 `CC` 调用迁 F6+D7+D3/E9，Feedback 改 F9，Direct 单电机改 D3，曲线改 FA/1D；保持是 ENABLE 的固定语义 |
| `D3` token0、不写 mask 就取当前 scope；旧 `F4/1B` 16 字节维护结果 | D3 强制明确 token/mask；只接受 F4/1B 的 v2 固定结果 |
| `0x0C` 按长度猜阶段性附加字段；`0x12` Manual v1/v2；`0x0E` 189/202 字节快照；短 `0x07` 故障包 | 只接受由 v4 schema 生成的完整精确长度；不补缺失字段默认值 |
| 无 `T2` 的旧触觉全量分片、旧校准诊断原因码、`0x18` 两种旧 trace body | 删除解析、发送和历史诊断映射；新全量必须有 T2/CRC；曲线/记录用 1D |
| `0x19` 硬件归零结果后 108 字节固定零填充 | v4 删除填充，上行改 `[token u32BE,successMask u32BE,failedMask u32BE]+T` 的 16 字节固定体；两端及测试同步更改 |
| `HAND_PROFILE=v1` 的 22 电机掩码分支、旧 Replay CSV 缺签名/缺发送状态的猜测导入、旧多圈锚点自动补齐、旧校准诊断文本 | 删除；Replay 只导入含当前 joint-order 签名、明确掩码和发送状态的现行 CSV；旧文件单独备份，不自动转换或播放 |
| 上位机 `comm_layer` 对历史命令的字符串入口、`protocol.py` 旧 builder、P4 旧 command switch、代码中遗留的 Pause 文本 | 跟同名协议一起删除；以静态搜索与生成器校验确保无僵尸入口 |

| `ControlConfig.h` 当前参数族 | v4 去向 |
|---|---|
| `kSingleTendonKp/Ki/IntegralLimit`、`kDualTendonKp/Ki/IntegralLimit` | 改为 JSON 中单腱/双腱各一组；**旧 Kp 单位不延用**，新 Kp 是乘有符号 LUT 局部斜率的无量纲倍率 |
| `kSingleTendonKd`、`kDualTendonKd`、两组 D 限幅和 `0.050 s` 速度滤波；`kSingleTendonDeadbandDeg` | 生产控制器 D=0，不暴露可调 Kd，删除 D 计算/状态/滤波/限幅及零值死区参数 |
| 近目标 Kp20、0.9/0.41 倍率、2.5°/3.0° 界点、所有单腱目标速度预测/前馈开关与时间常数 | 删除上下位机旧计算、宏、JSON/UI 字段和日志列，不能用旧值填新 `Kp` |
| `kDualTendonPairRecoveryEnabled/Gain/Deadband/Max/Rate` | 仅保留 JSON `k_pair`；`k_pair=0` 即关闭，不另设开关，死区/局部上限/局部速率删除 |
| `kSingleTendonDirectionalTakeupEnabled/Confirm/Initial/Rate/Exit/Max` | 保留必要位移、增长与限额到 JSON；`B_max=0` 即关闭，不另设开关；事件锚点保证只补一次 |
| 碰撞旧 200/600/750 ms 时间窗、旧收/放进度阈值 | 删除，两个检测阶段统一 P4 编译期 `COLLISION_WINDOW_MS=500`；分方向武装/停滞速度、最小误差、目标跳变/退让阈值迁 JSON |
| `kFeedbackDoneToleranceDeg/StableMs/RunTimeoutMs`（0.8°/500 ms/8 s）、`kDirectDoneAbsToleranceCounts/StableCycles` | 删除“到位后结束/超时离开控制”状态与参数；P4 `RUN_FEEDBACK_DONE/TIMEOUT` 和 `RUN_DIRECT_DONE` 自动释放事件一并删除；Feedback、Direct 均持续保持直到切换、STOP 或故障；UI 若显示误差直接由实时反馈计算，不作为退出门槛 |
| `kPidDiagLogEnabled/PeriodMs`、旧 trace 50 ms 周期 | 删除周期串口刷屏开关；FA/1D 的按需分频和可见丢帧计数替代 |
| `kPeriodSec`、编码器模数/断线码、`kFeedbackServoSpeed`、总线周期、硬件位置范围、主机租约、过载阈值/锁存 | P4 `config/` 保留编译期物理与硬安全参数，不开放给普通上位机调大；重复的 CAN/舵机失联时间常数集中到一处定义 |
| `kUpperWorkspaceWarningCounts/StableFrames` | P4 保留限位预警；仅状态显示，不并入比例增益 |
| `kTendonLutReleaseMarginCounts/TightMarginCounts` | 运行目标裕量移至 JSON，P4 另保留更严格的硬件绝对上限；标定专用的 500/900 counts ABS 限位裕量仍是独立校准参数，不混用 |

若静态搜索发现某个旧字段仍被标定/恢复或录制使用，先将调用改为上述明确的新数据源再删。删除后要以最新协议 schema 重新生成 Python/C++ 常量与文档，不能在源码里保留“兼容旧版”的隐藏分支。

## 4. 五个主页面：布局、操作与状态标签

常驻页眉：P4 设备/连接/直接接收时延、S3 设备/连接/直接有效帧时延、保存开关、全局 STOP。第二行是随状态改变背景颜色的系统徽章；文字保持可读，`UNKNOWN` 灰、进行中蓝、正常绿、需注意黄、故障/碰撞红。页脚 `System Log` 有可抓取分隔条，可上下拖高；不挤掉导航和 STOP。S3 启动 10 s 内区分“等待数据”“有字节无有效协议帧”“有效在线”“过期”“断开”；P4 中继不更新 S3 时钟，维护响应可证明 S3 在线而采集状态显示暂停。

| 主页面/子页 | 具体布局与动作 | 门禁与逐行标签 |
|---|---|---|
| Calibration / Joint Calibration | 左侧关节清单与未勾选框、实时 Raw、方向、Manual Min/Max；右侧本关节采集和保存阶段；Set Min/Max 点击立刻显示“采集中”。 | 300 ms 新鲜 Raw 快照优先、最多等待 1 s；忙/离线/运动/方向非法给原因，ACK→JSON→回读逐阶段反馈；失败保留草稿，不自动重写；`UNKNOWN/PENDING/READY/FAILED/OFFSET` 与安装告警。 |
| Calibration / Motor Calibration | 手指分组、未勾选关节、进度、运动/回零阶段、结果；P4 状态与故障原因显式。 | 当前关节准备至提交显示蓝 `CALIBRATING`，后续 `PENDING`，完成为质量 `READY/WEAK RETURN/HIGH FRICTION/FAILED`；完成不自动锁存 STOP；失败/取消清运行标识。 |
| Control / Feedback Control | 手指卡片、逐关节未勾选框、目标输入/滑条、`ENABLE Selected`、`Send Selected`、`Clear Faults`、ROS2 INPUT；逐行显示 Enc、Raw、控制 ABS、目标与当前角。 | 未选中或参数/标定未同步禁用；ENABLE 执行交接+当前位置保持，带 token 确认后 `ENABLED`；`UNKNOWN/DISABLED/ENABLED`，仅该关节碰撞时追加红 `COLLISION DETECTED`，故障另红标签。 |
| Control / Direct Control | 手指/电机清单、全部未勾选，ABS 输入、滑条、`ENABLE Selected Motors`、发送及 Clear Faults；每台真实位置/目标/在线状态。 | 按新鲜电机反馈保持当前位置；切换普通 owner，不使用角度 PI；只在最终第一批写入确认后显示 `ENABLED`；断线/未标定/超限给具体原因。 |
| Replay | 文件/轨迹概览、显式选择的关节与电机掩码、时间轴、播放/停止、目标/实测差曲线。 | 文件导入不自动勾选；开始时执行相同的授权交接与当前位姿保持；已滤波的录制目标不再滤波；显示真实 Feedback/Direct 标签与安全事件。 |
| Plot / Curves & Recordings | 可勾选但默认空的信号清单、时间对齐曲线、P/I/换向/配对/最终指令分量、记录与导出，标识丢帧。 | 控制 trace 用 P4 周期序号+时间，不凭主机到达时间拼伪同期数据。 |
| Plot / Tactile | 大致平面手形，拇指在右；每指指尖、远节、近节放置模块区；选择 `Fz/Fx/Fy/幅值`，统一色标、点位/合力提示。 | 指尖 `DP-S2015-Elite PXSR-STDDP03G` 52 点，指腹 `IP-S1610-Elite PXSR-STDIP03B` 25 点；S3 全量新鲜显示测点 XY 分布，只有 P4 摘要则仅位置+合力箭头与 `SUMMARY`，无数据仅空手形。 |
| Settings / Renumber Motor | 两按钮 `Scan All Buses`、`Renumber All`；BUS/当前 ID/逐台 New ID 表；右侧缩小到视口可见的完整关节—电机示意图，窄屏移到表下；固定文字说明“所有扫描到的电机还会把地址 84/85/86 设置为 254/254/0”。 | 必须有 P4 新鲜状态且停止；扫描 4 BUS×ID0–30 =124 候选，另单独探测中转/目的地址；每步与参数读回、失败恢复记录。 |
| Settings / Renumber Tactile | `Connect only one tactile sensor`；指尖0、远节1、近节2；`Scan Sensor`、Current ID、**初始空** New ID、`Renumber`、只读结果；无额外确认/取消/型号按钮。 | 必须直连 S3；零台/多台、未知型号、非法 0/1/2 或维护忙禁用；扫描状态与改号事务结果分开，断电验证提示只对应本次事务。 |
| Settings / Control Parameters | 单腱/双腱、反馈/积分、换向、碰撞、输出、ROS2 输入的分组表单；期望/P4 实际 revision；保存及 `Restore Defaults`。 | 运行中可保存但标 `Pending until idle`，下一次 ENABLE 前必须成功整组下发和读回。 |
| Settings / Diagnostics | P4/S3 时延和有效帧、协议/配置 CRC、总线路由、失联/过载/碰撞、传感器型号与原始日志、RTOS/控制 trace 订阅。 | 系统徽章可跳转；把 `ACK queued`、第一批总线写入、机械到位分开显示。 |

触觉 XY 必须从厂商坐标表按**点编号**一对一读取：指尖表 `指尖15mm PX6AX-GEN3-DP-S2015-Elite PXSR-STDDP03G.xlsx` 的 52 点，指腹表 `指腹S1610 PX6AX-GEN3-IP-S1610-Elite PXSR-STDIP03B.xlsx` 的 25 点；保留 XYZ 原始元数据，按坐标比例归一到各模块轮廓，不按数组下标拼矩形热图，也不合成 `_synthetic_contact_grid`。原始三轴/合力的物理单位和坐标单位需对照手册核定后再标，未经核定只显示“设备原始单位”。每模块新鲜度 600 ms，直连 S3 全量优先，其次 P4 有效摘要；没有新数据就清掉旧色块。先逐指按压验证 S3 原始 G0..G4 的手指顺序，不凭注释猜。

电机示意图保留用户原文件完整比例；示意图中的 0–26 是产品装配目标 ID，ID>26 明示不受产品控制映射驱动。选中状态与设备实际 active scope 分开：切换标签只是换视图，任何目标页 ENABLE 才释放旧普通 owner、显式设置新范围、锁定当前实测位姿并进入目标控制。全局 STOP 清待发队列、取消后续步骤并锁存安全停机；Clear Faults 仅在停稳且 P4 RESET 确认后解除可清故障，不自动 ENABLE 或恢复旧目标。

逐行标签统一由同一个设备状态快照生成，不能仅凭按钮点击更改：

| 运行模式 | 关节/电机常态标签 | 进行中与问题标签 |
|---|---|---|
| Joint Calibration | 未采集 `UNKNOWN` 灰；待执行 `PENDING` 灰；完成 `READY` 绿 | 采集/保存 `CAPTURING` 蓝；端点偏差 `OFFSET` 黄；失败 `FAILED` 红；失联 `OFFLINE` 灰 |
| Motor Calibration | 未标定 `UNKNOWN` 灰；所选待执行 `PENDING` 灰；成功 `READY` 绿 | 当前关节全过程 `CALIBRATING` 蓝；`WEAK RETURN`/`HIGH FRICTION` 黄；`FAILED` 红；STOP 后终止原因只读 |
| Feedback Control | 未授权 `DISABLED` 灰；实际 owner 已确认且保持中 `ENABLED` 绿 | 仅受影响关节 `COLLISION DETECTED` 红（可与 ENABLED 并列）；编码器/校准/参数问题 `FAULT` 红或 `UNAVAILABLE` 黄；执行中 `ENABLING` 蓝 |
| Direct Control | 未授权 `DISABLED` 灰；总线首批发送且状态确认 `ENABLED` 绿 | `ENABLING` 蓝、掉线 `OFFLINE` 灰、过载/范围问题 `FAULT` 红；不显示角度反馈碰撞标签 |
| Replay | 未播放的每行仍显示源控制模式真实 `DISABLED/ENABLED` | 播放中 `REPLAYING` 蓝；若借用反馈控制并发生碰撞，在该关节也追加红色碰撞标签；故障停止说明 |
| Plot、Settings | 不伪造控制使能状态；只读复用当前实际 `DISABLED/ENABLED` | 记录 `RECORDING` 蓝；ID 维护有逐步骤 `SCANNING/RENAMING/VERIFYING/FAILED`；触觉 `FULL` 绿、`SUMMARY` 黄、`NO DATA` 灰；诊断显示安全事件 |

页眉系统徽章分别显示 System（READY/STOPPED/FAULT）、Calibration（IDLE/RUNNING/FAILED）、Overload（CLEAR/LATCHED）、Collision（OFF 或事件数量）、Motor multi-turn（KNOWN/UNKNOWN）；每个徽章背景随自己的值改变，字体颜色仅选保证可读的对比度。点击徽章进入 Diagnostics。碰撞只在目标关节追加红块，不给所有未碰撞关节绘制绿色碰撞状态块。

## 5. ID 维护与 S3 SPI

### 5.1 电机批量改号和参数

上位机以 `(BUS,ID)` 标识扫描项，输入最终 ID 仅 0–252，253 作唯一中转，254 是广播不可写入最终 ID。扫描结果只访问 0–30 以快速发现产品电机；运行前另查四总线的 253、超出扫描范围的目标、全手最终 ID 唯一性。与未参与舵机冲突就拒绝；交换/循环先用 `旧→253→新` 编排可执行次序，并将每步观测写到事务日志。同 BUS 同 ID 多台无法区分，须拆开接线。旧/新/253 探测结果不明确时停批次、保留现场日志，重新连接先核实地址，不盲目重发 EEPROM 写。STOP 取消后续步骤。P4 维护租约期间暂停通常扫描、轮询和运动，但维持心跳；ServoBus 独占总线，依厂商协议执行 torque-off、解锁、写、再锁、回读。包括 ID 不变的行都设置 84/85/86 为 **254/254/0**，逐寄存器回读并断电后抽样复核；现有写寄存器路径尚不能当作飞特隐藏解锁机制已验证，必要时用原厂软件/总线记录核定，未核定的写法不能报告成功。成功后重扫路由，使受影响 Auto/LUT、ABS 限位、多圈锚点失效，保留 Manual；固定产品绑定不因 ID 改写而自动恢复运动。

### 5.2 PaXini 单台 SPI 编号

上位机 → **S3 直连 USB 串口** → S3 采集轮次之间的 SPI 维护事务 → 单台传感器；P4 的触觉 CAN 中继只能显示数据，不能为改号授权。UI 0/1/2 分别为指尖/远节/近节，对应传感器目标片选号和 `dest_id` 1/2/3。附件 `ID修改指令.xlsx`（Sheet1 D1:S5）提供：

```text
解锁             7A 01 00 01 00 A1 CE
写目标 0 / CS1  7E 23 00 01 00 01 22
写目标 1 / CS2  7E 23 00 01 00 02 23
写目标 2 / CS3  7E 23 00 01 00 03 E3
```

地址 `0x0023`、长度 `1` 用小端，两种写帧用厂商查表 CRC8 初值 `0xFF`。S3 唯一拥有片选和 SPI，先扫描恰好一台，记录来源/型号/长度，再发送一次解锁和一次写入；发送后读新旧片选，有效 CRC 全量数据只出现在新地址、旧地址不响应才报“即时生效”。超时/不明确时报告每一步观测而非再次盲写；独立提示断电再扫描，持久化成功与即时生效分开。附件没提供保存命令，不臆造。只可访问现有 CS1–3；其他编号报接线限制。型号与长度按硬件身份保存，绝不能从新 ID 反推；无法证实是 52 点 DP 或 25 点 IP 时拒绝改号。S3 改片选映射后仍以上报中的“指尖、远节、近节”语义排序；普通触觉摘要与 `T2` 全量使用唯一现行格式，维护结果独立帧与事务 token。P4 不加触觉改号代码。

## 6. 代码文件和职责迁移

```text
desktop/resources/config/
  control_parameters.default.json / control_parameters.json / control_parameters.schema.json
  upper_protocol_schema.json / s3_maintenance_protocol_schema.json / tactile_geometry.json
desktop/core/
  control_config/{models,store,codec,service}.py     # 验证、持久化、整组同步
  teleop_input/{one_euro,stationary_filter,service}.py
  tactile/{geometry,models,sources}.py                # 坐标、型号、数据源选择
  id_reset/{motor_workflow,s3_workflow,journal}.py
  communication/{现有帧编解码、ACK、状态归约、P4/S3维护码}.py
desktop/ui/
  browser_view.py                                     # 五主导航、状态栏、可拖日志
  web/{joint_calibration,motor_calibration,feedback_control,direct_control,
       replay,plots,tactile,renumber_motors,renumber_tactile,
       control_parameters,diagnostics}.py             # 只管叶页面呈现/交互
ESP32-P4/src/control/
  ControlParameters.{h,cpp} TransmissionModel.{h,cpp}
  FeedbackFilter.{h,cpp} ConditionalIntegral.{h,cpp}
  ReverseTakeup.{h,cpp} PairRecovery.{h,cpp}
  JointFeedbackController.{h,cpp}
ESP32-P4/src/tasks/motion_control/MotionControlService.cpp    # 调度与发指令
ESP32-P4/src/safety/policies/CollisionSafetyPolicy.cpp        # 统一 500 ms 保护
ESP32-P4/src/tasks/host_comm/                           # 版本握手/配置事务
ESP32-P4/src/tasks/servo_bus/                          # 独占总线、电机维护
ESP32-S3/src/tasks/TactileTask.cpp                      # 唯一 SPI 事务调度者
ESP32-S3/src/hal/HalTactile.cpp                         # 片选、帧和时序
ESP32-S3/src/maintenance/                              # 独立 USB 请求/结果
```

旧 `ControlSolver.cpp`、`DualTendonControl.cpp` 和 `pid.c` 在统一实测位置控制器通过对照后删除，不保留另一条能被误调用的 PID 路径。`MotionControlService` 只组合快照、调用控制器、输出诊断并排队 ServoBus；`CollisionSafetyPolicy` 只处理进度/持有，硬过载仍由安全层处理。桌面 `application_actions/hand_backend/comm_layer` 将配置、滤波和 ID 事务委托给专用服务，并删除只服务旧协议的门面方法；叶页面不直接写串口或猜 P4 owner。协议 schema 是数字与字段的唯一来源，生成 Python/C++ 常量与黄金向量测试。S3 维护与 P4 HostComm 在代码、设备门禁和串口上完全分开。

## 7. 上位机—P4 协议 v4：帧、命令和回包

### 7.1 基础契约与迁移

沿用 `[FE][LEN][CMD/TYPE][PAYLOAD][FF]`，`LEN=payload长度+2`，最大 LEN255/载荷253。当前协议工作树为 v3；升级到 v4，版本不匹配时仅允许握手、只读状态、STOP 与可清故障复位，不授权运动；**不解析旧版数据体、不转换旧目标命令、不保留旧版兼容分支**。现行多字节整型大端，新目标 `F9` 的 `float32` 与新浮点 TLV 明确小端，不能统称整个协议大端。常规上报尾部 4 字节 P4 时间戳为大端；校准/任务诊断时间戳在各自消息体中；`0x1B` 电机维护无尾时间戳。零值与缺失按 valid mask/协议哨兵区分：raw 断线 `0xFFFF`，角度断线 `0x7FFF`。沿用帧外壳并使用严格 v4 长度+业务 CRC/序号，配置包与独立维护必须校验数据 CRC。

保留现有 `F3 PROTOCOL_HELLO` 9 字节 `[version u8, control_model_crc u32BE, hardware_map_crc u32BE]`，升级 version=4 和控制模型指纹；运行参数 active CRC 单独同步，不能写进固件编译指纹。仍在使用的系统运行态值 `0 不可控/1 idle/2 feedback/5 ready/6 direct/8 rope init/9 recovery` 不改；Pause 枚举、处理、显示与生成常量删除，不设保留编号表。控制模式 `0 joint/1 direct` 不重排。`D7` 的最后一字节仍是 **require_can_feedback**，不是通用 control-enable。`ACK 0x06` 载荷 `[cmd u8, appliedMode/systemState u8, status u8, time u32BE]` 继续用于现行命令；带 token 的最终结果才能决定 UI ENABLED。ACK 码 0成功、1模式拒绝、2已入队、3缺 Manual、4数据非法、6 Manual CRC、7缺校准、8契约不匹配；v4 增 5忙、9缺运行配置、10版本不支持、11事务冲突、12反馈过期、13未使能。

### 7.2 全部现有上位机→P4 命令的去向

数字为十六进制；长度均不包含命令字/帧尾。表中 `删除` 意味着从 schema、生成常量、上位机构包/解析与 P4 分发器中全部去掉，v4 接收到该编号即按未知命令拒绝；不保留空处理器或旧版适配层。

| CMD | 载荷 | 当前用途 → v4 行为 |
|---|---:|---|
| CA | 0 | 已被上位机标定流程取代，当前 P4 仅返回“不支持” → 删除。 |
| CB | 84 | 旧 21 路完整目标 → 删除，GUI/ROS2/Replay 用带 token 的 F9。 |
| CC | 0 | 旧 START → 标定流程中的调用迁到 F6 释放+明确 D7 scope，再按需要调用 D3/E9；两端删除 CC。 |
| CD | 0 | 全局 STOP → 保留最高优先级，清队列和运动租约，锁存仍由 Clear Faults 清。 |
| CE | 0 | RESET/Clear Faults → 保留，不自动恢复运动或旧目标。 |
| D3 | 119 | `[token4, motorMask4, 27×i32BE, speed u16BE, accel u8]`；直控与上位机标定保留，P4 本身也强制非零 token 与非空 mask，删除零 token/空 mask 回退，最终看 1A。 |
| D7 | 9 | `[motorMask u32BE, jointMask u32BE, requireCanFeedback u8]` → 保留显式 scope，连接初始 mask0。 |
| D8 | 0 | 旧 RUN_FEEDBACK → F9/op1 原子申请，删除。 |
| D9 | 0 | PAUSE → 上下位机 schema、枚举、日志解释和状态转换全部删除。 |
| DA | 112 | 多圈快照 `[mask4,27×i32BE]` → 保留，最终看 17。 |
| DC | 0 | 主机心跳 → 保留，建议每 500 ms；P4 1 s 租约丢失停止运动。 |
| DE | 5 | 单电机 ABS 直控 → 改用 D3 单 bit mask，删除。 |
| DF | 189 | 单关节 Auto/LUT v4 快照 → 保留，兼容性与 Manual CRC 校验。 |
| E0 | 224 | 批量 ABS 限位 → 保留。 |
| E5 | 0 | 清全部 Auto/LUT → 保留维护用途，保留 Manual。 |
| E6 | 85 | 21 组 Manual 端点写入/清除 → 保留。 |
| E7 | 0 | 查询 Manual 端点 → 保留。 |
| E8 | 5 | 单关节 Manual Min/Max → 保留，Set Min/Max 分阶段核验。 |
| E9 | 4 | Rope Init 关节 mask → 保留，仅明确选中/授权标定。 |
| EA | 88 | 旧稀疏目标 → 改用 F9，删除。 |
| EB | 4 | 按电机 mask 清 Auto/LUT → 保留改号后失效。 |
| EC | 4 | 标定前舵机归零 → 保留，必须显式选中。 |
| ED | 4 | 状态同步 mask → 保留；增 bit7=运行参数，ALL=0xFF。 |
| EE | 1 | RTOS 任务诊断订阅 → 保留。 |
| EF | 1 | 旧控制 trace 开关 → 曲线改读 FA/1D，删除。 |
| F0 | 4 | 硬件 ABS rebase → 保留，结果看 19。 |
| F1 | 4 | Direct ABS 设零 → 保留维护用途，结果看 19。 |
| F2 | 4 | 校准内部直控释放 token → 保留内部清理，页面无 Exit。 |
| F3 | 9 | 版本+模型+硬件握手 → version4。 |
| F4 | 12 | 电机维护会话/探测/改号/寄存器操作 → 仅支持确定的 v2 结果 1B；删除旧维护结果解析。 |
| F5 | 1 | 单独的 Feedback hold 开关 → F9/op1 的启用语义始终保持当前位置，删除。 |
| F6 | 0 | 普通控制 owner 释放 → 保留，非 STOP、无旧目标恢复。 |
| F7 | 1 | 单独的 Direct hold 开关 → D3 首次启用始终保持当前位置，删除。 |

**新增命令**（数值不与当前表冲突）：

| CMD | 字节与格式 | 处理语义 |
|---|---|---|
| F8 CONTROL_PARAMETERS | 通用头 `[op u8, formatVersion u8, requestToken u32BE, transaction u32BE]` 10 字节；QUERY/CANCEL=10，BEGIN 再附 `[revision u32BE,totalBytes u16BE,schemaCRC u32BE,dataCRC u32BE]` 共24，CHUNK 再附 `[offset u16BE,count u8,data 1..192]` 共13+N，COMMIT 再附 dataCRC4 共14。 | `QUERY=0/BEGIN=1/CHUNK=2/COMMIT=3/CANCEL=4`；有界暂存，按 offset 覆盖检测与数据 CRC，约 5 s 过期；仅 idle 且无维护时原子 COMMIT，失败保留旧 active。格式为按字段 ID 升序的 TLV `[fieldId u16BE,type u8,length u8,value]`；int 大端、float32 小端；未知/重复字段拒绝。P4 返回真实 active revision+CRC，不仅回显输入。 |
| F9 FEEDBACK_TARGET | `[requestToken u32BE,jointMask u32BE,21×float32LE degree,operation u8]` =93 字节；op0 更新当前 scope 的稀疏目标，op1 ENABLE 在 P4 采集当前新鲜关节和电机位置并开始保持。 | op1 不把页面草稿角作为初始目标；op0 只合并掩码内关节，不清其他已有目标；无有效 scope/参数/校准/反馈则拒绝；每 token 有 1E 最终结果。 |
| FA CONTROL_DETAIL_SELECT | `[enable u8,jointMask u32BE,divider u16BE]`=7 字节。 | 受带宽限制的按关节周期日志；只选关心关节和分频，带丢包计数，控制任务不得阻塞。 |

普通模式交接确定顺序：取消当前 ROS2/Replay 目标生产者并使旧队列 epoch 失效 → F6 释放旧普通 owner（等 ACK 和新鲜状态）→ 若参数未同步则先 F8 完成整组配置 → D7 设置**当前页面明确勾选**的完整电机/关节 mask → Feedback 发 F9/op1，P4 捕获当前角/电机位置并持续闭环保持；Direct 发 D3 当前实测 ABS（非零 token），P4 持续保持最后目标。无需 F5/F7 或 Exit Control。待第一批 ServoBus 结果和状态确认后才显示 `ENABLED`。若 F6 无法释放或任何一步失败，新页面保持 `DISABLED` 并明确显示原因，不自动重复 ENABLE。浏览子页不执行交接。全局 STOP 可抢占所有步骤；Clear Faults 不再授权。反馈与直控硬件只允许一个 owner。

### 7.3 全部现有 P4→上位机消息的去向

数值、长度是既有 schema/解析约定；`+T` 表示尾随 u32BE P4 时间戳，具体扩展消息长度由带版本的 body 定义。

| TYPE | 内容 | v4 策略 |
|---|---|---|
| 01 SENSOR | 21×i16 映射角 counts+T | 保留；显示换算用 16384 counts/360°，哨兵 0x7FFF。 |
| 03 SERVO_ANGLE | 27×i32 logical ABS+T | 保留，ENABLE 当前姿态基底必须新鲜。 |
| 05 SERVO_TELEM | 27×速度i16/负载i16/电流i16/电压u8/温度u8+T | 保留，CAN/舵机健康与过载独立。 |
| 06 ACK | cmd/mode/status+T | 保留；入队≠执行。 |
| 07 FAULT_STATUS | overloadMask4+27×triggerLoad i16+T | 只接受完整现行长度；删除仅有 overloadMask 的旧短包。 |
| 09 SERVO_RAW | 27×i16 raw+T | 保留。 |
| 0A TACTILE | seq1+45轴摘要+validMask2+T | 保留；只提供合力/摘要，不能生成伪测点热图。 |
| 0B ENCODER_RAW | 21×u16+T | 保留，0xFFFF 为无效。 |
| 0C SYSTEM_STATUS | 系统态、owner、模式、scope、故障、碰撞、校准/反馈许可等 v4 body+T | schema 定义唯一精确长度；删除按 payload 长度逐段猜旧字段的解析。含 activeParameterRevision/CRC/valid、上次目标 token、配置错误原因。 |
| 0D MOTOR_ABS_LIMIT | validMask4+27×(min,max)i32+T | 保留。 |
| 0E JOINT_CALIB_SNAPSHOT | Auto/LUT v4、告警+T | 只接受当前完整体；删除 189/202 字节旧形态。 |
| 0F TACTILE_FULL | `T2` 分片、长度、CRC16、有效掩码+T | 只接受 `T2`；删除无 magic/CRC 的旧分片；有完整有效帧才显示 XY 点。 |
| 10 CALIB_DIAG | 校准分阶段诊断，自带时间戳 | 只解释 v4 诊断码；删除历史原因码映射，Motor Calibration 开始事件补出当前 joint。 |
| 11 TASK_DIAG | FreeRTOS 任务指标，自带时间戳 | 保留。 |
| 12 MANUAL_ENCODER_REFERENCE | Manual v3 表/方向/CRC/revision/commit+T | 只接受 v3 完整体；删除 v1/v2 解析。 |
| 13 SERVO_LOGICAL_RANGE | validMask+27×范围+T | 保留。 |
| 14 PRODUCT_FINGERPRINT | 模型/来源/映射指纹+T | 保留固件静态指纹；运行配置 CRC 另列。 |
| 15 SERVO_ROUTE_STATUS | 动态 BUS 路由/冲突/扫描代际+T | 保留，改号后重扫。 |
| 16 SAFETY_EVENT | 规则、严重度、owner、关节/电机、去重序号+T | 保留，碰撞给精确关节。 |
| 17 MULTITURN_RESTORE_RESULT | token、成功/拒绝位图+T | 保留。 |
| 18 CONTROL_TRACE | 旧共同周期/旧 detail 两种消息体 | 删除 P4 发布、上位机解析与旧订阅命令；Plot/录制直接消费 1D。 |
| 19 SERVO_REBASE_ZERO_RESULT | token4、成功位图4、失败位图4+T | v4 仅 12 字节内容+4 字节时间戳；删除旧 108 字节零填充。 |
| 1A DIRECT_TARGET_RESULT | token、请求/已发/拒绝 mask、状态+T | 保留；已发只是 ServoBus 同步写完成。 |
| 1B SERVO_MAINTENANCE | 维护 token、状态、探测/版本/阶段、84/85/86 回读 | v2 固定 19 字节体、无 T；删除 v1 16 字节体的解析与发送。 |

**新增消息**：

| TYPE | body（之后常规 `+T`） | 含义 |
|---|---|---|
| 1C CONTROL_PARAMETER_RESULT | `[version1,op1,requestToken4,transaction4,status1,detail2,activeRevision4,activeCRC4,receivedBytes2]`=23 字节 | BEGIN/CHUNK/COMMIT/QUERY 均回执；只有 COMMIT 成功且下一次状态包同 CRC 时 host 标记已同步。 |
| 1D CONTROL_DETAIL | version1、cycleSeq u32BE、targetToken u32BE、sampleTimestamp u32BE、actualDtUs u32BE、parameterRevision u32BE、jointId u8；P4 已授权目标、磁编原始/快慢、误差、窗口速度各 float32LE；I 门禁/换向/碰撞状态与输出限幅 flags；motorCount1，至多两组 `[motorId u8, measured i32BE, commanded i32BE, signedSlope/P/I/reverse/pair 各 float32LE]`。 | 每关节每周期的同源证据；P4 不可能凭 F9 反推 ROS2 原始目标，原始值由 host 以 targetToken 关联记录。计算明确长度并保证总载荷≤253。低优先级发送，有环形缓冲丢失计数。 |
| 1E FEEDBACK_TARGET_RESULT | `[token4,requestedMask4,appliedMask4,rejectedMask4,activeJointMask4,owner1,state1,reason2,status1,parameterRevision4]`=29 字节 | `ACTIVE_FIRST_BATCH_APPLIED/REJECTED/CANCELLED` 等最终结果；必须与最新 token、参数 revision、状态包匹配，才允许 GUI 显示 ENABLED。 |

状态同步 `bit7=CONTROL_PARAMETERS`；请求 `ALL=0xFF`。主机按**本次连接代际**+token 丢弃此前连接的迟到结果，P4 对同代 token 去重；断线/STOP 使旧 token 全失效。这是并发安全，不是旧版协议兼容。参数 TLV、F9 和新结果需 golden bytes 单元测试，特别覆盖 float 小端、整型大端、FE/FF 在载荷中、最大帧、丢分片、CRC 与重复事务。协议生成器支持经过显式校验的 F8 变长命令，同时从生成结果中删除全部旧命令/状态/包类型和旧编号保留表。

### 7.4 上位机—S3 独立维护协议

沿用 S3 直连串口既有 `FE/LEN/TYPE/PAYLOAD/FF` 外壳，指定 `CMD=0x31 TACTILE_MAINTENANCE_REQUEST`、`TYPE=0x32 TACTILE_MAINTENANCE_RESULT`；与现有 `0A` 摘要、`0E` 全量、`10` 诊断分开。独立 `s3_maintenance_protocol_schema.json` 生成解析常量。新请求固定 12 字节：`[version u8=1, op u8(0=SCAN/1=RENUMBER/2=QUERY), requestToken u32BE, scannedSelector u8, scannedModel u8(1=DP52/2=IP25), desiredUiId u8, flags u8=0, crc16 u16BE]`；CRC16 覆盖前 10 字节。SCAN 对无关字段用 `0xFF`，RENUMBER 必须引用本连接代际的唯一单台扫描结果。新结果固定 22 字节：`[version1,op1,token4,stage1,status1,oldSelector1,newSelector1,model1,pointCount1,oldValid1,newValid1,writeSent1,reserved1=0,s3TimeMs4,crc16 2]`；CRC16 覆盖前 20 字节。状态枚举 `0=FOUND_ONE/VERIFIED,1=NONE,2=MULTIPLE,3=UNKNOWN_MODEL,4=INVALID_TARGET,5=BUSY,6=WRITE_SENT_UNVERIFIED,7=TIMEOUT_AMBIGUOUS,8=VERIFY_FAILED,9=CANCELLED`，其中只有 `VERIFIED` 允许 UI 报即时成功；断电持久化由下一轮 Scan 另给结果。对同 token 重复的写请求只回上次观测结果，不二次发送 SPI 改号帧；丢电或重连后 token 不可信，先扫描。型号字段只能来自经过验证的只读身份信息或已核实的硬件注册，不能依据 UI 目标、扫描顺序或预期数据长度猜测；找不到可信身份来源就返回 `UNKNOWN_MODEL`。S3 在两采集轮之间暂时阻止并发采集，维护期间正常数据可短暂停止，上位机 S3 在线判断仍认维护有效帧。协议没有 P4 HostComm 分支。

维护串口 CRC16 复用现有 S3 `crc16Ccitt`：初值 `0xFFFF`、多项式 `0x1021`、不反射、无尾异或；两端用相同黄金向量核验。

## 8. USB、文档生成、测试和交付顺序

P4 固定通过扩展板自留的 USB Serial/JTAG Type-C 做运行 CDC 与断点调试，不回退 CH343 作为运行串口。Arduino 编译选项 `USBMode=hwcdc`、`CDCOnBoot=cdc`、`UploadMode=default`、`JTAGAdapter=builtin`，串口层用 HWCDC 事件；首次迁移可从微雪 CH343 Type-C 烧录。P4 原生 USB D−/D+ GPIO24/25，四条舵机总线 RX 21/23/27/33、TX 20/22/26/32，CAN TX47/RX48；S3 引脚以其 `Config.h` 与板实物复核后写入文档。OpenOCD 使用对应板卡固定 USB-JTAG 序号；调试时 CDC 与 JTAG 可在同一物理接口，命中断点控制任务停顿，恢复后重新检查通信/owner，不自动继续运动。遵循仓库 `docs/debug/esp32-jtag-debugging.md`：同一构建 BIN/ELF、OpenOCD 的 `gdb_memory_map disable`、detach/reset 前删除断点、OpenOCD 占用 JTAG 时不得 esptool 烧录；文档中“不能开串口监视”需按实际 CDC/JTAG interface 隔离验证后准确更新。

中英双语 AI README 的“安装、使用”必须包含 GPIO、P4/S3 分别接哪一个 Type-C、Arduino 全部选项、编译/烧录和断点调试；另说明 S3 改号直接连接、UI↔片选号换算、参数 JSON 与控制输入滤波。修改 `docs/ai/GENERATE.md` 的生成要求、证据抽取/源范围、渲染与验证；生成文件**不手改**，用已提交源码和导出图包按 **prepare → annotate → validate → publish**，证据缺失或内容陈旧拒绝发布。

实施及验证分阶段，任何阶段失败都可回到已备份固件、标定和多圈坐标：

1. **基线与协议固定**：备份 Manual/Auto/LUT、ABS 与多圈记录；记录当前 P4/S3 固件哈希、采样/外环/总线频率、控制与输入噪声、时延、快速换向、触觉 XY/型号和原厂寄存器证据。确认正在使用的 CSV/标定/多圈记录均为现行格式，旧文件只备份、不迁移到新运行目录。冻结严格 v4 协议黄金向量；不要把试验参数写成已验证最终值。
2. **控制与参数**：先 P-only 且 `Ki=0`、无新配对/换向，对单关节阶跃、双向三角波和静止噪声验证 20/500 ms 两个滑动均值并整定斜率倍率；再分别启用积分、双腱正交恢复、一次性换向、`q_slow` 碰撞。验证欠采样/NaN/跨圈、目标换向、限位、连接中断、卡住与不同 Bowden 布线。专门测从接触到碰撞 HOLD 的最坏时间，若 500 ms 慢均值导致危险漏检或迟报，停止该阶段验收并保留硬过载保护，不通过调低阈值掩盖时间延迟。日志必须能重建每周期 `m_measured, q_fast, q_slow, e, P/I/reverse/pair, u_cmd, saturation, collision`。
3. **界面/协议闭环**：五主导航、默认空选择、Feedback↔Direct 切换与当前位置保持、收到最终 token 后变 ENABLED、STOP/Clear Faults、无 Pause/Exit；配置运行中待应用、断线重连整组同步、未知 CRC 拒绝运动；一键恢复默认。测试无历史命令、无 v1/v2 Manual、无旧触觉分片和旧 CSV 解析；发送旧协议帧得到明确未知命令/版本拒绝。验证 One Euro 静止锚点不随噪声漂移、0.25° 内有意慢动作可能出现的保持延迟以及退出连续性。
4. **两类 ID 维护**：电机 124 地址扫描、全行 84/85/86、交换/环/253 冲突/部分失败与恢复、改号后的校准失效；S3 零/单/多个传感器、两型号、三目标 CRC/时序、新旧片选、生效与真实断电保存，未知型号与写入超时不报成功。S3 采集时维护互斥；P4 不得有触觉改号入口。
5. **触觉/校准/状态**：厂商 XY 点逐个注入假接触与实物点按；Direct S3 全量、P4 摘要、无数据三种 UI；Set Min/Max 一次点击完整结果；Motor Calibration 的 CALIBRATING 进入/退出；只对碰撞关节红牌；可拖高 System Log 与图片 fit。
6. **构建、烧录、实机验收**：按仓库 debug 指南以正确 Arduino FQBN 分别构建并烧录 P4/S3、确认 ELF 与 BIN 成对；恢复标定/多圈，重启桌面；分别测两个 USB 口 CDC/JTAG 与断点停止后的安全恢复。以至少一只单腱、一组双腱、指尖 DP、指腹 IP 验证前述场景，记录结果和未确认硬件证据；不把只通过单测的协议功能标为实机完成。最后再按生成工作流发布中英文文档。

验收结果须给出基线与新方案对照的关节 RMSE、超调、静止抖动 RMS、真实慢速动作检测延迟、换向误触发次数、碰撞误报/漏报、安全事件数、舵机目标抖动与串口丢帧。所有数值与图表记录配置 revision、固件哈希、腕姿/布线、负载条件；若安全事件增加或方向符号错，停止并回退该阶段配置。


## 最新实施修订：五指触觉改号

Settings / Renumber Tactile 扫描五根手指的全部15个可访问地址，按 Thumb、Index、Middle、Ring、Little 分组，每个响应目标显示 Current ID 与独立 New ID。允许不同手指重复使用0/1/2，同一指总线最终编号唯一。空输入保持原ID。已删除仅接一只传感器的页面限制。两类改号页面都提示：同一总线重复ID的多个设备无法区分及分别改号，需分开接入。交换需要一个空闲可访问地址；三个地址全占满时先断开一只。

SPI请求采用独立v2维护协议；型号/位置映射与新ID独立保存，禁止猜测未知型号。扫描、写入、验证及中断恢复分别维护，不自动重复EEPROM写入。具体使用说明见 docs/debug/id-reset.md。
