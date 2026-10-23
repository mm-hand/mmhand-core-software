# 自由运动控制调参

本流程只评价磁编坐标中的跟踪性能，不包含接触、抓物或碰撞检出验收。
当前执行计划已取消5分钟通信检查；运动前只做即时连接、反馈及坐标门禁。
运行入口为 `tools/debug/control_tuning.py`；指标和轨迹位于
`desktop/core/control_tuning.py`。正常 GUI、串口授权、使能、保护和记录链路均保留。

## 基线与使能门禁

每次试验使用一个新的输出目录，复制原始标定和当前运行参数，不覆盖旧记录。
必须显式指定 `--joint` 与 `--motion` 才能运行运动试验；该选项不绕过 P4 的使能门禁。
只检查所选关节的磁编和关联电机，不要求未选手指具有已知多圈坐标。

离线审计检查 LUT 覆盖、局部斜率、方向、Joint Calibration 和电机 ABS 限位。
内部工作区距离两端至少为 `max(5°, 标定行程的10%)`。
审计通过不等于设备坐标恢复成功：使能前仍需新鲜反馈和所选电机的可信坐标。

无运动通信检查始终要求控制禁用、无目标所有者，并检查选定传感器/电机在线和反馈年龄。
它可以记录 UNKNOWN，但这不授权运动，也不构成多圈恢复验收。
检查结束关闭串口，不启动后台长测。

## 使用

在仓库根目录运行，`CALIBRATION` 为备份的当前 `.mmcal.json`，`NEW_DIR` 必须不存在：

```powershell
python tools/debug/control_tuning.py audit --calibration CALIBRATION --output NEW_DIR
python tools/debug/control_tuning.py ros2-offline --calibration CALIBRATION --output NEW_DIR
```

确认所选关节坐标可信、现有故障已按正常流程处理后，才执行运动。
坐标无法恢复时，可以显式使用 `calibrate --joint N --motion` 重做所选关节Motor Calibration，保留Joint Calibration。
`--clear-before-test` 表示操作者明确要求本次准备时执行一次Clear Faults，不会在失败后自动重试。
例如先筛选 J13 的正向 0.5° 小阶跃：

```powershell
python tools/debug/control_tuning.py sweep --calibration CALIBRATION --output NEW_DIR --joint 13 --motion --clear-before-test --case B --trial step-0.5-0.5 --parameter kp --values 0.2,0.4,0.6,0.8,1.0
```

参数变更前释放控制，确认禁用和 ACK，保存参数后等待 P4 回读 APPLIED。
P 筛选关闭积分、配对恢复和换向补偿。
每次试验从当前位置保持开始，再以5°/s接近轨迹起点。
完成整组筛选后恢复该命令开始时的运行参数；不会自动推荐最后一个点。
异常时 STOP 并终止，不自动 Clear Faults、不继续下一组。
异常后的参数和日志保留用于分析，回滚应在禁用状态通过正常设置流程执行。
程序化紧急终止可在当前试验目录创建名为 `STOP` 的文件，脚本随后执行正常全局STOP并保存中断记录。
准备阶段也监测换向无响应，不能在控制器已进入保持后继续当作有效跟踪试验。

## 轨迹、数据和指标

生成矩阵包括 A 内部20/50/80%保持，B小阶跃、C阶跃及重复方波，D三角波，E升频正弦，
F快速换向、G配对恢复和K十分钟重复运行均覆盖三类波形。P为每档候选的三波形快速筛选。
H可选择内部保持并筛选Ki；Kd使用 `--parameter kd`，Ki/Kd 均按被测关节类型写入
`joint_ki` / `joint_kd`。`--base-kp`、`--base-parameters`明确指定非扫描项，配置归档在实验目录。
I需要用户实际摆放鞘管后，以 `--routing-label` 记录真实摆放；`--additional-joints` 显式指定其他关节，`--phase-deg 0/90` 选择同步或错相。不得把同一摆放重复记录为三种摆放。
J通过真正ROS2发布和订阅测试，需在已具备rclpy/std_msgs的mmhand环境运行，不能用离线仿真替代。
各阶段应根据上一阶段结果逐项选择，不能直接执行全部矩阵。

每组保存渐进写入的JSONL、完成后的样本JSON、指标、参数版本、固件指纹和正常记录器路径。
所有成功、失败和中断尝试独立保存。`export_control_tuning.py`可离线重建CSV和300dpi PNG、
可编辑SVG/PDF；不修改原始数据。packet_frames CSV及packets JSONL保留有效协议载荷，不能称为USB原始字节抓包。
跨任务采样可能比周期起点晚1ms，记录的有符号反馈年龄保留这一先后关系，不转成超大无符号数。
时间优先采用P4控制周期；原始记录含目标、q_fast、同周期q_slow与P/I/换向分量、
电机实测/下发位置、配对恢复量和有效位。缺失的同周期分量不能用异步数据补造。

计算RMSE、P95误差、稳态MAE、10–90%上升时间、连续300ms到位时间、超调，
正弦幅值比和相位延迟，以及目标反向和补偿启动两种起点的响应延迟。
小阶跃到位带为 `min(1°, max(0.15°, 幅值×10%))`，不动作不会判定到位。
双绳偏差由实际电机位置与LUT计算正交分量，它不是实测张力。
没有足够数据的指标返回无效或空值，不能报告零延迟或通过。

当前协议未直接提供逐关节饱和/积分冻结/碰撞武装的完整状态轨迹。
因此现有记录不能独立证明全部约束占比、积分条件覆盖或保护武装覆盖；这些项目仍需增加
可验证的同周期诊断后验收。当前入口还不进行性能评分后的自动参数推荐，也不覆盖默认参数。

## 2026-10-16 首轮前置检查记录（后续已重做Motor Calibration）

原始备份位于 `.debug/control-tuning-20261016-171324/baseline/`，
包含8个成功关节标定、原运行/默认参数以及修改前固件。
本次P4固件仅补充确认换向时清空积分；没有修改控制主公式、保护阈值或标定。

审计发现J08/M10在13–30°局部平台，暂不试验且不合成斜率。
J12/J13静态标定审计通过，但启动恢复后M15/M16/M17保持UNKNOWN。
无运动回读同时发现以下坐标：

| 电机 | 保存/请求坐标 | 实际回读 | 请求与回读相差 |
|---|---:|---:|---:|
| M11 | -461 | 3635 | -4096 |
| M12 | 8825 | 633 | 8192 |
| M13 | 10021 | 1829 | 8192 |
| M14 | 11021 | 2829 | 8192 |
| M15 | -429 | 3667 | -4096 |
| M16 | 14067 | 1779 | 12288 |
| M17 | -191 | 3905 | -4096 |

在这次实测中0x0B后的回读是请求坐标模4096。M10/M18/M19的单圈内请求能通过回读。
这是当前恢复过程的实测限制，不足以断言所有STS型号/固件均不支持多圈校正。
[飞特协议](https://ftservo.github.io/ftservo-wiki/reference/protocol/ft-scs/#49)
说明0x0B及型号/固件条件，但没有给出此处多圈恢复的充分保证。

现有驱动正确拒绝将上述不匹配结果标为可信。不得直接取模标定、清空圈数后沿用旧LUT、
关闭UNKNOWN门禁或通过运动积累圈数来规避问题。
需要确认实际舵机能力及受支持的坐标恢复方式，再验证反馈与目标使用同一坐标系。
在此之前，J12/J13的Kp/Ki、配对恢复、换向A/B和后续运动项目均未开始。

本轮已完成：1376项软件测试与架构/生成一致性检查全部通过；P4构建、烧录和Flash校验通过；
300秒电机禁用通信检查通过，运行参数保持原值。未发送运动目标。
离线ROS2滤波仿真中，输入7Hz、±0.05°噪声的静止段输出锁定，1°阶跃解除锁定约60ms、
到90%约320ms。这些是软件仿真结果，不是ROS2实链路或闭环性能。
完整当次结果和未执行Case记录在上述试验目录的 `status.json`。

后续按用户授权重做J12/J13 Motor Calibration，新LUT、限位和锚点均已保存。
本次实际运动数据保存在 `C:/Users/73547/mmhand_data/experiments/control-tuning/20261016-183731/`。
J13完成5档Kp的15组三波形筛选；J12筛选中出现反馈保护中断，失败样本同样保存。
数据还定位到周期起点与异步磁编快照相差1ms的无符号年龄误判；对应修正固件
（ELF SHA256: `87DD5B91B0040334DAC67A7947E44321CB72F5DD536CB44801E758C655E91245`）
已烧录，J12重标后完整通过另一个15组三波形Kp筛选。
后续又记录到周期锚点保存127ms导致上位机反馈发布延迟；周期保存已改成独立写入线程，
最新值队列只有一个待写快照，旧连接/旧坐标数据丢弃，显式坐标事务与最终保存仍串行确认。
慢磁盘注入测试通过，实机复验仍在进行。
这些仍是调参中间结果，不构成最终推荐参数。

## 2026-10-18 限位简化（用户拍板后实施）

输出边界收敛为唯一一层：LUT 标定区间 ± 可调扩放量
（`output.lut_release_margin_counts`=500 松端、`output.lut_tight_margin_counts`=1000 紧端）。
同一批删除的执行层：

- 会话可达范围（P4 ActuatorSafetyPolicy 的 LogicalRange 拒绝、控制器钳位、
  上位机直控钳位与直控准入范围检查；`ServoRejectReason::LogicalRange` 与
  `FEEDBACK_REJECT_ABS_LIMIT` 编号保留用于解读历史日志，规则标为 retired）。
- 电机标定 ABS 硬限位的强制执行（控制器钳位、RUN_FEEDBACK 准入、READY 门禁、
  上位机直控的"超出标定限位"拒绝；限位数据仍测量、上报并在 GUI 显示）。
- 输出速率限制（`output.max_step_counts` 与 `output.max_rate_counts_per_s` 两参数
  已从 schema/协议删除，参数表 39→37，revision 62）。
- 条件积分的"电机跟得上"门禁（A1：整条删除，±积分上限成为唯一防 windup 底线）。

保留不变：多圈 UNKNOWN 与圈溢出门禁、编译期硬件绝对边界、碰撞保护（q_slow，
武装 8°/s、停滞判定）、换向检测/确认/清积分/冷却、换向补偿仍为 max_counts=0 关闭。
当前固件 ELF SHA256: `D5835881B1FB0946020ADABD8C362801155C38EE3024DE1B710272BDC8A33956`。
删除限速后大误差瞬间指令一步到位，旧 Kp 数据（限速生效期测得）只作先验；
Kp 将在 {0.6, 0.8, 1.0} 三点重筛，波形为 0.2 Hz 方波/三角/正弦，
幅值取关节运动范围的 10% 与 80%（用户 2026-10-18 指定，待实跑确认）。

## 2026-10-18 控制器第二阶段简化（用户逐项拍板）

条件积分：删除"目标稳定"门禁（积分从纯静态保持修正改为持续误差积分器，
  保留快慢同号、0.1° 死区、同号 200ms、未饱和、Tracking 五条件）；
  删除饱和当周期增量回退（窗口裁剪冻结保留）。
换向/take-up：砍为三态（Tracking/Pending/Takeup）+ 单计时器（candidateSince）。
  拉绳公式收敛为"指令 = PI原始值 与 (锚点+方向×takeup_counts) 取更靠收绳方向者"，
  关节走 ≥exit_travel 提前收兵，电机走到靶子自动失效（无超时、无 Failed、
  无斜坡/沉降/冷却）；删除 REVERSAL_NO_RESPONSE 遥测与调参工具的自动停车。
  换向参数 9→4（confirm_deg/confirm_ms/takeup_counts/exit_travel_deg）。
参数：积分门禁 A1、JSON 单绳 ki=0；碰撞武装阈值临时设 100°/s（参数级关闭，
  试验期间不武装不触发；逻辑未动）。参数总数 37→32，SET 载荷 137 字节。
当前固件 ELF SHA256: `F86AAE5E07F3512973AEEFA81B051A36A8B9AB75DB7B74C18438FD3CD6DA86F8`。

## 2026-10-19 Kp 重筛结果（rev63，J13，0.2Hz，P-only，ki=0/拉绳关/配对0）

用户指定规格：小幅值=行程10%（±5.2°）、大幅值=行程70%（±36.4°，由80%下调）。
全部通过标准 2° 包络守卫，9/9×2 批完成。

| Kp | 小方波 | 小三角 | 小正弦 | 大方波 | 大三角 | 大正弦 | 大正弦相位 | 大正弦幅值比 |
|---|---|---|---|---|---|---|---|---|
| 0.6 | 4.73 | 2.47 | 2.85 | 31.92 | 5.72 | 6.03 | 0.166s | 1.049 |
| 0.8 | 4.21 | 2.00 | 2.27 | 29.86 | 4.60 | 4.55 | 0.121s | 1.041 |
| 1.0 | 4.01 | 1.65 | 1.88 | 30.07 | 4.17 | 3.73 | 0.096s | 1.036 |

RMSE 单位度。三角/正弦全指标随 Kp 单调改善且无过冲惩罚；方波三档持平
（误差由舵机全速渡越主导，Kp 无法压缩）。误差构成：方波=边缘渡越尖峰、
三角=速度滞后锯齿、正弦=峰顶相位滞后+顶部结构性短缺 0.86°（弯到头，与 Kp 无关）。

补充（80% 行程 ±41.6° 试验档，已被 70% 规格取代）：目标峰顶恰好压在审计包络
边缘，kp0.6 全过、kp0.8 方波过冲仅 0.57° 但三角反向过冲 >2° 触发守卫；
另两次停机为单帧 >6° 尖峰，真实过冲不可能一帧跳变 600°/s，结合 CAN 统计
err0=1 判定为编码器瞬时坏帧/振动误读——**编码器信号质量列为硬件待查项**。
试验档案：`kp-small-rev63-r2-20261018`、`kp-large70-rev63-20261019`（官方）、
`kp-large-rev63-r2/r3/r4-20261019`（80% 档与守卫调试，含异常记录）。
结论倾向：**Kp=1.0**（10%/70% 两档全指标最优、无过冲代价），待用户拍板后写入
JSON 并补一条"大幅值三角波 + 生产配置（拉绳开）"验证。

## 2026-10-19 Kp 全量战役（用户扩大规格：Kp 0.6→2.0 步长 0.2 × 食指四关节 × 两幅值）

合计 ~115 条新轨迹（两场战役）。J12 重做 Motor Calibration 成功（M16 锚点重建，
**J12 恢复可信任运行**，此前 0x0B 恢复一直被拒）。

| 关节 | 小幅值上限 | 大幅值上限 | 小幅值趋势 | 大幅值趋势 |
|---|---|---|---|---|
| J12 双绳 | **2.0 无惩罚** | ~1.2（1.4 越包络） | 单调降至 2.0 | 单调降至 1.2 |
| J13 单绳 | ~1.4（1.6 过冲停车） | ~1.0-1.2（1.2 越包络） | 单调降至 1.4 | 降至 1.0 |
| J14 单绳 | **2.0 无惩罚** | ~1.2（1.4 越包络） | 单调降至 2.0 | 单调降至 1.2 |
| J15 单绳 | **~0.6-0.8（1.0 过冲停车；0.8 已比 0.6 差）** | ~1.0（1.2 越包络） | **非单调** | 降至 1.0 |

关键发现：
1. **单全局 Kp 无法同时满足四关节**（J12/J14 想高、J15 想低）——逐关节 Kp 需要
   参数模式扩展，是后续控制方案的第一项依据。
2. J15（DIP，远端轻质量）小幅值非单调：kp0.8 起 RMSE 回升、1.0 方波过冲停车——
   振荡型关节，提高 Kp 前需要阻尼手段。
3. 大幅值上限全部落在 1.0~1.4 区间（无速率限制下的大阶跃过冲）。
4. 编码器单帧尖峰两次截断试验（±41.6° 档），CAN 统计 err0=1——硬件待查。
5. J15 kp=1.0 补测（过冲阈值放宽至 6° 后三波形完整）：三角 3.52°/正弦 4.21°
   优于 kp0.8，但同参数的前一次运行在第一拍就冲出 3°+（本次方波边缘过冲仅
   0.16°）——**同参数两次运行行为不同 = 阵发不稳定**。J15 的问题不是固定
   "Kp 上限"而是间歇振荡；其可靠选择仍为 0.6，提高增益需先有阻尼手段。

试验档案：`campaign-*` 与 `campaign2-*` 系列（含每关节 overview.png 总览图）。
当前 JSON 残留战役中断值，收官时按拍板结果重写。

## 2026-10-20 Kp 改为按关节类型 5 组

参数表 32→35：删除单/双腱共享的 `kp`，新增 `joint_kp.aa/mcp/pip/dip/cmc`
五组；SET 载荷 137→149 字节，参数 schema 版本 1→2。设置页按
AA/MCP/PIP/DIP/CMC 五组配置，无逐关节覆盖。关节索引按现有 21 关节拓扑：
J00–J19 每四个一组依次 AA/MCP/PIP/DIP，J20 = 拇指 CMC。

默认值（2026-10-21 中指 PID 调参后）：AA=1.3、MCP=1.3、PIP=1.0、DIP=1.3、
CMC=1.0；Kd AA=0.08、MCP=0.02，其余 0；Ki 全 0。`pair_recovery_gain=0`、
`takeup_counts=0` 继续作为默认值，避免默认启用未验证的补绳/配对恢复。
本轮不做自动识别；波形结论保持不变：0.2 Hz 正弦对 Kp 区分度最好，三角可作
交叉验证，方波只用于过冲/安全守卫。

## 2026-10-21 LUT 全表直线化 + Ki/Kd 五组 + D 项

标定完成后（`desktop/core/host_calibration/math.py:build_tendon_lut`，无条件）
对每根腱的 LUT 直接做首末点连线：

- 取 LUT 第一个点和最后一个点；
- 中间所有点替换为这两点直线上的值，整条 LUT 变成一条直线；
- 端点值不变，因此输出限位不变；双腱每根腱各自独立直线化；
- 没有阈值、没有端部检测、没有开关，也不保留旧 LUT 形状。

已有标定包不会自动回溯修正；需要重做相应关节的 Motor Calibration 才会生成
新的直线 LUT。

控制参数表 35→43→42：删除单/双腱共享的 `ki_per_s`，新增
`joint_ki.aa/mcp/pip/dip/cmc` 与 `joint_kd.aa/mcp/pip/dip/cmc` 各五组；
积分限幅、噪声门限、确认时间统一并入 `joint_ki`，单双腱共用一个
`integral_limit_counts`；删除 `single_tendon` 组。最终 42 参数、
SET 载荷 177 字节、参数 schema 版本 4。默认 Ki=Kd=0，因此不改变
现有行为。Kd 单位秒，D 项为 `-Kd·s(q)·q̇`，q̇ 取 20 ms 快窗均值的差分，
25 ms 一阶低通、±400°/s 限幅；只在 Tracking 相位生效，换向/补绳/碰撞保持
与反馈不新鲜时置零。D 分量不新增遥测字段，可由
`pid_counts − proportional − integral − reversal` 从现有同周期数据还原。

## 2026-10-21 第二小时：积分与快速换向自动标定

运行入口：`tools/debug/integral_reversal_autotune.py`；纯逻辑与指标：
`tools/debug/integral_reversal_core.py`；报告重建：
`tools/debug/integral_reversal_report.py`。只使用中指 J08–J11
（设备标定包含的唯一四个关节），Kp/Kd 冻结为 AA=1.3/0.08、MCP=1.3/0.02、
PIP=1.0/0、DIP=1.3/0，CMC 未调。

**换向识别（离线）**：12 组 `confirm_deg ∈ {0.3,0.6,1.0,1.5}` ×
`confirm_ms ∈ {20,60,100}` 全部回放。合成套件覆盖大幅/小幅真实换向、
慢速/快速、静止 7 Hz 微抖动和 0.2° 伪反转。`0.6°/20 ms` 取得
TP=51、FP=0、FN=0、p90 延迟 0.50 s；`1.0°/20 ms`（原值）在同一套件上
出现 11 FP / 11 FN，主要漏检 1.0° 与 0.6° 小往返。候选未部署，原因见下。

**积分**：每类六段保持（20/50/80%，上下两个接近方向，3.2 s/段），
Ki=0/0.1/0.2。MCP 基线 J_I=1.306，0.1→1.516、0.2→1.762；
DIP 基线 1.286，0.2→1.435；AA 的 0.1/0.2 为 1.092/1.246，但主运行中
AA Ki=0 基线因一次 `control disabled` 中止，无同流程基线；PIP 的
M13 在保持时触发 `load near overload`（≥800）全部中止。最终 Ki 全 0。

**补绳/退出**：固定离线选出的 `0.6°/20 ms`，Ki=0。
`takeup=0/20/40/80` 与 `exit=0.15/0.25/0.50` 在有界范围内测试：
MCP takeup=20 的 J_R=0.991，AA takeup=20 为 0.876；但 MCP takeup=0
基线因 M12 负载中止，缺少单腱同流程基线；联合验证中 AA/MCP/PIP 的
J_I 有改善，但 MCP J_R 从 0.868 降到 0.935（约 7.7%），DIP 基线再次因
M14 负载中止，联合验证不完整。按规范回退：**takeup=0、exit=0.25**，
`0.6°/20 ms` 也不部署。

**最终状态**：Applied revision 366 / CRC `F7903FE9`；Ki 全 0；
`confirm_deg=1.0`、`confirm_ms=20`、`takeup_counts=0`、`exit_travel_deg=0.25`；
`pair_recovery_gain=0`；控制 disabled；fault 0x10 已通过将 active scope 设为 J08–J11（0x7C00/0x0F00）并 RESET 清除。完整数据在
`run_data/integral_reversal_autotune_20261021_1645/`，其中
`report.md` 由选择 JSON 重建；原始 `trial_index.json` 在运行器修复前只含
validation 行，不能当作完整 trial 索引。

**本轮限制**：Host 通过状态快照读取控制 trace，USB 批处理使有效采样约
20–25 Hz，指标使用 P4 时间戳而非固定 100 Hz；检测器只有合成目标流证据，
没有真实遥操作日志；PIP/DIP 的 M13/M14 负载接近 800 阈值，这是硬件/机构
问题，不能通过调参绕过；AA 与单腱的 `integral_limit_counts` 实际共享，
没有单双腱分别整定。

## 2026-10-21 第三小时：schema v5 分组参数 + 复测

P4 固件与 Host 参数 schema 升为 **v5（58 参数，SET payload 241 B）**：
`joint_ki.integral_limit_counts_{aa,mcp,pip,dip,cmc}`、`reversal.confirm_deg_*`、
`reversal.takeup_counts_*`、`reversal.exit_travel_deg_*` 按关节类型独立；
`integral_noise_deg`、`integral_confirm_ms`、`pair_recovery_gain`、碰撞阈值、
反馈窗口、LUT 裕量和 ROS2 时序仍为全局。固件 ELF
`F5B015DB460AE9D101C33CB7D6D8555EBF1E2C011208E73EBE045BF3797EF8DA` 已烧录并校验。

复测结果（负载看门狗阈值提高到 1000 counts）：

- AA：Ki=0/0.1/0.2 的 J_I=0.805/1.156/1.374，Ki=0 最好。
- MCP：Ki=0/0.1/0.2 的 J_I=1.317/1.476/1.573，Ki=0 最好。
- DIP：2.5°/s 降速复测 Ki=0/0.1 的 J_I=0.659/0.823，Ki=0 最好。
- PIP：M13 即使在 2.5°/s 也达到 1000 counts，三档 Ki 与联合验证均中止；
  需先检查 PIP 机构/腱绳，不能靠调参绕过。
- 换向检测器：12 组离线回放仍选 `0.6°/20 ms`（TP=51/FP=0/FN=0）。
- AA takeup=0/20/40 的 J_R=1.329/0.930/0.881，最终 AA 采用 `takeup=40,
  confirm_deg=0.6`，并在最终 Apply 后复测 J_R=1.227、valid。
- MCP takeup=0 在 5°/s 与 20°/s 下均因 M12 到 1000 中止；3°/s 下 takeup=0/20
  为 0.914/0.877，但最终全速验证仍中止，故 MCP 保持 0。
- PIP/DIP/CMC 保持 takeup=0；DIP 的 takeup 未部署。

最终 Applied **revision 422 / CRC `7A230ECB`**；Ki 全 0、AA `confirm_deg=0.6`、
AA `takeup=40`、其余类型 `confirm_deg=1.0/takeup=0`、exit 全 0.25、
`pair_recovery_gain=0`；控制 disabled；fault 0x10 已通过将 active scope 设为 J08–J11（0x7C00/0x0F00）并 RESET 清除。完整报告：
`run_data/integral_reversal_v5_final/report.md`。

## 2026-10-21 第四小时：schema v6 渐进 takeup + 负载冻结

主参数仍 58 项（payload 241 B）；`reversal.takeup_counts_*` 更名为
`reversal.takeup_max_*`，上限提高到 2000。另开独立 ramp 参数帧（16 项、
payload 73 B、命令 0xFB/0xFC、回读 0x1E）：五组 `b0/vb/load_limit` 加
全局 `takeup_timeout_ms`。固件实现 PDF Eq.(7) 的**叠加式**渐进：

```text
b_i(t) = min(Bmax, B0 + vB*t)
raw_i  = PI_raw_i + motorDir_i * b_i
```

负载任一轴 ≥ `load_limit` 时冻结增长；到 1000 counts 或超时退出；0.25° 退出。
Host 看门狗仍为 1000。固件 ELF
`F337BEBEC0C5F485518DF38E7AC370BBA5B56A2DBA3F7876527299AADC58FAD2`
已烧录；最终 revision 435 / main CRC `4EFD32EB` / ramp revision 14 /
ramp CRC `59E6952F`，control disabled、faults=0。

AA J08 实测（Bmax=40, B0=5, vB=50, load_limit=900, exit=0.25°）：
`reversal_counts` 非零比例从固定下限的 3.6% 提升到 **10.1%**，实际 b 峰值
19 counts，峰值负载 744 counts，11/11 换向事件在超时前退出，J_R=0.901。
说明渐进叠加确实让 takeup 参与，而不是像旧下限那样被 PI 命令覆盖。

MCP J09 当前机械卡在上端（89–90°，标定上限 90°，M12 负载 1000），降到
Kp=0.2 也无法向下移动；已停止运动、恢复安全参数、faults=0。需人工把 MCP
退回内部工作区后再测。PIP 仍因 M13 到 1000 未完成；DIP 在 MCP 卡死后
enable 被取消，未完成。新增 `tools/debug/takeup_ramp_autotune.py` 做
Bmax 阶梯（40→…→2000）与 B0/vB 细化；MCP/PIP/DIP 需机械恢复后再跑。

### MCP 测试轨迹上端限制（2026-10-21 补充）

自动测试的 MCP 轨迹增加 74° 上端帽：`build_hold_plan`、`build_reversal_plan`、
`build_validation_plan` 均接受 `high_cap`，实际最大目标角：

- hold 60.48°
- reversal 62.48°
- validation 40.20°

全部 < 75°。`_run_plan` 在使能前还会检查 J09 当前角：若 >74.5° 直接拒绝，
避免在 89–90° 卡死状态下再次启动测试。曲线见
`run_data/analysis/mcp_capped_plan.png`。

### 中指测试的相邻手指与近端保持（2026-10-21 补充，22:05 更正）

**更正：** 关节顺序是 J00–J03 小指、J04–J07 无名指、J08–J11 中指、
J12–J15 食指、J16–J20 拇指。食指 root 是 **J12**，不是 J04；J04 是
无名指 root（M05/M06，当前不在 P4 总线上）。旧记录把 J04 写成食指是错的，
工具检查已改为 J12。

- 跑 J09/J10/J11 前要让**食指 J12–J15** 让开。本轮已通过维护流程松开食指
  腱绳并作废其标定，活动标定缩到 J08–J11；无法由软件直接确认时必须由操作者
  在预检时显式确认（`--assume-index-aa-clear`）。
- 测试 J09 时固定 J08（中指 AA）；测试 J10 时固定 J08+J09；测试 J11 时
  固定 J08+J09+J10，避免近端关节自由摆动把 MCP/PIP/DIP 带偏。
- 旧记录中的 `run_data/...` 路径已在 2026-10-21 清理；新的运行产物在
  `logs/` 下。

## 2026-10-21 第五小时：活动标定路径修复与自动换向重跑（未完成）

- 20:40 提交 `66233cf` 删除 `desktop/resources/config/calibration_bundle.mmcal.json`
  后，维护放绳流程找不到活动标定，第一次食指放绳失败；已从
  `mmhand_data/device_state/.../calibration_bundle.mmcal.json` 恢复到该路径。
  `mmhand_data` 随后已整体删除，备份在 `logs/calibration_backup/`。
- 食指放绳：J12–J15 通过正常维护流程 Reset Zero 并放 1500 counts，标定作废，
  P4 活动 scope 自动缩到 J08–J11（`0x0F00/0x7C00`）。
- J08 用 2°/s 受监控移动从 -28.5° 回到 -9.7°（保留标定，faults=0）。
- AA 回归（`logs/ramp_v7_20261021_2127/aa/`）：`Bmax=40, B0=5, vB=50` 入选，
  valid、11/11 退出、engage 5.29%、peak 432、J_R 0.661；R2 的 `b0=2` 候选因
  engage<5% 或 peak 904 被拒。
- MCP：第一次 trial 在 enable 阶段把 J09 带到 **-12.04°**，M12 实际 **-6951**
  （J09 LUT 约 -2358…2695 counts），再上抬时 M12 负载 1000、关节不动；已停止
  MCP/DIP/PIP，未部署新值。需要人工恢复/放绳并重做 J09 标定。
- 最终参数已恢复默认：**revision 446 / CRC `7A230ECB`，ramp 25 / `59E6952F`**；
  AA takeup=40、其余类型 0、MCP Kp=1.3/Kd=0.02、Ki=0、pair=0，控制禁用。

## 2026-10-21 第六小时：自动换向重跑（AA 安全中止，MCP 仍阻塞）

23:18 重新开始前先做无运动预检（`takeup_ramp_autotune.py --preflight-only`）：
J08=13.69°、J09=-12.19°、J10=-1.01°、J11=50.76°，J12–J15 仍不可读；
控制禁用、faults=0，活动 scope 仅 J08/J09（`0x300/0x1C00`）。

- **MCP 仍不可测**：J09 当前 -12.17°，M12 逻辑位置 **-11625**，而 J09 的 LUT
  只覆盖 **-2358…2695**，标定锚点为 -3814；相差约 2.3 圈。直接使能会把
  M12 拉回 LUT 范围，属于未恢复状态，本轮跳过。
- **AA 首次试验安全中止**：23:30 应用 tuning base（revision 447）后，从 13.69°
  以 5°/s 引导到测试下限 -25.8°；第 7.9 s 实测角到 **-30.26°**，越过 2° 包络，
  Watchdog 触发 `SafetyAbort: measured angle -30.26 outside envelope`。原始
  `aa_max40_vb50_b05.jsonl` 显示 actual/slow 分离、M10/M11 指令在
  -673…+1000 counts 间振荡、负载 ±900；没有形成有效 trial，不能据此调参。
  释放后 J08 自行回到 4.70°，M10/M11=26/352。
- PIP/DIP 未运行（用户本轮只批准 AA；且活动 scope 只有 J08/J09）。
- 中止后已恢复默认参数：**revision 450 / CRC `7A230ECB`，ramp 29 /
  `59E6952F`**，AA takeup=40、其余 0，控制禁用、faults=0。

下一步必须先做机械检查：J08 双腱的松弛/静摩擦和磁编稳定性（本轮表现为
粘滑式振荡，不是 Kp/换向参数问题）；建议先以 2°/s 受监控移动到 -9.7° 附近，
再从该点重跑 AA。MCP 需人工重新放绳/回零并重做 J09 标定后，才能进入换向
参数矩阵。

**23:44 复测（用户批准）**：先用 2°/s 受监控把 J08 从 4.79° 移到 -9.8°
（207 样本、9.25 s，末帧 actual -9.8 / slow -9.79，负载 0/-20），移动本身
稳定。23:46 从 -9.7° 重跑 AA，仍在 `center_0.20_hold` 段振荡，实测到
-27.53°，Watchdog 于 -30.18° 再次 `SafetyAbort`。同角度段对比：

- 21:48 成功轮：target -15.88° 时 M11 实际 ≈ 992、M10 ≈ -26；
- 23:46 重跑：同一 target 段 M11 实际 389–780、M10 -129…-515。

J08 的关节角与双腱电机位置对应关系已偏移约 600 counts，且释放后 J08 自行
回到 -1.65°（M10/M11=-112/392），符合双腱松弛/滑移，不是 Kp、Kd 或换向
参数能修正的问题。**本轮没有形成有效 trial。** 再次恢复默认：
revision 455 / CRC `7A230ECB`，ramp 34 / `59E6952F`，控制禁用、faults=0。

结论：AA 必须先重新张紧双腱并重做 J08 Motor Calibration（保留 Joint
Calibration），确认同一角度下 M10/M11 回到 LUT 附近后，才能重跑换向矩阵；
MCP 仍需人工恢复 J09/M12 并重做标定。PIP/DIP 本轮未动。

## 2026-10-22 00:18 J08 Motor Calibration 成功；AA 换向重跑仍受共模预紧限制

### Host 坐标事务修复

J08 标定前两次都在 rebase 后立即失败，日志为
`J08 logical ABS zero failed: previous servo ABS rebase is awaiting its result/new feedback`，
底层是 `MULTITURN save failed: multiturn anchor motors are not active in calibration snapshot`。
原因是 rebase 结果一到达就调用了 `invalidate_calibration_snapshot_scope`，把
J08/M10/M11 从快照活动 scope 中移除；随后新的 turn-zero 锚点保存检查
`active_motor_mask` 必然失败，capture mask 永不释放，下一次 rebase 被拒绝。
修复两处：

- `host_calibration/runtime.py::_rebase_and_wait` 在返回前等待
  `_servo_rebase_zero_capture_mask` 清零，确保新锚点在 `prepare_joint`
  使 scope 失效之前落盘；
- `calibration_snapshot.py::update_snapshot_multiturn_anchors` 允许最近一次
  `scoped_*_motor_mask` 记录的已失效 scope 继续保存新坐标锚点，不再要求
  这些电机仍在 `active_motor_mask` 中。

修复后 `test_calibration_snapshot.py`、`test_host_calibration_runtime.py`、
`test_host_calibration_flow.py`、`test_servo_coordinate_retry.py`、
`test_multiturn_restore_retry.py` 共 **207 passed**。

### J08 Motor Calibration 结果（00:18:30）

- success=true，assembly warning=0，自动行程 **2957 counts**
  （start 12214 / tight 15171 / zero 12442），
  startServoAbs -424 / tight 808 / zero 808；
- startDeg -5.0°、tightDeg 59.97°、totalDeg 64.97°；LUT 14 点；
- 新 LUT：M10 -428…858，M11 1329…101；
- 标定最终执行了 Rope Init 与 dual LUT zero，控制禁用、faults=0。

标定后恢复到 -9.71° 时，LUT 模型为 M10 13 / M11 908（共模和 921），
实测 M10 45 / M11 374（和 419）：**共模仍低约 502 counts**。双腱在标定后
没有回到 LUT 的公共张力，`pair_recovery_gain=0` 时控制器不会修正它。

### AA 换向重跑（部分结果）

- `aa3` 第一条 R1 有效：`Bmax=40, B0=5, vB=50, confirm=0.6°/20ms,
  exit=0.25°`，engage **5.43%**、exit 100%、峰值负载 **364**、
  `reversal_counts` 峰值 **15**、score **0.725**；
- 随后第二条试验在引导/低端越界 **-28.12°**，`SafetyAbort`；
- `aa4`（把 AA 引导速度改为 2°/s）仍在第一条试验低端越界 **-27.90°**，
  0 条有效 trial。abort 期间 `reversal_counts=0`，问题是共模预紧，不是
  takeup 本身。
- 最终恢复默认：**revision 465 / CRC `7A230ECB`，ramp 44 / `59E6952F`**，
  AA takeup=40、其余 0，控制禁用、faults=0。未部署新参数。

下一步：先把 J08 共模预紧补回来（机械张紧，或在确认 LUT 正确后小步试
`pair_recovery_gain`），再重跑 AA 识别；`aa3` 的 40/5/50 只是候选证据，
不能当作已部署结论。MCP J09/M12 仍需人工恢复后重标。

---

# P4 native USB stability investigation

## Runtime route and confirmed failure

The preferred runtime route remains expansion-board USB Serial/JTAG (GPIO24
D-, GPIO25 D+), Hardware CDC, CDC On Boot enabled. CH343 is an alternative
only after native USB isolation demonstrates an unresolved problem and the
UART route itself passes acceptance. Ports are identified by USB identity,
not by assuming COM numbers stay constant.

On2026-10-16, the old GDB timeout/rescue flow reported `detached`, but direct
OpenOCD examination found BOTH P4 cores still `halted`, at0x4ff0275a.
Explicit OpenOCD resume restored106295 serial bytes over2s without any reset
or flashing. This proves a debugger-exit failure mechanism. It does not prove
that every historical no-data incident has the same trigger.

An additional confirmed reset mechanism was reproduced on the diagnostic
firmware: the stock Espressif `esp_common.cfg` GDB-attach hook executes
`reset halt` when flash access requires disabling memory protection. The
OpenOCD log explicitly reported that action; the next `[USB_BOOT]` reported
`reset=12` (JTAG), with a retained previous uptime. RAM calibration was
cleared. This was an implicit debugger reset, not evidence of damaged Flash.
Both launchers now set `ESP_FLASH_SIZE 0` BEFORE loading the board config,
retain memory protection, disable flash-map probing and use hardware
breakpoints. Uploading remains a separate Arduino operation.

GDB also returned inconsistent FreeRTOS task context/registers in some
sessions. Such virtual-thread output alone is NOT evidence of actual memory
corruption or download mode. Use OpenOCD hardware core state and independent
serial observations to corroborate it.

Evidence (local, ignored):

- `.debug/usb-jtag-drained-first.log`: timed-out GDB/rescue.
- `.debug/usb-jtag-real-cores.log`: both cores remained halted.
- `.debug/usb-jtag-acceptance-baseline/`:10 successful corrected JTAG cycles
  followed by fresh protocol connections, plus5 software reconnects, on the
  original flashed firmware. These older observations did not yet include
  reset diagnostics and cannot exclude the implicit vendor attach reset.

The updated diagnostic firmware passed100 software reconnects, then exposed
the stock attach reset (`.debug/usb-firmware-final-acceptance/`, FAILED).
After disabling the flash attach hook,10 JTAG cycles passed with matching
calibration retained (`.debug/usb-live-attach-acceptance/`). However, its
subsequent20-second host-not-reading case FAILED: Windows could no longer
configure COM12 (error31). Physical replug restored communication. This
remaining USB/driver failure is distinct from the verified attach-reset
fix and prevents claiming complete stability. Physical and soak results
must be recorded separately as they complete.

The user subsequently moved P4 to the computer itself. Windows topology
confirmed the parent changed from the external `VID_35D6&PID_2510` hub to
`USB\\ROOT_HUB30` (`.debug/usb-direct-topology.json`). Before the comparison
could run, all serial devices disappeared: both Windows PnP and pyserial
reported no P4/S3/CH343. `.debug/usb-direct-restore.log` therefore failed on
port absence before calibration import; no firmware was flashed or reset.
Direct-port pressure/physical testing remains pending device enumeration.

A subsequent inventory first identified the actual CH343 as COM13,
VID1A86/PID55D3, serial `5AE6038249`; S3 remained COM14 and P4 native USB
was absent. A user RST with passive115200-baud UART0 capture produced
`ESP-ROM:esp32p4-eco2-20240710`, `SPI_FAST_FLASH_BOOT`, segment loads and
`entry 0x4ff29ed0`. This proves ROM selected Flash and transferred to the
second-stage loader; it does NOT prove the application completed startup.
Native USB was still absent after RST. Evidence:
`.debug/p4-ch343-boot-capture.bin` and `.log`. The minimal isolation sketch
adds independent CH343 text logging on Pico UART0 RX38/TX37 before/after
HWCDC initialization and once per second. This UART is diagnostic text only,
not a second control interface.

The minimal sketch compiled and uploaded/verified through COM13. Esptool
identified P4 revision1.3/MAC80:f1:b2:d0:6b:a5, confirming CH343 belongs to
the expected board. ELF SHA256:
`003B45F59A0EA8E1247E08642504D323DA3CBBDBCC60F3F183577C73A54B8270`.
UART captured monotonically increasing sequence1900..2600 and uptime20..27s,
heap442988, zero USB RX/reset counts. Native USB still did not enumerate.
TX counters here measure bytes accepted by HWCDC, NOT delivery to a host:
its disconnected policy may accept/drop data. Evidence:
`.debug/usb-probe-build.log`, `.debug/usb-probe-upload.log`,
`.debug/usb-probe-uart.log`. No business tasks are included in this image.
The production flashed-hash marker is removed while the probe is deployed,
preventing accidental debug with the production ELF. Production BIN/ELF and
the previous marker remain backed up; restoration is required after isolation.
A known-working CH343-cable-to-native-port swap is pending manual action.

After that cable swap, P4 native COM12/JTAG enumerated again, on the same
external-hub path1-7.1 formerly used by CH343. The first7s native read still
returned0 bytes although14 bytes were accepted for output. Matched probe
ELF live attach found both IDLE cores running and loop sequence60061;
interrupt enable0x20C, SOF/control requests observed, and OUT endpoint read
pointer14. Native USB's existence and a completed OS write are therefore
insufficient evidence of healthy bidirectional transport.
After two live inspections and an explicit diagnostic endpoint write
(`EP1.val=0x55`, `EP1_CONF.val=1`), native read recovered43720 bytes/5s;
another read recovered35802 bytes/4s with RX counter28. No reset/reflash
occurred. There was no read between the earlier live inspections and this
endpoint experiment, so it does NOT isolate the endpoint write as the cause
of recovery. No production endpoint-recovery patch is justified yet.
Evidence: `.debug/usb-probe-jtag-state.log`, `usb-probe-jtag-registers.log`,
`usb-probe-jtag-seed.log`, `usb-probe-after-seed.log`,
`usb-probe-recovered.log`, `usb-probe-cable-swap-pnp.json`.

After recovery the probe passed3 consecutive20s stopped-reading cases,
receiving48906/48750/49086 bytes, with RX count40/52/64. Then the original
production BIN/ELF was restored and flash verification passed on native COM12.
Joint calibration0xFF00, motor calibration0x100 and ABS limits0xC00 were
restored/read back from the device-specific JSON; enabled=false, scopes0,
selections empty. The unchanged production ELF hash is69BE0D6F...470FC2C.
On the known-working cable,100 software reconnects and20s stopped-reading
recovery passed (`.debug/usb-known-cable-production/`). This result does not
isolate cable replacement from the intervening debugger experiment/reflash,
and does not imply physical or2h acceptance. Hardware tests remain separate.

### Physical replug reproduction and endpoint-only recovery

The FIRST physical replug on that same known-working cable reproduced zero
RX after10s (`.debug/usb-known-cable-physical/`, FAILED). A read-only live
debug inspection found business tasks alive, enabled USB interrupts0x20C,
EP1_CONF=0 and IN_EP1_ST=1 (empty read/write pointers). After clean debugger
exit, a separate5s serial read still returned0 bytes. Thus read-only debugger
attachment alone did NOT recover this occurrence.
Only setting EP1_CONF.WR_DONE, with no FIFO payload write, then recovered
266240 bytes/5s. No chip reset/flash/calibration restore intervened.
Evidence: `usb-physical-known-cable-readonly-jtag.log`,
`usb-physical-after-readonly.bin`, `usb-physical-flush-only-jtag.log`,
`usb-physical-after-flush-only.bin` under `.debug/`.

`tools/debug/patch_p4_hwcdc.py` now backports a guarded empty-endpoint commit
in the producer's existing HWCDC critical section before arming IN_EMPTY.
It acts only on P4 and the observed EP1_CONF=0/IN_EP1_ST=1 signature, publishes
an empty packet, and neither fabricates payload nor resets the peripheral or
chip. It leaves nonempty endpoint pointers alone. The script accepts only
the audited official SHA256 or its exact patched form, saves the original,
and rejects unknown modifications. `build_debug.ps1` resolves the installed
Arduino core version and applies/verifies this prerequisite. Tests for both
driver patches and debugger cleanup:10 passed. Patched HWCDC SHA256:
`5FFD3BF0421554C337362A74CBDECED9E1417F6C3D1EC7937E5EDFB2E177608B`.
The patched firmware's physical validation is pending; endpoint-only recovery
is proven for this occurrence, but durable prevention must still be tested.

The patched production firmware was built and uploaded/verified through
native COM12. ELF SHA256:
`6258195D4007FF6EFBA0BAB0D7B3C45A47E24779E4283D8FFE229A5D0A5DA812`.
The source CRC remains0x4D8F76FE because this change is in the separately
audited Arduino core; use the ELF hash to distinguish builds. Calibration
0xFF00/0x100, limits0xC00 and disabled/empty invariants passed after restore.
Initial patched acceptance passed10 reconnects and20s stop-reading recovery
(`.debug/usb-empty-in-initial/`); physical watcher is ready separately.
The FIRST patched physical replug PASSED: native COM12 protocol returned,
12s fresh monitoring passed, joint/motor masks remained0xFF00/0x100 and
control/scopes stayed disabled/empty. Boot diagnostics retained the same
boot2/reset11 from upload, uptime230..235s: this replug did not reboot the
P4. `.debug/usb-empty-in-physical/physical.jsonl` records the actual USB
absence edge and result. Remaining physical/cold/JTAG/soak cases are pending.
The complete patched physical run subsequently PASSED all10 actual unplug/
replug cycles, with valid protocol, calibration0xFF00/0x100 and disabled/empty
control on each12s recovery observation. This repeats the same known-cable
condition that failed on its first cycle before the empty-IN fix. Cold starts,
patched JTAG stress and the2h soak remain separate pending cases.
The5 actual full cold starts subsequently PASSED as well. Each had
boot1/reset1(POWERON), retained=0, valid protocol, restored0xFF00/0x100
calibration and disabled/empty control (`.debug/usb-empty-in-cold/`). A
separate active-RX unplug test is needed because the10-cycle observer closes
its connection before each unplug. `usb_live_unplug.py` observes that separate
case, requires RX/TX threads to close within5s, then12s fresh recovery.
The matching final BIN/ELF, upload markers, FQBN/options, calibration JSON,
driver diff and SHA256 manifest are archived in
`.debug/usb-empty-in-delivery-6258195D/`.
The active-RX unplug case also PASSED transport/thread lifecycle: the link
had received7.45MB/75042 packets before removal; both RX/TX threads closed
in0.118s, then the next connection received641399 bytes/6428 packets over
12s with compatible protocol and no transport error. Physical removal's
ClearCommError is expected; it did not leave threads running.
This particular run's initial/final snapshot masks were0/0, so it is NOT
evidence that the loaded8/1 calibration set survived. The tool now accepts
explicit expected masks and otherwise reports calibration_verified=false.
Known JSON calibration was restored/read back again before the final full
100-reconnect/10-JTAG/7200s-soak run, which requires0xFF00/0x100 throughout.
Evidence: `.debug/usb-empty-in-live-unplug/observations.jsonl`,
`.debug/usb-final-soak-calibration.log`, `.debug/usb-final-empty-in-acceptance/`.
The final run passed100 reconnects,10 JTAG sessions and20s backpressure;
its7200s soak was then STOPPED AT THE USER'S REQUEST. The last periodic soak
record was90.2s, not2h. The process was interrupted and confirmed exited;
do not count the long-run or its final reconnect as passed. Evidence and an
explicit cancellation marker remain in the final acceptance directory.
No recurring follow-up was created (the attempted creation was rejected).

## Changes

`gdb_batch.py` continuously drains OpenOCD output into an individual log.
Its readiness timeout no longer depends on a blocking `readline`. Both the
batch tool and manual OpenOCD launcher install `esp_debug_cleanup.tcl`:
remove all break/watchpoints on all targets BEFORE resuming halted cores.
For P4, all three trigger CSRs on each core are also explicitly cleared and
read back while halted: a killed GDB reproduced failed logical breakpoint
removal. If OpenOCD warns that its metadata is inconsistent, restart that
server before further debugging; batch sessions already shut it down.
The batch tool also polls hardware state after GDB exits and requires every
core to be running; failed cleanup is a nonzero result. OpenOCD is shut down
gracefully. No automatic board reset or motor authorization is added.
An ELF hash matching the upload marker and recorded USB identity are
mandatory. Reusing a server requires matching target family, adapter identity,
no flash-attach reset and the cleanup hook; rejection leaves that server alone.

Close native CDC monitors before starting OpenOCD. Use the matching flashed
ELF/BIN; halt/resume interrupts feedback, and motion must not resume
automatically. A GDB timeout can remain a failed debug inspection even when
hardware cleanup successfully restores communication.

The desktop records per-connection byte/frame counts, errors and RX/TX
thread status independently of rate counters. Failed startup distinguishes
no bytes, no valid frames, handshake rejection and stale status. Opening a
port records DTR/RTS disabled; recovery never toggles them automatically.

P4 low-rate existing text logs add `[USB_BOOT]`, `[USB_DIAG]`, `[USB_TASK]`
and `[USB_TX]`: reset reason, firmware source CRC, boot/phase/uptime,
heap/stack/task counters, USB reset/byte/command counts, last activity,
short/zero writes and maximum write duration. Short writes/zero writes are
observations, not asserted SDK timeout causes. No wire packet, control
formula, ROS2 topic or calibration format changes.

Diagnostic snapshots update RAM once per second, with text emitted every5s
through zero-wait log admission. `.noinit` keeps a checksummed previous
snapshot across compatible software resets. Power-on/brownout discard it;
ROM download/flash may overwrite it. The boot counter is therefore a
retained-session counter, NOT a persistent lifetime counter. No diagnostic
flash writes are performed. No independent UART logger is enabled until
the CH343 supply path is verified.

## Driver and board evidence

Before this investigation's empty-IN patch, installed Arduino-ESP323.3.10
HWCDC.cpp was byte-for-byte identical to
the official tag: SHA256
`c5ed5fdd05aa0df9b74d390812643599223f96b459256718d0ad328aeaba6a8a`.
Do not attribute its pre-existing upstream fixes to local modifications.
The new guarded commit is recorded separately in `.debug/HWCDC-empty-in.diff`.
The P4
SDK direct FIFO-write backport is separate and checked by the build script.
Backups, official-source comparison and manifest are under the directory
recorded in `.debug/usb-investigation-current.txt`.

The supplied v8d7 KiCad schematic netlist matches these PCB connections:

- USB VBUS and the buck output share `/5V`; reverse-current/power sequencing
  remains a hardware investigation item.
- D2 PRTR5V0U2X pin1 is `/USB_SHIELD_1`, not direct GND. Its relationship to
  the connector's physical shell and transient return must be checked.
- USB has two5.1k CC pulldowns and routes D-/D+ to H1 pins19/20. Actual
  signal integrity and supply transients cannot be established from the
  netlist or a successful ROM reset.

Official references: [Waveshare schematic](https://files.waveshare.com/wiki/ESP32-P4-Pico/ESP32-P4-Pico-datasheet.pdf),
[ESD pin definitions](https://assets.nexperia.com/documents/data-sheet/PRTR5V0U2X-Q.pdf),
[Espressif hardware guidance](https://docs.espressif.com/projects/esp-hardware-design-guidelines/en/latest/esp32p4/schematic-checklist-esp32p4.html).

## Repeatable acceptance

Disconnect GUI device links first. The script sends no motion targets,
requires disabled control/empty scopes and selections, and stops on the first
failure without resetting or flashing. Restore the backed-up calibration
through the normal verified import before requiring its expected masks.

```powershell
python tools/debug/usb_acceptance.py --port COM12 `
  --cycles 100 --jtag-cycles 10 --soak-seconds 7200 `
  --expected-manual 0xFF00 --expected-auto 0x100 `
  --output .debug/usb-final-acceptance
```

The default backpressure case stops host reading for20s, then requires a
fresh protocol connection. Preserve JSONL observations and each JTAG log.
Physical10 unplug/replug cycles,5 cold starts, unplug during connection,
bare-board/cable/power comparisons and supply measurements are separate
manual cases; software disconnects must NEVER be counted as physical cycles.

`usb_physical_acceptance.py --serial 80:F1:B2:D0:6B:A5 --cycles 10 --mode unplug
--output .debug/usb-physical` records real absence/presence edges and verifies
fresh state plus calibration restore. Run a separate5-cycle `--mode cold`
case: it additionally requires POWERON reset diagnostics, not just a USB
reconnect. Wait for READY/CYCLE_PASSED before the next physical action. The
observer never simulates an unplug, resets a board or enables motors.

`tools/debug/usb_probe/usb_probe.ino` is an explicit isolation sketch using
the production native adapter, incrementing text frames and no motor/CAN
code. Flash it only as a documented comparison, retain the production
BIN/ELF, and restore that matched pair and calibration afterwards.
