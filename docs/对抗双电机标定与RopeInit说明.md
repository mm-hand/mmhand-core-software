# 对抗双电机标定与 Rope Init 说明

本文档说明当前对抗双电机关节的自动标定流程，以及上位机 `Rope Init` 按钮的最新行为。这里的“对抗双电机”指一个关节由两根互相拮抗的绳/电机控制，例如主电机拉向正向角度，副电机拉向反向角度。

## 相关代码位置

- 下位机自动标定：`ESP32-P4/ServoBoardMain/src/calibration/AutoCalibrationTask.cpp`
- 下位机状态机 READY 判定：`ESP32-P4/ServoBoardMain/src/system/StateMachineTask.cpp`
- 上位机 Rope Init：`desktop/ui/ui_control_panels.py`
- 电机和关节映射：`ESP32-P4/ServoBoardMain/src/hardware/HardwareMap.cpp`、`desktop/ui/ui_tendon_guard.py`

## 当前关键参数

### 下位机自动标定参数

| 参数 | 当前值 | 说明 |
|---|---:|---|
| `kDualAutoStartDeg` | `-3.0 deg` | 对抗双电机关节标定下端点 |
| `kDualAutoTightDeg` | `80.0 deg` | 对抗双电机关节标定上端点 |
| `kDualPrimaryTightLoadThreshold` | `175` | 主电机 POS80 阶段负载阈值 |
| `kDualSecondaryPreloadLoadThreshold` | `175` | 副电机预张紧阶段负载阈值 |
| `kReverseMinFollowDeg` | `12.0 deg` | NEG3 阶段必须先看到的关节跟随角度 |
| `kAutoCalibrationOverloadLoadThreshold` | `330` | 自动标定过程负载保护阈值 |
| `params.tightLoadHigh` | `200` | 单电机自动标定负载阈值 |

### 上位机 Rope Init 参数

| 参数 | 当前值 | 说明 |
|---|---:|---|
| `ROPE_INIT_STEP_COUNTS` | `50 counts` | 每次 Rope Init 调整的电机 ABS 步长 |
| `ROPE_INIT_TIMER_MS` | `50 ms` | Rope Init 周期 |
| `ROPE_INIT_TIMEOUT_S` | `8 s` | Rope Init 最大运行时间 |
| `ROPE_INIT_SINGLE_WORKSPACE_MIN_DEG` | `0.0 deg` | 单电机关节进入工作空间的判定下限 |
| `ROPE_INIT_DUAL_TARGET_DEG` | `40.0 deg` | 对抗双电机关节 Rope Init 中位目标 |
| `ROPE_INIT_DUAL_TOL_DEG` | `0.3 deg` | 对抗双电机关节中位停止容差 |

## 对抗双电机自动标定流程

### 目标

标定后得到该关节的两个机械边界：

- 反向端点：`-3 deg`，由副电机拉到机械限位得到。
- 正向端点：`80 deg`，由主电机拉到机械限位得到。

同时记录两侧电机的 ABS 限位，使后续 READY 判定和负反馈控制都有明确的控制边界。

### 执行流程

1. 等待主电机、副电机和磁编码器都有有效反馈。

2. 先松副电机：

   ```text
   secondary target = secondary current ABS - 4800 counts
   ```

   目的是让主电机正向拉紧时不被副电机明显干扰。

3. 主电机拉 POS80：

   - 主电机持续正向收绳。
   - 负载达到 `175`。
   - 电机仍在收绳，但关节角窗口内基本不再变化。
   - 该阶段不要求关节必须先跟随一定角度，因为关节可能一开始就在正向机械限位附近。

4. 主电机保持 POS80，不松开。

5. 副电机预张紧：

   - 副电机开始收绳。
   - 主电机保持 POS80 ABS。
   - 副电机负载达到 `175` 后，认为两根绳都已进入绷紧状态。

6. 进入 NEG3 同步换边阶段：

   - 副电机继续收绳。
   - 主电机同步等速放绳。
   - 两个电机方向相反，步长一致。
   - 先要求关节角实际跟随至少 `12 deg`。
   - 跟随达到后，不再用负载判断成功。
   - 当窗口内关节角停止明显变化，同时两个电机 ABS 仍在继续变化，认为副电机已经拉到反向机械限位。

7. 生成标定结果：

   - `NEG3` 作为起点。
   - `POS80` 作为终点。
   - `0 deg` 按 `-3 deg ~ 80 deg` 的比例插值得到。
   - 双电机关节的 `angleMax` 为 `80 deg`。

8. 写入运行时标定配置和 ABS 限位：

   - 主电机 ABS 右端限位取 POS80 阶段的主电机 ABS。
   - 副电机 ABS 右端限位取 NEG3 阶段的副电机 ABS。
   - 标定完成后不再执行最终松绳，两根绳保持当前绷紧状态。

### 串口日志关键字

调试时可以重点看这些日志：

```text
DUAL_BEGIN
DUAL_SLACK_SECONDARY_BEGIN
POS80_FOUND
DUAL_PRELOAD_SECONDARY_BEGIN
DUAL_PRELOAD_SECONDARY_FOUND
DUAL_EXCHANGE_NEG3_BEGIN
CHECK_DUAL_EXCHANGE_NEG3
DUAL_EXCHANGE_NEG3_FOUND
DUAL_RESULT
```

其中 `CHECK_DUAL_EXCHANGE_NEG3` 中的 `load=... ignored` 表示 NEG3 阶段会打印副电机负载，但成功条件不再依赖负载。

## 单电机自动标定相关变化

单电机关节仍使用原本的单电机流程，但当前逻辑已经做过两点调整：

1. 正向拉到最大端时不要求关节先跟随一定角度。

   原因是关节可能一开始已经接近正向机械限位，如果强制要求跟随角度，会导致无法正确识别端点。

2. 反向释放寻找最小端时要求关节至少跟随 `12 deg`。

   这样可以避免电机只是空转或绳状态异常时，被误判为反向机械端点。

## READY 工作空间判定

READY 判定位于下位机状态机 `activeJointsInControlWorkspace()`。

当前逻辑要求 active scope 中所有关节：

- 对应电机多圈状态已知。
- 对应电机 ABS limit 已标定。
- 关节标定结果有效。
- 当前角度在工作空间内。

当前工作空间判定形式为：

```text
-tolerance <= joint angle <= angleMax + tolerance
```

也就是说，READY 判定和 Rope Init 停止条件是两套逻辑：

- READY 只判断是否已经处于允许控制的工作空间。
- Rope Init 是上位机主动发送电机 ABS 命令，把关节带回期望位置。

## Rope Init 最新行为

`Rope Init` 现在不是单纯的 `+50` 一次性动作，而是一个短周期自动过程。

点击按钮后，上位机会：

1. 检查当前是否在 Manual 的 Joint Channels。
2. 检查串口连接。
3. 检查下位机已经 ENABLE，且状态处于 `IDLE` / `READY` / `RUN_DIRECT`。
4. 检查 active scope 内电机没有 `TURN UNKNOWN`。
5. 根据当前 active scope 生成 Rope Init 任务。
6. 每 `50 ms` 计算一次所有任务的目标。
7. 把所有任务合并成同一个 `targets[MOTOR_COUNT]`，一次性下发 `CMD_MOTOR_POS_ABS`。

### 单电机关节行为

单电机关节的目标是进入工作空间。

规则：

```text
如果当前角度 >= 0 deg：
    不动作

如果当前角度 < 0 deg：
    主电机 ABS += 50 counts
```

因此单电机只要已经进入工作空间，就不会继续收绳。

### 对抗双电机关节行为

对抗双电机关节的目标是回到中位 `40 deg`。

规则：

```text
如果 abs(current_angle - 40.0) <= 0.3 deg：
    不动作

如果 current_angle < 40.0 deg：
    主电机 ABS += 50 counts
    副电机 ABS -= 50 counts

如果 current_angle > 40.0 deg：
    主电机 ABS -= 50 counts
    副电机 ABS += 50 counts
```

这样可以保持对抗关系，一根收、一根放，让侧摆关节回到中间位置。

### 并行性

Rope Init 是并行的。

如果 active scope 内有多个关节需要 Rope Init，上位机会在每个周期内：

1. 读取所有相关关节当前角度。
2. 读取所有相关电机当前 ABS。
3. 计算所有关节本周期的 motor target。
4. 合并成一个完整 `targets` 数组。
5. 一次性发送给下位机。

因此单电机关节和双电机关节可以同时动作，不需要等待某个关节先完成。

### 防互相影响策略

上位机会为每次 Rope Init 建立 `seen_motors` 集合。

如果两个任务试图使用同一个 motor，会跳过冲突任务并提示。正常映射下，一个 motor 只属于一个 joint，因此不会互相覆盖目标。

### 停止条件

Rope Init 会在以下情况下停止：

- 所有关节达到目标。
- 超过 `8 s`。
- 丢失关节角反馈。
- 丢失电机 ABS 反馈。
- 下位机状态不再是 `IDLE` / `READY` / `RUN_DIRECT`。
- active scope 内出现 `TURN UNKNOWN`。
- 用户执行 E-STOP、切界面或启动其他测试流程。

## 使用建议

1. 自动标定完成后，先确认状态栏没有 `TURN UNKNOWN`。

2. 在 Joint Channels 中选择需要控制的 active scope。

3. 点击 `Rope Init`。

4. 对单电机关节，观察其是否进入 `0 deg` 以上工作空间。

5. 对对抗双电机关节，观察其是否回到 `40 deg` 附近。

6. Rope Init 完成后，再进入负反馈控制或其他测试。

## 后续可调参数

如果实测侧摆中位不是 `40 deg`，优先调整：

```python
ROPE_INIT_DUAL_TARGET_DEG = 40.0
```

如果中位附近容易来回抖动，可以适当放大：

```python
ROPE_INIT_DUAL_TOL_DEG = 0.3
```

如果动作过快或过慢，可以调整：

```python
ROPE_INIT_STEP_COUNTS = 50
ROPE_INIT_TIMER_MS = 50
```

修改步长时要注意：步长越大，越容易越过目标；步长越小，动作更细，但耗时更长。
