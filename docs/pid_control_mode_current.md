# 当前单电机与双电机 PID 类型说明

本文基于当前工程代码说明负反馈控制中的 PID 属于“位置式”还是“增量式”，并结合最近的参数调节解释各参数应该如何理解。

## 结论

当前单电机和双电机负反馈都不属于标准“增量式 PID”。

单电机使用的是明确的“位置式 PID”。代码中两级 PID 都通过 `PID_Init(..., PID_POSITION, ...)` 初始化，计算公式是：

```text
Output = Kp * e(k) + Ki * sum(e) + Kd * (e(k) - e(k-1))
```

双电机没有调用 `pid.c` 里的 PID 类，而是在 `DualTendonControl.cpp` 中手写了类似位置式的 P/I/D correction：

```text
correction = Kp * motorErr + Ki * integral - Kd * dFb
targetAbs = motorCurrent + correction
```

所以双电机也不是标准增量式 PID。它更准确地说是：基于当前模型误差计算位置修正量，然后把修正量叠加到当前电机 ABS 上。

需要特别区分两个概念：

```text
位置式 PID：PID 输出本身由当前误差、积分、微分直接算出。
增量式 PID：PID 输出每周期累加 delta_u，通常形式是 Output += ...
```

当前代码虽然最后会执行 `当前电机位置 + correction`，看起来像“每次加一个增量”，但 PID 算法本身不是增量式 PID。

## 单电机 PID

对应代码：

```text
ESP32-P4/ServoBoardMain/src/control/ControlSolver.cpp
```

当前参数：

```text
外环：Kp=5.5, Ki=0.0, Kd=0.7, IntegralLimit=100, OutputLimit=600
内环：Kp=1.0, Ki=0.0, Kd=0.0, IntegralLimit=0,   OutputLimit=30719
```

单电机是两级位置式 PID。

### 外环

输入：

```text
targetDeg - magActualDeg
```

输出：

```text
角度误差对应的 tendon/电机侧位置修正量
```

也就是日志里的 `l1`。

### 内环

当前代码中内环目标是：

```text
loop2Target = outerOutput + absolutePosition
loop2Actual = absolutePosition
```

然后内环 PID 输出：

```text
outServoPulses = innerOutput + absolutePosition
```

由于当前内环参数是 `Kp=1, Ki=0, Kd=0`，所以内环基本等价于：

```text
outServoPulses = absolutePosition + outerOutput
```

因此当前单电机实际行为可以简化理解为：

```text
目标角度 - 当前角度 -> 外环位置式 PID -> tendon 修正量
最终电机目标 = 当前 tendon/ABS + tendon 修正量
```

这不是标准增量式 PID，因为外环 PID 输出没有用 `Output += delta` 累加，而是每周期由当前误差直接计算。

## 双电机 PID

对应代码：

```text
ESP32-P4/ServoBoardMain/src/control/DualTendonControl.cpp
```

当前参数：

```text
Kp = 1.0
Ki = 0.0
Kd = 0.03
IntegralLimit = 0.0
```

双电机控制流程：

1. 根据目标关节角 `qRefDeg` 计算两根 tendon 的理论长度变化。
2. 根据当前磁编角 `qFbDeg` 计算两根 tendon 的当前模型长度变化。
3. 把 tendon 长度变化换算成两个电机的模型 ABS。
4. 得到模型误差：

```text
motorErr = motorRef - motorFbModel
```

5. 计算 PID 修正：

```text
correction = Kp * motorErr + Ki * integral - Kd * dFb
```

6. 最终下发：

```text
targetAbs = motorCurrent + correction
```

其中 `dFb` 是模型反馈位置的变化速度：

```text
dFb = (motorFbModel - prevFbModel) / 0.010
```

所以双电机当前更像是“模型位置误差控制 + 当前电机位置叠加修正”。它不是标准增量式 PID，因为没有保存上一周期 PID 输出并执行：

```text
Output += deltaOutput
```

## 为什么看起来像增量控制

无论单电机还是双电机，最终目标都会写成：

```text
target = currentPosition + correction
```

这会让电机每次从当前位置附近往目标角度方向推一段，看起来像“增量”。

但这只是命令输出形式，不是 PID 算法类型。

真正的增量式 PID 通常会计算：

```text
deltaOutput = Kp * (e(k)-e(k-1))
            + Ki * e(k)
            + Kd * (e(k)-2e(k-1)+e(k-2))

Output = Output + deltaOutput
```

当前单电机 `pid.c` 虽然支持这种 `PID_VELOCITY` 路径，但 `ControlSolver.cpp` 使用的是 `PID_POSITION`，不是 `PID_VELOCITY`。

## 当前调参含义

### 单电机

当前单电机主要调的是外环：

```text
Kp=5.5, Ki=0, Kd=0.7
```

含义：

```text
Kp：角度误差每差 1deg，给多少 tendon/ABS 修正量。
Kd：抑制角度误差变化太快导致的超调。
Ki：当前为 0，不补长期静差。
```

如果单电机曲线仍有稳定误差，可以先小幅调 Kp；如果过冲明显，优先增加 Kd 或降低 Kp。Ki 暂时不建议急着加，因为绳驱摩擦和回差会让积分更容易积累后突然释放。

### 双电机

当前双电机参数：

```text
Kp=1.0, Ki=0, Kd=0.03
```

含义：

```text
Kp：模型 tendon ABS 误差每差 1 count，给多少电机 ABS 修正量。
Kd：根据模型反馈速度提供阻尼，抑制接近目标时继续冲。
Ki：当前为 0，不补静态摩擦造成的长期偏差。
```

从最近的 J00 曲线看，`Kp=1.0, Kd=0.03` 已经能推到目标附近，但最后略有正向超调。下一步优化可以优先考虑：

```text
Kp = 0.9
Ki = 0.0
Kd = 0.04
```

如果响应变慢但稳态仍偏差不大，再微调 Kp；如果接近目标后仍慢慢爬高，则继续略增 Kd 或略降 Kp。

## 调试日志对应关系

单电机日志：

```text
[FB DBG] J03 valid=1 tgt=... act=... l1=... l2act=... l2out=... cmd=... curAbs=...
```

字段含义：

```text
tgt：目标关节角
act：磁编实际角
l1：外环 PID 输出，也就是 tendon/ABS 修正量
l2act：内环当前 tendon/ABS
l2out：内环 PID 输出
cmd：最终下发电机目标
curAbs：当前电机原始 ABS
```

双电机当前没有像单电机一样完整打印 `motorRef/motorFbModel/motorErr/pidCorrection`，如果后续要继续精调双电机，建议增加双电机专用日志：

```text
J00 qRef/qFb
M00 motorRef/motorFbModel/motorErr/correction/current/target
M22 motorRef/motorFbModel/motorErr/correction/current/target
```

这样可以判断主副电机到底是同向过推、某一根 tendon 滞后，还是几何模型误差导致的偏差。

## 最终判断

```text
单电机：位置式 PID，双环结构，输出作为当前位置上的修正量。
双电机：手写位置误差 P/I/D correction，不是标准增量式 PID，输出也作为当前 ABS 上的修正量。
```

因此当前工程不应称为“增量式 PID”。更准确的描述是：

```text
基于位置式误差的闭环位置修正控制。
```
