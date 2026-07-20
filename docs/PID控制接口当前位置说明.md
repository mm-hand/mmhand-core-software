# PID 控制接口当前位置说明

本文档用于说明当前固件中 PID 相关代码的位置、已经具备的接口、以及目前尚未实现的部分。重点区分三类情况：

- 单电机控制单关节
- 单关节双电机/拮抗电机
- 单电机控制双关节

## 总体入口

PID 闭环只在 ESP32-P4 固件的 `RUN_FEEDBACK` / joint feedback 控制路径中生效。

主循环入口：

```text
ESP32-P4/ServoBoardMain/src/control/ControlTask.cpp
```

关键调用位置：

```cpp
g_controlSolver.compute(localTargets, magAngles, absolutePosition, outPulses);
```

位置：

```text
ESP32-P4/ServoBoardMain/src/control/ControlTask.cpp:644
```

含义：

- `localTargets[joint]`：目标关节角度，单位 deg。
- `magAngles[joint]`：磁编码器反馈关节角度，单位 deg。
- `absolutePosition[joint]`：该 joint 对应主舵机当前绝对 ABS。
- `outPulses[joint]`：PID 算出来的主舵机 ABS 目标。

## 通用 PID 数学模块

文件：

```text
ESP32-P4/ServoBoardMain/src/control/pid.c
ESP32-P4/ServoBoardMain/src/control/pid.h
```

主要函数：

```cpp
void PID_Init(PID_Info_TypeDef *Pid, PID_Type_e type, float para[PID_PARAMETER_NUM]);
float f_PID_Calculate(PID_Info_TypeDef *Pid, float target, float measure);
```

位置：

```text
ESP32-P4/ServoBoardMain/src/control/pid.c:81
ESP32-P4/ServoBoardMain/src/control/pid.c:119
```

说明：

这两个函数只是通用 PID 算法，不知道手指、关节、舵机映射关系。实际每个 joint 的 PID 状态保存在 `ControlSolver` 里。

## 单电机控制单关节

### 当前状态

已经实现。

### 代码位置

文件：

```text
ESP32-P4/ServoBoardMain/src/control/ControlSolver.cpp
```

PID 初始化：

```cpp
PID_Init(&_pids[i], PID_POSITION, pidParams);
```

位置：

```text
ESP32-P4/ServoBoardMain/src/control/ControlSolver.cpp:50
```

PID 计算：

```cpp
f_PID_Calculate(&_pids[i], targetDegs[i], magActualDegs[i], dtSeconds);
outServoPulses[i] = absolutePosition[i] + (int32_t)_pids[i].Output;
```

位置：

```text
ESP32-P4/ServoBoardMain/src/control/ControlSolver.cpp:66
```

### 当前控制结构

单关节目前是单级外环 PID：

1. 关节角度 PID
   `targetDegs[i] - magActualDegs[i]`

2. 舵机位置命令
   外环输出直接叠加当前舵机 ABS，得到舵机 ABS 目标；
   舵机自身负责到达该位置。

最终输出是：

```cpp
outServoPulses[i]
```

这个值会被 `ControlTask` 下发给 joint 对应的主舵机。

### 下发位置

文件：

```text
ESP32-P4/ServoBoardMain/src/control/ControlTask.cpp
```

关键代码：

```cpp
targetPos = clampServoPosByMotorAbsLimit(
    sharedData,
    (uint8_t)jointMotorCh,
    outPulses[jointIndex]
);

appendServoTarget(&targetBatch, bus, id, targetPos, ...);
```

位置：

```text
ESP32-P4/ServoBoardMain/src/control/ControlTask.cpp:821
ESP32-P4/ServoBoardMain/src/control/ControlTask.cpp:834
```

## 单关节双电机/拮抗电机

### 当前状态

已有映射接口，也已有副电机目标生成逻辑。

但注意：当前不是“双电机分别独立 PID”。当前逻辑是：

```text
joint PID -> 主舵机目标 -> 根据拮抗关系推导副舵机目标
```

也就是说，PID 只对 joint 算一次，输出主舵机 ABS；副舵机通过符号、offset、tensionBias 跟随主舵机。

### 映射表位置

文件：

```text
ESP32-P4/ServoBoardMain/src/hardware/HardwareMap.cpp
```

主映射：

```cpp
JointActuatorGroup jointActuatorGroups[ENCODER_TOTAL_NUM]
```

位置：

```text
ESP32-P4/ServoBoardMain/src/hardware/HardwareMap.cpp:45
```

这里定义每个 joint 对应几个 actuator：

- `GROUP1(ch)`：一个 joint 对一个 motor。
- `GROUP2(primaryCh, secondaryAct)`：一个 joint 对两个 motor。
- `ACT_SECONDARY_OPPOSE(ch)`：副电机与主电机拮抗，sign = -1。

例如 V2 中 `J00` 是：

```cpp
GROUP2(0, ACT_SECONDARY_OPPOSE(22))
```

含义是：

```text
J00 -> 主电机 M00 + 副电机 M22
```

### 当前 ControlTask 实际使用的副电机接口

文件：

```text
ESP32-P4/ServoBoardMain/src/control/ControlTask.cpp
```

函数：

```cpp
static bool appendSecondaryTargetForJoint(...)
```

位置：

```text
ESP32-P4/ServoBoardMain/src/control/ControlTask.cpp:145
```

调用位置：

```text
ESP32-P4/ServoBoardMain/src/control/ControlTask.cpp:797
ESP32-P4/ServoBoardMain/src/control/ControlTask.cpp:813
ESP32-P4/ServoBoardMain/src/control/ControlTask.cpp:838
```

其中正常 PID 输出路径是：

```cpp
appendSecondaryTargetForJoint(sharedData, &targetBatch, jointIndex, targetPos, ...);
```

这里的 `targetPos` 是主舵机目标。

### 副电机目标如何计算

文件：

```text
ESP32-P4/ServoBoardMain/src/calibration/CalibrationTask.cpp
```

函数：

```cpp
bool computeSecondaryServoTargetForJoint(uint8_t jointIndex,
                                         int32_t primaryTarget,
                                         int16_t* outSecondaryTarget)
```

位置：

```text
ESP32-P4/ServoBoardMain/src/calibration/CalibrationTask.cpp:173
```

当前公式：

```cpp
secondaryTarget = sign * primaryTarget + secondaryOffset + tensionBias;
```

对拮抗电机来说，`sign = -1`。

### 已有但当前 ControlTask 没完全使用的通用接口

文件：

```text
ESP32-P4/ServoBoardMain/src/control/ControlOutputBuilder.cpp
ESP32-P4/ServoBoardMain/src/control/ControlOutputBuilder.h
```

接口：

```cpp
uint8_t buildJointActuatorRuntimeTargets(
    uint8_t jointIndex,
    int32_t primaryTargetAbs,
    JointActuatorRuntimeTarget* outTargets,
    uint8_t outCapacity
);

uint8_t appendJointActuatorTargets(
    ServoTargetBatch_t* batch,
    uint8_t jointIndex,
    int32_t primaryTargetAbs,
    uint16_t speed,
    uint8_t acc
);
```

位置：

```text
ESP32-P4/ServoBoardMain/src/control/ControlOutputBuilder.cpp:87
ESP32-P4/ServoBoardMain/src/control/ControlOutputBuilder.cpp:144
```

说明：

这套接口更通用，理论上可以统一处理一个 joint 对一个或多个 motor 的情况。但当前 `ControlTask.cpp` 正常 PID 下发路径仍然主要使用 `appendSecondaryTargetForJoint()`，没有完全切换到 `appendJointActuatorTargets()`。

后续如果要整理结构，建议把 `ControlTask.cpp` 中的主/副舵机下发逻辑统一改成 `appendJointActuatorTargets()`，这样 PID 同学只需要关心“joint PID 输出 primaryTargetAbs”，映射层统一展开到一个或多个电机。

## 单电机控制双关节

### 当前状态

目前没有真正实现。

### 为什么说没有实现

当前控制结构是：

```text
每个 joint 一套 PID
每个 joint 输出一个 outPulses[joint]
每个 joint 再映射到它自己的 motor 或 motor group
```

也就是：

```text
joint -> motor(s)
```

但“单电机控制双关节”需要的是：

```text
motor -> jointA + jointB
```

这要求把两个关节的误差合成为一个电机目标，例如：

```text
errorA = targetA - actualA
errorB = targetB - actualB
motor_cmd = f(errorA, errorB, current_abs, zero_abs, tight_abs)
```

当前代码里没有以下结构：

- 没有 `MotorJointCouplingGroup`
- 没有 `motor -> joints` 的映射表
- 没有双关节误差融合函数
- 没有针对一个 motor 同时服务两个 joint 的 PID 状态
- 旧版 `pid_abs_calib.json` 上位机保存链路已经移除，当前不再生成或读取这类 ABS 锚点文件。

所以单电机双关节目前不能认为已经接入 PID。

## PID ABS 标定数据状态

上位机已经增加 PID 控制锚点文件：

```text
该文件已废弃并移除。当前负反馈控制使用实时反馈、自动标定限位和当前控制参数，不依赖独立的 `pid_abs_calib.json`。
```

说明文档：

```text
docs/PID_ABS标定数据使用说明.md
```

这个文件保存：

- `zero_abs_at_encoder_0deg`
- `tight_abs_right_limit`
- `span_abs`

这些数据目前是给 PID 同学做控制映射参考用。当前 ESP32-P4 固件还没有自动读取这个 JSON。如果 PID 要跑在固件侧，需要后续增加下发协议或固件内置表。

## 建议后续怎么接

### 第一阶段：先整理单关节多电机下发接口

把 `ControlTask.cpp` 中当前的：

```cpp
appendServoTarget(... primary ...);
appendSecondaryTargetForJoint(...);
```

统一替换为：

```cpp
appendJointActuatorTargets(&targetBatch, jointIndex, targetPos, speed, acc);
```

好处：

- 单电机单关节和单关节双电机共用一套下发逻辑。
- PID 输出仍然只需要给出 joint 的主目标。
- 后续扩展更多拮抗电机时不需要继续在 `ControlTask.cpp` 写特殊分支。

### 第二阶段：新增单电机双关节接口

建议新增独立模块，例如：

```text
ESP32-P4/ServoBoardMain/src/control/CoupledJointPidSolver.h
ESP32-P4/ServoBoardMain/src/control/CoupledJointPidSolver.cpp
```

建议接口：

```cpp
struct CoupledMotorJointConfig {
    uint8_t motorChannel;
    uint8_t jointA;
    uint8_t jointB;
    int32_t zeroAbsAtEncoder0Deg;
    int32_t tightAbsRightLimit;
    float jointAWeight;
    float jointBWeight;
};

bool computeCoupledMotorTarget(
    const CoupledMotorJointConfig& cfg,
    float targetA,
    float actualA,
    float targetB,
    float actualB,
    int32_t currentAbs,
    int16_t* outTargetAbs
);
```

这个函数内部再决定：

- 两个 joint error 怎么加权。
- 是否用一个 PID 还是两个 PID 后融合。
- 如何用 `zeroAbsAtEncoder0Deg` 和 `tightAbsRightLimit` 做限幅。

### 第三阶段：明确 PID 参数来源

当前 PID 参数在 `ControlSolver::begin()` 里写死：

```text
ESP32-P4/ServoBoardMain/src/control/ControlSolver.cpp:31
```

后续建议改成：

- 固件默认参数表，或者
- 上位机下发参数，或者
- 编译期配置头文件。

这样 PID 同学调参不会每次都改控制主逻辑。
