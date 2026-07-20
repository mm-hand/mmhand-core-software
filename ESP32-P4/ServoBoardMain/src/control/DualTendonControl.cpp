#include "DualTendonControl.h"

#include <math.h>
#include <string.h>
#include "ControlOutputBuilder.h"
#include "ControlSafety.h"
#include "../calibration/CalibrationTask.h"
#include "../hardware/HardwareMap.h"

namespace {

// 单个双腱关节的静态调参与配置。当前值来自固件常量，
// 结构上保留了后续从 EEPROM/配置加载的可能。
struct DualTendonJointConfig {
    uint8_t enabled;
    float kp[2];
    float ki[2];
    float kd[2];
    float integralLimit[2];
    float dTermLimit[2];
    int32_t maxReleaseFromCalibCounts;
};

// 动态 PID/滤波状态，按逻辑关节索引。关节停止参与闭环输出时会重置。
struct DualTendonJointState {
    float integral[2];
    float prevMotorRef[2];
    float prevMotorFbModel[2];
    float filteredVelocityError[2];
    uint8_t prevValid;
};

static DualTendonJointState g_dualTendonState[ENCODER_TOTAL_NUM];

static const float kControlPeriodSec = 0.010f;
static const float kDefaultDualTendonKp = 0.00f;
static const float kDefaultDualTendonKi = 0.0f;
static const float kDefaultDualTendonKd = 0.6f;
static const float kDefaultDualTendonIntegralLimit = 0.0f;
static const float kDefaultDualTendonDTermLimitCounts = 100.0f;
static const float kDualTendonVelocityFilterTauSec = 0.050f;
static const float kDualTendonVelocityFilterAlpha =
    kControlPeriodSec / (kDualTendonVelocityFilterTauSec + kControlPeriodSec);
static const int32_t kDefaultDualTendonMaxReleaseFromCalibCounts = 1000;
static const int32_t kThumbDualTendonMaxReleaseFromCalibCounts = 1800;
static const int32_t kJoint20DualTendonMaxReleaseFromCalibCounts = 2500;

static bool getDualTendonConfig(uint8_t jointIndex, DualTendonJointConfig* out)
{
    if (!out) {
        return false;
    }

    memset(out, 0, sizeof(*out));
    out->enabled = 1;

    // 先使用保守默认值；特殊关节只覆盖自身的安全释放余量。
    out->kp[0] = kDefaultDualTendonKp;
    out->kp[1] = kDefaultDualTendonKp;
    out->ki[0] = kDefaultDualTendonKi;
    out->ki[1] = kDefaultDualTendonKi;
    out->kd[0] = kDefaultDualTendonKd;
    out->kd[1] = kDefaultDualTendonKd;
    out->integralLimit[0] = kDefaultDualTendonIntegralLimit;
    out->integralLimit[1] = kDefaultDualTendonIntegralLimit;
    out->dTermLimit[0] = kDefaultDualTendonDTermLimitCounts;
    out->dTermLimit[1] = kDefaultDualTendonDTermLimitCounts;
    out->maxReleaseFromCalibCounts = kDefaultDualTendonMaxReleaseFromCalibCounts;

    switch (jointIndex) {
        case 0:
        case 4:
        case 8:
        case 12:
            return true;
        case kThumbBaseJointIndex:
            out->maxReleaseFromCalibCounts =
                kThumbDualTendonMaxReleaseFromCalibCounts;
            return true;
        case 20:
            out->maxReleaseFromCalibCounts =
                kJoint20DualTendonMaxReleaseFromCalibCounts;
            return true;
        default:
            out->enabled = 0;
            return false;
    }
}

static float constrainFloat(float value, float minValue, float maxValue)
{
    // 本地 float 限幅函数，避免在这里引入 Arduino constrain 重载。
    if (value < minValue) return minValue;
    if (value > maxValue) return maxValue;
    return value;
}

static bool getDualTendonActuators(uint8_t jointIndex, JointActuatorItem outActuators[2])
{
    // 双腱关节由两个逻辑拉动方向相反的执行器表示。
    // 同号双执行器由通用输出构建器处理。
    if (!outActuators || getJointActuatorCount(jointIndex) < 2) {
        return false;
    }
    if (!getJointActuator(jointIndex, 0, &outActuators[0]) ||
        !getJointActuator(jointIndex, 1, &outActuators[1])) {
        return false;
    }
    return outActuators[0].sign != outActuators[1].sign;
}

static bool interpolateDualTendonLutMotorAbs(const JointCalibrationResult& calib,
                                             uint8_t actuatorIndex,
                                             float qDeg,
                                             float* outAbs)
{
    // 提前拒绝不完整 LUT；下面的插值假设角度步长均匀。
    if (!outAbs ||
        actuatorIndex >= DUAL_TENDON_LUT_ACTUATOR_COUNT ||
        calib.dualTendonLutValid == 0 ||
        calib.dualTendonLutCount < 3 ||
        calib.dualTendonLutCount > DUAL_TENDON_LUT_MAX_SAMPLES ||
        !isfinite(calib.dualTendonLutAngleStartDeg) ||
        !isfinite(calib.dualTendonLutAngleStepDeg) ||
        calib.dualTendonLutAngleStepDeg <= 0.0f) {
        return false;
    }

    const float firstDeg = calib.dualTendonLutAngleStartDeg;
    const float stepDeg = calib.dualTendonLutAngleStepDeg;
    // 超出范围的请求钳到最近的标定端点，不在实测腱绳关系之外外推。
    if (qDeg <= firstDeg) {
        *outAbs = (float)calib.dualTendonLutMotorAbs[actuatorIndex][0];
        return true;
    }

    const uint8_t lastIndex = calib.dualTendonLutCount - 1;
    const float lastDeg = firstDeg + stepDeg * (float)lastIndex;
    if (qDeg >= lastDeg) {
        *outAbs = (float)calib.dualTendonLutMotorAbs[actuatorIndex][lastIndex];
        return true;
    }

    const float position = (qDeg - firstDeg) / stepDeg;
    uint8_t lowerIndex = (uint8_t)floorf(position);
    if (lowerIndex >= lastIndex) {
        lowerIndex = lastIndex - 1;
    }
    const float t = position - (float)lowerIndex;
    const float lower =
        (float)calib.dualTendonLutMotorAbs[actuatorIndex][lowerIndex];
    const float upper =
        (float)calib.dualTendonLutMotorAbs[actuatorIndex][lowerIndex + 1];
    *outAbs = lower + (upper - lower) * t;
    return true;
}

static bool computeDualTendonModelAbsFromCalibration(uint8_t jointIndex,
                                                     const JointActuatorItem actuators[2],
                                                     float qRefDeg,
                                                     float qFbDeg,
                                                     float outRef[2],
                                                     float outFb[2])
{
    if (!actuators || !outRef || !outFb || jointIndex >= ENCODER_TOTAL_NUM) {
        return false;
    }

    const JointCalibrationResult& calib = g_jointCalibResult[jointIndex];
    if (!calib.success || !calib.scaled ||
        calib.dualTendonLutValid == 0 ||
        calib.dualTendonLutCount < 3) {
        return false;
    }

    float lutRefDeg = qRefDeg;
    float lutFbDeg = qFbDeg;
    if (calib.mappingVersion >= 2) {
        // 新版标定把 URDF 关节限位和 raw LUT 扫描范围分开保存，
        // 插值前需要先把 URDF 角度转换回 LUT 角度。
        const float urdfSpan = calib.angleMax - calib.angleMin;
        if (!isfinite(urdfSpan) || urdfSpan <= 0.0f ||
            !isfinite(calib.totalDeg) || calib.totalDeg <= 0.0f) {
            return false;
        }
        lutRefDeg = calib.startDeg +
            ((qRefDeg - calib.angleMin) / urdfSpan) * calib.totalDeg;
        lutFbDeg = calib.startDeg +
            ((qFbDeg - calib.angleMin) / urdfSpan) * calib.totalDeg;
    }

    for (uint8_t i = 0; i < 2; i++) {
        // 标定样本与电机通道绑定；执行器顺序混用会让拮抗对相互对抗，
        // 因此要求通道完全匹配。
        if (calib.dualTendonLutMotor[i] != actuators[i].motorChannel) {
            return false;
        }
        if (!interpolateDualTendonLutMotorAbs(calib, i, lutRefDeg, &outRef[i]) ||
            !interpolateDualTendonLutMotorAbs(calib, i, lutFbDeg, &outFb[i])) {
            return false;
        }
    }
    return true;
}

} // 匿名命名空间结束

bool isDualTendonJoint(uint8_t jointIndex)
{
    // 仅有配置还不够，硬件映射也必须提供两个异号执行器。
    DualTendonJointConfig cfg;
    if (!getDualTendonConfig(jointIndex, &cfg) || !cfg.enabled) {
        return false;
    }
    JointActuatorItem actuators[2];
    return getDualTendonActuators(jointIndex, actuators);
}

void resetDualTendonController(uint8_t jointIndex)
{
    if (jointIndex >= ENCODER_TOTAL_NUM) {
        return;
    }
    g_dualTendonState[jointIndex] = DualTendonJointState();
}

int32_t clampDualTendonReleaseFromCalib(uint8_t jointIndex,
                                       uint8_t motorChannel,
                                       int32_t calibratedAnchorRawAbs,
                                       int32_t currentRawAbs,
                                       int32_t targetRawAbs)
{
    // 在腱绳坐标中比较，使“释放”的符号不受各电机 raw ABS 方向影响。
    DualTendonJointConfig cfg;
    if (!getDualTendonConfig(jointIndex, &cfg) || !cfg.enabled) {
        return clampServoPos(targetRawAbs);
    }

    const int32_t calibratedAnchorTendonAbs =
        motorRawAbsToTendonAbs(motorChannel, calibratedAnchorRawAbs);
    const int32_t currentTendonAbs =
        motorRawAbsToTendonAbs(motorChannel, currentRawAbs);
    const int32_t targetTendonAbs =
        motorRawAbsToTendonAbs(motorChannel, targetRawAbs);
    const int32_t actualReleaseFromCalib =
        calibratedAnchorTendonAbs - currentTendonAbs;

    // 无状态反馈门限：反馈达到释放余量后，仅保持继续释放的命令。
    // 收紧命令始终允许通过。
    if (actualReleaseFromCalib >= cfg.maxReleaseFromCalibCounts &&
        targetTendonAbs < currentTendonAbs) {
        return clampServoPos(currentRawAbs);
    }
    return clampServoPos(targetRawAbs);
}

bool computeDualTendonMotorTargets(uint8_t jointIndex,
                                   float qRefDeg,
                                   float qFbDeg,
                                   const ServoAngleData_t* servoData,
                                   DualTendonMotorTargets* outTargets)
{
    // 双腱主求解流程：先把关节参考/反馈映射成两个已标定电机位置，
    // 再为每个电机叠加小幅 PID 修正。
    if (!servoData || !outTargets || jointIndex >= ENCODER_TOTAL_NUM) {
        return false;
    }

    DualTendonJointConfig cfg;
    if (!getDualTendonConfig(jointIndex, &cfg) || !cfg.enabled) {
        return false;
    }

    JointActuatorItem actuators[2];
    if (!getDualTendonActuators(jointIndex, actuators)) {
        return false;
    }

    float motorRefModel[2] = {0.0f, 0.0f};
    float motorFbModel[2] = {0.0f, 0.0f};
    if (!computeDualTendonModelAbsFromCalibration(jointIndex,
                                                  actuators,
                                                  qRefDeg,
                                                  qFbDeg,
                                                  motorRefModel,
                                                  motorFbModel)) {
        return false;
    }

    DualTendonJointState& state = g_dualTendonState[jointIndex];
    outTargets->count = 2;
    for (uint8_t i = 0; i < 2; i++) {
        // 两个电机都必须有效且在线；任一侧丢失都会使拮抗解失效，
        // 调用方随后会保持当前位置。
        const uint8_t motorChannel = actuators[i].motorChannel;
        if (motorChannel >= SERVO_TOTAL_NUM ||
            !isMotorChannelValid(motorChannel) ||
            servoData->onlineStatus[motorChannel] == 0) {
            return false;
        }

        outTargets->motorChannel[i] = motorChannel;
        outTargets->motorRef[i] = motorRefModel[i];
        outTargets->motorFbModel[i] = motorFbModel[i];
        outTargets->motorErr[i] = outTargets->motorRef[i] - outTargets->motorFbModel[i];
        outTargets->refVelocity[i] = 0.0f;
        outTargets->fbVelocity[i] = 0.0f;
        outTargets->velocityError[i] = 0.0f;
        outTargets->filteredVelocityError[i] = 0.0f;
        outTargets->pTerm[i] = 0.0f;
        outTargets->iTerm[i] = 0.0f;
        outTargets->dTerm[i] = 0.0f;
        outTargets->pidCorrection[i] = 0.0f;
    }

    const bool resetState = !state.prevValid;
    if (resetState) {
        // 用当前模型值初始化历史项，避免闭环刚启用的第一周期产生微分尖峰。
        state.integral[0] = 0.0f;
        state.integral[1] = 0.0f;
        state.prevMotorRef[0] = outTargets->motorRef[0];
        state.prevMotorRef[1] = outTargets->motorRef[1];
        state.prevMotorFbModel[0] = outTargets->motorFbModel[0];
        state.prevMotorFbModel[1] = outTargets->motorFbModel[1];
        state.filteredVelocityError[0] = 0.0f;
        state.filteredVelocityError[1] = 0.0f;
        state.prevValid = 1;
    }
    for (uint8_t i = 0; i < 2; i++) {
        // 微分项基于模型速度误差，而不是 raw 编码器速度，
        // 这样阻尼项始终处在已标定的电机空间中。
        const uint8_t motorChannel = outTargets->motorChannel[i];
        const float motorCurrent = (float)servoData->servoAngles[motorChannel];
        const float refVelocity = resetState
            ? 0.0f
            : (outTargets->motorRef[i] - state.prevMotorRef[i]) / kControlPeriodSec;
        const float fbVelocity = resetState
            ? 0.0f
            : (outTargets->motorFbModel[i] - state.prevMotorFbModel[i]) /
                  kControlPeriodSec;
        const float velocityError = refVelocity - fbVelocity;

        state.filteredVelocityError[i] +=
            kDualTendonVelocityFilterAlpha *
            (velocityError - state.filteredVelocityError[i]);

        state.integral[i] += outTargets->motorErr[i] * kControlPeriodSec;
        state.integral[i] = constrainFloat(state.integral[i],
                                           -cfg.integralLimit[i],
                                           cfg.integralLimit[i]);

        const float pTerm = cfg.kp[i] * outTargets->motorErr[i];
        const float iTerm = cfg.ki[i] * state.integral[i];
        const float dTerm = constrainFloat(
            cfg.kd[i] * state.filteredVelocityError[i],
            -cfg.dTermLimit[i],
            cfg.dTermLimit[i]);
        const float correction = pTerm + iTerm + dTerm;

        outTargets->refVelocity[i] = refVelocity;
        outTargets->fbVelocity[i] = fbVelocity;
        outTargets->velocityError[i] = velocityError;
        outTargets->filteredVelocityError[i] = state.filteredVelocityError[i];
        outTargets->pTerm[i] = pTerm;
        outTargets->iTerm[i] = iTerm;
        outTargets->dTerm[i] = dTerm;
        outTargets->pidCorrection[i] = correction;
        outTargets->targetAbs[i] =
            clampServoPos((int32_t)(motorCurrent + correction));
        state.prevMotorRef[i] = outTargets->motorRef[i];
        state.prevMotorFbModel[i] = outTargets->motorFbModel[i];
    }

    return true;
}
