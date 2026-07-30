/**
 * @file DualTendonControl.cpp
 * @brief 使用自动标定 LUT 计算双腱关节两台舵机的位置环目标。
 *
 * 两路 motorRef 与 motorFbModel 均来自同一份标定 LUT；本模块只计算，
 * 不读取系统 owner，也不直接发布舵机命令。
 */
#include "DualTendonControl.h"

#include <math.h>
#include <string.h>
#include "ControlOutputBuilder.h"
#include "ControlSafety.h"
#include "TendonCalibrationLut.h"
#include "../calibration/CalibrationTask.h"
#include "../config/ControlConfig.h"
#include "../hardware/HardwareMap.h"

namespace {

using namespace mmhand::config::control;

// 单个双腱关节的静态调参与配置。当前值来自固件常量，
// 结构上保留了后续从 EEPROM/配置加载的可能。
struct DualTendonJointConfig {
    uint8_t enabled;
    float kp[2];
    float ki[2];
    float kd[2];
    float integralLimit[2];
    float dTermLimit[2];
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

static const float kDualTendonVelocityFilterAlpha =
    kPeriodSec / (kDualTendonVelocityFilterTauSec + kPeriodSec);
static bool getDualTendonConfig(uint8_t jointIndex, DualTendonJointConfig* out)
{
    if (!out) {
        return false;
    }

    memset(out, 0, sizeof(*out));
    out->enabled = 1;

    // All supported dual-tendon joints currently share the same controller gains.
    out->kp[0] = kDualTendonKp;
    out->kp[1] = kDualTendonKp;
    out->ki[0] = kDualTendonKi;
    out->ki[1] = kDualTendonKi;
    out->kd[0] = kDualTendonKd;
    out->kd[1] = kDualTendonKd;
    out->integralLimit[0] = kDualTendonIntegralLimit;
    out->integralLimit[1] = kDualTendonIntegralLimit;
    out->dTermLimit[0] = kDualTendonDTermLimitCounts;
    out->dTermLimit[1] = kDualTendonDTermLimitCounts;
    switch (jointIndex) {
        case 0:
        case 4:
        case 8:
        case 12:
            return true;
        case kThumbBaseJointIndex:
            return true;
        case 20:
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
        calib.tendonLutValid == 0 ||
        calib.tendonLutCount < 3) {
        return false;
    }

    for (uint8_t i = 0; i < 2; i++) {
        // 标定样本与电机通道绑定；执行器顺序混用会让拮抗对相互对抗，
        // 因此要求通道完全匹配。
        if (calib.tendonLutMotor[i] != actuators[i].motorChannel) {
            return false;
        }
        // 公共 LUT 函数内部只做一次 URDF -> 标定角转换，避免重复缩放。
        if (!interpolateTendonCalibrationMotorAbs(calib, i, qRefDeg, &outRef[i]) ||
            !interpolateTendonCalibrationMotorAbs(calib, i, qFbDeg, &outFb[i])) {
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
        outTargets->rawTargetAbs[i] = 0;
        outTargets->lutTargetClamped[i] = 0;
        outTargets->lutMinAbs[i] = 0;
        outTargets->lutMaxAbs[i] = 0;
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
            : (outTargets->motorRef[i] - state.prevMotorRef[i]) / kPeriodSec;
        const float fbVelocity = resetState
            ? 0.0f
            : (outTargets->motorFbModel[i] - state.prevMotorFbModel[i]) /
                  kPeriodSec;
        const float velocityError = refVelocity - fbVelocity;

        state.filteredVelocityError[i] +=
            kDualTendonVelocityFilterAlpha *
            (velocityError - state.filteredVelocityError[i]);

        state.integral[i] += outTargets->motorErr[i] * kPeriodSec;
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
        const int32_t rawTarget =
            clampServoPos((int32_t)(motorCurrent + correction));
        outTargets->rawTargetAbs[i] = rawTarget;
        int32_t lutMinAbs = 0;
        int32_t lutMaxAbs = 0;
        if (!tendonCalibrationMotorAbsRange(g_jointCalibResult[jointIndex],
                                            i,
                                            &lutMinAbs,
                                            &lutMaxAbs)) {
            return false;
        }
        int32_t boundedTarget = rawTarget;
        if (boundedTarget < lutMinAbs) boundedTarget = lutMinAbs;
        if (boundedTarget > lutMaxAbs) boundedTarget = lutMaxAbs;
        outTargets->targetAbs[i] = clampServoPos(boundedTarget);
        outTargets->lutTargetClamped[i] = boundedTarget != rawTarget ? 1 : 0;
        outTargets->lutMinAbs[i] = lutMinAbs;
        outTargets->lutMaxAbs[i] = lutMaxAbs;
        state.prevMotorRef[i] = outTargets->motorRef[i];
        state.prevMotorFbModel[i] = outTargets->motorFbModel[i];
    }

    return true;
}
