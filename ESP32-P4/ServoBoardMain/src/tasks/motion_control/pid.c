/**
 * @file pid.c
 * @brief 实现标准离散 PID 的初始化、复位和单步计算。
 */
#include "pid.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

static float limitFloat(float value, float limit)
{
    const float absoluteLimit = fabsf(limit);
    if (value > absoluteLimit) {
        return absoluteLimit;
    }
    if (value < -absoluteLimit) {
        return -absoluteLimit;
    }
    return value;
}

static int parametersAreFinite(const float params[PID_PARAMETER_NUM])
{
    for (uint8_t i = 0U; i < PID_PARAMETER_NUM; i++) {
        if (!isfinite(params[i])) {
            return 0;
        }
    }
    return 1;
}

PID_Status_e PID_Init(PID_Info_TypeDef *pid,
                      const float params[PID_PARAMETER_NUM])
{
    if (pid == NULL || params == NULL || !parametersAreFinite(params)) {
        return PID_FAILED_INIT;
    }

    memset(pid, 0, sizeof(*pid));
    pid->param.kp = params[0];
    pid->param.ki = params[1];
    pid->param.kd = params[2];
    pid->param.Deadband = params[3];
    pid->param.limitIntegral = params[4];
    pid->status = PID_ERROR_NONE;
    return PID_ERROR_NONE;
}

void PID_Calc_Clear(PID_Info_TypeDef *pid)
{
    if (pid == NULL) {
        return;
    }

    pid->error = 0.0f;
    pid->previousError = 0.0f;
    pid->Integral = 0.0f;
    pid->Pout = 0.0f;
    pid->Iout = 0.0f;
    pid->Dout = 0.0f;
    pid->Output = 0.0f;
    pid->derivativeInitialized = 0U;
    pid->status = PID_ERROR_NONE;
}

float f_PID_Calculate(PID_Info_TypeDef *pid,
                      float target,
                      float measure,
                      float dtSeconds)
{
    if (pid == NULL) {
        return 0.0f;
    }

    if (pid->status != PID_ERROR_NONE) {
        PID_Calc_Clear(pid);
        return 0.0f;
    }

    if (!isfinite(target) || !isfinite(measure)) {
        PID_Calc_Clear(pid);
        pid->status = PID_CALC_NANINF;
        return 0.0f;
    }

    // 无效采样周期不推进状态，调用方可在下一周期继续计算。
    if (!isfinite(dtSeconds) || dtSeconds <= 0.0f) {
        return pid->Output;
    }

    pid->error = target - measure;

    if (fabsf(pid->error) >= fabsf(pid->param.Deadband)) {
        if (pid->param.ki != 0.0f) {
            pid->Integral += pid->error * dtSeconds;
        } else {
            pid->Integral = 0.0f;
        }
        pid->Integral = limitFloat(pid->Integral, pid->param.limitIntegral);

        pid->Pout = pid->param.kp * pid->error;
        pid->Iout = pid->param.ki * pid->Integral;
        pid->Dout = pid->derivativeInitialized
            ? pid->param.kd * (pid->error - pid->previousError) / dtSeconds
            : 0.0f;
        // 最终位置修正不再使用固定 +/-600 counts 限幅；绝对目标仍由
        // LUT 工作区、会话逻辑范围和硬件范围在发送链路统一约束。
        pid->Output = pid->Pout + pid->Iout + pid->Dout;
    }

    pid->previousError = pid->error;
    pid->derivativeInitialized = 1U;

    if (!isfinite(pid->Output)) {
        PID_Calc_Clear(pid);
        pid->status = PID_CALC_NANINF;
        return 0.0f;
    }
    return pid->Output;
}
