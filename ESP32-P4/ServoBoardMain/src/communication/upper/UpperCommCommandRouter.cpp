#include "UpperCommCommandRouter.h"

#include <math.h>
#include <string.h>

// 直控绝对位置命令使用与舵机扩展位置一致的安全范围。
int16_t clampServoAbsCommand(int32_t value)
{
    if (value > 30719) return 30719;
    if (value < -30719) return -30719;
    return (int16_t)value;
}

void upperApplyTargetAngles(TaskSharedData_t* sharedData, const float* angles, uint8_t count)
{
    if (!sharedData || !angles) {
        return;
    }
    if (count > ENCODER_TOTAL_NUM) count = ENCODER_TOTAL_NUM;
    SemaphoreHandle_t lock = sharedData->commandStateMutex ? sharedData->commandStateMutex : sharedData->targetAnglesMutex;
    if (xSemaphoreTake(lock, pdMS_TO_TICKS(10)) == pdTRUE)
    {
        // 关节角目标必须整组写入，ControlTask 每周期再整组读取快照。
        for (uint8_t i = 0; i < count; i++) {
            sharedData->targetAngles[i] = angles[i];
        }
        sharedData->joint_command_token++;
        sharedData->control_mode = CONTROL_MODE_JOINT;
        xSemaphoreGive(lock);
    }
}

void upperClearTargetAngles(TaskSharedData_t* sharedData)
{
    float zeroAngles[ENCODER_TOTAL_NUM] = {0.0f};
    upperApplyTargetAngles(sharedData, zeroAngles, ENCODER_TOTAL_NUM);
}

void upperApplyMotorTargets(TaskSharedData_t* sharedData,
                            const int32_t* targets,
                            uint8_t count,
                            uint8_t source)
{
    if (!sharedData || !targets) {
        return;
    }
    if (count > SERVO_TOTAL_NUM) {
        count = SERVO_TOTAL_NUM;
    }
    SemaphoreHandle_t lock = sharedData->commandStateMutex;
    if (lock && xSemaphoreTake(lock, pdMS_TO_TICKS(10)) != pdTRUE) {
        return;
    }
    // motorTargetRaw 按 motorMap 的 22 通道顺序排列，不按 servo ID 直接索引。
    for (uint8_t i = 0; i < count; i++) {
        sharedData->motorTargetRaw[i] = targets[i];
    }
    sharedData->motor_command_token++;
    sharedData->motor_direct_command_generation++;
    sharedData->motor_direct_command_source = source;
    sharedData->control_mode = CONTROL_MODE_DIRECT_MOTOR;
    if (lock) {
        xSemaphoreGive(lock);
    }
}

void upperApplyMotorSweepTargets(TaskSharedData_t* sharedData, const int32_t* targets, uint8_t count)
{
    if (!sharedData || !targets) {
        return;
    }
    if (count > SERVO_TOTAL_NUM) {
        count = SERVO_TOTAL_NUM;
    }
    SemaphoreHandle_t lock = sharedData->commandStateMutex;
    if (lock && xSemaphoreTake(lock, pdMS_TO_TICKS(10)) != pdTRUE) {
        return;
    }
    // sweep 目标保持单圈 raw 语义，连续多圈展开由 ControlOutputBuilder 完成。
    for (uint8_t i = 0; i < count; i++) {
        sharedData->motorSweepTargetRaw[i] = targets[i];
    }
    sharedData->motor_sweep_command_token++;
    sharedData->motor_direct_command_generation++;
    sharedData->motor_direct_command_source = MOTOR_DIRECT_SOURCE_SWEEP;
    sharedData->control_mode = CONTROL_MODE_DIRECT_MOTOR;
    if (lock) {
        xSemaphoreGive(lock);
    }
}

void upperApplyTendonGuardConfig(TaskSharedData_t* sharedData,
                                 const uint8_t* enabled,
                                 const int8_t* sign,
                                 const int16_t* x1Abs,
                                 uint8_t count)
{
    if (!sharedData || !enabled || !sign || !x1Abs) {
        return;
    }
    if (count > ENCODER_TOTAL_NUM) {
        count = ENCODER_TOTAL_NUM;
    }
    SemaphoreHandle_t lock = sharedData->commandStateMutex;
    if (lock && xSemaphoreTake(lock, pdMS_TO_TICKS(10)) != pdTRUE) {
        return;
    }
    // sign 只保留方向语义，x1Abs 存储舵机绝对位置边界。
    for (uint8_t i = 0; i < count; i++) {
        sharedData->tendon_guard_enabled[i] = enabled[i] ? 1 : 0;
        sharedData->tendon_guard_sign[i] = (sign[i] < 0) ? -1 : 1;
        sharedData->tendon_guard_x1_abs[i] = clampServoAbsCommand(x1Abs[i]);
    }
    if (lock) {
        xSemaphoreGive(lock);
    }
}

void upperCacheCalibZeroRaw(TaskSharedData_t* sharedData, const float* values, uint8_t count)
{
    if (!sharedData || !values) {
        return;
    }
    if (count > ENCODER_TOTAL_NUM) {
        count = ENCODER_TOTAL_NUM;
    }
    SemaphoreHandle_t lock = sharedData->commandStateMutex;
    if (lock && xSemaphoreTake(lock, pdMS_TO_TICKS(10)) != pdTRUE) {
        return;
    }
    // 桌面端以 float 传输历史格式，这里转成 int32 raw 计数缓存。
    for (uint8_t i = 0; i < count; i++) {
        float v = values[i];
        if (!isfinite(v)) {
            v = 0.0f;
        }
        sharedData->calib_zero_raw_cache[i] = (int32_t)lroundf(v);
    }
    for (uint8_t i = count; i < ENCODER_TOTAL_NUM; i++) {
        sharedData->calib_zero_raw_cache[i] = 0;
    }
    sharedData->calib_zero_raw_valid = 1;
    if (lock) {
        xSemaphoreGive(lock);
    }
}
