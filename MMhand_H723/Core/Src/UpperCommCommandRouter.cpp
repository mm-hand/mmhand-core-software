#include "UpperCommCommandRouter.h"

#include "HardwareMap.h"
#include "main.h"
#include "task.h"

#include <cmath>

// 根据上位机事件更新当前最小系统状态，并把事件投递给后续可能接入的状态机。
extern "C" void postSystemEvent(TaskSharedData_t *sharedData, uint8_t event)
{
    if (sharedData == nullptr) {
        return;
    }

    SystemEvent_t systemEvent = {};
    systemEvent.event = event;
    systemEvent.timestamp = HAL_GetTick();

    if (sharedData->stateEventQueue != nullptr) {
        (void)xQueueSend(sharedData->stateEventQueue, &systemEvent, 0);
    }

    SemaphoreHandle_t lock = sharedData->commandStateMutex;
    const bool locked = (lock != nullptr) && (xSemaphoreTake(lock, pdMS_TO_TICKS(10)) == pdTRUE);

    switch (event) {
    case SYSTEM_EVENT_START:
        sharedData->control_enabled = 1;
        sharedData->system_state = SYSTEM_STATE_READY;
        sharedData->overload_fault_reset_token++;
        sharedData->reverse_release_fault_reset_token++;
        break;
    case SYSTEM_EVENT_RUN_DIRECT:
        sharedData->control_enabled = 1;
        sharedData->system_state = SYSTEM_STATE_RUN_DIRECT;
        sharedData->servo_target_owner = SERVO_TARGET_OWNER_CONTROL;
        break;
    case SYSTEM_EVENT_RUN_FEEDBACK:
        sharedData->control_enabled = 1;
        sharedData->system_state = SYSTEM_STATE_RUN_FEEDBACK;
        sharedData->servo_target_owner = SERVO_TARGET_OWNER_CONTROL;
        break;
    case SYSTEM_EVENT_PAUSE:
        sharedData->control_enabled = 0;
        sharedData->paused_resume_state = sharedData->system_state;
        sharedData->system_state = SYSTEM_STATE_PAUSED;
        break;
    case SYSTEM_EVENT_STOP:
        sharedData->control_enabled = 0;
        sharedData->system_state = SYSTEM_STATE_STOPPED;
        sharedData->servo_target_owner = SERVO_TARGET_OWNER_NONE;
        sharedData->servo_emergency_stop_token++;
        break;
    case SYSTEM_EVENT_RESET:
        sharedData->control_enabled = 0;
        sharedData->system_state = SYSTEM_STATE_READY;
        sharedData->servo_target_owner = SERVO_TARGET_OWNER_NONE;
        sharedData->system_fault_bitmap = 0;
        sharedData->system_fault_latched_bitmap = 0;
        sharedData->overload_fault_bitmap = 0;
        sharedData->reverse_release_fault_bitmap = 0;
        sharedData->overload_fault_reset_token++;
        sharedData->reverse_release_fault_reset_token++;
        break;
    case SYSTEM_EVENT_CALIBRATE:
        sharedData->control_enabled = 0;
        sharedData->system_state = SYSTEM_STATE_CALIBRATION_RUNNING;
        sharedData->servo_target_owner = SERVO_TARGET_OWNER_CALIBRATION;
        break;
    default:
        break;
    }

    sharedData->state_transition_count++;

    if (locked) {
        xSemaphoreGive(lock);
    }
}

// 将上位机绝对位置命令钳制到多圈安全范围。
int16_t clampServoAbsCommand(int32_t value)
{
    if (value > 30719) {
        return 30719;
    }
    if (value < -30719) {
        return -30719;
    }
    return static_cast<int16_t>(value);
}

// H723 当前还没有完整 ControlTask，这里把直控电机命令转成 P4 同款 servoTargetQueue batch。
static void publishDirectMotorTargetBatch(TaskSharedData_t *sharedData,
                                          const int32_t *targets,
                                          uint8_t count)
{
    if ((sharedData == nullptr) || (targets == nullptr) || (sharedData->servoTargetQueue == nullptr)) {
        return;
    }
    if (count > SERVO_TOTAL_NUM) {
        count = SERVO_TOTAL_NUM;
    }

    ServoTargetBatch_t batch = {};
    batch.source = SERVO_TARGET_OWNER_CONTROL;
    batch.timestamp = HAL_GetTick();

    const uint32_t activeMask = sharedData->active_motor_mask & validMotorChannelMask();
    for (uint8_t ch = 0U; (ch < count) && (batch.count < SERVO_TARGET_BATCH_MAX); ch++) {
        const uint32_t bit = (1UL << ch);
        if (((activeMask & bit) == 0U) || !isMotorChannelValid(ch)) {
            continue;
        }

        ServoTargetCommand_t *cmd = &batch.commands[batch.count++];
        cmd->busIndex = motorMap[ch].busIndex;
        cmd->servoId = motorMap[ch].servoID;
        cmd->position = clampServoAbsCommand(targets[ch]);
        cmd->speed = SERVO_TARGET_SPEED_DEFAULT;
        cmd->acc = SERVO_TARGET_ACC_DEFAULT;
    }

    if (batch.count > 0U) {
        xQueueOverwrite(sharedData->servoTargetQueue, &batch);
    }
}

// 写入完整关节目标数组，并切换到关节控制模式。
void upperApplyTargetAngles(TaskSharedData_t *sharedData, const float *angles, uint8_t count)
{
    if ((sharedData == nullptr) || (angles == nullptr)) {
        return;
    }
    if (count > ENCODER_TOTAL_NUM) {
        count = ENCODER_TOTAL_NUM;
    }

    SemaphoreHandle_t lock =
        (sharedData->commandStateMutex != nullptr) ? sharedData->commandStateMutex : sharedData->targetAnglesMutex;
    const bool locked = (lock != nullptr) && (xSemaphoreTake(lock, pdMS_TO_TICKS(10)) == pdTRUE);

    for (uint8_t i = 0; i < count; i++) {
        sharedData->targetAngles[i] = angles[i];
    }
    sharedData->joint_command_token++;
    sharedData->control_mode = CONTROL_MODE_JOINT;

    if (locked) {
        xSemaphoreGive(lock);
    }
}

// 清零关节目标，兼容 P4 历史 'b' 命令。
void upperClearTargetAngles(TaskSharedData_t *sharedData)
{
    float zeroAngles[ENCODER_TOTAL_NUM] = {};
    upperApplyTargetAngles(sharedData, zeroAngles, ENCODER_TOTAL_NUM);
}

// 写入直控电机目标，并发布 RUN_DIRECT 事件。
void upperApplyMotorTargets(TaskSharedData_t *sharedData,
                            const int32_t *targets,
                            uint8_t count,
                            uint8_t source)
{
    if ((sharedData == nullptr) || (targets == nullptr)) {
        return;
    }
    if (count > SERVO_TOTAL_NUM) {
        count = SERVO_TOTAL_NUM;
    }

    SemaphoreHandle_t lock = sharedData->commandStateMutex;
    const bool locked = (lock != nullptr) && (xSemaphoreTake(lock, pdMS_TO_TICKS(10)) == pdTRUE);

    for (uint8_t i = 0; i < count; i++) {
        sharedData->motorTargetRaw[i] = targets[i];
    }
    sharedData->motor_command_token++;
    sharedData->motor_direct_command_generation++;
    sharedData->motor_direct_command_source = source;
    sharedData->control_mode = CONTROL_MODE_DIRECT_MOTOR;

    if (locked) {
        xSemaphoreGive(lock);
    }

    postSystemEvent(sharedData, SYSTEM_EVENT_RUN_DIRECT);
    publishDirectMotorTargetBatch(sharedData, targets, count);
}

// 写入 sweep 模式目标，并发布 RUN_DIRECT 事件。
void upperApplyMotorSweepTargets(TaskSharedData_t *sharedData, const int32_t *targets, uint8_t count)
{
    if ((sharedData == nullptr) || (targets == nullptr)) {
        return;
    }
    if (count > SERVO_TOTAL_NUM) {
        count = SERVO_TOTAL_NUM;
    }

    SemaphoreHandle_t lock = sharedData->commandStateMutex;
    const bool locked = (lock != nullptr) && (xSemaphoreTake(lock, pdMS_TO_TICKS(10)) == pdTRUE);

    for (uint8_t i = 0; i < count; i++) {
        sharedData->motorSweepTargetRaw[i] = targets[i];
    }
    sharedData->motor_sweep_command_token++;
    sharedData->motor_direct_command_generation++;
    sharedData->motor_direct_command_source = MOTOR_DIRECT_SOURCE_SWEEP;
    sharedData->control_mode = CONTROL_MODE_DIRECT_MOTOR;

    if (locked) {
        xSemaphoreGive(lock);
    }

    postSystemEvent(sharedData, SYSTEM_EVENT_RUN_DIRECT);
    publishDirectMotorTargetBatch(sharedData, targets, count);
}

// 写入 tendon guard 配置，数组顺序保持与编码器/关节通道一致。
void upperApplyTendonGuardConfig(TaskSharedData_t *sharedData,
                                 const uint8_t *enabled,
                                 const int8_t *sign,
                                 const int16_t *x1Abs,
                                 uint8_t count)
{
    if ((sharedData == nullptr) || (enabled == nullptr) || (sign == nullptr) || (x1Abs == nullptr)) {
        return;
    }
    if (count > ENCODER_TOTAL_NUM) {
        count = ENCODER_TOTAL_NUM;
    }

    SemaphoreHandle_t lock = sharedData->commandStateMutex;
    const bool locked = (lock != nullptr) && (xSemaphoreTake(lock, pdMS_TO_TICKS(10)) == pdTRUE);

    for (uint8_t i = 0; i < count; i++) {
        sharedData->tendon_guard_enabled[i] = enabled[i] ? 1U : 0U;
        sharedData->tendon_guard_sign[i] = (sign[i] < 0) ? -1 : 1;
        sharedData->tendon_guard_x1_abs[i] = clampServoAbsCommand(x1Abs[i]);
    }

    if (locked) {
        xSemaphoreGive(lock);
    }
}

// 缓存上位机下发的校准 raw 零点数据。
void upperCacheCalibZeroRaw(TaskSharedData_t *sharedData, const float *values, uint8_t count)
{
    if ((sharedData == nullptr) || (values == nullptr)) {
        return;
    }
    if (count > ENCODER_TOTAL_NUM) {
        count = ENCODER_TOTAL_NUM;
    }

    SemaphoreHandle_t lock = sharedData->commandStateMutex;
    const bool locked = (lock != nullptr) && (xSemaphoreTake(lock, pdMS_TO_TICKS(10)) == pdTRUE);

    for (uint8_t i = 0; i < count; i++) {
        float value = values[i];
        if (!std::isfinite(value)) {
            value = 0.0f;
        }
        sharedData->calib_zero_raw_cache[i] = static_cast<int32_t>(std::lround(value));
    }
    for (uint8_t i = count; i < ENCODER_TOTAL_NUM; i++) {
        sharedData->calib_zero_raw_cache[i] = 0;
    }
    sharedData->calib_zero_raw_valid = 1;

    if (locked) {
        xSemaphoreGive(lock);
    }
}

// 写入上位机当前激活范围，并清理范围切换导致的离线类故障。
void upperApplyActiveScope(TaskSharedData_t *sharedData,
                           uint32_t activeMotorMask,
                           uint32_t activeJointMask,
                           uint8_t requireCanFeedback)
{
    if (sharedData == nullptr) {
        return;
    }

    sharedData->active_motor_mask = activeMotorMask & validMotorChannelMask();
    sharedData->active_joint_mask = activeJointMask & JOINT_CHANNEL_MASK_ALL;
    sharedData->require_can_feedback = requireCanFeedback ? 1U : 0U;

    const uint32_t scopeFaultMask = SYSTEM_FAULT_CAN_OFFLINE | SYSTEM_FAULT_SERVO_OFFLINE;
    sharedData->system_fault_bitmap &= ~scopeFaultMask;
    sharedData->system_fault_latched_bitmap &= ~scopeFaultMask;
}
