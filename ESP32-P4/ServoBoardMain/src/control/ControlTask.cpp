#include "ControlTask.h"

#include <math.h>
#include <string.h>

#include "../calibration/CalibrationTask.h"
#include "ControlOutputBuilder.h"
#include "ControlSafety.h"
#include "ControlSolver.h"

#ifndef SOLVER_DIAG_LOG_ENABLE
#define SOLVER_DIAG_LOG_ENABLE 0
#endif

static ControlSolver g_controlSolver;

static const int32_t kEncoderModulo = 16384;
static const int32_t kEncoderMarginCounts = (kEncoderModulo * 10 + 180) / 360;

// 璋冭瘯鍖呭叧娉ㄧ殑鍏宠妭绱㈠紩鍒楄〃锛堜粎褰卞搷璋冭瘯杈撳嚭锛夈€?
static const uint8_t kDebugJointIndices[] = {4, 5, 6, 7};
static const uint8_t kDebugJointCount =
    (uint8_t)(sizeof(kDebugJointIndices) / sizeof(kDebugJointIndices[0]));

static const uint32_t kCanBusOfflineTimeoutMs = 300;
static const uint16_t kEncoderDisconnectRaw = 0xFFFF;

// 鍗忚鏄犲皠鍊奸檺骞咃紝閬垮紑鏂繛淇濈暀鍊艰寖鍥淬€?

static int16_t clampMappedCountForProtocol(int32_t value)
{
    if (value > 32766) return 32766;
    if (value < -32768) return -32768;
    return (int16_t)value;
}

// 鎸夊畨瑁呮柟鍚戠粺涓€缂栫爜鍣ㄦ柟鍚戯紝杈撳嚭鑼冨洿 [0, kEncoderModulo)銆?
static int32_t orientEncoderRaw(uint16_t rawValue, int8_t direction)
{
    int32_t oriented = (int32_t)rawValue & (kEncoderModulo - 1);
    if (direction < 0) {
        oriented = (kEncoderModulo - oriented) & (kEncoderModulo - 1);
    }
    return oriented;
}

static int32_t wrapEncoderCount(int32_t value)
{
    value %= kEncoderModulo;
    if (value < 0) value += kEncoderModulo;
    return value;
}

static int32_t forwardEncoderDelta(int32_t from, int32_t to)
{
    return wrapEncoderCount(to - from);
}

// Prefer calibrated offset, otherwise use the manual fallback table.
static int32_t getEncoderOffset(uint8_t jointIndex)
{
    if (jointIndex >= ENCODER_TOTAL_NUM) return 0;
    if (g_jointCalibResult[jointIndex].success) {
        return g_jointCalibResult[jointIndex].offset;
    }
    return g_encoderOffsetManual[jointIndex];
}

// Encoder counts to degrees.
static float convertEncoderCountToDeg(int32_t encoderCount)
{
    return (float)encoderCount * 360.0f / (float)kEncoderModulo;
}

static int32_t degToEncoderCount(float deg)
{
    const float scaled = deg * (float)kEncoderModulo / 360.0f;
    return (int32_t)(scaled + (scaled >= 0.0f ? 0.5f : -0.5f));
}

static float clampJointTargetDegByCalib(float targetDeg, uint8_t jointIndex)
{
    if (!isfinite(targetDeg)) {
        targetDeg = 0.0f;
    }
    if (jointIndex >= ENCODER_TOTAL_NUM) {
        return 0.0f;
    }

    float maxDeg = 0.0f;
    if (g_jointCalibResult[jointIndex].success) {
        maxDeg = g_jointCalibResult[jointIndex].angleMax;
    } else {
        maxDeg =
            g_jointCalibConfig[jointIndex].angleScope -
            g_jointCalibConfig[jointIndex].bottomReserved -
            g_jointCalibConfig[jointIndex].topReserved;
    }

    if (!isfinite(maxDeg) || maxDeg <= 0.0f) {
        return 0.0f;
    }
    if (targetDeg < 0.0f) {
        return 0.0f;
    }
    if (targetDeg > maxDeg) {
        return maxDeg;
    }
    return targetDeg;
}

// Find the motorMap channel for a physical bus/id pair.
static int findMotorChannel(uint8_t bus, uint8_t id)
{
    for (int i = 0; i < SERVO_TOTAL_NUM; i++)
    {
        if (motorMap[i].busIndex == bus && motorMap[i].servoID == id) {
            return i;
        }
    }
    return -1;
}

static bool readControlCommandSnapshot(TaskSharedData_t* sharedData, ControlCommandSnapshot_t* out)
{
// Read host commands once per control cycle to avoid mixed partial updates.
    if (!sharedData || !out) {
        return false;
    }

    SemaphoreHandle_t lock = sharedData->commandStateMutex ? sharedData->commandStateMutex : sharedData->targetAnglesMutex;
    if (lock && xSemaphoreTake(lock, pdMS_TO_TICKS(2)) != pdTRUE) {
        return false;
    }

    memcpy(out->targetAngles, sharedData->targetAngles, sizeof(out->targetAngles));
    memcpy(out->motorTargetRaw, sharedData->motorTargetRaw, sizeof(out->motorTargetRaw));
    memcpy(out->motorSweepTargetRaw, sharedData->motorSweepTargetRaw, sizeof(out->motorSweepTargetRaw));
    out->motorCommandToken = sharedData->motor_command_token;
    out->motorSweepCommandToken = sharedData->motor_sweep_command_token;
    out->jointCommandToken = sharedData->joint_command_token;
    out->motorDirectCommandGeneration = sharedData->motor_direct_command_generation;
    out->motorDirectCommandSource = sharedData->motor_direct_command_source;
    out->controlMode = sharedData->control_mode;
    memcpy(out->tendonGuardEnabled, (const void*)sharedData->tendon_guard_enabled, sizeof(out->tendonGuardEnabled));
    memcpy(out->tendonGuardSign, (const void*)sharedData->tendon_guard_sign, sizeof(out->tendonGuardSign));
    memcpy(out->tendonGuardX1Abs, (const void*)sharedData->tendon_guard_x1_abs, sizeof(out->tendonGuardX1Abs));

    if (lock) {
        xSemaphoreGive(lock);
    }
    return true;
}

// ControlTask 浠诲姟涓诲惊鐜細
// 1) 璇诲彇鑸垫満鍙嶉骞朵笂鎶ラ仴娴嬶紱
// 2) 鏋勫缓鍏宠妭鍙嶉閲忎笌浼犳劅鍣ㄦ槧灏勯噺锛?// 3) 鎵ц闄愬箙涓庡弻鐜В绠楋紱
// 4) 鏍规嵁鎺у埗妯″紡涓庡畨鍏ㄧ姸鎬佷笅鍙戝懡浠ゃ€?
void controlTask(void* parameter)
{
    TaskSharedData_t* sharedData = (TaskSharedData_t*)parameter;
    if (!sharedData) {
        vTaskDelete(NULL);
        return;
    }

    g_controlSolver.begin();

    float localTargets[ENCODER_TOTAL_NUM] = {0.0f};
    float magAngles[ENCODER_TOTAL_NUM] = {0.0f};
    int32_t absolutePosition[ENCODER_TOTAL_NUM] = {0};
    int16_t outPulses[ENCODER_TOTAL_NUM] = {0};

    RemoteSensorData_t sensorData;
    MappedAngleData_t mappedData;
    uint8_t joint16DiffFaultCounter = 0;
    const int joint16PrimaryCh =
        findMotorChannel(jointMap[kJoint16Index].busIndex, jointMap[kJoint16Index].servoID);
    const int joint16SecondaryCh =
        findMotorChannel(joint16SecondaryMotor.busIndex, joint16SecondaryMotor.servoID);
    ReleaseGuardState releaseGuards[ENCODER_TOTAL_NUM];
    resetAllReleaseGuardStates(releaseGuards);
    uint32_t lastReleaseFaultResetToken = sharedData->reverse_release_fault_reset_token;
    sharedData->reverse_release_fault_bitmap = 0;
    ControlCommandSnapshot_t commandSnapshot;
    memset(&commandSnapshot, 0, sizeof(commandSnapshot));
    readControlCommandSnapshot(sharedData, &commandSnapshot);

#if SOLVER_DIAG_LOG_ENABLE
    uint32_t lastDiagLogMs = 0;
#endif

    TickType_t lastWakeTime = xTaskGetTickCount();
    const TickType_t solverPeriodTicks = pdMS_TO_TICKS(10);
    bool prevMotorOnline[SERVO_TOTAL_NUM] = {false};
    bool hotplugHoldMotor[SERVO_TOTAL_NUM] = {false};
    uint32_t lastMotorCommandToken = commandSnapshot.motorCommandToken;
    uint32_t lastMotorSweepCommandToken = commandSnapshot.motorSweepCommandToken;
    uint32_t lastJointCommandToken = commandSnapshot.jointCommandToken;
    uint8_t lastMotorDirectSource = commandSnapshot.motorDirectCommandSource;
    int16_t prevSweepRawTarget[SERVO_TOTAL_NUM] = {0};
    int32_t sweepExpandedTarget[SERVO_TOTAL_NUM] = {0};
    bool sweepAnchorValid[SERVO_TOTAL_NUM] = {false};

    while (1)
    {
        if (!readControlCommandSnapshot(sharedData, &commandSnapshot)) {
            vTaskDelayUntil(&lastWakeTime, solverPeriodTicks);
            continue;
        }

        const uint32_t motorCommandToken = commandSnapshot.motorCommandToken;
        if (motorCommandToken != lastMotorCommandToken)
        {
            memset(hotplugHoldMotor, 0, sizeof(hotplugHoldMotor));
            lastMotorCommandToken = motorCommandToken;
        }

        const uint32_t motorSweepCommandToken = commandSnapshot.motorSweepCommandToken;
        if (motorSweepCommandToken != lastMotorSweepCommandToken)
        {
            memset(hotplugHoldMotor, 0, sizeof(hotplugHoldMotor));
            lastMotorSweepCommandToken = motorSweepCommandToken;
        }

        const uint32_t jointCommandToken = commandSnapshot.jointCommandToken;
        if (jointCommandToken != lastJointCommandToken)
        {
            memset(hotplugHoldMotor, 0, sizeof(hotplugHoldMotor));
            lastJointCommandToken = jointCommandToken;
        }

        const uint8_t motorDirectSource = commandSnapshot.motorDirectCommandSource;
        if (motorDirectSource != lastMotorDirectSource)
        {
            memset(sweepAnchorValid, 0, sizeof(sweepAnchorValid));
            lastMotorDirectSource = motorDirectSource;
        }

        const uint32_t releaseFaultResetToken = sharedData->reverse_release_fault_reset_token;
        if (releaseFaultResetToken != lastReleaseFaultResetToken)
        {
            resetAllReleaseGuardStates(releaseGuards);
            sharedData->reverse_release_fault_bitmap = 0;
            lastReleaseFaultResetToken = releaseFaultResetToken;
        }

        ServoTargetBatch_t targetBatch;
        memset(&targetBatch, 0, sizeof(targetBatch));
        targetBatch.timestamp = millis();
        targetBatch.source = SERVO_TARGET_OWNER_CONTROL;
        int16_t jointCmdPos[ENCODER_TOTAL_NUM] = {0};
        uint8_t jointCmdValid[ENCODER_TOTAL_NUM] = {0};

        ServoAngleData_t servoData;
        memset(&servoData, 0, sizeof(servoData));
        servoData.timestamp = millis();

        if (sharedData->servoFeedbackQueue) {
            xQueuePeek(sharedData->servoFeedbackQueue, &servoData, 0);
        }

        for (uint8_t ch = 0; ch < SERVO_TOTAL_NUM; ch++)
        {
            const bool onlineNow = (servoData.onlineStatus[ch] != 0);
            if (onlineNow && !prevMotorOnline[ch]) {
                hotplugHoldMotor[ch] = true;
                sweepAnchorValid[ch] = false;
            }
            if (!onlineNow) {
                sweepAnchorValid[ch] = false;
            }
            prevMotorOnline[ch] = onlineNow;
        }

        // 杩囪浇淇濇姢閫昏緫褰撳墠鍏抽棴锛屼絾淇濈暀鍗忚鐘舵€佸瓧娈电殑杈撳嚭鍏煎鎬с€?
        sharedData->overload_fault_bitmap = 0;

        // 闃舵3锛氫粠鑸垫満鍙嶉鏋勫缓鈥滃叧鑺備晶鈥濈粷瀵逛綅缃紙闂幆杈撳叆锛夈€?
        memset(absolutePosition, 0, sizeof(absolutePosition));
        for (uint8_t jointIndex = 0; jointIndex < ENCODER_TOTAL_NUM; jointIndex++)
        {
            const uint8_t bus = jointMap[jointIndex].busIndex;
            const uint8_t id = jointMap[jointIndex].servoID;
            const int ch = findMotorChannel(bus, id);
            if (ch >= 0 && servoData.onlineStatus[ch]) {
                absolutePosition[jointIndex] = servoData.servoAngles[ch];
            }
        }

        bool joint16DualFault = false;
        {
            const uint8_t pBus = jointMap[kJoint16Index].busIndex;
            const uint8_t pId = jointMap[kJoint16Index].servoID;
            const uint8_t sBus = joint16SecondaryMotor.busIndex;
            const uint8_t sId = joint16SecondaryMotor.servoID;
            const int pCh = findMotorChannel(pBus, pId);
            const int sCh = findMotorChannel(sBus, sId);
            const bool pOnline = (pCh >= 0) && (servoData.onlineStatus[pCh] != 0);
            const bool sOnline = (sCh >= 0) && (servoData.onlineStatus[sCh] != 0);

            if (pOnline && sOnline)
            {
                const int32_t p = servoData.servoAngles[pCh];
                const int32_t s = servoData.servoAngles[sCh];
                const int32_t sProj = -s + kJoint16SecondaryOffset;
                const int32_t fused = (p + sProj) / 2;
                const int32_t diff = (p >= sProj) ? (p - sProj) : (sProj - p);

                // 鍙屽弽棣堣瀺鍚堝悗鐨?joint16 浣嶇疆鐢ㄤ簬闂幆銆?
                absolutePosition[kJoint16Index] = fused;

                // 杩炵画瓒呭樊鍒ゆ晠闅滐紝鎶戝埗鍗曞懆鏈熸姈鍔ㄨ瑙﹀彂銆?
                if (diff > kJoint16DiffThresholdCounts) {
                    if (joint16DiffFaultCounter < 255) {
                        joint16DiffFaultCounter++;
                    }
                } else {
                    joint16DiffFaultCounter = 0;
                }
                joint16DualFault = (joint16DiffFaultCounter >= kJoint16DiffFaultCycles);
            }
            else
            {
                joint16DiffFaultCounter = 0;
                joint16DualFault = true;
            }
        }
        sharedData->joint16_dual_feedback_fault = joint16DualFault ? 1 : 0;

#if SOLVER_DIAG_LOG_ENABLE
        const uint32_t nowMs = millis();
        if (nowMs - lastDiagLogMs >= 500) {
            lastDiagLogMs = nowMs;
            const int pCh = findMotorChannel(jointMap[kJoint16Index].busIndex, jointMap[kJoint16Index].servoID);
            const int sCh = findMotorChannel(joint16SecondaryMotor.busIndex, joint16SecondaryMotor.servoID);
            const long pAbs = (pCh >= 0) ? (long)servoData.servoAngles[pCh] : 0;
            const long sAbs = (sCh >= 0) ? (long)servoData.servoAngles[sCh] : 0;
            Serial.printf("[Solver] j16 p=%ld s=%ld fault=%d\r\n", pAbs, sAbs, joint16DualFault ? 1 : 0);
        }
#endif

        bool canBusOnline = false;
        if (xQueuePeek(sharedData->canRxQueue, &sensorData, 0) == pdTRUE && sensorData.isValid) {
            const uint32_t nowMs = millis();
            canBusOnline = (nowMs - sensorData.timestamp) <= kCanBusOfflineTimeoutMs;
        }

        memset(&mappedData, 0, sizeof(mappedData));
        mappedData.timestamp = millis();
        mappedData.isValid = canBusOnline;

        if (canBusOnline)
        {
            for (int i = 0; i < ENCODER_TOTAL_NUM; i++)
            {
                if (sensorData.encoderValues[i] == kEncoderDisconnectRaw) {
                    mappedData.validFlags[i] = 0;
                    magAngles[i] = 0.0f;
                    continue;
                }

                const int32_t orientedRaw = orientEncoderRaw(sensorData.encoderValues[i], g_encoderDirection[i]);
                int32_t mappedCount = orientedRaw;
                if (g_jointCalibResult[i].success) {
                    const JointCalibrationResult& calib = g_jointCalibResult[i];
                    const int32_t offset = getEncoderOffset((uint8_t)i);
                    int32_t delta = forwardEncoderDelta(offset, orientedRaw);
                    int32_t hardMaxFromOffset = forwardEncoderDelta(offset, calib.encoderMax);
                    if (hardMaxFromOffset <= 0) {
                        hardMaxFromOffset = degToEncoderCount(calib.angleMax);
                    }
                    if (delta > hardMaxFromOffset + kEncoderMarginCounts) {
                        delta -= kEncoderModulo;
                    }
                    mappedCount = delta;
                }

                mappedData.angleValues[i] = clampMappedCountForProtocol(mappedCount);
                mappedData.validFlags[i] = 1;
                magAngles[i] = convertEncoderCountToDeg(mappedCount);
            }
        }

        if (sharedData->mappedAngleQueue) {
            xQueueOverwrite(sharedData->mappedAngleQueue, &mappedData);
        }

        memcpy(localTargets, commandSnapshot.targetAngles, sizeof(localTargets));
        const bool hostControlEnabled = (sharedData->control_enabled != 0);
        const uint8_t systemState = sharedData->system_state;
        const uint8_t targetOwner = sharedData->servo_target_owner;
        const bool controlOwnerActive = (targetOwner == SERVO_TARGET_OWNER_CONTROL);
        const bool runningState = (systemState == SYSTEM_STATE_RUNNING);
        const bool faultHoldState = (systemState == SYSTEM_STATE_FAULT_HOLD);
        const bool controlEnabled = hostControlEnabled && controlOwnerActive && (runningState || faultHoldState);
        const bool normalOutputAllowed = controlEnabled && runningState;

        // 浠呭叧鑺傛帶鍒舵ā寮忎笅鎵ц瑙掑害闄愬箙锛堢洿鎺фā寮忛€忎紶鍘熷鐩爣锛夈€?
        if (commandSnapshot.controlMode == CONTROL_MODE_JOINT) {
            for (uint8_t jointIndex = 0; jointIndex < ENCODER_TOTAL_NUM; jointIndex++) {
                localTargets[jointIndex] = clampJointTargetDegByCalib(localTargets[jointIndex], jointIndex);
            }
        }

        // 闃舵4锛氭墽琛屽弻鐜В绠楋紝寰楀埌姣忓叧鑺傜洰鏍囪剦鍐层€?
        g_controlSolver.compute(localTargets, magAngles, absolutePosition, outPulses);

        if (commandSnapshot.controlMode == CONTROL_MODE_DIRECT_MOTOR)
        {
            // 鐩存帶妯″紡锛氭寜 motorTargetRaw 涓嬪彂锛沯oint16 鍙屽弽棣堟晠闅滀粎浣滅敤浜庝富鍓埖鏈哄銆?
            if (controlEnabled) {
                for (uint8_t ch = 0; ch < SERVO_TOTAL_NUM; ch++)
                {
                    const uint8_t bus = motorMap[ch].busIndex;
                    const uint8_t id = motorMap[ch].servoID;
                    if (bus >= NUM_BUSES) {
                        continue;
                    }

                    const bool isJoint16Motor =
                        ((int)ch == joint16PrimaryCh) || ((int)ch == joint16SecondaryCh);
                    if (!normalOutputAllowed || (joint16DualFault && isJoint16Motor))
                    {
                        if (servoData.onlineStatus[ch] != 0) {
                            const int16_t holdPos = clampServoPos(servoData.servoAngles[ch]);
                            appendServoTarget(&targetBatch, bus, id, holdPos,
                                              SERVO_TARGET_SPEED_DEFAULT, SERVO_TARGET_ACC_DEFAULT);
                        }
                        continue;
                    }

                    int16_t targetPos = clampServoPos(servoData.servoAngles[ch]);
                    if (!hotplugHoldMotor[ch]) {
                        if (motorDirectSource == MOTOR_DIRECT_SOURCE_SWEEP) {
                            const int16_t rawTarget =
                                clampServoSingleTurnRaw(commandSnapshot.motorSweepTargetRaw[ch]);
                            if (!sweepAnchorValid[ch]) {
                                sweepExpandedTarget[ch] =
                                    anchorSingleTurnSweepToCurrent(servoData.servoAngles[ch], rawTarget);
                                prevSweepRawTarget[ch] = rawTarget;
                                sweepAnchorValid[ch] = true;
                            } else if (rawTarget != prevSweepRawTarget[ch]) {
                                const int32_t d = (int32_t)rawTarget - (int32_t)prevSweepRawTarget[ch];
                                sweepExpandedTarget[ch] += wrapServoRawDelta(d);
                                prevSweepRawTarget[ch] = rawTarget;
                            }
                            targetPos = clampServoPos(sweepExpandedTarget[ch]);
                            sweepExpandedTarget[ch] = targetPos;
                        } else if (motorDirectSource == MOTOR_DIRECT_SOURCE_ABSOLUTE) {
                            sweepAnchorValid[ch] = false;
                            targetPos = clampServoPos((int32_t)commandSnapshot.motorTargetRaw[ch]);
                        } else {
                            sweepAnchorValid[ch] = false;
                            targetPos = expandSingleTurnTargetNearCurrent(
                                servoData.servoAngles[ch],
                                clampServoSingleTurnRaw(commandSnapshot.motorTargetRaw[ch])
                            );
                        }
                    } else {
                        sweepAnchorValid[ch] = false;
                    }
                    appendServoTarget(&targetBatch, bus, id, targetPos,
                                      SERVO_TARGET_SPEED_DEFAULT, SERVO_TARGET_ACC_DEFAULT);
                }
            }
        }
        else
        {
            // 鍏宠妭妯″紡锛氭寜鍏宠妭閫愪竴璇勪及鎬ュ仠鏉′欢骞朵笅鍙戠洰鏍囥€?
            for (uint8_t jointIndex = 0; jointIndex < ENCODER_TOTAL_NUM; jointIndex++)
            {
                const uint8_t bus = jointMap[jointIndex].busIndex;
                const uint8_t id = jointMap[jointIndex].servoID;
                const int jointMotorCh = findMotorChannel(bus, id);
                const bool servoOnline = (jointMotorCh >= 0) && (servoData.onlineStatus[jointMotorCh] != 0);
                const bool mappedValid = canBusOnline && (mappedData.validFlags[jointIndex] != 0);
                const bool guardInputsValid = servoOnline && mappedValid;
                bool releaseFaultActive = false;
                bool holdByTendonGuard = false;

                if (jointIndex != kJoint16Index)
                {
                    const int32_t servoPos =
                        (jointMotorCh >= 0) ? servoData.servoAngles[jointMotorCh] : absolutePosition[jointIndex];
                    releaseFaultActive = updateReleaseGuardState(&releaseGuards[jointIndex],
                                                                 localTargets[jointIndex],
                                                                 magAngles[jointIndex],
                                                                 servoPos,
                                                                 guardInputsValid);
                    holdByTendonGuard = shouldBlockReleaseByTendonGuard(
                        &commandSnapshot,
                        jointIndex,
                        localTargets[jointIndex],
                        magAngles[jointIndex],
                        absolutePosition[jointIndex],
                        guardInputsValid
                    );
                }

                bool emergency =
                    (!normalOutputAllowed) ||
                    shouldEmergencyStop(canBusOnline, sensorData, mappedData, jointIndex);
                if (jointIndex == kJoint16Index) {
                    emergency = emergency || joint16DualFault;
                } else {
                    emergency = emergency || releaseFaultActive || holdByTendonGuard;
                }
                const bool hotplugHoldActive =
                    (jointMotorCh >= 0) &&
                    (jointMotorCh < SERVO_TOTAL_NUM) &&
                    hotplugHoldMotor[jointMotorCh];

                if (bus >= NUM_BUSES) {
                    continue;
                }

                if (!controlEnabled)
                {
                    // 鏈敹鍒版槑纭?START 鍓嶏紝涓嶅悜鑸垫満鍙戦€佷换浣曚繚鎸?鎺у埗鍛戒护銆?
                    continue;
                }

                if (emergency)
                {
                    // 鎬ュ仠绛栫暐锛氫繚鎸佸綋鍓嶈埖鏈轰綅缃紝閬垮厤寮傚父鐘舵€佷笅缁х画杩愬姩銆?
                    if (servoOnline) {
                        const int16_t holdPos = clampServoPos(absolutePosition[jointIndex]);
                        appendServoTarget(&targetBatch, bus, id, holdPos,
                                          SERVO_TARGET_SPEED_DEFAULT, SERVO_TARGET_ACC_DEFAULT);
                        jointCmdPos[jointIndex] = holdPos;
                        jointCmdValid[jointIndex] = 1;
                    }

                    if (jointIndex == kJoint16Index) {
                        const int secCh = findMotorChannel(joint16SecondaryMotor.busIndex, joint16SecondaryMotor.servoID);
                        if (secCh >= 0 && servoData.onlineStatus[secCh] != 0) {
                            const int16_t holdSec = (secCh >= 0) ? clampServoPos(servoData.servoAngles[secCh]) : 0;
                            appendServoTarget(&targetBatch,
                                              joint16SecondaryMotor.busIndex,
                                              joint16SecondaryMotor.servoID,
                                              holdSec,
                                              SERVO_TARGET_SPEED_DEFAULT,
                                              SERVO_TARGET_ACC_DEFAULT);
                        }
                    }
                    continue;
                }

                if (hotplugHoldActive)
                {
                    if (servoOnline) {
                        const int16_t holdPos = clampServoPos(absolutePosition[jointIndex]);
                        appendServoTarget(&targetBatch, bus, id, holdPos,
                                          SERVO_TARGET_SPEED_DEFAULT, SERVO_TARGET_ACC_DEFAULT);
                        jointCmdPos[jointIndex] = holdPos;
                        jointCmdValid[jointIndex] = 1;
                    }

                    if (jointIndex == kJoint16Index) {
                        const int secCh = findMotorChannel(joint16SecondaryMotor.busIndex, joint16SecondaryMotor.servoID);
                        if (secCh >= 0 && servoData.onlineStatus[secCh] != 0) {
                            const int16_t holdSec = (secCh >= 0) ? clampServoPos(servoData.servoAngles[secCh]) : 0;
                            appendServoTarget(&targetBatch,
                                              joint16SecondaryMotor.busIndex,
                                              joint16SecondaryMotor.servoID,
                                              holdSec,
                                              SERVO_TARGET_SPEED_DEFAULT,
                                              SERVO_TARGET_ACC_DEFAULT);
                        }
                    }
                    continue;
                }

                const int16_t targetPos = clampServoPos(outPulses[jointIndex]);
                appendServoTarget(&targetBatch, bus, id, targetPos,
                                  SERVO_TARGET_SPEED_DEFAULT, SERVO_TARGET_ACC_DEFAULT);
                jointCmdPos[jointIndex] = targetPos;
                jointCmdValid[jointIndex] = 1;

                if (jointIndex == kJoint16Index) {
                    if (joint16SecondaryMotor.busIndex < NUM_BUSES) {
                        // joint16 鎷姉涓嬪彂锛氫富鑸垫満鐩爣纭畾鍚庯紝鍓埖鏈烘寜鍙嶅悜鏄犲皠涓庡亸缃ˉ鍋胯绠椼€?
                        const int32_t secTarget = -((int32_t)targetPos) + kJoint16SecondaryOffset + kJoint16TensionBias;
                        appendServoTarget(&targetBatch,
                            joint16SecondaryMotor.busIndex,
                            joint16SecondaryMotor.servoID,
                            clampServoPos(secTarget),
                            SERVO_TARGET_SPEED_DEFAULT,
                            SERVO_TARGET_ACC_DEFAULT
                        );
                    }
                }
            }
        }

        sharedData->reverse_release_fault_bitmap = buildReleaseFaultBitmap(releaseGuards);

        if (sharedData->jointDebugQueue)
        {
            // 璋冭瘯涓婃姤锛氬彂閫佹寚瀹氬叧鑺傜殑鐩爣/鍙嶉/PID 杈撳嚭涓庡懡浠や綅缃€?
            for (uint8_t di = 0; di < kDebugJointCount; di++)
            {
                const uint8_t jointIndex = kDebugJointIndices[di];
                JointDebugData_t debugData;
                memset(&debugData, 0, sizeof(debugData));
                debugData.jointIndex = jointIndex;
                debugData.timestamp = millis();
                debugData.valid = (mappedData.validFlags[jointIndex] != 0) ? 1 : 0;
                debugData.targetDeg = localTargets[jointIndex];
                debugData.magActualDeg = magAngles[jointIndex];
                debugData.loop1Output = g_controlSolver.getPidOutput(jointIndex, 0);
                debugData.loop2Actual = (float)absolutePosition[jointIndex];
                debugData.loop2Output = g_controlSolver.getPidOutput(jointIndex, 1);
                debugData.cmdTargetPos = jointCmdPos[jointIndex];
                debugData.cmdValid = jointCmdValid[jointIndex];

                if (xQueueSend(sharedData->jointDebugQueue, &debugData, 0) != pdTRUE) {
                    JointDebugData_t drop;
                    xQueueReceive(sharedData->jointDebugQueue, &drop, 0);
                    xQueueSend(sharedData->jointDebugQueue, &debugData, 0);
                }
            }
        }

        if (sharedData->servoTargetQueue && targetBatch.count > 0) {
            xQueueOverwrite(sharedData->servoTargetQueue, &targetBatch);
        }

        vTaskDelayUntil(&lastWakeTime, solverPeriodTicks);
    }
}
