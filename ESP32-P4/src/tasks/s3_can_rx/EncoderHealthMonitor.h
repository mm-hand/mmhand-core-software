/**
 * @file EncoderHealthMonitor.h
 * @brief 声明 21 路磁编码器在线、警告和故障去抖状态机。
 * @details
 * - 线程：仅由 S3CanRxTask 调用，内部状态由该任务独占。
 * - 输入：CRC 通过快照的有效位图、整批坏帧事件和单调时间。
 * - 输出：原子发布 encoder online/warn/fault 与 CAN 链路状态。
 */
#ifndef P4_TASKS_S3_CAN_RX_ENCODER_HEALTH_MONITOR_H
#define P4_TASKS_S3_CAN_RX_ENCODER_HEALTH_MONITOR_H

#include <stdint.h>

#include "../../config/DimensionsConfig.h"
#include "../../safety/policies/EncoderHealthSafetyPolicy.h"

struct SystemRuntimeContext;

using EncoderHealthState = p4::safety::EncoderHealthState;

void updateEncoderHealthFromSnapshot(SystemRuntimeContext* sharedData,
                                     EncoderHealthState* health,
                                     uint32_t validMask,
                                     uint32_t nowMs);
void markEncoderSnapshotBad(SystemRuntimeContext* sharedData,
                            EncoderHealthState* health,
                            uint32_t nowMs);
bool applyEncoderHealthTimeout(SystemRuntimeContext* sharedData,
                               EncoderHealthState* health,
                               uint32_t nowMs);
void clearEncoderHealth(SystemRuntimeContext* sharedData,
                        EncoderHealthState* health);

#endif // P4_TASKS_S3_CAN_RX_ENCODER_HEALTH_MONITOR_H
