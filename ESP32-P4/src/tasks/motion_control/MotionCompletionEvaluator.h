/**
 * @file MotionCompletionEvaluator.h
 * @brief 判断直控或关节反馈命令是否到位。
 *
 * 完成判定属于运行流程语义，不属于安全策略，因此独立于 safety/policies。
 */
#ifndef P4_TASKS_MOTION_CONTROL_MOTION_COMPLETION_EVALUATOR_H
#define P4_TASKS_MOTION_CONTROL_MOTION_COMPLETION_EVALUATOR_H

#include "../../system/SystemRuntimeContext.h"

namespace p4::control {

bool directTargetsReached(SystemRuntimeContext* runtime,
                          const ServoAngleData_t& servoData,
                          const ControlCommandSnapshot_t& command);

bool feedbackTargetsReached(SystemRuntimeContext* runtime,
                            const MappedAngleData_t& mappedData,
                            const float* targetDegs,
                            const float* actualDegs,
                            uint32_t commandJointMask);

} // namespace p4::control

#endif // P4_TASKS_MOTION_CONTROL_MOTION_COMPLETION_EVALUATOR_H
