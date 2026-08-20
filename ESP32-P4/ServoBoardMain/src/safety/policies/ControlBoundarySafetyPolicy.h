/**
 * @file ControlBoundarySafetyPolicy.h
 * @brief 集中定义控制域的角度边界、热插拔保持和工作区告警判断。
 *
 * 本文件只接收事实并返回判断结果，不访问队列、不写系统状态，也不发送舵机命令。
 * MotionControlService 负责根据结果执行 HOLD、钳位、日志和安全事件上报。
 */
#ifndef P4_SAFETY_POLICIES_CONTROL_BOUNDARY_SAFETY_POLICY_H
#define P4_SAFETY_POLICIES_CONTROL_BOUNDARY_SAFETY_POLICY_H

#include <stdint.h>

namespace p4::safety {

/** @brief 标量边界判断结果；valid=false 表示边界本身不可用于控制。 */
struct BoundaryClampDecision {
    bool valid;
    bool clamped;
    float requested;
    float applied;
};

/** @brief 舵机 logical ABS 整数边界判断结果。 */
struct AbsBoundaryClampDecision {
    bool valid;
    bool clamped;
    int32_t requested;
    int32_t applied;
};

/** @brief 将浮点目标限制到有效闭区间；非法输入返回 valid=false。 */
BoundaryClampDecision clampFloatToBoundary(float requested,
                                           float minimum,
                                           float maximum);

/** @brief 将舵机 logical ABS 目标限制到有效闭区间。 */
AbsBoundaryClampDecision clampAbsToBoundary(int32_t requested,
                                            int32_t minimum,
                                            int32_t maximum);

/** @brief 上工作区越界的去抖结果。 */
struct WorkspaceWarningDecision {
    uint8_t nextStableCount;
    bool active;
    bool entered;
    bool recovered;
};

/** @brief 根据越界量和连续帧计数更新工作区告警状态。 */
WorkspaceWarningDecision evaluateUpperWorkspaceWarning(
    int32_t outsideCounts,
    int32_t warningThresholdCounts,
    uint8_t stableFrames,
    uint8_t previousStableCount,
    bool previouslyActive);

/** @brief 判断舵机是否发生离线->在线；首次建立反馈同样需要先 HOLD 当前 ABS。 */
bool shouldEnterHotplugHold(bool wasOnline, bool isOnline);

/** @brief 已建立在线基线后发生的离线->在线才上报为热插拔事件。 */
bool shouldReportHotplugHold(bool baselineValid,
                             bool wasOnline,
                             bool isOnline);

} // namespace p4::safety

#endif // P4_SAFETY_POLICIES_CONTROL_BOUNDARY_SAFETY_POLICY_H
