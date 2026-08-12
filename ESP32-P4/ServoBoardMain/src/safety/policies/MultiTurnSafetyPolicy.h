/**
 * @file MultiTurnSafetyPolicy.h
 * @brief 定义多圈快照何时保存以及单圈边界如何识别的纯策略函数。
 *
 * 本文件不访问队列、NVS 或舵机总线，因此可以独立测试；持久化任务只负责
 * 执行策略结果，不应重新复制状态判断。
 */
#ifndef P4_SAFETY_POLICIES_MULTI_TURN_SAFETY_POLICY_H
#define P4_SAFETY_POLICIES_MULTI_TURN_SAFETY_POLICY_H

#include <stdint.h>

namespace p4::recovery {

/** 返回当前系统状态是否可能改变舵机多圈位置。 */
bool isMultiTurnMotionState(uint8_t systemState);

/** 将刚结束的运动状态映射为 NVS 保存原因。 */
uint8_t completedMotionSaveReason(uint8_t systemState);

/** 判断 raw 是否位于 0/4096 跨圈边界附近。 */
bool isRawNearTurnEdge(int16_t raw);

} // namespace p4::recovery

#endif // P4_SAFETY_POLICIES_MULTI_TURN_SAFETY_POLICY_H
