/**
 * @file SingleTendonWorkspace.h
 * @brief 提供单电机关节在自动标定 LUT 工作空间内的舵机目标钳位。
 */
#ifndef P4_SINGLE_TENDON_WORKSPACE_H
#define P4_SINGLE_TENDON_WORKSPACE_H

#include <stdint.h>

namespace p4::control {

/**
 * 将单腱目标限制在本次标定实际走过的舵机 ABS 区间。
 *
 * @return true 表示存在可用 LUT 并已输出边界；false 表示该关节不能使用
 *         LUT 约束，调用方不得把未初始化的输出值用于安全判断。
 */
bool clampSingleTendonTargetByLut(uint8_t jointIndex,
                                  uint8_t motorChannel,
                                  int32_t requestedAbs,
                                  int32_t* appliedAbs,
                                  int32_t* lutMinAbs,
                                  int32_t* lutMaxAbs);

} // namespace p4::control

#endif // P4_SINGLE_TENDON_WORKSPACE_H
