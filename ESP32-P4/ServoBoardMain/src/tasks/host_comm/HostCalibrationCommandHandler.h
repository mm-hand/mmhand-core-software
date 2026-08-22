/**
 * @file HostCalibrationCommandHandler.h
 * @brief 声明上位机维护命令涉及的 Manual、Auto 和 Reset Zero 联动事务。
 * @details
 * - 输入：协议命令、Manual 参考表或 Reset Zero 结果 token。
 * - 输出：更新 P4 RAM 标定状态，并按 HardwareMap 精确失效受影响关节。
 * - 状态所有权：Manual RAM 提交由本服务串行处理；舵机硬件复位由 Servo 服务执行。
 * - 禁止依赖：本模块不读取串口帧、不组装上行包、不直接操作飞特 SDK。
 */
#ifndef P4_MODULES_COMMUNICATION_CALIBRATION_MAINTENANCE_SERVICE_H
#define P4_MODULES_COMMUNICATION_CALIBRATION_MAINTENANCE_SERVICE_H

#include <stdint.h>

#include "../../modules/calibration/CalibrationRuntimeStore.h"

struct SystemRuntimeContext;

/** 返回当前命令是否要求 active scope 已具备 Manual 参考。 */
bool commandRequiresManualReference(uint8_t command);

/** 返回 Manual 写入期间是否仍存在运动 owner 或非静止状态。 */
bool manualUpdateMotionActive(SystemRuntimeContext* sharedData);

/** 计算任意 Mxx 位图影响到的完整 Jxx 位图，双腱任一电机均失效整关节。 */
uint32_t jointMaskForMotorMask(uint32_t motorMask);

/** 清除指定 Jxx 的 Auto/LUT/ABS Limit，其他关节标定结果保持不变。 */
void invalidateAutomaticCalibration(SystemRuntimeContext* sharedData,
                                    uint32_t jointMask);

/** 提交 Manual 到 P4 RAM；跨重启副本由桌面 JSON 负责。 */
uint8_t commitManualReferences(
    SystemRuntimeContext* sharedData,
    const ManualEncoderReference references[ENCODER_TOTAL_NUM]);

/** 消费 Flash 回退分支的 Manual 结果；默认关闭时固定返回 false。 */

/** 消费一次 Reset Zero 完成结果，并联动失效对应关节标定。 */
void processServoResetZeroResult(SystemRuntimeContext* sharedData,
                                 uint32_t* lastResultToken);

/** 返回 Manual、Auto、URDF 和单关节 CRC 全部匹配的 Jxx 位图。 */
uint32_t autoCalibrationValidMask(SystemRuntimeContext* sharedData);

#endif // P4_MODULES_COMMUNICATION_CALIBRATION_MAINTENANCE_SERVICE_H
