/**
 * @file CalibrationDiagnostics.h
 * @brief 声明自动标定结构化诊断的唯一读写接口。
 * @details
 * - 输入：标定阶段、关节、电机和三个诊断值。
 * - 输出：更新 CalibrationRuntimeState，并向统一 Safety 事件缓存登记规则 ID。
 * - 线程：自动标定任务写入，上位机通信任务读取；内部使用共享安全锁保护快照。
 * - 禁止依赖：本模块不得发布舵机命令、修改系统状态或解释 UI 文案。
 */
#ifndef P4_MODULES_CALIBRATION_CALIBRATION_DIAGNOSTICS_H
#define P4_MODULES_CALIBRATION_CALIBRATION_DIAGNOSTICS_H

#include <stdint.h>

struct SystemRuntimeContext;

/** 原子写入一次标定失败诊断，并同步登记稳定 SafetyRuleId。 */
void setCalibrationDiag(SystemRuntimeContext* sharedData,
                        uint8_t reason,
                        uint8_t phase,
                        uint8_t joint,
                        uint8_t motor,
                        int32_t valueA,
                        int32_t valueB,
                        int32_t valueC,
                        uint32_t bitmap);

/** 仅在本轮尚无更具体根因时写入兜底诊断，防止外层覆盖内层证据。 */
void setCalibrationDiagIfEmpty(SystemRuntimeContext* sharedData,
                               uint8_t reason,
                               uint8_t phase,
                               uint8_t joint,
                               uint8_t motor,
                               int32_t valueA,
                               int32_t valueB,
                               int32_t valueC,
                               uint32_t bitmap);

/** 清空上一轮诊断，但保留单调递增 sequence，避免上位机漏判新事务。 */
void clearCalibrationDiag(SystemRuntimeContext* sharedData);

/** 返回当前快照是否包含有效诊断。 */
bool hasCalibrationDiag(SystemRuntimeContext* sharedData);

/** 返回当前诊断原因；无有效快照时返回 CALIB_DIAG_REASON_NONE。 */
uint8_t calibrationDiagReason(SystemRuntimeContext* sharedData);

/**
 * 在不覆盖具体 reason/phase/J/M/value 的前提下补充本轮结果位图。
 * 写入后递增 sequence，确保上位机收到带完整汇总的最终根因诊断。
 */
void setCalibrationDiagResultMasks(SystemRuntimeContext* sharedData,
                                   uint32_t completedJointMask,
                                   uint32_t failedJointMask);

#endif // P4_MODULES_CALIBRATION_CALIBRATION_DIAGNOSTICS_H
