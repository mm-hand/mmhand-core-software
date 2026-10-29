/**
 * @file ManualEncoderReferenceStore.h
 * @brief 定义 P4 当前会话的 Manual Min/Max RAM 参考表及其校验接口。
 * @details
 * - 数据来源：桌面端 V6 JSON 通过上位机协议写入 P4 RAM。
 * - 生命周期：P4 重启后为空，必须由桌面端重新同步；本模块不访问任何持久化介质。
 * - 调用方：标定、READY 门禁、Pull Joint into Workspace 与上位机维护命令。
 */
#ifndef P4_MODULES_CALIBRATION_MANUAL_ENCODER_REFERENCE_STORE_H
#define P4_MODULES_CALIBRATION_MANUAL_ENCODER_REFERENCE_STORE_H

#include "CalibrationRuntimeStore.h"

static const uint32_t kManualEncoderReferenceFullMask =
    (1UL << ENCODER_TOTAL_NUM) - 1UL; ///< 21 个磁编 Manual 端点全有效位图。

bool manualEncoderReferencesValidate(
    const ManualEncoderReference* references,
    uint32_t* outValidMask = NULL);
bool manualEncoderReferencesCoverMask(
    const ManualEncoderReference* references,
    uint32_t requiredMask,
    uint32_t* outValidMask = NULL);
uint32_t manualEncoderReferencesCrc(
    const ManualEncoderReference* references);
/** 计算单关节 Manual 端点的 CRC-32/ISO-HDLC；其字节布局属于上下位机协议契约。 */
uint32_t manualEncoderReferenceEntryCrc(
    uint8_t jointIndex,
    const ManualEncoderReference& reference);

/** 清空本次 P4 会话的 Manual RAM；不会修改桌面端 JSON。 */
void manualEncoderReferencesClearRam();
/** 原子替换本次 P4 会话的完整 Manual RAM 表。 */
void manualEncoderReferencesApplyRam(
    const ManualEncoderReference* references);
uint32_t manualEncoderReferencesValidMask();
/** 仅当目标 active scope 每路都存在有效且非零有向跨度时返回 true。 */
bool manualEncoderReferencesCoverActiveMask(uint32_t requiredMask);

#endif // P4_MODULES_CALIBRATION_MANUAL_ENCODER_REFERENCE_STORE_H
