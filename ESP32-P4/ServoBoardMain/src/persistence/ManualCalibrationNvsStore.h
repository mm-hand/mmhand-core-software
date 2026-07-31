/**
 * @file ManualCalibrationNvsStore.h
 * @brief 定义 Manual 标定 NVS 读写、CRC 和有效位接口。
 */
#ifndef MANUAL_CALIBRATION_NVS_STORE_H
#define MANUAL_CALIBRATION_NVS_STORE_H

#include <Arduino.h>

#include "../calibration/CalibrationTask.h"

static const uint32_t kManualEncoderReferenceFullMask =
    (1UL << ENCODER_TOTAL_NUM) - 1UL; ///< 21 个磁编 Manual 端点全有效位图。

// 校验人工端点是否落在 14 位 raw 范围。允许暂存零跨度，便于 Min/Max 分两次采集；
// 真正进入运动前由 manualEncoderReferencesCoverActiveMask 再要求非零有向跨度。
bool manualEncoderReferencesValidate(
    const ManualEncoderReference* references,
    uint32_t* outValidMask = NULL);
bool manualEncoderReferencesCoverMask(
    const ManualEncoderReference* references,
    uint32_t requiredMask,
    uint32_t* outValidMask = NULL);
uint32_t manualEncoderReferencesCrc(
    const ManualEncoderReference* references);
uint32_t manualEncoderReferenceEntryCrc(
    uint8_t jointIndex,
    const ManualEncoderReference& reference);

// 启动恢复、协议写入和运动硬门禁共用的 RAM 镜像操作。
void manualEncoderReferencesClearRam();
void manualEncoderReferencesApplyRam(
    const ManualEncoderReference* references);
uint32_t manualEncoderReferencesValidMask();
bool manualEncoderReferencesComplete();
// 仅当 requiredMask 每一路都有效且有可用非零跨度时返回 true。
bool manualEncoderReferencesCoverActiveMask(uint32_t requiredMask);

// 独立 A/B NVS 快照。当前 V2 保存有效位图，允许只配置一只手指/局部 scope；
// 仍读取 V1 全量快照，避免升级固件后丢失已录入的人工端点。
bool manualCalibrationNvsLoadLatest(
    ManualEncoderReference* outReferences,
    uint32_t* outSequence = NULL,
    uint32_t* outReferenceCrc = NULL);
bool manualCalibrationNvsSave(
    const ManualEncoderReference* references,
    uint32_t* outSequence = NULL,
    uint32_t* outReferenceCrc = NULL);
bool manualCalibrationNvsClear();

#endif // MANUAL_CALIBRATION_NVS_STORE_H
