/**
 * @file ManualCalibrationNvsStore.h
 * @brief 定义 Manual 标定 NVS 读写、CRC 和有效位接口。
 */
#ifndef MANUAL_CALIBRATION_NVS_STORE_H
#define MANUAL_CALIBRATION_NVS_STORE_H

#include <Arduino.h>

#include "../calibration/CalibrationTask.h"

static const uint32_t kManualEncoderReferenceFullMask =
    (1UL << ENCODER_TOTAL_NUM) - 1UL;

// Validate that operator-entered endpoint pairs are present and in the raw
// 14-bit range. A zero-span pair is allowed to be persisted so Min and Max can
// be captured in separate operations. Runtime motion gates separately require
// a non-zero directed span through manualEncoderReferencesCoverActiveMask().
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

// RAM helpers used by the boot path, protocol handler and hard gates.
void manualEncoderReferencesClearRam();
void manualEncoderReferencesApplyRam(
    const ManualEncoderReference* references);
uint32_t manualEncoderReferencesValidMask();
bool manualEncoderReferencesComplete();
// True only when every requested stored pair also has a usable non-zero span.
bool manualEncoderReferencesCoverActiveMask(uint32_t requiredMask);

// Independent A/B NVS snapshot. V2 persists a validity bitmap so one selected
// hand/finger scope can be commissioned without fabricating the other joints.
// Legacy V1 full snapshots remain readable.
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
