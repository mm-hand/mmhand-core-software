/**
 * @file ManualCalibrationNvsStore.cpp
 * @brief 实现 21 路 Manual Min/Max 的双槽 NVS 持久化。
 */
#include "ManualCalibrationNvsStore.h"

#include <stddef.h>
#include <string.h>

#include <esp_err.h>
#include <nvs.h>
#include <nvs_flash.h>

#include "../calibration/CalibrationMath.h"
#include "../system/FirmwareLog.h"

namespace {

static const char* kNamespace = "manual_cal";
static const char* kSlot0Key = "slot0";
static const char* kSlot1Key = "slot1";
static const uint32_t kMagic = 0x4D43414CUL; // MCAL
static const uint16_t kVersionV1 = 1;
static const uint16_t kVersionV2 = 2;
static bool g_nvsReady = false;

struct ManualCalibrationNvsSnapshotV1 {
    uint32_t magic;
    uint16_t version;
    uint16_t jointCount;
    uint32_t sequence;
    uint16_t minRaw[ENCODER_TOTAL_NUM];
    uint16_t maxRaw[ENCODER_TOTAL_NUM];
    uint32_t referenceCrc;
    uint32_t crc32;
};

struct ManualCalibrationNvsSnapshot {
    uint32_t magic;
    uint16_t version;
    uint16_t jointCount;
    uint32_t sequence;
    uint32_t validMask;
    uint16_t minRaw[ENCODER_TOTAL_NUM];
    uint16_t maxRaw[ENCODER_TOTAL_NUM];
    uint32_t referenceCrc;
    uint32_t crc32;
};

static_assert(sizeof(ManualCalibrationNvsSnapshotV1) == 104,
              "unexpected Manual calibration NVS V1 layout");
static_assert(sizeof(ManualCalibrationNvsSnapshot) == 108,
              "unexpected Manual calibration NVS V2 layout");

bool ensureNvsReady()
{
    if (g_nvsReady) {
        return true;
    }
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES ||
        err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        err = nvs_flash_erase();
        if (err == ESP_OK) {
            err = nvs_flash_init();
        }
    }
    g_nvsReady = (err == ESP_OK);
    return g_nvsReady;
}

uint32_t crc32Update(uint32_t crc, const uint8_t* data, size_t len)
{
    crc = ~crc;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8; bit++) {
            const uint32_t mask = (uint32_t)-(int32_t)(crc & 1U);
            crc = (crc >> 1) ^ (0xEDB88320UL & mask);
        }
    }
    return ~crc;
}

uint32_t snapshotCrc(const ManualCalibrationNvsSnapshot& snapshot)
{
    return crc32Update(
        0,
        reinterpret_cast<const uint8_t*>(&snapshot),
        offsetof(ManualCalibrationNvsSnapshot, crc32));
}

void snapshotToReferences(const ManualCalibrationNvsSnapshot& snapshot,
                          ManualEncoderReference* references)
{
    for (uint8_t joint = 0; joint < ENCODER_TOTAL_NUM; joint++) {
        references[joint].valid =
            (snapshot.validMask & (1UL << joint)) != 0 ? 1 : 0;
        references[joint].minRaw = snapshot.minRaw[joint];
        references[joint].maxRaw = snapshot.maxRaw[joint];
    }
}

void snapshotV1ToV2(const ManualCalibrationNvsSnapshotV1& source,
                    ManualCalibrationNvsSnapshot* destination)
{
    memset(destination, 0, sizeof(*destination));
    destination->magic = source.magic;
    destination->version = kVersionV2;
    destination->jointCount = source.jointCount;
    destination->sequence = source.sequence;
    destination->validMask = kManualEncoderReferenceFullMask;
    memcpy(destination->minRaw, source.minRaw, sizeof(destination->minRaw));
    memcpy(destination->maxRaw, source.maxRaw, sizeof(destination->maxRaw));
    destination->referenceCrc = source.referenceCrc;
}

bool validateSnapshot(const ManualCalibrationNvsSnapshot& snapshot)
{
    if (snapshot.magic != kMagic ||
        snapshot.version != kVersionV2 ||
        snapshot.jointCount != ENCODER_TOTAL_NUM ||
        snapshot.crc32 != snapshotCrc(snapshot)) {
        return false;
    }

    ManualEncoderReference references[ENCODER_TOTAL_NUM];
    snapshotToReferences(snapshot, references);
    uint32_t validMask = 0;
    const bool coversSnapshot = manualEncoderReferencesCoverMask(
        references, snapshot.validMask, &validMask);
    return snapshot.validMask != 0 && coversSnapshot &&
           validMask == snapshot.validMask &&
           snapshot.referenceCrc == manualEncoderReferencesCrc(references);
}

bool loadSlot(nvs_handle_t handle,
              const char* key,
              ManualCalibrationNvsSnapshot* outSnapshot)
{
    if (!outSnapshot) {
        return false;
    }
    size_t len = 0;
    if (nvs_get_blob(handle, key, NULL, &len) != ESP_OK) {
        return false;
    }
    ManualCalibrationNvsSnapshot snapshot;
    memset(&snapshot, 0, sizeof(snapshot));
    if (len == sizeof(ManualCalibrationNvsSnapshot)) {
        if (nvs_get_blob(handle, key, &snapshot, &len) != ESP_OK ||
            !validateSnapshot(snapshot)) {
            return false;
        }
    } else if (len == sizeof(ManualCalibrationNvsSnapshotV1)) {
        ManualCalibrationNvsSnapshotV1 legacy;
        if (nvs_get_blob(handle, key, &legacy, &len) != ESP_OK ||
            legacy.magic != kMagic || legacy.version != kVersionV1 ||
            legacy.jointCount != ENCODER_TOTAL_NUM ||
            legacy.crc32 != crc32Update(
                0,
                reinterpret_cast<const uint8_t*>(&legacy),
                offsetof(ManualCalibrationNvsSnapshotV1, crc32))) {
            return false;
        }
        snapshotV1ToV2(legacy, &snapshot);
        ManualEncoderReference references[ENCODER_TOTAL_NUM];
        snapshotToReferences(snapshot, references);
        if (!manualEncoderReferencesValidate(references) ||
            snapshot.referenceCrc != manualEncoderReferencesCrc(references)) {
            return false;
        }
    } else {
        return false;
    }
    *outSnapshot = snapshot;
    return true;
}

bool loadLatestSnapshot(ManualCalibrationNvsSnapshot* outSnapshot)
{
    if (!outSnapshot || !ensureNvsReady()) {
        return false;
    }
    nvs_handle_t handle = 0;
    if (nvs_open(kNamespace, NVS_READONLY, &handle) != ESP_OK) {
        return false;
    }
    ManualCalibrationNvsSnapshot slot0;
    ManualCalibrationNvsSnapshot slot1;
    const bool ok0 = loadSlot(handle, kSlot0Key, &slot0);
    const bool ok1 = loadSlot(handle, kSlot1Key, &slot1);
    nvs_close(handle);
    if (!ok0 && !ok1) {
        return false;
    }
    *outSnapshot = ok0 && (!ok1 || slot0.sequence >= slot1.sequence)
        ? slot0 : slot1;
    return true;
}

} // namespace

bool manualEncoderReferencesValidate(
    const ManualEncoderReference* references,
    uint32_t* outValidMask)
{
    uint32_t validMask = 0;
    if (references) {
        for (uint8_t joint = 0; joint < ENCODER_TOTAL_NUM; joint++) {
            const ManualEncoderReference& ref = references[joint];
            if (ref.valid && ref.minRaw < kCalibrationEncoderModulo &&
                ref.maxRaw < kCalibrationEncoderModulo) {
                validMask |= (1UL << joint);
            }
        }
    }
    if (outValidMask) {
        *outValidMask = validMask;
    }
    return validMask == kManualEncoderReferenceFullMask;
}

bool manualEncoderReferencesCoverMask(
    const ManualEncoderReference* references,
    uint32_t requiredMask,
    uint32_t* outValidMask)
{
    uint32_t validMask = 0;
    manualEncoderReferencesValidate(references, &validMask);
    requiredMask &= kManualEncoderReferenceFullMask;
    if (outValidMask) {
        *outValidMask = validMask;
    }
    return requiredMask != 0 && (validMask & requiredMask) == requiredMask;
}

uint32_t manualEncoderReferencesCrc(
    const ManualEncoderReference* references)
{
    if (!references) {
        return 0;
    }
    uint32_t crc = 0;
    for (uint8_t joint = 0; joint < ENCODER_TOTAL_NUM; joint++) {
        const uint8_t values[5] = {
            joint,
            (uint8_t)(references[joint].minRaw >> 8),
            (uint8_t)(references[joint].minRaw & 0xFF),
            (uint8_t)(references[joint].maxRaw >> 8),
            (uint8_t)(references[joint].maxRaw & 0xFF),
        };
        crc = crc32Update(crc, values, sizeof(values));
    }
    return crc;
}

uint32_t manualEncoderReferenceEntryCrc(
    uint8_t jointIndex,
    const ManualEncoderReference& reference)
{
    const uint8_t values[5] = {
        jointIndex,
        (uint8_t)(reference.minRaw >> 8),
        (uint8_t)(reference.minRaw & 0xFF),
        (uint8_t)(reference.maxRaw >> 8),
        (uint8_t)(reference.maxRaw & 0xFF),
    };
    return crc32Update(0, values, sizeof(values));
}

void manualEncoderReferencesClearRam()
{
    memset(g_manualEncoderReference, 0, sizeof(g_manualEncoderReference));
}

void manualEncoderReferencesApplyRam(
    const ManualEncoderReference* references)
{
    if (references) {
        memcpy(g_manualEncoderReference,
               references,
               sizeof(g_manualEncoderReference));
    }
}

uint32_t manualEncoderReferencesValidMask()
{
    uint32_t validMask = 0;
    manualEncoderReferencesValidate(g_manualEncoderReference, &validMask);
    return validMask;
}

bool manualEncoderReferencesComplete()
{
    return manualEncoderReferencesValidate(g_manualEncoderReference);
}

bool manualEncoderReferencesCoverActiveMask(uint32_t requiredMask)
{
    requiredMask &= kManualEncoderReferenceFullMask;
    if (requiredMask == 0) {
        return false;
    }

    uint32_t usableMask = 0;
    for (uint8_t joint = 0; joint < ENCODER_TOTAL_NUM; joint++) {
        const ManualEncoderReference& ref = g_manualEncoderReference[joint];
        if (!ref.valid || ref.minRaw >= kCalibrationEncoderModulo ||
            ref.maxRaw >= kCalibrationEncoderModulo) {
            continue;
        }
        const int32_t span = calibrationDirectedEncoderDelta(
            ref.minRaw, ref.maxRaw, g_encoderDirection[joint]);
        if (span > 0 && span < kCalibrationEncoderModulo) {
            usableMask |= (1UL << joint);
        }
    }
    return (usableMask & requiredMask) == requiredMask;
}

bool manualCalibrationNvsLoadLatest(
    ManualEncoderReference* outReferences,
    uint32_t* outSequence,
    uint32_t* outReferenceCrc)
{
    if (!outReferences) {
        return false;
    }
    ManualCalibrationNvsSnapshot snapshot;
    if (!loadLatestSnapshot(&snapshot)) {
        return false;
    }
    snapshotToReferences(snapshot, outReferences);
    if (outSequence) {
        *outSequence = snapshot.sequence;
    }
    if (outReferenceCrc) {
        *outReferenceCrc = snapshot.referenceCrc;
    }
    return true;
}

bool manualCalibrationNvsSave(
    const ManualEncoderReference* references,
    uint32_t* outSequence,
    uint32_t* outReferenceCrc)
{
    uint32_t validMask = 0;
    manualEncoderReferencesValidate(references, &validMask);
    if (validMask == 0 || !ensureNvsReady()) {
        return false;
    }

    ManualCalibrationNvsSnapshot previous;
    const uint32_t previousSequence = loadLatestSnapshot(&previous)
        ? previous.sequence : 0;

    ManualCalibrationNvsSnapshot snapshot;
    memset(&snapshot, 0, sizeof(snapshot));
    snapshot.magic = kMagic;
    snapshot.version = kVersionV2;
    snapshot.jointCount = ENCODER_TOTAL_NUM;
    snapshot.sequence = previousSequence + 1;
    snapshot.validMask = validMask;
    for (uint8_t joint = 0; joint < ENCODER_TOTAL_NUM; joint++) {
        snapshot.minRaw[joint] = references[joint].minRaw;
        snapshot.maxRaw[joint] = references[joint].maxRaw;
    }
    snapshot.referenceCrc = manualEncoderReferencesCrc(references);
    snapshot.crc32 = snapshotCrc(snapshot);

    nvs_handle_t handle = 0;
    if (nvs_open(kNamespace, NVS_READWRITE, &handle) != ESP_OK) {
        return false;
    }
    const char* key = (snapshot.sequence & 1U) ? kSlot1Key : kSlot0Key;
    esp_err_t err = nvs_set_blob(handle, key, &snapshot, sizeof(snapshot));
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }
    nvs_close(handle);
    if (err != ESP_OK) {
        return false;
    }

    // Do not acknowledge persistence until the just-written slot can be read
    // back and passes both the snapshot CRC and the endpoint-reference CRC.
    ManualCalibrationNvsSnapshot verified;
    if (!loadLatestSnapshot(&verified) ||
        verified.sequence != snapshot.sequence ||
        verified.referenceCrc != snapshot.referenceCrc) {
        return false;
    }
    if (outSequence) {
        *outSequence = snapshot.sequence;
    }
    if (outReferenceCrc) {
        *outReferenceCrc = snapshot.referenceCrc;
    }
    return true;
}

bool manualCalibrationNvsClear()
{
    if (!ensureNvsReady()) {
        return false;
    }
    nvs_handle_t handle = 0;
    if (nvs_open(kNamespace, NVS_READWRITE, &handle) != ESP_OK) {
        return false;
    }
    const esp_err_t err0 = nvs_erase_key(handle, kSlot0Key);
    const esp_err_t err1 = nvs_erase_key(handle, kSlot1Key);
    const bool ok0 = err0 == ESP_OK || err0 == ESP_ERR_NVS_NOT_FOUND;
    const bool ok1 = err1 == ESP_OK || err1 == ESP_ERR_NVS_NOT_FOUND;
    esp_err_t commitErr = ESP_OK;
    if (ok0 && ok1) {
        commitErr = nvs_commit(handle);
    }
    nvs_close(handle);
    return ok0 && ok1 && commitErr == ESP_OK;
}
