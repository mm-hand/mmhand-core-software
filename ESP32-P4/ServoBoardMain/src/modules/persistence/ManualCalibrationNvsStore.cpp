/**
 * @file ManualCalibrationNvsStore.cpp
 * @brief 实现 21 路 Manual Min/Max 的双槽 NVS 持久化。
 */
#include "ManualCalibrationNvsStore.h"

#include <stddef.h>
#include <string.h>

#include "../calibration/CalibrationMath.h"
#include "../integrity/Crc32Ieee.h"
#include "../../hal/storage/NvsBlobStore.h"
#include "../../platform/log/FirmwareLog.h"

namespace {

static const char* kNamespace = "manual_cal"; ///< NVS namespace，升级时不得随意改名。
static const char* kSlot0Key = "slot0";       ///< A 槽 key。
static const char* kSlot1Key = "slot1";       ///< B 槽 key。
static const uint32_t kMagic = 0x4D43414CUL;  ///< ASCII "MCAL"。
// 该版本号只描述 NVS 二进制布局，与 Auto 角度映射版本没有任何关系。
static const uint16_t kManualNvsSchemaVersionCurrent = 2;

struct ManualCalibrationNvsSnapshot {
    uint32_t magic;                         ///< kMagic。
    uint16_t version;                       ///< 固定为当前 NVS schema 版本。
    uint16_t jointCount;                    ///< 必须等于 ENCODER_TOTAL_NUM。
    uint32_t sequence;                      ///< A/B 槽序号。
    uint32_t validMask;                     ///< 已人工录入的 Jxx。
    uint16_t minRaw[ENCODER_TOTAL_NUM];     ///< 人工 Min raw。
    uint16_t maxRaw[ENCODER_TOTAL_NUM];     ///< 人工 Max raw。
    uint32_t referenceCrc;                  ///< 端点语义 CRC。
    uint32_t crc32;                         ///< 除本字段外结构体 CRC。
};

static_assert(sizeof(ManualCalibrationNvsSnapshot) == 108,
              "unexpected Manual calibration NVS schema 2 layout");

/** 初始化 NVS；存储页损坏或版本变化时按 ESP-IDF 建议擦除并重试。 */
bool ensureNvsReady()
{
    return NvsBlobStore::initialize();
}

/** 计算当前 NVS schema 2 快照除 crc32 字段外的完整性 CRC。 */
uint32_t snapshotCrc(const ManualCalibrationNvsSnapshot& snapshot)
{
    return p4::integrity::crc32IeeeUpdate(
        0,
        reinterpret_cast<const uint8_t*>(&snapshot),
        offsetof(ManualCalibrationNvsSnapshot, crc32));
}

/** 将持久化数组解包成运行时 ManualEncoderReference。 */
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

/** 同时校验布局、结构 CRC、有效位端点和语义 CRC。 */
bool validateSnapshot(const ManualCalibrationNvsSnapshot& snapshot)
{
    if (snapshot.magic != kMagic ||
        snapshot.version != kManualNvsSchemaVersionCurrent ||
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

/** 读取一个 A/B 槽；只接受当前布局，旧版内容视为无效并要求重新录入。 */
bool loadSlot(const NvsBlobStore& store,
              const char* key,
              ManualCalibrationNvsSnapshot* outSnapshot)
{
    if (!outSnapshot) {
        return false;
    }
    size_t len = 0;
    if (!store.getBlob(key, nullptr, &len)) {
        return false;
    }
    ManualCalibrationNvsSnapshot snapshot;
    memset(&snapshot, 0, sizeof(snapshot));
    if (len != sizeof(ManualCalibrationNvsSnapshot) ||
        !store.getBlob(key, &snapshot, &len) ||
        !validateSnapshot(snapshot)) {
        return false;
    }
    *outSnapshot = snapshot;
    return true;
}

/** 从两个有效槽中选择 sequence 最新者。 */
bool loadLatestSnapshot(ManualCalibrationNvsSnapshot* outSnapshot)
{
    if (!outSnapshot || !ensureNvsReady()) {
        return false;
    }
    NvsBlobStore store;
    if (!store.open(kNamespace, NvsBlobStore::Mode::ReadOnly)) {
        return false;
    }
    ManualCalibrationNvsSnapshot slot0;
    ManualCalibrationNvsSnapshot slot1;
    const bool ok0 = loadSlot(store, kSlot0Key, &slot0);
    const bool ok1 = loadSlot(store, kSlot1Key, &slot1);
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
        crc = p4::integrity::crc32IeeeUpdate(crc, values, sizeof(values));
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
    return p4::integrity::crc32IeeeUpdate(0, values, sizeof(values));
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
    snapshot.version = kManualNvsSchemaVersionCurrent;
    snapshot.jointCount = ENCODER_TOTAL_NUM;
    snapshot.sequence = previousSequence + 1;
    snapshot.validMask = validMask;
    for (uint8_t joint = 0; joint < ENCODER_TOTAL_NUM; joint++) {
        snapshot.minRaw[joint] = references[joint].minRaw;
        snapshot.maxRaw[joint] = references[joint].maxRaw;
    }
    snapshot.referenceCrc = manualEncoderReferencesCrc(references);
    snapshot.crc32 = snapshotCrc(snapshot);

    NvsBlobStore store;
    if (!store.open(kNamespace, NvsBlobStore::Mode::ReadWrite)) {
        return false;
    }
    const char* key = (snapshot.sequence & 1U) ? kSlot1Key : kSlot0Key;
    if (!store.setBlob(key, &snapshot, sizeof(snapshot)) || !store.commit()) {
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
    NvsBlobStore store;
    if (!store.open(kNamespace, NvsBlobStore::Mode::ReadWrite)) {
        return false;
    }
    const bool ok0 = store.eraseKey(kSlot0Key);
    const bool ok1 = store.eraseKey(kSlot1Key);
    return ok0 && ok1 && store.commit();
}
