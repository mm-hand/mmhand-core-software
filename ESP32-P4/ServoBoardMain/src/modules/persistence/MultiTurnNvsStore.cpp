/**
 * @file MultiTurnNvsStore.cpp
 * @brief 实现 27 路舵机多圈快照的双槽 NVS 存储。
 */
#include "MultiTurnNvsStore.h"

#include "../../config/BoardConfig.h"
#include "../../config/ServoConfig.h"
#if MULTITURN_NVS_PERSISTENCE_ENABLE

#include <stddef.h>
#include <string.h>

#include "../../config/topology/HardwareMap.h"
#include "../../hal/storage/NvsBlobStore.h"
#include "../integrity/Crc32Ieee.h"
#include "../../platform/log/FirmwareLog.h"

// NVS layout:
// - namespace: mt_abs
// - enabled: persistent user switch
// - slot0 / slot1: A/B complete snapshots with sequence and CRC
static const char* kNamespace = "mt_abs";
static const char* kSlot0Key = "slot0";
static const char* kSlot1Key = "slot1";
static const char* kEnabledKey = "enabled";
static const uint32_t kMagic = 0x4D544E31UL; // MTN1
static const uint16_t kVersion = 3;
static bool ensureNvsReady()
{
    return NvsBlobStore::initialize();
}

static uint32_t snapshotCrc(const MultiTurnNvsSnapshot& snapshot)
{
    return p4::integrity::crc32IeeeUpdate(
        0,
        reinterpret_cast<const uint8_t*>(&snapshot),
        offsetof(MultiTurnNvsSnapshot, crc32));
}

static uint32_t mappingSignature()
{
    uint32_t crc = 0;
    // BUS 是运行时接线结果，不属于持久化拓扑。签名只绑定 Mxx/Jxx 的
    // 逻辑身份；舵机换到另一条 BUS 后仍可恢复同一份多圈快照。
    uint8_t byte = kHardwareMappingVersion;
    crc = p4::integrity::crc32IeeeUpdate(crc, &byte, 1);
    byte = (uint8_t)SERVO_TOTAL_NUM;
    crc = p4::integrity::crc32IeeeUpdate(crc, &byte, 1);
    byte = (uint8_t)ENCODER_TOTAL_NUM;
    crc = p4::integrity::crc32IeeeUpdate(crc, &byte, 1);

    for (uint8_t ch = 0; ch < SERVO_TOTAL_NUM; ch++) {
        const uint8_t values[2] = {
            motorMap[ch].servoID,
            (uint8_t)(getMotorTendonShortenAbsSign(ch) < 0 ? 0xFF : 0x01),
        };
        crc = p4::integrity::crc32IeeeUpdate(crc, values, sizeof(values));
    }

    for (uint8_t joint = 0; joint < ENCODER_TOTAL_NUM; joint++) {
        byte = jointMap[joint].motorChannel;
        crc = p4::integrity::crc32IeeeUpdate(crc, &byte, 1);

        const JointActuatorGroup& group = jointActuatorGroups[joint];
        byte = group.count;
        crc = p4::integrity::crc32IeeeUpdate(crc, &byte, 1);
        for (uint8_t i = 0; i < 2; i++) {
            const JointActuatorItem& item = group.actuators[i];
            const uint8_t actValues[10] = {
                item.motorChannel,
                (uint8_t)(item.sign < 0 ? 0xFF : 0x01),
                (uint8_t)(((uint32_t)item.offset >> 24) & 0xFF),
                (uint8_t)(((uint32_t)item.offset >> 16) & 0xFF),
                (uint8_t)(((uint32_t)item.offset >> 8) & 0xFF),
                (uint8_t)((uint32_t)item.offset & 0xFF),
                (uint8_t)(((uint32_t)item.tensionBias >> 24) & 0xFF),
                (uint8_t)(((uint32_t)item.tensionBias >> 16) & 0xFF),
                (uint8_t)(((uint32_t)item.tensionBias >> 8) & 0xFF),
                (uint8_t)((uint32_t)item.tensionBias & 0xFF),
            };
            crc = p4::integrity::crc32IeeeUpdate(
                crc,
                actValues,
                sizeof(actValues));
        }
    }
    return crc;
}

static bool validateSnapshot(const MultiTurnNvsSnapshot& snapshot)
{
    if (snapshot.magic != kMagic ||
        snapshot.version != kVersion ||
        snapshot.motorCount != SERVO_TOTAL_NUM) {
        return false;
    }
    if (snapshot.mappingSignature != mappingSignature()) {
        FW_LOG_INFO_PRINTF("[NVS MULTITURN] mapping signature mismatch stored=0x%08lX current=0x%08lX\r\n",
                      (unsigned long)snapshot.mappingSignature,
                      (unsigned long)mappingSignature());
        return false;
    }
    return snapshot.crc32 == snapshotCrc(snapshot);
}

// Bad slots are treated as absent so unsafe turns never enter the servo layer.
static bool loadSlot(const NvsBlobStore& store,
                     const char* key,
                     MultiTurnNvsSnapshot* outSnapshot)
{
    if (!outSnapshot) {
        return false;
    }

    MultiTurnNvsSnapshot snapshot;
    size_t len = sizeof(snapshot);
    if (!store.getBlob(key, &snapshot, &len) ||
        len != sizeof(snapshot)) {
        return false;
    }
    if (!validateSnapshot(snapshot)) {
        return false;
    }
    *outSnapshot = snapshot;
    return true;
}

// A/B 槽统一从这里提交，避免普通保存与 Reset Zero 维护两套写入/CRC 逻辑。
static bool writeSnapshot(const MultiTurnNvsSnapshot& snapshot)
{
    if (!ensureNvsReady()) {
        return false;
    }

    NvsBlobStore store;
    if (!store.open(kNamespace, NvsBlobStore::Mode::ReadWrite)) {
        return false;
    }
    const char* key = (snapshot.sequence & 1U) ? kSlot1Key : kSlot0Key;
    return store.setBlob(key, &snapshot, sizeof(snapshot)) && store.commit();
}

bool multiTurnNvsLoadEnabled(bool* outEnabled)
{
    if (!outEnabled) {
        return false;
    }
    // Default ON keeps existing behavior for boards that have never stored this switch.
    *outEnabled = true;
    if (!ensureNvsReady()) {
        return false;
    }

    NvsBlobStore store;
    if (!store.open(kNamespace, NvsBlobStore::Mode::ReadOnly)) {
        return true;
    }

    uint8_t value = 1;
    if (store.getU8(kEnabledKey, &value)) {
        *outEnabled = value != 0;
    }
    return true;
}

bool multiTurnNvsSetEnabled(bool enabled)
{
    if (!ensureNvsReady()) {
        return false;
    }

    NvsBlobStore store;
    if (!store.open(kNamespace, NvsBlobStore::Mode::ReadWrite)) {
        return false;
    }
    return store.setU8(kEnabledKey, enabled ? 1 : 0) && store.commit();
}

bool multiTurnNvsClearSnapshots()
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

bool multiTurnNvsLoadLatest(MultiTurnNvsSnapshot* outSnapshot)
{
    if (!outSnapshot || !ensureNvsReady()) {
        return false;
    }

    NvsBlobStore store;
    if (!store.open(kNamespace, NvsBlobStore::Mode::ReadOnly)) {
        return false;
    }

    MultiTurnNvsSnapshot slot0;
    MultiTurnNvsSnapshot slot1;
    const bool ok0 = loadSlot(store, kSlot0Key, &slot0);
    const bool ok1 = loadSlot(store, kSlot1Key, &slot1);

    if (!ok0 && !ok1) {
        return false;
    }
    if (ok0 && (!ok1 || slot0.sequence >= slot1.sequence)) {
        *outSnapshot = slot0;
    } else {
        *outSnapshot = slot1;
    }
    return true;
}

// ABS = turn * 4096 + raw. Raw is stored separately for boot-time validation.
int32_t multiTurnSnapshotAbsForChannel(const MultiTurnNvsSnapshot& snapshot, uint8_t ch)
{
    if (ch >= SERVO_TOTAL_NUM) {
        return 0;
    }
    return (int32_t)snapshot.turns[ch] * 4096L + (int32_t)snapshot.raw[ch];
}

bool multiTurnNvsSaveSnapshot(uint32_t validMask,
                              const int32_t* motorAbs,
                              const int16_t* motorRaw,
                              uint32_t* outSequence,
                              uint32_t* outStoredValidMask,
                              MultiTurnNvsSnapshot* outStoredSnapshot)
{
    if (!motorAbs || !motorRaw) {
        return false;
    }

    bool enabled = true;
    if (!multiTurnNvsLoadEnabled(&enabled) || !enabled) {
        return false;
    }

    uint32_t previousSequence = 0;
    bool hasPrevious = false;
    MultiTurnNvsSnapshot previous;
    if (multiTurnNvsLoadLatest(&previous)) {
        hasPrevious = true;
        previousSequence = previous.sequence;
    }

    MultiTurnNvsSnapshot snapshot;
    memset(&snapshot, 0, sizeof(snapshot));
    snapshot.magic = kMagic;
    snapshot.version = kVersion;
    snapshot.motorCount = SERVO_TOTAL_NUM;
    snapshot.sequence = previousSequence + 1;
    snapshot.validMask = validMask |
        (hasPrevious ? (previous.validMask & validMotorChannelMask()) : 0);
    snapshot.mappingSignature = mappingSignature();
    if (outStoredValidMask) {
        *outStoredValidMask = snapshot.validMask;
    }

    // Only update trusted motors; reuse old values for channels not sampled this cycle.
    for (uint8_t ch = 0; ch < SERVO_TOTAL_NUM; ch++) {
        const uint32_t bit = (1UL << ch);
        if ((validMask & bit) != 0) {
            const int16_t raw = (int16_t)(((int32_t)motorRaw[ch] % 4096 + 4096) % 4096);
            snapshot.raw[ch] = raw;
            snapshot.turns[ch] = (int16_t)((motorAbs[ch] - raw) / 4096);
        } else if (hasPrevious && (previous.validMask & bit) != 0) {
            snapshot.raw[ch] = previous.raw[ch];
            snapshot.turns[ch] = previous.turns[ch];
        }
    }
    snapshot.crc32 = snapshotCrc(snapshot);

    const bool ok = writeSnapshot(snapshot);
    if (ok && outSequence) {
        *outSequence = snapshot.sequence;
    }
    if (ok && outStoredSnapshot) {
        *outStoredSnapshot = snapshot;
    }
    return ok;
}

bool multiTurnNvsReplaceChannelsWithTurnZero(
    uint32_t resetMask,
    const int16_t* motorRaw,
    uint32_t* outSequence,
    uint32_t* outStoredValidMask,
    MultiTurnNvsSnapshot* outStoredSnapshot)
{
    resetMask &= validMotorChannelMask();
    if (resetMask == 0 || !motorRaw) {
        return false;
    }

    bool enabled = true;
    if (!multiTurnNvsLoadEnabled(&enabled) || !enabled) {
        return false;
    }

    MultiTurnNvsSnapshot snapshot;
    memset(&snapshot, 0, sizeof(snapshot));
    MultiTurnNvsSnapshot previous;
    const bool hasPrevious = multiTurnNvsLoadLatest(&previous);
    if (hasPrevious) {
        snapshot = previous;
    } else {
        snapshot.magic = kMagic;
        snapshot.version = kVersion;
        snapshot.motorCount = SERVO_TOTAL_NUM;
        snapshot.mappingSignature = mappingSignature();
    }

    snapshot.sequence = hasPrevious ? previous.sequence + 1U : 1U;
    snapshot.validMask =
        (hasPrevious ? previous.validMask : 0U) | resetMask;
    for (uint8_t ch = 0; ch < SERVO_TOTAL_NUM; ch++) {
        const uint32_t bit = 1UL << ch;
        if ((resetMask & bit) == 0) {
            continue;
        }
        // RESET 只清圈数，不改变机械单圈位置；必须保存复位后重新读取的 raw。
        const int16_t raw =
            (int16_t)(((int32_t)motorRaw[ch] % 4096 + 4096) % 4096);
        snapshot.turns[ch] = 0;
        snapshot.raw[ch] = raw;
    }
    snapshot.crc32 = snapshotCrc(snapshot);

    const bool ok = writeSnapshot(snapshot);
    if (!ok) {
        return false;
    }
    if (outSequence) {
        *outSequence = snapshot.sequence;
    }
    if (outStoredValidMask) {
        *outStoredValidMask = snapshot.validMask;
    }
    if (outStoredSnapshot) {
        *outStoredSnapshot = snapshot;
    }
    return true;
}

#endif // MULTITURN_NVS_PERSISTENCE_ENABLE
