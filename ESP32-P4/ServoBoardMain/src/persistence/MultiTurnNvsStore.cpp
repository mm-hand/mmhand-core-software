#include "MultiTurnNvsStore.h"

#if MULTITURN_NVS_PERSISTENCE_ENABLE

#include <stddef.h>
#include <string.h>

#include <esp_err.h>
#include <nvs.h>
#include <nvs_flash.h>

#include "../hardware/HardwareMap.h"
#include "../system/FirmwareLog.h"

// NVS layout:
// - namespace: mt_abs
// - enabled: persistent user switch
// - slot0 / slot1: A/B complete snapshots with sequence and CRC
static const char* kNamespace = "mt_abs";
static const char* kSlot0Key = "slot0";
static const char* kSlot1Key = "slot1";
static const char* kEnabledKey = "enabled";
static const uint32_t kMagic = 0x4D544E31UL; // MTN1
static const uint16_t kVersion = 2;
static bool g_nvsReady = false;

static bool ensureNvsReady()
{
    if (g_nvsReady) {
        return true;
    }

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        err = nvs_flash_erase();
        if (err == ESP_OK) {
            err = nvs_flash_init();
        }
    }
    g_nvsReady = (err == ESP_OK);
    return g_nvsReady;
}

static uint32_t crc32Update(uint32_t crc, const uint8_t* data, size_t len)
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

static uint32_t snapshotCrc(const MultiTurnNvsSnapshot& snapshot)
{
    return crc32Update(0, (const uint8_t*)&snapshot, offsetof(MultiTurnNvsSnapshot, crc32));
}

static uint32_t mappingSignature()
{
    uint32_t crc = 0;
    uint8_t byte = (uint8_t)HAND_PROFILE;
    crc = crc32Update(crc, &byte, 1);
    byte = (uint8_t)SERVO_TOTAL_NUM;
    crc = crc32Update(crc, &byte, 1);
    byte = (uint8_t)ENCODER_TOTAL_NUM;
    crc = crc32Update(crc, &byte, 1);

    for (uint8_t ch = 0; ch < SERVO_TOTAL_NUM; ch++) {
        const uint8_t values[3] = {
            motorMap[ch].busIndex,
            motorMap[ch].servoID,
            (uint8_t)(getMotorTendonShortenAbsSign(ch) < 0 ? 0xFF : 0x01),
        };
        crc = crc32Update(crc, values, sizeof(values));
    }

    for (uint8_t joint = 0; joint < ENCODER_TOTAL_NUM; joint++) {
        uint8_t values[2] = {
            jointMap[joint].busIndex,
            jointMap[joint].servoID,
        };
        crc = crc32Update(crc, values, sizeof(values));

        const JointActuatorGroup& group = jointActuatorGroups[joint];
        byte = group.count;
        crc = crc32Update(crc, &byte, 1);
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
            crc = crc32Update(crc, actValues, sizeof(actValues));
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
static bool loadSlot(nvs_handle_t handle, const char* key, MultiTurnNvsSnapshot* outSnapshot)
{
    if (!outSnapshot) {
        return false;
    }

    MultiTurnNvsSnapshot snapshot;
    size_t len = sizeof(snapshot);
    if (nvs_get_blob(handle, key, &snapshot, &len) != ESP_OK ||
        len != sizeof(snapshot)) {
        return false;
    }
    if (!validateSnapshot(snapshot)) {
        return false;
    }
    *outSnapshot = snapshot;
    return true;
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

    nvs_handle_t handle = 0;
    if (nvs_open(kNamespace, NVS_READONLY, &handle) != ESP_OK) {
        return true;
    }

    uint8_t value = 1;
    const esp_err_t err = nvs_get_u8(handle, kEnabledKey, &value);
    nvs_close(handle);
    if (err == ESP_OK) {
        *outEnabled = value != 0;
    }
    return true;
}

bool multiTurnNvsSetEnabled(bool enabled)
{
    if (!ensureNvsReady()) {
        return false;
    }

    nvs_handle_t handle = 0;
    if (nvs_open(kNamespace, NVS_READWRITE, &handle) != ESP_OK) {
        return false;
    }

    esp_err_t err = nvs_set_u8(handle, kEnabledKey, enabled ? 1 : 0);
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }
    nvs_close(handle);
    return err == ESP_OK;
}

bool multiTurnNvsClearSnapshots()
{
    if (!ensureNvsReady()) {
        return false;
    }

    nvs_handle_t handle = 0;
    if (nvs_open(kNamespace, NVS_READWRITE, &handle) != ESP_OK) {
        return false;
    }

    esp_err_t err0 = nvs_erase_key(handle, kSlot0Key);
    esp_err_t err1 = nvs_erase_key(handle, kSlot1Key);
    const bool ok0 = (err0 == ESP_OK || err0 == ESP_ERR_NVS_NOT_FOUND);
    const bool ok1 = (err1 == ESP_OK || err1 == ESP_ERR_NVS_NOT_FOUND);
    esp_err_t commitErr = ESP_OK;
    if (ok0 && ok1) {
        commitErr = nvs_commit(handle);
    }
    nvs_close(handle);
    return ok0 && ok1 && commitErr == ESP_OK;
}

bool multiTurnNvsLoadLatest(MultiTurnNvsSnapshot* outSnapshot)
{
    if (!outSnapshot || !ensureNvsReady()) {
        return false;
    }

    nvs_handle_t handle = 0;
    if (nvs_open(kNamespace, NVS_READONLY, &handle) != ESP_OK) {
        return false;
    }

    MultiTurnNvsSnapshot slot0;
    MultiTurnNvsSnapshot slot1;
    const bool ok0 = loadSlot(handle, kSlot0Key, &slot0);
    const bool ok1 = loadSlot(handle, kSlot1Key, &slot1);
    nvs_close(handle);

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

bool multiTurnNvsHasSnapshot()
{
    MultiTurnNvsSnapshot snapshot;
    return multiTurnNvsLoadLatest(&snapshot);
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

    if (!ensureNvsReady()) {
        return false;
    }

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

    const bool ok = (err == ESP_OK);
    if (ok && outSequence) {
        *outSequence = snapshot.sequence;
    }
    if (ok && outStoredSnapshot) {
        *outStoredSnapshot = snapshot;
    }
    return ok;
}

#endif // MULTITURN_NVS_PERSISTENCE_ENABLE
