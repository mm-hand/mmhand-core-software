#ifndef MULTI_TURN_NVS_STORE_H
#define MULTI_TURN_NVS_STORE_H

#include <Arduino.h>

#include "../shared/TaskSharedData.h"

// Lower-side multi-turn safety snapshot. It stores turn + raw rather than only
// ABS, so boot restore can verify the physical single-turn raw before trusting it.
struct MultiTurnNvsSnapshot {
    uint32_t magic;
    uint16_t version;
    uint16_t motorCount;
    uint32_t sequence;
    uint32_t validMask;
    uint32_t mappingSignature;
    int16_t turns[SERVO_TOTAL_NUM];
    int16_t raw[SERVO_TOTAL_NUM];
    uint32_t crc32;
};

// Load the newest valid A/B slot. Bad CRC/version slots are ignored.
bool multiTurnNvsLoadLatest(MultiTurnNvsSnapshot* outSnapshot);
// Save trusted motors. Motors not present in this cycle keep their old values
// so a local active scope does not erase unrelated channels.
bool multiTurnNvsSaveSnapshot(uint32_t validMask,
                              const int32_t* motorAbs,
                              const int16_t* motorRaw,
                              uint32_t* outSequence,
                              uint32_t* outStoredValidMask = NULL,
                              MultiTurnNvsSnapshot* outStoredSnapshot = NULL);
// 将指定通道原子替换为 reset 后的 turn0/raw，保留其余通道和 A/B 冗余。
// 这会删除旧圈数，同时允许下一次上电从新的 turn0 快照恢复。
bool multiTurnNvsReplaceChannelsWithTurnZero(
    uint32_t resetMask,
    const int16_t* motorRaw,
    uint32_t* outSequence,
    uint32_t* outStoredValidMask = NULL,
    MultiTurnNvsSnapshot* outStoredSnapshot = NULL);
// Persistent user switch. Disabled means do not boot-restore and do not save.
bool multiTurnNvsLoadEnabled(bool* outEnabled);
bool multiTurnNvsSetEnabled(bool enabled);
bool multiTurnNvsClearSnapshots();
bool multiTurnNvsHasSnapshot();
// Rebuild logical ABS from stored turn + raw.
int32_t multiTurnSnapshotAbsForChannel(const MultiTurnNvsSnapshot& snapshot, uint8_t ch);

#endif // MULTI_TURN_NVS_STORE_H
