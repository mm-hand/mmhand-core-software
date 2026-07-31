/**
 * @file MultiTurnNvsStore.h
 * @brief 定义多圈快照格式、保存结果和恢复接口。
 */
#ifndef MULTI_TURN_NVS_STORE_H
#define MULTI_TURN_NVS_STORE_H

#include <Arduino.h>

#include "../shared/TaskSharedData.h"

// 下位机多圈安全快照保存 turn+raw，而不是只保存 ABS；上电时必须先比较
// 物理单圈 raw，匹配后才允许恢复 logical ABS。
struct MultiTurnNvsSnapshot {
    uint32_t magic;                  ///< 固定文件魔数。
    uint16_t version;                ///< 当前持久化布局版本。
    uint16_t motorCount;             ///< 写入时固件的舵机通道数。
    uint32_t sequence;               ///< A/B 双槽单调递增序号。
    uint32_t validMask;              ///< 本快照可信的 Mxx 位图。
    uint32_t mappingSignature;       ///< HardwareMap 拓扑签名。
    int16_t turns[SERVO_TOTAL_NUM];  ///< 每路完整圈数。
    int16_t raw[SERVO_TOTAL_NUM];    ///< 每路断电前单圈 raw。
    uint32_t crc32;                  ///< 除本字段外全部内容 CRC32。
};

// 读取 A/B 中 sequence 最新且校验通过的槽；坏 CRC/版本会被忽略。
bool multiTurnNvsLoadLatest(MultiTurnNvsSnapshot* outSnapshot);
// 保存可信通道；本轮不在 validMask 中的通道继承旧值，避免局部 scope 擦除其他关节。
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
// 用户持久化开关；关闭时既不在启动时恢复，也不写入新快照。
bool multiTurnNvsLoadEnabled(bool* outEnabled);
bool multiTurnNvsSetEnabled(bool enabled);
bool multiTurnNvsClearSnapshots();
bool multiTurnNvsHasSnapshot();
// 从快照 turn+raw 重建 P4 logical ABS。
int32_t multiTurnSnapshotAbsForChannel(const MultiTurnNvsSnapshot& snapshot, uint8_t ch);

#endif // MULTI_TURN_NVS_STORE_H
