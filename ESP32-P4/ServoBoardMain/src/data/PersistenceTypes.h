/**
 * @file PersistenceTypes.h
 * @brief 定义 Manual 标定异步持久化任务的纯值请求与结果。
 * @details 数据结构不包含队列句柄、RTOS 对象或业务服务指针，可安全跨任务复制。
 */
#ifndef P4_DATA_PERSISTENCE_TYPES_H
#define P4_DATA_PERSISTENCE_TYPES_H

#include <stdint.h>

#include "../config/DimensionsConfig.h"

struct ManualNvsSaveJob {
    uint32_t requestToken;                    ///< Host 生成的单调请求号。
    uint32_t validMask;                       ///< 本次提交的有效 Jxx 位图。
    uint16_t minRaw[ENCODER_TOTAL_NUM];       ///< 21 路 Manual Min 原始值。
    uint16_t maxRaw[ENCODER_TOTAL_NUM];       ///< 21 路 Manual Max 原始值。
    uint32_t queuedAtMs;                      ///< 入队时的 P4 单调毫秒时间。
};

struct ManualNvsSaveResult {
    uint32_t requestToken;                    ///< 对应 ManualNvsSaveJob::requestToken。
    uint32_t validMask;                       ///< 读回校验通过的有效 Jxx 位图。
    uint32_t referenceCrc;                    ///< 读回快照的端点语义 CRC。
    uint32_t sequence;                        ///< NVS A/B 槽递增序号。
    uint16_t minRaw[ENCODER_TOTAL_NUM];       ///< 已确认落盘的 Manual Min。
    uint16_t maxRaw[ENCODER_TOTAL_NUM];       ///< 已确认落盘的 Manual Max。
    uint32_t completedAtMs;                   ///< Flash 事务完成时间。
    uint8_t success;                          ///< 保存并读回校验成功时为 1。
};

struct NvsControlJob {
    uint32_t requestToken;                    ///< Host 生成的单调请求号。
    uint32_t queuedAtMs;                      ///< 入队时的 P4 单调毫秒时间。
    uint8_t action;                           ///< 0=关闭并清空，1=启用。
};

struct NvsControlResult {
    uint32_t requestToken;                    ///< 对应 NvsControlJob::requestToken。
    uint32_t snapshotValidMask;               ///< 事务后读回的有效 Mxx 位图。
    int16_t snapshotRaw[SERVO_TOTAL_NUM];      ///< 事务后读回的 hardware raw 快照。
    uint32_t completedAtMs;                   ///< Flash 事务完成时间。
    uint8_t action;                           ///< 原样返回控制动作。
    uint8_t success;                          ///< 整个设置事务成功时为 1。
    uint8_t enabled;                          ///< 事务完成后的持久化开关状态。
    uint8_t snapshotValid;                    ///< 事务完成后是否仍存在有效多圈快照。
};

#endif // P4_DATA_PERSISTENCE_TYPES_H
