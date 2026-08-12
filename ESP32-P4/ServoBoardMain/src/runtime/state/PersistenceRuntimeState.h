/**
 * @file PersistenceRuntimeState.h
 * @brief 保存多圈与 Manual 参考的持久化事务状态。
 * @details Flash 写入由 NvsPersistenceTask 执行；控制周期只递增请求 token，不直接写 NVS。
 */
#ifndef P4_SYSTEM_CONTEXT_PERSISTENCE_RUNTIME_STATE_H
#define P4_SYSTEM_CONTEXT_PERSISTENCE_RUNTIME_STATE_H

#include "../../config/Config.h"
#include <stdint.h>

struct PersistenceRuntimeState {
    volatile uint32_t multiturn_nvs_save_request_token; ///< Flash 保存请求代际。
    volatile uint8_t multiturn_nvs_save_reason; ///< MULTITURN_NVS_SAVE_REASON_*。
    volatile uint32_t multiturn_nvs_save_result_token; ///< Flash 保存完成代际。
    volatile uint32_t multiturn_nvs_save_valid_mask; ///< 最近成功写入 Mxx。
    volatile uint32_t multiturn_nvs_save_fail_count; ///< 累计写失败次数。
    volatile uint8_t multiturn_nvs_enabled; ///< 用户 Flash On/Off。
    volatile uint8_t multiturn_nvs_snapshot_valid; ///< 启动时是否有有效快照。
    volatile uint32_t nvs_control_request_token; ///< 多圈 NVS 开关/清空请求代次。
    int16_t multiturn_nvs_snapshot_raw[SERVO_TOTAL_NUM]; ///< Flash raw 比较基准。
    volatile uint32_t host_rx_heartbeat_ms; ///< 最近上位机有效帧 millis()。
    volatile uint32_t manual_reference_valid_bitmap; ///< Manual 有效 Jxx。
    volatile uint32_t manual_reference_save_request_token; ///< Manual 异步保存请求代次。
    volatile uint32_t manual_reference_crc; ///< 21 路 Manual RAM 整体 CRC。
    volatile uint32_t manual_reference_nvs_sequence; ///< 当前 NVS A/B 序号。
    volatile uint8_t manual_reference_nvs_valid; ///< 是否恢复有效 Manual 快照。
    volatile uint8_t manual_reference_save_status; ///< MANUAL_SAVE_STATUS_*。
};

#endif // P4_SYSTEM_CONTEXT_PERSISTENCE_RUNTIME_STATE_H
