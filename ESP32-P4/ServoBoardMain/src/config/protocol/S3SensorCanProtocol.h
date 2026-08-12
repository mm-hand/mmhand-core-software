/**
 * @file S3SensorCanProtocol.h
 * @brief 定义 ESP32-P4 与 ESP32-S3 之间 CAN 传感器协议的稳定帧布局。
 * @details 本文件只描述 CAN ID、分帧长度和重组超时，不持有驱动对象，也不执行收包。
 *          任务层和业务层不得重复声明这些常量，协议调整必须集中在此处完成。
 */
#ifndef P4_CONFIG_PROTOCOL_SENSOR_CAN_PROTOCOL_H
#define P4_CONFIG_PROTOCOL_SENSOR_CAN_PROTOCOL_H

#include <Arduino.h>

#include "../../data/JointTypes.h"
#include "../../data/TelemetryTypes.h"

// 磁编码器：21 路 uint16 原始值按每帧 4 路拆包，最后使用独立 CRC 元数据帧确认整批有效。
constexpr uint16_t CAN_ID_ENC_BASE = 0x100;
constexpr uint16_t CAN_ID_ENC_LAST = CAN_ID_ENC_BASE + (ENCODER_TOTAL_NUM + 3) / 4 - 1;
constexpr uint16_t CAN_ID_ENC_CRC = CAN_ID_ENC_LAST + 1;
constexpr uint8_t CAN_ENC_FRAME_COUNT = (ENCODER_TOTAL_NUM + 3) / 4;
constexpr uint8_t CAN_ENC_PAYLOAD_BYTES = ENCODER_TOTAL_NUM * 2;
constexpr uint8_t CAN_ENC_CRC_DLC = 8;
constexpr uint32_t CAN_ENC_REASSEMBLY_TIMEOUT_MS = 20;

// 磁编码器错误详情：三帧覆盖 21 路，每帧格式为 [seq][7 个 error code]。
constexpr uint16_t CAN_ID_ERROR_DETAIL_BASE = 0x1F0;
constexpr uint16_t CAN_ID_ERROR_DETAIL_LAST = 0x1F2;
constexpr uint8_t CAN_ERROR_DETAIL_FRAME_COUNT = 3;
constexpr uint8_t CAN_ERROR_DETAIL_CODES_PER_FRAME = 7;
constexpr uint8_t CAN_ERROR_DETAIL_DLC = 8;
constexpr uint32_t CAN_ERROR_REASSEMBLY_TIMEOUT_MS = 100;
constexpr uint32_t CAN_ERROR_TABLE_CLEAR_TIMEOUT_MS = 300;

// 触觉摘要：45 字节摘要和 CRC16 按 [seq][7 字节数据] 拆成七帧。
constexpr uint16_t CAN_ID_TAC_SUMMARY_BASE = 0x240;
constexpr uint16_t CAN_ID_TAC_SUMMARY_LAST = 0x246;
constexpr uint8_t CAN_TAC_SUMMARY_FRAME_COUNT = 7;
constexpr uint8_t CAN_TAC_SUMMARY_PAYLOAD_PER_FRAME = 7;
constexpr uint8_t CAN_TAC_SUMMARY_DLC = 8;
constexpr uint32_t CAN_TAC_REASSEMBLY_TIMEOUT_MS = 80;
constexpr uint8_t CAN_TAC_SUMMARY_CRC_BYTES = 2;
constexpr uint16_t CAN_TAC_SUMMARY_PACKET_BYTES = TACTILE_SUMMARY_BYTES + CAN_TAC_SUMMARY_CRC_BYTES;
constexpr uint16_t CAN_ID_TAC_RAW_DIAG = 0x3F0;

// 触觉全量：不完整批次超时后整批丢弃，禁止将不同 seq 的数据拼成一帧快照。
constexpr uint16_t CAN_ID_TAC_FULL_DUMP_BASE = 0x400;
constexpr uint8_t CAN_TAC_FULL_PAYLOAD_PER_FRAME = 7;
constexpr uint8_t CAN_TAC_FULL_DLC = 8;
constexpr uint16_t CAN_TAC_FULL_FRAME_COUNT =
    (TACTILE_FULL_BYTES + CAN_TAC_FULL_PAYLOAD_PER_FRAME - 1) / CAN_TAC_FULL_PAYLOAD_PER_FRAME;
constexpr uint16_t CAN_ID_TAC_FULL_DUMP_LAST = CAN_ID_TAC_FULL_DUMP_BASE + CAN_TAC_FULL_FRAME_COUNT - 1;
constexpr uint16_t CAN_TAC_FULL_FRAME_MASK_BYTES = (CAN_TAC_FULL_FRAME_COUNT + 7) / 8;
constexpr uint32_t CAN_TAC_FULL_REASSEMBLY_TIMEOUT_MS = 400;

#endif // P4_CONFIG_PROTOCOL_SENSOR_CAN_PROTOCOL_H
