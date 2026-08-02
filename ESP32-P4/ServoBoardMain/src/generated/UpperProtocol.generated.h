/**
 * @file UpperProtocol.generated.h
 * @brief 由 upper_protocol_schema.json 自动生成的 P4 协议常量和长度表。
 * @warning 禁止手工编辑；请运行 python tools/generate_upper_protocol.py。
 */
#ifndef MMHAND_UPPER_PROTOCOL_GENERATED_H
#define MMHAND_UPPER_PROTOCOL_GENERATED_H

#include <stddef.h>
#include <stdint.h>
#include "../config/DimensionsConfig.h"

inline constexpr uint8_t PROTOCOL_HEADER = 0xFE;
inline constexpr uint8_t PROTOCOL_TAIL = 0xFF;
inline constexpr int16_t PROTOCOL_DISCONNECT_SENTINEL = 32767;
inline constexpr size_t kP4TimestampBytes = 4;

static_assert(ENCODER_TOTAL_NUM == 21, "Protocol joint count drift");
static_assert(SERVO_TOTAL_NUM == 27, "Protocol motor count drift");

// P4 -> Desktop 包类型。
inline constexpr uint8_t PACKET_TYPE_SENSOR = 0x01; ///< 21 关节映射角
inline constexpr uint8_t PACKET_TYPE_CALIB_ACK = 0x02; ///< 标定状态变化 ACK
inline constexpr uint8_t PACKET_TYPE_SERVO_ANGLE = 0x03; ///< 27 路 logical ABS
inline constexpr uint8_t PACKET_TYPE_JOINT1_DEBUG = 0x04; ///< 关节 PID/LUT 诊断
inline constexpr uint8_t PACKET_TYPE_SERVO_TELEM = 0x05; ///< 舵机遥测
inline constexpr uint8_t PACKET_TYPE_PROTO_ACK = 0x06; ///< 命令 ACK
inline constexpr uint8_t PACKET_TYPE_FAULT_STATUS = 0x07; ///< 故障与在线位图
inline constexpr uint8_t PACKET_TYPE_SERVO_RAW = 0x09; ///< 27 路单圈 raw
inline constexpr uint8_t PACKET_TYPE_TACTILE = 0x0A; ///< 五指触觉摘要
inline constexpr uint8_t PACKET_TYPE_ENCODER_RAW = 0x0B; ///< 21 路磁编 raw
inline constexpr uint8_t PACKET_TYPE_SYSTEM_STATUS = 0x0C; ///< 状态机与 owner
inline constexpr uint8_t PACKET_TYPE_MOTOR_ABS_LIMIT = 0x0D; ///< 标定 ABS 边界
inline constexpr uint8_t PACKET_TYPE_JOINT_CALIB_SNAPSHOT = 0x0E; ///< Auto/LUT 快照
inline constexpr uint8_t PACKET_TYPE_TACTILE_FULL = 0x0F; ///< 全量触觉阵列
inline constexpr uint8_t PACKET_TYPE_CALIB_DIAG = 0x10; ///< 结构化标定诊断
inline constexpr uint8_t PACKET_TYPE_TASK_DIAG = 0x11; ///< RTOS 任务诊断
inline constexpr uint8_t PACKET_TYPE_MANUAL_ENCODER_REFERENCE = 0x12; ///< 21 路 Manual Min/Max
inline constexpr uint8_t PACKET_TYPE_SERVO_LOGICAL_RANGE = 0x13; ///< 27 路会话 logical range
inline constexpr uint8_t PACKET_TYPE_PRODUCT_FINGERPRINT = 0x14; ///< 产品配置指纹

// Desktop -> P4 命令及 ACK 状态。
inline constexpr uint8_t CMD_CALIBRATE = 0xCA; ///< 按当前 active scope 启动自动标定
inline constexpr uint8_t CMD_ANGLE_CTRL = 0xCB; ///< 下发完整 21 路关节目标角，单位 degree
inline constexpr uint8_t CMD_START = 0xCC; ///< 历史启动命令，进入控制申请流程
inline constexpr uint8_t CMD_STOP = 0xCD; ///< 停止当前运行并撤销待执行目标
inline constexpr uint8_t CMD_RESET = 0xCE; ///< 清除允许恢复的系统故障锁存
inline constexpr uint8_t CMD_MOTOR_POS = 0xD0; ///< 27 路单圈 raw 直控目标，每路 uint16
inline constexpr uint8_t CMD_MOTOR_POS_SWEEP = 0xD2; ///< 27 路 Motor Channel 滑条连续直控目标
inline constexpr uint8_t CMD_MOTOR_POS_ABS = 0xD3; ///< 27 路 logical ABS 直控目标，每路 int32 counts
inline constexpr uint8_t CMD_ACTIVE_SCOPE = 0xD7; ///< 同步 active joint/motor mask 与 control enable
inline constexpr uint8_t CMD_RUN_FEEDBACK = 0xD8; ///< 申请状态机进入负反馈运行
inline constexpr uint8_t CMD_PAUSE = 0xD9; ///< 暂停当前负反馈或直控运行
inline constexpr uint8_t CMD_RESTORE_MULTITURN = 0xDA; ///< 按位图恢复 27 路 logical ABS 多圈快照
inline constexpr uint8_t CMD_MOTOR_ABS_LIMIT_LOCK = 0xDB; ///< 设置指定电机位图的 ABS Limit 锁定状态
inline constexpr uint8_t CMD_HOST_HEARTBEAT = 0xDC; ///< 刷新上位机在线计时器，不携带业务数据
inline constexpr uint8_t CMD_MULTITURN_NVS_CONTROL = 0xDD; ///< 启停多圈 NVS 保存；关闭时清除旧快照
inline constexpr uint8_t CMD_MOTOR_POS_ABS_ONE = 0xDE; ///< 单台 Mxx logical ABS 直控，Mxx 加 int32 counts
inline constexpr uint8_t CMD_CALIB_LOAD_JOINT = 0xDF; ///< 恢复单关节 Auto/LUT v3 标定快照
inline constexpr uint8_t CMD_CALIB_LOAD_LIMITS = 0xE0; ///< 批量恢复 27 路 ABS Limit 与有效位图
inline constexpr uint8_t CMD_CLEAR_CALIBRATION = 0xE5; ///< 清除全部 Auto/LUT 标定结果，保留 Manual
inline constexpr uint8_t CMD_MANUAL_ENCODER_REFERENCE = 0xE6; ///< 写入或清除 21 组 Manual Min/Max 原始磁编值
inline constexpr uint8_t CMD_MANUAL_ENCODER_QUERY = 0xE7; ///< 请求 P4 立即回传 Manual 标定表
inline constexpr uint8_t CMD_MANUAL_ENCODER_SINGLE = 0xE8; ///< 写入单个 Jxx 的 Manual Min/Max
inline constexpr uint8_t CMD_ROPE_INIT = 0xE9; ///< 按 21 位关节掩码启动 P4 Rope Init
inline constexpr uint8_t CMD_ANGLE_CTRL_MASKED = 0xEA; ///< 按关节掩码下发稀疏关节目标角
inline constexpr uint8_t CMD_CLEAR_CALIBRATION_SCOPE = 0xEB; ///< 按电机位图清除关联关节 Auto/LUT
inline constexpr uint8_t CMD_SERVO_RESET_ZERO = 0xEC; ///< 复位越过硬圈数阈值舵机并清除对应多圈快照
inline constexpr uint8_t PROTO_ACK_STATUS_OK = 0; ///< 命令已同步执行成功
inline constexpr uint8_t PROTO_ACK_STATUS_UNSUPPORTED_MODE = 1; ///< 当前状态或 owner 不允许该命令
inline constexpr uint8_t PROTO_ACK_STATUS_QUEUED = 2; ///< 异步请求已入队，最终结果看状态包
inline constexpr uint8_t PROTO_ACK_STATUS_MANUAL_REFERENCE_REQUIRED = 3; ///< 缺少目标关节 Manual Min/Max
inline constexpr uint8_t PROTO_ACK_STATUS_INVALID_DATA = 4; ///< 字段、长度、索引或数值范围非法
inline constexpr uint8_t PROTO_ACK_STATUS_NVS_FAILURE = 5; ///< Flash/NVS 写入或校验失败
inline constexpr uint8_t PROTO_ACK_STATUS_MANUAL_CRC_MISMATCH = 6; ///< Auto 快照绑定的 Manual CRC 不一致
inline constexpr uint8_t PROTO_ACK_STATUS_CALIBRATION_REQUIRED = 7; ///< 缺少目标关节有效 Auto/LUT
inline constexpr uint8_t MANUAL_SAVE_STATUS_NONE = 0; ///< 本次上电尚未发生 Manual 保存事务
inline constexpr uint8_t MANUAL_SAVE_STATUS_OK = 1; ///< 最近一次 Manual NVS 保存并读回校验成功
inline constexpr uint8_t MANUAL_SAVE_STATUS_FAILED = 2; ///< 最近一次 Manual NVS 保存或校验失败

// 固定 payload 长度；单位 byte，不包含命令字本身。
inline constexpr size_t kFloatPayloadBytes = 84;
inline constexpr size_t kMaskedFloatPayloadBytes = 88;
inline constexpr size_t kMotorPosPayloadBytes = 54;
inline constexpr size_t kMotorPosAbsPayloadBytes = 108;
inline constexpr size_t kMotorPosAbsOnePayloadBytes = 5;
inline constexpr size_t kActiveScopePayloadBytes = 9;
inline constexpr size_t kRestoreMultiTurnPayloadBytes = 112;
inline constexpr size_t kMotorAbsLimitLockPayloadBytes = 5;
inline constexpr size_t kMultiTurnNvsControlPayloadBytes = 1;
inline constexpr size_t kCalibLoadJointPayloadBytes = 209;
inline constexpr size_t kCalibLoadLimitsPayloadBytes = 228;
inline constexpr size_t kManualEncoderReferencePayloadBytes = 85;
inline constexpr size_t kManualEncoderSinglePayloadBytes = 5;
inline constexpr size_t kRopeInitPayloadBytes = 4;
inline constexpr size_t kClearCalibrationScopePayloadBytes = 4;
inline constexpr size_t kServoResetZeroPayloadBytes = 4;
inline constexpr size_t kSerialRxBufferSize = 512;
inline constexpr int32_t kEncoderModulo = 16384;

/** 查询固定命令 payload 长度；未知命令返回 SIZE_MAX。 */
inline constexpr size_t generatedCommandPayloadLength(uint8_t command)
{
    switch (command) {
        case CMD_CALIBRATE:
        case CMD_START:
        case CMD_STOP:
        case CMD_RESET:
        case CMD_RUN_FEEDBACK:
        case CMD_PAUSE:
        case CMD_HOST_HEARTBEAT:
        case CMD_CLEAR_CALIBRATION:
        case CMD_MANUAL_ENCODER_QUERY:
            return 0;
        case CMD_MULTITURN_NVS_CONTROL:
            return 1;
        case CMD_ROPE_INIT:
        case CMD_CLEAR_CALIBRATION_SCOPE:
        case CMD_SERVO_RESET_ZERO:
            return 4;
        case CMD_MOTOR_ABS_LIMIT_LOCK:
        case CMD_MOTOR_POS_ABS_ONE:
        case CMD_MANUAL_ENCODER_SINGLE:
            return 5;
        case CMD_ACTIVE_SCOPE:
            return 9;
        case CMD_MOTOR_POS:
        case CMD_MOTOR_POS_SWEEP:
            return 54;
        case CMD_ANGLE_CTRL:
            return 84;
        case CMD_MANUAL_ENCODER_REFERENCE:
            return 85;
        case CMD_ANGLE_CTRL_MASKED:
            return 88;
        case CMD_MOTOR_POS_ABS:
            return 108;
        case CMD_RESTORE_MULTITURN:
            return 112;
        case CMD_CALIB_LOAD_JOINT:
            return 209;
        case CMD_CALIB_LOAD_LIMITS:
            return 228;
        default: return SIZE_MAX;
    }
}

inline constexpr uint32_t kFaultStatusHeartbeatMs = 200; ///< 对应上行包无变化时的强制心跳周期，单位 ms。
inline constexpr uint32_t kSystemStatusHeartbeatMs = 100; ///< 对应上行包无变化时的强制心跳周期，单位 ms。
inline constexpr uint32_t kTaskDiagHeartbeatMs = 250; ///< 对应上行包无变化时的强制心跳周期，单位 ms。
inline constexpr uint32_t kMotorAbsLimitHeartbeatMs = 500; ///< 对应上行包无变化时的强制心跳周期，单位 ms。
inline constexpr uint32_t kServoLogicalRangeHeartbeatMs = 500; ///< 对应上行包无变化时的强制心跳周期，单位 ms。
inline constexpr uint32_t kJointCalibSnapshotHeartbeatMs = 1000; ///< 对应上行包无变化时的强制心跳周期，单位 ms。
inline constexpr uint32_t kManualEncoderReferenceHeartbeatMs = 1000; ///< 对应上行包无变化时的强制心跳周期，单位 ms。
inline constexpr uint32_t kProductFingerprintHeartbeatMs = 2000; ///< 对应上行包无变化时的强制心跳周期，单位 ms。

#endif // MMHAND_UPPER_PROTOCOL_GENERATED_H
