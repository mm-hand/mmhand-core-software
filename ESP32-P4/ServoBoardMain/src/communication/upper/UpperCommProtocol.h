/**
 * @file UpperCommProtocol.h
 * @brief 定义当前上位机协议命令编号、状态位和固定包布局。
 */
#ifndef UPPER_COMM_PROTOCOL_H
#define UPPER_COMM_PROTOCOL_H

#include <Arduino.h>
#include "../../shared/TaskSharedData.h"

// 串口上行帧统一为 [0xFE][LEN][TYPE][PAYLOAD][0xFF]。
// TYPE、CMD 和字段顺序必须与 desktop/core/protocol.py 同步修改。
#define PROTOCOL_HEADER 0xFE
#define PROTOCOL_TAIL 0xFF

// P4 -> desktop 实时数据包类型。
#define PACKET_TYPE_SENSOR 0x01             // 21 关节映射角。
#define PACKET_TYPE_CALIB_ACK 0x02          // 标定状态变化 ACK。
#define PACKET_TYPE_SERVO_ANGLE 0x03        // 27 路 logical ABS。
#define PACKET_TYPE_JOINT1_DEBUG 0x04       // 单帧关节 PID/LUT 诊断。
#define PACKET_TYPE_SERVO_TELEM 0x05        // load/speed/current/voltage/temperature。
#define PACKET_TYPE_PROTO_ACK 0x06          // 下行命令接收 ACK。
#define PACKET_TYPE_FAULT_STATUS 0x07       // 故障、在线和保护位图。
#define PACKET_TYPE_SERVO_RAW 0x09          // 27 路单圈 raw。
#define PACKET_TYPE_TACTILE 0x0A            // 五指触觉摘要。
#define PACKET_TYPE_ENCODER_RAW 0x0B        // 21 路磁编 raw。
#define PACKET_TYPE_SYSTEM_STATUS 0x0C      // 状态机、owner、active scope。
#define PACKET_TYPE_MOTOR_ABS_LIMIT 0x0D    // 标定 ABS 边界和 LOCK 位。
#define PACKET_TYPE_JOINT_CALIB_SNAPSHOT 0x0E // 单关节 Auto/LUT 快照。
#define PACKET_TYPE_TACTILE_FULL 0x0F       // 可选全量触觉阵列。
#define PACKET_TYPE_CALIB_DIAG 0x10         // 结构化标定失败/警告。
#define PACKET_TYPE_TASK_DIAG 0x11          // FreeRTOS 周期、栈和 CPU 诊断。
#define PACKET_TYPE_MANUAL_ENCODER_REFERENCE 0x12 // 21 路 Manual Min/Max。
// 27 路舵机当前上电会话的动态逻辑范围，按 motor channel 独立上报。
#define PACKET_TYPE_SERVO_LOGICAL_RANGE 0x13

// int16 实时字段的断开哨兵，不能当成真实角度或 raw。
#define PROTOCOL_DISCONNECT_SENTINEL ((int16_t)0x7FFF)

// 当前上行协议的实时包统一在业务负载末尾追加 uint32_t P4 毫秒时间戳。
// 该值来自 millis() 或对应数据快照的 timestamp，按大端序传输。
static const size_t kP4TimestampBytes = sizeof(uint32_t);

// desktop -> P4 控制命令。命令 payload 长度见本文件末尾固定长度表。
#define CMD_CALIBRATE 0xCA           // 启动 active scope 自动标定。
#define CMD_ANGLE_CTRL 0xCB          // 21 路完整目标角。
#define CMD_START 0xCC               // 状态机 START/Enable。
#define CMD_STOP 0xCD                // E-STOP，立即请求断扭矩。
#define CMD_RESET 0xCE               // Clear Fault/运行状态复位。

#define CMD_MOTOR_POS 0xD0           // 27 路 uint16 兼容直控位置。
#define CMD_MOTOR_POS_SWEEP 0xD2     // 27 路 sweep 直控。
#define CMD_MOTOR_POS_ABS 0xD3       // 27 路 int32 logical ABS 直控。
#define CMD_MOTOR_POS_ABS_ONE 0xDE   // 单路 int32 logical ABS。
#define CMD_ACTIVE_SCOPE 0xD7        // active motor/joint/require CAN。
#define CMD_RUN_FEEDBACK 0xD8        // 请求进入/刷新负反馈。
#define CMD_PAUSE 0xD9               // 暂停并保存可恢复状态。
#define CMD_RESTORE_MULTITURN 0xDA   // New Device 多圈恢复候选。
#define CMD_MOTOR_ABS_LIMIT_LOCK 0xDB // 逐路启停标定 ABS 硬钳位。
// 上位机心跳中断时，多圈任务可按当前状态请求一次 Flash 备份。
#define CMD_HOST_HEARTBEAT 0xDC       // 上位机在线心跳。
#define CMD_MULTITURN_NVS_CONTROL 0xDD // Flash On/Off/查询/保存。
#define CMD_CALIB_LOAD_JOINT 0xDF     // 恢复一关节当前快照。
#define CMD_CALIB_LOAD_LIMITS 0xE0    // 恢复 27 路 ABS limits。

#define CMD_CLEAR_CALIBRATION 0xE5    // 清空全部 Auto/LUT。
#define CMD_MANUAL_ENCODER_REFERENCE 0xE6 // 写入 21 路 Manual。
#define CMD_MANUAL_ENCODER_QUERY 0xE7 // 请求立即上报 Manual。
#define CMD_MANUAL_ENCODER_SINGLE 0xE8 // 写入单路 Manual。
#define CMD_ROPE_INIT 0xE9            // P4 自主收绳进入 READY。
#define CMD_ANGLE_CTRL_MASKED 0xEA    // 稀疏 Jxx mask+21 浮点目标。
// 按发生多圈重建的电机掩码，只失效其所属关节的 Auto/LUT/ABS Limit。
#define CMD_CLEAR_CALIBRATION_SCOPE 0xEB
// 将 hardware ABS 严格越过 Reset Zero 阈值 ±30715 的指定舵机重置为 turn0。
#define CMD_SERVO_RESET_ZERO 0xEC

// 通用协议 ACK 状态；ACK 只说明命令接收结果，不代表机械动作已经完成。
#define PROTO_ACK_STATUS_OK 0
#define PROTO_ACK_STATUS_UNSUPPORTED_MODE 1
#define PROTO_ACK_STATUS_QUEUED 2
#define PROTO_ACK_STATUS_MANUAL_REFERENCE_REQUIRED 3
#define PROTO_ACK_STATUS_INVALID_DATA 4
#define PROTO_ACK_STATUS_NVS_FAILURE 5
#define PROTO_ACK_STATUS_MANUAL_CRC_MISMATCH 6
#define PROTO_ACK_STATUS_CALIBRATION_REQUIRED 7

// Manual NVS 最近一次写入状态。
#define MANUAL_SAVE_STATUS_NONE 0
#define MANUAL_SAVE_STATUS_OK 1
#define MANUAL_SAVE_STATUS_FAILED 2

// 下行命令固定 payload 长度。UpperCommCodec 按 CMD 查询这些值完成分帧。
static const size_t kFloatPayloadBytes = ENCODER_TOTAL_NUM * sizeof(float);
static const size_t kMaskedFloatPayloadBytes = sizeof(uint32_t) + kFloatPayloadBytes;
static const size_t kMotorPosPayloadBytes = SERVO_TOTAL_NUM * sizeof(uint16_t);
static const size_t kMotorPosAbsPayloadBytes = SERVO_TOTAL_NUM * sizeof(int32_t);
static const size_t kMotorPosAbsOnePayloadBytes = 1 + sizeof(int32_t);
static const size_t kActiveScopePayloadBytes = 9;
static const size_t kRestoreMultiTurnPayloadBytes =
    4 + SERVO_TOTAL_NUM * sizeof(int32_t);
static const size_t kMotorAbsLimitLockPayloadBytes = 5;
static const size_t kMultiTurnNvsControlPayloadBytes = 1;
// 当前唯一支持的 Calib Load 关节格式：关键 logical ABS 使用 int32，
// 每路 LUT 使用“int32 锚点 + 24 个 int16 相对量”。旧版长度不再接受。
static const size_t kCalibLoadJointPayloadBytes = 209;
static const size_t kCalibLoadLimitsPayloadBytes =
    3 * sizeof(uint32_t) + SERVO_TOTAL_NUM * 2 * sizeof(int32_t);
static const size_t kManualEncoderReferencePayloadBytes =
    1 + ENCODER_TOTAL_NUM * 2 * sizeof(uint16_t);
static const size_t kManualEncoderSinglePayloadBytes = 5;
static const size_t kRopeInitPayloadBytes = sizeof(uint32_t);
static const size_t kClearCalibrationScopePayloadBytes = sizeof(uint32_t);
static const size_t kServoResetZeroPayloadBytes = sizeof(uint32_t);
static const size_t kSerialRxBufferSize = 512;
// 14 位磁编一圈计数，仅供协议字段校验。
static const int32_t kEncoderModulo = 16384;
// 周期上行包的最大发送间隔；事件变化时允许提前发送。
static const uint32_t kFaultStatusHeartbeatMs = 200;
static const uint32_t kSystemStatusHeartbeatMs = 100;
static const uint32_t kTaskDiagHeartbeatMs = 250;
static const uint32_t kMotorAbsLimitHeartbeatMs = 500;
static const uint32_t kServoLogicalRangeHeartbeatMs = 500;
static const uint32_t kJointCalibSnapshotHeartbeatMs = 1000;
static const uint32_t kManualEncoderReferenceHeartbeatMs = 1000;

#endif // UPPER_COMM_PROTOCOL_H
