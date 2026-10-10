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
inline constexpr uint8_t kHostProtocolVersion = 2;
inline constexpr uint8_t PROTOCOL_TAIL = 0xFF;
inline constexpr int16_t PROTOCOL_DISCONNECT_SENTINEL = 32767;
inline constexpr size_t kP4TimestampBytes = 4;

static_assert(ENCODER_TOTAL_NUM == 21, "Protocol joint count drift");
static_assert(SERVO_TOTAL_NUM == 27, "Protocol motor count drift");

// P4 -> Desktop 包类型。
inline constexpr uint8_t PACKET_TYPE_SENSOR = 0x01; ///< 21 关节映射角
inline constexpr uint8_t PACKET_TYPE_SERVO_ANGLE = 0x03; ///< 27 路 logical ABS
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
inline constexpr uint8_t PACKET_TYPE_SERVO_ROUTE_STATUS = 0x15; ///< 动态舵机 BUS 路由、冲突、容量和扫描状态
inline constexpr uint8_t PACKET_TYPE_SAFETY_EVENT = 0x16; ///< 去重后的结构化安全规则事件
inline constexpr uint8_t PACKET_TYPE_MULTITURN_RESTORE_RESULT = 0x17; ///< 桌面 JSON 多圈恢复异步最终结果
inline constexpr uint8_t PACKET_TYPE_CONTROL_TRACE = 0x18; ///< 同一控制周期的目标角、反馈角、舵机CMD和下位机时间
inline constexpr uint8_t PACKET_TYPE_SERVO_REBASE_ZERO_RESULT = 0x19; ///< logical ABS 设零事务的最终token、成功失败位图与27路offset
inline constexpr uint8_t PACKET_TYPE_DIRECT_TARGET_RESULT = 0x1A; ///< 通用直控目标完成总线同步写后的事务结果，与机械到位独立

// Desktop -> P4 命令及 ACK 状态。
inline constexpr uint8_t CMD_CALIBRATE = 0xCA; ///< 按当前 active scope 启动自动标定
inline constexpr uint8_t CMD_ANGLE_CTRL = 0xCB; ///< 下发完整 21 路关节目标角，单位 degree
inline constexpr uint8_t CMD_START = 0xCC; ///< 历史启动命令，进入控制申请流程
inline constexpr uint8_t CMD_STOP = 0xCD; ///< 停止当前运行并撤销待执行目标
inline constexpr uint8_t CMD_RESET = 0xCE; ///< 清除允许恢复的系统故障锁存
inline constexpr uint8_t CMD_MOTOR_POS_ABS = 0xD3; ///< 通用logical ABS直控：token、实际电机位图、27路int32 counts、速度和加速度；token=0不跟踪，位图=0用当前scope
inline constexpr uint8_t CMD_ACTIVE_SCOPE = 0xD7; ///< 同步 active joint/motor mask 与 control enable
inline constexpr uint8_t CMD_RUN_FEEDBACK = 0xD8; ///< 申请状态机进入负反馈运行
inline constexpr uint8_t CMD_PAUSE = 0xD9; ///< 暂停当前负反馈或直控运行
inline constexpr uint8_t CMD_RESTORE_MULTITURN = 0xDA; ///< 按位图恢复 27 路 logical ABS 多圈快照
inline constexpr uint8_t CMD_HOST_HEARTBEAT = 0xDC; ///< 刷新上位机在线计时器，不携带业务数据
inline constexpr uint8_t CMD_MOTOR_POS_ABS_ONE = 0xDE; ///< 单台 Mxx logical ABS 直控，Mxx 加 int32 counts
inline constexpr uint8_t CMD_CALIB_LOAD_JOINT = 0xDF; ///< 恢复单关节唯一 Auto/LUT v4 标定快照
inline constexpr uint8_t CMD_CALIB_LOAD_LIMITS = 0xE0; ///< 批量恢复 27 路 ABS 标定范围与有效位图
inline constexpr uint8_t CMD_CLEAR_CALIBRATION = 0xE5; ///< 清除全部 Auto/LUT 标定结果，保留 Manual
inline constexpr uint8_t CMD_MANUAL_ENCODER_REFERENCE = 0xE6; ///< 写入或清除 21 组 Manual Min/Max 原始磁编值
inline constexpr uint8_t CMD_MANUAL_ENCODER_QUERY = 0xE7; ///< 请求 P4 立即回传 Manual 标定表
inline constexpr uint8_t CMD_MANUAL_ENCODER_SINGLE = 0xE8; ///< 写入单个 Jxx 的 Manual Min/Max
inline constexpr uint8_t CMD_ROPE_INIT = 0xE9; ///< 按 21 位关节掩码启动 P4 Rope Init
inline constexpr uint8_t CMD_ANGLE_CTRL_MASKED = 0xEA; ///< 按关节掩码下发稀疏关节目标角
inline constexpr uint8_t CMD_CLEAR_CALIBRATION_SCOPE = 0xEB; ///< 按电机位图清除关联关节 Auto/LUT
inline constexpr uint8_t CMD_SERVO_RESET_ZERO = 0xEC; ///< 标定前归零所选舵机硬件圈数并清除对应多圈锚点
inline constexpr uint8_t CMD_STATE_SYNC_REQUEST = 0xED; ///< 按位请求静态状态、标定快照、路由和产品指纹
inline constexpr uint8_t CMD_TASK_DIAG_SUBSCRIBE = 0xEE; ///< 打开或关闭 RTOS 任务诊断按需上报
inline constexpr uint8_t CMD_CONTROL_TRACE_SUBSCRIBE = 0xEF; ///< 打开或关闭统一下位机时间的控制跟踪流
inline constexpr uint8_t CMD_SERVO_REBASE_ZERO = 0xF0; ///< 按电机位图RESET硬件圈数，将当前位置logical ABS设为0并在RAM保存offset
inline constexpr uint8_t CMD_MOTOR_DIRECT_RELEASE = 0xF2; ///< 结束当前普通直控并保留扭矩和故障；非零token匹配已执行事务，零匹配当前直控代际
inline constexpr uint8_t CMD_PROTOCOL_HELLO = 0xF3; ///< 双向版本及控制模型与反馈拓扑契约握手，不授权运动
inline constexpr uint8_t PROTO_ACK_STATUS_OK = 0; ///< 命令已同步执行成功
inline constexpr uint8_t PROTO_ACK_STATUS_UNSUPPORTED_MODE = 1; ///< 当前状态或 owner 不允许该命令
inline constexpr uint8_t PROTO_ACK_STATUS_QUEUED = 2; ///< 异步请求已入队，最终结果看状态包
inline constexpr uint8_t PROTO_ACK_STATUS_MANUAL_REFERENCE_REQUIRED = 3; ///< 缺少目标关节 Manual Min/Max
inline constexpr uint8_t PROTO_ACK_STATUS_INVALID_DATA = 4; ///< 字段、长度、索引或数值范围非法
inline constexpr uint8_t PROTO_ACK_STATUS_MANUAL_CRC_MISMATCH = 6; ///< Auto 快照绑定的 Manual CRC 不一致
inline constexpr uint8_t PROTO_ACK_STATUS_CONTRACT_MISMATCH = 8; ///< 协议版本或运动模型契约未匹配
inline constexpr uint8_t PROTO_ACK_STATUS_CALIBRATION_REQUIRED = 7; ///< 缺少目标关节有效 Auto/LUT
inline constexpr uint8_t MANUAL_SAVE_STATUS_NONE = 0; ///< 本次上电尚未收到桌面端 Manual 数据
inline constexpr uint8_t MANUAL_SAVE_STATUS_OK = 1; ///< 最近一次 Manual 已提交到 P4 RAM 并完成校验
inline constexpr uint8_t MANUAL_SAVE_STATUS_FAILED = 2; ///< 最近一次 Manual RAM 提交或校验失败
inline constexpr uint8_t STATE_SYNC_MANUAL_REFERENCE = 1; ///< 同步 21 路 Manual Min/Max
inline constexpr uint8_t STATE_SYNC_MOTOR_ABS_LIMIT = 2; ///< 同步 27 路 ABS Limit
inline constexpr uint8_t STATE_SYNC_SERVO_LOGICAL_RANGE = 4; ///< 同步 27 路逻辑行程
inline constexpr uint8_t STATE_SYNC_JOINT_CALIBRATION = 8; ///< 同步全部有效 Auto/LUT 标定快照
inline constexpr uint8_t STATE_SYNC_SERVO_ROUTE = 16; ///< 同步动态舵机 BUS 路由
inline constexpr uint8_t STATE_SYNC_PRODUCT_FINGERPRINT = 32; ///< 同步产品配置指纹
inline constexpr uint8_t STATE_SYNC_MULTITURN_STATE = 64; ///< 同步多圈恢复、logical ABS设零及上位机标定目标的最终结果
inline constexpr uint8_t STATE_SYNC_ALL = 127; ///< 同步全部静态运行契约

// 固定 payload 长度；单位 byte，不包含命令字本身。
inline constexpr size_t kFloatPayloadBytes = 84;
inline constexpr size_t kMaskedFloatPayloadBytes = 88;
inline constexpr size_t kMotorPosAbsPayloadBytes = 119;
inline constexpr size_t kMotorPosAbsOnePayloadBytes = 5;
inline constexpr size_t kActiveScopePayloadBytes = 9;
inline constexpr size_t kRestoreMultiTurnPayloadBytes = 112;
inline constexpr size_t kCalibLoadJointPayloadBytes = 189;
inline constexpr size_t kCalibLoadLimitsPayloadBytes = 224;
inline constexpr size_t kManualEncoderReferencePayloadBytes = 85;
inline constexpr size_t kManualEncoderSinglePayloadBytes = 5;
inline constexpr size_t kRopeInitPayloadBytes = 4;
inline constexpr size_t kClearCalibrationScopePayloadBytes = 4;
inline constexpr size_t kServoResetZeroPayloadBytes = 4;
inline constexpr size_t kServoRebaseZeroPayloadBytes = 4;
inline constexpr size_t kMotorDirectReleasePayloadBytes = 4;
inline constexpr size_t kStateSyncRequestPayloadBytes = 4;
inline constexpr size_t kTaskDiagSubscribePayloadBytes = 1;
inline constexpr size_t kControlTraceSubscribePayloadBytes = 1;
inline constexpr size_t kSerialRxBufferSize = 1024;
inline constexpr int32_t kEncoderModulo = 16384;

/** 查询固定命令 payload 长度；未知命令返回 SIZE_MAX。 */
inline constexpr size_t generatedCommandPayloadLength(uint8_t command)
{
    switch (command) {
        case CMD_CALIBRATE: return 0;
        case CMD_ANGLE_CTRL: return kFloatPayloadBytes;
        case CMD_START: return 0;
        case CMD_STOP: return 0;
        case CMD_RESET: return 0;
        case CMD_MOTOR_POS_ABS: return kMotorPosAbsPayloadBytes;
        case CMD_ACTIVE_SCOPE: return kActiveScopePayloadBytes;
        case CMD_RUN_FEEDBACK: return 0;
        case CMD_PAUSE: return 0;
        case CMD_RESTORE_MULTITURN: return kRestoreMultiTurnPayloadBytes;
        case CMD_HOST_HEARTBEAT: return 0;
        case CMD_MOTOR_POS_ABS_ONE: return kMotorPosAbsOnePayloadBytes;
        case CMD_CALIB_LOAD_JOINT: return kCalibLoadJointPayloadBytes;
        case CMD_CALIB_LOAD_LIMITS: return kCalibLoadLimitsPayloadBytes;
        case CMD_CLEAR_CALIBRATION: return 0;
        case CMD_MANUAL_ENCODER_REFERENCE: return kManualEncoderReferencePayloadBytes;
        case CMD_MANUAL_ENCODER_QUERY: return 0;
        case CMD_MANUAL_ENCODER_SINGLE: return kManualEncoderSinglePayloadBytes;
        case CMD_ROPE_INIT: return kRopeInitPayloadBytes;
        case CMD_ANGLE_CTRL_MASKED: return kMaskedFloatPayloadBytes;
        case CMD_CLEAR_CALIBRATION_SCOPE: return kClearCalibrationScopePayloadBytes;
        case CMD_SERVO_RESET_ZERO: return kServoResetZeroPayloadBytes;
        case CMD_STATE_SYNC_REQUEST: return kStateSyncRequestPayloadBytes;
        case CMD_TASK_DIAG_SUBSCRIBE: return kTaskDiagSubscribePayloadBytes;
        case CMD_CONTROL_TRACE_SUBSCRIBE: return kControlTraceSubscribePayloadBytes;
        case CMD_SERVO_REBASE_ZERO: return kServoRebaseZeroPayloadBytes;
        case CMD_MOTOR_DIRECT_RELEASE: return kMotorDirectReleasePayloadBytes;
        case CMD_PROTOCOL_HELLO: return 9;
        default: return SIZE_MAX;
    }
}

/** 未握手时拒绝运动和运动配置命令。 */
inline constexpr bool generatedCommandRequiresHandshake(uint8_t command)
{
    switch (command) {
        case CMD_CALIBRATE: return true;
        case CMD_ANGLE_CTRL: return true;
        case CMD_START: return true;
        case CMD_MOTOR_POS_ABS: return true;
        case CMD_RUN_FEEDBACK: return true;
        case CMD_RESTORE_MULTITURN: return true;
        case CMD_MOTOR_POS_ABS_ONE: return true;
        case CMD_CALIB_LOAD_JOINT: return true;
        case CMD_CALIB_LOAD_LIMITS: return true;
        case CMD_MANUAL_ENCODER_REFERENCE: return true;
        case CMD_MANUAL_ENCODER_SINGLE: return true;
        case CMD_ROPE_INIT: return true;
        case CMD_ANGLE_CTRL_MASKED: return true;
        case CMD_SERVO_RESET_ZERO: return true;
        case CMD_SERVO_REBASE_ZERO: return true;
        default: return false;
    }
}

inline constexpr uint32_t kFaultStatusHeartbeatMs = 200; ///< 对应上行包无变化时的强制心跳周期，单位 ms。
inline constexpr uint32_t kSystemStatusHeartbeatMs = 200; ///< 对应上行包无变化时的强制心跳周期，单位 ms。

#endif // MMHAND_UPPER_PROTOCOL_GENERATED_H
