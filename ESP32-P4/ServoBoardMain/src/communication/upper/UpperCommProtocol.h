#ifndef UPPER_COMM_PROTOCOL_H
#define UPPER_COMM_PROTOCOL_H

#include <Arduino.h>
#include "../../shared/TaskSharedData.h"

// UpperCommProtocol 闂嗗棔鑵戠紒瀛樺Б娑撴彃褰涚痪鍨礂鐠侇喖鐖堕柌蹇嬧偓?// 濞夈劍鍓伴敍姘崇箹闁插瞼娈戦崨鎴掓姢 ID 娑?desktop 娑撳﹣缍呴張杞扮箽閹镐礁鍚嬬€圭櫢绱濇穱顔芥暭閺冨爼娓剁憰浣告倱濮濄儲娲块弬鐗堫攽闂堛垻顏憴锝嗙€?閹垫挸瀵橀柅鏄忕帆閵?
// 娑撳﹨顢戠敮褝绱欑拋鎯ь槵 -> 娑撳﹣缍呴張鐚寸礆閿涙瓟0xFE][LEN][TYPE][PAYLOAD][0xFF]
#define PROTOCOL_HEADER 0xFE
#define PROTOCOL_TAIL 0xFF
#define PACKET_TYPE_SENSOR 0x01
#define PACKET_TYPE_CALIB_ACK 0x02
#define PACKET_TYPE_SERVO_ANGLE 0x03
#define PACKET_TYPE_JOINT1_DEBUG 0x04
#define PACKET_TYPE_SERVO_TELEM 0x05
#define PACKET_TYPE_PROTO_ACK 0x06
#define PACKET_TYPE_FAULT_STATUS 0x07
#define PACKET_TYPE_RELEASE_FAULT 0x08
#define PACKET_TYPE_SERVO_RAW 0x09
#define PACKET_TYPE_TACTILE 0x0A
#define PACKET_TYPE_ENCODER_RAW 0x0B
#define PACKET_TYPE_SYSTEM_STATUS 0x0C
#define PACKET_TYPE_MOTOR_ABS_LIMIT 0x0D
#define PACKET_TYPE_JOINT_CALIB_SNAPSHOT 0x0E
#define PACKET_TYPE_TACTILE_FULL 0x0F
#define PACKET_TYPE_CALIB_DIAG 0x10

#define PROTOCOL_DISCONNECT_SENTINEL ((int16_t)0x7FFF)

#define CMD_CALIBRATE 0xCA
#define CMD_ANGLE_CTRL 0xCB
#define CMD_START 0xCC
#define CMD_STOP 0xCD
#define CMD_RESET 0xCE
#define CMD_CALIB_DATA 0xCF
#define CMD_MOTOR_POS 0xD0
#define CMD_SENSOR_STREAM_MODE 0xD1
#define CMD_MOTOR_POS_SWEEP 0xD2
#define CMD_MOTOR_POS_ABS 0xD3
#define CMD_MOTOR_POS_ABS_ONE 0xDE
#define CMD_TENDON_GUARD 0xD4
#define CMD_ACTIVE_SCOPE 0xD7
#define CMD_RUN_FEEDBACK 0xD8
#define CMD_PAUSE 0xD9
#define CMD_RESTORE_MULTITURN 0xDA
#define CMD_MOTOR_ABS_LIMIT_LOCK 0xDB
// Host heartbeat; servo tasks use missed heartbeats to decide whether to trigger NVS backup.
#define CMD_MULTITURN_NVS_CONTROL 0xDD
#define CMD_HOST_HEARTBEAT 0xDC
#define CMD_CALIB_LOAD_JOINT 0xDF
#define CMD_CALIB_LOAD_LIMITS 0xE0
#define CMD_LOCAL_TEST_PARAMS 0xE1
#define CMD_LOCAL_TEST_START 0xE2
#define CMD_LOCAL_TEST_STOP 0xE3
#define CMD_SIDE_SWAY_ADJUST 0xE4

#define PROTO_ACK_STATUS_OK 0
#define PROTO_ACK_STATUS_UNSUPPORTED_MODE 1
#define PROTO_ACK_STATUS_QUEUED 2

#define SENSOR_STREAM_MODE_LEGACY_U16_WRAP 0
#define SENSOR_STREAM_MODE_SIGNED_I16 1

static const size_t kFloatPayloadBytes = ENCODER_TOTAL_NUM * sizeof(float);
static const size_t kMotorPosPayloadBytes = SERVO_TOTAL_NUM * sizeof(uint16_t);
static const size_t kMotorPosAbsOnePayloadBytes = 3;
static const size_t kTendonGuardPayloadBytes = ENCODER_TOTAL_NUM * 4;
static const size_t kActiveScopePayloadBytes = 9;
static const size_t kRestoreMultiTurnPayloadBytes = 4 + SERVO_TOTAL_NUM * sizeof(uint16_t);
static const size_t kMotorAbsLimitLockPayloadBytes = 5;
static const size_t kMultiTurnNvsControlPayloadBytes = 1;
static const size_t kCalibLoadJointPayloadBytes = 54;
static const size_t kCalibLoadLimitsPayloadBytes = 8 + SERVO_TOTAL_NUM * 4;
static const size_t kLocalTestParamsPayloadBytes = 2 * sizeof(float);
static const size_t kSerialRxBufferSize = 512;
static const int32_t kEncoderModulo = 16384;
static const uint32_t kFaultStatusHeartbeatMs = 200;
static const uint32_t kSystemStatusHeartbeatMs = 100;
static const uint32_t kMotorAbsLimitHeartbeatMs = 500;
static const uint32_t kJointCalibSnapshotHeartbeatMs = 1000;

#endif // UPPER_COMM_PROTOCOL_H
