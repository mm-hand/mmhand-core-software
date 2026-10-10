/**
 * @file ActuatorTypes.h
 * @brief 定义业务层提交给舵机执行层的命令数据结构。
 * @details position 始终表示与舵机实际坐标一致的 ABS；驱动层负责硬件范围检查。
 */
#ifndef MMHAND_ACTUATOR_TYPES_H
#define MMHAND_ACTUATOR_TYPES_H

#include "../config/DimensionsConfig.h"
#include <stdint.h>

inline constexpr uint8_t SERVO_TARGET_OWNER_NONE = 0;        ///< 当前没有任务拥有舵机输出权。
inline constexpr uint8_t SERVO_TARGET_OWNER_CONTROL = 1;     ///< 负反馈与直控任务的执行 owner。
inline constexpr uint8_t SERVO_TARGET_OWNER_CALIBRATION = 2; ///< Rope Init 的执行 owner；保留既有协议数值。
inline constexpr uint8_t SERVO_TARGET_BATCH_MAX = SERVO_TOTAL_NUM; ///< 单批最多覆盖全手 27 台舵机。

/** 批次用途用于执行层确认安全事务；普通控制与 Rope Init 批次保持为 NORMAL。 */
inline constexpr uint8_t SERVO_TARGET_PURPOSE_NORMAL = 0;
inline constexpr uint8_t SERVO_TARGET_PURPOSE_HOST_LEASE_HOLD = 1;

/** 普通直控目标的总线写入结果，与机械到位完成分别报告。 */
inline constexpr uint8_t DIRECT_TARGET_APPLY_IDLE = 0;
inline constexpr uint8_t DIRECT_TARGET_APPLY_PENDING = 1;
inline constexpr uint8_t DIRECT_TARGET_APPLY_APPLIED = 2;
inline constexpr uint8_t DIRECT_TARGET_APPLY_REJECTED = 3;

/** 上位机租约丢失后的“保持当前 ABS”事务状态，覆盖控制和 Rope Init。 */
inline constexpr uint8_t HOST_CONTROL_HOLD_IDLE = 0;
inline constexpr uint8_t HOST_CONTROL_HOLD_PENDING = 1;
inline constexpr uint8_t HOST_CONTROL_HOLD_APPLIED = 2;
inline constexpr uint8_t HOST_CONTROL_HOLD_FAILED = 3;

struct ServoTargetCommand_t {
    uint8_t motorChannel; ///< 逻辑 Mxx；执行层在发送前解析当前 BUS 路由。
    int32_t position;  ///< 与舵机实际坐标一致的 ABS counts。
    uint16_t speed;    ///< 飞特位置命令速度参数，0 表示固件默认策略。
    uint8_t acc;       ///< 飞特位置命令加速度参数。
};

struct ServoTargetBatch_t {
    uint8_t count;     ///< commands 中有效命令数量。
    uint8_t source;    ///< SERVO_TARGET_OWNER_*，执行时再次核对 owner。
    uint8_t purpose;   ///< SERVO_TARGET_PURPOSE_*，NORMAL 之外需要执行层回写确认。
    uint8_t buildFailed; ///< 任一目标追加失败后置 1；Gateway 必须拒绝整批，禁止发送半截命令。
    ServoTargetCommand_t commands[SERVO_TARGET_BATCH_MAX]; ///< 同一控制周期的原子目标快照。
    uint32_t timestamp; ///< 生产者生成批次的单调毫秒时间。
    uint32_t transactionToken; ///< 安全事务代次；普通批次固定为 0。
    uint32_t directGeneration; ///< 普通直控代际；其他控制批次为 0。
    uint32_t directToken; ///< 上位机可选回执关联号；0 表示无需关联。
};

struct ControlCommandSnapshot_t {
    uint32_t directToken;
    uint32_t directMotorMask; ///< 本批实际输出范围，active scope 是最大授权范围。
    uint16_t directSpeed;
    uint8_t directAcc;
    float targetAngles[ENCODER_TOTAL_NUM];               ///< 关节目标角，单位 degree。
    int32_t motorTargetRaw[SERVO_TOTAL_NUM];             ///< 单次直控 logical ABS，单位 counts。
    uint32_t motorCommandToken;                          ///< 单次直控命令版本号。
    uint32_t jointCommandToken;                          ///< 关节目标命令版本号。
    uint32_t jointCommandAcceptedMs;                     ///< P4 接收该目标帧的时间。
    uint32_t feedbackCommandJointMask;                   ///< 本次反馈命令实际涉及的关节位图。
    uint32_t motorDirectCommandGeneration;               ///< 直控会话代次，隔离旧完成事件。
    uint8_t motorDirectCommandSource;                    ///< MOTOR_DIRECT_SOURCE_*。
    uint8_t controlMode;                                 ///< CONTROL_MODE_*。
};

#endif // MMHAND_ACTUATOR_TYPES_H
