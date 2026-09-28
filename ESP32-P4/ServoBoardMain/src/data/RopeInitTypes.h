/**
 * @file RopeInitTypes.h
 * @brief 定义 Rope Init 非阻塞状态机使用的状态、失败原因和运行时数据。
 * @details 本文件只保存纯数据，不依赖 FreeRTOS、硬件驱动或业务服务。
 */
#ifndef P4_DATA_ROPE_INIT_TYPES_H
#define P4_DATA_ROPE_INIT_TYPES_H

#include <stdint.h>

inline constexpr uint8_t ROPE_INIT_STATUS_IDLE = 0;
inline constexpr uint8_t ROPE_INIT_STATUS_RUNNING = 1;
inline constexpr uint8_t ROPE_INIT_STATUS_SUCCESS = 2;
inline constexpr uint8_t ROPE_INIT_STATUS_FAILED = 3;

inline constexpr uint8_t ROPE_INIT_REASON_NONE = 0;
inline constexpr uint8_t ROPE_INIT_REASON_TIMEOUT = 1;
inline constexpr uint8_t ROPE_INIT_REASON_FEEDBACK_STALE = 2;
inline constexpr uint8_t ROPE_INIT_REASON_SERVO_OFFLINE = 3;
inline constexpr uint8_t ROPE_INIT_REASON_MULTITURN_UNKNOWN = 4;
inline constexpr uint8_t ROPE_INIT_REASON_MANUAL_REQUIRED = 5;
inline constexpr uint8_t ROPE_INIT_REASON_AUTO_REQUIRED = 6;
inline constexpr uint8_t ROPE_INIT_REASON_TARGET_PUBLISH_FAILED = 7;
inline constexpr uint8_t ROPE_INIT_REASON_SYSTEM_FAULT = 8;
inline constexpr uint8_t ROPE_INIT_REASON_CANCELLED = 9;

/** Rope Init 由 SystemStateTask 单线程拥有，不需要锁或独立任务句柄。 */
struct RopeInitRuntimeState {
    uint8_t active;              ///< 1 表示状态机需要继续调用 step()。
    uint8_t insideStableSamples; ///< 连续进入工作区的反馈帧数。
    uint8_t zeroPublishActive;   ///< 标定后正在逐关节发布双腱 LUT(0 deg)。
    uint8_t zeroPublishComplete; ///< 最后一对目标已发布，等待执行器消费一周期。
    uint32_t zeroPendingJointMask; ///< 尚未发布 LUT(0 deg) 的双腱关节。
    uint32_t beginMs;            ///< 本轮开始的单调毫秒时间。
    uint32_t nextStepMs;         ///< 下一次允许推进的时间，保持原 50 ms 节拍。
};

#endif // P4_DATA_ROPE_INIT_TYPES_H
