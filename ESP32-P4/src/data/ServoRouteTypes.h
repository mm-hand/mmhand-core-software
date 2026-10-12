/**
 * @file ServoRouteTypes.h
 * @brief 定义逻辑电机 Mxx 到物理飞特 BUS/ID 的运行时路由数据。
 *
 * 路由只描述本次上电发现结果，不属于机械拓扑；多圈锚点由桌面端 JSON 管理。
 * 同一 ID 出现在多条 BUS 或单 BUS 超过十台时，设备可以被看见但不得运动。
 */
#ifndef MMHAND_SERVO_ROUTE_TYPES_H
#define MMHAND_SERVO_ROUTE_TYPES_H

#include "../config/DimensionsConfig.h"
#include <stdint.h>

inline constexpr uint8_t SERVO_ROUTE_BUS_UNKNOWN = 0xFF;

enum class ServoRouteState : uint8_t {
    Unknown = 0,
    Probing,
    Online,
    Offline,
    Conflict,
    BusOverloaded,
};

struct ServoRoute {
    uint8_t busIndex;       ///< 当前唯一响应 BUS；无效时为 0xFF。
    uint8_t servoId;        ///< Mxx 固定对应的飞特 ID。
    ServoRouteState state;  ///< 路由发现和安全状态。
};

struct ServoRouteSnapshot {
    ServoRoute routes[SERVO_TOTAL_NUM];
    uint32_t detectedIdMask[NUM_BUSES]; ///< 每条 BUS 实际响应的 ID0..ID26 位图。
    uint32_t validMotorMask;            ///< 具有唯一 BUS 的 Mxx。
    uint32_t controllableMotorMask;     ///< 唯一路由且 BUS 未超容量的 Mxx。
    uint32_t conflictMotorMask;         ///< 同一 ID 在多条 BUS 响应的 Mxx。
    uint8_t busDeviceCount[NUM_BUSES];  ///< 每条 BUS 响应的唯一 ID 数；协议无法区分同 BUS 同 ID 的多台设备。
    uint8_t busOverloadMask;            ///< bit0..bit3 表示 BUS0..BUS3 超过12台。
    uint8_t scanState;                  ///< 0空闲、1全量扫描、2单电机恢复扫描。
    uint8_t scanProgress;               ///< 0..100 的扫描进度。
    uint32_t generation;                ///< 整张路由表版本。
};

#endif // MMHAND_SERVO_ROUTE_TYPES_H
