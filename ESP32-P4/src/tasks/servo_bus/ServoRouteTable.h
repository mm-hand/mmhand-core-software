/**
 * @file ServoRouteTable.h
 * @brief 保存动态舵机路由，并集中执行冲突和单 BUS 十台容量裁决。
 *
 * 本模块不访问 UART。ServoBusTask 是唯一写入者；扫描开始时立即
 * 撤销旧 BUS 并发布 Probing，扫描完成后一次性重建可控位图，禁止继续使用旧路由。
 */
#ifndef P4_SERVO_ROUTE_TABLE_H
#define P4_SERVO_ROUTE_TABLE_H

#include "../../data/ServoRouteTypes.h"

class ServoRouteTable {
public:
    void begin();
    void beginFullScan();
    void beginMotorScan(uint8_t motorChannel);
    void recordProbe(uint8_t busIndex, uint8_t servoId, bool present);
    void finishScan();
    void setScanProgress(uint8_t progress);
    void markMotorOffline(uint8_t motorChannel);

    bool resolveDetected(uint8_t motorChannel, ServoRoute* out) const;
    bool resolveControllable(uint8_t motorChannel, ServoRoute* out) const;
    ServoRouteSnapshot snapshot() const;
    bool scanActive() const;
    uint8_t scanState() const;

private:
    void rebuildDerivedState();
    ServoRouteSnapshot snapshot_ = {};
};

ServoRouteTable& servoRouteTable();

#endif // P4_SERVO_ROUTE_TABLE_H
