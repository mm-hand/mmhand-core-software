#ifndef SERVO_STATE_H
#define SERVO_STATE_H

#include <Arduino.h>

#define EMERGENCY_LOAD 5000 // TODO

// 舵机错误类型枚举
enum class ServoError {
    Success,
    Communication,
    Timeout,
    Overload,
    Overheat,
    Voltage,
    InvalidPosition
};

// 简化的舵机状态结构体（不包含成员函数）
struct ServoState {
    uint8_t id = 0;
    uint16_t currentRawPos = 2048;// 原始位置值 (0-4095)
    int32_t turns = 0;              // 转过的圈数
    int32_t absolutePos = 0;       // 绝对位置
    int32_t currentLoad = 0;       // 当前负载值
    bool initialized = false;       // 是否已初始化
    uint16_t limitPos = 2048;      // 极限位置（默认保守值）
    ServoError lastError = ServoError::Success;
    uint32_t lastCommTime = 0;
};

#endif // SERVO_STATE_H