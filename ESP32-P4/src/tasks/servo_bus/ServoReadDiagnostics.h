/**
 * @file ServoReadDiagnostics.h
 * @brief 反馈失败只输出文本；不扩展上位机二进制包或参与控制判定。
 */
#pragma once

#include <stdint.h>

class FeetechServoDriver;

/** 在没有未完成同步读时调用，避免日志占用尚在收集回包的周期。 */
void logServoReadFailure(const FeetechServoDriver* driver,
                         uint8_t busIndex, uint8_t servoId);
