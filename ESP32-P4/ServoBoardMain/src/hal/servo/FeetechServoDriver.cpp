/**
 * @file FeetechServoDriver.cpp
 * @brief 编译飞特 SDK 实现，确保 SDK 实现只进入一个翻译单元。
 */
#include "FeetechServoDriver.h"

#ifndef FTSERVO_ARDUINO_EXTERNAL_BUILD
#include "../libraries/FTServo_Arduino/src/SCS.cpp"
#include "../libraries/FTServo_Arduino/src/SCSerial.cpp"
#include "../libraries/FTServo_Arduino/src/SMS_STS.cpp"
#endif
