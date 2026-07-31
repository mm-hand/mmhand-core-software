/**
 * @file ServoFeedbackPublisher.h
 * @brief 声明舵机反馈快照、恢复状态和硬件圈数越界状态的统一发布接口。
 *
 * 本模块只读取 FeetechServoDriver 已缓存的反馈，不主动发起串口读写。
 * ServoCommunicationTask 负责总线调度，其他任务只接触稳定的共享快照。
 */
#ifndef P4_COMMUNICATION_SERVO_FEEDBACK_PUBLISHER_H
#define P4_COMMUNICATION_SERVO_FEEDBACK_PUBLISHER_H

#include <Arduino.h>

#include "../../shared/TaskSharedData.h"

namespace p4::servo {

/** 返回当前全部映射舵机的在线位图。 */
uint32_t collectOnlineMotorMask();

/** 把驱动层恢复事务状态发布到 TaskSharedData 的恢复位图。 */
void publishRecoveryState(TaskSharedData_t* sharedData);

/**
 * 检测舵机本次上电会话的 hardware ABS 是否逼近物理多圈边界。
 * 新进入越界时关闭对应舵机力矩，并向总状态机发送 STOP。
 */
void publishTurnOverflowState(TaskSharedData_t* sharedData);

/**
 * 构造逻辑 ABS、raw、Flash raw 和遥测快照，并覆盖各反馈队列。
 * 同时维护 online、logical range 和 Turn Unknown 位图。
 */
void publishFeedback(TaskSharedData_t* sharedData);

} // namespace p4::servo

#endif // P4_COMMUNICATION_SERVO_FEEDBACK_PUBLISHER_H
