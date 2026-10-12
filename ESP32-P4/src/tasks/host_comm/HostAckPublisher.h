/**
 * @file HostAckPublisher.h
 * @brief 声明上位机命令 ACK 的有序发送与串口忙重试接口。
 */
#ifndef P4_TASKS_HOST_COMM_HOST_ACK_PUBLISHER_H
#define P4_TASKS_HOST_COMM_HOST_ACK_PUBLISHER_H

#include <stdint.h>

namespace p4::host {

/** @return ACK 已写出或已进入本任务 FIFO 时返回 true。 */
bool sendProtoAckPacket(uint8_t cmd, uint8_t appliedMode, uint8_t status);

/** 重试最早一条未写出的 ACK；队首仍忙时返回 false。 */
bool retryPendingProtoAckPacket();

} // namespace p4::host

#endif // P4_TASKS_HOST_COMM_HOST_ACK_PUBLISHER_H
