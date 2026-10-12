/**
 * @file ControlCommandSnapshotReader.h
 * @brief 为控制任务读取一份原子一致的上位机命令快照。
 *
 * 本文件只处理 RTOS 互斥和数据复制，不包含安全判断、PID 或舵机输出。
 */
#ifndef P4_TASKS_MOTION_CONTROL_COMMAND_SNAPSHOT_READER_H
#define P4_TASKS_MOTION_CONTROL_COMMAND_SNAPSHOT_READER_H

#include "../../system/SystemRuntimeContext.h"

namespace p4::control {

/**
 * 在 commandStateMutex 保护下复制目标、模式、掩码和 token。
 * 返回 false 表示本周期未能取得一致快照，调用方应保持当前输出。
 */
bool readCommandSnapshot(SystemRuntimeContext* runtime,
                         ControlCommandSnapshot_t* snapshot);

} // namespace p4::control

#endif // P4_TASKS_MOTION_CONTROL_COMMAND_SNAPSHOT_READER_H
