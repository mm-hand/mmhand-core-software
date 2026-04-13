#ifndef CAN_COMM_TASK_H
#define CAN_COMM_TASK_H

#include <Arduino.h>

#include "TaskSharedData.h"

// CanCommTask 模块职责：
// 1) 接收编码器与错误详情 CAN 帧并重组；
// 2) 将最新传感器快照写入共享队列；
// 3) 处理来自上层的 CAN 下发命令。

// 编码器数据帧：每帧携带 4 个通道。
#define CAN_ID_ENC_BASE 0x100
#define CAN_ID_ENC_LAST (CAN_ID_ENC_BASE + (ENCODER_TOTAL_NUM + 3) / 4 - 1)

// 错误详情协议：共 3 帧，每帧携带 7 个通道错误码。
#define CAN_ID_ERROR_DETAIL_BASE 0x1F0
#define CAN_ID_ERROR_DETAIL_LAST 0x1F2
#define CAN_ERROR_DETAIL_FRAME_COUNT 3
#define CAN_ERROR_DETAIL_CODES_PER_FRAME 7
#define CAN_ERROR_DETAIL_DLC 8
#define CAN_ERROR_REASSEMBLY_TIMEOUT_MS 100
#define CAN_ERROR_TABLE_CLEAR_TIMEOUT_MS 300

// CAN 通信任务入口（FreeRTOS 任务函数）。
void taskCanComm(void* parameter);

#endif
