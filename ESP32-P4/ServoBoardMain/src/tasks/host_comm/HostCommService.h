/**
 * @file HostCommService.h
 * @brief 声明上位机协议收发服务的运行入口。
 * @details Task 层只提供调度入口，协议解析、命令路由和遥测节奏由该服务维护。
 */
#ifndef P4_TASKS_HOST_COMM_HOST_COMM_SERVICE_H
#define P4_TASKS_HOST_COMM_HOST_COMM_SERVICE_H

#include <stdint.h>

struct SystemRuntimeContext;

/** 初始化上位机通信服务的协议状态；不进入任务循环。 */
bool initHostCommService(SystemRuntimeContext* sharedData);

/** 处理一轮收包、命令分发和遥测发布。 */
bool stepHostCommService(SystemRuntimeContext* sharedData, uint32_t nowMs);

/**
 * 请求上行服务补发指定静态状态。
 *
 * 该接口只设置 HostCommTask 私有的待发送位图，不直接写串口，避免命令分发
 * 阻塞在多包标定快照发送上。位定义由唯一协议 schema 的 STATE_SYNC_* 给出。
 */
void requestHostStateSync(uint32_t syncMask);

/**
 * 打开、续订或关闭 Task Runtime 诊断流。
 * 重复打开只更新独立诊断租约，不能重置正在发送的七包窗口。
 */
void setHostTaskDiagSubscribed(bool subscribed, uint32_t nowMs);

#endif // P4_TASKS_HOST_COMM_HOST_COMM_SERVICE_H
