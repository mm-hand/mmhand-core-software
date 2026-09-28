/**
 * @file AutoCalibrationTaskRuntime.h
 * @brief 声明系统层自动标定任务的创建、去重和退出登记接口。
 * @details 该端口隔离业务状态机与 FreeRTOS 任务句柄，机械标定逻辑不依赖本文件。
 */
#ifndef P4_SYSTEM_AUTO_CALIBRATION_TASK_RUNTIME_H
#define P4_SYSTEM_AUTO_CALIBRATION_TASK_RUNTIME_H

struct SystemRuntimeContext;

/** 创建唯一自动标定任务；已有任务运行时幂等返回 true。 */
bool startAutoCalibrationTaskRuntime(SystemRuntimeContext* sharedData);

/** 由 Task 薄入口退出路径调用，注销监控并释放运行句柄。 */
void finishAutoCalibrationTaskRuntime();

#endif // P4_SYSTEM_AUTO_CALIBRATION_TASK_RUNTIME_H
