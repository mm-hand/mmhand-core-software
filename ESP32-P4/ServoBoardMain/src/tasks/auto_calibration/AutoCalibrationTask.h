/**
 * @file AutoCalibrationTask.h
 * @brief 声明自动标定 FreeRTOS 薄入口。
 * @details 任务创建由 AutoCalibrationTaskRuntime 管理，会话流程由 Controller 实现。
 */
#ifndef P4_TASKS_AUTO_CALIBRATION_TASK_H
#define P4_TASKS_AUTO_CALIBRATION_TASK_H

/** FreeRTOS 入口只转调标定协调器，不在 Task 层保存业务状态。 */
void autoCalibrationTask(void* parameter);

#endif // P4_TASKS_AUTO_CALIBRATION_TASK_H
