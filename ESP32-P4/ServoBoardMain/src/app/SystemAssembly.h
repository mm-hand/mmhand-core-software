/**
 * @file SystemAssembly.h
 * @brief 声明 P4 运行上下文、持久化恢复和 FreeRTOS 任务的启动装配入口。
 * @details 该模块只在上电阶段运行，业务任务不得反向调用启动装配函数。
 */
#ifndef P4_SYSTEM_SYSTEM_ASSEMBLY_H
#define P4_SYSTEM_SYSTEM_ASSEMBLY_H

/** 创建共享资源、初始化状态、恢复 NVS 并启动全部业务任务。 */
bool assembleAndStartSystem();

#endif // P4_SYSTEM_SYSTEM_ASSEMBLY_H
