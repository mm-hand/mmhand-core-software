/**
 * @file AppBootstrap.h
 * @brief 声明 P4 系统初始化入口和全局共享运行上下文。
 */
#ifndef P4_SYSTEM_APP_BOOTSTRAP_H
#define P4_SYSTEM_APP_BOOTSTRAP_H

#include <stdint.h>

// Arduino setup() 的系统入口：创建共享资源并启动全部任务。
void appBootstrapSetup();

// Arduino loop() 的系统入口：这里只让出 CPU，业务逻辑均由 FreeRTOS 任务执行。
void appBootstrapLoop();

#endif // P4_SYSTEM_APP_BOOTSTRAP_H
