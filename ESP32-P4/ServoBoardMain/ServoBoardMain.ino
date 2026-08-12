/**
 * @file ServoBoardMain.ino
 * @brief Arduino 固件入口；只转交系统启动和空闲循环，不放置业务逻辑。
 */
#include <Arduino.h>
#include "src/app/AppBootstrap.h"

// setup 只负责建立资源并启动 FreeRTOS 任务。
void setup() {
    appBootstrapSetup();
}

// 所有业务由 FreeRTOS 任务执行，Arduino loop 持续让出 CPU。
void loop() {
    appBootstrapLoop();
}
