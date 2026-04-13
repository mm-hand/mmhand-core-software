#include <Arduino.h>
#include "SystemTask.h"

// ServoBoardMain 入口模块职责：
// 1) 初始化系统资源与后台任务；
// 2) 维持 Arduino 主循环节拍，并将业务执行交给 FreeRTOS 任务。
void setup() {
    // 系统初始化：创建任务、总线和共享数据。
    System_Init();
}

void loop() {
    // 主循环仅保留轻量调度入口。
    System_Loop();
}
