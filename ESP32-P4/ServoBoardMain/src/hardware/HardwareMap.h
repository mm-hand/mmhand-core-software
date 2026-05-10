#ifndef HARDWARE_MAP_H
#define HARDWARE_MAP_H

#include "../shared/TaskSharedData.h"

// HardwareMap 模块职责：
// 1) 集中定义关节/电机到物理总线与 ID 的映射；
// 2) 避免 SystemTask 承担硬件拓扑配置职责。
// 映射变量本身在 TaskSharedData.h 中声明为 extern，本头文件用于让调用方显式依赖硬件拓扑模块。

#endif // HARDWARE_MAP_H
