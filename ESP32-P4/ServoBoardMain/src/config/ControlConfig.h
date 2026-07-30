/**
 * @file ControlConfig.h
 * @brief 汇总控制周期、PID、死区和碰撞检测配置的公共入口。
 * @details 第一阶段仅建立边界；现有控制参数会在拆分 ControlTask 时迁入。
 */
#ifndef MMHAND_CONTROL_CONFIG_H
#define MMHAND_CONTROL_CONFIG_H

namespace mmhand::config::control {
// 参数将在保持数值和计算顺序不变的前提下，从各控制实现迁入此命名空间。
}

#endif // MMHAND_CONTROL_CONFIG_H
