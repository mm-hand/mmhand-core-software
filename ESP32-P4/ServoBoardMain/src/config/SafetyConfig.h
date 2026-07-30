/**
 * @file SafetyConfig.h
 * @brief 汇总舵机硬边界、反馈超时和运行安全阈值。
 * @details 安全规则由 SafetyPolicy 执行；本文件只定义不可变配置。
 */
#ifndef MMHAND_SAFETY_CONFIG_H
#define MMHAND_SAFETY_CONFIG_H

namespace mmhand::config::safety {
// 现有 ±30719 等硬边界将在安全模块迁移阶段统一到这里。
}

#endif // MMHAND_SAFETY_CONFIG_H
