/**
 * @file pid.h
 * @brief 声明单腱控制使用的标准离散 PID 数据结构。
 */
#ifndef MMHAND_PID_H
#define MMHAND_PID_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// PID 参数数组固定为：kp、ki、kd、死区、积分限幅、D低通时间常数、D项限幅。
#define PID_PARAMETER_NUM 7U

typedef enum {
    PID_ERROR_NONE = 0U,
    PID_FAILED_INIT = 1U,
    PID_CALC_NANINF = 2U
} PID_Status_e;

typedef struct {
    float kp;            ///< 比例增益，乘当前角度误差。
    float ki;            ///< 积分增益，乘误差时间积分。
    float kd;            ///< 微分增益，乘低通后的负反馈角速度。
    float Deadband;      ///< 绝对误差小于该角度时保持上一输出。
    float limitIntegral; ///< 积分状态绝对限幅。
    float derivativeFilterTau; ///< 磁编反馈速度低通时间常数，单位秒。
    float limitDerivative;     ///< D 项绝对限幅，单位舵机 counts。
} PID_Parameter_Typedef;

// 当前固件只使用位置式 PID，因此只保留位置误差所需的运行状态。
typedef struct {
    float error;         ///< 当前误差 target-measure。
    float previousMeasure; ///< 上个有效周期磁编反馈角。
    float filteredMeasureVelocity; ///< 低通后的磁编反馈角速度，单位 deg/s。
    float Integral;      ///< 已乘 dt、尚未乘 Ki 的误差积分。
    float Pout;          ///< 本周期比例项。
    float Iout;          ///< 本周期积分项。
    float Dout;          ///< 本周期磁编反馈速度阻尼项。
    float Output;        ///< 限幅后的总输出，单位舵机 counts。
    uint8_t derivativeInitialized; ///< 是否已有 previousMeasure 可用于微分。
    PID_Parameter_Typedef param;   ///< 固定控制参数。
    PID_Status_e status;           ///< 最近一次初始化/计算状态。
} PID_Info_TypeDef;

// 初始化控制器并加载六个 PID 参数。
PID_Status_e PID_Init(PID_Info_TypeDef *pid,
                      const float params[PID_PARAMETER_NUM]);

// 清空积分、误差历史和输出，保留当前 PID 参数。
void PID_Calc_Clear(PID_Info_TypeDef *pid);

// 计算位置式 PID 输出；dtSeconds 必须为正数。
float f_PID_Calculate(PID_Info_TypeDef *pid,
                      float target,
                      float measure,
                      float dtSeconds);

#ifdef __cplusplus
}
#endif

#endif
