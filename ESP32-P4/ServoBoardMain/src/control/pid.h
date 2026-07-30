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

// PID 参数数组固定为：kp、ki、kd、死区、积分限幅、输出限幅。
#define PID_PARAMETER_NUM 6U

typedef enum {
    PID_ERROR_NONE = 0U,
    PID_FAILED_INIT = 1U,
    PID_CALC_NANINF = 2U
} PID_Status_e;

typedef struct {
    float kp;
    float ki;
    float kd;
    float Deadband;
    float limitIntegral;
    float limitOutput;
} PID_Parameter_Typedef;

// 当前固件只使用位置式 PID，因此只保留位置误差所需的运行状态。
typedef struct {
    float error;
    float previousError;
    float Integral;
    float Pout;
    float Iout;
    float Dout;
    float Output;
    uint8_t derivativeInitialized;
    PID_Parameter_Typedef param;
    PID_Status_e status;
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
