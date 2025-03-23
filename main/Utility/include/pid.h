/*------------------------------------------------------------------------------
 * @file    PID.H
 * @author  Hovey https://space.bilibili.com/33582262?spm_id_from=333.1007.0.0
 * @date    2025/03/23 23:50:07
 * @brief   
 * -----------------------------------------------------------------------------
 * @attention 
 
------------------------------------------------------------------------------*/

/* Define to prevent recursive inclusion  ------------------------------------*/
#ifndef __PID_H
#define __PID_H

/* Files includes  -----------------------------------------------------------*/


/* Defines -------------------------------------------------------------------*/
class PIDController {
public:
    // 构造函数
    PIDController(float Kp, float Ki, float Kd, float setpoint, 
                  float output_min, float output_max, 
                  float integral_min, float integral_max);
    // 设置设定值
    void SetTarget(float setpoint);

    // 计算PID输出
    float compute(float input, float dt);

    // 重置PID控制器状态
    void reset();

private:
    // 限幅函数
    float clamp(float value, float min, float max);

    float Kp;           // 比例系数
    float Ki;           // 积分系数
    float Kd;           // 微分系数
    float setpoint;     // 设定值
    float integral      = 0.0f;     // 积分项
    float prev_error    = 0.0f;   // 上一次的误差
    float output_min;   // 输出最小值
    float output_max;   // 输出最大值
    float integral_min; // 积分项最小值
    float integral_max; // 积分项最大值
};

#endif // PIDCONTROLLER_H
