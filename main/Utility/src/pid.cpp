/*------------------------------------------------------------------------------
 * @file    PID.CPP
 * @author  Hovey https://space.bilibili.com/33582262?spm_id_from=333.1007.0.0
 * @date    2025/03/23 23:50:15
 * @brief   
 * -----------------------------------------------------------------------------
 * @attention 
 
------------------------------------------------------------------------------*/
#include "pid.h"

// 构造函数
PIDController::PIDController(float Kp, float Ki, float Kd, 
                             float output_max, 
                             float integral_max)
    : Kp(Kp), Ki(Ki), Kd(Kd), 
      output_max(output_max), 
      integral_max(integral_max) {

}

// 重置PID控制器状态
void PIDController::reset() {
    integral = 0.0f;
    prev_error = 0.0f;
}

// 限幅函数
float PIDController::clamp(float value, float min, float max) {
    if (value < min) {
        return min;
    } else if (value > max) {
        return max;
    }
    return value;
}

float PIDController::PIDout(float target, float current, float dt)
{
    float error = target - current;  // 计算误差
    float output = 0.0f;
    if(Kp != 0)
    {
        output += Kp * error;
    }
    if(Ki != 0)
    {
        integral += error * dt;          // 更新积分项
    // 积分限幅
        integral = clamp(integral, -integral_max, integral_max);
        output += Ki * integral;
    }
    if(Kd != 0)
    {
        float derivative = (error - prev_error) / dt;  // 计算微分项
        float output =  Kd * derivative;  // 计算输出
    }
    // 输出限幅
    output = clamp(output, -output_max, output_max);
    prev_error = error;  // 保存当前误差
    return output;
}