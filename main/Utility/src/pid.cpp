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
PIDController::PIDController(float Kp, float Ki, float Kd, float setpoint, 
                             float output_min, float output_max, 
                             float integral_min, float integral_max)
    : Kp(Kp), Ki(Ki), Kd(Kd), setpoint(setpoint), 
      output_min(output_min), output_max(output_max), 
      integral_min(integral_min), integral_max(integral_max) {

}

// 设置设定值
void PIDController::SetTarget(float setpoint) {
    this->setpoint = setpoint;
}

// 计算PID输出
float PIDController::compute(float input, float dt) {
    float error = setpoint - input;  // 计算误差
    integral += error * dt;          // 更新积分项

    // 积分限幅
    integral = clamp(integral, integral_min, integral_max);

    float derivative = (error - prev_error) / dt;  // 计算微分项
    float output = Kp * error + Ki * integral + Kd * derivative;  // 计算输出

    // 输出限幅
    output = clamp(output, output_min, output_max);

    prev_error = error;  // 保存当前误差
    return output;
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