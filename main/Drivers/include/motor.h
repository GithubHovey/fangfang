/*------------------------------------------------------------------------------
 * @file    MOTOR.H
 * @author  Hovey https://space.bilibili.com/33582262?spm_id_from=333.1007.0.0
 * @date    2025/03/08 04:00:49
 * @brief   
 * -----------------------------------------------------------------------------
 * @attention 
 
------------------------------------------------------------------------------*/

/* Define to prevent recursive inclusion  ------------------------------------*/
#ifndef MOTOR_H
#define MOTOR_H

// #include "driver/mcpwm_timer.h"
#include "driver/mcpwm_prelude.h"
#include "esp_adc/adc_oneshot.h"
#include "as5600.h"
#define USE_VOFA
#ifdef USE_VOFA
#include "utility.h"
#endif
#define MCPWM0 0 
#define MCPWM1 1

class Motor {
public:
    Motor(uint8_t pole_pairs, int8_t direction, float max_voltage,
            int mcpwm_unit, gpio_num_t pwm_a, gpio_num_t pwm_b, gpio_num_t pwm_c,
            i2c_port_t i2c_port, gpio_num_t sda_pin, gpio_num_t scl_pin, 
            
            float Kp1, float Ki1, float Kd1, float output_max1, float integral_max1,   //angle loop
            float Kp2, float Ki2, float Kd2, float output_max2, float integral_max2,   //speed loop
            float Kp3, float Ki3, float Kd3, float output_max3, float integral_max3    //torque loop
          );
    ~Motor();

    void init();
    void setSpeed(float speed); // Speed in radians per second
    void setTorque(float torque); // Torque in Nm
    void update(); // Update FOC control loop
    void debug(void);
    void SetTargetAngle(float target);
    

private:

//
    uint8_t pole_pairs;
    int8_t direction;
    float max_voltage;


    int16_t rotate_cnt = 0;
    uint16_t offset = 0;
    uint16_t pre_angle = 0;
    uint16_t raw_current_angle = 0;
    int mcpwm_unit;
    mcpwm_timer_handle_t mcpwm_timer;
    gpio_num_t pwm_a;
    gpio_num_t pwm_b;
    gpio_num_t pwm_c;
    mcpwm_cmpr_handle_t comparator_a = NULL, comparator_b = NULL, comparator_c = NULL;
    AS5600 encoder; // AS5600 encoder instance
    PIDController loop_angle;
    PIDController loop_speed;
    PIDController loop_torque;
    
    float current_speed;
    float target_speed;
    float current_torque;
    float target_torque;
    float target_angle;
    float current_angle = 0.0f;
    float Ua,Ub,Uc;

    mcpwm_oper_handle_t oper1 = NULL;
    mcpwm_oper_handle_t oper2 = NULL;
    mcpwm_gen_handle_t generator_a = NULL, generator_b = NULL, generator_c = NULL;

    //adc
    adc_oneshot_unit_handle_t adc_handle = NULL;
    adc_cali_handle_t adc_cali1_handle = NULL;
    adc_cali_handle_t adc_cali2_handle = NULL;
    bool adc_cali1_enable = false;
    bool adc_cali2_enable = false;
    int adc_rawdata;
    int voltage;


    
    

    void initADC();
    bool adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle);
    void applyPWM(float u, float v, float w);
    void focControl(float Uq, float Ud);
    float UpdateAngle(void);
    void calibration_offset(void);
    
};

#endif // MOTOR_H