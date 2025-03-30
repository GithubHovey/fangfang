/*------------------------------------------------------------------------------
 * @file    MOTOR.C
 * @author  Hovey https://space.bilibili.com/33582262?spm_id_from=333.1007.0.0
 * @date    2025/03/08 04:00:45
 * @brief   
 * -----------------------------------------------------------------------------
 * @attention 
 
------------------------------------------------------------------------------*/
#include "motor.h"
#include "esp_log.h"
#include <cmath>
#include "utility.h"

#define limit(value, min, max) ((value) < (min) ? (min) : ((value) > (max) ? (max) : (value)))
#define PWM_CYCLE 500 /*0.1us */
Motor::Motor(uint8_t pole_pairs, int8_t direction, float max_voltage,
            int mcpwm_unit, gpio_num_t pwm_a, gpio_num_t pwm_b, gpio_num_t pwm_c,
             i2c_port_t i2c_port, gpio_num_t sda_pin, gpio_num_t scl_pin, 
             adc_unit_t adc_unit, adc_channel_t u_chanel, adc_channel_t v_chanel, 
             float Kp1, float Ki1, float Kd1, float output_max1, float integral_max1,   //angle loop
             float Kp2, float Ki2, float Kd2, float output_max2, float integral_max2,   //speed loop
             float Kp3, float Ki3, float Kd3, float output_max3, float integral_max3    //torque loop
             )
    : 
        pole_pairs(pole_pairs),direction(direction),max_voltage(max_voltage),
        mcpwm_unit(mcpwm_unit), pwm_a(pwm_a), pwm_b(pwm_b), pwm_c(pwm_c),
        encoder(i2c_port, sda_pin, scl_pin), // Initialize AS5600 encoder
        adc_unit(adc_unit), u_chanel(u_chanel), v_chanel(v_chanel),
        loop_angle(Kp1, Ki1, Kd1, output_max1, integral_max1),
        loop_speed(Kp2, Ki2, Kd2, output_max2, integral_max2),
        loop_torque(Kp3,Ki3, Kd3, output_max3, integral_max3)
        
        {}


Motor::~Motor() {
    // Clean up MCPWM resources if needed
    mcpwm_del_generator(generator_a);
    mcpwm_del_generator(generator_b);
    mcpwm_del_generator(generator_c);
    mcpwm_del_comparator(comparator_a);
    mcpwm_del_comparator(comparator_b);
    mcpwm_del_comparator(comparator_c);
    mcpwm_del_operator(oper1);
    mcpwm_del_operator(oper2);
    mcpwm_del_timer(mcpwm_timer);
}

void Motor::init() {
    // Initialize MCPWM timer
    mcpwm_timer_config_t timer_config = {
        .group_id = mcpwm_unit,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = 10000000, // 1 MHz resolution
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
        .period_ticks = PWM_CYCLE, // 10 ms period
    };
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &mcpwm_timer));

    // Initialize MCPWM oper
    
    mcpwm_operator_config_t operator_config = {
        .group_id = mcpwm_unit,
    };
    ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &oper1));
    ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &oper2));

    // Connect timer to oper
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper1, mcpwm_timer));
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper2, mcpwm_timer));

    // Initialize MCPWM comparators
    
    mcpwm_comparator_config_t comparator_config = {
        .flags = {
            .update_cmp_on_tez = true,
        },
    };
    ESP_ERROR_CHECK(mcpwm_new_comparator(oper1, &comparator_config, &comparator_a));
    ESP_ERROR_CHECK(mcpwm_new_comparator(oper1, &comparator_config, &comparator_b));
    ESP_ERROR_CHECK(mcpwm_new_comparator(oper2, &comparator_config, &comparator_c));

    // Initialize MCPWM generators
    
    mcpwm_generator_config_t generator_config = {
        .gen_gpio_num = pwm_a,
    };
    ESP_ERROR_CHECK(mcpwm_new_generator(oper1, &generator_config, &generator_a));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(generator_a,
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(generator_a,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparator_a, MCPWM_GEN_ACTION_LOW))); 

    generator_config.gen_gpio_num = pwm_b;
    ESP_ERROR_CHECK(mcpwm_new_generator(oper1, &generator_config, &generator_b));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(generator_b,
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(generator_b,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparator_b, MCPWM_GEN_ACTION_LOW))); 
    
    generator_config.gen_gpio_num = pwm_c;
    ESP_ERROR_CHECK(mcpwm_new_generator(oper2, &generator_config, &generator_c));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(generator_c,
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(generator_c,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparator_c, MCPWM_GEN_ACTION_LOW))); 


    // Set PWM duty cycle
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator_a, PWM_CYCLE/2)); // 50% duty cycle
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator_b, PWM_CYCLE/2));
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator_c, PWM_CYCLE/2));

    // Start MCPWM timer
    ESP_ERROR_CHECK(mcpwm_timer_enable(mcpwm_timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(mcpwm_timer, MCPWM_TIMER_START_NO_STOP));

    // Initialize encoder
    if (!encoder.init()) {
        ESP_LOGE("Motor", "Failed to initialize encoder");
    }
    initADC();
#ifdef USE_VOFA
    
#endif
}


bool Motor::adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle)
{
    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;
    bool calibrated = false;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    if (!calibrated) {
        ESP_LOGI("motor", "calibration scheme version is %s", "Curve Fitting");
        adc_cali_curve_fitting_config_t cali_config = {
            .unit_id = unit,
            .chan = channel,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }
#endif

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    if (!calibrated) {
        ESP_LOGI("motor", "calibration scheme version is %s", "Line Fitting");
        adc_cali_line_fitting_config_t cali_config = {
            .unit_id = unit,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_line_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }
#endif

    *out_handle = handle;
    if (ret == ESP_OK) {
        ESP_LOGI("motor", "Calibration Success");
    } else if (ret == ESP_ERR_NOT_SUPPORTED || !calibrated) {
        ESP_LOGW("motor", "eFuse not burnt, skip software calibration");
    } else {
        ESP_LOGE("motor", "Invalid arg or no memory");
    }

    return calibrated;
}

void Motor::initADC()
{
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = adc_unit,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc_handle));
    
    adc_oneshot_chan_cfg_t config = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, u_chanel, &config));//gpio9
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, v_chanel, &config));//gpio10
    adc_cali1_enable = adc_calibration_init(adc_unit, u_chanel, ADC_ATTEN_DB_12, &adc_cali1_handle);
    adc_cali2_enable = adc_calibration_init(adc_unit, v_chanel, ADC_ATTEN_DB_12, &adc_cali2_handle);
}

void Motor::setSpeed(float speed) {
    target_speed = speed;
}

void Motor::setTorque(float torque) {
    target_torque = torque;
}

void Motor::update() {

    calibration_offset();
    SetTargetAngle(30);

}
void Motor::debug() {
    if(adc_handle == nullptr)
    {
        ESP_LOGE("Motor", "adc_handle is null");
        return ;
    }
    
    ESP_ERROR_CHECK(adc_oneshot_read(adc_handle, u_chanel, &adc_rawdata));
    ESP_LOGI("Motor", "ADC%d Channel[%d] Raw Data: %d", adc_unit + 1, u_chanel, adc_rawdata);
    if (adc_cali1_enable) {
    ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc_cali1_handle, adc_rawdata, &voltage));
    ESP_LOGI("Motor", "ADC%d Channel[%d] Cali Voltage: %d mV", adc_unit + 1, u_chanel, voltage);
    }

}


void Motor::focControl(float Uq, float Ud = 0.0f) { 
    // Implement FOC control logic here
    // This is a simplified example, actual FOC implementation will be more complex
    // Get current rotor position from encoder
//as5600 get
    // uint16_t raw_angle = encoder.getRawAngle();
    // uint16_t corrected_angle = (4096 - raw_angle  + 156) % 4096; // 156 : min offset
    // float motor_angle = (corrected_angle / 4096.0f) * 2.0f * M_PI;
//openloop
    // static float motor_angle = 0;
    // motor_angle += 0.001 * 10; //10rad/s

    // motor_angle = fmod(motor_angle, 2.0f * M_PI);
    // float angle_el = motor_angle * pole_pairs ; // 不再强制设为0
    // angle_el = fmod(angle_el, 2.0f * M_PI);
    float angle_el = UpdateAngle();

    /*park transform*/
    float u_alpha = Ud*cos(angle_el)-Uq*sin(angle_el);
    float u_beta = Ud*sin(angle_el)+Uq*cos(angle_el);
    
    /*clarke transform*/
    Ua = u_alpha; // U phase
    Ub = -0.5f * u_alpha + (sqrt(3) / 2.0f) * u_beta; // V phase
    Uc = -0.5f * u_alpha - (sqrt(3) / 2.0f) * u_beta; // W phase
    // Apply PWM signals
#ifdef USE_VOFA
    // vofa_printf(UART_NUM_1,"%f,%f,%f,%f",motor_angle,Ua,Ub,Uc);
#endif   
    applyPWM(Ua, Ub, Uc);
}

void Motor::applyPWM(float u, float v, float w) {

// 将目标电压（-6V~+6V）映射到占空比（0~1.0）
    float _u = limit((u + 6.0f) / 12.0f,0.0f,1.0f);  // 替换原有公式
    float _v = limit((v + 6.0f) / 12.0f,0.0f,1.0f);
    float _w = limit((w + 6.0f) / 12.0f,0.0f,1.0f);
    if(!comparator_a)
    {
        ESP_LOGE("Motor", "cmpr is null;");
        return ;
    }
    if((uint32_t)(_u * PWM_CYCLE)> PWM_CYCLE)
    {
        ESP_LOGE("Motor", ">=PWM_CYCL");
        return ;
    }
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator_a, (uint32_t)(_u * PWM_CYCLE)));
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator_b, (uint32_t)(_v * PWM_CYCLE)));
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator_c, (uint32_t)(_w * PWM_CYCLE)));
}
/**
* @brief  
* @param  
* @return electrical angle for FOC 
*/
float Motor::UpdateAngle(void)
{
    uint16_t raw_angle = (4096 + direction * (encoder.getRawAngle() - offset)) % 4096;
    int16_t err_angle = raw_angle - pre_angle;
    if(abs(err_angle) > 3200) // 78% * 4096
    {
        // cnt update
        rotate_cnt += (err_angle>0) ? -1 : 1; 
    }
    float ratio = (raw_angle / 4096.0f);
    current_angle = (ratio + (float)rotate_cnt) * 360.0f;
    pre_angle = raw_angle;
    float ele_angle = fmod((ratio * 2.0f * M_PI) * pole_pairs, 2.0f * M_PI);
    return  ele_angle;
}

/**
* @brief  
* @param  
* @return 
*/
void Motor::SetTargetAngle(float target)
{
    target_angle = target;
    float out1 = loop_angle.PIDout(target,current_angle);
    focControl(out1);
}

/**
* @brief  
* @param  
* @return 
*/
void Motor::calibration_offset()
{
    offset = 156;
}