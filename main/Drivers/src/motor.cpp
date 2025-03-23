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
Motor::Motor(int mcpwm_unit, gpio_num_t pwm_a, gpio_num_t pwm_b, gpio_num_t pwm_c,
             i2c_port_t i2c_port, gpio_num_t sda_pin, gpio_num_t scl_pin ,uint8_t pole_pairs)
    : pole_pairs(pole_pairs),mcpwm_unit(mcpwm_unit), pwm_a(pwm_a), pwm_b(pwm_b), pwm_c(pwm_c),
      encoder(i2c_port, sda_pin, scl_pin), // Initialize AS5600 encoder
      max_voltage(12.0f),current_speed(0), target_speed(0), current_torque(0), target_torque(0) {
}

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
    vofa_init(UART_NUM_1, GPIO_NUM_38, GPIO_NUM_39, 115200);
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
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc_handle));
    
    adc_oneshot_chan_cfg_t config = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, ADC_CHANNEL_8, &config));//gpio9
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, ADC_CHANNEL_9, &config));//gpio10
    adc_cali1_enable = adc_calibration_init(ADC_UNIT_1, ADC_CHANNEL_8, ADC_ATTEN_DB_12, &adc_cali1_handle);
    adc_cali2_enable = adc_calibration_init(ADC_UNIT_1, ADC_CHANNEL_9, ADC_ATTEN_DB_12, &adc_cali2_handle);
}

void Motor::setSpeed(float speed) {
    target_speed = speed;
}

void Motor::setTorque(float torque) {
    target_torque = torque;
}

void Motor::update() {
    // SetRotateVoltage(4.0f);
    focControl(1.0f,0);
    // debug();
}
void Motor::debug() {
    // if(adc_handle == nullptr)
    // {
    //     ESP_LOGE("Motor", "adc_handle is null");
    //     return ;
    // }
    
    // ESP_ERROR_CHECK(adc_oneshot_read(adc_handle, ADC_CHANNEL_8, &adc_rawdata));
    // ESP_LOGI("Motor", "ADC%d Channel[%d] Raw Data: %d", ADC_UNIT_1 + 1, ADC_CHANNEL_8, adc_rawdata);
    // if (adc_cali1_enable) {
    // ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc_cali1_handle, adc_rawdata, &voltage));
    // ESP_LOGI("Motor", "ADC%d Channel[%d] Cali Voltage: %d mV", ADC_UNIT_1 + 1, ADC_CHANNEL_8, voltage);
    // }

    // 手动设定三相电压（示例：旋转电压矢量）
    static float test_angle = 0.0f;
    Ua = sin(test_angle);
    Ub = sin(test_angle + 2*M_PI/3);
    Uc = sin(test_angle - 2*M_PI/3);
    test_angle += 0.1f; // 每次控制周期增加0.1rad
    applyPWM(Ua, Ub, Uc);
}
void Motor::applyPWM(float u, float v, float w) {
#if 0
    float _u = limit((u+max_voltage)/(2*max_voltage) ,0.0f,1.0f); //e.g 6v : (6+12)/(2*12) = 75%
    float _v = limit((v+max_voltage)/(2*max_voltage) ,0.0f,1.0f); //e.g -12v : (-12+12)/(2*12) = 0%
    float _w = limit((w+max_voltage)/(2*max_voltage) ,0.0f,1.0f); //e.g 0v : (0+12)/(2*12) = 50%
#else
// 将目标电压（-6V~+6V）映射到占空比（0~1.0）
    float _u = limit((u + 6.0f) / 12.0f,0.0f,1.0f);  // 替换原有公式
    float _v = limit((v + 6.0f) / 12.0f,0.0f,1.0f);
    float _w = limit((w + 6.0f) / 12.0f,0.0f,1.0f);
#endif
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

void Motor::focControl(float Uq, float Ud = 0.0f) { 
    // Implement FOC control logic here
    // This is a simplified example, actual FOC implementation will be more complex
    // Get current rotor position from encoder

    // uint16_t raw_angle = encoder.getRawAngle();
    // uint16_t corrected_angle = (raw_angle + 4096 - 156) % 4096; // 156 : min offset
    // float motor_angle = (corrected_angle / 4096.0f) * 2.0f * M_PI;
//openloop
    static float motor_angle = 0;
    motor_angle += 0.001 * 10; //10rad/s


    motor_angle = fmod(motor_angle, 2.0f * M_PI);
    float angle_el = motor_angle * pole_pairs ; // 不再强制设为0
    angle_el = fmod(angle_el, 2.0f * M_PI);
    // float debug_el = motor_angle*pole_pairs;
    // angle_el = 0.0f ;
    // ESP_LOGI("Motor", "raw_angle = %d ",raw_angle);

    // Calculate Clarke and Park transforms
    // Placeholder for actual FOC logic
    /*park transform*/
    float u_alpha = Ud*cos(angle_el)-Uq*sin(angle_el);
    float u_beta = Ud*sin(angle_el)+Uq*cos(angle_el);
    
    /*clarke transform*/
    Ua = u_alpha; // U phase
    Ub = -0.5f * u_alpha + (sqrt(3) / 2.0f) * u_beta; // V phase
    Uc = -0.5f * u_alpha - (sqrt(3) / 2.0f) * u_beta; // W phase
    // Apply PWM signals
#ifdef USE_VOFA
    // float vofadata[3] = {_u, _v, _w};
    // vofa_printf(UART_NUM_1, "%.2f,%.2f,%.2f", _u, _v, _w);
    // ESP_LOGE("Motor", "angle_el=%f,u_alpha=%f,u_beta=%f,Ua=%f,Ub=%f,Uc=%f",angle_el,u_alpha,u_beta,Ua,Ub,Uc);
    // vofa_printf(UART_NUM_1,"%d,%f,%f,%f,%f",motor_angle,Ua,Ub,Uc);
    // vofa_send_firewater(UART_NUM_1, &raw_angle, 1);
#endif   
    applyPWM(Ua, Ub, Uc);
}
void Motor::SetRotateVoltage(float V)
{
    focControl(V);
}