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

#define limit(value, min, max) ((value) < (min) ? (min) : ((value) > (max) ? (max) : (value)))
#define PWM_CYCLE 1000 /*us*/
Motor::Motor(int mcpwm_unit, gpio_num_t pwm_a, gpio_num_t pwm_b, gpio_num_t pwm_c,
             i2c_port_t i2c_port, gpio_num_t sda_pin, gpio_num_t scl_pin ,uint8_t pole_pairs)
    : pole_pairs(6),mcpwm_unit(mcpwm_unit), pwm_a(pwm_a), pwm_b(pwm_b), pwm_c(pwm_c),
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
        .resolution_hz = 1000000, // 1 MHz resolution
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
}

void Motor::setSpeed(float speed) {
    target_speed = speed;
}

void Motor::setTorque(float torque) {
    target_torque = torque;
}

void Motor::update() {
    // uint32_t count = 0;
    // ESP_ERROR_CHECK(mcpwm_timer_get_count(mcpwm_timer, &count));
    // ESP_LOGI("Motor", "Timer count: %lu", count);
    // focControl(1.0f,0);
    SetRotateVoltage(12.0f);
}
void Motor::debug() {
    int level_a = gpio_get_level((gpio_num_t)pwm_a);
    int level_b = gpio_get_level((gpio_num_t)pwm_b);
    int level_c = gpio_get_level((gpio_num_t)pwm_c);
    ESP_LOGI("Motor", "GPIO A: %d, B: %d, C: %d", level_a, level_b, level_c);
}
void Motor::applyPWM(float u, float v, float w) {
    // Apply PWM signals to the motor phases
    // Note: This is a placeholder. You need to map u, v, w to the actual PWM duty cycles.
    float _u = limit((u+max_voltage/2)/(2*max_voltage) ,0.0f,1.0f); //e.g 6v : (6+12)/(2*12) = 75%
    float _v = limit((v+max_voltage/2)/(2*max_voltage) ,0.0f,1.0f); //e.g -12v : (-12+12)/(2*12) = 0%
    float _w = limit((w+max_voltage/2)/(2*max_voltage) ,0.0f,1.0f); //e.g 0v : (0+12)/(2*12) = 50%

    if(!comparator_a)
    {
        ESP_LOGE("Motor", "cmpr is null;");
        return ;
    }
    if((uint32_t)(_u * PWM_CYCLE)>= PWM_CYCLE)
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
    uint16_t raw_angle = encoder.getRawAngle();
    float rotor_angle = (raw_angle / 4096.0f) * 2 * M_PI; // Convert to radians
    float angle_el = rotor_angle * pole_pairs;
    // Calculate Clarke and Park transforms
    // Placeholder for actual FOC logic
    /*park transform*/
    float u_alpha = Ud*cos(angle_el)-Uq*sin(angle_el);
    float u_beta = Ud*sin(angle_el)+Uq*cos(angle_el);
    
    /*clarke transform*/
    float u = u_alpha; // U phase
    float v = -0.5f * u_alpha + (sqrt(3) / 2.0f) * u_beta; // V phase
    float w = -0.5f * u_alpha - (sqrt(3) / 2.0f) * u_beta; // W phase
    // Apply PWM signals
    applyPWM(u, v, w);
}
void Motor::SetRotateVoltage(float V)
{
    focControl(V);
}