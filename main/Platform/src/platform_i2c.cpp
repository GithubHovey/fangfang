/*------------------------------------------------------------------------------
 * @file    PLATFORM_I2C.C
 * @author  Hovey https://space.bilibili.com/33582262?spm_id_from=333.1007.0.0
 * @date    2025/03/29 17:40:39
 * @brief   
 * -----------------------------------------------------------------------------
 * @attention 
 
------------------------------------------------------------------------------*/
#include "../include/platform_i2c.h"
#if USE_PLATFORM_I2C == 1
i2c_master_bus_handle_t i2c_bus_handle[2];
//i2c master init
platform_err_t platform_i2c_init(i2c_port_t port, gpio_num_t sda_io_num, gpio_num_t scl_io_num, uint8_t dev_addr uint32_t clk_speed = 100000)
{
    i2c_master_bus_config_t bus_config = {}; // 初始化所有字段为 0
    bus_config.i2c_port = port;
    bus_config.sda_io_num = sda_io_num;
    bus_config.scl_io_num = scl_io_num;
    bus_config.clk_source = I2C_CLK_SRC_DEFAULT;
    bus_config.glitch_ignore_cnt = 7;
    bus_config.flags.enable_internal_pullup = true;

    esp_err_t err = i2c_new_master_bus(&bus_config, &i2c_bus_handle[port]);
    if (err != ESP_OK) {
        ESP_LOGE("I2C", "Failed to initialize I2C bus: %s", esp_err_to_name(err));
        return PLATFORM_ERROR;
    }

    i2c_device_config_t dev_config = {}; // 初始化所有字段为 0
    dev_config.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dev_config.device_address = dev_addr;
    dev_config.scl_speed_hz = 100000;

    err = i2c_master_bus_add_device(i2c_bus_handle[port], &dev_config, &dev_handle);
    if (err != ESP_OK) {
        ESP_LOGE("AS5600", "Failed to add I2C device: %s", esp_err_to_name(err));
        return PLATFORM_ERROR;
    }
    ESP_LOGI("AS5600", "Init finish!");
    return PLATFORM_OK;
}



#endif