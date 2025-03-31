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
platform_err_t platform_i2c_init(i2c_port_t port)
{
    i2c_master_bus_config_t bus_config = {}; // 初始化所有字段为 0
    bus_config.i2c_port = port;
    bus_config.sda_io_num = PLATFORM_I2C1_SDA_IO;
    bus_config.scl_io_num = PLATFORM_I2C1_SCL_IO;
    bus_config.clk_source = I2C_CLK_SRC_DEFAULT;
    bus_config.glitch_ignore_cnt = 7;
    bus_config.flags.enable_internal_pullup = true;

    esp_err_t err = i2c_new_master_bus(&bus_config, &i2c_bus_handle[port]);
    if (err != ESP_OK) {
        ESP_LOGE("I2C", "Failed to initialize I2C bus: %s", esp_err_to_name(err));
        return PLATFORM_ERROR;
    }
    return PLATFORM_OK;  // Return PLATFORM_OK if initialization is successful.
    
}
platform_err_t platform_i2c_register_device(i2c_port_t port, uint8_t dev_addr, platform_i2c_dev *i2c_dev_handle)
{
    // Check if the I2C bus is already initialized on the given port. If not, initialize it.
    if(i2c_bus_handle[port] != NULL)
    {
        ESP_LOGW("platform_i2c_register_device", "I2C bus already initialized on port %d", port);
        return PLATFORM_OK;
    }
    else
    {
        if(platform_i2c_init(port) != PLATFORM_OK)
        {
            ESP_LOGE("platform_i2c_register_device", "Failed to initialize I2C bus on port %d", port);
            return PLATFORM_ERROR;
        }
        ESP_LOGI("platform_i2c_register_device", "Device registered successfully on port %d", port);
        i2c_device_config_t dev_config = {}; // 初始化所有字段为 0

        dev_config.dev_addr_length = I2C_ADDR_BIT_LEN_7;
        dev_config.device_address = dev_addr;
        dev_config.scl_speed_hz = 100000;

        if(i2c_master_bus_add_device(i2c_bus_handle[port], &dev_config, &i2c_dev_handle->dev_handle)!= ESP_OK)
        {
            ESP_LOGE("platform_i2c_register_device", "Failed to add I2C device");
            return PLATFORM_ERROR;
        }
        ESP_LOGI("platform_i2c_register_device", "Init finish!");
        return PLATFORM_OK;

    }
}

platform_err_t platform_i2c_read(platform_i2c_dev *i2c_dev_handle, uint8_t reg, uint8_t *data, size_t size)
{
    esp_err_t err = i2c_master_transmit_receive(i2c_dev_handle->dev_handle, &reg, 1, data, size, -1);
    if (err != ESP_OK) {
        ESP_LOGE("platform_i2c_read", "Failed to read register: %s", esp_err_to_name(err));
        return PLATFORM_ERROR;
    }
    return PLATFORM_OK;
}
#endif