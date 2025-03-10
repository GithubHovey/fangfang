/*------------------------------------------------------------------------------
 * @file    AS5600.Cpp
 * @author  Hovey https://space.bilibili.com/33582262?spm_id_from=333.1007.0.0
 * @date    2025/03/08 02:29:42
 * @brief   
 * -----------------------------------------------------------------------------
 * @attention 
 
------------------------------------------------------------------------------*/
#include "as5600.h"

AS5600::AS5600(i2c_port_t i2c_port, gpio_num_t sda_pin, gpio_num_t scl_pin)
    : i2c_port(i2c_port), sda_pin(sda_pin), scl_pin(scl_pin), bus_handle(nullptr), dev_handle(nullptr) 
    {
        // init();
    }

AS5600::~AS5600() {
    if (dev_handle) {
        i2c_master_bus_rm_device(dev_handle);
    }
    if (bus_handle) {
        i2c_del_master_bus(bus_handle);
    }
}

bool AS5600::init() {
    return i2c_init();
}

bool AS5600::i2c_init() {
    i2c_master_bus_config_t bus_config = {}; // 初始化所有字段为 0
    bus_config.i2c_port = i2c_port;
    bus_config.sda_io_num = sda_pin;
    bus_config.scl_io_num = scl_pin;
    bus_config.clk_source = I2C_CLK_SRC_DEFAULT;
    bus_config.glitch_ignore_cnt = 7;
    bus_config.flags.enable_internal_pullup = true;

    esp_err_t err = i2c_new_master_bus(&bus_config, &bus_handle);
    if (err != ESP_OK) {
        ESP_LOGE("AS5600", "Failed to initialize I2C bus: %s", esp_err_to_name(err));
        return false;
    }

    i2c_device_config_t dev_config = {}; // 初始化所有字段为 0
    dev_config.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dev_config.device_address = AS5600_I2C_ADDRESS;
    dev_config.scl_speed_hz = 100000;

    err = i2c_master_bus_add_device(bus_handle, &dev_config, &dev_handle);
    if (err != ESP_OK) {
        ESP_LOGE("AS5600", "Failed to add I2C device: %s", esp_err_to_name(err));
        return false;
    }
    ESP_LOGI("AS5600", "Init finish!");
    return true;
}

bool AS5600::readRegister(uint8_t reg, uint8_t *data, size_t len) {
    esp_err_t err = i2c_master_transmit_receive(dev_handle, &reg, 1, data, len, -1);
    if (err != ESP_OK) {
        ESP_LOGE("AS5600", "Failed to read register: %s", esp_err_to_name(err));
        return false;
    }
    return true;
}

uint16_t AS5600::getRawAngle() {
    uint8_t data[2];
    if (readRegister(AS5600_REG_RAW_ANGLE, data, 2)) {
        
        return (data[0] << 8) | data[1];
    }
    return 0;
}
