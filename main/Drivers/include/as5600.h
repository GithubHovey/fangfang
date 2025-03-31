/*------------------------------------------------------------------------------
 * @file    AS5600.H
 * @author  Hovey https://space.bilibili.com/33582262?spm_id_from=333.1007.0.0
 * @date    2025/03/08 02:29:37
 * @brief   
 * -----------------------------------------------------------------------------
 * @attention 
 
------------------------------------------------------------------------------*/

/* Define to prevent recursive inclusion  ------------------------------------*/
#ifndef AS5600_H
#define AS5600_H

// #include "driver/i2c_master.h"
// #include "driver/gpio.h"
// #include "esp_log.h"
#include "platform.h"

#define AS5600_I2C_ADDRESS 0x36
#define AS5600_REG_RAW_ANGLE 0x0C

class AS5600 {
public:
    AS5600(i2c_port_t i2c_port);
    ~AS5600();

    bool init();
    uint16_t getRawAngle();

private:
    i2c_port_t i2c_port;
    // gpio_num_t sda_pin;
    // gpio_num_t scl_pin;
    // i2c_master_bus_handle_t bus_handle;
    // i2c_master_dev_handle_t dev_handle;
    platform_i2c_dev i2c_dev;
    bool i2c_init();
    bool readRegister(uint8_t reg, uint8_t *data, size_t len);
};

#endif // AS5600_H
