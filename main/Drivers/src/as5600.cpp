/*------------------------------------------------------------------------------
 * @file    AS5600.Cpp
 * @author  Hovey https://space.bilibili.com/33582262?spm_id_from=333.1007.0.0
 * @date    2025/03/08 02:29:42
 * @brief   
 * -----------------------------------------------------------------------------
 * @attention 
 
------------------------------------------------------------------------------*/
#include "as5600.h"

AS5600::AS5600(i2c_port_t i2c_port)
    : i2c_port(i2c_port),i2c_dev(nullptr) 
    {
        // init();
    }

AS5600::~AS5600() {
//     if (dev_handle) {
//         i2c_master_bus_rm_device(dev_handle);
//     }
//     if (bus_handle) {
//         i2c_del_master_bus(bus_handle);
//     }
}

bool AS5600::init() {
    return i2c_init();
}

bool AS5600::i2c_init() {

    platform_err_t ret = platform_i2c_register_device(i2c_port,AS5600_I2C_ADDRESS,&i2c_dev);
    if(ret != PLATFORM_OK) {
        return false;
    }
    return true;
}

bool AS5600::readRegister(uint8_t reg, uint8_t *data, size_t len) {
    platform_err_t ret = platform_i2c_read(&i2c_dev, reg ,data, len);
    if(ret != PLATFORM_OK) {
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
