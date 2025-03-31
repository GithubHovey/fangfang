/*------------------------------------------------------------------------------
 * @file    PLATFORM_I2C.H
 * @author  Hovey https://space.bilibili.com/33582262?spm_id_from=333.1007.0.0
 * @date    2025/03/30 22:16:21
 * @brief   
 * -----------------------------------------------------------------------------
 * @attention 
 
------------------------------------------------------------------------------*/

/* Define to prevent recursive inclusion  ------------------------------------*/
#ifndef __PLATFORM_I2C_H
#define __PLATFORM_I2C_H

/* Files includes  -----------------------------------------------------------*/

extern "C"
{
    #include "platform_config.h"
    #include "platform_err.h"
    #include "platform_gpio.h"
    #include "driver/i2c_master.h"
    #include "esp_log.h"
    #include "esp_check.h"  
} 
/* Defines -------------------------------------------------------------------*/
#if USE_ESP32S3 == 1
#define PLATFORM_I2C_NUM_0  I2C_NUM_0
#define PLATFORM_I2C_NUM_1  I2C_NUM_1
#define PLATFORM_I2C1_SCL_IO   GPIO_NUM_7
#define PLATFORM_I2C1_SDA_IO   GPIO_NUM_6
#define PLATFORM_I2C1_CLK_SPEED  100000
#endif

typedef struct _platform_i2c_dev{
    i2c_master_dev_handle_t dev_handle;
}platform_i2c_dev;

/* Variables -----------------------------------------------------------------*/


/* Functions ----------------------------------------------------------------*/
platform_err_t platform_i2c_init(i2c_port_t port);

platform_err_t platform_i2c_register_device(i2c_port_t port, uint8_t dev_addr, platform_i2c_dev *i2c_dev_handle);

platform_err_t platform_i2c_write(platform_i2c_dev *i2c_dev_handle, uint8_t reg, uint8_t *data, size_t size);

platform_err_t platform_i2c_read(platform_i2c_dev *i2c_dev_handle, uint8_t reg ,uint8_t *data, size_t size);

#endif
