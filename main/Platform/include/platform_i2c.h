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
#endif

// typedef enum{

// }
/* Variables -----------------------------------------------------------------*/


/* Functions ----------------------------------------------------------------*/
platform_err_t platform_i2c_init(i2c_port_t port, gpio_num_t sda_io_num, gpio_num_t scl_io_num, uint32_t clk_speed);

#endif
