/*------------------------------------------------------------------------------
 * @file    PLATFORM.H
 * @author  Hovey https://space.bilibili.com/33582262?spm_id_from=333.1007.0.0
 * @date    2025/03/29 17:29:34
 * @brief   
 * -----------------------------------------------------------------------------
 * @attention 
 
------------------------------------------------------------------------------*/

/* Define to prevent recursive inclusion  ------------------------------------*/
#ifndef __PLATFORM_H
#define __PLATFORM_H

/* Files includes  -----------------------------------------------------------*/
#include "platform_config.h"

#if USE_PLATFORM_ADC == 1
#include "platform_adc.h"
#endif

#if USE_PLATFORM_UART == 1
#include "platform_uart.h"
#endif

#if USE_PLATFORM_I2C == 1
#include "platform_i2c.h"
#endif

#if USE_PLATFORM_TIMER == 1
#include "platform_timer.h"
#endif

#include "esp_log.h"
/* Defines -------------------------------------------------------------------*/


/* Variables -----------------------------------------------------------------*/
void PlatformInit();

/* Functions ----------------------------------------------------------------*/

#endif
