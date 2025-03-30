/*------------------------------------------------------------------------------
 * @file    PLATFORM_GPIO.H
 * @author  Hovey https://space.bilibili.com/33582262?spm_id_from=333.1007.0.0
 * @date    2025/03/30 22:31:27
 * @brief   
 * -----------------------------------------------------------------------------
 * @attention 
 
------------------------------------------------------------------------------*/

/* Define to prevent recursive inclusion  ------------------------------------*/
#ifndef __PLATFORM_GPIO_H
#define __PLATFORM_GPIO_H

/* Files includes  -----------------------------------------------------------*/
extern "C" {
    #include "../include/platform_config.h"
    #include "esp_log.h"
    #include "esp_check.h"  
    #include "driver/gpio.h"
}


/* Defines -------------------------------------------------------------------*/
#if USE_ESP32S3 == 1
#define PLATFORM_GPIO_PIN_0 GPIO_NUM_0
#define PLATFORM_GPIO_PIN_1 GPIO_NUM_1
#define PLATFORM_GPIO_PIN_2 GPIO_NUM_2
#define PLATFORM_GPIO_PIN_3 GPIO_NUM_3
#define PLATFORM_GPIO_PIN_4 GPIO_NUM_4
#define PLATFORM_GPIO_PIN_5 GPIO_NUM_5
#define PLATFORM_GPIO_PIN_6 GPIO_NUM_6
#define PLATFORM_GPIO_PIN_7 GPIO_NUM_7
#define PLATFORM_GPIO_PIN_8 GPIO_NUM_8
#define PLATFORM_GPIO_PIN_9 GPIO_NUM_9
#define PLATFORM_GPIO_PIN_10 GPIO_NUM_10
#define PLATFORM_GPIO_PIN_11 GPIO_NUM_11
#define PLATFORM_GPIO_PIN_12 GPIO_NUM_12
#define PLATFORM_GPIO_PIN_13 GPIO_NUM_13
#define PLATFORM_GPIO_PIN_14 GPIO_NUM_14
#define PLATFORM_GPIO_PIN_15 GPIO_NUM_15
#define PLATFORM_GPIO_PIN_16 GPIO_NUM_16
#define PLATFORM_GPIO_PIN_17 GPIO_NUM_17
#define PLATFORM_GPIO_PIN_18 GPIO_NUM_18
#define PLATFORM_GPIO_PIN_19 GPIO_NUM_19
#define PLATFORM_GPIO_PIN_20 GPIO_NUM_20
#define PLATFORM_GPIO_PIN_21 GPIO_NUM_21
#define PLATFORM_GPIO_PIN_22 GPIO_NUM_22
#define PLATFORM_GPIO_PIN_23 GPIO_NUM_23
#define PLATFORM_GPIO_PIN_24 GPIO_NUM_24
#define PLATFORM_GPIO_PIN_25 GPIO_NUM_25
#define PLATFORM_GPIO_PIN_26 GPIO_NUM_26
#define PLATFORM_GPIO_PIN_27 GPIO_NUM_27
#define PLATFORM_GPIO_PIN_28 GPIO_NUM_28
#define PLATFORM_GPIO_PIN_29 GPIO_NUM_29
#define PLATFORM_GPIO_PIN_30 GPIO_NUM_30
#define PLATFORM_GPIO_PIN_31 GPIO_NUM_31
#define PLATFORM_GPIO_PIN_32 GPIO_NUM_32
#define PLATFORM_GPIO_PIN_33 GPIO_NUM_33
#define PLATFORM_GPIO_PIN_34 GPIO_NUM_34
#define PLATFORM_GPIO_PIN_35 GPIO_NUM_35
#define PLATFORM_GPIO_PIN_36 GPIO_NUM_36
#define PLATFORM_GPIO_PIN_37 GPIO_NUM_37
#define PLATFORM_GPIO_PIN_38 GPIO_NUM_38
#define PLATFORM_GPIO_PIN_39 GPIO_NUM_39
#define PLATFORM_GPIO_PIN_40 GPIO_NUM_40
#define PLATFORM_GPIO_PIN_41 GPIO_NUM_41
#define PLATFORM_GPIO_PIN_42 GPIO_NUM_42
#define PLATFORM_GPIO_PIN_43 GPIO_NUM_43
#define PLATFORM_GPIO_PIN_44 GPIO_NUM_44
#define PLATFORM_GPIO_PIN_45 GPIO_NUM_45
#define PLATFORM_GPIO_PIN_46 GPIO_NUM_46
#define PLATFORM_GPIO_PIN_47 GPIO_NUM_47
#define PLATFORM_GPIO_PIN_48 GPIO_NUM_48
#define PLATFORM_GPIO_PIN_49 GPIO_NUM_49
#define PLATFORM_GPIO_PIN_50 GPIO_NUM_50
#define PLATFORM_GPIO_PIN_51 GPIO_NUM_51
#define PLATFORM_GPIO_PIN_52 GPIO_NUM_52
#define PLATFORM_GPIO_PIN_53 GPIO_NUM_53
#define PLATFORM_GPIO_PIN_54 GPIO_NUM_54
#endif

/* Variables -----------------------------------------------------------------*/


/* Functions ----------------------------------------------------------------*/

#endif
