/*------------------------------------------------------------------------------
 * @file    PLATFORM_ADC.H
 * @author  Hovey https://space.bilibili.com/33582262?spm_id_from=333.1007.0.0
 * @date    2025/03/29 17:33:35
 * @brief   这个文件用于定义Platform adc外设的接口，被driver层调用，包括初始化，ADC通道注册，ADC通道buffer读取。
 * -----------------------------------------------------------------------------
 * @attention 
 
------------------------------------------------------------------------------*/

/* Define to prevent recursive inclusion  ------------------------------------*/
#ifndef __PLATFORM_ADC_H
#define __PLATFORM_ADC_H

/* Files includes  -----------------------------------------------------------*/
extern "C"
{
    #include "../include/platform_config.h"
    #include "esp_log.h"
    #include "esp_check.h"  
    #include "esp_adc/adc_oneshot.h"
} 

/* Defines -------------------------------------------------------------------*/
#if USE_ESP32S3 == 1
#define  SOC_ADC_UNIT_NUM_MAX       (2)
#define  SOC_ADC_CHANNEL_NUM_MAX    (10)
#define  PLATFORM_ADC_1             ADC_UNIT_1
#define  PLATFORM_ADC_2             ADC_UNIT_2
#define  PLATFORM_ADC_CHANEL_0      ADC_CHANNEL_0
#define  PLATFORM_ADC_CHANEL_1      ADC_CHANNEL_1
#define  PLATFORM_ADC_CHANEL_2      ADC_CHANNEL_2
#define  PLATFORM_ADC_CHANEL_3      ADC_CHANNEL_3
#define  PLATFORM_ADC_CHANEL_4      ADC_CHANNEL_4
#define  PLATFORM_ADC_CHANEL_5      ADC_CHANNEL_5
#define  PLATFORM_ADC_CHANEL_6      ADC_CHANNEL_6
#define  PLATFORM_ADC_CHANEL_7      ADC_CHANNEL_7
#define  PLATFORM_ADC_CHANEL_8      ADC_CHANNEL_8
#define  PLATFORM_ADC_CHANEL_9      ADC_CHANNEL_9
typedef struct {
    bool cali_enabled;
    adc_channel_t index;
    // adc_oneshot_unit_handle_t sar_adc;
    adc_unit_t sar_adc_unit;
    adc_cali_handle_t cali_handle;
    int adc_rawdata;
} adc_channel_handle; 
#endif
// typedef adc_channel_handle* platform_adc_channel_t;



/* Variables -----------------------------------------------------------------*/


/* Functions ----------------------------------------------------------------*/
typedef enum {
    PLATFORM_ADC_OK = 0,
    PLATFORM_ADC_ERROR,
    PLATFORM_ADC_INVALID_ARG,
    PLATFORM_ADC_NOT_INIT,
    PLATFORM_ADC_TIMEOUT
} platform_adc_err_t;

typedef enum {
    PLATFORM_ADC_MODE_SINGLE,
    PLATFORM_ADC_MODE_CONTINUOUS
} platform_adc_mode_t;

/**
 * @brief Initialize ADC unit
 * @param unit ADC unit to initialize (PLATFORM_ADC_1 or PLATFORM_ADC_2)
 * @param mode ADC operation mode (single shot or continuous)
 * @return platform_adc_err_t PLATFORM_ADC_OK on success, error code otherwise
 */
platform_adc_err_t platform_adc_init(adc_unit_t unit, platform_adc_mode_t mode = PLATFORM_ADC_MODE_SINGLE);
/**
 * @brief Register an ADC channel
 * @param unit ADC unit (PLATFORM_ADC_1 or PLATFORM_ADC_2)
 * @param channel ADC channel to register
 * @param atten ADC attenuation level
 * @param bitwidth ADC bit width
 * @return platform_adc_err_t PLATFORM_ADC_OK on success, error code otherwise
 */
platform_adc_err_t platform_adc_register_channel(adc_unit_t unit, adc_channel_t channel_index, adc_channel_handle *channel,adc_atten_t atten= ADC_ATTEN_DB_12, adc_bitwidth_t bitwidth=ADC_BITWIDTH_12);

/**
 * @brief Read ADC channel value
 * @param unit ADC unit (PLATFORM_ADC_1 or PLATFORM_ADC_2)
 * @param channel ADC channel to read
 * @param out_value Pointer to store the read value
 * @return platform_adc_err_t PLATFORM_ADC_OK on success, error code otherwise
 */
platform_adc_err_t platform_adc_read(adc_unit_t unit, adc_channel_t channel, int *out_value);

/**
 * @brief Read multiple ADC channel values into buffer
 * @param unit ADC unit (PLATFORM_ADC_1 or PLATFORM_ADC_2)
 * @param channel ADC channel to read
 * @param buffer Pointer to buffer for storing values
 * @param buffer_size Size of the buffer
 * @return platform_adc_err_t PLATFORM_ADC_OK on success, error code otherwise
 */
platform_adc_err_t platform_adc_get_voltage(adc_channel_handle *channel, int *voltage);

/**
 * @brief Free an ADC channel
 * @param channel ADC channel to free
 * @return void
 */
// void platform_adc_free_channel(platform_adc_channel_t channel);
#endif
