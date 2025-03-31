 /*------------------------------------------------------------------------------
  * @file    PLATFORM_ADC.C
  * @author  Hovey https://space.bilibili.com/33582262?spm_id_from=333.1007.0.0
  * @date    2025/03/29 17:21:30
  * @brief   
  * -----------------------------------------------------------------------------
  * @attention 
  
-----------------------------------------------------------------------------*/
#include "../include/platform_adc.h"
#if USE_PLATFORM_ADC == 1
// static platform_adc_handle adc_handle[SOC_ADC_UNIT_NUM_MAX];
adc_oneshot_unit_handle_t adc_handle[SOC_ADC_UNIT_NUM_MAX];

/**
  * @brief  Initialize ADC peripheral
  * @param  unit ADC unit to initialize (PLATFORM_ADC_1 or PLATFORM_ADC_2)
  * @param  mode ADC operation mode (single shot or continuous)
  * @retval platform_adc_err_t PLATFORM_ADC_OK on success, error code otherwise
  */
platform_adc_err_t platform_adc_init(adc_unit_t unit, platform_adc_mode_t mode)
{
    /* ADC initialization code */
    if(unit != PLATFORM_ADC_1 && unit != PLATFORM_ADC_2)
    {
        return PLATFORM_ADC_INVALID_ARG;
    }
    // Check if ADC handle is already initialized
    if(mode == PLATFORM_ADC_MODE_SINGLE)
    {
        if(adc_handle[unit] != NULL)
        {
          return PLATFORM_ADC_OK;
        }
        adc_oneshot_unit_init_cfg_t init_config = {
          .unit_id = unit,
        };
        ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &adc_handle[unit]));
    }
    return PLATFORM_ADC_OK;
}


static bool adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle)
{
    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;
    bool calibrated = false;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    if (!calibrated) {
        ESP_LOGI("motor", "calibration scheme version is %s", "Curve Fitting");
        adc_cali_curve_fitting_config_t cali_config = {
            .unit_id = unit,
            .chan = channel,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }
#endif

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    if (!calibrated) {
        ESP_LOGI("motor", "calibration scheme version is %s", "Line Fitting");
        adc_cali_line_fitting_config_t cali_config = {
            .unit_id = unit,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_line_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }
#endif

    *out_handle = handle;
    if (ret == ESP_OK) {
        ESP_LOGI("motor", "Calibration Success");
    } else if (ret == ESP_ERR_NOT_SUPPORTED || !calibrated) {
        ESP_LOGW("motor", "eFuse not burnt, skip software calibration");
    } else {
        ESP_LOGE("motor", "Invalid arg or no memory");
    }

    return calibrated;
}



/**
  * @brief  Register an ADC channel
  * @param  unit ADC unit (PLATFORM_ADC_1 or PLATFORM_ADC_2)
  * @param  channel ADC channel to register
  * @param  atten ADC attenuation level
  * @param  bitwidth ADC bit width
  * @retval platform_adc_err_t PLATFORM_ADC_OK on success, error code otherwise
  */
platform_adc_err_t platform_adc_register_channel(adc_unit_t unit, adc_channel_t channel_index, adc_channel_handle *channel,
  adc_atten_t atten, adc_bitwidth_t bitwidth)
{
  // 检查参数有效性
  if (adc_handle[unit] != NULL || channel_index >= SOC_ADC_CHANNEL_NUM_MAX || !channel) {
    return PLATFORM_ADC_INVALID_ARG;
  }
  if(platform_adc_init(unit) != PLATFORM_ADC_OK) {
    return PLATFORM_ADC_ERROR;
  }
  // 配置通道
  adc_oneshot_chan_cfg_t config = {
  .atten = atten,
  .bitwidth = bitwidth,
  };
  ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle[unit], channel_index, &config));

  // 初始化校准
  channel->cali_enabled = adc_calibration_init(unit, channel_index, atten, &channel->cali_handle);
  channel->index = channel_index;
  channel->sar_adc_unit = unit;

  return PLATFORM_ADC_OK;
}

/**
  * @brief  Read multiple ADC channel values into buffer
  * @param  unit ADC unit (PLATFORM_ADC_1 or PLATFORM_ADC_2)
  * @param  channel ADC channel to read
  * @param  buffer Pointer to buffer for storing values
  * @param  buffer_size Size of the buffer
  * @retval platform_adc_err_t PLATFORM_ADC_OK on success, error code otherwise
  */
platform_adc_err_t platform_adc_get_voltage(adc_channel_handle *channel, int *voltage)
{
    /* ADC buffer read code */
    if(adc_handle[channel->sar_adc_unit] == nullptr)
    {
        ESP_LOGE("platform_adc", "adc_handle is null");
        return PLATFORM_ADC_ERROR;
    }
    
    ESP_ERROR_CHECK(adc_oneshot_read(adc_handle[channel->sar_adc_unit], channel->index, &channel->adc_rawdata));
    ESP_LOGI("Motor", "ADC%d Channel[%d] Raw Data: %d", channel->sar_adc_unit + 1, channel->index, channel->adc_rawdata);
    if (channel->cali_enabled) {
    ESP_ERROR_CHECK(adc_cali_raw_to_voltage(channel->cali_handle, channel->adc_rawdata, voltage));
    ESP_LOGI("Motor", "ADC%d Channel[%d] Cali Voltage: %d mV", channel->sar_adc_unit + 1, channel->index, *voltage);
    }
    return PLATFORM_ADC_OK;
}

/*
  * @param  None
  * @retval None
  * @brief  Read ADC channel value
  * @param  unit ADC unit (PLATFORM_ADC_1 or PLATFORM_ADC_2)
  * @param  channel ADC channel to read
  * @param  out_value Pointer to store the read value
  * @retval platform_adc_err_t PLATFORM_ADC_OK on success, error code otherwise
  */
platform_adc_err_t platform_adc_read(adc_unit_t unit, adc_channel_t channel, int *out_value)
{
    /* ADC deinitialization code */
    /* ADC value read code */
    return PLATFORM_ADC_OK;
}



#endif