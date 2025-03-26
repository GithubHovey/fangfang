/*------------------------------------------------------------------------------
 * @file    LOWPASSFILTER.H
 * @author  Hovey https://space.bilibili.com/33582262?spm_id_from=333.1007.0.0
 * @date    2025/03/26 20:51:22
 * @brief   
 * -----------------------------------------------------------------------------
 * @attention 
 
------------------------------------------------------------------------------*/

/* Define to prevent recursive inclusion  ------------------------------------*/
#ifndef __LOWPASSFILTER_H
#define __LOWPASSFILTER_H

/* Files includes  -----------------------------------------------------------*/


/* Defines -------------------------------------------------------------------*/


/* Variables -----------------------------------------------------------------*/


/* Functions ----------------------------------------------------------------*/
template <typename T>
T simpleLowPass(T input,float Tf,float dt) {
    static float prevOutput = 0;
    static int64_t pretimestamp = 0;
    int64_t timestamp = esp_timer_get_time();
    float dt = (timestamp - pretimestamp)*(1e-6f);
    if((dt>0.3f)||(dt < 0.0f))
    {
        prevOutput = input;
        pretimestamp = timestamp;
        return input;
    }

    float alpha = Tf/(Tf + dt);
    float output = alpha*prevOutput + (1.0f-alpha)*input;
    prevOutput = output;
    return prevOutput;
}
#endif

