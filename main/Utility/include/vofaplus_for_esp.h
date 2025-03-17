/*------------------------------------------------------------------------------
 * @file    VOFAPULS_FOR_ESP.H
 * @author  Hovey https://space.bilibili.com/33582262?spm_id_from=333.1007.0.0
 * @date    2025/03/17 21:48:01
 * @brief   
 * -----------------------------------------------------------------------------
 * @attention 
 
------------------------------------------------------------------------------*/

/* Define to prevent recursive inclusion  ------------------------------------*/
#ifndef __VOFAPULS_FOR_ESP_H
#define __VOFAPULS_FOR_ESP_H

/* Files includes  -----------------------------------------------------------*/
#ifdef __cplusplus
extern "C"
{
#endif

#include "driver/uart.h"
#include "esp_log.h"
/* Defines -------------------------------------------------------------------*/

/* Variables -----------------------------------------------------------------*/


class Vofa {
public:
    Vofa(uart_port_t uart_num, int tx_pin, int rx_pin); // 构造函数，传递UART端口号和引脚
    ~Vofa(); // 析构函数
    void begin(int baud_rate); // 初始化UART通信
    void sendData(float data[], int length); // 发送数据到Vofa+

private:
    uart_port_t uart_num; // UART端口号
    int tx_pin; // TX引脚
    int rx_pin; // RX引脚
};



#ifdef __cplusplus
}
#endif
#endif
