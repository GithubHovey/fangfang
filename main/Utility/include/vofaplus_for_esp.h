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


#include "driver/uart.h"
#include "esp_log.h"

#ifdef __cplusplus
extern "C" {
#endif

// 初始化 Vofa+ 的 UART
void vofa_init(uart_port_t uart_num, int tx_pin, int rx_pin, int baud_rate);

// 发送数据到 Vofa+（FireWater 协议）
void vofa_send_firewater(uart_port_t uart_num, float data[], int length);
void vofa_send_justfloat(uart_port_t uart_num, float data[], int length);
void vofa_printf(uart_port_t uart_num, const char* format, ...);

#ifdef __cplusplus
}
#endif

#endif // VOFA_H