/*------------------------------------------------------------------------------
 * @file    VOFAPLUS_FOR_ESP.CPP
 * @author  Hovey https://space.bilibili.com/33582262?spm_id_from=333.1007.0.0
 * @date    2025/03/17 21:47:36
 * @brief   
 * -----------------------------------------------------------------------------
 * @attention 
 
------------------------------------------------------------------------------*/
#include "../include/vofaplus_for_esp.h"

Vofa::Vofa(uart_port_t uart_num, int tx_pin, int rx_pin) {
    this->uart_num = uart_num;
    this->tx_pin = tx_pin;
    this->rx_pin = rx_pin;
}

Vofa::~Vofa() {
    uart_driver_delete(uart_num); // 删除UART驱动
}

void Vofa::begin(int baud_rate) {
    // UART配置
    uart_config_t uart_config = {
        .baud_rate = baud_rate,
        .data_bits = UART_DATA_8_BITS, 
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    // 安装UART驱动
    ESP_ERROR_CHECK(uart_driver_install(uart_num, 1024, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(uart_num, tx_pin, rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
}

void Vofa::sendData(float data[], int length) {
    // Vofa+的协议通常以0x00 0x00 0x80 0x7F结尾
    for (int i = 0; i < length; i++) {
        uint8_t* bytes = (uint8_t*)&data[i];
        uart_write_bytes(uart_num, (const char*)bytes, sizeof(float));
    }
    // 发送结束标志
    uint8_t end_marker[] = {0x00, 0x00, 0x80, 0x7F};
    uart_write_bytes(uart_num, (const char*)end_marker, sizeof(end_marker));
}
/*demo:*/
#if 0 
#include "Vofa.h"

extern "C" void app_main() {
    Vofa vofa(UART_NUM_1, GPIO_NUM_17, GPIO_NUM_16); // 使用UART1，TX=GPIO17，RX=GPIO16
    vofa.begin(115200); // 初始化UART通信，波特率为115200

    while (1) {
        float data[] = {1.23, 4.56, 7.89}; // 要发送的数据
        vofa.sendData(data, 3); // 发送数据
        vTaskDelay(1000 / portTICK_PERIOD_MS); // 每隔1秒发送一次
    }
}
#endif