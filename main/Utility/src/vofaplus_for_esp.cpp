/*------------------------------------------------------------------------------
 * @file    VOFAPLUS_FOR_ESP.CPP
 * @author  Hovey https://space.bilibili.com/33582262?spm_id_from=333.1007.0.0
 * @date    2025/03/17 21:47:36
 * @brief   
 * -----------------------------------------------------------------------------
 * @attention 
 
------------------------------------------------------------------------------*/
#include "../include/vofaplus_for_esp.h"

// 初始化 UART
void vofa_init(uart_port_t uart_num, int tx_pin, int rx_pin, int baud_rate) {
    // UART 配置
    uart_config_t uart_config = {
        .baud_rate = baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    // 安装 UART 驱动
    ESP_ERROR_CHECK(uart_driver_install(uart_num, 1024, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(uart_num, tx_pin, rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
}

// 发送数据到 Vofa+（FireWater 协议）
void vofa_send_firewater(uart_port_t uart_num, float data[], int length) {
    char buffer[128]; // 缓冲区，根据数据长度调整大小
    int offset = 0;   // 缓冲区偏移量

    // 将浮点数据转换为字符串，以逗号分隔
    for (int i = 0; i < length; i++) {
        offset += snprintf(buffer + offset, sizeof(buffer) - offset, "%.2f,", data[i]);
    }

    // 替换最后一个逗号为换行符
    if (offset > 0) {
        buffer[offset - 1] = '\n'; // 将最后一个逗号替换为换行符
    }

    // 发送数据
    uart_write_bytes(uart_num, buffer, offset);
}
// 发送数据到 Vofa+（JustFloat 协议）
void vofa_send_justfloat(uart_port_t uart_num, float data[], int length) {
    // 发送浮点数据
    for (int i = 0; i < length; i++) {
        uint8_t* bytes = (uint8_t*)&data[i]; // 将浮点数转换为字节数组
        uart_write_bytes(uart_num, (const char*)bytes, sizeof(float));
    }

    // 发送结束标志
    uint8_t end_marker[] = {0x00, 0x00, 0x80, 0x7F};
    uart_write_bytes(uart_num, (const char*)end_marker, sizeof(end_marker));
}

void vofa_printf(uart_port_t uart_num, const char* format, ...) {
    char buffer[128]; // 缓冲区，根据数据长度调整大小

    // 使用可变参数格式化字符串
    va_list args;
    va_start(args, format);
    int len = vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    // 如果格式化成功，发送数据
    if (len > 0) {
        // 确保以换行符结尾
        if (buffer[len - 1] != '\n') {
            buffer[len] = '\n'; // 添加换行符
            len++;
        }
        uart_write_bytes(uart_num, buffer, len); // 发送数据
    }
}

#if 0
#include "../include/vofaplus_for_esp.h"
#include "freertos/task.h"

void app_main() {
    // 初始化 Vofa+ 的 UART
    vofa_init(UART_NUM_1, GPIO_NUM_17, GPIO_NUM_16, 115200);

    float data[3]; // 定义要发送的数据数组
    while (1) {
        // 生成一些测试数据
        static float counter = 0.0f;
        data[0] = counter;          // 第一个数据点
        data[1] = counter * 0.5f;    // 第二个数据点
        data[2] = counter * 0.25f;   // 第三个数据点

        // 发送数据到 Vofa+
        vofa_send_xxx(UART_NUM_1, data, 3); //xxx = firewater , justfloat
        vofa_printf(UART_NUM_1, "%.2f,%.2f,%.2f", data[0], data[1], data[2]);
        counter += 0.1f; // 增加计数器
        vTaskDelay(100 / portTICK_PERIOD_MS); // 延时 100ms
    }
}

#endif