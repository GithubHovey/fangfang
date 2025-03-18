#include "../include/sys_init.h"
#include "../include/sys_internal.h"
#include "motor.h"
#include "utility.h"
// #include "driver/gpio.h"
#define CPU0 0
#define CPU1 1
TaskHandle_t main_task_handle;
TaskHandle_t debug_task_handle;
void Main_task(void * arg);
void Debug_task(void * arg);

void AppInit()
{
#if USE_SCREEN == 1
    xTaskCreatePinnedToCore(lvgl_task,"app.lvgl",6144,NULL,5,&lvgl_handle,CPU1);
#endif 
#if USE_AUDIO == 1
    xTaskCreatePinnedToCore(Audio_task,"app.audio",3072,NULL,1,&audio_handle,CPU1);
#endif 

#if USE_NETWORK == 1
    xTaskCreatePinnedToCore(NetworkTask,"app.network",3584,NULL,5,&network_handle,CPU1);
#endif 
    // xTaskCreatePinnedToCore(vofaTask,"app.vofa",3584,NULL,5,&vofa_task_handle,CPU0);
    // xTaskCreatePinnedToCore(Main_task,"app.main",3584,NULL,5,&main_task_handle,CPU1);
    xTaskCreatePinnedToCore(Debug_task,"app.debug",3584,NULL,5,&debug_task_handle,CPU0);
#ifdef USE_ASR
    #if USE_ASR == 1
    // xTaskCreatePinnedToCore(&detect_Task, "app.detect", 4 * 1024, NULL, 1, NULL, CPU1);
    // xTaskCreatePinnedToCore(&HearingTask, "app.feed", 4 * 1024, NULL, 1, NULL, CPU1);
    #endif
#endif

}
void Main_task(void * arg)
{
    const char* tag = pcTaskGetName(xTaskGetCurrentTaskHandle());
    static char InfoBuffer[1024] = {0}; 
    static char TaskLoadingBuffer[1024] = {0};
    ESP_LOGI(tag, "%s is created.",tag);
    vTaskDelay(1000);   
    while (1) {
        vTaskList((char *) &InfoBuffer);
        vTaskGetRunTimeStats((char *)&TaskLoadingBuffer); 
        printf("--------------\n");
        printf("任务名      任务状态 优先级   剩余栈 任务序号\r\n");
        printf("\r\n%s\r\n", InfoBuffer);
        printf("任务名         运行计数        CPU使用率 \r\n");
        printf("=================================================\r\n");
        printf("%s\r\n",TaskLoadingBuffer);
        printf("=================================================\r\n");
        heap_caps_print_heap_info(MALLOC_CAP_8BIT);     
        vTaskDelay(10000);
    }
}
Motor yaw(MCPWM0, GPIO_NUM_15, GPIO_NUM_16, GPIO_NUM_17,
             I2C_NUM_0,GPIO_NUM_6,GPIO_NUM_7 ,7);
Vofa vofa(UART_NUM_1,4,5);
    
void Debug_task(void * arg)
{
    TickType_t xLastWakeTime_t;
    xLastWakeTime_t = xTaskGetTickCount();
    const char* tag = pcTaskGetName(xTaskGetCurrentTaskHandle());
    ESP_LOGI(tag, "%s is created.",tag);
    
    vofa.begin(115200);
    uint16_t rxdata = 0;
    yaw.init();
    
    gpio_dump_io_configuration(stdout, (1ULL << 6) | (1ULL << 7) | (1ULL << 15) | (1ULL << 16) | (1ULL << 17));
    vTaskDelay(1000);   
    for(;;)
    {   vTaskDelayUntil(&xLastWakeTime_t, 10);
        // rxdata = encoder.getRawAngle();
        int64_t start = esp_timer_get_time();
        yaw.update();
        // yaw.debug();
        // ESP_LOGI(tag, "rxdata = %d",rxdata);
        int64_t duration = esp_timer_get_time() - start;

        float data[] = {yaw.Ua, yaw.Ub, yaw.Uc}; // 要发送的数据
        vofa.sendData(data, 3); // 发送数据
        if(duration > 3000) { // 超过 2ms
            ESP_LOGW(tag, "Update exceeded: %lld us", duration);
        }
        
    }
}