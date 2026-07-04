#include "lidar_ms200.h"
#include "math.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "uart2.h"
#include "ms200.h"


static const char *TAG = "LIDAR_MS200";

// ====== 调试断点追踪 ======
// 卡死后查看串口最后一行包含 "[TRACE-N]" 的输出，匹配下方 N 值定位卡死位置:
//   TRACE-0:  Lidar_Ms200_Task started
//   TRACE-1:  Uart2_Rx_Task started
//   TRACE-10: Lidar loop start
//   TRACE-11: Lidar_Available returned (with count)
//   TRACE-12: Lidar finished processing bytes
//   TRACE-13: Lidar new_package check done
//   TRACE-14: Lidar vTaskDelay about to block
//   TRACE-20: Uart2_Rx loop start
//   TRACE-21: uart_read_bytes returned (with count)
//   TRACE-22: pushed bytes to ringbuf
//   TRACE-23: Uart2_Rx vTaskDelay about to block
volatile uint32_t g_trace = 0;

ms200_data_t lidar_data = {0};


// 激光雷达解析数据任务
// Lidar data parsing mission
static void Lidar_Ms200_Task(void *arg)
{
    g_trace = 0;
    ESP_LOGI(TAG, "[TRACE-0] Start Lidar_Ms200_Task with core:%d", xPortGetCoreID());
    uint16_t rx_count = 0;
    TickType_t last_print_tick = xTaskGetTickCount();

    while (1)
    {
        g_trace = 10;
        rx_count = Uart2_Available();

        if (rx_count)
        {
            g_trace = 11;
            for (int i = 0; i < rx_count; i++)
            {
                Ms200_Data_Receive(Uart2_Read());
            }
            g_trace = 12;
        }

        if (Ms200_New_Package())
        {
            Ms200_Clear_New_Package_State();
            Ms200_Get_Data(&lidar_data);
        }
        g_trace = 13;

        // 无论有无数据都 yield，避免忙等卡死其他任务
        g_trace = 14;
        vTaskDelay(1);  // 1 tick = 10ms (CONFIG_FREERTOS_HZ=100)

        // 每秒打印一次4方向距离，验证雷达是否正常工作
        TickType_t now = xTaskGetTickCount();
        if (pdTICKS_TO_MS(now - last_print_tick) >= 1000)
        {
            last_print_tick = now;
            uint16_t d0   = Lidar_Ms200_Get_Distance(0);
            uint16_t d90  = Lidar_Ms200_Get_Distance(90);
            uint16_t d180 = Lidar_Ms200_Get_Distance(180);
            uint16_t d270 = Lidar_Ms200_Get_Distance(270);
            ESP_LOGI(TAG, "[TRACE-%lu] Lidar: 0°=%dmm, 90°=%dmm, 180°=%dmm, 270°=%dmm",
                     (unsigned long)g_trace, d0, d90, d180, d270);
        }
    }
    vTaskDelete(NULL);
}

// 读取激光雷达某个点检测的距离
// Read the distance detected by the lidar at a point
uint16_t Lidar_Ms200_Get_Distance(uint16_t point)
{
    if (point < MS200_POINT_MAX)
    {
        return lidar_data.points[point].distance;
    }
    return 0;
}

// 初始化MS200激光雷达
// Initialize the MS200 Lidar
void Lidar_Ms200_Init(void)
{
    xTaskCreate(Lidar_Ms200_Task, "Lidar_Ms200_Task", 10*1024, NULL, 8, NULL);
}
