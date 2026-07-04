#include "uart2.h"

#include "stdio.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "driver/gpio.h"

#include "ring_buffer.h"


static const char *TAG = "UART2";

Ring_Buffer_t uart2_ringbuf;
static SemaphoreHandle_t uart2_ringbuf_mutex = NULL;

// 引用 lidar_ms200.cc 中的调试追踪计数器
extern volatile uint32_t g_trace;


// 串口2接收和发送任务 Serial port 2 Receives and sends tasks
static void Uart2_Rx_Task(void *arg)
{
    g_trace = 1;
    ESP_LOGI(TAG, "[TRACE-1] Start Uart2_Rx_Task with core:%d", xPortGetCoreID());
    uint16_t temp_len = 255;
    uint8_t* temp_data = (uint8_t*) malloc(temp_len);
    while (1)
    {
        g_trace = 20;
        // 从串口2读取数据，并将读取的数据缓存到ring_buffer。
        // Data is read from serial port 2 and cached to ring_buffer
        // 1 tick = 10ms (CONFIG_FREERTOS_HZ=100)
        const int rxBytes = uart_read_bytes(UART_NUM_2, temp_data, temp_len, 1);

        if (rxBytes > 0)
        {
            g_trace = 21;
            if (uart2_ringbuf_mutex && xSemaphoreTake(uart2_ringbuf_mutex, pdMS_TO_TICKS(10))) {
                for (int i = 0; i < rxBytes; i++)
                {
                    RingBuffer_Push(&uart2_ringbuf, temp_data[i]);
                }
                xSemaphoreGive(uart2_ringbuf_mutex);
            }
            g_trace = 22;
        }

        g_trace = 23;
        vTaskDelay(1);  // 1 tick = 10ms, 确保让出 CPU 给 IDLE 任务
    }
    free(temp_data);
    vTaskDelete(NULL);
}


// 初始化串口2, 波特率为230400
// Initialize serial port 2, the baud rate is 230400.
void Uart2_Init(void)
{
    const uart_config_t uart_config = {
        .baud_rate = 230400,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    uart_param_config(UART_NUM_2, &uart_config);
    uart_set_pin(UART_NUM_2, UART2_GPIO_TXD, UART2_GPIO_RXD, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_NUM_2, RX2_BUF_SIZE * 2, 0, 0, NULL, 0);

    RingBuffer_Init(&uart2_ringbuf, RX2_BUF_SIZE);

    // 创建互斥锁保护环形缓冲区（双核并发访问 head/tail 需要同步）
    uart2_ringbuf_mutex = xSemaphoreCreateMutex();
    if (uart2_ringbuf_mutex == NULL) {
        ESP_LOGE(TAG, "创建RingBuffer互斥锁失败!");
    }

    // 优先级7: 高于舵机(5)和蓝牙(3)，低于雷达解析(8)，保证数据不丢失
    xTaskCreate(Uart2_Rx_Task, "Uart2_Rx_Task", 4*1024, NULL, 7, NULL);
}

// 通过串口2发送一串数据 Send a string of data through serial port 2
int Uart2_Send_Data(uint8_t* data, uint16_t len)
{
    const int txBytes = uart_write_bytes(UART_NUM_2, data, len);
    return txBytes;
}

// 通过串口2发送一个字节 Send a byte through serial port 2
int Uart2_Send_Byte(uint8_t data)
{
    uint8_t data1 = data;
    const int txBytes = uart_write_bytes(UART_NUM_2, &data1, 1);
    return txBytes;
}

// 返回串口2缓存数据的数量
// Return the amount of cached data in serial port 2
uint16_t Uart2_Available(void)
{
    uint16_t count = 0;
    if (uart2_ringbuf_mutex && xSemaphoreTake(uart2_ringbuf_mutex, pdMS_TO_TICKS(10))) {
        count = RingBuffer_Get_Used_Count(&uart2_ringbuf);
        xSemaphoreGive(uart2_ringbuf_mutex);
    }
    return count;
}

// 从串口2缓存数据中读取一个字节
// Reads a byte from serial port 2 cache data
uint8_t Uart2_Read(void)
{
    uint8_t byte = 0;
    if (uart2_ringbuf_mutex && xSemaphoreTake(uart2_ringbuf_mutex, pdMS_TO_TICKS(10))) {
        byte = RingBuffer_Pop(&uart2_ringbuf);
        xSemaphoreGive(uart2_ringbuf_mutex);
    }
    return byte;
}

// 清除串口2的缓存数据
// Clear cache data from serial port 2
void Uart2_Clean_Buffer(void)
{
    if (uart2_ringbuf_mutex && xSemaphoreTake(uart2_ringbuf_mutex, pdMS_TO_TICKS(10))) {
        RingBuffer_Clean_Queue(&uart2_ringbuf);
        xSemaphoreGive(uart2_ringbuf_mutex);
    }
}

