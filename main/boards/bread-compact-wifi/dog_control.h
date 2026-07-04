/*
    ============== 引脚分配总览 (ESP32-S3 N16R8) ==============

    功能分类        GPIO编号    详细定义                备注
    ----------      --------    --------                ----
    正交编码器      GPIO8       编码器1_A相             PCNT_UNIT_0
    正交编码器      GPIO3       编码器1_B相             PCNT_UNIT_0
    正交编码器      GPIO9       编码器2_A相             PCNT_UNIT_1
    正交编码器      GPIO14      编码器2_B相             PCNT_UNIT_1
    正交编码器      GPIO1       编码器3_A相             PCNT_UNIT_2
    正交编码器      GPIO2       编码器3_B相             PCNT_UNIT_2
    正交编码器      GPIO39      编码器4_A相             PCNT_UNIT_3 (与B相交换取反方向)
    正交编码器      GPIO38      编码器4_B相             PCNT_UNIT_3

    电机驱动        GPIO11      电机1_IN1(PWM)          TB6612第1片通道A, LEDC_CH0
    电机驱动        GPIO10      电机1_IN2               GPIO矩阵取反(LEDC_CH0)
    电机驱动        GPIO12      电机2_IN1(PWM)          TB6612第1片通道B, LEDC_CH1
    电机驱动        GPIO13      电机2_IN2               GPIO矩阵取反(LEDC_CH1)
    电机驱动        GPIO19      电机3_IN1(PWM)          TB6612第2片通道A, LEDC_CH2 (与IN2交换取反方向)
    电机驱动        GPIO40      电机3_IN2               GPIO矩阵取反(LEDC_CH2)
    电机驱动        GPIO20      电机4_IN1(PWM)          TB6612第2片通道B, LEDC_CH3
    电机驱动        GPIO21      电机4_IN2               GPIO矩阵取反(LEDC_CH3)
    电机驱动        —           PWMA/PWMB接VCC, STBY接VCC

    蓝牙串口        GPIO0       串口RX                  UART1_RX，接蓝牙模块TX（原为GPIO44，与UART0控制台冲突）

    激光雷达        GPIO17      串口TX                  UART2_TX，接MS200雷达RX
    激光雷达        GPIO18      串口RX                  UART2_RX，接MS200雷达TX，230400bps

    舵机控制        GPIO46      舵机1_PWM               50Hz, LEDC_CH4, Strapping引脚
    舵机控制        GPIO48      舵机2_PWM               50Hz, LEDC_CH5, Strapping引脚
    舵机控制        GPIO45      舵机3_PWM               50Hz, LEDC_CH6
    舵机控制        GPIO47      舵机4_PWM               50Hz, LEDC_CH7

    显示I2C         GPIO41      SDA                     OLED
    显示I2C         GPIO42      SCL                     OLED

    音频I2S         GPIO4~7,15,16                       麦克风+喇叭

    Boot按钮        GPIO0                               仅输入，已内置上拉
*/
#ifndef __DOG_CONTROL_H__
#define __DOG_CONTROL_H__

#include <driver/ledc.h>
#include <driver/gpio.h>
#include <driver/pulse_cnt.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <functional>
#include "config.h"
#include "mcp_server.h"

// ============== 舵机 PWM 配置（LEDC_TIMER_1, 50Hz, 14bit） ==============
#define SERVO_PWM_TIMER     LEDC_TIMER_1
#define SERVO_PWM_FREQ_HZ   50
#define SERVO_PWM_RESOLUTION LEDC_TIMER_14_BIT
#define NUM_SERVOS          4

const gpio_num_t servo_pins[NUM_SERVOS] = {
    GPIO_NUM_46,   // 舵机1 (LEDC_CH4)
    GPIO_NUM_48,   // 舵机2 (LEDC_CH5)
    GPIO_NUM_45,   // 舵机3 (LEDC_CH6)
    GPIO_NUM_47    // 舵机4 (LEDC_CH7)
};

const ledc_channel_t servo_channel[NUM_SERVOS] = {
    LEDC_CHANNEL_4,
    LEDC_CHANNEL_5,
    LEDC_CHANNEL_6,
    LEDC_CHANNEL_7
};

// ============== TB6612 电机驱动引脚（GPIO矩阵取反，每路1个GPIO同时驱动IN1+IN2） ==============
// IN1 接 GPIO（PWM信号），IN2 由 GPIO 矩阵自动取反驱动
// PWMA/PWMB 接 VCC，STBY 接 VCC
// 第1片 TB6612：电机1（通道A）+ 电机2（通道B）
#define MOTOR1_PWM          GPIO_NUM_11     // 电机1 IN1 (LEDC_CH0)
#define MOTOR1_IN2          GPIO_NUM_10     // 电机1 IN2 (GPIO矩阵取反)

#define MOTOR2_PWM          GPIO_NUM_12     // 电机2 IN1 (LEDC_CH1)
#define MOTOR2_IN2          GPIO_NUM_13     // 电机2 IN2 (GPIO矩阵取反)

// 第2片 TB6612：电机3（通道A）+ 电机4（通道B）
#define MOTOR3_PWM          GPIO_NUM_19     // 电机3 IN1 (LEDC_CH2) — 与IN2交换取反方向
#define MOTOR3_IN2          GPIO_NUM_40    // 电机3 IN2 (GPIO矩阵取反)

#define MOTOR4_PWM          GPIO_NUM_20     // 电机4 IN1 (LEDC_CH3)
#define MOTOR4_IN2          GPIO_NUM_21     // 电机4 IN2 (GPIO矩阵取反)

// ============== 电机 PWM 配置（LEDC_TIMER_0, 20KHz, 10bit） ==============
// 20kHz 超出人耳范围，消除电机蜂鸣声；10bit(1024级)足够电机调速
#define MOTOR_PWM_TIMER     LEDC_TIMER_0
#define MOTOR_PWM_FREQ_HZ   20000
#define MOTOR_PWM_RESOLUTION LEDC_TIMER_10_BIT
#define MOTOR_PWM_MAX_DUTY  1023            // 2^10 - 1
#define MOTOR_PWM_STOP_DUTY (MOTOR_PWM_MAX_DUTY / 2)  // 50% duty = 停止（锁-反相模式）

const ledc_channel_t motor_channel[4] = {
    LEDC_CHANNEL_0,  // 电机1
    LEDC_CHANNEL_1,  // 电机2
    LEDC_CHANNEL_2,  // 电机3
    LEDC_CHANNEL_3   // 电机4
};

// ============== 霍尔编码器引脚（正交解码 A+B） ==============
#define ENCODER1_A          GPIO_NUM_8      // 编码器1 A相 (PCNT_UNIT_0)
#define ENCODER1_B          GPIO_NUM_3      // 编码器1 B相 (PCNT_UNIT_0)
#define ENCODER2_A          GPIO_NUM_9      // 编码器2 A相 (PCNT_UNIT_1)
#define ENCODER2_B          GPIO_NUM_14     // 编码器2 B相 (PCNT_UNIT_1)
#define ENCODER3_A          GPIO_NUM_1      // 编码器3 A相 (PCNT_UNIT_2)
#define ENCODER3_B          GPIO_NUM_2      // 编码器3 B相 (PCNT_UNIT_2)
#define ENCODER4_A          GPIO_NUM_39     // 编码器4 A相 (PCNT_UNIT_3) — 与B相交换取反方向
#define ENCODER4_B          GPIO_NUM_38     // 编码器4 B相 (PCNT_UNIT_3)

// ============== 编码器配置 ==============
#define ENCODER_PPR         13              // 编码器线数
#define ENCODER_SAMPLE_MS   100             // 采样周期(ms)
#define ENCODER_GEAR_RATIO  45              // 减速比（典型TT马达约1:34）

// 左前：0 ～ 180   右前：180 ～ 0   左后：0 ～ 180   右后：180 ～ 0
class DogControl
{
private:
    // 动作相关变量
    uint16_t PAnumbers;           // 动作重复次数
    uint16_t SwingDelay{8};       // 摇摆速度延迟（ms）

    const int MIN_STEP{1};        // 最小步长
    const int MAX_STEP{10};       // 最大步长
    const float STEP_RATIO{0.1f}; // 步长占角度差的比例（10%）
    // 记录4个舵机当前角度（左前/右前/左后/右后）
    int current_angles_[4]{99, 92, 83, 90}; // 小范围调整机械误差
    // 舵机机械误差校准偏移（角度值，正=正向偏移，负=负向偏移）
    // 根据实际安装误差调整，例如 {-3, 2, -1, 4}
    int servo_offset_[4]{0, 0, 0, 0};
    // 舵机速度参数（步长，值越小越平滑）
    int SERVO_SPEED{1};
    // 动作延迟（ms，控制每步调整的间隔）
    int DELAY_MS{12};

    // 任务和队列
    TaskHandle_t task_handle_;
    QueueHandle_t command_queue_;

    // UART串口接收任务
    TaskHandle_t uart_task_handle_{nullptr};

    // PCNT编码器句柄（4组）
    pcnt_unit_handle_t pcnt_unit_[4]{nullptr, nullptr, nullptr, nullptr};
    pcnt_channel_handle_t pcnt_chan_a_[4]{nullptr, nullptr, nullptr, nullptr};
    pcnt_channel_handle_t pcnt_chan_b_[4]{nullptr, nullptr, nullptr, nullptr};

    // 命令类型
    enum CommandType
    {
        CMD_Stand,
        CMD_Getdown,
        CMD_Swing,
        CMD_Stretch,
        CMD_Lstretch,
    };

    // 工具函数
    uint32_t AngleToDuty(float angle);

    // 任务函数
    static void TaskCreate(void *parameter);
    void ProcessCommands();
    void SmoothSetServoAngle(int target_angle, int servo_index);
    void SmoothSetMultiServoAngles(const int target_angles[4]);
    void ExecuteStand();
    void ExecuteGetdown();
    void ExecuteSwing();
    void ExecuteStretch();
    void ExecuteLstretch();

    // 电机驱动初始化
    void MotorGpioInit();
    void MotorPwmInit();
    void EncoderInit();

    // UART1串口接收任务（接收蓝牙模块转发的电机控制命令）
    static void UartTaskEntry(void *parameter);
    void UartTaskLoop();

public:
    DogControl();
    ~DogControl();

    bool DogInitialize();      // 基本控制方法
    void DogInitializeTools(); // 初始化MCP工具
    void SetServoAngle(float angle, int i);

    // 电机控制（4路独立）
    void MotorSetSpeed(int motor, int speed);  // motor: 0~3, speed: -100 ~ 100
    void MotorStop();                          // 停止所有电机

    // 编码器读取（正交解码，正=正转，负=反转）
    int32_t EncoderGetCount(int encoder);      // encoder: 0~3
    void EncoderReset();                       // 清零所有编码器计数

    // 运动控制方法（通过宏定义批量生成声明）
#define DEFINE_DOG_COMMAND(name, cmd) void Dog##name();
    DEFINE_DOG_COMMAND(Stand, CMD_Stand)
    DEFINE_DOG_COMMAND(Getdown, CMD_Getdown)
    DEFINE_DOG_COMMAND(Swing, CMD_Swing)
    DEFINE_DOG_COMMAND(Stretch, CMD_Stretch)
    DEFINE_DOG_COMMAND(Lstretch, CMD_Lstretch)
#undef DEFINE_DOG_COMMAND
};

#endif
