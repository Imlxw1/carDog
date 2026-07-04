#include "dog_control.h"
#include <esp_log.h>
#include <cmath>
#include <cstring>
#include <esp_rom_gpio.h>
#include <soc/gpio_sig_map.h>
#include <driver/uart.h>
#include "uart2.h"
#include "lidar_ms200.h"

#define TAG "DogControl"

// 宏定义：生成运动控制方法
#define DEFINE_DOG_COMMAND_IMPL(name, cmd)                \
    void DogControl::Dog##name()                          \
    {                                                     \
        CommandType cmd_{cmd};                            \
        xQueueSend(command_queue_, &cmd_, portMAX_DELAY); \
    }

// 生成运动控制方法的实现
DEFINE_DOG_COMMAND_IMPL(Stand, CMD_Stand)
DEFINE_DOG_COMMAND_IMPL(Getdown, CMD_Getdown)
DEFINE_DOG_COMMAND_IMPL(Swing, CMD_Swing)
DEFINE_DOG_COMMAND_IMPL(Stretch, CMD_Stretch)
DEFINE_DOG_COMMAND_IMPL(Lstretch, CMD_Lstretch)

DogControl::DogControl()
    : task_handle_(nullptr),
      command_queue_(nullptr),
      uart_task_handle_(nullptr)
{
}

DogControl::~DogControl()
{
    MotorStop();

    if (uart_task_handle_ != nullptr){
        vTaskDelete(uart_task_handle_);
    }
    for (int i = 0; i < 4; i++) {
        if (pcnt_unit_[i] != nullptr){
            pcnt_unit_disable(pcnt_unit_[i]);
            pcnt_del_unit(pcnt_unit_[i]);
        }
    }
    ledc_timer_pause(LEDC_LOW_SPEED_MODE, SERVO_PWM_TIMER);

    if (task_handle_ != nullptr){
        vTaskDelete(task_handle_);
    }
    if (command_queue_ != nullptr){
        vQueueDelete(command_queue_);
    }
}

bool DogControl::DogInitialize()
{
    // 配置舵机LEDC定时器 (50Hz, 14bit)
    ledc_timer_config_t timer_config = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = SERVO_PWM_RESOLUTION,
        .timer_num = SERVO_PWM_TIMER,
        .freq_hz = SERVO_PWM_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK
    };

    esp_err_t ret = ledc_timer_config(&timer_config);
    if (ret != ESP_OK){
        ESP_LOGE(TAG, "舵机LEDC定时器配置失败: %s", esp_err_to_name(ret));
        return false;
    }

    // 配置舵机LEDC通道 (CH4~CH7)
    for (int i = 0; i < NUM_SERVOS; i++)
    {
        ledc_channel_config_t channel_config = {
            .gpio_num = servo_pins[i],
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel = servo_channel[i],
            .intr_type = LEDC_INTR_DISABLE,
            .timer_sel = SERVO_PWM_TIMER,
            .duty = AngleToDuty(current_angles_[i]),
            .hpoint = 0
        };

        ret = ledc_channel_config(&channel_config);
        if (ret != ESP_OK){
            ESP_LOGE(TAG, "舵机LEDC通道%d配置失败: %s", i, esp_err_to_name(ret));
            return false;
        }
    }

    ESP_LOGI(TAG, "舵机LEDC定时器和%d个通道配置成功", NUM_SERVOS);

    // 创建命令队列
    command_queue_ = xQueueCreate(15, sizeof(CommandType));
    if (command_queue_ == nullptr){
        ESP_LOGE(TAG, "创建命令队列失败");
        return false;
    }

    // 创建控制任务
    BaseType_t task_ret = xTaskCreate(
        TaskCreate,
        "task_create",
        8192,
        this,
        5,
        &task_handle_
    );

    if (task_ret != pdPASS){
        ESP_LOGE(TAG, "创建舵机任务失败");
        return false;
    }

    ESP_LOGI(TAG, "控制器初始化成功");

    // 初始化电机、编码器
    MotorGpioInit();
    MotorPwmInit();
    EncoderInit();

    // 创建UART串口接收任务 (蓝牙电机控制, UART1, GPIO43/44)
    BaseType_t uart_ret = xTaskCreate(
        UartTaskEntry,
        "uart_task",
        8192,
        this,
        3,
        &uart_task_handle_
    );
    if (uart_ret != pdPASS){
        ESP_LOGE(TAG, "创建UART任务失败");
    } else {
        ESP_LOGI(TAG, "UART串口接收任务创建成功");
    }

    // 初始化激光雷达 (UART2, GPIO17/18, 230400bps)
    Uart2_Init();
    Lidar_Ms200_Init();
    ESP_LOGI(TAG, "激光雷达MS200初始化完成 (UART2, TX=GPIO%d, RX=GPIO%d)",
             UART2_GPIO_TXD, UART2_GPIO_RXD);

    return true;
}

void DogControl::DogInitializeTools()
{
    auto &mcp_server = McpServer::GetInstance();
    ESP_LOGI(TAG, "开始注册舵机MCP工具...");

// 注册MCP工具
#define REGISTER_MCP_TOOL(tool_name, desc, method) \
    mcp_server.AddTool(tool_name,                  \
                       desc,                       \
                       PropertyList(),             \
                       [this](const PropertyList &) -> ReturnValue { \
                               this->method(); \
                               return true; });

    // 注册舵机相关工具
    REGISTER_MCP_TOOL("self.dog.stand", "控制舵机让机器狗站立,站起来,立正", DogStand)
    REGISTER_MCP_TOOL("self.dog.getdown", "控制舵机让机器狗趴下", DogGetdown)
    REGISTER_MCP_TOOL("self.dog.swing", "控制舵机让机器狗摇摆,跳舞", DogSwing)
    REGISTER_MCP_TOOL("self.dog.stretch", "控制舵机让机器狗伸懒腰", DogStretch)
    REGISTER_MCP_TOOL("self.dog.lstretch", "控制舵机让机器狗拉伸", DogLstretch)

#undef REGISTER_MCP_TOOL

    ESP_LOGI(TAG, "舵机MCP工具注册完成");
}

// 角度转PWM占空比（14位分辨率）
uint32_t DogControl::AngleToDuty(float angle)
{
    if (angle < 0)      angle = 0;
    if (angle > 180)    angle = 180;
    // 转换为占空比：(脉宽us / 20000us) * 16383（14位最大值）
    float pulse_us = angle / 180.0f * 2000.0f + 500.0f;
    return (uint32_t)(pulse_us / 20000.0f * 16383);
}

// 实现舵机控制
void DogControl::SetServoAngle(float angle, int i){
    uint32_t duty = AngleToDuty(angle);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, servo_channel[i], duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, servo_channel[i]);
}

// 单个舵机控制
void DogControl::SmoothSetServoAngle(int target_angle, int servo_index)
{
    if (servo_index < 0 || servo_index >= 4)
        return;

    int current = current_angles_[servo_index];
    target_angle = std::max(0, std::min(180, target_angle));

    while (current != target_angle)
    {
        int diff = target_angle - current;
        int step = static_cast<int>(std::abs(diff) * STEP_RATIO);
        step = std::max(MIN_STEP, std::min(MAX_STEP, step));
        step = diff > 0 ? step : -step;

        int new_angle = current + step;
        if ((step > 0 && new_angle > target_angle) ||
            (step < 0 && new_angle < target_angle))
            new_angle = target_angle;

        SetServoAngle(new_angle, servo_index);
        current_angles_[servo_index] = new_angle;
        current = new_angle;

        vTaskDelay(pdMS_TO_TICKS(DELAY_MS));
    }
}

// 多个舵机控制
void DogControl::SmoothSetMultiServoAngles(const int target_angles[4])
{
    bool all_reached = false;
    while (!all_reached)
    {
        all_reached = true;
        for (int i = 0; i < 4; i++)
        {
            int current = current_angles_[i];
            int target = std::max(0, std::min(180, target_angles[i]));
            if (current == target)
                continue;

            all_reached = false;
            int step = (target > current) ? SERVO_SPEED : -SERVO_SPEED;
            int new_angle = current + step;

            if ((step > 0 && new_angle > target) || (step < 0 && new_angle < target))
                new_angle = target;

            SetServoAngle(new_angle, i);
            current_angles_[i] = new_angle;
        }
        vTaskDelay(pdMS_TO_TICKS(DELAY_MS));
    }
}

void DogControl::TaskCreate(void *parameter){
    DogControl *controller = static_cast<DogControl *>(parameter);
    controller->ProcessCommands();
}

void DogControl::ProcessCommands()
{
    CommandType cmd;

    while (true)
    {
        if (xQueueReceive(command_queue_, &cmd, pdMS_TO_TICKS(100)) == pdTRUE)
        {
            switch (cmd)
            {
                case CMD_Stand:      ExecuteStand(); break;
                case CMD_Getdown:    ExecuteGetdown(); break;
                case CMD_Swing:      ExecuteSwing(); break;
                case CMD_Stretch:    ExecuteStretch(); break;
                case CMD_Lstretch:   ExecuteLstretch(); break;
            }
        }
    }
}

void DogControl::ExecuteStand(){
    int target_angles[4]{
        90 + servo_offset_[0], 90 + servo_offset_[1],
        90 + servo_offset_[2], 90 + servo_offset_[3]};
    SmoothSetMultiServoAngles(target_angles);
}

void DogControl::ExecuteGetdown(){
    int target_angles[4]{
        20 + servo_offset_[0], 160 + servo_offset_[1],
        160 + servo_offset_[2], 25 + servo_offset_[3]};
    SmoothSetMultiServoAngles(target_angles);
}

void DogControl::ExecuteSwing()
{
    PAnumbers = 3;
    int old_speed = SERVO_SPEED;
    int old_delay = DELAY_MS;
    SERVO_SPEED = 3;
    DELAY_MS = 10;

    while (PAnumbers)
    {
        for (int i = 30; i <= 150; i += SERVO_SPEED){
            int angles[4] = {i + servo_offset_[0], i + servo_offset_[1],
                             i + servo_offset_[2], i + servo_offset_[3]};
            SmoothSetMultiServoAngles(angles);
        }

        for (int i = 150; i >= 30; i -= SERVO_SPEED){
            int angles[4] = {i + servo_offset_[0], i + servo_offset_[1],
                             i + servo_offset_[2], i + servo_offset_[3]};
            SmoothSetMultiServoAngles(angles);
        }
        PAnumbers--;
    }
    SERVO_SPEED = old_speed;
    DELAY_MS = old_delay;
    ExecuteStand();
}

void DogControl::ExecuteStretch()
{
    int old_delay = DELAY_MS;
    DELAY_MS = 9;
    SmoothSetServoAngle(90 + servo_offset_[2], 2);
    SmoothSetServoAngle(90 + servo_offset_[3], 3);
    vTaskDelay(pdMS_TO_TICKS(100));

    for (int i = 90; i > 10; i--){
        SmoothSetServoAngle(i + servo_offset_[0], 0);
        SmoothSetServoAngle(180 - i + servo_offset_[1], 1);
        vTaskDelay(pdMS_TO_TICKS(15));
    }

    for (int i = 10; i < 90; i++){
        SmoothSetServoAngle(i + servo_offset_[0], 0);
        SmoothSetServoAngle(180 - i + servo_offset_[1], 1);
        vTaskDelay(pdMS_TO_TICKS(15));
    }

    for (int i = 90; i < 170; i++){
        SmoothSetServoAngle(i + servo_offset_[2], 2);
        SmoothSetServoAngle(180 - i + servo_offset_[3], 3);
        vTaskDelay(pdMS_TO_TICKS(15));
    }

    for (int i = 170; i > 90; i--){
        SmoothSetServoAngle(i + servo_offset_[2], 2);
        SmoothSetServoAngle(180 - i + servo_offset_[3], 3);
        vTaskDelay(pdMS_TO_TICKS(15));
    }
    DELAY_MS = old_delay;
    ExecuteLstretch();
}

void DogControl::ExecuteLstretch()
{
    int breakvalue = 1;
    int temp = 3;
    int old_delay = DELAY_MS;
    DELAY_MS = 10;

    while (breakvalue)
    {
        SmoothSetServoAngle(90 + servo_offset_[0], 0);
        SmoothSetServoAngle(160 + servo_offset_[1], 1);
        vTaskDelay(pdMS_TO_TICKS(60));
        SmoothSetServoAngle(70 + servo_offset_[3], 3);

        for (int i = 90; i < 180; i++){
            SmoothSetServoAngle(i + servo_offset_[2], 2);
            vTaskDelay(pdMS_TO_TICKS(6));
        }

        while (temp)
        {
            for (int i = 180; i > 150; i--){
                SmoothSetServoAngle(i + servo_offset_[2], 2);
                vTaskDelay(pdMS_TO_TICKS(15));
            }
            temp--;
        }

        vTaskDelay(pdMS_TO_TICKS(100));
        SmoothSetServoAngle(90 + servo_offset_[0], 0);
        SmoothSetServoAngle(90 + servo_offset_[1], 1);

        vTaskDelay(pdMS_TO_TICKS(80));
        SmoothSetServoAngle(90 + servo_offset_[2], 2);
        SmoothSetServoAngle(90 + servo_offset_[3], 3);
        vTaskDelay(pdMS_TO_TICKS(100));

        temp = 3;

        SmoothSetServoAngle(90 + servo_offset_[1], 1);
        SmoothSetServoAngle(20 + servo_offset_[0], 0);
        vTaskDelay(pdMS_TO_TICKS(60));
        SmoothSetServoAngle(70 + servo_offset_[2], 2);

        for (int i = 90; i < 180; i++){
            SmoothSetServoAngle(i + servo_offset_[3], 3);
            vTaskDelay(pdMS_TO_TICKS(6));
        }

        while (temp)
        {
            for (int i = 180; i > 150; i--){
                SmoothSetServoAngle(i + servo_offset_[3], 3);
                vTaskDelay(pdMS_TO_TICKS(15));
            }
            temp--;
        }
        DELAY_MS = old_delay;
        ExecuteStand();
        breakvalue = 0;
    }
}

// ============================================================
//  TB6612 电机驱动 & 霍尔编码器（4路独立）
// ============================================================

// ---------- GPIO 初始化（4个电机引脚，IN2由GPIO矩阵取反驱动） ----------
void DogControl::MotorGpioInit()
{
    gpio_config_t io_conf = {};
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.pin_bit_mask =
        (1ULL << MOTOR1_PWM) | (1ULL << MOTOR2_PWM) |
        (1ULL << MOTOR3_PWM) | (1ULL << MOTOR4_PWM) |
        (1ULL << MOTOR1_IN2) | (1ULL << MOTOR2_IN2) |
        (1ULL << MOTOR3_IN2) | (1ULL << MOTOR4_IN2);
    gpio_config(&io_conf);

    // 初始全部输出低（停止状态，防止上电瞬间IN2浮空导致电机误转）
    gpio_set_level(MOTOR1_PWM, 0);
    gpio_set_level(MOTOR2_PWM, 0);
    gpio_set_level(MOTOR3_PWM, 0);
    gpio_set_level(MOTOR4_PWM, 0);
    gpio_set_level(MOTOR1_IN2, 0);
    gpio_set_level(MOTOR2_IN2, 0);
    gpio_set_level(MOTOR3_IN2, 0);
    gpio_set_level(MOTOR4_IN2, 0);

    ESP_LOGI(TAG, "电机GPIO初始化完成 (IN1: GPIO%d,%d,%d,%d, IN2: GPIO%d,%d,%d,%d)",
             MOTOR1_PWM, MOTOR2_PWM, MOTOR3_PWM, MOTOR4_PWM,
             MOTOR1_IN2, MOTOR2_IN2, MOTOR3_IN2, MOTOR4_IN2);
}

// ---------- LEDC PWM 初始化（4路PWM + GPIO矩阵取反驱动IN2） ----------
// 原理：LEDC输出PWM到IN1，GPIO矩阵将同一信号取反后输出到IN2
// 50%duty = 停止, >50% = 正转, <50% = 反转
void DogControl::MotorPwmInit()
{
    ledc_timer_config_t timer_cfg = {};
    timer_cfg.speed_mode = LEDC_LOW_SPEED_MODE;
    timer_cfg.duty_resolution = MOTOR_PWM_RESOLUTION;
    timer_cfg.timer_num = MOTOR_PWM_TIMER;
    timer_cfg.freq_hz = MOTOR_PWM_FREQ_HZ;
    timer_cfg.clk_cfg = LEDC_AUTO_CLK;
    esp_err_t ret = ledc_timer_config(&timer_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "电机PWM定时器配置失败: %s", esp_err_to_name(ret));
        return;
    }

    // 4路电机PWM通道 (CH0~CH3)，配置IN1引脚
    const gpio_num_t motor_pwm_pins[4] = {MOTOR1_PWM, MOTOR2_PWM, MOTOR3_PWM, MOTOR4_PWM};
    for (int i = 0; i < 4; i++) {
        ledc_channel_config_t ch_cfg = {};
        ch_cfg.speed_mode = LEDC_LOW_SPEED_MODE;
        ch_cfg.gpio_num = motor_pwm_pins[i];
        ch_cfg.channel = motor_channel[i];
        ch_cfg.intr_type = LEDC_INTR_DISABLE;
        ch_cfg.timer_sel = MOTOR_PWM_TIMER;
        ch_cfg.duty = MOTOR_PWM_STOP_DUTY;  // 初始50%duty = 停止
        ch_cfg.hpoint = 0;
        ledc_channel_config(&ch_cfg);
    }

    // GPIO矩阵取反：把同一个LEDC CH信号路由到IN2引脚，开启out_inv取反
    // TB6612接线：IN1接GPIO（正相），IN2接另一个GPIO（取反）
    // 注意：IN2引脚需要在硬件上连接到TB6612的IN2引脚
    const gpio_num_t motor_in2_pins[4] = {MOTOR1_IN2, MOTOR2_IN2, MOTOR3_IN2, MOTOR4_IN2};
    for (int i = 0; i < 4; i++) {
        gpio_set_direction(motor_in2_pins[i], GPIO_MODE_OUTPUT);
        // 将LEDC CH信号路由到IN2，out_inv=true取反，oen_inv=false
        esp_rom_gpio_connect_out_signal(motor_in2_pins[i],
                                        LEDC_LS_SIG_OUT0_IDX + motor_channel[i],
                                        true, false);
    }

    ESP_LOGI(TAG, "电机PWM初始化完成 (4路, %dHz, %dbit, IN1:GPIO%d/%d/%d/%d, IN2:GPIO矩阵取反)",
             MOTOR_PWM_FREQ_HZ, 10,
             MOTOR1_PWM, MOTOR2_PWM, MOTOR3_PWM, MOTOR4_PWM);
}

// ---------- PCNT 脉冲计数器初始化（4组正交解码 A+B） ----------
// 简化模式：仅A相上升沿计数，B相判断方向
//   A上升沿 & B低 → +1（正转）
//   A上升沿 & B高 → -1（反转）
void DogControl::EncoderInit()
{
    const gpio_num_t enc_a[4] = {ENCODER1_A, ENCODER2_A, ENCODER3_A, ENCODER4_A};
    const gpio_num_t enc_b[4] = {ENCODER1_B, ENCODER2_B, ENCODER3_B, ENCODER4_B};

    pcnt_unit_config_t unit_cfg = {};
    unit_cfg.high_limit = 32767;
    unit_cfg.low_limit  = -32768;

    pcnt_glitch_filter_config_t filter_cfg = {};
    filter_cfg.max_glitch_ns = 1000;

    for (int i = 0; i < 4; i++) {
        // 创建PCNT单元
        esp_err_t ret = pcnt_new_unit(&unit_cfg, &pcnt_unit_[i]);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "编码器%d PCNT单元创建失败: %s", i, esp_err_to_name(ret));
            continue;
        }

        // 单通道：A相为边沿信号，B相为电平信号
        pcnt_chan_config_t chan_cfg = {};
        chan_cfg.edge_gpio_num  = enc_a[i];
        chan_cfg.level_gpio_num = enc_b[i];
        ret = pcnt_new_channel(pcnt_unit_[i], &chan_cfg, &pcnt_chan_a_[i]);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "编码器%d 通道创建失败 (A=GPIO%d, B=GPIO%d): %s",
                     i, enc_a[i], enc_b[i], esp_err_to_name(ret));
            pcnt_del_unit(pcnt_unit_[i]);
            pcnt_unit_[i] = nullptr;
            continue;
        }
        // A上升沿 → +1，A下降沿 → 不计数（简化模式，单倍频）
        pcnt_channel_set_edge_action(pcnt_chan_a_[i],
            PCNT_CHANNEL_EDGE_ACTION_INCREASE,
            PCNT_CHANNEL_EDGE_ACTION_HOLD);
        // B低 → 正向计数，B高 → 反向计数
        pcnt_channel_set_level_action(pcnt_chan_a_[i],
            PCNT_CHANNEL_LEVEL_ACTION_KEEP,     // B低时保持计数方向(+1)
            PCNT_CHANNEL_LEVEL_ACTION_INVERSE);  // B高时反转计数方向(-1)

        pcnt_unit_set_glitch_filter(pcnt_unit_[i], &filter_cfg);
        pcnt_unit_enable(pcnt_unit_[i]);
        pcnt_unit_clear_count(pcnt_unit_[i]);
        pcnt_unit_start(pcnt_unit_[i]);

        ESP_LOGI(TAG, "编码器%d 初始化成功 (A=GPIO%d, B=GPIO%d)", i, enc_a[i], enc_b[i]);
    }

    ESP_LOGI(TAG, "编码器PCNT初始化完成 (A=%d/%d/%d/%d, B=%d/%d/%d/%d)",
             ENCODER1_A, ENCODER2_A, ENCODER3_A, ENCODER4_A,
             ENCODER1_B, ENCODER2_B, ENCODER3_B, ENCODER4_B);
}

// ---------- 电机速度控制（4路独立，GPIO矩阵取反模式） ----------
// GPIO矩阵取反：IN1=PWM正相, IN2=PWM反相（硬件自动取反）
//   duty>50% → IN1占空比高 → 正转
//   duty<50% → IN2占空比高 → 反转
//   duty=50% → 停止

// GPIO矩阵取反模式的速度控制
// IN1 = PWM正相, IN2 = PWM反相（GPIO矩阵自动取反）
// speed: -100(反转) ~ 0(停止) ~ +100(正转)
// duty映射: 0=全反转, 50%=停止, 100=全正转
void DogControl::MotorSetSpeed(int motor, int speed)
{
    if (motor < 0 || motor > 3) return;
    if (speed > 100)  speed = 100;
    if (speed < -100) speed = -100;

    // 死区：速度绝对值<=5时直接停止，减少电机振动噪音
    if (speed > -5 && speed < 5) speed = 0;

    // -100~+100 映射到 0~MOTOR_PWM_MAX_DUTY
    // speed=0 → duty=50% → 停止
    // speed=+100 → duty=100% → 全速正转
    // speed=-100 → duty=0% → 全速反转
    uint32_t duty = (uint32_t)((speed + 100) * MOTOR_PWM_MAX_DUTY / 200);

    ledc_set_duty(LEDC_LOW_SPEED_MODE, motor_channel[motor], duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, motor_channel[motor]);
}

void DogControl::MotorStop()
{
    for (int i = 0; i < 4; i++) {
        MotorSetSpeed(i, 0);
    }
}

// ---------- 编码器读取 ----------
int32_t DogControl::EncoderGetCount(int encoder)
{
    if (encoder < 0 || encoder > 3) return 0;
    int count = 0;
    if (pcnt_unit_[encoder]) {
        pcnt_unit_get_count(pcnt_unit_[encoder], &count);
    }
    return count;
}

void DogControl::EncoderReset()
{
    for (int i = 0; i < 4; i++) {
        if (pcnt_unit_[i]) pcnt_unit_clear_count(pcnt_unit_[i]);
    }
}

// ---------- UART串口接收任务（麦克纳姆轮控制） ----------
// 通过蓝牙串口(UART0, GPIO43/44)接收控制命令：
//   0x40 = 前进    0x41 = 后退
//   0x42 = 左平移  0x43 = 右平移
//   其他 = 停止
// 麦克纳姆轮布局（俯视）：
//   左前(M0) ---- 右前(M1)
//      |             |
//   左后(M2) ---- 右后(M3)
void DogControl::UartTaskEntry(void *parameter)
{
    DogControl *ctrl = static_cast<DogControl *>(parameter);
    ctrl->UartTaskLoop();
}

void DogControl::UartTaskLoop()
{
    // 初始化UART1（接蓝牙模块RX，仅接收电机指令，不发送）
    // 蓝牙模块只发送不接收，ESP32端仅需RX，TX断开避免与UART0控制台(GPIO43/44)冲突
    uart_config_t uart_cfg = {};
    uart_cfg.baud_rate = 9600;
    uart_cfg.data_bits = UART_DATA_8_BITS;
    uart_cfg.parity    = UART_PARITY_DISABLE;
    uart_cfg.stop_bits = UART_STOP_BITS_1;
    uart_cfg.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    uart_cfg.source_clk = UART_SCLK_DEFAULT;

    esp_err_t ret = uart_param_config(UART_NUM_1, &uart_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "UART1参数配置失败: %s", esp_err_to_name(ret));
        vTaskDelete(NULL);
        return;
    }

    // TX=断开, RX=GPIO0 (复用Boot按钮引脚，启动后可用)
    ret = uart_set_pin(UART_NUM_1, UART_PIN_NO_CHANGE, GPIO_NUM_0, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "UART1引脚配置失败: %s", esp_err_to_name(ret));
        vTaskDelete(NULL);
        return;
    }

    ret = uart_driver_install(UART_NUM_1, 256, 0, 0, NULL, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "UART1驱动安装失败: %s", esp_err_to_name(ret));
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "===== UART串口控制任务启动 (UART1, 9600bps, TX=断开, RX=GPIO0) =====");

    const int SPEED = 25;  // 电机速度 (0~100)
    uint8_t data;

    while (true) {
        int len = uart_read_bytes(UART_NUM_1, &data, 1, pdMS_TO_TICKS(100));
        if (len > 0) {
            switch (data) {
                case 0x40:  // 前进
                    ESP_LOGI(TAG, "前进");
                    MotorSetSpeed(0, SPEED);
                    MotorSetSpeed(1, SPEED);
                    MotorSetSpeed(2, SPEED);
                    MotorSetSpeed(3, SPEED);
                    break;

                case 0x41:  // 后退
                    ESP_LOGI(TAG, "后退");
                    MotorSetSpeed(0, -SPEED);
                    MotorSetSpeed(1, -SPEED);
                    MotorSetSpeed(2, -SPEED);
                    MotorSetSpeed(3, -SPEED);
                    break;

                case 0x42:  // 左平移
                    ESP_LOGI(TAG, "左平移");
                    MotorSetSpeed(0, -SPEED);
                    MotorSetSpeed(1, SPEED);
                    MotorSetSpeed(2, SPEED);
                    MotorSetSpeed(3, -SPEED);
                    break;

                case 0x43:  // 右平移
                    ESP_LOGI(TAG, "右平移");
                    MotorSetSpeed(0, SPEED);
                    MotorSetSpeed(1, -SPEED);
                    MotorSetSpeed(2, -SPEED);
                    MotorSetSpeed(3, SPEED);
                    break;

                default:    // 停止
                    ESP_LOGI(TAG, "停止 (0x%02X)", data);
                    MotorStop();
                    break;
            }
        }
    }
}

// 清除宏定义（避免全局污染）
#undef DEFINE_DOG_COMMAND_IMPL
