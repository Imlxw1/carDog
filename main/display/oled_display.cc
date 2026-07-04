#include "oled_display.h"
#include <font_awesome.h>

#include <string>
#include <algorithm>

#include <esp_log.h>
#include <esp_err.h>
#include <esp_lvgl_port.h>

#define TAG "OledDisplay"

LV_FONT_DECLARE(font_awesome_30_1);

OledDisplay::OledDisplay(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_handle_t panel,
    int width, int height, bool mirror_x, bool mirror_y, DisplayFonts fonts)
    : panel_io_(panel_io), panel_(panel), fonts_(fonts) {
    width_ = width;
    height_ = height;

    ESP_LOGI(TAG, "Initialize LVGL");
    lvgl_port_cfg_t port_cfg = ESP_LVGL_PORT_INIT_CONFIG();
    port_cfg.task_priority = 1;
    port_cfg.timer_period_ms = 50;
    lvgl_port_init(&port_cfg);

    ESP_LOGI(TAG, "Adding LCD screen");
    const lvgl_port_display_cfg_t display_cfg = {
        .io_handle = panel_io_,
        .panel_handle = panel_,
        .control_handle = nullptr,
        .buffer_size = static_cast<uint32_t>(width_ * height_),
        .double_buffer = false,
        .trans_size = 0,
        .hres = static_cast<uint32_t>(width_),
        .vres = static_cast<uint32_t>(height_),
        .monochrome = true,
        .rotation = {
            .swap_xy = false,
            .mirror_x = mirror_x,
            .mirror_y = mirror_y,
        },
        .flags = {
            .buff_dma = 1,
            .buff_spiram = 0,
            .sw_rotate = 0,
            .full_refresh = 0,
            .direct_mode = 0,
        },
    };

    display_ = lvgl_port_add_disp(&display_cfg);
    if (display_ == nullptr) {
        ESP_LOGE(TAG, "Failed to add display");
        return;
    }

    if (height_ == 64) {
        SetupUI_128x64();
    } else {
        SetupUI_128x32();
    }
}

OledDisplay::~OledDisplay() {
    if (content_ != nullptr) {
        lv_obj_del(content_);
    }
    if (status_bar_ != nullptr) {
        lv_obj_del(status_bar_);
    }
    if (side_bar_ != nullptr) {
        lv_obj_del(side_bar_);
    }
    if (container_ != nullptr) {
        lv_obj_del(container_);
    }

    if (panel_ != nullptr) {
        esp_lcd_panel_del(panel_);
    }
    if (panel_io_ != nullptr) {
        esp_lcd_panel_io_del(panel_io_);
    }
    lvgl_port_deinit();
}

bool OledDisplay::Lock(int timeout_ms) {
    return lvgl_port_lock(timeout_ms);
}

void OledDisplay::Unlock() {
    lvgl_port_unlock();
}

void OledDisplay::SetupUI_128x64() {
    DisplayLockGuard lock(this);

    auto screen = lv_screen_active();
    lv_obj_set_style_text_font(screen, fonts_.text_font, 0);
    // 设置屏幕背景为黑色
    lv_obj_set_style_bg_color(screen, lv_color_black(), 0);

    /* Container */
    container_ = lv_obj_create(screen);
    lv_obj_set_size(container_, LV_HOR_RES, LV_VER_RES);
    lv_obj_set_flex_flow(container_, LV_FLEX_FLOW_ROW);
    lv_obj_set_style_pad_all(container_, 12, 0);
    lv_obj_set_style_border_width(container_, 0, 0);
    lv_obj_set_align(container_, LV_ALIGN_CENTER);
    // 容器背景设为黑色（与屏幕一致）
    lv_obj_set_style_bg_color(container_, lv_color_black(), 0);

    // 左侧眼睛（改为蓝色）
    content_left_ = lv_obj_create(container_);
    lv_obj_set_size(content_left_, 40, 40);
    lv_obj_set_style_bg_color(content_left_, lv_color_hex(0x0000FF), 0); // 蓝色背景
    lv_obj_set_style_border_width(content_left_, 2, 0);
    lv_obj_set_style_border_color(content_left_, lv_color_white(), 0); // 白色边框
    lv_obj_set_style_radius(content_left_, 11, 0);
    lv_obj_set_style_translate_x(content_left_, 8, LV_PART_MAIN);

    // 右侧眼睛（改为蓝色）
    content_right_ = lv_obj_create(container_);
    lv_obj_set_size(content_right_, 40, 40);
    lv_obj_set_style_bg_color(content_right_, lv_color_hex(0x0000FF), 0); // 蓝色背景
    lv_obj_set_style_border_width(content_right_, 2, 0);
    lv_obj_set_style_border_color(content_right_, lv_color_white(), 0); // 白色边框
    lv_obj_set_style_radius(content_right_, 11, 0);
    lv_obj_set_style_translate_x(content_right_, 10, LV_PART_MAIN);

    // 启动动画
    BlinkEyes();
    LookAround();
}

void OledDisplay::SetupUI_128x32() {
    DisplayLockGuard lock(this);

    auto screen = lv_screen_active();
    lv_obj_set_style_text_font(screen, fonts_.text_font, 0);
    // 设置屏幕背景为黑色
    lv_obj_set_style_bg_color(screen, lv_color_black(), 0);

    /* Container */
    container_ = lv_obj_create(screen);
    lv_obj_set_size(container_, LV_HOR_RES, LV_VER_RES);
    lv_obj_set_flex_flow(container_, LV_FLEX_FLOW_ROW);
    lv_obj_set_style_pad_all(container_, 5, 0);
    lv_obj_set_style_border_width(container_, 0, 0);
    lv_obj_set_align(container_, LV_ALIGN_CENTER);
    // 容器背景设为黑色
    lv_obj_set_style_bg_color(container_, lv_color_black(), 0);

    // 左侧眼睛（蓝色）
    content_left_ = lv_obj_create(container_);
    lv_obj_set_size(content_left_, 24, 24);
    lv_obj_set_style_bg_color(content_left_, lv_color_hex(0x0000FF), 0); // 蓝色
    lv_obj_set_style_border_width(content_left_, 2, 0);
    lv_obj_set_style_border_color(content_left_, lv_color_white(), 0);
    lv_obj_set_style_radius(content_left_, 16, 0);
    lv_obj_set_style_translate_x(content_left_, 3, LV_PART_MAIN);

    // 右侧眼睛（蓝色）
    content_right_ = lv_obj_create(container_);
    lv_obj_set_size(content_right_, 24, 24);
    lv_obj_set_style_bg_color(content_right_, lv_color_hex(0x0000FF), 0); // 蓝色
    lv_obj_set_style_border_width(content_right_, 2, 0);
    lv_obj_set_style_border_color(content_right_, lv_color_white(), 0);
    lv_obj_set_style_radius(content_right_, 16, 0);
    lv_obj_set_style_translate_x(content_right_, 3, LV_PART_MAIN);
}

void OledDisplay::SetChatMessage(const char* role, const char* content) {
    DisplayLockGuard lock(this);

    // 清空内容区域
    if (content_ != nullptr) {
        lv_obj_clean(content_);
    }

    // 创建标签显示角色和消息
    auto label = lv_label_create(content_);
    lv_label_set_text_fmt(label, "%s: %s", role, content);
    lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);
}

//眯眯眼表情
void OledDisplay::Clear() {
    DisplayLockGuard lock(this);

    auto screen = lv_screen_active();
    if (screen != nullptr) {
        lv_obj_clean(screen);
        // 确保清空后屏幕背景仍是黑色
        lv_obj_set_style_bg_color(screen, lv_color_black(), 0);
    }

    // 眯眯眼改为蓝色
    auto circle_left = lv_obj_create(screen);
    lv_obj_set_size(circle_left, 40, 20);
    lv_obj_set_style_bg_color(circle_left, lv_color_hex(0x0000FF), 0); // 蓝色
    lv_obj_set_style_border_width(circle_left, 2, 0);
    lv_obj_set_style_border_color(circle_left, lv_color_white(), 0);
    lv_obj_set_style_radius(circle_left, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_translate_x(circle_left, 14, LV_PART_MAIN);
    lv_obj_set_align(circle_left, LV_ALIGN_LEFT_MID);

    auto circle_right = lv_obj_create(screen);
    lv_obj_set_size(circle_right, 40, 20);
    lv_obj_set_style_bg_color(circle_right, lv_color_hex(0x0000FF), 0); // 蓝色
    lv_obj_set_style_border_width(circle_right, 2, 0);
    lv_obj_set_style_border_color(circle_right, lv_color_white(), 0);
    lv_obj_set_style_radius(circle_right, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_translate_x(circle_right, -13, LV_PART_MAIN);
    lv_obj_set_align(circle_right, LV_ALIGN_RIGHT_MID);

    ESP_LOGI(TAG, "Screen cleared and circles added");
}
//眯眯眼表情结尾

// 开心表情
void OledDisplay::ShowHappyFace() {
    DisplayLockGuard lock(this);

    // 获取当前屏幕
    auto screen = lv_screen_active();
    if (screen != nullptr) {
        lv_obj_clean(screen); // 直接清空屏幕内容
        // 设置屏幕背景为黑色
        lv_obj_set_style_bg_color(screen, lv_color_black(), 0);
    }

    // 大圆形（脸部，背景改为黑色）
    auto face = lv_obj_create(screen);
    lv_obj_set_size(face, 60, 60);  // 设置大圆形大小
    lv_obj_set_style_bg_color(face, lv_color_black(), 0);  // 黑色背景
    lv_obj_set_style_border_width(face, 2, 0);  // 边框宽度为2
    lv_obj_set_style_border_color(face, lv_color_white(), 0);  // 白色边框
    lv_obj_set_style_radius(face, LV_RADIUS_CIRCLE, 0);  // 设置为圆形
    lv_obj_set_align(face, LV_ALIGN_CENTER);  // 居中对齐

    // 左眼（改为蓝色）
    auto eye_left = lv_obj_create(face);
    lv_obj_set_size(eye_left, 15, 15);  // 设置眼睛大小
    lv_obj_set_style_bg_color(eye_left, lv_color_hex(0x0000FF), 0);  // 蓝色眼睛
    lv_obj_set_style_radius(eye_left, LV_RADIUS_CIRCLE, 0);  // 设置为圆形
    lv_obj_set_align(eye_left, LV_ALIGN_TOP_LEFT);  // 左上对齐
    lv_obj_set_style_translate_x(eye_left, -2, LV_PART_MAIN);  // 向右移动
    lv_obj_set_style_translate_y(eye_left, 1, LV_PART_MAIN);  // 向下移动

    // 右眼（改为蓝色）
    auto eye_right = lv_obj_create(face);
    lv_obj_set_size(eye_right, 15, 15);  // 设置眼睛大小
    lv_obj_set_style_bg_color(eye_right, lv_color_hex(0x0000FF), 0);  // 蓝色眼睛
    lv_obj_set_style_radius(eye_right, LV_RADIUS_CIRCLE, 0);  // 设置为圆形
    lv_obj_set_align(eye_right, LV_ALIGN_TOP_RIGHT);  // 右上对齐
    lv_obj_set_style_translate_x(eye_right, 2, LV_PART_MAIN);  // 向左移动
    lv_obj_set_style_translate_y(eye_right, 1, LV_PART_MAIN);  // 向下移动

    ESP_LOGI(TAG, "Happy face displayed");
}


//屏幕恢复开头
void OledDisplay::RestoreScreen() {
    DisplayLockGuard lock(this);

    // 获取当前屏幕
    auto screen = lv_screen_active();
    if (screen != nullptr) {
        lv_obj_clean(screen); // 清空屏幕内容
        // 恢复屏幕背景为黑色
        lv_obj_set_style_bg_color(screen, lv_color_black(), 0);

        // 恢复默认的 UI 布局
        if (height_ == 64) {
            SetupUI_128x64(); // 恢复 128x64 的默认布局
        } else {
            SetupUI_128x32(); // 恢复 128x32 的默认布局
        }
    }

    ESP_LOGI(TAG, "Screen restored to default layout");
}
//屏幕恢复结尾

void OledDisplay::BlinkEyes() {
    // 定义动画回调函数，用于改变正方体的高度
    auto blink_anim = [](void* obj, int32_t v) {
        lv_obj_set_size(static_cast<lv_obj_t*>(obj), 40, v);  // 动态改变高度
    };

    // 创建左眼的动画
    lv_anim_t anim_left;
    lv_anim_init(&anim_left);
    lv_anim_set_var(&anim_left, content_left_);
    lv_anim_set_exec_cb(&anim_left, blink_anim);
    lv_anim_set_values(&anim_left, 40, 20);  // 从40像素高度变为5像素
    lv_anim_set_time(&anim_left, 200);      // 动画时长150ms
    lv_anim_set_playback_time(&anim_left, 200);  // 回放时长150ms
    lv_anim_set_repeat_delay(&anim_left, 7000);  // 每3秒重复一次
    lv_anim_set_repeat_count(&anim_left, LV_ANIM_REPEAT_INFINITE);  // 无限循环
    lv_anim_set_path_cb(&anim_left, lv_anim_path_linear);  // 使用线性缓动效果
    lv_anim_start(&anim_left);

    // 创建右眼的动画
    lv_anim_t anim_right;
    lv_anim_init(&anim_right);
    lv_anim_set_var(&anim_right, content_right_);
    lv_anim_set_exec_cb(&anim_right, blink_anim);
    lv_anim_set_values(&anim_right, 40, 20);  // 从40像素高度变为5像素
    lv_anim_set_time(&anim_right, 200);      // 动画时长150ms
    lv_anim_set_playback_time(&anim_right, 200);  // 回放时长150ms
    lv_anim_set_repeat_delay(&anim_right, 7000);  // 每3秒重复一次
    lv_anim_set_repeat_count(&anim_right, LV_ANIM_REPEAT_INFINITE);  // 无限循环
    lv_anim_set_path_cb(&anim_right, lv_anim_path_linear);  // 使用线性缓动效果
    lv_anim_start(&anim_right);
}

void OledDisplay::RandomizeEyeMovement() {
    // TODO: 随机眼部运动动画
}

void OledDisplay::AnimateMovement() {
    // TODO: 动作动画
}

void OledDisplay::LookAround() {
    // 定义动画回调函数，用于改变左眼的水平位置和大小
    auto look_anim_left = [](void* obj, int32_t v) {
        lv_obj_set_style_translate_x(static_cast<lv_obj_t*>(obj), v, LV_PART_MAIN);
        if (v < 0) {  // 当左眼向左移动时，变大
            lv_obj_set_size(static_cast<lv_obj_t*>(obj), 45, 45);  // 变大为50x50像素
        } else {  // 返回时恢复正常大小
            lv_obj_set_size(static_cast<lv_obj_t*>(obj), 40, 40);  // 恢复为40x40像素
        }
    };

    // 定义右眼的动画回调函数
    auto look_anim = [](void* obj, int32_t v) {
        lv_obj_set_style_translate_x(static_cast<lv_obj_t*>(obj), v, LV_PART_MAIN);
    };

    // 创建左眼的左看动画
    lv_anim_t anim_left;
    lv_anim_init(&anim_left);
    lv_anim_set_var(&anim_left, content_left_);
    lv_anim_set_exec_cb(&anim_left, look_anim_left);
    lv_anim_set_values(&anim_left, 8, -10);  // 从右移到左移
    lv_anim_set_time(&anim_left, 300);      // 动画时长300ms
    lv_anim_set_playback_time(&anim_left, 350);  // 回放时长300ms
    lv_anim_set_playback_delay(&anim_left, 2000);  // 在终点停留4秒
    lv_anim_set_repeat_delay(&anim_left, 20000);  // 每5秒重复一次
    lv_anim_set_repeat_count(&anim_left, LV_ANIM_REPEAT_INFINITE);  // 无限循环
    lv_anim_set_path_cb(&anim_left, lv_anim_path_linear);  // 使用线性缓动效果
    lv_anim_start(&anim_left);

    // 创建右眼的左看动画
    lv_anim_t anim_right;
    lv_anim_init(&anim_right);
    lv_anim_set_var(&anim_right, content_right_);
    lv_anim_set_exec_cb(&anim_right, look_anim);
    lv_anim_set_values(&anim_right, 10, -10);  // 从左移到右移
    lv_anim_set_time(&anim_right, 300);      // 动画时长300ms
    lv_anim_set_playback_time(&anim_right, 350);  // 回放时长300ms
    lv_anim_set_playback_delay(&anim_right, 2000);  // 在终点停留4秒
    lv_anim_set_repeat_delay(&anim_right, 20000);  // 每5秒重复一次
    lv_anim_set_repeat_count(&anim_right, LV_ANIM_REPEAT_INFINITE);  // 无限循环
    lv_anim_set_path_cb(&anim_right, lv_anim_path_linear);  // 使用线性缓动效果
    lv_anim_start(&anim_right);
}
