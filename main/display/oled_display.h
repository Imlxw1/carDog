#ifndef OLED_DISPLAY_H
#define OLED_DISPLAY_H

#include "display.h"

#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_ops.h>

// OLED 显示字体配置
struct DisplayFonts {
    const lv_font_t* text_font = nullptr;  // 文本字体，nullptr = LVGL默认字体
    const lv_font_t* icon_font = nullptr;  // 图标字体
};

class OledDisplay : public Display {
private:
    esp_lcd_panel_io_handle_t panel_io_ = nullptr;
    esp_lcd_panel_handle_t panel_ = nullptr;

    lv_disp_t* display_ = nullptr;
    lv_obj_t* status_bar_ = nullptr;
    lv_obj_t* content_ = nullptr;
    lv_obj_t* content_left_ = nullptr;
    lv_obj_t* content_right_ = nullptr;
    lv_obj_t* container_ = nullptr;
    lv_obj_t* side_bar_ = nullptr;

    DisplayFonts fonts_;

    virtual bool Lock(int timeout_ms = 0) override;
    virtual void Unlock() override;

    void SetupUI_128x64();
    void SetupUI_128x32();
    void BlinkEyes(); // 添加 BlinkEyes 方法的声明
    void LookAround();
    void RandomizeEyeMovement();
    void AnimateMovement(); // 添加 AnimateMovement 方法的声明

public:
    OledDisplay(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_handle_t panel, int width, int height, bool mirror_x, bool mirror_y,
                DisplayFonts fonts = DisplayFonts());
    ~OledDisplay();

    virtual void SetChatMessage(const char* role, const char* content) override;

    // 添加 Clear 方法的声明
    void Clear();
    void RestoreScreen(); // 添加 RestoreScreen 方法的声明
    void ShowHappyFace();//开心表情

};

#endif // OLED_DISPLAY_H
