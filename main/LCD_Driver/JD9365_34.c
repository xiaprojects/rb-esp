#include "../RB/RB02.h"

#ifndef JD9365_C
#define JD9365_C
#if RB_02_DISPLAY_SIZE == RB_02_DISPLAY_34
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "bsp/esp-bsp.h"
#include "bsp/display.h"
#include "bsp_board_extra.h"

extern esp_lcd_panel_handle_t panel_handle;
uint8_t LCD_Backlight = 0;
esp_lcd_panel_handle_t panel_handle = NULL;

void LCD_Init(void)
{
}
void Backlight_Init(void)
{
   bsp_display_backlight_on();
   bsp_display_brightness_set(50);

}

void Set_Backlight(uint8_t Light)
{

}

void Touch_Init(void)
{

}

#endif
#endif