#include "../RB/RB02.h"

#ifndef JD9365_H
#define JD9365_H
#if RB_02_DISPLAY_SIZE == RB_02_DISPLAY_34
#define EXAMPLE_LCD_V_RES 800
#define EXAMPLE_LCD_H_RES 800

extern uint8_t LCD_Backlight;

void LCD_Init(void);
void Backlight_Init(void);
void Set_Backlight(uint8_t Light);
void Touch_Init(void);


#endif
#endif