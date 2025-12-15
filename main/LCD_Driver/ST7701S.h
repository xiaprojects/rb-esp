#include "../RB/RB02.h"

#ifndef ST7701S_INCLUDE
#define ST7701S_INCLUDE

// 1.2.1
#ifdef RB_02_28_WORKAROUND_BLACKSCREEN
#define EXAMPLE_LCD_PIXEL_CLOCK_MHZ 18
#define EXAMPLE_LCD_PIXEL_CLOCK_SPEED 4000000
#else
// 25 is the best performance achievable, but generates spike at 134 Mhz
// 18 is the original but generates a spike
#define EXAMPLE_LCD_PIXEL_CLOCK_MHZ 20
#define EXAMPLE_LCD_PIXEL_CLOCK_SPEED 40000000
#endif


#if RB_02_DISPLAY_SIZE == RB_02_DISPLAY_28
// ESP32-S3-2.8C
#include "ST7701S_28.h"
#endif
#if RB_02_DISPLAY_SIZE == RB_02_DISPLAY_21
// ESP32-S3-2.1C
#include "ST7701S_21.h"
#if CONFIG_EXAMPLE_AVOID_TEAR_EFFECT_WITH_SEM
extern SemaphoreHandle_t sem_vsync_end;
extern SemaphoreHandle_t sem_gui_ready;
#endif
#endif

#if RB_02_DISPLAY_SIZE == RB_02_DISPLAY_17
// ESP32-S3-Touch-AMOLED-1.75
#include "SH8691_17.h"
#endif

#endif