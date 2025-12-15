#include "../RB/RB02.h"

#ifndef ST7701S_C
#define ST7701S_C
#if RB_02_DISPLAY_SIZE == RB_02_DISPLAY_28
// ESP32-S3-2.8C
#include "ST7701S_28.c"
#endif
#if RB_02_DISPLAY_SIZE == RB_02_DISPLAY_21
// ESP32-S3-2.1C
#include "ST7701S_21.c"
#endif
#if RB_02_DISPLAY_SIZE == RB_02_DISPLAY_17
// ESP32-S3-Touch-AMOLED-1.75
#include "SH8691_17.c"
#endif
#endif