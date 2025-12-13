#include "../RB/RB02.h"


#if RB_02_DISPLAY_SIZE == RB_02_DISPLAY_28
// ESP32-S3-2.8C
#include "GT911.c"
#endif
#if RB_02_DISPLAY_SIZE == RB_02_DISPLAY_21
// ESP32-S3-2.1C
#include "CST820.c"
#endif
#if RB_02_DISPLAY_SIZE == RB_02_DISPLAY_17
// ESP32-S3-Touch-AMOLED-1.75
#include "FT5X06.c"
#endif