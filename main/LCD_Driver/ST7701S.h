/**
 * This file is part of RB.
 *
 * Copyright (C) 2025 XIAPROJECTS SRL
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as published
 * by the Free Software Foundation, version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Affero General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program. If not, see <https://www.gnu.org/licenses/>.

 * This source is part of the project RB:
 * 01 -> Display with Synthetic vision, Autopilot and ADSB
 * 02 -> Display with SixPack
 * 03 -> Display with Autopilot, ADSB, Radio, Flight Computer
 * 04 -> Display with EMS: Engine monitoring system
 * 05 -> Display with Stratux BLE Traffic
 * 06 -> Display with Android 6.25" 7" 8" 10" 10.2"
 * 07 -> Display with Stratux BLE Traffic composed by RB-05 + RB-03 in the same box
 *
 * Community edition will be free for all builders and personal use as defined by the licensing model
 * Dual licensing for commercial agreement is available
 *
 */
// TODO: Rename this file to PanelDisplayDriver.h
#include "../RB/RB02.h"

#ifndef ST7701S_INCLUDE
#define ST7701S_INCLUDE

// 1.2.1
/*
    Moved into build machine and templates
#ifdef RB_02_28_WORKAROUND_BLACKSCREEN
#define EXAMPLE_LCD_PIXEL_CLOCK_MHZ 18
#define EXAMPLE_LCD_PIXEL_CLOCK_SPEED 4000000
#else
// 25 is the best performance achievable, but generates spike at 134 Mhz
// 18 is the original but generates a spike
#define EXAMPLE_LCD_PIXEL_CLOCK_MHZ 20
#define EXAMPLE_LCD_PIXEL_CLOCK_SPEED 40000000
#endif
*/

#define EXAMPLE_LCD_PIXEL_CLOCK_SPEED 40000000

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

#if RB_02_DISPLAY_SIZE ==  RB_02_DISPLAY_18
#endif
#if RB_02_DISPLAY_SIZE ==  RB_02_DISPLAY_2834
#include "ST7789.h"
#endif
#if RB_02_DISPLAY_SIZE ==  RB_02_DISPLAY_40
#endif


#if RB_02_DISPLAY_SIZE == RB_02_DISPLAY_17
// ESP32-S3-Touch-AMOLED-1.75
#include "SH8691_17.h"
#endif

#endif