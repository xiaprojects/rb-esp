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
// TODO: Rename this file to PanelDisplayDriver.c
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
#if RB_02_DISPLAY_SIZE ==  RB_02_DISPLAY_18
#endif
#if RB_02_DISPLAY_SIZE ==  RB_02_DISPLAY_2834
// TODO: Migrate to new improved Driver
#include "esp_lcd_types.h"
extern lv_disp_drv_t disp_drv;
bool example_notify_lvgl_flush_ready(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx)
{
    lv_disp_drv_t *disp_driver = (lv_disp_drv_t *)user_ctx;
    lv_disp_flush_ready(disp_driver);
    return false;
}

#include "ST7789.c"
#include "Vernon_ST7789T/Vernon_ST7789T.c"
#endif
#if RB_02_DISPLAY_SIZE ==  RB_02_DISPLAY_40
#endif
#if RB_02_DISPLAY_SIZE == RB_02_DISPLAY_17
// ESP32-S3-Touch-AMOLED-1.75
#include "SH8691_17.c"
#include "../esp_lcd_sh8601/esp_lcd_sh8601.c"
#endif
#endif