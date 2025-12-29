#pragma once

/**
 * This file is part of RB.
 *
 * Copyright (C) 2024 XIAPROJECTS SRL
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
 * 
 * Community edition will be free for all builders and personal use as defined by the licensing model
 * Dual licensing for commercial agreement is available
 *
*/
#pragma once
#include "RB02.h"
#include "lvgl.h"

lv_obj_t *RB02_Navigator_CreateScreen(RB02_Status *status,lv_obj_t *parent);


typedef enum
{
  RB02_TOUCH_NW,
  RB02_TOUCH_N,
  RB02_TOUCH_NE,
  RB02_TOUCH_W,
  RB02_TOUCH_CENTER,
  RB02_TOUCH_E,
  RB02_TOUCH_SW,
  RB02_TOUCH_S,
  RB02_TOUCH_SE,
  RB02_TOUCH_UNKNOWN
} touchLocation;

typedef enum
{
#ifdef ENABLE_VENDOR
  RB02_TAB_SPL,
#endif
#ifdef RB_ENABLE_EMS
  RB02_TAB_EMS,
#endif
#ifdef ENABLE_DEMO_SCREENS
  RB02_TAB_SYS,
  RB02_TAB_SYN,
// RB02_TAB_HSI,
#endif
#ifdef RB_ENABLE_SPD
  RB02_TAB_SPD,
#endif
#ifdef RB_ENABLE_ATT
  RB02_TAB_ATT,
#endif
#ifdef RB_ENABLE_AAT
  RB02_TAB_AAT,
#endif
#ifdef RB_ENABLE_ALT
  RB02_TAB_ALT,
#endif
#ifdef RB_ENABLE_ALD
  RB02_TAB_ALD,
#endif
#ifdef RB_ENABLE_TRN
  RB02_TAB_TRN,
#endif
#ifdef RB_ENABLE_GPS
#ifdef RB_ENABLE_TRK
  RB02_TAB_TRK,
#endif
#ifdef RB_ENABLE_MAP
  RB02_TAB_MAP,
#endif
#endif
#ifdef RB_ENABLE_VAR
  RB02_TAB_VAR,
#endif
#ifdef RB_ENABLE_GMT
  RB02_TAB_GMT,
#endif
#ifdef RB_ENABLE_CLK
  RB02_TAB_CLK,
#endif
#ifdef RB_ENABLE_CHECKLIST
  RB02_TAB_CHK,
#endif
#ifdef RB_ENABLE_TRAFFIC
  RB02_TAB_RDR,
  RB02_TAB_TRA,
#endif
  RB02_TAB_SET,
#ifdef VIBRATION_TEST
  RB02_TAB_VBR,
#endif
#ifdef RB_ENABLE_GPS_DIAG
  RB02_TAB_GDG,
#endif
#ifdef RB_ENABLE_CONSOLE
  RB02_TAB_COS,
#endif
  RB02_TAB_DEV
} tabs;