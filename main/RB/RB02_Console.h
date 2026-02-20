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

#ifdef RB_ENABLE_CONSOLE
/*
typedef enum
{
  RB02_LOG_UNKNOWN,
  RB02_LOG_MAIN,
  RB02_LOG_GPS,
  RB02_LOG_UART,
  RB02_LOG_BMP,
  RB02_LOG_ATT,
  RB02_LOG_BLE,
  RB02_LOG_FLARM
} ConsoleSourceId;

typedef enum
{
  RB02_LOG_INFO,
  RB02_LOG_WARNING,
  RB02_LOG_ERROR
} ConsoleLogLevel;
*/

#define RB02_LOG_MAIN       1
#define RB02_LOG_INFO       0
#define RB02_LOG_WARNING    1
#define RB02_LOG_ERROR      2

#include "lvgl.h"
lv_obj_t *RB02_Console_CreateScreen(lv_obj_t *parent);
void RB02_Console_AppendLog(uint8_t sourceId,uint8_t logLevel,const char *string);
#endif