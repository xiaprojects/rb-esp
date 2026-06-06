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
 * 
 * Community edition will be free for all builders and personal use as defined by the licensing model
 * Dual licensing for commercial agreement is available
 *
*/
#pragma once
#include "RB02.h"
#ifdef RB_ENABLE_EMS
#include "lvgl.h"
lv_obj_t *RB04_EMS_CreateScreen(lv_obj_t *parent, RB02_Status *status);
void RB04_EMS_Tick(RB02_Status *status,RB04_EMSData *emsStatus);
void RB04_EMS_Touch_N(void *status);
void RB04_EMS_Touch_S(void *status);
void RB04_EMS_Touch_W(void *status);
void RB04_EMS_Touch_E(void *status);
#endif