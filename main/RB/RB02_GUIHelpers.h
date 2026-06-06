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
#include "lvgl.h"
#include "RB02.h"

#define PI 3.14159265358979323846


lv_obj_t *RB02_GUIHelpers_CreateBase(lv_obj_t *parent, const lv_img_dsc_t *backgroundImageName);
uint8_t RB02_CheckfileExists(const char *filename);

uint16_t RB02_SuggestedQNH(float GPSAltitudeMeters,int32_t CurrentPressure);
float nmea_to_decimal(float nmea_coord);
uint8_t nmeaChecksum(const char *s);