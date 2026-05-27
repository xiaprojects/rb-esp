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
 *
 * Community edition will be free for all builders and personal use as defined by the licensing model
 * Dual licensing for commercial agreement is available
 *
*/
#pragma once
#include "lvgl.h"
#include "RB02.h"
#ifdef RB_ENABLE_AAT
#include "RB02_NMEA.h"

#define RB_AAT_ARC_NUMBERS 15
#define RB_AAT_SKY_TILES 10

typedef struct
{
    lv_obj_t *lv_parent;
    lv_obj_t *lv_pitch;
    lv_obj_t *lv_roll;
    lv_obj_t *lv_speed;
    lv_obj_t *lv_speed_unit;
    lv_obj_t *lv_speed_background;
    lv_obj_t *lv_gyro;
    lv_obj_t *lv_gyro_pink;
    lv_obj_t *lv_track;
    lv_obj_t *lv_gmeter;
    lv_obj_t *lv_variometer;
    lv_obj_t *lv_altimeterM;
    lv_obj_t *lv_altimeterF;
    lv_obj_t *lv_altimeter_unit;
    lv_obj_t *lv_altimeter_background;
    lv_obj_t *lv_qnh;
    lv_obj_t *lv_ball;
    lv_obj_t *lv_left_arcs[RB_AAT_ARC_NUMBERS];
    lv_obj_t *lv_right_arcs[RB_AAT_ARC_NUMBERS];
    lv_obj_t *lv_right_arcs2[RB_AAT_ARC_NUMBERS];
    int32_t Altimeter;
    int32_t QNH;
    int32_t Variometer;
    int8_t GFactor;
    int8_t AttitudeYawDegreePerSecond;
    int8_t BallFactor;
    int8_t AttitudePitch;
    int8_t AttitudeRoll;
    int8_t SkyInverted;
    int16_t Speed;
    int16_t Track;
    uint8_t advancedAttitudeMaxHeigh100;
    int8_t *SkyMatrix;
    lv_obj_t *Lines;
    lv_obj_t *AttitudeLine;
    lv_obj_t *SkyTiles[RB_AAT_SKY_TILES * RB_AAT_SKY_TILES];
} RB02_AdvancedAttitude_Status;

extern RB02_AdvancedAttitude_Status advancedAttitude_Status;

lv_obj_t *RB02_AdvancedAttitude_CreateScreen(RB02_AdvancedAttitude_Status *aaStatus);
void RB02_AdvancedAttitude_Tick(RB02_AdvancedAttitude_Status *aaStatus, gps_t *gpsStatus, int32_t Altimeter, float QNH, int32_t Variometer);
#endif