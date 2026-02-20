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
 * Starting from version 1.1.3 both of displays are supported: 2.8 and 2.1
 *
 * RB02.c
 * Implementation of RoastBeef PN 02 The basic six pack version
 *
 * Features:
 * - GPS Speed -> Requires NMEA TTL RS232 GPS Reveiver such as ublox
 * - Attitude indicator
 * - Turn & Slip
 * - GPS Gyro-Track -> Requires NMEA TTL RS232 GPS Reveiver such as ublox
 * - Altimeter
 * - Variometer
 * - Chronometer
 *
 * Demo Features:
 * - Synthetic vision lateral, implemented in RB-01
 * - Synthetic vision front, implemented in RB-01
 *
 * Integrated Features:
 * - Speed indicator
 * - Track indicator
 * - Radar display
 * - GPS Time
 *
 * Supported hardware:
 * - ESP32-S3 2.1" Inch Round display 480x480
 * - ESP32-S3 2.8" Inch Round display 480x480 NON TOUCH
 * - ESP32-S3 2.8" Inch Round display 480x480 TOUCH
 * - https://www.waveshare.com/esp32-s3-touch-lcd-2.8c.htm
 */

#ifndef RB02_IMAGES
#define RB02_IMAGES

/* Warning updating from LVGL 8.x to 9.x many high quality images with or without resolution are not fully supported */
#if LVGL_VERSION_MAJOR >=9
// Do the trick
#undef LV_COLOR_DEPTH
#define LV_COLOR_DEPTH 32
#else
#endif

#ifdef RB_ENABLE_ALD
// Altimeter Digital
#include "DigitFont100x25.c"
#include "DigitFont70x20.c"
#endif

#ifdef RB_ENABLE_SPD
// Speed
#include "arcRed.c"
#include "arcYellow.c"
#include "arcGreen.c"
#include "arcWhite.c"
#endif

#ifdef RB_ENABLE_ATT
// Attitude
#include "att_circle_top_TL.c"
#include "att_circle_top_TR.c"
#include "att_circle_top_T.c"
#include "att_middle_big.c"
#include "att_aircraft.c"
#include "att_tri.c"
#endif

#ifdef RB_ENABLE_VAR
// Variometer
#include "RoundVariometer.c"
#endif

#ifdef RB_ENABLE_ALT
// Altimeter
#include "RoundAltimeter.c"
#ifndef LV_ATTRIBUTE_IMG_FI_NEEDLE
#include "fi_needle.c"
#include "fi_needle_small.c"
#endif
#endif

#ifdef RB_ENABLE_SPD
#ifndef LV_ATTRIBUTE_IMG_FI_NEEDLE
#include "fi_needle.c"
#include "fi_needle_small.c"
#endif
#endif

#ifdef RB_ENABLE_TRN
// Slip & Turn
#include "turn_coordinator.c"
#include "fi_tc_airplane.c"
#endif


#ifdef ENABLE_DEMO_SCREENS
#include "RoundSynthViewSide.c"
#include "Radar.c"
#include "RoundMapWithControlledSpaces.c"
#include "RoundHSI.c"
#endif

#if LVGL_VERSION_MAJOR >=9
// Do the trick
#undef LV_COLOR_DEPTH
#define LV_COLOR_DEPTH CONFIG_LV_COLOR_DEPTH
#else
#endif

#endif