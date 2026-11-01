/**
 * This file is part of RB.
 *
 * Copyright (C) 2026 XIAPROJECTS SRL
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
 * 08 -> Voice Recognition Box with LLM and Natural speaking and Voice Recorder
 * 09 -> Remote display SLAVE: attitude and air data from RB-Sim or RB-01 over USB
 * Cloud -> Cloud services for RB devices, including flight data recording, flight data analysis, flight sharing and more
 *
 * Community edition will be free for all builders and personal use as defined by the licensing model
 * Dual licensing for commercial agreement is available
 *
 * More about in https://www.rbavionics.com and Discord community
 *
 */
#pragma once
#include "RB02.h"

// Master side, in every build: the $RBAIR body without its checksum
int RB09_Slave_FormatAir(char *buf, size_t size, int32_t pressurePa, float qnhHpa, int32_t variometer);

#ifdef RB_ENABLE_SLAVE_DEVICE

// Data older than this stops driving the instruments and raises the INOP.
// RB02.c sends $RBATT at 10 Hz from rb_increase_lvgl_tick(); the $RBAIR rate
// is up to the master, 3 s lets a 1 Hz sender miss two sentences.
#ifndef RB09_SLAVE_ATTITUDE_TIMEOUT_US
#define RB09_SLAVE_ATTITUDE_TIMEOUT_US 1000000
#endif
#ifndef RB09_SLAVE_AIR_TIMEOUT_US
#define RB09_SLAVE_AIR_TIMEOUT_US 3000000
#endif

// $RBAIR field positions, also the bits of RB09_SlaveData.airPresent
#define RB09_RBAIR_PRESSURE       0
#define RB09_RBAIR_QNH            1
#define RB09_RBAIR_OAT            2
#define RB09_RBAIR_VARIOMETER     3
#define RB09_RBAIR_IAS            4
#define RB09_RBAIR_WIND           5
#define RB09_RBAIR_WIND_DIRECTION 6

uint8_t RB09_Slave_ParseData(const uint8_t *data, uint16_t len, RB09_SlaveData *slave);
void RB09_Slave_Loop(RB02_Status *status);
uint8_t RB09_Slave_AirspeedKmh10(const RB09_SlaveData *slave, int *kmh10);
#endif
