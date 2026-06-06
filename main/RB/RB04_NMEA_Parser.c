

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
 * 06 -> Display with Android 6.25" 7" 8" 10" 10.2"
 * 07 -> Display with Stratux BLE Traffic composed by RB-05 + RB-03 in the same box
 * 08 -> Voice Recognition Box with LLM and Natural speaking and Voice Recorder
 * Cloud -> Cloud services for RB devices, including flight data recording, flight data analysis, flight sharing and more
 *
 * Community edition will be free for all builders and personal use as defined by the licensing model
 * Dual licensing for commercial agreement is available
 *
 * More about in https://www.rbavionics.com and Discord community
 *
 */

#include "RB02_Config.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

/*
Example of parsing an NMEA sentence for EMS data. The actual implementation will depend on the specific NMEA sentences being used and the format of the data within those sentences.

$RBEMS,1234,56,78,90,12,34,56,78,79*hh

Where:
- 1234 = RPM
- 56 = Manifold Pressure
- 78 = Oil Temperature
- 90 = Oil Pressure
- 12 = Coolant Temperature
- 34 = Out Temperature
- 56 = Fuel Pressure
- 78 = Fuel level left
- 79 = Fuel level right
- *hh = Checksum

$RBCYL,123,135,100,120,700,750,735,800*hh

Where:
- 123 = Cylinder 1 Temperature
- 135 = Cylinder 2 Temperature
- 100 = Cylinder 3 Temperature
- 120 = Cylinder 4 Temperature
- 700 = EGT 1 Temperature
- 750 = EGT 2 Temperature
- 735 = EGT 3 Temperature
- 800 = EGT 4 Temperature
- *hh = Checksum

*/

uint8_t RB04_NMEA_EMS_ParseData(const uint8_t *data, uint8_t len, RB04_EMSData *emsData)
{
    return 0;
}