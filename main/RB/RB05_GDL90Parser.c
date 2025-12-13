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
 *
 * Community edition will be free for all builders and personal use as defined by the licensing model
 * Dual licensing for commercial agreement is available
 *
*/

#include "RB05_GDL90Parser.h"
#ifdef RB05_GDL90


#ifdef RB_ENABLE_GPS
#if RB01_GPS_PROTOCOL == RB01_GPS_PROTOCOL_BLE
void NMEA_ParseBuffer(const uint8_t *data, const int rxBytes, uint8_t SourceId);
#endif
#endif

void RB05_GDL90_ParseData(const uint8_t *data, uint8_t len)
{
    for (uint8_t x = 0; x < len; x++)
    {
        printf("%c",(char)data[x]);
        RB02_Config_Set_OperativeBluetooth(1);
    }
#ifdef RB_ENABLE_GPS
#if RB01_GPS_PROTOCOL == RB01_GPS_PROTOCOL_WIFI
    NMEA_ParseBuffer(data,len,RB01_GPS_PROTOCOL_WIFI);
#endif
#endif

}

#endif