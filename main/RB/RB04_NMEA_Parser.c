

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
 * 09 -> Display with Slave device for RB-01, RB-02, RB-03, RB-04, RB-05, RB-06, RB-07 and RB-08
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
static int16_t parse_int_token(const char *s, size_t len)
{
    if (s == NULL || len == 0)
        return 0;
    int sign = 1;
    size_t i = 0;
    if (s[0] == '-')
    {
        sign = -1;
        i = 1;
    }
    int16_t value = 0;
    for (; i < len; i++)
    {
        char c = s[i];
        if (c == '.' || c == '*' || c == '\r' || c == '\n')
            break;
        if (c < '0' || c > '9')
            break;
        value = value * 10 + (c - '0');
    }
    return sign * value;
}

uint8_t RB04_NMEA_EMS_ParseData(const uint8_t *data, uint8_t len, RB04_EMSData *emsData)
{
    if (data == NULL || len == 0 || emsData == NULL)
        return 0;

    /* Copy input into a local nul-terminated buffer for safe string ops */
    char buf[256];
    size_t copyLen = (len < (sizeof(buf) - 1)) ? len : (sizeof(buf) - 1);
    memcpy(buf, data, copyLen);
    buf[copyLen] = '\0';

    uint8_t parsed = 0;

    const char *p = buf;
    while ((p = strchr(p, '$')) != NULL)
    {
        /* Parse $RBEMS sentence */
        if (strncmp(p + 1, "RBEMS,", 6) == 0)
        {
            const char *q = p + 7; /* start of first numeric field */
            const char *tok = q;
            int field = 0;
            while (*q && *q != '\r' && *q != '\n' && *q != '*')
            {
                if (*q == ',')
                {
                    int16_t val = parse_int_token(tok, (size_t)(q - tok));
                    switch (field)
                    {
                    case 0:
                        emsData->rpm = val;
                        break;
                    case 1:
                        emsData->manifoldPressure = val;
                        break;
                    case 2:
                        emsData->oilTemperature = val;
                        break;
                    case 3:
                        emsData->oilPressure = val;
                        break;
                    case 4:
                        emsData->coolantTemperature = val;
                        break;
                    case 5:
                        emsData->outTemperature = val;
                        break;
                    case 6:
                        emsData->fuelLevel = val;
                        break;
                    case 7:
                        emsData->batteryVoltage = val;
                        break;
                    default:
                        break;
                    }
                    field++;
                    tok = q + 1;
                }
                q++;
            }
            /* Last token before '*' or EOL */
            if (tok < q)
            {
                int val = parse_int_token(tok, (size_t)(q - tok));
                switch (field)
                {
                case 0:
                    emsData->rpm = (uint16_t)val;
                    break;
                case 1:
                    emsData->manifoldPressure = (uint8_t)val;
                    break;
                case 2:
                    emsData->oilTemperature = (int8_t)val;
                    break;
                case 3:
                    emsData->oilPressure = (uint8_t)val;
                    break;
                case 4:
                    emsData->coolantTemperature = (int8_t)val;
                    break;
                case 5:
                    emsData->outTemperature = (int8_t)val;
                    break;
                case 6:
                    emsData->fuelLevel = (uint8_t)val;
                    break;
                case 7:
                    emsData->batteryVoltage = (uint8_t)val;
                    break;
                default:
                    break;
                }
            }
            parsed = 1;
            p = q;
            continue;
        }

        /* Parse $RBCYL sentence */
        if (strncmp(p + 1, "RBCYL,", 6) == 0)
        {
            const char *q = p + 7; /* start of first numeric field */
            const char *tok = q;
            int field = 0;
            while (*q && *q != '\r' && *q != '\n' && *q != '*')
            {
                if (*q == ',')
                {
                    int16_t val = parse_int_token(tok, (size_t)(q - tok));
                    if (field >= 0 && field < RB_EMS_VERTICAL_BARS_NUMBER)
                    {
                        emsData->chtCelsius[field] = val;
                    }
                    else if (field >= RB_EMS_VERTICAL_BARS_NUMBER && field < RB_EMS_VERTICAL_BARS_NUMBER * 2)
                    {
                        emsData->egtCelsius[field - RB_EMS_VERTICAL_BARS_NUMBER] = val;
                    }
                    field++;
                    tok = q + 1;
                }
                q++;
            }
            /* Last token */
            if (tok < q)
            {
                int val = parse_int_token(tok, (size_t)(q - tok));
                if (field >= 0 && field < RB_EMS_VERTICAL_BARS_NUMBER)
                {
                    // chtCelsius is int16_t: cylinder head temperatures pass
                    // 255 C routinely, so do not narrow to uint8_t here.
                    emsData->chtCelsius[field] = (int16_t)val;
                }
                else if (field >= RB_EMS_VERTICAL_BARS_NUMBER && field < RB_EMS_VERTICAL_BARS_NUMBER * 2)
                {
                    emsData->egtCelsius[field - RB_EMS_VERTICAL_BARS_NUMBER] = (int16_t)val;
                }
            }
            parsed = 1;
            p = q;
            continue;
        }

        p++;
    }
    if (parsed)
    {
        // Only an EMS sentence may mark the readings fresh. RB_Usb.c feeds
        // every console line to this parser, so a GPS sentence used to keep
        // dead engine data looking current.
        emsData->obsoleteData = 0;
    }
    return parsed;
}