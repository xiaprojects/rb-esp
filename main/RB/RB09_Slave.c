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

#include "RB09_Slave.h"
#include <stdio.h>

/**
 * The $RBAIR body rb_increase_lvgl_tick() sends, from the firmware units: Pa,
 * hPa and 0.01 feet/second. Outside the #ifdef so every master compiles it, and
 * the host test checks this sender against the parser below.
 * Temperature, air speed and wind stay empty: the BMP280 sits inside the case,
 * and no template has an air speed or wind source.
 * QNH carries 5 decimals: RB09_Slave_Loop() snaps it to 0.001 hPa, and with 4 a
 * 1015 hPa setting arrives as 1014.997, which the QNH labels truncate to 1014.
 */
int RB09_Slave_FormatAir(char *buf, size_t size, int32_t pressurePa, float qnhHpa, int32_t variometer)
{
    return snprintf(buf, size, "$RBAIR,%.4f,%.5f,,%.0f,,,",
                    pressurePa / 3386.39,
                    qnhHpa / 33.8639,
                    variometer * 60.0 / 100.0);
}

#ifdef RB_ENABLE_SLAVE_DEVICE
#include <string.h>
#include "esp_timer.h"
#include "RB02_DriverFactory.h"
#include "RB02_GUIHelpers.h"

/*
RB-09 reads no sensor of its own. RB-Sim on a PC, or an RB-01 USB with the NMEA
output enabled, streams these over the USB serial port:

$RBATT,-12.50,3.25,87.10,0.98,-0.21,0.05,0.10,-1.20,4.80,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.04*hh

Where, as rb_increase_lvgl_tick() in RB02.c sends it:
- -12.50, 3.25, 87.10 = Roll, Pitch, Yaw in degrees (AttitudeRoll, AttitudePitch, AttitudeYaw)
- 0.98, -0.21, 0.05   = Acceleration X Y Z in g, already filtered and panel aligned (AccelFiltered)
- 0.10, -1.20, 4.80   = Gyro X Y Z in degrees/second (GyroFiltered)
- 0.00, 0.00, 0.00    = GPS acceleration X Y Z in g fed to the attitude filter
- 0.00, 0.00, 0.00    = Magnetometer X Y Z, 0 until the master has one
- 0.00                = Compass in degrees
- 0.04                = Attitude filter beta (AttitudeBalanceAlphaAerobatics)
- *hh = Checksum

$RBAIR,29.921,29.92,15.0,500,120,20,270*hh

Where:
- 29.921 = Static pressure in inHg: send 3 decimals, 0.01 inHg is about 10 feet
- 29.92  = QNH in inHg: 2 decimals for a setting made in inHg, 5 for one made in
           hPa, see RB09_Slave_FormatAir()
- 15.0   = Outside temperature in Celsius
- 500    = Variometer in feet/minute
- 120    = Indicated air speed in km/h
- 20     = Wind speed in km/h
- 270    = Wind direction in degrees
- *hh = Checksum

Unlike $RBEMS the checksum is mandatory, so a corrupted line never reaches an
attitude indicator. Every field has a valid range, RB09_RBATT_RANGE and
RB09_RBAIR_RANGE below, and reads 0 when it is empty or blank, not a number,
out of that range, or missing because the sentence ends before it. A field may
be padded with blanks, as a fixed width printf writes it. A sentence is used
when at least one of its fields is valid, and dropped when none is.
*/

#define RB09_RBATT_FIELDS 17
#define RB09_RBAIR_FIELDS 7
#define RB09_PRESENT(mask, field) (((mask) >> (field)) & 1)

typedef struct
{
    float min;
    float max;
} RB09_Range;

// Values beyond these come from a broken master, not from an aircraft
static const RB09_Range RB09_RBATT_RANGE[RB09_RBATT_FIELDS] = {
    {-360, 360}, {-360, 360}, {-360, 360},       // Roll, Pitch, Yaw: degrees
    {-16, 16}, {-16, 16}, {-16, 16},             // Acceleration: g, the QMI8658 full scale
    {-2048, 2048}, {-2048, 2048}, {-2048, 2048}, // Gyro: degrees/second, the QMI8658 full scale
    {-16, 16}, {-16, 16}, {-16, 16},             // GPS acceleration: g
    // Magnetometer: no master has one yet, so any number parse_float_token() reads
    {-9999999, 9999999}, {-9999999, 9999999}, {-9999999, 9999999},
    {-360, 360},                                 // Compass: degrees
    {0, 1},                                      // Attitude filter beta
};

static const RB09_Range RB09_RBAIR_RANGE[RB09_RBAIR_FIELDS] = {
    // inHg: the BMP280 range of 300 to 1100 hPa
    [RB09_RBAIR_PRESSURE] = {300 / 33.8639, 1100 / 33.8639},
    // inHg: the QNH bounds of RB09_Slave_Loop(), 850 to 1100 hPa
    [RB09_RBAIR_QNH] = {850 / 33.8639, 1100 / 33.8639},
    [RB09_RBAIR_OAT] = {-100, 100},              // Celsius
    [RB09_RBAIR_VARIOMETER] = {-20000, 20000},   // feet/minute
    [RB09_RBAIR_IAS] = {0, 1000},                // km/h
    [RB09_RBAIR_WIND] = {0, 500},                // km/h
    [RB09_RBAIR_WIND_DIRECTION] = {0, 360},      // degrees
};

// As YAW_RATE_SMOOTHING_ALPHA in QMI8658.c, so the turn rate matches the master
#define RB09_YAW_RATE_SMOOTHING_ALPHA 0.01
float unwrapAngle(float prev, float current);

extern float QNH;
extern int32_t bmp280Pressure;
extern int32_t bmp280Temperature;
extern int32_t Altimeter;
extern int32_t Variometer;
extern uint8_t Operative_Attitude;
extern uint8_t Operative_BMP280;

/**
 * One numeric field: [-+]digits[.digits], blanks around it allowed. Anything
 * else returns 0, an empty or all blank field too, so a damaged field reads as
 * missing instead of as some other number.
 * At most 7 integer digits: no instrument value needs more, and it keeps every
 * conversion into the int32 instrument variables in range.
 */
static uint8_t parse_float_token(const char *s, const char *end, float *value)
{
    while (s < end && (*s == ' ' || *s == '\t'))
        s++;
    while (end > s && (end[-1] == ' ' || end[-1] == '\t'))
        end--;
    float sign = 1;
    if (s < end && (*s == '-' || *s == '+'))
    {
        if (*s == '-')
            sign = -1;
        s++;
    }
    int32_t integer = 0;
    int32_t fraction = 0;
    int32_t scale = 1;
    uint8_t integerDigits = 0;
    uint8_t sawDigit = 0;
    uint8_t sawDot = 0;
    for (; s < end; s++)
    {
        if (*s == '.' && sawDot == 0)
        {
            sawDot = 1;
            continue;
        }
        if (*s < '0' || *s > '9')
            return 0;
        sawDigit = 1;
        if (sawDot == 0)
        {
            if (++integerDigits > 7)
                return 0;
            integer = integer * 10 + (*s - '0');
        }
        else if (scale < 1000000)
        {
            // A float holds no more decimals than this anyway
            fraction = fraction * 10 + (*s - '0');
            scale *= 10;
        }
    }
    if (sawDigit == 0)
        return 0;
    *value = sign * (integer + (float)fraction / scale);
    return 1;
}

static int8_t hex_digit(char c)
{
    if (c >= '0' && c <= '9')
        return c - '0';
    if (c >= 'A' && c <= 'F')
        return c - 'A' + 10;
    if (c >= 'a' && c <= 'f')
        return c - 'a' + 10;
    return -1;
}

/**
 * Reads the fields of the sentence at sentence[0] == '$' into values, one per
 * entry of ranges: a field that is not a number in its range reads 0. Fields
 * past the ranges are ignored, so a master may append new ones. Returns a bit
 * per valid field, so 0 when none is, or the sentence is cut or fails its
 * checksum.
 */
static uint32_t parse_sentence(const char *sentence, const RB09_Range *ranges, uint8_t fields, float *values)
{
    // The sentence must reach its '*': a line end, another '$' or the end of
    // the buffer first means it was cut
    const char *end = sentence + 1;
    while (*end != '\0' && *end != '*' && *end != '\r' && *end != '\n' && *end != '$')
        end++;
    if (*end != '*')
        return 0;
    int8_t high = hex_digit(end[1]);
    int8_t low = (high < 0) ? -1 : hex_digit(end[2]);
    if (low < 0 || nmeaChecksum(sentence) != (uint8_t)((high << 4) | low))
        return 0;

    uint32_t valid = 0;
    const char *field = sentence + 7; /* after "$RBxxx," */
    for (uint8_t n = 0; n < fields; n++)
    {
        const char *comma = field;
        while (comma < end && *comma != ',')
            comma++;
        float value;
        if (parse_float_token(field, comma, &value) && value >= ranges[n].min && value <= ranges[n].max)
        {
            values[n] = value;
            valid |= 1u << n;
        }
        else
        {
            values[n] = 0;
        }
        // Past the last comma every field is empty
        field = (comma < end) ? comma + 1 : end;
    }
    return valid;
}

/**
 * Called by RB_Usb.c with each console line. Only stores what the master sent:
 * RB09_Slave_Loop() hands it to the instruments.
 */
uint8_t RB09_Slave_ParseData(const uint8_t *data, uint16_t len, RB09_SlaveData *slave)
{
    if (data == NULL || len == 0 || slave == NULL)
        return 0;

    /* Copy input into a local nul-terminated buffer for safe string ops */
    char buf[256];
    size_t copyLen = (len < (sizeof(buf) - 1)) ? len : (sizeof(buf) - 1);
    memcpy(buf, data, copyLen);
    buf[copyLen] = '\0';

    uint8_t parsed = 0;
    float values[RB09_RBATT_FIELDS];
    const char *p = buf;
    while ((p = strchr(p, '$')) != NULL)
    {
        /* Parse $RBATT sentence */
        if (strncmp(p + 1, "RBATT,", 6) == 0)
        {
            if (parse_sentence(p, RB09_RBATT_RANGE, RB09_RBATT_FIELDS, values) != 0)
            {
                slave->roll = values[0];
                slave->pitch = values[1];
                slave->yaw = values[2];
                slave->accel.x = values[3];
                slave->accel.y = values[4];
                slave->accel.z = values[5];
                slave->gyro.x = values[6];
                slave->gyro.y = values[7];
                slave->gyro.z = values[8];
                slave->gpsAcceleration.x = values[9];
                slave->gpsAcceleration.y = values[10];
                slave->gpsAcceleration.z = values[11];
                slave->mag.x = values[12];
                slave->mag.y = values[13];
                slave->mag.z = values[14];
                slave->compass = values[15];
                slave->beta = values[16];
                slave->attitudeTick = esp_timer_get_time();
                parsed++;
            }
            else
            {
                slave->rejected++;
            }
        }
        /* Parse $RBAIR sentence */
        else if (strncmp(p + 1, "RBAIR,", 6) == 0)
        {
            uint32_t valid = parse_sentence(p, RB09_RBAIR_RANGE, RB09_RBAIR_FIELDS, values);
            if (valid != 0)
            {
                slave->pressureInHg = values[RB09_RBAIR_PRESSURE];
                slave->qnhInHg = values[RB09_RBAIR_QNH];
                slave->outsideTemperature = values[RB09_RBAIR_OAT];
                slave->variometerFeetMin = values[RB09_RBAIR_VARIOMETER];
                slave->indicatedAirSpeedKmh = values[RB09_RBAIR_IAS];
                slave->windKmh = values[RB09_RBAIR_WIND];
                slave->windDirection = values[RB09_RBAIR_WIND_DIRECTION];
                // A 0 that was sent and a 0 that was not: airPresent tells them apart
                slave->airPresent = valid;
                slave->airTick = esp_timer_get_time();
                parsed++;
            }
            else
            {
                slave->rejected++;
            }
        }
        p++;
    }
    return parsed;
}

static uint8_t is_fresh(int64_t tick, int64_t now, int64_t timeout)
{
    return tick != 0 && now - tick < timeout;
}

/**
 * Driver loop of RB-09, in place of QMI8658_Loop() and Get_BMP280(): writes the
 * master's data into the same variables the local sensors would, and raises
 * the INOP when it goes stale.
 */
void RB09_Slave_Loop(RB02_Status *status)
{
    RB09_SlaveData *slave = &status->slaveData;
    int64_t now = esp_timer_get_time();

    if (is_fresh(slave->attitudeTick, now, RB09_SLAVE_ATTITUDE_TIMEOUT_US))
    {
        AttitudeRoll = slave->roll;
        AttitudePitch = slave->pitch;
        AttitudeYaw = slave->yaw;
        // Filtered and panel aligned by the master; they also stand in for the
        // raw readings, which the turn coordinator ball uses
        AccelFiltered = slave->accel;
        Accel = slave->accel;
        GyroFiltered = slave->gyro;
        Gyro = slave->gyro;
        // The master's G-meter logic, on the master's acceleration
        getGFactor();
        // As getAttitude() in QMI8658.c and at the same loop rate, so the turn
        // rate is smoothed over the same time as on the master
        if (slave->lastYawTick != 0 && now > slave->lastYawTick)
        {
            float elapsed = (now - slave->lastYawTick) / 1000000.0;
            float yaw_rate_dps = unwrapAngle(slave->lastYaw, AttitudeYaw) / elapsed;
            AttitudeYawDegreePerSecond = RB09_YAW_RATE_SMOOTHING_ALPHA * yaw_rate_dps + (1.0f - RB09_YAW_RATE_SMOOTHING_ALPHA) * AttitudeYawDegreePerSecond;
        }
        slave->lastYaw = AttitudeYaw;
        slave->lastYawTick = now;
        Operative_Attitude = 1;
    }
    else
    {
        // A frozen attitude must not look live. On reconnection the turn rate
        // restarts rather than differentiating across the gap.
        slave->lastYawTick = 0;
        Operative_Attitude = 0;
    }

    // No altitude without both pressure and QNH
    if (is_fresh(slave->airTick, now, RB09_SLAVE_AIR_TIMEOUT_US) == 0 ||
        RB09_PRESENT(slave->airPresent, RB09_RBAIR_PRESSURE) == 0 ||
        RB09_PRESENT(slave->airPresent, RB09_RBAIR_QNH) == 0)
    {
        Operative_BMP280 = 0;
        return;
    }
    float qnh = slave->qnhInHg * 33.8639;
    float pressure = slave->pressureInHg * 3386.39;
    // Same as Get_BMP280(): Altimeter is *100 feet
    float AltimeterNew = ((qnh * 100) - pressure) * 30.0;
    // Same range as Get_BMP280(), checked before anything is converted to int.
    // The QNH bounds are the recorded sea level extremes with margin.
    if (qnh < 850 || qnh > 1100 || AltimeterNew < -100000 || AltimeterNew > 1200000)
    {
        Operative_BMP280 = 0;
        return;
    }
    // An hPa setting crossed the wire in inHg: snap it back to 0.001 hPa so 1015
    // does not arrive as 1014.9999, which the QNH labels truncate to 1014.
    // In range by now, so the cast is safe.
    qnh = (int32_t)(qnh * 1000 + 0.5f) / 1000.0f;
    if (qnh != QNH)
    {
        // The master owns QNH: a local or automatic setting would only fight it
        QNH = qnh;
        status->settingsAutoQNH = 0;
        slave->qnhChanged = 1;
    }
    // Rounded, as the inHg on the wire is a fraction of a Pa off and truncating
    // would read 102966 as 102965. Positive after the range check.
    bmp280Pressure = pressure + 0.5f;
    Altimeter = AltimeterNew;
    // Variometer is 0.01 Feet/Second
    Variometer = slave->variometerFeetMin * 100.0 / 60.0;
    if (RB09_PRESENT(slave->airPresent, RB09_RBAIR_OAT))
    {
        bmp280Temperature = slave->outsideTemperature * 100;
    }
    Operative_BMP280 = 1;
}

/**
 * The master's indicated air speed in km/h * 10, as the speed tick wants it.
 * Returns 0 when the master sends none or it is stale; kmh10 may be NULL.
 */
uint8_t RB09_Slave_AirspeedKmh10(const RB09_SlaveData *slave, int *kmh10)
{
    if (RB09_PRESENT(slave->airPresent, RB09_RBAIR_IAS) == 0 ||
        is_fresh(slave->airTick, esp_timer_get_time(), RB09_SLAVE_AIR_TIMEOUT_US) == 0)
        return 0;
    if (kmh10 != NULL)
        *kmh10 = slave->indicatedAirSpeedKmh * 10;
    return 1;
}
#endif
