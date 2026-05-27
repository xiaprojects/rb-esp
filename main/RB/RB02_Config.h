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

#include "lvgl.h"

#if LVGL_VERSION_MAJOR >=9

#if LV_COLOR_DEPTH == 1
#define LV_COLOR_SIZE 8
#elif LV_COLOR_DEPTH == 8
#define LV_COLOR_SIZE 8
#elif LV_COLOR_DEPTH == 16
#define LV_COLOR_SIZE 16
#elif LV_COLOR_DEPTH == 32
#define LV_COLOR_SIZE 32
#else
#error "Invalid LV_COLOR_DEPTH in lv_conf.h! Set it to 1, 8, 16 or 32!"
#endif
/*
#if LV_COLOR_DEPTH == 1 || LV_COLOR_DEPTH == 8
#define LV_IMG_PX_SIZE_ALPHA_BYTE 2
#elif LV_COLOR_DEPTH == 16
#define LV_IMG_PX_SIZE_ALPHA_BYTE 3
#elif LV_COLOR_DEPTH == 32
#define LV_IMG_PX_SIZE_ALPHA_BYTE 4
#endif
*/
//#define LV_IMG_PX_SIZE_ALPHA_BYTE LV_COLOR_DEPTH / 8
#define LV_IMG_PX_SIZE_ALPHA_BYTE LV_COLOR_FORMAT_GET_SIZE(LV_COLOR_FORMAT_NATIVE_WITH_ALPHA)
#endif
#include "RB02_Defines.h"

#ifdef RB_ENABLE_GPS
#include "RB02_NMEA.h"
#endif

#ifdef RB_ENABLE_MAP
#include "RB02_GPSMap.h"
#endif

#ifdef RB_ENABLE_TRAFFIC
#include "RB05_Traffic.h"
#endif

#ifdef RB_ENABLE_CONSOLE
void RB02_Console_AppendLog(uint8_t sourceId,uint8_t logLevel,const char *string);
#endif

#define RB02_STRUCTURE_CONFIG 1

typedef struct __IMUdata {
    float x;
    float y;
    float z;
} IMUdata;

typedef struct {
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t dotw;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
}datetime_t;

#ifdef RB_ENABLE_TRK
typedef struct
{
    lv_obj_t *Numbers[12];
    lv_obj_t *Screen_Track_TrackSource;
    lv_obj_t *Screen_Track_TrackText;
    lv_obj_t *parent;
    int16_t AttitudeYawCorrection;
} RB02_Gyro;
#endif

#ifdef RB_ENABLE_ALT
typedef struct
{
    lv_obj_t *labelGPSAltitude;
} RB02_AltimeterAnalog;
#endif

typedef struct
{
    lv_obj_t *SettingsSpeedSummary;
    lv_obj_t *SettingsOperativeSummary;
#ifdef RB_ENABLE_TRK
    RB02_Gyro Gyro;
#endif
#ifdef RB_ENABLE_ALT
    RB02_AltimeterAnalog altimeterAnalog;
#endif
#ifdef RB_ENABLE_CONSOLE
    lv_obj_t *console;
#endif
    lv_obj_t *Loading_slider;
#ifdef RB_ENABLE_EMS
    void *ems;
#endif    
#ifdef RB_ENABLE_GPS_DIAG
  lv_obj_t *tGPSDiag;
#endif
#ifdef RB_ENABLE_CONSOLE
  lv_obj_t *tConsole;
#endif
#ifdef RB_ENABLE_EMS
  lv_obj_t *tEMS;
#endif
    lv_obj_t *panelMountAlignmentLabelPitch;
    lv_obj_t *panelMountAlignmentLabelRoll;
    lv_obj_t *panelMountAlignmentLabelYaw;
    lv_obj_t *panelMountAlignmentLabelHelper;
    lv_obj_t *SettingStatus2;

#ifdef RB_ENABLE_SPD
    lv_obj_t *labelGPSSpeed;
#ifdef RB_ENABLE_IAS
    lv_obj_t *labelIASSpeed;
#endif
#endif

} RB02_UI;

typedef struct
{
    IMUdata GyroHardwareCalibration;  
    uint8_t settingsCalibrateOnBoot;
#ifdef RB02_ESP_BLUETOOTH
    uint8_t settingsBluetoothEnabled;
    uint8_t Operative_Bluetooth;
    uint8_t settingsBluetoothGPS;
#endif
    uint8_t settingsAutoQNH;
    int32_t bmp280override;
    uint8_t bmp280Address;
    uint8_t qnhIsInInches;    
    uint8_t structureVersion;
#ifdef RB_ENABLE_DATALOGGER
    uint8_t settingsEnableDataLoggerRecording;
#endif
#ifdef RB_ENABLE_GPS
    gps_t NMEA_DATA;
#endif
#ifdef RB_ENABLE_MAP
RB02_GpsMapStatus gpsMapStatus;
#endif
#ifdef RB_ENABLE_TRAFFIC
RB05_TrafficStatus trafficStatus;
#endif
    RB02_UI ui;

#if RB_LICENSE_TYPE == RB_LICENSE_TYPE_COMMERCIAL
    bool licenseValid;
#else
#endif
} RB02_Status;

extern RB02_Status *rb02Status;

RB02_Status *singletonConfig();

#ifdef RB02_ESP_BLUETOOTH
uint8_t RB02_Config_NVS_Store_BluetoothSettings();
#define NVS_KEY_BT_ENABLE           "btenable"
#define NVS_KEY_BT_GPS              "usebtgps"

void RB02_Config_Set_OperativeBluetooth(uint8_t operative);
#endif


// NVS Defines
#define NVS_STORAGE             "storage"
#if LVGL_VERSION_MAJOR >=9

#define LV_IMG_CF_INDEXED_1BIT LV_COLOR_FORMAT_I1
#define LV_IMG_CF_INDEXED_2BIT LV_COLOR_FORMAT_I2
#define LV_IMG_CF_INDEXED_4BIT LV_COLOR_FORMAT_I4
#define LV_IMG_CF_INDEXED_8BIT LV_COLOR_FORMAT_I8
#define LV_IMG_CF_TRUE_COLOR_ALPHA LV_COLOR_FORMAT_NATIVE_WITH_ALPHA
#define LV_IMG_CF_TRUE_COLOR LV_COLOR_FORMAT_NATIVE
#define LV_IMG_CF_ALPHA_1BIT  LV_COLOR_FORMAT_I1
/*
#define LV_IMG_CF_INDEXED_1BIT LV_COLOR_FORMAT_RGB888
#define LV_IMG_CF_INDEXED_2BIT LV_COLOR_FORMAT_RGB888
#define LV_IMG_CF_INDEXED_4BIT LV_COLOR_FORMAT_RGB888
#define LV_IMG_CF_INDEXED_8BIT LV_COLOR_FORMAT_RGB888
#define LV_IMG_CF_TRUE_COLOR_ALPHA LV_COLOR_FORMAT_RGB888
#define LV_IMG_CF_TRUE_COLOR LV_COLOR_FORMAT_RGB888
#define LV_IMG_CF_ALPHA_1BIT  LV_COLOR_FORMAT_RGB888
*/ 

typedef uint8_t lv_img_cf_t;
#endif