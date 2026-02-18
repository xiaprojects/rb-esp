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
#include "RB02_Defines.h"
#include "QMI8658.h"

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
