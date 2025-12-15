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
*/


#define EXAMPLE1_LVGL_TICK_PERIOD_MS  1000


// 1.1.18 Unified single source for both displays
#define RB_02_DISPLAY_21 21
#define RB_02_DISPLAY_28 28
#define RB_02_DISPLAY_17 17

// 1.2.1
//#define RB_02_28_WORKAROUND_BLACKSCREEN 1

#define RB_LICENSE_TYPE_COMMUNITY   0
#define RB_LICENSE_TYPE_COMMERCIAL  1
#define RB_LICENSE_TYPE_CERTIFIED   2
#define RB_LICENSE_TYPE_DEVELOPER   3

#include "BuildMachine.h"
//#define RB_02_DISPLAY_TOUCH 1
#define RB_ENABLE_SETUP 1

//  1.1.3 Supports for 2.1 and 2.8 displays
//#define RB_02_DISPLAY_SIZE RB_02_DISPLAY_28

#ifndef RB_02_DISPLAY_SIZE
#error You must define the display size
#endif

#ifndef RB_PRODUCT_LINE
#error You must define the product line
#endif

#if RB_PRODUCT_LINE == 2
#define RB_PRODUCT_TITLE "RB 02"
#elif RB_PRODUCT_LINE == 4
#define RB_PRODUCT_TITLE "RB 04"
#elif RB_PRODUCT_LINE == 5
#define RB_PRODUCT_TITLE "RB 05"
#else
#error Product not found
#endif


#ifndef RB_LICENSE_TYPE
#error You must define the LICENSE
#endif


#define SCREEN_HEIGHT 480
#define SCREEN_WIDTH 480


// 1.1.24 int16_t
#define RB_GYRO_CALIBRATION_PRECISION 1000.0

// 1.1.19 Add Map capability
//#define RB_ENABLE_MAP 1
//#define RB_02_ENABLE_INTERNALMAP 1
//#define RB_02_ENABLE_EXTERNALMAP 1
//#define RB_ENABLE_CHECKLIST 1
//#define RB_ENABLE_DATALOGGER 1

// 1.1.20 Advanced Attitude Indicator
//#define RB_ENABLE_AAT 1


// 1.1.23 GPS Diagnostic Screen
//#define RB_ENABLE_GPS_DIAG 1
//#define RB_ENABLE_CONSOLE_DEBUG 1



// 1.2.1 Enble Bluetooth Services
//#define RB02_ESP_BLUETOOTH 1    // ONLY FOR PERSONAL USE this feature is under Partner contract, if you want to sell this you shall have the dual licensing

// 1.2.1 Enble Traffic Services
#ifdef RB02_ESP_BLUETOOTH
#define RB05_Flarm_SERIAL 10                     // ONLY FOR PERSONAL USE this feature is under Partner contract, if you want to sell this you shall have the dual licensing
#define RB_ENABLE_TRAFFIC RB05_Flarm_SERIAL      // ONLY FOR PERSONAL USE this feature is under Partner contract, if you want to sell this you shall have the dual licensing
#define RB02_BLUETOOTH_DIAG 1                    // ONLY FOR PERSONAL USE this feature is under Partner contract, if you want to sell this you shall have the dual licensing
#endif



// 1.1.2 Version is here
// #define RB_VERSION "1.1.27"
// 1.1.1 Remove tabs with GPS if not installed
#define RB01_GPS_PROTOCOL_UART  1
#ifdef RB02_ESP_BLUETOOTH
#define RB01_GPS_PROTOCOL_BLE   2
#define RB01_GPS_PROTOCOL RB01_GPS_PROTOCOL_BLE
#endif

#ifdef RB_ENABLE_UART
#define UART_RX_BUF_SIZE 1024
#ifdef LC76GABMD
#define TXD_PIN (GPIO_NUM_43)
#define RXD_PIN (GPIO_NUM_44)
#else
#define TXD_PIN (GPIO_NUM_43)
#define RXD_PIN (GPIO_NUM_44)
#endif
#define UART_N UART_NUM_1
#endif

// 1.1.19 Starting getting rid of demo screens
// #define ENABLE_DEMO_SCREENS 1
//#define VIBRATION_TEST 1


// 1.3.1 Enable Engine Monitoring System EMS
//#define RB_ENABLE_EMS 1      // ONLY FOR PERSONAL USE this feature is under Partner contract, if you want to sell this you shall have the dual licensing
#ifdef RB_ENABLE_EMS
#define RB_ENABLE_EMS_CHT 4      // Number of CHT Sensors
#define RB_ENABLE_EMS_EGT 4      // Number of EGT Sensors
#define RB04_PROTOCOL_SIMULATOR   0      // EMS Simulator
#define RB04_PROTOCOL_UART_TTL    1      // EMS Remote device connected via UART channel
#define RB04_PROTOCOL_BLE         2      // EMS Remote device connected via BLUETOOTH BLE CHAR channel
#define RB04_PROTOCOL_RS485       3      // EMS Remote device connected via RS485 channel
#define RB04_PROTOCOL RB04_PROTOCOL_SIMULATOR
#endif


//#define RB_ENABLE_REMOTE_BUTTONS 1
#ifdef RB_ENABLE_REMOTE_BUTTONS
#define RB02_BUTTON_BUTTON      1
#define RB02_BUTTON_KNOB        2
#ifdef RB02_ESP_BLUETOOTH
#define RB02_BUTTON_BLE         3
#endif
#endif

#if RB_PRODUCT_LINE == 2
#define RB_ENABLE_BMP280 1
#define RB_ENABLE_QMI8658 1
#endif

#ifdef RB_ENABLE_BME680
#include "BME680.h"
#ifdef RB_ENABLE_CODETECTOR
#endif
#undef RB_ENABLE_BMP280
#endif


#ifdef RB_ENABLE_NavCore
#define RB_ENABLE_IAS 1
#define RB_ENABLE_KNOB 1
#define RB_ENABLE_MODBUS 1
#define RB_ENABLE_CAN 1
//#define RB_ENABLE_BMI270 1
//#undef RB_ENABLE_QMI8658
#endif