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
void DidYouJoinedDiscord()
{

  if (youAreThere == true)
  {
    MeansYouShallUseThePreBuildReleasedAndJoinDiscord();
  }
}

#include "RB02.h"
#include "RB02_GUIHelpers.h"

// Images pre-loaded
#ifdef ENABLE_DEMO_SCREENS
#include "RoundSynthViewAttitude.c"
#endif

// 1.1.19 Refactoring, images
#include "RB02Images.c"

#include "RB02_Altimeter.h"
#include "RB02_Setup.h"

// 1.1.19 GPS MAP
#include "RB02_NMEA.h"
#ifdef RB_ENABLE_MAP
#include "RB02_GPSMap.h"
#endif

// 1.1.27
#ifdef RB_ENABLE_GPS
#include "RB02_Gyro.h"
#endif

#ifdef RB_ENABLE_CONSOLE
#include "RB02_Console.h"
#endif

#ifdef RB_ENABLE_GPS_DIAG
#include "RB02_GPSDiag.h"
#endif

#ifdef RB_ENABLE_EMS
#include "RB04_EMS.h"
#endif


#ifdef RB_ENABLE_CHECKLIST
#include "RB02_Checklist.h"
#endif
#ifdef RB_ENABLE_AAT
#include "RB02_AAttitude.h"
#endif

#ifdef RB02_ESP_BLUETOOTH
#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#endif

#ifdef RB_ENABLE_TRAFFIC
#include "RB05_Traffic.h"
#include "RB05_Radar.h"
#endif

// 1.1.5 Added Vendor Splashscreen
#include "Vendor.h"

// 1.1.6
#include "InopWarning.c"

// Load the right splash screen
#ifndef VendorSplashScreen_map
#if ENABLE_VENDOR == RB_VENDOR_1
#include "V0_VendorSplashScreen.c"
#endif
#if ENABLE_VENDOR == RB_VENDOR_2
#include "V1_VendorSplashScreen.c"
#endif
#if ENABLE_VENDOR == RB_VENDOR_3
#include "V2_VendorSplashScreen.c"
#endif
#endif

#include "RB02_Navigator.h"

// External dependencies
#include "nvs_flash.h"
#include "esp_mac.h"
#include "esp_sleep.h"
#include "QMI8658.h"
#include "driver/uart.h"

#ifdef RB_ENABLE_NavCore
#include "RB02_NavCore.h"
#endif

#include "PCF85063.h"
#include "RB02_SDCardInject.c"

#define Backlight_MAX 100
void Set_Backlight(uint8_t Light);
int64_t esp_timer_get_time(void);
void Backlight_adjustment_event_cb(lv_event_t *e);

void LVGL_Backlight_adjustment(uint8_t Backlight);
void draw_arch(lv_obj_t *parent, const lv_img_dsc_t *t, uint16_t degreeStartSlide, uint16_t degreeEndSlide);

extern uint8_t DriverLoopMilliseconds; // 1.1.9 Anti precession
extern float BAT_analogVolts;          // 1.1.4 Power management
extern uint8_t LCD_Backlight;
extern IMUdata AccelFilteredMax;
extern IMUdata GyroBias;
// 1.1.8 Quaternion alignment
extern IMUdata AccelBias;
extern lv_coord_t TouchPadLastX;
extern lv_coord_t TouchPadLastY;
extern float AttitudeBalanceAlpha;
extern float FilterMoltiplier;
extern float FilterMoltiplierOutput;
extern float FilterMoltiplierGyro;
extern float GFactor;
extern float GFactorMax;
extern float GFactorMin;
extern uint8_t GFactorDirty;
extern int32_t bmp280Pressure;
extern int32_t Variometer;
double fabs(double x);
/* 1.0.9 For future compatibility imported NMEA ESP32 Example header */
#define CONFIG_NMEA_PARSER_RING_BUFFER_SIZE 2048
#define CONFIG_NMEA_PARSER_TASK_STACK_SIZE 4 * 1024
#define CONFIG_NMEA_PARSER_TASK_PRIORITY 11
#define CONFIG_NMEA_PARSER_UART_RXD 44
#define CONFIG_NMEA_STATEMENT_RMC 1

// 1.1.12 Fusion
float FusionAHRSDeltaTrackFromGPS = 0;
float GPSLastTrack = 0;
extern float GPSLateralYAcceleration;
extern bool GPSIsReliable;
int64_t GPSDT = 0;
extern float q0, q1, q2, q3;

int32_t GpsSpeed0ForDisable = 0;
float GPSLastSpeedKmhForAttitudeComponesation = 0;
int64_t GPSLastSpeedKmhReceivedTick = -20000000;
float GPSCurrentSpeedKmhForAttitudeComponesation = 0;
extern float GPSAccelerationForAttitudeCompensation;
bool GPSAccelerationForAttitudeCompensationEnabled = true;
uint8_t EnableAttitudeMadgwick = 1;

// Attitude Panel Pitch Alignment Temporary workaround for demo #85
extern IMUdata PanelAlignment;

// DEFINES
#define RB02_TOUCH_SECTION 3
#define RB02_TOUCH_SECTION_SIZE 160
#define DIGIT_BIG_SEGMENTS 7
#define DIGIT_BIG_DIGIT 4
#define DIGIT_MINOR_DIGIT 2
#define BMP280_S64_t int64_t
#define BMP280_U32_t uint32_t
#define BMP280_S32_t int32_t

// Prototype declaration
void lvgl_register_sdcard_fs();
static lv_obj_t *Onboard_create_Base(lv_obj_t *parent, const lv_img_dsc_t *backgroundImage);
#ifdef RB_ENABLE_SPD
static void Onboard_create_Speed(lv_obj_t *parent);
#endif
#ifdef RB_ENABLE_ATT
static void Onboard_create_Attitude(lv_obj_t *parent);
#endif
static void Onboard_create_AltimeterDigital(lv_obj_t *parent);
static void Onboard_create_TurnSlip(lv_obj_t *parent);
static void Onboard_create_Clock(lv_obj_t *parent);

static void Onboard_create_Variometer(lv_obj_t *parent);
static void Onboard_create_GMeter(lv_obj_t *parent);
static void Onboard_create_Setup(lv_obj_t *parent);
static void Onboard_create_VibrationTest(lv_obj_t *parent);
static void speedBgClicked(lv_event_t *event);
uint32_t timerDiffByIndex(int st);
void rb_increase_lvgl_tick(lv_timer_t *t);
void update_GMeter_lvgl_tick(lv_timer_t *t);
void rotate_AttitudeGearByDegree(int lastAttitudeRoll);
void update_Attitude_lvgl_tick(lv_timer_t *t);
#ifdef VIBRATION_TEST
void update_Vibration_lvgl_tick(lv_timer_t *t);
#endif
void update_Clock_lvgl_tick(lv_timer_t *t);
void update_AltimeterDigital_lvgl_tick(lv_timer_t *t);
void update_Altimeter_lvgl_tick(lv_timer_t *t);
#ifdef RB_ENABLE_UART
void uart_fetch_data();
#endif
void nvsStorePCal();
void nvsStoreUARTBaudrate();
static void RB02_AltimeterQNHUpdated();
static void CreateSingleDigit(lv_obj_t *parent, const lv_img_dsc_t *font, lv_obj_t **segments, int dx, int dy);
void nvsStoreGMeter();
void nvsRestoreGMeter();
void nvsRestoreSettings();
void disableTVScroll();
void example1_BMP280_lvgl_tick(lv_timer_t *t);
// Variables

typedef enum
{
  RB02_TOUCH_NW,
  RB02_TOUCH_N,
  RB02_TOUCH_NE,
  RB02_TOUCH_W,
  RB02_TOUCH_CENTER,
  RB02_TOUCH_E,
  RB02_TOUCH_SW,
  RB02_TOUCH_S,
  RB02_TOUCH_SE,
  RB02_TOUCH_UNKNOWN
} touchLocation;

typedef enum
{
#ifdef ENABLE_VENDOR
  RB02_TAB_SPL,
#endif
#ifdef RB_ENABLE_EMS
  RB02_TAB_EMS,
#endif
#ifdef ENABLE_DEMO_SCREENS
  RB02_TAB_SYS,
  RB02_TAB_SYN,
// RB02_TAB_HSI,
#endif
#ifdef RB_ENABLE_SPD
  RB02_TAB_SPD,
#endif
#ifdef RB_ENABLE_ATT
  RB02_TAB_ATT,
#endif
#ifdef RB_ENABLE_AAT
  RB02_TAB_AAT,
#endif
#ifdef RB_ENABLE_ALT
  RB02_TAB_ALT,
#endif
#ifdef RB_ENABLE_ALD
  RB02_TAB_ALD,
#endif
#ifdef RB_ENABLE_TRN
  RB02_TAB_TRN,
#endif
#ifdef RB_ENABLE_GPS
#ifdef RB_ENABLE_TRK
  RB02_TAB_TRK,
#endif
#ifdef RB_ENABLE_MAP
  RB02_TAB_MAP,
#endif
#endif
#ifdef RB_ENABLE_VAR
  RB02_TAB_VAR,
#endif
#ifdef RB_ENABLE_GMT
  RB02_TAB_GMT,
#endif
#ifdef RB_ENABLE_CLK
  RB02_TAB_CLK,
#endif
#ifdef RB_ENABLE_CHECKLIST
  RB02_TAB_CHK,
#endif
#ifdef RB_ENABLE_TRAFFIC
  RB02_TAB_RDR,
  RB02_TAB_TRA,
#endif
  RB02_TAB_SET,
#ifdef VIBRATION_TEST
  RB02_TAB_VBR,
#endif
#ifdef RB_ENABLE_GPS_DIAG
  RB02_TAB_GDG,
#endif
#ifdef RB_ENABLE_CONSOLE
  RB02_TAB_COS,
#endif
  RB02_TAB_DEV
} tabs;

// 1.1.6 Red X
uint8_t Operative_GPS = 0;
uint8_t Operative_BMP280 = 0;
uint8_t Operative_Attitude = 0;

lv_obj_t *uartDropDown = NULL;
lv_obj_t *kmhDropDown = NULL;
lv_obj_t *SettingStatus0 = NULL;
lv_obj_t *SettingStatus1 = NULL;
// TODO: move all objects into the UI structure
lv_obj_t *SettingStatus3 = NULL;
lv_obj_t *SettingStatus4 = NULL;
lv_obj_t *SettingStatus5 = NULL;
lv_obj_t *SettingStatus4UART = NULL;

// char SettingStatus4UARTBuf[20];
lv_style_t style_title;
lv_obj_t *SettingLabelFilter = NULL;
lv_obj_t *SettingsEngineTimeLabel = NULL;
lv_obj_t *SettingLabelFilterOutput = NULL;
lv_obj_t *SettingLabelFilterGyro = NULL;
lv_obj_t *SettingLabelDriverLoopMilliseconds = NULL;

lv_obj_t *SettingAttitudeCompensation = NULL;
// 1.1.5 Added Vendor Splashscreen
#ifdef ENABLE_VENDOR
lv_obj_t *VendorSplashScreenImage = NULL;
lv_obj_t *lvTabSplashScreen = NULL;
#endif
lv_obj_t *Ball = NULL;
lv_obj_t *Screen_Attitude_Pitch = NULL;
lv_obj_t *Screen_Attitude_RollIndicator = NULL;
void *Screen_Attitude_Rounds[4];
uint16_t QNH = 1013;
// lv_obj_t *Screen_Gyro_Gear = NULL;
extern lv_obj_t *Screen_Altitude_Miles;
extern lv_obj_t *Screen_Altitude_Cents;
lv_obj_t *Screen_Variometer_Cents = NULL;
extern lv_obj_t *Screen_Altitude_QNH;
lv_obj_t *Screen_Altitude_QNH2 = NULL;
lv_obj_t *Screen_Altitude_Variometer2 = NULL;
lv_obj_t *Screen_Altitude_Pressure = NULL;
int lastAttitudePitch = 0;
int lastAttitudeRoll = 0;
datetime_t stopwatch = {0};
uint8_t DeviceIsDemoMode = 0;
tabs StartupPage = 0;
bool OperativeWarningVisible = false;
lv_obj_t *OperativeWarning = NULL;
lv_obj_t *tv = NULL;
const lv_font_t *font_large;
const lv_font_t *font_normal;
lv_obj_t *TimerLabelTop = NULL;
lv_obj_t *TimerLabelSW = NULL;
lv_obj_t *TimerLabelSE = NULL;
lv_obj_t *TimerSW = NULL;
lv_obj_t *TimerSE = NULL;
uint8_t selectedTimer = 0;
datetime_t datetimeTimer1 = {0};
datetime_t datetimeTimer2 = {0};
datetime_t datetimeTimer3 = {0};
const lv_res_t Screen_TurnSlip_Obj_Ball_Size = SCREEN_HEIGHT / 12;
lv_obj_t *Screen_TurnSlip_Obj_Ball = NULL;
lv_obj_t *Screen_TurnSlip_Obj_Label = NULL;
lv_obj_t *Screen_TurnSlip_Obj_Turn = NULL;
lv_obj_t *Screen_Speed_SpeedTick = NULL;

lv_obj_t *Screen_GMeter_Ball = NULL;
lv_obj_t *Screen_GMeter_BallMax = NULL;
#ifdef VIBRATION_TEST
extern acc_scale_t acc_scale;
extern gyro_scale_t gyro_scale;
extern acc_odr_t acc_odr;
extern gyro_odr_t gyro_odr;
extern lpf_t acc_lpf;
extern lpf_t gyro_lpf;
lv_obj_t *Screen_GMeter_BallGyro = NULL;
lv_obj_t *GMeterLabelGyro = NULL;
lv_obj_t *Screen_Vibration_Accel_Ball = NULL;
lv_obj_t *Screen_Vibration_GPSAccel_Ball = NULL;
lv_obj_t *Screen_Vibration_GPSAccel_Label = NULL;

lv_obj_t *Screen_Vibration_Yaw_Ball = NULL;

lv_obj_t *Screen_Vibration_Accel_Label = NULL;
lv_obj_t *Screen_Vibration_IMU_Label = NULL;
lv_obj_t *QMISetRangeGyroCombo = NULL;
lv_obj_t *QMISetRangeAccCombo = NULL;
lv_obj_t *QMISetODRGyroCombo = NULL;
lv_obj_t *QMISetODRACcCombo = NULL;
lv_obj_t *QMISetLPFGyroCombo = NULL;
lv_obj_t *QMISetLPFAccCombo = NULL;
#endif
lv_obj_t *GMeterLabel = NULL;
lv_obj_t *GMeterLabelMax = NULL;

float GMeterScale = 3.0;
float GMeterScaleGyro = 15.0;
IMUdata GyroBiasAcquire[3];
lv_obj_t *mbox1 = NULL;
uint8_t isKmh = 0;
uint16_t degreeStart = 0;
uint16_t degreeEnd = 320;
uint16_t speedKtStart = 0;
uint16_t speedKtEnd = 200;
uint16_t speedWhite = 60;
uint16_t speedGreen = 105;
uint16_t speedYellow = 225;
uint16_t speedRed = 315;
lv_obj_t *t_speedSummary = NULL;
lv_obj_t *t_speedStart = NULL;
lv_obj_t *t_speedEnd = NULL;
lv_obj_t *t_speedStartSpeed = NULL;
lv_obj_t *t_speedEndSpeed = NULL;
lv_obj_t *t_speedWhite = NULL;
lv_obj_t *t_speedGreen = NULL;
lv_obj_t *t_speedYellow = NULL;
lv_obj_t *t_speedRed = NULL;
uint64_t _chipmacid = 0LL;
uint8_t workflow = 0;

int32_t bmp280Calibration[12];
BMP280_S32_t t_fine = 0;
lv_obj_t *bmp280overrideLabel = NULL;
lv_obj_t *SegmentsA[DIGIT_BIG_DIGIT][DIGIT_BIG_SEGMENTS];
lv_obj_t *SegmentsAltDigit[5][DIGIT_BIG_SEGMENTS];
// lv_obj_t *SegmentsB[DIGIT_MINOR_DIGIT][DIGIT_BIG_SEGMENTS];

/**********************
 *  STATIC PROTOTYPES
 **********************/
void callbackTouch()
{
}

touchLocation getTouchLocation(lv_coord_t x, lv_coord_t y)
{
  int8_t sx = x / RB02_TOUCH_SECTION_SIZE;
  int8_t sy = y / RB02_TOUCH_SECTION_SIZE;
  touchLocation returnCode = sy * RB02_TOUCH_SECTION + sx;
  return returnCode;
}

void ApplyCoding(void)
{
  // 1.1.23 Removing Default Demo Version
#ifdef RB_02_DISPLAY_TOUCH
  DeviceIsDemoMode = 0;
  StartupPage = 1;
#else
  DeviceIsDemoMode = 0;
  StartupPage = RB02_TAB_SET - 1;
#endif
}

#ifdef RB_ENABLE_MAP
lv_obj_t *t0 = NULL;
#endif
void RB02_Example1(void)
{

  font_large = LV_FONT_DEFAULT;
  font_normal = LV_FONT_DEFAULT;

#if LV_FONT_MONTSERRAT_20
  font_large = &lv_font_montserrat_20;
#else
  LV_LOG_WARN("LV_FONT_MONTSERRAT_20 is not enabled for the widgets demo. Using LV_FONT_DEFAULT instead.");
#endif
#if LV_FONT_MONTSERRAT_14
  font_normal = &lv_font_montserrat_14;
#else
  LV_LOG_WARN("LV_FONT_MONTSERRAT_14 is not enabled for the widgets demo. Using LV_FONT_DEFAULT instead.");
#endif

  ApplyCoding();
  nvsRestoreSettings();
  // LCD_Backlight = 0;
  // Set_Backlight(LCD_Backlight);

  lv_style_init(&style_title);
  // lv_style_set_text_font(&style_title, font_large);
  lv_style_set_text_font(&style_title, &lv_font_montserrat_16);
  lv_style_set_text_color(&style_title, lv_color_white());
  lv_obj_set_style_text_font(lv_scr_act(), font_normal, 0);

  // 1.1.5 Added Vendor Splashscreen
#ifdef ENABLE_VENDOR
  lv_obj_t *ts = lv_tabview_add_tab(tv, RB_PRODUCT_TITLE);
  lv_obj_add_event_cb(ts, speedBgClicked, LV_EVENT_CLICKED, NULL);
  lvTabSplashScreen = ts;
#endif
#ifdef RB_ENABLE_EMS
  singletonConfig()->ui.tEMS = lv_tabview_add_tab(tv, "EMS");
#endif
#ifdef ENABLE_DEMO_SCREENS
  lv_obj_t *tu = lv_tabview_add_tab(tv, "SynthSide");
  lv_obj_t *tz = lv_tabview_add_tab(tv, "SynthBack");
  lv_obj_t *tw = lv_tabview_add_tab(tv, "Radar");
  // lv_obj_t *ty = lv_tabview_add_tab(tv, "HSI");
  // lv_obj_t *t0 = lv_tabview_add_tab(tv, "Map");
#endif
#ifdef RB_ENABLE_SPD
  lv_obj_t *t1 = lv_tabview_add_tab(tv, "Speed");
#endif

#ifdef RB_ENABLE_ATT
  lv_obj_t *t2 = lv_tabview_add_tab(tv, "Attitude");
#endif
#ifdef RB_ENABLE_AAT
  advancedAttitude_Status.lv_parent = lv_tabview_add_tab(tv, "Advanced");
#endif
#ifdef RB_ENABLE_ALT
  lv_obj_t *t3 = lv_tabview_add_tab(tv, "Altimeter");
#endif
#ifdef RB_ENABLE_ALD
  lv_obj_t *t3b = lv_tabview_add_tab(tv, "Altimeter");
#endif
#ifdef RB_ENABLE_TRN
  lv_obj_t *t4 = lv_tabview_add_tab(tv, "TurnSlip");
#endif
#ifdef RB_ENABLE_TRK
  // Track backup as Gyroscope Directional
  singletonConfig()->ui.Gyro.parent = lv_tabview_add_tab(tv, "Track");
#endif
#ifdef RB_ENABLE_MAP
  t0 = lv_tabview_add_tab(tv, "Map"); // 1.1.19 Last version with demo screens
#endif
#ifdef RB_ENABLE_VAR
  lv_obj_t *t6 = lv_tabview_add_tab(tv, "Variometer");
#endif
#ifdef RB_ENABLE_GMT
  lv_obj_t *t7 = lv_tabview_add_tab(tv, "GMeter");
#endif
#ifdef RB_ENABLE_CLK
  // TODO: Rename to TMR
  lv_obj_t *t8 = lv_tabview_add_tab(tv, "Clock");
  // Add the big analog clock with local time
#endif
  // 1.1.19
#ifdef RB_ENABLE_CHECKLIST
  lv_obj_t *tChecklist = lv_tabview_add_tab(tv, "Checklist");
#endif

#ifdef RB_ENABLE_TRAFFIC
  singletonConfig()->trafficStatus.lv_parent_radar = lv_tabview_add_tab(tv, "Radar");
  singletonConfig()->trafficStatus.lv_parent = lv_tabview_add_tab(tv, "Traffic");
#endif
  lv_obj_t *t9 = lv_tabview_add_tab(tv, "Setup");
#ifdef VIBRATION_TEST
  lv_obj_t *t10 = lv_tabview_add_tab(tv, "Vibration");
#else
#endif

#ifdef RB_ENABLE_GPS_DIAG
  singletonConfig()->ui.tGPSDiag = lv_tabview_add_tab(tv, "GPS Diag");
  lv_obj_add_event_cb(singletonConfig()->ui.tGPSDiag, speedBgClicked, LV_EVENT_CLICKED, NULL);
#endif

#ifdef RB_ENABLE_CONSOLE
  singletonConfig()->ui.tConsole = lv_tabview_add_tab(tv, "Console");
  lv_obj_add_event_cb(singletonConfig()->ui.tConsole, speedBgClicked, LV_EVENT_CLICKED, NULL);
#endif

#ifdef ENABLE_DEMO_SCREENS
  Onboard_create_Base(tu, &RoundSynthViewSide);
  lv_obj_add_event_cb(tu, speedBgClicked, LV_EVENT_CLICKED, NULL);
  Onboard_create_Base(tz, &RoundSynthViewAttitude);
  lv_obj_add_event_cb(tz, speedBgClicked, LV_EVENT_CLICKED, NULL);
  Onboard_create_Base(tw, &Radar);
  lv_obj_add_event_cb(tw, speedBgClicked, LV_EVENT_CLICKED, NULL);
  // Onboard_create_Base(ty, &RoundHSI);
  // Onboard_create_Base(t0, &RoundMapWithControlledSpaces);
  // lv_obj_add_event_cb(t0, speedBgClicked, LV_EVENT_CLICKED, NULL);
#endif

  // 1.1.19 Map
  lvgl_register_sdcard_fs();

#ifdef RB_ENABLE_SPD
  Onboard_create_Speed(t1);
#endif

#ifdef RB_ENABLE_ATT
  Onboard_create_Attitude(t2);
#endif
#ifdef RB_ENABLE_ALT
  RB02_Altimeter_CreateScreen(t3);
  lv_obj_add_event_cb(t3, speedBgClicked, LV_EVENT_CLICKED, NULL);
#endif
#ifdef RB_ENABLE_ALD
  Onboard_create_AltimeterDigital(t3b);
#endif
#ifdef RB_ENABLE_TRN
  Onboard_create_TurnSlip(t4);
#endif
#ifdef RB_ENABLE_GPS
#ifdef RB_ENABLE_MAP
  RB02_GPSMap_CreateScreen(&singletonConfig()->gpsMapStatus, t0);
  lv_obj_add_event_cb(t0, speedBgClicked, LV_EVENT_CLICKED, NULL);
  lv_obj_clear_flag(t0, LV_OBJ_FLAG_GESTURE_BUBBLE);
#endif
#endif
#ifdef RB_ENABLE_VAR
  Onboard_create_Variometer(t6);
#endif
#ifdef RB_ENABLE_GMT
  Onboard_create_GMeter(t7);
#endif
#ifdef RB_ENABLE_CLK
  Onboard_create_Clock(t8);
#endif

  // 1.1.3 Display unique serial number for device tracking
  esp_efuse_mac_get_default((uint8_t *)(&_chipmacid));
#ifdef RB_ENABLE_CONSOLE_DEBUG
  int8_t hasGPS = 0;
#ifdef RB_ENABLE_GPS
  hasGPS = 1;
#endif
  printf("$RB,02,%d,%s,%d,%llX\n", RB_02_DISPLAY_SIZE, RB_VERSION, hasGPS, _chipmacid);
#endif
#ifdef RB_ENABLE_CHECKLIST
  RB02_Checklist_CreateScreen(tChecklist, "/sdcard/check.txt");
  lv_obj_add_event_cb(tChecklist, speedBgClicked, LV_EVENT_CLICKED, NULL);
#endif
  Onboard_create_Setup(t9);
#ifdef VIBRATION_TEST
  Onboard_create_VibrationTest(t10);
#else
#endif

  // BMP280
  uint8_t bmp280BufferReset[1] = {0xB6};
  I2C_Write(0x76, 0xE0, &bmp280BufferReset[0], 1);

  // 1.1.5 Added Vendor Splashscreen
#ifndef ENABLE_VENDOR
  if (StartupPage != 0)
  {
    lv_tabview_set_act(tv, StartupPage, LV_ANIM_OFF);
  }
#endif

  // 1.1.3 Disable manual scroll
  lv_obj_clear_flag(tv, LV_OBJ_FLAG_SCROLLABLE);
  // lv_tabview_set_sliding(tv, false);
  /*
  lv_tabview_t *tabview = (lv_tabview_t *)tv;
  for (uint16_t cur = 0; cur < tabview->tab_cnt - 2; cur++)
  {

    lv_obj_clear_flag(t1, LV_OBJ_FLAG_SCROLLABLE);
  }
  */
  // 1.1.4 Add Deep Sleep and Wakeup
  // Notes on WS 2.8 Touch Panel
  // touchAttachInterrupt(16, callbackTouch, 50); Not found
  // esp_sleep_enable_touchpad_wakeup(); Not working
  // gpio_wakeup_enable(GPIO_NUM_16, GPIO_INTR_HIGH_LEVEL); Keep the device on
  // esp_sleep_enable_gpio_wakeup(); Not in the Default SDK
  // esp_deep_sleep_enable_gpio_wakeup(16,ESP_GPIO_WAKEUP_GPIO_HIGH); Not in the Default SDK
  // esp_deep_sleep_try_to_start(); // Works on WS2.8 but I2C and External ports are still powered by pattery line

  disableTVScroll();

#ifdef RB_ENABLE_CONSOLE
  RB02_Console_AppendLog(RB02_LOG_MAIN, RB02_LOG_INFO, "Workflow start");
#endif
}

void disableTVScroll()
{
  lv_obj_clear_flag(tv, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_clear_flag(tv, LV_OBJ_FLAG_SCROLL_CHAIN_HOR);
  lv_obj_clear_flag(tv, LV_OBJ_FLAG_SCROLL_CHAIN_VER);
  lv_obj_clear_flag(lv_tabview_get_content(tv), LV_OBJ_FLAG_SCROLLABLE);
}

lv_obj_t *RB_LV_Helper_CreateImageFromFile(lv_obj_t *parent, const void *backgroundImageName)
{

  lv_obj_t *backgroundImage = lv_img_create(parent);
  lv_img_set_src(backgroundImage, backgroundImageName);
  lv_obj_set_size(backgroundImage, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
  lv_obj_align(backgroundImage, LV_ALIGN_CENTER, 0, 0);
  lv_obj_set_scrollbar_mode(backgroundImage, LV_SCROLLBAR_MODE_OFF);
  lv_obj_set_scrollbar_mode(parent, LV_SCROLLBAR_MODE_OFF);
  // 1.1.9 Remove scrolling for Turbolence touch screen
  lv_obj_clear_flag(parent, LV_OBJ_FLAG_SCROLLABLE);
  return backgroundImage;
}
#ifdef RB_ENABLE_GPS
void nmea_GGA_UpdatedValueFor(uint8_t csvCounter, int32_t finalNumber, uint8_t decimalCounter)
{

  float conversionValue = finalNumber;
  for (uint8_t c = 0; c < decimalCounter; c++)
  {
    conversionValue = conversionValue / 10.0;
  }
  switch (csvCounter)
  {
  case 0: // GGA
    /* code */
    break;
  case 1: // UTC Time
    /* code */
    break;
  case 2: // Lat
    /* code */
    break;
  case 3: // N/S
    /* code */
    break;
  case 4: // Long
    /* code */
    break;
  case 5: // E/W
    /* code */
    break;
  case 6: // GPS Status
    /* code */
    // Operative_GPS = sentence[x] - '0';
    Operative_GPS = finalNumber;
    singletonConfig()->NMEA_DATA.valid = Operative_GPS > 0 ? true : false;
    singletonConfig()->NMEA_DATA.fix = Operative_GPS;
    switch (Operative_GPS)
    {
    case 0:
      singletonConfig()->NMEA_DATA.fix_mode = GPS_MODE_INVALID;
      break;
    case 1:
      // TODO: FIX MODE
      singletonConfig()->NMEA_DATA.fix_mode = GPS_MODE_3D;
      break;
    }

    // printf("GGA: %d\n",sentence[x]-'0');
    break;
  case 7: // # sats
          /* code */
    singletonConfig()->NMEA_DATA.sats_in_view = finalNumber;
#ifdef RB_ENABLE_CONSOLE_DEBUG
    printf("Number of SAT: %d\n", singletonConfig()->NMEA_DATA.sats_in_view);
#endif
    break;
  case 8: // hdop
    /* code */
    singletonConfig()->NMEA_DATA.dop_h = conversionValue;
    break;
  case 9: // ALT MT
    if (finalNumber != 0)
    {
      if (singletonConfig()->NMEA_DATA.altitude < 0.0001)
      {
        singletonConfig()->NMEA_DATA.altitude = conversionValue;
      }
      singletonConfig()->NMEA_DATA.altitude = (conversionValue + singletonConfig()->NMEA_DATA.altitude * 5.0) / 6.0;
      if (singletonConfig()->settingsAutoQNH > 0)
      {
        if (Operative_BMP280 > 0 && Operative_GPS > 0)
        {
          if (Variometer < 100 && Variometer > -100)
          {
            uint16_t SuggestedQNH = RB02_SuggestedQNH(singletonConfig()->NMEA_DATA.altitude, bmp280Pressure);
#ifdef RB_ENABLE_CONSOLE_DEBUG
            printf("Suggested QNH %d vs %d\n", SuggestedQNH, QNH);
#endif
            if (SuggestedQNH - QNH < 10 && SuggestedQNH - QNH > -10)
            {
              if (abs(SuggestedQNH - QNH) > 0)
              {
                QNH = RB02_SuggestedQNH(singletonConfig()->NMEA_DATA.altitude, bmp280Pressure);
                RB02_AltimeterQNHUpdated();
              }
            }
          }
        }
      }
    }
    break;
  case 10: // a-units
    /* code */
    break;
  default:
    break;
  }
#ifdef RB_ENABLE_CONSOLE_DEBUG
  printf("GGA: %d %ld %f\n", csvCounter, finalNumber, conversionValue);
#endif
}

#define DEG2RAD (3.14159265359f / 180.0f)
#define G 9.81f
void nmea_RMC_UpdatedValueFor(uint8_t csvCounter, int32_t finalNumber, uint8_t decimalCounter)
{

  float conversionValue = finalNumber;
  for (uint8_t c = 0; c < decimalCounter; c++)
  {
    conversionValue = conversionValue / 10.0;
  }
  switch (csvCounter)
  {
  case 0: // $GPRMC
    /* code */
    break;
  case 1: // UTC hhmmss.ss
    if (finalNumber != 0)
    {
      singletonConfig()->NMEA_DATA.tim.thousand = finalNumber % 100;
      singletonConfig()->NMEA_DATA.tim.second = (finalNumber / 100) % 100;
      singletonConfig()->NMEA_DATA.tim.minute = (finalNumber / 10000) % 100;
      singletonConfig()->NMEA_DATA.tim.hour = (finalNumber / 1000000);
    }
    /* code */
    break;
  case 2: // A
    /* code */
    break;
  case 3: // 4311.11936
    /* code */
    if (finalNumber != 0)
    {

      singletonConfig()->NMEA_DATA.latitude = nmea_to_decimal(conversionValue);
    }
    break;
  case 4: // N
    /* code */
    {
      char c = finalNumber + '0';
      if (c == 'S')
      {
        singletonConfig()->NMEA_DATA.latitude = -fabs(singletonConfig()->NMEA_DATA.latitude);
      }
    }
    break;
  case 5: // 01208.18660
    /* code */
    if (finalNumber != 0)
    {
      singletonConfig()->NMEA_DATA.longitude = nmea_to_decimal(conversionValue);
    }
    break;
  case 6: // E
    /* code */
    {
      char c = finalNumber + '0';
      if (c == 'W')
      {
        singletonConfig()->NMEA_DATA.longitude = -fabs(singletonConfig()->NMEA_DATA.longitude);
      }
    }
    break;
  case 7: // Speed KT
    singletonConfig()->NMEA_DATA.speed = conversionValue * 1.852;
    GPSCurrentSpeedKmhForAttitudeComponesation = singletonConfig()->NMEA_DATA.speed;
    /* code */
    break;
  case 8: // Track
          /* code */
    GPSLateralYAcceleration = 0;
    GPSAccelerationForAttitudeCompensation = 0;
    singletonConfig()->NMEA_DATA.cog = conversionValue;
    if (singletonConfig()->NMEA_DATA.cog > 0 && GPSCurrentSpeedKmhForAttitudeComponesation > 3)
    {

      // 1.1.12 Fusion
      FusionAHRSDeltaTrackFromGPS = singletonConfig()->NMEA_DATA.cog - GPSLastTrack;

      if (FusionAHRSDeltaTrackFromGPS > 180.0f)
        FusionAHRSDeltaTrackFromGPS -= 360.0f;
      if (FusionAHRSDeltaTrackFromGPS < -180.0f)
        FusionAHRSDeltaTrackFromGPS += 360.0f;
      if (GPSAccelerationForAttitudeCompensationEnabled == true)
      {

        // Front G during takeoff and brakes
        float GPSLastSpeedKmhForAttitudeComponesationElapsedTime = GPSDT / (1000000.0);
        // float GPSLastSpeedKmhForAttitudeComponesationElapsedTime = 0.1;
        float DV = ((GPSCurrentSpeedKmhForAttitudeComponesation) - (GPSLastSpeedKmhForAttitudeComponesation));
        GPSAccelerationForAttitudeCompensation = DV / GPSLastSpeedKmhForAttitudeComponesationElapsedTime / 3.6 / G;

        // Lateral G Force during long term turn
        float yaw_rate_gps_rad = (FusionAHRSDeltaTrackFromGPS / GPSLastSpeedKmhForAttitudeComponesationElapsedTime) * DEG2RAD;
        float a_lat = GPSCurrentSpeedKmhForAttitudeComponesation * yaw_rate_gps_rad / 3.6;
        GPSLateralYAcceleration = a_lat / G;

#ifdef RB_ENABLE_CONSOLE_DEBUG
        printf("GPS Speed: %.1f Last Speed %.1f Track: %.1f Last Track %.1f DV: %.1f DTrack: %.1f DT: %.5f AY: %.1f AX: %.1f\n",
               GPSCurrentSpeedKmhForAttitudeComponesation,
               GPSLastSpeedKmhForAttitudeComponesation,
               singletonConfig()->NMEA_DATA.cog,
               GPSLastTrack,
               DV,
               FusionAHRSDeltaTrackFromGPS,
               GPSLastSpeedKmhForAttitudeComponesationElapsedTime,
               GPSAccelerationForAttitudeCompensation,
               GPSLateralYAcceleration);
#endif
        if (GPSAccelerationForAttitudeCompensation > 1.5)
          GPSAccelerationForAttitudeCompensation = 1.5;
        if (GPSAccelerationForAttitudeCompensation < -1.5)
          GPSAccelerationForAttitudeCompensation = -1.5;

        if (GPSLateralYAcceleration > 2)
        {
          GPSLateralYAcceleration = 2;
        }
        if (GPSLateralYAcceleration < -2)
        {
          GPSLateralYAcceleration = -2;
        }
      }
    }
    else
    {
    }

    GPSLastSpeedKmhForAttitudeComponesation = GPSCurrentSpeedKmhForAttitudeComponesation;
    GPSLastTrack = singletonConfig()->NMEA_DATA.cog;

    break;
  case 9: // Date ddmmyy
    /* code */
    break;
  }
}

bool nmea_RMC_mini_parser(const uint8_t *sentence, uint16_t length)
{
  int64_t now = esp_timer_get_time();
  GPSDT = (now - GPSLastSpeedKmhReceivedTick);

  int8_t csvCounter = 0;
  int8_t decimalCounter = 0;
  int8_t decimalEnabled = 0;

  int32_t finalNumber = 0;

  for (uint8_t x = 0; x < length; x++)
  {
    if (sentence[x] == 0)
      break;
    if (sentence[x] == ',')
    {
      nmea_RMC_UpdatedValueFor(csvCounter, finalNumber, decimalCounter);
      decimalCounter = 0;
      csvCounter++;
      decimalEnabled = 0;
      finalNumber = 0;
      continue;
    }
    if (sentence[x] == '\n')
      break;
    if (sentence[x] == '\r')
      break;
    if (sentence[x] == '.')
    {
      decimalEnabled = 1;
      continue;
    }
    if (sentence[x] == '-')
    {
      finalNumber = finalNumber - 1;
    }
    else
    {
      decimalCounter += decimalEnabled;
      finalNumber = 10 * finalNumber + (sentence[x] - '0');
    }
  }

  GPSLastSpeedKmhReceivedTick = now;

  return true;
}

bool nmea_GGA_mini_parser(const uint8_t *sentence, uint16_t length)
{

  int8_t csvCounter = 0;

  int8_t decimalCounter = 0;
  int8_t decimalEnabled = 0;
  int32_t finalNumber = 0;

  for (uint8_t x = 0; x < length; x++)
  {
    if (sentence[x] == 0)
      break;
    if (sentence[x] == ',')
    {
      nmea_GGA_UpdatedValueFor(csvCounter, finalNumber, decimalCounter);
      decimalCounter = 0;
      csvCounter++;
      decimalEnabled = 0;
      finalNumber = 0;
      continue;
    }
    if (sentence[x] == '\n')
      break;
    if (sentence[x] == '\r')
      break;

    if (sentence[x] == '.')
    {
      decimalEnabled = 1;
      continue;
    }
    if (sentence[x] == '-')
    {
      finalNumber = finalNumber - 1;
    }
    else
    {
      decimalCounter += decimalEnabled;
      finalNumber = 10 * finalNumber + (sentence[x] - '0');
    }
  }

  return true;
}

#endif

#ifdef RB_ENABLE_GPS
void NMEA_ParseBuffer(const uint8_t *data, const int rxBytes, uint8_t SourceId);
#ifdef RB_ENABLE_UART
void uart_fetch_data()
{
  if (GpsSpeed0ForDisable == 0)
    return;
#ifdef RB_ENABLE_NavCore
  // In NavCore mode, SC16IS IRQ task fills a stream buffer. We only drain it here and parse NMEA
  // from the main context (safer for LVGL/UI code paths).
  static uint8_t data[UART_RX_BUF_SIZE + 1];

  //size_t rxBytes = xStreamBufferReceive(g_navcore_gps_sb, data, UART_RX_BUF_SIZE, 0);
  StreamBufferHandle_t sb = rb_navcore_gps_stream();
  size_t rxBytes = 0;
  if (sb) {
    rxBytes = xStreamBufferReceive(sb, data, UART_RX_BUF_SIZE, 0);
  }
  if (rxBytes > 0)
  {
    data[rxBytes] = 0;
    NMEA_ParseBuffer(data, (int)rxBytes, RB01_GPS_PROTOCOL_UART);
  }
  else
  {
    // keep existing GPS timeout supervision logic
    NMEA_ParseBuffer(data, 0, RB01_GPS_PROTOCOL_UART);
  }

#else
  uint8_t *data = (uint8_t *)malloc(UART_RX_BUF_SIZE + 1);
  if (data == NULL)
  {
    return;
  }
  const int rxBytes = uart_read_bytes(UART_N, data, UART_RX_BUF_SIZE, 10 / portTICK_PERIOD_MS);
  if (rxBytes > 1)
  {
    data[rxBytes] = 0;
    NMEA_ParseBuffer(data, rxBytes, RB01_GPS_PROTOCOL_UART);
  }
  free(data);
#endif
}
#endif

void NMEA_ParseBuffer(const uint8_t *data, const int rxBytes, uint8_t SourceId)
{
#ifdef RB01_GPS_PROTOCOL_BLE
  if (SourceId == RB01_GPS_PROTOCOL_BLE && singletonConfig()->settingsBluetoothGPS == 0)
  {
    return;
  }
#endif
  /*
  if(SourceId == RB01_GPS_PROTOCOL_UART && singletonConfig()->settingsBluetoothGPS == 1 && singletonConfig()->Operative_Bluetooth)
  {
    return;
  }
  */

  if (rxBytes > 0)
  {
    // data[rxBytes] = 0;
    /*
    printf("UART[%d]=", rxBytes);
    data[rxBytes] = 0;
    for (int x = 0; x < rxBytes; x++)
    {
      printf("%c", data[x]);
    }
    printf("\n");
    */
#ifdef RB_ENABLE_GPS_DIAG
    // Debug UART
    // snprintf(SettingStatus4UARTBuf,sizeof(SettingStatus4UARTBuf),"%s",data);
    lv_label_set_text(GPSDiag_NMEADebugLine, (char *)data);
#endif
    for (int x = 0; x < rxBytes - 23; x++)
    {
      //$GPRMC,,,,,,,,,,,,A*79
      if (data[x] == '$')
      {
        if (strncmp((char *)(data + x + 1), "GPRMC,", 5) == 0 || strncmp((char *)(data + x + 1), "GNRMC,", 5) == 0)
        {
          lv_label_set_text(SettingStatus4UART, "DATA RECEIVED");
          if (nmea_RMC_mini_parser(data + x, rxBytes - x))
          {
            // TODO: jump directly to the slice
            // break;
          }
#ifdef RB_ENABLE_GPS_DIAG
          if (GPSDiag_NMEADebugRMC != NULL)
          {
            lv_label_set_text(GPSDiag_NMEADebugRMC, (char *)(data + x));
          }
#endif
          // break; // Fast as possibile we go back and do not parse anymore
        }
        else
        {
          if (strncmp((char *)(data + x + 1), "GPGGA,", 5) == 0 || strncmp((char *)(data + x + 1), "GNGGA,", 5) == 0)
          {
            if (nmea_GGA_mini_parser(data + x, rxBytes - x))
            {
              // TODO: jump directly to the slice
              // break;
            }
            // break; // Fast as possibile we go back and do not parse anymore
#ifdef RB_ENABLE_GPS_DIAG
            if (GPSDiag_NMEADebugGGA != NULL)
            {
              lv_label_set_text(GPSDiag_NMEADebugGGA, (char *)(data + x));
            }
#endif
          }
        }
      }
    }
  }
  else
  {
    // 1.1.16 Enable the INOP in case the GPS is disconnected and does not trigger any data in 5 seconds
    int64_t now = esp_timer_get_time();
    int64_t isGPSTimeout = (now - GPSLastSpeedKmhReceivedTick);
    if (isGPSTimeout > 50000000)
    {
      Operative_GPS = false;
    }
  }
#ifdef RB_ENABLE_GPS_DIAG
  if (GPSDiag_NMEADebugSummary != NULL)
  {
    int64_t now = esp_timer_get_time();
    int64_t isGPSTimeout = (now - GPSLastSpeedKmhReceivedTick);
    int16_t isGPSTimeoutDeciseconds = isGPSTimeout / 100000;
    snprintf((char *)data, UART_RX_BUF_SIZE, "Parsed: %d %d %.1f %.1f %1.f", isGPSTimeoutDeciseconds, singletonConfig()->NMEA_DATA.valid, singletonConfig()->NMEA_DATA.speed, singletonConfig()->NMEA_DATA.cog, singletonConfig()->NMEA_DATA.altitude);
    lv_label_set_text(GPSDiag_NMEADebugSummary, (char *)(data));
  }
#endif
}
#endif
void nvsRestoreGMeter()
{
  //
  nvs_handle_t my_handle;
  esp_err_t err = nvs_open("storage", NVS_READONLY, &my_handle);
  if (err != ESP_OK)
  {
    printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
  }
  else
  {
    // Read
#ifdef RB_ENABLE_CONSOLE_DEBUG
    printf("Reading GMeter from NVS ...\n");
#endif
    int8_t gmeter_max = 0; // value will default to 0, if not set yet in NVS
    err = nvs_get_i8(my_handle, "gmeter_max", &gmeter_max);
    switch (err)
    {
    case ESP_OK:
#ifdef RB_ENABLE_CONSOLE_DEBUG
      printf("gmeter_max = %d\n", gmeter_max);
#endif
      float f = gmeter_max;
      GFactorMax = f / 10.0;
      break;
    case ESP_ERR_NVS_NOT_FOUND:
      break;
    default:
      printf("Error (%s) reading!\n", esp_err_to_name(err));
    }
    int8_t gmeter_min = 10; // value will default to 0, if not set yet in NVS
    err = nvs_get_i8(my_handle, "gmeter_min", &gmeter_min);
    switch (err)
    {
    case ESP_OK:
#ifdef RB_ENABLE_CONSOLE_DEBUG
      printf("gmeter_min = %d\n", gmeter_min);
#endif
      float f = gmeter_min;
      GFactorMin = f / 10.0;
      break;
    case ESP_ERR_NVS_NOT_FOUND:
      break;
    default:
      printf("Error (%s) reading!\n", esp_err_to_name(err));
    }

    printf("G-Meter read: %d %d\n",gmeter_max,gmeter_min);
    nvs_close(my_handle);
  }
}


void nvsRestoreSettings()
{
  //
  nvs_handle_t my_handle;
  esp_err_t err = nvs_open("storage", NVS_READONLY, &my_handle);
  if (err != ESP_OK)
  {
    printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
  }
  else
  {
    int32_t pcal = 0; // value will default to 0, if not set yet in NVS
    err = nvs_get_i32(my_handle, "pcal", &pcal);
    switch (err)
    {
    case ESP_OK:
#ifdef RB_ENABLE_CONSOLE_DEBUG
      printf("pcal = %ld\n", pcal);
#endif
      break;
    case ESP_ERR_NVS_NOT_FOUND:
      break;
    default:
      printf("Error (%s) reading!\n", esp_err_to_name(err));
    }

    singletonConfig()->bmp280override = pcal;
    uint8_t autoQNH = 1;
    nvs_get_u8(my_handle, "autoQNH", &autoQNH);
    singletonConfig()->settingsAutoQNH = autoQNH;

    uint8_t defaultPageOrDemo = 0xff; // value will default to 0, if not set yet in NVS
                                      // 1.1.23 Default is Advanced Attitude Indicator
#ifdef RB_02_DISPLAY_TOUCH
#else
    defaultPageOrDemo = 1;
#endif
    err = nvs_get_u8(my_handle, "default", &defaultPageOrDemo);
    switch (err)
    {
    case ESP_OK:
#ifdef RB_ENABLE_CONSOLE_DEBUG
      printf("defaultPageOrDemo = %u\n", defaultPageOrDemo);
#endif
      break;
    case ESP_ERR_NVS_NOT_FOUND:
      break;
    default:
      printf("Error (%s) reading!\n", esp_err_to_name(err));
    }

    if (defaultPageOrDemo == 0xff)
    {
      // Since we support non touch single instrument we will never start as demo
      // DeviceIsDemoMode = 1;
    }
    else
    {
      DeviceIsDemoMode = 0;
      StartupPage = defaultPageOrDemo;
    }

    // 1.1.23 GPS is mandatory
    GpsSpeed0ForDisable = 9600;

    err = nvs_get_i32(my_handle, "uart", &GpsSpeed0ForDisable);
    switch (err)
    {
    case ESP_OK:
#ifdef RB_ENABLE_CONSOLE_DEBUG
      printf("GpsSpeed0ForDisable = %ld\n", GpsSpeed0ForDisable);
#endif
      break;
    case ESP_ERR_NVS_NOT_FOUND:
      break;
    default:
      printf("Error (%s) reading!\n", esp_err_to_name(err));
    }

    GFactorDirty = 0;

    nvs_get_u8(my_handle, "isKmh", &isKmh);
    nvs_get_u16(my_handle, "degreeStart", &degreeStart);
    nvs_get_u16(my_handle, "degreeEnd", &degreeEnd);
    nvs_get_u16(my_handle, "speedKtStart", &speedKtStart);
    nvs_get_u16(my_handle, "speedKtEnd", &speedKtEnd);
    nvs_get_u16(my_handle, "speedWhite", &speedWhite);
    nvs_get_u16(my_handle, "speedGreen", &speedGreen);
    nvs_get_u16(my_handle, "speedYellow", &speedYellow);
    nvs_get_u16(my_handle, "speedRed", &speedRed);

    // 1.1.8
    uint8_t intBuffer = 3;
    nvs_get_u8(my_handle, "filterAcc", &intBuffer);
    FilterMoltiplier = intBuffer;
    intBuffer = 10;
    nvs_get_u8(my_handle, "filterGyro", &intBuffer);
    FilterMoltiplierGyro = intBuffer;
    intBuffer = 230;
    nvs_get_u8(my_handle, "filterAttitude", &intBuffer);
    AttitudeBalanceAlpha = intBuffer / 250.0;
    // 1.1.23 Bugfix Filter Output is not stored
    intBuffer = 5;
    nvs_get_u8(my_handle, "filterOut", &intBuffer);
    FilterMoltiplierOutput = intBuffer;

    nvs_get_u8(my_handle, "loopms", &DriverLoopMilliseconds);
    if (DriverLoopMilliseconds < 1)
    {
      DriverLoopMilliseconds = 1;
    }
#ifdef RB_ENABLE_CONSOLE_DEBUG
    printf("DriverLoopMilliseconds from NVS ... %d\n", DriverLoopMilliseconds);
#endif
    // 1.1.24 Warning the Gyro Calibration has being increased to 1000.0 (!)
    int16_t compBuffer = 0;
    nvs_get_i16(my_handle, "calGX", &compBuffer);
    singletonConfig()->GyroHardwareCalibration.x = compBuffer / RB_GYRO_CALIBRATION_PRECISION;
    nvs_get_i16(my_handle, "calGY", &compBuffer);
    singletonConfig()->GyroHardwareCalibration.y = compBuffer / RB_GYRO_CALIBRATION_PRECISION;
    nvs_get_i16(my_handle, "calGZ", &compBuffer);
    singletonConfig()->GyroHardwareCalibration.z = compBuffer / RB_GYRO_CALIBRATION_PRECISION;
#ifdef RB_ENABLE_CONSOLE_DEBUG
    printf("Gyro Calibration %.1f %.1f %.1f\n", singletonConfig()->GyroHardwareCalibration.x, singletonConfig()->GyroHardwareCalibration.z, singletonConfig()->GyroHardwareCalibration.z);
#endif

    compBuffer = 0;
    nvs_get_i16(my_handle, "panAZ", &compBuffer);
    PanelAlignment.z = compBuffer / (RB_GYRO_CALIBRATION_PRECISION / 2.0);

    compBuffer = 0;
    nvs_get_i16(my_handle, "panAY", &compBuffer);
    PanelAlignment.y = compBuffer / (RB_GYRO_CALIBRATION_PRECISION / 2.0);

    compBuffer = 0;
    nvs_get_i16(my_handle, "panAX", &compBuffer);
    PanelAlignment.x = compBuffer / (RB_GYRO_CALIBRATION_PRECISION / 2.0);

    gyroHardwareSetCalibration(GyroFiltered.x, GyroFiltered.y, GyroFiltered.z);

#ifdef RB_ENABLE_GPS
    // 1.1.25B
    int32_t i32buffer = 0;
    nvs_get_i32(my_handle, "latitude100", &i32buffer);
    singletonConfig()->NMEA_DATA.latitude = i32buffer / 100.0;
    nvs_get_i32(my_handle, "longitude100", &i32buffer);
    singletonConfig()->NMEA_DATA.longitude = i32buffer / 100.0;
#endif

#ifdef RB02_ESP_BLUETOOTH
    uint8_t settingsBluetoothEnabled = 0;
    nvs_get_u8(my_handle, NVS_KEY_BT_ENABLE, &settingsBluetoothEnabled);
    singletonConfig()->settingsBluetoothEnabled = settingsBluetoothEnabled;

    nvs_get_u8(my_handle, NVS_KEY_BT_GPS, &settingsBluetoothEnabled);
    singletonConfig()->settingsBluetoothGPS = settingsBluetoothEnabled;

#endif
    nvs_close(my_handle);
  }
}

void nvsStoreSpeedArc()
{
  //
  nvs_handle_t my_handle;
  esp_err_t err = nvs_open("storage", NVS_READWRITE, &my_handle);
  if (err != ESP_OK)
  {
    printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
  }
  else
  {
    err = nvs_set_u8(my_handle, "isKmh", isKmh);

    err = nvs_set_u16(my_handle, "degreeStart", degreeStart);

    err = nvs_set_u16(my_handle, "degreeEnd", degreeEnd);

    err = nvs_set_u16(my_handle, "speedKtStart", speedKtStart);

    err = nvs_set_u16(my_handle, "speedKtEnd", speedKtEnd);

    err = nvs_set_u16(my_handle, "speedWhite", speedWhite);

    err = nvs_set_u16(my_handle, "speedGreen", speedGreen);

    err = nvs_set_u16(my_handle, "speedYellow", speedYellow);

    err = nvs_set_u16(my_handle, "speedRed", speedRed);

    nvs_close(my_handle);
  }
}

void nvsStoreUARTBaudrate()
{
  //
  nvs_handle_t my_handle;
  esp_err_t err = nvs_open("storage", NVS_READWRITE, &my_handle);
  if (err != ESP_OK)
  {
    printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
  }
  else
  {
    // Read
    err = nvs_set_i32(my_handle, "uart", GpsSpeed0ForDisable);
    nvs_close(my_handle);
  }
}

// 1.1.8
void nvsStoreFilters()
{
  //
  nvs_handle_t my_handle;
  esp_err_t err = nvs_open("storage", NVS_READWRITE, &my_handle);
  if (err != ESP_OK)
  {
    printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
  }
  else
  {
    // Write
    uint8_t intBuffer = FilterMoltiplier;
    printf("Writing filterAcc from NVS ... %d\n", intBuffer);
    nvs_set_u8(my_handle, "filterAcc", intBuffer);
    intBuffer = FilterMoltiplierGyro;
    printf("Writing filterGyro from NVS ... %d\n", intBuffer);
    nvs_set_u8(my_handle, "filterGyro", intBuffer);
    intBuffer = 250 * AttitudeBalanceAlpha;
    printf("Writing filterAttitude from NVS ... %d\n", intBuffer);
    nvs_set_u8(my_handle, "filterAttitude", intBuffer);

    printf("Writing DriverLoopMilliseconds from NVS ... %d\n", DriverLoopMilliseconds);
    nvs_set_u8(my_handle, "loopms", DriverLoopMilliseconds);

    // 1.1.23
    intBuffer = FilterMoltiplierOutput;
    printf("Writing filterOut from NVS ... %d\n", intBuffer);
    nvs_set_u8(my_handle, "filterOut", intBuffer);

    nvs_close(my_handle);
  }
}

void nvsStoreGyroCalibration()
{
  //
  nvs_handle_t my_handle;
  esp_err_t err = nvs_open("storage", NVS_READWRITE, &my_handle);
  if (err != ESP_OK)
  {
    printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
  }
  else
  {
    // Write
    int16_t calBuffer = 0;

    calBuffer = singletonConfig()->GyroHardwareCalibration.x * RB_GYRO_CALIBRATION_PRECISION;
    nvs_set_i16(my_handle, "calGX", calBuffer);
    calBuffer = singletonConfig()->GyroHardwareCalibration.y * RB_GYRO_CALIBRATION_PRECISION;
    nvs_set_i16(my_handle, "calGY", calBuffer);
    calBuffer = singletonConfig()->GyroHardwareCalibration.z * RB_GYRO_CALIBRATION_PRECISION;
    nvs_set_i16(my_handle, "calGZ", calBuffer);

    calBuffer = PanelAlignment.z * (RB_GYRO_CALIBRATION_PRECISION / 2.0);
    nvs_set_i16(my_handle, "panAZ", calBuffer);

    calBuffer = PanelAlignment.y * (RB_GYRO_CALIBRATION_PRECISION / 2.0);
    nvs_set_i16(my_handle, "panAY", calBuffer);

    calBuffer = PanelAlignment.x * (RB_GYRO_CALIBRATION_PRECISION / 2.0);
    nvs_set_i16(my_handle, "panAX", calBuffer);


    nvs_close(my_handle);
  }
}

void nvsStorePCal()
{
  //
  nvs_handle_t my_handle;
  esp_err_t err = nvs_open("storage", NVS_READWRITE, &my_handle);
  if (err != ESP_OK)
  {
    printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
  }
  else
  {
    // Read
    err = nvs_set_i32(my_handle, "pcal", singletonConfig()->bmp280override);
    // 1.1.26
    nvs_set_u8(my_handle, "autoQNH", singletonConfig()->settingsAutoQNH);
    nvs_close(my_handle);
  }
}

void nvsStoreGMeter()
{
  //
  nvs_handle_t my_handle;
  esp_err_t err = nvs_open("storage", NVS_READWRITE, &my_handle);
  if (err != ESP_OK)
  {
    printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
  }
  else
  {
    // Read
    printf("G-Meter store: %f %f\n",GFactorMax,GFactorMin);
    err = nvs_set_i8(my_handle, "gmeter_max", 10.0 * GFactorMax);
    err = nvs_set_i8(my_handle, "gmeter_min", 10.0 * GFactorMin);

    nvs_close(my_handle);
  }
}

void nvsStoreDefaultScreenOrDemo()
{
  //
  nvs_handle_t my_handle;
  esp_err_t err = nvs_open("storage", NVS_READWRITE, &my_handle);
  if (err != ESP_OK)
  {
    printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
  }
  else
  {
    // Read
    uint8_t value = 0xff;

    if (DeviceIsDemoMode == 1)
    {
    }
    else
    {
      lv_tabview_t *tabview = (lv_tabview_t *)tv;
      value = (uint8_t)tabview->tab_cur;
    }

    err = nvs_set_u8(my_handle, "default", value);
    nvs_close(my_handle);
  }
}
#ifdef RB_ENABLE_ATT
void update_Attitude_lvgl_tick(lv_timer_t *t)
{
  float moveY = 4.0 * AttitudePitch;
  if (moveY > 120)
    moveY = 120;
  else
  {
    if (moveY < -100)
    {
      moveY = -100;
    }
  }

  if (moveY == lastAttitudePitch && lastAttitudeRoll == -AttitudeRoll)
  {
    return;
  }
  lastAttitudePitch = moveY;
  lastAttitudeRoll = -AttitudeRoll;
  // 1.1.1 Rotation of Aircraft symbol
  lv_img_set_angle(Screen_Attitude_Pitch, lastAttitudeRoll * 10.0);
  lv_obj_set_pos(Screen_Attitude_Pitch, 0, -lastAttitudePitch + att_aircraft.header.h / 2);

  bool isAttitudeFast = true;
  if (isAttitudeFast == false)
  {
    rotate_AttitudeGearByDegree(lastAttitudeRoll);
  }
  else
  {
    lv_img_set_angle(Screen_Attitude_RollIndicator, lastAttitudeRoll * 10.0);
  }
}

void rotate_AttitudeGearByDegree(int lastAttitudeRoll)
{

  for (int a = 0; a < 4; a++)
  {
    if (Screen_Attitude_Rounds[a] == NULL)
      continue;
    int32_t c20 = lv_trigo_sin(lastAttitudeRoll + a * 90) * 19 / 3276;
    int32_t s20 = lv_trigo_sin(lastAttitudeRoll + 90 + a * 90) * 19 / 3276;
    int32_t c21 = lv_trigo_sin(lastAttitudeRoll + a * 90) * 12 / 3276;
    int32_t s21 = lv_trigo_sin(lastAttitudeRoll + 90 + a * 90) * 12 / 3276;
    switch (a)
    {
    case 0:
      lv_obj_set_pos(Screen_Attitude_Rounds[a], -c20, s20);
      lv_img_set_angle(Screen_Attitude_Rounds[a], lastAttitudeRoll * 10.0);
      break;
    case 3: // TL
      lv_obj_set_pos(Screen_Attitude_Rounds[a], -c20 - s21, s20 - c21);
      lv_img_set_angle(Screen_Attitude_Rounds[a], lastAttitudeRoll * 10.0);
      break;
    case 1: // TR
      lv_obj_set_pos(Screen_Attitude_Rounds[a], -c20 + s21, s20 + c21);
      lv_img_set_angle(Screen_Attitude_Rounds[a], lastAttitudeRoll * 10.0);
      break;
    }
  }
}
#endif
#ifdef RB_ENABLE_TRN
void update_TurnSlip_lvgl_tick(lv_timer_t *t)
{

  float fay = AccelFiltered.y;
  float ydpg = AttitudeYawDegreePerSecond;

  if (fay < -1.1)
    fay = -1.1;
  else if (fay > 1.1)
    fay = 1.1;

  char buf[10];
  if (ydpg > 1)
  {
    snprintf(buf, sizeof(buf), "%.0f", ydpg);
  }
  else
  {
    if (ydpg < -1)
    {
      snprintf(buf, sizeof(buf), "%.0f", -ydpg);
    }
    else
    {
      buf[0] = 0;
    }
  }

  if (ydpg < -10)
    ydpg = -10;
  else if (ydpg > 10)
    ydpg = 10;

  lv_label_set_text(Screen_TurnSlip_Obj_Label, buf);

  float range_X = 120;
  float range_Y = 16;
  lv_obj_set_pos(Screen_TurnSlip_Obj_Ball, -range_X * fay, 88 - abs((int)(range_Y * fay)));
  // 1.1.3 Bugfix: Bias was not applied to Turn & Slip
  lv_img_set_angle(Screen_TurnSlip_Obj_Turn, -10.0 * (ydpg) * 8.0);
}

void update_TurnSlip2_lvgl_tick(lv_timer_t *t)
{
  int16_t BallAngle = 1800 - (Accel.x * 1800);
  if (Accel.y < 0)
    BallAngle = BallAngle * -1;
  lv_img_set_angle(Ball, BallAngle / 2);
}
#endif
int32_t bmp280Pressure = 0;
int32_t bmp280Temperature = 0;
int32_t Altimeter = 0;
int32_t Variometer = 0;
int32_t pressureCompensation2(int32_t adc_T, int32_t adc_P)
{
  int32_t var1t, var2t;
  int32_t t_fine;
  adc_T >>= 4;

  var1t = ((((adc_T >> 3) - ((int32_t)bmp280Calibration[0] << 1))) *
           ((int32_t)bmp280Calibration[1])) >>
          11;

  var2t = (((((adc_T >> 4) - ((int32_t)bmp280Calibration[0])) *
             ((adc_T >> 4) - ((int32_t)bmp280Calibration[0]))) >>
            12) *
           ((int32_t)bmp280Calibration[2])) >>
          14;

  t_fine = var1t + var2t;

  float T = (t_fine * 5 + 128) >> 8;

  bmp280Temperature = T / 100;

  int64_t var1p, var2p, p;

  adc_P >>= 4;

  var1p = ((int64_t)t_fine) - 128000;
  var2p = var1p * var1p * (int64_t)bmp280Calibration[8];
  var2p = var2p + ((var1p * (int64_t)bmp280Calibration[7]) << 17);
  var2p = var2p + (((int64_t)bmp280Calibration[6]) << 35);
  var1p = ((var1p * var1p * (int64_t)bmp280Calibration[5]) >> 8) +
          ((var1p * (int64_t)bmp280Calibration[4]) << 12);
  var1p =
      (((((int64_t)1) << 47) + var1p)) * ((int64_t)bmp280Calibration[3]) >> 33;

  if (var1p == 0)
  {
    return 0; // avoid exception caused by division by zero
  }
  p = 1048576 - adc_P;
  p = (((p << 31) - var2p) * 3125) / var1p;
  var1p = (((int64_t)bmp280Calibration[11]) * (p >> 13) * (p >> 13)) >> 25;
  var2p = (((int64_t)bmp280Calibration[10]) * p) >> 19;

  p = ((p + var1p + var2p) >> 8) + (((int64_t)bmp280Calibration[9]) << 4);
  bmp280Pressure = singletonConfig()->bmp280override + (float)(p / 256.0);
  return bmp280Pressure;
}

int32_t temperatureCompensation(int32_t adc_T)
{
  BMP280_S32_t var1, var2, T;
  BMP280_S32_t dig_T1 = bmp280Calibration[0];
  BMP280_S32_t dig_T2 = bmp280Calibration[1];
  BMP280_S32_t dig_T3 = bmp280Calibration[2];

  var1 = ((((adc_T >> 3) - (dig_T1 << 1))) * ((BMP280_S32_t)dig_T2)) >> 11;
  var2 = (((((adc_T >> 4) - ((BMP280_S32_t)dig_T1)) * ((adc_T >> 4) - ((BMP280_S32_t)dig_T1))) >> 12) *
          ((BMP280_S32_t)dig_T3)) >>
         14;
  t_fine = var1 + var2;
  T = (t_fine * 5 + 128) >> 8;

  bmp280Temperature = T;
  return T;
}

uint32_t pressureCompensation(int32_t adc_P)
{

  BMP280_S32_t dig_P1 = bmp280Calibration[3];
  BMP280_S32_t dig_P2 = bmp280Calibration[4];
  BMP280_S32_t dig_P3 = bmp280Calibration[5];
  BMP280_S32_t dig_P4 = bmp280Calibration[6];
  BMP280_S32_t dig_P5 = bmp280Calibration[7];
  BMP280_S32_t dig_P6 = bmp280Calibration[8];
  BMP280_S32_t dig_P7 = bmp280Calibration[9];
  BMP280_S32_t dig_P8 = bmp280Calibration[10];
  BMP280_S32_t dig_P9 = bmp280Calibration[11];

  BMP280_S64_t var1, var2, p;
  var1 = ((BMP280_S64_t)t_fine) - 128000;
  var2 = var1 * var1 * (BMP280_S64_t)dig_P6;
  var2 = var2 + ((var1 * (BMP280_S64_t)dig_P5) << 17);
  var2 = var2 + (((BMP280_S64_t)dig_P4) << 35);
  var1 = ((var1 * var1 * (BMP280_S64_t)dig_P3) >> 8) + ((var1 * (BMP280_S64_t)dig_P2) << 12);
  var1 = (((((BMP280_S64_t)1) << 47) + var1)) * ((BMP280_S64_t)dig_P1) >> 33;
  if (var1 == 0)
  {
    return 0; // avoid exception caused by division by zero
  }
  p = 1048576 - adc_P;
  p = (((p << 31) - var2) * 3125) / var1;
  var1 = (((BMP280_S64_t)dig_P9) * (p >> 13) * (p >> 13)) >> 25;
  var2 = (((BMP280_S64_t)dig_P8) * p) >> 19;
  p = ((p + var1 + var2) >> 8) + (((BMP280_S64_t)dig_P7) << 4);
  bmp280Pressure = singletonConfig()->bmp280override + p / 256;
  return bmp280Pressure;
}

void example1_BMP280_lvgl_tick(lv_timer_t *t)
{
  uint8_t buf[6] = {0, 0, 0, 0, 0, 0};
  I2C_Read(0x76, 0xF7, &buf[0], 6);

  int32_t adc_t = ((buf[3] << 12) | (buf[4] << 4) | (buf[5] >> 4));
  int32_t adc_p = ((buf[0] << 12) | (buf[1] << 4) | (buf[2] >> 4));
  temperatureCompensation(adc_t);
  pressureCompensation(adc_p);

  // Altimeter is *100 feet
  int32_t AltimeterNew = (((QNH * 100) - bmp280Pressure) * 30.0);
  // 1.1.6
  if (AltimeterNew < -100000 || AltimeterNew > 1200000)
  {
    Operative_BMP280 = 0;
  }
  else
  {
    Operative_BMP280 = 1;
  }
  // Polling is 1Hz = 0.01 Feet/Second
  int32_t VariometerInstant = AltimeterNew - Altimeter;
  // 1.0.9 Variometer is filterd
  Variometer = (Variometer + VariometerInstant) / 2;
  Altimeter = AltimeterNew;
// 1.1.1
#ifdef RB_ENABLE_CONSOLE_DEBUG
  printf("BMP280 ");
  for (int i = 0; i < 6; i++)
  {
    printf("%X ", buf[i]);
  }

  for (int i = 0; i < 12; i++)
  {
    printf("%ld ", bmp280Calibration[i]);
  }

  printf(" Pressure: %lu Temperature: %ld Altimeter: %ld Variometer: %ld Read T: %ld Read P: %ld V: %ld Gy:%.1f %.1f %.1f\n",
         bmp280Pressure,
         bmp280Temperature,
         Altimeter,
         Variometer,
         adc_t, adc_p, ((Variometer * 6) * 36 / 400),
         Gyro.x,
         Gyro.y,
         Gyro.z);
#endif
}
void Get_BMP280(void)
{
  example1_BMP280_lvgl_tick(NULL);
}
#ifdef RB_ENABLE_SPD
void update_Speed_lvgl_tick(lv_timer_t *t)
{
  static int lastSpeed = -1;
  int speed = 0;



  // First we go for GPS
#ifdef RB_ENABLE_GPS
  int Gpsspeed = 0;
  speed = singletonConfig()->NMEA_DATA.speed * 10;
  Gpsspeed = speed;
#endif

// We are using the NavCore define because it is already "protected by IAS"
#if defined(RB_ENABLE_NavCore)
  // Airspeed from MS4525DO (dP) instead of GPS speed
  // rho = 1.225 by default (ISA sea level). You can later compute rho from pressure/temp.
  if (MS4525DO_Update(1.225f) == ESP_OK)
  {
    int iASSpeed = MS4525DO_AirspeedKmh10(); // km/h * 10
    speed = (lastSpeed * 5 + iASSpeed) / 6; 
  }
  else
  {
    // Raise INOP
    // TODO
  }
#else

#endif

  if (speed != lastSpeed)
  {
    // printf("NMEA: Speed %d-->%d m/s Angle: %ld\n", lastSpeed, speed, (int32_t)(speed));
    lastSpeed = speed;
    // 1.1.2 You can configure the ARCs and Min-Max speed displayed
    float rangeDegree = degreeEnd - degreeStart;
    float rangeKt = speedKtEnd - speedKtStart;
    float ratioDK = (rangeDegree) / (rangeKt);

    float isKt = 1.852;
    if (isKmh == 1)
    {
      isKt = 1.0;
    }
    int speedCapped = (speed / isKt) - (speedKtStart * 10);
    /*
    if (speedCapped < 0)
    {
      // We decided to stay "before" nearby the first indicator
      speedCapped = -rangeKt/5;
    }
      */
    int32_t speedAngle = degreeStart * 10 + (ratioDK * speedCapped);
    if (speedAngle < 0)
    {
      speedAngle = 0;
    }

    lv_img_set_angle(Screen_Speed_SpeedTick, speedAngle + 900);

    char buf[15];
    float fspeed = 0;
#ifdef RB_ENABLE_GPS
    fspeed = singletonConfig()->NMEA_DATA.speed;
#endif
    snprintf(buf, sizeof(buf), "%.0f", fspeed / isKt); // KT
    lv_label_set_text(singletonConfig()->ui.labelGPSSpeed, buf);

#if defined(RB_ENABLE_IAS)
    snprintf(buf, sizeof(buf), "%.0f", (float)speed / 10 / isKt); // KT
    lv_label_set_text(singletonConfig()->ui.labelIASSpeed, buf);
#endif
  }
}
#endif

#ifdef RB_ENABLE_VAR
void update_Variometer_lvgl_tick(lv_timer_t *t)
{
  // 1.0.9 Variometer is *100 and 1Hz
  // Variometer => 0.01 Feet/Second => *60/100 => Feet/min
  // UI 90° => 1000ft/min => 180° = 2000ft/min => 360° = 4000ft/min
  // LVGL is *10 Degree;
  int32_t VDegree = ((Variometer * 6.0 / 10.0) * 36.0 / 40);
  lv_img_set_angle(Screen_Variometer_Cents, VDegree);
}
#endif

#ifdef RB_ENABLE_ALT
void update_Altimeter_lvgl_tick(lv_timer_t *t)
{
  // Altimeter Analog Screen
  // 1.0.9 Altimter is *100 Feet
  int32_t AltimeterInFeet = Altimeter / 100.0;
  float milesDegree = (36.0 * AltimeterInFeet) / 100.0;
  float centsDegree = (360.0 * ((AltimeterInFeet) % 1000)) / 100.0;
  lv_img_set_angle(Screen_Altitude_Miles, milesDegree + 900);
  lv_img_set_angle(Screen_Altitude_Cents, centsDegree + 900);

#ifdef RB_ENABLE_GPS
  char buf[10];
  snprintf(buf, sizeof(buf), "%.0f", singletonConfig()->NMEA_DATA.altitude * 3.28084);
  lv_label_set_text(singletonConfig()->ui.altimeterAnalog.labelGPSAltitude, buf);
#endif
}
#endif
#ifdef RB_ENABLE_GPS
void uartApplyRates()
{
  // snprintf(SettingStatus4UARTBuf,sizeof(SettingStatus4UARTBuf),"NO DATA RECEIVED");
  lv_label_set_text(SettingStatus4UART, "NO DATA RECEIVED");
#ifdef RB_ENABLE_GPS_DIAG
  if (GPSDiag_UARTBaud != NULL)
  {
    char buf[20];
    snprintf(buf, sizeof(buf), "UART Baud: %ld", GpsSpeed0ForDisable);
    lv_label_set_text(GPSDiag_UARTBaud, buf);
  }
#endif
  if (GpsSpeed0ForDisable > 0)
  {
#ifdef RB_ENABLE_UART
#ifdef RB_ENABLE_NavCore
    rb_navcore_init();
#else
    const uart_config_t uart_config = {
        .baud_rate = GpsSpeed0ForDisable, // TODO: Delay the setup up to the LVGL started
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    // Setup Baud Rate
    uart_param_config(1, &uart_config);
#endif
#endif
  }
  else
  {
  }
}
#endif
void bmp280Setup()
{
  uint8_t bmp280Control[1] = {0x57};
  uint8_t bmp280Settings[1] = {0x9C};
  I2C_Write(0x76, 0xF4, &bmp280Control[0], 1);
  I2C_Write(0x76, 0xF5, &bmp280Settings[0], 1);
}

void readCalibration()
{
  uint8_t buf[24] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  I2C_Read(0x76, 0x88, &buf[0], 24);
  /* Endianess. */
#ifdef RB_ENABLE_CONSOLE_DEBUG
  printf("Calibration: ");
#endif
  for (int i = 0; i < 12; i++)
  {
#ifdef RB_ENABLE_CONSOLE_DEBUG
    printf("%X%X ", buf[2 * i + 1], buf[2 * i]);
#endif
    int16_t signedPass = (((buf[2 * i + 1]) << 8) | buf[2 * i]);
    uint16_t usignedPass = (((buf[2 * i + 1]) << 8) | buf[2 * i]);
    if (i == 0 || i == 3)
      bmp280Calibration[i] = usignedPass;
    else
      bmp280Calibration[i] = signedPass;
  }
#ifdef RB_ENABLE_CONSOLE_DEBUG
  printf("\n");
#endif
  example1_BMP280_lvgl_tick(NULL);
  Variometer = 0;
}

void rb_check_attitude_inop()
{
  Operative_Attitude = 1;
  if (GyroFiltered.x == 0 || GyroFiltered.y == 0 || GyroFiltered.z == 0)
  {
    Operative_Attitude = 0;
  }

  /* 1.1.24 Calibration is changed. The Community version is simplified
  if (GyroBias.x == 0 || GyroBias.y == 0 || GyroBias.z == 0)
  {
    Operative_Attitude = 0;
  }
  else
  {
    if (GyroBias.x > 15 || GyroBias.x < -15 || GyroBias.y > 15 || GyroBias.y < -15 || GyroBias.z > 15 || GyroBias.z < -15)
    {
      Operative_Attitude = 0;
    }
  }
  */
}

void RB02_CreateScreens()
{
#ifdef RB_ENABLE_GPS_DIAG
  RB02_GPSDiag_CreateScreen(singletonConfig()->ui.tGPSDiag);
#endif

#ifdef RB_ENABLE_CONSOLE
  RB02_Console_CreateScreen(singletonConfig()->ui.tConsole);
#endif
  // 1.1.6
  OperativeWarning = lv_img_create(lv_scr_act());
  lv_obj_add_flag(OperativeWarning, LV_OBJ_FLAG_HIDDEN);
  lv_img_set_src(OperativeWarning, &InopWarning);
  lv_obj_set_size(OperativeWarning, InopWarning.header.w, InopWarning.header.h);
  lv_obj_align(OperativeWarning, LV_ALIGN_CENTER, 0, 200);
  lv_obj_set_scrollbar_mode(OperativeWarning, LV_SCROLLBAR_MODE_OFF);
  lv_obj_clear_flag(OperativeWarning, LV_OBJ_FLAG_CLICKABLE);
#ifdef DISABLED
  RB02_Navigator_CreateScreen(singletonConfig(), lv_scr_act());
#endif

#ifdef ENABLE_VENDOR
  if (VendorSplashScreenImage == NULL)
  {
    bool loadInternalSplashscreen = false;
#if ENABLE_VENDOR == RB_VENDOR_1
    loadInternalSplashscreen = true;
#endif
    if (SDCard_Size > 0 && loadInternalSplashscreen == false)
    {
      FILE *f = fopen("/sdcard/SS48016.bmp", "r");
      if (f == NULL)
      {
        loadInternalSplashscreen = true;
      }
      else
      {
        fclose(f);
        f = NULL;
        lv_obj_t *backgroundImage = lv_img_create(lvTabSplashScreen);
        lv_img_set_src(backgroundImage, "S:/SS48016.bmp");
        lv_obj_set_size(backgroundImage, SCREEN_WIDTH, SCREEN_HEIGHT);
        lv_obj_align(backgroundImage, LV_ALIGN_CENTER, 0, 0);
        lv_obj_set_scrollbar_mode(backgroundImage, LV_SCROLLBAR_MODE_OFF);
        lv_obj_set_scrollbar_mode(lvTabSplashScreen, LV_SCROLLBAR_MODE_OFF);
        // 1.1.9 Remove scrolling for Turbolence touch screen
        lv_obj_clear_flag(lvTabSplashScreen, LV_OBJ_FLAG_SCROLLABLE);
        VendorSplashScreenImage = backgroundImage;
        loadInternalSplashscreen = false;
#ifdef RB_ENABLE_CONSOLE
        RB02_Console_AppendLog(RB02_LOG_MAIN, RB02_LOG_INFO, "SS48016.bmp loaded");
#endif
      }
    }
    else
    {
      loadInternalSplashscreen = true;
    }
    if (loadInternalSplashscreen)
    {
      VendorSplashScreenImage = Onboard_create_Base(lvTabSplashScreen, &VendorSplashScreen);
    }
  }
#endif
#ifdef RB_ENABLE_AAT

  RB02_AdvancedAttitude_CreateScreen(&advancedAttitude_Status);
  lv_obj_add_event_cb(advancedAttitude_Status.lv_parent, speedBgClicked, LV_EVENT_CLICKED, NULL);
#endif

#ifdef RB_ENABLE_TRAFFIC
  RB05_Radar_CreateScreen(&(singletonConfig()->trafficStatus));
  lv_obj_add_event_cb(singletonConfig()->trafficStatus.lv_parent_radar, speedBgClicked, LV_EVENT_CLICKED, NULL);
  RB05_Traffic_CreateScreen(&(singletonConfig()->trafficStatus));
  lv_obj_add_event_cb(singletonConfig()->trafficStatus.lv_parent, speedBgClicked, LV_EVENT_CLICKED, NULL);
#endif
#ifdef RB_ENABLE_TRK
  RB02_Gyro_CreateScreen(singletonConfig()->ui.Gyro.parent);
  lv_obj_add_event_cb(singletonConfig()->ui.Gyro.parent, speedBgClicked, LV_EVENT_CLICKED, NULL);
#endif

#ifdef RB_ENABLE_EMS
  RB04_EMS_CreateScreen(singletonConfig()->ui.tEMS, singletonConfig());
  lv_obj_add_event_cb(singletonConfig()->ui.tEMS, speedBgClicked, LV_EVENT_CLICKED, NULL);
#endif
}

void rb_increase_lvgl_tick(lv_timer_t *t)
{
  // TODO: add a configuration panel
  uint8_t forcedCalibrationOnBoot = true;
#ifdef RB_02_DISPLAY_TOUCH
  // TODO: move in settings
  forcedCalibrationOnBoot = true;
#endif
  static int stepDown = 50;
  if (workflow > 100)
  {
    // Completed

    stepDown = stepDown - 1;
    if (stepDown < 1)
    {
      stepDown = 50;

      if (GFactorDirty != 0 && DeviceIsDemoMode == 0)
      {
        GFactorDirty = 0;
        nvsStoreGMeter();
      }

      if (DeviceIsDemoMode > 0 && mbox1 == NULL)
      {

        lv_tabview_t *tabview = (lv_tabview_t *)tv;
        uint16_t cur = tabview->tab_cur;
        cur++;
        // Skip setup page
        if (cur >= tabview->tab_cnt - 2)
        {
          cur = 0;
        }
        lv_tabview_set_act(tv, cur, LV_ANIM_ON);
      }
    }
  }
  else
  {
    switch (workflow)
    {
    case 0:
      Buzzer_On();
      break;
    case 1:
      Buzzer_Off();
      break;
    case 2:
      LCD_Backlight = 10;
      Set_Backlight(LCD_Backlight);
      break;
    case 3:
      gyroHardwareCalibrationToZero();
      break;
    case 35:
      LCD_Backlight = Backlight_MAX;
      Set_Backlight(LCD_Backlight);
      break;
    case 5:
      RB02_Example1();
      break;
    case 30:
      RB02_CreateScreens();
      break;
    case 31:
      break;
    case 32:
      break;
    case 33:
      break;
    case 34:
      break;
    case 6:
      bmp280Setup();
      break;
    case 7:
      readCalibration();
      break;
    case 78:
      if (forcedCalibrationOnBoot == true)
      {
        GyroBiasAcquire[0].x = GyroFiltered.x;
        GyroBiasAcquire[0].y = GyroFiltered.y;
        GyroBiasAcquire[0].z = GyroFiltered.z;
      }
      break;
    case 40:
#ifdef RB02_ESP_BLUETOOTH
      esp_ble_gap_start_scanning(30); // scan for 30 seconds
#ifdef RB_ENABLE_CONSOLE
      RB02_Console_AppendLog(6, RB02_LOG_INFO, "BLE Scan started");
#endif
#endif
      break;
    case 79:
      if (forcedCalibrationOnBoot == true)
      {
        GyroBiasAcquire[1].x = GyroFiltered.x;
        GyroBiasAcquire[1].y = GyroFiltered.y;
        GyroBiasAcquire[1].z = GyroFiltered.z;
      }
      break;
    case 60:
      lv_obj_clear_flag(OperativeWarning, LV_OBJ_FLAG_HIDDEN);
      break;
    case 80:
      if (forcedCalibrationOnBoot == true)
      {
        GyroBiasAcquire[2].x = GyroFiltered.x;
        GyroBiasAcquire[2].y = GyroFiltered.y;
        GyroBiasAcquire[2].z = GyroFiltered.z;
        GyroBias.x = -(GyroBiasAcquire[0].x + GyroBiasAcquire[1].x + GyroBiasAcquire[2].x) / 3.0;
        GyroBias.y = -(GyroBiasAcquire[0].y + GyroBiasAcquire[1].y + GyroBiasAcquire[2].y) / 3.0;
        GyroBias.z = -(GyroBiasAcquire[0].z + GyroBiasAcquire[1].z + GyroBiasAcquire[2].z) / 3.0;

#ifdef RB_ENABLE_CONSOLE
        char ConsoleLogBuffer[20];
        snprintf(ConsoleLogBuffer, sizeof(ConsoleLogBuffer), "CAL: %.1f,%.1f,%.1f", GyroBias.x, GyroBias.y, GyroBias.z);
        RB02_Console_AppendLog(RB02_LOG_MAIN, RB02_LOG_INFO, ConsoleLogBuffer);
#endif
      }
      rb_check_attitude_inop();

      break;
    case 81:
      lv_obj_add_flag(OperativeWarning, LV_OBJ_FLAG_HIDDEN);
      break;
    case 90:
      stopwatch = datetime;
      break;
    case 91:
#ifdef RB_ENABLE_GPS
      uartApplyRates();
      switch (GpsSpeed0ForDisable)
      {
      case 0:
        lv_dropdown_set_selected(uartDropDown, 0);
        break;
      case 4800:
        lv_dropdown_set_selected(uartDropDown, 1);
        break;
      case 9600:
        lv_dropdown_set_selected(uartDropDown, 2);
        break;
      case 19200:
        lv_dropdown_set_selected(uartDropDown, 3);
        break;
      case 38400:
        lv_dropdown_set_selected(uartDropDown, 4);
        break;
      case 115200:
        lv_dropdown_set_selected(uartDropDown, 5);
        break;
      }

#ifdef RB_ENABLE_CONSOLE
      char ConsoleLogBuffer[10];
      snprintf(ConsoleLogBuffer, sizeof(ConsoleLogBuffer), "UART %ld", GpsSpeed0ForDisable);
      RB02_Console_AppendLog(RB02_LOG_MAIN, RB02_LOG_INFO, ConsoleLogBuffer);
#endif
#else
      GpsSpeed0ForDisable = 0;
#endif
      break;
    case 99:
      // Bugfix for alignment Gyro-Accelometer during initialisation timing
      nvsRestoreGMeter();
      AccelFilteredMax.x = 1;
      AccelFilteredMax.y = 0;
      GFactorDirty = 0;
    break;
    case 100:

#ifdef RB_ENABLE_CONSOLE
      RB02_Console_AppendLog(RB02_LOG_MAIN, RB02_LOG_INFO, "Workflow completed");
#endif

      datetimeTimer1 = datetime;
      datetimeTimer2 = datetime;
      datetimeTimer3 = datetime;
      if (singletonConfig()->ui.Loading_slider != NULL)
      {
        lv_obj_add_flag(singletonConfig()->ui.Loading_slider, LV_OBJ_FLAG_HIDDEN);
        lv_obj_del(singletonConfig()->ui.Loading_slider);
        singletonConfig()->ui.Loading_slider = NULL;
      }
      // 1.1.5 Added Vendor Splashscreen
#ifdef ENABLE_VENDOR
      /*
      if(VendorSplashScreenImage!=NULL){
            lv_obj_add_flag(VendorSplashScreenImage, LV_OBJ_FLAG_HIDDEN);
            lv_obj_del(VendorSplashScreenImage);
            VendorSplashScreenImage = NULL;
      }
      */
      if (StartupPage != 0)
      {
        lv_tabview_set_act(tv, StartupPage, LV_ANIM_OFF);
      }
#endif

      break;
    default:
      break;
    }

    workflow++;
    if (singletonConfig()->ui.Loading_slider != NULL)
    {
      lv_slider_set_value(singletonConfig()->ui.Loading_slider, workflow, LV_ANIM_ON);
    }
  }

  static uint16_t lastTab = 0;

  if (lastTab != ((lv_tabview_t *)tv)->tab_cur)
  {
#ifdef RB_ENABLE_CONSOLE_DEBUG
    printf("Tab Changed to: %d\n", (((lv_tabview_t *)tv)->tab_cur));
#endif
    lastTab = ((lv_tabview_t *)tv)->tab_cur;
    stepDown = 50;
    return;
  }

  switch (((lv_tabview_t *)tv)->tab_cur)
  {

#ifdef RB_ENABLE_AAT
  case RB02_TAB_AAT:
#ifdef RB_ENABLE_UART
    uart_fetch_data();
#endif
    RB02_AdvancedAttitude_Tick(&advancedAttitude_Status, &singletonConfig()->NMEA_DATA, Altimeter, QNH, Variometer);
    {
      uint8_t isAttitudeDoNotNeedGPS = 1;
#ifdef RB_ENABLE_GPS
      isAttitudeDoNotNeedGPS = Operative_GPS;
#endif

      if (Operative_BMP280 && Operative_Attitude && isAttitudeDoNotNeedGPS && OperativeWarningVisible == true)
      {
        lv_obj_add_flag(OperativeWarning, LV_OBJ_FLAG_HIDDEN);
        OperativeWarningVisible = false;
      }
      else
      {
        if ((Operative_Attitude == 0 || Operative_BMP280 == 0 || isAttitudeDoNotNeedGPS == 0) && OperativeWarningVisible == false)
        {
          lv_obj_clear_flag(OperativeWarning, LV_OBJ_FLAG_HIDDEN);
          OperativeWarningVisible = true;
        }
        else
        {
        }
      }
    }
    break;
#endif

#ifdef RB_ENABLE_TRAFFIC
  case RB02_TAB_RDR:
    RB05_Radar_Tick(&(singletonConfig()->trafficStatus));
    if (singletonConfig()->Operative_Bluetooth && OperativeWarningVisible == true)
    {
      lv_obj_add_flag(OperativeWarning, LV_OBJ_FLAG_HIDDEN);
      OperativeWarningVisible = false;
    }
    else
    {
      if (singletonConfig()->Operative_Bluetooth == 0 && OperativeWarningVisible == false)
      {
        lv_obj_clear_flag(OperativeWarning, LV_OBJ_FLAG_HIDDEN);
        OperativeWarningVisible = true;
      }
      else
      {
      }
    }
    break;
  case RB02_TAB_TRA:
    RB05_Traffic_Tick(&(singletonConfig()->trafficStatus));
    if (singletonConfig()->Operative_Bluetooth && OperativeWarningVisible == true)
    {
      lv_obj_add_flag(OperativeWarning, LV_OBJ_FLAG_HIDDEN);
      OperativeWarningVisible = false;
    }
    else
    {
      if (singletonConfig()->Operative_Bluetooth == 0 && OperativeWarningVisible == false)
      {
        lv_obj_clear_flag(OperativeWarning, LV_OBJ_FLAG_HIDDEN);
        OperativeWarningVisible = true;
      }
      else
      {
      }
    }
    break;
#endif

#ifdef RB_ENABLE_SPD
  case RB02_TAB_SPD:
#ifdef RB_ENABLE_UART
    uart_fetch_data();
#endif
    update_Speed_lvgl_tick(t);
    if (Operative_GPS && OperativeWarningVisible == true)
    {
      lv_obj_add_flag(OperativeWarning, LV_OBJ_FLAG_HIDDEN);
      OperativeWarningVisible = false;
    }
    else
    {
      if (Operative_GPS == 0 && OperativeWarningVisible == false)
      {
        lv_obj_clear_flag(OperativeWarning, LV_OBJ_FLAG_HIDDEN);
        OperativeWarningVisible = true;
      }
      else
      {
      }
    }
    break;
#endif
#ifdef RB_ENABLE_EMS
  case RB02_TAB_EMS:
    if (OperativeWarningVisible == true)
    {
      lv_obj_add_flag(OperativeWarning, LV_OBJ_FLAG_HIDDEN);
      OperativeWarningVisible = false;
    }
    RB04_EMS_Tick(singletonConfig()->ui.ems);
    break;
#endif
#ifdef RB_ENABLE_MAP
  case RB02_TAB_MAP:
#ifdef RB_ENABLE_UART
    uart_fetch_data();
#endif
    RB02_GPSMap_Tick(&singletonConfig()->gpsMapStatus, &singletonConfig()->NMEA_DATA, t0);
    if (Operative_GPS && OperativeWarningVisible == true)
    {
      lv_obj_add_flag(OperativeWarning, LV_OBJ_FLAG_HIDDEN);
      OperativeWarningVisible = false;
    }
    else
    {
      if (Operative_GPS == 0 && OperativeWarningVisible == false)
      {
        lv_obj_clear_flag(OperativeWarning, LV_OBJ_FLAG_HIDDEN);
        OperativeWarningVisible = true;
      }
      else
      {
      }
    }
    break;
#endif
#ifdef RB_ENABLE_TRK
  case RB02_TAB_TRK:
#ifdef RB_ENABLE_UART
    uart_fetch_data();
#endif
    bool GyroWillUseTheGPS = false;

    if (Operative_GPS == true)
    {
      if (singletonConfig()->NMEA_DATA.speed > 5)
      {
        GyroWillUseTheGPS = true;
      }
    }
    if (GyroWillUseTheGPS == true)
    {
      // Gyroscope alignment managed by the new algorithm
      singletonConfig()->ui.Gyro.AttitudeYawCorrection = (singletonConfig()->NMEA_DATA.cog - (-AttitudeYaw));
      lv_label_set_text(singletonConfig()->ui.Gyro.Screen_Track_TrackSource, "GPS TRACK");
    }
    else
    {
      lv_label_set_text(singletonConfig()->ui.Gyro.Screen_Track_TrackSource, "GYRO");
    }
    RB02_Gyro_Tick(&(singletonConfig()->ui.Gyro));
#ifdef RB_ENABLE_GPS
    if (Operative_GPS && OperativeWarningVisible == true)
    {
      lv_obj_add_flag(OperativeWarning, LV_OBJ_FLAG_HIDDEN);
      OperativeWarningVisible = false;
    }
    else
    {
      if (Operative_GPS == 0 && OperativeWarningVisible == false)
      {
        lv_obj_clear_flag(OperativeWarning, LV_OBJ_FLAG_HIDDEN);
        OperativeWarningVisible = true;
      }
      else
      {
      }
    }
#endif
    break;
#endif
  case RB02_TAB_DEV:
    break;
#ifdef RB_ENABLE_VAR
  case RB02_TAB_VAR:
    update_Variometer_lvgl_tick(t);
    if (Operative_BMP280 && OperativeWarningVisible == true)
    {
      lv_obj_add_flag(OperativeWarning, LV_OBJ_FLAG_HIDDEN);
      OperativeWarningVisible = false;
    }
    else
    {
      if (Operative_BMP280 == 0 && OperativeWarningVisible == false)
      {
        lv_obj_clear_flag(OperativeWarning, LV_OBJ_FLAG_HIDDEN);
        OperativeWarningVisible = true;
      }
      else
      {
      }
    }
    break;
#endif
#ifdef RB_ENABLE_ALT
  case RB02_TAB_ALT:
    update_Altimeter_lvgl_tick(t);
    if (Operative_BMP280 && OperativeWarningVisible == true)
    {
      lv_obj_add_flag(OperativeWarning, LV_OBJ_FLAG_HIDDEN);
      OperativeWarningVisible = false;
    }
    else
    {
      if (Operative_BMP280 == 0 && OperativeWarningVisible == false)
      {
        lv_obj_clear_flag(OperativeWarning, LV_OBJ_FLAG_HIDDEN);
        OperativeWarningVisible = true;
      }
      else
      {
      }
    }
    break;
#endif
#ifdef RB_ENABLE_ALD
  case RB02_TAB_ALD:
    // 1.1.17 Add GPS Altimeter
#ifdef RB_ENABLE_UART
    uart_fetch_data();
#endif
    update_AltimeterDigital_lvgl_tick(t);
    if (Operative_BMP280 && OperativeWarningVisible == true)
    {
      lv_obj_add_flag(OperativeWarning, LV_OBJ_FLAG_HIDDEN);
      OperativeWarningVisible = false;
    }
    else
    {
      if (Operative_BMP280 == 0 && OperativeWarningVisible == false)
      {
        lv_obj_clear_flag(OperativeWarning, LV_OBJ_FLAG_HIDDEN);
        OperativeWarningVisible = true;
      }
      else
      {
      }
    }
    break;
#endif
#ifdef RB_ENABLE_ATT
  case RB02_TAB_ATT:
    if (GPSAccelerationForAttitudeCompensationEnabled == true)
    {
#ifdef RB_ENABLE_UART
      uart_fetch_data();
#endif
    }
    update_Attitude_lvgl_tick(t);

    // 1.1.16 The Attitude is assisted by GPS
    uint8_t isAttitudeDoNotNeedGPS = 1;
#ifdef RB_ENABLE_GPS
    isAttitudeDoNotNeedGPS = Operative_GPS;
#endif

    if (Operative_Attitude && isAttitudeDoNotNeedGPS && OperativeWarningVisible == true)
    {
      lv_obj_add_flag(OperativeWarning, LV_OBJ_FLAG_HIDDEN);
      OperativeWarningVisible = false;
    }
    else
    {
      if ((Operative_Attitude == 0 || isAttitudeDoNotNeedGPS == 0) && OperativeWarningVisible == false)
      {
        lv_obj_clear_flag(OperativeWarning, LV_OBJ_FLAG_HIDDEN);
        OperativeWarningVisible = true;
      }
      else
      {
      }
    }
    break;
#endif
#ifdef RB_ENABLE_TRN
  case RB02_TAB_TRN:
    update_TurnSlip_lvgl_tick(t);
    if (Operative_Attitude && OperativeWarningVisible == true)
    {
      lv_obj_add_flag(OperativeWarning, LV_OBJ_FLAG_HIDDEN);
      OperativeWarningVisible = false;
    }
    else
    {
      if (Operative_Attitude == 0 && OperativeWarningVisible == false)
      {
        lv_obj_clear_flag(OperativeWarning, LV_OBJ_FLAG_HIDDEN);
        OperativeWarningVisible = true;
      }
      else
      {
      }
    }
    break;
#endif
#ifdef RB_ENABLE_GMT
  case RB02_TAB_GMT:
    update_GMeter_lvgl_tick(t);
    if (Operative_Attitude && OperativeWarningVisible == true)
    {
      lv_obj_add_flag(OperativeWarning, LV_OBJ_FLAG_HIDDEN);
      OperativeWarningVisible = false;
    }
    else
    {
      if (Operative_Attitude == 0 && OperativeWarningVisible == false)
      {
        lv_obj_clear_flag(OperativeWarning, LV_OBJ_FLAG_HIDDEN);
        OperativeWarningVisible = true;
      }
      else
      {
      }
    }
    break;
#endif
#ifdef RB_ENABLE_GPS_DIAG
  case RB02_TAB_GDG:
#ifdef RB_ENABLE_UART
    uart_fetch_data();
#endif
    break;
#endif

#ifdef VIBRATION_TEST
  case RB02_TAB_VBR:
#ifdef RB_ENABLE_UART
    uart_fetch_data();
#endif

    update_Vibration_lvgl_tick(t);
    break;
#endif
#ifdef RB_ENABLE_CLK
  case RB02_TAB_CLK:
#ifdef RB_ENABLE_UART
    uart_fetch_data();
#endif
    update_Clock_lvgl_tick(t);
    break;
#endif

  case RB02_TAB_SET:
  {
    if (OperativeWarningVisible == true)
    {
      lv_obj_add_flag(OperativeWarning, LV_OBJ_FLAG_HIDDEN);
      OperativeWarningVisible = false;
    }
    char buf[4 + 4 + 4 + 4 + 4 + 4 + 22];
    sprintf(buf, "AX:%2.1f AY:%2.1f AZ:%2.1f GX:%2.1f GY:%2.1f GZ:%2.1f",
            Accel.x,
            Accel.y,
            Accel.z,
            Gyro.x,
            Gyro.y,
            Gyro.z);
    lv_label_set_text(SettingStatus0, buf);
    sprintf(buf, "FX:%2.1f FY:%2.1f FZ:%2.1f RX:%2.1f RY:%2.1f RZ:%2.1f",
            AccelFiltered.x,
            AccelFiltered.y,
            AccelFiltered.z,
            GyroFiltered.x + GyroBias.x,
            GyroFiltered.y + GyroBias.y,
            GyroFiltered.z + GyroBias.z);
    lv_label_set_text(SettingStatus1, buf);

    sprintf(buf, "R:%2.1f P:%2.1f T:%2.1f",
            AttitudeRoll,
            AttitudePitch,
            AttitudeYaw);
    lv_label_set_text(singletonConfig()->ui.SettingStatus2, buf);
    lv_label_set_text(singletonConfig()->ui.panelMountAlignmentLabelHelper, buf);

    sprintf(buf, "CX:%2.1f CY:%2.1f CZ:%2.1f BX:%2.1f BY:%2.1f BZ:%2.1f",
            singletonConfig()->GyroHardwareCalibration.x,
            singletonConfig()->GyroHardwareCalibration.y,
            singletonConfig()->GyroHardwareCalibration.z,
            GyroBias.x,
            GyroBias.y,
            GyroBias.z);
    lv_label_set_text(SettingStatus5, buf);

    sprintf(buf, "%.1f°C %.2fhPa",
            bmp280Temperature / 100.0,
            bmp280Pressure / 100.0);
    lv_label_set_text(SettingStatus3, buf);

    // 1.1.4
    sprintf(buf, "%.1f Volts", BAT_analogVolts);
    lv_label_set_text(SettingStatus4, buf);

    // 1.1.17
    snprintf(buf, sizeof(buf), "Engine MDHms: %02d/%02d %02d:%02d:%02d", datetime.month, datetime.day, datetime.hour, datetime.minute, datetime.second);
    lv_label_set_text(SettingsEngineTimeLabel, buf);
    int opBt = 0;
#ifdef RB02_ESP_BLUETOOTH
    opBt = singletonConfig()->Operative_Bluetooth;
#endif
    snprintf(buf, sizeof(buf), "Operative BMP:%d GPS:%d ATT:%d BT:%d", Operative_BMP280, Operative_GPS, Operative_Attitude, opBt);
    lv_label_set_text(singletonConfig()->ui.SettingsOperativeSummary, buf);

#ifdef RB_ENABLE_UART
    uart_fetch_data();
#endif
  }
  break;
  }
}

static void mbox1_timer_reset(lv_event_t *e)
{

  lv_event_code_t code = lv_event_get_code(e);
  lv_obj_t *msgbox = lv_event_get_current_target(e);

  if (code == LV_EVENT_VALUE_CHANGED)
  {
    const char *txt = lv_msgbox_get_active_btn_text(msgbox);
    if (txt)
    {
      if (strcmp("RESTART", txt) == 0)
      {
        switch (selectedTimer)
        {
        case 0:
          datetimeTimer1 = datetime;
          break;
        case 1:
          datetimeTimer2 = datetime;
          break;
        case 2:
          datetimeTimer3 = datetime;
          break;
        }
        update_Clock_lvgl_tick(NULL);
      }
      lv_msgbox_close(msgbox);
    }
  }

  if (code == LV_EVENT_DELETE)
  {
    mbox1 = NULL;
  }
}

static void mbox1_event_cb(lv_event_t *e)
{

  lv_event_code_t code = lv_event_get_code(e);
  lv_obj_t *msgbox = lv_event_get_current_target(e);

  if (code == LV_EVENT_VALUE_CHANGED)
  {
    const char *txt = lv_msgbox_get_active_btn_text(msgbox);
    if (txt)
    {
      if (strcmp("RESET", txt) == 0)
      {
        GFactorMax = 0;
        GFactorMin = 1.0;
        GFactorDirty = 1;

#ifdef VIBRATION_TEST
        GMeterScaleGyro = 15;
#endif
      }
      lv_msgbox_close(msgbox);
    }
  }

  if (code == LV_EVENT_DELETE)
  {
    mbox1 = NULL;
  }
}

static void mbox1_cage_event_cb(lv_event_t *e)
{

  lv_event_code_t code = lv_event_get_code(e);
  lv_obj_t *msgbox = lv_event_get_current_target(e);

  if (code == LV_EVENT_VALUE_CHANGED)
  {
    const char *txt = lv_msgbox_get_active_btn_text(msgbox);
    if (txt)
    {
#ifdef RB_ENABLE_GPS
      if (strcmp("QNH", txt) == 0)
      {
        QNH = RB02_SuggestedQNH(singletonConfig()->NMEA_DATA.altitude, bmp280Pressure);
        RB02_AltimeterQNHUpdated();
        singletonConfig()->settingsAutoQNH = 1;
      }
#endif
      if (strcmp("CAGE", txt) == 0)
      {
        // TODO: make an avg
        GyroBias.x = -GyroFiltered.x;
        GyroBias.y = -GyroFiltered.y;
        GyroBias.z = -GyroFiltered.z;

        rb_check_attitude_inop();
      }
      lv_msgbox_close(msgbox);
    }
  }

  if (code == LV_EVENT_DELETE)
  {
    mbox1 = NULL;
  }
}

void lv_att_reset_msgbox(void)
{
  if (mbox1 != NULL)
    return;
  static const char *btns[] = {"QNH", "CAGE", "CANCEL", ""};

  char buf[100];
  uint16_t SuggestedQNH = 1013;
#ifdef RB_ENABLE_GPS
  SuggestedQNH = RB02_SuggestedQNH(singletonConfig()->NMEA_DATA.altitude, bmp280Pressure);
#endif
  snprintf(buf, sizeof(buf), "Would you like to cage sensors?\nAuto QNH=%d", SuggestedQNH);

  mbox1 = lv_msgbox_create(NULL, "Attitude", buf, btns, true);
  lv_obj_set_style_text_font(mbox1, &lv_font_montserrat_16, 0);
  lv_obj_add_event_cb(mbox1, mbox1_cage_event_cb, LV_EVENT_ALL, mbox1);
  lv_obj_center(mbox1);
}

void lv_gmeter_reset_msgbox(void)
{
  if (mbox1 != NULL)
    return;
  static const char *btns[] = {"RESET", "CANCEL", ""};

  mbox1 = lv_msgbox_create(NULL, "G-Meter", "Would you like to reset G max-neg?", btns, true);
  lv_obj_set_style_text_font(mbox1, &lv_font_montserrat_16, 0);
  lv_obj_add_event_cb(mbox1, mbox1_event_cb, LV_EVENT_ALL, mbox1);
  lv_obj_center(mbox1);
}

// 1.1.3 Popup for timer restart
void lv_timer_restart_msgbox(void)
{
  if (mbox1 != NULL)
    return;
  static const char *btns[] = {"RESTART", "CANCEL", ""};

  mbox1 = lv_msgbox_create(NULL, "Timers", "Would you like to restart the timer?", btns, true);
  lv_obj_set_style_text_font(mbox1, &lv_font_montserrat_16, 0);
  lv_obj_add_event_cb(mbox1, mbox1_timer_reset, LV_EVENT_ALL, mbox1);
  lv_obj_center(mbox1);
}

static void RB02_AltimeterQNHUpdated()
{
  char buf[25];
  snprintf(buf, sizeof(buf), "%03u", QNH);
#ifdef RB_ENABLE_ALT
  lv_label_set_text(Screen_Altitude_QNH, buf);
#endif
  // 1.1.2 added mmHg conversion
  snprintf(buf, sizeof(buf), "QNH: %u %.02f", QNH, ((float)QNH) / 33.8639);
#ifdef RB_ENABLE_ALD
  lv_label_set_text(Screen_Altitude_QNH2, buf);
  snprintf(buf, sizeof(buf), "%+ld", Variometer);
  lv_label_set_text(Screen_Altitude_Variometer2, buf);
#endif
  example1_BMP280_lvgl_tick(NULL);
  // [ISSUE] with variometer on digital altimeter page #2
  Variometer = 0;
}

static void actionInTab(touchLocation location)
{

  // 1.1.25 Request to go to the default screen
  switch (location)
  {
  case RB02_TOUCH_NW:
#ifdef RB_ENABLE_AAT
    lv_tabview_set_act(tv, RB02_TAB_AAT, LV_ANIM_OFF);
#else
#ifdef RB_ENABLE_ATT
    lv_tabview_set_act(tv, RB02_TAB_ATT, LV_ANIM_OFF);
#else
    lv_tabview_set_act(tv, 0, LV_ANIM_OFF);
#endif
#endif
    break;
  default:
    break;
  }

  switch (((lv_tabview_t *)tv)->tab_cur)
  {
#ifdef RB_ENABLE_TRK
  case RB02_TAB_TRK:
    switch (location)
    {
    case RB02_TOUCH_CENTER:
      lv_att_reset_msgbox();
      break;
    case RB02_TOUCH_N:
      singletonConfig()->ui.Gyro.AttitudeYawCorrection = singletonConfig()->ui.Gyro.AttitudeYawCorrection + 10;
      break;
    case RB02_TOUCH_S:
      singletonConfig()->ui.Gyro.AttitudeYawCorrection = singletonConfig()->ui.Gyro.AttitudeYawCorrection - 10;
      break;
    default:
      break;
    }
    RB02_Gyro_Tick(&(singletonConfig()->ui.Gyro));
    break;
#endif
#ifdef RB_ENABLE_TRAFFIC
  case RB02_TAB_TRA:
    switch (location)
    {
    case RB02_TOUCH_N:
      RB05_Traffic_Touch_N(&singletonConfig()->trafficStatus);
      break;
    case RB02_TOUCH_S:
      RB05_Traffic_Touch_S(&singletonConfig()->trafficStatus);
      break;
    case RB02_TOUCH_CENTER:
      RB05_Traffic_Touch_Center(&singletonConfig()->trafficStatus);
      break;
    default:
      break;
    }
    break;
  case RB02_TAB_RDR:
    switch (location)
    {
    case RB02_TOUCH_CENTER:
      RB05_Radar_Touch_Center(&singletonConfig()->trafficStatus);
      break;
    case RB02_TOUCH_N:
      RB05_Radar_Touch_N(&singletonConfig()->trafficStatus);
      break;
    case RB02_TOUCH_S:
      RB05_Radar_Touch_S(&singletonConfig()->trafficStatus);
      break;
    default:
      break;
    }
    break;
#endif
#ifdef RB_ENABLE_EMS
  case RB02_TAB_EMS:
    switch (location)
    {
    case RB02_TOUCH_N:
      RB04_EMS_Touch_N(singletonConfig()->ui.ems);
      break;
    case RB02_TOUCH_S:
      RB04_EMS_Touch_S(singletonConfig()->ui.ems);
      break;
    case RB02_TOUCH_E:
      RB04_EMS_Touch_E(singletonConfig()->ui.ems);
      break;
    case RB02_TOUCH_W:
      RB04_EMS_Touch_W(singletonConfig()->ui.ems);
      break;

    default:
      break;
    }
    break;
#endif
#ifdef RB_ENABLE_MAP
  case RB02_TAB_MAP:
    switch (location)
    {
    case RB02_TOUCH_N:
      RB02_GPSMap_Touch_N(&singletonConfig()->gpsMapStatus);
      break;
    case RB02_TOUCH_S:
      RB02_GPSMap_Touch_S(&singletonConfig()->gpsMapStatus);
      break;
    case RB02_TOUCH_E:
      RB02_GPSMap_Touch_E(&singletonConfig()->gpsMapStatus);
      break;
    case RB02_TOUCH_W:
      RB02_GPSMap_Touch_W(&singletonConfig()->gpsMapStatus);
      break;

    default:
      break;
    }
    break;
#endif
#ifdef RB_ENABLE_AAT
  case RB02_TAB_AAT:
#endif
#ifdef RB_ENABLE_ALT
  case RB02_TAB_ALT:
#endif
#ifdef RB_ENABLE_ALD
  case RB02_TAB_ALD:
#endif
    switch (location)
    {
    case RB02_TOUCH_N:
      QNH++;
      singletonConfig()->settingsAutoQNH = 0;
      break;
    case RB02_TOUCH_S:
      QNH--;
      singletonConfig()->settingsAutoQNH = 0;
      break;
    case RB02_TOUCH_CENTER:
      lv_att_reset_msgbox();
      break;
    default:
      break;
    }
    RB02_AltimeterQNHUpdated();
    break;
#ifdef RB_ENABLE_CLK
  case RB02_TAB_CLK:
    switch (location)
    {
    case RB02_TOUCH_SE:
    case RB02_TOUCH_SW:
    case RB02_TOUCH_N:
    {
      selectedTimer++;
      if (selectedTimer > 2)
      {
        selectedTimer = 0;
      }
      char buf[12];
      sprintf(buf, "TIMER %u", selectedTimer + 1);
      lv_label_set_text(TimerLabelTop, buf);
      update_Clock_lvgl_tick(NULL);

      switch (selectedTimer)
      {
      case 0:
        sprintf(buf, "TIMER %u", 2);
        lv_label_set_text(TimerLabelSW, buf);
        sprintf(buf, "TIMER %u", 3);
        lv_label_set_text(TimerLabelSE, buf);
        break;
      case 1:
        sprintf(buf, "TIMER %u", 1);
        lv_label_set_text(TimerLabelSW, buf);
        sprintf(buf, "TIMER %u", 3);
        lv_label_set_text(TimerLabelSE, buf);
        break;
      case 2:
        sprintf(buf, "TIMER %u", 1);
        lv_label_set_text(TimerLabelSW, buf);
        sprintf(buf, "TIMER %u", 2);
        lv_label_set_text(TimerLabelSE, buf);
        break;
      }
// 1.1.17 Display UTC Clock on the Timer 3 slot
#ifdef RB_ENABLE_GPS
      sprintf(buf, "GPS UTC+0");
      lv_label_set_text(TimerLabelSE, buf);
#endif
    }
    break;
    case RB02_TOUCH_S:
      break;
    case RB02_TOUCH_CENTER:
      // 1.1.3 Restart timer moved to menu
      lv_timer_restart_msgbox();
      break;
    default:
      break;
    }
    break;
#endif
#ifdef RB_ENABLE_GMT
  case RB02_TAB_GMT:
    switch (location)
    {
    case RB02_TOUCH_N:
    case RB02_TOUCH_S:
    case RB02_TOUCH_CENTER:
      lv_gmeter_reset_msgbox();
      break;
    default:
      break;
    }
    break;
#endif
#ifdef RB_ENABLE_VBR
  case RB02_TAB_VBR:
#endif
#ifdef RB_ENABLE_TRN
  case RB02_TAB_TRN:
#endif
#ifdef RB_ENABLE_ATT
  case RB02_TAB_ATT:
    switch (location)
    {
    case RB02_TOUCH_CENTER:
      lv_att_reset_msgbox();
      break;
    default:
      break;
    }
    break;
#endif
  default:
    break;
  }
}

static void speedBgClicked(lv_event_t *event)
{
  if (mbox1 != NULL)
  {
    return;
  }

  bool changedTab = false;
  touchLocation location = getTouchLocation(TouchPadLastX, TouchPadLastY);
#ifdef RB_ENABLE_CONSOLE_DEBUG
  printf("Clicked at %ux%u => %u\n", TouchPadLastX, TouchPadLastY, location);
#endif
  lv_tabview_t *tabview = (lv_tabview_t *)tv;
  uint16_t cur = tabview->tab_cur;

  switch (location)
  {
  case RB02_TOUCH_W:
    if (cur > 0)
    {
      cur--;
      changedTab = true;
    }
    break;
  case RB02_TOUCH_E:
    cur++;
    changedTab = true;
    if (cur >= RB02_TAB_DEV)
    {
      cur = RB02_TAB_SET;
    }
    break;
  default:
    actionInTab(location);
    break;
  }

  // Skip setup page
#ifdef VIBRATION_TEST
#else
#endif

  if (changedTab == true)
  {
    lv_tabview_set_act(tv, cur, LV_ANIM_OFF);
  }
  if (DeviceIsDemoMode == 0 && changedTab)
  {
    nvsStoreDefaultScreenOrDemo();
  }
}

#ifdef RB_ENABLE_NavCore
void RB02_NavCore_InjectTouch(uint8_t touch_loc)
{
  // Same protections as speedBgClicked
  if (mbox1 != NULL)
    return;

  if (tv == NULL)
    return;

  bool changedTab = false;
  lv_tabview_t *tabview = (lv_tabview_t *)tv;
  uint16_t cur = tabview->tab_cur;

  // Reproduce the "global" W/E tab switching behavior from speedBgClicked()
  switch (touch_loc)
  {
    case RB02_TOUCHLOC_W:
      if (cur > 0)
      {
        cur--;
        changedTab = true;
      }
      break;

    case RB02_TOUCHLOC_E:
      cur++;
      changedTab = true;
      if (cur >= RB02_TAB_DEV)
      {
        cur = RB02_TAB_SET;
      }
      break;

    default:
      // For N/S/CENTER etc, reuse RB02 dispatching
      actionInTab((touchLocation)touch_loc);
      break;
  }

  if (changedTab)
  {
    lv_tabview_set_act(tv, cur, LV_ANIM_OFF);

    if (DeviceIsDemoMode == 0)
    {
      nvsStoreDefaultScreenOrDemo();
    }
  }
}
#endif


static lv_obj_t *Onboard_create_Base(lv_obj_t *parent, const lv_img_dsc_t *backgroundImageName)
{

  lv_obj_t *backgroundImage = lv_img_create(parent);
  lv_img_set_src(backgroundImage, backgroundImageName);
  lv_obj_set_size(backgroundImage, backgroundImageName->header.w, backgroundImageName->header.h);
  // TODO: Migrate to SizeContent
  // lv_obj_set_size(backgroundImage,LV_SIZE_CONTENT,LV_SIZE_CONTENT);
  lv_obj_align(backgroundImage, LV_ALIGN_CENTER, 0, 0);
  lv_obj_set_scrollbar_mode(backgroundImage, LV_SCROLLBAR_MODE_OFF);
  lv_obj_set_scrollbar_mode(parent, LV_SCROLLBAR_MODE_OFF);
  // 1.1.9 Remove scrolling for Turbolence touch screen
  lv_obj_clear_flag(parent, LV_OBJ_FLAG_SCROLLABLE);
  return backgroundImage;
}

static void ChangeAttitudeBalanceAlphaChanged(lv_event_t *e)
{
  uint8_t FilterMoltiplierInt = lv_slider_get_value(lv_event_get_target(e));
#ifdef RB_ENABLE_CONSOLE_DEBUG
  printf("Changed Filtering ratio from: %.2f to: %d\n", AttitudeBalanceAlpha, FilterMoltiplierInt);
#endif
  float f = FilterMoltiplierInt;
  AttitudeBalanceAlpha = f / 250.0;
  char buf[23 + 8];
  sprintf(buf, "Attitude compensation: %.0f", AttitudeBalanceAlpha * 250.0);
  lv_label_set_text(SettingAttitudeCompensation, buf);

  nvsStoreFilters();
}

static void AutoQNHModeChanged(lv_event_t *e)
{
  lv_obj_t *sw = (lv_obj_t *)lv_event_get_user_data(e);
  if (lv_obj_get_state(sw) & LV_STATE_CHECKED)
  {
    singletonConfig()->settingsAutoQNH = 1;
  }
  else
  {
    singletonConfig()->settingsAutoQNH = 0;
  }
  nvsStorePCal();
}
static void DeviceIsDemoModeChanged(lv_event_t *e)
{
  lv_obj_t *sw = (lv_obj_t *)lv_event_get_user_data(e);
  if (lv_obj_get_state(sw) & LV_STATE_CHECKED)
  {
    DeviceIsDemoMode = 1;
    nvsStoreDefaultScreenOrDemo();
  }
  else
  {
    DeviceIsDemoMode = 0;
  }
}
static void GPSAccelerationForAttitudeCompensationEnabledChanged(lv_event_t *e)
{
  lv_obj_t *sw = (lv_obj_t *)lv_event_get_user_data(e);
  if (lv_obj_get_state(sw) & LV_STATE_CHECKED)
  {
    GPSAccelerationForAttitudeCompensationEnabled = true;
  }
  else
  {
    GPSAccelerationForAttitudeCompensationEnabled = false;
  }
}

static void AttitudeMadwickChanged(lv_event_t *e)
{
  lv_obj_t *sw = (lv_obj_t *)lv_event_get_user_data(e);
  if (lv_obj_get_state(sw) & LV_STATE_CHECKED)
  {
    EnableAttitudeMadgwick = 1;
  }
  else
  {
    EnableAttitudeMadgwick = 0;
  }
}

static void AltimeterOverrideChanged(lv_event_t *e)
{
  int32_t FilterMoltiplierInt = lv_slider_get_value(lv_event_get_target(e));
#ifdef RB_ENABLE_CONSOLE_DEBUG
  printf("Changed BMP280 Override from: %ld to: %ld\n", singletonConfig()->bmp280override, FilterMoltiplierInt);
#endif
  singletonConfig()->bmp280override = FilterMoltiplierInt - 500;

  example1_BMP280_lvgl_tick(NULL);

  char buf[50];
  sprintf(buf, "Altimeter QNH: %d mmHg %ld feet (%ld)", QNH, Altimeter, singletonConfig()->bmp280override);
  lv_label_set_text(bmp280overrideLabel, buf);

  // Store
  nvsStorePCal();
}
#ifdef RB_ENABLE_SPD
static void SpeedDegreeStartChanged(lv_event_t *e)
{
  uint16_t new_Start = 15 * lv_slider_get_value(t_speedStart);
  uint16_t new_End = 15 * lv_slider_get_value(t_speedEnd);
  uint16_t new_StartSpeed = 10 * lv_slider_get_value(t_speedStartSpeed);
  uint16_t new_EndSpeed = 10 * lv_slider_get_value(t_speedEndSpeed);
  uint16_t new_White = 15 * lv_slider_get_value(t_speedWhite);
  uint16_t new_Green = 15 * lv_slider_get_value(t_speedGreen);
  uint16_t new_Yellow = 15 * lv_slider_get_value(t_speedYellow);
  uint16_t new_Red = 15 * lv_slider_get_value(t_speedRed);

  degreeStart = new_Start;
  degreeEnd = new_End;
  speedKtStart = new_StartSpeed;
  speedKtEnd = new_EndSpeed;
  speedWhite = new_White;
  speedGreen = new_Green;
  speedYellow = new_Yellow;
  speedRed = new_Red;

  char buf[60];
  snprintf(buf, 60, "%u->%u %u->%u W:%u G:%u Y:%u R:%u",
           degreeStart,
           degreeEnd,
           speedKtStart,
           speedKtEnd,
           speedWhite,
           speedGreen,
           speedYellow,
           speedRed);
  lv_label_set_text(t_speedSummary, buf);

  nvsStoreSpeedArc();
}
#endif

static void DisableFilteringChanged(lv_event_t *e)
{
  uint8_t FilterMoltiplierInt = lv_slider_get_value(lv_event_get_target(e));
#ifdef RB_ENABLE_CONSOLE_DEBUG
  printf("Changed Filtering ratio from: %.1f to: %d\n", FilterMoltiplier, FilterMoltiplierInt);
#endif
  FilterMoltiplier = FilterMoltiplierInt;

  char buf[16 + 8];
  sprintf(buf, "Sensor Acc: %.0f", FilterMoltiplier);
  lv_label_set_text(SettingLabelFilter, buf);

  nvsStoreFilters();
}
static void DisableFilteringOutputChanged(lv_event_t *e)
{
  uint8_t FilterMoltiplierInt = lv_slider_get_value(lv_event_get_target(e));
#ifdef RB_ENABLE_CONSOLE_DEBUG
  printf("Changed Filtering ratio from: %.1f to: %d\n", FilterMoltiplierOutput, FilterMoltiplierInt);
#endif
  FilterMoltiplierOutput = FilterMoltiplierInt;

  char buf[16 + 8];
  sprintf(buf, "Sensor Out: %.0f", FilterMoltiplierOutput);
  lv_label_set_text(SettingLabelFilterOutput, buf);

  nvsStoreFilters();
}
static void DisableFilteringGyroChanged(lv_event_t *e)
{
  uint8_t FilterMoltiplierInt = lv_slider_get_value(lv_event_get_target(e));
#ifdef RB_ENABLE_CONSOLE_DEBUG
  printf("Changed Filtering ratio from: %.1f to: %d\n", FilterMoltiplier, FilterMoltiplierInt);
#endif
  FilterMoltiplierGyro = FilterMoltiplierInt;

  char buf[16 + 8];
  sprintf(buf, "Sensor Gyro: %.0f", FilterMoltiplierGyro);
  lv_label_set_text(SettingLabelFilterGyro, buf);

  nvsStoreFilters();
}
static void DriverLoopMillisecondsChanged(lv_event_t *e)
{
  uint8_t FilterMoltiplierInt = lv_slider_get_value(lv_event_get_target(e));
#ifdef RB_ENABLE_CONSOLE_DEBUG
  printf("Changed Sensor Hz ratio from: %d to: %d\n", DriverLoopMilliseconds, FilterMoltiplierInt);
#endif
  DriverLoopMilliseconds = FilterMoltiplierInt;

  char buf[16 + 8];
  sprintf(buf, "Sensor Hz: %.0f", 1000.0 / (10 + DriverLoopMilliseconds));
  lv_label_set_text(SettingLabelDriverLoopMilliseconds, buf);

  nvsStoreFilters();
}

static void Backlight_adjustment_event_Changed(lv_event_t *e)
{
  uint8_t Backlight = lv_slider_get_value(lv_event_get_target(e));
#ifdef RB_ENABLE_CONSOLE_DEBUG
  printf("Changed Backlight ratio from: %d to: %d\n", LCD_Backlight, Backlight);
#endif
  Set_Backlight(Backlight);
}
static void GMeterMaxChanged(lv_event_t *e)
{
  uint8_t value = lv_slider_get_value(lv_event_get_target(e));
#ifdef RB_ENABLE_CONSOLE_DEBUG
  printf("Changed GMeter from: %.1f to: %d\n", GMeterScale, value);
#endif
  GMeterScale = value;
}

// 1.1.6
static void event_handler_reset_to_default(lv_event_t *e)
{
  lv_event_code_t code = lv_event_get_code(e);
  if (code == LV_EVENT_CLICKED)
  {
    VendorMakeDefaults();
  }
}
static void event_handler_power_off(lv_event_t *e)
{
  lv_event_code_t code = lv_event_get_code(e);
  if (code == LV_EVENT_CLICKED)
  {
    esp_deep_sleep_try_to_start();
  }
}

static void event_handler_gyro_calibrate(lv_event_t *e)
{
  lv_event_code_t code = lv_event_get_code(e);
  if (code == LV_EVENT_CLICKED)
  {

    // 1.1.30 we switched from software calibration to hardware calibration
    singletonConfig()->GyroHardwareCalibration.x = GyroFiltered.x;
    singletonConfig()->GyroHardwareCalibration.y = GyroFiltered.y;
    singletonConfig()->GyroHardwareCalibration.z = GyroFiltered.z;
    // 1.1.30
    gyroHardwareSetCalibration(GyroFiltered.x, GyroFiltered.y, GyroFiltered.z);
    GyroBias.x = 0;
    GyroBias.y = 0;
    GyroBias.z = 0;

    nvsStoreGyroCalibration();
  }
}

// 1.1.2
static void event_handler_kmh_menu(lv_event_t *e)
{
  lv_event_code_t code = lv_event_get_code(e);
  lv_obj_t *obj = lv_event_get_target(e);
  if (code == LV_EVENT_VALUE_CHANGED)
  {
    char buf[32];
    lv_dropdown_get_selected_str(obj, buf, sizeof(buf));
    LV_LOG_USER("Option: %s", buf);

    if (strcmp(buf, "Imperial Units") == 0)
      isKmh = 0;
    if (strcmp(buf, "Metric Units") == 0)
      isKmh = 1;

    nvsStoreSpeedArc();
  }
}
#ifdef RB_ENABLE_GPS
static void event_handler_gps_menu(lv_event_t *e)
{
  lv_event_code_t code = lv_event_get_code(e);
  lv_obj_t *obj = lv_event_get_target(e);
  if (code == LV_EVENT_VALUE_CHANGED)
  {
    char buf[32];
    lv_dropdown_get_selected_str(obj, buf, sizeof(buf));
    LV_LOG_USER("Option: %s", buf);

    GpsSpeed0ForDisable = 0;
    if (strcmp(buf, "4800") == 0)
      GpsSpeed0ForDisable = 4800;
    if (strcmp(buf, "9600") == 0)
      GpsSpeed0ForDisable = 9600;
    if (strcmp(buf, "19200") == 0)
      GpsSpeed0ForDisable = 19200;
    if (strcmp(buf, "38400") == 0)
      GpsSpeed0ForDisable = 38400;
    if (strcmp(buf, "115200") == 0)
      GpsSpeed0ForDisable = 115200;

    uartApplyRates();

    nvsStoreUARTBaudrate();
  }
}
#endif
static void Onboard_create_Setup(lv_obj_t *parent)
{
  int lineY = -200;

  // Version
  if (true)
  {
    lv_obj_t *VersionLabel = lv_label_create(parent);
    lv_obj_set_size(VersionLabel, 200, 48);
    lv_obj_align(VersionLabel, LV_ALIGN_CENTER, 0, lineY);
    lv_obj_set_style_text_font(VersionLabel, &lv_font_montserrat_48, 0);
    lv_obj_set_style_text_align(VersionLabel, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_text(VersionLabel, RB_PRODUCT_TITLE);
    lv_obj_add_style(VersionLabel, &style_title, LV_STATE_DEFAULT);
    lineY += 40;
  }
  if (true)
  {
    lv_obj_t *VersionLabel = lv_label_create(parent);
    lv_obj_set_size(VersionLabel, 200, 20);
    lv_obj_align(VersionLabel, LV_ALIGN_CENTER, 0, lineY);
    lv_obj_set_style_text_align(VersionLabel, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_font(VersionLabel, &lv_font_montserrat_16, 0);
    lv_label_set_text(VersionLabel, "Version " RB_VERSION);
    lv_obj_add_style(VersionLabel, &style_title, LV_STATE_DEFAULT);
    lineY += 30;
  }

  if (true)
  {
    lv_obj_t *VersionLabel = lv_label_create(parent);
    lv_obj_set_size(VersionLabel, 200, 20);
    lv_obj_align(VersionLabel, LV_ALIGN_CENTER, 0, lineY);
    lv_obj_set_style_text_align(VersionLabel, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_font(VersionLabel, &lv_font_montserrat_16, 0);
    char hwstring[20];
    snprintf(hwstring, 20, "%llX", _chipmacid);
    lv_label_set_text(VersionLabel, hwstring);
    lv_obj_add_style(VersionLabel, &style_title, LV_STATE_DEFAULT);
    lineY += 30;
  }
  // 1.1.17 Engine Time
  if (true)
  {
    lv_obj_t *VersionLabel = lv_label_create(parent);
    lv_obj_set_size(VersionLabel, 300, 20);
    lv_obj_align(VersionLabel, LV_ALIGN_CENTER, 0, lineY);
    lv_obj_set_style_text_font(VersionLabel, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_align(VersionLabel, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_text(VersionLabel, "Engine Time:");
    lv_obj_add_style(VersionLabel, &style_title, LV_STATE_DEFAULT);
    lineY += 40;

    SettingsEngineTimeLabel = VersionLabel;
  }

  // 1.1.6
  if (true)
  {
    lv_obj_t *btn1 = lv_btn_create(parent);
    lv_obj_add_event_cb(btn1, event_handler_reset_to_default, LV_EVENT_ALL, NULL);
    lv_obj_align(btn1, LV_ALIGN_CENTER, 0, lineY);

    lv_obj_t *label = lv_label_create(btn1);
    lv_label_set_text(label, "RESET SETTINGS");
    lv_obj_center(label);
    lineY += 60;
  }

  if (true)
  {
    lv_obj_t *btn1 = lv_btn_create(parent);
    lv_obj_add_event_cb(btn1, event_handler_power_off, LV_EVENT_ALL, NULL);
    lv_obj_align(btn1, LV_ALIGN_CENTER, 0, lineY);

    lv_obj_t *label = lv_label_create(btn1);
    lv_label_set_text(label, "POWER OFF");
    lv_obj_center(label);
    lineY += 60;
  }

  if (true)
  {
    lv_obj_t *btn1 = lv_btn_create(parent);
    lv_obj_add_event_cb(btn1, event_handler_gyro_calibrate, LV_EVENT_ALL, NULL);
    lv_obj_align(btn1, LV_ALIGN_CENTER, 0, lineY);

    lv_obj_t *label = lv_label_create(btn1);
    lv_label_set_text(label, "GYRO CALIBRATE");
    lv_obj_center(label);
    lineY += 60;
  }

  RB02_Setup_CreateScreen(singletonConfig(), parent, &lineY);

  if (true)
  {
    lv_obj_t *DisableFiltering = lv_slider_create(parent);
    lv_obj_add_flag(DisableFiltering, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_size(DisableFiltering, 300, 35);
    lv_obj_set_style_radius(DisableFiltering, 3, LV_PART_KNOB); // Adjust the value for more or less rounding
    lv_obj_set_style_bg_opa(DisableFiltering, LV_OPA_TRANSP, LV_PART_KNOB);

    lv_obj_set_style_bg_color(DisableFiltering, lv_color_hex(0xAAAAAA), LV_PART_KNOB);
    lv_obj_set_style_bg_color(DisableFiltering, lv_color_hex(0xFFFFFF), LV_PART_INDICATOR);
    lv_obj_set_style_outline_width(DisableFiltering, 2, LV_PART_INDICATOR);
    lv_obj_set_style_outline_color(DisableFiltering, lv_color_hex(0xD3D3D3), LV_PART_INDICATOR);
    lv_slider_set_range(DisableFiltering, 0, 25);
    lv_slider_set_value(DisableFiltering, FilterMoltiplierOutput, LV_ANIM_OFF);
    lv_obj_add_event_cb(DisableFiltering, DisableFilteringOutputChanged, LV_EVENT_VALUE_CHANGED, DisableFiltering);
    lv_obj_align(DisableFiltering, LV_ALIGN_CENTER, 0, lineY + 30);

    lv_obj_t *DisableFilteringLabel = lv_label_create(parent);
    lv_obj_set_size(DisableFilteringLabel, 300, 20);
    lv_obj_align(DisableFilteringLabel, LV_ALIGN_CENTER, 0, lineY);
    lv_obj_set_style_text_align(DisableFilteringLabel, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_font(DisableFilteringLabel, &lv_font_montserrat_16, 0);
    char buf[16 + 8];
    sprintf(buf, "Out filtering: %.0f", FilterMoltiplierOutput);
    lv_label_set_text(DisableFilteringLabel, buf);
    lv_obj_add_style(DisableFilteringLabel, &style_title, LV_STATE_DEFAULT);

    SettingLabelFilterOutput = DisableFilteringLabel;

    lineY += 70;
  }

  if (true)
  {
    lv_obj_t *DisableFiltering = lv_slider_create(parent);
    lv_obj_add_flag(DisableFiltering, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_size(DisableFiltering, 300, 35);
    lv_obj_set_style_radius(DisableFiltering, 3, LV_PART_KNOB); // Adjust the value for more or less rounding
    lv_obj_set_style_bg_opa(DisableFiltering, LV_OPA_TRANSP, LV_PART_KNOB);

    lv_obj_set_style_bg_color(DisableFiltering, lv_color_hex(0xAAAAAA), LV_PART_KNOB);
    lv_obj_set_style_bg_color(DisableFiltering, lv_color_hex(0xFFFFFF), LV_PART_INDICATOR);
    lv_obj_set_style_outline_width(DisableFiltering, 2, LV_PART_INDICATOR);
    lv_obj_set_style_outline_color(DisableFiltering, lv_color_hex(0xD3D3D3), LV_PART_INDICATOR);
    lv_slider_set_range(DisableFiltering, 0, 25);
    lv_slider_set_value(DisableFiltering, FilterMoltiplier, LV_ANIM_OFF);
    lv_obj_add_event_cb(DisableFiltering, DisableFilteringChanged, LV_EVENT_VALUE_CHANGED, DisableFiltering);
    lv_obj_align(DisableFiltering, LV_ALIGN_CENTER, 0, lineY + 30);

    lv_obj_t *DisableFilteringLabel = lv_label_create(parent);
    lv_obj_set_size(DisableFilteringLabel, 300, 20);
    lv_obj_align(DisableFilteringLabel, LV_ALIGN_CENTER, 0, lineY);
    lv_obj_set_style_text_align(DisableFilteringLabel, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_font(DisableFilteringLabel, &lv_font_montserrat_16, 0);
    char buf[16 + 8];
    sprintf(buf, "Acc filtering: %.0f", FilterMoltiplier);
    lv_label_set_text(DisableFilteringLabel, buf);
    lv_obj_add_style(DisableFilteringLabel, &style_title, LV_STATE_DEFAULT);

    SettingLabelFilter = DisableFilteringLabel;

    lineY += 70;
  }

  if (true)
  {
    lv_obj_t *DisableFiltering = lv_slider_create(parent);
    lv_obj_add_flag(DisableFiltering, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_size(DisableFiltering, 300, 35);
    lv_obj_set_style_radius(DisableFiltering, 3, LV_PART_KNOB); // Adjust the value for more or less rounding
    lv_obj_set_style_bg_opa(DisableFiltering, LV_OPA_TRANSP, LV_PART_KNOB);

    lv_obj_set_style_bg_color(DisableFiltering, lv_color_hex(0xAAAAAA), LV_PART_KNOB);
    lv_obj_set_style_bg_color(DisableFiltering, lv_color_hex(0xFFFFFF), LV_PART_INDICATOR);
    lv_obj_set_style_outline_width(DisableFiltering, 2, LV_PART_INDICATOR);
    lv_obj_set_style_outline_color(DisableFiltering, lv_color_hex(0xD3D3D3), LV_PART_INDICATOR);
    lv_slider_set_range(DisableFiltering, 0, 25);
    lv_slider_set_value(DisableFiltering, FilterMoltiplierGyro, LV_ANIM_OFF);
    lv_obj_add_event_cb(DisableFiltering, DisableFilteringGyroChanged, LV_EVENT_VALUE_CHANGED, DisableFiltering);
    lv_obj_align(DisableFiltering, LV_ALIGN_CENTER, 0, lineY + 30);

    lv_obj_t *DisableFilteringLabel = lv_label_create(parent);
    lv_obj_set_size(DisableFilteringLabel, 300, 20);
    lv_obj_align(DisableFilteringLabel, LV_ALIGN_CENTER, 0, lineY);
    lv_obj_set_style_text_align(DisableFilteringLabel, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_font(DisableFilteringLabel, &lv_font_montserrat_16, 0);
    char buf[16 + 8];
    sprintf(buf, "Gyro filtering: %.0f", FilterMoltiplierGyro);
    lv_label_set_text(DisableFilteringLabel, buf);
    lv_obj_add_style(DisableFilteringLabel, &style_title, LV_STATE_DEFAULT);

    SettingLabelFilterGyro = DisableFilteringLabel;

    lineY += 70;
  }

  if (true)
  {
    lv_obj_t *DisableFiltering = lv_slider_create(parent);
    lv_obj_add_flag(DisableFiltering, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_size(DisableFiltering, 300, 35);
    lv_obj_set_style_radius(DisableFiltering, 3, LV_PART_KNOB); // Adjust the value for more or less rounding
    lv_obj_set_style_bg_opa(DisableFiltering, LV_OPA_TRANSP, LV_PART_KNOB);

    lv_obj_set_style_bg_color(DisableFiltering, lv_color_hex(0xAAAAAA), LV_PART_KNOB);
    lv_obj_set_style_bg_color(DisableFiltering, lv_color_hex(0xFFFFFF), LV_PART_INDICATOR);
    lv_obj_set_style_outline_width(DisableFiltering, 2, LV_PART_INDICATOR);
    lv_obj_set_style_outline_color(DisableFiltering, lv_color_hex(0xD3D3D3), LV_PART_INDICATOR);
    lv_slider_set_range(DisableFiltering, 1, 240);
    lv_slider_set_value(DisableFiltering, DriverLoopMilliseconds, LV_ANIM_OFF);
    lv_obj_add_event_cb(DisableFiltering, DriverLoopMillisecondsChanged, LV_EVENT_VALUE_CHANGED, DisableFiltering);
    lv_obj_align(DisableFiltering, LV_ALIGN_CENTER, 0, lineY + 30);

    lv_obj_t *DisableFilteringLabel = lv_label_create(parent);
    lv_obj_set_size(DisableFilteringLabel, 300, 20);
    lv_obj_align(DisableFilteringLabel, LV_ALIGN_CENTER, 0, lineY);
    lv_obj_set_style_text_align(DisableFilteringLabel, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_font(DisableFilteringLabel, &lv_font_montserrat_16, 0);
    char buf[16 + 8];
    sprintf(buf, "Sensor Hz: %.0f", 1000.0 / (10 + DriverLoopMilliseconds));
    lv_label_set_text(DisableFilteringLabel, buf);
    lv_obj_add_style(DisableFilteringLabel, &style_title, LV_STATE_DEFAULT);

    SettingLabelDriverLoopMilliseconds = DisableFilteringLabel;

    lineY += 70;
  }

  if (true)
  {
    lv_obj_t *DisableFiltering = lv_slider_create(parent);
    lv_obj_add_flag(DisableFiltering, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_size(DisableFiltering, 300, 35);
    lv_obj_set_style_radius(DisableFiltering, 3, LV_PART_KNOB); // Adjust the value for more or less rounding
    lv_obj_set_style_bg_opa(DisableFiltering, LV_OPA_TRANSP, LV_PART_KNOB);

    lv_obj_set_style_bg_color(DisableFiltering, lv_color_hex(0xAAAAAA), LV_PART_KNOB);
    lv_obj_set_style_bg_color(DisableFiltering, lv_color_hex(0xFFFFFF), LV_PART_INDICATOR);
    lv_obj_set_style_outline_width(DisableFiltering, 2, LV_PART_INDICATOR);
    lv_obj_set_style_outline_color(DisableFiltering, lv_color_hex(0xD3D3D3), LV_PART_INDICATOR);
    lv_slider_set_range(DisableFiltering, 0, 250);
    lv_slider_set_value(DisableFiltering, AttitudeBalanceAlpha * 250, LV_ANIM_OFF);
    lv_obj_add_event_cb(DisableFiltering, ChangeAttitudeBalanceAlphaChanged, LV_EVENT_VALUE_CHANGED, DisableFiltering);
    lv_obj_align(DisableFiltering, LV_ALIGN_CENTER, 0, lineY + 30);

    lv_obj_t *DisableFilteringLabel = lv_label_create(parent);
    lv_obj_set_size(DisableFilteringLabel, 300, 20);
    lv_obj_align(DisableFilteringLabel, LV_ALIGN_CENTER, 0, lineY);
    lv_obj_set_style_text_align(DisableFilteringLabel, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_font(DisableFilteringLabel, &lv_font_montserrat_16, 0);
    char buf[23 + 8];
    sprintf(buf, "Attitude compensation: %.0f", AttitudeBalanceAlpha * 250);
    lv_label_set_text(DisableFilteringLabel, buf);
    lv_obj_add_style(DisableFilteringLabel, &style_title, LV_STATE_DEFAULT);

    SettingAttitudeCompensation = DisableFilteringLabel;

    lineY += 70;
  }

  if (true)
  {
    lv_obj_t *DisableFiltering = lv_slider_create(parent);
    lv_obj_add_flag(DisableFiltering, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_size(DisableFiltering, 300, 35);
    lv_obj_set_style_radius(DisableFiltering, 3, LV_PART_KNOB); // Adjust the value for more or less rounding
    lv_obj_set_style_bg_opa(DisableFiltering, LV_OPA_TRANSP, LV_PART_KNOB);

    lv_obj_set_style_bg_color(DisableFiltering, lv_color_hex(0xAAAAAA), LV_PART_KNOB);
    lv_obj_set_style_bg_color(DisableFiltering, lv_color_hex(0xFFFFFF), LV_PART_INDICATOR);
    lv_obj_set_style_outline_width(DisableFiltering, 2, LV_PART_INDICATOR);
    lv_obj_set_style_outline_color(DisableFiltering, lv_color_hex(0xD3D3D3), LV_PART_INDICATOR);
    lv_slider_set_range(DisableFiltering, 0, 1000);
    lv_slider_set_value(DisableFiltering, 500, LV_ANIM_OFF);
    lv_obj_add_event_cb(DisableFiltering, AltimeterOverrideChanged, LV_EVENT_VALUE_CHANGED, DisableFiltering);
    lv_obj_align(DisableFiltering, LV_ALIGN_CENTER, 0, lineY + 30);

    bmp280overrideLabel = lv_label_create(parent);
    lv_obj_set_size(bmp280overrideLabel, 400, 20);
    lv_obj_align(bmp280overrideLabel, LV_ALIGN_CENTER, 0, lineY);
    lv_obj_set_style_text_align(bmp280overrideLabel, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_font(bmp280overrideLabel, &lv_font_montserrat_16, 0);
    char buf[50];
    sprintf(buf, "Altimeter QNH: %d mmHg %ld feet (%ld)", QNH, Altimeter, singletonConfig()->bmp280override);
    lv_label_set_text(bmp280overrideLabel, buf);
    lv_obj_add_style(bmp280overrideLabel, &style_title, LV_STATE_DEFAULT);

    lineY += 70;
  }

  if (true)
  {
    lv_obj_t *DisableFilteringLabel = lv_label_create(parent);
    lv_obj_set_size(DisableFilteringLabel, 150, 16);
    lv_obj_align(DisableFilteringLabel, LV_ALIGN_CENTER, -75, lineY);
    lv_obj_set_style_text_align(DisableFilteringLabel, LV_TEXT_ALIGN_RIGHT, 0);
    lv_obj_set_style_text_font(DisableFilteringLabel, &lv_font_montserrat_16, 0);
    lv_label_set_text(DisableFilteringLabel, "Auto QNH");
    lv_obj_add_style(DisableFilteringLabel, &style_title, LV_STATE_DEFAULT);
    lv_obj_t *sw = lv_switch_create(parent);
    if (singletonConfig()->settingsAutoQNH != 0)
    {
      lv_obj_add_state(sw, LV_STATE_CHECKED);
    }
    lv_obj_set_size(sw, 65, 40);
    lv_obj_align(sw, LV_ALIGN_CENTER, 40, lineY);
    lv_obj_add_event_cb(sw, AutoQNHModeChanged, LV_EVENT_VALUE_CHANGED, sw);
    lineY += 40;
  }

  if (true)
  {
    lv_obj_t *DisableFiltering = lv_slider_create(parent);
    lv_obj_add_flag(DisableFiltering, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_size(DisableFiltering, 300, 35);
    lv_obj_set_style_radius(DisableFiltering, 3, LV_PART_KNOB); // Adjust the value for more or less rounding
    lv_obj_set_style_bg_opa(DisableFiltering, LV_OPA_TRANSP, LV_PART_KNOB);

    lv_obj_set_style_bg_color(DisableFiltering, lv_color_hex(0xAAAAAA), LV_PART_KNOB);
    lv_obj_set_style_bg_color(DisableFiltering, lv_color_hex(0xFFFFFF), LV_PART_INDICATOR);
    lv_obj_set_style_outline_width(DisableFiltering, 2, LV_PART_INDICATOR);
    lv_obj_set_style_outline_color(DisableFiltering, lv_color_hex(0xD3D3D3), LV_PART_INDICATOR);
    lv_slider_set_range(DisableFiltering, 2, 12);
    lv_slider_set_value(DisableFiltering, 3, LV_ANIM_OFF);
    lv_obj_add_event_cb(DisableFiltering, GMeterMaxChanged, LV_EVENT_VALUE_CHANGED, DisableFiltering);
    lv_obj_align(DisableFiltering, LV_ALIGN_CENTER, 0, lineY + 30);

    lv_obj_t *DisableFilteringLabel = lv_label_create(parent);
    lv_obj_set_size(DisableFilteringLabel, 300, 20);
    lv_obj_align(DisableFilteringLabel, LV_ALIGN_CENTER, 0, lineY);
    lv_obj_set_style_text_align(DisableFilteringLabel, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_font(DisableFilteringLabel, &lv_font_montserrat_16, 0);
    lv_label_set_text(DisableFilteringLabel, "GMeter Scale 2G 12G");
    lv_obj_add_style(DisableFilteringLabel, &style_title, LV_STATE_DEFAULT);

    lineY += 70;
  }

  if (true)
  {
    lv_obj_t *DisableFiltering = lv_slider_create(parent);
    lv_obj_add_flag(DisableFiltering, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_size(DisableFiltering, 300, 35);
    lv_obj_set_style_radius(DisableFiltering, 3, LV_PART_KNOB); // Adjust the value for more or less rounding
    lv_obj_set_style_bg_opa(DisableFiltering, LV_OPA_TRANSP, LV_PART_KNOB);

    lv_obj_set_style_bg_color(DisableFiltering, lv_color_hex(0xAAAAAA), LV_PART_KNOB);
    lv_obj_set_style_bg_color(DisableFiltering, lv_color_hex(0xFFFFFF), LV_PART_INDICATOR);
    lv_obj_set_style_outline_width(DisableFiltering, 2, LV_PART_INDICATOR);
    lv_obj_set_style_outline_color(DisableFiltering, lv_color_hex(0xD3D3D3), LV_PART_INDICATOR);
    lv_slider_set_range(DisableFiltering, 0, Backlight_MAX);
    lv_slider_set_value(DisableFiltering, 100, LV_ANIM_OFF);
    lv_obj_add_event_cb(DisableFiltering, Backlight_adjustment_event_Changed, LV_EVENT_VALUE_CHANGED, DisableFiltering);
    lv_obj_align(DisableFiltering, LV_ALIGN_CENTER, 0, lineY + 30);

    lv_obj_t *DisableFilteringLabel = lv_label_create(parent);
    lv_obj_set_size(DisableFilteringLabel, 300, 20);
    lv_obj_align(DisableFilteringLabel, LV_ALIGN_CENTER, 0, lineY);
    lv_obj_set_style_text_align(DisableFilteringLabel, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_font(DisableFilteringLabel, &lv_font_montserrat_16, 0);
    lv_label_set_text(DisableFilteringLabel, "Backlight brightness");
    lv_obj_add_style(DisableFilteringLabel, &style_title, LV_STATE_DEFAULT);

    lineY += 70;
  }

  if (true)
  {
    lv_obj_t *DisableFilteringLabel = lv_label_create(parent);
    lv_obj_set_size(DisableFilteringLabel, 150, 16);
    lv_obj_align(DisableFilteringLabel, LV_ALIGN_CENTER, -75, lineY);
    lv_obj_set_style_text_align(DisableFilteringLabel, LV_TEXT_ALIGN_RIGHT, 0);
    lv_obj_set_style_text_font(DisableFilteringLabel, &lv_font_montserrat_16, 0);
    lv_label_set_text(DisableFilteringLabel, "Demo mode");
    lv_obj_add_style(DisableFilteringLabel, &style_title, LV_STATE_DEFAULT);
    lv_obj_t *sw = lv_switch_create(parent);
    if (DeviceIsDemoMode != 0)
    {
      lv_obj_add_state(sw, LV_STATE_CHECKED);
    }
    lv_obj_set_size(sw, 65, 40);
    lv_obj_align(sw, LV_ALIGN_CENTER, 40, lineY);
    lv_obj_add_event_cb(sw, DeviceIsDemoModeChanged, LV_EVENT_VALUE_CHANGED, sw);
    lineY += 40;
  }

  if (true)
  {
    lv_obj_t *DisableFilteringLabel = lv_label_create(parent);
    lv_obj_set_size(DisableFilteringLabel, 150, 16);
    lv_obj_align(DisableFilteringLabel, LV_ALIGN_CENTER, -75, lineY);
    lv_obj_set_style_text_align(DisableFilteringLabel, LV_TEXT_ALIGN_RIGHT, 0);
    lv_obj_set_style_text_font(DisableFilteringLabel, &lv_font_montserrat_16, 0);
    lv_label_set_text(DisableFilteringLabel, "Att+GPS");
    lv_obj_add_style(DisableFilteringLabel, &style_title, LV_STATE_DEFAULT);
    lv_obj_t *sw = lv_switch_create(parent);
    if (GPSAccelerationForAttitudeCompensationEnabled == true)
    {
      lv_obj_add_state(sw, LV_STATE_CHECKED);
    }
    lv_obj_set_size(sw, 65, 40);
    lv_obj_align(sw, LV_ALIGN_CENTER, 40, lineY);
    lv_obj_add_event_cb(sw, GPSAccelerationForAttitudeCompensationEnabledChanged, LV_EVENT_VALUE_CHANGED, sw);
    lineY += 40;
  }

  if (true)
  {
    lv_obj_t *DisableFilteringLabel = lv_label_create(parent);
    lv_obj_set_size(DisableFilteringLabel, 150, 16);
    lv_obj_align(DisableFilteringLabel, LV_ALIGN_CENTER, -75, lineY);
    lv_obj_set_style_text_align(DisableFilteringLabel, LV_TEXT_ALIGN_RIGHT, 0);
    lv_obj_set_style_text_font(DisableFilteringLabel, &lv_font_montserrat_16, 0);
    lv_label_set_text(DisableFilteringLabel, "Madgwick");
    lv_obj_add_style(DisableFilteringLabel, &style_title, LV_STATE_DEFAULT);
    lv_obj_t *sw = lv_switch_create(parent);
    if (EnableAttitudeMadgwick == true)
    {
      lv_obj_add_state(sw, LV_STATE_CHECKED);
    }
    lv_obj_set_size(sw, 65, 40);
    lv_obj_align(sw, LV_ALIGN_CENTER, 40, lineY);
    lv_obj_add_event_cb(sw, AttitudeMadwickChanged, LV_EVENT_VALUE_CHANGED, sw);
    lineY += 40;
  }

  if (true)
  {
    SettingStatus0 = lv_label_create(parent);
    lv_obj_set_size(SettingStatus0, 400, 20);
    lv_obj_align(SettingStatus0, LV_ALIGN_CENTER, 0, lineY);
    lv_obj_set_style_text_font(SettingStatus0, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_align(SettingStatus0, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_add_style(SettingStatus0, &style_title, LV_STATE_DEFAULT);
    lineY += 20;
  }
  if (true)
  {
    SettingStatus1 = lv_label_create(parent);
    lv_obj_set_size(SettingStatus1, 400, 20);
    lv_obj_align(SettingStatus1, LV_ALIGN_CENTER, 0, lineY);
    lv_obj_set_style_text_font(SettingStatus1, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_align(SettingStatus1, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_add_style(SettingStatus1, &style_title, LV_STATE_DEFAULT);
    lineY += 20;
  }
  if (true)
  {
    singletonConfig()->ui.SettingStatus2 = lv_label_create(parent);
    lv_obj_set_size(singletonConfig()->ui.SettingStatus2, 400, 20);
    lv_obj_align(singletonConfig()->ui.SettingStatus2, LV_ALIGN_CENTER, 0, lineY);
    lv_obj_set_style_text_font(singletonConfig()->ui.SettingStatus2, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_align(singletonConfig()->ui.SettingStatus2, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_add_style(singletonConfig()->ui.SettingStatus2, &style_title, LV_STATE_DEFAULT);
    lineY += 20;
  }

  if (true)
  {
    SettingStatus3 = lv_label_create(parent);
    lv_obj_set_size(SettingStatus3, 400, 20);
    lv_obj_align(SettingStatus3, LV_ALIGN_CENTER, 0, lineY);
    lv_obj_set_style_text_font(SettingStatus3, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_align(SettingStatus3, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_add_style(SettingStatus3, &style_title, LV_STATE_DEFAULT);
    lineY += 20;
  }

  // 1.1.8
  if (true)
  {
    SettingStatus5 = lv_label_create(parent);
    lv_obj_set_size(SettingStatus5, 400, 20);
    lv_obj_align(SettingStatus5, LV_ALIGN_CENTER, 0, lineY);
    lv_obj_set_style_text_font(SettingStatus5, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_align(SettingStatus5, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_add_style(SettingStatus5, &style_title, LV_STATE_DEFAULT);
    lineY += 20;
  }
  // 1.1.4
  if (true)
  {
    SettingStatus4 = lv_label_create(parent);
    lv_obj_set_size(SettingStatus4, 400, 20);
    lv_obj_align(SettingStatus4, LV_ALIGN_CENTER, 0, lineY);
    lv_obj_set_style_text_font(SettingStatus4, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_align(SettingStatus4, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_add_style(SettingStatus4, &style_title, LV_STATE_DEFAULT);
    lineY += 20;
  }
#ifdef RB_ENABLE_GPS
  if (true)
  {
    lineY += 20;
    /*Create a normal drop down list*/
    uartDropDown = lv_dropdown_create(parent);
    lv_dropdown_set_options(uartDropDown, "GPS Disabled\n"
                                          "4800\n"
                                          "9600\n"
                                          "19200\n"
                                          "38400\n"
                                          "115200");

    lv_obj_align(uartDropDown, LV_ALIGN_CENTER, 0, lineY);
    lv_obj_add_event_cb(uartDropDown, event_handler_gps_menu, LV_EVENT_ALL, NULL);

    lineY += 30;
  }

  if (true)
  {
    SettingStatus4UART = lv_label_create(parent);
    lv_obj_set_size(SettingStatus4UART, 400, 20);
    lv_obj_align(SettingStatus4UART, LV_ALIGN_CENTER, 0, lineY);
    lv_obj_set_style_text_font(SettingStatus4UART, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_align(SettingStatus4UART, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_add_style(SettingStatus4UART, &style_title, LV_STATE_DEFAULT);
    // snprintf(SettingStatus4UARTBuf,sizeof(SettingStatus4UARTBuf),"NO DATA RECEIVED");
    lv_label_set_text(SettingStatus4UART, "NO DATA RECEIVED");

    lineY += 30;
  }
#else
  if (true)
  {
    SettingStatus4UART = lv_label_create(parent);
    lv_obj_set_size(SettingStatus4UART, 400, 20);
    lv_obj_align(SettingStatus4UART, LV_ALIGN_CENTER, 0, lineY);
    lv_obj_set_style_text_font(SettingStatus4UART, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_align(SettingStatus4UART, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_add_style(SettingStatus4UART, &style_title, LV_STATE_DEFAULT);
    lv_label_set_text(SettingStatus4UART, "GPS NOT ENABLED");
    lineY += 30;
  }
#endif
#ifdef RB_ENABLE_SPD
  if (true)
  {
    lv_obj_t *label = lv_label_create(parent);
    lv_obj_set_size(label, 400, 20);
    lv_obj_align(label, LV_ALIGN_CENTER, 0, lineY);
    lv_obj_set_style_text_font(label, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_add_style(label, &style_title, LV_STATE_DEFAULT);
    lv_label_set_text(label, "SPEED SETUP");

    lineY += 25;
  }

  if (true)
  {
    lineY += 10;
    /*Create a normal drop down list*/
    kmhDropDown = lv_dropdown_create(parent);
    lv_dropdown_set_options(kmhDropDown, "Imperial Units\n"
                                         "Metric Units");

    lv_obj_align(kmhDropDown, LV_ALIGN_CENTER, 0, lineY);
    lv_obj_add_event_cb(kmhDropDown, event_handler_kmh_menu, LV_EVENT_ALL, NULL);
    lv_dropdown_set_selected(kmhDropDown, isKmh);
    lineY += 32;
  }

  if (true)
  {
    t_speedSummary = lv_label_create(parent);
    lv_obj_set_size(t_speedSummary, 400, 20);
    lv_obj_align(t_speedSummary, LV_ALIGN_CENTER, 0, lineY);
    lv_obj_set_style_text_font(t_speedSummary, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_align(t_speedSummary, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_add_style(t_speedSummary, &style_title, LV_STATE_DEFAULT);
    char buf[60];
    snprintf(buf, 60, "%u->%u %u->%u W:%u G:%u Y:%u R:%u",
             degreeStart,
             degreeEnd,
             speedKtStart,
             speedKtEnd,
             speedWhite,
             speedGreen,
             speedYellow,
             speedRed);
    lv_label_set_text(t_speedSummary, buf);

    lineY += 32;
  }

  if (true)
  {
    lv_obj_t *progressObject = lv_slider_create(parent);
    lv_obj_add_flag(progressObject, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_size(progressObject, 300, 35);
    lv_obj_set_style_radius(progressObject, 3, LV_PART_KNOB); // Adjust the value for more or less rounding
    lv_obj_set_style_bg_opa(progressObject, LV_OPA_TRANSP, LV_PART_KNOB);

    lv_obj_set_style_bg_color(progressObject, lv_color_hex(0xAAAAAA), LV_PART_KNOB);
    lv_obj_set_style_bg_color(progressObject, lv_color_hex(0xFFFFFF), LV_PART_INDICATOR);
    lv_obj_set_style_outline_width(progressObject, 2, LV_PART_INDICATOR);
    lv_obj_set_style_outline_color(progressObject, lv_color_hex(0xD3D3D3), LV_PART_INDICATOR);
    lv_slider_set_range(progressObject, 0, 23);
    lv_slider_set_value(progressObject, degreeStart / 15, LV_ANIM_OFF);
    lv_obj_add_event_cb(progressObject, SpeedDegreeStartChanged, LV_EVENT_VALUE_CHANGED, progressObject);
    lv_obj_align(progressObject, LV_ALIGN_CENTER, 0, lineY + 30);

    lv_obj_t *progressObjectLabel = lv_label_create(parent);
    lv_obj_set_size(progressObjectLabel, 300, 20);
    lv_obj_align(progressObjectLabel, LV_ALIGN_CENTER, 0, lineY);
    lv_obj_set_style_text_align(progressObjectLabel, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_font(progressObjectLabel, &lv_font_montserrat_16, 0);
    lv_label_set_text(progressObjectLabel, "Speed start degree");
    lv_obj_add_style(progressObjectLabel, &style_title, LV_STATE_DEFAULT);

    t_speedStart = progressObject;

    lineY += 70;
  }
  if (true)
  {
    lv_obj_t *progressObject = lv_slider_create(parent);
    lv_obj_add_flag(progressObject, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_size(progressObject, 300, 35);
    lv_obj_set_style_radius(progressObject, 3, LV_PART_KNOB); // Adjust the value for more or less rounding
    lv_obj_set_style_bg_opa(progressObject, LV_OPA_TRANSP, LV_PART_KNOB);

    lv_obj_set_style_bg_color(progressObject, lv_color_hex(0xAAAAAA), LV_PART_KNOB);
    lv_obj_set_style_bg_color(progressObject, lv_color_hex(0xFFFFFF), LV_PART_INDICATOR);
    lv_obj_set_style_outline_width(progressObject, 2, LV_PART_INDICATOR);
    lv_obj_set_style_outline_color(progressObject, lv_color_hex(0xD3D3D3), LV_PART_INDICATOR);
    lv_slider_set_range(progressObject, 1, 24);
    lv_slider_set_value(progressObject, degreeEnd / 15, LV_ANIM_OFF);
    lv_obj_add_event_cb(progressObject, SpeedDegreeStartChanged, LV_EVENT_VALUE_CHANGED, progressObject);
    lv_obj_align(progressObject, LV_ALIGN_CENTER, 0, lineY + 30);

    lv_obj_t *progressObjectLabel = lv_label_create(parent);
    lv_obj_set_size(progressObjectLabel, 300, 20);
    lv_obj_align(progressObjectLabel, LV_ALIGN_CENTER, 0, lineY);
    lv_obj_set_style_text_align(progressObjectLabel, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_font(progressObjectLabel, &lv_font_montserrat_16, 0);
    lv_label_set_text(progressObjectLabel, "Speed end degree");
    lv_obj_add_style(progressObjectLabel, &style_title, LV_STATE_DEFAULT);

    t_speedEnd = progressObject;

    lineY += 70;
  }

  if (true)
  {
    lv_obj_t *progressObject = lv_slider_create(parent);
    lv_obj_add_flag(progressObject, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_size(progressObject, 300, 35);
    lv_obj_set_style_radius(progressObject, 3, LV_PART_KNOB); // Adjust the value for more or less rounding
    lv_obj_set_style_bg_opa(progressObject, LV_OPA_TRANSP, LV_PART_KNOB);

    lv_obj_set_style_bg_color(progressObject, lv_color_hex(0xAAAAAA), LV_PART_KNOB);
    lv_obj_set_style_bg_color(progressObject, lv_color_hex(0xFFFFFF), LV_PART_INDICATOR);
    lv_obj_set_style_outline_width(progressObject, 2, LV_PART_INDICATOR);
    lv_obj_set_style_outline_color(progressObject, lv_color_hex(0xD3D3D3), LV_PART_INDICATOR);
    lv_slider_set_range(progressObject, 0, 10);
    lv_slider_set_value(progressObject, speedKtStart / 10, LV_ANIM_OFF);
    lv_obj_add_event_cb(progressObject, SpeedDegreeStartChanged, LV_EVENT_VALUE_CHANGED, progressObject);
    lv_obj_align(progressObject, LV_ALIGN_CENTER, 0, lineY + 30);

    lv_obj_t *progressObjectLabel = lv_label_create(parent);
    lv_obj_set_size(progressObjectLabel, 300, 20);
    lv_obj_align(progressObjectLabel, LV_ALIGN_CENTER, 0, lineY);
    lv_obj_set_style_text_align(progressObjectLabel, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_font(progressObjectLabel, &lv_font_montserrat_16, 0);
    lv_label_set_text(progressObjectLabel, "Speed start");
    lv_obj_add_style(progressObjectLabel, &style_title, LV_STATE_DEFAULT);

    t_speedStartSpeed = progressObject;

    lineY += 70;
  }
  if (true)
  {
    lv_obj_t *progressObject = lv_slider_create(parent);
    lv_obj_add_flag(progressObject, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_size(progressObject, 300, 35);
    lv_obj_set_style_radius(progressObject, 3, LV_PART_KNOB); // Adjust the value for more or less rounding
    lv_obj_set_style_bg_opa(progressObject, LV_OPA_TRANSP, LV_PART_KNOB);

    lv_obj_set_style_bg_color(progressObject, lv_color_hex(0xAAAAAA), LV_PART_KNOB);
    lv_obj_set_style_bg_color(progressObject, lv_color_hex(0xFFFFFF), LV_PART_INDICATOR);
    lv_obj_set_style_outline_width(progressObject, 2, LV_PART_INDICATOR);
    lv_obj_set_style_outline_color(progressObject, lv_color_hex(0xD3D3D3), LV_PART_INDICATOR);
    lv_slider_set_range(progressObject, 18, 40);
    lv_slider_set_value(progressObject, speedKtEnd / 10, LV_ANIM_OFF);
    lv_obj_add_event_cb(progressObject, SpeedDegreeStartChanged, LV_EVENT_VALUE_CHANGED, progressObject);
    lv_obj_align(progressObject, LV_ALIGN_CENTER, 0, lineY + 30);

    lv_obj_t *progressObjectLabel = lv_label_create(parent);
    lv_obj_set_size(progressObjectLabel, 300, 20);
    lv_obj_align(progressObjectLabel, LV_ALIGN_CENTER, 0, lineY);
    lv_obj_set_style_text_align(progressObjectLabel, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_font(progressObjectLabel, &lv_font_montserrat_16, 0);
    lv_label_set_text(progressObjectLabel, "Speed end");
    lv_obj_add_style(progressObjectLabel, &style_title, LV_STATE_DEFAULT);

    t_speedEndSpeed = progressObject;

    lineY += 70;
  }

  if (true)
  {
    lv_obj_t *progressObject = lv_slider_create(parent);
    lv_obj_add_flag(progressObject, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_size(progressObject, 300, 35);
    lv_obj_set_style_radius(progressObject, 3, LV_PART_KNOB); // Adjust the value for more or less rounding
    lv_obj_set_style_bg_opa(progressObject, LV_OPA_TRANSP, LV_PART_KNOB);

    lv_obj_set_style_bg_color(progressObject, lv_color_hex(0xAAAAAA), LV_PART_KNOB);
    lv_obj_set_style_bg_color(progressObject, lv_color_hex(0xFFFFFF), LV_PART_INDICATOR);
    lv_obj_set_style_outline_width(progressObject, 2, LV_PART_INDICATOR);
    lv_obj_set_style_outline_color(progressObject, lv_color_hex(0xD3D3D3), LV_PART_INDICATOR);
    lv_slider_set_range(progressObject, 0, 24);
    lv_slider_set_value(progressObject, speedWhite / 15, LV_ANIM_OFF);
    lv_obj_add_event_cb(progressObject, SpeedDegreeStartChanged, LV_EVENT_VALUE_CHANGED, progressObject);
    lv_obj_align(progressObject, LV_ALIGN_CENTER, 0, lineY + 30);

    lv_obj_t *progressObjectLabel = lv_label_create(parent);
    lv_obj_set_size(progressObjectLabel, 300, 20);
    lv_obj_align(progressObjectLabel, LV_ALIGN_CENTER, 0, lineY);
    lv_obj_set_style_text_align(progressObjectLabel, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_font(progressObjectLabel, &lv_font_montserrat_16, 0);
    lv_label_set_text(progressObjectLabel, "White arc start degree");
    lv_obj_add_style(progressObjectLabel, &style_title, LV_STATE_DEFAULT);

    t_speedWhite = progressObject;

    lineY += 70;
  }
  if (true)
  {
    lv_obj_t *progressObject = lv_slider_create(parent);
    lv_obj_add_flag(progressObject, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_size(progressObject, 300, 35);
    lv_obj_set_style_radius(progressObject, 3, LV_PART_KNOB); // Adjust the value for more or less rounding
    lv_obj_set_style_bg_opa(progressObject, LV_OPA_TRANSP, LV_PART_KNOB);

    lv_obj_set_style_bg_color(progressObject, lv_color_hex(0xAAAAAA), LV_PART_KNOB);
    lv_obj_set_style_bg_color(progressObject, lv_color_hex(0xFFFFFF), LV_PART_INDICATOR);
    lv_obj_set_style_outline_width(progressObject, 2, LV_PART_INDICATOR);
    lv_obj_set_style_outline_color(progressObject, lv_color_hex(0xD3D3D3), LV_PART_INDICATOR);
    lv_slider_set_range(progressObject, 0, 24);
    lv_slider_set_value(progressObject, speedGreen / 15, LV_ANIM_OFF);
    lv_obj_add_event_cb(progressObject, SpeedDegreeStartChanged, LV_EVENT_VALUE_CHANGED, progressObject);
    lv_obj_align(progressObject, LV_ALIGN_CENTER, 0, lineY + 30);

    lv_obj_t *progressObjectLabel = lv_label_create(parent);
    lv_obj_set_size(progressObjectLabel, 300, 20);
    lv_obj_align(progressObjectLabel, LV_ALIGN_CENTER, 0, lineY);
    lv_obj_set_style_text_align(progressObjectLabel, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_font(progressObjectLabel, &lv_font_montserrat_16, 0);
    lv_label_set_text(progressObjectLabel, "Green arc start degree");
    lv_obj_add_style(progressObjectLabel, &style_title, LV_STATE_DEFAULT);

    t_speedGreen = progressObject;

    lineY += 70;
  }
  if (true)
  {
    lv_obj_t *progressObject = lv_slider_create(parent);
    lv_obj_add_flag(progressObject, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_size(progressObject, 300, 35);
    lv_obj_set_style_radius(progressObject, 3, LV_PART_KNOB); // Adjust the value for more or less rounding
    lv_obj_set_style_bg_opa(progressObject, LV_OPA_TRANSP, LV_PART_KNOB);

    lv_obj_set_style_bg_color(progressObject, lv_color_hex(0xAAAAAA), LV_PART_KNOB);
    lv_obj_set_style_bg_color(progressObject, lv_color_hex(0xFFFFFF), LV_PART_INDICATOR);
    lv_obj_set_style_outline_width(progressObject, 2, LV_PART_INDICATOR);
    lv_obj_set_style_outline_color(progressObject, lv_color_hex(0xD3D3D3), LV_PART_INDICATOR);
    lv_slider_set_range(progressObject, 0, 24);
    lv_slider_set_value(progressObject, speedYellow / 15, LV_ANIM_OFF);
    lv_obj_add_event_cb(progressObject, SpeedDegreeStartChanged, LV_EVENT_VALUE_CHANGED, progressObject);
    lv_obj_align(progressObject, LV_ALIGN_CENTER, 0, lineY + 30);

    lv_obj_t *progressObjectLabel = lv_label_create(parent);
    lv_obj_set_size(progressObjectLabel, 300, 20);
    lv_obj_align(progressObjectLabel, LV_ALIGN_CENTER, 0, lineY);
    lv_obj_set_style_text_align(progressObjectLabel, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_font(progressObjectLabel, &lv_font_montserrat_16, 0);
    lv_label_set_text(progressObjectLabel, "Yellow arc start degree");
    lv_obj_add_style(progressObjectLabel, &style_title, LV_STATE_DEFAULT);

    t_speedYellow = progressObject;

    lineY += 70;
  }
  if (true)
  {
    lv_obj_t *progressObject = lv_slider_create(parent);
    lv_obj_add_flag(progressObject, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_size(progressObject, 300, 35);
    lv_obj_set_style_radius(progressObject, 3, LV_PART_KNOB); // Adjust the value for more or less rounding
    lv_obj_set_style_bg_opa(progressObject, LV_OPA_TRANSP, LV_PART_KNOB);

    lv_obj_set_style_bg_color(progressObject, lv_color_hex(0xAAAAAA), LV_PART_KNOB);
    lv_obj_set_style_bg_color(progressObject, lv_color_hex(0xFFFFFF), LV_PART_INDICATOR);
    lv_obj_set_style_outline_width(progressObject, 2, LV_PART_INDICATOR);
    lv_obj_set_style_outline_color(progressObject, lv_color_hex(0xD3D3D3), LV_PART_INDICATOR);
    lv_slider_set_range(progressObject, 0, 24);
    lv_slider_set_value(progressObject, speedRed / 15, LV_ANIM_OFF);
    lv_obj_add_event_cb(progressObject, SpeedDegreeStartChanged, LV_EVENT_VALUE_CHANGED, progressObject);
    lv_obj_align(progressObject, LV_ALIGN_CENTER, 0, lineY + 30);

    lv_obj_t *progressObjectLabel = lv_label_create(parent);
    lv_obj_set_size(progressObjectLabel, 300, 20);
    lv_obj_align(progressObjectLabel, LV_ALIGN_CENTER, 0, lineY);
    lv_obj_set_style_text_align(progressObjectLabel, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_font(progressObjectLabel, &lv_font_montserrat_16, 0);
    lv_label_set_text(progressObjectLabel, "Red arc start degree");
    lv_obj_add_style(progressObjectLabel, &style_title, LV_STATE_DEFAULT);

    t_speedRed = progressObject;

    lineY += 70;
  }
#endif

  lineY += 10;
  if (true)
  {
    lv_obj_t *VersionLabel = lv_label_create(parent);
    lv_obj_set_size(VersionLabel, 200, 20);
    lv_obj_align(VersionLabel, LV_ALIGN_CENTER, 0, lineY);
    lv_obj_set_style_text_font(VersionLabel, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_align(VersionLabel, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_text(VersionLabel, "XIAPROJECTS SRL");
    lv_obj_add_style(VersionLabel, &style_title, LV_STATE_DEFAULT);
    lineY += 20;
  }

  lv_obj_add_event_cb(parent, speedBgClicked, LV_EVENT_CLICKED, NULL);
}

#ifdef RB_ENABLE_CLK
static void Onboard_create_Clock(lv_obj_t *parent)
{

  // 1.1.1 Branding RB-02 on every screen
  if (true)
  {
    lv_obj_t *label = lv_label_create(parent);
    lv_obj_set_size(label, 96, 40);
    lv_obj_align(label, LV_ALIGN_CENTER, 0, -200);
    lv_obj_set_style_text_font(label, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_text(label, RB_PRODUCT_TITLE);
    lv_obj_add_style(label, &style_title, LV_STATE_DEFAULT);
  }

  if (true)
  {
    lv_obj_t *label = lv_label_create(parent);
    lv_obj_set_size(label, 200, 40);
    lv_obj_align(label, LV_ALIGN_CENTER, 0, -160);
    lv_obj_set_style_text_font(label, &lv_font_montserrat_48, 0);
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_text(label, "TIMER 1");
    lv_obj_add_style(label, &style_title, LV_STATE_DEFAULT);

    TimerLabelTop = label;
  }

  // CreateDigitArray(parent, &DigitFont100x25, DIGIT_BIG_DIGITXXX,0,0);
  CreateSingleDigit(parent, &DigitFont100x25, SegmentsA[0], -(DigitFont100x25.header.w + 8 + DigitFont100x25.header.h) - 24, 0);
  CreateSingleDigit(parent, &DigitFont100x25, SegmentsA[1], -24, 0);
  CreateSingleDigit(parent, &DigitFont70x20, SegmentsA[2], (DigitFont100x25.header.w / 2 + DigitFont100x25.header.h + 16), 15);
  CreateSingleDigit(parent, &DigitFont70x20, SegmentsA[3], (DigitFont100x25.header.w / 2 + DigitFont100x25.header.h + 24) + (DigitFont70x20.header.w + DigitFont70x20.header.h), 15);

  // 1.1.1 Displays always 3 timers
  if (true)
  {
    lv_obj_t *label = lv_label_create(parent);
    lv_obj_set_size(label, 200, 40);
    lv_obj_align(label, LV_ALIGN_CENTER, -95, 165);
    lv_obj_set_style_text_font(label, &lv_font_montserrat_48, 0);
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_text(label, "TIMER 1");
    lv_obj_add_style(label, &style_title, LV_STATE_DEFAULT);

    TimerSW = label;
  }
  if (true)
  {
    lv_obj_t *label = lv_label_create(parent);
    lv_obj_set_size(label, 200, 40);
    lv_obj_align(label, LV_ALIGN_CENTER, -95, 150);
    lv_obj_set_style_text_font(label, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_text(label, "TIMER 2");
    lv_obj_add_style(label, &style_title, LV_STATE_DEFAULT);

    TimerLabelSW = label;
  }
  if (true)
  {
    lv_obj_t *label = lv_label_create(parent);
    lv_obj_set_size(label, 200, 40);
    lv_obj_align(label, LV_ALIGN_CENTER, 95, 165);
    lv_obj_set_style_text_font(label, &lv_font_montserrat_48, 0);
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_text(label, "--:--");
    lv_obj_add_style(label, &style_title, LV_STATE_DEFAULT);

    TimerSE = label;
  }
  if (true)
  {
    lv_obj_t *label = lv_label_create(parent);
    lv_obj_set_size(label, 200, 40);
    lv_obj_align(label, LV_ALIGN_CENTER, 95, 150);
    lv_obj_set_style_text_font(label, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
#ifdef RB_ENABLE_GPS
    lv_label_set_text(label, "GPS UTC+0");
#else
    lv_label_set_text(label, "TIMER 3");
#endif
    lv_obj_add_style(label, &style_title, LV_STATE_DEFAULT);

    TimerLabelSE = label;
  }

  lv_obj_add_event_cb(parent, speedBgClicked, LV_EVENT_CLICKED, NULL);

  // 1.1.9 Remove scrolling for Turbolence touch screen
  lv_obj_add_flag(parent, LV_OBJ_FLAG_SCROLLABLE);
}
#endif

#ifdef RB_ENABLE_ALD
static void Onboard_create_AltimeterDigital(lv_obj_t *parent)
{
  // 1.1.1 Branding RB-02 on every screen
  if (true)
  {
    lv_obj_t *label = lv_label_create(parent);
    lv_obj_set_size(label, 96, 40);
    lv_obj_align(label, LV_ALIGN_CENTER, 0, -200);
    lv_obj_set_style_text_font(label, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_text(label, RB_PRODUCT_TITLE);
    lv_obj_add_style(label, &style_title, LV_STATE_DEFAULT);
  }
  // 1.1.2 Added feet label
  if (true)
  {
    lv_obj_t *label = lv_label_create(parent);
    lv_obj_set_size(label, 96, 40);
    lv_obj_align(label, LV_ALIGN_CENTER, 185, 100);
    lv_obj_set_style_text_font(label, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_text(label, "feet");
    lv_obj_add_style(label, &style_title, LV_STATE_DEFAULT);
  }
  if (true)
  {
    lv_obj_t *label = lv_label_create(parent);
    lv_obj_set_size(label, 300, 40);
    lv_obj_align(label, LV_ALIGN_CENTER, 0, -160);
    lv_obj_set_style_text_font(label, &lv_font_montserrat_48, 0);
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_text(label, "ALTIMETER");
    lv_obj_add_style(label, &style_title, LV_STATE_DEFAULT);
  }
  if (true)
  {
    lv_obj_t *label = lv_label_create(parent);
    lv_obj_set_size(label, 440, 40);
    lv_obj_align(label, LV_ALIGN_CENTER, 0, -160 + 48);
    lv_obj_set_style_text_font(label, &lv_font_montserrat_48, 0);
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
    // 1.1.2 added mmHg conversion
    lv_label_set_text(label, "QNH: 1013 - 29.91");
    lv_obj_add_style(label, &style_title, LV_STATE_DEFAULT);
    Screen_Altitude_QNH2 = label;
  }

  const int numDigit = 5;
  for (int c = 0; c < numDigit; c++)
  {
    int k = (DigitFont70x20.header.w + DigitFont70x20.header.h + 8) * (c - 2) - 8; // we estimate that you will not fly more than 19999
    CreateSingleDigit(parent, &DigitFont70x20, SegmentsAltDigit[c], k, 0);
  }

  // 1.0.6 Adding the digital variometer
  if (true)
  {
    lv_obj_t *label = lv_label_create(parent);
    lv_obj_set_size(label, 300, 40);
    lv_obj_align(label, LV_ALIGN_CENTER, 0, 106);
    lv_obj_set_style_text_font(label, &lv_font_montserrat_48, 0);
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_text(label, "---");
    lv_obj_add_style(label, &style_title, LV_STATE_DEFAULT);
    Screen_Altitude_Variometer2 = label;
  }
  if (true)
  {
    lv_obj_t *label = lv_label_create(parent);
    lv_obj_set_size(label, 300, 40);
    lv_obj_align(label, LV_ALIGN_CENTER, 0, 150);
    lv_obj_set_style_text_font(label, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_text(label, "feet/min");
    lv_obj_add_style(label, &style_title, LV_STATE_DEFAULT);
  }
  if (true)
  {
    lv_obj_t *label = lv_label_create(parent);
    lv_obj_set_size(label, 300, 40);
    lv_obj_align(label, LV_ALIGN_CENTER, 0, 170);
    lv_obj_set_style_text_font(label, &lv_font_montserrat_48, 0);
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_text(label, "---.--");
    lv_obj_add_style(label, &style_title, LV_STATE_DEFAULT);
    Screen_Altitude_Pressure = label;
  }
  if (true)
  {
    lv_obj_t *label = lv_label_create(parent);
    lv_obj_set_size(label, 300, 40);
    lv_obj_align(label, LV_ALIGN_CENTER, 0, 214);
    lv_obj_set_style_text_font(label, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
#ifdef RB_ENABLE_GPS
    lv_label_set_text(label, "GPS Altitude feet");
#else
    lv_label_set_text(label, "hPa");
#endif
    lv_obj_add_style(label, &style_title, LV_STATE_DEFAULT);
  }
  lv_obj_add_event_cb(parent, speedBgClicked, LV_EVENT_CLICKED, NULL);
}
#endif

static void CreateSingleDigit(lv_obj_t *parent, const lv_img_dsc_t *font, lv_obj_t **segments, int dx, int dy)
{
  int16_t k = font->header.w / 2;
  int16_t j = font->header.w;
  int x = 0;

  for (int y = 0; y < DIGIT_BIG_SEGMENTS; y++)
  {
    segments[y] = Onboard_create_Base(parent, font);
    int16_t a = 0;
    int16_t b = 0;
    switch (y)
    {
    case 0:
      a = 0;
      b = 0;
      break;
    case 1:
      a = 0;
      b = -j;
      break;
    case 2:
      a = 0;
      b = j;
      break;
    case 3:
      a = -k;
      b = j / 2;
      lv_img_set_angle(segments[y], 900);
      break;
    case 4:
      lv_img_set_angle(segments[y], 900);
      a = k;
      b = j / 2;
      break;
    case 5:
      lv_img_set_angle(segments[y], 900);
      a = k;
      b = -j / 2;
      break;
    case 6:
      lv_img_set_angle(segments[y], 900);
      a = -k;
      b = -j / 2;
      break;
    }

    lv_obj_align(segments[y], LV_ALIGN_CENTER, dx + a + x * (j + font->header.h + 4) - 1 / 2 * (j + font->header.h + 4), dy + b);
  }
}

void draw_arch(lv_obj_t *parent, const lv_img_dsc_t *t, uint16_t degreeStartSlide, uint16_t degreeEndSlide)
{
  for (uint16_t degree = degreeStartSlide; degree <= degreeEndSlide; degree += 15)
  {

    lv_obj_t *slice = lv_img_create(parent);
    lv_img_set_src(slice, t);
    lv_obj_set_size(slice, t->header.w, t->header.h);
    lv_img_set_pivot(slice, 32, 240);
    lv_obj_align(slice, LV_ALIGN_CENTER, 0, -120 - 96 - 16);
    lv_img_set_angle(slice, degree * 10);
  }
}
#ifdef RB_ENABLE_SPD
static void Onboard_create_Speed(lv_obj_t *parent)
{
  // Degree shall be multiples of 7.5
  draw_arch(parent, &arcWhite, speedWhite, speedGreen - 15);
  draw_arch(parent, &arcGreen, speedGreen, speedYellow - 15);
  draw_arch(parent, &arcYellow, speedYellow, speedRed - 15);
  draw_arch(parent, &arcRed, speedRed, speedRed);

  // 1.1.1 Branding RB-02 on every screen
  if (true)
  {
    lv_obj_t *label = lv_label_create(parent);
    lv_obj_set_size(label, 96, 40);
    lv_obj_align(label, LV_ALIGN_CENTER, 0, -200);
    lv_obj_set_style_text_font(label, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_text(label, RB_PRODUCT_TITLE);
    lv_obj_add_style(label, &style_title, LV_STATE_DEFAULT);
  }

  if (true)
  {
    lv_obj_t *label = lv_label_create(parent);
    lv_obj_set_size(label, 300, 40);
    lv_obj_align(label, LV_ALIGN_CENTER, 0, -35);
    lv_obj_set_style_text_font(label, &lv_font_montserrat_24, 0);
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
    if (isKmh == 1)
    {
      #if defined(RB_ENABLE_IAS)
      lv_label_set_text(label, "AIR SPEED KMH");
      #else
      lv_label_set_text(label, "GPS SPEED KMH");
      #endif
    }
    else
    {
      #if defined(RB_ENABLE_IAS)
      lv_label_set_text(label, "AIR SPEED KT");
      #else
      lv_label_set_text(label, "GPS SPEED KT");
      #endif
    }
    lv_obj_add_style(label, &style_title, LV_STATE_DEFAULT);
  }
#if defined(RB_ENABLE_IAS)
  if (true)
  {
    lv_obj_t *label = lv_label_create(parent);
    lv_obj_set_size(label, 300, 40);
    lv_obj_align(label, LV_ALIGN_CENTER, 0, 55);
    lv_obj_set_style_text_font(label, &lv_font_montserrat_24, 0);
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
    if (isKmh == 1)
    {
      lv_label_set_text(label, "GPS SPEED");
    }
    else
    {
      lv_label_set_text(label, "GPS SPEED");
    }
    lv_obj_add_style(label, &style_title, LV_STATE_DEFAULT);
  }
#endif

  uint8_t radius = (460 - 98) / 2;
  uint16_t speedKt = 0;

  uint8_t numberOfItems = (degreeEnd - degreeStart) / 30;

  uint16_t speedKtIncrement = (speedKtEnd - speedKtStart) / numberOfItems;
  uint16_t degreeIncrement = (degreeEnd - degreeStart) / numberOfItems;

  for (uint16_t degree = degreeStart; degree <= degreeEnd; degree += degreeIncrement)
  {

    int16_t sin = lv_trigo_sin(degree - 90) / 327;
    int16_t cos = lv_trigo_sin(degree) / 327;

    uint32_t x = (cos)*radius / 100;
    uint32_t y = (sin)*radius / 100;

    // printf("Speed: %d %d %ld %ld\n", degree, speedKt, x, y);

    lv_obj_t *label = lv_label_create(parent);
    lv_obj_set_size(label, 94, 48);
    lv_obj_align(label, LV_ALIGN_CENTER, x + 4, y);
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_font(label, &lv_font_montserrat_48, 0);
    lv_obj_set_style_text_color(label, lv_color_white(), 0);

    char buf[8];
    snprintf(buf, sizeof(buf), "%u", speedKtStart + (speedKt));
    speedKt += speedKtIncrement;
    lv_label_set_text(label, buf);
  }

  Screen_Speed_SpeedTick = lv_img_create(parent);
  lv_img_set_src(Screen_Speed_SpeedTick, &fi_needle);
  lv_obj_set_size(Screen_Speed_SpeedTick, fi_needle.header.w, fi_needle.header.h);
  lv_obj_align(Screen_Speed_SpeedTick, LV_ALIGN_CENTER, 0, 0);

  if(true){
  lv_obj_t *Screen_Speed_SpeedText = NULL;
  Screen_Speed_SpeedText = lv_label_create(parent);
  lv_obj_set_size(Screen_Speed_SpeedText, 128, 48);
  lv_obj_align(Screen_Speed_SpeedText, LV_ALIGN_CENTER, 0, 0);
  lv_obj_set_style_text_align(Screen_Speed_SpeedText, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_set_style_text_font(Screen_Speed_SpeedText, &lv_font_montserrat_48, 0);
  lv_obj_set_style_text_color(Screen_Speed_SpeedText, lv_color_white(), 0);
  lv_obj_set_style_bg_color(Screen_Speed_SpeedText, lv_color_black(), 0);
  lv_obj_set_style_radius(Screen_Speed_SpeedText, LV_RADIUS_CIRCLE, 0);
  char buf[4];
  snprintf(buf, sizeof(buf), "%d", 0);
  lv_label_set_text(Screen_Speed_SpeedText, buf);
  singletonConfig()->ui.labelGPSSpeed = Screen_Speed_SpeedText;
  }
#if defined(RB_ENABLE_IAS)
  if(true){
  lv_obj_t *Screen_Speed_SpeedText = NULL;
  Screen_Speed_SpeedText = lv_label_create(parent);
  lv_obj_set_size(Screen_Speed_SpeedText, 128, 48);
  lv_obj_align(Screen_Speed_SpeedText, LV_ALIGN_CENTER, 0, 80);
  lv_obj_set_style_text_align(Screen_Speed_SpeedText, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_set_style_text_font(Screen_Speed_SpeedText, &lv_font_montserrat_48, 0);
  lv_obj_set_style_text_color(Screen_Speed_SpeedText, lv_color_white(), 0);
  lv_obj_set_style_bg_color(Screen_Speed_SpeedText, lv_color_black(), 0);
  lv_obj_set_style_radius(Screen_Speed_SpeedText, LV_RADIUS_CIRCLE, 0);
  char buf[4];
  snprintf(buf, sizeof(buf), "%d", 0);
  lv_label_set_text(Screen_Speed_SpeedText, buf);
  singletonConfig()->ui.labelIASSpeed = singletonConfig()->ui.labelGPSSpeed;
  singletonConfig()->ui.labelGPSSpeed = Screen_Speed_SpeedText;
  }
#endif

  lv_obj_add_event_cb(parent, speedBgClicked, LV_EVENT_CLICKED, NULL);
}
#endif

#ifdef RB_ENABLE_ATT
static void Onboard_create_Attitude(lv_obj_t *parent)
{

  lv_obj_set_style_bg_color(parent, lv_color_hex(0x4f3822), LV_PART_ITEMS | LV_STATE_CHECKED);
  lv_obj_set_style_bg_opa(parent, LV_OPA_COVER, LV_PART_ITEMS | LV_STATE_CHECKED);
  Onboard_create_Base(parent, &att_middle_big);

  // 1.1.23 Added Custom Logo on each screen
  // RB_LV_Helper_CreateImageFromFile(parent,"S:/logo.bmp");

  Screen_Attitude_Pitch = Onboard_create_Base(parent, &att_aircraft);
  // 1.1.1 Aircraft Symbol rotation displacement
  lv_img_set_pivot(Screen_Attitude_Pitch, att_aircraft.header.w / 2, 0);
  lv_obj_add_event_cb(parent, speedBgClicked, LV_EVENT_CLICKED, NULL);

  lv_obj_t *att_img_top = Onboard_create_Base(parent, &att_circle_top_T);
  lv_obj_set_pos(att_img_top, 0, -240 + att_circle_top_T.header.h / 2);
  // lv_obj_t *att_img_bottom=Onboard_create_Base(parent, &att_circle_top_B);
  // lv_obj_set_pos(att_img_bottom, 0,240-att_circle_top_B.header.h/2);
  lv_obj_t *att_img_tl = Onboard_create_Base(parent, &att_circle_top_TL);
  lv_obj_set_pos(att_img_tl, -240 + att_circle_top_TL.header.w / 2, -240 + att_circle_top_TL.header.h / 2);
  lv_obj_t *att_img_tr = Onboard_create_Base(parent, &att_circle_top_TR);
  lv_obj_set_pos(att_img_tr, 240 - att_circle_top_TR.header.w / 2, -240 + att_circle_top_TR.header.h / 2);

  Screen_Attitude_Rounds[0] = att_img_top;
  // Screen_Attitude_Rounds[2]=att_img_bottom; // Fixed image does not need any rotation
  Screen_Attitude_Rounds[2] = NULL;
  Screen_Attitude_Rounds[3] = att_img_tl;
  Screen_Attitude_Rounds[1] = att_img_tr;

  // 1.1.23 Added Custom Logo on each screen
  // RB_LV_Helper_CreateImageFromFile(parent,"S:/logo.bmp");

  Screen_Attitude_RollIndicator = Onboard_create_Base(parent, &att_tri);
  lv_obj_set_pos(Screen_Attitude_RollIndicator, 0, -240 + att_tri.header.h / 2);
  // 1.1.18 Roadmap to new indicator
  lv_img_set_pivot(Screen_Attitude_RollIndicator, att_tri.header.w / 2, 240);

  // 1.1.1 Branding RB-02 on every screen
  if (true)
  {
    lv_obj_t *label = lv_label_create(parent);
    lv_obj_set_size(label, 96, 40);
    lv_obj_align(label, LV_ALIGN_CENTER, 0, -150);
    lv_obj_set_style_text_font(label, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_text(label, RB_PRODUCT_TITLE);
    lv_obj_add_style(label, &style_title, LV_STATE_DEFAULT);
  }

  // 1.1.16 Warning GPS is used
#ifdef RB_ENABLE_GPS
  if (true)
  {
    lv_obj_t *label = lv_label_create(parent);
    lv_obj_set_size(label, 120, 40);
    lv_obj_align(label, LV_ALIGN_CENTER, 0, -135);
    lv_obj_set_style_text_font(label, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_text(label, "GPS ASSISTED");
    lv_obj_add_style(label, &style_title, LV_STATE_DEFAULT);
  }
#endif
  // 1.1.13 Warning labels
  if (true)
  {
    lv_obj_t *label = lv_label_create(parent);
    lv_obj_set_size(label, 96, 40);
    lv_obj_align(label, LV_ALIGN_CENTER, 0, 215);
    lv_obj_set_style_text_font(label, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_text(label, "NOT CERTIFIED");
    lv_obj_add_style(label, &style_title, LV_STATE_DEFAULT);
  }

  // 1.1.18 Roadmap to new indicator
  // rotate_AttitudeGearByDegree(0);
}
#endif

#ifdef RB_ENABLE_TRN
static void Onboard_create_TurnSlip(lv_obj_t *parent)
{
  Onboard_create_Base(parent, &turn_coordinator);
  lv_obj_add_event_cb(parent, speedBgClicked, LV_EVENT_CLICKED, NULL);

  lv_obj_t *_screenBall = lv_obj_create(parent);
  lv_obj_set_scrollbar_mode(_screenBall, LV_SCROLLBAR_MODE_OFF);
  lv_obj_set_size(_screenBall, Screen_TurnSlip_Obj_Ball_Size, Screen_TurnSlip_Obj_Ball_Size);

  lv_obj_align(_screenBall, LV_ALIGN_CENTER, 0, 0);
  lv_obj_set_style_bg_color(_screenBall, lv_color_black(), 0);
  lv_obj_set_style_radius(_screenBall, LV_RADIUS_CIRCLE, 0);
  static lv_style_t style;
  lv_style_init(&style);
  Screen_TurnSlip_Obj_Turn = lv_img_create(parent);

  lv_img_set_src(Screen_TurnSlip_Obj_Turn, &fi_tc_airplane);
  lv_obj_set_size(Screen_TurnSlip_Obj_Turn, fi_tc_airplane.header.w, fi_tc_airplane.header.h);
  lv_style_set_img_recolor(&style, lv_color_white());

  lv_obj_align(Screen_TurnSlip_Obj_Turn, LV_ALIGN_CENTER, 0, 0);
  Screen_TurnSlip_Obj_Ball = _screenBall;
  // 1.1.1 Branding RB-02 on every screen
  if (true)
  {
    lv_obj_t *label = lv_label_create(parent);
    lv_obj_set_size(label, 96, 40);
    lv_obj_align(label, LV_ALIGN_CENTER, 0, -210);
    lv_obj_set_style_text_font(label, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_text(label, RB_PRODUCT_TITLE);
    lv_obj_add_style(label, &style_title, LV_STATE_DEFAULT);
  }
  if (true)
  {
    lv_obj_t *label = lv_label_create(parent);
    lv_obj_set_size(label, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
    lv_obj_align(label, LV_ALIGN_CENTER, 0, -100);
    lv_obj_set_style_text_font(label, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_text(label, "GYRO ASSISTED\nNOT CERTIFIED");
    lv_obj_add_style(label, &style_title, LV_STATE_DEFAULT);
  }

  if (true)
  {
    lv_obj_t *label = lv_label_create(parent);
    lv_obj_set_size(label, 96, 40);
    lv_obj_align(label, LV_ALIGN_CENTER, 0, -5);
    lv_obj_set_style_text_font(label, &lv_font_montserrat_48, 0);
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_text(label, "0");

    Screen_TurnSlip_Obj_Label = label;
  }

  // 1.1.13 Warning labels
}
#endif

#ifdef RB_ENABLE_VAR
static void Onboard_create_Variometer(lv_obj_t *parent)
{

  Onboard_create_Base(parent, &RoundVariometer);

  // 1.1.1 Branding RB-02 on every screen
  if (true)
  {
    lv_obj_t *label = lv_label_create(parent);
    lv_obj_set_size(label, 96, 40);
    lv_obj_align(label, LV_ALIGN_CENTER, 0, -200);
    lv_obj_set_style_text_font(label, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_text(label, RB_PRODUCT_TITLE);
    lv_obj_add_style(label, &style_title, LV_STATE_DEFAULT);
  }

  lv_obj_add_event_cb(parent, speedBgClicked, LV_EVENT_CLICKED, NULL);

  Screen_Variometer_Cents = lv_img_create(parent);
  lv_img_set_src(Screen_Variometer_Cents, &fi_needle);
  lv_obj_set_size(Screen_Variometer_Cents, fi_needle.header.w, fi_needle.header.h);
  lv_obj_align(Screen_Variometer_Cents, LV_ALIGN_CENTER, 0, 0);
}
#endif

#ifdef VIBRATION_TEST
/*
extern acc_scale_t acc_scale;
extern gyro_scale_t gyro_scale;
extern acc_odr_t acc_odr;
extern gyro_odr_t gyro_odr;
extern lpf_t acc_lpf;
extern lpf_t gyro_lpf;

*/

void QMI8658_Init(void);

static void event_handler_combo_Gyro_Rance(lv_event_t *e)
{
  lv_event_code_t code = lv_event_get_code(e);
  lv_obj_t *obj = lv_event_get_target(e);
  if (code == LV_EVENT_VALUE_CHANGED)
  {
    char buf[32];
    lv_dropdown_get_selected_str(obj, buf, sizeof(buf));
    LV_LOG_USER("Option: %s", buf);

    if (strcmp(buf, "16DPS") == 0)
      gyro_scale = GYR_RANGE_16DPS;
    if (strcmp(buf, "32DPS") == 0)
      gyro_scale = GYR_RANGE_32DPS;
    if (strcmp(buf, "64DPS") == 0)
      gyro_scale = GYR_RANGE_64DPS;
    if (strcmp(buf, "128DPS") == 0)
      gyro_scale = GYR_RANGE_128DPS;
    if (strcmp(buf, "256DPS") == 0)
      gyro_scale = GYR_RANGE_256DPS;
    if (strcmp(buf, "512DPS") == 0)
      gyro_scale = GYR_RANGE_512DPS;
    if (strcmp(buf, "1024DPS") == 0)
      gyro_scale = GYR_RANGE_1024DPS;

    QMI8658_Init();
  }
}
static void event_handler_combo_Acc_Rance(lv_event_t *e)
{
  lv_event_code_t code = lv_event_get_code(e);
  lv_obj_t *obj = lv_event_get_target(e);
  if (code == LV_EVENT_VALUE_CHANGED)
  {
    char buf[32];
    lv_dropdown_get_selected_str(obj, buf, sizeof(buf));
    LV_LOG_USER("Option: %s", buf);

    if (strcmp(buf, "2G") == 0)
      acc_scale = GYR_RANGE_16DPS;
    if (strcmp(buf, "4G") == 0)
      acc_scale = GYR_RANGE_32DPS;
    if (strcmp(buf, "8G") == 0)
      acc_scale = GYR_RANGE_64DPS;
    if (strcmp(buf, "16G") == 0)
      acc_scale = GYR_RANGE_128DPS;

    QMI8658_Init();
  }
}
static void event_handler_combo_Gyro_ODR(lv_event_t *e)
{
  lv_event_code_t code = lv_event_get_code(e);
  lv_obj_t *obj = lv_event_get_target(e);
  if (code == LV_EVENT_VALUE_CHANGED)
  {
    char buf[32];
    lv_dropdown_get_selected_str(obj, buf, sizeof(buf));
    LV_LOG_USER("Option: %s", buf);

    if (strcmp(buf, "8000ODR") == 0)
      gyro_odr = gyro_odr_norm_8000;
    if (strcmp(buf, "4000ODR") == 0)
      gyro_odr = gyro_odr_norm_4000;
    if (strcmp(buf, "2000ODR") == 0)
      gyro_odr = gyro_odr_norm_2000;
    if (strcmp(buf, "1000ODR") == 0)
      gyro_odr = gyro_odr_norm_1000;
    if (strcmp(buf, "500ODR") == 0)
      gyro_odr = gyro_odr_norm_500;
    if (strcmp(buf, "250ODR") == 0)
      gyro_odr = gyro_odr_norm_250;
    if (strcmp(buf, "120ODR") == 0)
      gyro_odr = gyro_odr_norm_120;
    if (strcmp(buf, "60ODR") == 0)
      gyro_odr = gyro_odr_norm_60;
    if (strcmp(buf, "30ODR") == 0)
      gyro_odr = gyro_odr_norm_30;

    QMI8658_Init();
  }
}
static void event_handler_combo_Acc_ODR(lv_event_t *e)
{
  lv_event_code_t code = lv_event_get_code(e);
  lv_obj_t *obj = lv_event_get_target(e);
  if (code == LV_EVENT_VALUE_CHANGED)
  {
    char buf[32];
    lv_dropdown_get_selected_str(obj, buf, sizeof(buf));
    LV_LOG_USER("Option: %s", buf);

    if (strcmp(buf, "8000ODR") == 0)
      acc_odr = acc_odr_norm_8000;
    if (strcmp(buf, "4000ODR") == 0)
      acc_odr = acc_odr_norm_4000;
    if (strcmp(buf, "2000ODR") == 0)
      acc_odr = acc_odr_norm_2000;
    if (strcmp(buf, "1000ODR") == 0)
      acc_odr = acc_odr_norm_1000;
    if (strcmp(buf, "500ODR") == 0)
      acc_odr = acc_odr_norm_500;
    if (strcmp(buf, "250ODR") == 0)
      acc_odr = acc_odr_norm_250;
    if (strcmp(buf, "120ODR") == 0)
      acc_odr = acc_odr_norm_120;
    if (strcmp(buf, "60ODR") == 0)
      acc_odr = acc_odr_norm_60;
    if (strcmp(buf, "30ODR") == 0)
      acc_odr = acc_odr_norm_30;

    QMI8658_Init();
  }
}
static void event_handler_combo_Gyro_LPF(lv_event_t *e)
{
  lv_event_code_t code = lv_event_get_code(e);
  lv_obj_t *obj = lv_event_get_target(e);
  if (code == LV_EVENT_VALUE_CHANGED)
  {
    char buf[32];
    lv_dropdown_get_selected_str(obj, buf, sizeof(buf));
    LV_LOG_USER("Option: %s", buf);

    if (strcmp(buf, "0LPF") == 0)
      gyro_lpf = LPF_MODE_0;
    if (strcmp(buf, "1LPF") == 0)
      gyro_lpf = LPF_MODE_1;
    if (strcmp(buf, "2LPF") == 0)
      gyro_lpf = LPF_MODE_2;
    if (strcmp(buf, "3LPF") == 0)
      gyro_lpf = LPF_MODE_3;

    QMI8658_Init();
  }
}
static void event_handler_combo_Acc_LPF(lv_event_t *e)
{
  lv_event_code_t code = lv_event_get_code(e);
  lv_obj_t *obj = lv_event_get_target(e);
  if (code == LV_EVENT_VALUE_CHANGED)
  {
    char buf[32];
    lv_dropdown_get_selected_str(obj, buf, sizeof(buf));
    LV_LOG_USER("Option: %s", buf);

    if (strcmp(buf, "0LPF") == 0)
      acc_lpf = LPF_MODE_0;
    if (strcmp(buf, "1LPF") == 0)
      acc_lpf = LPF_MODE_1;
    if (strcmp(buf, "2LPF") == 0)
      acc_lpf = LPF_MODE_2;
    if (strcmp(buf, "3LPF") == 0)
      acc_lpf = LPF_MODE_3;

    QMI8658_Init();
  }
}
#endif

#ifdef VIBRATION_TEST

static void Onboard_create_VibrationTest(lv_obj_t *parent)
{

  // Onboard_create_Base(parent, &GMeter); // 1.1.19
  for (int i = 3; i > 0; i--)
  {
    if (true)
    {
      lv_obj_t *GMeterCircle = lv_obj_create(parent);
      lv_obj_set_scrollbar_mode(GMeterCircle, LV_SCROLLBAR_MODE_OFF);
      lv_obj_set_size(GMeterCircle, 480 / 4 * i, 480 / 4 * i);
      lv_obj_align(GMeterCircle, LV_ALIGN_CENTER, 0, 0);
      lv_obj_set_style_bg_color(GMeterCircle, lv_color_black(), 0);
      lv_obj_set_style_radius(GMeterCircle, LV_RADIUS_CIRCLE, 0);
      lv_obj_clear_flag(GMeterCircle, LV_OBJ_FLAG_CLICKABLE);
    }
  }
  lv_obj_clear_flag(parent, LV_OBJ_FLAG_SCROLLABLE);

  if (true)
  {
    QMISetRangeGyroCombo = lv_dropdown_create(parent);
    lv_dropdown_set_options(QMISetRangeGyroCombo, "16DPS\n"
                                                  "32DPS\n"
                                                  "64DPS\n"
                                                  "128DPS\n"
                                                  "256DPS\n"
                                                  "512DPS\n"
                                                  "1024DPS");

    lv_obj_align(QMISetRangeGyroCombo, LV_ALIGN_CENTER, -100, -180);
    lv_obj_add_event_cb(QMISetRangeGyroCombo, event_handler_combo_Gyro_Rance, LV_EVENT_ALL, NULL);
    lv_dropdown_set_selected(QMISetRangeGyroCombo, gyro_scale);
  }
  if (true)
  {
    QMISetODRGyroCombo = lv_dropdown_create(parent);
    lv_dropdown_set_options(QMISetODRGyroCombo, "8000ODR\n"
                                                "4000ODR\n"
                                                "2000ODR\n"
                                                "1000ODR\n"
                                                "500ODR\n"
                                                "250ODR\n"
                                                "120ODR\n"
                                                "60ODR\n"
                                                "30ODR");

    lv_obj_align(QMISetODRGyroCombo, LV_ALIGN_CENTER, 30, -180);
    lv_obj_add_event_cb(QMISetODRGyroCombo, event_handler_combo_Gyro_ODR, LV_EVENT_ALL, NULL);
    lv_dropdown_set_selected(QMISetODRGyroCombo, gyro_odr);
  }
  if (true)
  {
    QMISetLPFGyroCombo = lv_dropdown_create(parent);
    lv_dropdown_set_options(QMISetLPFGyroCombo, "0LPF\n"
                                                "1LPF\n"
                                                "2LPF\n"
                                                "3LPF");

    lv_obj_align(QMISetLPFGyroCombo, LV_ALIGN_CENTER, 160, -180);
    lv_obj_add_event_cb(QMISetLPFGyroCombo, event_handler_combo_Gyro_LPF, LV_EVENT_ALL, NULL);
    lv_dropdown_set_selected(QMISetLPFGyroCombo, gyro_lpf);
  }

  if (true)
  {
    QMISetRangeAccCombo = lv_dropdown_create(parent);
    lv_dropdown_set_options(QMISetRangeAccCombo, "2G\n"
                                                 "4G\n"
                                                 "8G\n"
                                                 "16G");

    lv_obj_align(QMISetRangeAccCombo, LV_ALIGN_CENTER, -100, -145);
    lv_obj_add_event_cb(QMISetRangeAccCombo, event_handler_combo_Acc_Rance, LV_EVENT_ALL, NULL);
    lv_dropdown_set_selected(QMISetRangeAccCombo, acc_scale);
  }
  if (true)
  {
    QMISetODRACcCombo = lv_dropdown_create(parent);
    lv_dropdown_set_options(QMISetODRACcCombo, "8000ODR\n"
                                               "4000ODR\n"
                                               "2000ODR\n"
                                               "1000ODR\n"
                                               "500ODR\n"
                                               "250ODR\n"
                                               "120ODR\n"
                                               "60ODR\n"
                                               "30ODR");

    lv_obj_align(QMISetODRACcCombo, LV_ALIGN_CENTER, 30, -145);
    lv_obj_add_event_cb(QMISetODRACcCombo, event_handler_combo_Acc_ODR, LV_EVENT_ALL, NULL);
    lv_dropdown_set_selected(QMISetODRACcCombo, acc_odr);
  }
  if (true)
  {
    QMISetLPFAccCombo = lv_dropdown_create(parent);
    lv_dropdown_set_options(QMISetLPFAccCombo, "0LPF\n"
                                               "1LPF\n"
                                               "2LPF\n"
                                               "3LPF");

    lv_obj_align(QMISetLPFAccCombo, LV_ALIGN_CENTER, 160, -145);
    lv_obj_add_event_cb(QMISetLPFAccCombo, event_handler_combo_Acc_LPF, LV_EVENT_ALL, NULL);
    lv_dropdown_set_selected(QMISetLPFAccCombo, acc_lpf);
  }

  Screen_GMeter_BallGyro = lv_obj_create(parent);
  lv_obj_set_scrollbar_mode(Screen_GMeter_BallGyro, LV_SCROLLBAR_MODE_OFF);
  lv_obj_set_size(Screen_GMeter_BallGyro, 32, 32);
  lv_obj_align(Screen_GMeter_BallGyro, LV_ALIGN_CENTER, 0, 0);
  lv_obj_set_style_bg_color(Screen_GMeter_BallGyro, lv_color_make(0, 0, 0xff), 0);
  lv_obj_set_style_radius(Screen_GMeter_BallGyro, LV_RADIUS_CIRCLE, 0);

  if (true)
  {
    lv_obj_t *label = lv_label_create(parent);
    lv_obj_set_size(label, 96, 40);
    lv_obj_align(label, LV_ALIGN_CENTER, 200, -48);
    lv_obj_set_style_text_font(label, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_text(label, "GYR");
    lv_obj_add_style(label, &style_title, LV_STATE_DEFAULT);
  }
  GMeterLabelGyro = lv_label_create(parent);
  lv_label_set_text(GMeterLabelGyro, "--");
  lv_obj_set_size(GMeterLabelGyro, 400, 48);
  lv_obj_align(GMeterLabelGyro, LV_ALIGN_CENTER, 0, -48);
  lv_obj_set_style_text_align(GMeterLabelGyro, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_set_style_text_font(GMeterLabelGyro, &lv_font_montserrat_48, 0);
  lv_obj_add_style(GMeterLabelGyro, &style_title, LV_STATE_DEFAULT);

  // lv_obj_align(XXX, LV_ALIGN_CENTER, 0, +48);

  Screen_Vibration_Yaw_Ball = lv_obj_create(parent);
  lv_obj_set_scrollbar_mode(Screen_Vibration_Yaw_Ball, LV_SCROLLBAR_MODE_OFF);
  lv_obj_set_size(Screen_Vibration_Yaw_Ball, 32, 32);
  lv_obj_align(Screen_Vibration_Yaw_Ball, LV_ALIGN_CENTER, 0, 0);
  lv_obj_set_style_bg_color(Screen_Vibration_Yaw_Ball, lv_color_make(0xff, 0, 0), 0);
  lv_obj_set_style_radius(Screen_Vibration_Yaw_Ball, LV_RADIUS_CIRCLE, 0);

  Screen_Vibration_Accel_Ball = lv_obj_create(parent);
  lv_obj_set_scrollbar_mode(Screen_Vibration_Accel_Ball, LV_SCROLLBAR_MODE_OFF);
  lv_obj_set_size(Screen_Vibration_Accel_Ball, 32, 32);
  lv_obj_align(Screen_Vibration_Accel_Ball, LV_ALIGN_CENTER, 0, 0);
  lv_obj_set_style_bg_color(Screen_Vibration_Accel_Ball, lv_color_white(), 0);
  lv_obj_set_style_radius(Screen_Vibration_Accel_Ball, LV_RADIUS_CIRCLE, 0);

  if (true)
  {
    lv_obj_t *label = lv_label_create(parent);
    lv_obj_set_size(label, 96, 40);
    lv_obj_align(label, LV_ALIGN_CENTER, 200, 0);
    lv_obj_set_style_text_font(label, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_text(label, "ACC");
    lv_obj_add_style(label, &style_title, LV_STATE_DEFAULT);
  }
  Screen_Vibration_Accel_Label = lv_label_create(parent);
  lv_label_set_text(Screen_Vibration_Accel_Label, "--");
  lv_obj_set_size(Screen_Vibration_Accel_Label, 400, 48);
  lv_obj_align(Screen_Vibration_Accel_Label, LV_ALIGN_CENTER, 0, 0);
  lv_obj_set_style_text_align(Screen_Vibration_Accel_Label, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_set_style_text_font(Screen_Vibration_Accel_Label, &lv_font_montserrat_48, 0);
  lv_obj_add_style(Screen_Vibration_Accel_Label, &style_title, LV_STATE_DEFAULT);

  if (true)
  {
    lv_obj_t *label = lv_label_create(parent);
    lv_obj_set_size(label, 96, 40);
    lv_obj_align(label, LV_ALIGN_CENTER, 200, 48 * 2);
    lv_obj_set_style_text_font(label, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_text(label, "IMU");
    lv_obj_add_style(label, &style_title, LV_STATE_DEFAULT);
  }
  Screen_Vibration_IMU_Label = lv_label_create(parent);
  lv_label_set_text(Screen_Vibration_IMU_Label, "--");
  lv_obj_set_size(Screen_Vibration_IMU_Label, 400, 48);
  lv_obj_align(Screen_Vibration_IMU_Label, LV_ALIGN_CENTER, 0, 48 * 2);
  lv_obj_set_style_text_align(Screen_Vibration_IMU_Label, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_set_style_text_font(Screen_Vibration_IMU_Label, &lv_font_montserrat_48, 0);
  lv_obj_add_style(Screen_Vibration_IMU_Label, &style_title, LV_STATE_DEFAULT);

  Screen_Vibration_GPSAccel_Ball = lv_obj_create(parent);
  lv_obj_set_scrollbar_mode(Screen_Vibration_GPSAccel_Ball, LV_SCROLLBAR_MODE_OFF);
  lv_obj_set_size(Screen_Vibration_GPSAccel_Ball, 32, 32);
  lv_obj_align(Screen_Vibration_GPSAccel_Ball, LV_ALIGN_CENTER, 0, 0);
  lv_obj_set_style_bg_color(Screen_Vibration_GPSAccel_Ball, lv_color_make(0, 0xff, 0), 0);
  lv_obj_set_style_radius(Screen_Vibration_GPSAccel_Ball, LV_RADIUS_CIRCLE, 0);

  if (true)
  {
    lv_obj_t *label = lv_label_create(parent);
    lv_obj_set_size(label, 96, 40);
    lv_obj_align(label, LV_ALIGN_CENTER, 200, 48);
    lv_obj_set_style_text_font(label, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_text(label, "GPS");
    lv_obj_add_style(label, &style_title, LV_STATE_DEFAULT);
  }

  Screen_Vibration_GPSAccel_Label = lv_label_create(parent);
  lv_label_set_text(Screen_Vibration_GPSAccel_Label, "--");
  lv_obj_set_size(Screen_Vibration_GPSAccel_Label, 400, 48);
  lv_obj_align(Screen_Vibration_GPSAccel_Label, LV_ALIGN_CENTER, 0, 48);
  lv_obj_set_style_text_align(Screen_Vibration_GPSAccel_Label, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_set_style_text_font(Screen_Vibration_GPSAccel_Label, &lv_font_montserrat_48, 0);
  lv_obj_add_style(Screen_Vibration_GPSAccel_Label, &style_title, LV_STATE_DEFAULT);

  lv_obj_add_event_cb(parent, speedBgClicked, LV_EVENT_CLICKED, NULL);
}
#endif

#ifdef RB_ENABLE_GMT
static void Onboard_create_GMeter(lv_obj_t *parent)
{
  // Onboard_create_Base(parent, &GMeter);
  lv_obj_clear_flag(parent, LV_OBJ_FLAG_SCROLLABLE);

  // 1.1.1 Branding RB-02 on every screen
  if (true)
  {
    lv_obj_t *label = lv_label_create(parent);
    lv_obj_set_size(label, 96, 40);
    lv_obj_align(label, LV_ALIGN_CENTER, 0, -200);
    lv_obj_set_style_text_font(label, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_text(label, RB_PRODUCT_TITLE);
    lv_obj_add_style(label, &style_title, LV_STATE_DEFAULT);
  }

  for (int i = 3; i > 0; i--)
  {
    if (true)
    {
      lv_obj_t *GMeterCircle = lv_obj_create(parent);
      lv_obj_set_scrollbar_mode(GMeterCircle, LV_SCROLLBAR_MODE_OFF);
      lv_obj_set_size(GMeterCircle, 480 / 4 * i, 480 / 4 * i);
      lv_obj_align(GMeterCircle, LV_ALIGN_CENTER, 0, 0);
      lv_obj_set_style_bg_color(GMeterCircle, lv_color_black(), 0);
      lv_obj_set_style_radius(GMeterCircle, LV_RADIUS_CIRCLE, 0);
      lv_obj_clear_flag(GMeterCircle, LV_OBJ_FLAG_CLICKABLE);
    }
  }

  lv_obj_add_event_cb(parent, speedBgClicked, LV_EVENT_CLICKED, NULL);

  Screen_GMeter_BallMax = lv_obj_create(parent);
  lv_obj_set_scrollbar_mode(Screen_GMeter_BallMax, LV_SCROLLBAR_MODE_OFF);
  lv_obj_set_size(Screen_GMeter_BallMax, 32, 32);
  lv_obj_align(Screen_GMeter_BallMax, LV_ALIGN_CENTER, 0, 0);
  lv_obj_set_style_bg_color(Screen_GMeter_BallMax, lv_color_make(0xff, 0, 0), 0);
  lv_obj_set_style_radius(Screen_GMeter_BallMax, LV_RADIUS_CIRCLE, 0);

  Screen_GMeter_Ball = lv_obj_create(parent);
  lv_obj_set_scrollbar_mode(Screen_GMeter_Ball, LV_SCROLLBAR_MODE_OFF);
  lv_obj_set_size(Screen_GMeter_Ball, 32, 32);
  lv_obj_align(Screen_GMeter_Ball, LV_ALIGN_CENTER, 0, 0);
  lv_obj_set_style_bg_color(Screen_GMeter_Ball, lv_color_white(), 0);
  lv_obj_set_style_radius(Screen_GMeter_Ball, LV_RADIUS_CIRCLE, 0);

  GMeterLabel = lv_label_create(parent);
  lv_label_set_text(GMeterLabel, "--");
  lv_obj_set_size(GMeterLabel, 400, 48);
  lv_obj_align(GMeterLabel, LV_ALIGN_CENTER, 0, 0);
  lv_obj_set_style_text_align(GMeterLabel, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_set_style_text_font(GMeterLabel, &lv_font_montserrat_48, 0);
  lv_obj_add_style(GMeterLabel, &style_title, LV_STATE_DEFAULT);

  GMeterLabelMax = lv_label_create(parent);
  lv_label_set_text(GMeterLabelMax, "+1.0 -0.0");
  lv_obj_set_size(GMeterLabelMax, 400, 48);
  lv_obj_align(GMeterLabelMax, LV_ALIGN_CENTER, 0, 200);
  lv_obj_set_style_text_align(GMeterLabelMax, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_set_style_text_font(GMeterLabelMax, &lv_font_montserrat_48, 0);
  lv_obj_add_style(GMeterLabelMax, &style_title, LV_STATE_DEFAULT);
}
#endif

void turnOnOffDigitsSymbols(lv_obj_t **segments, uint8_t symbol)
{
  switch (symbol)
  {
  case 0: // -
    lv_obj_clear_flag(segments[0], LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(segments[1], LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(segments[2], LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(segments[3], LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(segments[4], LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(segments[5], LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(segments[6], LV_OBJ_FLAG_HIDDEN);
    break;
  };
}

void turnOnOffDigits(lv_obj_t **segments, uint8_t number)
{
  switch (number)
  {
  case 0:
    lv_obj_add_flag(segments[0], LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(segments[1], LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(segments[2], LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(segments[3], LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(segments[4], LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(segments[5], LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(segments[6], LV_OBJ_FLAG_HIDDEN);
    break;
  case 1:
    lv_obj_add_flag(segments[0], LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(segments[1], LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(segments[2], LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(segments[3], LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(segments[4], LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(segments[5], LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(segments[6], LV_OBJ_FLAG_HIDDEN);
    break;
  case 2:
    lv_obj_clear_flag(segments[0], LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(segments[1], LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(segments[2], LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(segments[3], LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(segments[4], LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(segments[5], LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(segments[6], LV_OBJ_FLAG_HIDDEN);
    break;
  case 3:
    lv_obj_clear_flag(segments[0], LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(segments[1], LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(segments[2], LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(segments[3], LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(segments[4], LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(segments[5], LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(segments[6], LV_OBJ_FLAG_HIDDEN);
    break;
  case 4:
    lv_obj_clear_flag(segments[0], LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(segments[1], LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(segments[2], LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(segments[3], LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(segments[4], LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(segments[5], LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(segments[6], LV_OBJ_FLAG_HIDDEN);
    break;
  case 5:
    lv_obj_clear_flag(segments[0], LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(segments[1], LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(segments[2], LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(segments[3], LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(segments[4], LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(segments[5], LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(segments[6], LV_OBJ_FLAG_HIDDEN);
    break;
  case 6:
    lv_obj_clear_flag(segments[0], LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(segments[1], LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(segments[2], LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(segments[3], LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(segments[4], LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(segments[5], LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(segments[6], LV_OBJ_FLAG_HIDDEN);
    break;
  case 7:
    lv_obj_add_flag(segments[0], LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(segments[1], LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(segments[2], LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(segments[3], LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(segments[4], LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(segments[5], LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(segments[6], LV_OBJ_FLAG_HIDDEN);
    break;
  case 8:
    lv_obj_clear_flag(segments[0], LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(segments[1], LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(segments[2], LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(segments[3], LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(segments[4], LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(segments[5], LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(segments[6], LV_OBJ_FLAG_HIDDEN);
    break;
  case 9:
    lv_obj_clear_flag(segments[0], LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(segments[1], LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(segments[2], LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(segments[3], LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(segments[4], LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(segments[5], LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(segments[6], LV_OBJ_FLAG_HIDDEN);
    break;
  default:
    lv_obj_add_flag(segments[0], LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(segments[1], LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(segments[2], LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(segments[3], LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(segments[4], LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(segments[5], LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(segments[6], LV_OBJ_FLAG_HIDDEN);
    break;
  }
}

void update_AltimeterDigital_lvgl_tick(lv_timer_t *t)
{
  // 1.0.9 Altimter is *100 Feet
  int32_t AltimeterAbsolute = Altimeter / 100.0;
  if (Altimeter < 0)
  {
    AltimeterAbsolute = -AltimeterAbsolute;
    // TODO set minus
  }

  int k = 1;
  bool firstZero = false;
  for (int c = 4; c >= 0; c--)
  {
    int v = (AltimeterAbsolute / k);
    if (v > 0)
    {
      turnOnOffDigits(SegmentsAltDigit[c], v % 10);
      firstZero = false;
    }
    else
    {
      if (firstZero == false && Altimeter < 0)
      {
        turnOnOffDigitsSymbols(SegmentsAltDigit[c], 0);
      }
      else
      {
        turnOnOffDigits(SegmentsAltDigit[c], -1);
      }
      firstZero = true;
    }
    k = k * 10;
  }
  /*
  turnOnOffDigits(SegmentsAltDigit[0], (AltimeterAbsolute / 10000)%10);
  turnOnOffDigits(SegmentsAltDigit[1], (AltimeterAbsolute / 1000)%10);
  turnOnOffDigits(SegmentsAltDigit[2], (AltimeterAbsolute / 100)%10);
  turnOnOffDigits(SegmentsAltDigit[3], (AltimeterAbsolute / 10)%10);
  turnOnOffDigits(SegmentsAltDigit[4], AltimeterAbsolute % 10);
  */

  char buf[10];
  // 1.0.9 Variometer is *100 and 1Hz
  // Variometer => 0.01 Feet/Second => *60/100 => Feet/min
  snprintf(buf, sizeof(buf), "%+ld", Variometer);
  lv_label_set_text(Screen_Altitude_Variometer2, buf);

#ifdef RB_ENABLE_GPS
  snprintf(buf, sizeof(buf), "%.0f", singletonConfig()->NMEA_DATA.altitude * 3.28084);
  lv_label_set_text(Screen_Altitude_Pressure, buf);
#else
  snprintf(buf, sizeof(buf), "%.02f", bmp280Pressure / 100.0);
  lv_label_set_text(Screen_Altitude_Pressure, buf);
#endif
}

uint32_t timerDiffByIndex(int st)
{

  datetime_t datetimeTimer = {0};

  switch (st)
  {
  case 0:
    datetimeTimer = datetimeTimer1;
    break;
  case 1:
    datetimeTimer = datetimeTimer2;
    break;
  case 2:
    datetimeTimer = datetimeTimer3;
    break;

  default:
    break;
  }

  uint32_t epochA = datetime.second + 60 * datetime.minute + 60 * 60 * datetime.hour;
  uint32_t epochB = datetimeTimer.second + 60 * datetimeTimer.minute + 60 * 60 * datetimeTimer.hour;
  uint32_t diff = epochA - epochB;

  return diff;
}

void update_Clock_lvgl_tick(lv_timer_t *t)
{
  uint32_t diff = timerDiffByIndex(selectedTimer);

  turnOnOffDigits(SegmentsA[0], ((diff / 60) / 10) % 10);
  turnOnOffDigits(SegmentsA[1], ((diff / 60)) % 10);
  turnOnOffDigits(SegmentsA[2], ((diff % 60) / 10) % 10);
  turnOnOffDigits(SegmentsA[3], (diff % 60) % 10);
  char buf[20];
  uint32_t diffSW = 0;
  uint32_t diffSE = 0;
  switch (selectedTimer)
  {
  case 0:
    diffSW = timerDiffByIndex(1);
    sprintf(buf, "%02lu:%02lu", (diffSW / 60), (diffSW % 60));
    lv_label_set_text(TimerSW, buf);
    diffSE = timerDiffByIndex(2);
    sprintf(buf, "%02lu:%02lu", (diffSE / 60), (diffSE % 60));
    lv_label_set_text(TimerSE, buf);
    break;
  case 1:
    diffSW = timerDiffByIndex(0);
    sprintf(buf, "%02lu:%02lu", (diffSW / 60), (diffSW % 60));
    lv_label_set_text(TimerSW, buf);
    diffSE = timerDiffByIndex(2);
    sprintf(buf, "%02lu:%02lu", (diffSE / 60), (diffSE % 60));
    lv_label_set_text(TimerSE, buf);
    break;
  case 2:
    diffSW = timerDiffByIndex(0);
    sprintf(buf, "%02lu:%02lu", (diffSW / 60), (diffSW % 60));
    lv_label_set_text(TimerSW, buf);
    diffSE = timerDiffByIndex(1);
    sprintf(buf, "%02lu:%02lu", (diffSE / 60), (diffSE % 60));
    lv_label_set_text(TimerSE, buf);
    break;
  }

  // 1.1.17 Display UTC Clock on the Timer 3 slot
#ifdef RB_ENABLE_GPS
  sprintf(buf, "%02u:%02u", singletonConfig()->NMEA_DATA.tim.hour, singletonConfig()->NMEA_DATA.tim.minute);
  lv_label_set_text(TimerSE, buf);
#endif
}

#ifdef VIBRATION_TEST
void update_Vibration_lvgl_tick(lv_timer_t *t)
{
  char buf[100];
  lv_obj_set_pos(Screen_Vibration_GPSAccel_Ball, -220 * GPSLateralYAcceleration / GMeterScale, 220 * GPSAccelerationForAttitudeCompensation / GMeterScale);
  lv_obj_set_pos(Screen_Vibration_Accel_Ball, -220 * AccelFiltered.y / GMeterScale, 220 * AccelFiltered.x / GMeterScale);
  lv_obj_set_pos(Screen_GMeter_BallGyro, 220 * (GyroFiltered.z + GyroBias.z) / GMeterScaleGyro, 220 * (GyroFiltered.y + GyroBias.y) / GMeterScaleGyro);
  lv_obj_set_pos(Screen_Vibration_Yaw_Ball, -220 * (GyroFiltered.x + GyroBias.x) / GMeterScaleGyro, 220 * AccelFiltered.z / GMeterScale);
  snprintf(buf, sizeof(buf), "%2.0f %2.0f %2.0f", (GyroFiltered.x + GyroBias.x), (GyroFiltered.y + GyroBias.y), (GyroFiltered.z + GyroBias.z));
  lv_label_set_text(GMeterLabelGyro, buf);

  if (GMeterScaleGyro < GyroFiltered.y)
    GMeterScaleGyro = (GyroFiltered.y + GyroBias.y);
  if (GMeterScaleGyro < GyroFiltered.x)
    GMeterScaleGyro = (GyroFiltered.x + GyroBias.x);
  if (GMeterScaleGyro < GyroFiltered.z)
    GMeterScaleGyro = (GyroFiltered.z + GyroBias.z);

  snprintf(buf, sizeof(buf), "%.1f %.1f %.1f", AccelFiltered.x, AccelFiltered.y, AccelFiltered.z);
  lv_label_set_text(Screen_Vibration_Accel_Label, buf);

  snprintf(buf, sizeof(buf), "%.0f %.0f %.0f", AttitudeRoll, AttitudePitch, AttitudeYaw);
  lv_label_set_text(Screen_Vibration_IMU_Label, buf);

  snprintf(buf, sizeof(buf), "%.1f %.1f %.0f", GPSAccelerationForAttitudeCompensation, GPSLateralYAcceleration, FusionAHRSDeltaTrackFromGPS);
  lv_label_set_text(Screen_Vibration_GPSAccel_Label, buf);
}
#endif

void update_GMeter_lvgl_tick(lv_timer_t *t)
{
  lv_obj_set_pos(Screen_GMeter_Ball, -180 * AccelFiltered.y / GMeterScale, 180 * AccelFiltered.x / GMeterScale);
  char buf[100]; // +99.9 -99.9
  snprintf(buf, sizeof(buf), "%.1f", GFactor);
  lv_label_set_text(GMeterLabel, buf);

  /*
    float BeepFactor = GMeterScale / 3.0 * 2.0;
    if (GFactor > BeepFactor)
    {
      Buzzer_On();
    }
    else
    {
      Buzzer_Off();
    }
  */
  // TODO flag MAX Dirty
  lv_obj_set_pos(Screen_GMeter_BallMax, -60 * AccelFilteredMax.y, 60 * AccelFilteredMax.x);
  snprintf(buf, sizeof(buf), "+%.1f %.1f", GFactorMax, GFactorMin);
  lv_label_set_text(GMeterLabelMax, buf);
}
