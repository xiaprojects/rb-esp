
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
 * 06 -> Display with Android 6.25" 7" 8" 10" 10.2"
 *
 * Community edition will be free for all builders and personal use as defined by the licensing model
 * Dual licensing for commercial agreement is available
 *
*/
#include "RB02_DriverFactory.h"

IMUdata Accel;
IMUdata Gyro;

IMUdata AccelFiltered;
IMUdata AccelFilteredWithoutPanelAlignment;
IMUdata AccelFilteredMax;
IMUdata GyroFiltered;
IMUdata GyroFilteredWithoutPanelAlignment;
IMUdata GyroBias;
IMUdata PanelAlignment;

float GPSLateralYAcceleration = 0;
float GPSAccelerationForAttitudeCompensation = 0.0;
float GFactor = 1.0;
float GFactorMax = 0;
float GFactorMin = 1.0;
uint8_t GFactorDirty = 0;
float AttitudePitch = 0;
float AttitudeRoll = 0;
float AttitudeYaw = 0;
float AttitudeYawDegreePerSecond = 0;
// 1.1.8 Improve reliabilty
float FilterMoltiplier = 3.0;
float FilterMoltiplierOutput = 5.0;
float FilterMoltiplierGyro = 10.0;
uint32_t SDCard_Size = 0;
float AttitudeBalanceAlpha = 1.0 / 250.0;
float AttitudeBalanceAlphaAerobatics = 1.0 / 250.0;
datetime_t datetime= {0};
int16_t TouchPadLastX;
int16_t TouchPadLastY;

uint8_t DriverLoopMilliseconds = 40;


/**
 * TODO We shall move the source code in C++ or into structuct-Obj pointers to make a dinamic function to manage the data
 */

#ifdef RB_ENABLE_NavCore
// ---- NavCore: Enable the Air Speed
#include "ms4525do_rb.h"
#endif

#ifdef RB_ENABLE_BME680
#include "BME680.h"
#ifdef RB_ENABLE_CODETECTOR
#endif
#endif


#ifdef RB_ENABLE_NavCore
#endif
#ifdef RB_ENABLE_IAS
#endif
#ifdef RB02_BUTTON_KNOB
#endif
#ifdef RB_ENABLE_MODBUS
#endif
#ifdef RB_ENABLE_CAN
#endif
#ifdef RB_ENABLE_BMI270
#endif

/* Integration shall provide the following functions */
#ifdef RB_NOT_COMPLETED_INTEGRATION
uint8_t LCD_Backlight = 0;
float BAT_analogVolts = 0;

void Set_Backlight(uint8_t Light)
{
    LCD_Backlight = Light;
}

void Buzzer_On()
{

}

void Buzzer_Off()
{

}
#endif
#ifdef RB_ENABLE_BMP280
#include "BMP280.h"
void Get_BMP280(void);   // Declaration
#endif

#ifdef RB_ENABLE_QMI8658
#else
void gyroHardwareCalibrationToZero()
{
}
void gyroHardwareSetCalibration(float x, float y, float z)
{
}
#endif

#ifdef RB_ENABLE_PCF85063
#else
void RTC_Loop(void)
{
}
#endif


#ifdef RB_ENABLE_BMP280
#include "BMP280.h"
void Barometer_Init(void)
{
    BMP280_Init();
}
void Barometer_Setup(void)
{
    bmp280Setup();
}
void Barometer_ReadCalibration(void)
{
    bmp280readCalibration();
}
void Barometer_Update(void)
{
    Get_BMP280();
}
#else
void Barometer_Init(void)
{

}
void Barometer_Setup(void)
{
}
void Barometer_Update(void)
{
}
void Barometer_ReadCalibration(void)
{
}
#endif


#ifdef RB_ENABLE_SD_MMC
#else
void Flash_Searching(void)
{
}
void SD_Init(void)
{
}
#endif


void DriverFactory_Init(void) {
#ifdef RB_ENABLE_NavCore
    MS4525DO_Init();
#endif
#ifdef RB_ENABLE_QMI8658
    QMI8658_Init();
#endif
#ifdef RB_ENABLE_BMP280
    // TODO move initialisation vector from RB02.c to here
#endif
#ifdef RB_ENABLE_BME680
#endif
}

/**
 * Driver Loop for Attitude, this shall fetch all the data from chip and invoke the Matrix alignment
 */
void Driver_Attitude_Loop(void) {
#ifdef RB_ENABLE_QMI8658
        QMI8658_Loop();
#endif
#ifdef RB_ENABLE_BMI270
        BMI270_Loop();
#endif
    // TODO Move the Matrix management into a dedicated function (written in DriverAttitude.c)
}


/**
 * Driver Loop for Barometer
 * RB-02 used for static pressure
 * RB-04 used to compensate the MAP
 * RB-05 used for FL 1013 comparison
 */
void Driver_Barometer_Loop(void) {
#ifdef RB_ENABLE_BMP280
    Get_BMP280();
#endif
#ifdef RB_ENABLE_BME680
#endif
}