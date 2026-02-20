#pragma once
#include "RB02.h"

extern IMUdata Accel;
extern IMUdata Gyro;
extern IMUdata AccelFiltered;
extern IMUdata GyroFiltered;
extern IMUdata AccelFilteredWithoutPanelAlignment;
extern IMUdata AccelFilteredMax;
extern IMUdata GyroFilteredWithoutPanelAlignment;
extern IMUdata GyroBias;
extern IMUdata PanelAlignment;
extern float AttitudeBalanceAlpha;
extern float GFactor;
extern float GFactorMax;
extern float GFactorMin;
extern float AttitudePitch;
extern float AttitudeRoll;
extern float AttitudeYaw;
extern float AttitudeYawDegreePerSecond;
extern float FilterMoltiplier;
extern float FilterMoltiplierOutput;
extern float FilterMoltiplierGyro;
extern float GPSLateralYAcceleration;
extern float GPSAccelerationForAttitudeCompensation;
extern uint8_t GFactorDirty;
extern uint32_t SDCard_Size;
extern datetime_t datetime;


#ifdef RB_ENABLE_QMI8658
#include "QMI8658.h"
#else
#endif


void RTC_Loop(void);
void Flash_Searching(void);
void SD_Init(void);
void gyroHardwareCalibrationToZero();
void gyroHardwareSetCalibration(float x, float y, float z);
void Barometer_Init(void);
void Barometer_Update(void);
void Barometer_Setup(void);
void Barometer_ReadCalibration(void);
void Buzzer_On();
void Buzzer_Off();

#ifdef RB_ENABLE_PCF85063
void PCF85063_Init(void);
#else

#endif