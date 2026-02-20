
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

#include "BMP280.h"
#include "RB02_DriverFactory.h"
#include "I2C_Driver.h"
#define BMP280_S64_t int64_t
#define BMP280_U32_t uint32_t
#define BMP280_S32_t int32_t
int32_t bmp280Calibration[12];
BMP280_S32_t t_fine = 0;
extern int32_t bmp280Pressure;
extern int32_t bmp280Temperature;
extern int32_t Altimeter;
extern int32_t Variometer;

void BMP280_Init(void)
{
    // BMP280
    uint8_t bmp280BufferReset[1] = {0xB6};
    I2C_Write(singletonConfig()->bmp280Address, 0xE0, &bmp280BufferReset[0], 1);
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

extern uint8_t Operative_BMP280;
extern float QNH;

void Get_BMP280(void)
{
  uint8_t buf[6] = {0, 0, 0, 0, 0, 0};
  I2C_Read(singletonConfig()->bmp280Address, 0xF7, &buf[0], 6);

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

void bmp280Setup()
{
  uint8_t bmp280Control[1] = {0x57};
  uint8_t bmp280Settings[1] = {0x9C};
  I2C_Write(singletonConfig()->bmp280Address, 0xF4, &bmp280Control[0], 1);
  I2C_Write(singletonConfig()->bmp280Address, 0xF5, &bmp280Settings[0], 1);
}

void bmp280readCalibration()
{
  uint8_t buf[24] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  I2C_Read(singletonConfig()->bmp280Address, 0x88, &buf[0], 24);
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
  Get_BMP280();
  Variometer = 0;
}
