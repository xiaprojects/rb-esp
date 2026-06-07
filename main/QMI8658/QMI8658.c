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
 * - ESP32-S3 2.8" Inch Round display 480x480
 * - https://www.waveshare.com/esp32-s3-touch-lcd-2.8c.htm
 *
 *
 *  Roadmap
 * 1) Move away the attitude algs and keep this file only as sensor reader
 * 2) Add Magnetometer
 * 3) Get Attitude from RB-01
 * 
 * 
 * Console output for USB connection to RB-01 and RB-03:
 * $RBATT,roll,pitch,yaw,accelx,accely,accelz,gyrox,gyroy,gyroz,gpsx,gpsy,gpsz,magx,magy,magz,compass,beta*checksum
 *
 */
#include "QMI8658.h"
#include "lvgl.h"
#include "madgwick.h"
#include "nvs.h"
#include <math.h>
#include "RB02_Config.h"
#include "RB02_DriverFactory.h"
extern uint8_t EnableAttitudeMadgwick;
extern IMUdata Accel;
extern IMUdata Gyro;
extern IMUdata AccelFiltered;
extern IMUdata GyroFiltered;

extern float GFactor;
extern float GFactorMax;
extern float GFactorMin;
extern float AttitudePitch;
extern float AttitudeRoll;
extern float AttitudeYaw;
extern float AttitudeYawDegreePerSecond;

// 1.1.30 HW Calibration IMUdata GyroCalibration;
// Adjustable parameter: lower = slower yaw correction
#define YAW_CORRECTION_GAIN 0.01f
#define YAW_RATE_SMOOTHING_ALPHA 0.01

extern int64_t GPSLastSpeedKmhReceivedTick;

uint16_t lv_atan2(int x, int y);
// Define complementary filter constant (adjust as needed)

#define DT 0.1 // Time step HZ

uint8_t Device_addr; // default for SD0/SA0 low, 0x6A if high
// 1.7.0 Aerobatic grade
acc_scale_t acc_scale = ACC_RANGE_8G;
gyro_scale_t gyro_scale = GYR_RANGE_128DPS;
acc_odr_t acc_odr = acc_odr_norm_30;
gyro_odr_t gyro_odr = gyro_odr_norm_30;
sensor_state_t sensor_state = sensor_default;
lpf_t acc_lpf = LPF_MODE_0;
lpf_t gyro_lpf = LPF_MODE_0;

float accelScales, gyroScales;
float accelScales = 0;
uint8_t readings[12];
uint32_t reading_timestamp_us; // timestamp in arduino micros() time

// Matrix rotation for the panel mis-alignment
// TODO: 20250925 Flight TEST! as a backup if 0 skip
void PanelAlignmentMatrixApply(IMUdata aS, IMUdata *aB_out, IMUdata ref);

void gyroHardwareCalibrationToZero()
{
    uint8_t prev_ctrl7 = QMI8658_receive(QMI8658_CTRL7);
#ifdef RB_ENABLE_CONSOLE_DEBUG
    printf("Gyro Calibration to Zero... %02X\n", prev_ctrl7);
#endif
    QMI8658_transmit(QMI8658_CTRL7, 0x00);
    QMI8658_CTRL9_Write(0xA2);
    QMI8658_transmit(QMI8658_CTRL7, prev_ctrl7);
}

void gyroHardwareSetCalibration(float x, float y, float z)
{
    /* Save previous CTRL7, clear it to allow calibration writes */
    uint8_t prev_ctrl7 = QMI8658_receive(QMI8658_CTRL7);
#ifdef RB_ENABLE_CONSOLE_DEBUG
    printf("Gyro Calibration to... %f %f %f\n", x, y, z);
#endif    
    QMI8658_transmit(QMI8658_CTRL7, 0x00);
    uint16_t regval[3];

    // --------- Gyro host delta offset ----------

    int16_t valx = (int16_t)lrintf(x * 32.0f);
    regval[0] = (uint16_t)valx;
    int16_t valy = (int16_t)lrintf(y * 32.0f);
    regval[1] = (uint16_t)valy;
    int16_t valz = (int16_t)lrintf(z * 32.0f);
    regval[2] = (uint16_t)valz;

    // Write CAL1..3
    QMI8658_transmit(QMI8658_CAL1_L, regval[0] & 0xFF);
    QMI8658_transmit(QMI8658_CAL1_H, regval[0] >> 8);
    QMI8658_transmit(QMI8658_CAL2_L, regval[1] & 0xFF);
    QMI8658_transmit(QMI8658_CAL2_H, regval[1] >> 8);
    QMI8658_transmit(QMI8658_CAL3_L, regval[2] & 0xFF);
    QMI8658_transmit(QMI8658_CAL3_H, regval[2] >> 8);

    // Issue command to apply gyro delta offset
    QMI8658_CTRL9_Write(0x0A);

    /* Restore previous CTRL7 */
    QMI8658_transmit(QMI8658_CTRL7, prev_ctrl7);


#ifdef RB_ENABLE_CONSOLE_DEBUG
    /* Dump gyro config and calibration registers for debugging */
    {
        uint8_t ctrl3 = QMI8658_receive(QMI8658_CTRL3);
        uint8_t ctrl7 = QMI8658_receive(QMI8658_CTRL7);
        uint8_t cal1l = QMI8658_receive(QMI8658_CAL1_L);
        uint8_t cal1h = QMI8658_receive(QMI8658_CAL1_H);
        uint8_t cal2l = QMI8658_receive(QMI8658_CAL2_L);
        uint8_t cal2h = QMI8658_receive(QMI8658_CAL2_H);
        uint8_t cal3l = QMI8658_receive(QMI8658_CAL3_L);
        uint8_t cal3h = QMI8658_receive(QMI8658_CAL3_H);
        int16_t cal1 = (int16_t)((cal1h << 8) | cal1l);
        int16_t cal2 = (int16_t)((cal2h << 8) | cal2l);
        int16_t cal3 = (int16_t)((cal3h << 8) | cal3l);
        printf("QMI8658 Debug - CTRL3=0x%02X CTRL7=0x%02X CAL1=%d CAL2=%d CAL3=%d\n", ctrl3, ctrl7, cal1, cal2, cal3);
    }
#endif
}
/**
 * Inialize Wire and send default configs
 * @param addr I2C address of sensor, typically 0x6A or 0x6B
 */
void QMI8658_Init(void)
{
    uint8_t buf[1];
    Device_addr = QMI8658_L_SLAVE_ADDRESS;
    I2C_Read(Device_addr, QMI8658_REVISION_ID, buf, 1);
    //printf("QMI8658 Device ID: %x\r\n", buf[0]); // Get chip id
    setState(sensor_running);

    float sampleFreq = 20; // 1Hz ODR 60Hz 2%, Loop is 20Hz

    // 1.1.12 Enable possibility to setup motion
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open("storage", NVS_READONLY, &my_handle);
    if (err != ESP_OK)
    {
        printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
    }
    else
    {
        //printf("QMI8658 Restore setup from NVM:\n");
        uint8_t nvm_acc_scale = acc_scale;
        uint8_t nvm_acc_odr = acc_odr;
        uint8_t nvm_acc_lpf = acc_lpf;
        uint8_t nvm_gyro_scale = gyro_scale;
        uint8_t nvm_gyro_lpf = gyro_lpf;
        uint8_t nvm_gyro_odr = gyro_odr;

        nvs_get_u8(my_handle, "nvm_acc_scale", &nvm_acc_scale);
        nvs_get_u8(my_handle, "nvm_acc_odr", &nvm_acc_odr);
        nvs_get_u8(my_handle, "nvm_acc_lpf", &nvm_acc_lpf);
        nvs_get_u8(my_handle, "nvm_gyro_scale", &nvm_gyro_scale);
        nvs_get_u8(my_handle, "nvm_gyro_lpf", &nvm_gyro_lpf);
        nvs_get_u8(my_handle, "nvm_gyro_odr", &nvm_gyro_odr);

        uint8_t intBuffer = 99;
        nvs_get_u8(my_handle, "filterAttitude", &intBuffer);

        /*
        printf("filterAttitude: %d\n", intBuffer);
        printf("nvm_acc_scale: %d\n", nvm_acc_scale);
        printf("nvm_acc_odr: %d\n", nvm_acc_odr);
        printf("nvm_acc_lpf: %d\n", nvm_acc_lpf);
        printf("nvm_gyro_scale: %d\n", nvm_gyro_scale);
        printf("nvm_gyro_odr: %d\n", nvm_gyro_odr);
        printf("nvm_gyro_lpf: %d\n", nvm_gyro_lpf);
        */
        acc_scale = nvm_acc_scale;
        acc_odr = nvm_acc_odr;
        acc_lpf = nvm_acc_lpf;
        gyro_scale = nvm_gyro_scale;
        gyro_odr = nvm_gyro_odr;
        gyro_lpf = nvm_gyro_lpf;

        AttitudeBalanceAlpha = intBuffer / 250.0;

        nvs_close(my_handle);
    }

    setAccScale(acc_scale);
    setAccODR(acc_odr);
    setAccLPF(acc_lpf);
    switch (acc_scale)
    {
    // Possible accelerometer scales (and their register bit settings) are:
    // 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11).
    // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that
    // 2-bit value:
    case ACC_RANGE_2G:
        accelScales = 2.0 / 32768.0;
        break;
    case ACC_RANGE_4G:
        accelScales = 4.0 / 32768.0;
        break;
    case ACC_RANGE_8G:
        accelScales = 8.0 / 32768.0;
        break;
    case ACC_RANGE_16G:
        accelScales = 16.0 / 32768.0;
        break;
    }

    setGyroScale(gyro_scale);
    setGyroODR(gyro_odr);
    setGyroLPF(gyro_lpf);
    switch (gyro_scale)
    {
    // Possible gyro scales (and their register bit settings) are:
    // 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS  (11).
    // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that
    // 2-bit value:
    case GYR_RANGE_16DPS:
        gyroScales = 16.0 / 32768.0;
        break;
    case GYR_RANGE_32DPS:
        gyroScales = 32.0 / 32768.0;
        break;
    case GYR_RANGE_64DPS:
        gyroScales = 64.0 / 32768.0;
        break;
    case GYR_RANGE_128DPS:
        gyroScales = 128.0 / 32768.0;
        break;
    case GYR_RANGE_256DPS:
        gyroScales = 256.0 / 32768.0;
        break;
    case GYR_RANGE_512DPS:
        gyroScales = 512.0 / 32768.0;
        break;
    case GYR_RANGE_1024DPS:
        gyroScales = 1024.0 / 32768.0;
        break;
    }

    GyroBias.x = 0;
    GyroBias.y = 0;
    GyroBias.z = 0;

    Madgwick_Init(sampleFreq, 1.0 - AttitudeBalanceAlpha);
}
void QMI8658_Loop(void)
{
    getAccelerometer();
    getGyroscope();
    getAttitude();
    getGFactor();
}

/**
 * Transmit one uint8_t of data to QMI8658.
 * @param addr address of data to be written
 * @param data the data to be written
 */
void QMI8658_transmit(uint8_t addr, uint8_t data)
{
    I2C_Write(Device_addr, addr, &data, 1);
}

/**
 * Receive one uint8_t of data from QMI8658.
 * @param addr address of data to be read
 * @return the uint8_t of data that was read
 */
uint8_t QMI8658_receive(uint8_t addr)
{
    uint8_t retval;
    I2C_Read(Device_addr, addr, &retval, 1);
    return retval;
}

/**
 * Writes data to CTRL9 (command register) and waits for ACK.
 * @param command the command to be executed
 */
void QMI8658_CTRL9_Write(uint8_t command)
{
    // transmit command
    QMI8658_transmit(QMI8658_CTRL9, command);

    // wait for command to be done
    while (((QMI8658_receive(QMI8658_STATUSINT)) & 0x80) == 0x00)
        ;
}

/**
 * Set output data rate (ODR) of accelerometer.
 * @param odr acc_odr_t variable representing new data rate
 */
void setAccODR(acc_odr_t odr)
{
    if (sensor_state != sensor_default) // If the device is not in the default state
    {
        uint8_t ctrl2 = QMI8658_receive(QMI8658_CTRL2);
        ctrl2 &= ~QMI8658_AODR_MASK; // clear previous setting
        ctrl2 |= odr;                // OR in new setting
        QMI8658_transmit(QMI8658_CTRL2, ctrl2);
    }
    acc_odr = odr;
}

/**
 * Set output data rate (ODR) of gyro.
 * @param odr gyro_odr_t variable representing new data rate
 */
void setGyroODR(gyro_odr_t odr)
{
    if (sensor_state != sensor_default)
    {
        uint8_t ctrl3 = QMI8658_receive(QMI8658_CTRL3);
        ctrl3 &= ~QMI8658_GODR_MASK; // clear previous setting
        ctrl3 |= odr;                // OR in new setting
        QMI8658_transmit(QMI8658_CTRL3, ctrl3);
    }
    gyro_odr = odr;
}

/**
 * Set scale of accelerometer output.
 * @param scale acc_scale_t variable representing new scale
 */
void setAccScale(acc_scale_t scale)
{
    if (sensor_state != sensor_default)
    {
        uint8_t ctrl2 = QMI8658_receive(QMI8658_CTRL2);
        ctrl2 &= ~QMI8658_ASCALE_MASK;           // clear previous setting
        ctrl2 |= scale << QMI8658_ASCALE_OFFSET; // OR in new setting
        QMI8658_transmit(QMI8658_CTRL2, ctrl2);
    }
    acc_scale = scale;
}

/**
 * Set scale of gyro output.
 * @param scale gyro_scale_t variable representing new scale
 */
void setGyroScale(gyro_scale_t scale)
{
    if (sensor_state != sensor_default)
    {
        uint8_t ctrl3 = QMI8658_receive(QMI8658_CTRL3);
        ctrl3 &= ~QMI8658_GSCALE_MASK;           // clear previous setting
        ctrl3 |= scale << QMI8658_GSCALE_OFFSET; // OR in new setting
        QMI8658_transmit(QMI8658_CTRL3, ctrl3);
    }
    gyro_scale = scale;
}

/**
 * Set new low-pass filter value for accelerometer
 * @param lp lpf_t variable representing new low-pass filter value
 */
void setAccLPF(lpf_t lpf)
{
    if (sensor_state != sensor_default)
    {
        uint8_t ctrl5 = QMI8658_receive(QMI8658_CTRL5);
        ctrl5 &= !QMI8658_ALPF_MASK;
        ctrl5 |= lpf << QMI8658_ALPF_OFFSET;
        ctrl5 |= 0x01; // turn on acc low pass filter
        QMI8658_transmit(QMI8658_CTRL5, ctrl5);
    }
    acc_lpf = lpf;
}

/**
 * Set new low-pass filter value for gyro
 * @param lp lpf_t variable representing new low-pass filter value
 */
void setGyroLPF(lpf_t lpf)
{
    if (sensor_state != sensor_default)
    {
        uint8_t ctrl5 = QMI8658_receive(QMI8658_CTRL5);
        ctrl5 &= !QMI8658_GLPF_MASK;
        ctrl5 |= lpf << QMI8658_GLPF_OFFSET;
        ctrl5 |= 0x10; // turn on gyro low pass filter
        QMI8658_transmit(QMI8658_CTRL5, ctrl5);
    }
}

/**
 * Set new state of QMI8658.
 * @param state new state to transition to
 */
void setState(sensor_state_t state)
{
    uint8_t ctrl1;
    switch (state)
    {
    case sensor_running:
        ctrl1 = QMI8658_receive(QMI8658_CTRL1);
        // enable 2MHz oscillator
        ctrl1 &= 0xFE;
        // enable auto address increment for fast block reads
        ctrl1 |= 0x40;
        QMI8658_transmit(QMI8658_CTRL1, ctrl1);

        // enable high speed internal clock,
        // acc and gyro in full mode, and
        // disable syncSample mode
        // QMI8658_transmit(QMI8658_CTRL7, 0x43);

        // 1.1.17 enable syncSample mode to allow the filter works in sync
        QMI8658_transmit(QMI8658_CTRL7, 0x83);

        // disable AttitudeEngine Motion On Demand
        QMI8658_transmit(QMI8658_CTRL6, 0x00);
        break;
    case sensor_power_down:
        // disable high speed internal clock,
        // acc and gyro powered down
        QMI8658_transmit(QMI8658_CTRL7, 0x00);

        ctrl1 = QMI8658_receive(QMI8658_CTRL1);
        // disable 2MHz oscillator
        ctrl1 |= 0x01;
        QMI8658_transmit(QMI8658_CTRL1, ctrl1);
        break;
    case sensor_locking:
        ctrl1 = QMI8658_receive(QMI8658_CTRL1);
        // enable 2MHz oscillator
        ctrl1 &= 0xFE;
        // enable auto address increment for fast block reads
        ctrl1 |= 0x40;
        QMI8658_transmit(QMI8658_CTRL1, ctrl1);

        // enable high speed internal clock,
        // acc and gyro in full mode, and
        // enable syncSample mode
        QMI8658_transmit(QMI8658_CTRL7, 0x83);

        // disable AttitudeEngine Motion On Demand
        QMI8658_transmit(QMI8658_CTRL6, 0x00);

        // disable internal AHB clock gating:
        QMI8658_transmit(QMI8658_CAL1_L, 0x01);
        QMI8658_CTRL9_Write(0x12);
        // re-enable clock gating
        QMI8658_transmit(QMI8658_CAL1_L, 0x00);
        QMI8658_CTRL9_Write(0x12);
        break;
    default:
        break;
    }
    sensor_state = state;
}

void getAccelerometer(void)
{

    uint8_t buf[6];
    I2C_Read(Device_addr, QMI8658_AX_L, buf, 6);
    Accel.x = (float)((int16_t)((buf[1] << 8) | (buf[0])));
    Accel.y = (float)((int16_t)((buf[3] << 8) | (buf[2])));
    Accel.z = (float)((int16_t)((buf[5] << 8) | (buf[4])));
    Accel.x = Accel.x * accelScales;
    Accel.y = Accel.y * accelScales;
    Accel.z = Accel.z * accelScales;

    AccelFilteredWithoutPanelAlignment.x = (FilterMoltiplier * AccelFilteredWithoutPanelAlignment.x + Accel.x) / (FilterMoltiplier + 1.0);
    AccelFilteredWithoutPanelAlignment.y = (FilterMoltiplier * AccelFilteredWithoutPanelAlignment.y + Accel.y) / (FilterMoltiplier + 1.0);
    AccelFilteredWithoutPanelAlignment.z = (FilterMoltiplier * AccelFilteredWithoutPanelAlignment.z + Accel.z) / (FilterMoltiplier + 1.0);


    IMUdata aS;
    IMUdata aB_out;
        aS.x = AccelFilteredWithoutPanelAlignment.z;
    // Pitch
    aS.y = AccelFilteredWithoutPanelAlignment.y;
    // Yaw
    aS.z = AccelFilteredWithoutPanelAlignment.x;

    PanelAlignmentMatrixApply(aS, &aB_out, PanelAlignment);

    AccelFiltered.y = aB_out.y;
    AccelFiltered.z = aB_out.x;
    AccelFiltered.x = aB_out.z;

/*    // ROLL
    aS.x = AccelFilteredWithoutPanelAlignment.y;
    // Pitch
    aS.y = AccelFilteredWithoutPanelAlignment.z;
    // Yaw
    aS.z = AccelFilteredWithoutPanelAlignment.x;

    PanelAlignmentMatrixApply(aS, &aB_out, PanelAlignment);

    AccelFiltered.y = aB_out.x;
    AccelFiltered.z = aB_out.y;
    AccelFiltered.x = aB_out.z;
*/
    }

// Function to calculate roll (bank) from accelerometer
uint16_t getRollFromAccel(float axNotUsed, float ay, float az)
{

    int alpha = ay * 10.0;
    int beta = az * 10.0;
    if (alpha == 0 && beta == 0)
    {
        // Exception:
        return 0;
    }

    return lv_atan2(alpha, beta);
}

// Function to calculate pitch from accelerometer
int16_t getPitchFromAccel(float ax, float ay, float az)
{

    lv_sqrt_res_t res;
    lv_sqrt(ay * ay * 100.0 + az * az * 100.0, &res, 0x8000);

    int alpha = -ax * 10.0;
    if (alpha == 0 && res.i == 0)
    {
        // Exception:
        return 0;
    }

    uint16_t pitchReference = lv_atan2(alpha, res.i);
    if (pitchReference >= 90)
    {
        pitchReference = pitchReference - 360;
    }
    return pitchReference;
}


// Guru Meditation Error: Core  0 panic'ed (IntegerDivideByZero). Exception was unhandled.
void getGyroscope(void)
{
    uint8_t buf[6];
    I2C_Read(Device_addr, QMI8658_GX_L, buf, 6);
    Gyro.x = (float)((int16_t)((buf[1] << 8) | (buf[0])));
    Gyro.y = (float)((int16_t)((buf[3] << 8) | (buf[2])));
    Gyro.z = (float)((int16_t)((buf[5] << 8) | (buf[4])));
    Gyro.x = Gyro.x * gyroScales;
    Gyro.y = Gyro.y * gyroScales;
    Gyro.z = Gyro.z * gyroScales;

    GyroFilteredWithoutPanelAlignment.x = (FilterMoltiplierGyro * GyroFiltered.x + Gyro.x) / (FilterMoltiplierGyro + 1.0);
    GyroFilteredWithoutPanelAlignment.y = (FilterMoltiplierGyro * GyroFiltered.y + Gyro.y) / (FilterMoltiplierGyro + 1.0);
    GyroFilteredWithoutPanelAlignment.z = (FilterMoltiplierGyro * GyroFiltered.z + Gyro.z) / (FilterMoltiplierGyro + 1.0);

    IMUdata aS;
    IMUdata aB_out;
    IMUdata PanelAlignmentGyro;
    // ROLL
    aS.z = GyroFilteredWithoutPanelAlignment.x;
    PanelAlignmentGyro = PanelAlignment;
    // Pitch
    aS.y = GyroFilteredWithoutPanelAlignment.y;

    // Yaw
    aS.x = GyroFilteredWithoutPanelAlignment.z;


    PanelAlignmentMatrixApply(aS, &aB_out, PanelAlignmentGyro);

    GyroFiltered.x = aB_out.z;
    GyroFiltered.y = aB_out.y;
    GyroFiltered.z = aB_out.x;
}
void getGFactor(void)
{
    lv_sqrt_res_t res;
    lv_sqrt(AccelFiltered.x * AccelFiltered.x * 100.0 + AccelFiltered.y * AccelFiltered.y * 100.0 + AccelFiltered.z * AccelFiltered.z * 100.0, &res, 0x8000);
    GFactor = res.i / 10.0;
    /*
        Load Factor Maneuver
        15° -> 1.3G
        45° -> 1.4G
        50° -> 1.5G
        55° -> 1.6G
        57° -> 1.65G
        58° -> 1.7G
        60° -> 2G
    */
#define AttitudeBalanceAlphaAerobaticsGFactorThreshold 2.0
#define AttitudeBalanceAlphaAerobaticsGFactorLowPassFilter 6
    // Low pass filter to avoid turbolence and be sure you are under continuous Gs before switch to aerobatic mode, then switch to a more responsive filter to be more reactive to G changes
    static float GFactorLowPassFilter = 0;

    GFactorLowPassFilter = (GFactorLowPassFilter * (AttitudeBalanceAlphaAerobaticsGFactorLowPassFilter - 1) + GFactor) / AttitudeBalanceAlphaAerobaticsGFactorLowPassFilter;

    if(GFactorLowPassFilter > AttitudeBalanceAlphaAerobaticsGFactorThreshold)
    {
        AttitudeBalanceAlphaAerobatics = 1.0;
    } else {
        float attitudeFactor = (1.0-AttitudeBalanceAlpha) / AttitudeBalanceAlphaAerobaticsGFactorThreshold;
        AttitudeBalanceAlphaAerobatics = AttitudeBalanceAlpha+GFactorLowPassFilter*attitudeFactor;
    }

    if (AccelFiltered.x < 0)
    {
        GFactor = GFactor * -1.0;
    }
    if (GFactor > GFactorMax)
    {
#ifdef RB_ENABLE_CONSOLE_DEBUG
        printf("GMeter new Max %f %f", GFactor, GFactorMax);
        printf("\n");
#endif
        GFactorMax = GFactor;
        AccelFilteredMax.x = AccelFiltered.x;
        AccelFilteredMax.y = AccelFiltered.y;
        GFactorDirty = 1;
    }
    if (GFactor < GFactorMin)
    {
#ifdef RB_ENABLE_CONSOLE_DEBUG
        printf("GMeter new min %f %f", GFactor, GFactorMin);
        printf("\n");
#endif
        GFactorMin = GFactor;
        GFactorDirty = 1;
    }
}
int64_t esp_timer_get_time(void);
extern float invSampleFreq;

extern float q0, q1, q2, q3;
extern float GPSLastTrack;

#define DEG2RAD (3.14159265359f / 180.0f)
#define RAD2DEG (180.0f / 3.14159265f)
#define G 9.81f
#define PI 3.14159265f

float unwrapAngle(float prev, float current)
{
    float delta = current - prev;
    if (delta > 180.0f)
        delta -= 360.0f;
    if (delta < -180.0f)
        delta += 360.0f;
    return delta;
}

// Attitude Panel Pitch Alignment Temporary workaround for demo #85
void PanelAlignmentMatrixApply(IMUdata aS, IMUdata *aB_out, IMUdata ref)
{

    // Due to the high risk routing we add a backup, if 0 skip
    if ((int)ref.x == 0 && (int)ref.y == 0 && (int)ref.z == 0)
    {
        aB_out->x = aS.x;
        aB_out->y = aS.y;
        aB_out->z = aS.z;
        return;
    }

    // Alignment angles
    float roll = ref.x * DEG2RAD; // roll left
    float p = ref.y * DEG2RAD;
    float yaw = ref.z * DEG2RAD; // no yaw

    float cr = cosf(roll), sr = sinf(roll);
    float cp = cosf(p), sp = sinf(p);
    float cy = cosf(yaw), sy = sinf(yaw);

    // Rotation matrix (ZYX order: Rz * Ry * Rx)
    /*
    XYZ order (R = Rx * Ry * Rz)
    float R[3][3] = {
        {cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr},
        {sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr},
        {-sp, cp * sr, cp * cr}};
*/
    float R[3][3];
    R[0][0] = cp * cy;
    R[0][1] = -cp * sy;
    R[0][2] = sp;

    R[1][0] = sr * sp * cy + cr * sy;
    R[1][1] = -sr * sp * sy + cr * cy;
    R[1][2] = -sr * cp;

    R[2][0] = -cr * sp * cy + sr * sy;
    R[2][1] = cr * sp * sy + sr * cy;
    R[2][2] = cr * cp;


    R[0][0] = cp;
    R[0][1] = sr*sp;
    R[0][2] = -cr*sp;

    R[1][0] = 0;
    R[1][1] = cr;
    R[1][2] = sr;

    R[2][0] = sp;
    R[2][1] = -sr*cp;
    R[2][2] = cr*cp;


    aB_out->x = R[0][0] * aS.x + R[0][1] * aS.y + R[0][2] * aS.z;
    aB_out->y = R[1][0] * aS.x + R[1][1] * aS.y + R[1][2] * aS.z;
    aB_out->z = R[2][0] * aS.x + R[2][1] * aS.y + R[2][2] * aS.z;
}

void getAttitude(void)
{
    static int64_t last = 0;
    int64_t now = esp_timer_get_time();
    int32_t elapsed = now - last;
    last = now;
    invSampleFreq = elapsed/1000000.0;

    {
        // AY <= -Z
        // AX <= Y
        // AZ <= -X
        // GX <= GY
        // GY <= GZ
        // GZ <= -GX
        // float DEG2RAD = 3.14159265359f / 180.0f;
        // IMUdata panelMountAccel = AccelFiltered;
        // Anticipated to RAW
        // PanelAlignmentMatrixApply(AccelFiltered, &panelMountAccel,PanelAlignment);

        // Example: convert gyro values
        float gz_rad = (GyroFiltered.z + GyroBias.z) * DEG2RAD;
        float gy_rad = (GyroFiltered.y + GyroBias.y) * DEG2RAD;
        float gx_rad = (GyroFiltered.x + GyroBias.x) * DEG2RAD;
        // No interference between axis but Q output Roll and Pitch output are swapped

        Madgwick_UpdateIMU(gy_rad, gz_rad, gx_rad,
                           AccelFiltered.y - GPSLateralYAcceleration,                // ROLL
                           AccelFiltered.z - GPSAccelerationForAttitudeCompensation, // PITCH
                           AccelFiltered.x                                           // YAW
        );

        // Official setup not working
        /*
         Madgwick_UpdateIMU(
             -gx_rad,
             -gy_rad,
             gz_rad,
             AccelFiltered.z - GPSAccelerationForAttitudeCompensation, // PITCH
             AccelFiltered.y + GPSLateralYAcceleration,                // ROLL
             -AccelFiltered.x                                           // YAW
         );
        */
        // Interference but Axis are with minus (upside down)
        /*
                      Madgwick_UpdateIMU(gz_rad, gy_rad, gx_rad,
                           -(AccelFiltered.z - GPSAccelerationForAttitudeCompensation), // PITCH
                           -(AccelFiltered.y + GPSLateralYAcceleration), // ROLL
                           AccelFiltered.x // YAW
        );
        */
        // Upsidedown
        // Madgwick_UpdateIMU(gz_rad, gy_rad, -gx_rad, AccelFiltered.z, AccelFiltered.y, -AccelFiltered.x);
        int64_t now = esp_timer_get_time();
        int64_t GPSIsReliable = now - GPSLastSpeedKmhReceivedTick;

        if (GPSIsReliable < 10000000 && false) // If GPS data is recent (within 10 seconds)
        {
            // float yaw_madgwick = atan2f(2.0f * (q1 * q2 + q0 * q3),
            //                             q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3);
            float yaw_madgwick = atan2f(2.0f * (q0 * q3 + q1 * q2),
                                        1.0f - 2.0f * (q2 * q2 + q3 * q3));

            // printf("Yaw: %.1f GPS: %.1f\n", yaw_madgwick * RAD2DEG, GPSLastTrack);
            //  Convert GPS track to radians
            float yaw_gps = GPSLastTrack * DEG2RAD;
            // Ensure shortest angular difference
            float yaw_error = yaw_gps - yaw_madgwick;
            if (yaw_error > PI)
                yaw_error -= 2.0f * PI;
            if (yaw_error < -PI)
                yaw_error += 2.0f * PI;

            // Apply small correction to yaw
            float yaw_correction = yaw_error * YAW_CORRECTION_GAIN;
            // Rotate quaternion around Z by yaw_correction
            float half_yaw = yaw_correction / 2.0f;
            float sin_half = sinf(half_yaw);
            float cos_half = cosf(half_yaw);

            // Yaw correction quaternion (rotation around Z)
            float dq0 = cos_half;
            float dq1 = 0;
            float dq2 = 0;
            float dq3 = sin_half;
            float rq0 = dq0 * q0 - dq1 * q1 - dq2 * q2 - dq3 * q3;
            float rq1 = dq0 * q1 + dq1 * q0 + dq2 * q3 - dq3 * q2;
            float rq2 = dq0 * q2 - dq1 * q3 + dq2 * q0 + dq3 * q1;
            float rq3 = dq0 * q3 + dq1 * q2 - dq2 * q1 + dq3 * q0;

            // Normalize and set corrected quaternion
            float norm = sqrtf(rq0 * rq0 + rq1 * rq1 + rq2 * rq2 + rq3 * rq3);
            q0 = rq0 / norm;
            q1 = rq1 / norm;
            q2 = rq2 / norm;
            q3 = rq3 / norm;

            float updated_yaw = atan2f(2.0f * (q0 * q3 + q1 * q2),
                                       1.0f - 2.0f * (q2 * q2 + q3 * q3));
            printf("GPS Track: %.1f Yaw: %.1f New: %.1f\n",
                   GPSLastTrack,
                   yaw_madgwick * RAD2DEG,
                   updated_yaw * RAD2DEG);
        }
        //  3. Get Euler angles
        // Roll and Pitch are swapped
        Madgwick_GetEuler(&AttitudePitch, &AttitudeRoll, &AttitudeYaw);
        // Madgwick_GetEuler(&AttitudeRoll, &AttitudePitch, &AttitudeYaw);

        static float prev_yaw = 0;
        float yaw_rate_dps = unwrapAngle(prev_yaw, AttitudeYaw) / invSampleFreq;
        prev_yaw = AttitudeYaw;
        AttitudeYawDegreePerSecond = YAW_RATE_SMOOTHING_ALPHA * yaw_rate_dps + (1.0f - YAW_RATE_SMOOTHING_ALPHA) * AttitudeYawDegreePerSecond;
        static float LastAttitudeRoll = 0;
        static float LastAttitudePitch = 0;
        AttitudeRoll = (LastAttitudeRoll * FilterMoltiplierOutput + (AttitudeRoll)) / (FilterMoltiplierOutput + 1);
        AttitudePitch = (LastAttitudePitch * FilterMoltiplierOutput + AttitudePitch) / (FilterMoltiplierOutput + 1);
        LastAttitudeRoll = AttitudeRoll;
        LastAttitudePitch = AttitudePitch;
    }
}