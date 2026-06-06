#include "madgwick.h"
#include <math.h>
extern float AttitudeBalanceAlphaAerobatics;
float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;  // Quaternion
//static float beta = 0.1f;        // 2 * proportional gain
float invSampleFreq = 0.01f;  // 1 / sample frequency (100 Hz default)

void Madgwick_Init(float sampleFreq, float betaInit) {
    //beta = betaInit;
    invSampleFreq = 1.0f / sampleFreq;
    q0 = 1.0f; q1 = q2 = q3 = 0.0f;
}

void Madgwick_UpdateIMU(float gx, float gy, float gz, float ax, float ay, float az) {
    float norm, s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;

    // Normalize accelerometer
    norm = sqrtf(ax * ax + ay * ay + az * az);
    if (norm == 0.0f) return; // avoid NaN
    norm = 1.0f / norm;
    ax *= norm;
    ay *= norm;
    az *= norm;

    // Rate of change of quaternion from gyroscope
    qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    qDot2 = 0.5f * ( q0 * gx + q2 * gz - q3 * gy);
    qDot3 = 0.5f * ( q0 * gy - q1 * gz + q3 * gx);
    qDot4 = 0.5f * ( q0 * gz + q1 * gy - q2 * gx);

    // Gradient descent correction
    float _2q0 = 2.0f * q0;
    float _2q1 = 2.0f * q1;
    float _2q2 = 2.0f * q2;
    float _2q3 = 2.0f * q3;
    float _4q0 = 4.0f * q0;
    float _4q1 = 4.0f * q1;
    float _4q2 = 4.0f * q2;
    float _8q1 = 8.0f * q1;
    float _8q2 = 8.0f * q2;
    float q0q0 = q0 * q0;
    float q1q1 = q1 * q1;
    float q2q2 = q2 * q2;
    float q3q3 = q3 * q3;

    s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
    s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
    s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
    s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;

    norm = sqrtf(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
    if (norm == 0.0f) return;
    norm = 1.0f / norm;

    s0 *= norm;
    s1 *= norm;
    s2 *= norm;
    s3 *= norm;

    // Apply feedback
    float beta = 1.0 - AttitudeBalanceAlphaAerobatics;
    qDot1 -= beta * s0;
    qDot2 -= beta * s1;
    qDot3 -= beta * s2;
    qDot4 -= beta * s3;

    // Integrate
    q0 += qDot1 * invSampleFreq;
    q1 += qDot2 * invSampleFreq;
    q2 += qDot3 * invSampleFreq;
    q3 += qDot4 * invSampleFreq;

    // Normalize quaternion
    norm = sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    norm = 1.0f / norm;
    q0 *= norm;
    q1 *= norm;
    q2 *= norm;
    q3 *= norm;
}

// Convert quaternion to roll, pitch, yaw (degrees)
void Madgwick_GetEuler(float* roll, float* pitch, float* yaw) {
    *roll = atan2f(2.0f*(q0*q1 + q2*q3), 1.0f - 2.0f*(q1*q1 + q2*q2)) * (180.0f / M_PI);
    *pitch = asinf(2.0f*(q0*q2 - q3*q1)) * (180.0f / M_PI);
    *yaw = atan2f(2.0f*(q0*q3 + q1*q2), 1.0f - 2.0f*(q2*q2 + q3*q3)) * (180.0f / M_PI);
}
