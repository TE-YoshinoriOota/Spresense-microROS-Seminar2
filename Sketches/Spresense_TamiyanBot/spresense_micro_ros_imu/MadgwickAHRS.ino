#include <math.h>

void MadgwickAHRSinit(MadgwickAHRS *ahrs, float beta) {
    ahrs->q0 = 1.0f;
    ahrs->q1 = 0.0f;
    ahrs->q2 = 0.0f;
    ahrs->q3 = 0.0f;
    ahrs->beta = beta;
}

void MadgwickAHRSupdateIMU(MadgwickAHRS *ahrs, float gx, float gy, float gz, float ax, float ay, float az) {
    float q0 = ahrs->q0, q1 = ahrs->q1, q2 = ahrs->q2, q3 = ahrs->q3;
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
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

    // normalization of accel data
    recipNorm = 1.0f / sqrtf(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // gradient step
    s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
    s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
    s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
    s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
    recipNorm = 1.0f / sqrtf(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // 正規化
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    // integral of diffrential gyro
    qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz) - ahrs->beta * s0;
    qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy) - ahrs->beta * s1;
    qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx) - ahrs->beta * s2;
    qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx) - ahrs->beta * s3;

    q0 += qDot1;
    q1 += qDot2;
    q2 += qDot3;
    q3 += qDot4;

    // normalization of quaternion
    recipNorm = 1.0f / sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    ahrs->q0 = q0 * recipNorm;
    ahrs->q1 = q1 * recipNorm;
    ahrs->q2 = q2 * recipNorm;
    ahrs->q3 = q3 * recipNorm;
}