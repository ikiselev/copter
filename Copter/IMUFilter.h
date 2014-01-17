#include "ITG3205.h"
#include "bma180i.h"

#ifndef __IMUFilter_H_
#define __IMUFilter_H_


class IMUFilter {
    bma180i accel;
    ITG3205 gyro;

    // Stuff from FreeIMU DCM
    float iq0, iq1, iq2, iq3;
    float exInt, eyInt, ezInt;                // scaled integral error
    volatile float twoKp;                        // 2 * proportional gain (Kp)
    volatile float twoKi;                        // 2 * integral gain (Ki)
    volatile float q0, q1, q2, q3;        // quaternion of sensor frame relative to auxiliary frame
    volatile float integralFBx, integralFBy, integralFBz;
    unsigned long lastUpdate, now;        // sample period expressed in milliseconds
    float sampleFreq;                                // half the sample period expressed in seconds
    int startLoopTime;
    float lastRotationAngle = 0;

public:
    IMUFilter();

    void init();
    void getReadings();
    void getQuaternion(float* q);
    void updateAHRS(float gx, float gy, float gz, float ax, float ay, float az);
    void getEuler(float* angles);
    void getRPY(float* angles);

    int getGyroX();

    int getGyroY();

    int getGyroZ();

    int getAccX();

    int getAccY();

    int getAccZ();

    int getAccOffseetX();
    int getAccOffseetY();

    void getRPYPredicted(float *angles, float* angleRot, int angularSpeed, int timeDiff);

    void rotateQuaternion(float *q, float *in, float *rotQ);

    void getAngles(float *angles, float *q);
};

// inverted sqrt taken from quake3
float invSqrt(float number);


#endif //__IMUFilter_H_
