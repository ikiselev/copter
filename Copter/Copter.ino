#include "Arduino.h"
#define MAIN_FILE

#include "config.h"

#include <Wire.h>
#include "SDLogger.h"
#include "IMUFilter.h"
#include "PID.h"


IMUFilter imu;

float angles[3];

float xAngle, yAngle;


float xPIDSpeed;
float yPIDSpeed;

bool emergencyStop = false;

/**
 * Best tunings with fuzzy-logic on ropes:
 * 0.3, 0.2, 0.14
 * 0.35f, 0, 0.65f Best for free flight
 */
PID xPID(&xAngle, &xPIDSpeed, &config.targetAngleX, 0.35f, 0, 0.65f);
PID yPID(&yAngle, &yPIDSpeed, &config.targetAngleY, 0.35f, 0, 0.65f);

void motorsOff();
bool checkBattery();
void blink();
float mapf(float x, float in_min, float in_max, float out_min, float out_max);
float recalcTargetPidByOmega(int val, float max);


double xCopterSpeed = config.xSpeedStart;
double yCopterSpeed = config.ySpeedStart;

double xSpeedStep = 0;
double ySpeedStep = 0;

//int failsafePidOutputMax = config.pidOutputLimits - 10;
int failsafePidOutputMax = config.pidOutputLimits;

float xOffsetIMU;
float yOffsetIMU;

float maxCentrifugalAngleX = 3.5;
float maxCentrifugalAngleY = 3.5;

unsigned long now;
unsigned long startFlightTime;

int i = 0;

int gyroZ = 0;
int omegaRatePredict = 380;
int omegaRateChangeTarget = 400;
int xAccelDirection = 1;
int yAccelDirection = -1;


int const accelPredictionMax = 20;
int accelPredictionX[accelPredictionMax] = {0};
int accelPredictionY[accelPredictionMax] = {0};
int accelPreditionIndex = 0;
bool accelPredictionFull = false;

void setup() {
    #if DEBUG_ENABLE
    Serial.begin(115200);
    #endif

    if(!checkBattery())
    {
        debug(P("Low battery voltage"));
        while(1);
    }

    Wire.begin();


    if (!Logger.begin()) {
        debug(P("Logger initialization failed!"));
    }
    Logger.setCurrentBlock(config.sdCardStartBlock_config);


    imu.init();

    pinMode(config.esc_x1_pin, OUTPUT);
    pinMode(config.esc_x2_pin, OUTPUT);

    pinMode(config.esc_y1_pin, OUTPUT);
    pinMode(config.esc_y2_pin, OUTPUT);


    xPID.setLimits(-config.pidOutputLimits, config.pidOutputLimits);
    yPID.setLimits(-config.pidOutputLimits, config.pidOutputLimits);

    /**
     * Step in one millisecond for take-off
     */
    if(config.ySpeedStart != config.ySpeed)
    {
        ySpeedStep = (config.ySpeed - config.ySpeedStart) / config.takeoffTime;
    }
    if(config.xSpeedStart != config.xSpeed)
    {
        xSpeedStep = (config.xSpeed - config.xSpeedStart) / config.takeoffTime;
    }

    blink();

    float sumX, sumY = 0;
    for(int i=0; i < 100; i++)
    {
        imu.getRPY(angles);
        delay(15);
    }

    blink();
    blink();


    int countChecks = 50;

    for(int i=0; i < countChecks; i++)
    {
        imu.getRPY(angles);
        sumX += angles[0];
        sumY += angles[1];
        delay(20);
    }
    xOffsetIMU = -sumX / countChecks;
    yOffsetIMU = -sumY / countChecks;

    #if DEBUG_ENABLE
    Serial.print("Offset: ");
    Serial.print(xOffsetIMU);
    Serial.print(",");
    Serial.println(yOffsetIMU);

    Serial.print("Angles: ");
    Serial.print(angles[0] + 180 + xOffsetIMU);
    Serial.print(",");
    Serial.println(angles[1] + 180 + yOffsetIMU);
    #endif

    now = millis();
    startFlightTime = now;
}

unsigned long getFlightTime(unsigned long now)
{
    return now - startFlightTime;
}


void loop()
{
    if(emergencyStop)
    {
        return ;
    }


    unsigned long lastMillis = now;
    now = millis();

    if(config.flightTime > 0 && getFlightTime(now) > config.flightTime + config.heatUpTime + config.takeoffTime)
    {
        motorsOff();
        return;
    }

    gyroZ = abs(imu.getGyroZ());


    //imu.getRPY(angles);
    //TODO: imu.getGyroZ() instead of gyroZ
    float angleRot = -10;
    imu.getRPYPredicted(angles, &angleRot, imu.getGyroZ(), now - lastMillis);

    xAngle = angles[0] + 180 + xOffsetIMU;
    yAngle = angles[1] + 180 + yOffsetIMU;


    /*float gisteresis = 2.5f;
    if(xAngleT > config.targetAngleX + gisteresis)
    {
        xAngle += gisteresis;
    }
    else if(xAngleT < config.targetAngleX - gisteresis)
    {
        xAngle -= gisteresis;
    }

    if(yAngleT > config.targetAngleY + gisteresis)
    {
        yAngle += gisteresis;
    }
    else if(yAngleT < config.targetAngleY - gisteresis)
    {
        yAngle -= gisteresis;
    }*/

    //yAngle = angles[1] + 180 + yOffsetIMU;

    /*imu.getRPYPredicted(anglesPred, gyroZ, now - lastMillis);
    xAnglePred = anglesPred[0] + 180 + xOffsetIMU;
    yAnglePred = anglesPred[1] + 180 + yOffsetIMU;*/


    if((abs(xAngle - config.targetAngleX) > config.failsafeAngle)
        || (abs(yAngle - config.targetAngleY) > config.failsafeAngle))
    {
        emergencyStop = true;
        motorsOff();
        return;
    }

    if(getFlightTime(now) < config.heatUpTime)
    {
        return;
    }

    xPID.Compute(1, imu.getGyroX());
    yPID.Compute(2, imu.getGyroY());


    if(xPIDSpeed > failsafePidOutputMax || yPIDSpeed > failsafePidOutputMax)
    {
        motorsOff();
        emergencyStop = true;
        return;
    }

    analogWrite(config.esc_x1_pin, constrain(xCopterSpeed - xPIDSpeed / 2, 0, 255));
    analogWrite(config.esc_x2_pin, constrain(xCopterSpeed + xPIDSpeed / 2, 0, 255));

    analogWrite(config.esc_y1_pin, constrain(yCopterSpeed - yPIDSpeed / 2, 0, 255));
    analogWrite(config.esc_y2_pin, constrain(yCopterSpeed + yPIDSpeed / 2, 0, 255));

    //Logging
    Logger.log(FIELD_ANGLE_ROT_PREDICT, angleRot, false);
    Logger.log(FIELD_GYRO_X, imu.getGyroX(), false);
    Logger.log(FIELD_GYRO_Y, imu.getGyroY(), false);
    Logger.log(FIELD_GYRO_Z, gyroZ, false);
    Logger.log(FIELD_X_PID_SPEED, xPIDSpeed, false);
    Logger.log(FIELD_Y_PID_SPEED, yPIDSpeed, false);
    Logger.log(FIELD_X_ANGLE, xAngle, false);
    Logger.log(FIELD_Y_ANGLE, yAngle, false);
    Logger.log(FIELD_PID_TARGET_X, config.targetAngleX, false);
    Logger.log(FIELD_PID_TARGET_Y, config.targetAngleY, true);


    /**
     * Accel
     */

    /*Logger.log(FIELD_X_ANGLE, xAngle, false);
    Logger.log(FIELD_Y_ANGLE, yAngle, true);*/

    /*Logger.log(FIELD_GYRO_X, imu.getGyroX(), false);
    Logger.log(FIELD_X_ANGLE, xAngle, false);
    Logger.log(FIELD_Y_ANGLE, yAngle, false);
    Logger.log(FIELD_X_ANGLE_PREDICT, xAnglePred, false);
    Logger.log(FIELD_Y_ANGLE_PREDICT, yAnglePred, true);*/


    /**
     * Recalculate pid target
     */
    if(gyroZ < omegaRatePredict)
    {
        config.targetAngleX = 180;
        config.targetAngleY = 180;
    }
    #if PREDICT_ACCEL_DIRECTION
    else if(gyroZ >= omegaRatePredict && gyroZ < omegaRateChangeTarget)
    {
        accelPredictionX[accelPreditionIndex] = imu.getAccX();
        accelPredictionY[accelPreditionIndex] = imu.getAccY();
        accelPreditionIndex++;
        if(accelPreditionIndex > accelPredictionMax - 1)
        {
            accelPreditionIndex = 0;
            accelPredictionFull = true;
        }
    }
    #endif
    else if(gyroZ >= omegaRateChangeTarget)
    {
        #if PREDICT_ACCEL_DIRECTION
        if(xAccelDirection == 0) //||yAccel...
        {
            int collectedValues = (accelPredictionFull) ? accelPredictionMax : accelPreditionIndex - 1;


            int sumX = 0;
            int sumY = 0;
            for(int h=0; h < collectedValues; h++)
            {
                sumX += accelPredictionX[h];
                sumY += accelPredictionY[h];
            }

            sumX /= collectedValues;
            sumY /= collectedValues;

            xAccelDirection = (sumX > imu.getAccOffseetX()) ? 1 : -1;
            yAccelDirection = (sumY > imu.getAccOffseetY()) ? 1 : -1;
        }
        #endif

        config.targetAngleX = 180 + xAccelDirection * recalcTargetPidByOmega(gyroZ, maxCentrifugalAngleX);
        config.targetAngleY = 180 + yAccelDirection * recalcTargetPidByOmega(gyroZ, maxCentrifugalAngleY);
    }

    /**
     * Smooth take-off
     */
    if(xCopterSpeed < config.xSpeed)
    {
        xCopterSpeed += (now - lastMillis) * xSpeedStep;
        if(xCopterSpeed > config.xSpeed) xCopterSpeed = config.xSpeed;
    }

    if(yCopterSpeed < config.ySpeed)
    {
        yCopterSpeed += (now - lastMillis) * ySpeedStep;
        if(yCopterSpeed > config.ySpeed) yCopterSpeed = config.ySpeed;
    }
}



float recalcTargetPidByOmega(int val, float max) {
    val = constrain(val, omegaRateChangeTarget, 2000);
    float mappedIn = map(val, omegaRateChangeTarget, 2000, 1, 625);

    float res = sqrtf(sqrtf(mappedIn));
    res = mapf(res, 1.0, 5.0, 0.0, max);

    return res;
}


float mapf(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


void blink() {
    delay(100);
    analogWrite(config.ledPin, 20);
    delay(100);
    analogWrite(config.ledPin, 0);
}

void motorsOff() {
    analogWrite(config.esc_x1_pin, 0);
    analogWrite(config.esc_x2_pin, 0);

    analogWrite(config.esc_y1_pin, 0);
    analogWrite(config.esc_y2_pin, 0);
}


bool checkBattery() {
    if(!config.checkBatteryAtStartup)
    {

        return true;
    }

    int checks = 10;
    float checkValue = 0;
    for(int i=0; i < checks; i++)
    {
        checkValue += analogRead(config.batteryAnalogPing);
        delay(1);
    }
    checkValue /= checks;



    debug(P("Battery: "), checkValue);
    if(checkValue < config.batteryLow)
    {

        return false;
    }

    return true;
}