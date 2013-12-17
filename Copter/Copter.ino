#include "Arduino.h"
#include <Wire.h>
#include "SDLogger.h"
#include "IMUFilter.h"
#include "PID_v1.h"
#include "config.h"


IMUFilter imu;


float angles[3];
double xAngle, yAngle;


double xPIDSpeed;
double yPIDSpeed;


PID xPID(&xAngle, &xPIDSpeed, &targetAngleX, 0.148, 0.08, 0.004, DIRECT);
PID yPID(&yAngle, &yPIDSpeed, &targetAngleY, 0.148, 0.08, 0.004, DIRECT);


void motorsOff();


void setup() {
    Wire.begin();


    if (!Logger.begin(loggerType, 4)) {
        Serial.println("Logger initialization failed! Working without logging");
    }


    imu.init();

    pinMode(esc_x1_pin, OUTPUT);
    pinMode(esc_x2_pin, OUTPUT);

    pinMode(esc_y1_pin, OUTPUT);
    pinMode(esc_y2_pin, OUTPUT);


    xPID.SetOutputLimits(-200, 200);
    xPID.SetMode(AUTOMATIC);
    xPID.SetSampleTime(5);

    yPID.SetOutputLimits(-200, 200);
    yPID.SetMode(AUTOMATIC);
    yPID.SetSampleTime(5);
}



void loop()
{
    imu.getRPY(angles);

    xAngle = angles[1] + 180 + xOffsetIMU;
    yAngle = angles[0] + 180 + yOffsetIMU;

    if((abs(xAngle - targetAngleX) > failsafeAngle)
        || (abs(yAngle - targetAngleY) > failsafeAngle))
    {

        motorsOff();
        return;
    }

    xPID.Compute();
    yPID.Compute();

    if(millis() < heatUpTime)
    {
        return;
    }

    if(millis() > flightTime + heatUpTime)
    {
        motorsOff();
        return;
    }





    analogWrite(esc_x1_pin, constrain(xSpeed - xPIDSpeed / 2, 0, 255));
    analogWrite(esc_x2_pin, constrain(xSpeed + xPIDSpeed / 2, 0, 255));

    analogWrite(esc_y1_pin, constrain(ySpeed - yPIDSpeed / 2, 0, 255));
    analogWrite(esc_y2_pin, constrain(ySpeed + yPIDSpeed / 2, 0, 255));


    //Logging
    Logger.log("xAngle{100;260}", xAngle);
    Logger.log("yAngle{100;260}", yAngle);
    Logger.log("xPIDSpeed{-200;200}", xPIDSpeed);
    Logger.log("yPIDSpeed{-200;200}", yPIDSpeed, true);
}



void motorsOff() {
    analogWrite(esc_x1_pin, 0);
    analogWrite(esc_x2_pin, 0);

    analogWrite(esc_y1_pin, 0);
    analogWrite(esc_y2_pin, 0);
}

