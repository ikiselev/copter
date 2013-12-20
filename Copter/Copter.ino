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


PID xPID(&xAngle, &xPIDSpeed, &targetAngleX, 0.7, 0, 0, DIRECT);
PID yPID(&yAngle, &yPIDSpeed, &targetAngleY, 0.7, 0, 0, DIRECT);


void motorsOff();


void setup() {
    Wire.begin();
    Serial.begin(115200);

    if (!Logger.begin(loggerType)) {
        //Serial.println("Logger initialization failed! Working without logging");
    }
    Logger.setCurrentBlock(sdCardStartBlock_config);


    imu.init();

    pinMode(esc_x1_pin, OUTPUT);
    pinMode(esc_x2_pin, OUTPUT);

    pinMode(esc_y1_pin, OUTPUT);
    pinMode(esc_y2_pin, OUTPUT);


    xPID.SetOutputLimits(-pidOutputLimits, pidOutputLimits);
    xPID.SetMode(AUTOMATIC);
    xPID.SetSampleTime(5);

    yPID.SetOutputLimits(-pidOutputLimits, pidOutputLimits);
    yPID.SetMode(AUTOMATIC);
    yPID.SetSampleTime(5);
}



void loop()
{
    if(millis() > flightTime + heatUpTime)
    {
        motorsOff();
        return;
    }

    imu.getRPY(angles);
    xAngle = angles[1] + 180 + xOffsetIMU;
    yAngle = angles[0] + 180 + yOffsetIMU;

    if((abs(xAngle - targetAngleX) > failsafeAngle)
        || (abs(yAngle - targetAngleY) > failsafeAngle))
    {

        motorsOff();
        return;
    }

    if(millis() < heatUpTime)
    {
        return;
    }





    xPID.Compute();
    yPID.Compute();


    analogWrite(esc_x1_pin, constrain(xSpeed - xPIDSpeed / 2, 0, 255));
    analogWrite(esc_x2_pin, constrain(xSpeed + xPIDSpeed / 2, 0, 255));

    analogWrite(esc_y1_pin, constrain(ySpeed - yPIDSpeed / 2, 0, 255));
    analogWrite(esc_y2_pin, constrain(ySpeed + yPIDSpeed / 2, 0, 255));

    //Logging
    Logger.log("xAngle{90;270}", xAngle, false);
    Logger.log("xPIDSpeed{-40;40}", xPIDSpeed, false);
    Logger.log("yAngle{90;270}", yAngle, false);
    Logger.log("yPIDSpeed{-40;40}", yPIDSpeed, true);
}



void motorsOff() {
    analogWrite(esc_x1_pin, 0);
    analogWrite(esc_x2_pin, 0);

    analogWrite(esc_y1_pin, 0);
    analogWrite(esc_y2_pin, 0);
}

