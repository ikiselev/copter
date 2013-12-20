#include "Arduino.h"

#include "config.h"

#include <Wire.h>
#include "SDLogger.h"
#include "IMUFilter.h"
#include "PID_v1.h"


IMUFilter imu;


float angles[3];
double xAngle, yAngle;


double xPIDSpeed;
double yPIDSpeed;


PID xPID(&xAngle, &xPIDSpeed, &config.targetAngleX, 0.7, 0, 0, DIRECT);
PID yPID(&yAngle, &yPIDSpeed, &config.targetAngleY, 0.7, 0, 0, DIRECT);


void motorsOff();


void setup() {
    Wire.begin();
    Serial.begin(115200);

    if (!Logger.begin(config.loggerType)) {
        //Serial.println("Logger initialization failed! Working without logging");
    }
    Logger.setCurrentBlock(config.sdCardStartBlock_config);


    imu.init();

    pinMode(config.esc_x1_pin, OUTPUT);
    pinMode(config.esc_x2_pin, OUTPUT);

    pinMode(config.esc_y1_pin, OUTPUT);
    pinMode(config.esc_y2_pin, OUTPUT);


    xPID.SetOutputLimits(-config.pidOutputLimits, config.pidOutputLimits);
    xPID.SetMode(AUTOMATIC);
    xPID.SetSampleTime(5);

    yPID.SetOutputLimits(-config.pidOutputLimits, config.pidOutputLimits);
    yPID.SetMode(AUTOMATIC);
    yPID.SetSampleTime(5);
}



void loop()
{
    if(millis() > config.flightTime + config.heatUpTime)
    {
        motorsOff();
        return;
    }

    imu.getRPY(angles);
    xAngle = angles[1] + 180 + config.xOffsetIMU;
    yAngle = angles[0] + 180 + config.yOffsetIMU;

    if((abs(xAngle - config.targetAngleX) > config.failsafeAngle)
        || (abs(yAngle - config.targetAngleY) > config.failsafeAngle))
    {

        motorsOff();
        return;
    }

    if(millis() < config.heatUpTime)
    {
        return;
    }





    xPID.Compute();
    yPID.Compute();


    analogWrite(config.esc_x1_pin, constrain(config.xSpeed - xPIDSpeed / 2, 0, 255));
    analogWrite(config.esc_x2_pin, constrain(config.xSpeed + xPIDSpeed / 2, 0, 255));

    analogWrite(config.esc_y1_pin, constrain(config.ySpeed - yPIDSpeed / 2, 0, 255));
    analogWrite(config.esc_y2_pin, constrain(config.ySpeed + yPIDSpeed / 2, 0, 255));

    //Logging
    Logger.log("xAngle{90;270}", xAngle, false);
    Logger.log("xPIDSpeed{-40;40}", xPIDSpeed, false);
    Logger.log("yAngle{90;270}", yAngle, false);
    Logger.log("yPIDSpeed{-40;40}", yPIDSpeed, true);
}



void motorsOff() {
    analogWrite(config.esc_x1_pin, 0);
    analogWrite(config.esc_x2_pin, 0);

    analogWrite(config.esc_y1_pin, 0);
    analogWrite(config.esc_y2_pin, 0);
}

