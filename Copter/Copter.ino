#include "Arduino.h"

#include "config.h"

#include <Wire.h>
#include "SDLogger.h"
#include "IMUFilter.h"
#include "PID.h"


IMUFilter imu;

bool allowFly = true;

float angles[3];

float xAngle, yAngle;


float xPIDSpeed;
float yPIDSpeed;


PID xPID(&xAngle, &xPIDSpeed, &config.targetAngleX, 0.6, 0.2, 0.12);
PID yPID(&yAngle, &yPIDSpeed, &config.targetAngleY, 0.6, 0.2, 0.12);

void motorsOff();
bool checkBattery();


void setup() {
    Serial.begin(115200);

    if(!checkBattery())
    {
        debug("Low battery voltage");
        allowFly = false;
        return ;
    }

    Wire.begin();


    if (!Logger.begin(config.loggerType)) {
        debug("Logger initialization failed! Working without logging");
    }
    Logger.setCurrentBlock(config.sdCardStartBlock_config);


    imu.init();

    pinMode(config.esc_x1_pin, OUTPUT);
    pinMode(config.esc_x2_pin, OUTPUT);

    pinMode(config.esc_y1_pin, OUTPUT);
    pinMode(config.esc_y2_pin, OUTPUT);


    xPID.setLimits(-config.pidOutputLimits, config.pidOutputLimits);
    yPID.setLimits(-config.pidOutputLimits, config.pidOutputLimits);
}



void loop()
{
    if(!allowFly)
    {
        motorsOff();
        return;
    }

    if(config.flightTime > 0 && millis() > config.flightTime + config.heatUpTime)
    {
        motorsOff();
        return;
    }

    imu.getRPY(angles);
    xAngle = angles[0] + 180 + config.xOffsetIMU;
    yAngle = angles[1] + 180 + config.yOffsetIMU;

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
    Logger.log("p{-50;50}", xPID.p, false);
    Logger.log("i{-50;50}", xPID.i, false);
    Logger.log("d{-50;50}", xPID.d, false);
    Logger.log("s{-16;16}", xPID.s, false);
    Logger.log("xAngle{90;270}", xAngle, false);
    Logger.log("xPIDSpeed{-40;40}", xPIDSpeed, false);
    Logger.log("yAngle{90;270}", yAngle, false);
    Logger.log("xPIDSpeed{-40;40}", yPIDSpeed, true);
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


    if(checkValue < config.batteryLow)
    {

        return false;
    }

    return true;
}