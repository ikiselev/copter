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

/**
 * Best tunings with fuzzy-logic on ropes:
 * 0.3, 0.2, 0.14
 */
PID xPID(&xAngle, &xPIDSpeed, &config.targetAngleX, 0.3, 0.2, 0.14);
PID yPID(&yAngle, &yPIDSpeed, &config.targetAngleY, 0.3, 0.2, 0.14);

void motorsOff();
bool checkBattery();


double xCopterSpeed = config.xSpeedStart;
double yCopterSpeed = config.ySpeedStart;

double xSpeedStep = 0;
double ySpeedStep = 0;

unsigned long now;

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

    now = millis();
}



void loop()
{
    unsigned long lastMillis = now;
    now = millis();

    if(config.flightTime > 0 && now > config.flightTime + config.heatUpTime + config.takeoffTime)
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

    if(now < config.heatUpTime)
    {
        return;
    }

    xPID.Compute();
    yPID.Compute();

    analogWrite(config.esc_x1_pin, constrain(xCopterSpeed - xPIDSpeed / 2, 0, 255));
    analogWrite(config.esc_x2_pin, constrain(xCopterSpeed + xPIDSpeed / 2, 0, 255));

    analogWrite(config.esc_y1_pin, constrain(yCopterSpeed - yPIDSpeed / 2, 0, 255));
    analogWrite(config.esc_y2_pin, constrain(yCopterSpeed + yPIDSpeed / 2, 0, 255));

    //Logging
    Logger.log(FIELD_GYRO_Z, imu.getGyroZ(), false);
    Logger.log(FIELD_X_ANGLE, xAngle, false);
    Logger.log(FIELD_Y_ANGLE, yAngle, false);
    Logger.log(FIELD_X_PID_SPEED, xPIDSpeed, false);
    Logger.log(FIELD_Y_PID_SPEED, yPIDSpeed, true);

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