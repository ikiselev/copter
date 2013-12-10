#include <Wire.h>
#include "Arduino.h"
#include "ITG3205.h"
#include "bma180i.h"
#include "IMUFilter.h"
//#include <PID_v1.h>


#include <SD.h>

File myFile;


double targetAngleX = 180.0;
double targetAngleY = 180.0;

IMUFilter imu;


float angles[3];
double xAngle, yAngle;


double xPIDSpeed;
double yPIDSpeed;

/*
PID xPID(&xAngle, &xPIDSpeed, &targetAngleX, 0.148, 0.08, 0.004, DIRECT);
PID yPID(&yAngle, &yPIDSpeed, &targetAngleY, 0.148, 0.08, 0.004, DIRECT);
*/

double xSpeed = 40;
double ySpeed = 40;




int esc_x1_pin = 3;    // verified
int esc_x2_pin = 6;   // verified
int esc_y1_pin = 5;   // verified
int esc_y2_pin = 9;    // verified




int xOffsetIMU = 6;
int yOffsetIMU = -1;





void setup()
{
    Serial.begin(115200);

    pinMode(10, OUTPUT);

    if (!SD.begin(10)) {
        Serial.println("initialization failed!");
        while(1);
        return;
    }


    Wire.begin();

    imu.init();

    pinMode(esc_x1_pin, OUTPUT);
    pinMode(esc_x2_pin, OUTPUT);

    pinMode(esc_y1_pin, OUTPUT);
    pinMode(esc_y2_pin, OUTPUT);




    /*xPID.SetOutputLimits(-200, 200);
    xPID.SetMode(AUTOMATIC);
    xPID.SetSampleTime(5);

    yPID.SetOutputLimits(-200, 200);
    yPID.SetMode(AUTOMATIC);
    yPID.SetSampleTime(5);*/

    delay(3000);
}



void loop()
{

    imu.getRPY(angles);

    xAngle = angles[1] + 180 + xOffsetIMU;
    yAngle = angles[0] + 180 + yOffsetIMU;


    if(millis() < 7000)
    {
        return;
    }

    if(millis() > 16000)
    {
        analogWrite(esc_x1_pin, 0);
        analogWrite(esc_x2_pin, 0);

        analogWrite(esc_y1_pin, 0);
        analogWrite(esc_y2_pin, 0);
        return;
    }


    myFile = SD.open("test.txt", FILE_WRITE);


    myFile.print("xAngle: ");
    myFile.print(xAngle);
    myFile.print(" yAngle: ");
    myFile.println(yAngle);

    myFile.close();


    /*xPID.Compute();
    yPID.Compute();*/


    analogWrite(esc_x1_pin, constrain(xSpeed - xPIDSpeed / 2, 0, 255));
    analogWrite(esc_x2_pin, constrain(xSpeed + xPIDSpeed / 2, 0, 255));

    analogWrite(esc_y1_pin, constrain(ySpeed - yPIDSpeed / 2, 0, 255));
    analogWrite(esc_y2_pin, constrain(ySpeed + yPIDSpeed / 2, 0, 255));



}


