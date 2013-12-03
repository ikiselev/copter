#include <Wire.h>
#include "Arduino.h"
#include "bma180i.h"
#include "ITG3205.h"

bma180i bma180i;
ITG3205 itg3205;

void setup()
{
    Serial.begin(115200);
    Wire.begin();

    //Init gyro
    itg3205.initGyro();

    //Init acc
    bma180i.BMA180_Init();
    bma180i.BMA180_SetBandwidth(BMA180_BANDWIDTH_75HZ);
    bma180i.BMA180_SetRange(BMA180_RANGE_4G);

    Serial.println("Columns:g.x{-14375;14375},g.y{-14375;14375},g.z{-14375;14375},acc.x{-4096;4096},acc.y{-4096;4096},acc.z{-4096;4096}");

}



void loop()
{
    itg3205.GyroRead();

    Serial.print(itg3205.g.x);
    Serial.print(",");
    Serial.print(itg3205.g.y);
    Serial.print(",");
    Serial.print(itg3205.g.z);
    Serial.print(",");

    Serial.print(bma180i.BMA180_ReadX());
    Serial.print(",");
    Serial.print(bma180i.BMA180_ReadY());
    Serial.print(",");
    Serial.print(bma180i.BMA180_ReadZ());
    Serial.print("|");
    Serial.println(millis());
    delay(8);
}