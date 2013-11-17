#include <Wire.h>
#include "Arduino.h"
#include "bma180i.h"


bma180i bma180i;

void setup()
{
    Serial.begin(115200);
    Wire.begin();
    Serial.println("Demo started");

    bma180i.BMA180_Init();
    bma180i.BMA180_SetBandwidth(BMA180_BANDWIDTH_10HZ);
    bma180i.BMA180_SetRange(BMA180_RANGE_4G);

    Serial.println("Columns:acc.x in G{-4;4},acc.y in G{-4;4},acc.z in G{-4;4}");

}



void loop()
{
    Serial.print(bma180i.BMA180_ReadX_G());
    Serial.print(",");
    Serial.print(bma180i.BMA180_ReadY_G());
    Serial.print(",");
    Serial.print(bma180i.BMA180_ReadZ_G());
    Serial.print("|");
    Serial.println(millis());
    delay(20);
}