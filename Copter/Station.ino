#include <SPI.h>
#include <Mirf.h>
#include <nRF24L01.h>
#include <MirfHardwareSpiDriver.h>
#include "constants.h"


const int BUFFER_SIZE = 400;
byte radioBuffer[BUFFER_SIZE];

volatile int centrifugeSpeed = 0;
volatile int bufferIndex = 0;
volatile bool allowSpin = false;

//Cetrifuge

unsigned long startSpin = 0;

int speedUpTime = 5000;
float speedStep = 1;
int minSpeed = 0;
int maxSpeed = 255;
int currentSpeed = 0;


int motorPin = 9;

void blink();
void receive();

int getCentrifugeSpeed();



void setup(){
    Serial.begin(115200);



    Mirf.spi = &MirfHardwareSpi;


    Mirf.payload = NRF_PAYLOAD;
    Mirf.channel = 90;


    Mirf.csnPin = 2;
    Mirf.cePin = 4;


    Mirf.init();


    Mirf.setRADDR((byte *)"s");
    Mirf.setTADDR((byte *)"c");

    Mirf.configRegister(CONFIG, 0x48);
    Mirf.config();

    pinMode(motorPin, OUTPUT);

    Serial.println("Listenting");


    speedStep = (maxSpeed - minSpeed) / speedUpTime;


    attachInterrupt(1, receive, LOW);
}

void flushBuffer()
{
    noInterrupts();

        Serial.write(radioBuffer, bufferIndex);
        bufferIndex = 0;

    interrupts();
}

void loop()
{
    if(!allowSpin)
    {
        return;
    }

    if(startSpin == 0)
    {
        startSpin = millis();
    }

    if(bufferIndex > 0)
    {
        flushBuffer();
    }

    analogWrite(motorPin, getCentrifugeSpeed());
}

int getTimeSpinned()
{
    return millis() - startSpin;
}


int getCentrifugeSpeed() {
    int result = 0;
    if(getTimeSpinned() < speedUpTime)
    {
        result = getTimeSpinned() * speedStep;
        if(currentSpeed > maxSpeed) result = maxSpeed;
    }

    //TODO: speed Down


    return result;
}


void receive() {
    if(Mirf.dataReady()) {
        byte inData[NRF_PAYLOAD];
        Mirf.getData(inData);

        if(!allowSpin)
        {
            allowSpin = true;
        }

        for(int h=0; h < NRF_PAYLOAD; h++)
        {
            if(inData[h] == 0x00)
            {
                break;
            }


            //Serial.write(inData[h]);
            radioBuffer[bufferIndex++] = inData[h];
            radioBuffer[bufferIndex] = 0x00;
        }
    }
}
