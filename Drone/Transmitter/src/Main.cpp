#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>

RF24 radio(9, 10); // CE, CSN

void setup()
{
    Serial.begin(9600);

    Serial.println("Checking NRF24L01...");

    if (radio.begin())
    {
        Serial.println("Transmission is Start ");

        radio.printDetails(); // Module information print karega
    }
    else
    {
        Serial.println("NRF24L01 NOT Found!");
    }
}

void loop()
{
}