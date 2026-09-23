#include <Arduino.h>

void setup()
{
    pinMode(LED_BUILTIN, OUTPUT);
    Serial.begin(9600);
    Serial.println("Board is on");
}

void loop()
{
    digitalWrite(LED_BUILTIN, HIGH);
    Serial.println("Led blink");
    delay(1000);

    digitalWrite(LED_BUILTIN, LOW);
    Serial.println("Led Off");
    delay(1000);
}