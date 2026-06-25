// #include "DisplayManager.h"
// #include <Arduino.h>
// #include <SPI.h>
// #include <Wire.h>
// #include <Adafruit_GFX.h>
// #include <Adafruit_SSD1306.h>

#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <SPI.h>

// Define the configurable control pins
#define TFT_CS 10
#define TFT_RST 8
#define TFT_DC 9

// Initialize the Adafruit ST7789 driver object
Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

void setup()
{
    SPI.begin();
    Serial.begin(9600);
    Serial.println("ST7789 TFT Tracking Initialization");

    tft.init(240, 240);

    // Set screen orientation (0 to 3)
    tft.setRotation(1);
}

void loop()
{
}
