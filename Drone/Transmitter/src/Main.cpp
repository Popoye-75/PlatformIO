#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>
#include <Wire.h>
#include <stdint.h>
// #include <EEPROM.h>
#include <Encoder.h>
#include <Bounce2.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
// #include <avr/wdt.h>

// ===============================================================================
// ===========================>> Pin Definition <<================================
// ===============================================================================

/* Pin definition for Mode and Function buttton*/
#define MODE_BTN 2
#define FUNCTION_BTN 3

/* Pin definition for Encoder */
#define ENC_CLK 4
#define ENC_DT 5
#define ENC_SW A5

/* Pin definition for RF24 communication */
#define CE_PIN 7
#define CSN_PIN 8

/* Pin definition for TFT Display*/
#define TFT_RST 6
#define TFT_DC 9
#define TFT_CS 10
// Hardware SPI
// MOSI = D11
// MISO = D12
// SCK = D13

/* Pin definition for JoyStick ---> 1*/
#define JOY1_X A0
#define JOY1_Y A1
/* Pin definition for JoyStick ---> 2*/
#define JOY2_X A2
#define JOY2_Y A3

/* Pin defintion for Battery */
#define BATTERY_PIN A4

// ===============================================================================
// ===========================>> Objects Section <================================
// ===============================================================================
RF24 radio(CE_PIN, CSN_PIN);                                    // Initializing the NRF24L01 Driver Object
Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST); // Initializing ST7789 Driver Object

// ===============================================================================
// ===========================>> Packet Structure <<==============================
// ===============================================================================
typedef struct
{
    uint16_t throttle;
    int16_t roll;
    int16_t pitch;
    int16_t yaw;

    uint8_t mode;
    uint8_t flags;
} txData;
// ===============================================================================
// ===========================>> Global Variable <<===============================
// ===============================================================================
txData tx;
uint16_t throttle = 0;
int16_t roll = 0;
int16_t pitch = 0;
int16_t yaw = 0;

uint8_t currMode = 0;
int rollTrim = 0;
int pitchTim = 0;
float batteryVoltage = 0.0;
bool functionPressed = false;

// ===============================================================================
// ======================>> RF Address Communication <<===========================
// ===============================================================================
const byte droneAdd[8] = "DRN3458";
const byte carAdd[8] = "CAR2348";
const byte planeAdd[8] = "PLN3405";

// ===============================================================================
// =======================>> Setup Function to init <<============================
// ===============================================================================
void setup()
{
    Serial.begin(9600);
    SPI.begin();
    radio.begin();
    tft.init(240, 240);
}

// ===============================================================================
// =======================>> Loop Function to continue <<=========================
// ===============================================================================
void loop() {}