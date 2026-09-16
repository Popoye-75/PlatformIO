#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <RF24.h>

// ================= PIN CONFIG =================

// Joystick 1
#define JOY1_X A0
#define JOY1_Y A1

// Joystick 2
#define JOY2_X A2
#define JOY2_Y A3

// Encoder
#define ENC_CLK 2
#define ENC_DT  3
#define ENC_SW  4

// ST7789
#define TFT_RST 6
#define TFT_DC  9
#define TFT_CS  10

// NRF24
#define NRF_CE  7
#define NRF_CSN 8

// =================================================

// Display
Adafruit_ST7789 tft(TFT_CS, TFT_DC, TFT_RST);

// NRF
RF24 radio(NRF_CE, NRF_CSN);

// Encoder
int lastCLK = HIGH;
long encoderPos = 0;

// Status
bool nrfOK = false;

// Timing
unsigned long lastDisplayUpdate = 0;
unsigned long lastSerialUpdate = 0;


// =================================================
// DISPLAY SETUP
// =================================================

void showStatusScreen()
{
    tft.fillScreen(ST77XX_BLACK);

    tft.setTextSize(2);
    tft.setTextColor(ST77XX_CYAN);
    tft.setCursor(5, 5);
    tft.println("TX TEST");

    tft.setTextSize(1);
    tft.setTextColor(ST77XX_WHITE);

    tft.setCursor(5, 30);
    tft.print("DISPLAY: ");

    tft.setTextColor(ST77XX_GREEN);
    tft.println("OK");

    tft.setTextColor(ST77XX_WHITE);
    tft.setCursor(5, 45);
    tft.print("NRF24: ");

    if (nrfOK)
    {
        tft.setTextColor(ST77XX_GREEN);
        tft.println("OK");
    }
    else
    {
        tft.setTextColor(ST77XX_RED);
        tft.println("FAIL");
    }

    tft.setTextColor(ST77XX_WHITE);

    tft.setCursor(5, 65);
    tft.println("J1:");

    tft.setCursor(5, 85);
    tft.println("J2:");

    tft.setCursor(5, 110);
    tft.println("ENC:");

    tft.setCursor(5, 130);
    tft.println("SW:");

    tft.setCursor(5, 155);
    tft.println("STATUS:");

    tft.setTextColor(ST77XX_GREEN);
    tft.setCursor(5, 175);
    tft.println("RUNNING");
}


// =================================================
// SETUP
// =================================================

void setup()
{
    Serial.begin(115200);

    delay(300);

    Serial.println();
    Serial.println("================================");
    Serial.println("      TRANSMITTER TEST");
    Serial.println("================================");
    Serial.println();

    // ---------------- ENCODER ----------------

    pinMode(ENC_CLK, INPUT_PULLUP);
    pinMode(ENC_DT, INPUT_PULLUP);
    pinMode(ENC_SW, INPUT_PULLUP);

    Serial.println("[ENCODER]");
    Serial.println("CLK : D2");
    Serial.println("DT  : D3");
    Serial.println("SW  : D4");
    Serial.println("Encoder initialized");
    Serial.println();


    // ---------------- SPI ----------------

    SPI.begin();

    Serial.println("[SPI]");
    Serial.println("SCK  : D13");
    Serial.println("MOSI : D11");
    Serial.println("MISO : D12");
    Serial.println();


    // ---------------- DISPLAY ----------------

    Serial.println("[DISPLAY]");
    Serial.println("Initializing ST7789...");

    // Current assumption: 240x280
    tft.init(120, 80);

    // Landscape
    tft.setRotation(1);

    Serial.println("ST7789 initialized");

    showStatusScreen();

    Serial.println("DISPLAY : OK");
    Serial.println();


    // ---------------- NRF24 ----------------

    Serial.println("[NRF24]");
    Serial.println("CE  : D7");
    Serial.println("CSN : D8");
    Serial.println("SCK : D13");
    Serial.println("MOSI: D11");
    Serial.println("MISO: D12");
    Serial.println();

    Serial.println("Initializing NRF24...");

    if (radio.begin())
    {
        Serial.println("radio.begin() : OK");

        if (radio.isChipConnected())
        {
            nrfOK = true;

            Serial.println("NRF CHIP      : DETECTED");

            radio.setPALevel(RF24_PA_LOW);
            radio.setDataRate(RF24_1MBPS);
            radio.setChannel(108);
            radio.stopListening();

            Serial.println("PA LEVEL      : LOW");
            Serial.println("DATA RATE     : 1MBPS");
            Serial.println("CHANNEL       : 108");
            Serial.println("NRF CONFIG    : OK");
        }
        else
        {
            Serial.println("NRF CHIP      : NOT DETECTED");
        }
    }
    else
    {
        Serial.println("radio.begin() : FAILED");
    }

    Serial.println();


    // ---------------- FINAL STATUS ----------------

    Serial.println("================================");
    Serial.println("       HARDWARE STATUS");
    Serial.println("================================");

    Serial.print("DISPLAY : ");
    Serial.println("OK");

    Serial.print("NRF24   : ");
    Serial.println(nrfOK ? "OK" : "FAIL");

    Serial.println("JOYSTICKS: READY");
    Serial.println("ENCODER  : READY");

    Serial.println("================================");
    Serial.println();
    Serial.println("Live diagnostics started...");
    Serial.println();
}


// =================================================
// ENCODER
// =================================================

void readEncoder()
{
    int clk = digitalRead(ENC_CLK);
    int dt  = digitalRead(ENC_DT);

    if (clk != lastCLK)
    {
        if (dt != clk)
        {
            encoderPos++;
        }
        else
        {
            encoderPos--;
        }
    }

    lastCLK = clk;
}


// =================================================
// DISPLAY UPDATE
// =================================================

void updateDisplay()
{
    int j1x = analogRead(JOY1_X);
    int j1y = analogRead(JOY1_Y);

    int j2x = analogRead(JOY2_X);
    int j2y = analogRead(JOY2_Y);

    bool swPressed = (digitalRead(ENC_SW) == LOW);


    // Only overwrite value areas.
    // No full-screen clearing -> much less flicker.

    tft.setTextSize(1);


    // -------- J1 --------

    tft.fillRect(30, 63, 200, 10, ST77XX_BLACK);

    tft.setCursor(30, 65);
    tft.setTextColor(ST77XX_WHITE);

    tft.print(j1x);
    tft.print("  ");
    tft.print(j1y);


    // -------- J2 --------

    tft.fillRect(30, 83, 200, 10, ST77XX_BLACK);

    tft.setCursor(30, 85);
    tft.setTextColor(ST77XX_WHITE);

    tft.print(j2x);
    tft.print("  ");
    tft.print(j2y);


    // -------- Encoder --------

    tft.fillRect(35, 108, 190, 10, ST77XX_BLACK);

    tft.setCursor(35, 110);
    tft.setTextColor(ST77XX_WHITE);

    tft.print(encoderPos);


    // -------- Encoder switch --------

    tft.fillRect(25, 128, 200, 10, ST77XX_BLACK);

    tft.setCursor(25, 130);

    if (swPressed)
    {
        tft.setTextColor(ST77XX_GREEN);
        tft.print("PRESSED");
    }
    else
    {
        tft.setTextColor(ST77XX_WHITE);
        tft.print("RELEASED");
    }


    // -------- Status --------

    tft.fillRect(55, 153, 170, 10, ST77XX_BLACK);

    tft.setCursor(55, 155);

    if (nrfOK)
    {
        tft.setTextColor(ST77XX_GREEN);
        tft.print("ALL TEST RUNNING");
    }
    else
    {
        tft.setTextColor(ST77XX_RED);
        tft.print("CHECK NRF24");
    }
}


// =================================================
// SERIAL DIAGNOSTICS
// =================================================

void serialDiagnostics()
{
    int j1x = analogRead(JOY1_X);
    int j1y = analogRead(JOY1_Y);

    int j2x = analogRead(JOY2_X);
    int j2y = analogRead(JOY2_Y);

    int clk = digitalRead(ENC_CLK);
    int dt  = digitalRead(ENC_DT);
    int sw  = digitalRead(ENC_SW);


    Serial.println("--------------------------------");

    Serial.print("JOY1  X=");
    Serial.print(j1x);
    Serial.print("  Y=");
    Serial.println(j1y);

    Serial.print("JOY2  X=");
    Serial.print(j2x);
    Serial.print("  Y=");
    Serial.println(j2y);

    Serial.print("ENC   POS=");
    Serial.println(encoderPos);

    Serial.print("ENC   CLK=");
    Serial.print(clk);
    Serial.print(" DT=");
    Serial.print(dt);
    Serial.print(" SW=");

    if (sw == LOW)
        Serial.println("PRESSED");
    else
        Serial.println("RELEASED");


    Serial.print("NRF24 = ");

    if (nrfOK)
        Serial.println("CONNECTED");
    else
        Serial.println("NOT FOUND");


    // Center check
    Serial.print("CENTER CHECK: ");

    bool centerOK =
        (j1x > 400 && j1x < 620) &&
        (j1y > 400 && j1y < 620) &&
        (j2x > 400 && j2x < 620) &&
        (j2y > 400 && j2y < 620);

    if (centerOK)
        Serial.println("OK");
    else
        Serial.println("JOYSTICK MOVED / CHECK");


    Serial.println("--------------------------------");
}


// =================================================
// LOOP
// =================================================

void loop()
{
    // Encoder continuously read
    readEncoder();


    // Display every 300 ms
    if (millis() - lastDisplayUpdate >= 300)
    {
        lastDisplayUpdate = millis();

        updateDisplay();
    }


    // Serial diagnostics every 1000 ms
    if (millis() - lastSerialUpdate >= 1000)
    {
        lastSerialUpdate = millis();

        serialDiagnostics();
    }
}