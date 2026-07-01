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
// #include "Transmitter.h"
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
uint8_t calibrationAxis = 0;
const uint8_t TOTAL_AXIS = 4;

uint8_t trimsIndex = 0;
const uint8_t TOTAL_TRIMS = 4;

int8_t rollTrim = 0;
int8_t pitchTrim = 0;
int8_t yawTrim = 0;
int8_t throttleTrim = 0;

float batteryVoltage = 0.0;
bool functionPressed = false;
unsigned long bootStartTime = 0;

int16_t rawLeftX;
int16_t rawLeftY;
int16_t rawRightX;
int16_t rawRightY;

// ========================>> NRF24 global Variables <<==========================
uint8_t radioIndex = 0;
const uint8_t TOTAL_RADIO_ITEMS = 3;

// =========================>> JoyStick Calibration <<============================
int16_t joy1X_Min = 0;
int16_t joy1X_Center = 512;
int16_t joy1X_Max = 1023;

int16_t joy1Y_Min = 0;
int16_t joy1Y_Center = 512;
int16_t joy1Y_Max = 1023;

int16_t joy2X_Min = 0;
int16_t joy2X_Center = 512;
int16_t joy2X_Max = 1023;

int16_t joy2Y_Min = 0;
int16_t joy2Y_Center = 512;
int16_t joy2Y_Max = 1023;

// ============================>> JoyStick DeadBand <<=============================
const int16_t DEADBAND = 15;

// ==============================>> Button States <<===============================
bool modeState = HIGH;
bool lastModeState = HIGH;
bool functionState = HIGH;
bool lastFunctionState = HIGH;

// ============================>> Encoder Variables <<=============================
int lastCLK = HIGH;
int currentCLK = HIGH;

// ==============================>> Menu Variables <<==============================
uint8_t menuIndex = 0;
uint8_t homePage = 0;

bool encoderCW = false;
bool encoderCCW = false;
bool encoderPressed = false;

// ============================>> TFT DisplayScreen <<=============================
uint8_t displayIndex = 0;
const uint8_t TOTAL_DISPLAY_ITEMS = 1;

enum Screen
{
    BOOT_SCREEN,
    HOME_SCREEN,
    MENU_SCREEN,

    VEHICLE_SCREEN,
    CALIBRATION_SCREEN,
    TRIM_SCREEN,
    RADIO_SCREEN,
    DISPLAY_SCREEN,
    SYSTEM_SCREEN,
    ABOUT_SCREEN
};
Screen currentScreen = BOOT_SCREEN;
Screen previousScreen = BOOT_SCREEN;
enum Vehicle
{
    DRONE,
    CAR,
    PLANE
};

const uint8_t TOTAL_HOME_PAGES = 4;
const uint8_t TOTAL_MENU_ITEMS = 7;

// ======================>> Process System Variable  <<===========================
uint8_t systemIndex = 0;
const uint8_t TOTAL_SYSTEM_ITEMS = 3;

const char *mainMenu[] = {
    "Vehicle",
    "Calibration",
    "Trim",
    "Radio",
    "Display",
    "System",
    "About"};
const char *vehicleMenu[] = {
    "Drone",
    "Car",
    "Plane"};
// ======================>> RF Address Communication <<===========================
const byte DRONE_ADD[8] = "DRN3458";
const byte CAR_ADD[8] = "CAR2348";
const byte PLANE_ADD[8] = "PLN3405";

// ===============================================================================
// =======================>> Setup Function to init <<============================
// ===============================================================================
void setup()
{
    Serial.begin(115200);
    SPI.begin();

    // =======================>> Input Pins  <<=========================
    pinMode(MODE_BTN, INPUT_PULLUP);
    pinMode(FUNCTION_BTN, INPUT_PULLUP);
    pinMode(ENC_CLK, INPUT_PULLUP);
    pinMode(ENC_DT, INPUT_PULLUP);
    pinMode(ENC_SW, INPUT_PULLUP);

    // =======================>> Output Pins <<=========================
    // pinMode(BUZZER,OUTPUT);
    // pinMode(LED_BUILTIN_OUTPUT);

    // ==========================>> NRF24 Initialization <<===========================
    radio.begin();
    if (!radio.begin())
    {
        Serial.println("NRF24 Initialization Failed ...!");
        while (1)
            ;
    }
    radio.setPALevel(RF24_PA_LOW);   // Set Transmission Power to Low
    radio.setDataRate(RF24_250KBPS); // Set DataRate to 250kbps
    radio.setChannel(108);           // Set channel to 108
    radio.setAutoAck(true);          // To Send Acknowledgement to Reciever while reciever packet is accepted
    radio.setRetries(5, 15);         // If Ack not recieved then it send packet again on delay of (5) and retries of (15)
    radio.stopListening();           // It set Transmit mode except receiver mode

    // =======================>> EEPROM LOAD Initialization <<========================

    // =======================>> TFT Display Initialization <<========================
    tft.init(240, 240);
    tft.setRotation(1);
    tft.fillScreen(ST77XX_BLACK);
    tft.setTextColor(ST77XX_WHITE);
    tft.setTextSize(2);
    tft.setCursor(25, 20);
    tft.println("Universal");
    tft.setCursor(35, 45);
    tft.println("RC");
    tft.setCursor(10, 80);
    tft.println("Transmitter");

    // Default Vehicle
    tx.mode = DRONE;
    radio.openWritingPipe(DRONE_ADD); // The transmitter starts in default drone mode
    // radio.openWritingPipe(CAR_ADD);
    // radio.openWritingPipe(PLANE_ADD);

    // Boot timer
    bootStartTime = millis();
    currentScreen = BOOT_SCREEN;
}

// ===============================================================================
// ===========================>> Function Prototypes <<===========================
// ===============================================================================
void readJoyStick();
void readButtons();
void readEncoder();
void updateBattery();
void processMenu();
void processMainMenu();
void processHome();
void processVehicleMenu();
void processCalibration();
void processTrim();
void processRadio();
void processDisplay();
void processSystem();
void processAbout();
void sendPacket();
void updateDisplay();
void drawBootScreen();
void drawHomeScreen();
void drawMainMenu();
void drawVehicleScreen();
void drawCalibrationScreen();
void drawTrimScreen();
void drawRadioScreen();
void drawDisplayScreen();
void drawSystemScreen();
void drawAboutScreen();
void updateFailSafe();
void changeMode();
void saveSettings();
void loadSettings();
int16_t calibrationJoyStick(
    int16_t value,
    int16_t min,
    int16_t center,
    int16_t max);

int16_t applyDeadband(int16_t value);
// ===============================================================================
// =======================>> Loop Function to continue <<=========================
// ===============================================================================
void loop()
{
    readJoyStick();
    readButtons();
    readEncoder();
    //     updateBattery();
    processMenu();
    //     updateFailSafe();
    //     sendPacket();
    //     updateDisplay();
}

// ===============================================================================
// ===========================>> Function Definitions <<==========================
// ===============================================================================
int16_t applyDeadband(int16_t value)
{
    if (value > -DEADBAND && value < DEADBAND)
    {
        return 0;
    }
    return value;
}

int16_t calibrationJoyStick(
    int16_t value,
    int16_t min,
    int16_t center,
    int16_t max)
{
    if (value < center)
    {
        return map(value, min, center, -500, 0);
    }
    else
    {
        return map(value, center, max, 0, 500);
    }
}

void readJoyStick()
{
    rawLeftX = analogRead(JOY1_X);
    rawLeftY = analogRead(JOY1_Y);
    rawRightX = analogRead(JOY2_X);
    rawRightY = analogRead(JOY2_Y);
    // Calibration
    int16_t leftX = calibrationJoyStick(
        rawLeftX,
        joy1X_Min,
        joy1X_Center,
        joy1X_Max);
    int16_t leftY = calibrationJoyStick(
        rawLeftY,
        joy1Y_Min,
        joy1Y_Center,
        joy1Y_Max);
    int16_t rightX = calibrationJoyStick(
        rawRightX,
        joy2X_Min,
        joy2X_Center,
        joy2X_Max);
    int16_t rightY = calibrationJoyStick(
        rawRightY,
        joy2Y_Min,
        joy2Y_Center,
        joy2Y_Max);

    leftX = applyDeadband(leftX);
    leftY = applyDeadband(leftY);
    rightX = applyDeadband(rightX);
    rightY = applyDeadband(rightY);

    tx.throttle = map(rawLeftY,
                      joy1Y_Min,
                      joy1Y_Max,
                      1000,
                      2000);

    tx.yaw = map(leftX,
                 -500,
                 500,
                 1000,
                 2000);

    tx.roll = map(rightX,
                  -500,
                  500,
                  1000,
                  2000);
    tx.pitch = map(rightY,
                   -500,
                   500,
                   1000,
                   2000);

    tx.throttle = constrain(tx.throttle, 1000, 2000);
    tx.roll = constrain(tx.roll, 1000, 2000);
    tx.pitch = constrain(tx.pitch, 1000, 2000);
    tx.yaw = constrain(tx.yaw, 1000, 2000);
}

void changeMode()
{
    tx.mode++;
    if (tx.mode > 2)
    {
        tx.mode = 0;
    }
    switch (tx.mode)
    {
    case 0:
        radio.openWritingPipe(DRONE_ADD);
        break;
    case 1:
        radio.openWritingPipe(CAR_ADD);
        break;
    case 2:
        radio.openWritingPipe(PLANE_ADD);
        break;
    }
}

void readButtons()
{
    modeState = digitalRead(MODE_BTN);
    functionState = digitalRead(FUNCTION_BTN);

    if (lastModeState == HIGH && modeState == LOW)
    {
        changeMode();
    }
    lastModeState = modeState;
    if (lastFunctionState == HIGH && functionState == LOW)
    {
        // Function Button pressed
    }
    lastFunctionState = functionState;
}

void readEncoder()
{
    currentCLK = digitalRead(ENC_CLK);
    if (currentCLK != lastCLK)
    {
        if (digitalRead(ENC_DT) != currentCLK)
        {
            // clockwise
        }
        else
        {
            // anti clockwise
        }
    }
    lastCLK = currentCLK;
    if (digitalRead(ENC_SW) == LOW)
    {
        // select Menu
    }
}

void processMenu()
{
    switch (currentScreen)
    {
    case BOOT_SCREEN:
        break;
    case HOME_SCREEN:
        processHome();
        break;
    case MENU_SCREEN:
        processMainMenu();
        break;
    case VEHICLE_SCREEN:
        processVehicleMenu();
        break;
    case CALIBRATION_SCREEN:
        processCalibration();
        break;
    case TRIM_SCREEN:
        processTrim();
        break;
    case RADIO_SCREEN:
        processRadio();
        break;
    case DISPLAY_SCREEN:
        processDisplay();
        break;
    case SYSTEM_SCREEN:
        processSystem();
        break;
    case ABOUT_SCREEN:
        processAbout();
        break;
    }
}

void processHome()
{
    if (encoderCW)
    {
        homePage++;
        if (homePage >= TOTAL_HOME_PAGES)
        {
            homePage = 0;
        }
        encoderCW = false;
    }
    if (encoderCCW)
    {
        if (homePage == 0)
        {
            homePage = TOTAL_HOME_PAGES - 1;
        }
        else
        {
            homePage--;
        }
        encoderCCW = false;
    }
    if (encoderPressed)
    {
        currentScreen = MENU_SCREEN;
        encoderPressed = false;
    }
}

void processMainMenu()
{
    if (encoderCW)
    {
        menuIndex++;
        if (menuIndex >= TOTAL_MENU_ITEMS)
        {
            menuIndex = 0;
        }
        encoderCW = false;
    }
    if (encoderCCW)
    {
        if (menuIndex == 0)
        {
            menuIndex = TOTAL_MENU_ITEMS - 1;
        }
        else
        {
            menuIndex--;
        }
        encoderCCW = false;
    }
    if (encoderPressed)
    {
        switch (menuIndex)
        {
        case 0:
            currentScreen = VEHICLE_SCREEN;
            break;
        case 1:
            currentScreen = CALIBRATION_SCREEN;
            break;
        case 2:
            currentScreen = TRIM_SCREEN;
            break;
        case 3:
            currentScreen = RADIO_SCREEN;
            break;
        case 4:
            currentScreen = DISPLAY_SCREEN;
            break;
        case 5:
            currentScreen = SYSTEM_SCREEN;
            break;
        case 6:
            currentScreen = ABOUT_SCREEN;
            break;
        }
        encoderPressed = false;
    }
}

void processVehicleMenu()
{
    if (encoderCW)
    {
        if (tx.mode < 2)
        {
            tx.mode++;
        }
        encoderCW = false;
    }
    if (encoderCCW)
    {
        if (tx.mode > 0)
        {
            tx.mode--;
        }
        encoderCCW = false;
    }
    if (encoderPressed)
    {
        switch (tx.mode)
        {
        case DRONE:
            radio.openWritingPipe(DRONE_ADD);
            break;
        case CAR:
            radio.openWritingPipe(CAR_ADD);
            break;
        case PLANE:
            radio.openWritingPipe(PLANE_ADD);
            break;
        }
        currentScreen = HOME_SCREEN;
        encoderPressed = false;
    }
}

void processCalibration()
{
    if (encoderCW)
    {
        calibrationAxis++;
        if (calibrationAxis >= TOTAL_AXIS)
        {
            calibrationAxis = 0;
        }
        encoderCW = false;
    }
    if (encoderCCW)
    {
        if (calibrationAxis == 0)
        {
            calibrationAxis = TOTAL_AXIS - 1;
        }
        else
        {
            calibrationAxis--;
        }
        encoderCCW = false;
    }
    if (encoderPressed)
    {
        saveSettings();
        currentScreen = MENU_SCREEN;
        encoderPressed = false;
    }
}

void processTrim()
{
    if (encoderCW)
    {
        trimsIndex++;
        if (trimsIndex >= TOTAL_TRIMS)
        {
            trimsIndex = 0;
        }
        encoderCW = false;
    }
    if (encoderCCW)
    {
        if (trimsIndex == 0)
        {
            trimsIndex = TOTAL_TRIMS - 1;
        }
        else
        {
            trimsIndex--;
        }
        encoderCCW = false;
    }
    if (encoderPressed)
    {
        currentScreen = MENU_SCREEN;
        encoderPressed = false;
    }
}

void processRadio()
{
    if (encoderCW)
    {
        radioIndex++;
        if (radioIndex >= TOTAL_RADIO_ITEMS)
        {
            radioIndex = 0;
        }
        encoderCW = false;
    }
    if (encoderCCW)
    {
        if (radioIndex == 0)
        {
            radioIndex = TOTAL_RADIO_ITEMS - 1;
        }
        else
        {
            radioIndex--;
        }
        encoderCCW = false;
    }
    if (encoderPressed)
    {
        currentScreen = MENU_SCREEN;
        encoderPressed = false;
    }
}

void processDisplay()
{
    if (encoderCW)
    {
        displayIndex++;
        if (displayIndex >= TOTAL_DISPLAY_ITEMS)
        {
            displayIndex = 0;
        }
        encoderCW = false;
    }
    if (encoderCCW)
    {
        if (displayIndex == 0)
        {
            displayIndex = TOTAL_DISPLAY_ITEMS - 1;
        }
        else
        {
            displayIndex--;
        }
        encoderCCW = false;
    }
    if (encoderPressed)
    {
        currentScreen = MENU_SCREEN;
        encoderPressed = false;
    }
}

void processSystem()
{
    if (encoderCW)
    {
        systemIndex++;
        if (systemIndex >= TOTAL_SYSTEM_ITEMS)
        {
            systemIndex = 0;
        }
        encoderCW = false;
    }
    if (encoderCCW)
    {
        if (systemIndex == 0)
        {
            systemIndex = TOTAL_SYSTEM_ITEMS - 1;
        }
        else
        {
            systemIndex--;
        }
        encoderCCW = false;
    }
    if (encoderPressed)
    {
        switch (systemIndex)
        {
        case 0:
            // Reset Settings
            break;
        case 1:
            // Factory Reset
            break;
        case 2:
            // Battery Information
            break;
        }
        encoderPressed = false;
    }
}

void processAbout()
{
    if (encoderPressed)
    {
        currentScreen = MENU_SCREEN;
        encoderPressed = false;
    }
}

void updateDisplay()
{
    if (currentScreen != previousScreen)
    {
        tft.fillScreen(ST77XX_BLACK);
        previousScreen = currentScreen;
    }
    switch (currentScreen)
    {
    case BOOT_SCREEN:
        drawBootScreen();
        break;

    case HOME_SCREEN:
        drawHomeScreen();
        break;

    case MENU_SCREEN:
        drawMainMenu();
        break;

    case VEHICLE_SCREEN:
        drawVehicleScreen();
        break;

    case CALIBRATION_SCREEN:
        drawCalibrationScreen();
        break;

    case TRIM_SCREEN:
        drawTrimScreen();
        break;

    case RADIO_SCREEN:
        drawRadioScreen();
        break;

    case DISPLAY_SCREEN:
        drawDisplayScreen();
        break;

    case SYSTEM_SCREEN:
        drawSystemScreen();
        break;

    case ABOUT_SCREEN:
        drawAboutScreen();
        break;
    }
}

void drawBootScreen()
{
    tft.fillScreen(ST77XX_BLACK);
    tft.setTextColor(ST77XX_WHITE);
    tft.setTextSize(2);
    tft.setCursor(20, 40);
    tft.println("Universal RC");
    tft.setCursor(35, 70);
    tft.println("Loading....");
}

void drawHomeScreen()
{
    tft.setTextColor(ST77XX_WHITE);
    tft.setTextSize(2);

    tft.setCursor(5, 5);
    switch (tx.mode)
    {
    case DRONE:
        tft.print("DRONE");
        break;
    case CAR:
        tft.print("CAR");
        break;
    case PLANE:
        tft.print("PLANE");
        break;
    }
    tft.setCursor(5, 35);
    tft.print("THR : ");
    tft.println(tx.throttle);

    tft.setCursor(5, 60);
    tft.print("ROL : ");
    tft.println(tx.roll);

    tft.setCursor(5, 85);
    tft.print("PIT : ");
    tft.println(tx.pitch);

    tft.setCursor(5, 110);
    tft.print("YAW : ");
    tft.println(tx.yaw);
}

void drawMainMenu()
{
    // Heading
    tft.setTextColor(ST77XX_WHITE);
    tft.setTextSize(2);

    tft.setCursor(25, 5);
    tft.println("MAIN MENU");

    // Menu Items
    // tft.setTextSize(1);

    for (uint8_t i = 0; i < TOTAL_MENU_ITEMS; i++)
    {
        // int y = 40 + (i*20);

        if (i == menuIndex)
        {
            // tft.fillRect(0,y-2,240,16,ST77XX_BLUE);
            // tft.setTextColor(ST77XX_WHITE);
            // tft.print("> ");
        }
        else
        {
            // tft.setTextColor(ST77XX_WHITE);
            tft.print(" ");
        }
        // tft.setCursor(10,y);
        tft.println(mainMenu[i]);
    }
}

void drawVehicleScreen()
{
}