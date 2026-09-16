#include <Arduino.h>
#include <SPI.h> // was relied on transitively before - include it explicitly
#include <RF24.h>
#include <Wire.h>
#include <stdint.h>
#include <EEPROM.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>

// ===============================================================================
// This is the HANDHELD TRANSMITTER. It works for all 3 vehicles (Drone / Car /
// Plane) by switching which NRF24 address it writes to.
// ===============================================================================

// ===============================================================================
// ===========================>> Pin Definition <<================================
// ===============================================================================

#define MODE_BTN 2
#define FUNCTION_BTN 3

#define ENC_CLK 4
#define ENC_DT 5
#define ENC_SW A5

#define CE_PIN 7
#define CSN_PIN 8

#define TFT_RST 6
#define TFT_DC 9
#define TFT_CS 10
// Hardware SPI: MOSI = D11, MISO = D12, SCK = D13

#define JOY1_X A0
#define JOY1_Y A1
#define JOY2_X A2
#define JOY2_Y A3

#define BATTERY_PIN A4

/* Function declarations */
void saveSettings();
void loadSettings();
void failSafe();
bool initNRF24();
void initDisplay();
void processBootScreen();
void updateRadioSettings();
void applyDisplaySettings();
void resetSettings();
void factoryReset();

// ===============================================================================
// ===========================>> Objects Section <================================
// ===============================================================================
RF24 radio(CE_PIN, CSN_PIN);
Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

// ===============================================================================
// ===========================>> Packet Structure <<==============================
// ===============================================================================
// Must match the receiver's txData struct exactly (same field order/types).
typedef struct
{
    uint16_t throttle;
    int16_t roll;
    int16_t pitch;
    int16_t yaw;

    uint8_t mode;
    uint8_t flags; // bit0 = armed
} txData;

// ===============================================================================
// ===========================>> Global Variables <<===============================
// ===============================================================================
txData tx;

uint8_t calibrationAxis = 0;
const uint8_t TOTAL_AXIS = 4;

uint8_t trimIndex = 0;
const uint8_t TOTAL_TRIMS = 4;

int8_t rollTrim = 0;
int8_t pitchTrim = 0;
int8_t yawTrim = 0;
int8_t throttleTrim = 0;

float batteryVoltage = 0.0;
uint8_t batteryPercent = 0;
uint16_t batteryADC = 0;

bool functionPressed = false;
unsigned long bootStartTime = 0;

int16_t rawLeftX, rawLeftY, rawRightX, rawRightY;

// ========================>> NRF24 global Variables <<==========================
uint8_t radioIndex = 0;
unsigned long lastPacketTime = 0;
const uint8_t TOTAL_RADIO_ITEMS = 3;

// =========================>> JoyStick Calibration <<============================
int16_t joy1X_Min = 0, joy1X_Center = 512, joy1X_Max = 1023;
int16_t joy1Y_Min = 0, joy1Y_Center = 512, joy1Y_Max = 1023;
int16_t joy2X_Min = 0, joy2X_Center = 512, joy2X_Max = 1023;
int16_t joy2Y_Min = 0, joy2Y_Center = 512, joy2Y_Max = 1023;

bool calibrating = false; // tracks live min/max while CALIBRATION_SCREEN is open

// ============================>> JoyStick DeadBand <<=============================
const int16_t DEADBAND = 15;

// ==============================>> Button States <<===============================
bool modeState = HIGH, lastModeState = HIGH;
bool functionState = HIGH, lastFunctionState = HIGH;
unsigned long lastModeButtonTime = 0;
unsigned long lastFunctionButtonTime = 0;
const unsigned long DEBOUNCE_MS = 200;

// ============================>> Encoder Variables <<=============================
int lastCLK = HIGH;
int currentCLK = HIGH;
bool lastEncSwState = HIGH;
unsigned long lastEncSwTime = 0;

// ==============================>> Menu Variables <<==============================
uint8_t menuIndex = 0;
uint8_t homePage = 0;

bool encoderCW = false;
bool encoderCCW = false;
bool encoderPressed = false;

// ============================>> TFT DisplayScreen <<=============================
uint8_t displayIndex = 0;
uint8_t brightness = 100;
uint8_t rotation = 1;
uint8_t theme = 0;

const uint8_t TOTAL_DISPLAY_ITEMS = 3; // was 1 - brightness/rotation/theme are all editable

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

const char *mainMenu[] = {"Vehicle", "Calibration", "Trim", "Radio", "Display", "System", "About"};
const char *vehicleMenu[] = {"Drone", "Car", "Plane"};
const char *calibrationMenu[] = {"Left X", "Left Y", "Right X", "Right Y"};
const char *trimMenu[] = {"Roll", "Pitch", "Yaw", "Throttle"};
const char *radioMenu[] = {"Channel", "Power", "Data Rate"};
const char *displayMenu[] = {"Brightness", "Rotation", "Theme"};
const char *systemMenu[] = {"Reset Settings", "Factory Reset", "Battery Info"};

uint8_t radioChannel = 100;
// power: 0=MIN 1=LOW 2=HIGH 3=MAX
uint8_t radioPower = 3;
// data rate: 0=250KBPS 1=1MBPS 2=2MBPS
uint8_t radioDataRate = 1;

// ======================>> RF Address Communication <<===========================
const byte DRONE_ADD[8] = "DRN3458";
const byte CAR_ADD[8] = "CAR2348";
const byte PLANE_ADD[8] = "PLN3405";

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
void changeMode();
void openPipeForMode();
void selectVehicleAddress(uint8_t mode);

int16_t calibrationJoyStick(int16_t value, int16_t min, int16_t center, int16_t max);
int16_t applyDeadband(int16_t value);

// ===============================================================================
// =======================>> Setup Function <<=====================================
// ===============================================================================
void setup()
{
    Serial.begin(115200);
    SPI.begin();

    pinMode(MODE_BTN, INPUT_PULLUP);
    pinMode(FUNCTION_BTN, INPUT_PULLUP);
    pinMode(ENC_CLK, INPUT_PULLUP);
    pinMode(ENC_DT, INPUT_PULLUP);
    pinMode(ENC_SW, INPUT_PULLUP);

    // =======================>> EEPROM LOAD Initialization <<========================
    loadSettings();

    // =======================>> NRF24 Initialization <<===========================
    if (!initNRF24())
    {
        Serial.println("NRF24 Initialization Failed ...!");
    }
    else
    {
        Serial.println("NRF24 Initialized .....");
    }
    radio.setAutoAck(true);
    radio.setRetries(5, 15);
    radio.stopListening();

    // =======================>> TFT Display Initialization <<========================
    initDisplay();
    tft.setTextColor(ST77XX_WHITE);
    tft.setTextSize(2);
    tft.setCursor(25, 20);
    tft.println("Universal");
    tft.setCursor(35, 45);
    tft.println("RC");
    tft.setCursor(10, 80);
    tft.println("Transmitter");
    delay(1000); // let the splash actually be readable

    // Validate the vehicle mode loaded from EEPROM (0xFF on a blank chip
    // decodes as an out-of-range value) instead of unconditionally forcing
    // DRONE and throwing away whatever the user had saved.
    if (tx.mode > PLANE)
    {
        tx.mode = DRONE;
    }
    selectVehicleAddress(tx.mode);
    tx.flags = 0; // always boot disarmed

    bootStartTime = millis();
    currentScreen = BOOT_SCREEN;
}

// ===============================================================================
// =======================>> Loop Function <<=======================================
// ===============================================================================
void loop()
{
    readJoyStick();
    readButtons();
    readEncoder();
    processMenu();
    updateDisplay();
    if (millis() - lastPacketTime >= 20)
    {
        sendPacket();
        lastPacketTime = millis();
    }
    updateBattery();
    failSafe();
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

int16_t calibrationJoyStick(int16_t value, int16_t min, int16_t center, int16_t max)
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

    // Live-track calibration extremes while the calibration screen is open.
    if (calibrating)
    {
        if (rawLeftX < joy1X_Min) joy1X_Min = rawLeftX;
        if (rawLeftX > joy1X_Max) joy1X_Max = rawLeftX;
        if (rawLeftY < joy1Y_Min) joy1Y_Min = rawLeftY;
        if (rawLeftY > joy1Y_Max) joy1Y_Max = rawLeftY;
        if (rawRightX < joy2X_Min) joy2X_Min = rawRightX;
        if (rawRightX > joy2X_Max) joy2X_Max = rawRightX;
        if (rawRightY < joy2Y_Min) joy2Y_Min = rawRightY;
        if (rawRightY > joy2Y_Max) joy2Y_Max = rawRightY;
    }

    int16_t leftX = calibrationJoyStick(rawLeftX, joy1X_Min, joy1X_Center, joy1X_Max);
    int16_t leftY = calibrationJoyStick(rawLeftY, joy1Y_Min, joy1Y_Center, joy1Y_Max);
    int16_t rightX = calibrationJoyStick(rawRightX, joy2X_Min, joy2X_Center, joy2X_Max);
    int16_t rightY = calibrationJoyStick(rawRightY, joy2Y_Min, joy2Y_Center, joy2Y_Max);

    leftX = applyDeadband(leftX);
    leftY = applyDeadband(leftY);
    rightX = applyDeadband(rightX);
    rightY = applyDeadband(rightY);

    // Throttle uses the calibrated/deadbanded value too, not the raw ADC
    // reading straight off joy1Y_Min..joy1Y_Max (the original mixed a
    // calibrated axis for everything else but fed raw ADC counts here).
    tx.throttle = map(leftY, -500, 500, 1000, 2000) + (throttleTrim * 2);
    tx.yaw = map(leftX, -500, 500, 1000, 2000) + (yawTrim * 2);
    tx.roll = map(rightX, -500, 500, 1000, 2000) + (rollTrim * 2);
    tx.pitch = map(rightY, -500, 500, 1000, 2000) + (pitchTrim * 2);

    tx.throttle = constrain(tx.throttle, 1000, 2000);
    tx.roll = constrain(tx.roll, 1000, 2000);
    tx.pitch = constrain(tx.pitch, 1000, 2000);
    tx.yaw = constrain(tx.yaw, 1000, 2000);
}

void selectVehicleAddress(uint8_t mode)
{
    switch (mode)
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
}

void changeMode()
{
    // Switching vehicle mid-flight/drive is dangerous - only allow it
    // while disarmed.
    if (tx.flags & 0x01)
    {
        return;
    }
    tx.mode++;
    if (tx.mode > PLANE)
    {
        tx.mode = 0;
    }
    selectVehicleAddress(tx.mode);
    saveSettings();
}

void readButtons()
{
    modeState = digitalRead(MODE_BTN);
    if (lastModeState == HIGH && modeState == LOW && millis() - lastModeButtonTime > DEBOUNCE_MS)
    {
        changeMode();
        lastModeButtonTime = millis();
    }
    lastModeState = modeState;

    functionState = digitalRead(FUNCTION_BTN);
    if (lastFunctionState == HIGH && functionState == LOW && millis() - lastFunctionButtonTime > DEBOUNCE_MS)
    {
        functionPressed = true; // consumed by whichever screen is active
        lastFunctionButtonTime = millis();
    }
    lastFunctionState = functionState;
}

void readEncoder()
{
    // The original file read CLK/DT but never actually set encoderCW /
    // encoderCCW / encoderPressed - the whole menu was unreachable.
    currentCLK = digitalRead(ENC_CLK);
    if (currentCLK != lastCLK && currentCLK == LOW)
    {
        if (digitalRead(ENC_DT) != currentCLK)
        {
            encoderCW = true;
        }
        else
        {
            encoderCCW = true;
        }
    }
    lastCLK = currentCLK;

    bool swState = digitalRead(ENC_SW);
    if (swState == LOW && lastEncSwState == HIGH && millis() - lastEncSwTime > DEBOUNCE_MS)
    {
        encoderPressed = true;
        lastEncSwTime = millis();
    }
    lastEncSwState = swState;
}

void processMenu()
{
    switch (currentScreen)
    {
    case BOOT_SCREEN: processBootScreen(); break;
    case HOME_SCREEN: processHome(); break;
    case MENU_SCREEN: processMainMenu(); break;
    case VEHICLE_SCREEN: processVehicleMenu(); break;
    case CALIBRATION_SCREEN: processCalibration(); break;
    case TRIM_SCREEN: processTrim(); break;
    case RADIO_SCREEN: processRadio(); break;
    case DISPLAY_SCREEN: processDisplay(); break;
    case SYSTEM_SCREEN: processSystem(); break;
    case ABOUT_SCREEN: processAbout(); break;
    }
}

void processHome()
{
    if (encoderCW)
    {
        homePage++;
        if (homePage >= TOTAL_HOME_PAGES) homePage = 0;
        encoderCW = false;
    }
    if (encoderCCW)
    {
        homePage = (homePage == 0) ? TOTAL_HOME_PAGES - 1 : homePage - 1;
        encoderCCW = false;
    }
    if (encoderPressed)
    {
        currentScreen = MENU_SCREEN;
        encoderPressed = false;
    }
    // FUNCTION_BTN on the home screen is the arm/disarm switch.
    if (functionPressed)
    {
        tx.flags ^= 0x01;
        functionPressed = false;
    }
}

void processMainMenu()
{
    if (encoderCW)
    {
        menuIndex++;
        if (menuIndex >= TOTAL_MENU_ITEMS) menuIndex = 0;
        encoderCW = false;
    }
    if (encoderCCW)
    {
        menuIndex = (menuIndex == 0) ? TOTAL_MENU_ITEMS - 1 : menuIndex - 1;
        encoderCCW = false;
    }
    if (encoderPressed)
    {
        switch (menuIndex)
        {
        case 0: currentScreen = VEHICLE_SCREEN; break;
        case 1:
            currentScreen = CALIBRATION_SCREEN;
            calibrationAxis = 0;
            calibrating = true;
            // start each axis window at the current reading so a stick
            // that's already centered doesn't get thrown out as min/max
            joy1X_Min = joy1X_Max = rawLeftX;
            joy1Y_Min = joy1Y_Max = rawLeftY;
            joy2X_Min = joy2X_Max = rawRightX;
            joy2Y_Min = joy2Y_Max = rawRightY;
            break;
        case 2: currentScreen = TRIM_SCREEN; break;
        case 3: currentScreen = RADIO_SCREEN; break;
        case 4: currentScreen = DISPLAY_SCREEN; break;
        case 5: currentScreen = SYSTEM_SCREEN; break;
        case 6: currentScreen = ABOUT_SCREEN; break;
        }
        encoderPressed = false;
    }
    if (functionPressed) functionPressed = false; // unused on this screen
}

void processVehicleMenu()
{
    if (encoderCW)
    {
        if (tx.mode < PLANE) tx.mode++;
        encoderCW = false;
    }
    if (encoderCCW)
    {
        if (tx.mode > DRONE) tx.mode--;
        encoderCCW = false;
    }
    if (encoderPressed)
    {
        selectVehicleAddress(tx.mode);
        saveSettings();
        currentScreen = HOME_SCREEN;
        encoderPressed = false;
    }
}

void processCalibration()
{
    // FUNCTION_BTN moves to the next axis; while an axis is selected the
    // stick is being watched continuously in readJoyStick(). Encoder
    // press captures the axis's current position as its resting CENTER
    // and, once every axis has a center, saves and exits.
    if (functionPressed)
    {
        calibrationAxis++;
        if (calibrationAxis >= TOTAL_AXIS) calibrationAxis = 0;
        functionPressed = false;
    }
    if (encoderPressed)
    {
        switch (calibrationAxis)
        {
        case 0: joy1X_Center = rawLeftX; break;
        case 1: joy1Y_Center = rawLeftY; break;
        case 2: joy2X_Center = rawRightX; break;
        case 3: joy2Y_Center = rawRightY; break;
        }
        encoderPressed = false;
    }
    if (encoderCW || encoderCCW)
    {
        // long-press-to-exit is easier with a dedicated MODE_BTN tap here
        encoderCW = false;
        encoderCCW = false;
    }
    if (modeState == LOW) // reuse MODE_BTN to finish calibration & leave
    {
        calibrating = false;
        saveSettings();
        currentScreen = MENU_SCREEN;
    }
}

void processTrim()
{
    // FUNCTION_BTN selects which trim to edit, encoder CW/CCW adjusts it.
    if (functionPressed)
    {
        trimIndex++;
        if (trimIndex >= TOTAL_TRIMS) trimIndex = 0;
        functionPressed = false;
    }
    int8_t *target = nullptr;
    switch (trimIndex)
    {
    case 0: target = &rollTrim; break;
    case 1: target = &pitchTrim; break;
    case 2: target = &yawTrim; break;
    case 3: target = &throttleTrim; break;
    }
    if (encoderCW && target)
    {
        if (*target < 100) (*target)++;
        encoderCW = false;
    }
    if (encoderCCW && target)
    {
        if (*target > -100) (*target)--;
        encoderCCW = false;
    }
    if (encoderPressed)
    {
        saveSettings();
        currentScreen = MENU_SCREEN;
        encoderPressed = false;
    }
}

void processRadio()
{
    if (functionPressed)
    {
        radioIndex++;
        if (radioIndex >= TOTAL_RADIO_ITEMS) radioIndex = 0;
        functionPressed = false;
    }
    if (encoderCW)
    {
        switch (radioIndex)
        {
        case 0: if (radioChannel < 125) radioChannel++; break;
        case 1: if (radioPower < 3) radioPower++; break;
        case 2: if (radioDataRate < 2) radioDataRate++; break;
        }
        encoderCW = false;
    }
    if (encoderCCW)
    {
        switch (radioIndex)
        {
        case 0: if (radioChannel > 0) radioChannel--; break;
        case 1: if (radioPower > 0) radioPower--; break;
        case 2: if (radioDataRate > 0) radioDataRate--; break;
        }
        encoderCCW = false;
    }
    if (encoderPressed)
    {
        updateRadioSettings();
        saveSettings();
        currentScreen = MENU_SCREEN;
        encoderPressed = false;
    }
}

void processDisplay()
{
    if (functionPressed)
    {
        displayIndex++;
        if (displayIndex >= TOTAL_DISPLAY_ITEMS) displayIndex = 0;
        functionPressed = false;
    }
    if (encoderCW)
    {
        switch (displayIndex)
        {
        case 0: if (brightness <= 90) brightness += 10; break;
        case 1: rotation = (rotation + 1) % 4; break;
        case 2: theme = (theme == 0) ? 1 : 0; break;
        }
        encoderCW = false;
    }
    if (encoderCCW)
    {
        switch (displayIndex)
        {
        case 0: if (brightness >= 10) brightness -= 10; break;
        case 1: rotation = (rotation == 0) ? 3 : rotation - 1; break;
        case 2: theme = (theme == 0) ? 1 : 0; break;
        }
        encoderCCW = false;
    }
    if (encoderPressed)
    {
        applyDisplaySettings();
        saveSettings();
        currentScreen = MENU_SCREEN;
        encoderPressed = false;
    }
}

void processSystem()
{
    if (encoderCW)
    {
        systemIndex++;
        if (systemIndex >= TOTAL_SYSTEM_ITEMS) systemIndex = 0;
        encoderCW = false;
    }
    if (encoderCCW)
    {
        systemIndex = (systemIndex == 0) ? TOTAL_SYSTEM_ITEMS - 1 : systemIndex - 1;
        encoderCCW = false;
    }
    if (encoderPressed)
    {
        switch (systemIndex)
        {
        case 0: resetSettings(); break;
        case 1: factoryReset(); break;
        case 2: /* battery info is shown on this screen already */ break;
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
    case BOOT_SCREEN: drawBootScreen(); break;
    case HOME_SCREEN: drawHomeScreen(); break;
    case MENU_SCREEN: drawMainMenu(); break;
    case VEHICLE_SCREEN: drawVehicleScreen(); break;
    case CALIBRATION_SCREEN: drawCalibrationScreen(); break;
    case TRIM_SCREEN: drawTrimScreen(); break;
    case RADIO_SCREEN: drawRadioScreen(); break;
    case DISPLAY_SCREEN: drawDisplayScreen(); break;
    case SYSTEM_SCREEN: drawSystemScreen(); break;
    case ABOUT_SCREEN: drawAboutScreen(); break;
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
    case DRONE: tft.print("DRONE"); break;
    case CAR: tft.print("CAR"); break;
    case PLANE: tft.print("PLANE"); break;
    }

    tft.setCursor(150, 5);
    tft.setTextSize(1);
    tft.print((tx.flags & 0x01) ? "ARMED" : "SAFE");

    tft.setTextSize(2);
    switch (homePage)
    {
    case 0:
        tft.setCursor(5, 35); tft.print("THR : "); tft.println(tx.throttle);
        tft.setCursor(5, 60); tft.print("ROL : "); tft.println(tx.roll);
        tft.setCursor(5, 85); tft.print("PIT : "); tft.println(tx.pitch);
        tft.setCursor(5, 110); tft.print("YAW : "); tft.println(tx.yaw);
        break;
    case 1:
        tft.setCursor(5, 35); tft.print("BATT");
        tft.setTextSize(1);
        tft.setCursor(5, 65); tft.print(batteryVoltage); tft.println("V");
        tft.setCursor(5, 85); tft.print(batteryPercent); tft.println("%");
        break;
    case 2:
        tft.setTextSize(1);
        tft.setCursor(5, 35); tft.print("CH : "); tft.println(radioChannel);
        tft.setCursor(5, 55); tft.print("PWR : "); tft.println(radioPower);
        tft.setCursor(5, 75); tft.print("RATE: "); tft.println(radioDataRate);
        break;
    case 3:
        tft.setTextSize(1);
        tft.setCursor(5, 35); tft.print("L: "); tft.print(rawLeftX); tft.print(","); tft.println(rawLeftY);
        tft.setCursor(5, 55); tft.print("R: "); tft.print(rawRightX); tft.print(","); tft.println(rawRightY);
        break;
    }
}

void drawMainMenu()
{
    tft.setTextColor(ST77XX_WHITE);
    tft.setTextSize(2);
    tft.setCursor(25, 5);
    tft.println("MAIN MENU");
    tft.setTextSize(1);

    for (uint8_t i = 0; i < TOTAL_MENU_ITEMS; i++)
    {
        tft.setCursor(10, 30 + (i * 16));
        tft.print((i == menuIndex) ? "> " : "  ");
        tft.println(mainMenu[i]);
    }
}

void drawVehicleScreen()
{
    tft.setTextColor(ST77XX_WHITE);
    tft.setTextSize(2);
    tft.setCursor(20, 5);
    tft.println("VEHICLE");
    for (uint8_t i = 0; i < 3; i++)
    {
        tft.setCursor(20, 45 + (i * 30));
        tft.print((i == tx.mode) ? "> " : "  ");
        tft.println(vehicleMenu[i]);
    }
}

void drawCalibrationScreen()
{
    tft.setTextColor(ST77XX_WHITE);
    tft.setTextSize(2);
    tft.setCursor(10, 5);
    tft.println("CALIBRATION");
    tft.setTextSize(1);
    for (uint8_t i = 0; i < TOTAL_AXIS; i++)
    {
        tft.setCursor(10, 35 + (i * 15));
        tft.print((i == calibrationAxis) ? "> " : "  ");
        tft.println(calibrationMenu[i]);
    }

    tft.setCursor(120, 35); tft.print(rawLeftX);
    tft.setCursor(120, 50); tft.print(rawLeftY);
    tft.setCursor(120, 65); tft.print(rawRightX);
    tft.setCursor(120, 80); tft.print(rawRightY);

    tft.setCursor(10, 110);
    tft.println("FUNC=next  ENC=center");
    tft.setCursor(10, 125);
    tft.println("MODE=save & exit");
}

void drawTrimScreen()
{
    tft.setTextColor(ST77XX_WHITE);
    tft.setTextSize(2);
    tft.setCursor(35, 5);
    tft.println("TRIMS");
    tft.setTextSize(1);

    for (uint8_t i = 0; i < TOTAL_TRIMS; i++)
    {
        tft.setCursor(10, 35 + (i * 20));
        tft.print((i == trimIndex) ? "> " : "  ");
        tft.print(trimMenu[i]);
        tft.print(" : ");
        switch (i)
        {
        case 0: tft.println(rollTrim); break;
        case 1: tft.println(pitchTrim); break;
        case 2: tft.println(yawTrim); break;
        case 3: tft.println(throttleTrim); break;
        }
    }
}

void drawRadioScreen()
{
    tft.setTextColor(ST77XX_WHITE);
    tft.setTextSize(2);
    tft.setCursor(35, 5);
    tft.println("RADIO");
    tft.setTextSize(1);

    for (uint8_t i = 0; i < TOTAL_RADIO_ITEMS; i++)
    {
        tft.setCursor(10, 35 + (i * 20));
        tft.print((i == radioIndex) ? "> " : "  ");
        tft.print(radioMenu[i]);
        tft.print(" : ");
        switch (i)
        {
        case 0: tft.println(radioChannel); break;
        case 1: tft.println(radioPower); break;
        case 2: tft.println(radioDataRate); break;
        }
    }
}

void drawDisplayScreen()
{
    tft.setTextColor(ST77XX_WHITE);
    tft.setTextSize(2);
    tft.setCursor(25, 5);
    tft.println("DISPLAY");
    tft.setTextSize(1);

    for (uint8_t i = 0; i < TOTAL_DISPLAY_ITEMS; i++)
    {
        tft.setCursor(10, 35 + (i * 20));
        tft.print((i == displayIndex) ? "> " : "  ");
        tft.print(displayMenu[i]);
        tft.print(" : ");
        switch (i)
        {
        case 0: tft.println(brightness); break;
        case 1: tft.println(rotation); break;
        case 2: tft.println((theme == 0) ? "Dark" : "Light"); break;
        }
    }
}

void drawSystemScreen()
{
    tft.setTextColor(ST77XX_WHITE);
    tft.setTextSize(2);
    tft.setCursor(30, 5);
    tft.println("SYSTEM");
    tft.setTextSize(1);
    for (uint8_t i = 0; i < TOTAL_SYSTEM_ITEMS; i++)
    {
        tft.setCursor(10, 35 + (i * 20));
        tft.print((i == systemIndex) ? "> " : "  ");
        tft.println(systemMenu[i]);
    }
    if (systemIndex == 2)
    {
        tft.setCursor(10, 100);
        tft.print(batteryVoltage); tft.println("V");
        tft.setCursor(10, 115);
        tft.print(batteryPercent); tft.println("%");
    }
}

void drawAboutScreen()
{
    tft.setTextColor(ST77XX_WHITE);
    tft.setTextSize(2);
    tft.setCursor(40, 5);
    tft.println("ABOUT");
    tft.setTextSize(1);
    tft.setCursor(10, 40); tft.println("Universal RC");
    tft.setCursor(10, 60); tft.println("Version : 1.0");
    tft.setCursor(10, 80); tft.println("Board : Arduino Nano");
    tft.setCursor(10, 100); tft.println("Radio : NRF24L01");
    tft.setCursor(10, 120); tft.println("Display : ST7789");
    tft.setCursor(10, 140); tft.println("By : Popoye");
}

void sendPacket()
{
    radio.write(&tx, sizeof(tx));
}

void updateBattery()
{
    batteryADC = analogRead(BATTERY_PIN);
    batteryVoltage = (batteryADC * 5.0) / 1023.0;
    batteryPercent = map(batteryADC, 0, 1023, 0, 100);
    batteryPercent = constrain(batteryPercent, 0, 100);
}

void saveSettings()
{
    EEPROM.put(0, tx.mode);

    EEPROM.put(10, rollTrim);
    EEPROM.put(20, pitchTrim);
    EEPROM.put(30, yawTrim);
    EEPROM.put(40, throttleTrim);

    EEPROM.put(50, radioChannel);
    EEPROM.put(60, radioPower);
    EEPROM.put(70, radioDataRate);

    EEPROM.put(80, brightness);
    EEPROM.put(90, rotation);
    EEPROM.put(100, theme);

    // Joystick calibration - the original file never persisted this at all.
    EEPROM.put(110, joy1X_Min);
    EEPROM.put(120, joy1X_Center);
    EEPROM.put(130, joy1X_Max);
    EEPROM.put(140, joy1Y_Min);
    EEPROM.put(150, joy1Y_Center);
    EEPROM.put(160, joy1Y_Max);
    EEPROM.put(170, joy2X_Min);
    EEPROM.put(180, joy2X_Center);
    EEPROM.put(190, joy2X_Max);
    EEPROM.put(200, joy2Y_Min);
    EEPROM.put(210, joy2Y_Center);
    EEPROM.put(220, joy2Y_Max);
}

void loadSettings()
{
    EEPROM.get(0, tx.mode);
    EEPROM.get(10, rollTrim);
    EEPROM.get(20, pitchTrim);
    EEPROM.get(30, yawTrim);
    EEPROM.get(40, throttleTrim);

    EEPROM.get(50, radioChannel);
    EEPROM.get(60, radioPower);
    EEPROM.get(70, radioDataRate);

    EEPROM.get(80, brightness);
    EEPROM.get(90, rotation);
    EEPROM.get(100, theme);

    EEPROM.get(110, joy1X_Min);
    EEPROM.get(120, joy1X_Center);
    EEPROM.get(130, joy1X_Max);
    EEPROM.get(140, joy1Y_Min);
    EEPROM.get(150, joy1Y_Center);
    EEPROM.get(160, joy1Y_Max);
    EEPROM.get(170, joy2X_Min);
    EEPROM.get(180, joy2X_Center);
    EEPROM.get(190, joy2X_Max);
    EEPROM.get(200, joy2Y_Min);
    EEPROM.get(210, joy2Y_Center);
    EEPROM.get(220, joy2Y_Max);

    // Sanity check: a blank/never-calibrated EEPROM reads back as garbage
    // (0xFFFF -> -1, etc). Fall back to safe defaults rather than mapping
    // against a broken range.
    if (joy1X_Min >= joy1X_Max || joy1X_Center < joy1X_Min || joy1X_Center > joy1X_Max)
    {
        joy1X_Min = 0; joy1X_Center = 512; joy1X_Max = 1023;
    }
    if (joy1Y_Min >= joy1Y_Max || joy1Y_Center < joy1Y_Min || joy1Y_Center > joy1Y_Max)
    {
        joy1Y_Min = 0; joy1Y_Center = 512; joy1Y_Max = 1023;
    }
    if (joy2X_Min >= joy2X_Max || joy2X_Center < joy2X_Min || joy2X_Center > joy2X_Max)
    {
        joy2X_Min = 0; joy2X_Center = 512; joy2X_Max = 1023;
    }
    if (joy2Y_Min >= joy2Y_Max || joy2Y_Center < joy2Y_Min || joy2Y_Center > joy2Y_Max)
    {
        joy2Y_Min = 0; joy2Y_Center = 512; joy2Y_Max = 1023;
    }
    if (radioChannel > 125) radioChannel = 100;
    if (radioPower > 3) radioPower = 3;
    if (radioDataRate > 2) radioDataRate = 1;
    if (rotation > 3) rotation = 1;
}

void failSafe()
{
    // Only used as a last-resort clear if sendPacket() somehow doesn't run
    // for a while; normal operation always re-fills these from
    // readJoyStick() every loop before this is called.
    if (millis() - lastPacketTime > 1000)
    {
        tx.throttle = 1000;
        tx.roll = 1500;
        tx.pitch = 1500;
        tx.yaw = 1500;
        tx.flags = 0;
    }
}

bool initNRF24()
{
    if (!radio.begin())
    {
        return false;
    }
    radio.setPALevel(RF24_PA_HIGH);
    radio.setDataRate(RF24_1MBPS);
    radio.setChannel(radioChannel);
    radio.openWritingPipe(DRONE_ADD);
    radio.stopListening();
    return true;
}

void initDisplay()
{
    tft.init(240, 240);
    tft.setRotation(rotation);
    tft.fillScreen(ST77XX_BLACK);
}

void processBootScreen()
{
    if (millis() - bootStartTime >= 3000)
    {
        currentScreen = HOME_SCREEN;
    }
}

void updateRadioSettings()
{
    radio.setChannel(radioChannel);
    switch (radioPower)
    {
    case 0: radio.setPALevel(RF24_PA_MIN); break;
    case 1: radio.setPALevel(RF24_PA_LOW); break;
    case 2: radio.setPALevel(RF24_PA_HIGH); break; // was missing this break - selecting HIGH silently became MAX
    case 3: radio.setPALevel(RF24_PA_MAX); break;
    }

    switch (radioDataRate)
    {
    case 0: radio.setDataRate(RF24_250KBPS); break;
    case 1: radio.setDataRate(RF24_1MBPS); break;
    case 2: radio.setDataRate(RF24_2MBPS); break;
    }
}

void applyDisplaySettings()
{
    tft.setRotation(rotation);
}

void resetSettings()
{
    tx.mode = DRONE;
    rollTrim = 0;
    pitchTrim = 0;
    yawTrim = 0;
    throttleTrim = 0;

    radioChannel = 100;
    radioPower = 3;
    radioDataRate = 1;

    brightness = 100;
    rotation = 1;
    theme = 0;

    joy1X_Min = 0; joy1X_Center = 512; joy1X_Max = 1023;
    joy1Y_Min = 0; joy1Y_Center = 512; joy1Y_Max = 1023;
    joy2X_Min = 0; joy2X_Center = 512; joy2X_Max = 1023;
    joy2Y_Min = 0; joy2Y_Center = 512; joy2Y_Max = 1023;

    selectVehicleAddress(tx.mode);
    updateRadioSettings();
    applyDisplaySettings();
    saveSettings();
}

void factoryReset()
{
    resetSettings();
    saveSettings();
}
