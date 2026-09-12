#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <RF24.h>
#include <MPU6050.h>
#include <EEPROM.h>

// ===============================================================================
// This is the DRONE-SIDE flight controller / receiver.
// It listens for packets from the handheld transmitter and drives the 4 motors.
// ===============================================================================

// NRF24
#define CE_PIN 7
#define CSN_PIN 8

// Motors (must be PWM-capable pins on Uno/Nano: 3,5,6,9,10,11)
#define MOTOR_FL 3
#define MOTOR_FR 5
#define MOTOR_BL 6
#define MOTOR_BR 9

// Battery sense pin - was missing in the original file.
// Confirm this matches the analog pin your voltage divider is wired to.
#define BATTERY_PIN A0

const byte droneAdd[8] = "DRN3458";

typedef struct
{
    uint16_t throttle; // 1000-2000, matches transmitter
    int16_t roll;       // 1000-2000, centered at 1500
    int16_t pitch;       // 1000-2000, centered at 1500
    int16_t yaw;       // 1000-2000, centered at 1500

    uint8_t mode;
    uint8_t flags;
} txData;

RF24 radio(CE_PIN, CSN_PIN);
MPU6050 mpu;
txData rx;

// ---------------------------
// Radio
// ---------------------------
bool radioConnected = false;
unsigned long lastPacketTime = 0;
const uint16_t FAILSAFE_TIME = 500;

// ---------------------------
// MPU6050 raw readings
// ---------------------------
int16_t ax = 0, ay = 0, az = 0;
int16_t gx = 0, gy = 0, gz = 0;

// ---------------------------
// Angle (complementary filter)
// ---------------------------
float rollAngle = 0.0;
float pitchAngle = 0.0;
float gyroRoll = 0.0;   // was missing -> compile error in the original file
float gyroPitch = 0.0;  // was missing -> compile error in the original file
unsigned long previousAngleTime = 0;

// ---------------------------
// PID - Roll / Pitch
// ---------------------------
float rollError = 0.0, pitchError = 0.0;
float previousRollError = 0.0, previousPitchError = 0.0;
float rollIntegral = 0.0, pitchIntegral = 0.0;
float rollDerivative = 0.0, pitchDerivative = 0.0;
float rollPID = 0.0, pitchPID = 0.0;

// ---------------------------
// PID - Yaw (rate controlled, no absolute yaw angle without a compass)
// ---------------------------
float yawRateError = 0.0;
float previousYawRateError = 0.0;
float yawIntegral = 0.0;
float yawDerivative = 0.0;
float yawPID = 0.0;

// PID Gains
const float KP = 1.50;
const float KI = 0.02;
const float KD = 0.60;
const float YAW_KP = 2.0;
const float YAW_KI = 0.01;
const float YAW_KD = 0.2;

unsigned long previousPIDTime = 0;

bool armed = false;

enum FlightMode
{
    ACRO_MODE,
    STABILIZE_MODE
};
FlightMode currentFlightMode = STABILIZE_MODE;

// ---------------------------
// Motor Output (0-255, final PWM values only)
// ---------------------------
uint8_t motorFL = 0, motorFR = 0, motorBL = 0, motorBR = 0;

// Calibration offsets are stored in the SAME units as the values they
// are subtracted from. rollOffset/pitchOffset are degrees (they correct
// the complementary-filter angle), yawOffset is raw gyro LSB (it corrects
// gyro-z bias for yaw rate).
float rollOffset = 0.0;
float pitchOffset = 0.0;
int16_t yawOffset = 0;

// Stick -> angle/rate scaling
const float MAX_ANGLE = 30.0;     // deg, stabilize mode stick-to-angle range
const float MAX_YAW_RATE = 150.0; // deg/s, stick-to-yaw-rate range

// ---------------------------
// Battery
// ---------------------------
uint16_t batteryADC = 0;
float batteryVoltage = 0.0;

void readMPU();
void failsafe();
void mixMotors();
void armMotors();
bool initNRF24();
void stopMotors();
void initMPU6050();
void calculatePID();
void updateMotors();
void calibrateMPU();
void disarmMotors();
void receivePacket();
void updateBattery();
void calculateAngles();
void saveCalibration();
void loadCalibration();
void processArmState();
void updateFlightMode();
void batteryFailsafe(); // was missing a prototype -> used before declared

void setup()
{
    Serial.begin(9600);
    Wire.begin();
    pinMode(MOTOR_FL, OUTPUT);
    pinMode(MOTOR_FR, OUTPUT);
    pinMode(MOTOR_BL, OUTPUT);
    pinMode(MOTOR_BR, OUTPUT);
    stopMotors();
    if (!initNRF24())
    {
        Serial.println("NRF24 Initialization Failed!");
        while (1)
            ;
    }
    Serial.println("NRF24 Initialized.");
    initMPU6050();
    loadCalibration();
    previousAngleTime = millis();
    previousPIDTime = millis();
    lastPacketTime = millis();
}

void loop()
{
    receivePacket();
    processArmState();
    updateFlightMode();
    readMPU();
    calculateAngles();
    calculatePID();
    mixMotors();
    updateMotors();
    updateBattery();
    batteryFailsafe();
    failsafe();
}

bool initNRF24()
{
    if (!radio.begin())
    {
        return false;
    }
    radio.setPALevel(RF24_PA_HIGH);
    radio.setDataRate(RF24_1MBPS);
    radio.setChannel(100);
    radio.setCRCLength(RF24_CRC_16);
    // Auto-ack must match the transmitter (it enables setAutoAck(true) and
    // setRetries). Leaving this false here means the TX will burn through
    // its retry budget on every single packet waiting for an ACK that this
    // side is not configured to send.
    radio.setAutoAck(true);
    radio.openReadingPipe(1, droneAdd);
    radio.startListening();
    return true;
}

void initMPU6050()
{
    mpu.initialize();
    if (!mpu.testConnection())
    {
        Serial.println("MPU6050 Initialization Failed!");
        while (1)
            ;
    }
    mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
    Serial.println("MPU6050 Initialized.");
}

void receivePacket()
{
    if (radio.available())
    {
        radio.read(&rx, sizeof(rx));
        lastPacketTime = millis();
        radioConnected = true;
    }
    // Arming itself is handled only in processArmState() now (see note
    // there) - this function's job is just to pull the packet in.
}

void readMPU()
{
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
}

void stopMotors()
{
    motorFL = 0;
    motorFR = 0;
    motorBL = 0;
    motorBR = 0;
    analogWrite(MOTOR_FL, 0);
    analogWrite(MOTOR_FR, 0);
    analogWrite(MOTOR_BL, 0);
    analogWrite(MOTOR_BR, 0);
}

void calculateAngles()
{
    unsigned long currentTime = millis();
    float dt = (currentTime - previousAngleTime) / 1000.0;
    previousAngleTime = currentTime;
    if (dt <= 0.0)
    {
        return;
    }
    float accelRoll = atan2(ay, az) * 57.2958;
    float accelPitch = atan2(-ax, sqrt((long)ay * ay + (long)az * az)) * 57.2958;

    gyroRoll += (gx / 131.0) * dt;
    gyroPitch += (gy / 131.0) * dt;

    rollAngle = 0.98 * gyroRoll + 0.02 * accelRoll;
    pitchAngle = 0.98 * gyroPitch + 0.02 * accelPitch;

    rollAngle -= rollOffset;
    pitchAngle -= pitchOffset;

    // Both filter states must be re-synced with their OWN angle, not both
    // with rollAngle - the original code set gyroPitch = rollAngle, which
    // fed roll data into the pitch axis every single loop.
    gyroRoll = rollAngle;
    gyroPitch = pitchAngle;
}

void calculatePID()
{
    unsigned long currentTime = millis();
    float dt = (currentTime - previousPIDTime) / 1000.0;
    previousPIDTime = currentTime;
    if (dt <= 0.0)
    {
        return;
    }

    // Stick inputs arrive as 1000-2000 (centered on 1500). They must be
    // converted into a real setpoint before being compared against an
    // angle in degrees or a rate in deg/s - the original code compared
    // rx.roll (1000-2000) directly against rollAngle (~-90..90), which
    // meant full stick deflection always saturated the PID output.
    float rollSetpoint;
    float pitchSetpoint;
    float yawRateSetpoint = map(rx.yaw, 1000, 2000, -MAX_YAW_RATE, MAX_YAW_RATE);
    float actualYawRate = gz / 131.0;

    if (currentFlightMode == STABILIZE_MODE)
    {
        rollSetpoint = map(rx.roll, 1000, 2000, -MAX_ANGLE, MAX_ANGLE);
        pitchSetpoint = map(rx.pitch, 1000, 2000, -MAX_ANGLE, MAX_ANGLE);
        rollError = rollSetpoint - rollAngle;
        pitchError = pitchSetpoint - pitchAngle;
    }
    else // ACRO_MODE: sticks command rotation rate, not angle
    {
        rollSetpoint = map(rx.roll, 1000, 2000, -MAX_YAW_RATE, MAX_YAW_RATE);
        pitchSetpoint = map(rx.pitch, 1000, 2000, -MAX_YAW_RATE, MAX_YAW_RATE);
        rollError = rollSetpoint - (gx / 131.0);
        pitchError = pitchSetpoint - (gy / 131.0);
    }

    rollIntegral += rollError * dt;
    pitchIntegral += pitchError * dt;
    rollIntegral = constrain(rollIntegral, -200.0, 200.0);
    pitchIntegral = constrain(pitchIntegral, -200.0, 200.0);

    rollDerivative = (rollError - previousRollError) / dt;
    pitchDerivative = (pitchError - previousPitchError) / dt;

    rollPID = (KP * rollError) + (KI * rollIntegral) + (KD * rollDerivative);
    pitchPID = (KP * pitchError) + (KI * pitchIntegral) + (KD * pitchDerivative);

    previousRollError = rollError;
    previousPitchError = pitchError;

    // Yaw rate PID (this axis was declared but never actually computed
    // or used in the original file)
    yawRateError = yawRateSetpoint - actualYawRate;
    yawIntegral += yawRateError * dt;
    yawIntegral = constrain(yawIntegral, -200.0, 200.0);
    yawDerivative = (yawRateError - previousYawRateError) / dt;
    yawPID = (YAW_KP * yawRateError) + (YAW_KI * yawIntegral) + (YAW_KD * yawDerivative);
    previousYawRateError = yawRateError;
}

void mixMotors()
{
    // rx.throttle (1000-2000) must be scaled into the 0-255 PWM range
    // used by analogWrite - the original code fed the raw 1000-2000
    // value straight into a variable that was later clamped to 0-255,
    // so it was pinned near max throttle almost all the time.
    float throttleOut = map(rx.throttle, 1000, 2000, 0, 255);

    // Do the mixing math in a signed float, NOT in the unsigned motor
    // output variables. Assigning a negative value directly into an
    // unsigned/uint8_t variable wraps around to a huge number, and
    // constrain() would then clamp it to 255 instead of 0 - i.e. a motor
    // that should spin down could instead jump to full throttle.
    //
    // X-frame mixing with yaw: FL/BR spin one direction, FR/BL the other
    // (verify this matches your actual prop spin directions and flip the
    // yawPID sign for any motor that fights the intended yaw rotation).
    float flOut = throttleOut + pitchPID - rollPID + yawPID;
    float frOut = throttleOut + pitchPID + rollPID - yawPID;
    float blOut = throttleOut - pitchPID - rollPID - yawPID;
    float brOut = throttleOut - pitchPID + rollPID + yawPID;

    flOut = constrain(flOut, 0, 255);
    frOut = constrain(frOut, 0, 255);
    blOut = constrain(blOut, 0, 255);
    brOut = constrain(brOut, 0, 255);

    motorFL = (uint8_t)flOut;
    motorFR = (uint8_t)frOut;
    motorBL = (uint8_t)blOut;
    motorBR = (uint8_t)brOut;
}

void updateMotors()
{
    // Light smoothing to avoid abrupt PWM jumps between loops.
    static uint8_t lastFL = 0, lastFR = 0, lastBL = 0, lastBR = 0;

    if (!armed)
    {
        stopMotors();
        lastFL = lastFR = lastBL = lastBR = 0;
        return;
    }

    lastFL = (uint8_t)((lastFL * 3 + motorFL) / 4);
    lastFR = (uint8_t)((lastFR * 3 + motorFR) / 4);
    lastBL = (uint8_t)((lastBL * 3 + motorBL) / 4);
    lastBR = (uint8_t)((lastBR * 3 + motorBR) / 4);

    // The original code computed the smoothed lastFL/lastFR/... values
    // and then wrote the *unsmoothed* motorFL/... to the pins, so the
    // smoothing had no effect. Write the smoothed values instead.
    analogWrite(MOTOR_FL, lastFL);
    analogWrite(MOTOR_FR, lastFR);
    analogWrite(MOTOR_BL, lastBL);
    analogWrite(MOTOR_BR, lastBR);
}

void failsafe()
{
    if (millis() - lastPacketTime > FAILSAFE_TIME)
    {
        if (radioConnected)
        {
            // Disarm and clear the stale packet so that if the link comes
            // back the drone doesn't instantly resume whatever throttle
            // was last received.
            disarmMotors();
            rx.throttle = 1000;
            rx.roll = 1500;
            rx.pitch = 1500;
            rx.yaw = 1500;
            rx.flags = 0;
        }
        radioConnected = false;
        stopMotors();
    }
}

void loadCalibration()
{
    EEPROM.get(0, rollOffset);
    EEPROM.get(sizeof(rollOffset), pitchOffset);
    EEPROM.get(sizeof(rollOffset) + sizeof(pitchOffset), yawOffset);
}

void saveCalibration()
{
    EEPROM.put(0, rollOffset);
    EEPROM.put(sizeof(rollOffset), pitchOffset);
    EEPROM.put(sizeof(rollOffset) + sizeof(pitchOffset), yawOffset);
}

void calibrateMPU()
{
    // Average the *angle* the accelerometer reports while sitting level,
    // not the raw accelerometer counts - rollAngle/pitchAngle are in
    // degrees, so the offset subtracted from them must be in degrees too.
    // (The original code averaged raw ax into rollOffset and raw ay into
    // pitchOffset - both the wrong units, thousands of LSBs instead of a
    // few degrees, and swapped: ax feeds pitch, ay feeds roll.)
    float rollSum = 0.0, pitchSum = 0.0;
    int32_t yawSum = 0;
    for (int i = 0; i < 500; i++)
    {
        readMPU();
        float accelRoll = atan2(ay, az) * 57.2958;
        float accelPitch = atan2(-ax, sqrt((long)ay * ay + (long)az * az)) * 57.2958;
        rollSum += accelRoll;
        pitchSum += accelPitch;
        yawSum += gz;
        delay(2);
    }
    rollOffset = rollSum / 500.0;
    pitchOffset = pitchSum / 500.0;
    yawOffset = (int16_t)(yawSum / 500);
    saveCalibration();
}

void armMotors()
{
    armed = true;
    rollIntegral = 0;
    pitchIntegral = 0;
    yawIntegral = 0;
    previousRollError = 0;
    previousPitchError = 0;
    previousYawRateError = 0;
}

void disarmMotors()
{
    armed = false;
    stopMotors();
    rollIntegral = 0;
    pitchIntegral = 0;
    yawIntegral = 0;
    previousRollError = 0;
    previousPitchError = 0;
    previousYawRateError = 0;
}

void processArmState()
{
    // Single source of truth for arming: the flags bit from the
    // transmitter's arm switch. The original file also force-armed or
    // force-disarmed based on rx.throttle == 0 inside receivePacket(),
    // which fought with this function and made the arm state depend on
    // whichever check happened to run last.
    bool armSwitch = (rx.flags & 0x01);
    if (armSwitch)
    {
        // Standard safety interlock: refuse to arm unless the throttle
        // stick is down. Prevents an accidental spin-up at high throttle.
        if (!armed && rx.throttle <= 1050)
        {
            armMotors();
        }
    }
    else
    {
        if (armed)
        {
            disarmMotors();
        }
    }
}

void updateBattery()
{
    batteryADC = analogRead(BATTERY_PIN);
    // Assumes a 2:1 voltage divider - confirm this against your actual
    // resistor values before trusting the failsafe threshold below.
    batteryVoltage = (batteryADC * 5.0 / 1023.0) * 2.0;
}

void updateFlightMode()
{
    if (rx.mode == 0)
    {
        currentFlightMode = STABILIZE_MODE;
    }
    else
    {
        currentFlightMode = ACRO_MODE;
    }
}

void batteryFailsafe()
{
    if (batteryVoltage <= 3.3f)
    {
        disarmMotors();
        stopMotors();
    }
}
