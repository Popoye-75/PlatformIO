#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <RF24.h>
#include <MPU6050.h>
#include <EEPROM.h>

// NRF24
#define CE_PIN 7
#define CSN_PIN 8

// Motors
#define MOTOR_FL 3
#define MOTOR_FR 5
#define MOTOR_BL 6
#define MOTOR_BR 9

const byte droneAdd[8] = "DRN3458";

typedef struct
{
    uint16_t throttle;
    int16_t roll;
    int16_t pitch;
    int16_t yaw;

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
// MPU6050
// ---------------------------
int16_t ax = 0;
int16_t ay = 0;
int16_t az = 0;
int16_t gx = 0;
int16_t gy = 0;
int16_t gz = 0;

// ---------------------------
// Angle
// ---------------------------
float rollAngle = 0.0;
float pitchAngle = 0.0;
unsigned long previousAngleTime = 0;

// ---------------------------
// PID
// ---------------------------
// ---------------------------
// PID
// ---------------------------
float rollError = 0.0;
float pitchError = 0.0;
float previousRollError = 0.0;
float previousPitchError = 0.0;
float rollIntegral = 0.0;
float pitchIntegral = 0.0;
float rollDerivative = 0.0;
float pitchDerivative = 0.0;
float rollPID = 0.0;
float pitchPID = 0.0;

// ---------------------------
// Calibration
// ---------------------------
float yawAngle = 0.0;
float yawError = 0.0;
float yawIntegral = 0.0;
float yawDerivative = 0.0;
float previousYawError = 0.0;
float yawPID = 0.0;

// PID Gain
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
// Motor Output
// ---------------------------
uint16_t motorFL = 0;
uint16_t motorFR = 0;
uint16_t motorBL = 0;
uint16_t motorBR = 0;


int16_t rollOffset = 0;
int16_t pitchOffset = 0;
int16_t yawOffset = 0;

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
    radio.setAutoAck(false);
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
    if (rx.throttle == 0)
    {
        disarmMotors();
    }
    else
    {
        armMotors();
    }
}

void readMPU()
{
    mpu.getMotion6(
        &ax,
        &ay,
        &az,
        &gx,
        &gy,
        &gz);
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
    gyroRoll = rollAngle;
    gyroPitch = rollAngle;
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
    rollError = rx.roll - rollAngle;
    pitchError = rx.pitch - pitchAngle;
    rollIntegral += rollError * dt;
    pitchIntegral += pitchError * dt;
    rollIntegral = constrain(rollIntegral, -200.0, 200.0);
    pitchIntegral = constrain(pitchIntegral, -200.0, 200.0);

    rollDerivative = (rollError - previousRollError) / dt;
    pitchDerivative = (pitchError - previousPitchError) / dt;
    rollPID =
        (KP * rollError) +
        (KI * rollIntegral) +
        (KD * rollDerivative);
    pitchPID =
        (KP * pitchError) +
        (KI * pitchIntegral) +
        (KD * pitchDerivative);
        
    previousRollError = rollError;
    previousPitchError = pitchError;
}

void mixMotors()
{
    motorFL = rx.throttle + pitchPID - rollPID;
    motorFR = rx.throttle + pitchPID + rollPID;
    motorBL = rx.throttle - pitchPID - rollPID;
    motorBR = rx.throttle - pitchPID + rollPID;
    motorFL = constrain(motorFL, 0, 255);
    motorFR = constrain(motorFR, 0, 255);
    motorBL = constrain(motorBL, 0, 255);
    motorBR = constrain(motorBR, 0, 255);
}

void updateMotors()
{
    static uint16_t lastFL = 0;
    static uint16_t lastFR = 0;
    static uint16_t lastBL = 0;
    static uint16_t lastBR = 0;

    if (!armed)
    {
        stopMotors();
        return;
    }

    lastFL = (lastFL * 3 + motorFL) / 4;
    lastFR = (lastFR * 3 + motorFR) / 4;
    lastBL = (lastBL * 3 + motorBL) / 4;
    lastBR = (lastBR * 3 + motorBR) / 4;

    analogWrite(MOTOR_FL, motorFL);
    analogWrite(MOTOR_FR, motorFR);
    analogWrite(MOTOR_BL, motorBL);
    analogWrite(MOTOR_BR, motorBR);
}

void failsafe()
{
    if (millis() - lastPacketTime > FAILSAFE_TIME)
    {
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
    rollOffset = 0;
    pitchOffset = 0;
    yawOffset = 0;
    for (int i = 0; i < 500; i++)
    {
        readMPU();
        rollOffset += ax;
        pitchOffset += ay;
        yawOffset += gz;
        delay(2);
    }
    rollOffset /= 500;
    pitchOffset /= 500;
    yawOffset /= 500;
    saveCalibration();
}

void armMotors()
{
    armed = true;
    rollIntegral = 0;
    pitchIntegral = 0;
    previousRollError = 0;
    previousPitchError = 0;
}

void disarmMotors()
{
    armed = false;
    stopMotors();
    rollIntegral = 0;
    pitchIntegral = 0;
    previousRollError = 0;
    previousPitchError = 0;
}
void processArmState()
{
    bool armSwitch = (rx.flags & 0x01);
    if (armSwitch)
    {
        if (!armed)
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

void updateMotors()
{
    if (!armed)
    {
        stopMotors();
        return;
    }
    analogWrite(MOTOR_FL, motorFL);
    analogWrite(MOTOR_FR, motorFR);
    analogWrite(MOTOR_BL, motorBL);
    analogWrite(MOTOR_BR, motorBR);
}

void updateBattery()
{
    batteryADC = analogRead(BATTERY_PIN);
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

void batteryFailsafe(){
    if(batteryVoltage <= 3.3f){
        disarmMotors();
        radioConnected = false;
        stopMotors();
    }
}

void saveCalibration()
{
    EEPROM.put(0, rollOffset);
    EEPROM.put(sizeof(rollOffset), pitchOffset);
    EEPROM.put(sizeof(rollOffset) + sizeof(pitchOffset), yawOffset);
}