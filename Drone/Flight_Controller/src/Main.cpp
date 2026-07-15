#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>
#include <Wire.h>
#include <MPU6050.h>
#include <EEPROM.h>

#define CE_PIN 7
#define CSN_PIN 8

#define MOTOR_FL 3
#define MOTOR_FR 5
#define MOTOR_BL 6
#define MOTOR_BR 9

typedef struct
{
    uint16_t throttle;
    uint16_t roll;
    uint16_t pitch;
    uint16_t yaw;

    uint16_t mode;
    uint16_t flags;
} txData;

// Objects
RF24 radio(CE_PIN, CSN_PIN);
MPU6050 mpu;
txData rx;

// Global variable

int16_t ax = 0;
int16_t ay = 0;
int16_t az = 0;

int16_t gx = 0;
int16_t gy = 0;
int16_t gz = 0;

uint16_t motorFL = 0;
uint16_t motorFR = 0;
uint16_t motorBL = 0;
uint16_t motorBR = 0;

bool radioConnected = false;
unsigned long lastPacketTime = 0;

const uint16_t FAILSAFE_TIME = 500;

int16_t rollCorrection = 0;
int16_t pitchCorrection = 0;
int16_t yawCorrection = 0;

float batteryVoltage = 0.0;
uint16_t batteryADC = 0;

int16_t rollOffset = 0;
int16_t pitchOffset = 0;
int16_t yawOffset = 0;

void initNRF24();
void initMPU6050();
void receivePacket();
void readMPU();
void calculatePID();
void mixMotors();
void updateMotors();
void failSafe();
void saveCalibration();
void loadCalibration();

void setup()
{
    Serial.begin(115200);
    Wire.begin();
    if (!initNRF24)
    {
        Serial.println("NRF24 Initialization failed !");
        while (1)
            ;
    }
    Serial.println("NRF24 Initialized ");
    initMPU6050();
    loadCalibration();

    pinMode(MOTOR_FL, OUTPUT);
    pinMode(MOTOR_FR, OUTPUT);
    pinMode(MOTOR_BL, OUTPUT);
    pinMode(MOTOR_BR, OUTPUT);
    pinMode(MOTOR_FL, 0);
    pinMode(MOTOR_FR, 0);
    pinMode(MOTOR_BL, 0);
    pinMode(MOTOR_BR, 0);
}