#include <Arduino.h>
#include <Wire.h>
#include <math.h>

// ================= MPU6050 =================
#define MPU 0x68

// ================= MOTOR PINS =================
#define ENA 10
#define IN1 8
#define IN2 9

#define ENB 5
#define IN3 6
#define IN4 7

// ================= PID PARAMETERS =================
float Kp = 22.0;
float Ki = 0.0;
float Kd = 0.9;

float targetAngle = 0.0;

// ================= DIRECTION SETTINGS =================

// Change these if required after testing
#define MOTOR_SIGN 1
#define ANGLE_SIGN 1

// Individual motor wiring directions
#define LEFT_MOTOR_SIGN 1
#define RIGHT_MOTOR_SIGN 1

// ================= VARIABLES =================
int16_t ax, ay, az;
int16_t gx, gy, gz;

float gyroOffsetY = 0;
float angle = 0;

float error = 0;
float previousError = 0;
float integral = 0;
float pidOutput = 0;

unsigned long previousTime = 0;

const float LOOP_TIME = 0.005;  // 5 ms

// ================= MOTOR CONTROL =================
void setMotor(int left, int right)
{
    left = constrain(left, -255, 255);
    right = constrain(right, -255, 255);

    left *= LEFT_MOTOR_SIGN;
    right *= RIGHT_MOTOR_SIGN;

    left = constrain(left, -255, 255);
    right = constrain(right, -255, 255);

    // Left motor
    if (left >= 0) {
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
    }
    else {
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
    }

    analogWrite(ENA, abs(left));

    // Right motor
    if (right >= 0) {
        digitalWrite(IN3, HIGH);
        digitalWrite(IN4, LOW);
    }
    else {
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, HIGH);
    }

    analogWrite(ENB, abs(right));
}

// ================= STOP MOTORS =================
void stopMotors()
{
    analogWrite(ENA, 0);
    analogWrite(ENB, 0);

    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);

    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
}

// ================= MPU READ =================
bool readMPU()
{
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);

    if (Wire.endTransmission(false) != 0) {
        return false;
    }

    if (Wire.requestFrom(MPU, 14, true) != 14) {
        return false;
    }

    ax = (Wire.read() << 8) | Wire.read();
    ay = (Wire.read() << 8) | Wire.read();
    az = (Wire.read() << 8) | Wire.read();

    Wire.read();
    Wire.read();  // Temperature

    gx = (Wire.read() << 8) | Wire.read();
    gy = (Wire.read() << 8) | Wire.read();
    gz = (Wire.read() << 8) | Wire.read();

    return true;
}

// ================= GYRO CALIBRATION =================
void calibrateGyro()
{
    long sum = 0;

    const int samples = 1000;
    int validSamples = 0;

    Serial.println("Keep robot completely still...");

    for (int i = 0; i < samples; i++) {

        if (readMPU()) {
            sum += gy;
            validSamples++;
        }
    }

    if (validSamples == 0) {
        Serial.println("GYRO CALIBRATION FAILED!");

        while (true) {
            stopMotors();
        }
    }

    gyroOffsetY = (float)sum / validSamples;

    Serial.println("Gyro calibration complete!");

    Serial.print("Gyro offset: ");
    Serial.println(gyroOffsetY);
}

// ================= COMPLEMENTARY FILTER =================
void updateAngle(float dt)
{
    // Accelerometer angle
    float accelAngle = atan2(
        (float)ax,
        (float)az
    ) * 180.0 / PI;

    // Gyroscope angular velocity
    float gyroRate = (gy - gyroOffsetY) / 131.0;

    // Apply sensor direction correction
    accelAngle *= ANGLE_SIGN;
    gyroRate *= ANGLE_SIGN;

    // Complementary filter
    angle = 0.98 * (angle + gyroRate * dt)
          + 0.02 * accelAngle;
}

// ================= PID CONTROLLER =================
int calculatePID(float dt)
{
    error = angle - targetAngle;

    integral += error * dt;

    integral = constrain(integral, -50.0, 50.0);

    float derivative = (error - previousError) / dt;

    pidOutput = Kp * error
              + Ki * integral
              + Kd * derivative;

    previousError = error;

    return constrain((int)pidOutput, -255, 255);
}

// ================= SETUP =================
void setup()
{
    Serial.begin(9600);

    Wire.begin();
    Wire.setClock(400000);

    pinMode(ENA, OUTPUT);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);

    pinMode(ENB, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);

    stopMotors();

    // Wake up MPU6050
    Wire.beginTransmission(MPU);
    Wire.write(0x6B);
    Wire.write(0x00);

    if (Wire.endTransmission() != 0) {

        Serial.println("MPU6050 NOT FOUND!");

        while (true) {
            stopMotors();
        }
    }

    // Accelerometer range: +/- 2g
    Wire.beginTransmission(MPU);
    Wire.write(0x1C);
    Wire.write(0x00);
    Wire.endTransmission();

    // Gyroscope range: +/- 250 degrees/sec
    Wire.beginTransmission(MPU);
    Wire.write(0x1B);
    Wire.write(0x00);
    Wire.endTransmission();

    calibrateGyro();

    if (!readMPU()) {

        Serial.println("MPU READ ERROR!");

        while (true) {
            stopMotors();
        }
    }

    // Initial angle
    angle = atan2((float)ax, (float)az)
          * 180.0 / PI;

    angle *= ANGLE_SIGN;

    previousTime = micros();

    Serial.println("Self balancing controller ready!");
}

// ================= MAIN LOOP =================
void loop()
{
    unsigned long currentTime = micros();

    float dt = (currentTime - previousTime) / 1000000.0;

    // Controller timing: 5 ms
    if (dt < LOOP_TIME) {
        return;
    }

    previousTime = currentTime;

    // Read MPU6050
    if (!readMPU()) {
        stopMotors();
        return;
    }

    // Update angle
    updateAngle(dt);

    // Safety limit
    if (abs((int)angle) > 35) {

        stopMotors();

        integral = 0;
        previousError = 0;

        return;
    }

    // Calculate PID
    int correction = calculatePID(dt);

    correction *= MOTOR_SIGN;

    // Both wheels receive same logical correction
    setMotor(correction, correction);

    // Debug output every 100 ms
    static unsigned long lastPrint = 0;

    if (millis() - lastPrint >= 100) {

        lastPrint = millis();

        Serial.print("Angle: ");
        Serial.print(angle);

        Serial.print(" | PID: ");
        Serial.println(correction);
    }
}
