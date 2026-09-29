#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

#define IN1 6
#define IN2 9
#define IN3 10
#define IN4 11

float angle;
float targetAngle = 0;

float Kp = 20.0;
float Ki = 0.0;
float Kd = 1.2;

float error;
float previousError = 0;
float integral = 0;

unsigned long lastTime;

void setup() {

  Serial.begin(115200);

  Serial.println("================================");
  Serial.println("SELF BALANCING DEBUG START");
  Serial.println("================================");

  Wire.begin();
  Serial.println("I2C Started");

  mpu.initialize();
  Serial.println("MPU Initialize Called");

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  Serial.println("Motor Pins Configured");

  lastTime = millis();

  Serial.println("Setup Complete");
}

void loop() {

  int16_t ax, ay, az;

  mpu.getAcceleration(&ax, &ay, &az);

  Serial.print("AX=");
  Serial.print(ax);

  Serial.print(" AY=");
  Serial.print(ay);

  Serial.print(" AZ=");
  Serial.println(az);

  angle = atan2(ay, az) * 180.0 / PI;

  unsigned long now = millis();

  float dt = (now - lastTime) / 1000.0;

  lastTime = now;

  error = targetAngle - angle;

  integral += error * dt;

  float derivative = 0;

  if (dt > 0) {
    derivative = (error - previousError) / dt;
  }

  float output =
    Kp * error +
    Ki * integral +
    Kd * derivative;

  previousError = error;

  Serial.print("ANGLE=");
  Serial.print(angle);

  Serial.print(" ERROR=");
  Serial.print(error);

  Serial.print(" OUTPUT=");
  Serial.println(output);

  if (output > 5) {

    Serial.println("FORWARD");

    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);

    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  }

  else if (output < -5) {

    Serial.println("BACKWARD");

    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);

    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  }

  else {

    Serial.println("STOP");

    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);

    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
  }

  Serial.println("--------------------");

  delay(500);
}