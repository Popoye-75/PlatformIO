// Self-balancing robot | Arduino Uno + cloned MPU6050 + L298N
// Pins SAME as original code: ENA=10 IN1=8 IN2=9 | ENB=5 IN3=6 IN4=7
// Serial 115200. Commands: p20 i0 d0.8 t0 m70 (set values), c (capture balance point)
#include <Arduino.h>
#include <Wire.h>
#include <math.h>

#define MPU 0x68

#define ENA 10
#define IN1 8
#define IN2 9
#define ENB 5
#define IN3 6
#define IN4 7

// ---- flip these after testing ----
#define ANGLE_SIGN   1   // forward tilt must give POSITIVE angle
#define MOTOR_DIR    1   // flip if wheels run away from the tilt
#define LEFT_DIR     1
#define RIGHT_DIR    1

#define MOTOR_SELFTEST 1 // 1 = wheels twitch fwd/back once at power-on
#define TEST_MODE      0 // 1 = print sensor only, motors off

float Kp = 20.0, Ki = 0.0, Kd = 0.8;
float targetAngle = 0.0;
int   minPWM = 70;
#define MAX_PWM 230

#define FALL_ANGLE  35.0
#define REARM_ANGLE 15.0

int16_t ax, ay, az, gx, gy, gz;
float gyroOffsetY = 0, angle = 0, gyroRate = 0, integral = 0;
bool armed = false;
int stableCount = 0;
unsigned long lastLoop = 0;

// ---------- MPU ----------
static int16_t rd16() { uint8_t h = Wire.read(); uint8_t l = Wire.read(); return (int16_t)((h << 8) | l); }

bool readMPU()
{
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((uint8_t)MPU, (uint8_t)14, (uint8_t)true) != 14) return false;
  ax = rd16(); ay = rd16(); az = rd16();
  rd16();
  gx = rd16(); gy = rd16(); gz = rd16();
  return true;
}

static void wr(uint8_t reg, uint8_t val)
{
  Wire.beginTransmission(MPU);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

float accelAngle() { return ANGLE_SIGN * atan2(-(float)ax, (float)az) * 180.0 / PI; }

// ---------- motors ----------
static void drive(uint8_t en, uint8_t a, uint8_t b, int s)
{
  if (s > 0)      { digitalWrite(a, HIGH); digitalWrite(b, LOW); }
  else if (s < 0) { digitalWrite(a, LOW);  digitalWrite(b, HIGH); }
  else            { digitalWrite(a, LOW);  digitalWrite(b, LOW); }
  analogWrite(en, abs(s));
}

void stopMotors()
{
  analogWrite(ENA, 0); analogWrite(ENB, 0);
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
}

void setMotor(int l, int r)
{
  drive(ENA, IN1, IN2, constrain(l * LEFT_DIR, -255, 255));
  drive(ENB, IN3, IN4, constrain(r * RIGHT_DIR, -255, 255));
}

int toPWM(float u)
{
  float a = fabs(u);
  if (a < 1.0) return 0;
  if (a > 255.0) a = 255.0;
  int p = minPWM + (int)(a * (MAX_PWM - minPWM) / 255.0);
  return (u < 0) ? -p : p;
}

// ---------- calibration (never hangs) ----------
void calibrateGyro()
{
  Serial.println(F("Calibrating gyro - keep robot still..."));
  delay(500);
  for (int attempt = 0; attempt < 3; attempt++) {
    long sum = 0; int n = 0; int mn = 32767, mx = -32768;
    for (int i = 0; i < 1000; i++) {
      if (readMPU()) { sum += gy; n++; if (gy < mn) mn = gy; if (gy > mx) mx = gy; }
      delay(2);
    }
    if (n > 0) {
      gyroOffsetY = (float)sum / n;
      if ((mx - mn) < 500) break;   // good, robot was still
    }
  }
  Serial.print(F("Gyro offset: ")); Serial.println(gyroOffsetY);
}

// ---------- serial tuning ----------
void printParams()
{
  Serial.print(F("Kp=")); Serial.print(Kp);
  Serial.print(F(" Ki=")); Serial.print(Ki);
  Serial.print(F(" Kd=")); Serial.print(Kd);
  Serial.print(F(" target=")); Serial.print(targetAngle);
  Serial.print(F(" minPWM=")); Serial.println(minPWM);
}

void handleSerial()
{
  if (!Serial.available()) return;
  char c = Serial.read();
  if (c == 'c') { targetAngle = angle; printParams(); return; }
  if (c != 'p' && c != 'i' && c != 'd' && c != 't' && c != 'm') return;
  float v = Serial.parseFloat();
  if (c == 'p') Kp = v;
  if (c == 'i') { Ki = v; integral = 0; }
  if (c == 'd') Kd = v;
  if (c == 't') targetAngle = v;
  if (c == 'm') minPWM = constrain((int)v, 0, 150);
  printParams();
}

// ---------- setup ----------
void setup()
{
  Serial.begin(115200);
  Serial.setTimeout(20);

  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  stopMotors();

#if MOTOR_SELFTEST && !TEST_MODE
  // wheels twitch: forward then backward (wheels should be free / in the air)
  setMotor(150, 150); delay(300);
  stopMotors();       delay(200);
  setMotor(-150, -150); delay(300);
  stopMotors();       delay(500);
#endif

  Wire.begin();
  Wire.setClock(100000);   // slower = more noise-proof with motors running
#if defined(WIRE_HAS_TIMEOUT)
  Wire.setWireTimeout(3000, true);
#endif

  // wake MPU exactly like the original working code (no reset, internal clock)
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0x00);
  if (Wire.endTransmission() != 0) {
    while (true) {
      stopMotors();
      Serial.println(F("MPU6050 NOT FOUND (check SDA=A4 SCL=A5, try 0x69)"));
      delay(1000);
    }
  }
  delay(100);
  wr(0x1A, 0x03);  // low-pass filter
  wr(0x1B, 0x00);  // gyro +-250 dps
  wr(0x1C, 0x00);  // accel +-2g

  calibrateGyro();

  float s = 0; int n = 0;
  for (int i = 0; i < 50; i++) { if (readMPU()) { s += accelAngle(); n++; } delay(5); }
  angle = n ? s / n : 0;

  printParams();
  Serial.println(F("Ready. Hold robot upright to start."));
  lastLoop = micros();
}

// ---------- loop ----------
void loop()
{
  handleSerial();

  unsigned long now = micros();
  if (now - lastLoop < 5000UL) return;
  float dt = (now - lastLoop) / 1000000.0;
  lastLoop = now;
  if (dt > 0.05) dt = 0.05;

  if (!readMPU()) { stopMotors(); armed = false; stableCount = 0; return; }

  float accA = accelAngle();
  gyroRate = ANGLE_SIGN * ((float)gy - gyroOffsetY) / 131.0;
  angle = 0.98 * (angle + gyroRate * dt) + 0.02 * accA;

  static unsigned long lastPrint = 0;

#if TEST_MODE
  stopMotors();
  if (millis() - lastPrint >= 100) {
    lastPrint = millis();
    Serial.print(F("angle ")); Serial.print(angle);
    Serial.print(F("  accel ")); Serial.print(accA);
    Serial.print(F("  rate ")); Serial.println(gyroRate);
  }
  return;
#endif

  float err = angle - targetAngle;

  if (armed) {
    if (fabs(err) > FALL_ANGLE) {
      armed = false; stableCount = 0; integral = 0;
      stopMotors();
      Serial.println(F("FELL - stopped"));
    }
  } else {
    stopMotors();
    if (fabs(err) < REARM_ANGLE) stableCount++; else stableCount = 0;
    if (stableCount >= 20) {          // ~0.1 s upright
      armed = true; integral = 0;
      Serial.println(F("GO"));
    }
  }

  int pwm = 0;
  if (armed) {
    integral += err * dt;
    if (Ki > 0.0001) integral = constrain(integral, -80.0 / Ki, 80.0 / Ki);
    else integral = 0;

    float u = Kp * err + Ki * integral + Kd * gyroRate;
    pwm = toPWM(u * MOTOR_DIR);
    setMotor(pwm, pwm);
  }

  if (millis() - lastPrint >= 200) {
    lastPrint = millis();
    Serial.print(F("angle ")); Serial.print(angle);
    Serial.print(F("  pwm ")); Serial.print(pwm);
    Serial.println(armed ? F("  RUN") : F("  WAIT (hold upright)"));
  }
}
