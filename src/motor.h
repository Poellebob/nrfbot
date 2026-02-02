#include "HardwareSerial.h"
#include <Arduino.h>
#include <Wire.h>

struct Speeds {
  int left;
  int right;
};

struct Sensors {
  int leftOuter;
  int leftInner;
  int center;
  int rightInner;
  int rightOuter;
};

class Motor {
public:
  uint8_t forward;
  uint8_t backward;
  uint8_t enable;
  Motor(uint8_t pin_forward, uint8_t pin_backward, uint8_t pin_enable) {
    forward = pin_forward;
    backward = pin_backward;
    enable = pin_enable;
    pinMode(forward, OUTPUT);
    pinMode(backward, OUTPUT);
    pinMode(enable, OUTPUT);
  }
  void setSpeed(int speed) {
    speed = constrain(speed, -255, 255);
    digitalWrite(enable, true);
    if (speed == 0) {
      analogWrite(forward, 0);
      analogWrite(backward, 0);
    } else if (speed < 0) {
      analogWrite(forward, 0);
      analogWrite(backward, abs(speed));
    } else {
      analogWrite(backward, 0);
      analogWrite(forward, abs(speed));
    }
  }
  void setEnable(bool _enable) { digitalWrite(enable, _enable); }
};

class Car {
private:
  int centerX = 512;
  int centerY = 512;
  int deadzone = 40;
  Motor motorL;
  Motor motorR;
  int speedL = 0;
  int speedR = 0;

public:
  Car(uint8_t r_pin_forward, uint8_t l_pin_forward, uint8_t r_pin_backward,
      uint8_t l_pin_backward, uint8_t r_pin_enable, uint8_t l_pin_enable)
      : motorL(l_pin_forward, l_pin_backward, l_pin_enable),
        motorR(r_pin_forward, r_pin_backward, r_pin_enable) {}

  void drive(int steer = 0, int speed = 0) {
    int x = steer - centerX; // steering
    int y = speed - centerY; // forward/back
    if (abs(x) < deadzone)
      x = 0;
    if (abs(y) < deadzone)
      y = 0;
    int forward = map(y, -512, 512, -255, 255);
    int turn = map(x, -512, 512, -255, 255);
    int leftSpeed = forward - turn;
    int rightSpeed = forward + turn;
    leftSpeed = constrain(leftSpeed, -255, 255);
    rightSpeed = constrain(rightSpeed, -255, 255);
    motorL.setSpeed(speedL);
    motorR.setSpeed(speedR);
  }
  void stop() {
    motorL.setEnable(false);
    motorR.setEnable(false);
  }
  void setMotorSpeeds(int leftSpeed, int rightSpeed) {
    speedL = leftSpeed;
    speedR = rightSpeed;
    motorL.setSpeed(leftSpeed);
    motorR.setSpeed(rightSpeed);
  }
  Speeds getSpeeds() const { return {speedL, speedR}; }
};

class LineFollower {
private:
  Car &car;

  uint8_t sensorLeftOuter;
  uint8_t sensorLeftInner;
  uint8_t sensorCenter;
  uint8_t sensorRightInner;
  uint8_t sensorRightOuter;

  bool leftOuter;
  bool leftInner;
  bool center;
  bool rightInner;
  bool rightOuter;

#define SMOOTH_N 6

  bool loBuf[SMOOTH_N];
  bool liBuf[SMOOTH_N];
  bool cBuf[SMOOTH_N];
  bool riBuf[SMOOTH_N];
  bool roBuf[SMOOTH_N];

  int bufIndex = 0;

  int baseSpeed = 150;
  int rSpeed = baseSpeed;
  int lSpeed = baseSpeed;

  char path[100];
  int pathLength = 0;

#define L -1
#define R 1
#define S 0
#define TURN180 2
#define ERRC 30

  int dir = S;
  int errc = 0;
  int turnSpeed = 120;
  int serrc = -1;
#define TDZ 8
#define GEM 3

#define MPU_ADDR 0x68
  int16_t gyro_x, gyro_y, gyro_z;
  float gyro_bias_x = 0.0;
  float gyro_bias_y = 0.0;
  float gyro_bias_z = 0.0;

  float heading = 0.0;
  float target_heading = 0.0;
  bool turning = false;
  bool turning_wait = false;

  unsigned long last_us = 0;
  const float GYRO_SCALE = 131.0;

  unsigned long lastIntersectionTime = 0;

public:
  LineFollower(Car &_car, uint8_t s0, uint8_t s1, uint8_t s2, uint8_t s3,
               uint8_t s4, int speed = 150)
      : car(_car), sensorLeftOuter(s0), sensorLeftInner(s1), sensorCenter(s2),
        sensorRightInner(s3), sensorRightOuter(s4) {

    pinMode(sensorLeftOuter, INPUT);
    pinMode(sensorLeftInner, INPUT);
    pinMode(sensorCenter, INPUT);
    pinMode(sensorRightInner, INPUT);
    pinMode(sensorRightOuter, INPUT);

    setBaseSpeed(speed);
  }

  void init() {
    Serial.println("INIT START");

    Wire.begin();
    Serial.println("WIRE OK");

    Wire.setClock(400000);
    Serial.println("CLOCK OK");

    readSensors();
    Serial.println("SENSORS OK");

    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x6B);
    Wire.write(0);
    Wire.endTransmission();
    Serial.println("MPU WAKE OK");

    calibrateGyro(); // <-- likely freeze here
    Serial.println("CALIBRATION DONE");

    last_us = micros();
    Serial.println("INIT DONE");

    loBuf[0] = leftOuter;
    liBuf[0] = leftInner;
    cBuf[0] = center;
    riBuf[0] = rightInner;
    roBuf[0] = rightOuter;

    bufIndex++;
  }

  String prettyDB() {
    char buf[200];
    char hbuf[16];
    char thbuf[16];

    dtostrf(heading, 6, 2, hbuf); // width 6, 2 decimals
    dtostrf(target_heading, 6, 2, thbuf);

    snprintf(buf, sizeof(buf),
             " sensor: %d, %d, %d, %d, %d | h: %s | th: %s | err: %d | l: %d | "
             "r: %d",
             leftOuter, leftInner, center, rightInner, rightOuter, hbuf, thbuf,
             errc, lSpeed, rSpeed);

    return String(buf);
  }

  void run() {
    updateGyro();
    Serial.println(prettyDB());

    if (!turning) {
      readSensors();
      getDir();
    }

    if (turning) {
      handleTurn();
    } else {
      followLine();
    }
  }

  void readSensors() {
    bool lo = digitalRead(sensorLeftOuter);
    bool li = digitalRead(sensorLeftInner);
    bool c = digitalRead(sensorCenter);
    bool ri = digitalRead(sensorRightInner);
    bool ro = digitalRead(sensorRightOuter);

    for (int i = SMOOTH_N - 1; i > 0; i--) {
      loBuf[i] = loBuf[i - 1];
      liBuf[i] = liBuf[i - 1];
      cBuf[i] = cBuf[i - 1];
      riBuf[i] = riBuf[i - 1];
      roBuf[i] = roBuf[i - 1];
    }

    loBuf[0] = lo;
    liBuf[0] = li;
    cBuf[0] = c;
    riBuf[0] = ri;
    roBuf[0] = ro;

    if (bufIndex < SMOOTH_N - 1) {
      bufIndex++;
    }

    leftOuter = majorityVote(loBuf);
    leftInner = majorityVote(liBuf);
    rightInner = majorityVote(riBuf);
    rightOuter = majorityVote(roBuf);

    if (turning_wait) {
      if (!majorityVote(cBuf)) {
        turning_wait = false;
      }
      center = true;
    }

    if (!turning_wait) {
      center = majorityVote(cBuf);
    }
  }

  bool majorityVote(bool buf[]) {
    int count = 0;
    for (int i = 0; i < bufIndex; i++) {
      if (buf[i])
        count++;
    }
    return count > bufIndex / 2;
  }

  void getDir() {
    if (!turning_wait && leftInner && leftOuter && center && rightOuter &&
        rightInner) {
      startTurn180();
      return;
    }

    if (!leftOuter && !rightOuter && !center && !turning_wait) {
      startTurn(R);
      return;
    } else if (leftOuter && !rightOuter && !center & !turning_wait) {
      startTurn(R);
      return;
    }

    if (rightInner) {
      errc = -ERRC * serrc;
    }
    if (leftInner) {
      errc = ERRC * serrc;
    }
    if (!rightOuter && center) {
      errc = ERRC * serrc;
    }
    if (!rightOuter && center) {
      errc = -ERRC * serrc;
    }

    gyro_error_correction();
  }

  void gyro_error_correction() {
    float error = target_heading - heading;

    if (error > 180.0) {
      error -= 360.0;
    } else if (error < -180.0) {
      error += 360.0;
    }

    errc -= error * GEM;
  }

  void startTurn(int direction) {
    delay(0);
    if (turning)
      return;

    dir = direction;
    turning = true;
    lastIntersectionTime = millis();

    if (direction == R) {
      target_heading = fmod(target_heading + 90.0, 360.0);
      path[pathLength++] = R;
    } else {
      target_heading = fmod(target_heading - 90.0 + 360.0, 360.0);
      path[pathLength++] = L;
    }
  }

  void startTurn180() {
    if (turning)
      return;

    dir = TURN180;
    turning = true;
    lastIntersectionTime = millis();
    target_heading = fmod(heading + 180.0, 360.0);
    path[pathLength++] = TURN180;
  }

  void updateGyro() {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 6, true);

    gyro_x = Wire.read() << 8 | Wire.read();
    gyro_y = Wire.read() << 8 | Wire.read();
    gyro_z = Wire.read() << 8 | Wire.read();

    unsigned long now = micros();
    float dt = (now - last_us) * 1e-6;
    last_us = now;
    if (dt <= 0)
      return;

    float gyro_dps = (gyro_z - gyro_bias_z) / GYRO_SCALE;

    if (fabs(gyro_dps) < 0.8)
      gyro_dps = 0;

    heading += gyro_dps * dt;

    if (heading >= 360.0)
      heading -= 360.0;
    if (heading < 0.0)
      heading += 360.0;
  }

  void handleTurn() {
    float error = fabs(heading - target_heading);
    if (error > 180.0)
      error = 360.0 - error;

    if (dir == R) {
      lSpeed = 0;
      rSpeed = turnSpeed;
    } else if (dir == L) {
      lSpeed = turnSpeed;
      rSpeed = 0;

    } else if (dir == TURN180) {
      lSpeed = -turnSpeed;
      rSpeed = turnSpeed;
    } /*else {
      lSpeed = 0;
      rSpeed = 0;
    }*/

    if (error < TDZ) {
      gyro_error_correction();
      turning = false;
      lSpeed = baseSpeed - errc;
      rSpeed = baseSpeed + errc;
      turning_wait = true;
      car.setMotorSpeeds(lSpeed, rSpeed);
    }

    car.setMotorSpeeds(lSpeed, rSpeed);
  }

  void followLine() {
    lSpeed = baseSpeed + errc;
    rSpeed = baseSpeed - errc;

    lSpeed = constrain(lSpeed, 0, 255);
    rSpeed = constrain(rSpeed, 0, 255);

    car.setMotorSpeeds(lSpeed, rSpeed);
  }

  void calibrateGyro() {
    const int samples = 1500;
    long sum_x = 0;
    long sum_y = 0;
    long sum_z = 0;

    Serial.println("Calibrating gyro... keep still");

    for (int i = 0; i < samples; i++) {
      Wire.beginTransmission(MPU_ADDR);
      Wire.write(0x43);
      Wire.endTransmission(false);
      Wire.requestFrom(MPU_ADDR, 6, true);

      int16_t gx = Wire.read() << 8 | Wire.read();
      int16_t gy = Wire.read() << 8 | Wire.read();
      int16_t gz = Wire.read() << 8 | Wire.read();

      sum_x += gx;
      sum_y += gy;
      sum_z += gz;

      delay(2);
    }

    gyro_bias_x = (float)sum_x / samples;
    gyro_bias_y = (float)sum_y / samples;
    gyro_bias_z = (float)sum_z / samples;

    Serial.print("Bias X: ");
    Serial.println(gyro_bias_x);
    Serial.print("Bias Y: ");
    Serial.println(gyro_bias_y);
    Serial.print("Bias Z: ");
    Serial.println(gyro_bias_z);
  }

  void setBaseSpeed(int speed) { baseSpeed = constrain(speed, 0, 255); }

  int16_t getGyroX() const { return gyro_x; }
  int16_t getGyroY() const { return gyro_y; }
  int16_t getGyroZ() const { return gyro_z; }

  float getHeading() const { return heading; }
  float getTargetHeading() const { return target_heading; }

  bool isTurning() const { return turning; }

  Sensors getSensors() const {
    return {leftOuter, leftInner, center, rightInner, rightOuter};
  }
};
