#include <Arduino.h>
#include <Wire.h>

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
      analogWrite(forward, speed);
    }
  }
  void setEnable(bool _enable) { digitalWrite(enable, _enable); }
};

class Car {
public:
  int centerX = 512;
  int centerY = 512;
  int deadzone = 40;
  Motor motorL;
  Motor motorR;

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
    motorL.setSpeed(leftSpeed);
    motorR.setSpeed(rightSpeed);
  }
  void stop() {
    motorL.setEnable(false);
    motorR.setEnable(false);
  }
  void setMotorSpeeds(int leftSpeed, int rightSpeed) {
    motorL.setSpeed(leftSpeed);
    motorR.setSpeed(rightSpeed);
  }
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
  int baseSpeed = 0;
  char path[100];
  int pathLength = 0;
  int rot = 0;

#define L -1
#define R 1
#define S 0
#define ERRC 70

  int dir = S;
  int errc = 0;

  int16_t gyro_x, gyro_y, gyro_z;
#define MPU_ADDR 0x68

  // Intersection detection
  unsigned long lastIntersectionTime = 0;
  const unsigned long intersectionDelay = 300; // ms between intersections

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
    Wire.begin();
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x6B);
    Wire.write(0);
    Wire.endTransmission();
  }

  // Read all sensors (false = black/line, true = white)
  void readSensors() {
    leftOuter = digitalRead(sensorLeftOuter);
    leftInner = digitalRead(sensorLeftInner);
    center = digitalRead(sensorCenter);
    rightInner = digitalRead(sensorRightInner);
    rightOuter = digitalRead(sensorRightOuter);
  }

  void getDir() {
    if (leftOuter && rightOuter) {
      path[pathLength] = S;
      pathLength++;
      dir = S;
      return;
    } else if (!leftOuter && rightOuter) {
      path[pathLength] = S;
      pathLength++;
      dir = S;
    } else if (leftOuter && !rightOuter) {
      path[pathLength] = R;
      pathLength++;
      dir = R;
    } else if (!leftOuter && !rightOuter) {
      path[pathLength] = R;
      pathLength++;
      dir = R;
    }

    if (dir != S)
      return;

    if (leftInner && rightInner) {
      errc = 0;
    } else if (leftInner && !rightInner) {
      errc = ERRC;
    } else if (!leftInner && rightInner) {
      errc = -ERRC;
    }
  }

  // Main run loop
  void run() {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 3 * 2, true);

    gyro_x = Wire.read() << 8 | Wire.read();
    gyro_y = Wire.read() << 8 | Wire.read();
    gyro_z = Wire.read() << 8 | Wire.read();

    char tmp_str[7];
    sprintf(tmp_str, "%6d", gyro_y);
    Serial.println(tmp_str);

    readSensors();
    getDir();

    int turn_rot = 0;
    if (turn_rot >= 90)
      dir = S;

    car.drive(baseSpeed * dir, baseSpeed);
  }

  void setBaseSpeed(int speed) { baseSpeed = constrain(speed, 0, 255); }
};
