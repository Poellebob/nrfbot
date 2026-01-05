#include <Arduino.h>

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
  Motor motorL;
  Motor motorR;

  int centerX = 512;
  int centerY = 512;
  int deadzone = 40;

  Car(uint8_t r_pin_forward, uint8_t l_pin_forward, uint8_t r_pin_backward,
      uint8_t l_pin_backward, uint8_t r_pin_enable, uint8_t l_pin_enable)
      : motorL(l_pin_forward, l_pin_backward, l_pin_enable),
        motorR(r_pin_forward, r_pin_backward, r_pin_enable) {}

  void drive(int joystick_x, int joystick_y) {
    int x = joystick_x - centerX; // steering
    int y = joystick_y - centerY; // forward/back

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
};
