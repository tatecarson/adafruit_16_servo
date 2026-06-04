#pragma once

#include <Arduino.h>

#define MOTOR_RPWM_PIN 10
#define MOTOR_LPWM_PIN 11

struct MotorState {
  int8_t currentSpeed;
};

extern MotorState motorState;

inline uint8_t writeMotorOutputs(int8_t speed) {
  uint8_t pwmVal = map(abs(speed), 0, 100, 0, 255);
  if (speed > 0) {
    analogWrite(MOTOR_RPWM_PIN, pwmVal);
    analogWrite(MOTOR_LPWM_PIN, 0);
  } else if (speed < 0) {
    analogWrite(MOTOR_RPWM_PIN, 0);
    analogWrite(MOTOR_LPWM_PIN, pwmVal);
  } else {
    analogWrite(MOTOR_RPWM_PIN, 0);
    analogWrite(MOTOR_LPWM_PIN, 0);
  }
  return pwmVal;
}

inline void motorInit() {
  pinMode(MOTOR_RPWM_PIN, OUTPUT);
  pinMode(MOTOR_LPWM_PIN, OUTPUT);
  analogWrite(MOTOR_RPWM_PIN, 0);
  analogWrite(MOTOR_LPWM_PIN, 0);

  motorState.currentSpeed = 0;
}

inline void setMotorSpeed(int8_t speed) {
  speed = constrain(speed, -100, 100);
  motorState.currentSpeed = speed;

  uint8_t pwmVal = writeMotorOutputs(speed);

  Serial.print(F("Motor speed "));
  Serial.print(speed);
  Serial.print(F("% -> PWM "));
  Serial.println(pwmVal);
}

inline void setMotorSpeedQuiet(int8_t speed) {
  speed = constrain(speed, -100, 100);
  motorState.currentSpeed = speed;
  writeMotorOutputs(speed);
}

inline void stopMotor() {
  motorState.currentSpeed = 0;
  analogWrite(MOTOR_RPWM_PIN, 0);
  analogWrite(MOTOR_LPWM_PIN, 0);
  Serial.println(F("Motor stopped"));
}
