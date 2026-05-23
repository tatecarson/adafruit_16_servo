#pragma once

#include <Arduino.h>

#define MOTOR_RPWM_PIN 10
#define MOTOR_LPWM_PIN 11

struct MotorState {
  int8_t currentSpeed;
  int8_t targetSpeed;
  int8_t startSpeed;
  unsigned long rampStartMs;
  uint32_t rampDurationMs;
  bool ramping;
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
  motorState.targetSpeed = 0;
  motorState.startSpeed = 0;
  motorState.rampStartMs = 0;
  motorState.rampDurationMs = 0;
  motorState.ramping = false;
}

inline void setMotorSpeed(int8_t speed) {
  speed = constrain(speed, -100, 100);
  motorState.currentSpeed = speed;
  motorState.ramping = false;

  uint8_t pwmVal = writeMotorOutputs(speed);

  Serial.print(F("Motor speed "));
  Serial.print(speed);
  Serial.print(F("% -> PWM "));
  Serial.println(pwmVal);
}

inline void setMotorSpeedQuiet(int8_t speed) {
  speed = constrain(speed, -100, 100);
  motorState.currentSpeed = speed;
  motorState.ramping = false;
  writeMotorOutputs(speed);
}

inline void rampMotorSpeed(int8_t targetSpeed, uint32_t rampMs) {
  targetSpeed = constrain(targetSpeed, -100, 100);

  if (rampMs == 0) {
    setMotorSpeed(targetSpeed);
    return;
  }

  motorState.startSpeed = motorState.currentSpeed;
  motorState.targetSpeed = targetSpeed;
  motorState.rampStartMs = millis();
  motorState.rampDurationMs = rampMs;
  motorState.ramping = true;

  Serial.print(F("Motor ramping "));
  Serial.print(motorState.currentSpeed);
  Serial.print(F("% -> "));
  Serial.print(targetSpeed);
  Serial.print(F("% over "));
  Serial.print(rampMs);
  Serial.println(F("ms"));
}

inline void stopMotor() {
  motorState.ramping = false;
  motorState.currentSpeed = 0;
  analogWrite(MOTOR_RPWM_PIN, 0);
  analogWrite(MOTOR_LPWM_PIN, 0);
  Serial.println(F("Motor stopped"));
}

inline void updateMotorRamp() {
  if (!motorState.ramping) return;

  unsigned long elapsed = millis() - motorState.rampStartMs;

  if (elapsed >= motorState.rampDurationMs) {
    setMotorSpeed(motorState.targetSpeed);
    motorState.ramping = false;
    return;
  }

  float t = (float)elapsed / motorState.rampDurationMs;
  if (t < 0.5f) {
    t = 4.0f * t * t * t;
  } else {
    t = 1.0f - pow(-2.0f * t + 2.0f, 3.0f) / 2.0f;
  }

  float interp = (float)motorState.startSpeed +
    ((float)motorState.targetSpeed - (float)motorState.startSpeed) * t;
  int8_t speed = (int8_t)constrain((int)interp, -100, 100);

  if (speed != motorState.currentSpeed) {
    motorState.currentSpeed = speed;
    writeMotorOutputs(speed);
  }
}
