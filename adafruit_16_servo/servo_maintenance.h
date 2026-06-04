#pragma once

#include "servo_runtime.h"
#include "dc_motor.h"

void sweepServo(uint8_t servo) {
  if (servo >= NUM_SERVOS) {
    Serial.println(F("Invalid servo"));
    return;
  }

  Serial.print(F("Sweeping servo ")); Serial.println(servo);
  Serial.print(F("Range: ")); Serial.print(servoConfig[servo].minPulse);
  Serial.print(F(" - ")); Serial.println(servoConfig[servo].maxPulse);

  for (uint16_t p = servoConfig[servo].minPulse; p <= servoConfig[servo].maxPulse; p += 2) {
    pwm.setPWM(servo, 0, p);
    delay(5);
  }
  delay(300);

  for (uint16_t p = servoConfig[servo].maxPulse; p > servoConfig[servo].minPulse + 1; p -= 2) {
    pwm.setPWM(servo, 0, p);
    delay(5);
  }
  pwm.setPWM(servo, 0, servoConfig[servo].minPulse);

  uint16_t center = (servoConfig[servo].minPulse + servoConfig[servo].maxPulse) / 2;
  pwm.setPWM(servo, 0, center);
  servoState[servo].posPulse = center;
  Serial.println(F("Sweep complete"));
}

void setCalibration(uint8_t servo, uint16_t minVal, uint16_t maxVal) {
  if (servo >= NUM_SERVOS) {
    Serial.println(F("Invalid servo"));
    return;
  }
  servoConfig[servo].minPulse = minVal;
  servoConfig[servo].maxPulse = maxVal;
  Serial.print(F("Servo ")); Serial.print(servo);
  Serial.print(F(" calibration: ")); Serial.print(minVal);
  Serial.print(F(" - ")); Serial.println(maxVal);
}

void showStatus() {
  Serial.println(F("\n--- Servo Status ---"));
  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    Serial.print(F("Servo ")); Serial.print(i);
    Serial.print(F(": min=")); Serial.print(servoConfig[i].minPulse);
    Serial.print(F(" max=")); Serial.print(servoConfig[i].maxPulse);
    Serial.print(F(" pos=")); Serial.print(servoState[i].posPulse);
    Serial.print(F(" range=0-")); Serial.print(servoConfig[i].totalDegrees);
    if (servoConfig[i].downDegrees != 0) {
      Serial.print(F(" travel=")); Serial.print(servoConfig[i].upDegrees);
      Serial.print(F("-")); Serial.print(servoConfig[i].downDegrees);
    }
    Serial.println();
  }
  Serial.println(F("--- DC Motor ---"));
  Serial.print(F("Speed: ")); Serial.print(motorState.currentSpeed); Serial.println(F("%"));
  Serial.println();
}
