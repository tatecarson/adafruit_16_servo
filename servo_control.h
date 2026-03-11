#pragma once

#include "servo_runtime.h"

/**
 * @brief Convert a servo angle to the calibrated PWM pulse for a specific servo.
 */
uint16_t degreesToPulse(uint8_t servo, uint16_t degrees) {
  uint16_t maxDeg = servoConfig[servo].totalDegrees;
  degrees = constrain(degrees, 0, maxDeg);
  return map(degrees, 0, maxDeg, servoConfig[servo].minPulse, servoConfig[servo].maxPulse);
}

// Convert a percentage of the configured servo travel to degrees.
uint16_t percentToDegrees(uint8_t servo, uint8_t percent) {
  percent = constrain(percent, 0, 100);
  return ((uint32_t)servoConfig[servo].totalDegrees * percent) / 100;
}

// Convert an absolute "up" percentage to degrees assuming 0% = fully down, 100% = fully up.
uint16_t upPercentToDegrees(uint8_t servo, uint8_t percentUp) {
  percentUp = constrain(percentUp, 0, 100);
  return percentToDegrees(servo, 100 - percentUp);
}

/**
 * Convert a signed speed (-100..100) into the calibrated PWM pulse for a servo.
 */
uint16_t speedToPulse(uint8_t servo, int8_t speed) {
  speed = constrain(speed, -100, 100);
  uint16_t stopPulse = servoConfig[servo].stopPulse;
  if (speed == 0) {
    return stopPulse;
  } else if (speed > 0) {
    return map(speed, 1, 100, stopPulse + 1, servoConfig[servo].maxPulse);
  } else {
    return map(speed, -100, -1, servoConfig[servo].minPulse, stopPulse - 1);
  }
}

// Easing function: ease-in-out cubic for smooth organic motion
float easeInOutCubic(float t) {
  if (t < 0.5f) {
    return 4.0f * t * t * t;
  } else {
    float f = (2.0f * t) - 2.0f;
    return 0.5f * f * f * f + 1.0f;
  }
}

// Linear interpolation with easing
uint16_t lerpEased(uint16_t start, uint16_t end, float progress) {
  float easedProgress = easeInOutCubic(progress);
  return start + (uint16_t)((float)(end - start) * easedProgress);
}

void setServoPulse(uint8_t servo, uint16_t pulse) {
  if (servo >= NUM_SERVOS) {
    Serial.println(F("Invalid servo"));
    return;
  }
  pulse = constrain(pulse, servoConfig[servo].minPulse, servoConfig[servo].maxPulse);
  servoState[servo].posPulse = pulse;
  pwm.setPWM(servo, 0, pulse);
  Serial.print(F("Servo ")); Serial.print(servo);
  Serial.print(F(" -> pulse ")); Serial.println(pulse);
}

void setServoDegrees(uint8_t servo, uint16_t degrees) {
  uint16_t pulse = degreesToPulse(servo, degrees);
  setServoPulse(servo, pulse);
}

void setServoSpeed(uint8_t servo, int8_t speed) {
  if (servo >= NUM_SERVOS) {
    Serial.println(F("Invalid servo"));
    return;
  }
  if (!servoConfig[servo].continuous) {
    Serial.println(F("Not a continuous servo (use MODE command)"));
    return;
  }
  uint16_t pulse = speedToPulse(servo, speed);
  servoState[servo].posPulse = pulse;
  pwm.setPWM(servo, 0, pulse);
  Serial.print(F("Servo ")); Serial.print(servo);
  Serial.print(F(" speed ")); Serial.print(speed);
  Serial.print(F("% -> pulse ")); Serial.println(pulse);
}

void rampServoSpeed(uint8_t servo, int8_t targetSpeed, uint32_t rampMs) {
  if (servo >= NUM_SERVOS) {
    Serial.println(F("Invalid servo"));
    return;
  }
  if (!servoConfig[servo].continuous) {
    Serial.println(F("Not a continuous servo"));
    return;
  }

  int8_t currentSpeed = 0;
  if (servoState[servo].posPulse != servoConfig[servo].stopPulse) {
    if (servoState[servo].posPulse > servoConfig[servo].stopPulse) {
      currentSpeed = map(servoState[servo].posPulse, servoConfig[servo].stopPulse, servoConfig[servo].maxPulse, 0, 100);
    } else {
      currentSpeed = map(servoState[servo].posPulse, servoConfig[servo].minPulse, servoConfig[servo].stopPulse, -100, 0);
    }
  }

  if (rampMs == 0) {
    setServoSpeed(servo, targetSpeed);
    servoState[servo].speedRamping = false;
  } else {
    servoState[servo].startSpeed = currentSpeed;
    servoState[servo].targetSpeed = targetSpeed;
    servoState[servo].speedRampStartMs = millis();
    servoState[servo].speedRampDurationMs = rampMs;
    servoState[servo].speedRamping = true;

    Serial.print(F("Servo ")); Serial.print(servo);
    Serial.print(F(" ramping ")); Serial.print(currentSpeed);
    Serial.print(F("% -> ")); Serial.print(targetSpeed);
    Serial.print(F("% over ")); Serial.print(rampMs);
    Serial.println(F("ms"));
  }
}

void moveServoAnimated(uint8_t servo, uint16_t targetPulse, uint32_t duration) {
  if (servo >= NUM_SERVOS) return;
  targetPulse = constrain(targetPulse, servoConfig[servo].minPulse, servoConfig[servo].maxPulse);

  servoState[servo].startPulse = servoState[servo].posPulse;
  servoState[servo].targetPulse = targetPulse;
  servoState[servo].moveDurationMs = duration;
  servoState[servo].moveStartMs = millis();
  servoState[servo].moving = true;
}

void moveServoDegrees(uint8_t servo, uint16_t degrees, uint32_t duration) {
  uint16_t pulse = degreesToPulse(servo, degrees);
  moveServoAnimated(servo, pulse, duration);
}

void stopActivePatterns() {
  waveActive = false;
  sequenceActive = false;
  speedSeqActive = false;
}

void setServoPercent(uint8_t servo, uint8_t percent) {
  if (servo >= NUM_SERVOS) {
    Serial.println(F("Invalid servo"));
    return;
  }
  stopActivePatterns();
  servoState[servo].moving = false;
  setServoDegrees(servo, percentToDegrees(servo, percent));
}

void moveServoPercent(uint8_t servo, uint8_t percent, uint32_t duration) {
  if (servo >= NUM_SERVOS) {
    Serial.println(F("Invalid servo"));
    return;
  }
  stopActivePatterns();
  moveServoDegrees(servo, percentToDegrees(servo, percent), duration);
}

void setServoPercentUp(uint8_t servo, uint8_t percentUp) {
  if (servo >= NUM_SERVOS) {
    Serial.println(F("Invalid servo"));
    return;
  }
  stopActivePatterns();
  servoState[servo].moving = false;
  setServoDegrees(servo, upPercentToDegrees(servo, percentUp));
}

void moveServoPercentUp(uint8_t servo, uint8_t percentUp, uint32_t duration) {
  if (servo >= NUM_SERVOS) {
    Serial.println(F("Invalid servo"));
    return;
  }
  stopActivePatterns();
  moveServoDegrees(servo, upPercentToDegrees(servo, percentUp), duration);
}

void setAllProtectedWinchesPercent(bool percentUp, uint8_t percent) {
  stopActivePatterns();
  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    if (servoConfig[i].continuous || servoConfig[i].allowRelease) continue;
    servoState[i].moving = false;
    if (percentUp) {
      setServoDegrees(i, upPercentToDegrees(i, percent));
    } else {
      setServoDegrees(i, percentToDegrees(i, percent));
    }
  }
}

void moveAllProtectedWinchesPercent(bool percentUp, uint8_t percent, uint32_t duration) {
  stopActivePatterns();
  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    if (servoConfig[i].continuous || servoConfig[i].allowRelease) continue;
    if (percentUp) {
      moveServoDegrees(i, upPercentToDegrees(i, percent), duration);
    } else {
      moveServoDegrees(i, percentToDegrees(i, percent), duration);
    }
  }
}

void servoOff(uint8_t servo) {
  if (servo >= NUM_SERVOS) return;
  if (!servoConfig[servo].allowRelease) {
    Serial.print(F("Servo ")); Serial.print(servo);
    Serial.println(F(" is release-protected; use RELEASE <n> to intentionally drop it"));
    return;
  }
  pwm.setPWM(servo, 0, 0);
  Serial.print(F("Servo ")); Serial.print(servo); Serial.println(F(" off"));
}

void releaseServo(uint8_t servo) {
  if (servo >= NUM_SERVOS) return;
  pwm.setPWM(servo, 0, 0);
  Serial.print(F("Servo ")); Serial.print(servo); Serial.println(F(" released"));
}
