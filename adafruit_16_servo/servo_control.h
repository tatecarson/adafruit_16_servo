#pragma once

#include "servo_runtime.h"
#include "dc_motor.h"

/**
 * @brief Convert a servo angle to the calibrated PWM pulse for a specific servo.
 *
 * Applies the calibrated offsetDeg trim before mapping into the pulse range.
 * A positive offsetDeg shifts the commanded angle UP, so if the physical
 * horn reads 3 degrees below the commanded value, set offsetDeg=+3 in the
 * // 08 calibration UI and "S0 90" will drive to whatever pulse corresponds
 * to 93 internal, producing 90 physical.
 */
uint16_t degreesToPulse(uint8_t servo, uint16_t degrees) {
  uint16_t maxDeg = servoConfig[servo].totalDegrees;
  int32_t adjusted = (int32_t)degrees + (int32_t)servoConfig[servo].offsetDeg;
  if (adjusted < 0) adjusted = 0;
  if (adjusted > (int32_t)maxDeg) adjusted = (int32_t)maxDeg;
  return map((uint16_t)adjusted, 0, maxDeg, servoConfig[servo].minPulse, servoConfig[servo].maxPulse);
}

uint16_t sequenceDegreesToPulse(uint8_t servo, uint16_t degrees) {
  uint16_t maxDeg = servoConfig[servo].downDegrees;
  if (maxDeg == 0) maxDeg = servoConfig[servo].totalDegrees;
  degrees = constrain(degrees, 0, maxDeg);
  return map(degrees, 0, maxDeg, servoConfig[servo].minPulse, servoConfig[servo].maxPulse);
}

// Convert a percentage of the configured servo travel to degrees.
uint16_t percentToDegrees(uint8_t servo, uint8_t percent) {
  percent = constrain(percent, 0, 100);
  uint16_t lo = servoConfig[servo].upDegrees;
  uint16_t hi = servoConfig[servo].downDegrees;
  if (hi == 0) hi = servoConfig[servo].totalDegrees;
  if (servoConfig[servo].reverseDir) {
    return hi - (uint16_t)(((uint32_t)(hi - lo) * percent) / 100);
  }
  return lo + (uint16_t)(((uint32_t)(hi - lo) * percent) / 100);
}

// Convert an absolute "up" percentage to degrees assuming 0% = fully down, 100% = fully up.
uint16_t upPercentToDegrees(uint8_t servo, uint8_t percentUp) {
  percentUp = constrain(percentUp, 0, 100);
  return percentToDegrees(servo, 100 - percentUp);
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
  float delta = (float)((int32_t)end - (int32_t)start);
  return (uint16_t)((float)start + (delta * easedProgress));
}

void setServoPulse(uint8_t servo, uint16_t pulse) {
  if (servo >= NUM_SERVOS) {
    Serial.println(F("Invalid servo"));
    return;
  }
  cancelMotionPlayback();
  cancelSequencePlayback();
  servoState[servo].stopped = false;
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
  (void)servo;
  setMotorSpeed(speed);
}

void rampServoSpeed(uint8_t servo, int8_t targetSpeed, uint32_t rampMs) {
  (void)servo;
  rampMotorSpeed(targetSpeed, rampMs);
}

void moveServoAnimated(uint8_t servo, uint16_t targetPulse, uint32_t duration) {
  if (servo >= NUM_SERVOS) return;
  cancelMotionPlayback();
  cancelSequencePlayback();
  servoState[servo].stopped = false;
  targetPulse = constrain(targetPulse, servoConfig[servo].minPulse, servoConfig[servo].maxPulse);

  servoState[servo].startPulse = servoState[servo].posPulse;
  servoState[servo].targetPulse = targetPulse;
  servoState[servo].moveDurationMs = duration;
  servoState[servo].moveStartMs = millis();
  servoState[servo].moving = true;
}

void moveSequenceDegrees(uint8_t servo, uint16_t degrees, uint32_t duration) {
  uint16_t pulse = sequenceDegreesToPulse(servo, degrees);
  moveServoAnimated(servo, pulse, duration);
}

void stopActivePatterns() {
  sequenceActive = false;
  speedSeqActive = false;
  programActive = false;
  cancelMotionPlayback();
  cancelSequencePlayback();
}

void clearServoStop(uint8_t servo) {
  if (servo >= NUM_SERVOS) return;
  servoState[servo].stopped = false;
}

void stopServoNow(uint8_t servo) {
  if (servo >= NUM_SERVOS) {
    Serial.println(F("Invalid servo"));
    return;
  }

  servoState[servo].moving = false;
  servoState[servo].stopped = true;

  pwm.setPWM(servo, 0, servoState[servo].posPulse);
  Serial.print(F("Servo ")); Serial.print(servo);
  Serial.println(F(" held"));
}

void setServoPercent(uint8_t servo, uint8_t percent) {
  if (servo >= NUM_SERVOS) {
    Serial.println(F("Invalid servo"));
    return;
  }
  stopActivePatterns();
  clearServoStop(servo);
  servoState[servo].moving = false;
  setServoDegrees(servo, percentToDegrees(servo, percent));
}

void setServoPercentUp(uint8_t servo, uint8_t percentUp) {
  if (servo >= NUM_SERVOS) {
    Serial.println(F("Invalid servo"));
    return;
  }
  stopActivePatterns();
  clearServoStop(servo);
  servoState[servo].moving = false;
  setServoDegrees(servo, upPercentToDegrees(servo, percentUp));
}

void setTestPulse(uint16_t pulse) {
  for (uint8_t i = 0; i < 3; i++) {
    setServoPulse(i, pulse);
  }
}

