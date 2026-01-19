#pragma once

// Installation-specific servo setup.
//
// Edit this file to match the servos (and behavior) connected to each channel.
// This file is intentionally separate from `adafruit_16_servo.ino` so you can
/**
 * Apply installation-specific servo calibration values to the provided arrays.
 *
 * Updates per-servo pulse ranges, continuous-rotation flags, stop pulses, and initial position
 * pulses for servos that require installation-specific adjustments. Modifies the arrays in-place:
 * - Servo 0: minPulse=150, maxPulse=440, continuous=true, stopPulse=295, posPulse=295
 * - Servo 1: minPulse=150, maxPulse=440, continuous=true, stopPulse=295, posPulse=295
 * - Servo 4: minPulse=150, maxPulse=450
 *
 * @param servoConfig Array of ServoConfig structures to update with calibration values.
 * @param servoState Array of ServoState structures to update initial position pulses.
 */

inline void applyCustomServoSetup(ServoConfig servoConfig[], ServoState servoState[]) {
  // === CUSTOM SERVO CALIBRATIONS ===
  // Add your servo-specific calibrations here so they persist across uploads

  // Servo 0: SM-S4303R continuous rotation (stop pulse = 295)
  servoConfig[0].minPulse = 150;
  servoConfig[0].maxPulse = 440;
  servoConfig[0].continuous = true;
  servoConfig[0].stopPulse = 295;
  servoState[0].posPulse = 295;

  // Servo 1: SM-S4303R continuous rotation (stop pulse = 295)
  servoConfig[1].minPulse = 150;
  servoConfig[1].maxPulse = 440;
  servoConfig[1].continuous = true;
  servoConfig[1].stopPulse = 295;
  servoState[1].posPulse = 295;

  // Servo 4: Hitec HS-805BB+ mega quarter scale (standard positional)
  servoConfig[4].minPulse = 150;
  servoConfig[4].maxPulse = 450;

  // === END CUSTOM CALIBRATIONS ===
}
