#pragma once

// Installation-specific servo setup.
//
// Edit this file to match the servos (and behavior) connected to each channel.
// This file is intentionally separate from `adafruit_16_servo.ino` so you can
// reuse the project logic unchanged across different installations.

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

