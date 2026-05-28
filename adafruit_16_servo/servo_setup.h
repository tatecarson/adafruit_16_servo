#pragma once

// Installation-specific servo setup.
//
// Edit this file to match the servos (and behavior) connected to each channel.
// This file is intentionally separate from `adafruit_16_servo.ino` so you can

inline void applyCustomServoSetup(ServoConfig servoConfig[], ServoState servoState[]) {
  // === CUSTOM SERVO CALIBRATIONS ===
  // Add your servo-specific calibrations here so they persist across uploads

  // Servo 0: goBILDA 2000 Series 5-Turn Dual Mode (25-2 Torque) - standard
  servoConfig[0].minPulse = 110;
  servoConfig[0].maxPulse = 480;
  servoConfig[0].totalDegrees = 1800;
  servoConfig[0].allowRelease = false;
  servoConfig[0].downDegrees = 1800;  // full 5-turn travel (servo-xus)
  servoConfig[0].reverseDir = true;

  // Servo 1: goBILDA 2000 Series 5-Turn Dual Mode (25-2 Torque) - standard
  servoConfig[1].minPulse = 110;
  servoConfig[1].maxPulse = 480;
  servoConfig[1].totalDegrees = 1800;
  servoConfig[1].allowRelease = false;
  servoConfig[1].downDegrees = 1800;  // full 5-turn travel (servo-xus)
  servoConfig[1].reverseDir = true;

  // Servo 2: goBILDA 2000 Series 5-Turn Dual Mode (25-2 Torque) - standard
  servoConfig[2].minPulse = 110;
  servoConfig[2].maxPulse = 480;
  servoConfig[2].totalDegrees = 1800;
  servoConfig[2].allowRelease = false;
  servoConfig[2].downDegrees = 1800;  // full 5-turn travel (servo-xus)
  servoConfig[2].reverseDir = true;

  // Servo 3: (channel freed — rotation now handled by DC motor via IBT-2)

  // === END CUSTOM CALIBRATIONS ===
}
