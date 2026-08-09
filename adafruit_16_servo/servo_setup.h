#pragma once

// Installation-specific servo setup.
//
// Edit this file to match the servos (and behavior) connected to each channel.
// This file is intentionally separate from `adafruit_16_servo.ino` so you can
//
// One firmware is flashed to every board, but the boards no longer drive the
// same machine, so this branches on the board's own id. setup() calls
// storageInit() before it calls us, so storageBoardId() is already valid here.
//
// The servos are identical everywhere: goBILDA 2000 Series 5-Turn Dual Mode.
// "Dual mode" switches between 5-turn positional and CONTINUOUS ROTATION —
// not between two positional ranges. Continuous rotation cannot hold an angle,
// so every board keeps the 5-turn positional mode and totalDegrees stays 1800.
// What differs per machine is how much of that travel the mechanism uses.

#define SERVO_TURNS_5_DEGREES 1800   // the servo's full positional travel

// Board 3 — dowel curtain. The winch drum wants every one of its five turns:
// that is what pays out enough cable to drop the ring through its full range.
#define CURTAIN_WINCH_DOWN_DEGREES 1800

// Board 1 — centre wands. The servo horn IS the wand pivot, so a degree of
// servo is a degree of wand, and the wand only sweeps far enough to reach the
// tubes without carrying its tip past them into the outer ring. That angle is
// geometry, not preference: with a 166 mm wand on a 45 mm pivot against a
// 180 mm tube orbit it comes out at ~36°.
//
// Be aware what this costs in resolution. The pulse range spans 370 PCA9685
// ticks across all 1800°, so 36° is only ~7 ticks — the wand has roughly seven
// distinct positions, not a hundred. That is fine for "clear" versus "in" and
// coarse for anything expressive. The fix if it matters is mechanical: gear
// the wand down so it uses more of the servo's rotation.
#define WAND_DOWN_DEGREES 36

inline void applyCustomServoSetup(ServoConfig servoConfig[], ServoState servoState[]) {
  // === CUSTOM SERVO CALIBRATIONS ===
  // Add your servo-specific calibrations here so they persist across uploads

  // storageBoardId() returns 0 when the id has never been set or the EEPROM
  // has been cleared, so the branch has to say which way an UNKNOWN board
  // falls — and the two mistakes are not equally bad. A winch handed the
  // wand's 36° barely moves: obviously wrong, harmless, easy to spot. A wand
  // handed the winch's 1800° drives five turns into a lever with 36° of room.
  //
  // So the winch config is opt-in by positive identification, and anything
  // else — board 1, board 2, an unprogrammed board fresh off the bench —
  // gets the short travel. Failure lands on the side that cannot break
  // anything.
  const uint8_t boardId = storageBoardId();
  const bool isWinch = (boardId == 3);

  // How far 100 %down drives the mechanism, and which way round it runs.
  // The winches are wound so a higher value RAISES the ring, hence the
  // reversal; the wands are direct-driven and want 100 %down to mean down.
  const uint16_t downDegrees = isWinch ? CURTAIN_WINCH_DOWN_DEGREES : WAND_DOWN_DEGREES;
  const bool reverseDir = isWinch;

  for (uint8_t ch = 0; ch < 3; ch++) {
    // Servos 0-2: goBILDA 2000 Series 5-Turn Dual Mode (25-2 Torque)
    servoConfig[ch].minPulse = 110;
    servoConfig[ch].maxPulse = 480;
    servoConfig[ch].totalDegrees = SERVO_TURNS_5_DEGREES;
    servoConfig[ch].allowRelease = false;
    servoConfig[ch].downDegrees = downDegrees;
    servoConfig[ch].reverseDir = reverseDir;
  }

  // Servo 3: (channel freed — rotation now handled by DC motor via IBT-2)

  // Board 2 — field. No servos are connected at all; the config above is
  // harmless and simply never actuates anything.

  // Say out loud which profile this board took. One binary goes to every
  // board and configures itself from its stored id, so the only way to be
  // sure the right machine got the right travel is to have it tell you.
  Serial.print(F("servos: "));
  Serial.print(isWinch ? F("winch profile") : F("wand profile"));
  Serial.print(F(" · 100%down = "));
  Serial.print(downDegrees);
  Serial.print(F("deg · reverseDir="));
  Serial.print(reverseDir ? F("yes") : F("no"));
  if (boardId == 0) Serial.print(F("  [!] boardId unset — using the safe short travel"));
  Serial.println();

  // === END CUSTOM CALIBRATIONS ===
}
