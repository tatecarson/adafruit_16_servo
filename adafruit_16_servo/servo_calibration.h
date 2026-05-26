#pragma once
#include <stdint.h>
#include <EEPROM.h>
#include "servo_runtime.h"

// Per-servo calibration persisted to the previously-unused 16-byte tail
// of the dataflash (the sequencer slot region ends at byte 8175 leaving
// bytes 8176-8191 unused — see storage.h). Stores three channels' worth
// of {minUs, maxUs, offsetDeg} plus a single flags byte with magic +
// per-channel "calibrated" bits.
//
// Why persist:
//   Real hobby servos vary +/- 200us from the nominal 1000-2000us pulse
//   range and horns are often installed off-center. Without per-unit
//   calibration "S0 90" produces different physical angles on different
//   sculptures. The compile-time defaults in applyCustomServoSetup() are
//   board-wide hand-tuned guesses; this module lets each board's three
//   servos be trimmed individually and have the trim survive reboot.
//
// Why micro-seconds:
//   Wire format is microseconds (matches RC servo datasheets, intuitive
//   in the // 08 calibration UI). Firmware converts to PCA9685 12-bit
//   ticks at apply time so the hot S<n> path doesn't pay for division.
//
// Storage layout at byte 8176:
//   byte 0:      flags  (bit 7 = magic, bits 0..2 = per-channel calibrated)
//   bytes 1-5:   channel 0 record: uint16 minUs, uint16 maxUs, int8 offsetDeg
//   bytes 6-10:  channel 1 record (same shape)
//   bytes 11-15: channel 2 record (same shape)

#define CALIB_OFF                8176
#define CALIB_OFF_FLAGS          8176
#define CALIB_OFF_CHANNELS       8177
#define CALIB_NUM_CHANNELS       3
#define CALIB_BYTES_PER_CH       5
#define CALIB_MAGIC              0x80  // top bit of flags byte = initialized

#define CALIB_DEFAULT_MIN_US     1000
#define CALIB_DEFAULT_MAX_US     2000
#define CALIB_DEFAULT_OFFSET_DEG 0

// Acceptable pulse-width envelope. RC servos universally accept pulses in
// the 1000-2000us range; the wider 400-2600us bound here is what we'll
// accept as user input (a few servos and continuous-rotation units use
// wider ranges) without flagging the value as corrupt. Anything outside
// this band is treated as garbage from a botched EEPROM read.
#define CALIB_BOUND_MIN_US       400
#define CALIB_BOUND_MAX_US       2600
#define CALIB_BOUND_OFFSET_DEG   127  // int8 max; symmetric -127..127

struct CalibrationRecord {
  uint16_t minUs;
  uint16_t maxUs;
  int8_t   offsetDeg;
  bool     calibrated;
};

inline CalibrationRecord _calibrationDefaults() {
  return { CALIB_DEFAULT_MIN_US, CALIB_DEFAULT_MAX_US, CALIB_DEFAULT_OFFSET_DEG, false };
}

// Convert microseconds to PCA9685 12-bit PWM ticks. Integer math via a 32-bit
// intermediate so SERVO_FREQ * 4096 * us doesn't overflow uint16 on the way.
inline uint16_t calibUsToTicks(uint16_t us) {
  return (uint16_t)((uint32_t)us * SERVO_FREQ * 4096UL / 1000000UL);
}

inline uint8_t _calibFlags() { return EEPROM.read(CALIB_OFF_FLAGS); }

inline bool _calibInitialized() { return (_calibFlags() & CALIB_MAGIC) != 0; }

inline CalibrationRecord calibrationGet(uint8_t ch) {
  if (ch >= CALIB_NUM_CHANNELS) return _calibrationDefaults();
  if (!_calibInitialized()) return _calibrationDefaults();
  uint8_t flags = _calibFlags();
  bool calibrated = (flags & (1 << ch)) != 0;
  // CAL_RESET clears the flag bit but leaves the byte values in EEPROM
  // (cheap undo if the operator changes their mind). For the public-facing
  // read we surface DEFAULTS when the flag is clear so the boot log and
  // GET /calibration don't show stale "uncalibrated" values that look
  // exactly like the previously-saved calibration.
  if (!calibrated) {
    CalibrationRecord d = _calibrationDefaults();
    return d;
  }
  CalibrationRecord r;
  int off = CALIB_OFF_CHANNELS + ch * CALIB_BYTES_PER_CH;
  r.minUs = ((uint16_t)EEPROM.read(off + 0) << 8) | EEPROM.read(off + 1);
  r.maxUs = ((uint16_t)EEPROM.read(off + 2) << 8) | EEPROM.read(off + 3);
  r.offsetDeg = (int8_t)EEPROM.read(off + 4);
  r.calibrated = true;
  // Defensive: surface defaults if the saved bytes look corrupted. Catches
  // erased-flash 0xFFFF, inverted ranges, and pulses outside the universally
  // safe servo envelope. The flag bit may be set but the bytes wrong if
  // the dataflash was wiped or only partially written; treat that as
  // "uncalibrated" rather than letting bogus values drive the S<n> path.
  if (r.minUs < CALIB_BOUND_MIN_US || r.minUs > CALIB_BOUND_MAX_US ||
      r.maxUs < CALIB_BOUND_MIN_US || r.maxUs > CALIB_BOUND_MAX_US ||
      r.minUs >= r.maxUs) {
    return _calibrationDefaults();
  }
  return r;
}

inline void calibrationSet(uint8_t ch, uint16_t minUs, uint16_t maxUs, int8_t offsetDeg) {
  if (ch >= CALIB_NUM_CHANNELS) return;
  if (minUs >= maxUs) return;  // reject inverted ranges
  uint8_t flags = _calibFlags();
  if (!(flags & CALIB_MAGIC)) flags = CALIB_MAGIC;
  flags |= (1 << ch);
  EEPROM.update(CALIB_OFF_FLAGS, flags);
  int off = CALIB_OFF_CHANNELS + ch * CALIB_BYTES_PER_CH;
  EEPROM.update(off + 0, (uint8_t)(minUs >> 8));
  EEPROM.update(off + 1, (uint8_t)(minUs & 0xFF));
  EEPROM.update(off + 2, (uint8_t)(maxUs >> 8));
  EEPROM.update(off + 3, (uint8_t)(maxUs & 0xFF));
  EEPROM.update(off + 4, (uint8_t)offsetDeg);
}

inline void calibrationReset(uint8_t ch) {
  if (ch >= CALIB_NUM_CHANNELS) return;
  uint8_t flags = _calibFlags();
  flags &= ~(1 << ch);
  if (!(flags & CALIB_MAGIC)) flags = CALIB_MAGIC;
  EEPROM.update(CALIB_OFF_FLAGS, flags);
  // Leave the channel record bytes as-is; the cleared flag bit means
  // calibrationGet returns calibrated=false and callers ignore the values.
}

// First-time init: stamp the magic byte and write default {1000us, 2000us, 0deg}
// into each channel's record so reads return sensible values even before any
// channel is explicitly calibrated.
inline void calibrationInit() {
  if (_calibInitialized()) return;
  EEPROM.update(CALIB_OFF_FLAGS, CALIB_MAGIC);
  for (uint8_t ch = 0; ch < CALIB_NUM_CHANNELS; ch++) {
    int off = CALIB_OFF_CHANNELS + ch * CALIB_BYTES_PER_CH;
    EEPROM.update(off + 0, (uint8_t)(CALIB_DEFAULT_MIN_US >> 8));
    EEPROM.update(off + 1, (uint8_t)(CALIB_DEFAULT_MIN_US & 0xFF));
    EEPROM.update(off + 2, (uint8_t)(CALIB_DEFAULT_MAX_US >> 8));
    EEPROM.update(off + 3, (uint8_t)(CALIB_DEFAULT_MAX_US & 0xFF));
    EEPROM.update(off + 4, (uint8_t)CALIB_DEFAULT_OFFSET_DEG);
  }
}

// Print per-channel calibrated-vs-defaults status to Serial so operators
// can confirm whether each servo has been trimmed without leaving the
// Arduino IDE / Serial Monitor (answers "how do I know it's calibrated"
// from servo-2n9 / servo-ehd).
inline void calibrationPrintBootStatus() {
  Serial.println(F("--- Servo Calibration ---"));
  for (uint8_t ch = 0; ch < CALIB_NUM_CHANNELS; ch++) {
    CalibrationRecord r = calibrationGet(ch);
    Serial.print(F("  S")); Serial.print(ch); Serial.print(F(": "));
    if (r.calibrated) {
      Serial.print(F("calibrated  minUs=")); Serial.print(r.minUs);
      Serial.print(F(" maxUs=")); Serial.print(r.maxUs);
      Serial.print(F(" offsetDeg=")); Serial.println(r.offsetDeg);
    } else {
      Serial.print(F("defaults    minUs=")); Serial.print(r.minUs);
      Serial.print(F(" maxUs=")); Serial.print(r.maxUs);
      Serial.println(F(" offsetDeg=0 (uncalibrated)"));
    }
  }
}

// Patch the in-memory servoConfig[] for any channel the operator has
// explicitly calibrated. Channels still on defaults keep the hand-tuned
// values from applyCustomServoSetup(), so existing sculptures don't
// regress to nominal 1000-2000us when this feature ships.
//
// offsetDeg is also written into ServoConfig (new field) so the hot
// degreesToPulse path can apply it without re-reading EEPROM per frame.
inline void calibrationApplyToServoConfig(ServoConfig configs[NUM_SERVOS]) {
  for (uint8_t ch = 0; ch < CALIB_NUM_CHANNELS; ch++) {
    CalibrationRecord r = calibrationGet(ch);
    if (!r.calibrated) continue;
    configs[ch].minPulse = calibUsToTicks(r.minUs);
    configs[ch].maxPulse = calibUsToTicks(r.maxUs);
    configs[ch].offsetDeg = (int16_t)r.offsetDeg;
  }
}
