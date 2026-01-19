#pragma once

// Animation sequence setup (user-editable).
//
// Edit this file to define your own choreographed keyframe (positional) and
// speed (continuous rotation) sequences. This is intentionally separated from
// `adafruit_16_servo.ino` so the core logic can stay unchanged.
//
// Notes:
// - This file depends on the `Keyframe` and `SpeedFrame` structs defined in
//   `adafruit_16_servo.ino`, so it must be included *after* those type
//   definitions.
// - Sequences are stored in PROGMEM (flash memory) to save RAM on Arduino.
// - Sequences are scanned in array order, and entries are triggered when
//   `elapsedMs >= time`. Keep frames in ascending `time` order to avoid
//   surprises.
// - Use `servo = 255` as an end marker (last entry in each sequence). The
//   playback engine treats this as "sequence complete" (or loop point).
// - Add more sequences by copying the pattern for `sequence1` / `speedSeq1`
//   and extending `selectPositionSequence()` / `selectSpeedSequence()`.
//
// Keyframe (positional) sequence structure:
//   { servo, degrees, timeMs, durationMs }
// - servo:      0-15 (servo channel). Use 255 to end the sequence.
// - degrees:    0-180 (mapped using that servo's configured min/max pulses).
// - timeMs:     when to start this move, relative to sequence start.
// - durationMs: how long the move should take (0 = instant).
//
// Speed (continuous) sequence structure:
//   { servo, speed, timeMs, rampMs }
// - servo:  0-15 (servo channel). Use 255 to end the sequence.
// - speed:  -100..100, where 0 = stop; negative/positive = opposite directions.
// - timeMs: when to start this speed change, relative to sequence start.
// - rampMs: how long to ramp to the target speed (0 = instant).
//
// End marker timing:
// - The end marker is also time-gated. Give it a `timeMs` >= your last frame's
//   `timeMs` so it doesn't terminate early.

#include <Arduino.h>
#include <avr/pgmspace.h>

static const uint8_t SEQUENCE_END_MARKER_SERVO = 255;

// --- Keyframe (positional) sequences ---

// Example sequence 1:
// - Multiple servos can overlap in time (moves are non-blocking).
// - `degrees` are converted to pulses with per-servo calibration, so the same
//   degrees may correspond to different absolute pulse widths per servo.
// - Stored in PROGMEM to save RAM; use memcpy_P() to read.
const Keyframe sequence1[] PROGMEM = {
  {0, 0, 0, 500},          // Servo 0 to 0° at t=0, over 500ms
  {1, 0, 100, 500},        // Servo 1 to 0° at t=100ms
  {2, 0, 200, 500},        // Servo 2 to 0° at t=200ms
  {0, 180, 1000, 500},     // Servo 0 to 180° at t=1s
  {1, 180, 1100, 500},
  {2, 180, 1200, 500},
  {0, 90, 2000, 500},      // Return to center
  {1, 90, 2100, 500},
  {2, 90, 2200, 500},
  {SEQUENCE_END_MARKER_SERVO, 0, 2700, 0}  // End marker
};

static const uint8_t sequence1Length = sizeof(sequence1) / sizeof(sequence1[0]);

// Map a sequence number from the Serial command (`PLAY <n>`) to a PROGMEM
// array and length.
//
// When adding a new sequence:
// - create `const Keyframe sequenceN[] PROGMEM = { ... }`
// - create `static const uint8_t sequenceNLength = ...`
// - add a `seqNum == N` branch below
inline bool selectPositionSequence(uint8_t seqNum, const Keyframe*& outSeq, uint8_t& outLen) {
  if (seqNum == 1) {
    outSeq = sequence1;
    outLen = sequence1Length;
    return true;
  }
  return false;
}

// --- Speed (continuous) sequences ---

// Example speed sequence 1:
// - Intended for continuous-rotation servos (MODE <n> CONT).
// - Uses per-servo min/max/stop calibration to translate `speed` into pulses.
// - Stored in PROGMEM to save RAM; use memcpy_P() to read.
const SpeedFrame speedSeq1[] PROGMEM = {
  {0, 50, 0, 500},         // Servo 0 ramp to 50% over 500ms at t=0
  {1, -50, 0, 500},        // Servo 1 ramp to -50% (opposite direction)
  {0, 0, 3000, 500},       // Servo 0 ramp to stop at t=3s
  {1, 0, 3000, 500},       // Servo 1 ramp to stop
  {0, -50, 4000, 500},     // Reverse directions
  {1, 50, 4000, 500},
  {0, 0, 7000, 500},       // Stop
  {1, 0, 7000, 500},
  {SEQUENCE_END_MARKER_SERVO, 0, 7500, 0}  // End marker (triggers after last frame completes)
};

static const uint8_t speedSeq1Length = sizeof(speedSeq1) / sizeof(speedSeq1[0]);

// Map a sequence number from the Serial command (`SPLAY <n>`) to a PROGMEM
// array and length.
//
// When adding a new speed sequence:
// - create `const SpeedFrame speedSeqN[] PROGMEM = { ... }`
// - create `static const uint8_t speedSeqNLength = ...`
// - add a `seqNum == N` branch below
inline bool selectSpeedSequence(uint8_t seqNum, const SpeedFrame*& outSeq, uint8_t& outLen) {
  if (seqNum == 1) {
    outSeq = speedSeq1;
    outLen = speedSeq1Length;
    return true;
  }
  return false;
}
