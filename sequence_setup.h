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
// - degrees:    0 to totalDegrees (mapped using that servo's configured min/max pulses).
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
  {0, 1800, 1000, 500},    // Servo 0 to max at t=1s
  {1, 1800, 1100, 500},
  {2, 1800, 1200, 500},
  {0, 900, 2000, 500},     // Return to center
  {1, 900, 2100, 500},
  {2, 900, 2200, 500},
  {SEQUENCE_END_MARKER_SERVO, 0, 2700, 0}  // End marker
};

static const uint8_t sequence1Length = sizeof(sequence1) / sizeof(sequence1[0]);

// Sequence 2: "Slow raise and lower"
// Raises entire ring level over 4s, holds 2s, lowers over 4s.
const Keyframe sequence2[] PROGMEM = {
  {0, 1800, 0, 4000},       // All three raise together over 4s
  {1, 1800, 0, 4000},
  {2, 1800, 0, 4000},
  {0, 0, 6000, 4000},       // Hold 2s, then lower over 4s
  {1, 0, 6000, 4000},
  {2, 0, 6000, 4000},
  {SEQUENCE_END_MARKER_SERVO, 0, 10000, 0}
};

static const uint8_t sequence2Length = sizeof(sequence2) / sizeof(sequence2[0]);

// Sequence 3: "Tilt sweep"
// Tilts ring toward each winch point in turn, sweeping around the ring.
const Keyframe sequence3[] PROGMEM = {
  // Start level at bottom
  {0, 0, 0, 1000},
  {1, 0, 0, 1000},
  {2, 0, 0, 1000},
  // Tilt toward servo 0
  {0, 1200, 1500, 2000},
  // Level out, transition tilt to servo 1
  {0, 0, 4000, 2000},
  {1, 1200, 4000, 2000},
  // Transition tilt to servo 2
  {1, 0, 6500, 2000},
  {2, 1200, 6500, 2000},
  // Return to level
  {2, 0, 9000, 2000},
  {SEQUENCE_END_MARKER_SERVO, 0, 11000, 0}
};

static const uint8_t sequence3Length = sizeof(sequence3) / sizeof(sequence3[0]);

// Sequence 4: "Gentle bob"
// All winches oscillate low-to-mid — breathing/bobbing motion. Good for looping.
const Keyframe sequence4[] PROGMEM = {
  {0, 300, 0, 1500},        // Rise to ~1/6 height
  {1, 300, 0, 1500},
  {2, 300, 0, 1500},
  {0, 900, 1500, 1500},     // Rise to mid
  {1, 900, 1500, 1500},
  {2, 900, 1500, 1500},
  {0, 300, 3000, 1500},     // Back down
  {1, 300, 3000, 1500},
  {2, 300, 3000, 1500},
  {SEQUENCE_END_MARKER_SERVO, 0, 4500, 0}
};

static const uint8_t sequence4Length = sizeof(sequence4) / sizeof(sequence4[0]);

// Sequence 5: "Wave tilt"
// Rolling wave — each winch takes turns as the high point. Best with LOOP.
const Keyframe sequence5[] PROGMEM = {
  // Servo 0 leads up
  {0, 1200, 0, 1500},
  {1, 400, 0, 1500},
  {2, 400, 0, 1500},
  // Servo 1 takes over
  {0, 400, 1500, 1500},
  {1, 1200, 1500, 1500},
  {2, 400, 1500, 1500},
  // Servo 2 takes over
  {0, 400, 3000, 1500},
  {1, 400, 3000, 1500},
  {2, 1200, 3000, 1500},
  // Reset to start position for smooth loop
  {0, 1200, 4500, 1500},
  {1, 400, 4500, 1500},
  {2, 400, 4500, 1500},
  {SEQUENCE_END_MARKER_SERVO, 0, 6000, 0}
};

static const uint8_t sequence5Length = sizeof(sequence5) / sizeof(sequence5[0]);

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
  if (seqNum == 2) {
    outSeq = sequence2;
    outLen = sequence2Length;
    return true;
  }
  if (seqNum == 3) {
    outSeq = sequence3;
    outLen = sequence3Length;
    return true;
  }
  if (seqNum == 4) {
    outSeq = sequence4;
    outLen = sequence4Length;
    return true;
  }
  if (seqNum == 5) {
    outSeq = sequence5;
    outLen = sequence5Length;
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
