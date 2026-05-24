#pragma once

// Animation sequence setup (user-editable).
//
// Edit this file to define your own choreographed keyframe (positional) and
// speed (DC motor) sequences. This is intentionally separated from
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
//   and extending the selector functions at the bottom of this file.
//
// Keyframe (positional) sequence structure:
//   { servo, degrees, timeMs, durationMs }
// - servo:      0-15 (servo channel). Use 255 to end the sequence.
// - degrees:    0 to totalDegrees (mapped using that servo's configured min/max pulses).
// - timeMs:     when to start this move, relative to sequence start.
// - durationMs: how long the move should take (0 = instant).
//
// Speed (DC motor) sequence structure:
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

// ============================================================================
// PLAY sequences (positional keyframes)
// ============================================================================

// Sequence 7: "Progressive winch drop and reverse"
// One winch drops to shallow, then two to medium, then all three to deep (full
// down). Mirrors in reverse. Uses many small keyframes for smooth motion on
// loaded servos (duration alone is unreliable under load).
// Total duration: ~90 seconds. Peak (all down) at ~45s.
const Keyframe sequence7[] PROGMEM = {
  // --- Phase 1 (0-15s): Winch 0 drops 150→500 (shallow) ---
  {0, 150,  0, 1400},
  {0, 190, 1500, 1400},
  {0, 230, 3000, 1400},
  {0, 270, 4500, 1400},
  {0, 310, 6000, 1400},
  {0, 350, 7500, 1400},
  {0, 390, 9000, 1400},
  {0, 420, 10500, 1400},
  {0, 460, 12000, 1400},
  {0, 500, 13500, 1400},

  // --- Phase 2 (15-30s): W0 500→1000, W1 150→1000 ---
  {0, 550, 15000, 1400},  {1, 245, 15000, 1400},
  {0, 600, 16500, 1400},  {1, 340, 16500, 1400},
  {0, 650, 18000, 1400},  {1, 430, 18000, 1400},
  {0, 700, 19500, 1400},  {1, 525, 19500, 1400},
  {0, 750, 21000, 1400},  {1, 620, 21000, 1400},
  {0, 800, 22500, 1400},  {1, 715, 22500, 1400},
  {0, 850, 24000, 1400},  {1, 810, 24000, 1400},
  {0, 900, 25500, 1400},  {1, 905, 25500, 1400},
  {0, 950, 27000, 1400},  {1, 950, 27000, 1400},
  {0, 1000, 28500, 1400}, {1, 1000, 28500, 1400},

  // --- Phase 3 (30-45s): All three drop to 1500 (deep) — peak ---
  {0, 1050, 30000, 1400}, {1, 1050, 30000, 1400}, {2, 150, 30000, 1400},
  {0, 1100, 31500, 1400}, {1, 1100, 31500, 1400}, {2, 300, 31500, 1400},
  {0, 1150, 33000, 1400}, {1, 1150, 33000, 1400}, {2, 450, 33000, 1400},
  {0, 1200, 34500, 1400}, {1, 1200, 34500, 1400}, {2, 600, 34500, 1400},
  {0, 1250, 36000, 1400}, {1, 1250, 36000, 1400}, {2, 750, 36000, 1400},
  {0, 1300, 37500, 1400}, {1, 1300, 37500, 1400}, {2, 900, 37500, 1400},
  {0, 1350, 39000, 1400}, {1, 1350, 39000, 1400}, {2, 1050, 39000, 1400},
  {0, 1400, 40500, 1400}, {1, 1400, 40500, 1400}, {2, 1200, 40500, 1400},
  {0, 1450, 42000, 1400}, {1, 1450, 42000, 1400}, {2, 1350, 42000, 1400},
  {0, 1500, 43500, 1400}, {1, 1500, 43500, 1400}, {2, 1500, 43500, 1400},

  // --- Phase 4 (45-60s): All three rise 1500→1000 ---
  {0, 1450, 45000, 1400}, {1, 1450, 45000, 1400}, {2, 1450, 45000, 1400},
  {0, 1400, 46500, 1400}, {1, 1400, 46500, 1400}, {2, 1400, 46500, 1400},
  {0, 1350, 48000, 1400}, {1, 1350, 48000, 1400}, {2, 1350, 48000, 1400},
  {0, 1300, 49500, 1400}, {1, 1300, 49500, 1400}, {2, 1300, 49500, 1400},
  {0, 1250, 51000, 1400}, {1, 1250, 51000, 1400}, {2, 1250, 51000, 1400},
  {0, 1200, 52500, 1400}, {1, 1200, 52500, 1400}, {2, 1200, 52500, 1400},
  {0, 1150, 54000, 1400}, {1, 1150, 54000, 1400}, {2, 1150, 54000, 1400},
  {0, 1100, 55500, 1400}, {1, 1100, 55500, 1400}, {2, 1100, 55500, 1400},
  {0, 1050, 57000, 1400}, {1, 1050, 57000, 1400}, {2, 1050, 57000, 1400},
  {0, 1000, 58500, 1400}, {1, 1000, 58500, 1400}, {2, 1000, 58500, 1400},

  // --- Phase 5 (60-75s): W2 1000→150, W0+W1 1000→500 ---
  {0, 950, 60000, 1400}, {1, 950, 60000, 1400}, {2, 905, 60000, 1400},
  {0, 900, 61500, 1400}, {1, 900, 61500, 1400}, {2, 810, 61500, 1400},
  {0, 850, 63000, 1400}, {1, 850, 63000, 1400}, {2, 715, 63000, 1400},
  {0, 800, 64500, 1400}, {1, 800, 64500, 1400}, {2, 620, 64500, 1400},
  {0, 750, 66000, 1400}, {1, 750, 66000, 1400}, {2, 525, 66000, 1400},
  {0, 700, 67500, 1400}, {1, 700, 67500, 1400}, {2, 430, 67500, 1400},
  {0, 650, 69000, 1400}, {1, 650, 69000, 1400}, {2, 340, 69000, 1400},
  {0, 600, 70500, 1400}, {1, 600, 70500, 1400}, {2, 245, 70500, 1400},
  {0, 550, 72000, 1400}, {1, 550, 72000, 1400}, {2, 195, 72000, 1400},
  {0, 500, 73500, 1400}, {1, 500, 73500, 1400}, {2, 150, 73500, 1400},

  // --- Phase 6 (75-90s): W0+W1 500→150 ---
  {0, 460, 75000, 1400}, {1, 460, 75000, 1400},
  {0, 420, 76500, 1400}, {1, 420, 76500, 1400},
  {0, 390, 78000, 1400}, {1, 390, 78000, 1400},
  {0, 350, 79500, 1400}, {1, 350, 79500, 1400},
  {0, 310, 81000, 1400}, {1, 310, 81000, 1400},
  {0, 270, 82500, 1400}, {1, 270, 82500, 1400},
  {0, 230, 84000, 1400}, {1, 230, 84000, 1400},
  {0, 190, 85500, 1400}, {1, 190, 85500, 1400},
  {0, 170, 87000, 1400}, {1, 170, 87000, 1400},
  {0, 150, 88500, 1400}, {1, 150, 88500, 1400},

  {SEQUENCE_END_MARKER_SERVO, 0, 90000, 0}
};
static const uint8_t sequence7Length = sizeof(sequence7) / sizeof(sequence7[0]);

// Sequence 8: "Progressive winch drop and reverse (compact)"
// Same motion as sequence 7 but uses fewer keyframes with longer durations
// instead of many small steps. Total duration: ~90 seconds.
const Keyframe sequence8[] PROGMEM = {
  // --- Phase 1 (0-15s): Winch 0 drops 150→500 (shallow) ---
  {0, 150,     0, 15000},

  // --- Phase 2 (15-30s): W0 500→1000, W1 150→1000 ---
  {0, 500, 15000,     0},
  {1, 150, 15000,     0},
  {0, 1000, 15000, 15000},
  {1, 1000, 15000, 15000},

  // --- Phase 3 (30-45s): All three drop to 1100 (deep) — peak ---
  {2, 150, 30000,     0},
  {0, 1100, 30000, 15000},
  {1, 1100, 30000, 15000},
  {2, 1100, 30000, 15000},

  // --- Phase 4 (45-60s): All three rise 1300→1000 ---
  {0, 1000, 45000, 15000},
  {1, 1000, 45000, 15000},
  {2, 1000, 45000, 15000},

  // --- Phase 5 (60-75s): W2 1000→150, W0+W1 1000→500 ---
  {0, 500, 60000, 15000},
  {1, 500, 60000, 15000},
  {2, 150, 60000, 15000},

  // --- Phase 6 (75-90s): W0+W1 500→150 ---
  {0, 150, 75000, 15000},
  {1, 150, 75000, 15000},

  // --- Phase 7 (90-105s): All winches raise to 300 ---
  {0, 300, 90000, 15000},
  {1, 300, 90000, 15000},
  {2, 300, 90000, 15000},

  {SEQUENCE_END_MARKER_SERVO, 0, 105000, 0}
};
static const uint8_t sequence8Length = sizeof(sequence8) / sizeof(sequence8[0]);

// ============================================================================
// SPLAY sequences (DC motor speed)
// ============================================================================

// Speed sequence 1: "Demo rotation"
// Drives the DC motor via the IBT-2 driver.
const SpeedFrame speedSeq1[] PROGMEM = {
  {3, 50, 0, 500},         // Servo 3 ramp to 50% over 500ms at t=0
  {3, 0, 3000, 500},       // Ramp to stop at t=3s
  {3, -50, 4000, 500},     // Reverse direction
  {3, 0, 7000, 500},       // Stop
  {SEQUENCE_END_MARKER_SERVO, 0, 7500, 0}
};
static const uint8_t speedSeq1Length = sizeof(speedSeq1) / sizeof(speedSeq1[0]);

// Speed sequence 2: "Drift rotation accelerate/decelerate"
// Ramps servo 3 from speed 80 up to 90 over 45s, then back down to 80 over 45s.
const SpeedFrame speedSeq2[] PROGMEM = {
  {3, 80,      0,     0},      // t=0s:   Start at speed 80 (instant)
  {3, 90,      0, 45000},      // t=0s:   Ramp up to 90 over 45s
  {3, 80,  90000, 45000},      // t=90s:  Ramp down to 80 over 45s (held 90 for 45s)
  {SEQUENCE_END_MARKER_SERVO, 0, 165000, 0}  // t=165s: End (held 80 for 30s)
};
static const uint8_t speedSeq2Length = sizeof(speedSeq2) / sizeof(speedSeq2[0]);

// Speed sequence 3: "Slow drift rotation"
// Gentler version of sequence 2: tops out at 60 instead of 90 and takes 60s
// to get there, then eases back down over 60s.
const SpeedFrame speedSeq3[] PROGMEM = {
  {3, 30,      0,     0},      // t=0s:   Start at speed 30 (instant)
  {3, 60,      0, 60000},      // t=0s:   Ramp up to 60 over 60s
  {3, 30, 120000, 60000},      // t=120s: Ramp down to 30 over 60s (held 60 for 60s)
  {SEQUENCE_END_MARKER_SERVO, 0, 210000, 0}  // t=210s: End (held 30 for 30s)
};
static const uint8_t speedSeq3Length = sizeof(speedSeq3) / sizeof(speedSeq3[0]);



// ============================================================================
// RUN programs (chained PLAY + SPLAY sequences, run in parallel)
// ============================================================================
//
// Programs stitch together existing PLAY / SPLAY sequences so longer-running
// shows can be assembled without duplicating frame data.
//
// Per-track step structure:
//   { sequenceNumber, repeatCount }
// - sequenceNumber: the existing PLAY or SPLAY ID to trigger
// - repeatCount: number of consecutive times to run that sequence (minimum 1)

// Program 2: "Drift"
// Rotation accelerates/decelerates while winches progressively drop and reverse.
// Position and speed tracks run in parallel; peak rotation aligns with all
// winches fully down at ~45s.
const ProgramSequenceStep program2PositionTrack[] PROGMEM = {
  {8, 1},   // Progressive winch drop and reverse
};
const ProgramSequenceStep program2SpeedTrack[] PROGMEM = {
  {2, 1},   // Drift rotation accelerate/decelerate
};
const SequenceProgramDefinition program2 = {
  program2PositionTrack,
  sizeof(program2PositionTrack) / sizeof(program2PositionTrack[0]),
  program2SpeedTrack,
  sizeof(program2SpeedTrack) / sizeof(program2SpeedTrack[0]),
};

// Program 3: "Slow drift"
// Same winch motion as program 2, but uses the gentler speed sequence 3.
const ProgramSequenceStep program3PositionTrack[] PROGMEM = {
  {8, 1},   // Progressive winch drop and reverse
};
const ProgramSequenceStep program3SpeedTrack[] PROGMEM = {
  {3, 1},   // Slow drift rotation accelerate/decelerate
};
const SequenceProgramDefinition program3 = {
  program3PositionTrack,
  sizeof(program3PositionTrack) / sizeof(program3PositionTrack[0]),
  program3SpeedTrack,
  sizeof(program3SpeedTrack) / sizeof(program3SpeedTrack[0]),
};

// ============================================================================
// Selector functions (map serial command numbers to PROGMEM data)
// ============================================================================

inline bool selectPositionSequence(uint8_t seqNum, const Keyframe*& outSeq, uint8_t& outLen) {
  if (seqNum == 7) { outSeq = sequence7; outLen = sequence7Length; return true; }
  if (seqNum == 8) { outSeq = sequence8; outLen = sequence8Length; return true; }
  return false;
}

inline bool selectSpeedSequence(uint8_t seqNum, const SpeedFrame*& outSeq, uint8_t& outLen) {
  if (seqNum == 1) { outSeq = speedSeq1; outLen = speedSeq1Length; return true; }
  if (seqNum == 2) { outSeq = speedSeq2; outLen = speedSeq2Length; return true; }
  if (seqNum == 3) { outSeq = speedSeq3; outLen = speedSeq3Length; return true; }
  return false;
}

inline bool selectSequenceProgram(uint8_t programNum, const SequenceProgramDefinition*& outProgram) {
  if (programNum == 2) { outProgram = &program2; return true; }
  if (programNum == 3) { outProgram = &program3; return true; }
  return false;
}
