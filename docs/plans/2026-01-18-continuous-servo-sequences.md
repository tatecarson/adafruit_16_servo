# Continuous Servo Sequences Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Add timed speed sequences for continuous rotation servos, allowing choreographed spinning patterns with speed ramping.

**Architecture:** Mirror the existing `Keyframe` system but for speeds instead of positions. Add a `SpeedFrame` struct that stores `{servo, speed, time, rampDuration}`. The `updateSpeedSequence()` function runs in `loop()` and ramps between speeds using easing. Reuse PLAY command with automatic servo type detection.

**Tech Stack:** Arduino, Adafruit PCA9685 PWM driver, existing servo infrastructure

---

## Task 1: Add SpeedFrame Data Structure

**Files:**
- Modify: `adafruit_16_servo.ino:56-78` (after existing Keyframe struct)

**Step 1: Add SpeedFrame struct after Keyframe struct (line ~77)**

```cpp
// Speed sequence structure for continuous servos
struct SpeedFrame {
  uint8_t servo;      // Which servo (255 = end marker)
  int8_t speed;       // Target speed (-100 to 100, 0 = stop)
  uint16_t time;      // Time offset from sequence start (ms)
  uint16_t rampMs;    // Ramp duration to reach speed (0 = instant)
};

// Example speed sequence storage
#define MAX_SPEEDFRAMES 32
SpeedFrame speedSeq1[MAX_SPEEDFRAMES] = {
  {0, 50, 0, 500},       // Servo 0 ramp to 50% over 500ms at t=0
  {1, -50, 0, 500},      // Servo 1 ramp to -50% (opposite direction)
  {0, 0, 3000, 500},     // Servo 0 ramp to stop at t=3s
  {1, 0, 3000, 500},     // Servo 1 ramp to stop
  {0, -50, 4000, 500},   // Reverse directions
  {1, 50, 4000, 500},
  {0, 0, 7000, 500},     // Stop
  {1, 0, 7000, 500},
  {255, 0, 0, 0}         // End marker
};
uint8_t speedSeq1Length = 8;
```

**Step 2: Add speed sequence playback state (after line ~86)**

```cpp
// Speed sequence playback state
bool speedSeqActive = false;
bool speedSeqLoop = false;
SpeedFrame* currentSpeedSeq = nullptr;
uint8_t currentSpeedSeqLength = 0;
unsigned long speedSeqStartTime = 0;
uint8_t lastTriggeredSpeedFrame = 0;

// Per-servo speed ramping state
int8_t servoTargetSpeed[NUM_SERVOS];
int8_t servoStartSpeed[NUM_SERVOS];
unsigned long servoSpeedRampStart[NUM_SERVOS];
uint16_t servoSpeedRampDuration[NUM_SERVOS];
bool servoSpeedRamping[NUM_SERVOS];
```

**Step 3: Initialize speed state in setup() (after line ~113)**

```cpp
    servoTargetSpeed[i] = 0;
    servoStartSpeed[i] = 0;
    servoSpeedRampStart[i] = 0;
    servoSpeedRampDuration[i] = 0;
    servoSpeedRamping[i] = false;
```

**Step 4: Commit**

```bash
git add adafruit_16_servo.ino
git commit -m "feat: add SpeedFrame data structure for continuous servo sequences"
```

---

## Task 2: Add Speed Ramping Function

**Files:**
- Modify: `adafruit_16_servo.ino` (after setServoSpeed function, ~line 220)

**Step 1: Add rampServoSpeed function**

```cpp
// Ramp continuous servo to target speed over duration
void rampServoSpeed(uint8_t servo, int8_t targetSpeed, uint16_t rampMs) {
  if (servo >= NUM_SERVOS) {
    Serial.println(F("Invalid servo"));
    return;
  }
  if (!servoContinuous[servo]) {
    Serial.println(F("Not a continuous servo"));
    return;
  }

  // Get current speed from pulse
  int8_t currentSpeed = 0;
  if (servoPos[servo] != servoStopPulse[servo]) {
    // Approximate current speed from pulse position
    if (servoPos[servo] > servoStopPulse[servo]) {
      currentSpeed = map(servoPos[servo], servoStopPulse[servo], servoMax[servo], 0, 100);
    } else {
      currentSpeed = map(servoPos[servo], servoMin[servo], servoStopPulse[servo], -100, 0);
    }
  }

  if (rampMs == 0) {
    // Instant speed change
    setServoSpeed(servo, targetSpeed);
    servoSpeedRamping[servo] = false;
  } else {
    // Start ramping
    servoStartSpeed[servo] = currentSpeed;
    servoTargetSpeed[servo] = targetSpeed;
    servoSpeedRampStart[servo] = millis();
    servoSpeedRampDuration[servo] = rampMs;
    servoSpeedRamping[servo] = true;

    Serial.print(F("Servo ")); Serial.print(servo);
    Serial.print(F(" ramping ")); Serial.print(currentSpeed);
    Serial.print(F("% -> ")); Serial.print(targetSpeed);
    Serial.print(F("% over ")); Serial.print(rampMs);
    Serial.println(F("ms"));
  }
}
```

**Step 2: Commit**

```bash
git add adafruit_16_servo.ino
git commit -m "feat: add rampServoSpeed function for eased speed transitions"
```

---

## Task 3: Add Speed Ramp Update Function

**Files:**
- Modify: `adafruit_16_servo.ino` (after rampServoSpeed, before updateSequence)

**Step 1: Add updateSpeedRamps function**

```cpp
// Update all active speed ramps - call from loop()
void updateSpeedRamps() {
  unsigned long now = millis();

  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    if (!servoSpeedRamping[i]) continue;

    unsigned long elapsed = now - servoSpeedRampStart[i];

    if (elapsed >= servoSpeedRampDuration[i]) {
      // Ramp complete
      setServoSpeed(i, servoTargetSpeed[i]);
      servoSpeedRamping[i] = false;
    } else {
      // Calculate eased speed
      float t = (float)elapsed / servoSpeedRampDuration[i];
      // Cubic ease in-out (same as position animation)
      if (t < 0.5) {
        t = 4 * t * t * t;
      } else {
        t = 1 - pow(-2 * t + 2, 3) / 2;
      }

      int8_t speed = servoStartSpeed[i] + (servoTargetSpeed[i] - servoStartSpeed[i]) * t;

      // Set speed without printing (to avoid spam)
      uint16_t pulse = speedToPulse(i, speed);
      servoPos[i] = pulse;
      pwm.setPWM(i, 0, pulse);
    }
  }
}
```

**Step 2: Add call in loop() (after updateAnimations, before updateWave)**

Find in loop():
```cpp
  updateAnimations();
```

Add after:
```cpp
  updateSpeedRamps();
```

**Step 3: Commit**

```bash
git add adafruit_16_servo.ino
git commit -m "feat: add updateSpeedRamps for smooth speed transitions in loop"
```

---

## Task 4: Add Speed Sequence Playback Engine

**Files:**
- Modify: `adafruit_16_servo.ino` (after updateSequence function)

**Step 1: Add updateSpeedSequence function**

```cpp
// Update speed sequence playback - call from loop()
void updateSpeedSequence() {
  if (!speedSeqActive || currentSpeedSeq == nullptr) return;

  unsigned long elapsed = millis() - speedSeqStartTime;

  // Find speed frames to trigger
  for (uint8_t i = lastTriggeredSpeedFrame; i < currentSpeedSeqLength; i++) {
    SpeedFrame* sf = &currentSpeedSeq[i];

    // End marker check
    if (sf->servo == 255) {
      if (speedSeqLoop) {
        // Restart sequence
        speedSeqStartTime = millis();
        lastTriggeredSpeedFrame = 0;
        Serial.println(F("Speed sequence looping"));
      } else {
        speedSeqActive = false;
        // Stop all continuous servos
        for (uint8_t j = 0; j < NUM_SERVOS; j++) {
          if (servoContinuous[j]) {
            setServoSpeed(j, 0);
          }
        }
        Serial.println(F("Speed sequence complete"));
      }
      return;
    }

    // Trigger speed frame if time reached
    if (elapsed >= sf->time && i >= lastTriggeredSpeedFrame) {
      rampServoSpeed(sf->servo, sf->speed, sf->rampMs);
      lastTriggeredSpeedFrame = i + 1;
    }
  }
}
```

**Step 2: Add call in loop() (after updateSpeedRamps)**

```cpp
  updateSpeedSequence();
```

**Step 3: Commit**

```bash
git add adafruit_16_servo.ino
git commit -m "feat: add updateSpeedSequence engine for continuous servo choreography"
```

---

## Task 5: Add SPLAY Command for Speed Sequences

**Files:**
- Modify: `adafruit_16_servo.ino` (in processCommand function and showHelp)

**Step 1: Add to showHelp() output**

Find:
```cpp
  Serial.println(F("PLAY <n> [LOOP]      Play sequence n"));
```

Add after:
```cpp
  Serial.println(F("SPLAY <n> [LOOP]     Play speed sequence n (continuous servos)"));
```

**Step 2: Add SPLAY command parsing (after PLAY handling)**

Find the PLAY command block and add after it:

```cpp
  else if (cmd.startsWith("SPLAY")) {
    // SPLAY <sequence_num> [LOOP]
    int spaceIdx = cmd.indexOf(' ', 6);
    int seqNum;
    bool loop = false;

    if (spaceIdx > 0) {
      seqNum = cmd.substring(6, spaceIdx).toInt();
      if (cmd.indexOf("LOOP") > 0) loop = true;
    } else {
      seqNum = cmd.substring(6).toInt();
    }

    // Stop any running sequences
    speedSeqActive = false;
    sequenceActive = false;
    waveActive = false;

    // Select speed sequence
    if (seqNum == 1) {
      currentSpeedSeq = speedSeq1;
      currentSpeedSeqLength = speedSeq1Length;
    } else {
      Serial.println(F("Unknown speed sequence"));
      return;
    }

    speedSeqActive = true;
    speedSeqLoop = loop;
    speedSeqStartTime = millis();
    lastTriggeredSpeedFrame = 0;

    Serial.print(F("Playing speed sequence ")); Serial.print(seqNum);
    if (loop) Serial.print(F(" (looping)"));
    Serial.println();
  }
```

**Step 3: Update STOP command to also stop speed sequences**

Find in STOP handling:
```cpp
    waveActive = false;
    sequenceActive = false;
```

Add:
```cpp
    speedSeqActive = false;
    // Stop all speed ramps
    for (uint8_t i = 0; i < NUM_SERVOS; i++) {
      servoSpeedRamping[i] = false;
      if (servoContinuous[i]) {
        setServoSpeed(i, 0);
      }
    }
```

**Step 4: Commit**

```bash
git add adafruit_16_servo.ino
git commit -m "feat: add SPLAY command for speed sequence playback"
```

---

## Task 6: Update Documentation

**Files:**
- Modify: `README.md`
- Modify: `docs/PROGRESS.md`
- Modify: `AGENTS.md` (line numbers if changed significantly)

**Step 1: Add SPLAY to README command table**

Find the command table and add:
```markdown
| `SPLAY <n> [LOOP]` | `SPLAY 1 LOOP` | Play speed sequence n for continuous servos |
```

**Step 2: Add usage example to README**

Add a new section after the continuous servo example:

```markdown
### Speed Sequences

Speed sequences choreograph continuous servos with timed speed changes and ramping:

```
SPLAY 1        # Play speed sequence 1 once
SPLAY 1 LOOP   # Loop speed sequence 1
STOP           # Stop sequence
```

Speed sequences are defined in the code (similar to keyframe sequences) with `{servo, speed, time, rampMs}` tuples.
```

**Step 3: Update PROGRESS.md with new commits**

Add to Git Commits section:
```markdown
## Git Commits (Speed Sequences)

12. `feat: add SpeedFrame data structure for continuous servo sequences`
13. `feat: add rampServoSpeed function for eased speed transitions`
14. `feat: add updateSpeedRamps for smooth speed transitions in loop`
15. `feat: add updateSpeedSequence engine for continuous servo choreography`
16. `feat: add SPLAY command for speed sequence playback`
17. `docs: add speed sequence documentation`
```

**Step 4: Commit**

```bash
git add README.md docs/PROGRESS.md AGENTS.md
git commit -m "docs: add speed sequence documentation"
```

---

## Task 7: Manual Testing

**Files:**
- Modify: `docs/TESTING.md`

**Step 1: Upload sketch to Arduino**

Upload the modified `adafruit_16_servo.ino` to the Arduino.

**Step 2: Test SPLAY command**

Run:
```
SPLAY 1
```

**Expected:**
- Servos 0 and 1 ramp up to speed (opposite directions)
- At t=3s, both ramp to stop
- At t=4s, both ramp to opposite speeds
- At t=7s, both stop
- "Speed sequence complete" prints

**Step 3: Test SPLAY LOOP**

Run:
```
SPLAY 1 LOOP
```

Wait for 2 cycles, then:
```
STOP
```

**Expected:**
- Sequence repeats
- STOP halts playback and stops servos

**Step 4: Add test results to TESTING.md**

Add new test section:
```markdown
## Test 14: Speed Sequences (Continuous Servos)

**Commands:**
1. `SPLAY 1`
2. Observe sequence

**Expected:** Servos 0 and 1 execute choreographed speed changes with ramping

**Result:**
- [ ] Pass
- [ ] Fail - describe:
```

**Step 5: Commit**

```bash
git add docs/TESTING.md
git commit -m "test: add speed sequence manual test results"
```

---

## Summary

This plan adds:
1. `SpeedFrame` struct mirroring `Keyframe` but for speed control
2. `rampServoSpeed()` for smooth speed transitions with easing
3. `updateSpeedRamps()` engine running in loop()
4. `updateSpeedSequence()` playback engine
5. `SPLAY <n> [LOOP]` command
6. Documentation updates
7. Manual test procedure

The implementation follows existing patterns for keyframe sequences, making the code consistent and maintainable.
