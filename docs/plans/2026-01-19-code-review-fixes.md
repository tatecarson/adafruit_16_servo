# Plan: Address Code Review Findings

**Status: COMPLETE** (2026-01-19)

## Overview
Implement fixes for issues identified in the Arduino servo control code review.

## Tasks

### Task 1: Replace String with Fixed Char Buffer
**Priority:** Important
**File:** [adafruit_16_servo.ino](../../adafruit_16_servo.ino)

Replace heap-fragmenting `String` class with fixed-size char buffer for long-term stability.

**Changes:**
- Replace lines 136-137:
  ```cpp
  // Before
  String inputString = "";
  bool stringComplete = false;

  // After
  #define INPUT_BUFFER_SIZE 50
  char inputBuffer[INPUT_BUFFER_SIZE];
  uint8_t inputIndex = 0;
  bool inputComplete = false;
  ```

- Remove `inputString.reserve(50)` from `setup()` (line 153)

- Update serial input loop (lines 767-774):
  ```cpp
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      inputBuffer[inputIndex] = '\0';
      inputComplete = true;
    } else if (inputIndex < INPUT_BUFFER_SIZE - 1) {
      inputBuffer[inputIndex++] = c;
    }
  }
  ```

- Update command processing (lines 777-780):
  ```cpp
  if (inputComplete) {
    processCommand(inputBuffer);
    inputIndex = 0;
    inputComplete = false;
  }
  ```

- Update `processCommand()` signature from `String cmd` to `char* cmd` and adjust string operations

---

### Task 2: Add WAVE Command Bounds Validation
**Priority:** Suggestion
**File:** [adafruit_16_servo.ino](../../adafruit_16_servo.ino)

Add validation for servo range in WAVE command (lines 616-647).

**Changes:**
- After parsing `waveStartServo` and `waveEndServo`, add:
  ```cpp
  if (waveStartServo > waveEndServo || waveEndServo >= NUM_SERVOS) {
    Serial.println(F("Invalid servo range"));
    return;
  }
  ```

---

### Task 3: Clean Up Sweep Underflow Protection
**Priority:** Important
**File:** [adafruit_16_servo.ino](../../adafruit_16_servo.ino)

Restructure the sweep-down loop (lines 467-471) to be cleaner.

**Changes:**
```cpp
// Before
for (uint16_t p = servoConfig[servo].maxPulse; p >= servoConfig[servo].minPulse; p -= 2) {
  pwm.setPWM(servo, 0, p);
  delay(5);
  if (p < servoConfig[servo].minPulse + 2) break;
}

// After
for (uint16_t p = servoConfig[servo].maxPulse; p > servoConfig[servo].minPulse + 1; p -= 2) {
  pwm.setPWM(servo, 0, p);
  delay(5);
}
pwm.setPWM(servo, 0, servoConfig[servo].minPulse);
```

---

### Task 4: Document Blocking Nature of SWEEP
**Priority:** Suggestion
**File:** [adafruit_16_servo.ino](../../adafruit_16_servo.ino)

Add comment explaining SWEEP is intentionally blocking (calibration function).

**Changes:**
- Add comment before `sweepServo()` function (around line 448):
  ```cpp
  // Sweep a servo through its full range (BLOCKING - for calibration/debug only)
  // Note: Uses delay() which blocks all other operations for several seconds
  ```

---

### Task 5: Add PROGMEM for Sequence Data
**Priority:** Suggestion
**Files:** [sequence_setup.h](../../sequence_setup.h), [adafruit_16_servo.ino](../../adafruit_16_servo.ino)

Store sequence arrays in flash (PROGMEM) instead of RAM.

**Changes to sequence_setup.h:**
```cpp
// Change sequence declarations from:
static Keyframe sequence1[] = { ... };
static SpeedFrame speedSeq1[] = { ... };

// To:
const Keyframe sequence1[] PROGMEM = { ... };
const SpeedFrame speedSeq1[] PROGMEM = { ... };
```

**Changes to adafruit_16_servo.ino:**

Update `updateSequence()` to read from PROGMEM:
```cpp
void updateSequence() {
  if (!sequenceActive || currentSequence == nullptr) return;

  unsigned long elapsed = millis() - sequenceStartTime;
  Keyframe kf;  // Local copy for PROGMEM read

  for (uint8_t i = lastTriggeredKeyframe; i < currentSequenceLength; i++) {
    memcpy_P(&kf, &currentSequence[i], sizeof(Keyframe));

    if (kf.servo == 255) {
      // End marker handling...
    }
    // ... rest of logic using kf instead of pointer
  }
}
```

Update `updateSpeedSequence()` similarly for SpeedFrame.

---

## Files Modified
- `adafruit_16_servo.ino` - Tasks 1-4, PROGMEM reads in Task 5
- `sequence_setup.h` - Task 5 PROGMEM declarations

## Verification

### Manual Testing (Serial Monitor at 9600 baud)
1. **Basic input:** Type `HELP` - should display commands
2. **Long input:** Type 50+ characters - should truncate gracefully without crash
3. **WAVE validation:** `WAVE 10 5 50 30 90` - should print "Invalid servo range"
4. **WAVE validation:** `WAVE 0 20 50 30 90` - should print "Invalid servo range"
5. **SWEEP:** `SWEEP 4` - should complete sweep without hanging at end
6. **Rapid commands:** Send multiple commands quickly - should handle without memory issues

### Compile Check
```bash
arduino-cli compile --fqbn arduino:avr:uno adafruit_16_servo
```

### PROGMEM Verification
7. **Sequence playback:** `PLAY 1` - should execute choreography correctly
8. **Speed sequence:** `SPLAY 1` - should execute speed changes correctly
9. **Check memory savings:** Compare "Global variables" RAM usage before/after PROGMEM changes

---

## Implementation Results

**Compile output:**
```
Sketch uses 16108 bytes (49%) of program storage space. Maximum is 32256 bytes.
Global variables use 1051 bytes (51%) of dynamic memory, leaving 997 bytes for local variables. Maximum is 2048 bytes.
```

All tasks completed successfully. Manual hardware testing required for full verification.
