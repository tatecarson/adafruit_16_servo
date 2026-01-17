# Servo Animation System Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Add smooth keyframe animations and wave pattern generators to the servo control system for art installation use.

**Architecture:** Extend existing Serial command interface with animation engine that runs in loop(). Keyframes stored in PROGMEM arrays, wave patterns computed mathematically. Non-blocking design allows animations to run while still accepting Serial commands.

**Tech Stack:** Arduino C++, Adafruit PWM Servo Driver library, PROGMEM for sequence storage

---

## Task 1: Add Easing Functions

**Files:**
- Modify: `adafruit_16_servo.ino` (add after line 64, before `setServoPulse`)

**Step 1: Add easing function**

Add this code after the `degreesToPulse` function:

```cpp
// Easing function: ease-in-out cubic for smooth organic motion
// t = progress (0.0 to 1.0), returns eased value (0.0 to 1.0)
float easeInOutCubic(float t) {
  if (t < 0.5f) {
    return 4.0f * t * t * t;
  } else {
    float f = (2.0f * t) - 2.0f;
    return 0.5f * f * f * f + 1.0f;
  }
}

// Linear interpolation with easing
uint16_t lerpEased(uint16_t start, uint16_t end, float progress) {
  float easedProgress = easeInOutCubic(progress);
  return start + (uint16_t)((float)(end - start) * easedProgress);
}
```

**Step 2: Test manually**

Upload and verify compilation succeeds. No runtime test yet - these are utility functions.

**Step 3: Commit**

```bash
git add adafruit_16_servo.ino
git commit -m "feat: add easing functions for smooth servo motion"
```

---

## Task 2: Add Non-Blocking Servo Move Function

**Files:**
- Modify: `adafruit_16_servo.ino`

**Step 1: Add animation state variables**

Add after the `servoPos` array declaration (around line 29):

```cpp
// Animation state per servo
uint16_t servoTarget[NUM_SERVOS];    // Target position
uint16_t servoStart[NUM_SERVOS];     // Starting position for current move
unsigned long servoMoveStart[NUM_SERVOS];  // When move started (millis)
uint16_t servoMoveDuration[NUM_SERVOS];    // Duration in ms (0 = instant)
bool servoMoving[NUM_SERVOS];        // Is this servo currently animating?
```

**Step 2: Initialize arrays in setup()**

Add inside the existing `for` loop in `setup()` (around line 46-50):

```cpp
    servoTarget[i] = servoPos[i];
    servoStart[i] = servoPos[i];
    servoMoveStart[i] = 0;
    servoMoveDuration[i] = 0;
    servoMoving[i] = false;
```

**Step 3: Add animated move function**

Add after `setServoDegrees` function:

```cpp
// Start an animated move to target position over duration ms
void moveServoAnimated(uint8_t servo, uint16_t targetPulse, uint16_t duration) {
  if (servo >= NUM_SERVOS) return;
  targetPulse = constrain(targetPulse, servoMin[servo], servoMax[servo]);

  servoStart[servo] = servoPos[servo];
  servoTarget[servo] = targetPulse;
  servoMoveDuration[servo] = duration;
  servoMoveStart[servo] = millis();
  servoMoving[servo] = true;
}

// Animated move using degrees
void moveServoDegrees(uint8_t servo, uint8_t degrees, uint16_t duration) {
  uint16_t pulse = degreesToPulse(servo, degrees);
  moveServoAnimated(servo, pulse, duration);
}
```

**Step 4: Add update function for animation loop**

Add after the move functions:

```cpp
// Update all servo animations - call from loop()
void updateAnimations() {
  unsigned long now = millis();

  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    if (!servoMoving[i]) continue;

    unsigned long elapsed = now - servoMoveStart[i];

    if (elapsed >= servoMoveDuration[i]) {
      // Animation complete
      servoPos[i] = servoTarget[i];
      pwm.setPWM(i, 0, servoPos[i]);
      servoMoving[i] = false;
    } else {
      // Interpolate position
      float progress = (float)elapsed / (float)servoMoveDuration[i];
      uint16_t newPos = lerpEased(servoStart[i], servoTarget[i], progress);
      if (newPos != servoPos[i]) {
        servoPos[i] = newPos;
        pwm.setPWM(i, 0, newPos);
      }
    }
  }
}
```

**Step 5: Call updateAnimations in loop()**

Add at the beginning of `loop()` function:

```cpp
void loop() {
  updateAnimations();  // Add this line

  // Read serial input...
```

**Step 6: Test manually**

Upload. Verify compilation. No visible change yet since we haven't added commands.

**Step 7: Commit**

```bash
git add adafruit_16_servo.ino
git commit -m "feat: add non-blocking animated servo movement"
```

---

## Task 3: Add MOVE Command for Animated Movement

**Files:**
- Modify: `adafruit_16_servo.ino`

**Step 1: Add MOVE command parsing**

Add in `processCommand()` after the `OFF` command block (around line 214):

```cpp
  else if (cmd.startsWith("MOVE") || cmd.startsWith("M ")) {
    // MOVE <servo> <degrees> <duration_ms>
    // or M <servo> <degrees> <duration_ms>
    int idx = cmd.startsWith("MOVE") ? 5 : 2;
    int space1 = cmd.indexOf(' ', idx);
    int space2 = cmd.indexOf(' ', space1 + 1);
    if (space1 > 0 && space2 > 0) {
      uint8_t servo = cmd.substring(idx, space1).toInt();
      uint8_t degrees = cmd.substring(space1 + 1, space2).toInt();
      uint16_t duration = cmd.substring(space2 + 1).toInt();
      moveServoDegrees(servo, degrees, duration);
      Serial.print(F("Moving servo ")); Serial.print(servo);
      Serial.print(F(" to ")); Serial.print(degrees);
      Serial.print(F("° over ")); Serial.print(duration);
      Serial.println(F("ms"));
    }
  }
```

**Step 2: Update help text**

Add to `showHelp()`:

```cpp
  Serial.println(F("MOVE <n> <deg> <ms>  Animated move (eased)"));
```

**Step 3: Update header comment**

Update the Serial Commands section in the file header:

```cpp
    MOVE <n> <deg> <ms> - Animated move with easing
```

**Step 4: Test manually**

Upload. Open Serial Monitor. Test:
```
MOVE 0 180 2000
```
Servo 0 should smoothly ease to 180° over 2 seconds.

```
MOVE 0 0 1000
```
Should smoothly return to 0°.

**Step 5: Commit**

```bash
git add adafruit_16_servo.ino
git commit -m "feat: add MOVE command for animated servo movement"
```

---

## Task 4: Add Wave Pattern Generator

**Files:**
- Modify: `adafruit_16_servo.ino`

**Step 1: Add wave state variables**

Add after the animation state variables:

```cpp
// Wave pattern state
bool waveActive = false;
uint8_t waveStartServo = 0;
uint8_t waveEndServo = 7;
uint16_t waveSpeed = 50;        // Period in ms per degree
uint8_t wavePhaseOffset = 30;   // Degrees offset between adjacent servos
uint8_t waveAmplitude = 90;     // Degrees of motion (center +/- amplitude/2)
uint8_t waveCenter = 90;        // Center position in degrees
unsigned long waveStartTime = 0;
```

**Step 2: Add wave update function**

Add after `updateAnimations()`:

```cpp
// Update wave pattern - call from loop()
void updateWave() {
  if (!waveActive) return;

  unsigned long elapsed = millis() - waveStartTime;

  for (uint8_t i = waveStartServo; i <= waveEndServo; i++) {
    // Calculate phase for this servo
    uint8_t servoIndex = i - waveStartServo;
    float phase = (float)(elapsed) / (float)(waveSpeed * 360 / 1000);
    phase += (float)(servoIndex * wavePhaseOffset) / 360.0f;

    // Sine wave: -1 to 1
    float sineVal = sin(phase * 2.0f * PI);

    // Map to servo position
    float degrees = waveCenter + (sineVal * (float)waveAmplitude / 2.0f);
    uint16_t pulse = degreesToPulse(i, (uint8_t)constrain(degrees, 0, 180));

    if (pulse != servoPos[i]) {
      servoPos[i] = pulse;
      pwm.setPWM(i, 0, pulse);
    }
  }
}
```

**Step 3: Call updateWave in loop()**

Add after `updateAnimations()` in `loop()`:

```cpp
void loop() {
  updateAnimations();
  updateWave();  // Add this line
```

**Step 4: Commit**

```bash
git add adafruit_16_servo.ino
git commit -m "feat: add wave pattern generator engine"
```

---

## Task 5: Add WAVE Command

**Files:**
- Modify: `adafruit_16_servo.ino`

**Step 1: Add WAVE command parsing**

Add in `processCommand()`:

```cpp
  else if (cmd.startsWith("WAVE")) {
    // WAVE <start> <end> <speed> <offset> <amplitude>
    // Example: WAVE 0 7 50 30 90
    int idx = 5;
    int s1 = cmd.indexOf(' ', idx);
    int s2 = cmd.indexOf(' ', s1 + 1);
    int s3 = cmd.indexOf(' ', s2 + 1);
    int s4 = cmd.indexOf(' ', s3 + 1);

    if (s1 > 0 && s2 > 0) {
      waveStartServo = cmd.substring(idx, s1).toInt();
      waveEndServo = cmd.substring(s1 + 1, s2).toInt();

      if (s3 > 0) waveSpeed = cmd.substring(s2 + 1, s3).toInt();
      else waveSpeed = 50;

      if (s4 > 0) wavePhaseOffset = cmd.substring(s3 + 1, s4).toInt();
      else wavePhaseOffset = 30;

      if (s4 > 0) waveAmplitude = cmd.substring(s4 + 1).toInt();
      else waveAmplitude = 90;

      waveStartTime = millis();
      waveActive = true;

      Serial.print(F("Wave: servos ")); Serial.print(waveStartServo);
      Serial.print(F("-")); Serial.print(waveEndServo);
      Serial.print(F(" speed=")); Serial.print(waveSpeed);
      Serial.print(F(" offset=")); Serial.print(wavePhaseOffset);
      Serial.print(F(" amp=")); Serial.println(waveAmplitude);
    }
  }
  else if (cmd.startsWith("STOP")) {
    waveActive = false;
    Serial.println(F("Wave stopped"));
  }
```

**Step 2: Update help text**

Add to `showHelp()`:

```cpp
  Serial.println(F("WAVE <s> <e> [spd] [off] [amp]  Start wave pattern"));
  Serial.println(F("STOP                 Stop wave pattern"));
```

**Step 3: Update header comment**

Add to Serial Commands in header:

```cpp
    WAVE <start> <end> [speed] [offset] [amp] - Wave pattern
    STOP               - Stop wave pattern
```

**Step 4: Test manually**

Upload. Open Serial Monitor. Test:
```
WAVE 0 7 50 30 90
```
Servos 0-7 should ripple in a sine wave pattern.

```
STOP
```
Wave should stop.

**Step 5: Commit**

```bash
git add adafruit_16_servo.ino
git commit -m "feat: add WAVE and STOP commands for wave patterns"
```

---

## Task 6: Add Keyframe Sequence Storage

**Files:**
- Modify: `adafruit_16_servo.ino`

**Step 1: Add keyframe structures and storage**

Add after wave state variables:

```cpp
// Keyframe sequence structure
struct Keyframe {
  uint8_t servo;      // Which servo (255 = all servos use same pos)
  uint8_t degrees;    // Target position
  uint16_t time;      // Time offset from sequence start (ms)
  uint16_t duration;  // Move duration (ms)
};

// Example sequence storage (modify for your choreography)
#define MAX_KEYFRAMES 32
Keyframe sequence1[MAX_KEYFRAMES] = {
  {0, 0, 0, 500},       // Servo 0 to 0° at t=0, over 500ms
  {1, 0, 100, 500},     // Servo 1 to 0° at t=100ms
  {2, 0, 200, 500},     // Servo 2 to 0° at t=200ms
  {0, 180, 1000, 500},  // Servo 0 to 180° at t=1s
  {1, 180, 1100, 500},
  {2, 180, 1200, 500},
  {0, 90, 2000, 500},   // Return to center
  {1, 90, 2100, 500},
  {2, 90, 2200, 500},
  {255, 0, 0, 0}        // End marker (servo=255)
};
uint8_t sequence1Length = 9;

// Sequence playback state
bool sequenceActive = false;
bool sequenceLoop = false;
Keyframe* currentSequence = nullptr;
uint8_t currentSequenceLength = 0;
unsigned long sequenceStartTime = 0;
uint8_t lastTriggeredKeyframe = 0;
```

**Step 2: Commit**

```bash
git add adafruit_16_servo.ino
git commit -m "feat: add keyframe sequence data structure"
```

---

## Task 7: Add Sequence Playback Engine

**Files:**
- Modify: `adafruit_16_servo.ino`

**Step 1: Add sequence update function**

Add after `updateWave()`:

```cpp
// Update sequence playback - call from loop()
void updateSequence() {
  if (!sequenceActive || currentSequence == nullptr) return;

  unsigned long elapsed = millis() - sequenceStartTime;

  // Find keyframes to trigger
  for (uint8_t i = lastTriggeredKeyframe; i < currentSequenceLength; i++) {
    Keyframe* kf = &currentSequence[i];

    // End marker check
    if (kf->servo == 255) {
      if (sequenceLoop) {
        // Restart sequence
        sequenceStartTime = millis();
        lastTriggeredKeyframe = 0;
        Serial.println(F("Sequence looping"));
      } else {
        sequenceActive = false;
        Serial.println(F("Sequence complete"));
      }
      return;
    }

    // Trigger keyframe if time reached
    if (elapsed >= kf->time && i >= lastTriggeredKeyframe) {
      moveServoDegrees(kf->servo, kf->degrees, kf->duration);
      lastTriggeredKeyframe = i + 1;
    }
  }
}
```

**Step 2: Add to loop()**

```cpp
void loop() {
  updateAnimations();
  updateWave();
  updateSequence();  // Add this line
```

**Step 3: Commit**

```bash
git add adafruit_16_servo.ino
git commit -m "feat: add keyframe sequence playback engine"
```

---

## Task 8: Add PLAY Command

**Files:**
- Modify: `adafruit_16_servo.ino`

**Step 1: Add PLAY command parsing**

Add in `processCommand()`:

```cpp
  else if (cmd.startsWith("PLAY")) {
    // PLAY <sequence_num> [LOOP]
    int space = cmd.indexOf(' ');
    if (space > 0) {
      uint8_t seqNum = cmd.substring(space + 1).toInt();
      sequenceLoop = (cmd.indexOf("LOOP") > 0);

      // Select sequence (add more sequences as needed)
      if (seqNum == 1) {
        currentSequence = sequence1;
        currentSequenceLength = sequence1Length;
      } else {
        Serial.println(F("Unknown sequence"));
        return;
      }

      sequenceStartTime = millis();
      lastTriggeredKeyframe = 0;
      sequenceActive = true;
      waveActive = false;  // Stop wave if running

      Serial.print(F("Playing sequence ")); Serial.print(seqNum);
      if (sequenceLoop) Serial.print(F(" (looping)"));
      Serial.println();
    }
  }
```

**Step 2: Update STOP command to also stop sequences**

Modify the STOP handler:

```cpp
  else if (cmd.startsWith("STOP")) {
    waveActive = false;
    sequenceActive = false;
    Serial.println(F("Stopped"));
  }
```

**Step 3: Update help text**

Add to `showHelp()`:

```cpp
  Serial.println(F("PLAY <n> [LOOP]      Play sequence n"));
```

**Step 4: Update header comment**

Add to Serial Commands in header:

```cpp
    PLAY <n> [LOOP]    - Play keyframe sequence
```

**Step 5: Test manually**

Upload. Open Serial Monitor:
```
PLAY 1
```
Should play the demo sequence once.

```
PLAY 1 LOOP
```
Should loop continuously.

```
STOP
```
Should stop playback.

**Step 6: Commit**

```bash
git add adafruit_16_servo.ino
git commit -m "feat: add PLAY command for keyframe sequences"
```

---

## Task 9: Update README

**Files:**
- Modify: `README.md`

**Step 1: Add Animation section to README**

Add after the Commands table:

```markdown
## Animation Commands

| Command | Example | Description |
|---------|---------|-------------|
| `MOVE <n> <deg> <ms>` | `MOVE 0 180 2000` | Smooth animated move with easing |
| `WAVE <s> <e> [spd] [off] [amp]` | `WAVE 0 7 50 30 90` | Start sine wave pattern |
| `PLAY <n> [LOOP]` | `PLAY 1 LOOP` | Play keyframe sequence |
| `STOP` | `STOP` | Stop wave or sequence |

### Wave Parameters

- `s`, `e`: Start and end servo numbers
- `spd`: Speed (ms per cycle, default 50)
- `off`: Phase offset between servos in degrees (default 30)
- `amp`: Amplitude in degrees (default 90)

### Creating Custom Sequences

Edit the `sequence1` array in the code to define your own keyframe animations:

```cpp
Keyframe sequence1[MAX_KEYFRAMES] = {
  {servo, degrees, time_ms, duration_ms},
  // ...
  {255, 0, 0, 0}  // End marker
};
```

- `servo`: Which servo (0-15)
- `degrees`: Target position (0-180)
- `time_ms`: When to start this move (ms from sequence start)
- `duration_ms`: How long the move takes
```

**Step 2: Commit**

```bash
git add README.md
git commit -m "docs: add animation commands to README"
```

---

## Summary

After completing all tasks, the system will support:

1. **MOVE command** - Smooth eased single-servo movements
2. **WAVE command** - Ambient sine wave patterns across servo groups
3. **PLAY command** - Keyframe sequence playback with optional looping
4. **STOP command** - Halt any running animation

The architecture is non-blocking, so Serial commands continue to work during animations.
