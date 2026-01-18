# Servo Config/State Refactor Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use `superpowers:executing-plans` to implement this plan task-by-task.

**Goal:** Separate per-servo setup from control logic by replacing parallel arrays with `ServoConfig`/`ServoState` structs and moving installation-specific setup into a dedicated file.

**Architecture:** Keep the existing Serial command surface and animation engines, but store all per-servo calibration/mode data in `ServoConfig[NUM_SERVOS]` and all runtime/animation data in `ServoState[NUM_SERVOS]`. `setup()` initializes defaults, then calls a single “custom setup” function that is easy to edit for different servos/installations.

**Tech Stack:** Arduino C++ (`.ino`), Adafruit PCA9685 (`Adafruit_PWMServoDriver`).

---

### Task 1: Add `ServoConfig`/`ServoState` model

**Files:**
- Modify: `adafruit_16_servo.ino`

**Step 1: Replace parallel arrays with structs**

- Add:
  - `struct ServoConfig { uint16_t minPulse; uint16_t maxPulse; bool continuous; uint16_t stopPulse; };`
  - `struct ServoState { uint16_t posPulse; uint16_t targetPulse; uint16_t startPulse; unsigned long moveStartMs; uint16_t moveDurationMs; bool moving; int8_t targetSpeed; int8_t startSpeed; unsigned long speedRampStartMs; uint16_t speedRampDurationMs; bool speedRamping; };`
- Replace globals:
  - `servoMin/servoMax/servoPos/servoContinuous/servoStopPulse`
  - `servoTarget/servoStart/servoMoveStart/servoMoveDuration/servoMoving`
  - `servoTargetSpeed/servoStartSpeed/servoSpeedRampStart/servoSpeedRampDuration/servoSpeedRamping`

**Step 2: Add a default initializer**

- Add a function like `initServoDefaults()` that:
  - sets config defaults (`DEFAULT_MIN/DEFAULT_MAX`, `continuous=false`, `stopPulse=center`)
  - sets state defaults (`posPulse=center`, targets to current, ramps/moves inactive)

---

### Task 2: Convert all logic to use structs

**Files:**
- Modify: `adafruit_16_servo.ino`

**Step 1: Update conversion + control helpers**

- Update `degreesToPulse()` and `speedToPulse()` to use `servoConfig[servo].*`.
- Update `setServoPulse()`, `setServoDegrees()`, `setServoSpeed()`, `rampServoSpeed()`, `moveServoAnimated()` to read/write `servoState[servo].*`.

**Step 2: Update engines**

- Update `updateAnimations()`, `updateSpeedRamps()`, `updateWave()`, `updateSequence()`, `updateSpeedSequence()` to use `servoState[]` and `servoConfig[]`.

**Step 3: Update commands + reporting**

- Update `sweepServo()`, `setCalibration()`, `showStatus()`, and the relevant parts of `processCommand()` (`CENTER`, `STOP`, `MODE`) to use structs.

**Step 4: Verification**

Run: `rg -n "servoMin\\[|servoMax\\[|servoPos\\[|servoContinuous\\[|servoStopPulse\\[" adafruit_16_servo.ino`

Expected: no matches.

---

### Task 3: Extract installation-specific servo setup

**Files:**
- Create: `servo_setup.h`
- Modify: `adafruit_16_servo.ino`

**Step 1: Add a `applyCustomServoSetup()` hook**

- In `adafruit_16_servo.ino`:
  - declare `void applyCustomServoSetup();`
  - call it from `setup()` after `initServoDefaults()`

**Step 2: Move “CUSTOM SERVO CALIBRATIONS” into `servo_setup.h`**

- Implement `applyCustomServoSetup()` in `servo_setup.h` (included once by the sketch).
- Keep the same customizations for servos 0, 1, and 4, but set fields through `servoConfig[]`/`servoState[]`.

---

### Task 4: Update docs for the new setup point

**Files:**
- Modify: `README.md`
- Modify: `AGENTS.md`

**Step 1: README**

- Add a short note telling users to edit `servo_setup.h` to configure servo ranges/modes.

**Step 2: AGENTS**

- Update “File Structure” to mention the new `servo_setup.h`.
- Update “Code Organization” line ranges if they’re now inaccurate.

---

### Task 5: Manual verification checklist (Arduino IDE)

**Step 1: Compile + upload**

- Open `adafruit_16_servo.ino` in Arduino IDE and upload to your board.

**Step 2: Serial smoke test**

- `STATUS` prints all servos and modes.
- `S0 90` moves positional servos, and does not break continuous servos.
- `MODE 0 CONT` then `SPEED 0 50` spins, `SPEED 0 0` stops.
- `MOVE 4 180 500` animates as before.

