
# Project Progress

## Overview

Servo calibration and control system for the Adafruit PCA9685 16-channel PWM driver, with animation capabilities for art installations.

## Completed Features

### 1. Core Servo Control
- Serial command interface at 9600 baud
- Per-servo calibration (min/max pulse values)
- Commands: `S<n> <deg>`, `P<n> <pulse>`, `CAL`, `SWEEP`, `CENTER`, `OFF`, `STATUS`, `HELP`

### 2. Animation System (Tasks 1-9 Complete)
- **Easing functions** - Smooth ease-in-out cubic motion
- **Non-blocking animation engine** - Runs in `loop()`, doesn't block Serial
- **MOVE command** - Animated movement: `MOVE 0 180 2000` (servo 0 to 180° over 2 seconds)
- **WAVE command** - Sine wave patterns across servo groups: `WAVE 0 7 50 30 90`
- **Keyframe sequences** - Pre-programmed choreography with `PLAY 1 LOOP`
- **STOP command** - Halts all animations

### 3. Continuous Servo Mode (Complete)
- `MODE <n> CONT` - Marks a servo as continuous rotation
- `MODE <n> STD` - Switches back to standard positional servo
- `SPEED <n> <-100 to 100>` - Speed control command
- `STOP` - Also stops all continuous servos
- `CENTER <n>` - Stops a continuous servo (sends stop pulse)
- STATUS shows `[CONT]` or `[STD]` for each servo

### 4. Speed Sequences for Continuous Servos (Complete)
- `SpeedFrame` struct for timed speed changes with ramping
- `rampServoSpeed()` for eased speed transitions
- `updateSpeedRamps()` engine in loop() for smooth interpolation
- `updateSpeedSequence()` playback engine for choreography
- `SPLAY <n> [LOOP]` command for speed sequence playback
- Cubic ease-in-out for organic speed ramping

## File Structure

```
adafruit_16_servo/
├── adafruit_16_servo.ino    # Main Arduino sketch
├── README.md                 # User documentation
├── AGENTS.md                 # AI assistant context
└── docs/
    ├── plans/
    │   ├── 2026-01-17-animation-system.md  # Animation implementation plan (done)
    │   └── 2026-01-18-continuous-servo-sequences.md  # Speed sequences plan (done)
    ├── TESTING.md            # Manual test results
    └── PROGRESS.md           # This file
```

## Git Commits (Animation System)

1. `feat: add easing functions for smooth servo motion`
2. `feat: add non-blocking animated servo movement`
3. `feat: add MOVE command for animated servo movement`
4. `feat: add wave pattern generator engine`
5. `feat: add WAVE and STOP commands for wave patterns`
6. `feat: add keyframe sequence data structure`
7. `feat: add keyframe sequence playback engine`
8. `feat: add PLAY command for keyframe sequences`
9. `docs: add animation commands to README`

## Git Commits (Continuous Servo)

10. `feat: add continuous servo mode and speed control commands`
11. `fix: adjust speed mapping for continuous servos and update command feedback`

## Git Commits (Speed Sequences)

12. `feat: add SpeedFrame data structure for continuous servo sequences`
13. `feat: add rampServoSpeed function for eased speed transitions`
14. `feat: add updateSpeedRamps for smooth speed transitions in loop`
15. `feat: add updateSpeedSequence engine for continuous servo choreography`
16. `feat: add SPLAY command for speed sequence playback`
17. `docs: add speed sequence documentation`

## Future Features

- [ ] **EEPROM calibration storage** - Save/load servo calibrations (type, min, max, stop pulse) to persist across power cycles. Commands: `SAVE`, `LOAD`, `CLEAR`

## Hardware Notes

- Adafruit PCA9685 16-channel PWM driver (I2C)
- Default pulse range: 150-600 (12-bit, out of 4096)
- PWM frequency: 50Hz (standard servo)
- Requires external 5-6V power for servos
