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

### 3. Continuous Servo Mode (Partially Complete)
- `MODE <n> CONT` - Marks a servo as continuous rotation
- `MODE <n> STD` - Switches back to standard positional servo
- `SPEED <n> <-100 to 100>` - Speed control command
- `STOP` - Also stops all continuous servos
- `CENTER <n>` - Stops a continuous servo (sends stop pulse)
- STATUS shows `[CONT]` or `[STD]` for each servo

## Current Issue: Continuous Servo Speed Control

**Problem:** The `SPEED` command parses correctly and outputs different pulse values, but the servo doesn't respond to different speeds.

**Symptoms:**
- `SPEED 1 10` → outputs `Servo 1 speed 10% -> pulse 398`
- `SPEED 1 50` → outputs `Servo 1 speed 50% -> pulse 489`
- `SPEED 1 100` → outputs `Servo 1 speed 100% -> pulse 600`
- Servo behavior doesn't change between these pulse values

**Possible Causes:**
1. The servo's actual speed range may be narrower than 150-600
2. The stop pulse (375) may not match the servo's true dead zone
3. Hardware issue with the specific servo

**Suggested Next Steps:**
1. Test with raw pulse commands to verify servo responds:
   ```
   P1 375    # Should stop (or close to it)
   P1 400    # Should spin slowly one direction
   P1 500    # Should spin faster
   P1 600    # Should spin at max speed
   P1 350    # Should spin slowly other direction
   P1 150    # Should spin at max speed other direction
   ```
2. If servo only responds in a narrow range (e.g., 360-390), adjust calibration:
   ```
   CAL 1 360 390
   MODE 1 CONT
   SPEED 1 50
   ```
3. The continuous servo's dead band (stop zone) varies by servo - some need precise calibration

## File Structure

```
adafruit_16_servo/
├── adafruit_16_servo.ino    # Main Arduino sketch
├── README.md                 # User documentation
└── docs/
    ├── plans/
    │   └── 2026-01-17-animation-system.md  # Implementation plan (all tasks done)
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

## Continuous Servo Changes (Not Yet Committed)

- Added `servoContinuous[]` and `servoStopPulse[]` arrays
- Added `speedToPulse()` function
- Added `setServoSpeed()` function
- Added `MODE` and `SPEED` command parsing
- Modified `STOP` and `CENTER` to handle continuous servos
- Updated `showStatus()` and `showHelp()`
- Fixed `S` command parsing to not match `SPEED`/`STATUS`/etc.
- Updated README with continuous servo section

## Hardware Notes

- Adafruit PCA9685 16-channel PWM driver (I2C)
- Default pulse range: 150-600 (12-bit, out of 4096)
- PWM frequency: 50Hz (standard servo)
- Requires external 5-6V power for servos
