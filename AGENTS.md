# Project Overview for AI Agents

## What This Project Is

A servo calibration and control system for the **Adafruit PCA9685 16-channel PWM/Servo driver**, designed for art installations. It provides an interactive Serial command interface to control up to 16 servos with support for animations, wave patterns, and keyframe sequences.

## Hardware

- **Controller**: Arduino (Uno, Nano, Mega, etc.)
- **Driver Board**: [Adafruit 16-Channel 12-bit PWM/Servo Shield](https://www.adafruit.com/product/1411) (PCA9685) - stackable Arduino shield
  - Also compatible with: [Adafruit 16-Channel PWM/Servo Driver](http://www.adafruit.com/products/815) (breakout board version)
- **Library**: [Adafruit-PWM-Servo-Driver-Library](https://github.com/adafruit/Adafruit-PWM-Servo-Driver-Library) - Arduino library for I2C control of the PCA9685
- **Servos**: Standard analog servos (positional) or continuous rotation servos

## Key Technical Details

| Parameter | Value | Notes |
|-----------|-------|-------|
| Communication | I2C (SDA/SCL) | Pins A4/A5 on most Arduinos |
| PWM Resolution | 12-bit (0-4095) | Pulse values typically 150-600 |
| PWM Frequency | 50Hz | Standard servo frequency |
| Serial Baud | 9600 | For command interface |
| Servo Channels | 16 (0-15) | |

## Architecture

```
┌─────────────────┐     Serial      ┌─────────────────┐
│  Serial Monitor │◄───────────────►│     Arduino     │
│   (Commands)    │                 │                 │
└─────────────────┘                 │  loop():        │
                                    │  - updateAnims  │
                                    │  - updateWave   │
                                    │  - updateSeq    │
                                    │  - readSerial   │
                                    └────────┬────────┘
                                             │ I2C
                                    ┌────────▼────────┐
                                    │    PCA9685      │
                                    │  PWM Driver     │
                                    └────────┬────────┘
                                             │ PWM signals
                              ┌──────────────┼──────────────┐
                              ▼              ▼              ▼
                          Servo 0       Servo 1  ...   Servo 15
```

## File Structure

```
adafruit_16_servo/
├── adafruit_16_servo.ino    # Main Arduino sketch (all code)
├── README.md                 # User documentation
├── AGENTS.md                 # This file - AI context
└── docs/
    ├── PROGRESS.md           # Development progress/issues
    └── plans/
        └── 2026-01-17-animation-system.md  # Completed implementation plan
```

## Code Organization (adafruit_16_servo.ino)

The sketch is organized in this order:

1. **Includes & Globals** (lines 1-95): Library includes, state arrays, keyframe structures
2. **Utility Functions** (lines 96-161): `degreesToPulse()`, `speedToPulse()`, easing functions
3. **Servo Control** (lines 163-216): `setServoPulse()`, `setServoDegrees()`, `setServoSpeed()`, `moveServoAnimated()`
4. **Animation Engines** (lines 218-300): `updateAnimations()`, `updateWave()`, `updateSequence()`
5. **Commands** (lines 302-387): `sweepServo()`, `servoOff()`, `setCalibration()`, `showStatus()`, `showHelp()`
6. **Command Parser** (lines 389-586): `processCommand()` - parses all Serial commands
7. **Main Loop** (lines 588-609): Calls update functions, reads Serial

## State Arrays

Per-servo state is tracked in parallel arrays:

| Array | Type | Purpose |
|-------|------|---------|
| `servoMin[]` | uint16_t | Calibrated minimum pulse |
| `servoMax[]` | uint16_t | Calibrated maximum pulse |
| `servoPos[]` | uint16_t | Current pulse position |
| `servoContinuous[]` | bool | True if continuous rotation servo |
| `servoStopPulse[]` | uint16_t | Stop/center pulse for continuous servos |
| `servoTarget[]` | uint16_t | Animation target position |
| `servoStart[]` | uint16_t | Animation start position |
| `servoMoveStart[]` | unsigned long | Animation start time (millis) |
| `servoMoveDuration[]` | uint16_t | Animation duration (ms) |
| `servoMoving[]` | bool | Is servo currently animating? |

## Command Reference

### Basic Control
- `S<n> <deg>` - Move servo to degrees (0-180)
- `P<n> <pulse>` - Move servo to raw pulse value
- `CAL <n> <min> <max>` - Set pulse calibration
- `SWEEP <n>` - Test sweep full range
- `CENTER <n>` - Move to 90° (or stop continuous servo)
- `OFF <n>` - Stop sending PWM

### Animation
- `MOVE <n> <deg> <ms>` - Smooth eased movement
- `WAVE <s> <e> [spd] [off] [amp]` - Sine wave pattern across servos
- `PLAY <n> [LOOP]` - Play keyframe sequence
- `STOP` - Stop all animations

### Continuous Servo Mode
- `MODE <n> STD|CONT` - Switch between standard/continuous
- `SPEED <n> <-100:100>` - Set speed (0=stop, ±100=full speed)

### Info
- `STATUS` - Show all servo states
- `HELP` - Show commands

## Design Principles

1. **Non-blocking**: All animations run via `millis()` polling in `loop()`. Serial commands work during animations.
2. **Per-servo calibration**: Each servo has its own min/max pulse range.
3. **Eased motion**: Uses cubic ease-in-out for organic movement.
4. **Art installation focus**: Wave patterns and sequences for kinetic sculptures.

## Known Issues

See [docs/PROGRESS.md](docs/PROGRESS.md) for current issues. Main concern:
- Continuous servo speed control may need per-servo calibration to find the actual stop pulse

## Adding New Features

When adding features:
1. Add state variables after existing state arrays (around line 55)
2. Add update function after `updateSequence()` (around line 300)
3. Call update function at start of `loop()`
4. Add command parsing in `processCommand()` (around line 389)
5. Update `showHelp()` in the sketch

## Before Committing

**Always do these before committing changes:**

1. **Manual test** - Use the `arduino-manual-testing` skill to create and execute test cases. Upload to Arduino and verify via Serial Monitor. Update `docs/TESTING.md` with pass/fail results as you test.
2. **Update README.md** - Update command tables if adding/changing commands
3. **Update docs/PROGRESS.md** - Add new commits to the Git Commits section, update "Current Issue" if relevant
4. **Update AGENTS.md** (this file) - Update line numbers in "Code Organization" section if code structure changes significantly
