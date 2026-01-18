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
├── adafruit_16_servo.ino    # Main Arduino sketch (core logic)
├── servo_setup.h            # Installation-specific per-servo setup
├── README.md                 # User documentation
├── AGENTS.md                 # This file - AI context
└── docs/
    ├── PROGRESS.md           # Development progress/issues
    └── plans/
        └── 2026-01-17-animation-system.md  # Completed implementation plan
```

## Code Organization (adafruit_16_servo.ino)

The sketch is organized in this order:

1. **Includes & Globals** (lines 1-167): Library includes, `ServoConfig`/`ServoState`, wave + sequence globals
2. **Setup** (lines 168-183): Serial init, default init, `applyCustomServoSetup()`, PWM init
3. **Utility Functions** (lines 185-223): `degreesToPulse()`, `speedToPulse()`, easing functions
4. **Servo Control** (lines 224-320): `setServoPulse()`, `setServoDegrees()`, `setServoSpeed()`, `moveServoAnimated()`
5. **Animation Engines** (lines 321-475): `updateAnimations()`, `updateWave()`, `updateSequence()`
6. **Commands** (lines 477-563): `sweepServo()`, `servoOff()`, `setCalibration()`, `showStatus()`, `showHelp()`
7. **Command Parser** (lines 565-794): `processCommand()` - parses all Serial commands
8. **Main Loop** (lines 796-819): Calls update functions, reads Serial

## State Model

Per-servo setup and runtime state is tracked in two arrays of structs:

| Array | Type | Purpose |
|-------|------|---------|
| `servoConfig[]` | `ServoConfig` | Calibration + behavior (`minPulse`, `maxPulse`, `continuous`, `stopPulse`) |
| `servoState[]` | `ServoState` | Runtime state (position + move animation + speed ramping) |

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
1. Add per-servo fields to `ServoConfig`/`ServoState` (around line 33)
2. Add update function after `updateSpeedSequence()` (around line 475)
3. Call update function at start of `loop()` (around line 796)
4. Add command parsing in `processCommand()` (around line 565)
5. Update `showHelp()` in the sketch

## Before Committing

**Always do these before committing changes:**

1. **Manual test** - Use the `arduino-manual-testing` skill to create and execute test cases. Upload to Arduino and verify via Serial Monitor. Update `docs/TESTING.md` with pass/fail results as you test.
2. **Update README.md** - Update command tables if adding/changing commands
3. **Update docs/PROGRESS.md** - Add new commits to the Git Commits section, update "Current Issue" if relevant
4. **Update AGENTS.md** (this file) - Update line numbers in "Code Organization" section if code structure changes significantly
