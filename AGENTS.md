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
├── motion_engine.h          # Browser-baked Motion parser/playback engine
├── servo_setup.h            # Installation-specific per-servo setup
├── sequence_setup.h         # Installation-specific animation sequence setup
├── README.md                 # User documentation
├── AGENTS.md                 # This file - AI context
└── docs/
    ├── PROGRESS.md           # Development progress/issues
    └── plans/
        └── 2026-01-17-animation-system.md  # Completed implementation plan
```

## Code Organization

The sketch is split into small headers included by `adafruit_16_servo.ino`:

1. **Sketch shell** (`adafruit_16_servo.ino`): Wi-Fi/OTA/web setup, global runtime state, `setup()`, and `loop()`
2. **Runtime model** (`servo_runtime.h`): `ServoConfig`, `ServoState`, sequence structs, and browser-baked `MotionRuntime`
3. **Servo/DC control** (`servo_control.h`, `dc_motor.h`): calibrated servo writes, eased moves, percent travel commands, and motor speed/ramp helpers
4. **Animation engines** (`animation_engine.h`, `motion_engine.h`): `PLAY`, `SPLAY`, `RUN`, `WAVE`, and baked `MOTION <id>` playback
5. **Command interface** (`command_interface.h`): `showHelp()`, `processCommand()`, and mirrored command dispatch
6. **Persistence/web** (`storage.h`, `bake_validate.h`, `Web.cpp`): EEPROM bake slots, schema validation, boardId, and HTTP endpoints

## State Model

Per-servo setup and runtime state is tracked in two arrays of structs:

| Array | Type | Purpose |
|-------|------|---------|
| `servoConfig[]` | `ServoConfig` | Calibration + behavior (`minPulse`, `maxPulse`, `continuous`, `stopPulse`) |
| `servoState[]` | `ServoState` | Runtime state (position + move animation + speed ramping) |
| `motionRuntime` | `MotionRuntime` | Active browser-baked Motion tracks/keyframes loaded from EEPROM |

## Command Reference

### Basic Control
- `S<n> <deg>` - Move servo to degrees (0-180)
- `L<n> <pct>` - Move servo to percent of configured travel
- `P<n> <pulse>` - Move servo to raw pulse value
- `CAL <n> <min> <max>` - Set pulse calibration
- `SWEEP <n>` - Test sweep full range
- `OFF <n>` - Stop sending PWM

### Animation
- `MOVE <n> <deg> <ms>` - Smooth eased movement
- `LMOVE <n> <pct> <ms>` - Smooth eased move to percent of travel
- `WAVE <s> <e> [spd] [off] [amp]` - Sine wave pattern across servos
- `MOTION <id>` - Play a browser-baked Motion from EEPROM
- `PLAY <n> [LOOP]` - Play keyframe sequence
- `RUN <n> [LOOP]` - Run a chained program of sequences
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

## Landing the Plane (Session Completion)

**When ending a work session**, you MUST complete ALL steps below. Work is NOT complete until `git push` succeeds.

**MANDATORY WORKFLOW:**

1. **File issues for remaining work** - Create issues for anything that needs follow-up
2. **Run quality gates** (if code changed) - Tests, linters, builds
3. **Update issue status** - Close finished work, update in-progress items
4. **PUSH TO REMOTE** - This is MANDATORY:
   ```bash
   git pull --rebase
   bd sync
   git push
   git status  # MUST show "up to date with origin"
   ```
5. **Clean up** - Clear stashes, prune remote branches
6. **Verify** - All changes committed AND pushed
7. **Hand off** - Provide context for next session

**CRITICAL RULES:**
- Work is NOT complete until `git push` succeeds
- NEVER stop before pushing - that leaves work stranded locally
- NEVER say "ready to push when you are" - YOU must push
- If push fails, resolve and retry until it succeeds

<!-- BEGIN BEADS INTEGRATION -->
## Issue Tracking with bd (beads)

**IMPORTANT**: This project uses **bd (beads)** for ALL issue tracking. Do NOT use markdown TODOs, task lists, or other tracking methods.

### Why bd?

- Dependency-aware: Track blockers and relationships between issues
- Git-friendly: Dolt-powered version control with native sync
- Agent-optimized: JSON output, ready work detection, discovered-from links
- Prevents duplicate tracking systems and confusion

### Quick Start

**Check for ready work:**

```bash
bd ready --json
```

**Create new issues:**

```bash
bd create "Issue title" --description="Detailed context" -t bug|feature|task -p 0-4 --json
bd create "Issue title" --description="What this issue is about" -p 1 --deps discovered-from:bd-123 --json
```

**Claim and update:**

```bash
bd update <id> --claim --json
bd update bd-42 --priority 1 --json
```

**Complete work:**

```bash
bd close bd-42 --reason "Completed" --json
```

### Issue Types

- `bug` - Something broken
- `feature` - New functionality
- `task` - Work item (tests, docs, refactoring)
- `epic` - Large feature with subtasks
- `chore` - Maintenance (dependencies, tooling)

### Priorities

- `0` - Critical (security, data loss, broken builds)
- `1` - High (major features, important bugs)
- `2` - Medium (default, nice-to-have)
- `3` - Low (polish, optimization)
- `4` - Backlog (future ideas)

### Workflow for AI Agents

1. **Check ready work**: `bd ready` shows unblocked issues
2. **Claim your task atomically**: `bd update <id> --claim`
3. **Work on it**: Implement, test, document
4. **Discover new work?** Create linked issue:
   - `bd create "Found bug" --description="Details about what was found" -p 1 --deps discovered-from:<parent-id>`
5. **Complete**: `bd close <id> --reason "Done"`

### Auto-Sync

bd automatically syncs via Dolt:

- Each write auto-commits to Dolt history
- Use `bd dolt push`/`bd dolt pull` for remote sync
- No manual export/import needed!

### Important Rules

- ✅ Use bd for ALL task tracking
- ✅ Always use `--json` flag for programmatic use
- ✅ Link discovered work with `discovered-from` dependencies
- ✅ Check `bd ready` before asking "what should I work on?"
- ❌ Do NOT create markdown TODO lists
- ❌ Do NOT use external issue trackers
- ❌ Do NOT duplicate tracking systems

For more details, see README.md and docs/QUICKSTART.md.

<!-- END BEADS INTEGRATION -->
