
# Project Progress

## Overview

Servo calibration and control system for the Adafruit PCA9685 16-channel PWM driver, with animation capabilities for art installations.

## Completed Features

### 1. Core Servo Control
- Serial command interface at 9600 baud
- Per-servo calibration (min/max pulse values)
- Commands: `S<n> <deg>`, `P<n> <pulse>`, `CAL`, `SWEEP`, `OFF`, `STATUS`, `HELP`

### 2. Animation System (Tasks 1-9 Complete)
- **Easing functions** - Smooth ease-in-out cubic motion
- **Non-blocking animation engine** - Runs in `loop()`, doesn't block Serial
- **MOVE command** - Animated movement: `MOVE 0 180 2000` (servo 0 to 180° over 2 seconds)
- **WAVE command** - Sine wave patterns across servo groups: `WAVE 0 7 50 30 90`
- **Keyframe sequences** - Pre-programmed choreography with `PLAY 1 LOOP`
- **Chained programs** - Back-to-back `PLAY` / `SPLAY` sequence programs with `RUN 1 LOOP`
- **STOP command** - Halts all animations globally, or `STOP <n>` halts one servo immediately

### 3. Continuous Servo Mode (Complete)
- `MODE <n> CONT` - Marks a servo as continuous rotation
- `MODE <n> STD` - Switches back to standard positional servo
- `ROTATE <spd>` - Installation rotation control for the configured continuous servo
- `STOP` - Also stops all continuous servos
- STATUS shows `[CONT]` or `[STD]` for each servo

### 4. Speed Sequences for Continuous Servos (Complete)
- `SpeedFrame` struct for timed speed changes with ramping
- `rampServoSpeed()` for eased speed transitions
- `updateSpeedRamps()` engine in loop() for smooth interpolation
- `updateSpeedSequence()` playback engine for choreography
- `SPLAY <n> [LOOP]` command for speed sequence playback
- Cubic ease-in-out for organic speed ramping

### 5. Installation-Specific Setup Files (Complete)
- `servo_setup.h` for per-installation servo calibration and mode setup
- `sequence_setup.h` for per-installation animation sequence definitions (keyframes + speed sequences)

### 6. Code Review Improvements (Complete)
- Replaced Arduino `String` class with fixed 50-byte char buffer to prevent heap fragmentation
- Added WAVE command bounds validation (rejects invalid servo ranges)
- Cleaned up sweep function underflow protection
- Moved sequence data to PROGMEM (flash) to save RAM
- Added helper functions for C-style string operations (`trimString`, `toUpperCase`, `findChar`, `startsWith`, `containsStr`)

### 7. Percent-of-Travel Commands (Complete)
- `UP <n> <pct>` / `DOWN <n> <pct>` for explicit installation-oriented winch positioning
- `UMOVE <n> <pct> <ms>` / `DMOVE <n> <pct> <ms>` for eased directional percent moves
- Directional percent commands still reuse each servo's configured `totalDegrees`, which fits the 5-turn winch setup

### 8. Combined Rig Manual Testing Command (Complete)
- `RIG <UP|DOWN> <pct> <spd> [ms]` command for coordinated manual testing
- Reuses protected winch targeting plus continuous-servo speed/ramp control
- Intended for live testing outside of `PLAY` / `SPLAY`

### 9. Installation Sequence Library Updates (Complete)
- Added `PLAY 6` tripod walk sequence for the three winches
- Keeps one winch low enough to touch the base while the other two lift
- Rotates the base contact point between servos 0, 1, and 2 for a stepping motion

### 10. Browser-Baked Motion Playback (Complete)
- Added `MOTION <id>` command for schema v1 browser-baked Motions stored in EEPROM
- Parses the active bake blob without heap allocation and loads bounded servo/DC track data into runtime state
- Interpolates servo degrees and DC motor speed linearly against `millis()`
- Cancels active Motion on manual servo moves, `PLAY`, `SPLAY`, `RUN`, and `STOP`
- Added host regression coverage for Motion parsing, boardId filtering, and playback interpolation

### 11. Browser Sequence Editor (Complete)
- Added section `// 04 Sequencer` to `servo_controller.html`
- Authors schema v1 Sequences in the browser library with picker controls, tags, timed steps, target selection, labels, holds, preview, scrub, and drag reorder
- Supports Play/Pause/Stop/Loop auditioning from the browser and record-from-terminal step capture
- Saves authored Sequences to the existing `servoCluster.library` localStorage object used by import/export/bake

### 12. Browser Motion Keyframe Editor (Complete)
- Added section `// 06 Motion` to `servo_controller.html`
- Authors schema v1 cluster Motions across 12 tracks: three boards, each with three servo tracks and one DC track
- Supports create, duplicate, rename, delete, tag chips, duration edits, board collapse, per-track solo/mute, click-to-add keyframes, drag-to-move, double-click delete, snapped 100ms/1-unit editing, and Shift unsnapped editing
- Adds playhead scrub, previous-keyframe onion skin, live browser playback for tuning, direct `MOTION <id>` fire, and localStorage save through the shared bake library

## In Progress

### Motion Feasibility Guardrails
- Baked `MOTION` servo playback now uses the same 0..100% down semantic as the browser Motion editor instead of the older 0..180 degree path
- The browser now analyzes each servo keyframe segment against the measured winch floor (`77ms` per percent of travel), blocks impossible Play/Bake/Export sends, and warns on slow staccato-prone segments
- Servo lanes render per-segment feasibility overlays and first-pass drag/click clamping keeps new keyframes inside physically possible timing windows
- Remaining gap: no dedicated typed per-keyframe editor yet, and the browser-side feasibility helper does not have its own standalone test harness

## File Structure

```
adafruit_16_servo/
├── adafruit_16_servo.ino    # Main Arduino sketch
├── servo_setup.h            # Installation-specific servo setup
├── sequence_setup.h         # Installation-specific animation sequences
├── README.md                 # User documentation
├── AGENTS.md                 # AI assistant context
└── docs/
    ├── plans/
    │   ├── 2026-01-17-animation-system.md  # Animation implementation plan (done)
    │   ├── 2026-01-18-servo-config-refactor.md  # Servo config refactor plan (done)
    │   ├── 2026-01-18-continuous-servo-sequences.md  # Speed sequences plan (done)
    │   ├── 2026-01-18-sequence-setup-extraction.md  # Sequence setup extraction plan (done)
    │   └── 2026-01-19-code-review-fixes.md  # Code review improvements plan (done)
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

## Git Commits (Refactors)

18. `refactor: replace parallel arrays with ServoConfig/ServoState structs and separate installation-specific setup`
19. `refactor: move sequence definitions to sequence_setup.h for better organization`

## Git Commits (Code Review Fixes)

20. `refactor: replace String with fixed char buffer and add PROGMEM for sequences`

## Git Commits (Winch Percent Control)

21. `feat: add percent-of-travel commands for winch positioning`

## Git Commits (Manual Rig Testing)

22. `feat: add combined rig test command for winches and rotation`
23. `refactor: clarify installation control commands`
24. `feat: add per-servo stop command`

## Git Commits (Sequence Library)

25. `feat: add tripod walk winch sequence`

## Git Commits (Browser-Baked Motion Playback)

26. `feat: add MOTION playback for baked browser motions`

## Git Commits (Browser Sequence Editor)

27. `feat: add browser sequence editor`

## Git Commits (Browser Motion Editor)

28. `feat: add browser motion editor`

## Git Commits (Browser Bake Recovery)

29. `fix: retry bake recovery info probes`

## Git Commits (Network Recovery)

30. `fix: restart network services after wifi reconnect`

## Git Commits (Setlist Scheduler)

33. `feat: setlist scheduler — RUN AUTO (servo-dos)` — leader-gated scheduler that runs the active Setlist forever: ordered or weighted-shuffle (minGapEntries, per-entry repeat/gapMs), riding the existing RUN/STOP command mirror so followers stay in lock-step. New `setlist_scheduler.h` + `test_setlist_scheduler.cpp` (10 tests). avoidSameTag/moodArc deferred (schema v2, servo-yxd). Rebased onto servo-voc (legacy removal), so OTA headroom is healthy again (see the rebase build).

## Git Commits (Library Hydration)

31. `feat: hydrate browser library from boards' baked EEPROM (servo-7mn)` — adds firmware `GET /sequences` (streams the active baked payload), a browser "Pull from Boards" button, and empty-on-load auto-pull. Reconciles per-board slices: motion tracks union by board, full-library fields must match across boards or the operator picks a source-of-truth board. Also fixes a latent `normalizeKeyframes` crash on sliced (partial-track) motions.

## Git Commits (Legacy Removal)

32. `refactor: remove legacy PLAY/SPLAY/RUN-n sequence engine + TIMESCALE (servo-voc)` — deletes `sequence_setup.h`, the legacy playback engines (`updateSequence`/`updateSpeedSequence`/`updateSequenceProgram` + start/select helpers), their runtime structs/globals, the `PLAY`/`SPLAY`/`RUN <n>`/`TIMESCALE` command handlers, and the matching browser controls (// 01 quick-play buttons, // 02 Position/Speed-sequence telemetry + timescale, status.json `sequence`/`speedSeq`/`timescale` fields). Schema-v1 `MOTION <id>` / `RUN <id>` are the only playback path. Reclaimed ~5.1 KB OTA flash: 120200 → 115076 (+7804 headroom). Webpage verified (renderPills now shows RUN/MOTION; no console errors). Legacy time-multiplier host suite removed; `make -C test` now aggregates storage/motion/sequence.

## Future Features

- [ ] **EEPROM calibration storage** - Save/load servo calibrations (type, min, max, stop pulse) to persist across power cycles. Commands: `SAVE`, `LOAD`, `CLEAR`

## Hardware Notes

- Adafruit PCA9685 16-channel PWM driver (I2C)
- Default pulse range: 150-600 (12-bit, out of 4096)
- PWM frequency: 50Hz (standard servo)
- Requires external 5-6V power for servos
