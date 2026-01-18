# Sequence Setup Extraction Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Move the keyframe and speed sequence “setup” (the arrays users edit) out of `adafruit_16_servo.ino` into a dedicated file so installation owners can find and edit sequences without wading through core logic.

**Architecture:** Keep the playback engines (`updateSequence()`, `updateSpeedSequence()`) and runtime state in the sketch, but move the “data” (sequence arrays + selection helpers) into a new `sequence_setup.h` that is included after the `Keyframe` and `SpeedFrame` structs are defined.

**Tech Stack:** Arduino (C++), Adafruit PCA9685 PWM driver

---

## Task 1: Add dedicated sequence setup header

**Files:**
- Create: `sequence_setup.h`

**Step 1: Create `sequence_setup.h` with editable sequences**

- Define `sequence1` and `speedSeq1` arrays in this file.
- Compute lengths via `sizeof(array) / sizeof(array[0])` so end markers are always included.

**Step 2: Add small selection helpers**

Add `inline` helper functions:
- `selectPositionSequence(uint8_t seqNum, Keyframe*& outSeq, uint8_t& outLen)`
- `selectSpeedSequence(uint8_t seqNum, SpeedFrame*& outSeq, uint8_t& outLen)`

**Step 3: Commit**

```bash
git add sequence_setup.h
git commit -m "refactor: move sequence setup into sequence_setup.h"
```

---

## Task 2: Remove sequence definitions from the sketch and wire in the new header

**Files:**
- Modify: `adafruit_16_servo.ino`

**Step 1: Keep `Keyframe`/`SpeedFrame` structs in the sketch**

Ensure the structs remain in `adafruit_16_servo.ino` (so the playback engines keep their types).

**Step 2: Include `sequence_setup.h` after the struct definitions**

```cpp
#include "sequence_setup.h"
```

**Step 3: Delete the in-sketch `sequence1`/`speedSeq1` arrays and length globals**

Remove the “Example sequence storage” blocks that currently live near the top of the sketch.

**Step 4: Update `PLAY`/`SPLAY` command handlers to use the selection helpers**

Replace the inline `if (seqNum == 1) { ... }` selection with calls to:
- `selectPositionSequence(...)`
- `selectSpeedSequence(...)`

**Step 5: Commit**

```bash
git add adafruit_16_servo.ino
git commit -m "refactor: load sequences via sequence_setup.h selectors"
```

---

## Task 3: Update documentation pointers

**Files:**
- Modify: `README.md`

**Step 1: Update “Creating Custom Sequences” to point to `sequence_setup.h`**

Change the text and code examples so users edit sequences in `sequence_setup.h` instead of hunting inside the main sketch.

**Step 2: Commit**

```bash
git add README.md
git commit -m "docs: point sequence editing to sequence_setup.h"
```

---

## Task 4: Verification

**Step 1: Compile (if Arduino CLI is available)**

Run:
```bash
arduino-cli version
arduino-cli compile --fqbn arduino:avr:uno .
```

Expected: compile succeeds with no multiple-definition errors and no missing includes.

