# Manual Testing Plan

Current installation reference from `servo_setup.h`:
- Servo 0: 5-turn standard winch
- Servo 1: 5-turn standard winch
- Servo 2: 5-turn standard winch
- Servo 3: continuous rotation installation drive

## Setup Checklist

- [x] Arduino connected via USB
- [x] Serial Monitor open at 9600 baud
- [x] External power connected to servo power terminals
- [x] At least one winch servo connected (recommended: channel 0, 1, or 2)

---

## Test 1: Basic Communication

**Command:** `HELP`

**Expected:** List of available commands prints to Serial Monitor

**Result:**
- [x] Pass
- [ ] Fail - describe:

---

## Test 2: Status Check

**Command:** `STATUS`

**Expected:** Shows all 16 servos with default calibration (min=150, max=600, pos=375)

**Result:**
- [x] Pass
- [ ] Fail - describe:

---

## Test 3: Basic Servo Movement (Degrees)

**Commands (using servo 0 - standard winch):**
1. `STATUS` - Confirm servo 0 is standard, not continuous
2. `S0 900` - Move servo 0 to mid travel
3. `S0 0` - Move servo 0 to minimum
4. `S0 1800` - Move servo 0 to maximum

**Expected:** Servo 0 physically moves to each position across its configured 5-turn range, Serial confirms each move

**Result:**
- [ ] Not tested yet on current hardware
- [ ] Fail - describe:

**Note:** Current installation uses servo 3 for continuous rotation. Servos 0-2 are standard winch channels.

---

## Test 4: Raw Pulse Control

**Commands:**
1. `P0 150` - Move to minimum pulse
2. `P0 375` - Move to center pulse
3. `P0 600` - Move to maximum pulse

**Expected:** Servo moves, may buzz at extremes if outside physical range

**Result:**
- [x] Pass (used to find stop pulse 295 for servo 0)
- [ ] Fail - describe:

---

## Test 5: Calibration

**Commands:**
1. `CAL 0 110 480` - Restore expected range for servo 0
2. `STATUS` - Verify calibration saved
3. `S0 0` - Should go to pulse 110
4. `S0 1800` - Should go to pulse 480

**Expected:** Calibration updates, servo respects new range

**Result:**
- [ ] Not tested yet on current hardware
- [ ] Fail - describe:

---

## Test 7: Sweep

**Command:** `SWEEP 0`

**Expected:** Servo sweeps from min to max to min, returns to center

**Result:**
- [ ] Not tested yet on current hardware
- [ ] Fail - describe:

---

## Test 8: Release Protection

**Commands:**
1. `OFF 0`
2. Confirm Serial reports that servo 0 is release-protected
3. `RELEASE 0` only if it is mechanically safe to drop holding torque
4. If released, verify the winch can move freely or loses holding torque

**Expected:** `OFF 0` is blocked on the protected winch, and `RELEASE 0` only disengages when explicitly requested

**Result:**
- [ ] Not tested yet on current hardware
- [ ] Fail - describe:

---

## Test 9: Animated Move (MOVE)

**Commands:**
1. `S0 0` - Start at minimum
2. `MOVE 0 1800 2000` - Animate to full travel over 2 seconds

**Expected:** Smooth eased motion over ~2 seconds (not instant)

**Result:**
- [ ] Not tested yet on current hardware
- [ ] Fail - describe:

---

## Test 10: Wave Pattern

**Commands:**
1. `WAVE 0 3 50 30 90` - Wave on servos 0-3
2. Wait 5 seconds, observe
3. `STOP`

**Expected:** Servos 0-3 oscillate in sine wave pattern with phase offset

**Result:**
- [ ] Pass
- [ ] Skipped - requires multiple connected winch channels

---

## Test 11: Keyframe Sequence

**Commands:**
1. `PLAY 1`
2. Observe sequence

**Expected:** Servos 0, 1, 2 execute choreographed moves, "Sequence complete" prints

**Result:**
- [x] Pass (command executes, servo 0 moves; full test needs multiple positional servos)
- [ ] Fail - describe:

---

## Test 12: Sequence Loop

**Commands:**
1. `PLAY 1 LOOP`
2. Wait for at least 2 cycles
3. `STOP`

**Expected:** Sequence repeats, stops when STOP issued

**Result:**
- [ ] Pass
- [x] Skipped - requires multiple positional servos for meaningful test

---

## Test 13: Continuous Servo Mode (installation rotation servo)

**Setup required first:**
1. Confirm servo 3 is the configured continuous channel in `servo_setup.h`
2. Verify stop pulse is still correct with `P3 <value>` if needed
3. Use `STATUS` to confirm servo 3 reports `[CONT]`

**Commands:**
1. `ROTATE 0` - Stop
2. `ROTATE 35` - Slow forward rotation
3. `ROTATE 70` - Faster forward rotation
4. `ROTATE -35` - Reverse rotation
5. `ROTATE 0` - Stop

**Expected:** Servo 3 spins at different speeds/directions, and `ROTATE 0` returns it to stop

**Result:**
- [ ] Not tested yet on current hardware
- [ ] Fail - describe:

---

## Issues Found

| Test | Issue Description | Severity |
|------|-------------------|----------|
| 18 | Confirm whether `UP` / `DOWN` semantics match the physical installation direction labels | Behavior check |

---

## Verification Note: Directional Percent Commands

Manual hardware verification for the current `UP` / `DOWN` / `UMOVE` / `DMOVE` commands should be run against the live winch installation.

Suggested checks:
1. Pick one winch channel, preferably servo 0
2. Mark the drum or horn so direction changes are obvious
3. Run:
   - `DOWN 0 0`
   - `DOWN 0 50`
   - `DOWN 0 100`
   - `UP 0 0`
   - `UP 0 50`
   - `UP 0 100`
4. Repeat with animation:
   - `DMOVE 0 25 2000`
   - `UMOVE 0 75 2000`

Expected:
- `DOWN 0 0` goes to the fully up / retracted end
- `DOWN 0 100` goes to the fully down / lowered end
- `UP 0 0` goes to the fully down / lowered end
- `UP 0 100` goes to the fully up / retracted end
- `UMOVE` / `DMOVE` follow the same endpoints with smooth easing
- `UP x p` and `DOWN x (100-p)` land at the same physical point

---

## Test 14: Speed Sequences (Continuous Servos)

**Setup:** Servo 3 must be configured as continuous and the selected speed sequence must target the current installation layout

**Commands:**
1. `SPLAY 1` - Play speed sequence 1 once

**Expected:**
- Sequence should drive the configured continuous servo(s) according to `sequence_setup.h`
- Observe ramp-up, hold, reversal, and stop timing against the authored sequence
- Serial shows "Playing speed sequence 1" then ramp messages, then "Speed sequence complete"

**Result:**
- [x] Pass
- [ ] Fail - describe:

---

## Test 15: Speed Sequence Loop

**Commands:**
1. `SPLAY 1 LOOP` - Play speed sequence 1 looping
2. Wait for at least 2 cycles (~14 seconds)
3. `STOP` - Stop the sequence

**Expected:**
- Sequence repeats after 7s
- Serial shows "Speed sequence looping" at each restart
- STOP halts playback and stops both servos
- Serial shows "Stopped"

**Result:**
- [x] Pass
- [ ] Fail - describe:

---

## Test 16: Code Review Fixes (2026-01-19)

### 16a: Fixed Char Buffer - Basic Input

**Command:** `HELP`

**Expected:** Commands list displays correctly

**Result:**
- [x] Pass
- [ ] Fail - describe:

---

### 16b: Fixed Char Buffer - Long Input

**Command:** Type 50+ characters (e.g., `ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ`)

**Expected:** Truncates gracefully, shows "Unknown command. Type HELP" (no crash)

**Result:**
- [x] Pass
- [ ] Fail - describe:

---

### 16c: WAVE Bounds Validation - Invalid Range (start > end)

**Command:** `WAVE 10 5 50 30 90`

**Expected:** Shows "Invalid servo range"

**Result:**
- [x] Pass
- [ ] Fail - describe:

---

### 16d: WAVE Bounds Validation - Invalid Range (end >= NUM_SERVOS)

**Command:** `WAVE 0 20 50 30 90`

**Expected:** Shows "Invalid servo range"

**Result:**
- [x] Pass
- [ ] Fail - describe:

---

### 16e: Sweep Underflow Fix

**Command:** `SWEEP 0`

**Expected:** Sweep completes without hanging at end

**Result:**
- [x] Pass
- [ ] Fail - describe:

---

### 16f: Rapid Commands

**Commands:** Send quickly:
```
S0 45
S1 90
S2 135
STATUS
```

**Expected:** All commands process correctly, no memory issues

**Result:**
- [x] Pass
- [ ] Fail - describe:

---

### 16g: PROGMEM Keyframe Sequence

**Command:** `PLAY 1`

**Expected:** Sequence plays, "Sequence complete" message appears

**Result:**
- [x] Pass (PROGMEM read working; servos 0,1 are continuous so only partial motion)
- [ ] Fail - describe:

---

### 16h: PROGMEM Speed Sequence

**Command:** `SPLAY 1`

**Expected:** Speed sequence plays correctly on continuous servos

**Result:**
- [x] Pass
- [ ] Fail - describe:

---

### 16i: Memory Savings

**Compile output:**
```
Sketch uses 16108 bytes (49%) of program storage space.
Global variables use 1051 bytes (51%) of dynamic memory.
```

Sequence data moved to PROGMEM saves ~114 bytes RAM.

---

## Test 17: Combined Rig Manual Command

**Commands:**
1. `RIG UP 80 35 3000`
2. Observe all protected winches move together while the rotation servo ramps up
3. `RIG DOWN 20 -25`
4. Observe all protected winches move together while the rotation servo reverses immediately
5. `STOP`

**Expected:**
- All protected winch servos move to the requested shared position
- The first configured continuous servo changes speed from the same command
- Optional duration ramps the rotation servo over the same interval as the winch move
- `STOP` halts any remaining winch motion and returns continuous servos to stop

**Result:**
- [ ] Not tested yet on hardware

---

## Test 18: Winch Direction Semantics

This test is the main check for the current `UP` / `DOWN` concern.

**Setup:**
1. Use servo 0 first
2. Make sure the winch has enough clearance for full travel
3. Add a tape flag or visual mark so travel direction is easy to see

**Commands:**
1. `STATUS`
2. `DOWN 0 0`
3. `DOWN 0 100`
4. `UP 0 0`
5. `UP 0 100`
6. `DOWN 0 25`
7. `UP 0 75`
8. `DOWN 0 50`
9. `UP 0 50`

**Expected:**
- `DOWN 0 0` and `UP 0 100` match
- `DOWN 0 100` and `UP 0 0` match
- `DOWN 0 25` and `UP 0 75` match
- `DOWN 0 50` and `UP 0 50` match
- The labels feel physically correct for the installation:
  `UP` should retract / raise
  `DOWN` should lower / extend

**Result:**
- [ ] Not tested yet on hardware

---

## Test 19: Animated Winch Direction Semantics

**Commands:**
1. `DMOVE 0 20 2000`
2. Wait for completion
3. `UMOVE 0 80 2000`
4. Wait for completion
5. `DMOVE 0 50 2000`
6. `UMOVE 0 50 2000`

**Expected:**
- Animated commands reach the same endpoints as the non-animated commands
- Motion is smooth and non-blocking
- Final positions match the same `UP` / `DOWN` equivalence pairs

**Result:**
- [ ] Not tested yet on hardware

---

## Test 20: Installation Rotation Command

**Commands:**
1. `ROTATE 0`
2. `ROTATE 20`
3. `ROTATE 50`
4. `ROTATE -20`
5. `ROTATE 0`

**Expected:**
- Only the installation rotation servo responds
- Positive and negative values reverse direction
- Command no longer requires a servo number
- `ROTATE 0` stops motion cleanly

**Result:**
- [ ] Not tested yet on hardware

---

## Test 21: Per-Servo Stop

**Commands:**
1. `UMOVE 0 80 4000`
2. While servo 0 is still moving, send `STOP 0`
3. `ROTATE 40`
4. While the installation is rotating, send `STOP 3`

**Expected:**
- `STOP 0` immediately halts servo 0 and holds it at its current position
- `STOP 3` immediately stops the continuous rotation servo
- A later direct command to that servo should allow it to move again

**Result:**
- [ ] Not tested yet on hardware

---

## Servo Calibration Notes

| Channel | Servo Model | Type | Stop Pulse | Calibration |
|---------|-------------|------|------------|-------------|
| 0 | goBILDA 2000 Series 25-2 | Standard, 5-turn | N/A | `CAL 0 110 480` |
| 1 | goBILDA 2000 Series 25-2 | Standard, 5-turn | N/A | `CAL 1 110 480` |
| 2 | goBILDA 2000 Series 25-2 | Standard, 5-turn | N/A | `CAL 2 110 480` |
| 3 | goBILDA 2000 Series 25-3 | Continuous | 298 | `CAL 3 186 410`, configured continuous in `servo_setup.h` |
