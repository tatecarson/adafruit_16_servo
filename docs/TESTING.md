# Manual Testing Plan

## Setup Checklist

- [x] Arduino connected via USB
- [x] Serial Monitor open at 9600 baud
- [x] External power connected to servo power terminals
- [x] At least one servo connected (note which channel: 0, 1, 4)

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

**Commands (using servo 4 - standard positional):**
1. `CAL 4 150 450` - Set calibration first
2. `S4 90` - Move servo 4 to center
3. `S4 0` - Move servo 4 to minimum
4. `S4 180` - Move servo 4 to maximum

**Expected:** Servo physically moves to each position, Serial confirms each move

**Result:**
- [x] Pass (servo 4 with calibration)
- [ ] Fail - describe:

**Note:** Servo 0 is continuous rotation - use MODE/SPEED commands instead.

---

## Test 4: Raw Pulse Control

**Commands:**
1. `P0 150` - Move to minimum pulse
2. `P0 375` - Move to center pulse
3. `P0 600` - Move to maximum pulse

**Expected:** Servo moves, may buzz at extremes if outside physical range

**Result:**
- [ ] Pass
- [ ] Fail - describe:

---

## Test 5: Calibration

**Commands:**
1. `CAL 0 200 550` - Set narrower range
2. `STATUS` - Verify calibration saved
3. `S0 0` - Should go to pulse 200
4. `S0 180` - Should go to pulse 550

**Expected:** Calibration updates, servo respects new range

**Result:**
- [ ] Pass
- [ ] Fail - describe:

---

## Test 6: Center Command

**Command:** `CENTER 0`

**Expected:** Servo moves to 90° (center of calibrated range)

**Result:**
- [ ] Pass
- [ ] Fail - describe:

---

## Test 7: Sweep

**Command:** `SWEEP 0`

**Expected:** Servo sweeps from min to max to min, returns to center

**Result:**
- [ ] Pass
- [ ] Fail - describe:

---

## Test 8: Off Command

**Commands:**
1. `OFF 0`
2. Physically push the servo horn

**Expected:** Servo goes limp (no holding torque), can be moved by hand

**Result:**
- [ ] Pass
- [ ] Fail - describe:

---

## Test 9: Animated Move (MOVE)

**Commands:**
1. `S0 0` - Start at 0°
2. `MOVE 0 180 2000` - Animate to 180° over 2 seconds

**Expected:** Smooth eased motion over ~2 seconds (not instant)

**Result:**
- [ ] Pass
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
- [ ] Fail - describe:

---

## Test 11: Keyframe Sequence

**Commands:**
1. `PLAY 1`
2. Observe sequence

**Expected:** Servos 0, 1, 2 execute choreographed moves, "Sequence complete" prints

**Result:**
- [ ] Pass
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
- [ ] Fail - describe:

---

## Test 13: Continuous Servo Mode (if you have a continuous servo)

**Setup required first:**
1. Find stop pulse with `P0 <value>` (was 295 for SM-S4303R)
2. `CAL 0 150 440` - Set range centered on stop pulse
3. `MODE 0 CONT`

**Commands:**
1. `SPEED 0 0` - Stop
2. `SPEED 0 50` - 50% speed forward
3. `SPEED 0 -50` - 50% speed reverse
4. `SPEED 0 0` - Stop

**Expected:** Servo spins at different speeds/directions, stops at 0

**Result:**
- [x] Pass (after calibration)
- [ ] Fail - describe:

---

## Issues Found

| Test | Issue Description | Severity |
|------|-------------------|----------|
| 3 | Servo 0 is continuous rotation, not positional | Hardware config |

---

## Servo Calibration Notes

| Channel | Servo Model | Type | Stop Pulse | Calibration |
|---------|-------------|------|------------|-------------|
| 0 | SM-S4303R | Continuous | 295 | `CAL 0 150 440` then `MODE 0 CONT` |
| 1 | | | | |
| 4 | Hitec HS-805BB+ | Standard | N/A | `CAL 4 150 450` |

