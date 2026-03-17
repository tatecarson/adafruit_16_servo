# Travel Limits - Manual Test Plan

Tests for the per-servo `upDegrees` / `downDegrees` travel limits feature.
Placeholder `downDegrees = 900` is configured for servos 0-2.

---

## Test 1: STATUS Display

**Command:** `STATUS`

**Expected:**
- Servos 0, 1, 2 show `travel=0-900` after the range field
- Other servos do NOT show travel info

**Result:**
- [x] Confirmed on hardware 

---

## Test 2: Raw Degrees Bypass

**Commands:**
1. `S0 0`
2. `S0 450`
3. `S0 900`
4. `S0 1800`

**Expected:**
- `S0 0` → minPulse (110) — since higher degrees = physically up on this winch, this is the physically lowest position
- `S0 450` → quarter turn — winch should wind up slightly from the lowest position
- `S0 900` → half of full range — winch should be noticeably higher
- `S0 1800` → maxPulse (480) — full 5 turns, physically highest position
- Raw degree commands bypass travel limits and reverseDir — the full 0-1800 range is always accessible for calibration

**Result:**
- [x] Confirmed on hardware

---

## Test 3: DOWN 0 100 Respects Limit

**Commands:**
1. `UP 0 100` - Go fully up
2. `DOWN 0 100` - Go fully down

**Expected:**
- `UP 0 100` moves servo to 0 degrees (fully up)
- `DOWN 0 100` moves servo to 900 degrees (NOT 1800) — ring goes fully down without overshooting or rewinding

**Result:**
- [x] Confirmed on hardware

---

## Test 4: Midpoint Percentage

**Command:** `DOWN 0 50`

**Expected:**
- Servo moves to ~450 degrees (midpoint of 0-900 travel range)
- Should be roughly halfway between fully up and fully down

**Result:**
- [x] Confirmed on hardware

---

## Test 5: Round Trip

**Commands:**
1. `DOWN 0 100` - Go fully down
2. `UP 0 100` - Go fully up

**Expected:**
- Ring goes fully down then fully back up
- No overshoot in either direction
- Repeatable without drift

**Result:**
- [x] Confirmed on hardware
