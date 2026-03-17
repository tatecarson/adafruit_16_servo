# All Commands - Manual Test Plan

Ordered from simplest to most complex, ending with sequences.
Assumes servo 0-2 are standard winches (reverseDir configured), servo 3 is continuous rotation.

**Note:** Sequences use raw degrees (not percent commands), so they bypass travel limits and reverseDir. Higher degrees = physically up on all winch servos. Sequences have been updated to stay within 0-1500 degrees.

---

## 1. STATUS

**Command:** `STATUS`

**Expected:**
- All 16 servos listed with calibration values
- Servos 0-2: min=110 max=480, travel=0-1500, range=0-1800
- Servo 3: [CONT] with stop=298

**Result:**
- [ ] Pass

---

## 2. CENTER

**Commands:**
1. `CENTER 0` — standard servo goes to midpoint of travel
2. `CENTER 3` — continuous servo stops (goes to stopPulse)

**Expected:**
- Servo 0 moves to 900 degrees (midpoint of 1800)
- Servo 3 stops spinning (if it was moving)

**Result:**
- [ ] Pass

---

## 3. OFF and RELEASE (protection check)

**Commands:**
1. `OFF 0` — should be BLOCKED (servo 0 is release-protected)
2. `OFF 3` — should work (servo 3 allows release)
3. `RELEASE 0` — force-releases servo 0 (WARNING: winch will drop under load)

**Expected:**
- `OFF 0` prints "release-protected" and does nothing
- `OFF 3` turns off servo 3
- Only run `RELEASE 0` if mechanically safe — servo loses holding torque

**Result:**
- [ ] OFF blocked on protected servo
- [ ] OFF works on unprotected servo
- [ ] RELEASE (skip if unsafe)

---

## 4. ROTATE

**Commands:**
1. `ROTATE 0` — stop
2. `ROTATE 30` — slow forward
3. `ROTATE 70` — faster forward
4. `ROTATE -30` — reverse
5. `ROTATE 0` — stop

**Expected:**
- Servo 3 (continuous) spins at different speeds
- Positive/negative control direction
- `ROTATE 0` reliably stops

**Result:**
- [ ] Pass

---

## 5. STOP (global and per-servo)

**Commands:**
1. `ROTATE 50` — start servo 3 spinning
2. `STOP 3` — stop only servo 3
3. Confirm servo 3 stopped
4. `ROTATE 50` — start again
5. `STOP` — stop everything

**Expected:**
- `STOP 3` holds servo 3, other servos unaffected
- `STOP` (no argument) stops all motion globally

**Result:**
- [ ] Per-servo stop works
- [ ] Global stop works

---

## 6. MOVE (animated degrees)

**Commands:**
1. `S0 0` — start at minimum
2. `MOVE 0 900 2000` — animate to 900 degrees over 2 seconds
3. `MOVE 0 0 2000` — animate back over 2 seconds

**Expected:**
- Smooth eased motion (not instant jump)
- Takes approximately 2 seconds each way

**Result:**
- [ ] Pass

---

## 7. UMOVE / DMOVE (animated percent)

**Commands:**
1. `UP 0 100` — start fully up
2. `DMOVE 0 100 3000` — animate fully down over 3 seconds
3. `UMOVE 0 100 3000` — animate fully up over 3 seconds

**Expected:**
- Smooth eased motion respecting travel limits and reverseDir
- Direction matches UP/DOWN commands
- Takes approximately 3 seconds each way

**Result:**
- [ ] Pass

---

## 8. STOP mid-animation

**Commands:**
1. `UMOVE 0 100 5000` — start a slow 5-second move
2. After ~2 seconds: `STOP 0` — freeze mid-move

**Expected:**
- Servo 0 stops and holds at its current position (not snapping to start or end)
- Serial prints "Servo 0 held"

**Result:**
- [ ] Pass

---

## 9. ALLUP / ALLDOWN (instant)

**Commands:**
1. `ALLDOWN 100` — all protected winches fully down
2. `ALLUP 100` — all protected winches fully up

**Expected:**
- Servos 0, 1, 2 all move together (servo 3 continuous is skipped)
- Respects travel limits and reverseDir per servo
- Ring stays level if all servos are calibrated the same

**Result:**
- [ ] Pass

---

## 10. ALLUP / ALLDOWN (animated)

**Commands:**
1. `ALLUP 100` — start fully up
2. `ALLDOWN 100 3000` — all winches lower together over 3 seconds
3. `ALLUP 100 3000` — all winches raise together over 3 seconds

**Expected:**
- All three winch servos move in sync with eased animation
- Ring stays level throughout the move

**Result:**
- [ ] Pass

---

## 11. WAVE

**Commands:**
1. `WAVE 0 2 50 30 90` — wave pattern on servos 0-2
2. Observe for 10 seconds
3. `STOP`

**Expected:**
- Servos 0-2 oscillate in a sine wave with phase offset between them
- Ring tilts and rotates smoothly
- `STOP` halts the wave immediately

**Result:**
- [ ] Pass

---

## 12. RIG (combined winch + rotation)

**Commands:**
1. `RIG UP 80 35 3000` — winches to 80% up + rotation at 35% speed, ramped over 3s
2. Observe: winches moving + rotation starting
3. `RIG DOWN 20 -25` — winches to 20% down + reverse rotation, instant
4. `STOP`

**Expected:**
- All protected winch servos move to the shared position
- Servo 3 (continuous) starts spinning at the requested speed
- Duration parameter ramps both winches and rotation together
- Without duration, changes are instant

**Result:**
- [ ] Pass

---

## 13. TIMESCALE

**Commands:**
1. `TIMESCALE` — show current multiplier (should be 1)
2. `TIMESCALE 2` — set to 2x slower
3. `TIMESCALE` — confirm shows 2
4. `TIMESCALE 1` — reset to normal

**Expected:**
- Multiplier affects PLAY/SPLAY sequence timing (tested in next steps)
- Setting persists until changed or Arduino reset

**Result:**
- [ ] Pass

---

## 14. PLAY 1 — Staggered sweep

Sequence: servos 0-2 sweep down, up, then center with a slight stagger.

**Command:** `PLAY 1`

**Expected:**
- t=0: servos 0,1,2 move to 0 degrees (physically lowest) in quick succession
- t=1s: servos move to 1500 degrees (physically highest) in quick succession
- t=2s: servos return to 750 degrees (mid position)
- t=2.7s: "Sequence complete" prints

**Result:**
- [ ] Pass

---

## 15. PLAY 2 — Slow raise and lower

Sequence: all three servos raise together over 4s, hold 2s, lower over 4s.

**Command:** `PLAY 2`

**Expected:**
- t=0: all servos begin rising to 1500 degrees over 4 seconds
- t=4-6s: hold at top
- t=6s: all servos lower to 0 degrees over 4 seconds
- t=10s: "Sequence complete" prints
- Motion should be smooth and level

**Result:**
- [ ] Pass

---

## 16. PLAY 3 — Tilt sweep

Sequence: ring tilts toward each winch point in turn, sweeping around.

**Command:** `PLAY 3`

**Expected:**
- t=0: all servos go to 0 (level at bottom)
- t=1.5s: servo 0 rises to 1000 (ring tilts toward servo 0)
- t=4s: servo 0 lowers, servo 1 rises to 1000 (tilt shifts to servo 1)
- t=6.5s: servo 1 lowers, servo 2 rises to 1000 (tilt shifts to servo 2)
- t=9s: servo 2 lowers (back to level)
- t=11s: "Sequence complete"

**Result:**
- [ ] Pass

---

## 17. PLAY 4 — Gentle bob

Sequence: all winches oscillate low-to-mid for a breathing/bobbing effect. Good for looping.

**Command:** `PLAY 4`

**Expected:**
- t=0: all servos rise to 250 degrees over 1.5s
- t=1.5s: rise to 750 degrees over 1.5s
- t=3s: lower back to 250 degrees over 1.5s
- t=4.5s: "Sequence complete"
- Ring stays level, gentle up-and-down motion

**Result:**
- [ ] Pass

---

## 18. PLAY 4 LOOP — Looping bob

**Commands:**
1. `PLAY 4 LOOP`
2. Watch for at least 2 full cycles (~9 seconds)
3. `STOP`

**Expected:**
- Bobbing motion repeats seamlessly
- Serial shows "Sequence looping" at each restart
- `STOP` halts immediately

**Result:**
- [ ] Pass

---

## 19. PLAY 5 — Wave tilt

Sequence: rolling wave where each winch takes turns as the high point. Best with LOOP.

**Command:** `PLAY 5`

**Expected:**
- t=0: servo 0 rises to 1000, servos 1-2 stay low at 300
- t=1.5s: servo 1 becomes high point, servo 0 drops
- t=3s: servo 2 becomes high point, servo 1 drops
- t=4.5s: resets to servo 0 high for smooth loop transition
- t=6s: "Sequence complete"

**Result:**
- [ ] Pass

---

## 20. PLAY 5 LOOP — Looping wave tilt

**Commands:**
1. `PLAY 5 LOOP`
2. Watch for at least 2 full cycles (~12 seconds)
3. `STOP`

**Expected:**
- Wave pattern rolls continuously around the ring
- Smooth transition at loop point (no stutter)
- `STOP` halts immediately

**Result:**
- [ ] Pass

---

## 21. SPLAY 1 — Speed sequence

Sequence: servo 3 (continuous) ramps to 50%, stops, reverses, stops.

**Command:** `SPLAY 1`

**Expected:**
- t=0: servo 3 ramps to 50% speed over 500ms
- t=3s: ramps to stop
- t=4s: ramps to -50% (reverse)
- t=7s: ramps to stop
- t=7.5s: "Speed sequence complete"

**Result:**
- [ ] Pass

---

## 22. TIMESCALE with sequences

**Commands:**
1. `TIMESCALE 2`
2. `PLAY 4` — should take ~9s instead of ~4.5s
3. `TIMESCALE 1` — reset

**Expected:**
- Sequence plays at half speed (2x duration)
- All timing doubles but motion remains smooth

**Result:**
- [ ] Pass
