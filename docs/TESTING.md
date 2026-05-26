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

## Test 10: Wave Pattern (REMOVED)

The `WAVE` command was removed in servo-dz7 to free flash space for the
OTA partition (the `sinf()` call pulled in ~3 KB of trig library code).
Sending `WAVE` now returns `WAVE not supported in this firmware` so old
gallery scripts fail gracefully rather than executing.

To re-add a wave-style pattern, author it as a browser-baked Motion with
keyframes — schema doc §2.

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

## Test 11b: Tripod Walk Sequence

**Commands:**
1. `PLAY 6`
2. Observe the three winches through one full cycle

**Expected:** Two winches lift while the third remains low enough to keep the platform touching the base, and the touch point rotates across servos 0, 1, and 2

**Result:**
- [ ] Not tested yet on current hardware
- [ ] Fail - describe:

**Bench verification:**
- [x] `arduino-cli compile --fqbn arduino:avr:uno .`

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

## Browser Sequencer Editor Smoke Test - 2026-05-24

**Scope:** `servo_controller.html` section `// 04 Sequencer`

**Environment:**
- Served locally with `python3 -m http.server 4173 --bind 127.0.0.1`
- Opened in the Codex in-app browser at `http://127.0.0.1:4173/servo_controller.html`

**Checks:**
1. Load page and verify sections `// 01` through `// 05` render with `// 04 Sequencer`.
2. Verify the sequencer initializes a default one-step Sequence with total duration `1s`.
3. Click `Add Step`.
4. Edit step 1 to `PLAY 1`, duration `1500`.
5. Edit step 2 to `STOP`, duration `500`.
6. Click `Save`.

**Expected:** Two steps remain visible, total duration reads `2s`, and status reads `saved to localStorage`.

**Result:** Pass.

**Not run:** Hardware playback, terminal record-to-board, and bake-to-board were not run because no reachable Arduino hardware was attached in this session.

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

## Test 22: Browser-Baked Motion Playback

**Prerequisite:** Upload a schema v1 bake blob with a Motion id such as `tidal-drift` through `POST /sequences`.

**Commands:**
1. `STORAGEINFO`
2. `MOTION tidal-drift`
3. While it is still playing, send `S0 90`
4. Run `MOTION tidal-drift` again
5. While it is still playing, send `STOP`

**Expected:**
- `STORAGEINFO` reports an active EEPROM slot
- `MOTION tidal-drift` starts the baked servo/DC tracks and prints the track count
- Servo/DC values interpolate smoothly between keyframes
- `S0 90` interrupts the active Motion, stops any Motion-owned DC track, and holds the manual servo command
- `STOP` cancels the active Motion and stops the DC motor

**Result:**
- [x] **2026-05-23 — PASS on board 192.168.8.198 (USB-flashed `5a8f4f6` + doc commit `7b11f5d`, boardId=1).** `STORAGEINFO` reported active+previous slots. `MOTION tidal-drift` started cleanly with both servo and DC tracks (initial servo wiring was loose — reseated and the servo sweep matched the keyframes). Manual servo commands (`S0 90`) and `STOP` cancelled the active Motion as expected. DC steps were instant (per the documented v1 bypass-ramp behavior). No regressions in the existing PLAY/SPLAY/ROTATE commands.

OTA path discovered to be unsafe during this session (silent flash corruption above 120 KB partition limit); tracked in `servo-8zb` and fixed in commit `1180518`. Hardware re-test against `1180518` deferred — fix is host-test-covered behavior and the change to `Web.cpp` is a pre-body 413 short-circuit that does not affect MOTION playback.

---

## Test 23: Browser-Baked Sequence Runner

**Prerequisite:** Upload a schema v1 bake blob with a `sequences[]` array — e.g. add an `evening-arc` sequence with steps that fire `MOTION <id>` + `ROTATE n` + `STOP`, mixed targets.

**Commands:**
1. `STORAGEINFO` — verify active slot
2. `RUN evening-arc` — alphabetic id routes to schema-v1 path
3. While running, send `RUN evening-arc LOOP` — verify mid-run cancel + restart with loop
4. While looping, send `STOP` — verify clean halt
5. `RUN 2` — verify legacy numeric path still works (was `RUN 1` before the showcase program was removed in the flash-cleanup commit)
6. `RUN nope` — verify "RUN failed: sequence-not-found"

**Expected:**
- Each step's `cmd` fires through `processCommand` at its `durationMs` mark.
- `target: 2` steps only execute on board 2 — boards 1/3 advance the clock past them silently.
- LOOP restarts at step 0 indefinitely until STOP.
- `STOP` halts the runner AND any in-flight inner cmd (MOTION/SPLAY/etc).
- `/status.json` exposes `runseq.{active, id, step, steps, loop, stepMs}`.

**Result:**
- [x] **2026-05-24 — PASS on hardware (firmware `6a4c606`, post-cleanup).** Bake with `test-arc` + `board-filter` sequences uploaded via `POST /sequences`. `RUN test-arc` fired all six steps at their `durationMs` marks; mid-run `RUN test-arc LOOP` cancelled cleanly and restarted with the loop flag; `STOP` halted the runner immediately. Legacy `RUN 2` still works. `RUN nope` returned the expected `RUN failed: sequence-not-found`. `/status.json` exposed the `runseq` block with the expected fields. Closes servo-3a9.

---

## Test 24: Servo Calibration — Boot Visibility (servo-2n9)

**Prerequisite:** Flash `feat/servo-2n9-firmware-calibration` to one board. Open Serial Monitor at 115200 baud. Note the test device:

- Board IP: `___.___.___.___`  (boardId: `1` / `2` / `3`)
- Tester: ___________________
- Date: ___________

**Commands:**
1. Power-cycle the board (full reset, not just re-opening Serial Monitor — boot log prints once).
2. Watch Serial output during boot.

**Expected:**
- Existing `storage: boardId=N hasActive=… hasPrevious=…` line still appears.
- Followed by a new block:
  ```
  --- Servo Calibration ---
    S0: defaults    minUs=1000 maxUs=2000 offsetDeg=0 (uncalibrated)
    S1: defaults    minUs=1000 maxUs=2000 offsetDeg=0 (uncalibrated)
    S2: defaults    minUs=1000 maxUs=2000 offsetDeg=0 (uncalibrated)
  ```
  (or `calibrated  minUs=… maxUs=… offsetDeg=…` for any channel already calibrated on a prior run)

**Result:**
- [ ] Pass
- [ ] Fail — describe:

---

## Test 25: Servo Calibration — Serial Commands (servo-2n9)

**Prerequisite:** Test 24 passing.

### 25a — `CAL_GET` reports current state

**Command:** `CAL_GET`

**Expected:** Same three-line block from Test 24's boot output.

- [ ] Pass / [ ] Fail:

### 25b — `CAL_PULSE` finds safe mechanical limits (bypasses calibration)

**Important — leave a safety margin.** Find the value where the servo
*just* starts to bind/buzz/stall, then **back off by ~30-50μs**. Saving the
exact bind point means commands at the extremes (e.g. `S0 1800`) will hit
the mechanical stop and the position feedback goes erratic (motor draws
stall current). The discovered bind values minus a margin are your safe
`minUs` / `maxUs`.

**Commands:**
1. `CAL_PULSE 0 1500` → servo should move to mechanical center.
2. `CAL_PULSE 0 800` → toward one end; raise until just before it stalls/binds. Note that bind value.
3. **Back off ~30-50μs** from the bind value → record as **minUs**. Re-run `CAL_PULSE 0 <minUs>` to confirm it's smooth and reaches near the lower end of travel.
4. `CAL_PULSE 0 2200` → toward the other end; lower from 2600 until just before it stalls. Note that bind value.
5. **Back off ~30-50μs** from the bind value → record as **maxUs**. Confirm with `CAL_PULSE 0 <maxUs>`.

**Record:**
- S0: minUs = ________  maxUs = ________
- S1: minUs = ________  maxUs = ________
- S2: minUs = ________  maxUs = ________

- [ ] Pass / [ ] Fail:

### 25c — `CAL_PULSE` rejects out-of-range microseconds

**Commands:**
- `CAL_PULSE 0 300` (below 400) → expect usage hint, no motion.
- `CAL_PULSE 0 3000` (above 2600) → same.

- [ ] Pass / [ ] Fail:

### 25d — `CAL_SET` persists + applies live

**Commands:**
1. `CAL_SET 0 <minUs> <maxUs> 0` (use values from 25b).
2. `CAL_GET` — S0 row should now say `calibrated  minUs=<minUs> maxUs=<maxUs> offsetDeg=0`.

- [ ] Pass / [ ] Fail:

### 25e — Calibrated `S0` hits the recorded limits

**Note on command magnitude:** channels 0/1/2 are 5-turn winches configured
in `applyCustomServoSetup()` with `totalDegrees=1800`. So `S<n> <deg>` sweeps
the full calibrated pulse range only when `<deg>` reaches `totalDegrees` —
**not** at 180. Send `1800` for the upper limit. (Standard 0-180 servos
would use the smaller numbers; if you ever change a channel to
`totalDegrees=180`, use those commands instead.) The motion editor's
percentage UX is tracked separately as `servo-6oa`.

**Commands (right after 25d, no reboot):**
- `S0 0` → moves to the lower mechanical limit you found, NOT the original default range.
- `S0 1800` → moves to upper limit (full sweep across the winch's 5-turn range).
- `S0 900` → physically centered (half of `totalDegrees=1800`).

Alternative for winches — use the existing percentage commands that already
go through `percentToDegrees()` against each servo's calibrated up/down
range:
- `DOWN 100` → full down position.
- `DOWN 0` → full up position.
- `DOWN 50` → midpoint.

- [ ] Pass / [ ] Fail:

### 25f — `offsetDeg` trim shifts position

**Commands:**
1. `CAL_SET 0 <minUs> <maxUs> -10`
2. `S0 90` → should sit ~10° away from where it sat at 90° in 25e.
3. Restore: `CAL_SET 0 <minUs> <maxUs> 0`

- [ ] Pass / [ ] Fail:

### 25g — `CAL_RESET` clears flag, effective after reboot

**Commands:**
1. `CAL_RESET 0` → Serial says `(takes effect after reboot — flag cleared but live servoConfig unchanged)`.
2. `CAL_GET` → S0 row should now say `defaults` (flag bit drives the label).
3. Power-cycle, then `CAL_GET` again → still `defaults`.

- [ ] Pass / [ ] Fail:

### 25h — Legacy `CAL <servo> <min> <max>` still works (RAM-only ticks)

**Command:** `CAL 0 200 500`

**Expected:** Serial echoes `Servo 0 calibration: 200 - 500`. Reboot clears it.

- [ ] Pass / [ ] Fail:

---

## Test 26: Servo Calibration — HTTP API (servo-2n9)

**Prerequisite:** Test 25 passing. Run curl from your laptop.

### 26a — `GET /calibration` returns 3-channel JSON

**Command:**
```bash
curl -sS http://<board-ip>/calibration | jq
```

**Expected:** `{"ok":true,"boardId":N,"servos":[{ch:0,minUs:…,maxUs:…,offsetDeg:…,calibrated:…}, {ch:1,…}, {ch:2,…}]}` — exactly 3 channels, fields match the boot log.

- [ ] Pass / [ ] Fail:

### 26b — `POST /calibration` persists and applies live

**Command:**
```bash
curl -X POST http://<board-ip>/calibration \
  -H 'Content-Type: application/json' \
  -d '{"ch":0,"minUs":950,"maxUs":2050,"offsetDeg":-2}'
```

**Expected:** `{"ok":true,"ch":0,"minUs":950,"maxUs":2050,"offsetDeg":-2,"calibrated":true}`.
Then send `S0 90` via the controller UI's free-text terminal to that board → servo moves to a position ~2° offset from before.

- [ ] Pass / [ ] Fail:

### 26c — Calibration survives reboot

**Commands:**
1. Power-cycle the board.
2. Boot log line for S0 should now say `calibrated  minUs=950 maxUs=2050 offsetDeg=-2`.
3. `curl -sS http://<board-ip>/calibration | jq '.servos[0]'` → same values.

- [ ] Pass / [ ] Fail:

### 26d — POST validation rejects bad payloads

**Each of these should return HTTP 400 with `{"ok":false,"error":"invalid-fields"}`:**

- [ ] `'{"ch":5,"minUs":1000,"maxUs":2000}'` (ch out of range)
- [ ] `'{"ch":0,"minUs":2000,"maxUs":1000}'` (min ≥ max)
- [ ] `'{"ch":0,"minUs":300,"maxUs":2000}'` (min < 400)
- [ ] `'{"ch":0,"minUs":1000,"maxUs":3000}'` (max > 2600)
- [ ] `'{"ch":0,"minUs":1000,"maxUs":2000,"offsetDeg":200}'` (offset > int8)

Fail describe:

### 26e — OPTIONS preflight returns CORS headers

**Command:**
```bash
curl -i -X OPTIONS http://<board-ip>/calibration \
  -H 'Origin: http://localhost' \
  -H 'Access-Control-Request-Method: POST'
```

**Expected:**
```
HTTP/1.1 204 No Content
Access-Control-Allow-Origin: *
Access-Control-Allow-Methods: POST, GET, OPTIONS
Access-Control-Allow-Headers: Content-Type
```

- [ ] Pass / [ ] Fail:

---

## Test 27: Servo Calibration — Integration with S/Sequence/Motion (servo-2n9)

**Prerequisite:** S0 calibrated via Tests 25-26.

### 27a — Calibrated `S<n>` hits true mechanical limits silently

**Commands** (use `totalDegrees` value for full sweep — 1800 for 5-turn winches in this project; see Test 25e note):
- `S0 0` → servo travels to lower limit, no stall sounds.
- `S0 1800` → upper limit, no stall sounds.
- Compare: `S1 0` / `S1 1800` on an uncalibrated channel. Should overshoot or stall (audible).

- [ ] Pass / [ ] Fail:

### 27b — Existing baked sequence still plays

Pick a sequence already in your library (e.g. PLAY 7). Play it via // 04 Sequencer against this board.

**Expected:** Sequence runs as before. Any inner `S<n>` commands now respect calibration.

- [ ] Pass / [ ] Fail:

### 27c — Motion editor playback honors calibration

**Caveat:** until `servo-6oa` ships, the motion editor uses 0-180° on the
value axis, so on a 5-turn winch (`totalDegrees=1800`) a keyframe at value
180 only sweeps ~10% of the calibrated pulse range. This sub-test still
verifies the calibration **IS** being applied at the firmware layer — just
to a small visible range. Once `servo-6oa` switches the editor to
percentage values, this test should be rewritten to use 0-100%.

In // 06 Motion editor, author a tiny motion: B<this-board>.S0 keyframes
at `atMs=0 value=0` and `atMs=2000 value=180`. Click Play Live.

**Expected:** Servo sweeps the lower ~10% of its calibrated range over 2s
(NOT the full sweep — see caveat above). An uncalibrated channel using
the same motion still produces noticeably different physical position
than the calibrated one — that's the signal calibration is plumbed in.

- [ ] Pass / [ ] Fail:

---

## Test 28: Servo Calibration — Regression (servo-2n9)

### 28a — Bake to all boards still succeeds

In // 03 Sequencer Bake, hit "Bake to boards". Should land in ≤30s per board (no slot layout change).

- [ ] Pass / [ ] Fail:

### 28b — `GET /sequences/info` shape unchanged

```bash
curl -sS http://<board-ip>/sequences/info | jq
```

**Expected:** Same fields as before this PR (`ok`, `boardId`, `hasActive`, `hasPrevious`, `bytesUsed`, `slotPayloadMax`).

- [ ] Pass / [ ] Fail:

### 28c — No JavaScript errors in controller HTML console

Open DevTools console, reload `servo_controller.html`, navigate through // 01..// 07.

**Expected:** Console clean throughout.

- [ ] Pass / [ ] Fail:

### 28d — Calibration survives unrelated power-cycle

Calibrate S0, do unrelated operations (bake, play a motion), then power-cycle. Confirm calibration persists.

- [ ] Pass / [ ] Fail:

---

## Test 29: Servo Calibration — Stress / Sanity (servo-2n9)

### 29a — Rapid alternating `CAL_SET` doesn't lock dataflash

Send 10 alternating `CAL_SET 0 1000 2000 0` and `CAL_SET 0 950 2050 -3` back-to-back via Serial.

**Expected:** All succeed; no EEPROM errors; `CAL_GET` reports the last write.

- [ ] Pass / [ ] Fail:

### 29b — OTA budget intact

```bash
make -C test size
```

**Expected:** Headroom remains > +1500 bytes (PR baseline was +2584).

Recorded headroom: `__________` bytes  → [ ] OK  /  [ ] Concerning

---

## Test 30: Servo Calibration — Per-board Replication (servo-2n9)

Once Tests 24-29 pass on the first board, run them in abbreviated form on the others.

| Board IP / boardId | Calibration done | Notes |
|--------------------|------------------|-------|
| 192.168.8.____ / `1` | [ ] | |
| 192.168.8.____ / `2` | [ ] | |
| 192.168.8.____ / `3` | [ ] | |

---

## Host Regression Tests

**2026-05-23:**
- [x] `make -C test` — 7/7 time multiplier
- [x] `make -C test motion` — 4/4 motion engine
- [x] `make -C test storage` — 22/22 storage / CRC / bake validation

**2026-05-24 (servo-3a9):**
- [x] `make -C test sequence` — 8/8 sequence engine
- [x] `make -C test size` — 116664 / 120831 (+4167 headroom; post-cleanup)

**2026-05-25 (servo-iga browser Motion editor):**
- [x] `node -e "new Function(...inline script...)"` — `servo_controller.html` inline script syntax OK
- [x] `make -C test` — 7/7 time multiplier
- [x] `make -C test motion` — 6/6 motion engine
- [x] `make -C test storage` — 22/22 storage / CRC / bake validation
- [ ] Browser Playwright smoke — blocked in this environment: bundled wrapper fetched through npm path, then exited with `sh: playwright-cli: command not found`

---

## Servo Calibration Notes

| Channel | Servo Model | Type | Stop Pulse | Calibration |
|---------|-------------|------|------------|-------------|
| 0 | goBILDA 2000 Series 25-2 | Standard, 5-turn | N/A | `CAL 0 110 480` |
| 1 | goBILDA 2000 Series 25-2 | Standard, 5-turn | N/A | `CAL 1 110 480` |
| 2 | goBILDA 2000 Series 25-2 | Standard, 5-turn | N/A | `CAL 2 110 480` |
| 3 | goBILDA 2000 Series 25-3 | Continuous | 298 | `CAL 3 186 410`, configured continuous in `servo_setup.h` |
