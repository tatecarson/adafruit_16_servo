# Adafruit 16-Channel Servo Calibration & Control

Interactive Serial interface for the Adafruit PCA9685 16-channel PWM/Servo driver board.

## Hardware

- [Adafruit 16-Channel PWM/Servo Driver](http://www.adafruit.com/products/815)
- Arduino (Uno, Nano, Mega, etc.)
- Standard analog servos

## Wiring

| PCA9685 | Arduino |
|---------|---------|
| VCC     | 5V      |
| GND     | GND     |
| SDA     | A4 (SDA)|
| SCL     | A5 (SCL)|
| V+      | External 5-6V servo power |

## Installation

1. Install the [Adafruit PWM Servo Driver Library](https://github.com/adafruit/Adafruit-PWM-Servo-Driver-Library) via Arduino Library Manager
2. Edit `servo_setup.h` to match your servos (min/max pulses, standard vs continuous, stop pulse)
3. Upload `adafruit_16_servo.ino` to your Arduino
4. Open Serial Monitor at 9600 baud

## Commands

| Command | Example | Description |
|---------|---------|-------------|
| `S<n> <deg>` | `S0 90` | Move servo n to degrees (0-180) |
| `UP <n> <pct>` | `UP 0 80` | Move servo n to absolute percent up |
| `DOWN <n> <pct>` | `DOWN 0 30` | Move servo n to absolute percent down |
| `ALLUP <pct> [ms]` | `ALLUP 100 3000` | Move all protected winch servos up together |
| `ALLDOWN <pct> [ms]` | `ALLDOWN 25` | Move all protected winch servos down together |
| `RIG <UP\|DOWN> <pct> <spd> [ms]` | `RIG UP 80 35 3000` | Manual test: move all protected winches and set rotation speed together |
| `P<n> <pulse>` | `P0 375` | Move servo n to raw pulse value |
| `TPULSE <pulse>` | `TPULSE 320` | Set servos 0-2 to the same raw pulse for comparison |
| `CAL <n> <min> <max>` | `CAL 0 160 580` | Set min/max pulse calibration |
| `SWEEP <n>` | `SWEEP 0` | Test sweep through full range |
| `OFF <n>` | `OFF 0` | Stop sending PWM signal; blocked on protected winches |
| `RELEASE <n>` | `RELEASE 0` | Force-release a servo by stopping PWM |
| `STATUS` | `STATUS` | Show all servo calibrations |
| `HELP` | `HELP` | Show available commands |

## Animation Commands

| Command | Example | Description |
|---------|---------|-------------|
| `MOVE <n> <deg> <ms>` | `MOVE 0 180 2000` | Smooth animated move with easing |
| `UMOVE <n> <pct> <ms>` | `UMOVE 0 80 3000` | Smooth animated move to absolute percent up |
| `DMOVE <n> <pct> <ms>` | `DMOVE 0 30 3000` | Smooth animated move to absolute percent down |
| `MOTION <id>` | `MOTION tidal-drift` | Play a browser-baked Motion from EEPROM |
| `RUN <id> [LOOP]` | `RUN evening-arc` | Run a browser-baked Sequence by id (schema v1) |
| `RUN AUTO` | `RUN AUTO` | Run the active baked Setlist forever (schema v1). The leader board (`schedulerConfig.leaderBoardId`) schedules entries — ordered or weighted shuffle honoring `minGapEntries`, per-entry `repeat`/`gapMs` — and mirrors `RUN`/`STOP` to followers. `STOP` halts it. (`avoidSameTag`/`moodArc` are schema-v2, not yet honored.) |
| `GALLERY [ON\|OFF]` | `GALLERY ON` | Get or set the persistent gallery-mode boot flag. When on, the board auto-runs the active Setlist after the boot grace period. |
| `STOP` | `STOP` | Stop all active motion and sequences |
| `STOP <n>` | `STOP 0` | Stop and hold one servo at its current position |
| `MODE <n> STD\|CONT` | `MODE 2 CONT` | Set servo to standard or continuous |
| `ROTATE <spd>` | `ROTATE 50` | Set installation rotation speed (-100 to 100) |

### Browser-Baked Motions

`MOTION <id>` plays a Motion from the active EEPROM bake uploaded through the browser `/sequences` endpoint.

```text
MOTION tidal-drift
STOP
```

- Motion ids come from `docs/sequencer-schema.md` / `servo_controller.html`
- Servo tracks interpolate linearly between absolute percent-down keyframes and map through each channel's calibrated travel before writing PCA9685 pulses every loop tick
- DC tracks interpolate signed speed values from `-100` to `100`
- Each board executes only its local sliced tracks; if a baked track still has `boardId`, mismatched boards skip it
- Manual servo/DC commands, `RUN <id>`, `MOTION`, or `STOP` cancel the active Motion cleanly
- For the current goBILDA 5-turn winches, see `docs/sequencer-schema.md` for measured slew-rate limits: full-range moves physically bottom out around `7.7s`, while very slow full-range moves can become visibly staccato.

### Browser Sequence Editor

Serve `servo_controller.html` through the local helper to author schema v1 Sequences in a shared project-folder library before exporting or baking:

```bash
python3 servo_library_server.py
```

Then open `http://127.0.0.1:4173/servo_controller.html`. The helper serves the page and lets any browser read/write `library.json` in this project folder. If the helper is not running, the page falls back to browser `localStorage`.

- Section `// 04 Sequencer` creates, duplicates, renames, deletes, tags, and saves Sequences in the shared library
- Steps support free-text commands, duration, target board, private labels, hold points, drag reorder, preview, scrub, looped playback, and pause/resume
- Record mode appends commands sent through the free-form terminal and fills each new step's duration from elapsed time
- Section `// 03 Sequencer Bake` still imports, exports, slices, and POSTs the same library to the boards
- **Pull from Boards** reads the baked library back from every reachable board (`GET /sequences`) and rebuilds the editor library: motion tracks are unioned by board, and the full-library fields (sequences/setlists/active/scheduler) must match across boards or the operator is asked to pick a source-of-truth board. Use it to recover the library on a fresh machine, after a branch swap, or after a cleared cache — it reflects what's physically baked on the boards. It never overwrites a non-empty local library without confirmation.
- The masthead **Gallery** toggle reads `/status.json` `gallery`, confirms before enabling unattended boot behavior, and sends `GALLERY ON/OFF` to every currently reachable board.

### Compile Firmware for Browser OTA

To avoid exporting a `.bin` from the Arduino IDE by hand:

```bash
./compile-firmware.sh --serve
```

Then open `http://127.0.0.1:4173/servo_controller.html`, go to `// 07 Firmware Upload`, click **Use compiled bin**, enter the OTA password, and upload to one board or all boards. The script writes `firmware/adafruit_16_servo.ino.bin` and `firmware/manifest.json`; those generated artifacts are ignored by git.


## Percent-of-Travel Commands

For winches or other long-travel servos, percentage commands are often easier than working in raw degrees.

```text
UP 0 80
DOWN 1 25
UMOVE 2 90 3000
DMOVE 2 10 3000
ALLUP 100 3000
ALLDOWN 20
```

- `0` = minimum calibrated position
- `100` = maximum calibrated position
- Intermediate percentages are mapped linearly across the servo's configured `totalDegrees`

For winches, the more explicit commands are usually better:

- `DOWN <n> <pct>` treats `0` as fully up/retracted and `100` as fully down/lowered
- `UP <n> <pct>` treats `0` as fully down/lowered and `100` as fully up/retracted
- `UMOVE` and `DMOVE` are the animated versions of those absolute commands

Examples:

```text
UP 0 30    # servo 0 to 30% up
UP 0 80    # servo 0 further up, not relative
DOWN 1 60  # servo 1 to 60% down
ALLUP 100 3000   # all protected winches move fully up together over 3s
ALLDOWN 25       # all protected winches move to 25% down
RIG UP 80 35 3000    # winches go 80% up while rotation ramps to 35%
RIG DOWN 20 -25      # winches go 20% down while rotation immediately runs reverse
```

`RIG` is a manual integration-testing command. It targets all protected non-continuous winches plus the first configured continuous servo, for live testing outside of baked Motion/Sequence playback.

`RIG` parameters:

- `UP` or `DOWN` chooses whether the shared winch target is interpreted as percent up or percent down
- `<pct>` is the shared winch target position from `0` to `100`
- `<spd>` is the rotation servo target speed from `-100` to `100`
- `[ms]` is optional:
  if omitted, the winches jump to position immediately and the rotation servo jumps to the target speed immediately
  if provided, the winches move over that duration and the rotation servo ramps to the target speed over the same duration

## Release Safety

The configured winch servos are release-protected. For those channels, `OFF <n>` will not disable holding torque.

Use:

```text
OFF 0       # blocked on protected winches
RELEASE 0   # intentionally releases the winch
```

This avoids accidental drops when testing commands.

## Calibration Guide

Each servo has different physical limits. Calibration finds the safe pulse range.

1. **Test the default range**
   ```
   SWEEP 0
   ```
   Watch and listen - buzzing or straining means the pulse is past the physical limit.

2. **Find minimum position**
   ```
   P0 150
   ```
   Increase the value if the servo buzzes (e.g., `P0 160`, `P0 170`).

3. **Find maximum position**
   ```
   P0 600
   ```
   Decrease the value if the servo buzzes (e.g., `P0 580`, `P0 560`).

4. **Save calibration**
   ```
   CAL 0 160 580
   ```

5. **Verify**
   ```
   SWEEP 0
   ```
   Should move smoothly without straining.

6. Repeat for each servo.

## Default Values

| Parameter | Value | Description |
|-----------|-------|-------------|
| `DEFAULT_MIN` | 150 | Minimum pulse (0°) |
| `DEFAULT_MAX` | 600 | Maximum pulse (180°) |
| `SERVO_FREQ` | 50 Hz | Standard servo frequency |
| `NUM_SERVOS` | 16 | Total servo channels |

## Continuous Servo Mode

Continuous rotation servos spin instead of moving to positions. The PWM signal controls speed and direction rather than angle.

1. **Set servo to continuous mode**
   ```
   MODE 0 CONT
   ```

2. **Control installation rotation** (-100 to 100, 0 = stop)
   ```
   ROTATE 50      # 50% speed forward
   ROTATE -50     # 50% speed reverse
   ROTATE 0       # Stop
   ```

3. **Stop the servo**
   ```
   ROTATE 0       # Stop the primary continuous rotation servo
   STOP           # Stops all continuous servos
   ```

4. **Calibration for continuous servos**
   - `CAL` sets the speed range (min = full reverse, max = full forward)
   - The stop pulse is auto-calculated as the midpoint
   - Use `P<n> <pulse>` to find the exact stop point, then adjust calibration

5. **Switch back to standard mode**
   ```
   MODE 0 STD
   ```

## Notes

- Calibration is stored in RAM and resets on power cycle
- Pulse values are out of 4096 (12-bit resolution)
- Typical servo range: 150-600 pulse counts (adjust per servo)
- Always use external power for servos (not USB power)
- Sequences are stored in PROGMEM (flash) to conserve RAM
- Serial input uses a fixed 50-byte buffer for long-term stability
