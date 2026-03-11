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
| `L<n> <pct>` | `L0 10` | Move servo n to percent of total travel |
| `UP <n> <pct>` | `UP 0 80` | Move servo n to absolute percent up |
| `DOWN <n> <pct>` | `DOWN 0 30` | Move servo n to absolute percent down |
| `ALLUP <pct> [ms]` | `ALLUP 100 3000` | Move all protected winch servos up together |
| `ALLDOWN <pct> [ms]` | `ALLDOWN 25` | Move all protected winch servos down together |
| `RIG <UP\|DOWN> <pct> <spd> [ms]` | `RIG UP 80 35 3000` | Manual test: move all protected winches and set rotation speed together |
| `P<n> <pulse>` | `P0 375` | Move servo n to raw pulse value |
| `CAL <n> <min> <max>` | `CAL 0 160 580` | Set min/max pulse calibration |
| `SWEEP <n>` | `SWEEP 0` | Test sweep through full range |
| `CENTER <n>` | `CENTER 0` | Move servo to center (90°) |
| `OFF <n>` | `OFF 0` | Stop sending PWM signal; blocked on protected winches |
| `RELEASE <n>` | `RELEASE 0` | Force-release a servo by stopping PWM |
| `STATUS` | `STATUS` | Show all servo calibrations |
| `HELP` | `HELP` | Show available commands |

## Animation Commands

| Command | Example | Description |
|---------|---------|-------------|
| `MOVE <n> <deg> <ms>` | `MOVE 0 180 2000` | Smooth animated move with easing |
| `LMOVE <n> <pct> <ms>` | `LMOVE 0 90 3000` | Smooth animated move to percent of travel |
| `UMOVE <n> <pct> <ms>` | `UMOVE 0 80 3000` | Smooth animated move to absolute percent up |
| `DMOVE <n> <pct> <ms>` | `DMOVE 0 30 3000` | Smooth animated move to absolute percent down |
| `WAVE <s> <e> [spd] [off] [amp]` | `WAVE 0 7 50 30 90` | Start sine wave pattern |
| `PLAY <n> [LOOP]` | `PLAY 1 LOOP` | Play keyframe sequence |
| `SPLAY <n> [LOOP]` | `SPLAY 1 LOOP` | Play speed sequence (continuous servos) |
| `STOP` | `STOP` | Stop wave or sequence |
| `MODE <n> STD\|CONT` | `MODE 0 CONT` | Set servo to standard or continuous |
| `SPEED <n> <spd>` | `SPEED 0 50` | Set continuous servo speed (-100 to 100) |

### Wave Parameters

- `s`, `e`: Start and end servo numbers
- `spd`: Speed (ms per cycle, default 50)
- `off`: Phase offset between servos in degrees (default 30)
- `amp`: Amplitude in degrees (default 90)

### Creating Custom Sequences

Edit `sequence_setup.h` to define your own keyframe animations. Sequences are stored in PROGMEM (flash memory) to save RAM:

```cpp
const Keyframe sequence1[] PROGMEM = {
  {servo, degrees, time_ms, duration_ms},
  // ...
  {255, 0, end_time_ms, 0}  // End marker
};
```

- `servo`: Which servo (0-15)
- `degrees`: Target position (0-180)
- `time_ms`: When to start this move (ms from sequence start)
- `duration_ms`: How long the move takes

### Speed Sequences (Continuous Servos)

Speed sequences choreograph continuous servos with timed speed changes and ramping:

```
SPLAY 1        # Play speed sequence 1 once
SPLAY 1 LOOP   # Loop speed sequence 1
STOP           # Stop sequence
```

Edit `sequence_setup.h` to define speed sequences (also stored in PROGMEM):

```cpp
const SpeedFrame speedSeq1[] PROGMEM = {
  {servo, speed, time_ms, ramp_ms},
  // ...
  {255, 0, end_time_ms, 0}  // End marker
};
```

- `servo`: Which continuous servo (0-15)
- `speed`: Target speed (-100 to 100, 0 = stop)
- `time_ms`: When to start this speed change (ms from sequence start)
- `ramp_ms`: How long to ramp to the target speed (0 = instant)

## Percent-of-Travel Commands

For winches or other long-travel servos, percentage commands are often easier than working in raw degrees.

```
L0 10
L1 50
LMOVE 2 90 3000
UP 0 80
DOWN 1 25
UMOVE 2 90 3000
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

`RIG` is a manual integration-testing command. It targets all protected non-continuous winches plus the first configured continuous servo, and stays separate from `PLAY` / `SPLAY`.

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

2. **Control speed** (-100 to 100, 0 = stop)
   ```
   SPEED 0 50     # 50% speed forward
   SPEED 0 -50    # 50% speed reverse
   SPEED 0 0      # Stop
   ```

3. **Stop the servo**
   ```
   CENTER 0       # Same as SPEED 0 0
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
