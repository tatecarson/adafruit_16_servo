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
2. Upload `adafruit_16_servo.ino` to your Arduino
3. Open Serial Monitor at 9600 baud

## Commands

| Command | Example | Description |
|---------|---------|-------------|
| `S<n> <deg>` | `S0 90` | Move servo n to degrees (0-180) |
| `P<n> <pulse>` | `P0 375` | Move servo n to raw pulse value |
| `CAL <n> <min> <max>` | `CAL 0 160 580` | Set min/max pulse calibration |
| `SWEEP <n>` | `SWEEP 0` | Test sweep through full range |
| `CENTER <n>` | `CENTER 0` | Move servo to center (90°) |
| `OFF <n>` | `OFF 0` | Stop sending PWM signal |
| `STATUS` | `STATUS` | Show all servo calibrations |
| `HELP` | `HELP` | Show available commands |

## Animation Commands

| Command | Example | Description |
|---------|---------|-------------|
| `MOVE <n> <deg> <ms>` | `MOVE 0 180 2000` | Smooth animated move with easing |
| `WAVE <s> <e> [spd] [off] [amp]` | `WAVE 0 7 50 30 90` | Start sine wave pattern |
| `PLAY <n> [LOOP]` | `PLAY 1 LOOP` | Play keyframe sequence |
| `STOP` | `STOP` | Stop wave or sequence |
| `MODE <n> STD\|CONT` | `MODE 0 CONT` | Set servo to standard or continuous |
| `SPEED <n> <spd>` | `SPEED 0 50` | Set continuous servo speed (-100 to 100) |

### Wave Parameters

- `s`, `e`: Start and end servo numbers
- `spd`: Speed (ms per cycle, default 50)
- `off`: Phase offset between servos in degrees (default 30)
- `amp`: Amplitude in degrees (default 90)

### Creating Custom Sequences

Edit the `sequence1` array in the code to define your own keyframe animations:

```cpp
Keyframe sequence1[MAX_KEYFRAMES] = {
  {servo, degrees, time_ms, duration_ms},
  // ...
  {255, 0, 0, 0}  // End marker
};
```

- `servo`: Which servo (0-15)
- `degrees`: Target position (0-180)
- `time_ms`: When to start this move (ms from sequence start)
- `duration_ms`: How long the move takes

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
