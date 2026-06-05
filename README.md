# Servo Cluster — Kinetic Installation Control

Firmware and a browser control surface for a multi-board kinetic installation. Each
**Arduino UNO R4 WiFi** drives a PCA9685 16-channel PWM board (winch servos) plus a
DC gear motor, and the boards run as a WiFi cluster that plays browser-authored
motion timelines in sync.

The system has two halves:

- **Firmware** (`adafruit_16_servo/`) — runs on each board. Drives servos and the DC
  motor, parses commands over Serial and HTTP, stores baked content in EEPROM, and
  keeps the cluster in step over UDP.
- **Browser dashboard** (`servo_controller.html`) — a single-file web app that
  monitors the boards, authors Motions/Sequences/Setlists, bakes them to the boards,
  calibrates servos, and pushes firmware over the air.

For the content data model (Motion/Sequence/Setlist JSON, interpolation rules, slew
limits), see [`docs/sequencer-schema.md`](docs/sequencer-schema.md).

## Hardware

- **Arduino UNO R4 WiFi** — one per board (the cluster is built around three).
- **[Adafruit PCA9685](http://www.adafruit.com/products/815)** 16-channel PWM/servo driver (I2C).
- **goBILDA 2000-series 5-turn winch servos** on channels 0–2 (configured in `servo_setup.h`).
- **DC gear motor** on a dual-PWM driver (`RPWM` = pin 10, `LPWM` = pin 11) for installation rotation.
- External 5–6 V supply for the servos and 12V/10A for the motor.

## Setup

1. **Credentials.** Copy `adafruit_16_servo/Secrets.h.example` to
   `adafruit_16_servo/Secrets.h` and fill in your WiFi SSID/password and an OTA
   password. `Secrets.h` is gitignored, so real credentials never get committed.
2. **Per-channel servo config.** Edit `adafruit_16_servo/servo_setup.h` for your
   mechanism — travel (`totalDegrees`/`downDegrees`), direction (`reverseDir`), and
   pulse limits. The shipped config is three 5-turn winches at 1800° travel.
3. **Flash the first time over USB:**
   ```bash
   arduino-cli compile --fqbn arduino:renesas_uno:unor4wifi adafruit_16_servo
   arduino-cli upload  --fqbn arduino:renesas_uno:unor4wifi -p /dev/<port> adafruit_16_servo
   ```
   Or use the Arduino IDE. After the first flash, update boards over the air (see
   [Firmware updates](#firmware-updates-ota)).
4. **Open the dashboard.** Serve the page from the project folder so authored content
   persists to `library.json`:
   ```bash
   python3 servo_library_server.py
   ```
   Then open <http://127.0.0.1:4173/servo_controller.html>. Without the helper the
   page still runs, falling back to browser `localStorage`.

On boot each board joins WiFi, prints its IP over Serial (115200 baud), serves its
HTTP API, and announces itself to the cluster over UDP.

## Browser dashboard

The page is organized into numbered sections:

- **`// 01 Master Command`** — free-text command terminal and a global `STOP`. Commands
  go to one board over HTTP and mirror to the rest over UDP.
- **`// 02 Board Telemetry`** — live cards per board: online state, servo positions, DC
  motor speed, and what's currently playing.
- **`// 03 Sequencer Bake`** — import/export the library, slice it per board, and POST
  it to the boards' EEPROM. **Pull from Boards** rebuilds the editor library from
  what's physically baked.
- **`// 04 Sequencer`** — author Sequences: ordered steps (each a command + duration +
  target board), drag-reorder, hold points, record mode, preview, scrub, and looped
  playback.
- **`// 05 Setlist`** — group Sequences into playlists for unattended playback, with an
  ordered or weighted-shuffle scheduler and a **Simulate hour** preview that
  fast-forwards an hour of scheduling in the browser.
- **`// 06 Motion`** — the keyframe motion editor: per-channel servo and DC tracks,
  draggable keyframes, marquee + group selection, shape curves, and slew-feasibility
  warnings. Play live (browser-streamed) or fire the baked motion (on-device).
- **`// 07 Firmware Upload`** — OTA flash one board or all of them (see below).

## Content model: Motions, Sequences, Setlists

- A **Motion** is a keyframed timeline of servo (percent-of-travel) and DC (signed
  speed) tracks. Authored in `// 06`, baked to EEPROM, played by `MOTION <id>`.
- A **Sequence** is an ordered list of command steps with durations. Played by
  `RUN <id> [LOOP]`.
- A **Setlist** is a playlist of Sequences with a scheduler (ordered or weighted
  shuffle, `minGapEntries`, per-entry `repeat`/`gapMs`). Played by `RUN AUTO`.
- **Gallery mode** (`GALLERY ON`) makes a board auto-run the active Setlist after a
  boot grace period, for unattended exhibition.

Each board stores and plays only its own sliced tracks. The full schema, interpolation
rules, and measured winch slew limits live in
[`docs/sequencer-schema.md`](docs/sequencer-schema.md).

## Cluster sync

Boards coordinate over UDP (port 4210) with no shared clock:

- Each board broadcasts a **heartbeat** every second; peers track liveness and uptime
  (visible at `/peers.json`).
- A command sent to one board is **mirrored** to peers, so `RUN`/`STOP`/etc. apply
  cluster-wide.
- **Synchronized Motion start** is relative: the originator unicasts "begin Motion
  `<id>` in `<leadMs>` ms" to each peer, and every board arms the Motion on its own
  `millis()` clock — so starts line up without clock synchronization.
- For `RUN AUTO`, only the configured leader board schedules; it mirrors `RUN`/`STOP`
  to the followers.

## Command reference

Commands work over Serial (115200 baud) and over HTTP via
`GET /cmd?c=<command>` (the receiving board mirrors them to the cluster).

### Servo + motor

| Command | Example | Description |
|---------|---------|-------------|
| `S<n> <deg>` | `S0 90` | Move servo n to a degree position |
| `P<n> <pulse>` | `P0 375` | Move servo n to a raw pulse (testing/calibration) |
| `UP <n> <pct>` | `UP 0 80` | Move servo n to an absolute percent "up" |
| `DOWN <n> <pct>` | `DOWN 0 30` | Move servo n to an absolute percent "down" |
| `UMOVE <n> <pct> <ms>` | `UMOVE 0 80 3000` | Animated `UP` over a duration |
| `DMOVE <n> <pct> <ms>` | `DMOVE 0 30 3000` | Animated `DOWN` over a duration |
| `ROTATE <spd>` | `ROTATE 50` | Set DC motor speed, −100…100 (0 = stop) |
| `SWEEP <n>` | `SWEEP 0` | Sweep servo n through its range |
| `TPULSE <pulse>` | `TPULSE 320` | Set servos 0–2 to the same raw pulse for comparison |
| `STOP` / `STOP <n>` | `STOP 0` | Stop all motion, or hold one servo in place |

### Playback

| Command | Example | Description |
|---------|---------|-------------|
| `MOTION <id>` | `MOTION tidal-drift` | Play a baked Motion from EEPROM |
| `RUN <id> [LOOP]` | `RUN evening-arc LOOP` | Run a baked Sequence (optionally looping) |
| `RUN AUTO` | `RUN AUTO` | Run the active Setlist forever (leader schedules, followers mirror) |
| `GALLERY [ON\|OFF]` | `GALLERY ON` | Get/set the persistent gallery-mode boot flag |

### Calibration + status

| Command | Example | Description |
|---------|---------|-------------|
| `CAL <n> <min> <max>` | `CAL 0 160 580` | Set pulse limits in RAM (lost on reboot) |
| `CAL_GET` | `CAL_GET` | Print persisted calibration for all channels |
| `CAL_SET <n> <minUs> <maxUs> [<offsetDeg>]` | `CAL_SET 0 600 2400 3` | Persist calibration + angle trim to EEPROM |
| `CAL_RESET <n>` | `CAL_RESET 0` | Restore channel n to defaults |
| `CAL_PULSE <n> <us>` | `CAL_PULSE 0 1500` | Drive a raw microsecond pulse to find limits live |
| `STATUS` | `STATUS` | Print servo calibrations and DC motor state |
| `STORAGEINFO` | `STORAGEINFO` | Show baked-storage / board-id info |
| `BOARDID [n]` | `BOARDID 2` | Read or set this board's cluster id |
| `HELP` | `HELP` | List commands |

## Calibration

Each channel maps a 0–100% travel range onto a calibrated pulse window, with an
optional `offsetDeg` trim. Calibration **persists in EEPROM** across power cycles.

Find a channel's limits, then persist them:

1. `SWEEP 0` — watch and listen; buzzing or straining means a pulse is past the
   physical limit.
2. `CAL_PULSE 0 600` / `CAL_PULSE 0 2400` — nudge the raw microsecond endpoints until
   the servo reaches its travel without straining.
3. `CAL_SET 0 600 2400` — persist the window (add a fourth value to trim the angle,
   e.g. `CAL_SET 0 600 2400 3` if the horn reads 3° low).
4. `CAL_GET` — verify, and `CAL_RESET 0` to start over.

`CAL` still sets limits for the current session only; use `CAL_SET` to keep them.

## Firmware updates (OTA)

After the first USB flash, update boards wirelessly:

- **From the browser:** `./compile-firmware.sh --serve`, open the dashboard, go to
  `// 07 Firmware Upload`, click **Use compiled bin**, enter the OTA password, and
  upload to one board or all. The script writes `firmware/adafruit_16_servo.ino.bin`
  and `firmware/manifest.json` (both gitignored).
- **From the command line:** `./ota-all.sh` flashes every board listed in the script.
  The OTA password comes from the environment or a local `Secrets.h`; it is never
  baked into the script.

The build is held under the UNO R4 WiFi's 122,880-byte OTA partition cap.

## HTTP API (per board)

| Method | Path | Purpose |
|--------|------|---------|
| GET | `/status.json` | Telemetry: servos, DC motor, active Motion/Sequence, gallery flag, OTA status |
| GET | `/cmd?c=<command>` | Run a command locally and mirror it to peers |
| GET/POST | `/boardId` | Read or set the board's cluster id |
| GET | `/sequences` | Stream the board's baked library |
| GET | `/sequences/info` | Baked-library metadata |
| POST | `/sequences` | Bake a (sliced) library to EEPROM |
| POST | `/sequences/restore` | Restore the previous baked library |
| GET | `/peers.json` | Cluster peers and their uptimes |
| POST | `/ota` | Firmware upload |

## Project layout

```
adafruit_16_servo/      Firmware (one sketch, modular headers)
  adafruit_16_servo.ino   Setup/loop, Serial + HTTP dispatch, status.json
  command_interface.h     Command parser
  servo_control.h         Servo motion + animation
  dc_motor.h              DC motor output
  servo_calibration.h     EEPROM-persisted calibration
  motion_engine.h         Motion playback
  sequence_engine.h       Sequence playback
  setlist_scheduler.h     RUN AUTO scheduler
  gallery_mode.h          Unattended boot autoplay
  Sync.cpp / Sync.h       UDP cluster sync
  Web.cpp / Web.h         HTTP server + OTA
  storage.h               EEPROM bake storage
  servo_setup.h           Per-channel hardware config
  Secrets.h.example       WiFi + OTA credential template
servo_controller.html   Browser dashboard (single file)
servo_library_server.py Local helper: serves the page, persists library.json
compile-firmware.sh     Build the OTA bin (and optionally serve the page)
ota-all.sh              OTA-flash every board
docs/sequencer-schema.md  Content data model and interpolation rules
test/                   Host-side tests (C++ engines + JS editor logic)
```

## Tests

```bash
make -C test          # firmware engine tests + browser editor-logic tests
```

## Notes

- Pulse values are 12-bit (out of 4096); the servo PWM runs at 50 Hz.
- Calibration persists in EEPROM (`CAL_SET`); `CAL` alone is RAM-only.
- Serial input uses a fixed 50-byte command buffer.
- `NUM_SERVOS` is 16 (the channel count of the PCA9685); this installation wires three.
