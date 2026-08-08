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
- **DC gear motor** on an [IBT_2 (BTS7960)](https://www.handsontec.com/dataspecs/module/BTS7960%20Motor%20Driver.pdf) dual-PWM driver (`RPWM` = pin 10, `LPWM` = pin 11) for installation rotation.
- External 5–6 V supply for the servos and 12V/10A for the motor.

### Motor driver wiring (IBT_2 ↔ UNO R4 WiFi)

The firmware drives only `RPWM`/`LPWM` and leaves the enable pins alone, so
`R_EN` and `L_EN` must be jumpered to **5 V** or the driver stays disabled. The
motor supply ground and the Arduino ground **must be common**.

```
      Arduino UNO R4 WiFi                 IBT_2 (BTS7960)              12V/10A supply
     ┌────────────────────┐            ┌──────────────────┐          ┌─────────────┐
     │              pin 10 ├──────────► │ RPWM             │          │             │
     │              pin 11 ├──────────► │ LPWM        B+   ├──────────┤ +12V        │
     │                     │            │             B-   ├────┬─────┤ GND         │
     │                 5V  ├──────┬───► │ R_EN             │    │     └─────────────┘
     │                     │      ├───► │ L_EN             │    │
     │                     │      └───► │ VCC          M+  ├──┐ │
     │                 GND ├──────┬───► │ GND          M-  ├┐ │ │      ┌─────────────┐
     │                     │      │     │ R_IS  (n/c)      ││ └─┼────► │  +  DC gear │
     └────────────────────┘      │     │ L_IS  (n/c)      │└───┼────► │  -   motor  │
                                 │     └──────────────────┘    │      └─────────────┘
                                 └───────── common ground ─────┘
```

| IBT_2 pin      | Connects to                | Notes                                   |
| -------------- | -------------------------- | --------------------------------------- |
| `RPWM`         | Arduino pin 10             | Forward PWM (speed > 0)                 |
| `LPWM`         | Arduino pin 11             | Reverse PWM (speed < 0)                 |
| `R_EN`, `L_EN` | Arduino 5 V                | Tie high to enable the driver           |
| `VCC`          | Arduino 5 V                | Logic-side supply for the driver        |
| `GND`          | Arduino GND                | Must share ground with the 12 V supply  |
| `R_IS`, `L_IS` | not connected              | Current-alarm outputs, unused here      |
| `B+` / `B-`    | 12 V/10 A supply + / −      | Motor power input                       |
| `M+` / `M-`    | DC gear motor leads        | Motor output (swap to reverse spin)     |

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
   npm start
   ```
   Then open <http://127.0.0.1:4173/servo_controller.html>. No install step — the
   helper has no dependencies. Without it the page still runs, falling back to
   browser `localStorage`.

   The 3D simulator is served from the same origin at
   <http://127.0.0.1:4173/sculpture_3d.html>, which is what lets the dashboard drive
   it over a BroadcastChannel. Opened as bare files the two are same-origin in some
   browsers and not in others — Firefox gives every `file://` document its own opaque
   origin — so serve both if the link will not connect.

On boot each board joins WiFi, prints its IP over Serial (115200 baud), serves its
HTTP API, and announces itself to the cluster over UDP.

## Browser dashboard

The page is organized into numbered sections:

- **`// 01 Master Command`** — free-text command terminal and a global `STOP`. Commands
  go to one board over HTTP and mirror to the rest over UDP. **Motor Test** opens a
  slide-in drawer to spot-check hardware on one board without authoring a Motion:
  jog each winch servo to bounded positions (`UP <n> 0/50/80`), send all servos
  down/up at once, and drive the DC motor forward/reverse (`ROTATE ±30`). The
  manual Up controls stop at 80% to keep winches from binding at the absolute
  upper limit; the drawer intentionally has no full-range Sweep control.
  Positions are absolute, so "All Down" just holds any servo already down —
  nothing reverses. Live commands only — nothing is baked.
- **`// 02 Board Telemetry`** — live cards per board: online state, servo positions, DC
  motor speed, and what's currently playing.
- **`// 03 Sequencer Bake`** — import/export the library, slice it per board, and POST
  it to the boards' EEPROM. Use **Bake One** with the adjacent board picker when
  only one board is connected; the other configured boards are not contacted.
  **Pull from Boards** rebuilds the editor library from what's physically baked.
  Deployment strips editor-only metadata and default fields, validates every
  library reference, and stores compact firmware pre-roll timing instead of
  generated bridge Motions. Payloads through 4,080 bytes retain atomic rollback;
  the browser can use an explicitly warned 6,000-byte large mode when needed,
  but that tier cannot restore a previous bake. Replaying a baked Motion first
  glides slow winch servos back to keyframe 0 when the preceding Motion ended elsewhere.
- **`// 04 Sequencer`** — author Sequences: ordered steps (each a command + duration +
  target board), drag-reorder, hold points, record mode, preview, scrub, and looped
  playback. **Arrange ↗** opens a full-screen, zoomable DAW-style timeline whose
  time-proportional blocks can be dragged to reorder the same steps without replacing
  the detailed table editor. MOTION blocks show compact keyframe previews; the Motion
  Library strip previews every available Motion and can add one after the selected
  step or replace the selected step in place. Expanded MOTION previews show each
  track's start/end values, full range, and key count; Arrange blocks label their
  line endpoints for quick comparison.
- **`// 05 Setlist`** — group Sequences into playlists for unattended playback, with an
  ordered or weighted-shuffle scheduler and a **Simulate hour** preview that
  fast-forwards an hour of scheduling in the browser.
- **`// 06 Motion`** — the keyframe motion editor: per-channel servo and DC tracks,
  connected keyframe curves, marquee + group selection, shape curves, and
  slew-feasibility warnings. Dragging a keyframe shows its changing value directly
  beside the moving diamond. DC Motion values are safety-limited to −50…+50 in the
  editor, bake output, and firmware playback. Play live (browser-streamed) or fire
  the baked motion (on-device). Baked replay uses the same measured 77ms/%
  start-pose preparation as live replay, then issues one cluster-synchronized
  `MOTION` command.
- **`// 07 Firmware Upload`** — OTA flash one board or all of them (see below).

## 3D simulator

`sculpture_3d.html` puts all three machines in one room and drives them from exactly what
the dashboard can command: the winch servos as percent-down (0 = fully up, 100 = fully
down, unpowered rest) and the DC motors as signed speed. Open the file directly — it needs
no server — or serve it alongside the dashboard with `npm start`, at
<http://127.0.0.1:4173/sculpture_3d.html>. Serving both is what makes the dashboard
link work reliably, because a BroadcastChannel is scoped to an origin.
`three.js` loads from jsDelivr, so the page needs a network the first time it is opened;
`vendor/` is a leftover of an earlier vendored copy and is no longer used.

**The room.** The three boards stand on a 5 m triangle, far enough apart that walking up
to one clearly favours it and close enough that the middle of the room hears all three.

| Board | Machine | Drive |
|---|---|---|
| 1 | Centre wands — three servo-driven wands strike six tubes orbiting on a rim-belt ring | S0–S2 = wand throw, DC = ring speed |
| 2 | Field — bearings roll on a 300 mm tone wheel cut into five tuned bands | **DC motor only, no servos** |
| 3 | Dowel curtain — winched ring over a wooden deck | S0–S2 = winches, DC = ring |

Press **Walk** (or `V`) to stand in the room at head height: `WASD` to move, mouse to
look, shift to hurry, escape back to orbit. This is the point of the page — every voice is
panned where the thing making it actually is, and the listener rides your head, so what
you hear is a consequence of where you stand rather than a fixed mix. **Solo** and **mute**
are a gain on everything a machine makes, drones included.

Board 2 has no servo channels at all: its rake, ball count and cut width are build
parameters. Servo commands addressed to it report themselves rather than moving something
invisible, and it ignores the servo tracks a Motion carries for board 2.

Detail scales with distance — constraint iterations and substeps fall off as you walk
away — but **gain never does**. Distance belongs to the panners alone; anything that
quietened a far machine here would be counting distance twice.

`__selfTest()` in the console re-mounts board 3 with a pinned seed and runs a fixed
fast-forward, so a change to the solver can be checked by diffing numbers rather than
squinting at chains. It reports 1331 strikes at seed 7.

**Board 3's mechanism.** A ceiling housing holds the three winch servos and the DC gear
motor. Three cables run down and out to an inner ring; chains of wooden dowels linked by
small metal rings hang free from it, over a stationary base whose wooden deck is slightly
wider than that ring. A second, larger ring is fixed to the ceiling and carries its own
dowel curtain — it never moves, and it sits just *outside* the base, so its dowels hang
past the base wall and rest on the floor rather than on the deck. The base is modelled as
a solid cylinder for that reason: resolving it as a top surface alone would snap the
outer curtain up onto the deck. The winches set how much dowel piles onto the deck; the
motor turns the inner curtain only.

The two behaviours are consequences of the simulation, not separate modes:

- **slow** — the piled ends drag across the stationary wood and scrape.
- **fast** — centrifugal flare lifts them off the deck and throws them out against the
  outer ring's dowels, and they clack.

Rotation is calibrated against the real installation, where the inner curtain starts
tangling with the outer ring at about `ROTATE 22` — the practical ceiling. Full scale is
set so that lands on the threshold rather than being a speed nobody uses:

| `ROTATE` | rpm | s/rev | contact |
|---|---|---|---|
| 10–18 | 2.5–4.5 | 24–13 | clear |
| **22** | **5.5** | **10.9** | **42 clacks/s — starting to tangle** |
| 25–30 | 6.2–7.5 | 9.7–8.0 | 80–105 |
| 50 | 12.5 | 4.8 | 696 — unusable |

That threshold falls out of the ring gap, so it is only as right as the geometry. If
`ROTATE 22` looks too slow, the real gap between the two rings is wider than the modelled
7 cm — raise `Outer ring m`, which pushes the knee up, then raise `Max RPM @100` to put it
back on 22.

- The dowel chains are simulated, not posed — Verlet particles with distance constraints,
  frictional deck contact, and segment-to-segment collision between the two curtains
  (joint-only tests thread straight through the ~12 cm gaps between chains). Pile-up,
  buckling, drag, flare and swing all fall out of that.
- The deck accumulates the scrape marks the dowels leave, which is the pattern the piece
  draws; **Clear marks** resets it and `μ` sets deck friction.
- **Settle 5 s** runs five seconds of physics in one go and reports where it landed —
  clacks/s, scrape, ring height, joints on the deck. Flare takes seconds to develop, so
  this answers "does this height and speed clack, or just scrape?" without waiting.
- **Sound** is synthesised from those same contacts, nothing sampled or sequenced: a
  noise-based scrape voice driven by how fast the contacting dowels slide, and one
  wooden-bar resonance per strike. Above ~45 strikes/s the discrete voices give way to a
  clatter bed — at full speed the rig really does produce several hundred impacts a
  second, which is a roar rather than a sequence of taps. Audio needs one click to start
  (browser autoplay policy).
- Cable payout converts to ring height through the cable's real geometry, so percent-down
  is slightly non-linear in ring height, as on the rig. Differential winch values tilt the
  ring on the plane through its three anchors.
- The slew limiter uses the measured `77ms/%` mechanical floor, so a Motion asking for
  travel the winches cannot deliver visibly lags its commanded position.
- **Library playback** loads `library.json` and plays any Motion or Sequence, including
  `ROTATE`, `STOP`, and per-board DC lanes. Authoring form only — bake pre-rolls
  (`MOTION … PREP`) are not simulated.
- **Dashboard link** mirrors the dashboard itself. `servo_controller.html` publishes
  every command it dispatches on a same-origin `BroadcastChannel`, so opening both pages
  from the same server makes the 3D rig follow the real control surface — Motor Test,
  Master Command, live Motion play, `RUN` — **with no boards connected at all**, because
  the tap sits in front of the HTTP request rather than behind a reply. The simulator
  applies the firmware's own mirroring rule: `RUN`, `ROTATE`, bare `STOP` and `MOTION`
  are cluster-wide and always apply; `UP`/`DOWN`/`DMOVE`/`S<n>`/`STOP <n>` are
  board-local and apply only when addressed to the selected board.
- **Live telemetry** polls a board's `/status.json` at 2 Hz and mirrors its real pulses,
  inverted through the `servo_setup.h` calibration. Overrides the dashboard link.
- **Geometry** panel exposes every dimension. The mechanism is taken from the firmware;
  the dimensions are scaled off a reference render and are meant to be corrected against
  the real build.

Only board 3's rig exists so far. Boards 1 and 2 draw a placeholder until their designs
land — add them to the `RIGS` registry near the top of the module.

## Content model: Motions, Sequences, Setlists

- A **Motion** is a keyframed timeline of servo (percent-of-travel) and DC (signed
  speed) tracks. Authored in `// 06`, baked to EEPROM, played by `MOTION <id>`.
- A **Sequence** is an ordered list of command steps with durations. Played by
  `RUN <id> [LOOP]`. At bake time, unsafe Motion boundaries receive compact
  firmware-assisted pre-rolls without adding authored or stored steps.
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
| GET | `/sequences/info` | Baked-library size, storage mode, and rollback metadata |
| POST | `/sequences` | Bake a (sliced) library to EEPROM |
| POST | `/sequences/restore` | Restore the previous baked library (dual-slot mode only) |
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
serve.mjs               Local helper: serves both pages on one origin, persists library.json
package.json            npm start (serve) and npm test (browser-side verifiers)
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
