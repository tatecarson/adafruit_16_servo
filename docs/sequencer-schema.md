# Sequencer Schema

The on-the-wire JSON contract shared by `servo_controller.html` (composer) and the Arduino firmware (player). The browser authors against this schema, bakes a single blob to each board over HTTP, and the firmware parses + executes it without further help.

This document is the single source of truth. Any change that breaks compatibility **must** bump `schemaVersion` in the bake blob.

- Current `schemaVersion`: **1**
- Encoding: UTF-8 JSON, no comments, no trailing commas. Field order doesn't matter.
- Units: durations in **milliseconds** (integer), angles in **degrees** (0–180), DC Motion speed signed **−50..+50** with sign indicating direction.
- Time origin: every `atMs` and `durationMs` is relative to the start of its containing entity (Motion or Sequence step). No wall-clock dependencies.

---

## 1. Identifiers and boards

### `boardId`

Each Arduino persists a stable integer in EEPROM under `boardId`. Values: **1**, **2**, **3**. The browser writes this once during setup (one-board-at-a-time) so cluster Motions can address specific sculptures even when DHCP shuffles IPs.

- The schema **never** uses IP addresses. IPs live in the browser's runtime config, not in the baked blob.
- A board only stores+executes tracks whose `boardId` matches its own. The bake pipeline slices the cluster Motion accordingly before POSTing per device.

### IDs

`id` fields are short stable strings the browser generates and never reuses; the user identifies content by the separate free-form `name` field. Ids stay fixed across rebakes and renames, so references never break.

- **Motions** and **Sequences** use opaque random tokens — `mt-<6 base36>` / `sq-<6 base36>` (e.g. `mt-k7f2q3`, `sq-swgvbr`). They are deliberately not derived from the name, so there is no id to "rename" and nothing that drifts from the name.
- **Setlists** still derive a kebab-case id from their name (`gallery-evening`).
- `id` regex (both forms): `^[a-z][a-z0-9-]{0,31}$`
- `name`: free-form UTF-8 string, ≤64 chars, display-only.

(Older content keeps whatever id it already has — including legacy kebab-case motion/sequence ids like `tidal-drift`; only newly created content gets the opaque form.)

### Tags

Free-form lowercase strings used by the shuffle scheduler. Conventionally one word (`calm`, `agitated`, `mechanical`, `slow`). Max 8 tags per item, max 24 chars each.

---

## 2. Motion (Layer 0 primitive)

A keyframed pattern across servos and DC motors. Cluster-scope by default; board-scope is a subset (a Motion with all tracks sharing one `boardId`).

```json
{
  "id": "tidal-drift",
  "name": "Tidal Drift",
  "tags": ["calm", "slow"],
  "scope": "cluster",
  "durationMs": 8000,
  "tracks": [
    {
      "kind": "servo",
      "boardId": 1,
      "channel": 0,
      "label": "left wing",
      "keyframes": [
        { "atMs": 0,    "value": 0   },
        { "atMs": 4000, "value": 90,  "easing": "linear" },
        { "atMs": 8000, "value": 0   }
      ]
    },
    {
      "kind": "servo",
      "boardId": 2,
      "channel": 0,
      "label": "right wing",
      "keyframes": [
        { "atMs": 0,    "value":  0  },
        { "atMs": 2000, "value": 40  },
        { "atMs": 6000, "value": -30 },
        { "atMs": 8000, "value":  0  }
      ]
    }
  ]
}
```

### Field reference

| Field | Type | Required | Notes |
|---|---|---|---|
| `id` | string | yes | See §1. |
| `name` | string | yes | Display name. |
| `tags` | string[] | no | Default `[]`. |
| `scope` | `"board"` \| `"cluster"` | yes | `"board"` is informational; firmware behavior is identical. |
| `durationMs` | integer ≥ 1 | yes | Total length. May extend past a track's final explicit keyframe; that value is held through the end. Must be at least the largest `atMs` in the Motion. |
| `tracks` | Track[] | yes | ≥ 1 track. |

### Track

| Field | Type | Required | Notes |
|---|---|---|---|
| `kind` | `"servo"` | yes | Motions are servo-only. DC speed is authored on Sequence steps — see [DC lanes](#dc-lanes). |
| `boardId` | 1 \| 2 \| 3 | yes | Stable per-device ID. |
| `channel` | integer | yes | 0–2 (current hardware uses 3 of 16 PCA9685 channels). |
| `label` | string | no | ≤32 chars. Composer's note. Not used by firmware. |
| `keyframes` | Keyframe[] | yes | ≥ 1 keyframe; first must have `atMs: 0`. A one-keyframe track is a constant hold. |

A `(boardId, kind, channel)` triple must be unique within a Motion (no two tracks targeting the same physical actuator).

### Keyframe

| Field | Type | Required | Notes |
|---|---|---|---|
| `atMs` | integer ≥ 0 | yes | Strictly increasing within a track. First keyframe must be `atMs: 0`; the last must not exceed the Motion's `durationMs`. Its value is held implicitly for any remaining time. |
| `value` | number | yes | 0–100 absolute percent down (`0` = fully up/retracted, `100` = fully down/lowered). |
| `easing` | `"linear"` | no | Default `"linear"`. Only `"linear"` is supported in `schemaVersion: 1`. Reserved values for future use: `"easeIn"`, `"easeOut"`, `"easeInOut"`, `"hold"`. The `easing` on a keyframe controls how the value is approached **from the previous keyframe** (i.e., it's the easing of the incoming segment). The first keyframe's `easing` is ignored. |
| `bypassRamp` | boolean | no | Legacy DC-track field. Motions no longer carry DC tracks; accepted and ignored. |

### Interpolation rules (firmware)

- Between two adjacent keyframes at `t0`/`v0` and `t1`/`v1`, with `t0 < t < t1`:
  - `linear`: `value(t) = v0 + (v1 - v0) * (t - t0) / (t1 - t0)`
- Servo values are interpreted as absolute percent-down positions, then mapped through each channel's calibrated up/down travel before being written to the PCA9685 every tick.
- Motion playback ends exactly at `durationMs`; each track implicitly holds its final keyframe value until then.
- Motions do not touch the DC motor. `ROTATE` therefore no longer cancels Motion playback — the two drive disjoint hardware.

### Servo slew-rate authoring notes

The firmware's interpolation clock and the servo's physical travel speed are separate. Firmware can finish a requested pulse ramp before the servo has mechanically arrived, or it can send pulse changes so small that the servo ignores several ticks before correcting.

Measured on the current goBILDA 5-turn winches on channels 0-2:

- Calibration: `minPulse=98`, `maxPulse=485` in PCA9685 ticks, equivalent to approximately `480-2370us` on board `192.168.8.198`.
- Full-range `DMOVE` requests below the mechanical floor still take about `7.7-7.8s` physically. Example: `DMOVE 0 0 1000`, `DMOVE 0 100 1000`, `DMOVE 0 0 6000`, and `DMOVE 0 100 6000` all measured near that floor.
- Full-range requests above the floor are honored by firmware, but long durations such as `DMOVE 0 0 15000` produce visible staccato motion. Each tick's pulse delta is below the servo's dead band, so the servo accumulates error and then jumps in a small max-speed correction.

Practical guidance for Motion authoring:

- Use roughly `4-7s` full-range segments when you want smooth max-speed winch travel.
- Treat `~7.7s` as the physical full-range floor for these winches; shorter full-range segment durations will not make the physical travel complete faster.
- Use durations above `12s` only when the stepped / staccato texture is acceptable for the piece.

---

## 3. Sequence (Layer 1: timed command list)

An ordered list of commands with explicit durations.

```json
{
  "id": "evening-arc",
  "name": "Evening Arc",
  "tags": ["calm"],
  "steps": [
    { "cmd": "MOTION tidal-drift", "durationMs": 8000, "target": "all" },
    { "cmd": "ROTATE 30",          "durationMs": 4000, "target": 2, "label": "spool wakes" },
    { "cmd": "PLAY 3",             "durationMs": 6000, "target": "all" },
    { "cmd": "STOP",               "durationMs": 1000, "target": "all" }
  ]
}
```

### Field reference

| Field | Type | Required | Notes |
|---|---|---|---|
| `id` | string | yes | See §1. |
| `name` | string | yes | Display name. |
| `tags` | string[] | no | Used by shuffle scheduler (§4). |
| `steps` | Step[] | yes | ≥ 1 step. |

### Step

| Field | Type | Required | Notes |
|---|---|---|---|
| `cmd` | string | yes | Any command the existing parser accepts: `MOTION <id>`, `PLAY n`, `SPLAY n [LOOP]`, `ROTATE <-100..100>`, `RIG ...`, `MOVE <ch> <deg> <ms>`, `S<ch> <deg>`, `TIMESCALE <n>`, `STOP`, etc. Max 96 chars. |
| `durationMs` | integer ≥ 0 | yes | How long until the next step fires. `0` = fire next step on the same tick (used for "chord" parallel firing across boards). |
| `target` | `"all"` \| 1 \| 2 \| 3 | yes | `"all"` broadcasts; a `boardId` restricts execution to that one board. Boards skip steps not addressed to them. Baked as a **bare integer** — a quoted `"2"` fails `bakeParseInteger` and the firmware treats the step as addressed to no board at all. |
| `dc` | object | no | Authoring-only per-board motor speeds — see [DC lanes](#dc-lanes). Stripped at bake time; never reaches the device. |
| `label` | string | no | ≤64 chars. Composer's note. Never sent to firmware (stripped at bake time). |
| `hold` | boolean | no | Composition-only. Stripped at bake time. |
| `noPrep` | boolean | no | Authoring-only. `true` deletes the automatic Motion pre-roll in front of this step — see [Pre-rolls](#pre-rolls). Absent means "keep it". Stripped at bake time. |

### Pre-rolls

A step whose `cmd` is `MOTION <id>` normally gets an automatic pre-roll: the
bake planner emits `MOTION <id> PREP <ms>` and extends `durationMs` by the same
amount, so the firmware glides the servos to keyframe zero before the Motion
starts. Nothing extra is stored — no bridge Motion, no dwell step.

The glide is sized at 77ms per 1% of the largest servo move it has to cover,
floored at 800ms. A servo already within 5% of its entry value is skipped, so a
Motion whose entry pose matches the previous Motion's exit pose gets no pre-roll
at all. Snapping keyframe values to multiples of 5 in the Motion editor exists to
make that alignment reachable by hand.

Setting `noPrep: true` refuses the pre-roll for one step: no `PREP`, no added
duration, and the Motion opens by snapping to its first keyframe. Clicking the
pre-roll block in the Arrange timeline toggles it; a refused pre-roll stays
visible there as a zero-duration ghost so it can be restored. Refusing a pre-roll
does not change where the servos end up, so the *following* step's pre-roll is
still planned from this Motion's exit pose.

### DC lanes

DC motor speed is authored on the Sequence, not inside a Motion. Each step
carries an optional `dc` map of board id to signed speed:

```json
{ "cmd": "MOTION tidal-drift", "durationMs": 8000, "target": "all",
  "dc": { "1": -40, "3": 25 } }
```

- Range is −50..+50, clamped by the editor. (Manual `ROTATE` keeps its wider
  ±100 range; the narrower lane cap is an authoring guardrail.)
- Speeds are **sticky**. A board absent from the map holds its last commanded
  speed — a motor has a running state, not a pose. `null` / omitted means
  *unchanged*; `0` means *stop*.
- At bake time the lanes flatten into targeted zero-duration `ROTATE` chord
  steps emitted immediately before the step they were authored against, so the
  device schema is unchanged and no new firmware command exists. Each change
  costs one step against the 16-step budget.
- A `STOP` step zeroes every motor, and the flattener resets its tracked state
  there. A non-looping Sequence ends by parking any board still turning.
- Flattening runs **after** pre-roll rewriting, so a DC change on a step that
  became `MOTION <id> PREP <ms>` takes effect at the step boundary — during the
  servo glide, not at the Motion's true start. The Arrange view draws the lane
  across the pre-roll block so this overlap is visible while authoring.

### Step execution rules (firmware)

- Each board runs the sequence in parallel; per-board skipping makes coordination implicit.
- A new command interrupts whatever the board was doing — there is no queueing.
- `STOP` halts the current Sequence's clock as well as any in-flight Motion / SPLAY / ROTATE on that board.
- `ROTATE` cancels a running Sequence (a manual override takes over the show) but **not** a running Motion: Motions are servo-only, so the two no longer contend. A DC lane chord dispatched from inside a Sequence step does not abort its own runner.

---

## 4. Setlist (Layer 2: shuffle-loop scheduler input)

A collection of Sequence references with optional shuffle rules. Runs forever once started.

```json
{
  "id": "gallery-evening",
  "name": "Gallery Evening",
  "mode": "shuffle",
  "entries": [
    { "seqId": "evening-arc",    "repeat": 1, "gapMs": 4000, "weight": 1 },
    { "seqId": "tidal-drift-seq","repeat": 2, "gapMs": 8000, "weight": 2 },
    { "seqId": "mechanical-pulse","repeat": 1,"gapMs": 12000,"weight": 1 }
  ],
  "shuffleRules": {
    "avoidSameTag": true,
    "minGapEntries": 3,
    "moodArc": "breathing",
    "seed": null
  }
}
```

### Field reference

| Field | Type | Required | Notes |
|---|---|---|---|
| `id` | string | yes | See §1. |
| `name` | string | yes | Display name. |
| `mode` | `"ordered"` \| `"shuffle"` | yes | `ordered` plays entries in order, looping; `shuffle` uses weighted random with rules. |
| `entries` | Entry[] | yes | ≥ 1 entry. |
| `shuffleRules` | ShuffleRules | only if `mode == "shuffle"` | See below. |

### Entry

| Field | Type | Required | Notes |
|---|---|---|---|
| `seqId` | string | yes | References a Sequence in the same bake blob. |
| `repeat` | integer ≥ 1 | yes | Play this entry N times before advancing. |
| `gapMs` | integer ≥ 0 | yes | Pause after this entry before the next. During the gap, the leader broadcasts `STOP`. |
| `weight` | integer ≥ 1 | only in `shuffle` | Relative pick probability. Ignored in `ordered`. |

### ShuffleRules

| Field | Type | Required | Notes |
|---|---|---|---|
| `avoidSameTag` | boolean | no | Default `true`. Picks won't follow an entry sharing any tag with the previous entry. |
| `minGapEntries` | integer ≥ 0 | no | Default `0`. The same `seqId` won't repeat within this many entries. |
| `moodArc` | `"random"` \| `"rising"` \| `"falling"` \| `"breathing"` | no | Default `"random"`. `breathing` alternates calm/agitated tags; `rising`/`falling` walk an energy axis defined by the order of tags in `schedulerConfig.tagEnergy`. Reserved; effective in `schemaVersion: 2`. |
| `seed` | integer \| null | no | Default `null` (true random). Set an integer for reproducible "shuffle" runs (documentation, video capture). |

### Scheduler execution rules (firmware)

- The board with `boardId: 1` is the **leader** by default and runs the scheduler. Followers (`boardId` 2, 3) execute commands broadcast over UDP but do not pick.
- Leader broadcasts `RUN_SEQ <seqId>` over the existing UDP mirror channel at the start of each entry.
- Between entries, leader broadcasts `STOP` and holds for `gapMs`.
- Looping is implicit: when the entries list is exhausted (ordered) or after each pick (shuffle), the scheduler picks the next entry indefinitely until a `STOP` command arrives over HTTP or serial.

---

## 5. Bake blob (the thing the browser POSTs)

A single compact deployment envelope sent to each board's `POST /sequences` endpoint. Each board persists it to the UNO R4's on-chip EEPROM (8 KB dataflash, two 4 KB slots; see Section 6). It is deliberately not a byte-for-byte copy of the editable `library.json`: display-only names, tags, labels, reserved fields, and default-valued options are omitted. Each board receives only the Motion tracks where `track.boardId` matches its own, but receives the full compact Sequence + Setlist library so any board can become leader. Pulling a bake back into the editor restores safe defaults and uses stable ids when omitted human-readable labels cannot be reconstructed.

```json
{
  "schemaVersion": 1,
  "bakedAtMs": 1747900000000,
  "motions": [ /* §2 */ ],
  "sequences": [ /* §3 */ ],
  "setlists": [ /* §4 */ ],
  "activeSetlistId": "gallery-evening",
  "schedulerConfig": {
    "leaderBoardId": 1,
    "graceMs": 10000
  }
}
```

### Field reference

| Field | Type | Required | Notes |
|---|---|---|---|
| `schemaVersion` | integer | yes | Currently `1`. Firmware rejects unsupported versions with HTTP 400. |
| `bakedAtMs` | integer | yes | Unix epoch milliseconds at bake time. Used for the diff display and the rollback log. |
| `motions` | Motion[] | yes | After bake-time slicing, may be empty on a board if no Motion targets it. |
| `sequences` | Sequence[] | yes | Full library; same on every board. |
| `setlists` | Setlist[] | yes | Full library; same on every board. |
| `activeSetlistId` | string \| null | yes | Setlist that `RUN AUTO` plays. `null` = no active setlist; `RUN AUTO` is a no-op. |
| `schedulerConfig` | object | yes | See below. |

### SchedulerConfig

| Field | Type | Required | Notes |
|---|---|---|---|
| `leaderBoardId` | 1 \| 2 \| 3 | yes | Which board runs the scheduler in gallery mode. |
| `graceMs` | integer ≥ 0 | yes | Boot grace period before auto-`RUN AUTO` when `gallery_mode` is on. Default `10000`. |
| `tagEnergy` | string[] | no | Authoring-only reserved field in schema v1. It is omitted from compact deployment payloads because firmware does not use it. |

Before slicing, the browser also rewrites an unsafe Sequence transition from `MOTION <id>` to the internal compact form `MOTION <id> PREP <ms>` and extends that step's `durationMs` by the same preparation time. Firmware glides its local servo tracks to the Motion's first keyframe during that interval, then begins normal playback. This preserves the authored step count and avoids storing generated bridge Motions or `DMOVE` chord steps. Pull/import reverses the deploy-only annotation before the Sequence becomes editable again.

---

## 6. Persistence and versioning (firmware)

Storage on the Arduino UNO R4 WiFi uses the standard `EEPROM` library, which on the Renesas RA4M1 maps to ~8 KB of dedicated dataflash. The first 8 bytes remain the stable header (magic, active-record marker, `boardId`, and `gallery_mode`); bytes 8176–8191 remain reserved for persisted servo calibration. The sequencer region supports two compatible modes:

- **Dual mode (default, ≤4080-byte payload):** two 4084-byte slots. Each slot is `[length:2][crc16:2][payload:≤4080]`.
- **Large mode (4081–6000-byte payload):** one overlapping record beginning at byte 8, using the same length/CRC/payload format. The active marker value `2` identifies this mode. Firmware leases its larger JSON read buffer temporarily from the heap so normal runtime RAM usage does not increase.

For dual mode, `POST /sequences` writes the payload to the *inactive* slot, then atomically flips the one-byte active-slot pointer. A power loss anywhere before the pointer flip leaves the previous active slot intact. The browser's restore-previous-bake button calls `POST /sequences/restore`, which flips back only when the other slot's CRC validates.

Large mode necessarily overlaps both dual slots, so it has **no previous-bake rollback and no power-loss atomicity while the write is in progress**. Firmware marks storage inactive before overwriting and writes the CRC last; an interrupted large write is rejected rather than treated as valid. The browser shows an explicit warning and requires confirmation. Returning to a small bake recreates dual mode; the second subsequent small bake restores normal previous-bake rollback.

All records use CRC16-CCITT (poly 0x1021, init 0xFFFF) over the length bytes concatenated with the payload.

Validation checks (firmware, at `POST /sequences` time):

1. `Content-Length` is between 1 and 6000.
2. Body is well-formed JSON shape — balanced braces, terminated strings.
3. Top-level object contains `"schemaVersion": 1`.

Semantic validation (Motion `id` references, keyframe ordering, value-range bounds) is deferred to the playback engines (servo-2cw, servo-3a9), which fail the relevant `MOTION`/`RUN` command rather than the bake. This keeps the bake endpoint fast and the storage layer independent of the schema's domain semantics.

The maximum bake-blob payload is 6000 bytes, while 4080 bytes remains the rollback-safe threshold. The browser minifies and compacts JSON before POSTing, validates all Motion/Sequence/Setlist references, surfaces actual UTF-8 bytes used per board, and labels which storage tier each board will use. Current project payloads remain well inside dual mode.

---

## 7. Reserved keys and future extensions

These appear in the schema but are not yet honored by `schemaVersion: 1` firmware. The browser may emit them; firmware will accept and ignore them. They become effective at the noted version.

| Key | Location | Effective in |
|---|---|---|
| `easing` non-linear values | Keyframe | 2 |
| `moodArc` non-`random` values | ShuffleRules | 2 |
| `tagEnergy` | SchedulerConfig | 2 |

Any new field added to the schema in a backward-compatible way (firmware safely ignores unknown fields) does **not** require a `schemaVersion` bump. A bump is required only when:

- An existing field changes meaning or range.
- A new required field is added.
- Parsing rules change.

---

## 8. Worked example: minimal viable bake blob

The smallest possible bake that demonstrates every layer. One Motion, one Sequence that fires it, one Setlist with that one Sequence, ordered loop.

```json
{
  "schemaVersion": 1,
  "bakedAtMs": 1747900000000,
  "motions": [
    {
      "id": "hello",
      "name": "Hello",
      "tags": ["test"],
      "scope": "board",
      "durationMs": 2000,
      "tracks": [
        {
          "kind": "servo",
          "boardId": 1,
          "channel": 0,
          "keyframes": [
            { "atMs": 0,    "value": 0   },
            { "atMs": 1000, "value": 90  },
            { "atMs": 2000, "value": 0   }
          ]
        }
      ]
    }
  ],
  "sequences": [
    {
      "id": "hello-seq",
      "name": "Hello Sequence",
      "tags": ["test"],
      "steps": [
        { "cmd": "MOTION hello", "durationMs": 2000, "target": 1 }
      ]
    }
  ],
  "setlists": [
    {
      "id": "hello-set",
      "name": "Hello Setlist",
      "mode": "ordered",
      "entries": [
        { "seqId": "hello-seq", "repeat": 1, "gapMs": 1000 }
      ]
    }
  ],
  "activeSetlistId": "hello-set",
  "schedulerConfig": {
    "leaderBoardId": 1,
    "graceMs": 10000
  }
}
```

This blob, posted to board 1, then `RUN AUTO`'d, should sweep servo channel 0 from 0° → 90° → 0° over two seconds, pause one second, and repeat forever.
