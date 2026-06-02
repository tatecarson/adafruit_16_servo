# Synchronized Motion Start (servo-vna)

Date: 2026-06-02

## Problem

A multi-board kinetic piece needs the three boards to move as **one gesture**,
not merely to "all be running something." Two distinct concerns:

- **Setlist sync** — *are all boards playing the same piece?* Already solved
  (servo-dos): the leader fires `RUN <seq>`, broadcast over UDP, followers run
  the same one. Discrete events, no drift problem.
- **Motion sync** — *is every board at the same frame of that piece at the same
  instant?* This is the **continuous** layer: `updateMotion()` interpolates
  keyframes every loop tick from `millis() - startMs`. Today each board starts
  interpolating from *its own arrival time*, so boards can be tens of ms out of
  phase for a Motion's entire duration, and because each free-runs its own
  `millis()` there is nothing correcting that error — it accumulates.

servo-vna closes the Motion-sync gap. Its acceptance criteria — begin within
20ms, drift <50ms over an hour — are *phase* and *drift* properties, which only
make sense for a continuous timeline (Motion), not for discrete setlist events.

Scope: **Motion start only.** Sequence step clocks are out of scope (steps are
mostly discrete poses where small offsets aren't visible).

## Existing primitives (reused, not rebuilt)

- `syncMillis() = millis() + clockOffset` — a shared clock. Followers learn
  `clockOffset = leaderUptime - millis()` from the lowest-alive peer's heartbeat
  (`Sync.cpp`), so `syncMillis()` ≈ the leader's `millis()` on every board. The
  leader's own offset stays 0.
- Heartbeats every 1000ms → the offset is re-learned each second, so drift
  between updates is bounded (well under the 50ms/hour budget).
- `Sync.cpp` packet framing: `INST1 <TYPE> <node> <seq> <payload>`.

## Mechanism

The board that **originates** a Motion picks a start instant on the shared
synced clock, broadcasts it, and every board (including the originator) anchors
that Motion's `startMs` to it.

Originator rather than strictly "leader": a browser can hit any board's `/cmd`,
and since all boards share the synced clock, any originator can name a valid
shared future time — no forwarding hop. In gallery mode the scheduler runs on
the leader, so the leader-driven case falls out for free.

### New UDP message — `MST` (motion-start)

Payload: `<motionId> <syncStartMs>`. Wire form:
`INST1 MST <node> <seq> <motionId> <syncStartMs>`.

- Sender: `broadcastMotionStart(const char* id, unsigned long syncStartMs)` in
  `Sync.cpp` (mirrors `broadcastEvent`).
- Receiver: `Sync.cpp` owns `clockOffset`, so it computes
  `localStartMs = syncStartMs - clockOffset` and calls a new extern hook
  `onMotionStartSync(const char* motionId, unsigned long localStartMs)`,
  implemented in `adafruit_16_servo.ino`.

### Lead time

`#define MOTION_START_LEAD_MS 150`. `syncStart = syncMillis() + 150`. Enough for
a LAN broadcast (<5ms typical) to land before the start instant; imperceptible
to a viewer.

### Trigger paths

- **Originating** (`MOTION <id>` from serial / HTTP `/cmd` / scheduler →
  `processCommand`): broadcast `MST(id, syncMillis() + LEAD)`, then arm locally
  at `millis() + LEAD`. (`millis()+LEAD` is algebraically identical to
  `syncStart - clockOffset`, so the originator needs no offset knowledge.)
- **Received `MST`** (`onMotionStartSync`): arm at `syncStartMs - clockOffset`.
- **Remove `MOTION` from the EVT mirror whitelist** (`shouldMirrorCommand`) so
  it is no longer double-fired by the old immediate-mirror path.

### Deferred / catch-up start (the one motion_engine.h change)

Add `startMotionFromStorageAt(id, localStartMs, announce)`; keep
`startMotionFromStorage(id, announce)` as `…At(id, millis(), announce)` so
existing behavior and tests are unchanged. In `updateMotion()`:

```c
int32_t signedElapsed = (int32_t)(millis() - motionRuntime.startMs);
if (signedElapsed < 0) return;          // armed, not started yet — hold pose
uint32_t elapsed = (uint32_t)signedElapsed;
```

One guard, three behaviors:

- **future** start → holds pose until the instant arrives, then begins at t=0;
- **on-time** start → begins at t=0;
- **late** packet, or a board that was busy/rebooted → starts mid-Motion **at
  the correct phase** (catch-up), so it rejoins in lockstep rather than
  restarting out of phase. This is what satisfies "follower reboot rejoins
  cleanly within one clock-broadcast cycle."

## Testing

The time math is pure logic and is host-unit-tested in
`test_motion_engine.cpp` (no hardware):

- deferred start: arm in the future, tick before `startMs` → no keyframe
  applied, motion not complete;
- on-time start: arm at now → t=0 keyframe applied;
- catch-up start: arm in the past → first tick lands at the correct elapsed
  offset, not at t=0;
- existing tests for `startMotionFromStorage` continue to pass via the wrapper.

The UDP broadcast/receive path uses the host mock; the physical acceptance
(three boards moving as one gesture, <50ms drift over an hour, reboot rejoin)
**cannot** be verified until boards 2 & 3 have their servos built. Tracked by
**servo-9oh** (blocked on servo-vna).

## Known limitation (noted, not fixed)

`clockOffset` ignores one-way UDP latency (~1–5ms on LAN) — a fixed sub-frame
error well inside the 20ms budget. YAGNI for now; a future refinement could
timestamp on arrival.
