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

- `Sync.cpp` packet framing: `INST1 <TYPE> <node> <seq> <payload>`.
- A peer table (`peers[]`) populated from heartbeats, with each peer's `id` and
  `ip` — used to unicast the start (see below).

> **Note — relative, not absolute clock.** The first implementation used an
> absolute shared-clock timestamp (`syncMillis()`/`clockOffset`). Hardware
> testing showed that approach is catastrophically fragile: if any board's
> `clockOffset` is stale or unlearned, its start timestamp is mis-converted by
> peers and the Motion instant-completes or hangs (servo-dvi). The shipped
> design below is **relative** and needs no shared clock at all.

## Mechanism

The board that **originates** a Motion broadcasts "start in `LEAD` ms"; every
board (including the originator) arms the Motion at **its own**
`millis() + LEAD`. No shared-clock conversion, so no board's clock offset can
desync the start. The only spread between boards is UDP propagation (a few ms
on a LAN), well inside the 20ms budget.

Originator rather than strictly "leader": a browser can hit any board's `/cmd`,
and any board can say "start in LEAD ms" — no forwarding hop. In gallery mode
the scheduler runs on the leader, so the leader-driven case falls out for free.

### UDP message — `MST` (motion-start)

Payload: `<motionId> <leadMs>`. Wire form: `INST1 MST <node> <seq> <id> <leadMs>`.

- Sender: `broadcastMotionStart(const char* id, unsigned long leadMs)` in
  `Sync.cpp`. **Unicast, not broadcast** — see Delivery below.
- Receiver: arms at `millis() + leadMs` via the extern hook
  `onMotionStartSync(const char* motionId, unsigned long localStartMs)`,
  implemented in `command_interface.h`. No `clockOffset` involved.

### Lead time

`#define MOTION_START_LEAD_MS 150`. Just has to exceed UDP propagation so every
board arms together; imperceptible to a viewer.

### Delivery — unicast, not broadcast

`MST` is a **one-shot** trigger with no retransmit. WiFi *broadcast* frames get
no 802.11 ACK/retry and are dropped often — hardware testing measured a single
broadcast missing a given follower ~50% of the time (servo-dvi). So
`broadcastMotionStart` builds the packet **once** (one `seq`) and **unicasts** a
copy to each peer in the table; unicast gets link-layer ACK + retransmission.
The shared `seq` means any incidental duplicate is deduped (`lastMotionSeq`),
not replayed as a second arm. (Heartbeats stay broadcast — they repeat every
second, so individual drops are invisible.)

### Trigger paths

- **Originating** (`MOTION <id>` from serial / HTTP `/cmd` / scheduler →
  `dispatchCommand` intercept): `broadcastMotionStart(id, LEAD)`, then arm
  locally at `millis() + LEAD`.
- **Received `MST`** (`onMotionStartSync`): arm at `millis() + leadMs`.
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

- **future** start (the normal case: `millis() + LEAD`) → holds pose until the
  instant arrives, then begins at t=0;
- **on-time** start → begins at t=0;
- **slightly-late** arm (processing jitter pushes the arm a few ms past the
  intended instant) → starts at the correct small elapsed offset rather than
  snapping to t=0, keeping boards aligned. A board that *misses* the `MST`
  entirely (rebooting, off-network) simply doesn't play that Motion and joins
  the next one — "rejoins within one trigger" — which is acceptable now that
  starts are frequent and unicast-reliable.

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

## Sender-reboot handling (dedup)

`MST` is de-duplicated per peer with `lastMotionSeq`, alongside the existing
`lastEventSeq`/`lastHeartbeatSeq`. Because `txSeq` resets to 0 when a board
reboots, a non-rebooted receiver holding a high stored seq would otherwise
silently drop the restarted sender's packets until its seq climbed back —
dropping its mirrored commands and synchronized starts after a reboot.
`handleSyncPacket` detects this: if an incoming seq falls more than
`SEQ_REBOOT_GAP` (16) below the per-peer high-water mark, it clears all three
dedup counters so the restarted peer is accepted immediately. The gap tolerates
ordinary UDP reordering (a reboot drops thousands behind; a reorder, a few).
(A *follower* reboot needs no special handling — it clears its own RAM on boot.)

## Notes

- The heartbeat `clockOffset`/`syncMillis()` machinery still exists but is no
  longer used for Motion start (relative timing replaced it). Left in place as
  it's harmless and may serve future needs; removing it is out of scope.
- Start *alignment* (≤20ms) and *drift* (<50ms/hour) on real servos still need
  the multi-board physical check in **servo-9oh** — HTTP `/status.json` polling
  can't measure sub-frame alignment. What hardware testing *did* confirm:
  all boards start (no instant-complete/hang regardless of clock state) and
  100% unicast delivery across 14 trials from every originator.
