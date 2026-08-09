# Manual checklist — three machines through the control page

Written after the branch that made the dashboard machine-aware, ported the other
two sculptures into the simulator, and retargeted the library to the curtain.

Each board now drives a different machine, so most of what this checks is that
the software has stopped assuming they are alike.

| Board | Machine | Servos | Motor |
|---|---|---|---|
| 1 | centre wands | 3 wand throws, **36°** of travel | 4:1 — `ROTATE 100` = 6.25 rpm |
| 2 | field | **none** | direct — `ROTATE 100` = 128 rpm |
| 3 | dowel curtain | 3 winches, **1800°** (5 turns) | direct — `ROTATE 100` = 25 rpm |

## Launch

```bash
npm start
```

Both pages come off one origin, which is what makes the dashboard→simulator
BroadcastChannel reliable:

- dashboard <http://127.0.0.1:4173/servo_controller.html>
- simulator <http://127.0.0.1:4173/sculpture_3d.html>

Firmware, if a board needs it: `./compile-firmware.sh`, then the OTA section's
**Use compiled bin**. One binary serves every board — it reads its own stored
id and configures itself.

---

## A. Nothing connected

- [ ] Masthead reads **three machines · one cluster**; section 01 is **Machines**.
- [ ] Motion editor shows **three board groups and six track rows**.
      Nine rows means a stale page — hard-reload.
- [ ] Board 2's group says **"field has no servos"** instead of listing three.
- [ ] Machine filter (left of the motion picker): `wands` lists only wand
      motions, `curtain` lists the seven originals.
- [ ] Storage badge reads **"loaded from library.json"**.
      If it says browser cache, the file did not parse — stop, nothing below is
      trustworthy.
- [ ] Sequencer, `Bench · all three`: every step's target reads a machine name,
      and the DC column is headed **wands / field / curtain** with an rpm figure
      under any value that is set.

## B. Boards powered, nothing baked

- [ ] **Board strip names sit on the right IPs.** Everything else rests on this;
      a wrong name means `/boardId` is reporting something unexpected.
- [ ] The DC pill shows rpm alongside the percentage. The same `+30` should read
      about **1.88 / 38 / 7.50 rpm** across the three.
- [ ] Motor Test → field: says it has no servos, and the **All Down / All Up
      buttons are gone** (they used to fire at absent servos).
- [ ] Motor Test → wands: channels labelled *Wand I/II/III throw*, note says
      `ROTATE 30 is about 1.88 rpm`.
- [ ] Board 1, one wand: `DOWN 0 100` then `DOWN 0 0`.
      Expect a **near-instant** sweep — about 154 ms.
      **If it takes ~7 s the 36° did not apply**; check the Serial boot line
      (115200) for `servos: wand profile · 100%down = 36deg`.
- [ ] Direction: 100 drives the wand **down**, into the tube orbit.
      Backwards means `reverseDir` — one line in `servo_setup.h`.
- [ ] Resolution: `DOWN 0 0 / 50 / 100` should be three clear positions, but
      `40 / 50 / 60` may be indistinguishable. 36° is only ~7 PWM ticks, so the
      wand has roughly **seven** usable positions, not a hundred. Expected, and
      the fix if it matters is mechanical, not software.

## C. Bake

- [ ] Bake budget reads roughly **5277 / 3090 / 5484** bytes.
- [ ] Boards 1 and 3 warn **large mode, no rollback** — expected. Under the 6000
      hard cap, over the 4080 rollback-safe line.
- [ ] Bake succeeds. **Board 2 receives zero motions** — correct, it has no
      servos. Its payload is the sequences, which every board needs in full.

## D. Run

- [ ] `Bench · wands` on board 1 — about 7 s. Wands clear, drive in, cascade,
      clear.
- [ ] `Bench · field` — the deck steps 15 → 38 → 64 rpm.
- [ ] Any `Curtain ·` sequence — **only the curtain moves.** Before the
      migration these fired at all three machines; that is the change this
      whole branch exists to make.
- [ ] `Bench · all three`, then the `bench-test` setlist end to end.

## E. The room

- [ ] Open the simulator, press **Audio on**, then **Walk** (or `V`).
      WASD to move, mouse to look, shift to hurry, Esc back to orbit.
- [ ] Click a machine — the camera flies to it and the servo/DC panels switch to
      that board.
- [ ] Walk between the three with all of them running. Solo and mute each in
      turn; a soloed machine should be the only thing audible, drones included.
- [ ] Board 2: swap deck material `ply12` → `steel15` and confirm it audibly
      changes. Verified numerically, never by ear.
- [ ] Drive the simulator from the dashboard: a Motor Test command should show
      in the sim's Dashboard-link line as `→ board N`.

---

## Things that look wrong and are not

- **Board 2 bakes with no motions.** It has no servos. Its payload is sequences.
- **Large-mode warning on boards 1 and 3.** Over rollback-safe, under the cap.
- **Every board carries every sequence.** Each one filters per step at runtime
  and needs the whole timeline to keep time — that is what keeps three machines
  synchronised with no conductor.

## Things that would be wrong

- A curtain sequence moving the wands or the field **when run from the bake**.
- The board strip naming a machine against the wrong IP.
- A wand sweep taking seconds rather than milliseconds.

## Known gap

Browser **Play** still fans a targeted `ROTATE` out to the whole cluster, while
the **baked** run targets it correctly. The firmware cannot tell a targeted step
from a typed command, so it mirrors both. The sequencer warns on Play. Tracked
as `servo-sh2`; a real fix needs a wire form that distinguishes the two.

## If something is off

- Serial at 115200 prints the resolved servo profile at boot. A board reporting
  the wrong profile has the wrong stored id — `BOARDID <n>` fixes it without a
  reflash.
- `npm test` runs the browser-side verifiers; `make -C test` runs the firmware
  suites.
- `__selfTest()` in the simulator's console re-runs the solver from a pinned
  seed and should report **1331 strikes**.
