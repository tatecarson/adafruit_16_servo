# Manual checklist — Setlist "Simulate hour" (servo-6j4)

No hardware needed; this is a pure browser feature.

## Launch
```bash
python3 servo_library_server.py
```
Open the printed URL (default http://127.0.0.1:4173/servo_controller.html).
Scroll to section **// 05 Setlist**.

> Tip: have at least one **shuffle** setlist with 3+ entries pointing at real
> sequences (so durations are non-zero). Create/import a library first if needed.

---

## A. Button + panel basics
- [ ] A **Simulate hour** button appears in the Setlist transport row (next to *Dry Run*).
- [ ] Hovering it shows the tooltip about previewing an hour without moving servos.
- [ ] Clicking it opens a results panel below the entries (no servo moves, no commands sent).
- [ ] **Close** hides the panel.
- [ ] Re-opening works after closing.

## B. Empty / invalid setlist
- [ ] Select (or make) a setlist with **no entries**, click Simulate hour.
- [ ] No panel opens; the status line reads **"add entries before simulating"**.

## C. Seeded shuffle setlist (reproducible)
- [ ] In **shuffle rules**, set **seed** to a non-zero value (e.g. 42). Click Simulate hour.
- [ ] Badge reads **"seed 42 · reproducible"**.
- [ ] **No Re-roll** button is shown.
- [ ] Clicking Simulate hour again produces the **same** timeline order each time.

## D. Unseeded shuffle setlist (random each boot)
- [ ] Set **seed = 0**. Click Simulate hour.
- [ ] Badge reads **"unseeded · this draw seed … · stats over 100 runs"**.
- [ ] A **Re-roll** button is shown.
- [ ] Clicking **Re-roll** changes the representative timeline, but the play-balance
      bars / pacing numbers stay stable (they're averaged over 100 runs).

## E. Verdict banner
- [ ] With balanced equal-weight entries → banner is green and says something like **"✓ looks balanced"**.
- [ ] Give one entry a much larger **weight** (e.g. 50 vs 1), set minGap 0, simulate →
      banner turns amber **⚠** and mentions one sequence playing **~N× more** than another.
- [ ] (Optional) Point an entry at a deleted/renamed sequence → banner warns about a **missing sequence**.

## F. Play-balance column ("play balance")
- [ ] One row per sequence, sorted most-played first.
- [ ] Each row shows a bar, a **count** (`N×`), and a **% airtime**.
- [ ] Heavier-weighted entries get higher counts/longer bars (roughly proportional).
- [ ] For unseeded, counts may show a range (e.g. `12–18`); for seeded a single number.
- [ ] A missing sequence row is highlighted amber with a ⚠.

## G. Pacing line
- [ ] Shows **gap avg**, **longest** gap, and **soonest repeat** with sensible units (ms / s).
- [ ] Increasing an entry's **gap ms** and re-simulating raises the avg/longest gap.

## H. Representative timeline ("representative timeline")
- [ ] Scrollable chronological list: `m:ss` start · sequence name · duration (and `+gap` if any).
- [ ] An entry with **repeat > 1** shows `×N` next to the name and a longer duration.
- [ ] 🕳 appears on rows whose trailing gap is long (dead air).
- [ ] 🔁 appears when the same sequence recurs too soon (e.g. minGap 0 with a back-to-back repeat).
- [ ] ⚠ appears on rows for missing sequences.
- [ ] Total spans ~one hour (last rows near `60:00`); very long lists show a "… N more plays" footer.

## I. Ordered mode (sanity)
- [ ] Switch the setlist to **ordered**, simulate.
- [ ] Timeline cycles entries in list order, looping for the hour; no Re-roll (ordered is deterministic).

## J. Cross-checks
- [ ] Editing entries/weights/seed and re-simulating reflects the changes.
- [ ] Switching the selected setlist (dropdown) and simulating uses the newly-selected one.
- [ ] Nothing in the page console errors during any of the above (open DevTools console).

---

### If something's off
Note which box failed + the setlist config (mode, seed, entries/weights/gaps) and the
observed vs expected. The pick-order math is verified against firmware
(`make -C test sim-verify`), so discrepancies are most likely in the **display**
layer (labels, flags, formatting) rather than the simulation itself.
