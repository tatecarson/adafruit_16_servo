# Sequence Transition Bridges — Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** At bake time, automatically insert glide "bridges" between sequence steps so servos travel from one motion's exit pose to the next motion's entry pose instead of snapping — plus a small authoring default that seeds new motions from the previous motion's ending pose.

**Architecture:** A pure, testable transform (`insertSequenceBridges`) walks each sequence's steps, tracks a virtual servo position, and inserts bridges: interior transitions become synthesized 2-keyframe **bridge motions** (one `MOTION __bridge_…` step each — cheap against the 16-step firmware cap); the cold-start transition becomes a `DMOVE` chord (ramps from real hardware position). Thin glue (`buildBakeLibrary`) wires it into the existing bake path. The firmware is unchanged. Design: `docs/plans/2026-07-18-sequence-bridge-transitions-design.md`.

**Tech Stack:** Single-file browser app (`servo_controller.html`, vanilla JS), Node `.mjs` host tests that extract marked regions (pattern: `test/verify_seq_arrangement.mjs`), Arduino C++ firmware (unchanged here).

---

## Conventions for this plan

- All new pure functions go **inside one marked region** so the test harness can extract them:
  ```
  // === SEQ-BRIDGE-CORE START ===
  ...functions...
  // === SEQ-BRIDGE-CORE END ===
  ```
  Place the region in `servo_controller.html` immediately **before** `function sliceForBoard(` (currently ~line 6410).
- The region must be **self-contained**: it references no module-level constants. Tunables arrive via an `opts` object with literal defaults (kept in sync with the real constants by the glue that calls them).
- Commit after every green step. Run host tests with: `make -C test sim-verify` (fast, Node only). Full suite: `make -C test`.
- Data shapes (confirmed in code):
  - **Motion:** `{ id, name, scope, durationMs, tracks: [ { kind:"servo"|"dc", boardId:1|2|3, channel, label, keyframes:[{atMs,value}] } ] }`
  - **Track key:** `` `${boardId}:${kind}:${channel}` `` (see `motionTrackKey`)
  - **Servo specs:** kind `"servo"`, value 0..100. DC: kind `"dc"`, value -100..100 (excluded from bridging).
  - **Sequence step:** `{ cmd, durationMs, target:"all"|"1"|"2"|"3", label, hold }`; a motion step's `cmd` is `` `MOTION ${motionId}` ``.
  - Constants to mirror: `SERVO_FEASIBILITY_MS_PER_PERCENT = 77`, `SEQ_MAX_STEPS = 16` (firmware `servo_runtime.h`).

---

## Task 0: Add the test file skeleton and the `SEQ_MAX_STEPS` JS mirror

**Files:**
- Create: `test/verify_seq_bridges.mjs`
- Modify: `test/Makefile` (add to `sim-verify` list)
- Modify: `servo_controller.html` (add JS constant near the other mirrors, ~line 3501)

**Step 1: Add the JS mirror constant.** After `const MOTION_MAX_TRACKS = 17;` (~line 3501) add:
```js
// Mirror of firmware SEQ_MAX_STEPS (adafruit_16_servo/servo_runtime.h). Baked
// sequences (including auto-inserted transition bridges) must fit this many steps.
const SEQ_MAX_STEPS = 16;
```

**Step 2: Create the empty test harness** `test/verify_seq_bridges.mjs`, copying the extraction boilerplate from `test/verify_seq_arrangement.mjs` (lines 1–36) but with:
```js
const START = "// === SEQ-BRIDGE-CORE START ===";
const END = "// === SEQ-BRIDGE-CORE END ===";
```
and export list:
```js
writeFileSync(modPath, `${core}\nexport { bridgeServoPose, planColdStartBridge, planInteriorBridge, insertSequenceBridges, seedFirstKeyframesFromExit };\n`, "utf8");
const { bridgeServoPose, planColdStartBridge, planInteriorBridge, insertSequenceBridges, seedFirstKeyframesFromExit } = await import(pathToFileURL(modPath).href);
```
Keep the `eq/ok/fail` helpers. End with the `passed/failed` summary + `process.exit`. Add a placeholder `console.log("=== Sequence transition bridges ===")` and no assertions yet.

**Step 3: Wire into the Makefile.** In `test/Makefile`, add `&& \` after `node verify_seq_arrangement.mjs` and a new line `node verify_seq_bridges.mjs;` inside the `sim-verify` target.

**Step 4: Add the region markers** in `servo_controller.html` immediately before `function sliceForBoard(`:
```js
// === SEQ-BRIDGE-CORE START ===
// (functions added in later tasks)
// === SEQ-BRIDGE-CORE END ===
```

**Step 5: Verify the harness runs (and fails clean).** Run: `node test/verify_seq_bridges.mjs`
Expected: it errors on the missing exports (functions not defined yet) — confirms extraction is wired. That's fine; the next task makes it green.

**Step 6: Commit.**
```bash
git add servo_controller.html test/verify_seq_bridges.mjs test/Makefile
git commit -m "test: scaffold sequence-bridge host test + SEQ_MAX_STEPS mirror"
```

---

## Task 1: `bridgeServoPose` — entry/exit pose extraction

**Files:** Modify region in `servo_controller.html`; Modify `test/verify_seq_bridges.mjs`.

**Step 1: Write failing tests** in `verify_seq_bridges.mjs`:
```js
const rise = { id:"rise", durationMs:1000, tracks:[
  { kind:"servo", boardId:1, channel:0, keyframes:[{atMs:0,value:0},{atMs:1000,value:80}] },
  { kind:"servo", boardId:2, channel:1, keyframes:[{atMs:0,value:50},{atMs:1000,value:50}] },
  { kind:"dc",    boardId:1, channel:0, keyframes:[{atMs:0,value:0},{atMs:1000,value:60}] },
]};
eq("entry pose reads first-keyframe servo values", bridgeServoPose(rise,"entry"), {"1:servo:0":0,"2:servo:1":50});
eq("exit pose reads last-keyframe servo values", bridgeServoPose(rise,"exit"), {"1:servo:0":80,"2:servo:1":50});
eq("dc tracks are excluded", "1:dc:0" in bridgeServoPose(rise,"exit"), false);
eq("single-keyframe track holds its constant", bridgeServoPose(
  { tracks:[{kind:"servo",boardId:3,channel:2,keyframes:[{atMs:0,value:33}]}] }, "exit"), {"3:servo:2":33});
eq("missing tracks yield empty pose", bridgeServoPose({}, "entry"), {});
```

**Step 2: Run — expect FAIL** (`bridgeServoPose is not a function`). Run: `node test/verify_seq_bridges.mjs`

**Step 3: Implement** inside the region:
```js
// Servo-only pose map { "boardId:servo:channel": value(0..100) } at a motion's
// first ("entry") or last ("exit") keyframe. DC tracks have no position and are
// skipped. A single-keyframe track is a constant hold (entry === exit).
function bridgeServoPose(motion, edge) {
  const pose = {};
  for (const t of (motion && motion.tracks) || []) {
    if (t.kind !== "servo") continue;
    const kfs = Array.isArray(t.keyframes) ? t.keyframes : [];
    if (!kfs.length) continue;
    let pick = kfs[0];
    for (const kf of kfs) {
      const at = Number(kf && kf.atMs) || 0;
      const cur = Number(pick && pick.atMs) || 0;
      if (edge === "exit" ? at >= cur : at <= cur) pick = kf;
    }
    const v = Math.max(0, Math.min(100, Number(pick && pick.value) || 0));
    pose[`${t.boardId}:servo:${t.channel}`] = v;
  }
  return pose;
}
```

**Step 4: Run — expect PASS.**

**Step 5: Commit.**
```bash
git add servo_controller.html test/verify_seq_bridges.mjs
git commit -m "feat: bridgeServoPose entry/exit pose extraction"
```

---

## Task 2: `planInteriorBridge` — synthesized bridge motion

**Files:** Modify region; Modify test.

**Step 1: Write failing tests:**
```js
const from = {"1:servo:0":80,"2:servo:1":50};
const to   = {"1:servo:0":0, "2:servo:1":52};   // ch0 moves 80, ch1 moves 2 (below 5% slop)
const b = planInteriorBridge(from, to, "__bridge_seq_0", { msPerPercent:77, minDeltaPercent:5, floorMs:800 });
eq("bridge has a single MOTION step", b.step.cmd, "MOTION __bridge_seq_0");
eq("duration scales with the farthest-moving servo", b.motion.durationMs, Math.ceil(80*77)); // 6160
eq("step duration matches motion duration", b.step.durationMs, b.motion.durationMs);
eq("only servos past the slop threshold get a track", b.motion.tracks.map(t=>t.channel), [0]);
eq("track ramps from previous value to next value", b.motion.tracks[0].keyframes,
   [{atMs:0,value:80},{atMs:6160,value:0}]);
eq("synthesized motion id is prefixed for UI hiding", b.motion.id.startsWith("__bridge_"), true);
eq("no-op transition (all within slop) returns null",
   planInteriorBridge({"1:servo:0":50}, {"1:servo:0":52}, "x", {}), null);
eq("duration honors the 800ms floor",
   planInteriorBridge({"1:servo:0":50}, {"1:servo:0":56}, "x", {floorMs:800,msPerPercent:77,minDeltaPercent:5}).motion.durationMs, 800);
```

**Step 2: Run — expect FAIL.**

**Step 3: Implement** in the region:
```js
// Interior transition (previous pose fully known): a 2-keyframe "bridge motion"
// that ramps each moved servo from its previous value to the next entry value.
// Firmware asserts keyframe-0 instantly, but the servo is already sitting there
// (previous motion held its exit pose), so it's a no-op followed by a smooth
// ramp. Returns { motion, step, channels } or null when nothing moves enough.
function planInteriorBridge(fromPose, toPose, id, opts) {
  opts = opts || {};
  const msPerPercent = opts.msPerPercent != null ? opts.msPerPercent : 77;
  const minDeltaPercent = opts.minDeltaPercent != null ? opts.minDeltaPercent : 5;
  const floorMs = opts.floorMs != null ? opts.floorMs : 800;

  const tracks = [];
  const channels = [];
  let maxDelta = 0;
  for (const key of Object.keys(toPose)) {
    const to = toPose[key];
    const from = fromPose[key];
    if (from == null) continue;                 // unknown handled by cold-start path
    const delta = Math.abs(to - from);
    if (delta < minDeltaPercent) continue;
    const parts = key.split(":");
    const boardId = Number(parts[0]);
    const channel = Number(parts[2]);
    tracks.push({ kind:"servo", boardId, channel, label:`B${boardId}.S${channel}`,
      keyframes:[{atMs:0, value:from}, {atMs:0, value:to}] });
    channels.push(key);
    maxDelta = Math.max(maxDelta, delta);
  }
  if (!tracks.length) return null;
  const durationMs = Math.max(floorMs, Math.ceil(maxDelta * msPerPercent));
  for (const t of tracks) t.keyframes[1].atMs = durationMs;
  const motion = { id, name:"↳ bridge", scope:"cluster", tags:[], durationMs, tracks };
  const step = { cmd:`MOTION ${id}`, durationMs, target:"all", label:"↳ bridge", hold:false, bridge:true };
  return { motion, step, channels };
}
```

**Step 4: Run — expect PASS. Step 5: Commit** (`feat: planInteriorBridge synthesized bridge motion`).

---

## Task 3: `planColdStartBridge` — DMOVE chord for unknown start

**Files:** Modify region; Modify test.

**Step 1: Write failing tests:**
```js
const cs = planColdStartBridge({"1:servo:0":30,"2:servo:2":90},
  { msPerPercent:77, floorMs:800, unknownDeltaPercent:100 });
eq("cold-start duration uses the conservative unknown delta", cs.durationMs, Math.ceil(100*77)); // 7700
eq("one DMOVE per driven servo, targeted at its board", cs.steps.slice(0,2).map(s=>[s.cmd,s.target]),
   [["DMOVE 0 30 7700","1"],["DMOVE 2 90 7700","2"]]);
eq("DMOVE steps are zero-duration (chord)", cs.steps.slice(0,2).every(s=>s.durationMs===0), true);
eq("trailing dwell step holds the clock", [cs.steps.at(-1).cmd, cs.steps.at(-1).durationMs], ["",7700]);
eq("empty pose yields no cold-start bridge", planColdStartBridge({}, {}), null);
```

**Step 2: Run — expect FAIL.**

**Step 3: Implement** in the region:
```js
// Cold start: the servos' real position is unknown, so no baked keyframe-0 value
// is correct. DMOVE ramps from the servo's ACTUAL hardware position, so we emit a
// chord of zero-duration DMOVEs (one per driven servo, targeted per board) plus a
// dwell step that holds the sequence clock while they glide. Duration is sized to
// a conservative worst-case delta since it can't be measured.
function planColdStartBridge(toPose, opts) {
  opts = opts || {};
  const msPerPercent = opts.msPerPercent != null ? opts.msPerPercent : 77;
  const floorMs = opts.floorMs != null ? opts.floorMs : 800;
  const unknownDeltaPercent = opts.unknownDeltaPercent != null ? opts.unknownDeltaPercent : 100;

  const keys = Object.keys(toPose);
  if (!keys.length) return null;
  const durationMs = Math.max(floorMs, Math.ceil(unknownDeltaPercent * msPerPercent));
  const channels = [];
  const steps = keys.map(key => {
    const parts = key.split(":");
    channels.push(key);
    return { cmd:`DMOVE ${Number(parts[2])} ${toPose[key]} ${durationMs}`,
      durationMs:0, target:String(Number(parts[0])), label:"↳ bridge", hold:false, bridge:true };
  });
  steps.push({ cmd:"", durationMs, target:"all", label:"↳ bridge dwell", hold:false, bridge:true });
  return { steps, durationMs, channels };
}
```

**Step 4: Run — expect PASS. Step 5: Commit** (`feat: planColdStartBridge DMOVE chord`).

---

## Task 4: `insertSequenceBridges` — the walker

**Files:** Modify region; Modify test.

**Step 1: Write failing tests:**
```js
const lib = [
  { id:"rise", durationMs:1000, tracks:[{kind:"servo",boardId:1,channel:0,keyframes:[{atMs:0,value:0},{atMs:1000,value:80}]}] },
  { id:"fall", durationMs:1000, tracks:[{kind:"servo",boardId:1,channel:0,keyframes:[{atMs:0,value:0},{atMs:1000,value:0}]}] },
];
const opts = { seqId:"s", msPerPercent:77, minDeltaPercent:5, floorMs:800, unknownDeltaPercent:100, maxSteps:16 };

// rise (entry 0, unknown before) → cold-start bridge; then rise; then rise.exit=80
// -> fall.entry=0 differs by 80 -> interior bridge; then fall.
const r = insertSequenceBridges([{cmd:"MOTION rise",durationMs:1000},{cmd:"MOTION fall",durationMs:1000}], lib, opts);
eq("cold-start bridge precedes first motion",
   [r.steps[0].cmd.startsWith("DMOVE"), r.steps[1].cmd, r.steps[2].cmd],
   [true, "", "MOTION rise"]);
eq("interior bridge inserted before fall",
   r.steps.slice(3).map(s=>s.cmd), ["MOTION __bridge_s_0","MOTION fall"]);
eq("one synth bridge motion produced", r.bridgeMotions.map(m=>m.id), ["__bridge_s_0"]);
eq("no error under budget", r.error, null);

// threshold skip: consecutive identical exits/entries need no interior bridge
const r2 = insertSequenceBridges([{cmd:"MOTION rise",durationMs:1000},{cmd:"MOTION rise",durationMs:1000}], lib, opts);
eq("identical transition inserts no interior bridge", r2.bridgeMotions.length, 0);

// non-MOTION step resets virtual position -> next motion gets cold-start bridge
const r3 = insertSequenceBridges([{cmd:"MOTION rise",durationMs:1000},{cmd:"STOP",durationMs:500},{cmd:"MOTION fall",durationMs:1000}], lib, opts);
eq("opaque step passes through untouched", r3.steps.find(s=>s.cmd==="STOP")!=null, true);
eq("motion after opaque step gets a DMOVE cold-start bridge",
   r3.steps.filter(s=>s.cmd.startsWith("DMOVE")).length>0, true);

// budget overflow reports an error (does not throw)
const many = Array.from({length:20},()=>({cmd:"MOTION rise",durationMs:1000}));
eq("over-budget sequence reports an error", insertSequenceBridges(many, lib, opts).error.code, "bridge-budget");
```

**Step 2: Run — expect FAIL.**

**Step 3: Implement** in the region:
```js
function bridgeMotionIdFromCmd(cmd) {
  const m = /^\s*MOTION\s+(\S+)\s*$/i.exec(String(cmd || ""));
  return m ? m[1] : null;
}

// Walk a sequence's steps, tracking a virtual servo position, and insert bridges:
//  - cold start / after an opaque (non-MOTION) step: position unknown -> DMOVE bridge
//  - interior transition (position known): synthesized bridge motion (1 step)
// Returns { steps, bridgeMotions, summary, error }. Never throws; a step-budget
// overflow is reported in `error` so the bake orchestrator can block + name it.
function insertSequenceBridges(steps, motions, opts) {
  opts = opts || {};
  const seqId = opts.seqId || "seq";
  const maxSteps = opts.maxSteps != null ? opts.maxSteps : 16;
  const byId = new Map((motions || []).map(m => [String(m.id).toLowerCase(), m]));

  const outSteps = [];
  const bridgeMotions = [];
  const summary = [];
  let virt = null;              // null = unknown position (cold start / post-opaque)
  let bridgeSeq = 0;

  for (const step of (steps || [])) {
    const motionId = bridgeMotionIdFromCmd(step.cmd);
    const motion = motionId ? byId.get(motionId.toLowerCase()) : null;
    if (!motion) { outSteps.push(step); virt = null; continue; }

    const entry = bridgeServoPose(motion, "entry");
    if (virt === null) {
      const cold = planColdStartBridge(entry, opts);
      if (cold) { outSteps.push(...cold.steps); summary.push({ kind:"cold", durationMs:cold.durationMs, channels:cold.channels }); }
    } else {
      const bridge = planInteriorBridge(virt, entry, `__bridge_${seqId}_${bridgeSeq}`, opts);
      if (bridge) {
        bridgeMotions.push(bridge.motion);
        outSteps.push(bridge.step);
        summary.push({ kind:"interior", id:bridge.motion.id, durationMs:bridge.motion.durationMs, channels:bridge.channels });
        bridgeSeq++;
      }
    }
    outSteps.push(step);
    virt = bridgeServoPose(motion, "exit");
  }

  const error = outSteps.length > maxSteps
    ? { code:"bridge-budget", produced:outSteps.length, max:maxSteps }
    : null;
  return { steps:outSteps, bridgeMotions, summary, error };
}
```

**Step 4: Run — expect PASS. Step 5: Commit** (`feat: insertSequenceBridges walker with budget guard`).

---

## Task 5: `seedFirstKeyframesFromExit` — Layer 1 authoring default

**Files:** Modify region; Modify test.

**Step 1: Write failing tests:**
```js
const prev = { tracks:[
  {kind:"servo",boardId:1,channel:0,keyframes:[{atMs:0,value:0},{atMs:1000,value:70}]},
  {kind:"dc",   boardId:1,channel:0,keyframes:[{atMs:0,value:0},{atMs:1000,value:40}]},
]};
const fresh = { tracks:[
  {kind:"servo",boardId:1,channel:0,keyframes:[{atMs:0,value:0}]},
  {kind:"servo",boardId:2,channel:1,keyframes:[{atMs:0,value:0}]},
  {kind:"dc",   boardId:1,channel:0,keyframes:[{atMs:0,value:0}]},
]};
seedFirstKeyframesFromExit(prev, fresh);
eq("servo keyframe-0 seeded from previous exit", fresh.tracks[0].keyframes[0].value, 70);
eq("servo with no previous exit stays at 0", fresh.tracks[1].keyframes[0].value, 0);
eq("dc tracks are never seeded", fresh.tracks[2].keyframes[0].value, 0);
eq("no previous motion is a safe no-op", seedFirstKeyframesFromExit(null, {tracks:[]}).tracks, []);
```

**Step 2: Run — expect FAIL.**

**Step 3: Implement** in the region:
```js
// Layer 1: when creating a new motion, seed each servo track's first keyframe
// from the previous motion's exit pose so authoring "starts where the last one
// ended". Convenience only (reduces generated bridges); mutates + returns newMotion.
function seedFirstKeyframesFromExit(prevMotion, newMotion) {
  if (!prevMotion || !newMotion) return newMotion;
  const exit = bridgeServoPose(prevMotion, "exit");
  for (const t of (newMotion.tracks || [])) {
    if (t.kind !== "servo") continue;
    const key = `${t.boardId}:servo:${t.channel}`;
    if (!(key in exit)) continue;
    if (!Array.isArray(t.keyframes)) t.keyframes = [];
    if (t.keyframes.length && Number(t.keyframes[0].atMs) === 0) t.keyframes[0].value = exit[key];
    else t.keyframes.unshift({ atMs:0, value:exit[key] });
  }
  return newMotion;
}
```

**Step 4: Run — expect PASS. Step 5: Commit** (`feat: seedFirstKeyframesFromExit motion authoring default`).

---

## Task 6: Wire the transform into the bake path (`buildBakeLibrary`)

**Files:** Modify `servo_controller.html` (glue **outside** the region).

**Step 1: Read the bake orchestrator.** Locate the function(s) that call `sliceForBoard` for the real upload, the size preview (`renderBakeInventory`, ~3711), and the diff (~3796):
```
grep -n "sliceForBoard\|analyzeBakeConformance\|renderBakeInventory" servo_controller.html
```
Read each caller so the bridged library is threaded through consistently.

**Step 2: Add `buildBakeLibrary`** just after `sliceForBoard` (outside the region). It is the impure glue that supplies the real constants:
```js
// Bridge every sequence against the full motion library, producing an augmented
// library: bridge motions appended to `motions`, sequences rewritten with bridge
// steps. Pure fns live in SEQ-BRIDGE-CORE; this passes the real firmware limits.
function buildBakeLibrary(lib) {
  const sequences = [];
  const bridgeMotions = [];
  const summaries = [];
  const errors = [];
  for (const seq of (lib.sequences || [])) {
    const r = insertSequenceBridges(seq.steps || [], lib.motions || [], {
      seqId: seq.id,
      msPerPercent: SERVO_FEASIBILITY_MS_PER_PERCENT,
      minDeltaPercent: 5, floorMs: 800, unknownDeltaPercent: 100,
      maxSteps: SEQ_MAX_STEPS,
    });
    sequences.push({ ...seq, steps: r.steps });
    bridgeMotions.push(...r.bridgeMotions);
    for (const s of r.summary) summaries.push({ seqId: seq.id, seqName: seq.name || seq.id, ...s });
    if (r.error) errors.push({ seqId: seq.id, seqName: seq.name || seq.id, ...r.error });
  }
  return {
    ...lib,
    motions: [...(lib.motions || []), ...bridgeMotions],
    sequences,
    _bridgeSummaries: summaries,
    _bridgeErrors: errors,
  };
}
```

**Step 3: Thread it through the actual bake.** In the upload orchestrator, immediately after loading `lib`, compute `const bakeLib = buildBakeLibrary(lib);` then:
- If `bakeLib._bridgeErrors.length`, **block the bake**: for each error `bakeLog(\`Bridge overflow — sequence "${e.seqName}" needs ${e.produced} steps > ${e.max}. Shorten it or split it.\`, "red")` and return (same pattern as the `analyzeBakeConformance` error gate).
- Otherwise use `bakeLib` (not `lib`) for `sliceForBoard(bakeLib, boardId)`, `analyzeBakeConformance(bakeLib)`, and the size preview so estimates include bridges.

**Step 4: Verify no double-bridging.** Confirm `buildBakeLibrary` is called exactly once per bake and its output is never fed back into itself (bridge motions have `__bridge_` ids; `insertSequenceBridges` only bridges `MOTION` steps, and a bridge step *is* a MOTION step — but bridging runs on the **authored** `seq.steps`, never on already-bridged output, so this is safe as long as `buildBakeLibrary` receives the raw `lib`). Add a brief code comment stating this invariant.

**Step 5: Manual smoke via host oracle.** There is no DOM test here; instead confirm the pure path is exercised: `make -C test sim-verify` still passes.

**Step 6: Commit** (`feat: bridge sequences at bake time via buildBakeLibrary`).

---

## Task 7: Hide synth bridge motions from the motion-library UI

**Files:** Modify `servo_controller.html`.

**Step 1:** Find where the motion library list is rendered (`grep -n "motions" servo_controller.html` around the motion picker / inventory). Bridge motions only exist in the baked blob, not in the saved `lib.motions`, so the UI generally won't show them — **but** `renderBakeInventory` (~3711) iterates the bake blob and would list them. Filter them there:
```js
// skip auto-generated transition bridges (servo-bridge) — they are not user motions
if (String(m.id).startsWith("__bridge_")) continue;
```
Apply the same guard to any inventory/count that iterates `bakeLib.motions`.

**Step 2:** Verify the bake inventory count/labels look right (visual check in browser preview, Task 9). **Commit** (`chore: hide bridge motions from bake inventory`).

---

## Task 8: Show bridges in the Arrange timeline (visibility)

**Files:** Modify `servo_controller.html` (`renderSequenceArrangement`, ~5990; CSS ~1145).

**Step 1: Read** `renderSequenceArrangement` and `sequenceArrangementLayout` (~5912) to see how blocks are built from `seq.steps`.

**Step 2:** For **display only**, compute a bridged step list and render bridges as distinct dashed blocks:
```js
const motions = (sequenceLibrary().motions) || [];
const preview = insertSequenceBridges(seq.steps || [], motions, {
  seqId: seq.id, msPerPercent: SERVO_FEASIBILITY_MS_PER_PERCENT,
  minDeltaPercent: 5, floorMs: 800, unknownDeltaPercent: 100, maxSteps: SEQ_MAX_STEPS,
});
// render preview.steps; mark steps with `.bridge === true` using a dashed class
```
Give bridge blocks the existing `.seq-arrange-block.zero` dashed treatment plus a new `.bridge` modifier (amber, reduced opacity), label `↳ bridge <ms>ms`, and make them non-draggable / non-selectable (they aren't authored steps).

**Step 3:** If `preview.error`, surface a small inline warning in the Arrange header (e.g. "⚠ transitions exceed 16-step budget") so the operator sees it before baking.

**Step 4:** Add a `matches(...)` assertion in `verify_seq_bridges.mjs` for the new CSS rule (mirrors how `verify_seq_arrangement.mjs` checks `.seq-arrange-preview-row polyline`), e.g. a `.seq-arrange-block.bridge` rule exists.

**Step 5:** Run `make -C test sim-verify`. **Commit** (`feat: show transition bridges in the Arrange timeline`).

---

## Task 9: Bake-log summary + end-to-end verification

**Files:** Modify `servo_controller.html`; Modify `docs/TESTING.md`.

**Step 1: Bake-log lines.** In the bake orchestrator (Task 6), after a successful bridge build, log the summary:
```js
for (const s of bakeLib._bridgeSummaries) {
  const kind = s.kind === "cold" ? "cold-start" : "bridge";
  bakeLog(`↳ ${kind} in "${s.seqName}": ${s.channels.length} servo(s), ${s.durationMs}ms`, "amber");
}
if (bakeLib._bridgeSummaries.length) {
  const added = bakeLib._bridgeSummaries.reduce((n,s)=>n+s.durationMs,0);
  bakeLog(`${bakeLib._bridgeSummaries.length} bridge(s) inserted, +${(added/1000).toFixed(1)}s`, "amber");
}
```

**Step 2: Browser verification.** Use the preview browser tools:
- Open `servo_controller.html`, author a 2-motion sequence where motion A ends high and motion B starts low, drop both into Arrange.
- Confirm a dashed bridge block appears between them; confirm the bake log reports the bridge; confirm the bake inventory does **not** list any `__bridge_` motion.
- Read console for errors (`read_console_messages`).

**Step 3: Firmware manual test.** Use the `arduino-manual-testing` skill to add a test case to `docs/TESTING.md`: bake a sequence with a known A→B discontinuity, `RUN` it on hardware, and confirm the servo **glides** across the transition instead of snapping (and that a cold start from a raised position glides rather than yanks — the original incident). Record pass/fail.

**Step 4: Full host suite.** Run: `make -C test`  — expected: all green (firmware tests unchanged, `sim-verify` includes the new file).

**Step 5: Commit** (`feat: bake-log bridge summary + verification notes`).

---

## Task 10: Final review + branch finish

**Step 1:** `make -C test` green; `git status` clean.
**Step 2:** REQUIRED SUB-SKILL: Use superpowers:requesting-code-review to review the branch against this plan.
**Step 3:** Address findings (fresh commits).
**Step 4:** REQUIRED SUB-SKILL: Use superpowers:finishing-a-development-branch to choose merge/PR.

---

## Notes / deferred (from design doc)

- **Loop-seam bridging is out of scope** — `LOOP` is a runtime flag not known at bake. Follow-up: a per-sequence "loops" schema hint, then bridge last→first.
- **`unknownDeltaPercent` (cold-start duration)** defaults to 100% (~7.7s). Tunable in `buildBakeLibrary` if that feels too slow in practice.
- **DC tracks** are never bridged (speed, not position).
