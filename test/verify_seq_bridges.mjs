// Host tests for the pure sequence transition-bridge planning helpers
// embedded in servo_controller.html (servo-uyb).

import { readFileSync, writeFileSync, mkdtempSync } from "node:fs";
import { tmpdir } from "node:os";
import { join, dirname } from "node:path";
import { fileURLToPath, pathToFileURL } from "node:url";

const here = dirname(fileURLToPath(import.meta.url));
const html = readFileSync(join(here, "..", "servo_controller.html"), "utf8");
const START = "// === SEQ-BRIDGE-CORE START ===";
const END = "// === SEQ-BRIDGE-CORE END ===";

let passed = 0, failed = 0;
function fail(message) { console.error(`  FAIL: ${message}`); failed++; }
function ok(name) { console.log(`  PASS  ${name}`); passed++; }
function eq(name, got, want) {
  if (JSON.stringify(got) === JSON.stringify(want)) ok(name);
  else fail(`${name}\n    expected: ${JSON.stringify(want)}\n    got:      ${JSON.stringify(got)}`);
}
function matches(name, pattern) {
  if (pattern.test(html)) ok(name);
  else fail(`${name}\n    pattern not found: ${pattern}`);
}

const start = html.indexOf(START);
const end = html.indexOf(END, start + START.length);
if (start < 0 || end <= start) {
  fail("SEQ-BRIDGE-CORE markers not found or out of order");
  process.exit(1);
}
const core = html.slice(start + START.length, end);

// seedFirstKeyframesFromExit snaps its seed to the 5-grid (servo-3o6), so this
// block needs the SNAP-CORE helpers prepended to run standalone.
const SNAP_START = "// === SNAP-CORE START ===";
const SNAP_END = "// === SNAP-CORE END ===";
const ss = html.indexOf(SNAP_START);
const se = html.indexOf(SNAP_END, ss + SNAP_START.length);
if (ss < 0 || se <= ss) { fail("SNAP-CORE markers not found or out of order"); process.exit(1); }
const snapCore = html.slice(ss + SNAP_START.length, se);

const dir = mkdtempSync(join(tmpdir(), "seq-bridge-core-"));
const modPath = join(dir, "core.mjs");
writeFileSync(modPath, `${snapCore}${core}\nexport { bridgeServoPose, planColdStartBridge, planInteriorBridge, insertSequenceBridges, rewriteSequencePreRolls, expandPreRollStepsForDisplay, describePreRollTravel, seedFirstKeyframesFromExit, stripBridges, validateLibraryReferences };\n`, "utf8");
const { bridgeServoPose, planColdStartBridge, planInteriorBridge, insertSequenceBridges, rewriteSequencePreRolls, expandPreRollStepsForDisplay, describePreRollTravel, seedFirstKeyframesFromExit, stripBridges, validateLibraryReferences } = await import(pathToFileURL(modPath).href);

console.log("=== Sequence transition bridges ===");

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

// Cold start is measured from the rig's rest pose (100%, fully down), not
// worst-cased. Entries at 30 and 90 are 70 and 10 away from rest, so the
// cluster-wide glide is sized by the 70 and lands at 5390ms, not 7700ms.
const cs = planColdStartBridge({"1:servo:0":30,"2:servo:2":90},
  { msPerPercent:77, floorMs:800, restPercent:100 });
eq("cold-start duration is measured from the rest pose", cs.durationMs, Math.ceil(70*77)); // 5390
eq("one DMOVE per driven servo, targeted at its board", cs.steps.slice(0,2).map(s=>[s.cmd,s.target]),
   [["DMOVE 0 30 5390","1"],["DMOVE 2 90 5390","2"]]);
eq("DMOVE steps are zero-duration (chord)", cs.steps.slice(0,2).every(s=>s.durationMs===0), true);
eq("trailing dwell step holds the clock", [cs.steps.at(-1).cmd, cs.steps.at(-1).durationMs], ["",5390]);
eq("empty pose yields no cold-start bridge", planColdStartBridge({}, {}), null);

// The point of the whole change: a motion that opens on the rest pose is
// already in position at power-on, so it needs no cold-start bridge at all.
eq("motion entering at the rest pose needs no cold-start bridge",
   planColdStartBridge({"1:servo:0":100,"2:servo:2":100}, { msPerPercent:77, floorMs:800, restPercent:100 }), null);
eq("sub-slop distance from rest does not bridge",
   planColdStartBridge({"1:servo:0":97}, { msPerPercent:77, floorMs:800, restPercent:100, minDeltaPercent:5 }), null);

// A re-run does not start from rest — it starts wherever the previous run
// parked. Size for whichever start is further away so neither is under-timed.
eq("re-run pose lengthens the glide when it is further than rest",
   planColdStartBridge({"1:servo:0":100}, { msPerPercent:77, floorMs:800, restPercent:100,
     rerunFromPose:{"1:servo:0":20} }).durationMs, Math.ceil(80*77)); // 6160
eq("rest still wins when it is the further start",
   planColdStartBridge({"1:servo:0":0}, { msPerPercent:77, floorMs:800, restPercent:100,
     rerunFromPose:{"1:servo:0":10} }).durationMs, Math.ceil(100*77)); // 7700
eq("a channel absent from the re-run pose falls back to rest",
   planColdStartBridge({"1:servo:0":0}, { msPerPercent:77, floorMs:800, restPercent:100,
     rerunFromPose:{"9:servo:9":50} }).durationMs, Math.ceil(100*77));

const lib = [
  { id:"rise", durationMs:1000, tracks:[{kind:"servo",boardId:1,channel:0,keyframes:[{atMs:0,value:0},{atMs:1000,value:80}]}] },
  { id:"fall", durationMs:1000, tracks:[{kind:"servo",boardId:1,channel:0,keyframes:[{atMs:0,value:0},{atMs:1000,value:0}]}] },
];
const wopts = { seqId:"s", msPerPercent:77, minDeltaPercent:5, floorMs:800, restPercent:100, maxSteps:16 };

const r = insertSequenceBridges([{cmd:"MOTION rise",durationMs:1000},{cmd:"MOTION fall",durationMs:1000}], lib, wopts);
eq("cold-start bridge precedes first motion",
   [r.steps[0].cmd.startsWith("DMOVE"), r.steps[1].cmd, r.steps[2].cmd],
   [true, "", "MOTION rise"]);
eq("interior bridge inserted before fall",
   r.steps.slice(3).map(s=>s.cmd), ["MOTION __bridge_s_0","MOTION fall"]);
eq("one synth bridge motion produced", r.bridgeMotions.map(m=>m.id), ["__bridge_s_0"]);
eq("no error under budget", r.error, null);

// Real bakes use compact PREP annotations, not the legacy materialized bridge
// objects above. The old planner remains covered for backwards hydration and
// display math, while these assertions guard the deploy representation.
const compact = rewriteSequencePreRolls(
  [{cmd:"MOTION rise",durationMs:1000},{cmd:"MOTION fall",durationMs:1000}], lib, wopts);
eq("compact planner preserves authored step count", compact.steps.length, 2);
eq("cold transition becomes an in-step PREP command", compact.steps[0].cmd, "MOTION rise PREP 7700");
eq("PREP duration extends the authored play window", compact.steps[0].durationMs, 8700);
eq("interior transition is also in-step", compact.steps[1].cmd, "MOTION fall PREP 6160");
eq("compact planner produces summaries but no hidden Motions", compact.summary.map(s=>s.kind), ["cold","interior"]);
const displayed = expandPreRollStepsForDisplay(compact.steps);
eq("Arrange expansion shows two display-only PREP blocks", displayed.filter(s=>s.bridge).length, 2);
eq("Arrange expansion restores original Motion durations", displayed.filter(s=>!s.bridge).map(s=>s.durationMs), [1000,1000]);

// --- servo-cjv: per-step pre-roll opt-out ---
// A pre-roll is generated, never authored, so "delete this one" is recorded as
// a refusal (`noPrep`) on the step it would have preceded.
const optOut = rewriteSequencePreRolls(
  [{cmd:"MOTION rise",durationMs:1000,noPrep:true},{cmd:"MOTION fall",durationMs:1000}], lib, wopts);
eq("noPrep step keeps its bare MOTION command", optOut.steps[0].cmd, "MOTION rise");
eq("noPrep step gains no pre-roll duration", optOut.steps[0].durationMs, 1000);
// The trap: skipping the pre-roll must NOT skip the exit-pose bookkeeping, or
// the next motion's pre-roll gets planned from a stale position and mis-sized.
eq("following pre-roll is still sized from the skipped motion's exit pose",
   optOut.steps[1].cmd, "MOTION fall PREP 6160");
eq("disabled pre-roll is reported with the glide it gave up",
   optOut.summary.map(s => [s.kind, s.durationMs, s.disabled === true]),
   [["cold",7700,true],["interior",6160,false]]);

// Arrange has to keep drawing something clickable, or switching a pre-roll off
// would make it unrecoverable. The summary carries the ghost.
const displayedOptOut = expandPreRollStepsForDisplay(optOut.steps, optOut.summary);
eq("disabled pre-roll still draws a block in Arrange",
   displayedOptOut.filter(s => s.bridge).length, 2);
eq("the disabled one is a zero-duration ghost, the live one keeps its time",
   displayedOptOut.filter(s => s.bridge).map(s => [s.durationMs, s.disabled === true]),
   [[0,true],[6160,false]]);
eq("ghost carries the would-be glide for the operator warning",
   displayedOptOut.find(s => s.disabled)?.wouldBeMs, 7700);
eq("ghost is pinned to the step it belongs to", displayedOptOut[0].bridge, true);

// Summary must stay index-correct when a NON-motion step sits in the middle,
// since stepIndex is what pins a ghost to its block.
const mixed = rewriteSequencePreRolls(
  [{cmd:"STOP",durationMs:500},{cmd:"MOTION rise",durationMs:1000,noPrep:true}], lib, wopts);
eq("summary stepIndex points at the authored step, not the motion count",
   mixed.summary.map(s => s.stepIndex), [1]);
eq("ghost lands after the non-motion step",
   expandPreRollStepsForDisplay(mixed.steps, mixed.summary).map(s => s.cmd),
   ["STOP", "", "MOTION rise"]);

// Turning every pre-roll off must cost nothing at bake: no PREP, no padding.
const allOff = rewriteSequencePreRolls(
  [{cmd:"MOTION rise",durationMs:1000,noPrep:true},{cmd:"MOTION fall",durationMs:1000,noPrep:true}],
  lib, wopts);
eq("all pre-rolls off bakes the authored steps verbatim",
   allOff.steps.map(s => [s.cmd, s.durationMs]),
   [["MOTION rise",1000],["MOTION fall",1000]]);

// Absent `noPrep` must behave exactly as before — no migration for saved work.
eq("steps without noPrep are unchanged",
   rewriteSequencePreRolls([{cmd:"MOTION rise",durationMs:1000}], lib, wopts).steps[0].cmd,
   "MOTION rise PREP 7700");

// --- servo-rvn: the plan must say which winches travel, and from where ---
// The planners already know every endpoint; they used to report only channel
// keys, which is why a PREP block could say "7700ms" and nothing else.
{
  const moved = planInteriorBridge(
    { "1:servo:0": 20, "1:servo:1": 90, "1:servo:2": 50 },
    { "1:servo:0": 70, "1:servo:1": 40, "1:servo:2": 52 },
    "__x", wopts);
  eq("interior plan reports both endpoints per channel",
     moved.moves.map(m => [m.label, m.from, m.to]),
     [["B1.S0", 20, 70], ["B1.S1", 90, 40]]);
  eq("a channel under the movement threshold is not reported as travelling",
     moved.moves.some(m => m.label === "B1.S2"), false);

  // Cold start sizes against the worse of two plausible starts: a fresh
  // power-on at rest, or an immediate re-run from where the sequence parked.
  // The reported `from` has to be whichever one it actually sized against, or
  // the explanation would name a distance the operator is not being charged.
  // Entry 90 is only 10 from rest but 80 from where a re-run would start, so
  // the re-run is the start that sizes this glide — and the one to report.
  const cold = planColdStartBridge({ "1:servo:0": 90 },
    { ...wopts, rerunFromPose: { "1:servo:0": 10 } });
  eq("cold plan reports the start it sized against, not just the rest pose",
     cold.moves.map(m => [m.label, m.from, m.to]), [["B1.S0", 10, 90]]);
  eq("and sizes the glide from that same distance", cold.durationMs, 80 * 77);
  // When rest is the further start, rest is what gets reported.
  eq("rest is reported when rest is what sized the glide",
     planColdStartBridge({ "1:servo:0": 40 },
       { ...wopts, rerunFromPose: { "1:servo:0": 10 } }).moves.map(m => [m.from, m.to]),
     [[100, 40]]);

  const fromRest = planColdStartBridge({ "1:servo:0": 40 }, wopts);
  eq("with no re-run history it reports the rest pose",
     fromRest.moves.map(m => [m.label, m.from, m.to]), [["B1.S0", 100, 40]]);
}

// The moves ride along in the summary so Arrange can explain a pre-roll it is
// drawing — including one that has been switched off.
{
  const r = rewriteSequencePreRolls([{cmd:"MOTION rise",durationMs:1000}], lib, wopts);
  eq("summary carries the per-channel moves",
     r.summary[0].moves.map(m => [m.label, m.from, m.to]), [["B1.S0", 100, 0]]);
  const offSummary = rewriteSequencePreRolls(
    [{cmd:"MOTION rise",durationMs:1000,noPrep:true}], lib, wopts).summary[0];
  eq("a refused pre-roll still reports what it would have moved",
     offSummary.moves.map(m => [m.label, m.from, m.to]), [["B1.S0", 100, 0]]);
  eq("and the display ghost carries them through",
     expandPreRollStepsForDisplay(
       rewriteSequencePreRolls([{cmd:"MOTION rise",durationMs:1000,noPrep:true}], lib, wopts).steps,
       [offSummary])[0].moves.length, 1);
}

// --- describePreRollTravel: the plain-language line the block shows ---------
// 0% is fully UP and 100% is fully down, so a RISING value is travel DOWN.
// Getting this backwards in the UI would be worse than saying nothing.
{
  const d = describePreRollTravel;
  eq("a rising value reads as travelling down",
     d([{label:"B1.S0", from:0, to:100}]).headline, "B1.S0 down to 100%");
  eq("a falling value reads as travelling up",
     d([{label:"B1.S0", from:100, to:0}]).headline, "B1.S0 up to 0%");
  eq("several winches to a shared target are counted",
     d([{label:"B1.S0",from:100,to:40},{label:"B1.S1",from:100,to:40}]).headline,
     "2 winches up to 40%");
  eq("a spread of targets is shown as a range",
     d([{label:"B1.S0",from:100,to:96},{label:"B1.S1",from:100,to:90}]).headline,
     "2 winches up to 90–96%");
  // A channel whose endpoints match isn't travelling, so it is not counted.
  eq("a channel that does not actually move is left out",
     d([{label:"B1.S0",from:100,to:96},{label:"B1.S1",from:100,to:100}]).count, 1);
  eq("mixed directions do not claim a direction",
     d([{label:"B1.S0",from:0,to:50},{label:"B1.S1",from:100,to:50}]).headline,
     "2 winches to 50%");
  eq("nothing moving says so", d([]).headline, "nothing to move");
  eq("rows list every channel for the tooltip",
     d([{label:"B1.S0",from:100,to:40},{label:"B1.S1",from:90,to:40}]).rows,
     ["B1.S0 100% → 40%", "B1.S1 90% → 40%"]);
  eq("the count is the number of winches that actually travel",
     d([{label:"B1.S0",from:100,to:40}]).count, 1);
}

// --- servo-3o6 round trip: snapping removes the pre-roll at the source ---
// Two motions whose exit and entry both land on 40 leave a delta of 0, which
// falls under the 5% interior threshold. This is the whole point of the
// 5-grid: no opt-out needed because no pre-roll is planned.
const alignedLib = [
  { id:"a", durationMs:1000, tracks:[{kind:"servo",boardId:1,channel:0,
    keyframes:[{atMs:0,value:100},{atMs:1000,value:40}]}] },
  { id:"b", durationMs:1000, tracks:[{kind:"servo",boardId:1,channel:0,
    keyframes:[{atMs:0,value:40},{atMs:1000,value:100}]}] },
];
const aligned = rewriteSequencePreRolls(
  [{cmd:"MOTION a",durationMs:1000},{cmd:"MOTION b",durationMs:1000}], alignedLib, wopts);
eq("motions meeting on a snapped value need no interior pre-roll",
   aligned.steps[1].cmd, "MOTION b");

// Seeding a new Motion from the previous one's exit is the other way a first
// keyframe gets set. An off-grid legacy exit must not seed an off-grid entry:
// the drift it introduces (at most 2.5%) stays well under the 5% threshold, so
// it still costs no pre-roll, and the new Motion starts life on the grid.
{
  const prev = { id:"p", durationMs:1000, tracks:[{kind:"servo",boardId:1,channel:0,
    keyframes:[{atMs:0,value:0},{atMs:1000,value:43}]}] };
  const fresh = { id:"n", durationMs:1000, tracks:[{kind:"servo",boardId:1,channel:0,
    keyframes:[{atMs:0,value:0},{atMs:1000,value:80}]}] };
  eq("seeded first keyframe lands on the grid",
     seedFirstKeyframesFromExit(prev, fresh).tracks[0].keyframes[0].value, 45);
}

// A no-movement transition (fall holds 0 -> next fall entry 0) needs no bridge:
// planInteriorBridge returns null when nothing moves past the slop threshold.
const r2 = insertSequenceBridges([{cmd:"MOTION fall",durationMs:1000},{cmd:"MOTION fall",durationMs:1000}], lib, wopts);
eq("no-movement transition inserts no interior bridge", r2.bridgeMotions.length, 0);

// End to end: a sequence whose motions open and close on the rest pose is
// already in position at power-on and after a re-run, so it bakes with no
// cold-start bridge at all — the timeline opens directly on the motion.
const restLib = [
  { id:"parked", durationMs:1000, tracks:[{kind:"servo",boardId:1,channel:0,
    keyframes:[{atMs:0,value:100},{atMs:1000,value:100}]}] },
];
const rRest = insertSequenceBridges([{cmd:"MOTION parked",durationMs:1000}], restLib, wopts);
eq("sequence starting at rest bakes with no cold-start bridge",
   rRest.steps.map(s=>s.cmd), ["MOTION parked"]);
eq("and reports no cold-start in the summary", rRest.summary.length, 0);

// But if that same sequence ends somewhere else, replaying it no longer starts
// from rest, so the cold-start bridge has to come back sized for the re-run.
const driftLib = [
  { id:"drift", durationMs:1000, tracks:[{kind:"servo",boardId:1,channel:0,
    keyframes:[{atMs:0,value:100},{atMs:1000,value:20}]}] },
];
const rDrift = insertSequenceBridges([{cmd:"MOTION drift",durationMs:1000}], driftLib, wopts);
eq("a sequence that drifts away from its entry keeps a cold-start bridge",
   [rDrift.steps[0].cmd, rDrift.summary[0].kind], ["DMOVE 0 100 6160", "cold"]);

const r3 = insertSequenceBridges([{cmd:"MOTION rise",durationMs:1000},{cmd:"STOP",durationMs:500},{cmd:"MOTION fall",durationMs:1000}], lib, wopts);
eq("opaque step passes through untouched", r3.steps.find(s=>s.cmd==="STOP")!=null, true);
eq("motion after opaque step gets a DMOVE cold-start bridge",
   r3.steps.filter(s=>s.cmd.startsWith("DMOVE")).length>0, true);

const many = Array.from({length:20},()=>({cmd:"MOTION rise",durationMs:1000}));
eq("over-budget sequence reports an error", insertSequenceBridges(many, lib, wopts).error.code, "bridge-budget");

// --- Task 5: seedFirstKeyframesFromExit ---
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

// --- code-review follow-ups ---

// Merge regression (Fix 1): s0 set by motion "a", NOT driven by "b", must still
// be bridged from 90 -> 0 at motion "c" (not forgotten / snapped).
const libM = [
  { id:"a", tracks:[{kind:"servo",boardId:1,channel:0,keyframes:[{atMs:0,value:0},{atMs:1000,value:90}]}] },
  { id:"b", tracks:[{kind:"servo",boardId:1,channel:1,keyframes:[{atMs:0,value:0},{atMs:1000,value:60}]}] },
  { id:"c", tracks:[{kind:"servo",boardId:1,channel:0,keyframes:[{atMs:0,value:0},{atMs:1000,value:0}]}] },
];
const rm = insertSequenceBridges([{cmd:"MOTION a"},{cmd:"MOTION b"},{cmd:"MOTION c"}], libM, wopts);
const cS0 = rm.bridgeMotions.map(m=>m.tracks.find(t=>t.channel===0&&t.boardId===1)).filter(Boolean)
  .find(t=>t.keyframes[0].value===90);
eq("s0 remembered across an intervening motion that doesn't drive it", cS0 && cS0.keyframes.map(k=>k.value), [90,0]);

// Multi-servo interior bridge covers every moving servo.
eq("interior bridge covers every moving servo",
   planInteriorBridge({"1:servo:0":80,"1:servo:1":80},{"1:servo:0":0,"1:servo:1":0},"x",{}).channels.length, 2);

// DC track passes through the walker untouched (never bridged).
const libDc = [{ id:"d", tracks:[
  {kind:"servo",boardId:1,channel:0,keyframes:[{atMs:0,value:0},{atMs:1000,value:50}]},
  {kind:"dc",   boardId:1,channel:0,keyframes:[{atMs:0,value:0},{atMs:1000,value:70}]},
]}];
eq("walker never emits a DC bridge track",
   insertSequenceBridges([{cmd:"MOTION d"},{cmd:"MOTION d"}], libDc, wopts)
     .bridgeMotions.every(m=>m.tracks.every(t=>t.kind==="servo")), true);

// Budget boundary: exactly maxSteps ok, one over errors (opaque steps, no bridges).
const nops = Array.from({length:5},()=>({cmd:"NOP",durationMs:0}));
eq("exactly maxSteps is allowed", insertSequenceBridges(nops, [], {maxSteps:5}).error, null);
eq("one over maxSteps errors", insertSequenceBridges(nops, [], {maxSteps:4}).error.code, "bridge-budget");

// MOTION id lookup is case-insensitive (recognized as a motion -> cold-start DMOVE emitted).
eq("MOTION id lookup is case-insensitive",
   insertSequenceBridges([{cmd:"MOTION RISE"}], lib, wopts).steps.some(s=>s.cmd.startsWith("DMOVE")), true);

// A MOTION step with extra tokens does not match -> treated as opaque (no bridge).
eq("MOTION step with extra args is opaque",
   insertSequenceBridges([{cmd:"MOTION rise extra"}], lib, wopts).steps.filter(s=>s.cmd.startsWith("DMOVE")).length, 0);

// --- stripBridges (Task 7): undo bridges when a library is pulled/imported back ---
const baked = {
  motions: [
    { id:"real", tracks:[] },
    { id:"__bridge_sq-1_0", tracks:[] },
  ],
  sequences: [ { id:"sq-1", steps:[
    { cmd:"DMOVE 0 0 7700", bridge:true },
    { cmd:"", bridge:true },
    { cmd:"MOTION real" },
    { cmd:"MOTION __bridge_sq-1_0", bridge:true },
    { cmd:"MOTION real2" },
  ] } ],
  setlists: [{ id:"s" }],
};
const stripped = stripBridges(baked);
eq("bridge motions removed", stripped.motions.map(m=>m.id), ["real"]);
eq("bridge steps removed, authored steps kept", stripped.sequences[0].steps.map(s=>s.cmd), ["MOTION real","MOTION real2"]);
eq("interior bridge caught even without the flag",
   stripBridges({ sequences:[{steps:[{cmd:"MOTION __bridge_x_0"},{cmd:"MOTION keep"}]}] }).sequences[0].steps.map(s=>s.cmd), ["MOTION keep"]);
eq("other library fields preserved", stripped.setlists.map(s=>s.id), ["s"]);
eq("null library is a safe no-op", stripBridges(null), null);
eq("source library object is not mutated", baked.motions.length, 2);

const preparedBake = { motions:[{id:"rise"}], sequences:[{id:"s",steps:[
  {cmd:"MOTION rise PREP 6160",durationMs:7160},
]}], setlists:[] };
const unprepared = stripBridges(preparedBake);
eq("pulled PREP command returns to an authored Motion step", unprepared.sequences[0].steps[0].cmd, "MOTION rise");
eq("pulled PREP duration is removed from authored duration", unprepared.sequences[0].steps[0].durationMs, 1000);

eq("valid references pass the bake gate", validateLibraryReferences({
  motions:[{id:"m"}], sequences:[{id:"q",steps:[{cmd:"MOTION m"}]}],
  setlists:[{id:"sl",entries:[{seqId:"q"}]}], activeSetlistId:"sl",
}).ok, true);
eq("missing Motion reference is named", validateLibraryReferences({
  motions:[], sequences:[{id:"q",steps:[{cmd:"MOTION ghost"}]}], setlists:[],
}).errors[0], {code:"missing-motion",owner:"q",index:0,missing:"ghost"});
eq("missing Sequence reference is named", validateLibraryReferences({
  motions:[], sequences:[], setlists:[{id:"sl",entries:[{seqId:"ghost"}]}],
}).errors[0], {code:"missing-sequence",owner:"sl",index:0,missing:"ghost"});

// --- Task 8: bridges are visible in the Arrange timeline (display-only) ---
// The Arrange render tags auto-inserted bridge blocks with a distinct style so
// operators see them before baking.
matches("Arrange timeline has a .seq-arrange-block.bridge style rule",
  /\.seq-arrange-block\.bridge\s*\{/s);

console.log(`\n${passed} passed, ${failed} failed`);
process.exit(failed ? 1 : 0);
