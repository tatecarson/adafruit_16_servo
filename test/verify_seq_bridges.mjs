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

const start = html.indexOf(START);
const end = html.indexOf(END, start + START.length);
if (start < 0 || end <= start) {
  fail("SEQ-BRIDGE-CORE markers not found or out of order");
  process.exit(1);
}
const core = html.slice(start + START.length, end);
const dir = mkdtempSync(join(tmpdir(), "seq-bridge-core-"));
const modPath = join(dir, "core.mjs");
writeFileSync(modPath, `${core}\nexport { bridgeServoPose, planColdStartBridge, planInteriorBridge, insertSequenceBridges, seedFirstKeyframesFromExit, stripBridges };\n`, "utf8");
const { bridgeServoPose, planColdStartBridge, planInteriorBridge, insertSequenceBridges, seedFirstKeyframesFromExit, stripBridges } = await import(pathToFileURL(modPath).href);

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

const cs = planColdStartBridge({"1:servo:0":30,"2:servo:2":90},
  { msPerPercent:77, floorMs:800, unknownDeltaPercent:100 });
eq("cold-start duration uses the conservative unknown delta", cs.durationMs, Math.ceil(100*77)); // 7700
eq("one DMOVE per driven servo, targeted at its board", cs.steps.slice(0,2).map(s=>[s.cmd,s.target]),
   [["DMOVE 0 30 7700","1"],["DMOVE 2 90 7700","2"]]);
eq("DMOVE steps are zero-duration (chord)", cs.steps.slice(0,2).every(s=>s.durationMs===0), true);
eq("trailing dwell step holds the clock", [cs.steps.at(-1).cmd, cs.steps.at(-1).durationMs], ["",7700]);
eq("empty pose yields no cold-start bridge", planColdStartBridge({}, {}), null);

const lib = [
  { id:"rise", durationMs:1000, tracks:[{kind:"servo",boardId:1,channel:0,keyframes:[{atMs:0,value:0},{atMs:1000,value:80}]}] },
  { id:"fall", durationMs:1000, tracks:[{kind:"servo",boardId:1,channel:0,keyframes:[{atMs:0,value:0},{atMs:1000,value:0}]}] },
];
const wopts = { seqId:"s", msPerPercent:77, minDeltaPercent:5, floorMs:800, unknownDeltaPercent:100, maxSteps:16 };

const r = insertSequenceBridges([{cmd:"MOTION rise",durationMs:1000},{cmd:"MOTION fall",durationMs:1000}], lib, wopts);
eq("cold-start bridge precedes first motion",
   [r.steps[0].cmd.startsWith("DMOVE"), r.steps[1].cmd, r.steps[2].cmd],
   [true, "", "MOTION rise"]);
eq("interior bridge inserted before fall",
   r.steps.slice(3).map(s=>s.cmd), ["MOTION __bridge_s_0","MOTION fall"]);
eq("one synth bridge motion produced", r.bridgeMotions.map(m=>m.id), ["__bridge_s_0"]);
eq("no error under budget", r.error, null);

// A no-movement transition (fall holds 0 -> next fall entry 0) needs no bridge:
// planInteriorBridge returns null when nothing moves past the slop threshold.
const r2 = insertSequenceBridges([{cmd:"MOTION fall",durationMs:1000},{cmd:"MOTION fall",durationMs:1000}], lib, wopts);
eq("no-movement transition inserts no interior bridge", r2.bridgeMotions.length, 0);

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

console.log(`\n${passed} passed, ${failed} failed`);
process.exit(failed ? 1 : 0);
