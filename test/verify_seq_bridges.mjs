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
writeFileSync(modPath, `${core}\nexport { bridgeServoPose, planColdStartBridge, planInteriorBridge, insertSequenceBridges, seedFirstKeyframesFromExit };\n`, "utf8");
const { bridgeServoPose, planColdStartBridge, planInteriorBridge, insertSequenceBridges, seedFirstKeyframesFromExit } = await import(pathToFileURL(modPath).href);

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

console.log(`\n${passed} passed, ${failed} failed`);
process.exit(failed ? 1 : 0);
