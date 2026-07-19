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

console.log(`\n${passed} passed, ${failed} failed`);
process.exit(failed ? 1 : 0);
