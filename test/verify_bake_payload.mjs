// Regression checks for the EEPROM deploy representation. Extracts the pure
// planner/compactor from servo_controller.html and exercises a fixed authoring
// fixture without requiring a browser or depending on the operator's library.

import { readFileSync, writeFileSync, mkdtempSync } from "node:fs";
import { tmpdir } from "node:os";
import { join, dirname } from "node:path";
import { fileURLToPath, pathToFileURL } from "node:url";

const here = dirname(fileURLToPath(import.meta.url));
const root = join(here, "..");
const html = readFileSync(join(root, "servo_controller.html"), "utf8");

const tracks = (value0, value1) => [1, 2, 3].map(boardId => ({
  kind: "servo", boardId, channel: 0,
  label: `Board ${boardId} winch with deliberately verbose editor metadata`,
  keyframes: [{ atMs:0, value:value0 }, { atMs:1000, value:value1 }],
}));
const library = {
  schemaVersion: 1,
  motions: [
    { id:"rise", name:"A verbose human-readable rise name", tags:["calm", "gallery"],
      scope:"cluster", durationMs:1000, tracks:tracks(100, 90) },
    { id:"fall", name:"A verbose human-readable fall name", tags:["mechanical", "gallery"],
      scope:"cluster", durationMs:1000, tracks:tracks(10, 20) },
  ],
  sequences: [{
    id:"rise-fall", name:"Rise, then fall", tags:["test"],
    steps:[
      { cmd:"MOTION rise", durationMs:1000, target:"all", label:"first", hold:false },
      { cmd:"MOTION fall", durationMs:1000, target:"all", label:"second", hold:false },
    ],
  }],
  setlists: [{
    id:"show", name:"Test show", mode:"ordered",
    entries:[{ seqId:"rise-fall", repeat:1, gapMs:0, weight:1 }],
    shuffleRules:{ avoidSameTag:false, minGapEntries:0, moodArc:"random", seed:0 },
  }],
  activeSetlistId:"show",
  schedulerConfig:{ leaderBoardId:1, graceMs:10000, tagEnergy:["calm", "mechanical"] },
};

function block(start, end) {
  const a = html.indexOf(start);
  const b = html.indexOf(end, a + start.length);
  if (a < 0 || b <= a) throw new Error(`missing block ${start}`);
  return html.slice(a + start.length, b);
}

const bridgeCore = block("// === SEQ-BRIDGE-CORE START ===", "// === SEQ-BRIDGE-CORE END ===");
const machineCore = block("// === MACHINE-CORE START ===", "// === MACHINE-CORE END ===");
const dcLaneCore = machineCore + block("// === DC-LANE-CORE START ===", "// === DC-LANE-CORE END ===");
const payloadCore = block("// === BAKE-PAYLOAD-CORE START ===", "// === BAKE-PAYLOAD-CORE END ===");
const dir = mkdtempSync(join(tmpdir(), "bake-payload-core-"));
const modulePath = join(dir, "core.mjs");
writeFileSync(modulePath, `
const SCHEMA_VERSION = 1;
const SERVO_FEASIBILITY_MS_PER_PERCENT = 77;
const MOTION_SERVO_REST_PERCENT = 100;
const SEQ_MAX_STEPS = 16;
const STORAGE_DUAL_PAYLOAD_MAX = 4080;
const STORAGE_PAYLOAD_MAX = 6000;
function conformTrackForBake(track) { return track; }
${bridgeCore}
${dcLaneCore}
${payloadCore}
export { buildBakeLibrary, sliceForBoard, hydrateDeviceLibraryForEditor, validateLibraryReferences, bakeStorageTier };
`, "utf8");

const core = await import(pathToFileURL(modulePath).href);
let passed = 0;
function check(condition, message) {
  if (!condition) throw new Error(`FAIL: ${message}`);
  passed++;
  console.log(`PASS: ${message}`);
}
const bytes = value => Buffer.byteLength(JSON.stringify(value));

check(core.bakeStorageTier(4080).mode === "dual" && core.bakeStorageTier(4080).rollbackSafe,
  "4080-byte payload keeps rollback-safe dual-slot mode");
check(core.bakeStorageTier(4081).mode === "large" && !core.bakeStorageTier(4081).rollbackSafe,
  "4081-byte payload selects explicit no-rollback large mode");
check(core.bakeStorageTier(6001).mode === "over",
  "payload beyond 6000 bytes is rejected by the capacity tier");
check(html.includes("overwrites both rollback slots") && html.includes("not power-loss safe"),
  "large-mode confirmation names rollback and power-loss risks");
check(html.includes("large · no rollback") && html.includes("rollback safe"),
  "bake budget labels both storage modes visibly");

const baked = core.buildBakeLibrary(library);
check(baked.motions.length === library.motions.length,
  "pre-roll planning creates no hidden bridge Motions");
check(baked.sequences.every((seq, i) => seq.steps.length === library.sequences[i].steps.length),
  "pre-roll planning preserves every authored Sequence step count");
check(baked.sequences.some(seq => seq.steps.some(step => / PREP \d+$/.test(step.cmd))),
  "unsafe Motion entries are encoded as compact PREP commands");

const boardPayloads = [1, 2, 3].map(boardId => core.sliceForBoard(baked, boardId));
const boardBytes = boardPayloads.map(bytes);
check(boardBytes.every(size => size <= 4080),
  `compact fixture fits all boards (${boardBytes.join("/ ")} bytes)`);
check(boardPayloads.every(payload => payload.motions.every(m =>
  !Object.hasOwn(m, "name") && !Object.hasOwn(m, "tags") &&
  m.tracks.every(t => !Object.hasOwn(t, "label")))),
  "device Motions omit editor-only metadata");
check(boardPayloads.every(payload => payload.sequences.every(seq =>
  !Object.hasOwn(seq, "name") && seq.steps.every(step =>
    !Object.hasOwn(step, "label") && !Object.hasOwn(step, "hold")))),
  "device Sequences omit editor-only metadata and flags");
// The device compactors are allowlists, which is why a new authoring field
// costs the bake nothing. Pin that, so the next one added is free too.
check(boardPayloads.every(payload =>
  payload.motions.every(m => !Object.hasOwn(m, "machine")) &&
  payload.sequences.every(seq => !Object.hasOwn(seq, "machine"))),
  "the machine label never reaches the device");

// A board that only sits at rest in a motion does not get that motion baked to
// it — a curtain piece should not carry wand tracks commanding the wands to
// where they already are. But a flat track at a NON-rest value is a hold, and
// holds have to survive: "lower one arm, keep the other two up" is a real
// piece and dropping its flat tracks would let those arms fall.
const holdLib = structuredClone(library);
holdLib.motions = [{
  id: "hold", name: "one moves, one holds high, one idles", tags: [], scope: "cluster",
  durationMs: 1000,
  tracks: [
    { kind:"servo", boardId:1, channel:0, label:"B1.S0", keyframes:[{atMs:0,value:100},{atMs:1000,value:40}] },
    { kind:"servo", boardId:1, channel:1, label:"B1.S1", keyframes:[{atMs:0,value:50},{atMs:1000,value:50}] },
    { kind:"servo", boardId:3, channel:0, label:"B3.S0", keyframes:[{atMs:0,value:100},{atMs:1000,value:100}] },
  ],
}];
holdLib.sequences = [{ id:"s", name:"s", tags:[], steps:[{ cmd:"MOTION hold", durationMs:1000, target:"all" }] }];
holdLib.setlists = [{ id:"show", name:"", mode:"ordered", entries:[{seqId:"s",repeat:1,gapMs:0,weight:1}],
  shuffleRules:{avoidSameTag:false,minGapEntries:0,moodArc:"random",seed:0} }];
const holdBaked = core.buildBakeLibrary(holdLib);
check(core.sliceForBoard(holdBaked, 1).motions.length === 1,
  "a board that moves in a motion still gets it");
check(core.sliceForBoard(holdBaked, 1).motions[0].tracks.length === 2,
  "and keeps its flat hold track alongside the moving one");
check(core.sliceForBoard(holdBaked, 3).motions.length === 0,
  "a board that only sits at rest does not get the motion at all");

const hydrated = core.hydrateDeviceLibraryForEditor(boardPayloads[0]);
check(hydrated.motions.every(m => m.name && Array.isArray(m.tags) &&
  m.tracks.every(t => t.label)),
  "pulled device payload restores editor defaults");
check(hydrated.sequences.every(seq => seq.steps.every(step => !/ PREP \d+$/.test(step.cmd))),
  "pulled device payload removes deploy-only PREP syntax");

const dangling = structuredClone(library);
dangling.sequences[0].steps.push({ cmd:"MOTION deleted-motion", durationMs:1000 });
dangling.setlists[0].entries.push({ seqId:"deleted-sequence" });
dangling.activeSetlistId = "deleted-setlist";
const refs = core.validateLibraryReferences(dangling);
check(!refs.ok && refs.errors.some(e => e.code === "missing-motion") &&
  refs.errors.some(e => e.code === "missing-sequence") &&
  refs.errors.some(e => e.code === "missing-active-setlist"),
  "dangling Motion, Sequence, and active Setlist references are detected");

// servo-vp8: winch-direction compensation inverts servo values for reversed
// boards (board 3) at the device bake boundary, and undoes it on pull-back.
const b3rise = core.sliceForBoard(baked, 3).motions.find(m => m.id === "rise")
  .tracks.find(t => t.channel === 0);
check(b3rise.keyframes[0].value === 0 && b3rise.keyframes[1].value === 10,
  "reversed board 3 servo values are inverted at bake (100→0, 90→10)");
const b1rise = core.sliceForBoard(baked, 1).motions.find(m => m.id === "rise")
  .tracks.find(t => t.channel === 0);
check(b1rise.keyframes[0].value === 100 && b1rise.keyframes[1].value === 90,
  "non-reversed board 1 servo values are left untouched at bake");
const rehydrated = core.hydrateDeviceLibraryForEditor(core.sliceForBoard(baked, 3))
  .motions.find(m => m.id === "rise").tracks.find(t => t.channel === 0);
check(rehydrated.keyframes[0].value === 100 && rehydrated.keyframes[1].value === 90,
  "pull-back restores board 3's authored values (0→100, 10→90)");

console.log(`\n${passed} bake-payload checks passed`);
