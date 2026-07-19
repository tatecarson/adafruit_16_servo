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

console.log(`\n${passed} bake-payload checks passed`);
