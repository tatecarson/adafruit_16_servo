// Schema v2 wire format (servo-zzo).
//
// v1 spends most of a bake on repeated key names. The worst offender is the
// keyframe: {"atMs":1000,"value":90} is 24 bytes to carry two numbers. At
// 79 keyframes across the three board slices that is most of a kilobyte in
// punctuation.
//
// v2 keeps the payload valid JSON — the firmware's bounded scanner, the CRC
// record, the storage tiers and pull-back-to-editor all survive — and does
// three things: positional keyframes, one-character structural keys, and
// dropping two fields that are implied by where they sit.
//
// Short keys are only safe because bakeFindValue matches a key at depth 0 of
// the window it is scanning, so a nested object cannot shadow an outer key.
// That means keys need to be unique WITHIN an object level, not globally —
// which is why `g` can be gapMs in an entry and graceMs in schedulerConfig.
// These tests pin that per-level uniqueness, because a collision inside one
// level is silent and would hand the firmware the wrong number.
//
// Run: node verify_bake_v2.mjs   (or via test/run-all.mjs)

import { readFileSync, writeFileSync, mkdtempSync } from "node:fs";
import { tmpdir } from "node:os";
import { join, dirname } from "node:path";
import { fileURLToPath, pathToFileURL } from "node:url";

const here = dirname(fileURLToPath(import.meta.url));
const root = join(here, "..");
const html = readFileSync(join(root, "servo_controller.html"), "utf8");

function block(start, end) {
  const a = html.indexOf(start);
  const b = html.indexOf(end, a + start.length);
  if (a < 0 || b <= a) throw new Error(`missing block ${start}`);
  return html.slice(a + start.length, b);
}

const dir = mkdtempSync(join(tmpdir(), "bake-v2-"));
const modulePath = join(dir, "core.mjs");
writeFileSync(modulePath, `
const SCHEMA_VERSION = 1;      // authoring (library.json)
const DEVICE_SCHEMA_VERSION = 2;  // the wire format a board receives
const SERVO_FEASIBILITY_MS_PER_PERCENT = 77;
const MOTION_SERVO_REST_PERCENT = 100;
const SEQ_MAX_STEPS = 16;
const STORAGE_DUAL_PAYLOAD_MAX = 4080;
const STORAGE_PAYLOAD_MAX = 6000;
function conformTrackForBake(track) { return track; }
${block("// === MACHINE-CORE START ===", "// === MACHINE-CORE END ===")}
${block("// === SEQ-BRIDGE-CORE START ===", "// === SEQ-BRIDGE-CORE END ===")}
${block("// === DC-LANE-CORE START ===", "// === DC-LANE-CORE END ===")}
${block("// === BAKE-PAYLOAD-CORE START ===", "// === BAKE-PAYLOAD-CORE END ===")}
export { buildBakeLibrary, sliceForBoard, hydrateDeviceLibraryForEditor, expandDeviceBlob, diffBlob, BAKE_V2_KEYS };
`, "utf8");
const core = await import(pathToFileURL(modulePath).href);

let passed = 0;
function check(condition, message) {
  if (!condition) throw new Error(`FAIL: ${message}`);
  passed++;
  console.log(`PASS: ${message}`);
}
const eq = (actual, expected, message) =>
  check(JSON.stringify(actual) === JSON.stringify(expected),
    `${message} (got ${JSON.stringify(actual)}, expected ${JSON.stringify(expected)})`);

// --- The key table is the contract with the firmware --------------------
// bake_parse.h callers hard-code these same strings. If this table changes
// without the firmware changing, a board silently reads defaults for every
// field whose name moved.
const K = core.BAKE_V2_KEYS;

check(K, "the browser exports the v2 key table");

// The bug this catches: the page declared schemaVersion 1 while emitting v2
// keys, because SCHEMA_VERSION was doing double duty as both the authoring
// version and the wire version. The harness stubbed the constant, so nothing
// noticed until a board was baked. Read the real values out of the source.
const authoring = /^const SCHEMA_VERSION = (\d+);/m.exec(html);
const device = /^const DEVICE_SCHEMA_VERSION = (\d+);/m.exec(html);
check(authoring && authoring[1] === "1",
  "the page keeps authoring at schemaVersion 1 (library.json is unchanged by v2)");
check(device && device[1] === "2",
  "the page stamps board payloads with DEVICE_SCHEMA_VERSION 2");
check(/schemaVersion: DEVICE_SCHEMA_VERSION/.test(html),
  "sliceForBoard stamps the device version, not the authoring one — a v2 blob "
  + "declaring v1 would be accepted by v1 firmware and silently read as empty");

eq(K.motion, { id:"i", durationMs:"d", tracks:"r" }, "motion keys");
eq(K.track, { channel:"c", keyframes:"k" }, "track keys");
eq(K.sequence, { id:"i", steps:"s" }, "sequence keys");
eq(K.step, { cmd:"c", durationMs:"d", target:"t" }, "step keys");
eq(K.setlist, { id:"i", entries:"e", mode:"o", shuffleRules:"u" }, "setlist keys");
eq(K.entry, { seqId:"q", repeat:"p", gapMs:"g", weight:"w" }, "entry keys");
eq(K.shuffleRules, { minGapEntries:"n", seed:"s" }, "shuffleRules keys");
eq(K.schedulerConfig, { leaderBoardId:"b", graceMs:"g" }, "schedulerConfig keys");
eq(K.root, { motions:"m", sequences:"q", setlists:"l", activeSetlistId:"a",
             schedulerConfig:"g" }, "root keys");

// The whole scheme rests on this. A duplicate inside one level makes the
// firmware's scanner return whichever came first, silently.
for (const [level, table] of Object.entries(K)) {
  const short = Object.values(table);
  check(new Set(short).size === short.length,
    `${level} keys are unique within their own object`);
}

// schemaVersion must NOT be shortened: bake_validate.h looks for that literal
// key to decide whether it will accept the blob at all, and it is how the
// firmware tells a v1 payload from a v2 one.
check(!Object.values(K).some(t => "schemaVersion" in t),
  "schemaVersion is never abbreviated");

// --- What actually goes on the wire -------------------------------------

const fixture = {
  schemaVersion: 2,
  motions: [{
    id: "rise", name: "editor-only name", tags: ["calm"], scope: "cluster",
    durationMs: 2000,
    tracks: [
      { kind:"servo", boardId:1, channel:0, label:"editor-only label",
        keyframes: [{ atMs:0, value:100 }, { atMs:2000, value:0 }] },
      { kind:"servo", boardId:2, channel:5,
        keyframes: [{ atMs:0, value:0 }, { atMs:2000, value:50 }] },
    ],
  }],
  sequences: [{
    id: "show", name: "editor-only", tags: [],
    steps: [
      { cmd:"MOTION rise", durationMs:2000, target:"all", label:"x", hold:false },
      { cmd:"STOP", durationMs:500, target:2, label:"y", hold:true },
    ],
  }],
  setlists: [{
    id: "set", name: "editor-only", mode: "ordered",
    entries: [{ seqId:"show", repeat:2, gapMs:1000, weight:3 }],
    shuffleRules: { avoidSameTag:false, minGapEntries:1, moodArc:"random", seed:7 },
  }],
  activeSetlistId: "set",
  schedulerConfig: { leaderBoardId:1, graceMs:10000 },
};

const built = core.buildBakeLibrary(fixture);
const slice = core.sliceForBoard(built, 1);

check(slice.schemaVersion === 2, "the slice declares schemaVersion 2");
eq(Object.keys(slice).filter(k => k !== "schemaVersion" && k !== "bakedAtMs").sort(),
   ["a", "g", "l", "m", "q"], "root uses short keys");

const motion = slice.m[0];
eq(Object.keys(motion).sort(), ["d", "i", "r"], "a motion carries only id/durationMs/tracks");
check(motion.i === "rise", "motion id survives");
check(motion.d === 2000, "motion duration survives");

const track = motion.r[0];
eq(Object.keys(track).sort(), ["c", "k"], "a track carries only channel/keyframes");
check(!("kind" in track) && !("n" in track), "kind is dropped — slices are servo-only");
check(!("boardId" in track) && !("b" in track), "boardId is dropped — the slice implies it");
check(track.r === undefined, "no stray v1 keys on a track");

// The single biggest win: two numbers instead of two named fields.
eq(track.k, [[0, 100], [2000, 0]], "keyframes are positional [atMs, value]");

// Only board 1's track is in board 1's slice, so channel 5 must not appear.
check(motion.r.length === 1, "a slice carries one board's tracks only");

const step = slice.q[0].s[0];
eq(Object.keys(slice.q[0]).sort(), ["i", "s"], "a sequence carries only id/steps");
eq(Object.keys(step).sort(), ["c", "d"], "an all-target step omits target entirely");
// The planner rewrites a Motion step with its firmware PREP glide, so the
// command carries an annotation. What matters here is that the command
// itself rides through the key change intact.
check(step.c.startsWith("MOTION rise"), "step command survives the key change");
eq(Object.keys(slice.q[0].s[1]).sort(), ["c", "d", "t"], "a board-targeted step keeps target");
check(slice.q[0].s[1].t === 2, "target stays a number, not the select's string");

const setlist = slice.l[0];
// `mode` is omitted for an ordered setlist — the firmware's default — so an
// ordered one carries no `o` at all. That default-omission predates v2 and
// still has to hold once the key is one character.
eq(Object.keys(setlist).sort(), ["e", "i", "u"], "an ordered setlist omits mode");
const shuffled = core.sliceForBoard(core.buildBakeLibrary({
  ...fixture,
  setlists: [{ ...fixture.setlists[0], mode: "shuffle" }],
}), 1).l[0];
check(shuffled.o === "shuffle", "a shuffle setlist carries mode under the short key");
eq(Object.keys(setlist.e[0]).sort(), ["g", "p", "q", "w"], "entry uses short keys");
check(setlist.u.n === 1 && setlist.u.s === 7, "shuffleRules minGapEntries/seed survive");
check(slice.g.b === 1 && slice.g.g === 10000, "schedulerConfig survives");
check(slice.a === "set", "activeSetlistId survives");

// --- Pull from Boards has to survive the format change ------------------
const rehydrated = core.hydrateDeviceLibraryForEditor(JSON.parse(JSON.stringify(slice)), 1);
check(rehydrated.schemaVersion === 2, "a hydrated library keeps schemaVersion 2");
const rm = rehydrated.motions[0];
check(rm.id === "rise" && rm.durationMs === 2000, "hydrate restores motion id/duration");
eq(rm.tracks[0].keyframes, [{ atMs:0, value:100 }, { atMs:2000, value:0 }],
   "hydrate restores keyframes to named fields the editor can render");
check(rm.tracks[0].kind === "servo", "hydrate restores the implied kind");
check(rm.tracks[0].boardId === 1, "hydrate restores the implied boardId from the slice");
check(rehydrated.sequences[0].steps[0].cmd === "MOTION rise", "hydrate restores steps");
check(rehydrated.setlists[0].entries[0].seqId === "show", "hydrate restores setlist entries");

// --- The point of all of it ---------------------------------------------
const v1Bytes = 1183;   // the same fixture through v1, pinned so a regression shows
const v2Bytes = Buffer.byteLength(JSON.stringify(slice));
check(v2Bytes < v1Bytes * 0.75,
  `the fixture slice shrinks by at least a quarter (v1 ${v1Bytes}B -> v2 ${v2Bytes}B)`);


// --- Every reader of a device payload has to speak v2 --------------------
// This is the bug class that kept recurring: the wire format changed, and
// then another place that reads it silently kept using v1 names. A reader
// that misses does not throw — it reads undefined, treats the payload as
// empty, and reports something plausible. So sweep the source instead of
// waiting for each one to be noticed.

// diffBlob decides "edited since bake". With v1 names it indexed nothing on
// either side and returned a clean zero, so the warning would just have
// stopped appearing.
{
  const lib = core.buildBakeLibrary(fixture);
  const a = core.sliceForBoard(lib, 1);
  check(core.diffBlob(a, a).added === 0 &&
        core.diffBlob(a, a).changed === 0 &&
        core.diffBlob(a, a).deleted === 0,
    "diffBlob sees no change between a v2 payload and itself");

  const edited = JSON.parse(JSON.stringify(a));
  edited[K.root.motions][0][K.motion.durationMs] = 9999;
  check(core.diffBlob(a, edited).changed === 1,
    "diffBlob detects an edited motion in a v2 payload");

  const removed = JSON.parse(JSON.stringify(a));
  removed[K.root.sequences] = [];
  check(core.diffBlob(a, removed).deleted === a[K.root.sequences].length,
    "diffBlob detects removed sequences in a v2 payload");

  // A snapshot cached by an older build is v1. Comparing it to a v2 slice
  // should report churn, not silence — the operator does need to re-bake.
  const v1ish = { motions: [{ id: "rise", durationMs: 1 }], sequences: [], setlists: [] };
  check(core.diffBlob(v1ish, a).added + core.diffBlob(v1ish, a).deleted > 0,
    "diffBlob reports churn between a stale v1 snapshot and a v2 slice");
}

// The readers that touch the DOM cannot be imported, so check the source. Any
// of these reading a device snapshot with a v1 name is the bug again.
{
  const deviceReaders = [
    ["snap.motions", "the baked-on-boards panel"],
    ["snap.sequences", "the setlist readiness gate"],
    ["?.tracks?.[0]?.boardId", "the panel's board-id derivation"],
  ];
  for (const [needle, what] of deviceReaders) {
    // A `||` fallback for an older cached snapshot is fine; a bare read is not.
    const bare = new RegExp(String.raw`(?<!\|\|\s)` + needle.replace(/[.?[\]]/g, "\\$&"));
    check(!bare.test(html), `${what} no longer reads a device payload as v1`);
  }
}

console.log(`\n${passed} bake-v2 checks passed`);
