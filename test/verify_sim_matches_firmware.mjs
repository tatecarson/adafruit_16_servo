// Oracle test: the browser setlist simulation (servo-6j4 Part 2) must reproduce
// the firmware scheduler's pick order EXACTLY. The expected pick sequences below
// were captured from the C++ firmware code (setlist_scheduler.h via
// test_setlist_scheduler.cpp machinery) — they are the source of truth.
//
// The simulation core lives inline in servo_controller.html between
//   // === SETLIST-SIM-CORE START ===
//   // === SETLIST-SIM-CORE END ===
// as PURE functions (no DOM). This test extracts that block, loads it as an ES
// module, and asserts simulateSetlist() yields the firmware's order.
//
// Run: node verify_sim_matches_firmware.mjs   (or: make sim-verify)

import { readFileSync, writeFileSync, mkdtempSync } from "node:fs";
import { tmpdir } from "node:os";
import { join, dirname } from "node:path";
import { fileURLToPath, pathToFileURL } from "node:url";

const here = dirname(fileURLToPath(import.meta.url));
const htmlPath = join(here, "..", "servo_controller.html");

const START = "// === SETLIST-SIM-CORE START ===";
const END = "// === SETLIST-SIM-CORE END ===";

function fail(msg) {
  console.error(`FAIL: ${msg}`);
  process.exit(1);
}

const html = readFileSync(htmlPath, "utf8");
const s = html.indexOf(START);
const e = html.indexOf(END);
if (s < 0 || e < 0 || e < s) {
  fail(`could not find sim-core markers in servo_controller.html\n` +
       `  expected ${START} ... ${END}`);
}
const core = html.slice(s + START.length, e);

// The core lives in a regular <script> (not type=module), so it can't contain
// `export`. We append the export here so the same source is browser-valid AND
// loadable as a module for this test. simulateSetlist must be a top-level fn.
const dir = mkdtempSync(join(tmpdir(), "setlist-sim-"));
const modPath = join(dir, "core.mjs");
writeFileSync(modPath, core + "\nexport { simulateSetlist };\n", "utf8");

let mod;
try {
  mod = await import(pathToFileURL(modPath).href);
} catch (err) {
  fail(`sim-core block failed to load as a module: ${err.message}`);
}
if (typeof mod.simulateSetlist !== "function") {
  fail("sim-core does not export simulateSetlist()");
}

// ---- Oracle 1: the existing `gallery` test setlist (seed 42, minGap 2) -------
const galleryLib = {
  sequences: [
    { id: "arc", name: "Arc", steps: [{ durationMs: 100 }] },
    { id: "pulse", name: "Pulse", steps: [{ durationMs: 100 }] },
    { id: "drift", name: "Drift", steps: [{ durationMs: 100 }] },
  ],
  setlists: [{
    id: "gallery", name: "Gallery", mode: "shuffle",
    entries: [
      { seqId: "arc", repeat: 1, gapMs: 0, weight: 1 },
      { seqId: "pulse", repeat: 1, gapMs: 0, weight: 1 },
      { seqId: "drift", repeat: 1, gapMs: 0, weight: 1 },
    ],
    shuffleRules: { minGapEntries: 2, seed: 42 },
  }],
};
const galleryExpected =
  ["arc", "pulse", "drift", "arc", "pulse", "drift", "arc", "pulse", "drift", "arc", "pulse", "drift"];

// ---- Oracle 2: richer weighted RNG (weights 3,1,2,1,4; minGap 1; seed 12345) -
const richLib = {
  sequences: ["a", "b", "c", "d", "e"].map((id) => ({ id, name: id.toUpperCase(), steps: [{ durationMs: 100 }] })),
  setlists: [{
    id: "rich", name: "Rich", mode: "shuffle",
    entries: [
      { seqId: "a", repeat: 1, gapMs: 0, weight: 3 },
      { seqId: "b", repeat: 1, gapMs: 0, weight: 1 },
      { seqId: "c", repeat: 1, gapMs: 0, weight: 2 },
      { seqId: "d", repeat: 1, gapMs: 0, weight: 1 },
      { seqId: "e", repeat: 1, gapMs: 0, weight: 4 },
    ],
    shuffleRules: { minGapEntries: 1, seed: 12345 },
  }],
};
const richExpected =
  ["a", "e", "b", "a", "d", "a", "b", "a", "b", "e", "d", "b", "a", "e", "a", "e", "a", "c", "e", "c"];

function picksFrom(lib, n) {
  const set = lib.setlists[0];
  const res = mod.simulateSetlist(lib, set, { horizonMs: (n + 5) * 100 });
  if (!res || !Array.isArray(res.events)) fail("simulateSetlist did not return { events: [...] }");
  return res.events.slice(0, n).map((ev) => ev.seqId);
}

function check(name, lib, expected) {
  const got = picksFrom(lib, expected.length);
  const ok = got.length === expected.length && got.every((v, i) => v === expected[i]);
  if (!ok) {
    fail(`${name}: pick order does not match firmware\n` +
         `  expected: ${JSON.stringify(expected)}\n` +
         `  got:      ${JSON.stringify(got)}`);
  }
  console.log(`  PASS  ${name} (${expected.length} picks match firmware)`);
}

console.log("=== Setlist simulation vs firmware oracle ===");
check("gallery seed=42 minGap=2", galleryLib, galleryExpected);
check("rich weighted seed=12345 minGap=1", richLib, richExpected);
console.log("\nsim matches firmware ✓");
