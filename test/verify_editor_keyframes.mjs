// Tests for the Motion-editor keyframe selection/deletion helpers (servo-dmu).
// Pure logic lives in the EDITOR-CORE block of servo_controller.html:
//   keyframesInMarquee, deleteKeyframeIndices
//
// Run: node verify_editor_keyframes.mjs   (or via the make test node run)

import { readFileSync, writeFileSync, mkdtempSync } from "node:fs";
import { tmpdir } from "node:os";
import { join, dirname } from "node:path";
import { fileURLToPath, pathToFileURL } from "node:url";

const here = dirname(fileURLToPath(import.meta.url));
const html = readFileSync(join(here, "..", "servo_controller.html"), "utf8");
const START = "// === EDITOR-CORE START ===";
const END = "// === EDITOR-CORE END ===";

let passed = 0, failed = 0;
function fail(m) { console.error(`  FAIL: ${m}`); failed++; }
function ok(n) { console.log(`  PASS  ${n}`); passed++; }
function eq(n, got, want) {
  if (JSON.stringify(got) === JSON.stringify(want)) ok(n);
  else fail(`${n}\n    expected: ${JSON.stringify(want)}\n    got:      ${JSON.stringify(got)}`);
}

const s = html.indexOf(START);
const e = html.indexOf(END, s + START.length);
if (s < 0 || e <= s) { fail("EDITOR-CORE markers not found or out of order"); process.exit(1); }
const core = html.slice(s + START.length, e);

const dir = mkdtempSync(join(tmpdir(), "editor-core-"));
const modPath = join(dir, "core.mjs");
writeFileSync(modPath, core + "\nexport { keyframesInMarquee, deleteKeyframeIndices };\n", "utf8");
const mod = await import(pathToFileURL(modPath).href);
for (const fn of ["keyframesInMarquee", "deleteKeyframeIndices"]) {
  if (typeof mod[fn] !== "function") fail(`EDITOR-CORE does not export ${fn}()`);
}
if (failed) process.exit(1);
const { keyframesInMarquee, deleteKeyframeIndices } = mod;

console.log("=== Motion editor keyframe select/delete ===");

const kfs = [{ atMs: 0, value: 0 }, { atMs: 1000, value: 50 }, { atMs: 2000, value: 100 }, { atMs: 3000, value: 20 }];

// ---- marquee selects by time AND value bounds (inclusive) ------------------
eq("time+value box selects middle two", keyframesInMarquee(kfs, 900, 2100, 40, 100), [1, 2]);
eq("full-time, low-value box selects only the 0", keyframesInMarquee(kfs, 0, 3000, 0, 10), [0]);
eq("box matching nothing returns empty", keyframesInMarquee(kfs, 100, 900, 0, 100), []);
eq("inclusive bounds include endpoints", keyframesInMarquee(kfs, 0, 1000, 0, 50), [0, 1]);

// ---- bounds are order-independent (marquee dragged any direction) ----------
eq("reversed bounds normalize", keyframesInMarquee(kfs, 2100, 900, 100, 40), [1, 2]);

// ---- delete by indices ------------------------------------------------------
eq("delete two indices", deleteKeyframeIndices(kfs, [1, 2]), [{ atMs: 0, value: 0 }, { atMs: 3000, value: 20 }]);
eq("delete is order-insensitive", deleteKeyframeIndices(kfs, [2, 1]), [{ atMs: 0, value: 0 }, { atMs: 3000, value: 20 }]);
eq("delete none returns same", deleteKeyframeIndices(kfs, []), kfs);
eq("original array not mutated", kfs.length, 4);

console.log(`\n${passed} passed, ${failed} failed`);
process.exit(failed ? 1 : 0);
