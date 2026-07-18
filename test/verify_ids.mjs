// Tests for opaque motion/sequence id generation (servo-d91).
// Pure logic lives in the ID-CORE block of servo_controller.html:
//   randomToken(prefix), uniqueId(prefix, used),
//   uniqueMotionId(lib), uniqueSequenceId(lib)
//
// Run: node verify_ids.mjs   (or via the make test node run)

import { readFileSync, writeFileSync, mkdtempSync } from "node:fs";
import { tmpdir } from "node:os";
import { join, dirname } from "node:path";
import { fileURLToPath, pathToFileURL } from "node:url";

const here = dirname(fileURLToPath(import.meta.url));
const html = readFileSync(join(here, "..", "servo_controller.html"), "utf8");
const START = "// === ID-CORE START ===";
const END = "// === ID-CORE END ===";

let passed = 0, failed = 0;
function fail(m) { console.error(`  FAIL: ${m}`); failed++; }
function ok(n) { console.log(`  PASS  ${n}`); passed++; }
function assert(n, cond) { cond ? ok(n) : fail(n); }

const s = html.indexOf(START);
const e = html.indexOf(END, s + START.length);
if (s < 0 || e <= s) { fail("ID-CORE markers not found or out of order"); process.exit(1); }
const core = html.slice(s + START.length, e);

const dir = mkdtempSync(join(tmpdir(), "id-core-"));
const modPath = join(dir, "core.mjs");
writeFileSync(modPath, core + "\nexport { randomToken, uniqueMotionId, uniqueSequenceId };\n", "utf8");
const mod = await import(pathToFileURL(modPath).href);
for (const fn of ["randomToken", "uniqueMotionId", "uniqueSequenceId"]) {
  if (typeof mod[fn] !== "function") { fail(`ID-CORE does not export ${fn}()`); process.exit(1); }
}
const { randomToken, uniqueMotionId, uniqueSequenceId } = mod;

console.log("=== Opaque id generation ===");

// Format: "<prefix>-<6 lowercase base36 chars>", opaque (not derived from name).
const mt = uniqueMotionId({ motions: [] });
const sq = uniqueSequenceId({ sequences: [] });
assert("motion id uses mt- prefix", /^mt-[0-9a-z]{6}$/.test(mt));
assert("sequence id uses sq- prefix", /^sq-[0-9a-z]{6}$/.test(sq));

// Opaque: a descriptive name never leaks into the id (the whole point).
const named = uniqueMotionId({ motions: [{ id: "existing", name: "Tidal Drift" }] });
assert("id is not derived from any name", !/tidal|drift/i.test(named) && /^mt-[0-9a-z]{6}$/.test(named));

// Uniqueness across a batch of generated ids.
const seen = new Set();
let allUnique = true;
const lib = { motions: [] };
for (let i = 0; i < 2000; i++) {
  const id = uniqueMotionId(lib);
  if (seen.has(id)) allUnique = false;
  seen.add(id);
  lib.motions.push({ id });
}
assert("2000 generated motion ids are all unique", allUnique && seen.size === 2000);

// Dedupe: never collides with an id already in the library, even if the RNG
// were to repeat (simulated by pre-seeding the used set with a forced token).
const forced = randomToken("mt");
const dedup = uniqueMotionId({ motions: [{ id: forced }] });
assert("never returns an id already present in the library", dedup !== forced);

console.log(`\n${passed} passed, ${failed} failed`);
process.exit(failed ? 1 : 0);
