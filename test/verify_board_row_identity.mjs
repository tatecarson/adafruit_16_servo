// Host tests for which machine a board-strip row claims to be (servo-bfe).
//
// The strip read `status.node` as a board id. It is not one: Sync.cpp's
// initNodeId sets nodeId to the last byte of the board's MAC, so a board whose
// MAC ends in 0xD0 reports node 208. machineOf(208) is nothing, so every
// online board rendered as "unknown machine" — and renderPills, keyed off the
// same value, could not convert a DC percentage to rpm either.
//
// The board id is discovered over /boardId and cached per-IP. That is the only
// source; a status payload must never be allowed to stand in for it.
//
// Run: node verify_board_row_identity.mjs   (or via test/run-all.mjs)

import { readFileSync, writeFileSync, mkdtempSync } from "node:fs";
import { tmpdir } from "node:os";
import { join, dirname } from "node:path";
import { fileURLToPath, pathToFileURL } from "node:url";

const here = dirname(fileURLToPath(import.meta.url));
const html = readFileSync(join(here, "..", "servo_controller.html"), "utf8");

let passed = 0, failed = 0;
function fail(m) { console.error(`  FAIL: ${m}`); failed++; }
function ok(n) { console.log(`  PASS  ${n}`); passed++; }
function eq(n, got, want) {
  if (JSON.stringify(got) === JSON.stringify(want)) ok(n);
  else fail(`${n}\n    expected: ${JSON.stringify(want)}\n    got:      ${JSON.stringify(got)}`);
}

function block(startMarker, endMarker) {
  const s = html.indexOf(startMarker);
  const e = html.indexOf(endMarker, s + startMarker.length);
  if (s < 0 || e <= s) { fail(`${startMarker} not found or out of order`); process.exit(1); }
  return html.slice(s + startMarker.length, e);
}

const core = block("// === MACHINE-CORE START ===", "// === MACHINE-CORE END ===")
  + block("// === BOARD-IDENTITY-CORE START ===", "// === BOARD-IDENTITY-CORE END ===");

const dir = mkdtempSync(join(tmpdir(), "board-row-identity-"));
const modPath = join(dir, "core.mjs");
writeFileSync(modPath, core + "\nexport { resolveBoardId, machineOf };\n", "utf8");
const mod = await import(pathToFileURL(modPath).href);
if (typeof mod.resolveBoardId !== "function") fail("BOARD-IDENTITY-CORE does not export resolveBoardId()");
if (failed) process.exit(1);
const { resolveBoardId, machineOf } = mod;

console.log("=== Board strip · which machine is this ===");

// The board that prompted this: board 3, MAC ending 0xD0, so node 208.
const realStatus = { fw: "servo-inw-1", node: 208, ip: "192.168.8.138", uptimeMs: 39423 };

eq("the discovered board id wins", resolveBoardId(realStatus, 3), 3);
eq("...and names the machine", machineOf(resolveBoardId(realStatus, 3)).name, "dowel curtain");

// The regression itself. 208 must never reach machineOf.
eq("a sync node id is not a board id", resolveBoardId(realStatus, undefined), null);
eq("...even when it happens to look like one",
   resolveBoardId({ node: 1 }, undefined), null);
eq("...and a discovered id is not overridden by one",
   resolveBoardId({ node: 1 }, 3), 3);

// Offline boards have no status at all and have never answered /boardId.
eq("an unheard-from board has no machine", resolveBoardId(null, undefined), null);
eq("...and neither does a board that is offline now but was discovered before",
   resolveBoardId(null, 2), 2);

// Ids that are not machines stay null rather than becoming `board 208`.
eq("an out-of-range discovered id is rejected", resolveBoardId(realStatus, 208), null);
eq("a non-numeric discovered id is rejected", resolveBoardId(realStatus, "three"), null);

console.log(`\n${passed} passed, ${failed} failed`);
process.exit(failed ? 1 : 0);
