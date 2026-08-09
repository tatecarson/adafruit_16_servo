// Host tests for the per-board motion send queue (servo-ua4 follow-up).
//
// The board runs a single-socket HTTP server, so playback keeps at most one
// request in flight per board and coalesces the rest. That is correct for one
// stream of values — a newer ROTATE really does supersede an older one — but
// the pending slot was one command per BOARD, and a motion frame dispatches
// every servo channel in the same synchronous pass:
//
//     DMOVE 0 -> sent          (nothing in flight)
//     DMOVE 1 -> parked        (one slot, now holds channel 1)
//     DMOVE 2 -> parked        (overwrites channel 1 — lost)
//
// So the middle channel of a three-servo board never reached the wire. It cost
// an afternoon on the bench looking for a dead servo header that was fine.
//
// A newer command supersedes an older one only when it addresses the same
// thing. These check that: same channel coalesces, different channels queue.
//
// Run: node verify_motion_send_queue.mjs   (or via test/run-all.mjs)

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

const core = block("// === SEND-QUEUE-CORE START ===", "// === SEND-QUEUE-CORE END ===");

const dir = mkdtempSync(join(tmpdir(), "motion-send-queue-"));
const modPath = join(dir, "core.mjs");
writeFileSync(modPath, core + "\nexport { motionCoalesceKey, makeSendQueue };\n", "utf8");
const mod = await import(pathToFileURL(modPath).href);
for (const fn of ["motionCoalesceKey", "makeSendQueue"]) {
  if (typeof mod[fn] !== "function") fail(`SEND-QUEUE-CORE does not export ${fn}()`);
}
if (failed) process.exit(1);
const { motionCoalesceKey, makeSendQueue } = mod;

console.log("=== Motion send queue ===");

// --- what counts as "the same thing" --------------------------------------
eq("a servo move is keyed by its channel", motionCoalesceKey("DMOVE 1 40 800"), "DMOVE:1");
eq("a direct position is keyed by its channel", motionCoalesceKey("DOWN 2 100"), "DOWN:2");
eq("two moves on one channel share a key",
   motionCoalesceKey("DMOVE 1 40 800"), motionCoalesceKey("DMOVE 1 90 200"));
eq("moves on different channels do not",
   motionCoalesceKey("DMOVE 1 40 800") === motionCoalesceKey("DMOVE 2 40 800"), false);
// The motor is one per board, so every ROTATE supersedes the last.
eq("the motor is one stream", motionCoalesceKey("ROTATE 30"), "ROTATE");
eq("...whatever its value", motionCoalesceKey("ROTATE -12"), "ROTATE");
eq("a stop is a stop", motionCoalesceKey("STOP"), "STOP");

// --- the queue -------------------------------------------------------------
// drain() reports what actually goes to the wire, in order, one at a time.

function drainAll(q) {
  const out = [];
  let next = q.takeNext();
  while (next) { out.push(next.cmd); next = q.takeNext(); }
  return out;
}

// The bug, as a test. Three channels dispatched in one pass must all arrive.
const frame = makeSendQueue();
eq("the first command goes straight out", frame.push("DMOVE 0 0 1155").sendNow, true);
eq("the second is queued behind it", frame.push("DMOVE 1 0 1155").sendNow, false);
eq("the third is queued too, not swapped for the second",
   frame.push("DMOVE 2 0 1155").sendNow, false);
eq("both queued channels reach the wire, in order",
   drainAll(frame), ["DMOVE 1 0 1155", "DMOVE 2 0 1155"]);

// Coalescing still has to work, or playback floods a single-socket server.
const flood = makeSendQueue();
flood.push("DMOVE 1 10 100");     // in flight
flood.push("DMOVE 1 20 100");
flood.push("DMOVE 1 30 100");
eq("repeated moves on one channel collapse to the newest",
   drainAll(flood), ["DMOVE 1 30 100"]);

const mixed = makeSendQueue();
mixed.push("ROTATE 10");          // in flight
mixed.push("DMOVE 1 40 800");
mixed.push("ROTATE 30");
mixed.push("DMOVE 1 60 800");
mixed.push("DMOVE 2 60 800");
eq("each stream keeps its own newest value, in first-seen order",
   drainAll(mixed), ["DMOVE 1 60 800", "ROTATE 30", "DMOVE 2 60 800"]);

// A queued command must not outlive the playback that made it.
const cleared = makeSendQueue();
cleared.push("DMOVE 0 0 100");
cleared.push("DMOVE 1 0 100");
cleared.clear();
eq("clearing drops the backlog", drainAll(cleared), []);
eq("...and frees the board for the next send", cleared.push("DMOVE 0 50 100").sendNow, true);

// Nothing in flight and nothing queued.
const idle = makeSendQueue();
eq("an idle queue has nothing to take", idle.takeNext(), null);

console.log(`\n${passed} passed, ${failed} failed`);
process.exit(failed ? 1 : 0);
