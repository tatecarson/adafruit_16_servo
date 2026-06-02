// Tests for the pure analysis layer of the setlist simulation (servo-6j4):
// play-count / pacing / too-soon stats and the verdict. These functions live in
// the SETLIST-SIM-CORE block of servo_controller.html alongside simulateSetlist.
//
// Run: node verify_sim_analysis.mjs   (or: make sim-verify)

import { readFileSync, writeFileSync, mkdtempSync } from "node:fs";
import { tmpdir } from "node:os";
import { join, dirname } from "node:path";
import { fileURLToPath, pathToFileURL } from "node:url";

const here = dirname(fileURLToPath(import.meta.url));
const html = readFileSync(join(here, "..", "servo_controller.html"), "utf8");
const START = "// === SETLIST-SIM-CORE START ===";
const END = "// === SETLIST-SIM-CORE END ===";

let passed = 0, failed = 0;
function fail(msg) { console.error(`  FAIL: ${msg}`); failed++; }
function ok(name) { console.log(`  PASS  ${name}`); passed++; }
function eq(name, got, want) {
  if (JSON.stringify(got) === JSON.stringify(want)) ok(name);
  else fail(`${name}\n    expected: ${JSON.stringify(want)}\n    got:      ${JSON.stringify(got)}`);
}
function approx(name, got, want, tol = 1e-9) {
  if (Math.abs(got - want) <= tol) ok(name);
  else fail(`${name}: expected ~${want}, got ${got}`);
}

const s = html.indexOf(START);
const e = html.indexOf(END, s + START.length);
if (s < 0 || e <= s) { fail("sim-core markers not found or out of order"); process.exit(1); }
const core = html.slice(s + START.length, e);

const dir = mkdtempSync(join(tmpdir(), "setlist-an-"));
const modPath = join(dir, "core.mjs");
writeFileSync(modPath, core + "\nexport { simAnalyzeRun, simAggregate, simVerdict, simulateSetlist };\n", "utf8");
const mod = await import(pathToFileURL(modPath).href);

for (const fn of ["simAnalyzeRun", "simAggregate", "simVerdict"]) {
  if (typeof mod[fn] !== "function") fail(`sim-core does not export ${fn}()`);
}
if (failed) process.exit(1);
const { simAnalyzeRun, simAggregate, simVerdict, simulateSetlist } = mod;

console.log("=== Setlist simulation analysis ===");

// Helper to build a fake simulate result from a compact [seqId, durationMs, gapMs] list.
function mkResult(rows) {
  let t = 0;
  const events = rows.map(([seqId, durationMs, gapMs, missing]) => {
    const ev = { tStartMs: t, seqId, seqName: seqId.toUpperCase(), missing: !!missing,
                 durationMs, repeat: 1, playMs: durationMs, gapAfterMs: gapMs };
    t += durationMs + gapMs;
    return ev;
  });
  return { seedUsed: 1, events, capped: false, horizonMs: t, mode: "shuffle" };
}

// ---- play counts + airtime --------------------------------------------------
{
  const r = mkResult([["a", 100, 0], ["b", 200, 0], ["a", 100, 0]]);
  const an = simAnalyzeRun(r, {});
  eq("count: a played twice", an.perSeq.a.count, 2);
  eq("count: b played once", an.perSeq.b.count, 1);
  eq("airtime: a = 200ms", an.perSeq.a.airtimeMs, 200);
  eq("airtime: b = 200ms", an.perSeq.b.airtimeMs, 200);
}

// ---- pacing / dead air ------------------------------------------------------
{
  const r = mkResult([["a", 100, 500], ["b", 100, 2000], ["c", 100, 0]]);
  const an = simAnalyzeRun(r, {});
  eq("longest dead air = 2000ms", an.gaps.longestMs, 2000);
  eq("min gap = 0", an.gaps.min, 0);
  eq("max gap = 2000", an.gaps.max, 2000);
}

// ---- too-soon: same seq recurs within tooSoonPicks other picks --------------
{
  // a, a back-to-back (0 picks between) → flagged at default tooSoonPicks=1.
  const back = mkResult([["a", 100, 200], ["a", 100, 0], ["b", 100, 0]]);
  const anBack = simAnalyzeRun(back, {});
  eq("too-soon: back-to-back flagged", anBack.sameSeq.tooSoonCount, 1);
  eq("min same-seq interval = 300", anBack.sameSeq.minIntervalMs, 300);

  // a, b, a → one pick between → NOT flagged at default tooSoonPicks=1.
  const spaced = mkResult([["a", 100, 0], ["b", 100, 0], ["a", 100, 0]]);
  eq("too-soon: one-apart not flagged", simAnalyzeRun(spaced, {}).sameSeq.tooSoonCount, 0);
  // …but flagged if the operator demands ≥2 picks of separation.
  eq("too-soon: one-apart flagged at tooSoonPicks=2", simAnalyzeRun(spaced, { tooSoonPicks: 2 }).sameSeq.tooSoonCount, 1);
}

// ---- missing sequence flagged ----------------------------------------------
{
  const r = mkResult([["ghost", 0, 0, true], ["a", 100, 0]]);
  const an = simAnalyzeRun(r, {});
  eq("missing seq surfaced", an.missingSeqIds.sort(), ["ghost"]);
}

// ---- verdict: balanced vs dominated ----------------------------------------
{
  const balanced = simAggregate(
    { sequences: [{ id: "a", steps: [{ durationMs: 100 }] }, { id: "b", steps: [{ durationMs: 100 }] }] },
    { mode: "shuffle", entries: [{ seqId: "a", weight: 1 }, { seqId: "b", weight: 1 }],
      shuffleRules: { minGapEntries: 1, seed: 7 } },
    { horizonMs: 4000 });
  const v = simVerdict(balanced);
  eq("balanced verdict level ok", v.level, "ok");

  const dominated = simAggregate(
    { sequences: [{ id: "a", steps: [{ durationMs: 100 }] }, { id: "b", steps: [{ durationMs: 100 }] }] },
    { mode: "shuffle", entries: [{ seqId: "a", weight: 50 }, { seqId: "b", weight: 1 }],
      shuffleRules: { minGapEntries: 0, seed: 7 } },
    { horizonMs: 4000 });
  const v2 = simVerdict(dominated);
  eq("dominated verdict level warn", v2.level, "warn");
  if (!v2.messages.some(m => /more/i.test(m))) fail("dominated verdict should mention imbalance");
  else ok("dominated verdict mentions imbalance");
}

// ---- aggregate: unseeded runs many times, seeded runs once ------------------
{
  const lib = { sequences: [{ id: "a", steps: [{ durationMs: 100 }] }, { id: "b", steps: [{ durationMs: 100 }] }] };
  const seeded = simAggregate(lib,
    { mode: "shuffle", entries: [{ seqId: "a", weight: 1 }, { seqId: "b", weight: 1 }],
      shuffleRules: { minGapEntries: 1, seed: 99 } }, { horizonMs: 2000, runs: 100 });
  eq("seeded → 1 run", seeded.runs, 1);
  eq("seeded flag true", seeded.seeded, true);

  const unseeded = simAggregate(lib,
    { mode: "shuffle", entries: [{ seqId: "a", weight: 1 }, { seqId: "b", weight: 1 }],
      shuffleRules: { minGapEntries: 1, seed: 0 } }, { horizonMs: 2000, runs: 50 });
  eq("unseeded → N runs", unseeded.runs, 50);
  eq("unseeded flag false", unseeded.seeded, false);
}

// ---- degenerate setlist: zero-duration sequence caps the run, analysis must
//      not throw on the huge (100k-event) array (Math.max spread overflow) ----
{
  const lib = { sequences: [{ id: "z", name: "Z", steps: [{ durationMs: 0 }] }] };
  const set = { mode: "shuffle", entries: [{ seqId: "z", repeat: 1, gapMs: 0, weight: 1 }],
                shuffleRules: { minGapEntries: 0, seed: 1 } };
  const res = simulateSetlist(lib, set, { horizonMs: 3600000 });
  eq("zero-duration run is capped", res.capped, true);
  let an;
  try { an = simAnalyzeRun(res, {}); ok("analysis of capped run does not throw"); }
  catch (e) { fail(`analysis threw on capped run: ${e.message}`); an = null; }
  if (an) eq("capped run gaps computed", an.gaps.longestMs, 0);
}

// ---- single-sequence setlist must NOT be flagged "too soon" (intentional) ---
{
  const lib = { sequences: [{ id: "solo", name: "Solo", steps: [{ durationMs: 1000 }] }] };
  const set = { mode: "ordered", entries: [{ seqId: "solo", repeat: 1, gapMs: 0 }] };
  const v = simVerdict(simAggregate(lib, set, { horizonMs: 5000 }));
  eq("single-sequence setlist verdict ok (no false too-soon)", v.level, "ok");
}

// ---- aggregate min must count runs where a sequence didn't play as 0 --------
{
  const lib = { sequences: [
    { id: "x", name: "X", steps: [{ durationMs: 1000 }] },
    { id: "y", name: "Y", steps: [{ durationMs: 1000 }] }] };
  const set = { mode: "shuffle",
    entries: [{ seqId: "x", repeat: 1, gapMs: 0, weight: 1 },
              { seqId: "y", repeat: 1, gapMs: 0, weight: 1 }],
    shuffleRules: { minGapEntries: 0, seed: 0 } };
  // horizon=1ms → exactly one pick per run, so each run plays only X or only Y.
  const sum = simAggregate(lib, set, { horizonMs: 1, runs: 20 });
  eq("both sequences sampled across runs", Object.keys(sum.perSeq).sort(), ["x", "y"]);
  eq("min counts zero-play runs (x)", sum.perSeq.x.count.min, 0);
  eq("min counts zero-play runs (y)", sum.perSeq.y.count.min, 0);
}

console.log(`\n${passed} passed, ${failed} failed`);
process.exit(failed ? 1 : 0);
