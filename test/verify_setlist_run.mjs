// Host tests for the firmware-run setlist player core (servo-17r).
// Play walks setlist entries and dispatches RUN <seqId> per entry; these
// helpers decide (a) whether the selected setlist is safe to run against the
// last bake and (b) how the entry walker reacts to runseq telemetry.
import { readFileSync, writeFileSync, mkdtempSync } from "node:fs";
import { tmpdir } from "node:os";
import { join, dirname } from "node:path";
import { fileURLToPath, pathToFileURL } from "node:url";

const here = dirname(fileURLToPath(import.meta.url));
const html = readFileSync(join(here, "..", "servo_controller.html"), "utf8");
const START = "// === SETLIST-RUN-CORE START ===";
const END = "// === SETLIST-RUN-CORE END ===";

let passed = 0, failed = 0;
function fail(m) { console.error(`  FAIL: ${m}`); failed++; }
function ok(n) { console.log(`  PASS  ${n}`); passed++; }
function eq(n, got, want) {
  if (JSON.stringify(got) === JSON.stringify(want)) ok(n);
  else fail(`${n}\n    expected: ${JSON.stringify(want)}\n    got:      ${JSON.stringify(got)}`);
}

const s = html.indexOf(START), e = html.indexOf(END, s + START.length);
if (s < 0 || e <= s) { fail("SETLIST-RUN-CORE markers not found"); process.exit(1); }
const dir = mkdtempSync(join(tmpdir(), "setlist-run-core-"));
const modPath = join(dir, "core.mjs");
writeFileSync(modPath, html.slice(s + START.length, e) +
  "\nexport { assessSetlistRunReadiness, decideEntryWatch };\n", "utf8");
const { assessSetlistRunReadiness, decideEntryWatch } = await import(pathToFileURL(modPath).href);

console.log("=== Setlist firmware-run readiness ===");

// assessSetlistRunReadiness(entries, bakedSeqIds, editedSinceBake)
//   entries: normalized setlist entries [{ seqId, ... }]
//   bakedSeqIds: sequence ids in the leader's last bake snapshot, or null when
//                no bake has been recorded from this browser
//   editedSinceBake: library-vs-snapshot diff found changes
// -> { ok, status, missingSeqIds }
const E = (...ids) => ids.map(seqId => ({ seqId, repeat: 1, gapMs: 0, weight: 1 }));

eq("no bake snapshot blocks with not-baked",
   assessSetlistRunReadiness(E("sq-a"), null, false),
   { ok: false, status: "not-baked", missingSeqIds: [] });
eq("an entry missing from the bake blocks and names the sequence",
   assessSetlistRunReadiness(E("sq-a", "sq-b"), ["sq-a"], false),
   { ok: false, status: "missing-seq", missingSeqIds: ["sq-b"] });
eq("missing-seq wins over stale-bake (can't run what isn't there)",
   assessSetlistRunReadiness(E("sq-b"), ["sq-a"], true),
   { ok: false, status: "missing-seq", missingSeqIds: ["sq-b"] });
eq("edits since bake warn but stay playable",
   assessSetlistRunReadiness(E("sq-a"), ["sq-a"], true),
   { ok: true, status: "stale-bake", missingSeqIds: [] });
eq("fully baked setlist is ok",
   assessSetlistRunReadiness(E("sq-a", "sq-b"), ["sq-a", "sq-b"], false),
   { ok: true, status: "ok", missingSeqIds: [] });
eq("duplicate missing entries are reported once",
   assessSetlistRunReadiness(E("sq-b", "sq-b"), ["sq-a"], false),
   { ok: false, status: "missing-seq", missingSeqIds: ["sq-b"] });
eq("empty entries are ok — the caller rejects an empty resolved order",
   assessSetlistRunReadiness([], ["sq-a"], false),
   { ok: true, status: "ok", missingSeqIds: [] });

console.log("\n=== Entry walker telemetry watch ===");

// decideEntryWatch({ run, expectedSeqId, sawExpectedRun, telemetryFresh,
//                    sinceSentMs, startTimeoutMs, fallbackMs })
//   run: normalized runseq from readRunseqTelemetry (null = nothing active)
//   sawExpectedRun: the walker has already observed its RUN going active
//   telemetryFresh: at least one board reported recently (run:null is
//                   trustworthy evidence of "ended", not just silence)
// -> "wait" | "ended" | "fallback-ended" | "no-start" | "superseded"
const W = decideEntryWatch;
const running = id => ({ id, firmwareStep: 0, firmwareStepCount: 3, loop: false, stepMs: 0 });

eq("just sent, nothing active yet: wait for the run to appear",
   W({ run: null, expectedSeqId: "sq-a", sawExpectedRun: false, telemetryFresh: true,
       sinceSentMs: 900, startTimeoutMs: 6000, fallbackMs: 20000 }),
   "wait");
eq("expected run active: wait for it to finish",
   W({ run: running("sq-a"), expectedSeqId: "sq-a", sawExpectedRun: true, telemetryFresh: true,
       sinceSentMs: 4000, startTimeoutMs: 6000, fallbackMs: 20000 }),
   "wait");
eq("expected run active even past fallback: trust live telemetry",
   W({ run: running("sq-a"), expectedSeqId: "sq-a", sawExpectedRun: true, telemetryFresh: true,
       sinceSentMs: 60000, startTimeoutMs: 6000, fallbackMs: 20000 }),
   "wait");
eq("run seen then gone with fresh telemetry: entry ended",
   W({ run: null, expectedSeqId: "sq-a", sawExpectedRun: true, telemetryFresh: true,
       sinceSentMs: 9000, startTimeoutMs: 6000, fallbackMs: 20000 }),
   "ended");
eq("run seen then telemetry lost: hold until the fallback duration",
   W({ run: null, expectedSeqId: "sq-a", sawExpectedRun: true, telemetryFresh: false,
       sinceSentMs: 9000, startTimeoutMs: 6000, fallbackMs: 20000 }),
   "wait");
eq("telemetry lost past the fallback duration: advance anyway",
   W({ run: null, expectedSeqId: "sq-a", sawExpectedRun: true, telemetryFresh: false,
       sinceSentMs: 20000, startTimeoutMs: 6000, fallbackMs: 20000 }),
   "fallback-ended");
eq("never went active and telemetry is fresh past the start timeout: no-start",
   W({ run: null, expectedSeqId: "sq-a", sawExpectedRun: false, telemetryFresh: true,
       sinceSentMs: 6000, startTimeoutMs: 6000, fallbackMs: 20000 }),
   "no-start");
eq("never went active with dead telemetry: fall back on the duration estimate",
   W({ run: null, expectedSeqId: "sq-a", sawExpectedRun: false, telemetryFresh: false,
       sinceSentMs: 20000, startTimeoutMs: 6000, fallbackMs: 20000 }),
   "fallback-ended");
eq("never went active with dead telemetry before fallback: wait",
   W({ run: null, expectedSeqId: "sq-a", sawExpectedRun: false, telemetryFresh: false,
       sinceSentMs: 6000, startTimeoutMs: 6000, fallbackMs: 20000 }),
   "wait");
eq("a different run id means another operator took over: superseded",
   W({ run: running("sq-x"), expectedSeqId: "sq-a", sawExpectedRun: true, telemetryFresh: true,
       sinceSentMs: 4000, startTimeoutMs: 6000, fallbackMs: 20000 }),
   "superseded");
eq("a different run id before ours even started is still superseded",
   W({ run: running("sq-x"), expectedSeqId: "sq-a", sawExpectedRun: false, telemetryFresh: true,
       sinceSentMs: 500, startTimeoutMs: 6000, fallbackMs: 20000 }),
   "superseded");

console.log(`\n${passed} passed, ${failed} failed`);
process.exit(failed ? 1 : 0);
