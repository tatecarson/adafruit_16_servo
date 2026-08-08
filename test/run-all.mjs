#!/usr/bin/env node
/**
 * Run every verify_*.mjs in this directory and report once.
 *
 * These check the browser-side authoring logic — bake payloads, keyframe
 * normalisation, sequence arrangement — by importing the same functions the
 * dashboard runs. The firmware's own C++ tests are separate: `make -C test`.
 *
 * A verifier signals failure by exiting non-zero. Several of them also print
 * "N failed" on success with N = 0, so the exit code is the only thing worth
 * trusting here.
 */

import { readdirSync } from "node:fs";
import { dirname, join } from "node:path";
import { fileURLToPath } from "node:url";
import { spawnSync } from "node:child_process";

const HERE = dirname(fileURLToPath(import.meta.url));
const files = readdirSync(HERE).filter(f => f.startsWith("verify_") && f.endsWith(".mjs")).sort();

let failed = 0;
for (const f of files) {
  const r = spawnSync(process.execPath, [join(HERE, f)], { encoding: "utf8" });
  const ok = r.status === 0;
  if (!ok) failed++;
  const tail = (r.stdout || "").trim().split("\n").pop() || (r.stderr || "").trim().split("\n").pop() || "";
  console.log(`${ok ? "  ok  " : "FAIL  "}${f.padEnd(34)}${tail.slice(0, 70)}`);
  if (!ok && r.stderr) console.log(r.stderr.trim().split("\n").map(l => "        " + l).join("\n"));
}

console.log(`\n${files.length - failed}/${files.length} verifier files passed`);
if (failed) {
  console.log("Firmware tests are separate: make -C test");
  process.exit(1);
}
