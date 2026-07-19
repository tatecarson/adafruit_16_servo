import { readFileSync } from "node:fs";
import { dirname, join } from "node:path";
import { fileURLToPath } from "node:url";

const here = dirname(fileURLToPath(import.meta.url));
const html = readFileSync(join(here, "..", "servo_controller.html"), "utf8");
const drawer = html.match(/<div class="mtest-backdrop"[\s\S]*?<\/aside>/)?.[0] || "";
const setup = html.match(/\(function setupMotorTest\(\) \{[\s\S]*?\n\}\)\(\);/)?.[0] || "";

let passed = 0;
function check(condition, message) {
  if (!condition) throw new Error(`FAIL: ${message}`);
  passed++;
  console.log(`PASS: ${message}`);
}

check(/MOTOR_TEST_SAFE_UP_PCT\s*=\s*80\b/.test(html), "safe manual Up target is 80%");
check(/data-mtest-all="80"/.test(drawer), "All Up sends 80%");
check(/All Up · 80%/.test(drawer), "All Up visibly labels the safe target");
check(/UP \$\{n\} \$\{MOTOR_TEST_SAFE_UP_PCT\}/.test(setup), "per-servo Up uses the shared safe target");
check(/Up \$\{MOTOR_TEST_SAFE_UP_PCT\}%/.test(setup), "per-servo Up visibly labels the safe target");
check(!/data-mtest-all="100"/.test(drawer), "All Up no longer commands 100%");
check(!/UP \$\{n\} 100/.test(setup), "per-servo Up no longer commands 100%");
check(/data-mtest-all="0"/.test(drawer), "All Down remains at 0%");
check(/UP \$\{n\} 50/.test(setup), "Mid remains at 50%");

console.log(`\n${passed}/9 Motor Test safety checks passed.`);
