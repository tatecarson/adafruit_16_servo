#!/usr/bin/env node
/**
 * Serve the repo on one origin, and persist the browser library to ./library.json.
 *
 * Two reasons this exists rather than reaching for any static file server:
 *
 *  1. The dashboard and the 3D simulator talk over a BroadcastChannel, which is
 *     scoped to an origin. Opened as files they are same-origin in some browsers
 *     and not in others — Firefox gives every file:// document its own opaque
 *     origin, so the link silently never connects. Served from here they are
 *     unambiguously the same origin and it always works.
 *  2. POST/PUT to /library.json writes the authoring library back to the repo,
 *     so Motions and Sequences survive a browser cache clear and can be diffed
 *     in git. A plain file server would 405 that and the dashboard would fall
 *     back to localStorage without you noticing.
 *
 * No dependencies, so `npm start` works on a clean checkout with no install.
 */

import { createServer } from "node:http";
import { createReadStream } from "node:fs";
import { rename, stat, writeFile } from "node:fs/promises";
import { dirname, extname, join, normalize, resolve, sep } from "node:path";
import { fileURLToPath } from "node:url";

const ROOT = dirname(fileURLToPath(import.meta.url));
const LIBRARY_PATH = join(ROOT, "library.json");
const MAX_LIBRARY_BYTES = 512 * 1024;

const TYPES = {
  ".html": "text/html; charset=utf-8",
  ".js": "text/javascript; charset=utf-8",
  ".mjs": "text/javascript; charset=utf-8",
  ".json": "application/json; charset=utf-8",
  ".css": "text/css; charset=utf-8",
  ".svg": "image/svg+xml",
  ".png": "image/png",
  ".jpg": "image/jpeg",
  ".jpeg": "image/jpeg",
  ".ico": "image/x-icon",
  ".woff2": "font/woff2",
  ".txt": "text/plain; charset=utf-8",
  ".md": "text/plain; charset=utf-8",
  ".bin": "application/octet-stream",
  ".map": "application/json; charset=utf-8",
};

function args() {
  const out = { host: process.env.HOST || "127.0.0.1", port: Number(process.env.PORT) || 4173 };
  const argv = process.argv.slice(2);
  for (let i = 0; i < argv.length; i++) {
    if (argv[i] === "--port" || argv[i] === "-p") out.port = Number(argv[++i]);
    else if (argv[i] === "--host") out.host = argv[++i];
    else if (argv[i] === "--help" || argv[i] === "-h") out.help = true;
  }
  return out;
}

function cors(res) {
  res.setHeader("Access-Control-Allow-Origin", "*");
  res.setHeader("Access-Control-Allow-Methods", "GET, HEAD, POST, PUT, OPTIONS");
  res.setHeader("Access-Control-Allow-Headers", "Content-Type");
}

function fail(res, code, message) {
  cors(res);
  res.writeHead(code, { "Content-Type": "text/plain; charset=utf-8" });
  res.end(message + "\n");
}

/* Resolve a URL path inside ROOT, or null if it escapes. Decoding first and
   checking after is the order that matters: %2e%2e%2f is ".." and a check
   against the raw string would wave it through. */
function safePath(urlPath) {
  let decoded;
  try { decoded = decodeURIComponent(urlPath.split("?")[0]); }
  catch { return null; }
  if (decoded.includes("\0")) return null;
  const target = resolve(ROOT, "." + normalize(decoded));
  if (target !== ROOT && !target.startsWith(ROOT + sep)) return null;
  return target;
}

async function serveFile(req, res, target) {
  let info;
  try { info = await stat(target); }
  catch { return fail(res, 404, "Not found"); }

  if (info.isDirectory()) {
    const index = join(target, "index.html");
    try { await stat(index); target = index; info = await stat(index); }
    catch { return fail(res, 403, "Directory listing is off"); }
  }

  cors(res);
  res.setHeader("Content-Type", TYPES[extname(target).toLowerCase()] || "application/octet-stream");
  res.setHeader("Content-Length", info.size);
  // Authoring means reloading constantly; a cached library.json or a cached
  // page is never what you want here.
  res.setHeader("Cache-Control", "no-store");
  if (req.method === "HEAD") return res.end();
  res.writeHead(200);
  createReadStream(target).pipe(res);
}

async function writeLibrary(req, res) {
  if ((req.url || "").split("?")[0] !== "/library.json") {
    return fail(res, 404, "Only /library.json accepts writes");
  }

  /* Read with a ceiling. Reply *before* tearing the request down: destroying an
     upload that is still in flight resets the connection, and the dashboard
     reports that as a transport error rather than the "too large" it actually
     is. Answer first, then stop reading. */
  const chunks = [];
  let size = 0;
  let tooBig = false;
  req.on("error", () => {});     // a client that hangs up mid-upload is not our problem
  for await (const chunk of req) {
    size += chunk.length;
    if (size > MAX_LIBRARY_BYTES) { tooBig = true; break; }
    chunks.push(chunk);
  }
  if (tooBig) {
    fail(res, 413, "Library JSON is too large");
    req.destroy();
    return;
  }
  if (!size) return fail(res, 400, "Empty request body");

  let parsed;
  try { parsed = JSON.parse(Buffer.concat(chunks).toString("utf8")); }
  catch (e) { return fail(res, 400, "Invalid JSON: " + e.message); }

  // Refuse anything that is not the schema the editor speaks, so a stray POST
  // cannot quietly replace the library with something unloadable.
  if (parsed?.schemaVersion !== 1) return fail(res, 400, "Expected schemaVersion 1");

  /* Write a temp file next to the target and rename over it. Rename within one
     filesystem is atomic, so a crash mid-write leaves the previous library
     intact rather than a half-written one — this file is the authoring master. */
  const tmp = join(ROOT, `.library.json.${process.pid}.${Date.now()}.tmp`);
  try {
    await writeFile(tmp, JSON.stringify(parsed, null, 2) + "\n", "utf8");
    await rename(tmp, LIBRARY_PATH);
  } catch (e) {
    return fail(res, 500, "Write failed: " + e.message);
  }

  const { size: bytes } = await stat(LIBRARY_PATH);
  const body = JSON.stringify({ ok: true, bytes });
  cors(res);
  res.writeHead(200, { "Content-Type": "application/json", "Content-Length": Buffer.byteLength(body) });
  res.end(body);
}

const opts = args();
if (opts.help) {
  console.log(`Usage: npm start [-- --port N] [--host H]

Serves this directory at http://HOST:PORT and persists POST/PUT /library.json.
Defaults: 127.0.0.1:4173 (override with --port/--host or PORT/HOST).`);
  process.exit(0);
}

const server = createServer((req, res) => {
  if (req.method === "OPTIONS") { cors(res); res.writeHead(204); return res.end(); }
  if (req.method === "POST" || req.method === "PUT") {
    return writeLibrary(req, res).catch(e => fail(res, 500, e.message));
  }
  if (req.method !== "GET" && req.method !== "HEAD") return fail(res, 405, "Method not allowed");

  const target = safePath(req.url || "/");
  if (!target) return fail(res, 403, "Forbidden");
  serveFile(req, res, target).catch(e => fail(res, 500, e.message));
});

server.listen(opts.port, opts.host, () => {
  const base = `http://${opts.host}:${opts.port}`;
  console.log(`Serving   ${ROOT}`);
  console.log(`Dashboard ${base}/servo_controller.html`);
  console.log(`Simulator ${base}/sculpture_3d.html`);
  console.log(`Library   ${LIBRARY_PATH}`);
  console.log(`\nOpen both in the same browser — same origin, so the dashboard's`);
  console.log(`BroadcastChannel reaches the simulator. Ctrl-C to stop.`);
});

server.on("error", (e) => {
  if (e.code === "EADDRINUSE") {
    console.error(`Port ${opts.port} is already in use — something else is serving.`);
    console.error(`Stop it, or pick another: npm start -- --port 4174`);
    process.exit(1);
  }
  throw e;
});
